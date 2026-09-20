/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_process.cc
 * Brief: The whole assembly, end to end, on real captured frames
 *
 * Description:
 * A loopback listener stands in for the chassis, and the frames it sends back
 * are the ones the real machine sent on 2026-09-15 (test/golden/). So this
 * exercises the actual path: socket, framer, codec, report parser, session,
 * Tier 1, tx owner, odometry -- with no test double anywhere between them.
 *
 * No threads. CtrlTick and RxPump are called directly with an explicit clock,
 * which is why a 200 ms Tier 1 timeout and a 5 s probe window are exercised in
 * microseconds. A test that started the threads would have to wait for real
 * time and would be flaky on a loaded machine; worse, it could only observe the
 * assembly from outside, and the ordering inside a control period is precisely
 * what needs pinning.
 *
 * The cases that matter most, each with the failure it prevents:
 *
 *   * a heartbeat goes out on the dialling period. The chassis reports state
 *     ONLY to an address that keeps sending them (13 S2.2), so a probe that
 *     waited a period first would spend it guaranteed to hear nothing.
 *   * an axis command needs BOTH gates: Tier 1 must produce motion AND the
 *     session must allow it. Collapsing them sends into a socket that is
 *     reconnecting.
 *   * a soft stop sends its zero frame from the CALLBACK, not from the next
 *     period. The next period is up to 10 ms away and 11 S9.12.6 T-1 budgets
 *     5 ms; at 2 m/s those 10 ms are 2 cm of extra travel per period of delay.
 */

#include "quadruped/process.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cctype>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "quadruped/chs_a_codec.h"

using namespace quadruped;  // NOLINT: test-local

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes FromHex(const std::string& hex) {
  Bytes out;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

std::map<std::string, Bytes> LoadGolden(const std::string& path) {
  std::map<std::string, Bytes> out;
  std::ifstream f(path);
  if (!f) {
    std::printf("FAIL cannot open golden file: %s\n", path.c_str());
    ++g_failures;
    return out;
  }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream is(line);
    std::string tag, hex;
    std::size_t n = 0;
    if (!(is >> tag >> n >> hex)) continue;
    out[tag] = FromHex(hex);
  }
  if (out.empty()) {
    std::printf("FAIL golden file parsed to zero vectors\n");
    ++g_failures;
  }
  return out;
}

// A loopback listener standing in for the chassis.
class FakeChassis {
 public:
  FakeChassis() {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a;
    std::memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a));
    ::listen(fd_, 4);
    socklen_t len = sizeof(a);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
    port_ = ntohs(a.sin_port);
  }
  ~FakeChassis() {
    if (conn_ >= 0) ::close(conn_);
    if (fd_ >= 0) ::close(fd_);
  }
  int port() const { return port_; }
  bool Accept() {
    if (conn_ >= 0) ::close(conn_);
    conn_ = ::accept(fd_, nullptr, nullptr);
    return conn_ >= 0;
  }
  // Everything the process has sent so far, appended.
  std::size_t Drain() {
    std::uint8_t buf[8192];
    long n = 0;
    std::size_t total = 0;
    while ((n = ::recv(conn_, buf, sizeof(buf), MSG_DONTWAIT)) > 0) {
      sent_.insert(sent_.end(), buf, buf + n);
      total += static_cast<std::size_t>(n);
    }
    return total;
  }
  void Send(const Bytes& b) { ::send(conn_, b.data(), b.size(), MSG_NOSIGNAL); }
  const Bytes& sent() const { return sent_; }
  void ClearSent() { sent_.clear(); }

  // How many whole CHS-A frames the process has sent, and what the last one
  // routes to. Counting FRAMES rather than bytes is what makes an assertion
  // about "a heartbeat went out" mean something.
  int CountFrames(std::uint32_t* last_type, std::uint32_t* last_cmd) const {
    int count = 0;
    std::size_t i = 0;
    while (i + chs_a::kHeaderBytes <= sent_.size()) {
      chs_a::Header h;
      if (!chs_a::ReadHeader(sent_.data() + i, sent_.size() - i, &h)) {
        ++i;
        continue;
      }
      const std::size_t total = chs_a::kHeaderBytes + h.asdu_len;
      if (i + total > sent_.size()) break;
      chs_a::AsduRouting r;
      if (chs_a::ParseAsduRouting(sent_.data() + i + chs_a::kHeaderBytes,
                                  h.asdu_len, &r)) {
        if (last_type) *last_type = r.type;
        if (last_cmd) *last_cmd = r.command;
      }
      ++count;
      i += total;
    }
    return count;
  }

  // The axis values of the LAST real-axis frame, read out of the JSON that
  // actually went on the wire. Reading the numbers matters: every assertion
  // above is satisfied by a frame of the right SHAPE carrying any velocity at
  // all, so without this a process that sends the unclamped command instead of
  // the Tier 1 output looks exactly like a correct one.
  bool LastAxis(double* vx, double* vy, double* yaw) const {
    bool found = false;
    std::size_t i = 0;
    while (i + chs_a::kHeaderBytes <= sent_.size()) {
      chs_a::Header h;
      if (!chs_a::ReadHeader(sent_.data() + i, sent_.size() - i, &h)) {
        ++i;
        continue;
      }
      const std::size_t total = chs_a::kHeaderBytes + h.asdu_len;
      if (i + total > sent_.size()) break;
      chs_a::AsduRouting r;
      if (chs_a::ParseAsduRouting(sent_.data() + i + chs_a::kHeaderBytes,
                                  h.asdu_len, &r) &&
          r.type == chs_a::kRealAxis.type && r.command == chs_a::kRealAxis.command) {
        const std::string asdu(
            reinterpret_cast<const char*>(sent_.data() + i + chs_a::kHeaderBytes),
            h.asdu_len);
        double a = 0.0, b = 0.0, c = 0.0;
        const std::size_t px = asdu.find("\"X\":");
        const std::size_t py = asdu.find("\"Y\":");
        const std::size_t pw = asdu.find("\"Yaw\":");
        if (px != std::string::npos && py != std::string::npos &&
            pw != std::string::npos &&
            std::sscanf(asdu.c_str() + px + 4, "%lf", &a) == 1 &&
            std::sscanf(asdu.c_str() + py + 4, "%lf", &b) == 1 &&
            std::sscanf(asdu.c_str() + pw + 6, "%lf", &c) == 1) {
          *vx = a;
          *vy = b;
          *yaw = c;
          found = true;
        }
      }
      i += total;
    }
    return found;
  }

 private:
  int fd_ = -1;
  int conn_ = -1;
  int port_ = 0;
  Bytes sent_;
};

// The UDP side of channel one. 13 S2.2 lists udp:30004 alongside tcp:30003 and
// the resolved config enables BOTH, so this is a real endpoint the process can
// land on -- not a hypothetical.
//
// It binds and then waits for the process's first datagram so it learns the
// peer address; the process connect()s its own socket, so a plain sendto back
// to that address is delivered.
class FakeUdpChassis {
 public:
  FakeUdpChassis() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a;
    std::memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a));
    socklen_t len = sizeof(a);
    ::getsockname(fd_, reinterpret_cast<sockaddr*>(&a), &len);
    port_ = ntohs(a.sin_port);
  }
  ~FakeUdpChassis() {
    if (fd_ >= 0) ::close(fd_);
  }
  int port() const { return port_; }

  // Read whatever the process has sent, and remember where it came from.
  bool LearnPeer() {
    std::uint8_t buf[8192];
    socklen_t len = sizeof(peer_);
    const long n = ::recvfrom(fd_, buf, sizeof(buf), MSG_DONTWAIT,
                              reinterpret_cast<sockaddr*>(&peer_), &len);
    if (n <= 0) return have_peer_;
    have_peer_ = true;
    return true;
  }
  // Send EXACTLY these bytes as one datagram. Callers use it to send a whole
  // frame, half a frame, or two frames at once -- FR-5 says only the first is
  // a frame, and the other two are what this endpoint must refuse.
  void SendRaw(const std::uint8_t* d, std::size_t n) {
    if (!have_peer_) return;
    ::sendto(fd_, d, n, MSG_NOSIGNAL, reinterpret_cast<sockaddr*>(&peer_),
             sizeof(peer_));
  }
  void SendRaw(const Bytes& b) { SendRaw(b.data(), b.size()); }

 private:
  int fd_ = -1;
  int port_ = 0;
  sockaddr_in peer_{};
  bool have_peer_ = false;
};

QuadrupedConfig Cfg(int port) {
  QuadrupedConfig c;
  c.robot_id = "gj-001";
  EndpointCandidate ep;
  ep.proto = "tcp";
  ep.host = "127.0.0.1";
  ep.port = port;
  ep.tls = false;
  ep.enabled = true;
  c.link.endpoints = {ep};
  c.link.probe_timeout_ms = 2000;
  c.link.heartbeat_hz = 2.0;
  c.link.codebook = "hex32";
  c.link.resync_max_bytes = 4096;
  c.link.frame_assembly_timeout_ms = 500;
  c.link.partial_send_retry = 3;
  c.link.tcp_nodelay = true;
  c.link.axis_cmd_hz = 20.0;
  c.link.cmd_fail_threshold = 3;
  c.link.state_timeout_degraded_s = 1.0;
  c.link.state_timeout_lost_s = 3.0;
  c.link.reconnect_backoff_s = {0.5, 1.0, 5.0};
  c.tier1.limits.max_vx_mps = 2.0;
  c.tier1.limits.max_vy_mps = 1.0;
  c.tier1.limits.max_wz_radps = 0.8;
  c.tier1.limits.holonomic = true;
  c.tier1.cmd_timeout_ms = 200;
  c.tier1.control_loop_hz = 100.0;
  c.odom.publish_hz = 100.0;
  c.odom.sigma_v0_mps = 0.05;
  c.odom.a_max_mps2 = 2.5;
  c.odom.gyro_bias_radps = 0.008;
  c.odom.arw_rad_sqrt_s = 0.002;
  c.odom.trust_flat = 1.0;
  c.odom.trust_stair = 0.3;
  c.odom.stale_warn_ms = 150;
  c.odom.stale_invalid_ms = 300;
  c.odom.stale_stop_publish_ms = 1000;
  // Channel two. imu_age_warn_ms is the ONE number that decides the yaw
  // fallback of 13 S4.2, and it is set here rather than left at the struct's
  // zero: a zero threshold classifies every sample as stale, so the IMU branch
  // would never be taken and the fallback cases below would pass against an
  // implementation that had no IMU path at all.
  c.dds.domain_id = 0;
  c.dds.imu_topic = "/IMU";
  c.dds.imu_expect_hz = 200.0;
  c.dds.imu_age_warn_ms = 50;
  c.dds.imu_frame_id = "imu_link";
  c.dds.forward_imu_to_rt = false;
  // 13 PR-1 / QC-9 / MS-2 / TR-1. Set HERE, in the fixture the process is
  // actually built from -- test_mode_machine.cc builds its own ModeConfig and
  // therefore could not see the process discarding this block entirely.
  c.motion.prone_forbidden_gaits = {0x3003, 0x1003};
  // 13 GS-1. The SECOND gait list, and a different rule from the one above:
  // stair_standard is on both, because we never command it (GS-1) and it can
  // still arrive from the handset (GS-3).
  c.motion.command_forbidden_gaits = {0x1003};
  // *** Deliberately NOT 5.0. A fixture that matches the literal the code used
  // to hardcode makes "hardcode it again" an equivalent mutation -- nothing
  // could tell the two apart. Measured: that mutant survived until this value
  // changed.
  c.motion.mode_switch_timeout_s = 2.0;
  c.motion.external_transition_hold_s = 3.5;
  return c;
}

// The same config with its one endpoint switched to UDP. Same shape as the real
// resolved file, whose second candidate is udp:30004 with enabled: true.
QuadrupedConfig UdpCfg(int port) {
  QuadrupedConfig c = Cfg(port);
  c.link.endpoints[0].proto = "udp";
  return c;
}

// A report frame of an arbitrary Type, carrying no ErrorCode.
//
// It exists because the capture has no such frame: the two non-modelled frames
// in it (the location report and the heartbeat response) BOTH carry an
// ErrorCode and are therefore consumed by the error branch of RxPump, never
// reaching the "unmodelled report" path. Without this builder that path has no
// input at all, and a mutant that forwards unparsed bytes onto a schema'd key
// survives for lack of a test case rather than for lack of a defect.
Bytes TypedReportFrame(std::uint32_t type, std::uint32_t command) {
  char items[256];
  std::snprintf(items, sizeof(items),
                "{\"PatrolDevice\":{\"Command\":%u,\"Items\":{},"
                "\"Time\":\"2026-09-15 14:55:55.457\",\"Type\":%u}}",
                command, type);
  const std::size_t asdu_len = std::strlen(items);
  Bytes out(chs_a::kHeaderBytes + asdu_len);
  chs_a::Header h;
  h.asdu_len = static_cast<std::uint16_t>(asdu_len);
  h.msg_id = 0;
  chs_a::WriteHeader(h, out.data(), out.size());
  std::memcpy(out.data() + chs_a::kHeaderBytes, items, asdu_len);
  return out;
}

// A captured frame with ONLY its Command field rewritten, header length fixed.
//
// Used instead of a hand-built frame wherever the payload has to be real: a
// synthetic fault frame with an empty Items block does not parse, so it would
// test nothing. This keeps the vendor's own bytes and changes exactly the field
// under test.
Bytes WithCommand(const Bytes& frame, std::uint32_t command) {
  const std::string asdu(reinterpret_cast<const char*>(frame.data() + chs_a::kHeaderBytes),
                         frame.size() - chs_a::kHeaderBytes);
  const std::string needle = "\"Command\":";
  const std::size_t at = asdu.find(needle);
  if (at == std::string::npos) return frame;
  std::size_t end = at + needle.size();
  while (end < asdu.size() && (asdu[end] == ' ' || std::isdigit(
             static_cast<unsigned char>(asdu[end])))) {
    ++end;
  }
  // hex32 codes are serialised as DECIMAL integers (13 S5.5 / CB-1): JSON has
  // no hex literal, and writing one is how the first frame gets 0xE002.
  const std::string out_asdu = asdu.substr(0, at + needle.size()) +
                               std::to_string(command) + asdu.substr(end);
  Bytes out(chs_a::kHeaderBytes + out_asdu.size());
  chs_a::Header h;
  h.asdu_len = static_cast<std::uint16_t>(out_asdu.size());
  h.msg_id = 0;
  chs_a::WriteHeader(h, out.data(), out.size());
  std::memcpy(out.data() + chs_a::kHeaderBytes, out_asdu.data(), out_asdu.size());
  return out;
}

// A BasicStatus frame with chosen fields, built the way the chassis builds one.
Bytes BasicFrame(int usage_mode, int motion_state, int gait, bool hes,
                 bool sleep) {
  char items[512];
  std::snprintf(items, sizeof(items),
                "{\"PatrolDevice\":{\"Command\":15728640,\"Items\":"
                "{\"BasicStatus\":{\"ControlUsageMode\":%d,\"Gait\":%d,"
                "\"HES\":%d,\"MotionState\":%d,\"Sleep\":%d,\"Model\":\"CA9C\"}},"
                "\"Time\":\"2026-09-15 14:55:55.457\",\"Type\":1048676}}",
                usage_mode, gait, hes ? 1 : 0, motion_state, sleep ? 1 : 0);
  const std::size_t asdu_len = std::strlen(items);
  Bytes out(chs_a::kHeaderBytes + asdu_len);
  chs_a::Header h;
  h.asdu_len = static_cast<std::uint16_t>(asdu_len);
  h.msg_id = 0;
  chs_a::WriteHeader(h, out.data(), out.size());
  std::memcpy(out.data() + chs_a::kHeaderBytes, items, asdu_len);
  return out;
}


// A MotionStatus frame. It carries motion_state and gait -- but NOT
// ControlUsageMode, which is the asymmetry the snapshot merge bug turned on:
// BasicStatus is the only report that mentions the usage mode.
Bytes MotionFrame(int motion_state, int gait) {
  char items[512];
  std::snprintf(items, sizeof(items),
                "{\"PatrolDevice\":{\"Command\":15728640,\"Items\":"
                "{\"MotionStatus\":{\"MotionState\":%d,\"Gait\":%d,"
                "\"LinearX\":0.0,\"LinearY\":0.0,\"AngularZ\":0.0}},"
                "\"Time\":\"2026-09-15 14:55:55.457\",\"Type\":1048577}}",
                motion_state, gait);
  const std::size_t asdu_len = std::strlen(items);
  Bytes out(chs_a::kHeaderBytes + asdu_len);
  chs_a::Header h;
  h.asdu_len = static_cast<std::uint16_t>(asdu_len);
  h.msg_id = 0;
  chs_a::WriteHeader(h, out.data(), out.size());
  std::memcpy(out.data() + chs_a::kHeaderBytes, items, asdu_len);
  return out;
}

// Run the process from `from` to `to` while the chassis keeps REPORTING the
// given gait at 2 Hz, the way a real one does.
//
// The reporting is not decoration. 13 CA-7 makes an arriving report the only
// evidence the link is alive, so a loop that merely ticks for four seconds
// takes the session through degraded to lost and every later request is
// refused for a link reason -- which looks exactly like the refusal under
// test. Measured: the first draft did that and the final assertion failed for
// a reason that had nothing to do with PR-1.
//
// Used where 13 TR-1's external-transition hold has to expire before PR-1's
// verdict means anything: during the hold the STEADY triple is still the old
// one (TR-2), so an assertion made too early is answered by the previous gait.
void Settle(QuadrupedProcess* p, FakeChassis* chassis, double from, double to,
            int gait) {
  double next_report = from;
  for (double t = from; t < to; t += 0.1) {
    if (t >= next_report) {
      chassis->Send(BasicFrame(/*usage_mode=*/1, /*motion_state=*/17, gait,
                               /*hes=*/false, /*sleep=*/false));
      next_report = t + 0.5;      // 2 Hz, 13 S7.1
    }
    p->RxPump(t);
    p->CtrlTick(t);
  }
  chassis->Drain();
}


}  // namespace

int main(int argc, char** argv) {
  const std::string golden_path =
      (argc >= 2) ? argv[1] : "test/golden/chs_a_frames.txt";
  const auto golden = LoadGolden(golden_path);
  if (golden.empty()) {
    std::printf("%d PROCESS TEST(S) FAILED\n", g_failures);
    return 1;
  }

  // ---- dial, heartbeat, real captured report, link up --------------------
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));

    // The first period dials and sends a heartbeat. Both happen on THIS period:
    // the chassis reports only to an address already sending them, so a probe
    // that waited would spend the wait guaranteed to hear nothing.
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Drain();
    std::uint32_t type = 0, cmd = 0;
    CHECK(chassis.CountFrames(&type, &cmd) >= 1);
    CHECK(type == chs_a::kHeartbeat.type);
    CHECK(cmd == chs_a::kHeartbeat.command);
    CHECK(p.heartbeats_sent() == 1);
    CHECK(p.conn_state() == chs_a::ConnState::kProbing);

    // *** A REAL captured report comes back. These bytes came off the chassis
    // on 2026-09-15; nothing in this path has been mocked.
    chassis.Send(golden.at("RX_00100064_00f00000"));
    // The receive thread's body, called directly.
    int frames = 0;
    for (int i = 0; i < 50 && frames == 0; ++i) frames = p.RxPump(0.01 * i);
    CHECK(frames >= 1);
    CHECK(p.frames_received() >= 1);

    // The next period takes the snapshot and declares the link up.
    p.CtrlTick(0.6);
    CHECK(p.conn_state() == chs_a::ConnState::kOk);

    // No command has arrived from above, so Tier 1 reports a timeout and holds
    // zero. The link being healthy does not make an absent command safe.
    CHECK(p.last_tier1().stop_reason == StopReason::kTimeout);
    CHECK(p.last_tier1().timeout_lock == true);
    CHECK(p.motion_allowed() == true);   // the LINK allows it; Tier 1 does not
  }

  // ---- an axis command needs BOTH gates ---------------------------------
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());

    // The chassis is in navigation mode, standing, on the flat gait.
    chassis.Send(BasicFrame(1, 17, 0x3002, false, false));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.6);

    // Clear the timeout lock the opening silence set: it needs a fresh command
    // AND an enable, and the process has no enable path yet, so this case
    // drives a fresh instance instead of unlocking one.
    CHECK(p.last_tier1().stop_reason == StopReason::kTimeout);

    chassis.ClearSent();
    chassis.Drain();
    // A command arrives with the generation we hold.
    p.OnCmdVel(0.61, 1.0, 0.0, 0.2, p.estop_epoch());
    p.CtrlTick(0.62);
    chassis.Drain();
    // Still locked, so nothing but heartbeats goes out. This is the gate: a
    // link that is up is not permission to move.
    std::uint32_t type = 0, cmd = 0;
    const int n = chassis.CountFrames(&type, &cmd);
    if (n > 0) CHECK(type != chs_a::kRealAxis.type || cmd != chs_a::kRealAxis.command);
    CHECK(p.axis_frames_sent() == 0);
  }

  // ---- a soft stop sends its zero frame from the CALLBACK ---------------
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    // Unlocked and moving first. Without that the opening timeout lock holds
    // the robot anyway, and "the soft stop worked" cannot be told apart from
    // "something else was already stopping it" -- which is what the estop
    // generation mutant survived on before this was added.
    chassis.Send(BasicFrame(1, 17, 0x3002, false, false));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.6);
    p.OnCmdVel(0.61, 1.0, 0.0, 0.0, p.estop_epoch());
    p.OnEnable();
    p.CtrlTick(0.62);
    p.OnCmdVel(0.63, 1.0, 0.0, 0.0, p.estop_epoch());
    p.CtrlTick(0.64);
    CHECK(p.last_tier1().stop_reason == StopReason::kNone);
    chassis.Drain();
    chassis.ClearSent();

    const std::uint64_t before = p.estop_epoch();
    const std::uint64_t axis_before = p.axis_frames_sent();
    // *** No CtrlTick between the stop and the assertion. The frame must
    // already be on the wire: the next period is up to 10 ms away and
    // 11 S9.12.6 T-1 budgets 5 ms.
    p.OnSoftEstop(0.65);
    CHECK(p.estop_epoch() == before + 1);
    chassis.Drain();
    std::uint32_t type = 0, cmd = 0;
    CHECK(chassis.CountFrames(&type, &cmd) >= 1);
    CHECK(type == chs_a::kRealAxis.type);
    CHECK(cmd == chs_a::kRealAxis.command);
    CHECK(p.axis_frames_sent() == axis_before + 1);
    // ZERO on every axis. The frame going out is not the point -- what it
    // carries is; a stop frame holding the last commanded velocity would pass
    // every other assertion here.
    double zx = 1.0, zy = 1.0, zw = 1.0;
    CHECK(chassis.LastAxis(&zx, &zy, &zw));
    CHECK(zx == 0.0 && zy == 0.0 && zw == 0.0);

    // And the generation now disagrees with whatever the upstream last echoed,
    // so the next period holds zero until it catches up (13 S9.12.2 (3)).
    p.OnCmdVel(0.66, 1.0, 0.0, 0.0, before);   // still the OLD generation
    p.CtrlTick(0.67);
    // EXACTLY soft_estop. Nothing else is stopping this robot: the command is
    // fresh, the mode is navigation, there is no HES and the link is up.
    CHECK(p.last_tier1().stop_reason == StopReason::kSoftEstop);
    CHECK(p.last_tier1().vx == 0.0);

    // And it converges: once the upstream echoes OUR generation, motion is
    // allowed again. A hold that never released would be a robot that needs a
    // restart after every stop.
    p.OnCmdVel(0.68, 1.0, 0.0, 0.0, p.estop_epoch());
    p.CtrlTick(0.69);
    CHECK(p.last_tier1().stop_reason == StopReason::kNone);
  }

  // ---- HES in the readback latches, and the link stays up ---------------
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Send(BasicFrame(1, 17, 0x3002, /*hes=*/true, false));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.6);
    // The hardware stop wins over everything, including the absent command.
    CHECK(p.last_tier1().stop_reason == StopReason::kHes);
    CHECK(p.last_tier1().hes_lock == true);
    // ...and it is a STOP, not a link failure: the chassis is talking to us.
    CHECK(p.conn_state() == chs_a::ConnState::kOk);
  }

  // ---- the sleep flag reaches the session -------------------------------
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Send(BasicFrame(1, 17, 0x3002, false, /*sleep=*/true));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.6);
    // 13 F-21: asleep means no motion command goes out at all, and the gate
    // lives in the session rather than in Tier 1's stop reason.
    CHECK(p.motion_allowed() == false);
  }

  // ---- a motion report drives the odometry ------------------------------
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    // The captured motion report: the machine was at rest, so the velocities
    // are zero and the covariance is what moves.
    chassis.Send(golden.at("RX_00100001_00f00000"));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.6);
    p.CtrlTick(0.61);
    // A sample arrived, so the odometry is publishing rather than stopped.
    CHECK(p.last_odom().publish == true);
    CHECK(p.last_odom().var_x > 0.0);
  }

  // ---- the enable unlocks, and then BOTH gates decide --------------------
  //
  // Every case above leaves Tier 1 locked by the opening silence, so the axis
  // branch of CtrlTick was never reached and the SESSION half of its gate was
  // never the thing that decided. This case reaches it.
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    // Navigation mode, standing. 17 and 0x3002 are the READ-BACK values the
    // machine actually returns (13 V-66); 1 is what the manual says.
    chassis.Send(BasicFrame(1, 17, 0x3002, false, false));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.6);
    CHECK(p.last_tier1().timeout_lock == true);

    chassis.ClearSent();
    chassis.Drain();
    // A fresh command AND an enable: 11 S9.12.1 needs both, and the enable
    // alone against a stale command must not move anything.
    p.OnCmdVel(0.61, 0.5, 0.0, 0.1, p.estop_epoch());
    p.OnEnable();
    p.CtrlTick(0.62);
    CHECK(p.last_tier1().timeout_lock == false);

    // The period AFTER the unlock is the first that can move: the unlocking
    // period still returns a stop (11 S9.12.1 -- the enable clears the lock, it
    // does not command motion).
    // *** Over the limit on purpose: max_vx_mps is 2.0 and this asks for 9.
    // What goes on the wire must be the Tier 1 OUTPUT, not the request.
    p.OnCmdVel(0.63, 9.0, 0.0, 5.0, p.estop_epoch());
    p.CtrlTick(0.64);
    CHECK(p.last_tier1().stop_reason == StopReason::kNone);
    CHECK(p.axis_frames_sent() >= 1);
    chassis.Drain();
    std::uint32_t type = 0, cmd = 0;
    CHECK(chassis.CountFrames(&type, &cmd) >= 1);
    CHECK(type == chs_a::kRealAxis.type);
    CHECK(cmd == chs_a::kRealAxis.command);
    double wx = 0.0, wy = 0.0, ww = 0.0;
    CHECK(chassis.LastAxis(&wx, &wy, &ww));
    CHECK(wx == p.last_tier1().vx);
    CHECK(ww == p.last_tier1().wz);
    // ...and the limits are what they are clamped TO, so the assertion above
    // cannot be satisfied by a Tier 1 that passes the request through.
    CHECK(wx <= 2.0);
    CHECK(ww <= 0.8);

    // *** The enable is ONE SHOT. If it were latched, the next timeout would
    // unlock itself on the following period and the lock would never hold.
    // Let the command go stale: Tier 1 locks, and it STAYS locked.
    p.CtrlTick(1.2);
    CHECK(p.last_tier1().stop_reason == StopReason::kTimeout);
    CHECK(p.last_tier1().timeout_lock == true);
    p.OnCmdVel(1.21, 0.5, 0.0, 0.1, p.estop_epoch());
    p.CtrlTick(1.22);
    CHECK(p.last_tier1().timeout_lock == true);   // no second enable arrived
  }

  // ---- the SESSION gate alone stops an otherwise-legal command ----------
  //
  // The case the double gate exists for: Tier 1 says kNone and there is no
  // link. Tier 1 cannot see that -- it judges the COMMAND, and the command is
  // fresh, inside the limits, in navigation mode, with no HES -- so the second
  // half of the && is the only thing that stops a frame being written into a
  // socket that is reconnecting.
  //
  // Sleep does NOT produce this case, which is worth recording because it was
  // the first thing tried: 13 F-21 gates sleep in the session, but Tier 1 also
  // carries a sleep stop reason, so both halves refuse and the case proves
  // nothing about either.
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Send(BasicFrame(1, 17, 0x3002, false, false));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.6);
    p.OnCmdVel(0.61, 0.5, 0.0, 0.1, p.estop_epoch());
    p.OnEnable();
    p.CtrlTick(0.62);
    p.OnCmdVel(0.63, 0.5, 0.0, 0.1, p.estop_epoch());
    p.CtrlTick(0.64);
    CHECK(p.last_tier1().stop_reason == StopReason::kNone);
    const std::uint64_t sent_while_awake = p.axis_frames_sent();
    CHECK(sent_while_awake >= 1);

    // Now the reports stop. state_timeout_lost_s is 3 s, so the session walks
    // ok -> degraded -> lost while the last readback it holds still says
    // navigation and standing -- which is exactly why Tier 1 goes on saying
    // yes. Commands keep arriving fresh throughout.
    double t = 0.66;
    for (; t < 6.0 && p.conn_state() == chs_a::ConnState::kOk; t += 0.01) {
      p.OnCmdVel(t, 0.5, 0.0, 0.1, p.estop_epoch());
      p.CtrlTick(t + 0.001);
    }
    CHECK(p.conn_state() != chs_a::ConnState::kOk);
    // Keep driving. Everything Tier 1 can see is still fine, and it says so.
    const std::uint64_t sent_at_loss = p.axis_frames_sent();
    for (int i = 0; i < 200; ++i, t += 0.01) {
      p.OnCmdVel(t, 0.5, 0.0, 0.1, p.estop_epoch());
      p.CtrlTick(t + 0.001);
    }
    CHECK(p.motion_allowed() == false);
    CHECK(p.last_tier1().stop_reason == StopReason::kNone);
    // Not one frame in 200 periods. Dropping the session half of the gate
    // writes every one of them into a socket that is reconnecting.
    CHECK(p.axis_frames_sent() == sent_at_loss);
  }

  // ---- ctrl offers every tick to rt_pub, and offers the STOPS too -------
  //
  // 13 S9.1 v1.11 / V-69: ctrl integrates and hands over, rt_pub publishes.
  // The hand-off is a slot, so what is asserted is "the newest one is there",
  // never "all of them arrived" -- a publisher that falls behind must send the
  // current pose, not work through a backlog (RTC-6).
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Send(golden.at("RX_00100001_00f00000"));   // the captured motion report
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);

    p.CtrlTick(0.6);
    CHECK(p.odom_offered() == 2);          // the dial tick and this one
    OdomSample got;
    CHECK(p.TakeOdomForPublish(&got) == true);
    // The SAME sample ctrl computed, not a default-constructed one.
    CHECK(got.x == p.last_odom().x);
    CHECK(got.y == p.last_odom().y);
    CHECK(got.yaw == p.last_odom().yaw);
    CHECK(got.var_x == p.last_odom().var_x);
    CHECK(got.publish == p.last_odom().publish);

    // Taken once, gone. A publisher must not resend the same integration as if
    // it were a new one -- downstream would read a stalled robot as a moving
    // one whose pose happens not to change.
    OdomSample again;
    CHECK(p.TakeOdomForPublish(&again) == false);

    // Falling behind costs the intermediate samples, not the current one.
    for (int i = 0; i < 10; ++i) p.CtrlTick(0.61 + 0.01 * i);
    CHECK(p.odom_offered() == 12);
    CHECK(p.TakeOdomForPublish(&got) == true);
    CHECK(got.x == p.last_odom().x);       // the NEWEST, not the oldest
    CHECK(p.TakeOdomForPublish(&again) == false);

    // *** The ticks that say "do not publish" are offered too. Skipping them
    // would leave rt_pub holding the last good pose forever, and 13 S4.4 (4)
    // requires the TF to stop with the odometry -- a frozen TF makes Nav2
    // believe the robot is stationary and keep commanding rotation.
    //
    // No reports for well past stale_stop_publish_ms, so the band reaches kStop.
    const std::uint64_t before = p.odom_offered();
    for (double t = 0.8; t < 3.0; t += 0.01) p.CtrlTick(t);
    CHECK(p.odom_offered() > before);
    CHECK(p.last_odom().publish == false);
    CHECK(p.TakeOdomForPublish(&got) == true);
    CHECK(got.publish == false);           // the DECISION reached the publisher
  }

  // ---- the state snapshot ctrl hands to rt_pub --------------------------
  //
  // Same slot discipline as the odometry, and asserted HERE rather than only in
  // the bridge's tests: building it is the process's behaviour, and a mutant
  // that stops offering it has to be caught by the process's own suite.
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());

    QuadrupedProcess::StateSnapshot snap;
    CHECK(p.TakeStateForPublish(&snap) == true);
    // Taken once, gone -- a publisher must not resend one period's state as if
    // it were the next.
    QuadrupedProcess::StateSnapshot again;
    CHECK(p.TakeStateForPublish(&again) == false);

    // The connection is the REAL one, not a constant. Before any report has
    // arrived the session is still probing, and a snapshot that reported "ok"
    // here would be the process claiming a link it does not have.
    CHECK(snap.conn == p.conn_state());
    CHECK(snap.conn != chs_a::ConnState::kOk);

    chassis.Send(BasicFrame(1, 17, 0x3002, false, false));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.6);
    CHECK(p.TakeStateForPublish(&snap) == true);
    CHECK(snap.conn == chs_a::ConnState::kOk);     // ...and it followed
    CHECK(snap.estop_epoch == p.estop_epoch());
    CHECK(snap.tier1.stop_reason == p.last_tier1().stop_reason);
    CHECK(snap.cmd_age_ms < 0.0);                  // nothing commanded yet

    // *** The generation disagreement, which is what holds zero (13 S9.12.2
    // (3)). A stop advances ours; a command still echoing the old one must show
    // up here, or an operator looking at state/robot sees a stopped robot with
    // no reason attached.
    const std::uint64_t before = p.estop_epoch();
    p.OnSoftEstop(0.61);
    CHECK(p.estop_epoch() == before + 1);
    p.OnCmdVel(0.62, 0.2, 0.0, 0.0, before);       // the OLD generation
    p.CtrlTick(0.63);
    CHECK(p.TakeStateForPublish(&snap) == true);
    CHECK(snap.soft_estop_active == true);
    CHECK(snap.estop_epoch == before + 1);
    CHECK(snap.cmd_age_ms > 0.0);

    // ...and it clears by itself once the upstream catches up. It is not a
    // lock, and reporting it as one would send someone looking for an unlock.
    p.OnCmdVel(0.64, 0.2, 0.0, 0.0, p.estop_epoch());
    p.CtrlTick(0.65);
    CHECK(p.TakeStateForPublish(&snap) == true);
    CHECK(snap.soft_estop_active == false);
  }

  // ---- the four report streams reach the sink, the fifth does not -------
  //
  // Every frame here came off the chassis on 2026-09-15. That matters more than
  // usual for this case: the routing is by Type code, and a frame this
  // repository invented would be routed by the same constant the code uses --
  // which proves the constant matches itself and nothing else.
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());

    struct Seen {
      std::uint64_t basic = 0, motion = 0, device = 0, fault = 0;
      int both = 0;
      double last_now = -1.0;
      std::string model, version;
      std::size_t battery_count = 0;
      std::size_t fault_count = 0;
    } seen;

    p.SetReportSink([&seen](double now, const chs_a::BasicStatus* b,
                            const chs_a::MotionStatus* m,
                            const chs_a::DeviceStatus* d,
                            const chs_a::FaultReport* f) {
      seen.last_now = now;
      int n = 0;
      if (b != nullptr) { ++seen.basic; ++n; seen.model = b->model; seen.version = b->version; }
      if (m != nullptr) { ++seen.motion; ++n; }
      if (d != nullptr) { ++seen.device; ++n; seen.battery_count = d->batteries.size(); }
      if (f != nullptr) { ++seen.fault; ++n; seen.fault_count = f->faults.size(); }
      // Exactly one report per call. A sink that had to guess which pointer is
      // live would eventually read the wrong one, and the bug would look like
      // a chassis that reports the wrong thing.
      if (n != 1) ++seen.both;
    });

    chassis.Send(golden.at("RX_00100064_00f00000"));   // basic
    chassis.Send(golden.at("RX_00100001_00f00000"));   // motion
    chassis.Send(golden.at("RX_00100002_00f00000"));   // device
    chassis.Send(golden.at("RX_0010007f_00f00000"));   // fault
    // *** A type this build does not model, and with NO ErrorCode.
    //
    // The capture's location report cannot serve here: it carries an ErrorCode
    // (measured 2026-09-17) and is consumed by RxPump's error branch, so it
    // never reaches the unmodelled-report path at all. This frame does.
    chassis.Send(TypedReportFrame(0x00100099u, chs_a::kReportCommand));
    // Pumped from a NON-ZERO time on purpose. All five frames are already in
    // the socket buffer, so they come out on the first call -- starting at 0.0
    // would make the clock assertion below pass against a sink that invented
    // its own timestamp, which is the thing it is there to catch.
    for (int i = 0; i < 200; ++i) p.RxPump(5.0 + 0.01 * i);

    CHECK(seen.basic == 1);
    CHECK(seen.motion == 1);
    CHECK(seen.device == 1);
    CHECK(seen.fault == 1);
    CHECK(seen.both == 0);
    // Four forwarded, five received: the unmodelled report counts as liveness
    // and is NOT forwarded, because putting bytes nobody parsed onto a key with
    // a schema is how a consumer starts seeing fields that were never there.
    CHECK(p.reports_forwarded() == 4);
    CHECK(p.frames_received() >= 5);
    // The sink is handed the CALLER's clock, not one it read itself: every
    // age in this process is measured on one monotonic reading per tick
    // (CLK-C1, mono_clock.h), and a layer that reads its own would be a second
    // time base inside one program.
    CHECK(seen.last_now == 5.0);

    // The strings really came through -- this is the half a POD snapshot cannot
    // carry, and the reason the sink exists at all.
    CHECK(seen.model == "CA9C");
    CHECK(!seen.version.empty());
    CHECK(seen.battery_count > 0);

    // ...and the link is up, because an arriving report is the evidence
    // (13 CA-7) no matter which kind it was.
    p.CtrlTick(2.5);
    CHECK(p.conn_state() == chs_a::ConnState::kOk);

    // *** A fault frame is forwarded whatever its Command field says.
    //
    // 13 S7.3 makes the fault stream "2 Hz PLUS on change", and the
    // change-driven frame is the one an operator is waiting for. We do NOT know
    // what Command the vendor puts on it -- the capture contains only the
    // periodic form -- so the code does not gate on Command at all, and this
    // case pins that choice. See the note in process.cc: forwarding a frame
    // whose Command we did not expect is recoverable; dropping the frame that
    // reports a new fault is not.
    const std::uint64_t before_fault = seen.fault;
    chassis.Send(WithCommand(golden.at("RX_0010007f_00f00000"), 1u));
    for (int i = 0; i < 100; ++i) p.RxPump(7.0 + 0.01 * i);
    CHECK(seen.fault == before_fault + 1);
  }

  // ---- no sink installed: nothing crashes, nothing is counted -----------
  //
  // The production binary installs one, but the offline tests and any future
  // consumer that only wants control do not. An unset std::function called is
  // undefined behaviour, so the guard is asserted rather than assumed.
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Send(golden.at("RX_00100064_00f00000"));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    CHECK(p.frames_received() >= 1);
    CHECK(p.reports_forwarded() == 0);
  }

  // ---- 13 S4.2's two priority tables, including the fall BACK ------------
  //
  // The fallback is the half worth testing. Both sources feed the same
  // odometry, so a wrong choice produces a pose that is merely less accurate --
  // no error, no warning, and a covariance that does not know the difference.
  // The only way it becomes visible is by being reported, so it is reported and
  // that report is what these cases assert.
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    // Nothing from either source yet.
    CHECK(p.angular_source() == QuadrupedProcess::OdomSource::kNone);

    // The monitor report arrives: the 10 Hz source, priority 2 on both tables.
    chassis.Send(golden.at("RX_00100001_00f00000"));
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.60);
    CHECK(p.angular_source() == QuadrupedProcess::OdomSource::kMonitor);
    CHECK(p.linear_source() == QuadrupedProcess::OdomSource::kMonitor);

    // *** /IMU arrives. Priority 1 on the angular table, and it takes over.
    p.OnImu(0.61, 0.25);
    p.CtrlTick(0.62);
    CHECK(p.angular_source() == QuadrupedProcess::OdomSource::kDrdds);

    // *** and now it stops. imu_age_warn_ms is 50 in the fixture, so by +0.2 s
    // the IMU is stale and 13 S4.2 says fall back to the 10 Hz source. A
    // process that kept integrating the last IMU sample would produce a yaw
    // that is smooth, plausible, and frozen.
    p.CtrlTick(0.85);
    CHECK(p.angular_source() == QuadrupedProcess::OdomSource::kMonitor);

    // ...and it comes back when the IMU does. The fallback is a state, not a
    // latch: a degradation that never clears means one dropped packet costs the
    // good source until a restart.
    p.OnImu(0.86, 0.25);
    p.CtrlTick(0.87);
    CHECK(p.angular_source() == QuadrupedProcess::OdomSource::kDrdds);

    // *** /MOTION_INFO: priority 1 on the LINEAR table, chosen by being newer.
    p.OnMotionInfo(0.88, 0.4, 0.0);
    p.CtrlTick(0.89);
    CHECK(p.linear_source() == QuadrupedProcess::OdomSource::kDrdds);
    CHECK(p.last_odom().publish == true);

    // ...and a NEWER monitor report wins it back, with no threshold anywhere:
    // that is the whole point of choosing by arrival time rather than by a
    // constant CLAUDE.md 3.1 would not let us invent.
    chassis.Send(golden.at("RX_00100001_00f00000"));
    for (int i = 0; i < 50; ++i) p.RxPump(1.0 + 0.01 * i);
    p.CtrlTick(1.6);
    CHECK(p.linear_source() == QuadrupedProcess::OdomSource::kMonitor);
  }

  // ---- the IMU yaw really reaches the integration ------------------------
  //
  // Asserting the SOURCE alone would pass on an implementation that recorded
  // the choice and integrated the other value.
  {
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Send(golden.at("RX_00100001_00f00000"));   // captured AT REST: wz = 0
    for (int i = 0; i < 50; ++i) p.RxPump(0.01 * i);
    p.CtrlTick(0.60);
    const double yaw_at_rest = p.last_odom().yaw;

    // A real turn rate from the IMU, integrated over ten periods.
    for (int i = 0; i < 10; ++i) {
      p.OnImu(0.61 + 0.01 * i, 0.5);
      p.CtrlTick(0.615 + 0.01 * i);
    }
    // 0.5 rad/s over ~0.1 s. The bound is loose on purpose -- what is asserted
    // is that the IMU value was INTEGRATED, not that the integrator is exact
    // (13 S4.4's numbers are T-ODOM-2's job).
    const double turned = p.last_odom().yaw - yaw_at_rest;
    CHECK(turned > 0.02);
    CHECK(turned < 0.12);
  }

  // ---- many periods without a chassis: no crash, no motion --------------
  {
    // Nothing is listening. The process must survive the whole probe and
    // backoff cycle, keep reporting, and never emit an axis command.
    QuadrupedProcess p(Cfg(1));   // port 1: nothing there
    for (int i = 0; i < 2000; ++i) {
      p.CtrlTick(0.01 * i);
      p.RxPump(0.01 * i);
    }
    CHECK(p.ctrl_ticks() == 2000);
    CHECK(p.axis_frames_sent() == 0);
    CHECK(p.last_tier1().stop_reason == StopReason::kTimeout);
    CHECK(p.conn_state() != chs_a::ConnState::kOk);
  }

  // ---- the odom sample carries its OWN stamp (13 S9.1 rt_pub row) --------
  {
    // 13 S9.1 verbatim: "时间戳取自样本, 不取自发布时刻 => 发布晚了是到得晚,
    // 不是数据错". ctrl stamps at integration time and the value rides the
    // lock-free slot; rt_pub hands THAT to the ROS header rather than reading
    // a clock of its own.
    //
    // Why it matters and why it is invisible without a case like this: a
    // publish loop that lost its slot to the scheduler would re-label a 40 ms
    // old pose as current, and the very jitter V-69 traded away would stop
    // being observable downstream. The measurement would look BETTER the worse
    // the scheduling got.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    OdomSample first;
    // mutant: leave stamp_wall_s at its 0.0 default -> red. A zero stamp
    // reaches ROS as 1970 and every consumer that filters on age drops it.
    CHECK(p.TakeOdomForPublish(&first));
    CHECK(first.stamp_wall_s > 1.0e9);        // a real epoch, not the default

    // Sub-second resolution. WallNow() elsewhere in this file is ::time(),
    // i.e. WHOLE SECONDS -- reusing it here would give all hundred samples in
    // a second the same stamp, and downstream would see a 100 Hz stream whose
    // timestamps advance in 1 Hz steps. That reads as a stalled publisher, so
    // the defect would be reported as the opposite of what it is.
    // mutant: stamp with WallNow() instead of WallNowSeconds() -> the two
    // stamps below become equal -> red.
    p.CtrlTick(0.01);
    OdomSample second;
    CHECK(p.TakeOdomForPublish(&second));
    CHECK(second.stamp_wall_s > first.stamp_wall_s);

    // The stamp travels WITH the sample through the slot. A stamp written
    // beside the slot rather than inside the payload would be read by rt_pub
    // at a different moment than the pose it labels.
    // mutant: stamp after the Publish call -> the sample taken here carries
    // the PREVIOUS tick's stamp -> the monotonic check above goes red.
    CHECK(second.stamp_wall_s - first.stamp_wall_s < 5.0);
  }

  // ---- a 10 Hz report must not erase what the 2 Hz one delivered ---------
  {
    // Found on the bench 2026-09-18. BasicStatus (2 Hz) is the ONLY report
    // carrying ControlUsageMode; MotionStatus (10 Hz) does not mention it.
    // `latest_ = fresh` therefore reset usage_mode to the struct default five
    // times out of six.
    //
    // *** What it cost: Tier 1 holds every axis command at zero unless
    // usage_mode equals navigation (NAV-111). Reset at 10 Hz it could never
    // equal navigation, so the robot could not be commanded to move AT ALL --
    // and it reported mode_mismatch, which reads as "the chassis is in the
    // wrong mode" rather than "we are dropping the field on the floor".
    //
    // Asserted through STOP_REASON rather than through the snapshot, because
    // the stop reason is the consequence an operator sees and the snapshot is
    // an implementation detail. mutant: restore `latest_ = fresh` -> the
    // second stop_reason check goes red.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Drain();

    // Navigation mode, standing, navigation gait -- everything Tier 1 needs.
    chassis.Send(BasicFrame(/*usage_mode=*/1, /*motion_state=*/17,
                            /*gait=*/0x3002, /*hes=*/false, /*sleep=*/false));
    p.RxPump(0.05);
    // enable clears the boot-time timeout_lock. WITHOUT it the stop reason is
    // kTimeout in every implementation, and an assertion phrased as
    // "not kModeMismatch" is then true whatever the merge does -- which is
    // exactly how the first draft of this case passed the mutant it was
    // written to catch (CLAUDE.md S3.2 form 1).
    p.OnEnable();
    p.OnCmdVel(0.06, 0.1, 0.0, 0.0, p.estop_epoch());
    // Two periods, not one: the enable CLEARS the lock on the period that
    // consumes it but that period still stops (tier1.cc "the upstream came
    // back is not the same event as the upstream is trusted", 11 S9.12.1).
    // Asserting after one period would fail on a correct implementation.
    p.CtrlTick(0.06);
    p.OnCmdVel(0.07, 0.1, 0.0, 0.0, p.estop_epoch());
    p.CtrlTick(0.07);
    // kNone, not "not kModeMismatch": the gate is either fully open or it is
    // not, and the weaker phrasing cannot tell the two implementations apart.
    CHECK(p.last_tier1().stop_reason == StopReason::kNone);
    CHECK(p.axis_frames_sent() >= 1);

    // Now a MotionStatus, which says nothing about usage_mode. The gate must
    // stay open -- this is the assertion the bug broke.
    const std::uint64_t before = p.axis_frames_sent();
    chassis.Send(MotionFrame(/*motion_state=*/17, /*gait=*/0x3002));
    p.RxPump(0.10);
    p.OnCmdVel(0.11, 0.1, 0.0, 0.0, p.estop_epoch());
    p.CtrlTick(0.11);
    CHECK(p.last_tier1().stop_reason == StopReason::kNone);
    CHECK(p.axis_frames_sent() > before);
  }

  // ---- the UDP endpoint frames by DATAGRAM, not by stream (FR-5) --------
  {
    // The gap this closes: Framer::PushDatagram had ZERO production call
    // sites. Every byte went through Push(), the STREAM entry point, on both
    // endpoint candidates -- and 13 S2.2 gives channel one a udp:30004
    // candidate that the resolved config has enabled.
    //
    // *** Why that is not merely untidy. The stream framer keeps leftover
    // bytes across pushes, so a TRUNCATED datagram is concatenated with the
    // next, unrelated one. Two halves of two different reports can then pass
    // the header check together and be handed upward as one frame. FR-5 exists
    // to make that impossible -- one datagram is one frame, anything else is
    // dropped -- because UDP has no ordering that would make a continuation
    // meaningful.
    FakeUdpChassis chassis;
    QuadrupedProcess p(UdpCfg(chassis.port()));
    // A tick so the process connects and sends its first frame, which is what
    // teaches the peer where to reply.
    p.CtrlTick(0.0);
    for (int i = 0; i < 20 && !chassis.LearnPeer(); ++i) p.CtrlTick(0.01 * i);
    CHECK(chassis.LearnPeer());

    // A whole frame in one datagram: accepted.
    const Bytes good = BasicFrame(/*usage_mode=*/1, /*motion_state=*/17,
                                  /*gait=*/0x3002, /*hes=*/false,
                                  /*sleep=*/false);
    chassis.SendRaw(good);
    p.RxPump(0.30);
    CHECK(p.frames_received() == 1);

    // *** The assertion the defect fails. Two halves of that frame, sent as
    // two datagrams. Under FR-5 both are dropped and the counter does not
    // move; under the stream framer the second push completes the first and
    // yields a frame -- which is the concatenation this endpoint must never
    // perform. mutant: route UDP through Push() -> frames_received becomes 2.
    const std::size_t half = good.size() / 2;
    chassis.SendRaw(good.data(), half);
    p.RxPump(0.31);
    chassis.SendRaw(good.data() + half, good.size() - half);
    p.RxPump(0.32);
    CHECK(p.frames_received() == 1);

    // And a datagram carrying TWO frames is one frame too many, not one frame
    // plus a remainder: FR-5 drops it whole.
    Bytes two = good;
    two.insert(two.end(), good.begin(), good.end());
    chassis.SendRaw(two);
    p.RxPump(0.33);
    CHECK(p.frames_received() == 1);

    // Asserted HERE, before any further good frame arrives. FR-5 asks for a
    // `warn` on a refused frame; this process cannot emit events (11 RT-C4),
    // so the count is the part that is ours -- and it has to be visible WHEN
    // THE DROP HAPPENS. On the link FR-5 is actually about, the peer and we
    // disagree about the format and there IS no next good frame; a count
    // published only alongside a successful frame would then read 0 forever,
    // which is the same picture as a silent link.
    //
    // THREE, enumerated so the number is a derivation rather than whatever the
    // first run printed: the truncated first half (header parses, length
    // disagrees), the second half (no sync word at all), and the two-frame
    // datagram. The two whole frames are not refusals.
    // mutant: publish the count only on the success path -> reads 0 here.
    CHECK(p.link_status().dropped == 3);

    // The link still works afterwards -- a dropped datagram must not leave
    // state that poisons the next good one. A framer that kept the leftovers
    // would parse this one against them.
    chassis.SendRaw(good);
    p.RxPump(0.34);
    CHECK(p.frames_received() == 2);

    // And the refusals are VISIBLE. FR-5 asks for a `warn`, which this process
    // cannot emit (11 RT-C4); the count is the part that is ours, and until
    // now nothing in production read it -- the framer had kept the number
    // since day one with no reader, which makes it a number, not a diagnostic.
    // and a good frame afterwards must not disturb the refusal count.
    CHECK(p.link_status().dropped == 3);
  }

  // ---- FR-5 / SD-3: TCP_NODELAY is VERIFIED, not assumed -----------------
  {
    // The gap this closes: Dial called setsockopt and threw the return away,
    // and ChassisSocket::nodelay_enabled() -- which reads the option BACK from
    // the kernel -- had no caller anywhere in production. So "Nagle is off"
    // was a guarantee nobody held (CLAUDE.md S3.2), while every latency figure
    // in 13 S3.6 rests on it.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    // Before any connection the verdict is optimistic on purpose: there is
    // nothing to complain about yet, and a false here would have the
    // supervisor report a timing fault on a process that has not dialled.
    CHECK(!p.link_status().nodelay_expected);
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    // *** Drive until the session actually reports a connection, and ASSERT
    // that it did. The first draft ticked twice and moved on -- the connect
    // event had not fired, so nodelay_ok was still its initial true and the
    // case passed without exercising anything. Found by a surviving mutant.
    for (int i = 0; i < 40; ++i) {
      chassis.Send(BasicFrame(1, 17, 0x3002, false, false));
      p.RxPump(0.01 * i);
      p.CtrlTick(0.01 * i);
    }
    CHECK(p.conn_state() == chs_a::ConnState::kOk);
    // The config asked, so we had business asking; the kernel says it is on.
    CHECK(p.link_status().nodelay_expected);
    CHECK(p.link_status().nodelay_active);
  }
  {
    // A DATAGRAM endpoint. TCP_NODELAY is meaningless on one, and
    // nodelay_enabled() answers false for it by construction -- so a check
    // that only asked "did the kernel say yes" would report a timing fault on
    // every connection to the udp:30004 candidate 13 S8.2 has enabled.
    //
    // This is also what makes "assume true instead of reading back"
    // observable: on TCP both answers agree, and only here do they part.
    FakeUdpChassis chassis;
    QuadrupedProcess p(UdpCfg(chassis.port()));
    p.CtrlTick(0.0);
    for (int i = 0; i < 20 && !chassis.LearnPeer(); ++i) p.CtrlTick(0.01 * i);
    CHECK(chassis.LearnPeer());
    for (int i = 0; i < 40; ++i) {
      chassis.SendRaw(BasicFrame(1, 17, 0x3002, false, false));
      p.RxPump(0.01 * i);
      p.CtrlTick(0.01 * i);
    }
    CHECK(p.conn_state() == chs_a::ConnState::kOk);
    // mutant: drop the is_udp term -> expected goes true on a socket the
    // option does not apply to, and the supervisor reports a timing fault.
    CHECK(!p.link_status().nodelay_expected);
  }
  {
    // With the config NOT asking for it, the verdict stays true -- the check
    // is "did we get what we asked for", not "is Nagle off unconditionally".
    // mutant: demand it regardless of the config -> red, because nothing ever
    // called setsockopt on this socket.
    FakeChassis chassis;
    QuadrupedConfig c = Cfg(chassis.port());
    c.link.tcp_nodelay = false;
    QuadrupedProcess p(c);
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    for (int i = 0; i < 40; ++i) {
      chassis.Send(BasicFrame(1, 17, 0x3002, false, false));
      p.RxPump(0.01 * i);
      p.CtrlTick(0.01 * i);
    }
    CHECK(p.conn_state() == chs_a::ConnState::kOk);
    CHECK(!p.link_status().nodelay_expected);
    // *** And ACTIVE is false -- nobody called setsockopt on this socket.
    // This is the assertion that makes the read-back observable at all: on a
    // socket where it WAS set the two are both true, so only here can a
    // hardcoded `true` be told apart from asking the kernel.
    CHECK(!p.link_status().nodelay_active);
  }

  // ---- 13 GS-1: stair_standard is REFUSED, not commanded -----------------
  {
    // ModeConfig has TWO gait lists, and the second one was missed when the
    // first was wired -- GS-1 stayed dead a batch longer for exactly that
    // reason. Without command_forbidden_gaits, GaitCommandable returns true
    // for everything and 0x1003 goes out to a chassis that can never read it
    // back (13 G-02, "读回枚举中无此值") -- so the read-back check has nothing
    // to match and MS-2 turns the request into a five-second timeout instead
    // of an immediate, honest refusal.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Send(BasicFrame(1, 17, 0x3002, false, false));
    p.RxPump(0.05);
    p.CtrlTick(0.06);
    chassis.Drain();
    chassis.ClearSent();

    const ModeRequestResult r = p.OnChassisAction(0.1, ModeAction::kSetGait, 0x1003);
    CHECK(!r.accepted);
    // The REASON. A refusal for "a switch is in flight" would satisfy
    // !accepted and mean something else entirely.
    CHECK(std::string(ModeRejectItem(r.reject)) == "gait_readback_gap");
    // And NOTHING went on the wire. 13 GS-1: 不静默丢弃, 不假装成功 -- but
    // also not sent. A counter-only check would pass on an implementation
    // that refused upward while still writing the frame.
    p.CtrlTick(0.11);
    chassis.Drain();
    CHECK(chassis.CountFrames(nullptr, nullptr) == 0);

    // A commandable gait still goes through, so the assertions above are not
    // satisfied by a build that refuses every gait.
    CHECK(p.OnChassisAction(0.2, ModeAction::kSetGait, 0x3002).accepted);
  }

  // ---- every ModeConfig field is actually wired --------------------------
  {
    // *** This case exists because of HOW the last two defects were found.
    // The process built its ModeConfig from a lambda that discarded cfg, and
    // the four fields were fixed in two batches -- the second gait list looked
    // done because the first one had been. Asserting the struct field by field
    // is the only thing that makes "a field nobody wired" visible without a
    // chassis and without waiting for the behaviour to be noticed.
    QuadrupedConfig c = Cfg(1);
    c.motion.mode_switch_timeout_s = 7.5;
    c.motion.external_transition_hold_s = 1.25;
    c.motion.prone_forbidden_gaits = {0x3003, 0x1003};
    c.motion.command_forbidden_gaits = {0x1003};
    QuadrupedProcess p(c);
    // Distinctive values throughout: a field wired to the WRONG source, or
    // left at a literal, cannot coincide with all of these.
    CHECK(p.mode_switch_timeout_s_for_test() == 7.5);
    CHECK(p.external_transition_hold_s_for_test() == 1.25);
    CHECK(p.prone_forbidden_gaits_for_test().size() == 2);
    CHECK(p.command_forbidden_gaits_for_test().size() == 1);
    CHECK(p.command_forbidden_gaits_for_test()[0] == 0x1003);
  }

  // ---- 13 TR-1: an EXTERNAL transition holds the robot at zero -----------
  {
    // The gap this closes: Tier 1 was handed mode_switching(), which is only
    // OUR OWN commanded switch. TR-1 says the other case in as many words --
    // a MotionState that changes without our having commanded it must "置
    // mode_switching = true 并保持 external_transition_hold_s, 期间零速".
    //
    // So the factory handset could put the robot into a 2-3 s stand-up while
    // we kept feeding axis commands into it. 13 V-61 is why we cannot see the
    // transition end any other way, and TR-1 calls the alternative "believing
    // a moving robot is stationary".
    //
    // *** Every existing test stayed green when this was fixed, which is the
    // whole reason this case exists: nothing covered it at all.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    // Standing, navigation, flat. First read-back of the process's life, so it
    // is NOT an external transition (there was no previous value to differ
    // from) -- the machine treats it as the steady baseline.
    chassis.Send(BasicFrame(1, 17, 0x3002, false, false));
    p.RxPump(0.05);
    p.CtrlTick(0.06);
    CHECK(!p.motion_state_transitioning());

    // Unlock and prove the robot CAN move here. Without this half, every
    // assertion below is satisfied by a robot that never moves at all.
    p.OnCmdVel(0.07, 0.5, 0.0, 0.1, p.estop_epoch());
    p.OnEnable();
    p.CtrlTick(0.08);
    p.OnCmdVel(0.09, 0.5, 0.0, 0.1, p.estop_epoch());
    p.CtrlTick(0.10);
    CHECK(p.last_tier1().stop_reason == StopReason::kNone);
    CHECK(p.last_tier1().vx > 0.0);

    // Now the handset lies the robot down: MotionState 17 -> 0, and WE never
    // commanded it.
    chassis.Send(BasicFrame(1, 0, 0x3002, false, false));
    p.RxPump(0.15);
    p.OnCmdVel(0.16, 0.5, 0.0, 0.1, p.estop_epoch());
    p.CtrlTick(0.17);
    CHECK(p.motion_state_transitioning());
    CHECK(p.last_tier1().stop_reason == StopReason::kModeSwitching);
    // ZERO, on the axis that was moving a moment ago.
    CHECK(p.last_tier1().vx == 0.0);
    // *** And the PUBLISHED field says so too. 13 TR-1 requires the external
    // change to set mode_switching, and a state key reading false while the
    // robot is held at zero leaves the operator with no explanation for a
    // robot that stopped. Read from the snapshot rt_pub actually publishes,
    // not from the accessor above -- nothing covered the published field, and
    // a mutant that reverted only that line survived until this assertion.
    {
      QuadrupedProcess::StateSnapshot snap;
      CHECK(p.TakeStateForPublish(&snap));
      CHECK(snap.mode_switching);
    }

    // Still held most of the way through the window. Without this, an
    // implementation that released on the next period passes the check above.
    for (double t = 0.2; t < 3.5; t += 0.1) {
      if (t > 3.0 && t < 3.1) chassis.Send(BasicFrame(1, 0, 0x3002, false, false));
      p.RxPump(t);
      p.OnCmdVel(t, 0.5, 0.0, 0.1, p.estop_epoch());
      p.CtrlTick(t);
    }
    CHECK(p.motion_state_transitioning());
    CHECK(p.last_tier1().vx == 0.0);

    // And released after it. A hold that never expired would leave the robot
    // at zero for the rest of the session after one handset press -- and the
    // configured external_transition_hold_s would be a key that changed
    // nothing, which is the defect this package keeps finding.
    for (double t = 3.5; t < 4.3; t += 0.1) {
      if (t > 3.9 && t < 4.0) chassis.Send(BasicFrame(1, 0, 0x3002, false, false));
      p.RxPump(t);
      p.OnCmdVel(t, 0.5, 0.0, 0.1, p.estop_epoch());
      p.CtrlTick(t);
    }
    CHECK(!p.motion_state_transitioning());
  }

  // ---- 13 MS-2: the switch window comes from CONFIG, not a literal -------
  {
    // The window was a literal 5.0 in the process constructor while
    // motion.mode_switch_timeout_s sat in the config doing nothing -- 13 v1.7's
    // sentence for special_gaits applies verbatim: 填了不生效 = 让设置的人
    // 以为改了什么.
    //
    // *** The fixture uses 2.0 on purpose. With 5.0 there, "hardcode it back
    // to 5.0" is an EQUIVALENT mutation and no assertion could tell the two
    // apart -- measured, that mutant survived until the fixture changed.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    // A steady read-back first, so the request is not refused for having no
    // triple to reason from.
    chassis.Send(BasicFrame(/*usage_mode=*/1, /*motion_state=*/17,
                            /*gait=*/0x3002, /*hes=*/false, /*sleep=*/false));
    p.RxPump(0.05);
    p.CtrlTick(0.06);

    // Ask for a gait the chassis will never confirm -- no read-back is sent
    // after this point, which is exactly the case MS-2 exists for.
    CHECK(p.OnChassisAction(0.1, ModeAction::kSetGait, 0x3003).accepted);
    p.CtrlTick(0.11);
    CHECK(p.mode_switching());

    // Still in flight just before the configured window closes. Without this
    // half, an implementation that gave up immediately would pass the check
    // below.
    for (double t = 0.2; t < 1.9; t += 0.1) p.CtrlTick(t);
    CHECK(p.mode_switching());
    CHECK(p.mode_switch_failures() == 0);

    // And failed just after it. A hardcoded 5.0 is still waiting here.
    for (double t = 1.9; t < 2.6; t += 0.1) p.CtrlTick(t);
    CHECK(!p.mode_switching());
    CHECK(p.mode_switch_failures() == 1);
  }

  // ---- PR-1 reaches the process: prone REFUSED on a staircase ------------
  {
    // *** THE DEFECT THIS CLOSES, and it is the worst one in this file.
    // QuadrupedProcess built its ModeConfig from a lambda that took cfg and
    // threw it away with (void)cfg. prone_forbidden_gaits was therefore EMPTY,
    // and ProneAllowed answers !Contains(list, gait) -- true for every gait.
    // PR-1 never fired: `prone` was accepted on a staircase, and 13 V-54 calls
    // that a safety incident outright ("楼梯上不防侧翻 = 安全事故", P0).
    //
    // Everything else was in place: the predicate, the refusal code, the
    // config key with both stair gaits in it, and test_mode_machine.cc, which
    // builds its OWN list and passes either way. That last part is the lesson
    // -- a unit test that supplies the configuration cannot see a process that
    // never reads it. So this case drives the PROCESS with chassis frames.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());

    // *** ORDER MATTERS, and it is not cosmetic. An ACCEPTED prone starts a
    // mode switch, and 13 MS-3 then refuses the next request with
    // mode_switch_in_flight -- which satisfies `!accepted` for the wrong
    // reason and would make the two stair cases below pass on an
    // implementation where PR-1 does nothing. The refusals come FIRST because
    // a refusal starts no switch; the "flat is allowed" half is last.
    //
    // The navigation stair gait, which is the one the robot reaches by itself
    // (13 G-05, 自主上下楼梯).
    chassis.Send(BasicFrame(/*usage_mode=*/1, /*motion_state=*/17,
                            /*gait=*/0x3003, /*hes=*/false, /*sleep=*/false));
    Settle(&p, &chassis, 0.2, 4.2, 0x3003);
    const ModeRequestResult on_stair = p.OnChassisAction(4.3, ModeAction::kProne, 0);
    CHECK(!on_stair.accepted);
    // The REASON, not just the refusal: a prone refused for "a switch is
    // already in flight" would satisfy `!accepted` and mean something else
    // entirely -- and it would clear by itself a second later.
    CHECK(std::string(ModeRejectItem(on_stair.reject)) == "prone_on_stair");

    // 13 GS-3: "我方不发" is not "它不会出现". The factory handset can set
    // stair_standard, and PR-1 refuses prone on whatever the chassis REPORTS.
    // An implementation listing only 0x3003 passes every check above.
    // *** Settled FIRST, and that is the whole point of this step. The first
    // draft asserted right after the frame arrived and passed with the list
    // narrowed to {0x3003} -- because TR-1 still held the STEADY gait at the
    // previous staircase, so the refusal came from the stale value, not from
    // 0x1003 being on the list. An assertion a wrong implementation passes
    // (CLAUDE.md S3.2 form 1), caught by running the mutant.
    chassis.Send(BasicFrame(/*usage_mode=*/1, /*motion_state=*/17,
                            /*gait=*/0x1003, /*hes=*/false, /*sleep=*/false));
    Settle(&p, &chassis, 4.4, 8.4, 0x1003);
    const ModeRequestResult standard = p.OnChassisAction(8.5, ModeAction::kProne, 0);
    CHECK(!standard.accepted);
    CHECK(std::string(ModeRejectItem(standard.reject)) == "prone_on_stair");

    // And back to flat: prone is ALLOWED again. Two things at once -- the
    // refusal does not latch (a latching one would leave the robot unable to
    // lie down for the rest of the sortie after one staircase), and PR-1 is
    // not simply refusing prone unconditionally, which every assertion above
    // would tolerate.
    // *** NOT immediately. The first draft asserted this right after the frame
    // and got prone_on_stair -- and the CODE was right, the test was wrong.
    // 13 TR-2: PR-1 judges on the last STEADY read-back, not on the value seen
    // during a transition, and every gait change here is EXTERNAL (we never
    // commanded one), so TR-1 holds the machine in transition for
    // external_transition_hold_s. During that hold the steady value is still
    // the staircase, and refusing is "正是我们要的方向" in TR-2's own words.
    chassis.Send(BasicFrame(/*usage_mode=*/1, /*motion_state=*/17,
                            /*gait=*/0x3002, /*hes=*/false, /*sleep=*/false));
    Settle(&p, &chassis, 8.6, 12.6, 0x3002);
    CHECK(p.OnChassisAction(12.7, ModeAction::kProne, 0).accepted);
  }

  // ---- the light command puts a FRAME on the wire (C-07 / 11 S9.4.1) -----
  {
    // Asserted on the WIRE, not on a counter. 13 ASM-6 is the reason: the mode
    // path incremented its counter beside a send that never happened, and
    // every layer looked healthy. A counter that moves next to a frame nobody
    // sent is the same defect wearing a number.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Drain();
    chassis.ClearSent();

    chs_a::LedSetting head;
    head.pattern = 5;            // blink
    head.color = 1;              // white
    head.cycle_s = 1;
    chs_a::LedSetting tail;
    tail.pattern = 4;            // breath
    tail.color = 2;              // green
    tail.cycle_s = 2;
    CHECK(p.SendLightFrame(true, head, tail));
    CHECK(p.light_frames_sent() == 1);
    chassis.Drain();

    std::uint32_t type = 0, cmd = 0;
    CHECK(chassis.CountFrames(&type, &cmd) == 1);
    // C-07: Type 0x00100005 / Command 0x00200002 (13 S5.1, vendor guide 1.2.7).
    CHECK(type == 0x00100005u);
    CHECK(cmd == 0x00200002u);
    const std::string wire(reinterpret_cast<const char*>(chassis.sent().data()),
                           chassis.sent().size());
    CHECK(wire.find("\"CustomMode\": true") != std::string::npos);
    // Led is positional -- [0] head, [1] tail. Different colours on the two
    // lamps so a swapped encoder fails here rather than passing on symmetry.
    CHECK(wire.find("\"Color\": [1]") < wire.find("\"Color\": [2]"));
  }

  // ---- a stair gait REACHES the odometry (13 S4.4 (4) / 11 S9.9) ---------
  {
    // The gap this closes: Odometry::OnGait had ZERO production call sites, so
    // is_stair_gait_ stayed at its initialiser for the life of the process. A
    // robot on a staircase therefore published wheel odometry with FLAT-ground
    // trust and valid = true, which is the one thing 11 S9.9 names outright.
    //
    // *** Why the existing unit test did not catch it. test_odometry.cc calls
    // stair.OnGait(true) DIRECTLY and asserts the divisor and the flag, and it
    // passes either way -- it tests the setter, not the wiring. A defect that
    // lives in "nobody calls this" is invisible to every test that calls it.
    // So this case drives the PROCESS with a chassis frame and reads what the
    // process publishes. mutant: drop the odom_.OnGait line -> both stair
    // assertions below go red while test_odometry stays green.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    chassis.Drain();

    // Flat gait first, and a velocity sample so the band is fresh and the
    // sample is actually published -- valid is false on a STALE sample too, so
    // asserting it on a robot that never reported a velocity would pass on an
    // unwired implementation.
    chassis.Send(MotionFrame(/*motion_state=*/17, /*gait=*/0x3002));
    p.RxPump(0.05);
    p.CtrlTick(0.06);
    const OdomSample flat = p.last_odom();
    CHECK(flat.publish);
    CHECK(flat.valid);
    CHECK(flat.var_wz > 0.0);

    // 0x3003, the navigation stair gait. This is the one the robot reaches by
    // itself (G-05, 自主上下楼梯).
    chassis.Send(MotionFrame(/*motion_state=*/17, /*gait=*/0x3003));
    p.RxPump(0.10);
    p.CtrlTick(0.11);
    const OdomSample stair = p.last_odom();
    // Still PUBLISHED -- 13 S4.4's table says 发布 for the stair row. Dropping
    // the message would leave the consumer with no pose at all, which is a
    // different failure from an untrustworthy one.
    CHECK(stair.publish);
    CHECK(!stair.valid);
    // And inflated. 13 takes BOTH measures; an implementation that only cleared
    // the flag would pass the line above and still hand a downstream filter a
    // flat-ground covariance to fuse.
    //
    // *** Asserted on var_wz, which is (gyro_bias^2) / divisor -- a pure
    // function of the trust divisor, with no tau term and no accumulated
    // history. The first draft of this line used var_x, and a mutant that
    // dropped the inflation entirely SURVIVED it: p_xx_committed_ grows tick to
    // tick, so "the later sample has a larger var_x" is true whether or not
    // anything inflates. An assertion a do-nothing implementation passes
    // (CLAUDE.md S3.2 form 1), caught only because the mutant was run.
    //
    // 3.0 rather than the exact 3.333 (trust_flat 1.0 / trust_stair 0.3 in the
    // fixture above): the ratio is a CONFIGURED quantity, and pinning the
    // fixture's arithmetic here would make this case fail when someone retunes
    // the config rather than when the inflation breaks.
    CHECK(stair.var_wz > flat.var_wz * 3.0);

    // 0x1003, the standard-mode stair gait. GS-1 forbids US from commanding it;
    // GS-3 says verbatim that "我方不发" is not "它不会出现" -- the factory
    // handset can set it and the read-back path resolves it. An implementation
    // that only listed 0x3003 passes every test above and fails here.
    chassis.Send(MotionFrame(/*motion_state=*/17, /*gait=*/0x1003));
    p.RxPump(0.15);
    p.CtrlTick(0.16);
    CHECK(!p.last_odom().valid);

    // Back to flat: the flag must CLEAR, not latch. A latching implementation
    // is safe-looking and wrong -- after one staircase the robot would report
    // invalid odometry for the rest of the sortie, and the consumer that has to
    // choose between "always invalid" and "ignore the flag" chooses the second.
    chassis.Send(MotionFrame(/*motion_state=*/17, /*gait=*/0x3002));
    p.RxPump(0.20);
    p.CtrlTick(0.21);
    CHECK(p.last_odom().valid);
    // The INFLATION must clear too, not just the flag -- same reason, and the
    // same var_wz for the same reason.
    CHECK(p.last_odom().var_wz < flat.var_wz * 1.5);

    // Gait 0 at rest is a MEASURED value (13 V-66) that kGaits does not
    // contain, so it resolves to unknown_0x0000. It must NOT be read as a
    // staircase: treating every unregistered code as one would clear valid on
    // every boot before RL control, on the most ordinary report there is.
    chassis.Send(MotionFrame(/*motion_state=*/17, /*gait=*/0));
    p.RxPump(0.25);
    p.CtrlTick(0.26);
    CHECK(p.last_odom().valid);
  }

  // ---- the mode sequence actually puts FRAMES on the wire (13 ASM-6) ------
  {
    // The gap this closes, measured on the bench 2026-09-18: Request returned
    // accepted, switching_ went true, the read-back was waited for -- and
    // NOTHING was sent. stand acked "accepted" while the chassis reported
    // MotionState 0 throughout, so the robot could not be made to move and
    // every layer looked healthy.
    //
    // Asserted on the WIRE, not on a counter alone: a counter that increments
    // beside a send that failed is the same defect wearing a number.
    // mutant: drop the tx_.Send call -> the frame checks below go red.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    // Read back a posture so the machine has a steady triple to reason from
    // (prone is refused without one, and the sequence would stall).
    chassis.Send(BasicFrame(/*usage_mode=*/0, /*motion_state=*/0, /*gait=*/0,
                            /*hes=*/false, /*sleep=*/false));
    p.RxPump(0.01);
    p.CtrlTick(0.02);
    chassis.Drain();

    // The whole triple in one message, as 11 S9.2.4 sends it.
    CHECK(p.OnChassisMode(/*has_usage=*/true, /*usage=*/1,
                          /*has_state=*/true, /*state=*/kCommandMotionStateStand,
                          /*has_gait=*/true, /*gait=*/0x3002));
    CHECK(p.mode_sequence_pending());

    // Step 1 is motion_state -- the order is fixed (13 MS-5), and usage_mode
    // goes last because it is the gate Tier 1 opens on.
    p.CtrlTick(0.03);
    std::uint32_t type = 0, cmd = 0;
    chassis.Drain();
    CHECK(chassis.CountFrames(&type, &cmd) >= 1);
    CHECK(type == chs_a::kMotionStateSwitch.type);
    CHECK(cmd == chs_a::kMotionStateSwitch.command);
    CHECK(p.mode_frames_sent() == 1);

    // The next step waits for the read-back: MS-3 forbids a second switch in
    // flight, so no further frame goes out until the chassis confirms.
    // mutant: dispatch regardless of mode_switching -> this goes red, and two
    // expectations would be outstanding with no way to say which read-back
    // belongs to which.
    chassis.ClearSent();
    p.CtrlTick(0.04);
    chassis.Drain();
    CHECK(chassis.CountFrames(&type, &cmd) == 0);
    CHECK(p.mode_frames_sent() == 1);

    // Confirm the posture; the gait step then goes out.
    chassis.Send(BasicFrame(/*usage_mode=*/0, /*motion_state=*/17,
                            /*gait=*/0x1001, /*hes=*/false, /*sleep=*/false));
    p.RxPump(0.05);
    chassis.ClearSent();
    p.CtrlTick(0.06);
    chassis.Drain();
    CHECK(chassis.CountFrames(&type, &cmd) >= 1);
    CHECK(type == chs_a::kGaitSwitch.type);
    CHECK(cmd == chs_a::kGaitSwitch.command);
    CHECK(p.mode_frames_sent() == 2);

    // Confirm the gait; usage_mode is LAST -- the permissive step.
    chassis.Send(BasicFrame(/*usage_mode=*/0, /*motion_state=*/17,
                            /*gait=*/0x3002, /*hes=*/false, /*sleep=*/false));
    p.RxPump(0.07);
    chassis.ClearSent();
    p.CtrlTick(0.08);
    chassis.Drain();
    CHECK(chassis.CountFrames(&type, &cmd) >= 1);
    CHECK(type == chs_a::kUsageModeSwitch.type);
    CHECK(cmd == chs_a::kUsageModeSwitch.command);
    CHECK(p.mode_frames_sent() == 3);
    CHECK(!p.mode_sequence_pending());
  }

  // ---- the ctrl path sends its frame too (13 ASM-6, second half) ---------
  {
    // The first ASM-6 fix wired only the rt/chassis/mode sequencer. The
    // rt/chassis/ctrl stand/prone path still returned accepted and sent
    // nothing -- measured on the bench within the same hour: the robot stood
    // up through the sequencer and then would NOT lie down, because prone
    // came in on the ctrl key. An operator's stop-what-you-are-doing command
    // silently doing nothing is the worst shape this defect can take.
    // mutant: drop the SendModeFrame call in OnChassisAction -> red.
    FakeChassis chassis;
    QuadrupedProcess p(Cfg(chassis.port()));
    p.CtrlTick(0.0);
    CHECK(chassis.Accept());
    // A steady read-back, so prone is not refused for an unknown gait (PR-1).
    chassis.Send(BasicFrame(/*usage_mode=*/1, /*motion_state=*/17,
                            /*gait=*/0x3002, /*hes=*/false, /*sleep=*/false));
    p.RxPump(0.01);
    p.CtrlTick(0.02);
    chassis.ClearSent();

    const ModeRequestResult r = p.OnChassisAction(0.03, ModeAction::kProne, 0);
    CHECK(r.accepted);
    std::uint32_t type = 0, cmd = 0;
    chassis.Drain();
    CHECK(chassis.CountFrames(&type, &cmd) >= 1);
    CHECK(type == chs_a::kMotionStateSwitch.type);
    CHECK(cmd == chs_a::kMotionStateSwitch.command);
    CHECK(p.mode_frames_sent() == 1);
  }

  if (g_failures == 0) {
    std::printf("ALL PROCESS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d PROCESS TEST(S) FAILED\n", g_failures);
  return 1;
}
