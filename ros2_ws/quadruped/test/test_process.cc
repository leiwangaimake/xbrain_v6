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

  if (g_failures == 0) {
    std::printf("ALL PROCESS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d PROCESS TEST(S) FAILED\n", g_failures);
  return 1;
}
