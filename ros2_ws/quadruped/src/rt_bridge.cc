/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_bridge.cc
 * Brief: RT plane routing (see rt_bridge.h)
 *
 * Description:
 * The order inside HandleEstop is the only thing in this file that is not
 * obvious, and it is the most important: the STOP IS ISSUED FIRST, before the
 * payload is looked at for the ack. Parsing first and stopping afterwards reads
 * identically and behaves identically right up until the parse throws, hangs,
 * or an early return is added by someone "cleaning up" -- and then the stop is
 * gone. Putting the stop above the parse means no edit to the parsing can reach
 * it (11 S3.0.1, 99 U75).
 *
 * Everything else here is routing, and the routing is deliberately dull: one
 * message in, one parse, one call, one ack. No decision that belongs to a lower
 * layer is re-made here -- the stair precondition, the switch window and the
 * lock rules all live in the mode machine and Tier 1, and a second opinion at
 * this level is how two copies of a rule drift apart.
 */

#include "quadruped/rt_bridge.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "quadruped/rt_keys.h"
#include "quadruped/rt_payloads.h"
#include "xbrain/errors/errors.h"

namespace quadruped {
namespace rt {
namespace {

// Big enough for every payload this file writes; the writers all take a cap and
// return 0 rather than truncating, so a buffer that is too small fails loudly
// instead of emitting half a message.
constexpr std::size_t kOutCap = 2048;

namespace err = hachist::xbrain::errors;

// The three keys this file publishes on, by SUFFIX. They are looked up in the
// table at construction (see the constructor): publishing on an undeclared key
// is how a key escapes the set 11 S2.2.1 is checked against, and the lookup is
// what makes that impossible rather than merely discouraged.
constexpr const char* kCtrlAckSuffix = "rt/chassis/ctrl/ack";
constexpr const char* kEstopAckSuffix = "rt/safety/estop/ack";
constexpr const char* kPongSuffix = "rt/safety/probe/pong";
constexpr const char* kStateSuffix = "rt/chassis/state";
// 13 S7.1 Q-5: the four report streams, each on its own key. They are NOT
// folded into rt/chassis/state -- that would put one fact in two places, and
// the state key is built on the ctrl thread where these cannot go (they hold
// std::string, 12 RTC-6).
constexpr const char* kBasicSuffix = "rt/chassis/basic";
constexpr const char* kMotionSuffix = "rt/chassis/motion";
constexpr const char* kDeviceSuffix = "rt/chassis/device";
constexpr const char* kFaultSuffix = "rt/chassis/fault";
constexpr const char* kHelloAckSuffix = "rt/chassis/hello_ack";
// 11 S9.1.4: major.minor, major mismatch is incompatible. Ours is the JSON
// contract's version, NOT the chassis monitor protocol's APDU version byte --
// 11 S9.1.4 says in so many words the two evolve independently and must not
// be mixed.
constexpr const char* kProtoVersion = "1.0";

std::uint64_t ToMs(double s) {
  return s < 0.0 ? 0 : static_cast<std::uint64_t>(s * 1000.0 + 0.5);
}

}  // namespace

RtBridge::RtBridge(QuadrupedProcess* proc, std::string rid, std::string boot,
                   PublishFn publish)
    : proc_(proc),
      rid_(std::move(rid)),
      boot_(std::move(boot)),
      publish_(std::move(publish)) {
  // *** Every key this class publishes on must be IN THE TABLE. 11 S2.2.1 is
  // the closed set of RT-plane keys, rt_keys.cc is its transcription, and a
  // suffix that is not there is a key nobody reviewed. Checked once here, at
  // construction, so the failure is a startup abort rather than a message that
  // goes somewhere nobody is listening.
  for (const char* k : {kCtrlAckSuffix, kEstopAckSuffix, kPongSuffix,
                        kStateSuffix, kBasicSuffix, kMotionSuffix,
                        kDeviceSuffix, kFaultSuffix, kHelloAckSuffix}) {
    if (FindKey(k) == nullptr) {
      std::fprintf(stderr,
                   "rt_bridge: key %s is not declared in rt_keys.cc -- it is "
                   "not part of the 11 S2.2.1 closed set\n", k);
      std::abort();
    }
  }
}

bool RtBridge::Publish(const std::string& suffix, const char* data,
                       std::size_t len) {
  if (!publish_ || len == 0) return false;
  ++acks_;
  return publish_(suffix, data, len);
}

void RtBridge::PublishReports(double now_mono_s,
                             const chs_a::BasicStatus* basic,
                             const chs_a::MotionStatus* motion,
                             const chs_a::DeviceStatus* device,
                             const chs_a::FaultReport* fault) {
  // *** Called from the chs_a_rx thread, NOT from rt_pub. That is the whole
  // reason this is a callback instead of a slot: these four structs hold
  // std::string and cannot cross a LockfreeSlot (12 RTC-6), while forwarding
  // them is not realtime work -- which is exactly what the rx thread is for
  // (13 S9.1).
  //
  // Before this existed, 13 ASM-4 (3) recorded the forwarding as "v1.15 已做"
  // while SetReportSink had ZERO production call sites: all four keys were
  // declared, all four writers were implemented and tested, and not one frame
  // ever went out. Measured 2026-09-18 -- subscribing to xbrain/dev/rt/chassis/**
  // for 12 s returned only rt/chassis/state.
  (void)now_mono_s;
  char out[8192];
  if (basic != nullptr) {
    // Cache the two identity strings for hello_ack (11 S9.7 sources both from
    // the chassis). Done HERE because this is the only place the full
    // BasicStatus is available -- the state snapshot carries the numeric
    // triple but not the strings (12 RTC-6).
    {
      std::lock_guard<std::mutex> lk(chassis_id_mu_);
      chassis_model_ = basic->model;
      chassis_version_ = basic->version;
    }
    const std::size_t n = WriteChassisBasic(*basic, out, sizeof(out));
    if (n > 0) Publish(kBasicSuffix, out, n);
  }
  if (motion != nullptr) {
    const std::size_t n = WriteChassisMotion(*motion, out, sizeof(out));
    if (n > 0) Publish(kMotionSuffix, out, n);
  }
  if (device != nullptr) {
    const std::size_t n = WriteChassisDevice(*device, out, sizeof(out));
    if (n > 0) Publish(kDeviceSuffix, out, n);
  }
  if (fault != nullptr) {
    const std::size_t n = WriteChassisFault(*fault, out, sizeof(out));
    if (n > 0) Publish(kFaultSuffix, out, n);
  }
  // A zero-length write is dropped rather than published: the writers return 0
  // only when the buffer was too small, and a truncated JSON on the wire is
  // worse than a missing message -- the consumer cannot tell it from a
  // malformed sender.
}

bool RtBridge::PublishState(const QuadrupedProcess::StateSnapshot& snap) {
  RobotStateInput in;
  in.conn = snap.conn;
  // basic / motion / faults stay null. They hold std::string and therefore
  // cannot cross the lock-free slot from ctrl (see StateSnapshot's comment);
  // forwarding those four report streams belongs to the rx thread and is not
  // in this batch. WriteRobotState emits the block as absent rather than as
  // zeros -- "not reported yet" and "reported as zero" are different claims,
  // and a zeroed chassis reads as a level robot at rest.
  in.tier1 = snap.tier1;
  in.estop_epoch = snap.estop_epoch;
  in.has_triple = snap.has_readback;
  // Remembered for hello_ack, which is answered on another thread and cannot
  // consume from the slot.
  {
    std::lock_guard<std::mutex> lk(chassis_id_mu_);
    last_has_triple_ = snap.has_readback;
    last_usage_mode_ = snap.usage_mode_raw;
    last_motion_state_ = snap.motion_state_raw;
    last_gait_ = snap.gait_raw;
  }
  in.usage_mode_raw = snap.usage_mode_raw;
  in.motion_state_raw = snap.motion_state_raw;
  in.gait_raw = snap.gait_raw;
  // 11 S9.9's third output. The sample rides in the snapshot rather than being
  // taken from the odom slot here: that slot is consuming (12 RTC-6), and a
  // second consumer would steal every other pose from the uplink.
  in.odom = &snap.odom;
  in.soft_estop_active = snap.soft_estop_active;
  in.cmd_age_ms = snap.cmd_age_ms;
  in.mode_switching = snap.mode_switching;

  char out[kOutCap];
  const std::size_t n = WriteRobotState(in, out, sizeof(out));
  if (n == 0) return false;
  if (!Publish(kStateSuffix, out, n)) return false;
  ++states_;
  return true;
}

void RtBridge::HandleCmdVel(double now_mono_s, const char* data,
                            std::size_t len) {
  CmdVelMsg m;
  const RtParse r = ParseCmdVel(data, len, rid_, boot_, &m);
  if (r != RtParse::kOk) {
    ++cmd_refused_;
    // Remembered, not logged: at 20 Hz a log line per refusal buries the one
    // that matters. The FIRST reason is what an operator needs -- see the
    // header for why a silently refused stream is indistinguishable from an
    // absent one.
    if (first_refusal_ == RtParse::kOk) first_refusal_ = r;
    return;
  }
  ++cmd_ok_;
  // The age is measured from OUR receipt, not from the envelope's mono, and
  // that is on purpose: this is the moment the command became available to
  // Tier 1, and 11 S3.0 forbids trusting a foreign mono anyway when the boot
  // ids differ (rt_parse reports that as mono_usable == false).
  proc_->OnCmdVel(now_mono_s, m.vx, m.vy, m.wz, m.estop_epoch);
}

void RtBridge::SetTransport(const std::string& endpoint,
                            const std::string& codebook,
                            int chassis_dds_domain, int uplink_ros_domain,
                            const std::string& imu_frame_id,
                            bool drdds_available) {
  tp_endpoint_ = endpoint;
  tp_codebook_ = codebook;
  tp_chs_b_domain_ = chassis_dds_domain;
  tp_uplink_domain_ = uplink_ros_domain;
  tp_imu_frame_ = imu_frame_id;
  tp_drdds_ = drdds_available;
}

void RtBridge::SetSpec(bool holonomic, double max_vx, double max_vy,
                       double max_wz) {
  spec_holonomic_ = holonomic;
  spec_max_vx_ = max_vx;
  spec_max_vy_ = max_vy;
  spec_max_wz_ = max_wz;
}

void RtBridge::HandleHello(double now_mono_s, const char* data,
                           std::size_t len) {
  (void)now_mono_s;
  HelloMsg m;
  const RtParse r = ParseHello(data, len, rid_, boot_, &m);
  if (r != RtParse::kOk) {
    ++hello_refused_;
    return;
  }
  // 11 S9.1.4: major.minor, "major 不同即不兼容, quadruped 拒绝进入可运动
  // 状态并上报 E_PROTO_VERSION". Refusing to ANSWER is not the same as
  // refusing to move -- an unanswered hello looks like a dead process, so the
  // mismatch is reported through the counter and the upstream's own timeout
  // rather than by silence. Entering the non-movable state is Tier 1's job and
  // is not reachable from this thread.
  if (m.proto_major != 1) {
    ++hello_refused_;
    return;
  }
  HelloAckInput in;
  in.proto_version = kProtoVersion;
  std::string model, version;
  {
    std::lock_guard<std::mutex> lk(chassis_id_mu_);
    model = chassis_model_;
    version = chassis_version_;
  }
  // Empty means no BasicStatus has arrived yet -> emitted as null, not as "".
  // An empty string reads as a chassis that answered with a blank model.
  if (!model.empty()) in.model = model.c_str();
  if (!version.empty()) in.version = version.c_str();
  // The triple, from the last state this bridge PUBLISHED -- not from the
  // process. The snapshot slot is consume-only by design (LockfreeSlot has no
  // Read(), and its header explains why: a non-consuming read cannot say
  // whether the value is new), so peeking would have meant stealing the sample
  // rt_pub is about to publish and dropping a state frame per handshake.
  {
    std::lock_guard<std::mutex> lk(chassis_id_mu_);
    in.has_triple = last_has_triple_;
    in.usage_mode_raw = last_usage_mode_;
    in.motion_state_raw = last_motion_state_;
    in.gait_raw = last_gait_;
  }
  if (!tp_endpoint_.empty()) in.endpoint = tp_endpoint_.c_str();
  if (!tp_codebook_.empty()) in.codebook = tp_codebook_.c_str();
  if (!tp_imu_frame_.empty()) in.imu_frame_id = tp_imu_frame_.c_str();
  in.chassis_dds_domain = tp_chs_b_domain_;
  in.uplink_ros_domain = tp_uplink_domain_;
  in.drdds_available = tp_drdds_;
  in.holonomic = spec_holonomic_;
  in.max_vx_mps = spec_max_vx_;
  in.max_vy_mps = spec_max_vy_;
  in.max_wz_radps = spec_max_wz_;
  char out[4096];
  const std::size_t n = WriteHelloAck(in, out, sizeof(out));
  if (n == 0) {
    ++hello_refused_;
    return;
  }
  if (Publish(kHelloAckSuffix, out, n)) ++hello_ok_;
}

void RtBridge::HandleChassisMode(double now_mono_s, const char* data,
                                std::size_t len) {
  // now_mono_s is unused: the sequence is dispatched from the control period,
  // not from this callback (see QuadrupedProcess::OnChassisMode on why). Kept
  // in the signature so every subscriber handler has the same shape and the
  // subscription table needs no special case.
  (void)now_mono_s;
  ChassisModeMsg m;
  const RtParse r = ParseChassisMode(data, len, rid_, boot_, &m);
  if (r != RtParse::kOk) {
    // No ack key for this one (11 registers rt/chassis/mode alone), so a
    // refusal is a COUNTER, not a message. main.cc's supervisor prints it --
    // a refused mode switch is otherwise indistinguishable from one that was
    // never sent, and the symptom of both is a robot that will not move.
    ++mode_refused_;
    return;
  }
  if (proc_->OnChassisMode(m.has_usage_mode, m.usage_mode,
                           m.has_motion_state, m.motion_state,
                           m.has_gait, m.gait)) {
    ++mode_ok_;
  } else {
    // 13 MS-3: a switch is already in flight. Counted as refused rather than
    // queued -- see OnChassisMode.
    ++mode_refused_;
  }
}

void RtBridge::HandleChassisCtrl(double now_mono_s, const char* data,
                                 std::size_t len) {
  ChassisCtrlMsg m;
  const RtParse r = ParseChassisCtrl(data, len, rid_, boot_, &m);

  char out[kOutCap];
  CtrlAckInput ack;
  ack.cmd_id = m.cmd_id.empty() ? "anonymous" : m.cmd_id.c_str();
  ack.action = m.raw_action.c_str();
  // 11 CR-12: the locks in an ack are READ-BACK values, never the request's.
  // "Accepted" is not "the lock is gone", and carrying the real ones is what
  // lets a caller see the difference without a second round trip.
  ack.hes_lock = proc_->last_tier1().hes_lock;
  ack.timeout_lock = proc_->last_tier1().timeout_lock;

  if (r != RtParse::kOk) {
    ++ctrl_refused_;
    ack.result = "rejected";
    // 11 S9.3.3: an action the contract DELETED answers E_CAPABILITY, not
    // E_SCHEMA. Telling an operator "malformed" about a word that was valid
    // last release sends them hunting a typo instead of reading release notes.
    ack.code = (r == RtParse::kUnsupportedAction) ? err::kECapability.data()
                                                  : err::kESchema.data();
    const std::size_t n = WriteCtrlAck(ack, out, sizeof(out));
    Publish(kCtrlAckSuffix, out, n);
    return;
  }

  ModeRequestResult verdict;
  switch (m.action) {
    case CtrlAction::kEnable:
      // 11 S9.3.3: local only, nothing goes to the chassis. It clears
      // timeout_lock, and hes_lock only when the physical signal is already
      // gone -- both of those rules live in Tier 1, not here.
      proc_->OnEnable();
      verdict.accepted = true;
      break;
    case CtrlAction::kStand:
      verdict = proc_->OnChassisAction(now_mono_s, ModeAction::kStand, 0);
      break;
    case CtrlAction::kProne:
      // The stair precondition (13 PR-1 / D-40) is inside the machine. Checking
      // it here as well would be a second copy of a safety rule.
      verdict = proc_->OnChassisAction(now_mono_s, ModeAction::kProne, 0);
      break;
    case CtrlAction::kSetSdkMode:
      // 11 S9.3.4 keeps this off the general plane; it is a deployment-time
      // action and there is no mode-machine state for it, so it is acked as
      // rejected-by-capability rather than silently accepted and dropped.
      verdict.accepted = false;
      verdict.reject = ModeReject::kNone;
      break;
  }

  if (verdict.accepted) {
    ++ctrl_ok_;
    ack.result = "accepted";
    ack.code = "OK";
  } else {
    ++ctrl_refused_;
    ack.result = "rejected";
    ack.code = (m.action == CtrlAction::kSetSdkMode)
                   ? err::kECapability.data()
                   : ModeRejectCode(verdict.reject);
  }
  const std::size_t n = WriteCtrlAck(ack, out, sizeof(out));
  Publish(kCtrlAckSuffix, out, n);
}

void RtBridge::HandleEstop(double now_mono_s, const char* data,
                           std::size_t len) {
  // *** THE STOP IS FIRST. See the file comment: anything above it can be
  // reached by an edit to the parsing, and nothing below it can.
  //
  // The dedup window is the one exception, and it is not a refusal: 11 S9.12.6
  // says a repeat inside 50 ms is swallowed -- generation NOT advanced, no
  // event -- and is STILL ACKED. A sender that gets no answer retries, which is
  // the storm the window exists to prevent.
  const bool duplicate =
      (last_estop_mono_s_ >= 0.0) &&
      (now_mono_s - last_estop_mono_s_) < kEstopDedupS;
  if (!duplicate) {
    proc_->OnSoftEstop(now_mono_s);
    last_estop_mono_s_ = now_mono_s;
    ++estop_applied_;
  } else {
    ++estop_deduped_;
  }

  // Only now is the payload looked at, and only to fill the ack. There is no
  // branch below that can undo the stop above.
  EstopMsg m;
  ParseEstop(data, len, rid_, boot_, &m);

  EstopAckInput ack;
  ack.cmd_id = m.cmd_id_present ? m.cmd_id.c_str() : "anonymous";
  ack.result = duplicate ? "duplicate" : "accepted";
  ack.code = "OK";
  ack.estop_epoch = proc_->estop_epoch();
  ack.applied_zero_vel = !duplicate;
  ack.recv_mono_ms = ToMs(now_mono_s);
  ack.latency_ms = 0;
  ack.hes = proc_->last_tier1().hes_lock;
  ack.timeout_lock = proc_->last_tier1().timeout_lock;

  char out[kOutCap];
  const std::size_t n = WriteEstopAck(ack, out, sizeof(out));
  Publish(kEstopAckSuffix, out, n);
}

void RtBridge::HandlePing(double now_mono_s, const char* data,
                          std::size_t len) {
  // 13 Q-4: driven by the ping, never by a timer of our own. A timer would go
  // on answering after the subscription died, which is precisely the failure
  // this probe exists to detect -- and a probe that cannot fail is decoration.
  //
  // The envelope is parsed only for the seq to echo. A malformed ping still
  // gets a pong: the far end treats silence as "the estop chain is dead" and
  // degrades the whole system to hold (13 F-15), so withholding the answer over
  // a bad field would turn a publisher's bug into a stopped robot.
  CmdVelMsg probe;   // reused only for its envelope
  const RtParse r = ParseCmdVel(data, len, rid_, boot_, &probe);
  PongInput pong;
  pong.seq = (r == RtParse::kOk || probe.env.seq != 0) ? probe.env.seq : 0;
  pong.t_mono_ms = ToMs(now_mono_s);
  pong.estop_epoch = proc_->estop_epoch();
  pong.hes = proc_->last_tier1().hes_lock;
  pong.hes_lock = proc_->last_tier1().hes_lock;
  pong.timeout_lock = proc_->last_tier1().timeout_lock;
  pong.stop_reason = proc_->last_tier1().stop_reason;

  char out[kOutCap];
  const std::size_t n = WritePong(pong, out, sizeof(out));
  if (Publish(kPongSuffix, out, n)) ++pongs_;
}

}  // namespace rt
}  // namespace quadruped
