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
                        kStateSuffix}) {
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
