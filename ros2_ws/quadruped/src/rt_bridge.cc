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

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>

#include "quadruped/mono_clock.h"
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
// The same payload plus 11 S3.0's envelope. The envelope is bounded:
// eight fields, the two variable ones being rid and boot, both short.
constexpr std::size_t kEnvCap = kOutCap + 512;

namespace err = hachist::xbrain::errors;

// 11 CLK-A3 / T-11, verbatim: a process that has not received ClockStatus
// for >= 5 s fills ts_sync = false, on the monotonic clock. A CONTRACT
// constant, not a config key -- same standing as kEstopDedupS below: 11
// S14.2 lists this class of value on F-5's unfrozen side, T-11 names the 5
// for every local publisher at once, and a per-process config for a
// system-wide protocol number would be sixty-two chances to disagree.
// (configs/safety/clock.yaml carries the same 5.0 for the Python-side
// readers; neither is derived from the other -- both cite T-11.)
constexpr double kClockSyncTimeoutS = 5.0;

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
constexpr const char* kPowerSuffix = "rt/chassis/power";
// 13 S7.1 gives PowerState 1 Hz. The device report that carries the
// batteries arrives at 2 Hz (S7.2), so the rate is enforced on the clock
// rather than by counting reports -- a divider would silently double the
// rate the day the chassis speeds that stream up.
constexpr double kPowerPeriodS = 1.0;
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
                        kDeviceSuffix, kFaultSuffix, kHelloAckSuffix,
                        kPowerSuffix}) {
    if (FindKey(k) == nullptr) {
      std::fprintf(stderr,
                   "rt_bridge: key %s is not declared in rt_keys.cc -- it is "
                   "not part of the 11 S2.2.1 closed set\n", k);
      std::abort();
    }
  }
}

// Wall clock, seconds, for the envelope's `ts` only.
//
// WALL-CLOCK-OK(align): 11 S3.0 makes this field mandatory on every locally
// produced message and confines it to cross-host alignment, recording and
// latency statistics. Every age, period and timeout in this process is
// steady_clock -- the envelope's `mono` next to it is what those use.
double WallNowSeconds() {
  const auto d = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(d).count();
}

std::uint64_t RtBridge::NextSeq(const std::string& suffix) {
  // Per key (11 S3.0), and under a lock because Publish is reached from two
  // threads: rt_pub for the state stream, chs_a_rx for the four report keys
  // (see PublishReports). A racing ++ would hand two messages the same seq,
  // and a subscriber reads a repeated seq as a replay rather than a race.
  std::lock_guard<std::mutex> lk(seq_mu_);
  return seq_[suffix]++;
}

void RtBridge::HandleClockStatus(double now_mono_s, const char* data,
                                 std::size_t len) {
  ClockStatusMsg m;
  if (ParseClockStatus(data, len, rid_, boot_, &m) != RtParse::kOk) {
    // Refused, not defaulted -- in either direction. A malformed report
    // simply fails to refresh the age, and CLK-A3's aging does the safe
    // thing on its own. Counted because a schema drift between rtk_driver
    // and us would otherwise look exactly like rtk_driver being down.
    ++clock_refused_;
    return;
  }
  ++clock_accepted_;
  // Store order: verdict first, then the arrival time that VALIDATES it.
  // A reader interleaving between the two sees the new verdict beside the
  // OLD time -- which is aged out or about to be, the conservative side.
  // The reverse order would validate a stale verdict with a fresh time.
  clock_sync_.store(m.sync, std::memory_order_relaxed);
  clock_rx_mono_.store(now_mono_s, std::memory_order_release);
}

bool RtBridge::TsSyncAt(double now_mono_s) const {
  const double rx = clock_rx_mono_.load(std::memory_order_acquire);
  // Never received: false. 13 PB-Q3 forbids a true fallback on any branch,
  // and 11 S3.0 gives a missing report the same meaning.
  if (rx < 0.0) return false;
  // CLK-A5: rtk_driver down -> silence -> false within the window, system
  // wide. The comparison is strict-greater so a report exactly at the edge
  // still counts -- the failure direction of the boundary is the safe one
  // either way at 1 Hz, but the contract says "≥ 5 s 未收到", and 4.999 s
  // since the last arrival is not yet that.
  if (now_mono_s - rx > kClockSyncTimeoutS) return false;
  return clock_sync_.load(std::memory_order_relaxed);
}

bool RtBridge::Publish(const std::string& suffix, const char* data,
                       std::size_t len) {
  if (!publish_ || len == 0) return false;
  // 11 S3.0 / 13 PB-Q3: the envelope goes on HERE, at the one exit every
  // publish passes through, rather than at the eleven call sites. Eight
  // fields written eleven times is eight chances for one to drift, and a
  // wrong `mono` does not fail -- it produces an age against the wrong epoch.
  EnvelopeInput env;
  env.rid = rid_.c_str();
  env.boot = boot_.c_str();
  env.ts = WallNowSeconds();
  env.mono = MonoNowSeconds();
  env.seq = NextSeq(suffix);
  // 13 Q-5: ts_sync copies the latest ClockStatus.sync, aged out by CLK-A3.
  // Judged at the envelope's own mono -- the same instant the message claims
  // to have been produced -- rather than at a second clock read.
  env.ts_sync = TsSyncAt(env.mono);
  char wrapped[kEnvCap];
  const std::size_t n = WriteEnvelope(env, data, len, wrapped, sizeof(wrapped));
  // Nothing rather than a truncated object, same rule as the writers.
  //
  // Unreachable by construction today, and deliberately kept: every payload
  // is written into a kOutCap buffer by a writer that returns 0 rather than
  // truncating, so len < 2048; the envelope's own cost is bounded (~190
  // bytes: eight fixed names, rid capped at 32 by IsValidRobotId, boot at 8,
  // src fixed) and kEnvCap leaves 512. A mutant forcing this branch cannot be
  // killed by any test that goes through the writers, which is the CLAUDE.md
  // 7.2.1 definition of an equivalent mutant -- noted here instead of assert-
  // ed. The guard stays because the arithmetic above lives in two constants
  // someone can change independently.
  if (n == 0) {
    ++envelope_overflows_;
    return false;
  }
  ++acks_;
  return publish_(suffix, wrapped, n);
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

  // 11 S4.2 PowerState on rt/chassis/power, 1 Hz (13 S7.1), relayed to
  // state/power by CR-5.
  //
  // *** Before this, the key was declared in rt_keys.cc and WritePowerState was
  // fully implemented WITH tests -- and nothing ever called it. CHG-10's
  // low-battery return reads soc_pct from state/power and the HMI battery
  // display reads the same message, so both had no data source at all. The
  // eighth instance of this shape in this process.
  //
  // Assembled from THREE reports, which is why the three pieces are cached:
  // the batteries come with ChassisDevice, power_management and charge with
  // ChassisBasic, remain_mile with ChassisMotion. Publishing only on the report
  // that happens to be in hand would emit a PowerState missing whichever
  // fields the other two carry.
  if (basic != nullptr) {
    std::lock_guard<std::mutex> lk(chassis_id_mu_);
    last_basic_ = *basic;
    have_basic_ = true;
  }
  if (motion != nullptr) {
    std::lock_guard<std::mutex> lk(chassis_id_mu_);
    last_remain_mile_km_ = motion->remain_mile;
  }
  if (device != nullptr) {
    // The device report is the trigger: it is the one carrying the batteries,
    // and a PowerState without them is the message's whole point missing.
    if (power_next_s_ < 0.0 || now_mono_s >= power_next_s_) {
      power_next_s_ = now_mono_s + kPowerPeriodS;
      // Copied out UNDER the lock, then released before writing and
      // publishing. Holding it across a zenoh put would stall HandleHello on
      // the subscription thread for the length of a network call, and the two
      // have nothing to do with each other.
      chs_a::BasicStatus basic_copy;
      bool have_basic = false;
      double remain_mile = 0.0;
      {
        std::lock_guard<std::mutex> lk(chassis_id_mu_);
        have_basic = have_basic_;
        if (have_basic) basic_copy = last_basic_;
        remain_mile = last_remain_mile_km_;
      }
      PowerStateInput p;
      p.device = device;
      if (have_basic) p.basic = &basic_copy;
      p.remain_mile_km = remain_mile;
      // index_map_known stays false: 13 BAT-2 keeps left/right null until
      // V-55 closes, and 13 BAT-3's config switch is deliberately not wired
      // while the key can only be null (CLAUDE.md S9.3, no reserved hooks).
      const std::size_t n = WritePowerState(p, out, sizeof(out));
      if (n > 0) Publish(kPowerSuffix, out, n);
    }
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
  // 11 S4.1 charge, from the snapshot's raw int. The full BasicStatus
  // cannot cross the slot (12 RTC-6), which is why this field was null
  // on every state message until the int was carried across.
  in.has_charge = snap.has_charge;
  in.charge_raw = snap.charge_raw;
  in.hes = snap.hes;
  in.sleep = snap.sleep;
  in.odom = &snap.odom;
  in.soft_estop_active = snap.soft_estop_active;
  in.cmd_age_ms = snap.cmd_age_ms;
  in.mode_switching = snap.mode_switching;
  in.motion_state_transitioning = snap.motion_state_transitioning;
  // The enum crosses one struct boundary here; the writer maps it to the
  // closed names. A static_cast between the two enums would compile today
  // and silently misalign the day someone reorders one of them.
  switch (snap.odom_source) {
    case QuadrupedProcess::OdomSource::kDrdds:
      in.odom_source = RobotStateInput::OdomSrc::kDrdds;
      break;
    case QuadrupedProcess::OdomSource::kMonitor:
      in.odom_source = RobotStateInput::OdomSrc::kMonitor;
      break;
    case QuadrupedProcess::OdomSource::kNone:
      in.odom_source = RobotStateInput::OdomSrc::kNone;
      break;
  }

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

void RtBridge::HandleLight(double now_mono_s, const char* data,
                           std::size_t len) {
  (void)now_mono_s;
  LightMsg m;
  if (ParseLight(data, len, rid_, boot_, &m) != RtParse::kOk) {
    ++light_refused_;
    return;
  }
  // 13 V-47, verbatim: the front/back illumination switch 11 S9.4 lists comes
  // from the OLD manual and does not exist in the current guide, so until the
  // vendor answers it is refused -- "不静默丢弃, 也不假装设置成功".
  //
  // *** The refusal has NOWHERE TO GO. 11 S2.2.1 declares no
  // rt/chassis/light/ack, so E_CAPABILITY cannot be answered on any key this
  // process may publish (RT-C4 forbids the general plane, where the cmd acks
  // live). This is the same shape 13 MS-3a records for rt/chassis/mode: the
  // refusal is real, and the only witnesses are this counter and the log.
  // Registered in 13 rather than papered over with a key we invented.
  if (m.has_illumination) {
    ++light_refused_;
    std::fprintf(stderr,
                 "rt_bridge: light cmd_id=%s carries `illumination`, which is "
                 "REFUSED (13 V-47: the 1.2.6 lamp switch is absent from the "
                 "current vendor guide). E_CAPABILITY has no ack key on this "
                 "plane, so this line is the only report.\n",
                 m.cmd_id.c_str());
    // The custom half is NOT applied either. A message asking for two things
    // and getting one is worse than a refusal: the sender has no way to learn
    // which half took effect.
    return;
  }
  chs_a::LedSetting head;
  head.pattern = m.head_pattern;
  head.color = m.head_color;
  head.cycle_s = m.head_cycle_s;
  chs_a::LedSetting tail;
  tail.pattern = m.tail_pattern;
  tail.color = m.tail_color;
  tail.cycle_s = m.tail_cycle_s;
  // ACCEPTED is counted before the send, and separately from it. "The message
  // was accepted" and "the frame reached the chassis" are two facts, and 13
  // ASM-6 is exactly what happens when one counter stands for both: the mode
  // path reported accepted while nothing went out.
  ++light_accepted_;
  if (!proc_->SendLightFrame(m.custom_enable, head, tail)) ++light_send_failed_;
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
    // 11 S9.3.3 wants the NAME of the refusal alongside the code. The mapping
    // already existed in ModeRejectItem and was reached only by tests: the ack
    // on the wire never carried it, so a bench run saw E_CAPABILITY and had no
    // way to tell a stair refusal from set_sdk_mode. Measured 2026-09-21 on the
    // chassis -- the refusal was correct, the ack was not.
    ack.item = ModeRejectItem(verdict.reject);
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
