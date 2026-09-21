/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_payloads.h
 * Brief: The data objects quadruped publishes -- RobotState, PowerState, acks
 *
 * Description:
 * Turns the internal structs into the JSON `data` objects 11 defines. The outer
 * envelope is not this file's: it comes from the shared writer
 * (common/include/xbrain/envelope/envelope_writer.h), which is the one place
 * the eight outer fields are spelled (PB-Q3).
 *
 * Everything writes into a caller buffer and returns 0 rather than truncating.
 * A truncated JSON object is a parse error at the far end, and a parse error on
 * state/robot is indistinguishable from the robot having stopped reporting.
 *
 * Four things here are easy to get wrong, and each has its own case in the test:
 *
 *   * hes, hes_lock and timeout_lock are THREE fields, not one with three
 *     names. 11's own D-08 ruling spells out why: hes is the raw level, hes_lock
 *     is a latch software cannot clear, timeout_lock is a latch a human clears.
 *     Collapsing any two loses the distinction that decides whether a robot can
 *     be restarted from a screen.
 *   * faults[].code carries its namespace prefix here too. CF-5 is explicit that
 *     RobotState.faults[] and ChassisFault.faults[] are the same values and must
 *     use the SAME converter -- the §4.1 example in 11 still shows a bare
 *     "0x1007", and copying that example is how the two sides diverge.
 *   * batteries.left / .right are NULL while the index mapping is unknown
 *     (13 BAT-2), with mapping "unknown" beside them. Filling them from array
 *     order would be the seventh instance of the failure mode CLAUDE.md 3.2
 *     catalogues: a guess presented as a measurement. The serial field that was
 *     supposed to disambiguate came back empty on the real machine.
 *   * EstopAck.recv_mono_ms is in MILLISECONDS while the envelope's mono is in
 *     SECONDS. Two units for the same clock in one message is a trap, and it is
 *     the contract's, not ours -- 11 S7.1.1 names the field with its unit and
 *     11 S3.0 names the envelope field with a different one. It is spelled out
 *     here so nobody "fixes" one to match the other.
 *
 * Boundary: no session, no key, no clock. The caller supplies every reading, so
 * a test can state a whole situation and no case depends on wall time.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_RT_PAYLOADS_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_RT_PAYLOADS_H_

#include <cstddef>
#include <cstdint>

#include "quadruped/chs_a_reports.h"
#include "quadruped/chs_a_session.h"
#include "quadruped/odometry.h"
#include "quadruped/tier1.h"

namespace quadruped {
namespace rt {

// Everything RobotState needs, gathered by the caller. Pointers may be null:
// before the first report of each kind there is genuinely nothing to say, and
// the assembler writes JSON null rather than a zeroed struct that would read as
// "idle, stopped, no faults" -- a picture indistinguishable from a healthy
// robot standing still.
// 11 S9.1.4 hello_ack. Answered once per hello, on the rt_pub subscription
// thread. 13 ASM-4 (2) recorded "the process does not answer hello_ack" as an
// open gap, and 10 S3.3 Stage 1 does not complete without it.
//
// The blocks come from three different places on purpose:
//   runtime.*           what the CHASSIS reports (BasicStatus + the triple)
//   runtime.transport   what THIS PROCESS actually bound to -- 13 CB-4 /
//                       DDS-9 / TF-1 all require the EFFECTIVE values here,
//                       because a misconfigured domain or codebook has no
//                       other low-cost way to announce itself
//   spec.*              static limits from configs/models/m20s.yaml (11 S9.6)
//
// `services` from 11 S9.7 is deliberately NOT emitted: the contract marks its
// query method "待确认 (Q20)". An absent field says "we do not know"; a
// fabricated one would be read as fact by whoever consumes the handshake.
struct HelloAckInput {
  const char* proto_version = "1.0";
  // Null until the first BasicStatus arrives. Absent rather than zeroed --
  // a zeroed model/version reads as a real chassis that answered.
  const char* model = nullptr;
  const char* version = nullptr;
  // The mode triple, same source and same has_ flag discipline as RobotState.
  bool has_triple = false;
  std::int64_t usage_mode_raw = 0;
  std::int64_t motion_state_raw = 0;
  std::int64_t gait_raw = 0;
  // 13 S8.2 / CB-4 / DDS-9 / TF-1: the transport block.
  const char* endpoint = nullptr;      // effective endpoint, "tcp://ip:port"
  const char* codebook = nullptr;      // effective codebook name
  int chassis_dds_domain = -1;         // channel two, as CONFIGURED
  int uplink_ros_domain = -1;          // channel three
  const char* imu_frame_id = nullptr;  // TF-1: the frame WE assign
  bool drdds_available = false;
  // 11 S9.6 spec, from the resolved config.
  bool holonomic = false;
  double max_vx_mps = 0.0;
  double max_vy_mps = 0.0;
  double max_wz_radps = 0.0;
};

std::size_t WriteHelloAck(const HelloAckInput& in, char* out, std::size_t cap);

struct RobotStateInput {
  chs_a::ConnState conn = chs_a::ConnState::kProbing;
  const chs_a::BasicStatus* basic = nullptr;
  const chs_a::MotionStatus* motion = nullptr;
  const chs_a::FaultReport* faults = nullptr;

  // Tier 1's verdict for the period being reported, and the generation the
  // process currently holds.
  Tier1Output tier1;
  std::uint64_t estop_epoch = 0;
  // True while the last command's generation disagreed with ours. NOT a lock:
  // it clears by itself when the upstream echoes the new generation, and 11's
  // D-08 ruling keeps it separate from stop_reason for exactly that reason.
  bool soft_estop_active = false;

  // 13 S6.2 mode triple, as raw chassis numbers. Used ONLY when `basic` is
  // null: the full BasicStatus carries the same three plus the strings, and
  // emitting both would put one fact in two places in one message.
  //
  // has_triple distinguishes "read back as 0" from "never read back". Zero is
  // a real value on every one of the three (normal mode / idle / no gait), so
  // a bare 0 would read as a healthy idle robot on a link that has said
  // nothing at all.
  bool has_triple = false;
  std::int64_t usage_mode_raw = 0;
  std::int64_t motion_state_raw = 0;
  std::int64_t gait_raw = 0;

  // Age of the last cmd_vel, in MILLISECONDS -- 11 S4.1 names the field
  // cmd_age_ms. Negative means no command has arrived yet, which is reported as
  // null rather than as a very large age.
  double cmd_age_ms = -1.0;

  bool mode_switching = false;
  // 11 S4.1 `locked`: either latch is on, so an operator does not have to know
  // which one to read.
  // Derived here rather than passed, so it cannot disagree with the two fields
  // it summarises.

  // 11 S4.1 RobotState.odom, and 11 S9.9's output table which names
  // RobotState.odom.* as one of this process's three outputs. Null until the
  // first control period has run: an all-zero pose reads as a robot at the
  // origin, which is a claim rather than an absence.
  // 11 S4.1 charge. As a raw int plus a flag rather than through `basic`:
  // the state path has no BasicStatus to offer (it cannot cross the
  // lock-free slot), and sourcing it from `basic` made the field null on
  // every message the process actually published.
  // The basic-report trio: charge, hes, sleep all ride the same 2 Hz
  // BasicStatus and cross the slot as PODs, so one presence flag covers all
  // three. hes and sleep were added 2026-09-21 -- the charge fix earlier the
  // same day walked past them on the SAME writer line: state-path RobotState
  // said "hes":null, "sleep":null forever while rt/chassis/basic carried
  // both correctly. hes is the HES emergency-stop read-back; a consumer
  // watching state/robot could not see it at all.
  bool has_charge = false;
  int charge_raw = 0;
  bool hes = false;
  bool sleep = false;
  const OdomSample* odom = nullptr;
  // 11 S4.1 after the 2026-09-21 F-5 unfreeze (S14.3 entry): TR-4's computed
  // bit, beside mode_switching rather than replacing it. mode_switching is
  // the MS-3 answer (our own switch in flight); this one is "the machine is
  // mid-transition, ours or the handset's" -- the bit Tier 1 zeroes on and
  // the one HMI's "正在起立..." must come from (TR-4: never fabricate an
  // enum value for it).
  bool motion_state_transitioning = false;
  // 13 S4.4's detail.{tau_ms, source} for the odom block, same unfreeze.
  // The names are the table's own: motion_info_20hz / monitor_10hz. kNone
  // writes null -- "no source yet" is an absence, not a third source.
  enum class OdomSrc { kNone, kMonitor, kDrdds };
  OdomSrc odom_source = OdomSrc::kNone;
};

// 11 S4.1. Returns bytes written, or 0 if the buffer is too small.
std::size_t WriteRobotState(const RobotStateInput& in, char* out,
                            std::size_t cap);

// 11 S4.2. `remain_mile_km` comes from MotionStatus, everything else from
// DeviceStatus and BasicStatus, which is why both are taken.
struct PowerStateInput {
  const chs_a::DeviceStatus* device = nullptr;
  const chs_a::BasicStatus* basic = nullptr;
  double remain_mile_km = 0.0;
  // 13 BAT-3: null until the vendor tells us which array index is which side.
  // The two ints are only read when this is true.
  bool index_map_known = false;
  int left_index = 0;
  int right_index = 1;
};

std::size_t WritePowerState(const PowerStateInput& in, char* out,
                            std::size_t cap);

// 11 S7.1.1 EstopAck. Every field is required, and the deadline that governs
// this message (100 ms, 13 Q-3) is the caller's to meet -- assembling it is
// microseconds, and putting the deadline here would hide where the time goes.
struct EstopAckInput {
  const char* cmd_id = "anonymous";   // 11: echo the request, or this literal
  const char* result = "accepted";    // accepted | duplicate | rejected
  const char* code = "OK";
  std::uint64_t estop_epoch = 0;
  // Which actions actually ran. 11 lists zero_vel / charge_abort / damping;
  // this batch produces only the first, and the array form is kept because a
  // scalar would have to change shape the day a second one appears.
  bool applied_zero_vel = true;
  bool applied_charge_abort = false;
  // MILLISECONDS on the monotonic clock. See the file comment: the envelope's
  // mono is seconds, and these are two different fields of the same message.
  std::uint64_t recv_mono_ms = 0;
  std::uint64_t latency_ms = 0;
  bool hes = false;
  bool timeout_lock = false;
};

std::size_t WriteEstopAck(const EstopAckInput& in, char* out, std::size_t cap);

// 11 S7.7 Ack, used for rt/chassis/ctrl/ack (13 Q-2).
// 11 S3.0's common envelope, the outer object EVERY Zenoh JSON payload this
// process publishes must carry. 13 PB-Q3 requires one writer for all eight
// fields and only one -- eight fields written at eleven call sites is eight
// chances for one of them to drift, and the symptom of a wrong `mono` is an
// age computed against the wrong epoch rather than an error.
//
// *** Until this existed, quadruped published BARE payloads: no v, no rid, no
// mono, no boot, no seq, no ts_sync. Measured on the live chassis 2026-09-21
// by subscribing to rt/chassis/state and printing the top-level keys. Nothing
// downstream noticed because chassis_relay (the CR-4 consumer) is not built
// yet -- which is exactly why it had to be fixed before it is.
struct EnvelopeInput {
  const char* rid = "";
  const char* boot = "";
  // 11 S3.0: the producing PROCESS name, not the key or the thread.
  const char* src = "quadruped";
  // Wall clock, seconds. S3.0 confines it to cross-host alignment, recording
  // and latency statistics -- never an age or a timeout (CLK-C1).
  double ts = 0.0;
  // CLOCK_MONOTONIC seconds. S3.0: "一切超时与年龄判定的唯一依据".
  double mono = 0.0;
  // Per-KEY, monotonically increasing, from 0 at process start (S3.0). Per
  // key rather than per process because it is what a subscriber uses to
  // detect a gap on the key it subscribed to; one shared counter would show
  // a gap on every key whenever any other key published.
  std::uint64_t seq = 0;
  // 13 PB-Q3: "禁止在任何分支填 true 兜底". The only thing that may set this
  // true is a fresh ClockStatus from rt/clock/status (13 Q-5), and until that
  // subscription exists the honest value is false -- which is also what
  // S3.0 says a MISSING field means, so the wire meaning does not change.
  bool ts_sync = false;
};

// Wraps `data` (which must already be a complete JSON object) in the envelope.
// Returns 0 if it does not fit -- callers publish nothing rather than a
// truncated object, same rule as every other writer here.
std::size_t WriteEnvelope(const EnvelopeInput& in, const char* data,
                          std::size_t dlen, char* out, std::size_t cap);

struct CtrlAckInput {
  const char* cmd_id = "anonymous";
  const char* result = "accepted";
  const char* code = "OK";
  // 13 Q-2 makes this REQUIRED: stand / prone / enable / set_sdk_mode.
  const char* action = "";
  // 11 CR-12: after `enable`, these must be the READ-BACK values, not the
  // request's. "Ack = accepted" is not "the lock is gone"; the only proof is
  // the state readback, and carrying it here is what lets a caller see the
  // difference without a second round trip.
  bool hes_lock = false;
  bool timeout_lock = false;
  // 11 S9.3.3 names this for the one refusal that has a name: a prone refused
  // on a stair gait answers E_CAPABILITY with detail.item = "prone_on_stair".
  // The code alone is not enough -- E_CAPABILITY is also what a deleted action
  // and set_sdk_mode answer, so a caller that saw only the code would have to
  // guess which of the three happened. Empty means "no named item", and the
  // key is then OMITTED rather than written as "": an empty string is a value,
  // and 11 S13.9 requires item to be drawn from a closed set when present.
  const char* item = "";
};

std::size_t WriteCtrlAck(const CtrlAckInput& in, char* out, std::size_t cap);

// 13 Q-4 / 11 S8.5. Sent ON RECEIPT of a ping, never on a timer of its own:
// a self-timed pong reports the link alive while the ping path is dead, which
// is the one thing the probe exists to detect.
struct PongInput {
  std::uint64_t seq = 0;          // echoed from the ping
  std::uint64_t t_mono_ms = 0;    // MILLISECONDS, as 11 names the field
  std::uint64_t estop_epoch = 0;
  bool hes = false;
  bool hes_lock = false;
  bool timeout_lock = false;
  StopReason stop_reason = StopReason::kNone;
};

std::size_t WritePong(const PongInput& in, char* out, std::size_t cap);

// ---------------------------------------------------------------------------
// The four chassis report streams (13 S7.1). Each is the parsed report
// forwarded onto its own RT key, and each is written by the chs_a_rx thread --
// which is why these take the parsed struct rather than a snapshot: the reports
// hold std::string (model, serial, fault names) and cannot cross the lock-free
// slot to ctrl (12 RTC-6). Publishing them from the thread that parsed them is
// not a shortcut; it is the only place they exist.
//
// Open-set values go out as BOTH the raw number and the resolved label, never
// as the label alone: 13 S6.5 forbids mapping an unregistered value onto a
// known one, and a consumer that sees only "unknown_0x0000" cannot tell which
// unregistered value it was. 13 V-66's Gait 0 is exactly that case and it
// appears on every boot.
// ---------------------------------------------------------------------------

std::size_t WriteChassisBasic(const chs_a::BasicStatus& in, char* out,
                              std::size_t cap);
std::size_t WriteChassisMotion(const chs_a::MotionStatus& in, char* out,
                               std::size_t cap);
std::size_t WriteChassisDevice(const chs_a::DeviceStatus& in, char* out,
                               std::size_t cap);
// Both lists travel (13 S7.3): `faults` is what is asserted now and `cleared`
// is what just stopped. Sending only the first leaves a consumer unable to tell
// "still broken" from "was broken, now fine" without keeping its own history --
// and a consumer's history is the thing that goes stale across a restart.
std::size_t WriteChassisFault(const chs_a::FaultReport& in, char* out,
                              std::size_t cap);

}  // namespace rt
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_RT_PAYLOADS_H_
