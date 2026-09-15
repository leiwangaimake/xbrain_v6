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
#include "quadruped/tier1.h"

namespace quadruped {
namespace rt {

// Everything RobotState needs, gathered by the caller. Pointers may be null:
// before the first report of each kind there is genuinely nothing to say, and
// the assembler writes JSON null rather than a zeroed struct that would read as
// "idle, stopped, no faults" -- a picture indistinguishable from a healthy
// robot standing still.
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

  // Age of the last cmd_vel, in MILLISECONDS -- 11 S4.1 names the field
  // cmd_age_ms. Negative means no command has arrived yet, which is reported as
  // null rather than as a very large age.
  double cmd_age_ms = -1.0;

  bool mode_switching = false;
  // 11 S4.1 `locked`: either latch is on, so an operator does not have to know
  // which one to read.
  // Derived here rather than passed, so it cannot disagree with the two fields
  // it summarises.
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

}  // namespace rt
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_RT_PAYLOADS_H_
