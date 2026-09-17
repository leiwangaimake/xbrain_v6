/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: mode_machine.h
 * Brief: The mode triple state machine -- MS-1..6, TR-1..4, PR-1, GS-1
 *
 * Description:
 * Owns the question "is a mode switch in flight, and did it work". Everything
 * here comes from 13 S6.2-S6.6 plus the 2026-09-14/15 measurements, and three
 * of the rules are the opposite of the obvious implementation:
 *
 *   * a switch is NOT timed. MS-1 sets the flag when the command is SENT and
 *     clears it when the READ-BACK equals the expectation. A timer-based
 *     implementation declares success while the robot is still moving, which is
 *     precisely the 2-3 s window in which the machine is standing up.
 *   * the read-back holding its OLD value during the transition is expected,
 *     not a failure (MS-6 / TR-3). The vendor's state machine has "standing up"
 *     and "lying down" nodes that the report enumeration does not contain
 *     (13 V-61), so there is no value to see. Judging failure on "it has not
 *     changed yet" fails every switch at the first tick.
 *   * the whole TRIPLE is compared, not the field that was commanded (MS-5).
 *     The vendor manual states that switching a gait also switches the motion
 *     mode and the other way round, so a command that touches one field moves
 *     two, and comparing only the commanded one calls a half-applied switch a
 *     success.
 *
 * And one that the measurements corrected: `stand` is commanded as 1, but the
 * firmware switches itself to RL control within the same millisecond and the
 * steady read-back is {17, 0x1001}. Expecting 1 makes MS-2 time out on every
 * successful stand. `prone` is the mirror image: 4 is transitional and 0 is the
 * steady value, reached about four seconds later.
 *
 * Boundary: it decides and remembers. It sends nothing -- the caller owns the
 * single tx path (13 CA-4) -- and it reads no clock, so a five-second timeout
 * is exercised in microseconds.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_MODE_MACHINE_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_MODE_MACHINE_H_

#include <cstdint>
#include <string>
#include <vector>

#include "quadruped/chs_a_reports.h"

namespace quadruped {

// What the layer above asked for. One enum rather than a string, because the
// string form is the contract's (11 S9.3.3) and translating it once at the
// boundary keeps every comparison in here a comparison of values.
enum class ModeAction {
  kStand,
  kProne,
  kRlControl,    // idempotent; the firmware usually gets there by itself
  kSetGait,
  kSetUsageMode,
};

// Why a request was refused. Mapped to an 11 S13 error code by the caller --
// this file names the CAUSE, and the caller owns the code, because 11 S8.13.5
// puts the mapping to E_* on one side of the system and this is not it.
enum class ModeReject {
  kNone,
  kProneOnStair,      // PR-1: E_CAPABILITY, detail.item "prone_on_stair"
  kGaitReadbackGap,   // GS-1: E_NOT_IMPLEMENTED, "gait_readback_gap"
  kSwitchInFlight,    // MS-3: a switch is already running
  kUnknownGait,       // a gait this build does not model
};

const char* ModeRejectItem(ModeReject r);

// The closed-set error code that goes with a rejection. Here rather than at the
// ack site because the MEANING of each rejection is this machine's -- PR-1 is a
// capability limit, GS-1 is an unimplemented read-back, MS-3 is "busy" -- and a
// caller picking codes would be a second copy of that judgement.
//
// Returns the exported constant from common/errors (CLAUDE.md 3.5), never a
// literal: a hardcoded "E_CAPABILITY" here and an "E_Capability" there is the
// kind of divergence that only surfaces during integration.
const char* ModeRejectCode(ModeReject r);

// The three read-back values, raw. Compared as a triple (MS-5).
struct ModeTriple {
  std::int64_t usage_mode = 0;
  std::int64_t motion_state = 0;
  std::int64_t gait = 0;

  bool operator==(const ModeTriple& o) const {
    return usage_mode == o.usage_mode && motion_state == o.motion_state &&
           gait == o.gait;
  }
  bool operator!=(const ModeTriple& o) const { return !(*this == o); }
};

struct ModeConfig {
  // MS-2. 5.0 s by default: it has to cover the slowest measured switch (the
  // four seconds a prone takes to settle to 0) with margin.
  double switch_timeout_s = 0.0;
  // TR-1. How long an externally triggered transition is assumed to last when
  // we did not command it and therefore cannot see it end.
  double external_transition_hold_s = 0.0;
  // PR-1 / QC-9. Gait raw values on which `prone` is refused. A LIST and not a
  // predicate: QC-9 allows it to be widened and forbids narrowing, which is a
  // statement about data, not about code.
  std::vector<std::int64_t> prone_forbidden_gaits;
  // GS-1. Gait raw values this build refuses to COMMAND, while still
  // recognising them on read-back (GS-3: "we do not send it" is not "it cannot
  // appear" -- somebody may switch it with the factory handset).
  std::vector<std::int64_t> command_forbidden_gaits;
};

struct ModeRequestResult {
  bool accepted = false;
  ModeReject reject = ModeReject::kNone;
  // The triple the read-back must reach for this switch to count as done.
  ModeTriple expect;
};

class ModeMachine {
 public:
  explicit ModeMachine(ModeConfig cfg);

  // A command from above, at the moment it is about to be SENT. MS-1 starts the
  // window here, not when the frame leaves: the chassis begins moving on
  // receipt, and a window that started later would leave a gap in which the
  // robot is moving and mode_switching is false.
  ModeRequestResult Request(double now_mono_s, ModeAction action,
                            std::int64_t param);

  // A BasicStatus arrived. Drives MS-1's clear condition, MS-2's timeout and
  // TR-1's external-transition detection.
  void OnReadback(double now_mono_s, const chs_a::BasicStatus& b);

  // Called every control period so the timeout can fire even when the chassis
  // has gone quiet. Returns true on the tick a switch is declared FAILED
  // (MS-2), so the caller raises the fault exactly once.
  bool Tick(double now_mono_s);

  // MS-3: true while a switch is in flight. Tier 1 reads this and outputs zero.
  bool mode_switching() const { return switching_; }

  // TR-4: our INFERENCE that the machine is mid-transition, published beside
  // the steady value and marked as computed rather than reported. It is true
  // both for our own switches and for one somebody else triggered.
  bool motion_state_transitioning() const;

  // TR-2: the last read-back that was NOT part of a transition. The prone
  // pre-check uses this rather than the instantaneous value, because during a
  // transition the read-back holds the OLD value -- and the old value is the
  // more conservative thing to judge against.
  const ModeTriple& steady() const { return steady_; }
  const ModeTriple& last() const { return last_; }
  bool has_readback() const { return has_readback_; }

  std::uint64_t switch_failures() const { return switch_failures_; }
  std::uint64_t switches_completed() const { return switches_completed_; }

 private:
  bool ProneAllowed() const;
  bool GaitCommandable(std::int64_t gait) const;

  ModeConfig cfg_;
  bool switching_ = false;
  ModeTriple expect_;
  double switch_started_s_ = -1.0;

  // Set when a read-back changes with no command of ours outstanding (TR-1).
  double external_change_s_ = -1.0;

  ModeTriple last_;
  ModeTriple steady_;
  bool has_readback_ = false;

  std::uint64_t switch_failures_ = 0;
  std::uint64_t switches_completed_ = 0;
};

// The steady read-back values the measurements pinned down (13 S6.2 v1.3).
// Named because the numbers alone invite the two mistakes the table records:
// expecting 1 after a stand, and expecting 4 after a prone.
inline constexpr std::int64_t kMotionStateStandSteady = 17;      // NOT 1
inline constexpr std::int64_t kGaitStandSteady = 0x1001;
inline constexpr std::int64_t kMotionStateProneSteady = 0;       // NOT 4
inline constexpr std::int64_t kMotionStateRlControl = 17;

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_MODE_MACHINE_H_
