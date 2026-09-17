/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: mode_machine.cc
 * Brief: Mode triple state machine implementation (see mode_machine.h)
 *
 * Description:
 * The state is five values: whether a switch is in flight, what triple it is
 * waiting for, when it started, the last read-back, and the last read-back that
 * was not part of a transition. The last two are separate on purpose -- TR-2
 * needs the STEADY one for the prone pre-check, and during a transition the
 * instantaneous read-back holds the old value anyway, so keeping only one would
 * make the pre-check judge against a value that is about to be wrong.
 *
 * The expectations are built in one place, BuildExpect. That matters because of
 * MS-5: a command that touches one field of the triple moves two, and the
 * expectation has to describe the whole resulting triple. Building it at the
 * call site would give each action its own idea of what the other two fields
 * become, and the one that is wrong is whichever is written last.
 */

#include "quadruped/mode_machine.h"

#include "xbrain/errors/errors.h"

#include <algorithm>

namespace quadruped {

const char* ModeRejectItem(ModeReject r) {
  switch (r) {
    case ModeReject::kNone: return "";
    // 13 PR-1. The vendor answered that a prone from a NAVIGATION gait first
    // switches back to the basic gait to avoid a roll-over, and said nothing
    // about the stair gaits. No anti-roll path on a staircase is an accident,
    // so the conservative admission refuses it until that is answered.
    case ModeReject::kProneOnStair: return "prone_on_stair";
    // 13 GS-1. 0x1003 can be commanded and can never be read back, so the
    // read-back comparison MS-1 requires cannot ever succeed: a branch that is
    // guaranteed to report failure is a defect, not a feature.
    case ModeReject::kGaitReadbackGap: return "gait_readback_gap";
    case ModeReject::kSwitchInFlight: return "mode_switch_in_flight";
    case ModeReject::kUnknownGait: return "unknown_gait";
  }
  return "invalid";
}

const char* ModeRejectCode(ModeReject r) {
  // The codes come from the generated closed-set export, never from literals
  // (CLAUDE.md 3.5). The mapping itself is the contract's:
  //   PR-1  prone on a stair gait      -> a capability limit (11 S9.3.3)
  //   GS-1  gait with no read-back     -> not implemented, deliberately
  //   MS-3  a switch already in flight -> busy, retry after the window
  switch (r) {
    case ModeReject::kNone:
      return "";
    case ModeReject::kProneOnStair:
      return hachist::xbrain::errors::kECapability.data();
    case ModeReject::kGaitReadbackGap:
      return hachist::xbrain::errors::kENotImplemented.data();
    case ModeReject::kSwitchInFlight:
      return hachist::xbrain::errors::kEBusy.data();
    case ModeReject::kUnknownGait:
      return hachist::xbrain::errors::kESchema.data();
  }
  return hachist::xbrain::errors::kESchema.data();
}

namespace {

// The triple the read-back must reach. 13 S6.2 v1.3 measured both surprises:
// a stand settles at {17, 0x1001} because the firmware switches itself to RL
// control within the same millisecond, and a prone settles at 0 because 4 is
// only a waypoint on the way there.
ModeTriple BuildExpect(const ModeTriple& current, ModeAction action,
                       std::int64_t param) {
  ModeTriple e = current;
  switch (action) {
    case ModeAction::kStand:
      e.motion_state = kMotionStateStandSteady;
      e.gait = kGaitStandSteady;
      break;
    case ModeAction::kProne:
      e.motion_state = kMotionStateProneSteady;
      // The gait after a prone is not specified anywhere, and the measurement
      // showed 0 -- a value 13 S5.3's table does not contain (V-66). Leaving it
      // at the current value would make MS-2 time out on every prone, so the
      // gait is deliberately NOT part of the expectation for this action.
      e.gait = current.gait;
      break;
    case ModeAction::kRlControl:
      e.motion_state = kMotionStateRlControl;
      break;
    case ModeAction::kSetGait:
      e.gait = param;
      // MS-5: a gait switch also moves the motion mode. The high nibble selects
      // it (0x1xxx standard, 0x3xxx navigation), and both land in RL control.
      e.motion_state = kMotionStateRlControl;
      break;
    case ModeAction::kSetUsageMode:
      e.usage_mode = param;
      break;
  }
  return e;
}

bool Contains(const std::vector<std::int64_t>& v, std::int64_t x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}

}  // namespace

ModeMachine::ModeMachine(ModeConfig cfg) : cfg_(std::move(cfg)) {}

bool ModeMachine::ProneAllowed() const {
  // TR-2: judged on the STEADY read-back, not the instantaneous one. During a
  // transition the instantaneous value holds the OLD triple, and judging on the
  // old one is the more conservative choice -- which is the direction this
  // pre-check is supposed to err in.
  if (!has_readback_) {
    // Nothing has been read back yet, so the gait is unknown. Refusing is the
    // conservative answer: a prone on a staircase has no anti-roll path.
    return false;
  }
  return !Contains(cfg_.prone_forbidden_gaits, steady_.gait);
}

bool ModeMachine::GaitCommandable(std::int64_t gait) const {
  return !Contains(cfg_.command_forbidden_gaits, gait);
}

ModeRequestResult ModeMachine::Request(double now_mono_s, ModeAction action,
                                       std::int64_t param) {
  ModeRequestResult r;
  if (switching_) {
    // MS-3: a second switch while one is in flight would leave two
    // expectations and no way to say which read-back belongs to which.
    r.reject = ModeReject::kSwitchInFlight;
    return r;
  }
  if (action == ModeAction::kProne && !ProneAllowed()) {
    r.reject = ModeReject::kProneOnStair;
    return r;
  }
  if (action == ModeAction::kSetGait && !GaitCommandable(param)) {
    r.reject = ModeReject::kGaitReadbackGap;
    return r;
  }

  r.expect = BuildExpect(last_, action, param);
  r.accepted = true;
  switching_ = true;
  expect_ = r.expect;
  // MS-1: the window opens now, at SEND time. The chassis starts moving on
  // receipt, so a window that opened on the first read-back would leave a gap
  // in which the machine is moving and mode_switching reads false.
  switch_started_s_ = now_mono_s;
  // Our own switch supersedes any external-transition hold: we now know what
  // is happening and for how long.
  external_change_s_ = -1.0;
  return r;
}

void ModeMachine::OnReadback(double now_mono_s, const chs_a::BasicStatus& b) {
  ModeTriple t;
  t.usage_mode = b.usage_mode.raw;
  t.motion_state = b.motion_state.raw;
  t.gait = b.gait.raw;

  const bool first = !has_readback_;
  const bool changed = !first && t != last_;

  if (switching_) {
    last_ = t;
    has_readback_ = true;
    if (t == expect_) {
      // MS-1's clear condition, and the only one. MS-5: the WHOLE triple.
      switching_ = false;
      switch_started_s_ = -1.0;
      steady_ = t;
      ++switches_completed_;
    }
    // MS-6 / TR-3: a read-back still holding the old value is expected during
    // the transition and is NOT a failure. Only Tick's timeout decides that.
    return;
  }

  // TR-1. Nothing of ours is outstanding and the triple moved, so somebody else
  // moved it -- the factory handset, or a second client. We cannot see when
  // that transition ends (13 V-61: the "standing up" and "lying down" states
  // have no enumeration values), so a hold is assumed and zero is output for
  // its duration. Conservative by construction: the alternative is believing a
  // moving robot is stationary.
  if (changed) {
    external_change_s_ = now_mono_s;
  }
  last_ = t;
  has_readback_ = true;
  if (!motion_state_transitioning()) {
    steady_ = t;
  }
  // The first read-back of the process's life is not an external transition:
  // there was no previous value for it to differ from.
  if (first) {
    steady_ = t;
  }
}

bool ModeMachine::Tick(double now_mono_s) {
  // TR-1's hold expires here, on the control tick, because there is nothing to
  // observe that would end it: 13 V-61 records that the "standing up" and
  // "lying down" states have no enumeration values, so the read-back cannot
  // say "finished". A hold that never expired would leave the robot at zero
  // velocity for the rest of the session after one handset press -- and the
  // configured external_transition_hold_s would be a key that changed nothing,
  // which is the defect this package keeps finding.
  if (external_change_s_ >= 0.0 &&
      now_mono_s - external_change_s_ > cfg_.external_transition_hold_s) {
    external_change_s_ = -1.0;
    // Whatever it settled on is now the steady value, and the prone pre-check
    // judges against it from here on (TR-2).
    steady_ = last_;
  }
  if (!switching_) return false;
  if (now_mono_s - switch_started_s_ <= cfg_.switch_timeout_s) return false;
  // MS-2. The switch did not complete. Keep the state we had, raise a fault,
  // and do NOT fall into an unknown state -- 11 S9.2.2 is explicit that an
  // unknown mode is worse than a stale one, because a stale one still has a
  // defined kinematic model.
  switching_ = false;
  switch_started_s_ = -1.0;
  ++switch_failures_;
  return true;
}

bool ModeMachine::motion_state_transitioning() const {
  if (switching_) return true;
  if (external_change_s_ < 0.0) return false;
  // The hold is a duration rather than a read-back condition because there is
  // no read-back that says "still moving" -- see TR-1.
  return true;
}

}  // namespace quadruped
