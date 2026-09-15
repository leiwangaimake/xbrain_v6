/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_mode_machine.cc
 * Brief: MS-1..6, TR-1..4, PR-1, GS-1, and the two measured surprises
 *
 * Description:
 * The cases that matter most are the three where the obvious implementation is
 * the wrong one:
 *
 *   * a stand settles at motion_state 17, not 1. The firmware switches itself
 *     to RL control within the same millisecond of accepting the 1 (13 S6.2
 *     v1.3, measured). An implementation that waits for 1 times out on every
 *     SUCCESSFUL stand, and the fault it raises points at the chassis.
 *   * a prone settles at 0, not 4. The 4 appears about 0.3 s in and is gone
 *     four seconds later. Waiting for 4 succeeds by accident if the read-back
 *     happens to land in that window and fails otherwise, which is the worst of
 *     both.
 *   * a read-back still holding the OLD value is expected, not a failure
 *     (MS-6 / TR-3). The vendor's own state machine has "standing up" and
 *     "lying down" nodes that its report enumeration does not contain, so there
 *     is nothing to see for two to three seconds. Judging on "it has not
 *     changed" fails every switch on its first tick.
 *
 * Time is injected, so the five-second timeout and the 3.5 s external hold are
 * exercised in microseconds and nothing here sleeps.
 */

#include "quadruped/mode_machine.h"

#include <cstdio>

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

ModeConfig Cfg() {
  ModeConfig c;
  c.switch_timeout_s = 5.0;
  c.external_transition_hold_s = 3.5;
  // 13 PR-1 / QC-9, as configs/quadruped.yaml ships it: both stair gaits.
  c.prone_forbidden_gaits = {0x3003, 0x1003};
  // 13 GS-1: 0x1003 can be commanded and can never be read back.
  c.command_forbidden_gaits = {0x1003};
  return c;
}

chs_a::BasicStatus Readback(std::int64_t usage, std::int64_t motion,
                            std::int64_t gait) {
  chs_a::BasicStatus b;
  b.usage_mode = chs_a::ResolveUsageMode(usage);
  b.motion_state = chs_a::ResolveMotionState(motion);
  b.gait = chs_a::ResolveGait(gait);
  return b;
}

}  // namespace

int main() {
  // ---- a stand completes at {17, 0x1001}, not at 1 -----------------------
  {
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(0, 0, 0));      // idle, the measured rest state
    CHECK(m.mode_switching() == false);

    const ModeRequestResult r = m.Request(1.0, ModeAction::kStand, 0);
    CHECK(r.accepted);
    // *** The expectation the measurements forced. Waiting for 1 makes MS-2
    // time out on every successful stand.
    CHECK(r.expect.motion_state == 17);
    CHECK(r.expect.gait == 0x1001);
    CHECK(m.mode_switching() == true);
    CHECK(m.motion_state_transitioning() == true);

    // MS-6 / TR-3: the read-back holds the old value for the first tenth of a
    // second, then passes through 1. NEITHER is a failure, and neither
    // completes the switch.
    m.OnReadback(1.05, Readback(0, 0, 0));
    CHECK(m.mode_switching() == true);
    m.OnReadback(1.10, Readback(0, 1, 0));
    CHECK(m.mode_switching() == true);
    CHECK(m.Tick(1.10) == false);            // and no failure is declared

    // The steady triple arrives about three seconds in.
    m.OnReadback(4.0, Readback(0, 17, 0x1001));
    CHECK(m.mode_switching() == false);
    CHECK(m.motion_state_transitioning() == false);
    CHECK(m.switches_completed() == 1);
    CHECK(m.switch_failures() == 0);
    CHECK(m.steady().motion_state == 17);
  }

  // ---- a prone completes at 0; 4 is a waypoint --------------------------
  {
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(0, 17, 0x1001));
    const ModeRequestResult r = m.Request(1.0, ModeAction::kProne, 0);
    CHECK(r.accepted);
    // *** 4 is transitional. Expecting it succeeds only if a read-back happens
    // to land inside a 3.7 s window, which is worse than failing outright.
    CHECK(r.expect.motion_state == 0);
    m.OnReadback(1.3, Readback(0, 4, 0x1001));
    CHECK(m.mode_switching() == true);         // 4 does not complete it
    m.OnReadback(5.0, Readback(0, 0, 0x1001));
    CHECK(m.mode_switching() == false);
    CHECK(m.switches_completed() == 1);
  }

  // ---- MS-2: the timeout keeps the old state and reports once ------------
  {
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(0, 0, 0));
    m.Request(1.0, ModeAction::kStand, 0);
    CHECK(m.Tick(5.9) == false);               // inside the 5 s window
    CHECK(m.mode_switching() == true);
    CHECK(m.Tick(6.1) == true);                // past it: failed
    CHECK(m.switch_failures() == 1);
    CHECK(m.mode_switching() == false);
    // Reported ONCE. A per-tick edge at 100 Hz would bury the fault that
    // mattered under a hundred copies of itself every second.
    CHECK(m.Tick(6.2) == false);
    CHECK(m.switch_failures() == 1);
    // 11 S9.2.2: keep the state we had rather than falling into an unknown one.
    // A stale triple still has a defined kinematic model; an unknown one does
    // not, which is why the contract prefers the stale one.
    CHECK(m.steady().motion_state == 0);
  }

  // ---- MS-5: the WHOLE triple is compared -------------------------------
  {
    // The vendor manual states that switching a gait also switches the motion
    // mode. A comparison that only looked at the commanded field would call a
    // half-applied switch a success.
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(1, 17, 0x1001));
    const ModeRequestResult r = m.Request(1.0, ModeAction::kSetGait, 0x3002);
    CHECK(r.accepted);
    CHECK(r.expect.gait == 0x3002);
    CHECK(r.expect.motion_state == 17);
    CHECK(r.expect.usage_mode == 1);
    // The gait arrives but the motion state is wrong: NOT complete.
    m.OnReadback(1.5, Readback(1, 1, 0x3002));
    CHECK(m.mode_switching() == true);
    // The usage mode drifts too: still not complete.
    m.OnReadback(1.8, Readback(0, 17, 0x3002));
    CHECK(m.mode_switching() == true);
    m.OnReadback(2.0, Readback(1, 17, 0x3002));
    CHECK(m.mode_switching() == false);
  }

  // ---- MS-3: one switch at a time ---------------------------------------
  {
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(0, 0, 0));
    CHECK(m.Request(1.0, ModeAction::kStand, 0).accepted);
    const ModeRequestResult second = m.Request(1.1, ModeAction::kProne, 0);
    CHECK(second.accepted == false);
    CHECK(second.reject == ModeReject::kSwitchInFlight);
  }

  // ---- PR-1: prone is refused on a stair gait ---------------------------
  {
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(1, 17, 0x3003));   // stair_agile
    const ModeRequestResult r = m.Request(1.0, ModeAction::kProne, 0);
    CHECK(r.accepted == false);
    CHECK(r.reject == ModeReject::kProneOnStair);
    CHECK(std::string(ModeRejectItem(r.reject)) == "prone_on_stair");
    // 13 GS-3: "we do not command 0x1003" is not "it cannot appear" -- somebody
    // may have switched it with the factory handset, and the refusal must hold
    // for that one too.
    ModeMachine m2(Cfg());
    m2.OnReadback(0.0, Readback(1, 17, 0x1003));
    CHECK(m2.Request(1.0, ModeAction::kProne, 0).reject ==
          ModeReject::kProneOnStair);
    // ...and on a flat gait it is allowed, so the rule is a refusal on stairs
    // rather than a blanket refusal.
    ModeMachine m3(Cfg());
    m3.OnReadback(0.0, Readback(1, 17, 0x3002));
    CHECK(m3.Request(1.0, ModeAction::kProne, 0).accepted);
  }

  // ---- PR-1 before any read-back: refuse -------------------------------
  {
    // The gait is unknown, and a prone on a staircase has no anti-roll path.
    // Refusing is the conservative answer; accepting would be a guess made in
    // the one direction that cannot be taken back.
    ModeMachine m(Cfg());
    const ModeRequestResult r = m.Request(1.0, ModeAction::kProne, 0);
    CHECK(r.accepted == false);
    CHECK(r.reject == ModeReject::kProneOnStair);
  }

  // ---- GS-1: 0x1003 is never commanded ---------------------------------
  {
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(1, 17, 0x1001));
    const ModeRequestResult r = m.Request(1.0, ModeAction::kSetGait, 0x1003);
    CHECK(r.accepted == false);
    CHECK(r.reject == ModeReject::kGaitReadbackGap);
    CHECK(std::string(ModeRejectItem(r.reject)) == "gait_readback_gap");
    // The commandable ones still are, so this is a refusal of one value rather
    // than of gait switching.
    CHECK(m.Request(1.0, ModeAction::kSetGait, 0x3003).accepted);
  }

  // ---- TR-1: an externally triggered transition -------------------------
  {
    // Nothing of ours is outstanding and the triple moves. Somebody used the
    // handset. We cannot see when that ends (13 V-61), so a hold is assumed and
    // the robot is held at zero for its duration.
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(0, 17, 0x1001));
    CHECK(m.motion_state_transitioning() == false);
    m.OnReadback(10.0, Readback(0, 4, 0x1001));     // somebody pressed prone
    CHECK(m.mode_switching() == false);             // not OUR switch
    CHECK(m.motion_state_transitioning() == true);  // ...but still moving
    // *** The hold EXPIRES. Without this it would hold the robot at zero for
    // the rest of the session after one handset press, and the configured
    // external_transition_hold_s would be a key that changes nothing.
    CHECK(m.Tick(13.0) == false);
    CHECK(m.motion_state_transitioning() == true);  // inside 3.5 s
    m.Tick(13.6);
    CHECK(m.motion_state_transitioning() == false); // past it
    // ...and what it settled on is the steady value from here on.
    CHECK(m.steady().motion_state == 4);
  }

  // ---- the FIRST read-back is not an external transition ---------------
  {
    // There is no previous value for it to differ from, so treating it as a
    // change would hold every robot at zero for 3.5 s after every start.
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(0, 17, 0x1001));
    CHECK(m.motion_state_transitioning() == false);
    CHECK(m.steady().motion_state == 17);
  }

  // ---- TR-2: the prone pre-check uses the STEADY value -----------------
  {
    // During an external transition the instantaneous read-back holds the old
    // triple. Judging the pre-check on the steady value is both what TR-2 says
    // and the more conservative choice.
    // *** The scenario that separates the two, and it is the vendor's own
    // documented behaviour: 13 S6.4 says a prone from a NAVIGATION gait first
    // switches back to the basic gait to avoid a roll-over. So a machine on the
    // stairs, mid-transition, reads back gait 0x1001 -- a gait the pre-check
    // permits. Judging the instantaneous value would admit a prone at exactly
    // the moment the robot is part-way down a staircase.
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(1, 17, 0x3003));     // steady: on the stairs
    m.OnReadback(1.0, Readback(1, 4, 0x1001));      // anti-roll path, transient
    CHECK(m.motion_state_transitioning() == true);
    CHECK(m.last().gait == 0x1001);                 // the instant says basic
    CHECK(m.steady().gait == 0x3003);               // ...the steady says stairs
    CHECK(m.Request(1.5, ModeAction::kProne, 0).reject ==
          ModeReject::kProneOnStair);
    // And once the transition has settled, the steady value follows -- so this
    // is a refusal during the window, not a permanent one.
    m.Tick(5.0);
    CHECK(m.motion_state_transitioning() == false);
    CHECK(m.steady().gait == 0x1001);
    CHECK(m.Request(5.1, ModeAction::kProne, 0).accepted);
  }

  // ---- our own switch supersedes an external hold ----------------------
  {
    ModeMachine m(Cfg());
    m.OnReadback(0.0, Readback(0, 17, 0x1001));
    m.OnReadback(10.0, Readback(0, 4, 0x1001));     // external
    CHECK(m.motion_state_transitioning() == true);
    m.Request(10.5, ModeAction::kStand, 0);          // now we command one
    CHECK(m.mode_switching() == true);
    m.OnReadback(13.0, Readback(0, 17, 0x1001));
    CHECK(m.mode_switching() == false);
    // The external hold must not outlive the switch that replaced it: it was
    // an assumption about a transition we could not see, and we can see this
    // one finish.
    CHECK(m.motion_state_transitioning() == false);
  }

  if (g_failures == 0) {
    std::printf("ALL MODE_MACHINE TESTS PASSED\n");
    return 0;
  }
  std::printf("%d MODE_MACHINE TEST(S) FAILED\n", g_failures);
  return 1;
}
