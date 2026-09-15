/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_tier1.cc
 * Brief: Tier 1 -- the precedence ladder, both locks, clamp, trim, non-finite
 *
 * Description:
 * The case that matters most in this file is the PRECEDENCE LADDER. 11 S4.1
 * says the stop_reason closed set is written in decision order, so the order is
 * part of the contract and not an implementation detail. The ladder case trips
 * every condition at once and then releases them one at a time, asserting the
 * reason reported at each step. A suite that tested each condition in isolation
 * would pass on an implementation whose branches are in any order at all, and
 * the order decides what a robot reports when two things are wrong together --
 * which is the situation a field engineer is actually looking at.
 *
 * The two locks are tested from the direction that matters: what does NOT
 * release them.
 *   * the hardware stop needs the signal gone AND a human. Either half alone
 *     must not release it, because releasing on the signal alone restarts the
 *     robot the moment somebody pulls the button back out.
 *   * the timeout lock needs a fresh command AND a human. Releasing on the
 *     fresh command alone would make the lock self-clearing, and then it is not
 *     a lock -- it is a one-period gap nobody will ever see.
 *
 * Nothing here sleeps. Every deadline is driven by the injected clock, so the
 * 200 ms timeout is exercised in microseconds. This is the one function whose
 * tests must never be quarantined for flakiness, so they are built not to be
 * flaky in the first place.
 *
 * One thing deliberately NOT asserted: that stop_reason "no_source" is ever
 * produced. It is in the closed set with no producer anywhere, which 11's own
 * self-audit registers as a contradiction. The contract's answer for "no
 * command has ever arrived" is the timeout branch, and there is a case for it.
 */

#include "quadruped/tier1.h"

#include <cmath>
#include <cstdio>
#include <string>

#include "xbrain/enums/closed_sets.h"

using quadruped::StopReason;
using quadruped::StopReasonName;
using quadruped::Tier1;
using quadruped::Tier1Config;
using quadruped::Tier1Input;
using quadruped::Tier1Output;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

// Deliberately asymmetric limits. max_wz is SMALLER than max_vx so that a yaw
// rate clamped against the linear limit produces a different number -- with
// equal limits that mistake is invisible.
Tier1Config Cfg() {
  Tier1Config c;
  c.limits.max_vx_mps = 2.0;
  c.limits.max_vy_mps = 1.0;
  c.limits.max_wz_radps = 0.8;
  c.limits.max_accel_mps2 = 2.5;
  c.limits.max_decel_mps2 = 2.5;
  c.limits.holonomic = true;
  c.cmd_timeout_ms = 200;
  c.estop_ack_ms = 100;
  c.estop_dedup_ms = 50;
  c.control_loop_hz = 100.0;
  c.safety_probe_stale_s = 3.0;
  return c;
}

// A period in which nothing is wrong: fresh command, epochs agreed, navigation
// mode, awake, not switching.
Tier1Input Healthy(double t = 1.0) {
  Tier1Input in;
  in.now_mono_s = t;
  in.last_cmd_rx_mono_s = t - 0.01;   // 10 ms old, well inside 200
  in.has_cmd = true;
  in.usage_mode_raw = quadruped::kUsageModeNavigation;
  return in;
}

bool IsStopped(const Tier1Output& o) {
  return o.vx == 0.0 && o.vy == 0.0 && o.wz == 0.0 && o.vz == 0.0 &&
         o.v_roll == 0.0 && o.v_pitch == 0.0;
}

}  // namespace

int main() {
  // ---- the enum is the closed set, by name and by index -------------------
  {
    // CLAUDE.md 3.5: the strings live in the generated library, never as
    // literals here. This case is what proves the enum indexes it correctly --
    // an off-by-one would report every stop as the reason next to the real one.
    CHECK(StopReasonName(StopReason::kNone) == "none");
    CHECK(StopReasonName(StopReason::kHes) == "hes");
    CHECK(StopReasonName(StopReason::kTimeout) == "timeout");
    CHECK(StopReasonName(StopReason::kSoftEstop) == "soft_estop");
    CHECK(StopReasonName(StopReason::kModeSwitching) == "mode_switching");
    CHECK(StopReasonName(StopReason::kSleep) == "sleep");
    CHECK(StopReasonName(StopReason::kModeMismatch) == "mode_mismatch");
    CHECK(StopReasonName(StopReason::kNan) == "nan");
    CHECK(StopReasonName(StopReason::kNoSource) == "no_source");
  }

  // ---- THE PRECEDENCE LADDER ---------------------------------------------
  {
    // Everything wrong at once, then released one condition at a time. Each
    // step asserts the reason AND that the output is still zero.
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.hes_raw = true;
    in.has_cmd = false;                 // also a timeout
    in.cmd_estop_epoch = 7;             // also a soft stop
    in.local_estop_epoch = 8;
    in.mode_switching = true;
    in.sleep_readback = true;
    in.usage_mode_raw = 0;              // also a mode mismatch
    in.vx = std::nan("");               // also a poisoned payload

    Tier1Output o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kHes);
    CHECK(IsStopped(o));

    // Release the hardware stop, and clear the lock it set. Both halves are
    // needed; that the lock needs both is its own case further down.
    in.hes_raw = false;
    in.enable_requested = true;
    t.Step(in);                          // this period clears hes_lock
    in.enable_requested = false;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kTimeout);
    CHECK(IsStopped(o));

    // Give it a fresh command, then clear the lock that stale one set.
    in.has_cmd = true;
    in.last_cmd_rx_mono_s = in.now_mono_s - 0.01;
    in.enable_requested = true;
    t.Step(in);
    in.enable_requested = false;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kSoftEstop);
    CHECK(IsStopped(o));

    in.cmd_estop_epoch = 8;              // upstream echoes our generation
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kModeSwitching);
    CHECK(IsStopped(o));

    in.mode_switching = false;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kSleep);
    CHECK(IsStopped(o));

    in.sleep_readback = false;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kModeMismatch);
    CHECK(IsStopped(o));
    CHECK(o.event_mode_mismatch == true);
    CHECK(o.mode_mismatch_actual == 0);

    in.usage_mode_raw = quadruped::kUsageModeNavigation;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kNan);
    CHECK(IsStopped(o));
    CHECK(o.event_nan == true);

    in.vx = 1.0;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kNone);
    CHECK(o.vx == 1.0);
  }

  // ---- the hardware stop needs BOTH halves to release --------------------
  {
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.hes_raw = true;
    CHECK(t.Step(in).stop_reason == StopReason::kHes);
    CHECK(t.hes_lock() == true);

    // The signal alone is not enough. This is the case that matters: somebody
    // pulls the button back out and the robot must NOT resume by itself.
    in.hes_raw = false;
    Tier1Output o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kHes);
    CHECK(o.hes_lock == true);
    CHECK(o.event_estop_unlock == false);

    // The request alone is not enough either: the button is still pressed.
    in.hes_raw = true;
    in.enable_requested = true;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kHes);
    CHECK(o.hes_lock == true);
    CHECK(o.event_estop_unlock == false);

    // Both together, and only then.
    in.hes_raw = false;
    o = t.Step(in);
    CHECK(o.hes_lock == false);
    CHECK(o.event_estop_unlock == true);
    // ...and the period AFTER the release runs normally.
    in.enable_requested = false;
    in.vx = 0.5;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kNone);
    CHECK(o.vx == 0.5);
  }

  // ---- the timeout locks, and a fresh command alone does not release it ---
  {
    Tier1 t(Cfg());
    Tier1Input in = Healthy(10.0);
    in.vx = 1.0;
    CHECK(t.Step(in).stop_reason == StopReason::kNone);

    // *** The boundary, built so it IS the boundary. The contract says
    // "> CMD_TIMEOUT_MS", so an age of exactly the timeout must still pass.
    //
    // Writing 10.0 - 0.200 does NOT produce an age of 0.2: neither 0.2 nor the
    // difference is exactly representable, and the age comes out 0.19999...,
    // which sits on the same side of the comparison for BOTH > and >=. That
    // version of this case looked like a boundary test and tested nothing --
    // the mutant that changed > into >= survived it.
    //
    // Taking the timeout back from the object and using it as the age makes the
    // two values bit-identical whatever the configured milliseconds are.
    const double tmo = t.cmd_timeout_s();
    in.now_mono_s = tmo;
    in.last_cmd_rx_mono_s = 0.0;            // age == tmo, exactly
    CHECK(t.Step(in).stop_reason == StopReason::kNone);
    // One ulp past it, which must trip. nextafter rather than a hand-written
    // epsilon: an epsilon large enough to be safe is large enough to hide an
    // off-by-one of any smaller size.
    in.now_mono_s = std::nextafter(tmo, 1.0e9);
    CHECK(t.Step(in).stop_reason == StopReason::kTimeout);
    // Clearing that lock takes BOTH a fresh command and an enable, in that
    // order: while the command is still stale the first timeout branch fires
    // and never looks at enable_requested. Getting this wrong here is what
    // made the assertions further down fail on an already-latched lock.
    in.now_mono_s = 10.0;
    in.last_cmd_rx_mono_s = 10.0 - 0.01;
    in.enable_requested = true;
    t.Step(in);
    in.enable_requested = false;
    CHECK(t.timeout_lock() == false);
    in.last_cmd_rx_mono_s = 10.0 - 0.201;
    Tier1Output o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kTimeout);
    CHECK(o.timeout_lock == true);
    CHECK(o.event_timeout_lock == true);

    // The fault event fires ONCE. At 100 Hz a repeated edge is 100 events a
    // second, which buries the one that mattered.
    o = t.Step(in);
    CHECK(o.event_timeout_lock == false);
    CHECK(o.timeout_lock == true);

    // *** The upstream comes back. That is NOT permission to move: 11 S9.12.1
    // requires an explicit enable, and a lock that clears itself when the
    // upstream returns is a one-period gap nobody will ever see.
    in.last_cmd_rx_mono_s = 10.0 - 0.01;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kTimeout);
    CHECK(o.timeout_lock == true);
    CHECK(IsStopped(o));

    in.enable_requested = true;
    o = t.Step(in);
    CHECK(o.timeout_lock == false);
    CHECK(o.event_estop_unlock == true);
    in.enable_requested = false;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kNone);
    CHECK(o.vx == 1.0);
  }

  // ---- no command has EVER arrived is a timeout, not "no_source" ---------
  {
    // 11's own self-audit records that no_source has no producer. The age of a
    // command that never came is unbounded, so the contract's answer is the
    // timeout branch -- and it locks, which is right: nobody is driving.
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.has_cmd = false;
    in.last_cmd_rx_mono_s = 0.0;   // would look FRESH if has_cmd were ignored
    in.now_mono_s = 0.0;
    Tier1Output o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kTimeout);
    CHECK(o.timeout_lock == true);
    CHECK(StopReasonName(o.stop_reason) != "no_source");
  }

  // ---- the soft stop holds zero, and converges in BOTH directions --------
  {
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.vx = 1.0;
    in.local_estop_epoch = 5;
    in.cmd_estop_epoch = 4;          // upstream is behind
    // Held, not pulsed: ten periods and every one of them is zero. A "clear
    // for one period" implementation passes a single-period assertion and
    // lets the robot move 9 ms later.
    for (int i = 0; i < 10; ++i) {
      Tier1Output o = t.Step(in);
      CHECK(o.stop_reason == StopReason::kSoftEstop);
      CHECK(IsStopped(o));
    }
    // A HIGHER echo is just as much a disagreement: it means our own state is
    // behind, and proceeding would be acting on a stop we have not processed.
    // A < comparison would let this through.
    in.cmd_estop_epoch = 6;
    CHECK(t.Step(in).stop_reason == StopReason::kSoftEstop);
    // Agreement, and it resumes with no unlock action of any kind.
    in.cmd_estop_epoch = 5;
    Tier1Output o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kNone);
    CHECK(o.vx == 1.0);
  }

  // ---- sleep does not lock ------------------------------------------------
  {
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.vx = 1.0;
    in.sleep_readback = true;
    Tier1Output o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kSleep);
    CHECK(o.hes_lock == false);
    CHECK(o.timeout_lock == false);
    // A human wakes the machine; the readback clears and so does the stop, with
    // no enable required. 13 V-63: there is no wake command to send, so the
    // only thing that can clear this is the chassis itself.
    in.sleep_readback = false;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kNone);
    CHECK(o.vx == 1.0);
  }

  // ---- mode mismatch: idempotent event, carries the actual value ---------
  {
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.vx = 1.0;
    in.usage_mode_raw = 2;            // assist
    Tier1Output o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kModeMismatch);
    CHECK(o.event_mode_mismatch == true);
    // Without the actual value, "mode mismatch" says that something happened
    // and not what -- and 11 UM-4 needs it to build the warning event.
    CHECK(o.mode_mismatch_actual == 2);
    // At 100 Hz, a repeated event is an event storm.
    for (int i = 0; i < 5; ++i) {
      o = t.Step(in);
      CHECK(o.stop_reason == StopReason::kModeMismatch);
      CHECK(o.event_mode_mismatch == false);
    }
    // Back to navigation: released with no action, per 13 S3.1.
    in.usage_mode_raw = quadruped::kUsageModeNavigation;
    o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kNone);
    // ...and a SECOND excursion fires the event again. A latch that was set
    // once and never reset would report the first mismatch of the robot's life
    // and stay silent for every one after it.
    in.usage_mode_raw = 0;
    o = t.Step(in);
    CHECK(o.event_mode_mismatch == true);
    CHECK(o.mode_mismatch_actual == 0);
  }

  // ---- an unregistered usage mode also stops -----------------------------
  {
    // The comparison is against the navigation VALUE, not against a resolved
    // label, so a readback the open set cannot name still fails it. 13 S6.5
    // requires exactly this: an unregistered usage_mode takes the mismatch
    // path rather than being guessed into one of the known modes.
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.vx = 1.0;
    in.usage_mode_raw = 99;
    Tier1Output o = t.Step(in);
    CHECK(o.stop_reason == StopReason::kModeMismatch);
    CHECK(o.mode_mismatch_actual == 99);
  }

  // ---- a non-finite value on ANY of the six axes stops the robot ---------
  {
    // The branch exists because the PAYLOAD is contaminated, so it covers the
    // three axes that are trimmed to zero anyway: a message with a NaN in an
    // unused field is not a message whose other fields can be trusted.
    const double bad[] = {std::nan(""), INFINITY, -INFINITY};
    for (double v : bad) {
      for (int axis = 0; axis < 6; ++axis) {
        Tier1 t(Cfg());
        Tier1Input in = Healthy();
        double* axes[6] = {&in.vx, &in.vy, &in.wz, &in.vz, &in.v_roll, &in.v_pitch};
        *axes[axis] = v;
        Tier1Output o = t.Step(in);
        CHECK(o.stop_reason == StopReason::kNan);
        CHECK(o.event_nan == true);
        CHECK(IsStopped(o));
      }
    }
  }

  // ---- clamp: both signs, and the RIGHT limit per axis -------------------
  {
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.vx = 5.0;
    in.vy = 5.0;
    in.wz = 5.0;
    Tier1Output o = t.Step(in);
    CHECK(o.vx == 2.0);
    CHECK(o.vy == 1.0);
    // *** max_wz is 0.8 and max_vx is 2.0. A yaw rate clamped against the
    // LINEAR limit would come out 2.0 here, which is two and a half times the
    // permitted turn rate and reads as an ordinary line of code. The units
    // types make that call fail to compile; this asserts the number too.
    CHECK(o.wz == 0.8);

    in.vx = -5.0;
    in.vy = -5.0;
    in.wz = -5.0;
    o = t.Step(in);
    // The negative side is what a one-sided clamp gets wrong, and the symptom
    // is a robot that will not reverse.
    CHECK(o.vx == -2.0);
    CHECK(o.vy == -1.0);
    CHECK(o.wz == -0.8);

    // Inside the band, untouched.
    in.vx = 0.7;
    in.vy = -0.3;
    in.wz = 0.2;
    o = t.Step(in);
    CHECK(o.vx == 0.7);
    CHECK(o.vy == -0.3);
    CHECK(o.wz == 0.2);
  }

  // ---- axis trimming: an inactive axis is ZEROED, never passed through ---
  {
    // 11 S9.3.1 is explicit -- passing an inactive axis through makes the layer
    // above believe a command it issued was executed.
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.vx = 1.0;
    in.vz = 1.0;
    in.v_roll = 1.0;
    in.v_pitch = 1.0;
    Tier1Output o = t.Step(in);
    CHECK(o.vx == 1.0);
    // *** Unconditionally zero, and there is no switch that changes it. spec.*
    // defines max_vx_mps / max_vy_mps / max_wz_radps and NOTHING for these
    // three, so Tier 1 has no limit to hold them inside. Letting them through
    // unclamped would put an unbounded axis past the one function that exists
    // to bound them; clamping them against max_vx would be inventing a safety
    // number here. The config key that would enable them is asserted empty at
    // load instead, so an operator who sets it gets a refusal and not silence.
    CHECK(o.vz == 0.0);
    CHECK(o.v_roll == 0.0);
    CHECK(o.v_pitch == 0.0);
    // Large values, to be sure this is a trim and not a coincidence of the
    // command happening to be small.
    in.vz = 99.0;
    in.v_roll = -99.0;
    in.v_pitch = 99.0;
    o = t.Step(in);
    CHECK(o.vz == 0.0);
    CHECK(o.v_roll == 0.0);
    CHECK(o.v_pitch == 0.0);
    CHECK(o.stop_reason == StopReason::kNone);  // legal command, unused axes
  }

  // ---- a non-holonomic chassis has no lateral axis -----------------------
  {
    Tier1Config c = Cfg();
    c.limits.holonomic = false;
    Tier1 t(c);
    Tier1Input in = Healthy();
    in.vx = 1.0;
    in.vy = 1.0;
    Tier1Output o = t.Step(in);
    CHECK(o.vx == 1.0);
    CHECK(o.vy == 0.0);
    // Not a stop: the command is legal, one axis of it simply does not exist.
    CHECK(o.stop_reason == StopReason::kNone);
  }

  // ---- an uncalibrated limit clamps to zero, it does not pass through ----
  {
    // The real snapshot carries null for spec.max_* (V-01) and the process
    // refuses to start, so this state should be unreachable. Asserted anyway
    // because the failure direction matters: a limit that arrived as 0.0
    // through some other path must stop the robot, not release it.
    Tier1Config c = Cfg();
    c.limits.max_vx_mps = 0.0;
    Tier1 t(c);
    Tier1Input in = Healthy();
    in.vx = 5.0;
    Tier1Output o = t.Step(in);
    CHECK(o.vx == 0.0);
    CHECK(o.stop_reason == StopReason::kNone);  // clamped, not a fault
  }

  // ---- the locks are reported every period, not only on their edges ------
  {
    // 11 CR-12: the readback of these two is the only proof a lock is gone, so
    // a consumer must be able to read them at any moment rather than
    // reconstruct them from a stream of edges it may have missed.
    Tier1 t(Cfg());
    Tier1Input in = Healthy();
    in.hes_raw = true;
    t.Step(in);
    in.hes_raw = false;
    for (int i = 0; i < 5; ++i) {
      Tier1Output o = t.Step(in);
      CHECK(o.hes_lock == true);
      CHECK(o.event_estop_unlock == false);
    }
  }

  // ---- Step is noexcept, which is a compile-time claim -------------------
  {
    Tier1 t(Cfg());
    const Tier1Input in = Healthy();
    static_assert(noexcept(std::declval<Tier1&>().Step(std::declval<const Tier1Input&>())),
                  "Tier1::Step runs on ctrl (SCHED_FIFO 80) and must not throw: "
                  "an exception there unwinds the control loop");
    (void)t.Step(in);
  }

  if (g_failures == 0) {
    std::printf("ALL TIER1 TESTS PASSED\n");
    return 0;
  }
  std::printf("%d TIER1 TEST(S) FAILED\n", g_failures);
  return 1;
}
