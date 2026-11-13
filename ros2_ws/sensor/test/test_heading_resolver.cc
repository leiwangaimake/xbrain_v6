/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_heading_resolver.cc
 * Brief: Offline unit test for the L1/L2/L3 heading resolver (no ROS, no gtest)
 *
 * Description:
 * Drives HeadingResolver with injected facts and injected monotonic time, one
 * case per 11 S3.3.3 transition plus the L2 active/blind substate, the per-level
 * fields, the anti-chatter dwell, and the ENU conversion. Time is stepped at a
 * fixed dt so the S3.3.3 hysteresis (0.5 / 1.0 / 2.0 / 30 s) is exercised. Each
 * assertion is paired with a mutation that turns it red (CLAUDE.md 3.3): the
 * load-bearing ones are the L3 null cov (NAV-02) and the degrade dwell (a
 * degrade-on-first-tick mutant chatters).
 */

#include "sensor/heading_resolver.h"

#include <cmath>
#include <cstdio>

using sensor::GnssHeading;
using sensor::HeadingEvent;
using sensor::HeadingInputs;
using sensor::HeadingResolver;
using sensor::LostReason;
using sensor::ResolveResult;
using sensor::ResolverConfig;

static int g_failures = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
      ++g_failures;                                                       \
    }                                                                     \
  } while (0)
static bool near(double a, double b, double eps) { return std::fabs(a - b) < eps; }

// The engineering-default thresholds (11 S3.3.1/S3.3.3), the values the driver
// will load from configs/. cov 0.02 / age 0.2 / cog 0.5 / degrade 0.5 /
// recover 2.0 / fix_lost 1.0 / blind_rise 0.5 / blind_timeout 30 / cov_h 0.05 /
// dt 0.9 / i 1.0,0.4,0.0.
static ResolverConfig cfg() {
  return {0.02, 0.2, 0.5, 0.5, 2.0, 1.0, 0.5, 30.0, 0.05, 0.9, 1.0, 0.4, 0.0};
}

static HeadingInputs l1_good() {
  HeadingInputs in;
  in.heading_present = true; in.baseline_valid = true;
  in.heading_true_deg = 90.0; in.heading_cov_rad = 0.01; in.heading_age_s = 0.1;
  in.fix_is_lost = false;   // COG-capable fix (rtk_fixed/float/dgps)
  in.speed_mps = 1.0; in.cog_true_deg = 95.0; in.cog_present = true;
  return in;
}
// L1 admission fails (no dual-antenna) but the fix can still COG while moving.
static HeadingInputs l2_only() {
  HeadingInputs in = l1_good();
  in.baseline_valid = false; in.heading_present = false;
  return in;
}
static HeadingInputs fix_lost() {
  HeadingInputs in;
  in.fix_is_lost = true; in.baseline_valid = false; in.heading_present = false;
  in.speed_mps = 0.0; in.cog_present = false;
  return in;
}

// Step the resolver over `dur` seconds at dt. Returns the tick where a
// transition event fired during the drive (events fire on ONE tick, then the
// level is stable and the fields are the same), or the last tick if none did.
static ResolveResult drive(HeadingResolver& r, const HeadingInputs& in,
                           double& now, double dur, double dt) {
  ResolveResult res = r.update(in, now);
  ResolveResult evt = res;
  const double end = now + dur;
  now += dt;
  while (now <= end + 1e-9) {
    res = r.update(in, now);
    if (res.event != HeadingEvent::kNone) evt = res;
    now += dt;
  }
  return (evt.event != HeadingEvent::kNone) ? evt : res;
}

static void test_l3_to_l1_and_fields() {
  HeadingResolver r(cfg());
  double now = 100.0;
  ResolveResult res = drive(r, l1_good(), now, 2.5, 0.1);  // recover 2.0 s
  CHECK(r.level() == 1);
  CHECK(res.heading.heading_valid);
  CHECK(res.heading.source == "dual_antenna");
  CHECK(res.heading.level == 1);
  CHECK(res.heading.yaw_capable);
  CHECK(near(res.heading.i_heading, 1.0, 1e-9));
  CHECK(res.heading.cov_rad.has_value() && near(*res.heading.cov_rad, 0.01, 1e-9));
  // 90 deg true (east) -> ENU 0.
  CHECK(near(res.heading.heading_rad, 0.0, 1e-6));
}

static void test_l1_to_l2_degrade() {
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);           // reach L1
  ResolveResult res = drive(r, l2_only(), now, 0.7, 0.1);  // degrade 0.5 s
  CHECK(r.level() == 2);
  CHECK(res.heading.source == "cog");
  CHECK(res.heading.heading_valid);            // moving -> L2-active
  CHECK(!res.heading.yaw_capable);
  CHECK(near(res.heading.i_heading, 0.4, 1e-9));
}

static void test_l2_substates() {
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);
  drive(r, l2_only(), now, 0.7, 0.1);           // L2-active (moving)
  CHECK(r.level() == 2);
  // Stop: fall to blind is IMMEDIATE, level stays 2, no event.
  HeadingInputs slow = l2_only(); slow.speed_mps = 0.0; slow.cog_present = false;
  ResolveResult res = r.update(slow, now); now += 0.1;
  CHECK(r.level() == 2);
  CHECK(!res.heading.heading_valid);           // L2-blind
  CHECK(res.event == HeadingEvent::kNone);     // blind is not a fault
  // Move again: rise to active needs 0.5 s sustained.
  res = drive(r, l2_only(), now, 0.7, 0.1);
  CHECK(r.level() == 2);
  CHECK(res.heading.heading_valid);
}

static void test_l2_to_l3_fix_lost() {
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);
  drive(r, l2_only(), now, 0.7, 0.1);
  ResolveResult res = drive(r, fix_lost(), now, 1.2, 0.1);   // fix_lost 1.0 s
  CHECK(r.level() == 3);
  CHECK(res.event == HeadingEvent::kLost);
  CHECK(res.lost_reason == LostReason::kFixLost);
}

static void test_l1_to_l3_dual_antenna_fail() {
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);
  // Baseline lost AND fix lost (cannot COG) -> dual_antenna_fail after 1.0 s.
  HeadingInputs bad = fix_lost();               // fix_is_lost, no heading
  ResolveResult res = drive(r, bad, now, 1.2, 0.1);
  CHECK(r.level() == 3);
  CHECK(res.lost_reason == LostReason::kDualAntennaFail);
}

// L3 -> L2 recovery on a COG-capable fix that is NOT dual-antenna -- the
// DGPS-admitted-to-COG path (2026-08-16 user ruling: COG on {rtk_fixed,
// rtk_float, dgps}). A dgps fix is fix_is_lost=false, so l2_admissible
// (!fix_is_lost && speed) holds and L3 recovers to L2 (COG). MUTATION: the old
// l2_admissible = fix_is_rtk ({rtk_fixed,rtk_float} only) left a dgps/non-rtk
// fix STUCK in L3 -- this recovery would never fire.
static void test_l3_to_l2_recover_cog_capable() {
  HeadingResolver r(cfg());
  double now = 0.0;
  r.update(fix_lost(), now); now += 0.1;                    // start at L3
  CHECK(r.level() == 3);
  ResolveResult res = drive(r, l2_only(), now, 3.0, 0.1);   // recover_sustain 2.0 s
  CHECK(r.level() == 2);
  CHECK(res.heading.source == "cog");
  CHECK(res.heading.heading_valid);
}

static void test_l3_fields() {
  HeadingResolver r(cfg());
  double now = 0.0;
  ResolveResult res = r.update(fix_lost(), now);   // starts at L3
  CHECK(r.level() == 3);
  CHECK(res.heading.source == "none");
  CHECK(!res.heading.heading_valid);
  CHECK(!res.heading.cov_rad.has_value());        // null, NOT 0 (NAV-02)
  CHECK(!res.heading.yaw_capable);
  CHECK(near(res.heading.i_heading, 0.0, 1e-9));
}

static void test_blind_timeout() {
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);
  drive(r, l2_only(), now, 0.7, 0.1);
  // Stop with a pending autonomous move: after > 30 s blind -> L3.
  HeadingInputs blind = l2_only();
  blind.speed_mps = 0.0; blind.cog_present = false;
  blind.pending_autonomous_motion = true;
  ResolveResult res = drive(r, blind, now, 31.0, 0.5);
  CHECK(r.level() == 3);
  CHECK(res.lost_reason == LostReason::kHeadingBlindTimeout);
}

static void test_anti_chatter_dwell() {
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);           // L1
  CHECK(r.level() == 1);
  // 0.4 s of bad (< 0.5 dwell) -> still L1.
  drive(r, l2_only(), now, 0.4, 0.1);
  CHECK(r.level() == 1);
  // ONE good tick must RESET the degrade dwell (single false restarts it).
  r.update(l1_good(), now); now += 0.1;
  CHECK(r.level() == 1);
  // Another 0.4 s of bad -> still L1 (dwell restarted, mutation: no reset -> L2).
  drive(r, l2_only(), now, 0.4, 0.1);
  CHECK(r.level() == 1);
}

// 99 U151: the two sources switch to each other on the tick, both ways.
static void test_cog_blind_falls_to_a_usable_dual_antenna_at_once() {
  // In L2 (dual antenna had failed), COG blind on one slow tick, the dual
  // antenna usable again but not yet 2 s stable: the heading must stay VALID
  // from the dual antenna. Mutation: keep the old L2-blind output -> invalid.
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);
  drive(r, l2_only(), now, 1.0, 0.1);             // -> L2 active
  CHECK(r.level() == 2);
  HeadingInputs slow = l1_good();                 // dual antenna back, speed dips
  slow.speed_mps = 0.2;
  ResolveResult res = r.update(slow, now); now += 0.1;
  CHECK(r.level() == 2);                          // level dwell unchanged
  CHECK(res.heading.heading_valid);
  CHECK(res.heading.source == "dual_antenna");
  CHECK(res.heading.yaw_capable);
  CHECK(near(res.heading.heading_rad, 0.0, 1e-9));   // 90 deg true -> 0 ENU
}

static void test_failed_baseline_falls_to_a_live_cog_at_once() {
  // In L1 while moving, the baseline fails: COG is already live, so the very
  // first bad tick answers COG (no 0.5 s on a stale dual heading). Mutation:
  // old L1 output -> source dual_antenna on the bad tick.
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);             // L1, moving, COG live
  ResolveResult res = r.update(l2_only(), now); now += 0.1;
  CHECK(r.level() == 1);                          // the degrade dwell still runs
  CHECK(res.heading.heading_valid);
  CHECK(res.heading.source == "cog");
  CHECK(!res.heading.yaw_capable);
}

// 99 U189: in L1, a failed baseline with COG NOT live (slow / turning on the
// spot) must not leave the receiver heading valid for the 0.5 s dwell -- that
// heading is garbage (2026-11-13: 84 deg off, and P1's bridge anchored on it).
// The tick answers the L2-blind shape at once, holding the last VALID heading,
// and the level dwell itself is unchanged. Mutation: the U151 line
// use_dual = l1_ok || !cog_live -> valid dual_antenna at 30 deg -> red.
static void test_failed_baseline_without_cog_is_invalid_at_once() {
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);             // L1 at 90 deg true (ENU 0)
  HeadingInputs bad = l1_good();                  // receiver still reports a heading
  bad.baseline_valid = false;                     // ... with a failed baseline
  bad.heading_true_deg = 30.0;                    // ... that is wrong
  bad.speed_mps = 0.2;                            // slow: COG not live
  ResolveResult res = r.update(bad, now); now += 0.1;
  CHECK(r.level() == 1);                          // the degrade dwell still runs
  CHECK(!res.heading.heading_valid);              // never valid on a failed baseline
  CHECK(res.heading.level == 2);                  // NOT L1-shaped (P3 dock capture
  CHECK(res.heading.source == "cog");             //   checks heading_level == 1)
  CHECK(!res.heading.yaw_capable);
  CHECK(near(res.heading.heading_rad, 0.0, 1e-9));  // held last valid, not 30 deg
  // A covariance failure is the same L1 admission failure (baseline fine).
  HeadingResolver r2(cfg());
  double now2 = 0.0;
  drive(r2, l1_good(), now2, 2.5, 0.1);
  HeadingInputs noisy = l1_good();
  noisy.heading_cov_rad = 0.5; noisy.speed_mps = 0.2;
  res = r2.update(noisy, now2);
  CHECK(!res.heading.heading_valid);
}

// 99 U189: the same tick with the fix lost answers the L3 shape, not L2-blind
// (the dwell would end in L3 dual_antenna_fail). Mutation: drop the
// !fix_is_lost guard -> source "cog" at level 2 on a lost fix -> red.
static void test_failed_baseline_on_a_lost_fix_is_l3_shaped() {
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);
  HeadingInputs bad = l1_good();
  bad.baseline_valid = false; bad.fix_is_lost = true; bad.speed_mps = 0.2;
  ResolveResult res = r.update(bad, now);
  CHECK(r.level() == 1);
  CHECK(!res.heading.heading_valid);
  CHECK(res.heading.level == 3 && res.heading.source == "none");
}

static void test_both_usable_keeps_the_level_preference() {
  // L2 active and the dual antenna usable again: COG until the 2 s recover
  // dwell promotes to L1, then dual antenna (preferred once stable). Mutation:
  // prefer dual antenna at once -> chatter on every flicker.
  HeadingResolver r(cfg());
  double now = 0.0;
  drive(r, l1_good(), now, 2.5, 0.1);
  drive(r, l2_only(), now, 1.0, 0.1);
  ResolveResult res = r.update(l1_good(), now); now += 0.1;
  CHECK(res.heading.source == "cog" && res.heading.heading_valid);
  drive(r, l1_good(), now, 2.5, 0.1);
  res = r.update(l1_good(), now);
  CHECK(r.level() == 1 && res.heading.source == "dual_antenna");
}

int main() {
  test_l3_to_l1_and_fields();
  test_l1_to_l2_degrade();
  test_l2_substates();
  test_l2_to_l3_fix_lost();
  test_l1_to_l3_dual_antenna_fail();
  test_l3_to_l2_recover_cog_capable();
  test_l3_fields();
  test_blind_timeout();
  test_anti_chatter_dwell();
  test_cog_blind_falls_to_a_usable_dual_antenna_at_once();
  test_failed_baseline_falls_to_a_live_cog_at_once();
  test_failed_baseline_without_cog_is_invalid_at_once();
  test_failed_baseline_on_a_lost_fix_is_l3_shaped();
  test_both_usable_keeps_the_level_preference();
  if (g_failures == 0) {
    std::printf("ALL HEADING RESOLVER TESTS PASSED\n");
    return 0;
  }
  std::printf("%d HEADING RESOLVER TEST(S) FAILED\n", g_failures);
  return 1;
}
