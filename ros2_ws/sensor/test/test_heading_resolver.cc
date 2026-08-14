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
static bool Near(double a, double b, double eps) { return std::fabs(a - b) < eps; }

// The engineering-default thresholds (11 S3.3.1/S3.3.3), the values the driver
// will load from configs/. cov 0.02 / age 0.2 / cog 0.5 / degrade 0.5 /
// recover 2.0 / fix_lost 1.0 / blind_rise 0.5 / blind_timeout 30 / cov_h 0.05 /
// dt 0.9 / i 1.0,0.4,0.0.
static ResolverConfig Cfg() {
  return {0.02, 0.2, 0.5, 0.5, 2.0, 1.0, 0.5, 30.0, 0.05, 0.9, 1.0, 0.4, 0.0};
}

static HeadingInputs L1Good() {
  HeadingInputs in;
  in.heading_present = true; in.baseline_valid = true;
  in.heading_true_deg = 90.0; in.heading_cov_rad = 0.01; in.heading_age_s = 0.1;
  in.fix_is_rtk = true; in.fix_is_lost = false;
  in.speed_mps = 1.0; in.cog_true_deg = 95.0; in.cog_present = true;
  return in;
}
// L1 admission fails (no dual-antenna) but the fix can still COG while moving.
static HeadingInputs L2Only() {
  HeadingInputs in = L1Good();
  in.baseline_valid = false; in.heading_present = false;
  return in;
}
static HeadingInputs FixLost() {
  HeadingInputs in;
  in.fix_is_lost = true; in.baseline_valid = false; in.heading_present = false;
  in.fix_is_rtk = false; in.speed_mps = 0.0; in.cog_present = false;
  return in;
}

// Step the resolver over `dur` seconds at dt. Returns the tick where a
// transition event fired during the drive (events fire on ONE tick, then the
// level is stable and the fields are the same), or the last tick if none did.
static ResolveResult Drive(HeadingResolver& r, const HeadingInputs& in,
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

static void TestL3ToL1AndFields() {
  HeadingResolver r(Cfg());
  double now = 100.0;
  ResolveResult res = Drive(r, L1Good(), now, 2.5, 0.1);  // recover 2.0 s
  CHECK(r.level() == 1);
  CHECK(res.heading.heading_valid);
  CHECK(res.heading.source == "dual_antenna");
  CHECK(res.heading.level == 1);
  CHECK(res.heading.yaw_capable);
  CHECK(Near(res.heading.i_heading, 1.0, 1e-9));
  CHECK(res.heading.cov_rad.has_value() && Near(*res.heading.cov_rad, 0.01, 1e-9));
  // 90 deg true (east) -> ENU 0.
  CHECK(Near(res.heading.heading_rad, 0.0, 1e-6));
}

static void TestL1ToL2Degrade() {
  HeadingResolver r(Cfg());
  double now = 0.0;
  Drive(r, L1Good(), now, 2.5, 0.1);           // reach L1
  ResolveResult res = Drive(r, L2Only(), now, 0.7, 0.1);  // degrade 0.5 s
  CHECK(r.level() == 2);
  CHECK(res.heading.source == "cog");
  CHECK(res.heading.heading_valid);            // moving -> L2-active
  CHECK(!res.heading.yaw_capable);
  CHECK(Near(res.heading.i_heading, 0.4, 1e-9));
}

static void TestL2Substates() {
  HeadingResolver r(Cfg());
  double now = 0.0;
  Drive(r, L1Good(), now, 2.5, 0.1);
  Drive(r, L2Only(), now, 0.7, 0.1);           // L2-active (moving)
  CHECK(r.level() == 2);
  // Stop: fall to blind is IMMEDIATE, level stays 2, no event.
  HeadingInputs slow = L2Only(); slow.speed_mps = 0.0; slow.cog_present = false;
  ResolveResult res = r.update(slow, now); now += 0.1;
  CHECK(r.level() == 2);
  CHECK(!res.heading.heading_valid);           // L2-blind
  CHECK(res.event == HeadingEvent::kNone);     // blind is not a fault
  // Move again: rise to active needs 0.5 s sustained.
  res = Drive(r, L2Only(), now, 0.7, 0.1);
  CHECK(r.level() == 2);
  CHECK(res.heading.heading_valid);
}

static void TestL2ToL3FixLost() {
  HeadingResolver r(Cfg());
  double now = 0.0;
  Drive(r, L1Good(), now, 2.5, 0.1);
  Drive(r, L2Only(), now, 0.7, 0.1);
  ResolveResult res = Drive(r, FixLost(), now, 1.2, 0.1);   // fix_lost 1.0 s
  CHECK(r.level() == 3);
  CHECK(res.event == HeadingEvent::kLost);
  CHECK(res.lost_reason == LostReason::kFixLost);
}

static void TestL1ToL3DualAntennaFail() {
  HeadingResolver r(Cfg());
  double now = 0.0;
  Drive(r, L1Good(), now, 2.5, 0.1);
  // Baseline lost AND fix lost (cannot COG) -> dual_antenna_fail after 1.0 s.
  HeadingInputs bad = FixLost();               // fix_is_lost, no heading
  ResolveResult res = Drive(r, bad, now, 1.2, 0.1);
  CHECK(r.level() == 3);
  CHECK(res.lost_reason == LostReason::kDualAntennaFail);
}

static void TestL3Fields() {
  HeadingResolver r(Cfg());
  double now = 0.0;
  ResolveResult res = r.update(FixLost(), now);   // starts at L3
  CHECK(r.level() == 3);
  CHECK(res.heading.source == "none");
  CHECK(!res.heading.heading_valid);
  CHECK(!res.heading.cov_rad.has_value());        // null, NOT 0 (NAV-02)
  CHECK(!res.heading.yaw_capable);
  CHECK(Near(res.heading.i_heading, 0.0, 1e-9));
}

static void TestBlindTimeout() {
  HeadingResolver r(Cfg());
  double now = 0.0;
  Drive(r, L1Good(), now, 2.5, 0.1);
  Drive(r, L2Only(), now, 0.7, 0.1);
  // Stop with a pending autonomous move: after > 30 s blind -> L3.
  HeadingInputs blind = L2Only();
  blind.speed_mps = 0.0; blind.cog_present = false;
  blind.pending_autonomous_motion = true;
  ResolveResult res = Drive(r, blind, now, 31.0, 0.5);
  CHECK(r.level() == 3);
  CHECK(res.lost_reason == LostReason::kHeadingBlindTimeout);
}

static void TestAntiChatterDwell() {
  HeadingResolver r(Cfg());
  double now = 0.0;
  Drive(r, L1Good(), now, 2.5, 0.1);           // L1
  CHECK(r.level() == 1);
  // 0.4 s of bad (< 0.5 dwell) -> still L1.
  Drive(r, L2Only(), now, 0.4, 0.1);
  CHECK(r.level() == 1);
  // ONE good tick must RESET the degrade dwell (single false restarts it).
  r.update(L1Good(), now); now += 0.1;
  CHECK(r.level() == 1);
  // Another 0.4 s of bad -> still L1 (dwell restarted, mutation: no reset -> L2).
  Drive(r, L2Only(), now, 0.4, 0.1);
  CHECK(r.level() == 1);
}

int main() {
  TestL3ToL1AndFields();
  TestL1ToL2Degrade();
  TestL2Substates();
  TestL2ToL3FixLost();
  TestL1ToL3DualAntennaFail();
  TestL3Fields();
  TestBlindTimeout();
  TestAntiChatterDwell();
  if (g_failures == 0) {
    std::printf("ALL HEADING RESOLVER TESTS PASSED\n");
    return 0;
  }
  std::printf("%d HEADING RESOLVER TEST(S) FAILED\n", g_failures);
  return 1;
}
