/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: heading_resolver.cc
 * Brief: L1/L2/L3 heading degradation state machine (see heading_resolver.h)
 *
 * Description:
 * Implements the 11 S3.3.1~S3.3.4 chain: per-level transition gates with the
 * S3.3.3 hysteresis (degrade fast, recover slow), L2's active/blind substate, and
 * the per-level GnssHeading field derivation. No serial, no clock, no config
 * file, no ROS: everything is a parameter, so every transition is unit-tested
 * with injected facts and injected monotonic time.
 *
 * Angle convention (11 S3.3): the module reports heading/COG as true-north
 * clockwise degrees; heading_rad is ENU (east=0, ccw) via wrap(pi/2 - deg2rad),
 * and heading_true_north_rad keeps the raw value for the HMI only.
 */

#include "sensor/heading_resolver.h"

#include <algorithm>
#include <cmath>

namespace sensor {

namespace {
constexpr double kPi = 3.14159265358979323846;

double WrapPi(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return a - kPi;
}
// True-north clockwise degrees -> ENU radians (east=0, ccw). 11 S3.3:
// heading_enu = wrap(pi/2 - heading_ned). This is the one place the frame flips.
double TrueDegToEnu(double deg) { return WrapPi(kPi / 2.0 - deg * kPi / 180.0); }
double TrueDegToRad(double deg) { return deg * kPi / 180.0; }
}  // namespace

HeadingResolver::HeadingResolver(ResolverConfig cfg) : cfg_(cfg) {}

ResolveResult HeadingResolver::update(const HeadingInputs& in, double now_s) {
  // ---- instantaneous conditions -----------------------------------------
  const bool l1_ok = in.heading_present && in.baseline_valid &&
                     in.heading_cov_rad <= cfg_.cov_thresh_rad &&
                     in.heading_age_s <= cfg_.age_thresh_s;
  const bool speed_ok = in.speed_mps >= cfg_.cog_speed_thresh_mps;
  // Speed sustained >= 0.5 s: drives BOTH L2 admission and the blind->active
  // rise (11 S3.3.3 "rise 0.5 s / fall immediate"). Advanced once per tick.
  const bool speed_sustained =
      g_speed_.held(speed_ok, now_s, cfg_.blind_rise_sustain_s);
  const bool l2_admissible = in.fix_is_rtk && speed_sustained;

  HeadingEvent event = HeadingEvent::kNone;
  LostReason reason = LostReason::kNone;
  const int prev_level = level_;

  // ---- transitions (11 S3.3.3), evaluated from the current level ---------
  switch (level_) {
    case 1:
      // L1 -> L3 dual_antenna_fail: L1 lost AND the fix cannot start COG.
      if (g_l1_l3_.held(!l1_ok && in.fix_is_lost, now_s, cfg_.fix_lost_sustain_s)) {
        level_ = 3;
        reason = LostReason::kDualAntennaFail;
        event = HeadingEvent::kLost;
      } else if (g_l1_l2_.held(!l1_ok && !in.fix_is_lost, now_s,
                               cfg_.degrade_sustain_s)) {
        // L1 -> L2 degrade: L1 admission fails but the fix can still COG.
        level_ = 2;
        event = HeadingEvent::kDegraded;
      }
      break;
    case 2: {
      // Substate: active iff speed sustained; else blind (fall is immediate
      // because g_speed_ resets on a single below-threshold tick). Track the
      // blind spell for the heading_blind_timeout below.
      l2_blind_ = !speed_sustained;
      if (l2_blind_) {
        if (blind_since_s_ < 0.0) blind_since_s_ = now_s;
      } else {
        blind_since_s_ = -1.0;
      }
      if (g_l2_l1_.held(l1_ok, now_s, cfg_.recover_sustain_s)) {
        level_ = 1;
        event = HeadingEvent::kRecovered;
      } else if (g_l2_l3_.held(in.fix_is_lost, now_s, cfg_.fix_lost_sustain_s)) {
        level_ = 3;
        reason = LostReason::kFixLost;
        event = HeadingEvent::kLost;
      } else if (l2_blind_ && in.pending_autonomous_motion &&
                 blind_since_s_ >= 0.0 &&
                 (now_s - blind_since_s_) > cfg_.blind_timeout_s) {
        // L2-blind with a pending autonomous move, held too long -> L3.
        level_ = 3;
        reason = LostReason::kHeadingBlindTimeout;
        event = HeadingEvent::kLost;
      }
      break;
    }
    case 3:
      if (g_l3_l1_.held(l1_ok, now_s, cfg_.recover_sustain_s)) {
        level_ = 1;
        event = HeadingEvent::kRecovered;
      } else if (g_l3_l2_.held(l2_admissible, now_s, cfg_.recover_sustain_s)) {
        level_ = 2;
        event = HeadingEvent::kRecovered;
      }
      break;
  }

  // On any level change, drop the OLD level's transition dwell so a stale timer
  // cannot leak into the new state. g_speed_ is NOT reset -- it tracks the
  // physical speed continuously across levels.
  if (level_ != prev_level) {
    g_l1_l2_.reset(); g_l1_l3_.reset(); g_l2_l1_.reset(); g_l2_l3_.reset();
    g_l3_l1_.reset(); g_l3_l2_.reset();
    if (level_ == 2) {
      l2_blind_ = !speed_sustained;
      blind_since_s_ = l2_blind_ ? now_s : -1.0;
    } else {
      blind_since_s_ = -1.0;
    }
  }

  // ---- derive the GnssHeading for the resulting level -------------------
  GnssHeading h;
  h.t_mono = now_s;
  h.speed_mps = in.speed_mps;
  if (in.cog_present) h.cog_rad = TrueDegToEnu(in.cog_true_deg);
  h.baseline_m = in.baseline_m;
  h.baseline_valid = in.baseline_valid;

  if (level_ == 1) {
    h.source = "dual_antenna";
    h.level = 1;
    h.heading_rad = TrueDegToEnu(in.heading_true_deg);
    h.heading_true_north_rad = TrueDegToRad(in.heading_true_deg);
    h.heading_valid = true;
    h.cov_rad = in.heading_cov_rad;
    h.yaw_capable = true;
    h.i_heading = cfg_.i_heading_l1;
    h.age_s = in.heading_age_s;
    last_valid_heading_rad_ = h.heading_rad;
    last_valid_mono_s_ = now_s;
  } else if (level_ == 2) {
    h.source = "cog";
    h.level = 2;
    h.yaw_capable = false;
    h.i_heading = cfg_.i_heading_l2;
    // 11 S3.3 L2 cov estimate: cov_h / max(speed, thresh) / dt.
    const double v = std::max(in.speed_mps, cfg_.cog_speed_thresh_mps);
    h.cov_rad = cfg_.cov_h_m / v / cfg_.cog_diff_dt_s;
    if (l2_blind_) {
      // L2-blind: COG undefined at low speed -> heading_valid FALSE, but stay
      // level 2 and do NOT fire rtk_lost (11 S3.3.1 blind note). Hold the last
      // valid heading (H-2: heading_rad may carry the last known value).
      h.heading_valid = false;
      if (last_valid_heading_rad_) h.heading_rad = *last_valid_heading_rad_;
      h.age_s = last_valid_heading_rad_ ? (now_s - last_valid_mono_s_) : 0.0;
    } else {
      h.heading_valid = true;
      h.heading_rad = TrueDegToEnu(in.cog_true_deg);
      h.heading_true_north_rad = TrueDegToRad(in.cog_true_deg);
      h.age_s = 0.0;
      last_valid_heading_rad_ = h.heading_rad;
      last_valid_mono_s_ = now_s;
    }
  } else {
    h.source = "none";
    h.level = 3;
    h.heading_valid = false;
    h.cov_rad = std::nullopt;   // null at L3, never 0 (NAV-02)
    h.yaw_capable = false;
    h.i_heading = cfg_.i_heading_l3;
    if (last_valid_heading_rad_) h.heading_rad = *last_valid_heading_rad_;
    h.age_s = last_valid_heading_rad_ ? (now_s - last_valid_mono_s_) : 0.0;
  }

  ResolveResult r;
  r.heading = h;
  r.event = event;
  r.lost_reason = reason;
  return r;
}

}  // namespace sensor
