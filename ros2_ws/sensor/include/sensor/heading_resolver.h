/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: heading_resolver.h
 * Brief: The 11 S3.3.1~S3.3.4 heading degradation state machine (L1/L2/L3)
 *
 * Description:
 * The problem this solves. rtk_driver must turn raw GNSS facts into a stable
 * GnssHeading level: dual-antenna heading is best (L1), course-over-ground is a
 * moving-only fallback (L2), and no heading is L3. The hard part is not the
 * instantaneous choice but the ANTI-CHATTER: 11 S3.3.3 makes degradation fast
 * and recovery slow (L1->L2 0.5 s, ->L3 1.0 s, any recovery 2.0 s), plus L2's
 * own active/blind substate (speed crossing 0.5 m/s, rise 0.5 s / fall immediate).
 * Getting the hysteresis wrong floods the alarm cursor with rtk_lost on every
 * stop (11 S3.3.1 L2-blind note) or steers on a stale heading. This class is that
 * state machine, and NOTHING else: it holds the level, the timers and the last
 * valid heading, and returns a GnssHeading + which transition event fired.
 *
 * Which section this follows: 11 S3.3.1 (levels + admission), S3.3.2 (COG
 * constraints), S3.3.3 (the transition/hysteresis table), S3.3.4 (rtk_lost). All
 * timing is CLOCK_MONOTONIC seconds (11 S0.2.1 / CLK-C1) -- the caller reads
 * mono_now_s() and passes it in, so the maths is testable with injected time and
 * there is no realtime clock to read the wrong one from.
 *
 * What it does NOT do, and the boundary. It reads NO serial, NO clock, NO config
 * file: inputs (the parsed facts), the monotonic now, and the resolved config are
 * all parameters. It does NOT publish -- the driver stamps the S3.0 envelope and
 * emits the rt/gnss/heading + event/alarm/rtk messages. It does NOT stop the
 * robot or suspend the task (RL-1/RL-2, 11 S3.3.4): those are the consumers'
 * response to heading_valid=false and the rtk_lost event. It does NOT implement
 * L1.5 (odom_aligned, 11 S3.3.2a): that level's i_heading and hard cap are still
 * blank pending T7 (11 S14.4 G-15), so implementing it would mean inventing
 * safety numbers -- left out, and the caller sees only L1/L2/L3.
 *
 * Traps this exists to avoid. cov_rad MUST be null at L3 (no solution has no
 * covariance) -- a 0 reads as a perfect heading (NAV-02). L2-blind (in L2 but
 * slow) is NOT a fault: it sets heading_valid=false but stays level 2 and does
 * NOT fire rtk_lost -- only a real S3.3.3 L3 trigger does. yaw_capable is emitted
 * from the level (L1 true, else false), never derived by the consumer (H-1).
 * Every threshold is injected, never defaulted here: they are safety values still
 * pending T7 calibration (11 S3.3.3), and CLAUDE.md 3.1 forbids a code default.
 */

#ifndef SENSOR__HEADING_RESOLVER_H_
#define SENSOR__HEADING_RESOLVER_H_

#include <optional>

#include "sensor/gnss_heading.h"

namespace sensor {

// The facts one tick feeds the resolver, already parsed and unit-converted. The
// resolver never sees NMEA -- nmea_parser + the driver produce these.
struct HeadingInputs {
  // Position fix quality, mapped from GGA. ONE fact drives COG: fix_is_lost.
  // COG is admissible iff !fix_is_lost, i.e. fix in {rtk_fixed, rtk_float, dgps}
  // (2026-08-16 user ruling: DGPS admitted to COG). fix_is_lost = {single,
  // no_fix, DR} OR GnssFix stale (>1 s) -- the no-usable-heading set.
  bool fix_is_lost = false;
  // Dual-antenna heading (from TRA / heading log):
  bool heading_present = false;   // a dual-antenna heading arrived this window
  bool baseline_valid = false;    // fixed-integer baseline (TRA QF == NARROW_INT)
  double heading_true_deg = 0.0;  // dual-antenna heading, true-north clockwise deg
  double heading_cov_rad = 0.0;   // 1sigma of the dual-antenna heading
  double heading_age_s = 0.0;     // age of the dual-antenna solution
  std::optional<double> baseline_m;  // measured baseline length, if known
  // Course/speed over ground (from RMC): the L2 source.
  double speed_mps = 0.0;         // GNSS-Doppler ground speed
  double cog_true_deg = 0.0;      // course over ground, true-north clockwise deg
  bool cog_present = false;       // false at a standstill (course field empty)
  // Whether an autonomous-motion request is pending -- the ONLY extra fact the
  // heading_blind_timeout (11 S3.3.3) needs. The driver supplies it; false is the
  // safe default (no blind timeout without a real request). A wiring seam today.
  bool pending_autonomous_motion = false;
};

// Injected thresholds (11 S3.3.1/S3.3.3). NO defaults -- safety values pending T7
// (11 S3.3.3 note), CLAUDE.md 3.1. Filled from configs/ by the driver.
struct ResolverConfig {
  double cov_thresh_rad;        // L1 admission: cov_rad <= this (0.02)
  double age_thresh_s;          // L1 admission: age_s <= this (0.2)
  double cog_speed_thresh_mps;  // L2/blind: speed >= this (0.5)
  double degrade_sustain_s;     // L1->L2 hold (0.5)
  double recover_sustain_s;     // any recovery hold (2.0)
  double fix_lost_sustain_s;    // ->L3 fix_lost / dual_antenna_fail hold (1.0)
  double blind_rise_sustain_s;  // L2-blind -> L2-active hold (0.5)
  double blind_timeout_s;       // L2-blind + pending motion -> L3 (30.0)
  double cov_h_m;               // horizontal position 1sigma, for the L2 cov estimate
  double cog_diff_dt_s;         // COG differencing window, for the L2 cov estimate
  double i_heading_l1;          // 1.00
  double i_heading_l2;          // 0.40
  double i_heading_l3;          // 0.00
};

// Which 11 S3.3.3 / S3.3.4 event this tick produced (the driver publishes it).
enum class HeadingEvent {
  kNone,
  kDegraded,   // heading_degraded (warn) -- L1 -> L2
  kRecovered,  // heading_recovered (info) -- any level up
  kLost,       // rtk_lost (alarm) -- entered L3
};

// The 11 S3.3.4 reason, meaningful only when event == kLost.
enum class LostReason { kNone, kFixLost, kHeadingBlindTimeout, kDualAntennaFail };

struct ResolveResult {
  GnssHeading heading;       // the S3.3 payload to publish this tick
  HeadingEvent event = HeadingEvent::kNone;
  LostReason lost_reason = LostReason::kNone;
};

// A condition that must hold continuously for a duration (the S3.3.3 hysteresis).
// One per transition. mono seconds in; reset the moment the condition is false,
// so a single false tick restarts the whole dwell -- that is the anti-chatter.
class SustainedGate {
 public:
  bool held(bool cond, double now_s, double duration_s) {
    if (!cond) {
      since_ = -1.0;
      return false;
    }
    if (since_ < 0.0) {
      since_ = now_s;
    }
    return (now_s - since_) >= duration_s;
  }
  void reset() { since_ = -1.0; }

 private:
  double since_ = -1.0;   // mono when the condition became true; <0 = not true now
};

// The degradation state machine. Constructed at L3 (no heading) -- the safe start
// (11 S3.3, and matches the HMI dial's "startup points North").
class HeadingResolver {
 public:
  explicit HeadingResolver(ResolverConfig cfg);

  // Advance one tick. `now_s` is CLOCK_MONOTONIC seconds. Returns the GnssHeading
  // to publish plus any transition event. The heading it returns is always
  // GnssHeadingConsistent().
  ResolveResult update(const HeadingInputs& in, double now_s);

  int level() const { return level_; }

 private:
  ResolverConfig cfg_;
  int level_ = 3;                 // 1/2/3; start L3 (safe)
  bool l2_blind_ = true;          // within L2: true = blind (slow), false = active
  double blind_since_s_ = -1.0;   // mono when the current blind spell began (<0 none)
  std::optional<double> last_valid_heading_rad_;  // for freeze + the rtk_lost event
  double last_valid_mono_s_ = 0.0;

  // One gate per S3.3.3 transition; a shared speed gate drives both L2 admission
  // and the blind->active rise (same "speed sustained 0.5 s" condition).
  SustainedGate g_speed_;      // speed >= thresh, sustained (L2 admit + blind rise)
  SustainedGate g_l1_l2_;      // !L1_ok, degrade
  SustainedGate g_l1_l3_;      // !L1_ok && fix_is_lost, dual_antenna_fail
  SustainedGate g_l2_l1_;      // L1_ok, recover
  SustainedGate g_l2_l3_;      // fix_lost, ->L3
  SustainedGate g_l3_l1_;      // L1_ok, recover
  SustainedGate g_l3_l2_;      // L2 admissible, recover
};

}  // namespace sensor

#endif  // SENSOR__HEADING_RESOLVER_H_
