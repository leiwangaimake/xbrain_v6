/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: gnss_heading.h
 * Brief: The rt/gnss/heading DATA payload struct (11 S3.3) + its JSON serialiser
 *
 * Description:
 * The problem this solves. rtk_driver publishes rt/gnss/heading on the RT plane;
 * its inner data object is the 11 S3.3 GnssHeading message -- fourteen fields the
 * whole nav stack gates on. This file is the C++ shape of that message plus the
 * one place it is turned into JSON text, so the field names and null rules live
 * once and the resolver fills a struct rather than hand-building a string.
 *
 * Which section this follows: 11 S3.3 (the field table). Conventions are the
 * contract's, NOT invented here: heading_rad is ENU (east = 0, ccw positive, the
 * REP-105 frame the control loop uses); heading_true_north_rad is the raw
 * true-north clockwise value (0 = north) and is HMI-display-only -- 11 S3.3 marks
 * it "控制回路禁用". cov_rad is std::optional because 11 S3.3 makes it null at L3
 * (no heading solution has no covariance); the other optionals are the "--"
 * (not-required) fields (heading_true_north_rad, cog_rad, baseline_m,
 * baseline_valid).
 *
 * What it does NOT do, and the boundary. It does NOT DECIDE any field -- source /
 * level / heading_valid / cov_rad / yaw_capable / i_heading are the resolver's
 * 11 S3.3.1 output (the degradation chain, a stateful decision). It does NOT
 * stamp the 11 S3.0 outer envelope (v/rid/ts/mono/boot/seq/src/ts_sync) -- that
 * is common/envelope/envelope_writer.h, wrapped by the driver. It links no zenoh
 * (emit-text only, same rule as common/zenoh/session_config.h) and no ROS.
 *
 * Traps this exists to avoid. source is a closed set (kHeadingSource) and is
 * one-to-one with level (11 S3.3): GnssHeadingConsistent() enforces both, and the
 * caller must NOT publish a struct it rejects -- an out-of-set or mismatched
 * source is the silent-degrade 11 S13.6 forbids, not a value to serialise anyway.
 * A null-valued cov_rad must serialise as JSON null, never as 0.0: a 0 covariance
 * reads as a perfect heading, the exact over-trust NAV-02 rules out.
 */

#ifndef SENSOR__GNSS_HEADING_H_
#define SENSOR__GNSS_HEADING_H_

#include <optional>
#include <string>

namespace sensor {

// The 11 S3.3 GnssHeading DATA payload. Defaults are the SAFE state (L3, no
// heading, not yaw-capable) so a partially-filled struct never reads as a valid
// fix by omission. std::optional marks the fields 11 S3.3 allows to be null.
struct GnssHeading {
  double heading_rad = 0.0;                       // ENU east=0 ccw (required)
  std::optional<double> heading_true_north_rad;   // raw true-north cw; HMI only
  bool heading_valid = false;                     // THE availability criterion (H-1)
  std::string source = "none";                    // kHeadingSource: dual_antenna|cog|none
  int level = 3;                                  // 1/2/3, one-to-one with source
  std::optional<double> cov_rad;                  // 1sigma heading stddev; null at L3
  double speed_mps = 0.0;                         // ground speed (GNSS Doppler)
  std::optional<double> cog_rad;                  // course over ground (rad, ENU)
  std::optional<double> baseline_m;               // measured dual-antenna baseline
  std::optional<bool> baseline_valid;             // dual-antenna fixed solve (L1 gate)
  bool yaw_capable = false;                        // allow in-place rotation (level==1)
  double i_heading = 0.0;                          // [0,1] heading -> speed gate factor
  double age_s = 0.0;                              // solution age, s
  double t_mono = 0.0;                             // CLOCK_MONOTONIC seconds (CLK-C1)
};

// True iff source is a kHeadingSource member AND matches level one-to-one
// (dual_antenna<->1, cog<->2, none<->3, 11 S3.3). The caller must reject a false
// return before publishing (11 S13.6: no silent degrade to a nearby value).
bool GnssHeadingConsistent(const GnssHeading& h);

// Serialise the inner DATA object as JSON text ({"heading_rad":...,...}). Null
// optionals emit JSON null (never 0). The 11 S3.0 envelope is the driver's
// concern (EnvelopeWriter) -- this returns only the data object.
std::string ToJsonData(const GnssHeading& h);

}  // namespace sensor

#endif  // SENSOR__GNSS_HEADING_H_
