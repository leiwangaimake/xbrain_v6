/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: gnss_fix.h
 * Brief: GnssFix message (11 S3.2) -- position half of the RT-plane GNSS output
 *
 * Description:
 * The position companion to GnssHeading. rtk_driver publishes it on
 * xbrain/{rid}/rt/gnss/fix (11 S3.2): lat/lon/alt + fix_type + accuracy, so
 * p1_motion can fill the fix half of state/pose (fix_type / lat / lon / cov_h_m)
 * and the speed gate can apply the U34/B6 quality tiers.
 *
 * fix_type is the 11 S4.5 closed set (no_fix|single|dgps|rtk_float|rtk_fixed),
 * mapped from the raw GGA quality field. cov_h_m MUST reflect solution quality,
 * never a fixed value (NAV-02): here it is nominal_sigma(fix_type) scaled by HDOP,
 * so it moves with both the solution class and the geometry. The exact per-
 * solution sigma comes from a GST/BESTPOS sentence the module does not emit by
 * default -- parsing it is the T7 refinement; the HDOP estimate is the honest
 * interim, not a constant.
 *
 * The looks-right-but-wrong case: serialising lat/lon as 0.0 when there is no
 * position. 0,0 is the Gulf of Guinea, a real coordinate a consumer would plot.
 * When has_position is false, lat/lon/alt/cov serialise as JSON null.
 */
#ifndef SENSOR__GNSS_FIX_H_
#define SENSOR__GNSS_FIX_H_

#include <string>

namespace sensor {

// One GNSS position solution (11 S3.2). Defaults are the safe no-fix shell.
struct GnssFix {
  bool has_position = false;   // false -> lat/lon/alt/cov serialise as null
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  std::string fix_type = "no_fix";  // closed set 11 S4.5
  double hdop = 0.0;
  int sats = 0;
  double cov_h_m = 0.0;        // horizontal 1sigma (m), from nominal x HDOP
  double cov_v_m = 0.0;        // vertical 1sigma (m), estimate ~1.5 x horizontal
  double age_s = 0.0;          // fix freshness age (s), monotonic
  double t_mono = 0.0;         // CLOCK_MONOTONIC seconds (11 S3.2: age uses THIS)
};

// Raw GGA fix-quality field -> the 11 S4.5 closed-set fix_type. 4=RTK fixed,
// 5=RTK float, 2=DGPS, 1=single; 0 (invalid) and 6 (dead-reckoning) are no_fix.
std::string FixTypeFromGgaQuality(int quality);

// True iff fix_type is a member of the 11 S4.5 closed set.
bool FixTypeValid(const std::string& fix_type);

// Serialise the GnssFix data object (11 S3.2). lat/lon/alt/cov are JSON null when
// has_position is false; fix_type is always present.
std::string ToJsonData(const GnssFix& fix);

}  // namespace sensor

#endif  // SENSOR__GNSS_FIX_H_
