/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: gga_vrs.h
 * Brief: GGA uplink for a VRS caster + the anchor that keeps it location-free
 *
 * Description:
 * A VRS caster builds a virtual reference station near the position the rover
 * reports in GGA. The 2026-11-02 requirement (11 S3.2A RTG-7 / RTG-12) is that
 * the commercial backup works anywhere -- tested in Shanghai today, Nanjing
 * next week, the same config -- so:
 *   - the reported position is the robot's own measured fix and nothing else:
 *     no configured site coordinate, no position cached before a power-off
 *     (the robot may have been trucked to another city). There is deliberately
 *     no API here that accepts a substitute;
 *   - a fix older than max_pos_age_s is not reportable: the caller waits;
 *   - the first reported position becomes the VRS anchor, and moving more than
 *     rebase_km away from it asks for a reconnect so the caster builds a new
 *     virtual station where the robot now is.
 *
 * Pure: no I/O, no clock. The UTC time-of-day for the GGA is passed in.
 */

#ifndef SENSOR__GGA_VRS_H_
#define SENSOR__GGA_VRS_H_

#include <string>

namespace sensor {

struct GgaPosition {
  double lat_deg = 0.0;   // + north
  double lon_deg = 0.0;   // + east
  double alt_m = 0.0;     // above mean sea level
  int quality = 0;        // GGA fix-quality digit
  int sats = 0;
  double hdop = 0.0;
};

// The robot's latest fix as seen by the network thread.
struct PositionSample {
  bool has_position = false;
  GgaPosition pos;
  double t_mono = -1.0;        // when the GGA behind it arrived (CLOCK_MONOTONIC s)
  bool has_utc = false;        // the receiver's GGA carried a time field
  double utc_s_of_day = 0.0;   // that time, seconds since UTC midnight
};

// 11 S4.5 fix_type -> GGA quality digit (rtk_fixed 4, rtk_float 5, dgps 2,
// single 1, anything else 0).
int gga_quality_from_fix_type(const std::string& fix_type);

// "$GPGGA,...*CS\r\n". utc_s_of_day is seconds since UTC midnight.
std::string build_gga(const GgaPosition& p, double utc_s_of_day);

// Great-circle distance on a 6371 km sphere (haversine).
double great_circle_km(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);

class VrsAnchor {
 public:
  VrsAnchor(double max_pos_age_s, double rebase_km)
      : max_pos_age_s_(max_pos_age_s), rebase_km_(rebase_km) {}

  // A measured fix, fresh enough to tell the caster where we are, with the
  // receiver's own UTC time (the GGA time field is taken from it, not from the
  // host wall clock).
  bool reportable(const PositionSample& s, double now_mono_s) const;

  void setAnchor(const GgaPosition& p) {
    anchor_ = p;
    has_anchor_ = true;
  }
  void clear() { has_anchor_ = false; }
  bool hasAnchor() const { return has_anchor_; }

  // Moved far enough from the anchor that the virtual station should be rebuilt.
  bool needsRebase(const GgaPosition& p) const;

 private:
  double max_pos_age_s_;
  double rebase_km_;
  bool has_anchor_ = false;
  GgaPosition anchor_;
};

}  // namespace sensor

#endif  // SENSOR__GGA_VRS_H_
