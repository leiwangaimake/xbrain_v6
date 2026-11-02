/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: gga_vrs.cc
 * Brief: GGA formatting, great-circle distance, VRS anchor (see gga_vrs.h)
 *
 * Description:
 * Coordinates are formatted from an integer count of 1e-5 arc-minutes, so the
 * rounding that turns 59.999996' into 60.00000' carries into the degrees instead
 * of printing an invalid "60.00000" minutes field. The geoid separation is left
 * empty: this driver does not know it, and a 0 would be a made-up value.
 */

#include "sensor/gga_vrs.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace sensor {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusKm = 6371.0;
constexpr long long kMinScale = 100000;   // 1e-5 arc-minute

// "ddmm.mmmmm" (deg_digits = 2) or "dddmm.mmmmm" (3) for |deg|.
std::string format_angle(double deg, int deg_digits) {
  const long long units = std::llround(std::fabs(deg) * 60.0 * kMinScale);
  const long long whole_deg = units / (60 * kMinScale);
  const long long rem = units % (60 * kMinScale);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%0*lld%02lld.%05lld", deg_digits, whole_deg,
                rem / kMinScale, rem % kMinScale);
  return buf;
}
}  // namespace

int gga_quality_from_fix_type(const std::string& t) {
  if (t == "rtk_fixed") return 4;
  if (t == "rtk_float") return 5;
  if (t == "dgps") return 2;
  if (t == "single") return 1;
  return 0;
}

std::string build_gga(const GgaPosition& p, double utc_s_of_day) {
  const long long cs = std::llround(utc_s_of_day * 100.0) % (86400LL * 100);
  const long long hh = cs / 360000;
  const long long mm = (cs / 6000) % 60;
  const long long ss = (cs / 100) % 60;
  const long long frac = cs % 100;
  char body[160];
  std::snprintf(body, sizeof(body),
                "GPGGA,%02lld%02lld%02lld.%02lld,%s,%c,%s,%c,%d,%02d,%.1f,%.3f,M,,M,,",
                hh, mm, ss, frac, format_angle(p.lat_deg, 2).c_str(),
                p.lat_deg < 0.0 ? 'S' : 'N', format_angle(p.lon_deg, 3).c_str(),
                p.lon_deg < 0.0 ? 'W' : 'E', p.quality, p.sats, p.hdop, p.alt_m);
  unsigned char sum = 0;
  for (const char* c = body; *c != '\0'; ++c) sum ^= static_cast<unsigned char>(*c);
  char tail[8];
  std::snprintf(tail, sizeof(tail), "*%02X\r\n", sum);
  return std::string("$") + body + tail;
}

double great_circle_km(double lat1, double lon1, double lat2, double lon2) {
  const double r = kPi / 180.0;
  const double dlat = (lat2 - lat1) * r;
  const double dlon = (lon2 - lon1) * r;
  const double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(lat1 * r) * std::cos(lat2 * r) * std::sin(dlon / 2) *
                       std::sin(dlon / 2);
  return 2.0 * kEarthRadiusKm * std::asin(std::sqrt(std::fmin(1.0, a)));
}

bool VrsAnchor::reportable(const PositionSample& s, double now_mono_s) const {
  return s.has_position && s.t_mono >= 0.0 && (now_mono_s - s.t_mono) <= max_pos_age_s_;
}

bool VrsAnchor::needsRebase(const GgaPosition& p) const {
  if (!has_anchor_) return false;
  return great_circle_km(anchor_.lat_deg, anchor_.lon_deg, p.lat_deg, p.lon_deg) > rebase_km_;
}

}  // namespace sensor
