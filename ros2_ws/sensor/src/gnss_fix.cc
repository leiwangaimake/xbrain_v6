/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: gnss_fix.cc
 * Brief: GnssFix mapping + JSON (see gnss_fix.h)
 *
 * Description:
 * The GGA-quality -> fix_type table and the null-safe serialiser. opt_num emits
 * JSON null (not 0.0) for the position/cov fields when there is no fix, so a
 * consumer cannot plot 0,0 or read a 0 m covariance as a perfect fix (NAV-02).
 * The same rule covers age_s: without corrections there is no differential age,
 * and a 0 would read as "corrections fresh this instant".
 * Numbers use a fixed precision via snprintf, not std::to_string (always %f, 6
 * places): lat/lon get 9 places (1e-9 deg ~= 0.1 mm). Six places of a degree is
 * ~0.11 m and threw the RTK fixed centimetres away on the bus (11 S3.2, 2026-11-03
 * field test).
 */

#include "sensor/gnss_fix.h"

#include <array>
#include <cstdio>
#include <string>

namespace sensor {

namespace {
constexpr int kDegDecimals = 9;    // lat / lon (11 S3.2)
constexpr int kDecimals = 6;       // every other float field

// Emit a number with `decimals` places, or the literal null when this field is
// absent (no position).
std::string opt_num(double v, bool present, int decimals) {
  if (!present) return "null";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
  return std::string(buf);
}

const std::array<const char*, 5> kFixTypes = {
    "no_fix", "single", "dgps", "rtk_float", "rtk_fixed"};
}  // namespace

std::string fix_type_from_gga_quality(int quality) {
  switch (quality) {
    case 1: return "single";
    case 2: return "dgps";
    case 4: return "rtk_fixed";
    case 5: return "rtk_float";
    // 0 (invalid) and 6 (dead-reckoning) are not a GNSS position fix.
    default: return "no_fix";
  }
}

bool fix_type_valid(const std::string& fix_type) {
  for (const char* t : kFixTypes) {
    if (fix_type == t) return true;
  }
  return false;
}

std::string to_json_data(const GnssFix& fix) {
  const bool p = fix.has_position;
  std::string o = "{";
  o += "\"lat\":" + opt_num(fix.lat, p, kDegDecimals);
  o += ",\"lon\":" + opt_num(fix.lon, p, kDegDecimals);
  o += ",\"alt\":" + opt_num(fix.alt, p, kDecimals);
  o += ",\"fix_type\":\"" + fix.fix_type + "\"";
  o += ",\"hdop\":" + opt_num(fix.hdop, p, kDecimals);
  o += ",\"sats\":" + std::to_string(fix.sats);
  o += ",\"cov_h_m\":" + opt_num(fix.cov_h_m, p, kDecimals);
  o += ",\"cov_v_m\":" + opt_num(fix.cov_v_m, p, kDecimals);
  o += ",\"age_s\":" + opt_num(fix.age_s, fix.has_age, kDecimals);
  o += ",\"t_mono\":" + std::to_string(fix.t_mono);
  o += "}";
  return o;
}

}  // namespace sensor
