/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: gnss_fix.cc
 * Brief: GnssFix mapping + JSON (see gnss_fix.h)
 *
 * Description:
 * The GGA-quality -> fix_type table and the null-safe serialiser. OptNum emits
 * JSON null (not 0.0) for the position/cov fields when there is no fix, so a
 * consumer cannot plot 0,0 or read a 0 m covariance as a perfect fix (NAV-02).
 */

#include "sensor/gnss_fix.h"

#include <array>
#include <string>

namespace sensor {

namespace {
// Emit a number, or the literal null when this field is absent (no position).
std::string OptNum(double v, bool present) {
  if (!present) return "null";
  return std::to_string(v);
}

const std::array<const char*, 5> kFixTypes = {
    "no_fix", "single", "dgps", "rtk_float", "rtk_fixed"};
}  // namespace

std::string FixTypeFromGgaQuality(int quality) {
  switch (quality) {
    case 1: return "single";
    case 2: return "dgps";
    case 4: return "rtk_fixed";
    case 5: return "rtk_float";
    // 0 (invalid) and 6 (dead-reckoning) are not a GNSS position fix.
    default: return "no_fix";
  }
}

bool FixTypeValid(const std::string& fix_type) {
  for (const char* t : kFixTypes) {
    if (fix_type == t) return true;
  }
  return false;
}

std::string ToJsonData(const GnssFix& fix) {
  const bool p = fix.has_position;
  std::string o = "{";
  o += "\"lat\":" + OptNum(fix.lat, p);
  o += ",\"lon\":" + OptNum(fix.lon, p);
  o += ",\"alt\":" + OptNum(fix.alt, p);
  o += ",\"fix_type\":\"" + fix.fix_type + "\"";
  o += ",\"hdop\":" + OptNum(fix.hdop, p);
  o += ",\"sats\":" + std::to_string(fix.sats);
  o += ",\"cov_h_m\":" + OptNum(fix.cov_h_m, p);
  o += ",\"cov_v_m\":" + OptNum(fix.cov_v_m, p);
  o += ",\"age_s\":" + std::to_string(fix.age_s);
  o += ",\"t_mono\":" + std::to_string(fix.t_mono);
  o += "}";
  return o;
}

}  // namespace sensor
