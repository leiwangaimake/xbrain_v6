/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: gnss_heading.cc
 * Brief: GnssHeading consistency check + JSON data serialiser (see gnss_heading.h)
 *
 * Description:
 * Implements the two free functions of gnss_heading.h. gnss_heading_consistent()
 * uses the shared closed set kHeadingSource (common/enums, 3.5: no hardcoded
 * literal) and the 11 S3.3 source<->level one-to-one rule. to_json_data() hand-
 * writes the data object -- no JSON library, so this stays linkable on the RT-
 * plane process alongside the emit-text-only common headers. No ROS, no zenoh,
 * no clock: offline-unit-tested with plain g++ like nmea_parser.cc.
 *
 * The one trap: a null cov_rad (L3) MUST serialise as JSON null, never 0.0 -- a
 * zero covariance reads as a perfect heading downstream (NAV-02 over-trust). The
 * opt_num helper is the single place that decides value-or-null.
 */

#include "sensor/gnss_heading.h"

#include "xbrain/enums/closed_sets.h"

#include <cstddef>
#include <cstdio>
#include <string>

namespace sensor {

namespace {

// Fixed 6-decimal JSON number. Six decimals resolve heading (1e-6 rad ~= 6e-5
// deg), covariance, i_heading and t_mono seconds; snprintf with an explicit
// format avoids the locale-dependent default and never throws.
std::string num(double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.6f", v);
  return std::string(buf);
}

// A nullable number: the value, or the literal JSON null (NOT 0 -- see header).
std::string opt_num(const std::optional<double>& v) {
  return v.has_value() ? num(*v) : std::string("null");
}

// A nullable bool: true / false / null.
std::string opt_bool(const std::optional<bool>& v) {
  return v.has_value() ? std::string(*v ? "true" : "false") : std::string("null");
}

}  // namespace

bool gnss_heading_consistent(const GnssHeading& h) {
  namespace e = hachist::xbrain::enums;
  // kHeadingSource is [dual_antenna, cog, none] -> index 0/1/2, so a valid
  // source's index + 1 must equal level (11 S3.3 one-to-one). An out-of-set
  // source returns kNotAMember and fails here rather than being serialised.
  const std::size_t idx = e::index_of(e::kHeadingSource, h.source);
  if (idx == e::kNotAMember) {
    return false;
  }
  return h.level == static_cast<int>(idx) + 1;
}

std::string to_json_data(const GnssHeading& h) {
  std::string o = "{";
  o += "\"heading_rad\":" + num(h.heading_rad);
  o += ",\"heading_true_north_rad\":" + opt_num(h.heading_true_north_rad);
  o += ",\"heading_valid\":" + std::string(h.heading_valid ? "true" : "false");
  o += ",\"source\":\"" + h.source + "\"";
  o += ",\"level\":" + std::to_string(h.level);
  o += ",\"cov_rad\":" + opt_num(h.cov_rad);
  o += ",\"speed_mps\":" + num(h.speed_mps);
  o += ",\"cog_rad\":" + opt_num(h.cog_rad);
  o += ",\"baseline_m\":" + opt_num(h.baseline_m);
  o += ",\"baseline_valid\":" + opt_bool(h.baseline_valid);
  o += ",\"yaw_capable\":" + std::string(h.yaw_capable ? "true" : "false");
  o += ",\"i_heading\":" + num(h.i_heading);
  o += ",\"age_s\":" + num(h.age_s);
  o += ",\"t_mono\":" + num(h.t_mono);
  o += "}";
  return o;
}

}  // namespace sensor
