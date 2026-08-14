/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtk_config.cc
 * Brief: rtk_driver config loader (see rtk_config.h)
 *
 * Description:
 * Reads the resolved yaml with yaml_lite and pulls each field with a require_*
 * accessor. The ordering is deliberate: identity first, then the driver-level
 * timeouts, then the resolver block. Any require_* throws with its dotted key
 * path, so the first uncalibrated null names itself and the process stops (3.1).
 */

#include "sensor/rtk_config.h"

#include "xbrain/config/yaml_lite.h"

namespace sensor {

namespace {
// deg -> rad. Local constant so the file needs no M_PI (non-standard in C++17).
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}  // namespace

RtkConfig LoadRtkConfig(const std::string& path, const std::string& rid,
                        const std::string& src, const std::string& boot) {
  const xbrain::config::YamlNode cfg = xbrain::config::LoadYamlFile(path);
  RtkConfig out;

  // Identity is injected by the caller (not in the file, see header).
  out.driver.rid = rid;
  out.driver.src = src;
  out.driver.boot = boot;

  // Serial I/O params.
  out.serial_port = cfg.require_string("serial.port");
  out.serial_baud = static_cast<int>(cfg.require_int("serial.baudrate"));

  // Driver-level timeouts + heading sigma (stored in degrees, kept in radians).
  out.driver.sync_timeout_ms = cfg.require_int("sync_timeout_ms");
  out.driver.heading_stddev_rad = cfg.require_double("heading_stddev_deg") * kDegToRad;
  out.driver.gga_timeout_s = cfg.require_double("gga_timeout_s");
  out.driver.tra_timeout_s = cfg.require_double("tra_timeout_s");
  out.driver.rmc_timeout_s = cfg.require_double("rmc_timeout_s");

  // Resolver L1/L2/L3 thresholds (11 S3.3.1/S3.3.3).
  ResolverConfig& r = out.driver.resolver;
  r.cov_thresh_rad = cfg.require_double("resolver.cov_thresh_rad");
  r.age_thresh_s = cfg.require_double("resolver.age_thresh_s");
  r.cog_speed_thresh_mps = cfg.require_double("resolver.cog_speed_thresh_mps");
  r.degrade_sustain_s = cfg.require_double("resolver.degrade_sustain_s");
  r.recover_sustain_s = cfg.require_double("resolver.recover_sustain_s");
  r.fix_lost_sustain_s = cfg.require_double("resolver.fix_lost_sustain_s");
  r.blind_rise_sustain_s = cfg.require_double("resolver.blind_rise_sustain_s");
  r.blind_timeout_s = cfg.require_double("resolver.blind_timeout_s");
  r.cov_h_m = cfg.require_double("resolver.cov_h_m");
  r.cog_diff_dt_s = cfg.require_double("resolver.cog_diff_dt_s");
  r.i_heading_l1 = cfg.require_double("resolver.i_heading_l1");
  r.i_heading_l2 = cfg.require_double("resolver.i_heading_l2");
  r.i_heading_l3 = cfg.require_double("resolver.i_heading_l3");

  return out;
}

}  // namespace sensor
