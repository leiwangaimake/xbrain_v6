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

#include <stdexcept>

#include "sensor/rtcm3_framer.h"
#include "xbrain/config/yaml_lite.h"

namespace sensor {

namespace {
// deg -> rad. Local constant so the file needs no M_PI (non-standard in C++17).
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}  // namespace

RtkConfig load_rtk_config(const std::string& path, const std::string& rid,
                        const std::string& src, const std::string& boot) {
  const xbrain::config::YamlNode cfg = xbrain::config::load_yaml_file(path);
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

  // rt/gnss/fix cov_h_m derivation nominals (11 S3.2, T7-refinable).
  FixCovConfig& fc = out.driver.fix_cov;
  fc.rtk_fixed_h_m = cfg.require_double("fix_cov.rtk_fixed_h_m");
  fc.rtk_float_h_m = cfg.require_double("fix_cov.rtk_float_h_m");
  fc.dgps_h_m = cfg.require_double("fix_cov.dgps_h_m");
  fc.single_h_m = cfg.require_double("fix_cov.single_h_m");
  fc.vertical_factor = cfg.require_double("fix_cov.vertical_factor");

  // fix_type cap on stale corrections (11 S3.2A RTG-10). The float step must come
  // before the dgps step, or rtk_fixed would skip straight to dgps.
  DiffAgeConfig& da = out.driver.diff_age;
  da.fixed_to_float_s = cfg.require_double("diff_age.fixed_to_float_s");
  da.to_dgps_s = cfg.require_double("diff_age.to_dgps_s");
  if (!(da.fixed_to_float_s > 0.0 && da.fixed_to_float_s < da.to_dgps_s)) {
    throw std::runtime_error("diff_age: need 0 < fixed_to_float_s < to_dgps_s");
  }

  // rt/clock/status sync-judgement thresholds (11 S3.11).
  ClockConfig& ck = out.driver.clock;
  ck.offset_threshold_ms = cfg.require_double("clock.offset_threshold_ms");
  ck.ref_max_age_s = cfg.require_double("clock.ref_max_age_s");
  ck.rtc_trusted = cfg.require_bool("clock.rtc_trusted");

  // RTCM differential input (11 S3.2A).
  out.sink.queue_max_bytes = static_cast<std::size_t>(cfg.require_int("rtcm.sink.queue_max_bytes"));
  out.sink.frame_max_age_s = cfg.require_double("rtcm.sink.frame_max_age_s");
  if (out.sink.queue_max_bytes < Rtcm3Framer::kMaxFrame) {
    throw std::runtime_error("rtcm.sink.queue_max_bytes must hold one maximum RTCM frame (>= " +
                             std::to_string(Rtcm3Framer::kMaxFrame) + ")");
  }
  CorrectionConfig& cc = out.corrections;
  cc = CorrectionConfig{};
  cc.queue_max_bytes = out.sink.queue_max_bytes;
  cc.health.obs_window_s = cfg.require_double("rtcm.health.obs_window_s");
  cc.health.station_window_s = cfg.require_double("rtcm.health.station_window_s");
  cc.selector.hold_up_s = cfg.require_double("rtcm.selector.hold_up_s");
  cc.selector.backup_after_s = cfg.require_double("rtcm.selector.backup_after_s");

  LanSourceConfig& lan = cc.lan;
  lan.enabled = cfg.require_bool("rtcm.lan.enabled");
  if (lan.enabled) {
    lan.host = cfg.require_string("rtcm.lan.host");
    lan.port = static_cast<int>(cfg.require_int("rtcm.lan.port"));
    lan.connect_timeout_s = cfg.require_double("rtcm.lan.connect_timeout_s");
    lan.read_timeout_s = cfg.require_double("rtcm.lan.read_timeout_s");
    lan.backoff_min_s = cfg.require_double("rtcm.lan.backoff_min_s");
    lan.backoff_max_s = cfg.require_double("rtcm.lan.backoff_max_s");
  }

  NtripSourceConfig& nt = cc.ntrip;
  nt.enabled = cfg.require_bool("rtcm.ntrip.enabled");
  if (nt.enabled) {
    nt.host = cfg.require_string("rtcm.ntrip.host");
    nt.port = static_cast<int>(cfg.require_int("rtcm.ntrip.port"));
    nt.mountpoint = cfg.require_string("rtcm.ntrip.mountpoint");
    nt.version = static_cast<int>(cfg.require_int("rtcm.ntrip.version"));
    if (nt.version != 1 && nt.version != 2) {
      throw std::runtime_error("rtcm.ntrip.version must be 1 or 2");
    }
    nt.user = cfg.require_string("rtcm.ntrip.user");
    nt.password = cfg.require_string("rtcm.ntrip.password");
    nt.connect_timeout_s = cfg.require_double("rtcm.ntrip.connect_timeout_s");
    nt.read_timeout_s = cfg.require_double("rtcm.ntrip.read_timeout_s");
    nt.backoff_min_s = cfg.require_double("rtcm.ntrip.backoff_min_s");
    nt.backoff_max_s = cfg.require_double("rtcm.ntrip.backoff_max_s");
    nt.backoff_auth_s = cfg.require_double("rtcm.ntrip.backoff_auth_s");
    nt.gga_period_s = cfg.require_double("rtcm.ntrip.gga_period_s");
    nt.gga_max_pos_age_s = cfg.require_double("rtcm.ntrip.gga_max_pos_age_s");
    nt.vrs_rebase_km = cfg.require_double("rtcm.ntrip.vrs_rebase_km");
    cc.selector.linger_s = cfg.require_double("rtcm.ntrip.linger_s");
  }

  return out;
}

}  // namespace sensor
