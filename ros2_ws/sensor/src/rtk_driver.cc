/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtk_driver.cc
 * Brief: rtk_driver core wiring (see rtk_driver.h)
 *
 * Description:
 * Accumulates serial bytes into NMEA lines, tracks the latest GGA/TRA/RMC facts
 * with their monotonic arrival time, and each tick maps them into the resolver's
 * HeadingInputs, runs the L1/L2/L3 state machine, stamps the 11 S3.0 envelope,
 * and publishes xbrain/{rid}/rt/gnss/heading. No serial I/O, no zenoh, no clock
 * read -- feed()/tick() take bytes and time as parameters.
 *
 * The GGA fix-quality -> resolver mapping (11 S3.3.3; 2026-08-16 user ruling:
 * DGPS is admitted to COG). A fix supports COG heading iff it is NOT lost --
 * quality in {4,5,2} = {rtk_fixed, rtk_float, dgps}. quality 0/1/6
 * (invalid/single/DR) or a stale GGA -> fix_is_lost = no usable heading. So the
 * COG gate is the SINGLE predicate !fix_is_lost; there is no separate
 * fix_is_rtk (the old {4,5}-only split wrongly kept DGPS out of COG, so DGPS
 * could enter L2 via L1-degrade yet never recover L3->L2 -- an asymmetry).
 */

#include "sensor/rtk_driver.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace sensor {

namespace {
constexpr std::size_t kMaxRx = 8192;  // drop runaway garbage, keep the last lines

// Serialise the 11 S3.0 envelope around a data object. The envelope fields come
// from EnvelopeWriter (v/rid/ts/mono/boot/seq/src/ts_sync); data is the caller's.
std::string WrapEnvelope(const hachist::xbrain::envelope::StampedEnvelope& e,
                         const std::string& data_json) {
  std::string o = "{";
  o += "\"v\":" + std::to_string(e.v);
  o += ",\"rid\":\"" + e.rid + "\"";
  o += ",\"ts\":" + std::to_string(e.ts);
  o += ",\"mono\":" + std::to_string(e.mono);
  o += ",\"boot\":\"" + e.boot + "\"";
  o += ",\"seq\":" + std::to_string(e.seq);
  o += ",\"src\":\"" + e.src + "\"";
  o += ",\"ts_sync\":" + std::string(e.ts_sync ? "true" : "false");
  o += ",\"data\":" + data_json;
  o += "}";
  return o;
}

// Per-fix_type nominal horizontal sigma from config. no_fix never reaches here
// (it has no position); single is the widest usable class.
double NominalCovH(const std::string& fix_type, const FixCovConfig& c) {
  if (fix_type == "rtk_fixed") return c.rtk_fixed_h_m;
  if (fix_type == "rtk_float") return c.rtk_float_h_m;
  if (fix_type == "dgps") return c.dgps_h_m;
  return c.single_h_m;
}
}  // namespace

RtkDriver::RtkDriver(DriverConfig cfg, PublishSink* sink)
    : cfg_(cfg),
      sink_(sink),
      resolver_(cfg.resolver),
      envelope_(cfg.rid, cfg.src, cfg.boot, cfg.sync_timeout_ms),
      envelope_fix_(cfg.rid, cfg.src, cfg.boot, cfg.sync_timeout_ms),
      envelope_clock_(cfg.rid, cfg.src, cfg.boot, cfg.sync_timeout_ms),
      heading_key_("xbrain/" + cfg.rid + "/rt/gnss/heading"),
      fix_key_("xbrain/" + cfg.rid + "/rt/gnss/fix"),
      clock_key_("xbrain/" + cfg.rid + "/rt/clock/status") {}

void RtkDriver::feed(const char* data, std::size_t n, double now_mono_s) {
  rx_.append(data, n);
  std::size_t nl;
  while ((nl = rx_.find('\n')) != std::string::npos) {
    processLine(rx_.substr(0, nl), now_mono_s);
    rx_.erase(0, nl + 1);
  }
  if (rx_.size() > kMaxRx) {
    rx_.erase(0, rx_.size() - kMaxRx);
  }
}

void RtkDriver::processLine(const std::string& line, double now_mono_s) {
  if (line.find('$') == std::string::npos) return;
  GgaFix fix;
  if (ParseGga(line, &fix)) {
    gga_ = fix;
    gga_mono_ = now_mono_s;
    return;
  }
  TraHeading tra;
  if (ParseTra(line, &tra)) {
    tra_ = tra;
    tra_mono_ = now_mono_s;
    return;
  }
  RmcData rmc;
  if (ParseRmc(line, &rmc)) {
    rmc_ = rmc;
    rmc_mono_ = now_mono_s;
  }
}

HeadingInputs RtkDriver::buildInputs(double now_mono_s) const {
  HeadingInputs in;
  // Fix: fresh GGA within gga_timeout, mapped to rtk / lost.
  const bool gga_fresh =
      gga_mono_ >= 0.0 && (now_mono_s - gga_mono_) <= cfg_.gga_timeout_s;
  // fix_is_lost = no usable heading: no_fix(0) / single(1) / DR(6) / stale.
  // Its complement {rtk_fixed(4), rtk_float(5), dgps(2)} is exactly the COG-
  // admissible set, so the resolver gates COG on !fix_is_lost (no fix_is_rtk).
  in.fix_is_lost = (!gga_fresh) || gga_.quality == 0 || gga_.quality == 1 ||
                   gga_.quality == 6;
  // Dual-antenna heading (TRA): present iff fresh; baseline fixed iff QF==4.
  in.heading_present =
      tra_mono_ >= 0.0 && (now_mono_s - tra_mono_) <= cfg_.tra_timeout_s;
  in.baseline_valid = in.heading_present && tra_.quality == 4;
  in.heading_true_deg = tra_.heading_true_deg;
  in.heading_cov_rad = cfg_.heading_stddev_rad;   // config sigma (UNIHEADINGA TODO)
  in.heading_age_s = in.heading_present ? (now_mono_s - tra_mono_) : 1.0e9;
  // COG (RMC): only when fresh; speed 0 when stale so L2 admission fails.
  const bool rmc_fresh =
      rmc_mono_ >= 0.0 && (now_mono_s - rmc_mono_) <= cfg_.rmc_timeout_s;
  in.speed_mps = rmc_fresh ? rmc_.speed_mps : 0.0;
  in.cog_true_deg = rmc_.cog_deg;
  in.cog_present = rmc_fresh && rmc_.cog_present;
  // pending_autonomous_motion: the heading_blind_timeout input. rtk_driver does
  // not yet subscribe to the cmd plane, so it is false (safe) -- a wiring seam.
  in.pending_autonomous_motion = false;
  return in;
}

GnssFix RtkDriver::buildFix(double now_mono_s) const {
  GnssFix fix;
  const bool fresh =
      gga_mono_ >= 0.0 && (now_mono_s - gga_mono_) <= cfg_.gga_timeout_s;
  // fix_type follows the raw quality when fresh; a stale GGA is no_fix (T-09).
  fix.fix_type = fresh ? FixTypeFromGgaQuality(gga_.quality) : "no_fix";
  fix.t_mono = now_mono_s;
  fix.age_s = (gga_mono_ >= 0.0) ? (now_mono_s - gga_mono_) : 1.0e9;
  fix.sats = gga_.num_satellites;
  fix.hdop = gga_.hdop;
  // has_position only with a fresh, valid GGA that is an actual fix. Otherwise
  // lat/lon/cov serialise null (NAV-02: no plotting 0,0, no 0 m cov).
  fix.has_position = fresh && gga_.valid && fix.fix_type != "no_fix";
  if (fix.has_position) {
    fix.lat = gga_.latitude_deg;
    fix.lon = gga_.longitude_deg;
    fix.alt = gga_.altitude_m;
    // cov_h_m = nominal(fix_type) x max(hdop, 1): moves with class + geometry,
    // never a constant (NAV-02). GST/BESTPOS exact sigma is the T7 refinement.
    const double hdop_factor = (gga_.hdop > 1.0) ? gga_.hdop : 1.0;
    fix.cov_h_m = NominalCovH(fix.fix_type, cfg_.fix_cov) * hdop_factor;
    fix.cov_v_m = fix.cov_h_m * cfg_.fix_cov.vertical_factor;
  }
  return fix;
}

void RtkDriver::tick(double now_mono_s, int64_t wall_ms) {
  // Envelope: mono/ts in ms (11 S3.0). ts is wall (align/log only, CLK-C1).
  const int64_t mono_ms = static_cast<int64_t>(now_mono_s * 1000.0);
  // rt/gnss/heading (11 S3.3).
  const HeadingInputs in = buildInputs(now_mono_s);
  const ResolveResult r = resolver_.update(in, now_mono_s);
  const hachist::xbrain::envelope::StampedEnvelope env_h =
      envelope_.stamp(wall_ms, mono_ms);
  const std::string hpayload = WrapEnvelope(env_h, ToJsonData(r.heading));
  // rt/gnss/fix (11 S3.2). Own envelope writer -> its own seq for gap detection.
  const GnssFix fix = buildFix(now_mono_s);
  const hachist::xbrain::envelope::StampedEnvelope env_f =
      envelope_fix_.stamp(wall_ms, mono_ms);
  const std::string fpayload = WrapEnvelope(env_f, ToJsonData(fix));
  if (sink_ != nullptr) {
    sink_->publish(heading_key_, hpayload);
    sink_->publish(fix_key_, fpayload);
  }
  // r.event (rtk_lost / heading_degraded / heading_recovered) is produced here
  // but its full 11 S3.3.4 event message needs P3-owned fields -- follow-up.
}

void RtkDriver::tickClock(const ChronyReading& r, double now_mono_s,
                          int64_t wall_ms) {
  // Wall-step detection (CLK-C6): between two 1 Hz clock ticks the wall delta
  // should track the monotonic delta. A disagreement beyond 500 ms is a step
  // (an authoritative source corrected the wall clock). First tick sets a base.
  if (last_clock_mono_s_ >= 0.0) {
    const double mono_delta_ms = (now_mono_s - last_clock_mono_s_) * 1000.0;
    const double wall_delta_ms = static_cast<double>(wall_ms - last_clock_wall_ms_);
    if (std::fabs(wall_delta_ms - mono_delta_ms) > 500.0) {
      ++step_count_;
    }
  }
  last_clock_mono_s_ = now_mono_s;
  last_clock_wall_ms_ = wall_ms;

  const ClockStatus cs = JudgeClock(r, cfg_.clock, step_count_, now_mono_s, cfg_.boot);
  const int64_t mono_ms = static_cast<int64_t>(now_mono_s * 1000.0);
  const hachist::xbrain::envelope::StampedEnvelope env =
      envelope_clock_.stamp(wall_ms, mono_ms);
  const std::string payload = WrapEnvelope(env, ToJsonData(cs));
  if (sink_ != nullptr) {
    sink_->publish(clock_key_, payload);
  }
}

}  // namespace sensor
