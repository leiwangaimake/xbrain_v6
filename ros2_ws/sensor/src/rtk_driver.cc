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
 * The GGA fix-quality -> resolver mapping (11 S3.3.3): quality 4/5 (RTK
 * fixed/float) -> fix_is_rtk (COG admissible); 0/1/6 (invalid/single/DR) or a
 * stale GGA -> fix_is_lost; 2 (DGPS) is neither (cannot start COG, is not lost).
 */

#include "sensor/rtk_driver.h"

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
}  // namespace

RtkDriver::RtkDriver(DriverConfig cfg, PublishSink* sink)
    : cfg_(cfg),
      sink_(sink),
      resolver_(cfg.resolver),
      envelope_(cfg.rid, cfg.src, cfg.boot, cfg.sync_timeout_ms),
      heading_key_("xbrain/" + cfg.rid + "/rt/gnss/heading") {}

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
    gga_quality_ = fix.quality;
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
  in.fix_is_rtk = gga_fresh && (gga_quality_ == 4 || gga_quality_ == 5);
  in.fix_is_lost = (!gga_fresh) || gga_quality_ == 0 || gga_quality_ == 1 ||
                   gga_quality_ == 6;
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

void RtkDriver::tick(double now_mono_s, int64_t wall_ms) {
  const HeadingInputs in = buildInputs(now_mono_s);
  const ResolveResult r = resolver_.update(in, now_mono_s);
  // Envelope: mono/ts in ms (11 S3.0). ts is wall (align/log only, CLK-C1).
  const int64_t mono_ms = static_cast<int64_t>(now_mono_s * 1000.0);
  const hachist::xbrain::envelope::StampedEnvelope env =
      envelope_.stamp(wall_ms, mono_ms);
  const std::string payload = WrapEnvelope(env, ToJsonData(r.heading));
  if (sink_ != nullptr) {
    sink_->publish(heading_key_, payload);
  }
  // r.event (rtk_lost / heading_degraded / heading_recovered) is produced here
  // but its full 11 S3.3.4 event message needs P3-owned fields -- follow-up.
}

}  // namespace sensor
