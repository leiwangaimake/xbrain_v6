/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtk_driver.h
 * Brief: rtk_driver core: NMEA facts -> resolver -> GnssHeading -> RT-plane publish
 *
 * Description:
 * The problem this solves. This is the wiring between the pieces: it accumulates
 * serial bytes into NMEA lines, keeps the latest GGA / TRA / RMC facts with their
 * monotonic arrival time, and once per tick builds the resolver's HeadingInputs,
 * runs the L1/L2/L3 state machine, stamps the 11 S3.0 envelope, and publishes
 * xbrain/{rid}/rt/gnss/heading. It is deliberately split from I/O: feed() takes
 * bytes + a monotonic time, tick() takes a monotonic + wall time, so the whole
 * driver is unit-tested by feeding synthetic sentences and injected time into a
 * capturing PublishSink -- no serial port, no zenoh, no clock read.
 *
 * Which sections this follows: 11 S3.3 (GnssHeading + resolver), S3.0 (envelope,
 * via common/envelope/envelope_writer.h), S1.1.7 (keyexpr xbrain/{rid}/rt/...),
 * CLK-C1 (all time monotonic seconds/ms, passed in).
 *
 * What it does NOT do yet, and why (each a scoped follow-up, not a silent gap):
 *   * rt/clock/status (ClockStatus, CLK-A1): rtk_driver is the sole sync judge,
 *     but the 11 S3.11 judgement needs chronyd state + wall-step detection -- a
 *     separate sub-feature. Tracked as the next batch; the envelope's ts_sync is
 *     wired through EnvelopeWriter (CLK-A2 copy of its OWN ClockStatus once that
 *     lands) and is false until then (fail-safe, CLK-A3).
 *   * rt/gnss/fix (position from GGA) -- a straightforward add on the same path.
 *   * the rtk_lost / heading_degraded / heading_recovered EVENT (11 S3.3.4): the
 *     resolver already returns which event fired; publishing the full event needs
 *     fields this process does not own (task_id / task_progress from P3), so the
 *     event is surfaced by the resolver here but not yet emitted.
 *   * the L1 heading covariance is the injected config stddev, not the module's
 *     per-solution covariance (UNIHEADINGA is not parsed yet) -- a known
 *     approximation, noted so it is not mistaken for the real baseline sigma.
 *
 * Boundary: no rclcpp, no zenoh, no clock read. Serial I/O + the 20 Hz loop are a
 * thin main() (a later batch); this class is the testable core.
 */

#ifndef SENSOR__RTK_DRIVER_H_
#define SENSOR__RTK_DRIVER_H_

#include <cstdint>
#include <string>

#include "sensor/clock_status.h"
#include "sensor/gnss_fix.h"
#include "sensor/heading_resolver.h"
#include "sensor/nmea_parser.h"
#include "sensor/publish_sink.h"
#include "xbrain/envelope/envelope_writer.h"

namespace sensor {

// cov_h_m derivation (11 S3.2 / NAV-02): cov_h_m = nominal_sigma(fix_type) x
// max(hdop, 1). The nominals are per solution class (T7-refinable; GST gives the
// exact per-solution sigma later). Injected, no code default (3.1).
struct FixCovConfig {
  double rtk_fixed_h_m;   // ~0.02
  double rtk_float_h_m;   // ~0.30
  double dgps_h_m;        // ~1.50
  double single_h_m;      // ~3.00
  double vertical_factor; // cov_v_m = cov_h_m x this (~1.5)
};

// Injected identity + freshness thresholds. NO defaults (CLAUDE.md 3.1): the
// driver refuses to invent a robot id, a boot id, or a staleness window. Filled
// from configs/ by main().
struct DriverConfig {
  std::string rid;              // robot id (envelope rid + keyexpr segment)
  std::string src;              // this process's source id ("rtk_driver")
  std::string boot;             // boot id, first 8 hex (envelope boot)
  int64_t sync_timeout_ms;      // CLK-A3 window for the envelope ts_sync (5000)
  double heading_stddev_rad;    // L1 heading 1sigma (config; UNIHEADINGA TODO)
  double gga_timeout_s;         // a GGA older than this counts as fix stale
  double tra_timeout_s;         // a TRA older than this -> no dual-antenna heading
  double rmc_timeout_s;         // an RMC older than this -> no COG
  ResolverConfig resolver;      // the L1/L2/L3 thresholds
  FixCovConfig fix_cov;         // cov_h_m derivation for rt/gnss/fix (11 S3.2)
  ClockConfig clock;            // sync-judgement thresholds for rt/clock/status (11 S3.11)
};

// The driver core. Owns the resolver + envelope writer; borrows a PublishSink.
class RtkDriver {
 public:
  RtkDriver(DriverConfig cfg, PublishSink* sink);

  // Accumulate serial bytes and parse any complete NMEA lines, timestamping each
  // parsed fact with now_mono_s (its arrival time, for age judgements).
  void feed(const char* data, std::size_t n, double now_mono_s);

  // Build the resolver inputs from the latest facts, run one resolve step, and
  // publish xbrain/{rid}/rt/gnss/heading + rt/gnss/fix. wall_ms -> envelope ts.
  void tick(double now_mono_s, int64_t wall_ms);

  // Judge sync from the chrony reading (CLK-A1), track wall-step count, and
  // publish xbrain/{rid}/rt/clock/status. Called at 1 Hz by main() (the chrony
  // read is I/O, kept out of this testable core -- 11 S3.11 CLK-A2: only
  // rtk_driver reads chrony, and it does so in the process, not this class).
  void tickClock(const ChronyReading& r, double now_mono_s, int64_t wall_ms);

  int level() const { return resolver_.level(); }

 private:
  DriverConfig cfg_;
  PublishSink* sink_;
  HeadingResolver resolver_;
  hachist::xbrain::envelope::EnvelopeWriter envelope_;
  hachist::xbrain::envelope::EnvelopeWriter envelope_fix_;    // own seq for the fix topic
  hachist::xbrain::envelope::EnvelopeWriter envelope_clock_;  // own seq for the clock topic
  std::string rx_;                    // partial-line accumulator
  std::string heading_key_;           // precomputed xbrain/{rid}/rt/gnss/heading
  std::string fix_key_;               // precomputed xbrain/{rid}/rt/gnss/fix
  std::string clock_key_;             // precomputed xbrain/{rid}/rt/clock/status

  // Wall-step detection for ClockStatus.step_count: a wall delta that disagrees
  // with the monotonic delta by more than a threshold is a clock step (CLK-C6).
  int step_count_ = 0;
  double last_clock_mono_s_ = -1.0;
  int64_t last_clock_wall_ms_ = 0;

  // Latest parsed facts + the monotonic time each arrived (-1 = never).
  GgaFix gga_{};
  double gga_mono_ = -1.0;
  TraHeading tra_{};
  double tra_mono_ = -1.0;
  RmcData rmc_{};
  double rmc_mono_ = -1.0;

  void processLine(const std::string& line, double now_mono_s);
  HeadingInputs buildInputs(double now_mono_s) const;
  GnssFix buildFix(double now_mono_s) const;
};

}  // namespace sensor

#endif  // SENSOR__RTK_DRIVER_H_
