/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: clock_status.h
 * Brief: ClockStatus (11 S3.11) -- rtk_driver is the sole sync judge (CLK-A1)
 *
 * Description:
 * rtk_driver is the ONE process allowed to judge clock sync (CLK-A1); every other
 * process copies the last ClockStatus.sync into its envelope ts_sync (CLK-A2).
 * This is the pure, testable judgement: a ChronyReading (parsed from chronyc by
 * the process, since rtk_driver -- and only it -- may read chrony, CLK-A2) plus
 * the config thresholds and the boot's wall-step count go in; a ClockStatus goes
 * out. No I/O here.
 *
 * The priority is strict rtk -> ntp -> rtc, first hit wins, no fusion (11 S1.5.2:
 * fusion hides which source is actually in use, which is exactly what field
 * triage needs to see).
 *
 * ★★★ A deliberate, documented reading of 11 S1.5.2 (surfaced for review):
 * S1.5.2 gives NTP the SAME gate as the PPS rtk source: |offset|<=20 AND
 * ref_age<=5 s. But a chrony NTP source polls every minutes (ORIN: ~1000 s), so
 * its ref_age (age of the last poll sample) is ALWAYS >> 5 s even while the clock
 * is perfectly disciplined (Leap Normal, offset ~ms). The 5 s ref_age is a PPS
 * staleness check (a lost pulse), not a per-second requirement a disciplined NTP
 * clock could meet. So for source=ntp the freshness criterion here is 'chrony is
 * synchronised' (Leap Normal) + offset gate; ref_age is reported but not the sync
 * gate. For source=rtk (PPS) the literal ref_age<=5 s IS applied. This resolves
 * the S1.5.2 'same threshold' tension toward the physically correct check; it is
 * flagged, not silent -- open items: wire PPS from the module (a real rtk source
 * where ref_age<=5 s holds), or ratify this NTP reading in 11 S1.5.2.
 */
#ifndef SENSOR__CLOCK_STATUS_H_
#define SENSOR__CLOCK_STATUS_H_

#include <string>

namespace sensor {

// What the process parses out of `chronyc tracking` and hands to the judge.
// is_pps_refclock: the reference is a PPS/GPS/NMEA refclock (an rtk source).
// is_ntp: the reference is an NTP server. leap_normal: chrony reports synchronised.
struct ChronyReading {
  bool have = false;          // false -> chrony unreachable / not running
  bool is_pps_refclock = false;
  bool is_ntp = false;
  bool leap_normal = false;   // Leap status Normal (chrony considers itself synced)
  double offset_ms = 0.0;     // system offset from the source (signed, + = local fast)
  double rms_ms = 0.0;
  double ref_age_s = 0.0;     // age of chrony's last reference update (mono)
  double utc_ref = 0.0;       // paired UTC reference epoch (s)
};

// Injected thresholds (11 S3.11 config; no code default, 3.1).
struct ClockConfig {
  double offset_threshold_ms;  // 20.0
  double ref_max_age_s;        // 5.0 (applied to PPS rtk; see header for NTP)
  bool rtc_trusted;            // false until a coin cell is confirmed (S1.5.2)
};

// The 11 S3.11 message.
struct ClockStatus {
  bool sync = false;
  std::string source = "none";     // rtk | ntp | rtc | none
  std::string quality = "none";    // precise | coarse | none
  bool has_offset = false;         // false -> offset_ms/rms_ms serialise null (rtc/none)
  double offset_ms = 0.0;
  double rms_ms = 0.0;
  double ref_age_s = 0.0;
  double mono_ref = 0.0;
  double utc_ref = 0.0;
  std::string boot;
  int step_count = 0;
  std::string detail;
};

// The 11 S3.11 judgement (pure). step_count is the boot's wall-step count so far.
ClockStatus JudgeClock(const ChronyReading& r, const ClockConfig& cfg,
                       int step_count, double mono_now_s, const std::string& boot);

// Serialise the ClockStatus data object (11 S3.11). offset/rms are JSON null when
// there is no external reference (has_offset false), never 0.
std::string ToJsonData(const ClockStatus& cs);

}  // namespace sensor

#endif  // SENSOR__CLOCK_STATUS_H_
