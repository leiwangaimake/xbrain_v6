/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtcm_source.h
 * Brief: Per-source validity (RTG-3) and the primary/backup selector (RTG-4/5)
 *
 * Description:
 * SourceHealth answers "is this correction source usable right now". Bytes, or
 * an open TCP connection, do not count: the LAN bridge box keeps the connection up
 * when the base station UART dies (RTB-8), so a source is valid only while it
 * keeps delivering observation messages AND has delivered a station position
 * message (1005/1006) recently. Without the station message the receiver cannot
 * use the observations at all.
 *
 * SourceSelector decides which ONE source feeds the receiver (RTG-4: never two,
 * the receiver restarts its RTK solution when the reference changes):
 *   - primary first;
 *   - primary invalid -> switch away immediately (down fast);
 *   - back to primary only after it has been valid continuously for hold_up_s
 *     (up slow), so a flapping primary does not make the receiver re-converge
 *     on every flap;
 *   - from NONE straight to PRIMARY needs no hold: nothing is in use, sooner is
 *     better.
 * wantBackup() is the cold-standby rule (U-RTCM-2): the backup link is wanted
 * once the primary has been invalid continuously for backup_after_s, while the
 * backup is in use, and for linger_s after returning to the primary FROM the
 * backup. backup_after_s exists because a cold primary is invalid at every
 * start-up until its first station message arrives (1005/1006 come every few
 * seconds); without it every boot would dial the caster -- and the first dial of
 * an unactivated commercial account starts its paid period.
 *
 * Pure: time is passed in (monotonic seconds, CLK-C1); no I/O.
 */

#ifndef SENSOR__RTCM_SOURCE_H_
#define SENSOR__RTCM_SOURCE_H_

#include <cstdint>

#include "sensor/rtcm3_framer.h"

namespace sensor {

// Windows for SourceHealth (configs rtcm.health.*). No defaults (CLAUDE.md 3.1).
struct HealthConfig {
  double obs_window_s;       // an observation message within this window
  double station_window_s;   // a 1005/1006 within this window
};

class SourceHealth {
 public:
  explicit SourceHealth(HealthConfig cfg) : cfg_(cfg) {}

  // Account one CRC-valid frame received at now_mono_s.
  void onFrame(const Rtcm3Frame& f, double now_mono_s);

  // Usable now: recent observations AND a recent station position message.
  bool valid(double now_mono_s) const;

  // Forget arrival times (connection lost): invalid immediately, not after the
  // window runs out. Counters and the last station id are kept for the logs.
  void reset();

  double lastObsMono() const { return last_obs_mono_; }
  double lastStationMono() const { return last_station_mono_; }
  int stationId() const { return station_id_; }
  uint64_t stationIdChanges() const { return station_id_changes_; }
  uint64_t frames() const { return frames_; }

 private:
  HealthConfig cfg_;
  double last_obs_mono_ = -1.0;       // -1 = never
  double last_station_mono_ = -1.0;
  int station_id_ = -1;
  uint64_t station_id_changes_ = 0;
  uint64_t frames_ = 0;
};

enum class ActiveSource { kNone, kPrimary, kBackup };

const char* active_source_name(ActiveSource s);

// Selector timings (configs rtcm.selector.hold_up_s / rtcm.ntrip.linger_s).
struct SelectorConfig {
  double hold_up_s;       // primary valid this long, continuously, before switching back
  double linger_s;        // cold standby: keep the backup link this long after returning
  double backup_after_s;  // cold standby: primary invalid this long before dialling backup
};

struct SelectorResult {
  ActiveSource active;
  bool switched;      // active changed in this update -> flush queued frames
};

class SourceSelector {
 public:
  explicit SourceSelector(SelectorConfig cfg) : cfg_(cfg) {}

  SelectorResult update(bool primary_valid, bool backup_valid, double now_mono_s);

  // Cold standby: should the backup link be up now?
  bool wantBackup(double now_mono_s) const;

  ActiveSource active() const { return active_; }

 private:
  SelectorConfig cfg_;
  ActiveSource active_ = ActiveSource::kNone;
  double primary_valid_since_ = -1.0;    // start of the current valid run, -1 = not valid
  double primary_invalid_since_ = -1.0;  // start of the current invalid run, -1 = valid / no update yet
  double back_to_primary_ = -1.0;        // when BACKUP -> PRIMARY last happened, -1 = never
};

}  // namespace sensor

#endif  // SENSOR__RTCM_SOURCE_H_
