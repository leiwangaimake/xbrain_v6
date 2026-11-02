/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtcm_source.cc
 * Brief: SourceHealth + SourceSelector (see rtcm_source.h)
 *
 * Description:
 * The selector keeps the start of the primary's current valid run rather than a
 * counter of good updates, so hold_up_s means seconds whatever the update rate,
 * and one invalid update anywhere in the run restarts it.
 */

#include "sensor/rtcm_source.h"

namespace sensor {

void SourceHealth::onFrame(const Rtcm3Frame& f, double now_mono_s) {
  ++frames_;
  if (rtcm_is_observation(f.msg_type)) last_obs_mono_ = now_mono_s;
  if (rtcm_is_station(f.msg_type)) last_station_mono_ = now_mono_s;
  if (f.station_id >= 0) {
    if (station_id_ >= 0 && f.station_id != station_id_) ++station_id_changes_;
    station_id_ = f.station_id;
  }
}

bool SourceHealth::valid(double now_mono_s) const {
  if (last_obs_mono_ < 0.0 || last_station_mono_ < 0.0) return false;
  return (now_mono_s - last_obs_mono_) <= cfg_.obs_window_s &&
         (now_mono_s - last_station_mono_) <= cfg_.station_window_s;
}

void SourceHealth::reset() {
  last_obs_mono_ = -1.0;
  last_station_mono_ = -1.0;
}

const char* active_source_name(ActiveSource s) {
  switch (s) {
    case ActiveSource::kPrimary: return "lan";
    case ActiveSource::kBackup: return "ntrip";
    default: return "none";
  }
}

SelectorResult SourceSelector::update(bool primary_valid, bool backup_valid,
                                      double now_mono_s) {
  if (!primary_valid) {
    primary_valid_since_ = -1.0;
    if (primary_invalid_since_ < 0.0) primary_invalid_since_ = now_mono_s;
  } else {
    primary_invalid_since_ = -1.0;
    if (primary_valid_since_ < 0.0) primary_valid_since_ = now_mono_s;
  }
  const bool primary_held =
      primary_valid && (now_mono_s - primary_valid_since_) >= cfg_.hold_up_s;

  ActiveSource next = active_;
  switch (active_) {
    case ActiveSource::kNone:
      if (primary_valid) {
        next = ActiveSource::kPrimary;            // nothing in use: no hold
      } else if (backup_valid) {
        next = ActiveSource::kBackup;
      }
      break;
    case ActiveSource::kPrimary:
      if (!primary_valid) {                       // down fast
        next = backup_valid ? ActiveSource::kBackup : ActiveSource::kNone;
      }
      break;
    case ActiveSource::kBackup:
      if (primary_held) {                         // up slow
        next = ActiveSource::kPrimary;
      } else if (!backup_valid) {
        next = primary_valid ? ActiveSource::kPrimary : ActiveSource::kNone;
      }
      break;
  }
  const bool switched = next != active_;
  if (switched && next == ActiveSource::kPrimary) {
    // Linger only after leaving the backup; NONE -> PRIMARY never used it.
    back_to_primary_ = (active_ == ActiveSource::kBackup) ? now_mono_s : -1.0;
  }
  active_ = next;
  return {active_, switched};
}

bool SourceSelector::wantBackup(double now_mono_s) const {
  if (active_ == ActiveSource::kBackup) return true;
  if (primary_invalid_since_ >= 0.0 &&
      (now_mono_s - primary_invalid_since_) >= cfg_.backup_after_s) {
    return true;
  }
  return active_ == ActiveSource::kPrimary && back_to_primary_ >= 0.0 &&
         (now_mono_s - back_to_primary_) < cfg_.linger_s;
}

}  // namespace sensor
