/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: clock_status.cc
 * Brief: ClockStatus judgement + JSON (see clock_status.h)
 *
 * Description:
 * The 11 S3.11 sync judgement, priority rtk -> ntp -> rtc, evaluated against the
 * CURRENTLY selected chrony reference (chrony has one active reference, so the
 * active source IS the highest-priority available one). See the header for the
 * documented NTP ref_age reading. offset/rms serialise JSON null when there is no
 * external reference, so a consumer cannot read a 0 ms offset as a perfect clock.
 */

#include "sensor/clock_status.h"

#include <cmath>
#include <string>

namespace sensor {

namespace {
std::string OptNum(double v, bool present) {
  return present ? std::to_string(v) : std::string("null");
}
}  // namespace

ClockStatus JudgeClock(const ChronyReading& r, const ClockConfig& cfg,
                       int step_count, double mono_now_s, const std::string& boot) {
  ClockStatus cs;
  cs.boot = boot;
  cs.step_count = step_count;
  cs.mono_ref = mono_now_s;
  cs.utc_ref = r.utc_ref;
  cs.ref_age_s = r.ref_age_s;

  // rtk (PPS) and ntp both come from chrony, so they need a reachable chrony.
  // rtc (below) is the hardware RTC, independent of chrony -- so it is checked
  // even when chrony is down, which is exactly when rtc is the fallback.
  if (r.have && r.is_pps_refclock) {
    // rtk (PPS refclock): the literal S1.5.2 gate, offset + ref_age <= 5 s (a lost
    // pulse is a real staleness at 1 Hz).
    cs.has_offset = true;
    cs.offset_ms = r.offset_ms;
    cs.rms_ms = r.rms_ms;
    if (r.leap_normal && std::fabs(r.offset_ms) <= cfg.offset_threshold_ms &&
        r.ref_age_s <= cfg.ref_max_age_s) {
      cs.sync = true;
      cs.source = "rtk";
      cs.quality = "precise";
      cs.detail = "rtk pps locked";
      return cs;
    }
  } else if (r.have && r.is_ntp) {
    // ntp: chrony disciplines continuously, so the freshness criterion is 'chrony
    // synchronised' (leap_normal), NOT ref_age <= 5 s (see header). offset gate
    // still applies.
    cs.has_offset = true;
    cs.offset_ms = r.offset_ms;
    cs.rms_ms = r.rms_ms;
    if (r.leap_normal && std::fabs(r.offset_ms) <= cfg.offset_threshold_ms) {
      cs.sync = true;
      cs.source = "ntp";
      cs.quality = "precise";
      cs.detail = "ntp synced";
      return cs;
    }
  }

  // rtc: no external reference. Trusted only if configured AND the wall clock has
  // not stepped since boot (a step breaks the RTC-continuity assumption, S3.11
  // step_count rule). No offset for rtc.
  if (cfg.rtc_trusted && step_count == 0) {
    cs.sync = true;
    cs.source = "rtc";
    cs.quality = "coarse";
    cs.has_offset = false;
    cs.detail = "rtc, no external reference";
    return cs;
  }

  cs.detail = "source below threshold";
  return cs;  // sync=false, source=none
}

std::string ToJsonData(const ClockStatus& cs) {
  const bool o = cs.has_offset;
  std::string j = "{";
  j += "\"sync\":" + std::string(cs.sync ? "true" : "false");
  j += ",\"source\":\"" + cs.source + "\"";
  j += ",\"quality\":\"" + cs.quality + "\"";
  j += ",\"offset_ms\":" + OptNum(cs.offset_ms, o);
  j += ",\"rms_ms\":" + OptNum(cs.rms_ms, o);
  j += ",\"ref_age_s\":" + std::to_string(cs.ref_age_s);
  j += ",\"mono_ref\":" + std::to_string(cs.mono_ref);
  j += ",\"utc_ref\":" + std::to_string(cs.utc_ref);
  j += ",\"boot\":\"" + cs.boot + "\"";
  j += ",\"step_count\":" + std::to_string(cs.step_count);
  j += ",\"detail\":\"" + cs.detail + "\"";
  j += "}";
  return j;
}

}  // namespace sensor
