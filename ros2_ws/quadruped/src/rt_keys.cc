/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_keys.cc
 * Brief: The key table itself, plus allocation-free composition
 *
 * Description:
 * The table below is transcribed from 13 S7.1 (the four chassis report streams
 * plus the two aggregates) and 13 S7.1.1 (Q-1 to Q-5), with the QoS class from
 * 11 S2.4. test_rt_keys.cc takes every suffix here and requires it to appear
 * verbatim in docs/11-接口契约.md, so a typo is a failing test rather than a
 * process nobody can hear.
 *
 * Three rows deserve a second look when reading this against the contract:
 *
 *   * rt/chassis/ctrl is SUBSCRIBED and rt/chassis/ctrl/ack is PUBLISHED. The
 *     asymmetry is 11 R-2: chassis_relay publishes the command, quadruped
 *     answers, and the ack key was a late addition because relay only forwards
 *     and cannot generate one.
 *   * rt/clock/status is subscribed but produces no reply. It feeds one field
 *     of every envelope this process writes (CLK-A2), which is why Q-5 exists
 *     at all and why its absence has to degrade ts_sync rather than be ignored.
 *   * rt/safety/probe/pong is published on receipt of a ping and NOT on a timer
 *     of its own (13 Q-4). A self-timed pong reports the link alive while the
 *     ping path is dead, which is the one thing the probe is for.
 */

#include "quadruped/rt_keys.h"

#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace quadruped {
namespace rt {

const KeySpec kKeys[] = {
    // ---- 13 S7.1: the four chassis report streams, forwarded verbatim -----
    {"rt/chassis/basic", KeyRole::kPublish, "Q2_state",
     "BasicStatus at 2 Hz; carries HES and Sleep, which Tier 1 reads"},
    {"rt/chassis/motion", KeyRole::kPublish, "Q2_state",
     "MotionStatus at 10 Hz; the odometry velocity source"},
    {"rt/chassis/device", KeyRole::kPublish, "Q2_state",
     "DeviceStatus at 2 Hz; batteries, temperatures, device enables"},
    {"rt/chassis/fault", KeyRole::kPublish, "Q2_state",
     "ErrorList at 2 Hz and on change; codes carry the chs: prefix (CF-2)"},
    // ---- 13 S7.1: the two aggregates -------------------------------------
    {"rt/chassis/state", KeyRole::kPublish, "Q2_state",
     "RobotState at 10 Hz, assembled from the four streams plus Tier 1"},
    {"rt/chassis/power", KeyRole::kPublish, "Q2_state",
     "PowerState at 1 Hz; SOC is the MINIMUM over the packs (11 S9.8.3)"},
    // ---- 13 S7.1.1 Q-1: handshake ----------------------------------------
    {"rt/chassis/hello", KeyRole::kSubscribe, "Q3_cmd",
     "the upstream announces itself and asks for our runtime description"},
    {"rt/chassis/hello_ack", KeyRole::kPublish, "Q3_cmd",
     "runtime.transport: effective endpoint, codebook, both DDS domains (DDS-9)"},
    // ---- 13 S7.1.1 Q-2: control, and the ack relay cannot generate --------
    {"rt/chassis/ctrl", KeyRole::kSubscribe, "Q0_safety",
     "stand / prone / enable / set_sdk_mode, forwarded by chassis_relay (R-2)"},
    {"rt/chassis/ctrl/ack", KeyRole::kPublish, "Q0_safety",
     "detail.action required; enable echoes the READ-BACK locks (CR-12)"},
    // ---- 13 S7.1.1 Q-3: the emergency stop --------------------------------
    {"rt/safety/estop", KeyRole::kSubscribe, "Q0_safety",
     "soft stop; the callback sends a zero frame itself, not next tick (T-1)"},
    {"rt/safety/estop/ack", KeyRole::kPublish, "Q0_safety",
     "within 100 ms; idempotent by cmd_id, estop_epoch unchanged on a repeat"},
    // ---- 13 S7.1.1 Q-4: liveness ------------------------------------------
    {"rt/safety/probe/ping", KeyRole::kSubscribe, "Q0_safety",
     "1 Hz from the safety probe"},
    {"rt/safety/probe/pong", KeyRole::kPublish, "Q0_safety",
     "answered ON RECEIPT, never self-timed: a self-timed pong reports a dead "
     "ping path as healthy, and its absence downgrades estop_path to down"},
    // ---- 13 S7.1.1 Q-5: the clock ----------------------------------------
    {"rt/clock/status", KeyRole::kSubscribe, "Q2_state",
     "rtk_driver is the only judge of sync; we copy it into every envelope "
     "(CLK-A2) and force false after 5 s of silence (CLK-A3)"},
    // ---- 13 S2.5 / 11 S9.3.1: the commanded velocity ----------------------
    {"rt/motion/cmd_vel", KeyRole::kSubscribe, "Q1_rt",
     "20 Hz from p1_motion; its age drives the Tier 1 timeout lock"},
    // ---- 11 S9.2.2 / S9.4: mode and light, both commanded from above ------
    {"rt/chassis/mode", KeyRole::kSubscribe, "Q3_cmd",
     "the mode triple; the ONLY path to usage_mode = navigation (NAV-111)"},
    {"rt/chassis/light", KeyRole::kSubscribe, "Q3_cmd",
     "light and voice, non-periodic, sent on the chs_a_tx thread"},
};

const std::size_t kKeyCount = sizeof(kKeys) / sizeof(kKeys[0]);

std::size_t BuildKey(const char* rid, const char* suffix, char* out,
                     std::size_t cap) {
  if (rid == nullptr || suffix == nullptr || out == nullptr || cap == 0) {
    return 0;
  }
  const int n = std::snprintf(out, cap, "%s/%s/%s", kKeyRoot, rid, suffix);
  // snprintf reports what it WOULD have written. A value reaching the end of
  // the buffer means the key was truncated, and a truncated key is well formed
  // and matches nothing -- the exact failure this file exists to prevent, so it
  // is reported as zero rather than returned.
  if (n < 0 || static_cast<std::size_t>(n) >= cap) return 0;
  return static_cast<std::size_t>(n);
}

std::string BuildKey(const std::string& rid, const std::string& suffix) {
  // Sized from the inputs rather than from a fixed maximum: a key is short, and
  // a constant here would be a second place for the format's length to be
  // assumed.
  std::string out(rid.size() + suffix.size() + std::strlen(kKeyRoot) + 3, '\0');
  const std::size_t n = BuildKey(rid.c_str(), suffix.c_str(), &out[0], out.size());
  if (n == 0) {
    throw std::length_error("rt::BuildKey: key does not fit: " + rid + " / " + suffix);
  }
  out.resize(n);
  return out;
}

const KeySpec* FindKey(const char* suffix) {
  if (suffix == nullptr) return nullptr;
  for (std::size_t i = 0; i < kKeyCount; ++i) {
    if (std::strcmp(kKeys[i].suffix, suffix) == 0) return &kKeys[i];
  }
  return nullptr;
}

}  // namespace rt
}  // namespace quadruped
