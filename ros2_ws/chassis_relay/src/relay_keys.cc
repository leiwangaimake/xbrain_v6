/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: relay_keys.cc
 * Brief: The CR-1..CR-12 table body, plus allocation-free key composition
 *
 * Description:
 * Transcribed from 11 S1.1.6 (3) "chassis_relay -- 12 条", after the
 * 2026-09-26 column-swap correction (the general-plane key sits in the THIRD
 * column of that table; the nine RT-to-GEN rows had the two key columns
 * swapped until then, and configs/generated/whitelist.yaml inherited the
 * swap). test_relay_keys.cc requires every key here to appear verbatim in
 * docs/11, so a typo is a failing test rather than a publisher nobody hears.
 *
 * QoS column, row by row:
 *   * the six safety keys (estop, ping/pong, both acks, ctrl) are Q0_safety --
 *     11 S2.4.7 v0.3 added the xbrain rt/safety wildcard binding and the estop rows to
 *     Q0 precisely because the bare rt wildcard fallback used to drop them into
 *     Q3-rt (the "most serious" finding of the S2.2.12 QoS audit table);
 *   * the five chassis state streams are Q2_state (periodic state, drop);
 *   * the fault stream publishes to event/fault/chassis, and the event wildcard class is
 *     Q3_cmd by anti-pattern A-4 (reliable + FIFO + block). That block is why
 *     CR-9 is the ONE row whose general-plane put happens off the zenoh
 *     callback thread -- see relay_core.h for the CRL-6 argument.
 *
 * estop_exempt is true on CR-1 alone. CR-11 looks similar (also a command,
 * also safety-critical) but its dangerous direction is the OPPOSITE one:
 * "enable" releases a lock, so a malformed frame must be dropped, never
 * raw-forwarded (11 S3.0.1 row 2: 放松型, 完整校验, 解析失败拒绝).
 */

#include "chassis_relay/relay_keys.h"

#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace chassis_relay {

// One row per contract entry, contract order. Directions read from the table's
// second column; keys from the corrected third (general) and fourth (RT)
// columns; QoS from the S2.4.7 bindings as resolved for each key.
const RelaySpec kRelayTable[kRelayCount] = {
    // ---- GEN -> RT: the three downstream safety commands ------------------
    {"CR-1", Direction::kGenToRt, "cmd/estop", "rt/safety/estop",
     "Q0_safety", true,
     "event; the soft stop -- unconditional forward (11 S3.0.1)"},
    {"CR-2", Direction::kGenToRt, "probe/estop/ping", "rt/safety/probe/ping",
     "Q0_safety", false,
     "1 Hz liveness ping from p5_gateway (11 S8.5)"},
    {"CR-11", Direction::kGenToRt, "cmd/chassis/ctrl", "rt/chassis/ctrl",
     "Q0_safety", false,
     "event; stand/prone/enable -- the ONLY unlock path (11 R-2)"},
    // ---- RT -> GEN: liveness answer, acks, and the state uplink -----------
    {"CR-3", Direction::kRtToGen, "probe/estop/pong", "rt/safety/probe/pong",
     "Q0_safety", false,
     "1 Hz liveness answer from quadruped (11 S8.5)"},
    {"CR-10", Direction::kRtToGen, "cmd/estop/ack", "rt/safety/estop/ack",
     "Q0_safety", false,
     "event, <=100 ms; the only legal estop-ack path (RT-C4)"},
    {"CR-12", Direction::kRtToGen, "cmd/chassis/ctrl/ack", "rt/chassis/ctrl/ack",
     "Q0_safety", false,
     "event; enable ack echoes READ-BACK locks (11 CR-12)"},
    {"CR-4", Direction::kRtToGen, "state/robot", "rt/chassis/state",
     "Q2_state", false, "10 Hz RobotState aggregate (11 S4.1)"},
    {"CR-5", Direction::kRtToGen, "state/power", "rt/chassis/power",
     "Q2_state", false, "1 Hz PowerState (11 S4.2)"},
    {"CR-6", Direction::kRtToGen, "state/chassis_basic", "rt/chassis/basic",
     "Q2_state", false, "2 Hz BasicStatus (11 S9.8.1)"},
    {"CR-7", Direction::kRtToGen, "state/chassis_motion", "rt/chassis/motion",
     "Q2_state", false, "10 Hz MotionStatus (11 S9.8.2)"},
    {"CR-8", Direction::kRtToGen, "state/chassis_device", "rt/chassis/device",
     "Q2_state", false, "2 Hz DeviceStatus (11 S9.8.3)"},
    {"CR-9", Direction::kRtToGen, "event/fault/chassis", "rt/chassis/fault",
     "Q3_cmd", false,
     "2 Hz + on change; Q3 block put runs on the event thread (CRL-6)"},
};

// The header promises kRelayCount rows so counter arrays can be sized at
// compile time; this is what keeps the promise honest when a row is added.
static_assert(sizeof(kRelayTable) / sizeof(kRelayTable[0]) == kRelayCount,
              "kRelayCount must match the table body");

std::size_t BuildRtKey(const char* rid, const char* suffix, char* out,
                       std::size_t cap) {
  if (rid == nullptr || suffix == nullptr || out == nullptr || cap == 0) {
    return 0;
  }
  const int n = std::snprintf(out, cap, "%s/%s/%s", kKeyRoot, rid, suffix);
  // snprintf reports what it WOULD have written. A value reaching the end of
  // the buffer means truncation, and a truncated key is well formed and
  // matches nothing -- reported as zero rather than returned, same contract
  // as quadruped's rt_keys.cc.
  if (n < 0 || static_cast<std::size_t>(n) >= cap) return 0;
  return static_cast<std::size_t>(n);
}

std::string BuildRtKey(const std::string& rid, const std::string& suffix) {
  // Sized from the inputs rather than a fixed maximum, so no second place
  // assumes the format's length. Startup only -- this allocates.
  std::string out(rid.size() + suffix.size() + std::strlen(kKeyRoot) + 3, '\0');
  const std::size_t n =
      BuildRtKey(rid.c_str(), suffix.c_str(), &out[0], out.size());
  if (n == 0) {
    throw std::length_error("chassis_relay::BuildRtKey: key does not fit: " +
                            rid + " / " + suffix);
  }
  out.resize(n);
  return out;
}

const RelaySpec* FindRelaySpec(const char* cr_id) {
  if (cr_id == nullptr) return nullptr;
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    if (std::strcmp(kRelayTable[i].cr_id, cr_id) == 0) return &kRelayTable[i];
  }
  // Not found is a real answer: the caller asked about a row that does not
  // exist, and inventing one would defeat the closed-set property CRL-3 is
  // named after.
  return nullptr;
}

}  // namespace chassis_relay
