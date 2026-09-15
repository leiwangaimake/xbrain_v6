/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rt_payloads.cc
 * Brief: The published data objects -- parsed back, not substring-matched
 *
 * Description:
 * Every case here PARSES the produced text and inspects the resulting value.
 * A substring check would pass on output that is not valid JSON at all, and
 * invalid JSON on state/robot has a very specific field symptom: the consumer
 * sees nothing, which looks like the robot having stopped reporting rather than
 * like a formatting bug. Parsing is the only assertion that separates those.
 *
 * What the cases are built around, each with the failure it prevents:
 *
 *   * hes / hes_lock / timeout_lock / locked are FOUR distinct fields. 11's
 *     D-08 ruling exists because collapsing any two of them loses the
 *     distinction that decides whether a robot can be restarted from a screen.
 *     The case drives them to four different combinations rather than checking
 *     that the keys exist.
 *   * faults[].code carries the chs: prefix here as well (CF-5). The example in
 *     11 S4.1 still shows a bare "0x1007", so an implementer copying the
 *     example produces something that disagrees with the fault stream about
 *     what a code means.
 *   * batteries.left/right are null while the mapping is unknown (13 BAT-2).
 *     Filling them from array order is a guess presented as a measurement.
 *   * an absent report is null, not a zeroed struct. A zeroed BasicStatus reads
 *     as "idle, awake, no emergency stop" -- indistinguishable from a healthy
 *     robot standing still, which is the worst possible default for the message
 *     an operator watches.
 *   * every writer returns 0 rather than truncating.
 */

#include "quadruped/rt_payloads.h"

#include <cstdio>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

using namespace quadruped;        // NOLINT: test-local
using namespace quadruped::rt;    // NOLINT
using Json = nlohmann::json;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

// Parse the produced text, failing loudly when it is not JSON at all. Returns a
// discarded value on failure so the caller's field checks fail rather than
// throw out of main.
Json ParseOrFail(const char* what, const char* buf, std::size_t n) {
  if (n == 0) {
    std::printf("FAIL %s: writer returned 0\n", what);
    ++g_failures;
    return Json();
  }
  Json j = Json::parse(buf, buf + n, nullptr, false);
  if (j.is_discarded()) {
    std::printf("FAIL %s: not valid JSON: %.*s\n", what,
                static_cast<int>(n > 200 ? 200 : n), buf);
    ++g_failures;
    return Json();
  }
  return j;
}

chs_a::BasicStatus MakeBasic() {
  chs_a::BasicStatus b;
  b.usage_mode = chs_a::ResolveUsageMode(1);     // navigation
  b.motion_state = chs_a::ResolveMotionState(17);  // rl_control
  b.gait = chs_a::ResolveGait(0x3002);           // flat
  b.model = "CA9C";
  b.version = "PRO";
  b.hes = false;
  b.sleep = false;
  b.power_management = 0;
  return b;
}

}  // namespace

int main() {
  char buf[8192];

  // ---- RobotState, everything present ------------------------------------
  {
    const chs_a::BasicStatus basic = MakeBasic();
    chs_a::MotionStatus motion;
    motion.linear_x = 0.25;
    motion.angular_z = -0.1;
    motion.yaw = 1.5;

    chs_a::FaultReport faults;
    chs_a::FaultEntry f;
    f.code = chs_a::FormatChassisFaultCode(0x8001);
    f.level = "fatal";
    f.name = "motor_over_temperature";
    faults.faults.push_back(f);

    RobotStateInput in;
    in.conn = chs_a::ConnState::kOk;
    in.basic = &basic;
    in.motion = &motion;
    in.faults = &faults;
    in.tier1.stop_reason = StopReason::kNone;
    in.estop_epoch = 42;
    in.cmd_age_ms = 12.0;

    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("RobotState", buf, n);
    CHECK(j["conn"] == "ok");
    // The open-set fields go out as LABEL plus RAW (13 S6.5 ban 3): a label
    // alone is not something a field engineer can match to the manual.
    CHECK(j["usage_mode"] == "navigation");
    CHECK(j["usage_mode_raw"] == 1);
    CHECK(j["motion_state"] == "rl_control");
    CHECK(j["motion_state_raw"] == 17);
    CHECK(j["gait"] == "flat");
    CHECK(j["gait_raw"] == 0x3002);
    CHECK(j["active_axes"] == Json::array({"vx", "vy", "wz"}));
    CHECK(j["stop_reason"] == "none");
    CHECK(j["estop_epoch"] == 42);
    CHECK(j["cmd_age_ms"] == 12.0);
    CHECK(j["motion"]["vx"] == 0.25);
    // *** CF-5: the same prefixed code the fault stream carries. The 11 S4.1
    // example shows a bare 0x1007, and copying it makes the two sides disagree
    // about which of the two overlapping code spaces a number belongs to.
    CHECK(j["faults"].size() == 1);
    CHECK(j["faults"][0]["code"] == "chs:0x8001");
    CHECK(j["faults"][0]["level"] == "fatal");
    CHECK(j["faults"][0]["name"] == "motor_over_temperature");
  }

  // ---- the four stop-related fields are four fields ----------------------
  {
    // 11 D-08: hes is the raw level, hes_lock is a latch software cannot clear,
    // timeout_lock is a latch a human clears, locked summarises the two. Driven
    // to four combinations, because a test that only checks the keys exist
    // passes on an implementation that writes one value into all of them.
    struct Case {
      bool hes, hes_lock, timeout_lock, expect_locked;
    };
    const Case cases[] = {
        {false, false, false, false},
        {true, true, false, true},     // pressed and latched
        {false, true, false, true},    // released, still latched -- the case
                                       // that matters: the robot must not move
        {false, false, true, true},    // upstream went away
        {true, true, true, true},
    };
    for (const Case& c : cases) {
      chs_a::BasicStatus basic = MakeBasic();
      basic.hes = c.hes;
      RobotStateInput in;
      in.conn = chs_a::ConnState::kOk;
      in.basic = &basic;
      in.tier1.hes_lock = c.hes_lock;
      in.tier1.timeout_lock = c.timeout_lock;
      in.tier1.stop_reason =
          c.hes_lock ? StopReason::kHes
                     : (c.timeout_lock ? StopReason::kTimeout : StopReason::kNone);
      const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
      const Json j = ParseOrFail("RobotState/locks", buf, n);
      CHECK(j["hes"] == c.hes);
      CHECK(j["hes_lock"] == c.hes_lock);
      CHECK(j["timeout_lock"] == c.timeout_lock);
      CHECK(j["locked"] == c.expect_locked);
    }
  }

  // ---- every stop reason round-trips through the closed set --------------
  {
    const StopReason all[] = {
        StopReason::kNone,     StopReason::kHes,           StopReason::kTimeout,
        StopReason::kSoftEstop, StopReason::kModeSwitching, StopReason::kSleep,
        StopReason::kModeMismatch, StopReason::kNan,        StopReason::kNoSource};
    for (StopReason r : all) {
      RobotStateInput in;
      in.tier1.stop_reason = r;
      const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
      const Json j = ParseOrFail("RobotState/stop_reason", buf, n);
      CHECK(j["stop_reason"] == std::string(StopReasonName(r)));
    }
  }

  // ---- before the first report, null -- never a zeroed struct ------------
  {
    RobotStateInput in;
    in.conn = chs_a::ConnState::kProbing;
    in.cmd_age_ms = -1.0;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("RobotState/empty", buf, n);
    CHECK(j["conn"] == "probing");
    // *** A zeroed BasicStatus would read as usage_mode "normal",
    // motion_state "idle", hes false, sleep false -- which is what a healthy
    // robot standing still looks like. null is the only honest answer before
    // the chassis has said anything.
    CHECK(j["usage_mode"].is_null());
    CHECK(j["motion_state"].is_null());
    CHECK(j["hes"].is_null());
    CHECK(j["sleep"].is_null());
    CHECK(j["motion"].is_null());
    // "never received a command" is a different fact from "the command is very
    // old", so it is null rather than a large number.
    CHECK(j["cmd_age_ms"].is_null());
    CHECK(j["faults"].is_array() && j["faults"].empty());
  }

  // ---- PowerState: the minimum, and the absent slot beside it ------------
  {
    chs_a::DeviceStatus dev;
    chs_a::BatteryEntry a0;
    a0.level = 0; a0.voltage = 0.0; a0.temperature_c = -273.0; a0.present = false;
    chs_a::BatteryEntry a1;
    a1.level = 26; a1.voltage = 69.28; a1.temperature_c = 38.2; a1.present = true;
    dev.batteries = {a0, a1};
    dev.min_level = 0;
    dev.present_count = 1;

    chs_a::BasicStatus basic = MakeBasic();
    basic.power_management = 1;   // single_battery, as measured with one pack

    PowerStateInput in;
    in.device = &dev;
    in.basic = &basic;
    in.remain_mile_km = 4.2;

    const std::size_t n = WritePowerState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("PowerState", buf, n);
    // 11 S4.2 / 13 BAT-1: the minimum, unchanged even with a slot empty. The
    // honest response to 13 V-68 is to publish present_count beside it, not to
    // quietly exclude the slot.
    CHECK(j["soc_pct"] == 0);
    CHECK(j["present_count"] == 1);
    CHECK(j["remain_mile_km"] == 4.2);
    CHECK(j["power_management"] == "single_battery");
    // 13 BAT-1: the array is authoritative and keeps its original indices.
    CHECK(j["list"].size() == 2);
    CHECK(j["list"][0]["index"] == 0);
    CHECK(j["list"][0]["present"] == false);
    CHECK(j["list"][1]["level_pct"] == 26);
    CHECK(j["list"][1]["present"] == true);
    // *** 13 BAT-2: null while the mapping is unknown, and the mapping field
    // says so. Array order is not a measurement.
    CHECK(j["batteries"].is_null());
    CHECK(j["battery_mapping"] == "unknown");
  }

  // ---- PowerState with the mapping configured ---------------------------
  {
    chs_a::DeviceStatus dev;
    chs_a::BatteryEntry a0; a0.level = 43; a0.voltage = 73.15; a0.present = true;
    chs_a::BatteryEntry a1; a1.level = 44; a1.voltage = 73.35; a1.present = true;
    dev.batteries = {a0, a1};
    dev.min_level = 43;
    dev.present_count = 2;
    chs_a::BasicStatus basic = MakeBasic();

    PowerStateInput in;
    in.device = &dev;
    in.basic = &basic;
    in.index_map_known = true;
    in.left_index = 0;
    in.right_index = 1;
    const std::size_t n = WritePowerState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("PowerState/mapped", buf, n);
    CHECK(j["battery_mapping"] == "known");
    CHECK(j["batteries"]["left"]["level_pct"] == 43);
    CHECK(j["batteries"]["right"]["level_pct"] == 44);
    CHECK(j["soc_pct"] == 43);
    CHECK(j["power_management"] == "normal");

    // A mapping that points outside the array is a configuration error, and
    // null is the same answer as "unknown". Inventing a side would be worse.
    PowerStateInput bad = in;
    bad.right_index = 7;
    const std::size_t m = WritePowerState(bad, buf, sizeof(buf));
    const Json jb = ParseOrFail("PowerState/badmap", buf, m);
    CHECK(jb["batteries"].is_null());
  }

  // ---- an unregistered power_management is reported as itself -----------
  {
    chs_a::DeviceStatus dev;
    chs_a::BasicStatus basic = MakeBasic();
    basic.power_management = 7;
    PowerStateInput in;
    in.device = &dev;
    in.basic = &basic;
    const std::size_t n = WritePowerState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("PowerState/unknownpm", buf, n);
    // Same open-set discipline as the mode fields: not mapped onto one of the
    // two known values, and the raw number survives in the label.
    CHECK(j["power_management"] == "unknown_7");
  }

  // ---- EstopAck ----------------------------------------------------------
  {
    EstopAckInput in;
    in.cmd_id = "e-3d91";
    in.estop_epoch = 42;
    in.applied_zero_vel = true;
    in.recv_mono_ms = 918273645;
    in.latency_ms = 7;
    const std::size_t n = WriteEstopAck(in, buf, sizeof(buf));
    const Json j = ParseOrFail("EstopAck", buf, n);
    CHECK(j["cmd_id"] == "e-3d91");
    CHECK(j["result"] == "accepted");
    CHECK(j["estop_epoch"] == 42);
    CHECK(j["applied"] == Json::array({"zero_vel"}));
    // *** MILLISECONDS here, while the envelope around this message carries
    // mono in SECONDS. Two units for one clock in one message is the contract's
    // own shape, and this case exists so nobody "fixes" one to match the other.
    CHECK(j["recv_mono_ms"] == 918273645);
    CHECK(j["latency_ms"] == 7);

    // A repeat of the same cmd_id is `duplicate` with the SAME epoch: 11 makes
    // the stop idempotent, and a new epoch would make the upstream believe a
    // second stop happened.
    EstopAckInput dup = in;
    dup.result = "duplicate";
    const std::size_t m = WriteEstopAck(dup, buf, sizeof(buf));
    const Json jd = ParseOrFail("EstopAck/dup", buf, m);
    CHECK(jd["result"] == "duplicate");
    CHECK(jd["estop_epoch"] == 42);

    // An anonymous request echoes the literal 11 names, not an empty string.
    EstopAckInput anon;
    const std::size_t k = WriteEstopAck(anon, buf, sizeof(buf));
    const Json ja = ParseOrFail("EstopAck/anon", buf, k);
    CHECK(ja["cmd_id"] == "anonymous");

    // Both actions, so the array is an array and not a scalar in disguise.
    EstopAckInput two = in;
    two.applied_charge_abort = true;
    const std::size_t t = WriteEstopAck(two, buf, sizeof(buf));
    const Json jt = ParseOrFail("EstopAck/two", buf, t);
    CHECK(jt["applied"] == Json::array({"zero_vel", "charge_abort"}));
  }

  // ---- CtrlAck: the action, and the READ-BACK locks ---------------------
  {
    CtrlAckInput in;
    in.cmd_id = "c-9a3f2e";
    in.action = "enable";
    // 11 CR-12: after enable these are the values READ BACK, not the ones
    // requested. "Ack = accepted" is not "the lock is gone", and an ack that
    // omitted them would leave the caller unable to tell the difference.
    in.hes_lock = true;
    in.timeout_lock = false;
    const std::size_t n = WriteCtrlAck(in, buf, sizeof(buf));
    const Json j = ParseOrFail("CtrlAck", buf, n);
    CHECK(j["cmd_id"] == "c-9a3f2e");
    CHECK(j["result"] == "accepted");
    CHECK(j["detail"]["action"] == "enable");
    CHECK(j["detail"]["hes_lock"] == true);
    CHECK(j["detail"]["timeout_lock"] == false);
  }

  // ---- Pong --------------------------------------------------------------
  {
    PongInput in;
    in.seq = 991;
    in.t_mono_ms = 12345;
    in.estop_epoch = 42;
    in.hes_lock = true;
    in.stop_reason = StopReason::kHes;
    const std::size_t n = WritePong(in, buf, sizeof(buf));
    const Json j = ParseOrFail("Pong", buf, n);
    CHECK(j["type"] == "pong");
    // The seq is ECHOED from the ping, so the prober can match a reply to its
    // request; a pong with its own counter answers a question nobody asked.
    CHECK(j["seq"] == 991);
    CHECK(j["t_mono_ms"] == 12345);
    CHECK(j["estop_epoch"] == 42);
    CHECK(j["hes_lock"] == true);
    CHECK(j["stop_reason"] == "hes");
  }

  // ---- a quote in a fault name must not break the object ----------------
  {
    // The names come from a vendor firmware, not from an operator, so this is
    // unlikely rather than impossible. The symptom if it happened would be
    // state/robot going silent, which reads as the robot having died.
    chs_a::FaultReport faults;
    chs_a::FaultEntry f;
    f.code = chs_a::FormatChassisFaultCode(0x8001);
    f.level = "warn";
    f.name = "joint \"11\" \\ over\nlimit";
    faults.faults.push_back(f);
    RobotStateInput in;
    in.faults = &faults;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("RobotState/escape", buf, n);
    CHECK(j["faults"][0]["name"] == "joint \"11\" \\ over\nlimit");
  }

  // ---- every writer refuses a short buffer ------------------------------
  {
    // Half an object is valid-looking text that decodes to the wrong thing, or
    // to nothing at all. Each writer is given exactly one byte less than it
    // needs, which is the boundary a length check gets wrong.
    RobotStateInput rs;
    const std::size_t full_rs = WriteRobotState(rs, buf, sizeof(buf));
    CHECK(full_rs > 0);
    CHECK(WriteRobotState(rs, buf, full_rs) == 0);
    CHECK(WriteRobotState(rs, buf, 0) == 0);
    CHECK(WriteRobotState(rs, nullptr, 64) == 0);

    chs_a::DeviceStatus dev;
    PowerStateInput ps;
    ps.device = &dev;
    const std::size_t full_ps = WritePowerState(ps, buf, sizeof(buf));
    CHECK(full_ps > 0);
    CHECK(WritePowerState(ps, buf, full_ps) == 0);

    EstopAckInput ea;
    const std::size_t full_ea = WriteEstopAck(ea, buf, sizeof(buf));
    CHECK(full_ea > 0);
    CHECK(WriteEstopAck(ea, buf, full_ea) == 0);

    CtrlAckInput ca;
    const std::size_t full_ca = WriteCtrlAck(ca, buf, sizeof(buf));
    CHECK(full_ca > 0);
    CHECK(WriteCtrlAck(ca, buf, full_ca) == 0);

    PongInput pg;
    const std::size_t full_pg = WritePong(pg, buf, sizeof(buf));
    CHECK(full_pg > 0);
    CHECK(WritePong(pg, buf, full_pg) == 0);
  }

  if (g_failures == 0) {
    std::printf("ALL RT_PAYLOADS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RT_PAYLOADS TEST(S) FAILED\n", g_failures);
  return 1;
}
