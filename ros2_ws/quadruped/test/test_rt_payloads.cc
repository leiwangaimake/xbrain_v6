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

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "quadruped/chs_a_codec.h"
#include "quadruped/chs_a_reports.h"

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

namespace {

// The same self-contained loader the other four capture-driven tests carry.
// Kept local rather than shared: each test here is a standalone binary, and a
// shared header would make four passing tests depend on a fifth one's edits.
using Bytes = std::vector<std::uint8_t>;

Bytes FromHex(const std::string& hex) {
  Bytes out;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

std::map<std::string, Bytes> LoadGolden(const std::string& path, int* failures) {
  std::map<std::string, Bytes> out;
  std::ifstream f(path);
  if (!f) {
    std::printf("FAIL cannot open golden file: %s\n", path.c_str());
    ++*failures;
    return out;
  }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream is(line);
    std::string tag, hex;
    std::size_t n = 0;
    if (!(is >> tag >> n >> hex)) continue;
    out[tag] = FromHex(hex);
  }
  if (out.empty()) {
    std::printf("FAIL golden file parsed to zero vectors\n");
    ++*failures;
  }
  return out;
}

// Equal to the precision the writer actually emits.
//
// Num formats with "%.6g" -- six significant digits -- so a round trip is NOT
// bit-exact and an == on a double that went through it is a test that passes by
// luck when the value happens to be 0.0 and fails otherwise. That is exactly
// what happened here: vel.x and vel.y were zero in the captured frame and
// passed, while the yaw rate and omega_z did not.
//
// The tolerance is the format's, not a fudge factor: six significant digits on
// a rad/s figure is far below any actuator or sensor resolution in this system.
bool Close(double got, double want) {
  const double scale = std::fabs(want) > 1.0 ? std::fabs(want) : 1.0;
  return std::fabs(got - want) <= 1e-6 * scale;
}

// The ASDU of a captured frame, ready for a parser.
const std::uint8_t* Asdu(const Bytes& frame) {
  return frame.data() + chs_a::kHeaderBytes;
}
std::size_t AsduLen(const Bytes& frame) {
  return frame.size() - chs_a::kHeaderBytes;
}

}  // namespace

int main(int argc, char** argv) {
  char buf[8192];

  // ---- RobotState, everything present ------------------------------------
  {
    const chs_a::BasicStatus basic = MakeBasic();
    chs_a::MotionStatus motion;
    motion.linear_x = 0.25;
    motion.angular_z = -0.1;
    motion.yaw = 1.5;

    // The fault entries arrive as the VIEW rt_bridge builds from its cache --
    // already prefixed, already levelled (CF-5's "same converter", satisfied
    // by copying the fault stream's own values).
    const std::string fcode = chs_a::FormatChassisFaultCode(0x8001);
    const RobotStateFault fview[] = {
        {fcode.c_str(), "fatal", "motor_over_temperature"},
    };

    RobotStateInput in;
    // Wire vocabulary, as the bridge maps it -- the writer echoes it.
    in.conn_wire = "connected";
    in.proto_version = "1.0";
    in.basic = &basic;
    in.motion = &motion;
    in.faults = fview;
    in.fault_count = 1;
    in.tier1.stop_reason = StopReason::kNone;
    in.estop_epoch = 42;
    in.cmd_age_ms = 12.0;

    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("RobotState", buf, n);
    CHECK(j["conn"] == "connected");
    CHECK(j["proto_version"] == "1.0");
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
    // The third key is `desc` -- the contract example's own spelling; this
    // writer said "name" until 2026-09-26 and a consumer coded against the
    // contract found nothing.
    CHECK(j["faults"].size() == 1);
    CHECK(j["faults"][0]["code"] == "chs:0x8001");
    CHECK(j["faults"][0]["level"] == "fatal");
    CHECK(j["faults"][0]["desc"] == "motor_over_temperature");
    CHECK(!j["faults"][0].contains("name"));
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
      in.conn_wire = "connected";
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
    in.conn_wire = "connecting";
    in.cmd_age_ms = -1.0;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("RobotState/empty", buf, n);
    CHECK(j["conn"] == "connecting");
    // The contract fields whose data has not happened yet are null, with one
    // exception: mode_mismatch is OMITTED (当且仅当 in 11 S4.1 -- an absent
    // condition is an absent key, not a null).
    CHECK(j["proto_version"].is_null());
    CHECK(j["last_soft_estop"].is_null());
    CHECK(!j.contains("mode_mismatch"));
    CHECK(j["model"].is_null());
    CHECK(j["version"].is_null());
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

  // ---- conn is the writer's most defensive field --------------------------
  {
    // A nullptr conn_wire is a caller defect (the bridge always fills it);
    // the writer answers null rather than inventing a member (11 S13.6).
    // mutant: fall back to any fixed member here -> red.
    RobotStateInput in;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("RobotState/conn-null", buf, n);
    CHECK(j["conn"].is_null());
  }

  // ---- 11 S4.1 last_soft_estop: the object, and its two null spellings ----
  {
    // Fully attributed stop: all four fields as given.
    RobotStateInput in;
    in.has_last_estop = true;
    in.last_estop_epoch = 42;
    in.last_estop_reason = "operator_hmi";
    in.last_estop_src_role = "hmi";
    in.last_estop_age_ms = 3200.0;
    std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    Json j = ParseOrFail("RobotState/last_estop", buf, n);
    CHECK(j["last_soft_estop"]["epoch"] == 42);
    CHECK(j["last_soft_estop"]["reason"] == "operator_hmi");
    CHECK(j["last_soft_estop"]["src_role"] == "hmi");
    CHECK(j["last_soft_estop"]["age_ms"] == 3200.0);

    // A stop that arrived WITHOUT the audit pair (best-effort on that key,
    // 11 S9.12) still has an epoch and an age; the two strings are null, not
    // "" -- an empty string reads as a sender that supplied a blank reason.
    // mutant: emit "" for an absent reason -> red.
    RobotStateInput bare;
    bare.has_last_estop = true;
    bare.last_estop_epoch = 7;
    bare.last_estop_age_ms = 15.5;
    n = WriteRobotState(bare, buf, sizeof(buf));
    j = ParseOrFail("RobotState/last_estop-bare", buf, n);
    CHECK(j["last_soft_estop"]["epoch"] == 7);
    CHECK(j["last_soft_estop"]["reason"].is_null());
    CHECK(j["last_soft_estop"]["src_role"].is_null());
    CHECK(j["last_soft_estop"]["age_ms"] == 15.5);
    // And the age is the caller's number, not something re-derived: the two
    // cases above already pin two different values on the same field, which
    // is what kills a writer that stamps its own clock here.
  }

  // ---- 11 S4.1 mode_mismatch: 当且仅当 stop_reason == mode_mismatch --------
  {
    // Present, with the read-back actual, exactly when the reason says so.
    RobotStateInput in;
    in.tier1.stop_reason = StopReason::kModeMismatch;
    in.has_triple = true;
    in.usage_mode_raw = 0;                    // read back "normal"
    in.motion_state_raw = 17;
    in.gait_raw = 0x3002;
    std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    Json j = ParseOrFail("RobotState/mode_mismatch", buf, n);
    CHECK(j.contains("mode_mismatch"));
    CHECK(j["mode_mismatch"]["expect"] == "navigation");
    CHECK(j["mode_mismatch"]["actual"] == "normal");

    // No read-back yet IS one of the ways a mismatch happens: the object is
    // present and actual is null, never a guessed member (11 S13.6).
    RobotStateInput cold;
    cold.tier1.stop_reason = StopReason::kModeMismatch;
    n = WriteRobotState(cold, buf, sizeof(buf));
    j = ParseOrFail("RobotState/mode_mismatch-cold", buf, n);
    CHECK(j.contains("mode_mismatch"));
    CHECK(j["mode_mismatch"]["expect"] == "navigation");
    CHECK(j["mode_mismatch"]["actual"].is_null());

    // *** The other half of 当且仅当: ANY other stop_reason omits the key --
    // not null, absent. Driven across every member so a writer gating on the
    // wrong reason (or on none) is red, not merely the none case.
    const StopReason others[] = {
        StopReason::kNone,      StopReason::kHes,
        StopReason::kTimeout,   StopReason::kSoftEstop,
        StopReason::kModeSwitching, StopReason::kSleep,
        StopReason::kNan,       StopReason::kNoSource};
    for (StopReason r : others) {
      RobotStateInput o;
      o.tier1.stop_reason = r;
      o.has_triple = true;
      o.usage_mode_raw = 0;
      n = WriteRobotState(o, buf, sizeof(buf));
      j = ParseOrFail("RobotState/mode_mismatch-absent", buf, n);
      CHECK(!j.contains("mode_mismatch"));
    }
  }

  // ---- model/version travel on the state path once cached -----------------
  {
    // The report-side cache can be filled while the triple has not yet come
    // through the ctrl slot -- the two ride different threads. Both branches
    // of the writer must pass the strings through.
    RobotStateInput in;                      // no basic, no triple
    in.model = "CA9C";
    in.version = "PRO";
    std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    Json j = ParseOrFail("RobotState/model-cold", buf, n);
    CHECK(j["model"] == "CA9C");
    CHECK(j["version"] == "PRO");

    RobotStateInput tr;                      // triple present, same strings
    tr.has_triple = true;
    tr.usage_mode_raw = 1;
    tr.model = "CA9C";
    tr.version = "PRO";
    n = WriteRobotState(tr, buf, sizeof(buf));
    j = ParseOrFail("RobotState/model-triple", buf, n);
    CHECK(j["model"] == "CA9C");
    CHECK(j["version"] == "PRO");
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
    // Nothing named it, so the key is ABSENT -- not present-and-empty. See the
    // next case for why the writer distinguishes the two.
    CHECK(j["detail"].find("item") == j["detail"].end());

    // 11 S9.3.3: the refusal that HAS a name carries it. Checked here on the
    // bytes, because the mapping function on its own was already covered by a
    // test and was still unreachable from any ack for a whole batch.
    CtrlAckInput named = in;
    named.result = "rejected";
    named.action = "prone";
    named.item = "prone_on_stair";
    const std::size_t m = WriteCtrlAck(named, buf, sizeof(buf));
    const Json jn = ParseOrFail("CtrlAck/item", buf, m);
    CHECK(jn["detail"]["item"] == "prone_on_stair");
    // The item lives under detail, next to action -- not at the top level.
    // 11 S13.9's shape is {code, detail:{item, ...}}, and a consumer reading
    // detail.item finds nothing if it is hoisted.
    CHECK(jn.find("item") == jn.end());
  }

  // ---- Envelope: 11 S3.0's eight fields, and the payload under `data` ----
  {
    EnvelopeInput env;
    env.rid = "gj-001";
    env.boot = "a1b2c3d4";
    env.src = "quadruped";
    // A real wall clock and a real monotonic reading, both with a fractional
    // part -- see the TimeSec assertions below for why these values matter.
    env.ts = 1789455340.125;
    env.mono = 940821.337215;
    env.seq = 7;
    const char kPayload[] = "{\"hello\":1}";
    // NOT strlen at the call site: the writer takes a length because the
    // payloads it wraps are not NUL-terminated.
    const std::size_t n =
        WriteEnvelope(env, kPayload, sizeof(kPayload) - 1, buf, sizeof(buf));
    const Json j = ParseOrFail("Envelope", buf, n);
    CHECK(j["v"] == 1);
    CHECK(j["rid"] == "gj-001");
    CHECK(j["boot"] == "a1b2c3d4");
    CHECK(j["src"] == "quadruped");
    CHECK(j["seq"] == 7);
    // 13 PB-Q3 forbids a true fallback, and 11 S3.0 gives a missing field the
    // same meaning as false. Until 13 Q-5's rt/clock/status subscription
    // exists, false is the only value this may carry.
    CHECK(j["ts_sync"] == false);
    // The payload arrives intact and NESTED, not merged into the envelope.
    CHECK(j["data"]["hello"] == 1);
    CHECK(j.find("hello") == j.end());
    // All eight, and nothing else: a ninth field means something leaked out
    // of data into the envelope.
    CHECK(j.size() == 9);   // the eight + data

    // *** The timestamps, to the microsecond. This is the assertion that
    // catches Num()'s "%.6g": it would render ts as 1.78996e+09 (six
    // SIGNIFICANT digits) and mono as 940821, losing every fractional digit.
    // 11 S3.0 makes mono "一切超时与年龄判定的唯一依据", so a one-second
    // resolution there coarsens every age in the system -- and it would do it
    // silently, because 940821 is still a valid JSON number.
    CHECK(j["ts"].get<double>() > 1789455340.0);
    CHECK(j["ts"].get<double>() < 1789455340.5);
    CHECK(j["mono"].get<double>() > 940821.3);
    CHECK(j["mono"].get<double>() < 940821.4);

    // Too small to hold the wrapped object: nothing, never a truncated one.
    CHECK(WriteEnvelope(env, kPayload, sizeof(kPayload) - 1, buf, n) == 0);
    // An empty payload is refused rather than wrapped as `"data":`, which
    // would not be parseable.
    CHECK(WriteEnvelope(env, kPayload, 0, buf, sizeof(buf)) == 0);
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
    const std::string fcode = chs_a::FormatChassisFaultCode(0x8001);
    const RobotStateFault fview[] = {
        {fcode.c_str(), "warn", "joint \"11\" \\ over\nlimit"},
    };
    RobotStateInput in;
    in.faults = fview;
    in.fault_count = 1;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("RobotState/escape", buf, n);
    CHECK(j["faults"][0]["desc"] == "joint \"11\" \\ over\nlimit");
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

  // ---- the four report streams, from REAL frames -------------------------
  //
  // Parsed by the real parsers from the capture taken on 2026-09-15, then
  // written by the real writers. Neither half is stubbed, so what is checked is
  // the pair: a field that the parser fills and the writer drops would pass a
  // test of either one alone.
  {
    const std::string golden_path =
        (argc >= 2) ? argv[1] : "test/golden/chs_a_frames.txt";
    const auto golden = LoadGolden(golden_path, &g_failures);
    if (!golden.empty()) {
      char out[8192];

      // --- basic ---------------------------------------------------------
      chs_a::BasicStatus b;
      const Bytes& bf = golden.at("RX_00100064_00f00000");
      CHECK(chs_a::ParseBasicStatus(Asdu(bf), AsduLen(bf), &b));
      std::size_t n = WriteChassisBasic(b, out, sizeof(out));
      CHECK(n > 0);
      Json jb = Json::parse(out, out + n, nullptr, false);
      CHECK(!jb.is_discarded());
      if (!jb.is_discarded()) {
        // The open set travels as BOTH the number and the label. 13 S6.5 bans
        // mapping an unregistered value onto a known one, and a consumer given
        // only "unknown_0x0000" cannot tell WHICH unregistered value it was --
        // 13 V-66's Gait 0 is exactly that case and it is in this very frame.
        // FLAT `name` + `name_raw`, which is what OpenSet emits and what
        // WriteRobotState already publishes. This test first asserted a NESTED
        // shape and caught the writer emitting a second one -- two shapes for
        // the same value is a consumer having to choose.
        CHECK(jb["gait"] == b.gait.label);
        CHECK(jb["gait_raw"] == b.gait.raw);
        CHECK(jb["motion_state_raw"] == b.motion_state.raw);
        CHECK(jb["usage_mode_raw"] == b.usage_mode.raw);
        CHECK(jb["hes"] == b.hes);
        CHECK(jb["sleep"] == b.sleep);
        CHECK(jb["model"] == b.model);
        // 13 S5.6 / V-53: PRO is what gates the chassis navigation licence, and
        // an operator cannot tell a STD machine from a PRO one without it.
        CHECK(jb["version"] == b.version);
      }

      // --- motion --------------------------------------------------------
      chs_a::MotionStatus m2;
      const Bytes& mf = golden.at("RX_00100001_00f00000");
      CHECK(chs_a::ParseMotionStatus(Asdu(mf), AsduLen(mf), &m2));
      n = WriteChassisMotion(m2, out, sizeof(out));
      CHECK(n > 0);
      Json jm = Json::parse(out, out + n, nullptr, false);
      CHECK(!jm.is_discarded());
      if (!jm.is_discarded()) {
        CHECK(Close(jm["vel"]["x"], m2.linear_x));
        CHECK(Close(jm["vel"]["y"], m2.linear_y));
        // The manual's units column says raw/s for this axis and 13 V-46
        // records that as an error: the wire value is rad/s. Forwarding it
        // under another name would make every consumer wrong by 57.
        CHECK(Close(jm["vel"]["yaw"], m2.angular_z));
        CHECK(Close(jm["rpy"]["yaw"], m2.yaw));
        CHECK(jm["imu"]["acc"].size() == 3);
        CHECK(jm["imu"]["omega"].size() == 3);
        CHECK(Close(jm["imu"]["omega"][2], m2.omega_z));
      }

      // --- device --------------------------------------------------------
      chs_a::DeviceStatus d;
      const Bytes& df = golden.at("RX_00100002_00f00000");
      CHECK(chs_a::ParseDeviceStatus(Asdu(df), AsduLen(df), &d));
      n = WriteChassisDevice(d, out, sizeof(out));
      CHECK(n > 0);
      Json jd = Json::parse(out, out + n, nullptr, false);
      CHECK(!jd.is_discarded());
      if (!jd.is_discarded()) {
        CHECK(jd["list"].size() == d.batteries.size());
        CHECK(jd["min_level_pct"] == d.min_level);
        // 13 V-68: an empty slot reports 0, so min_level alone cannot tell a
        // flat battery from an absent one. present_count is the fact that
        // would otherwise be missing -- the contract's min() rule is untouched.
        CHECK(jd["present_count"] == d.batteries.size() ||
              jd["present_count"] == d.present_count);
        for (std::size_t i = 0; i < d.batteries.size(); ++i) {
          // The ORIGINAL index, not a position after filtering: 13 V-55 records
          // that the left/right mapping is unknown, so a moved index destroys
          // the only handle anyone has on which slot is which.
          CHECK(jd["list"][i]["index"] == i);
          CHECK(jd["list"][i]["level_pct"] == d.batteries[i].level);
          CHECK(jd["list"][i]["present"] == d.batteries[i].present);
        }
      }

      // --- fault ---------------------------------------------------------
      chs_a::FaultReport f;
      const Bytes& ff = golden.at("RX_0010007f_00f00000");
      CHECK(chs_a::ParseFaultReport(Asdu(ff), AsduLen(ff), &f));
      n = WriteChassisFault(f, out, sizeof(out));
      CHECK(n > 0);
      Json jf = Json::parse(out, out + n, nullptr, false);
      CHECK(!jf.is_discarded());
      if (!jf.is_discarded()) {
        // BOTH lists, always, including when one is empty. An empty `cleared`
        // and an absent `cleared` are different claims, and a consumer that has
        // to guess keeps a fault asserted forever.
        CHECK(jf.contains("faults"));
        CHECK(jf.contains("cleared"));
        CHECK(jf["faults"].is_array());
        CHECK(jf["cleared"].is_array());
        CHECK(jf["faults"].size() == f.faults.size());
        CHECK(jf["fault_count"] == f.faults.size());
        for (std::size_t i = 0; i < f.faults.size(); ++i) {
          // The prefixed form (13 S7.3): "chs:0x8001", never a bare number --
          // the chassis and charger code spaces overlap.
          CHECK(jf["faults"][i]["code"] == f.faults[i].code);
          CHECK(chs_a::IsValidPrefixedFaultCode(
              jf["faults"][i]["code"].get<std::string>()));
          CHECK(jf["faults"][i]["level"] == f.faults[i].level);
        }
      }

      // --- the cap is honoured, never truncated --------------------------
      //
      // The writers return 0 rather than emitting half a message: invalid JSON
      // on a state key makes the consumer see nothing at all, which reads as
      // the robot having stopped reporting.
      char tiny[16];
      CHECK(WriteChassisBasic(b, tiny, sizeof(tiny)) == 0);
      CHECK(WriteChassisMotion(m2, tiny, sizeof(tiny)) == 0);
      CHECK(WriteChassisDevice(d, tiny, sizeof(tiny)) == 0);
      CHECK(WriteChassisFault(f, tiny, sizeof(tiny)) == 0);
    }
  }

  // ---- the triple without a full BasicStatus (13 ASM-4 boundary) ---------
  {
    // Tier 1 gates every axis command on usage_mode (NAV-111), so a consumer
    // that cannot see it cannot tell "the chassis is in the wrong mode" from
    // "we never learned what mode it is in". On the bench those two looked
    // identical: usage_mode came out null for an hour while the robot sat in
    // normal mode, and the state key gave no way to notice.
    //
    // The STRINGS cannot cross the lock-free slot (13 ASM-4); they travel via
    // the bridge's report-side cache instead, so HERE -- caller passing none
    // -- they must come out null rather than zeroed.
    char buf[4096];
    RobotStateInput in;
    in.conn_wire = "connected";
    in.basic = nullptr;            // no full BasicStatus available
    in.has_triple = true;
    in.usage_mode_raw = 1;
    in.motion_state_raw = 17;
    in.gait_raw = 0x3002;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("RobotState triple", buf, n);
    // mutant: drop the has_triple branch -> these go null -> red.
    CHECK(j["usage_mode"] == "navigation");
    CHECK(j["usage_mode_raw"] == 1);
    CHECK(j["motion_state"] == "rl_control");
    CHECK(j["gait_raw"] == 0x3002);
    // The strings stay absent, deliberately.
    CHECK(j["model"].is_null());
    CHECK(j["version"].is_null());
  }

  // ---- never read back is NOT "read back as zero" ------------------------
  {
    // Zero is a real value on all three (normal mode / idle / no gait), so a
    // bare 0 reads as a healthy idle robot on a link that has said nothing.
    // mutant: emit the triple whenever basic is null, ignoring has_triple ->
    // red, and a silent chassis would report itself as idle-and-fine.
    char buf[4096];
    RobotStateInput in;
    in.conn_wire = "connecting";
    in.basic = nullptr;
    in.has_triple = false;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("RobotState no readback", buf, n);
    CHECK(j["usage_mode"].is_null());
    CHECK(j["motion_state"].is_null());
    CHECK(j["gait"].is_null());
  }

  // ---- RobotState.charge / services_ok (11 S4.1) -------------------------
  {
    // Both were absent from RobotState entirely. `charge` is what tells the
    // upper stack whether the robot is on a dock, and 11 S9.11's whole flow
    // keys off it; state/robot (CR-4 relays this key) could not say.
    char buf[8192];
    chs_a::BasicStatus basic;
    basic.charge = 2;                      // charging (11 S9.8.1)
    RobotStateInput in;
    in.basic = &basic;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("robot state charge", buf, n);
    CHECK(j["charge"] == "charging");
    // 21 V-14: services_ok is null, and null is the ANSWER -- the query method
    // itself is unanswered (Q20), so S9.10.2's check cannot be run. A `true`
    // would assert every required chassis service is healthy on the strength
    // of never having looked.
    CHECK(j.contains("services_ok"));
    CHECK(j["services_ok"].is_null());
  }
  {
    // Out of the closed set -> null, never a nearby member (11 S13.6).
    // `idle` in particular would tell the upper stack the robot is free to
    // drive away from a dock.
    char buf[8192];
    chs_a::BasicStatus basic;
    basic.charge = 9;
    RobotStateInput in;
    in.basic = &basic;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("robot state charge odd", buf, n);
    CHECK(j["charge"].is_null());
  }
  {
    // No chassis at all -> also null, for the same reason every other
    // chassis-sourced field is null here.
    char buf[8192];
    RobotStateInput in;                    // basic left null
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("robot state charge cold", buf, n);
    CHECK(j["charge"].is_null());
  }

  // ---- RobotState.charge from the RAW int, not from `basic` --------------
  {
    // The state path has no BasicStatus to offer: it holds std::string and
    // cannot cross the lock-free slot (12 RTC-6). Sourcing charge from `basic`
    // alone therefore made the field null on EVERY state message the process
    // published, while the identical value went out correctly on
    // rt/chassis/power -- measured on the live chassis 2026-09-21.
    char buf[8192];
    RobotStateInput in;
    in.has_charge = true;
    in.charge_raw = 1;                     // going_to_dock (11 S9.8.1)
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("robot state charge raw", buf, n);
    CHECK(j["charge"] == "going_to_dock");
  }
  {
    // has_charge false -> null. "Never reported" is not "idle", and idle in
    // particular tells the upper stack the robot is free to drive away.
    char buf[8192];
    RobotStateInput in;                    // has_charge stays false
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("robot state charge none", buf, n);
    CHECK(j["charge"].is_null());
  }

  // ---- hes / sleep ride the same flag as charge (found 2026-09-21) -------
  {
    // The charge fix above walked PAST these two on the same writer line:
    // state-path RobotState said "hes":null, "sleep":null forever, while the
    // 2 Hz report stream carried both. hes is the HES emergency-stop
    // read-back -- a consumer watching state/robot could not see it at all.
    // Requires has_triple: without the triple the whole block is the cold
    // "never heard from the chassis" nulls, which is a different claim.
    char buf[8192];
    RobotStateInput in;
    in.has_triple = true;
    in.usage_mode_raw = 1;
    in.motion_state_raw = 17;
    in.gait_raw = 0x3002;
    in.has_charge = true;
    in.hes = true;
    in.sleep = false;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("robot state hes/sleep raw", buf, n);
    CHECK(j["hes"] == true);
    CHECK(j["sleep"] == false);
    // And the flag off: null, not false -- "never reported" and "reported
    // as awake, no HES" are different claims, and the second one is exactly
    // what a healthy robot looks like.
    RobotStateInput cold;
    cold.has_triple = true;
    cold.usage_mode_raw = 1;
    cold.motion_state_raw = 17;
    cold.gait_raw = 0x3002;
    const std::size_t m = WriteRobotState(cold, buf, sizeof(buf));
    const Json jc = ParseOrFail("robot state hes/sleep none", buf, m);
    CHECK(jc["hes"].is_null());
    CHECK(jc["sleep"].is_null());
  }

  // ---- RobotState.odom (11 S4.1 / S9.9 / 13 S4.4 (4)) --------------------
  {
    // The gap this closes: WriteRobotState emitted no odom block at all, while
    // 11 S9.9's output table names RobotState.odom.* as one of this process's
    // three outputs and 11 CD-6 / N-2 gate relative-displacement delegation on
    // odom.valid. A ROS nav_msgs/Odometry has no valid field, so before this
    // block the stair-gait rule 13 S4.4 (4) had NO reader anywhere.
    char buf[8192];
    RobotStateInput in;
    OdomSample od;
    od.x = 1.5;
    od.y = -2.5;
    od.yaw = 0.25;
    od.vx = 0.4;
    od.var_x = 0.04;    // sigma 0.2 m
    od.var_yaw = 0.01;  // sigma 0.1 rad
    od.valid = true;
    in.odom = &od;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("robot state odom", buf, n);
    CHECK(j["odom"]["x"] == 1.5);
    CHECK(j["odom"]["yaw_rad"] == 0.25);
    CHECK(j["odom"]["valid"] == true);
    // SIGMA, not variance. The schema defines the unit only through the field
    // name (_m, _rad), and 13 S4.4's numeric table is in sigma throughout.
    // mutant: publish var_x directly -> 0.04 instead of 0.2, which is wrong in
    // the SAFE-looking direction below 1 m and the unsafe one above it.
    CHECK(j["odom"]["cov_xy_m"] == 0.2);
    CHECK(j["odom"]["cov_yaw_rad"] == 0.1);

    // 13 S4.4's detail pair (11 S4.1, 2026-09-21 unfreeze). tau_ms is the
    // SAMPLE's age at integration -- in.odom->tau_s scaled -- not anything
    // measured at publish time; the fourth band's question is "how stale is
    // what we integrated", and a publish-time age answers a different one.
    // The source arrives under the table's own closed names.
    od.tau_s = 0.123;
    in.odom_source = RobotStateInput::OdomSrc::kDrdds;
    const std::size_t n2 = WriteRobotState(in, buf, sizeof(buf));
    const Json j2 = ParseOrFail("robot state odom tau", buf, n2);
    CHECK(j2["odom"]["tau_ms"] == 123.0);
    CHECK(j2["odom"]["source"] == "motion_info_20hz");

    in.odom_source = RobotStateInput::OdomSrc::kMonitor;
    const std::size_t n3 = WriteRobotState(in, buf, sizeof(buf));
    const Json j3 = ParseOrFail("robot state odom src", buf, n3);
    CHECK(j3["odom"]["source"] == "monitor_10hz");

    // No source yet: null, never a third name and never a near neighbour
    // (13 S6.5 ban 1).
    in.odom_source = RobotStateInput::OdomSrc::kNone;
    const std::size_t n4 = WriteRobotState(in, buf, sizeof(buf));
    const Json j4 = ParseOrFail("robot state odom nosrc", buf, n4);
    CHECK(j4["odom"]["source"].is_null());
  }

  // ---- TR-4's computed bit is its own field (11 S4.1 unfreeze) ----------
  {
    // The pair diverges: mode_switching answers MS-3 (our own switch in
    // flight), the TR-4 bit also covers an external transition. A writer
    // that copies one into the other passes any test that sets them equal,
    // so the case sets them APART.
    char buf[8192];
    RobotStateInput in;
    in.mode_switching = false;
    in.motion_state_transitioning = true;
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("robot state transitioning", buf, n);
    CHECK(j["mode_switching"] == false);
    CHECK(j["motion_state_transitioning"] == true);
  }

  // ---- no control period yet: odom is null, not an origin pose -----------
  {
    // An all-zero pose reads as a robot sitting at the origin -- a claim.
    // "We have not integrated anything yet" is an absence. Same distinction
    // the basic/motion blocks make, and for the same reason.
    // mutant: emit a zeroed odom object -> red.
    char buf[8192];
    RobotStateInput in;  // odom left null
    const std::size_t n = WriteRobotState(in, buf, sizeof(buf));
    const Json j = ParseOrFail("robot state no odom", buf, n);
    CHECK(j["odom"].is_null());
  }

  // ---- hello_ack (11 S9.1.4 / 13 ASM-4 (2)) ------------------------------
  {
    // 10 S3.3 Stage 1 does not complete without this answer, and 13 ASM-4 (2)
    // carried it as an open gap. The transport block is the part 13 cares
    // about most: CB-4, DDS-9 and TF-1 each require an EFFECTIVE value here,
    // and each names the same reason -- a wrong codebook, a wrong domain or an
    // unannounced frame assignment all fail as "connected, receiving nothing",
    // which is indistinguishable from a dead network.
    char buf[4096];
    HelloAckInput in;
    in.model = "CA9C";
    in.version = "PRO";
    in.has_triple = true;
    in.usage_mode_raw = 1;
    in.motion_state_raw = 17;
    in.gait_raw = 0x3002;
    in.endpoint = "tcp://10.21.33.103:30003";
    in.codebook = "hex32";
    in.chassis_dds_domain = 0;
    in.uplink_ros_domain = 42;
    in.imu_frame_id = "imu_link";
    in.drdds_available = true;
    in.holonomic = true;
    in.max_vx_mps = 2.0;
    const std::size_t n = WriteHelloAck(in, buf, sizeof(buf));
    const Json j = ParseOrFail("hello_ack", buf, n);
    CHECK(j["type"] == "hello_ack");
    CHECK(j["proto_version"] == "1.0");
    CHECK(j["runtime"]["model"] == "CA9C");
    CHECK(j["runtime"]["usage_mode"] == "navigation");
    // mutant: drop the transport block -> red. DDS-9 calls it the only
    // low-cost way to catch a two-domain misconfiguration.
    CHECK(j["runtime"]["transport"]["codebook"] == "hex32");
    CHECK(j["runtime"]["transport"]["chassis_dds_domain"] == 0);
    CHECK(j["runtime"]["transport"]["uplink_ros_domain"] == 42);
    CHECK(j["runtime"]["transport"]["imu_frame_id"] == "imu_link");
    // The two domains must NOT be equal -- DDS-7 keeps them opposite, and an
    // implementation that echoed one value into both would still satisfy a
    // test that only checked "the field is present".
    CHECK(j["runtime"]["transport"]["chassis_dds_domain"]
          != j["runtime"]["transport"]["uplink_ros_domain"]);
    // Both polarities are asserted (the cold case below checks false), so an
    // implementation that hardcodes either one is red. 11 S9.7 has this field
    // mean "the caller has the drdds package with its reader up" -- with only
    // the false case covered, a constant false would pass while telling the
    // upstream the better odometry source does not exist.
    CHECK(j["runtime"]["drdds_available"] == true);
    // 11 S9.6: spec comes from config, never from the chassis.
    CHECK(j["spec"]["holonomic"] == true);
    CHECK(j["spec"]["max_vx_mps"] == 2.0);
    // 21 V-14: `services` is 恒填 [不可查] for this period, because the query
    // method itself is unanswered (Q20). Present AND null -- an omitted key
    // reads as an older ack, an object of guesses reads as six queried states.
    // mutant: emit a services object, or omit the key -> red.
    CHECK(j["runtime"].contains("services"));
    CHECK(j["runtime"]["services"].is_null());
  }

  // ---- no chassis yet: identity is null, not empty string ----------------
  {
    // An empty model reads as a chassis that answered with a blank name; null
    // says nothing has answered. The upstream cannot check this for itself --
    // hello_ack is the first thing it sees.
    // mutant: emit "" instead of null -> red.
    char buf[4096];
    HelloAckInput in;                 // model / version left null
    const std::size_t n = WriteHelloAck(in, buf, sizeof(buf));
    const Json j = ParseOrFail("hello_ack cold", buf, n);
    CHECK(j["runtime"]["model"].is_null());
    CHECK(j["runtime"]["version"].is_null());
    CHECK(j["runtime"]["usage_mode"].is_null());
    CHECK(j["runtime"]["drdds_available"] == false);
  }

  if (g_failures == 0) {
    std::printf("ALL RT_PAYLOADS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RT_PAYLOADS TEST(S) FAILED\n", g_failures);
  return 1;
}
