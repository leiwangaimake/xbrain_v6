/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_chs_a_reports.cc
 * Brief: Report parsing against the real capture, plus the open-set bans
 *
 * Description:
 * The four report payloads come from test/golden/chs_a_frames.txt, captured off
 * the chassis on 2026-09-15. Everything asserted about them is a value the
 * machine actually sent, which is the only way these cases can disagree with
 * the parser (CLAUDE.md 3.2, form 1).
 *
 * The capture earns its keep immediately: at rest the machine reports Gait 0,
 * and 13 S5.3's gait table does not contain 0. A closed-enum implementation
 * would reject the healthiest report the robot produces. That is not a
 * hypothetical about firmware drift -- it is in the first capture ever taken.
 *
 * What the synthetic cases add, and why each one cannot come from the capture:
 *   * the -2 / 2 trap. 11 S9.2 (written against the older manual) says
 *     soft_estop is 2; the current guide says 2 is joint_damp and soft_estop is
 *     -2. A parser built from 11 reads an emergency stop as a damping state.
 *     The measured machine was never in either state, so only a synthetic
 *     payload can pin it;
 *   * ban 2 -- an unregistered Gait must not cost the report. HES rides in the
 *     same message, so the failure mode is losing the hardware emergency stop
 *     because a gait value was strange;
 *   * the severity mapping's DEFAULT. The capture has an empty ErrorList, so
 *     the machine has never shown us a fault; the whole mapping would be
 *     untested without synthetic entries.
 *
 * One thing deliberately NOT asserted: that a missing leaf field fails the
 * parse. It must not (13 S6.5 ban 2), and there are cases below proving it.
 */

#include "quadruped/chs_a_reports.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "quadruped/chs_a_codec.h"

using namespace quadruped::chs_a;  // NOLINT: test-local, keeps the cases readable

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes FromHex(const std::string& hex) {
  Bytes out;
  out.reserve(hex.size() / 2);
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

// An empty map FAILS rather than letting every capture-driven case below pass
// on nothing -- the "zero tests ran, all green" shape.
std::map<std::string, Bytes> LoadGolden(const std::string& path) {
  std::map<std::string, Bytes> out;
  std::ifstream f(path);
  if (!f) {
    std::printf("FAIL cannot open golden file: %s\n", path.c_str());
    ++g_failures;
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
    ++g_failures;
  }
  return out;
}

// The ASDU of a captured frame: everything past the 16-byte header.
const std::uint8_t* Asdu(const Bytes& frame) { return frame.data() + kHeaderBytes; }
std::size_t AsduLen(const Bytes& frame) { return frame.size() - kHeaderBytes; }

// A synthetic payload. Written as text because that is how a reader compares it
// with the manual, and wrapped the way every real ASDU is.
Bytes Wrap(const std::string& items) {
  const std::string s = "{\"PatrolDevice\":{\"Command\":15728640,\"Items\":" +
                        items + ",\"Time\":\"2026-09-15 14:55:55.457\"," +
                        "\"Type\":1048676}}";
  return Bytes(s.begin(), s.end());
}

}  // namespace

int main(int argc, char** argv) {
  const std::string golden_path =
      (argc >= 2) ? argv[1] : "test/golden/chs_a_frames.txt";
  const auto golden = LoadGolden(golden_path);
  if (golden.empty()) {
    std::printf("%d CHS_A_REPORTS TEST(S) FAILED\n", g_failures);
    return 1;
  }

  // ---- basic status, straight off the wire --------------------------------
  {
    const Bytes& f = golden.at("RX_00100064_00f00000");
    BasicStatus s;
    CHECK(ParseBasicStatus(Asdu(f), AsduLen(f), &s));
    CHECK(s.motion_state.raw == 0);
    CHECK(s.motion_state.known == true);
    CHECK(s.motion_state.label == "idle");
    // *** The finding this capture handed us: at rest the machine reports
    // Gait 0, and 13 S5.3 lists 0x1001/0x1002/0x1003/0x3002/0x3003 -- no 0.
    // A closed enum would reject the most ordinary report the robot sends.
    CHECK(s.gait.raw == 0);
    CHECK(s.gait.known == false);
    CHECK(s.gait.label == "unknown_0x0000");
    CHECK(s.usage_mode.known == true);
    CHECK(s.usage_mode.label == "normal");
    CHECK(s.hes == false);
    CHECK(s.sleep == false);
    // Identity fields, carried through verbatim. DeviceNum is how a field
    // engineer tells two robots apart in a log.
    CHECK(s.device_num == "CM20200041");
    CHECK(s.model == "CA9C");
    CHECK(s.sn == "9f3f39d6638660ec");
    // Version gates the chassis navigation licence (0xE00A is returned on a
    // non-Pro machine), so it is read even though we do not use that stack.
    CHECK(s.version == "PRO");
  }

  // ---- motion status, straight off the wire -------------------------------
  {
    const Bytes& f = golden.at("RX_00100001_00f00000");
    MotionStatus s;
    CHECK(ParseMotionStatus(Asdu(f), AsduLen(f), &s));
    CHECK(s.linear_x == 0.0);
    CHECK(s.linear_y == 0.0);
    CHECK(s.motion_state.label == "idle");
    CHECK(s.gait.known == false);
    // Standing height and the gravity vector: both non-zero on a real machine
    // at rest, and both zero if the parser silently returned defaults. This is
    // the pair that separates "parsed" from "returned an empty struct".
    CHECK(s.height > 0.1 && s.height < 0.2);
    CHECK(s.acc_z > 9.0 && s.acc_z < 10.5);
    CHECK(s.remain_mile > 7.0 && s.remain_mile < 8.0);
  }

  // ---- device status: two packs, SOC on the minimum ----------------------
  {
    const Bytes& f = golden.at("RX_00100002_00f00000");
    DeviceStatus s;
    CHECK(ParseDeviceStatus(Asdu(f), AsduLen(f), &s));
    CHECK(s.batteries.size() == 2);
    CHECK(s.batteries[0].level == 43);
    CHECK(s.batteries[1].level == 44);
    // 11 S9.8.3: the emptier pack decides. Asserting 43 and not 44 is the whole
    // point -- a max() or an average would pass an assertion on "about 43".
    CHECK(s.min_level == 43);
    CHECK(s.any_charging == false);
    CHECK(s.batteries[0].voltage > 70.0);
    // 13 BAT-3: the serial is EMPTY on this machine, which is why the left/
    // right mapping (V-55) cannot be resolved by serial prefix. Asserted so
    // that a future firmware which starts populating it shows up here.
    CHECK(s.batteries[0].serial.empty());
  }

  // ---- fault report: the healthy case is an EMPTY list, and it must parse --
  {
    const Bytes& f = golden.at("RX_0010007f_00f00000");
    FaultReport r;
    // Treating "no faults" as a parse failure would make a healthy link look
    // broken twice a second.
    CHECK(ParseFaultReport(Asdu(f), AsduLen(f), &r));
    CHECK(r.faults.empty());
    CHECK(r.cleared.empty());
  }

  // ---- the -2 / 2 trap ----------------------------------------------------
  {
    // 11 S9.2 says soft_estop is 2. The current guide says 2 is joint_damp and
    // soft_estop is -2 (13 S6.1). A parser built from 11 reads an emergency
    // stop as a damping state -- and reads a damping state as an emergency
    // stop, which is the direction that stops a healthy robot for no reason.
    CHECK(ResolveMotionState(-2).label == "soft_estop");
    CHECK(ResolveMotionState(-2).known == true);
    CHECK(ResolveMotionState(2).label == "joint_damp");
    CHECK(ResolveMotionState(2).known == true);
    // The negative value also exercises the label format: 0x%04X on -2 would
    // render 0xFFFE and send a reader hunting for a code never sent.
    CHECK(ResolveMotionState(-7).known == false);
    CHECK(ResolveMotionState(-7).label == "unknown_-7");
  }

  // ---- ban 1: an unregistered value is never mapped onto a neighbour ------
  {
    // 0x3004 sits right next to stair_agile (0x3003) and shares its high
    // nibble, which is exactly the shape a "nearest known mode" heuristic
    // would swallow. 0x1003 is registered but unreachable on readback, and
    // 16 (cart_move) is registered and MUST resolve even though we never
    // command it: readback is how we learn someone used the factory handset.
    CHECK(ResolveGait(0x3004).known == false);
    CHECK(ResolveGait(0x3004).label == "unknown_0x3004");
    CHECK(ResolveGait(0x1003).label == "stair_standard");
    CHECK(ResolveMotionState(16).label == "cart_move");
    CHECK(ResolveMotionState(17).label == "rl_control");
    CHECK(ResolveUsageMode(1).label == "navigation");
    CHECK(ResolveUsageMode(9).known == false);
  }

  // ---- ban 2: a strange field must not cost the whole report --------------
  {
    // The one that matters. HES travels in the same message as Gait, so a
    // parser that rejects the report over an unregistered gait throws away the
    // hardware emergency stop -- and the robot is then held by a signal
    // nothing upstream can see.
    const Bytes p = Wrap("{\"BasicStatus\":{\"Gait\":39321,\"HES\":1,"
                         "\"MotionState\":17,\"Sleep\":1,"
                         "\"ControlUsageMode\":1}}");
    BasicStatus s;
    CHECK(ParseBasicStatus(p.data(), p.size(), &s));
    CHECK(s.hes == true);
    CHECK(s.sleep == true);
    CHECK(s.motion_state.label == "rl_control");
    CHECK(s.usage_mode.label == "navigation");
    CHECK(s.gait.known == false);
    // ...and ban 3: the raw value survives alongside the label, because
    // "unknown" on its own is not something a field office can act on.
    CHECK(s.gait.raw == 39321);
    CHECK(s.gait.label == "unknown_0x9999");
  }

  // ---- a MISSING leaf field leaves its default and does not fail ----------
  {
    // Same rule seen from the other side: BasicStatus with almost nothing in
    // it still parses, so one firmware that drops a field cannot blind us to
    // the rest of the report.
    const Bytes p = Wrap("{\"BasicStatus\":{\"HES\":1}}");
    BasicStatus s;
    CHECK(ParseBasicStatus(p.data(), p.size(), &s));
    CHECK(s.hes == true);
    CHECK(s.sleep == false);
    CHECK(s.device_num.empty());
  }

  // ---- the ENVELOPE, by contrast, is a real failure -----------------------
  {
    // A payload that is not this report at all must say so, rather than hand
    // back a zeroed struct that reads as "the robot is idle and fine".
    BasicStatus s;
    const Bytes not_json = {'n', 'o', 't', ' ', 'j', 's', 'o', 'n'};
    CHECK(!ParseBasicStatus(not_json.data(), not_json.size(), &s));
    const std::string no_wrap = "{\"BasicStatus\":{\"HES\":1}}";
    CHECK(!ParseBasicStatus(reinterpret_cast<const std::uint8_t*>(no_wrap.data()),
                            no_wrap.size(), &s));
    const Bytes wrong = Wrap("{\"MotionStatus\":{\"LinearX\":1.0}}");
    CHECK(!ParseBasicStatus(wrong.data(), wrong.size(), &s));
    CHECK(!ParseBasicStatus(nullptr, 0, &s));
    CHECK(!ParseBasicStatus(wrong.data(), wrong.size(), nullptr));
    // The measured machine sends ErrorList as an array; anything else is an
    // envelope failure, not an empty fault list.
    FaultReport r;
    const Bytes bad_list = Wrap("{\"ErrorList\":{}}");
    CHECK(!ParseFaultReport(bad_list.data(), bad_list.size(), &r));
  }

  // ---- severity -> level, including the default that matters -------------
  {
    CHECK(SeverityToLevel(true, 3) == "warn");
    CHECK(SeverityToLevel(true, 4) == "degraded");
    CHECK(SeverityToLevel(true, 5) == "fatal");
    // *** 13 S7.3: absent or unrecognised is DEGRADED, not warn. The chassis
    // fault space is open, so an unknown severity is an ordinary event -- and
    // calling it "warn" reports a machine in trouble as merely noisy. Both
    // sides of the default are pinned because "always degraded" would also
    // pass an assertion that only checked the unknown case.
    CHECK(SeverityToLevel(false, 0) == "degraded");
    // The `present` flag is the whole reason this takes two arguments: an
    // ABSENT severity must not be read as whatever number happens to sit in
    // the variable. Without this line the flag could be ignored entirely and
    // every case above would still pass, because the caller zeroes the value
    // when the field is missing.
    CHECK(SeverityToLevel(false, 3) == "degraded");
    CHECK(SeverityToLevel(false, 5) == "degraded");
    CHECK(SeverityToLevel(true, 0) == "degraded");
    CHECK(SeverityToLevel(true, 99) == "degraded");
    CHECK(SeverityToLevel(true, 2) == "degraded");
  }

  // ---- CF-1 / CF-2: the prefix is written here, and validated -------------
  {
    CHECK(FormatChassisFaultCode(0x8001) == "chs:0x8001");
    CHECK(FormatChargeFaultCode(0x1007) == "chg:0x1007");
    // Upper-case hex body, four digits, zero padded: CF-4's dedup key IS this
    // string, so two spellings of one number would be two faults.
    CHECK(FormatChassisFaultCode(0x800f) == "chs:0x800F");
    CHECK(FormatChassisFaultCode(0x12) == "chs:0x0012");
    CHECK(IsValidPrefixedFaultCode("chs:0x8001"));
    CHECK(IsValidPrefixedFaultCode("chg:0x1007"));
    CHECK(IsValidPrefixedFaultCode("chs:0x800f"));  // CF-1 allows either case
    // A bare code cannot be interpreted at all: 0x1007 is "no current at the
    // dock" in one space and undefined in the other.
    CHECK(!IsValidPrefixedFaultCode("0x8001"));
    CHECK(!IsValidPrefixedFaultCode("chs:8001"));
    CHECK(!IsValidPrefixedFaultCode("xyz:0x8001"));
    CHECK(!IsValidPrefixedFaultCode("chs:0x801"));
    CHECK(!IsValidPrefixedFaultCode("chs:0x80011"));
    CHECK(!IsValidPrefixedFaultCode("chs:0xZZZZ"));
    CHECK(!IsValidPrefixedFaultCode(""));
  }

  // ---- faults and cleared are split by Type, and both carry the prefix ----
  {
    const Bytes p = Wrap(
        "{\"ErrorList\":["
        "{\"Code\":32769,\"Name\":\"motor_over_temperature\",\"Type\":1,"
        " \"Severities\":5,\"Grouped\":true,\"Resources\":[\"11\"],"
        " \"Source\":[\"rl_deploy\"],\"SourceIds\":[\"motion_master#0\"],"
        " \"Timestamp\":{\"Sec\":1789455340,\"Nanosec\":500},"
        " \"Details\":\"motor 11 at 97C\"},"
        "{\"Code\":32783,\"Name\":\"joint_position_over_limit\",\"Type\":1},"
        "{\"Code\":33025,\"Name\":\"battery_low\",\"Type\":2,\"Severities\":3}"
        "]}");
    FaultReport r;
    CHECK(ParseFaultReport(p.data(), p.size(), &r));
    CHECK(r.faults.size() == 2);
    CHECK(r.cleared.size() == 1);
    if (r.faults.size() == 2 && r.cleared.size() == 1) {
      CHECK(r.faults[0].code == "chs:0x8001");
      CHECK(r.faults[0].level == "fatal");
      CHECK(r.faults[0].grouped == true);
      CHECK(r.faults[0].resources.size() == 1 && r.faults[0].resources[0] == "11");
      CHECK(r.faults[0].source[0] == "rl_deploy");
      CHECK(r.faults[0].source_ids[0] == "motion_master#0");
      CHECK(r.faults[0].since_sec == 1789455340);
      CHECK(r.faults[0].since_nanosec == 500);
      CHECK(r.faults[0].details == "motor 11 at 97C");
      // *** 0x800F is the code the vendor's own EXAMPLE uses while the table
      // lists 0x8014 for the same fault name (13 S7.3 reason 3). It is the
      // proof that the code space is open, and it must arrive with its name --
      // which is the only readable clue an unregistered code has.
      CHECK(r.faults[1].code == "chs:0x800F");
      CHECK(r.faults[1].name == "joint_position_over_limit");
      CHECK(r.faults[1].level == "degraded");  // no Severities -> not warn
      // CF-3: a cleared fault keeps the identical spelling it was raised with.
      CHECK(r.cleared[0].code == "chs:0x8101");
      CHECK(r.cleared[0].level == "warn");
    }
    // Every code this parser emits must satisfy CF-1, including the ones built
    // from a code the table has never seen.
    for (const FaultEntry& f : r.faults) CHECK(IsValidPrefixedFaultCode(f.code));
    for (const FaultEntry& f : r.cleared) CHECK(IsValidPrefixedFaultCode(f.code));
  }

  // ---- a wrong-typed field falls back rather than throwing ----------------
  {
    // A firmware that sent Gait as a string would otherwise throw out of the
    // middle of the parse and cost the whole report -- ban 2 again, reached
    // through a type error rather than through an unknown value.
    const Bytes p = Wrap("{\"BasicStatus\":{\"Gait\":\"flat\",\"HES\":1,"
                         "\"MotionState\":4,\"DeviceNum\":7}}");
    BasicStatus s;
    CHECK(ParseBasicStatus(p.data(), p.size(), &s));
    CHECK(s.hes == true);
    CHECK(s.motion_state.label == "prone");
    CHECK(s.gait.raw == 0);          // the documented default, not the string
    CHECK(s.device_num.empty());     // a number where a string belongs
  }

  if (g_failures == 0) {
    std::printf("ALL CHS_A_REPORTS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CHS_A_REPORTS TEST(S) FAILED\n", g_failures);
  return 1;
}
