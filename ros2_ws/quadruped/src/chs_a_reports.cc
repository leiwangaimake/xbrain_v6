/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_a_reports.cc
 * Brief: Report ASDU parsing implementation (see chs_a_reports.h)
 *
 * Description:
 * Three things in here are easy to get subtly wrong, so each is written once
 * and used everywhere:
 *
 *   * EVERY field read goes through the Get* helpers below, which return the
 *     caller's default when the key is absent or holds the wrong JSON type.
 *     The obvious alternative, json.at("Gait").get<int>(), throws -- and a
 *     throw here costs the whole report including HES (13 S6.5 ban 2).
 *   * the mode values are looked up in ONE table each, and the lookup returns
 *     the raw number alongside the label. Two tables (one for parsing, one for
 *     display) is how a value ends up registered in one place and unknown in
 *     the other.
 *   * the fault level comes from the severity the chassis sent, never from a
 *     local table keyed on the code. That inversion is what lets the code space
 *     stay open without the level becoming a guess.
 *
 * A note on the numbers, because they contradict a document that is still
 * authoritative elsewhere: the mode values follow the CURRENT vendor guide and
 * the 2026-09-15 measurements (13 S6.1), not 11 S9.2's table, which was written
 * against the older manual. In that older table soft_estop is 2; here 2 is
 * joint_damp and soft_estop is -2. Implementing the old one maps an emergency
 * stop onto a damping state. 11 is a frozen surface (F-5) and is NOT edited to
 * match; the divergence is registered in 13 S14 and contained here.
 */

#include "quadruped/chs_a_reports.h"

#include <cstdio>

#include "nlohmann/json.hpp"
#include "xbrain/enums/closed_sets.h"

namespace quadruped {
namespace chs_a {
namespace {

using Json = nlohmann::json;

// One row of an open-set table. Deliberately not a std::map: these tables have
// under a dozen entries, they are searched a few times a second, and an array
// of PODs is something a reviewer can compare against the manual line by line.
struct CodeName {
  std::int64_t value;
  const char* name;
};

// 13 S5.2 + S6.1, named per 11 S9.2.4. Note -2: see the file comment.
constexpr CodeName kMotionStates[] = {
    {-2, "soft_estop"},   // NOT 2; 2 is joint_damp
    {0, "idle"},
    {1, "stand"},
    {2, "joint_damp"},
    {3, "boot_damp"},
    {4, "prone"},
    {5, "zero_cal"},
    {16, "cart_move"},    // 13 V-50: suggests a wheel-legged machine; read
                          // back and reported, never commanded
    {17, "rl_control"},   // the state autonomous motion requires
    {0x1001, "damped_prone"},
};

// 13 S5.3. The gaps are the point: 0x1003 can be commanded but never read back,
// 0x1002 can be read back but never commanded, and the measured machine reports
// 0 at rest -- a value this table does not contain and deliberately does not
// invent a name for.
constexpr CodeName kGaits[] = {
    {0x1001, "basic"},
    {0x1002, "platform"},        // read-only
    {0x1003, "stair_standard"},  // command-only, not sent this batch (GS-1)
    {0x3002, "flat"},
    {0x3003, "stair_agile"},
};

// The stair members of the table above, kept ADJACENT to it: 13 S4.4 (4) makes
// the odometry's valid flag turn on this membership, so a gait added to kGaits
// without a look at this list would silently publish wheel odometry as valid
// on a staircase. Both are listed even though GS-1 forbids US from commanding
// 0x1003 -- GS-3 says it verbatim: "我方不发" 不等于 "它不会出现" (the factory
// handset can set it, and the read-back path still resolves it).
constexpr std::int64_t kStairGaits[] = {0x1003, 0x3003};

// 11 S9.2.4 / 13 C-02.
constexpr CodeName kUsageModes[] = {
    {0, "normal"},
    {1, "navigation"},
    {2, "assist"},
};

// Render the label for a value that is not in its table. The documented form is
// unknown_0x%04X (13 S6.5), which assumes a value that fits four hex digits.
// motion_state breaks that assumption in one direction only -- soft_estop is
// negative -- so a negative or oversized value is rendered in decimal instead
// of being mangled into 0xFFFE, which would send a reader looking for a code
// that was never sent.
std::string UnknownLabel(std::int64_t raw) {
  char buf[40];
  if (raw >= 0 && raw <= 0xFFFF) {
    std::snprintf(buf, sizeof(buf), "unknown_0x%04X", static_cast<unsigned>(raw));
  } else {
    std::snprintf(buf, sizeof(buf), "unknown_%lld", static_cast<long long>(raw));
  }
  return std::string(buf);
}

template <std::size_t N>
OpenSetValue Resolve(const CodeName (&table)[N], std::int64_t raw) {
  OpenSetValue v;
  v.raw = raw;
  for (std::size_t i = 0; i < N; ++i) {
    if (table[i].value == raw) {
      v.known = true;
      v.label = table[i].name;
      return v;
    }
  }
  // Ban 1: no nearest-match, no default member. The caller is told plainly
  // that this value is not modelled, and the raw number rides along.
  v.known = false;
  v.label = UnknownLabel(raw);
  return v;
}

// ---- type-safe field reads ------------------------------------------------
//
// Each returns `dflt` when the key is absent OR holds a different JSON type.
// The type check is not pedantry: the chassis sends Gait as a number and Name
// as a string, and a firmware that swapped one for the other would otherwise
// throw out of the middle of a parse and cost the entire report.

std::int64_t GetInt(const Json& j, const char* key, std::int64_t dflt) {
  auto it = j.find(key);
  if (it == j.end() || !it->is_number()) return dflt;
  return it->get<std::int64_t>();
}

double GetDouble(const Json& j, const char* key, double dflt) {
  auto it = j.find(key);
  if (it == j.end() || !it->is_number()) return dflt;
  return it->get<double>();
}

std::string GetString(const Json& j, const char* key) {
  auto it = j.find(key);
  if (it == j.end() || !it->is_string()) return std::string();
  return it->get<std::string>();
}

bool GetBool(const Json& j, const char* key, bool dflt) {
  auto it = j.find(key);
  if (it == j.end()) return dflt;
  // The chassis writes booleans both ways: `charge` is a real JSON bool in the
  // battery array, while HES and Sleep arrive as 0/1 integers. Accepting both
  // here is not laxity -- it is what the wire actually carries.
  if (it->is_boolean()) return it->get<bool>();
  if (it->is_number()) return it->get<std::int64_t>() != 0;
  return dflt;
}

std::vector<std::string> GetStringArray(const Json& j, const char* key) {
  std::vector<std::string> out;
  auto it = j.find(key);
  if (it == j.end() || !it->is_array()) return out;
  for (const Json& e : *it) {
    // A non-string element is rendered rather than dropped: Resources arrives
    // as ["11"] on this firmware but the guide shows joint numbers, and losing
    // the element would leave a fault with no location at all.
    out.push_back(e.is_string() ? e.get<std::string>() : e.dump());
  }
  return out;
}

// Parse the ASDU and descend to PatrolDevice.Items, the object every report
// puts its payload in. Returns nullptr when the envelope is not what it must
// be -- which is a real failure, unlike a missing leaf field.
const Json* ItemsOf(const Json& root) {
  auto pd = root.find("PatrolDevice");
  if (pd == root.end() || !pd->is_object()) return nullptr;
  auto items = pd->find("Items");
  if (items == pd->end() || !items->is_object()) return nullptr;
  return &(*items);
}

bool ParseRoot(const std::uint8_t* asdu, std::size_t len, Json* out) {
  if (asdu == nullptr || len == 0) return false;
  // Non-throwing parse: a malformed frame is a link event, not an exception to
  // unwind through the receive thread.
  *out = Json::parse(asdu, asdu + len, nullptr, false);
  // *** EQUIVALENT-MUTANT NOTE (CLAUDE.md 7.2.1). No test can make this line
  // fail, and that is worth stating rather than leaving for the next person to
  // rediscover. Every public parse function calls ItemsOf immediately after
  // this, and find() on a discarded value -- or on any non-object -- returns
  // end() without throwing, so the report is refused one step later with the
  // same answer. Measured against the vendored nlohmann build for "not json",
  // "[1,2]", "\"x\"", "123" and "null".
  //
  // It stays because it states WHERE "is this JSON at all" is decided. The
  // alternative is a function whose correctness rests on what find() happens to
  // do to a discarded value, which is a library detail and not a contract.
  // scripts/ci/cxx_mutants.py carries a mutant marked expect="equivalent" for
  // this line: if a future edit makes it load-bearing, that mutant starts being
  // killed and the runner says so.
  return !out->is_discarded() && out->is_object();
}

}  // namespace

OpenSetValue ResolveMotionState(std::int64_t raw) { return Resolve(kMotionStates, raw); }
OpenSetValue ResolveGait(std::int64_t raw) { return Resolve(kGaits, raw); }

bool IsStairGait(std::int64_t raw) {
  for (const std::int64_t g : kStairGaits) {
    if (g == raw) return true;
  }
  // An UNREGISTERED value answers false, and that is deliberate rather than
  // careless. The conservative-looking alternative -- treat anything unknown as
  // a staircase -- would fire on the most ordinary report there is: 13 V-66
  // measured the chassis reporting Gait 0 at rest, a value kGaits does not
  // contain, on every boot before RL control. Inflating covariance and clearing
  // valid on a standing robot teaches the consumer to ignore the flag, which
  // costs more than it buys on the one gait it was built for.
  return false;
}
OpenSetValue ResolveUsageMode(std::int64_t raw) { return Resolve(kUsageModes, raw); }

std::string SeverityToLevel(bool present, std::int64_t severity) {
  namespace sets = hachist::xbrain::enums;
  if (present) {
    if (severity == 3) return std::string(sets::kFaultLevel[0]);  // warn
    if (severity == 4) return std::string(sets::kFaultLevel[1]);  // degraded
    if (severity == 5) return std::string(sets::kFaultLevel[2]);  // fatal
  }
  // 13 S7.3: absent or unrecognised maps to degraded, not warn. The mapping is
  // asymmetric on purpose -- the code space is open, so an unknown severity is
  // an ordinary event, and calling it "warn" reports a machine in trouble as
  // merely noisy.
  return std::string(sets::kFaultLevel[1]);
}

namespace {

// CF-1's body format: exactly four hex digits, UPPER case. Shared by both
// spaces so they cannot drift into different spellings of the same number --
// which would defeat CF-4's dedup key, since that key IS the formatted string.
std::string FormatWithPrefix(const char* prefix, std::int64_t code) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%s:0x%04X", prefix,
                static_cast<unsigned>(code & 0xFFFF));
  return std::string(buf);
}

}  // namespace

std::string FormatChassisFaultCode(std::int64_t code) {
  return FormatWithPrefix("chs", code);
}

std::string FormatChargeFaultCode(std::int64_t code) {
  return FormatWithPrefix("chg", code);
}

bool IsValidPrefixedFaultCode(const std::string& code) {
  // Hand-checked rather than a regex: this runs per fault per report, and the
  // pattern is fixed at ten characters. Written as the shape it accepts so a
  // reader can compare it with CF-1 directly: ^(chs|chg):0x[0-9A-Fa-f]{4}$
  if (code.size() != 10) return false;
  const bool ns_ok = (code.compare(0, 4, "chs:") == 0) ||
                     (code.compare(0, 4, "chg:") == 0);
  if (!ns_ok) return false;
  if (code[4] != '0' || code[5] != 'x') return false;
  for (std::size_t i = 6; i < 10; ++i) {
    const char c = code[i];
    const bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
                     (c >= 'A' && c <= 'F');
    if (!hex) return false;
  }
  return true;
}

bool ParseBasicStatus(const std::uint8_t* asdu, std::size_t len, BasicStatus* out) {
  if (out == nullptr) return false;
  Json root;
  if (!ParseRoot(asdu, len, &root)) return false;
  const Json* items = ItemsOf(root);
  if (items == nullptr) return false;
  auto bs = items->find("BasicStatus");
  if (bs == items->end() || !bs->is_object()) return false;

  BasicStatus s;
  s.motion_state = ResolveMotionState(GetInt(*bs, "MotionState", 0));
  s.gait = ResolveGait(GetInt(*bs, "Gait", 0));
  s.usage_mode = ResolveUsageMode(GetInt(*bs, "ControlUsageMode", 0));
  s.hes = GetBool(*bs, "HES", false);
  s.sleep = GetBool(*bs, "Sleep", false);
  s.charge = static_cast<int>(GetInt(*bs, "Charge", 0));
  s.status_code = static_cast<int>(GetInt(*bs, "StatusCode", 0));
  s.robot_type = static_cast<int>(GetInt(*bs, "RobotType", 0));
  s.direction = static_cast<int>(GetInt(*bs, "Direction", 0));
  s.ooa = static_cast<int>(GetInt(*bs, "OOA", 0));
  s.ota_status = static_cast<int>(GetInt(*bs, "OTAStatus", 0));
  s.power_management = static_cast<int>(GetInt(*bs, "PowerManagement", 0));
  s.reset_joints_zero = static_cast<int>(GetInt(*bs, "ResetJointsZero", 0));
  s.device_num = GetString(*bs, "DeviceNum");
  s.model = GetString(*bs, "Model");
  s.sn = GetString(*bs, "Sn");
  s.version = GetString(*bs, "Version");
  *out = s;
  return true;
}

bool ParseMotionStatus(const std::uint8_t* asdu, std::size_t len, MotionStatus* out) {
  if (out == nullptr) return false;
  Json root;
  if (!ParseRoot(asdu, len, &root)) return false;
  const Json* items = ItemsOf(root);
  if (items == nullptr) return false;
  auto ms = items->find("MotionStatus");
  if (ms == items->end() || !ms->is_object()) return false;

  MotionStatus s;
  s.motion_state = ResolveMotionState(GetInt(*ms, "MotionState", 0));
  s.gait = ResolveGait(GetInt(*ms, "Gait", 0));
  s.linear_x = GetDouble(*ms, "LinearX", 0.0);
  s.linear_y = GetDouble(*ms, "LinearY", 0.0);
  s.angular_z = GetDouble(*ms, "AngularZ", 0.0);
  s.roll = GetDouble(*ms, "Roll", 0.0);
  s.pitch = GetDouble(*ms, "Pitch", 0.0);
  s.yaw = GetDouble(*ms, "Yaw", 0.0);
  s.height = GetDouble(*ms, "Height", 0.0);
  s.payload = GetDouble(*ms, "Payload", 0.0);
  s.remain_mile = GetDouble(*ms, "RemainMile", 0.0);
  s.acc_x = GetDouble(*ms, "AccX", 0.0);
  s.acc_y = GetDouble(*ms, "AccY", 0.0);
  s.acc_z = GetDouble(*ms, "AccZ", 0.0);
  s.omega_x = GetDouble(*ms, "OmegaX", 0.0);
  s.omega_y = GetDouble(*ms, "OmegaY", 0.0);
  s.omega_z = GetDouble(*ms, "OmegaZ", 0.0);
  *out = s;
  return true;
}

bool ParseDeviceStatus(const std::uint8_t* asdu, std::size_t len, DeviceStatus* out) {
  if (out == nullptr) return false;
  Json root;
  if (!ParseRoot(asdu, len, &root)) return false;
  const Json* items = ItemsOf(root);
  if (items == nullptr) return false;
  auto bl = items->find("BatteryList");
  if (bl == items->end() || !bl->is_array()) return false;

  DeviceStatus s;
  bool first = true;
  for (const Json& e : *bl) {
    if (!e.is_object()) continue;
    BatteryEntry b;
    b.level = static_cast<int>(GetInt(e, "BatteryLevel", 0));
    b.voltage = GetDouble(e, "Voltage", 0.0);
    b.temperature_c = GetDouble(e, "battery_temperature", 0.0);
    b.charging = GetBool(e, "charge", false);
    b.serial = GetString(e, "serial");
    // An empty slot reports 0 V. A pack that can deliver anything cannot, so
    // voltage is the discriminator; the -273 temperature (the absolute-zero
    // "no sensor" sentinel) and level 0 corroborate it but are not the test --
    // a genuinely flat pack also reads level 0, and telling those two apart is
    // the whole point of this flag.
    b.present = b.voltage > 0.0;
    if (b.present) ++s.present_count;
    if (b.charging) s.any_charging = true;
    // The minimum over the packs, per 11 S9.8.3. Seeded from the first entry
    // rather than from 0 or 100: seeding from 0 would report a full robot as
    // empty when the list is short, and seeding from 100 would hide an empty
    // pack if the list came back with a single malformed entry.
    if (first || b.level < s.min_level) {
      s.min_level = b.level;
      first = false;
    }
    s.batteries.push_back(b);
  }
  *out = s;
  return true;
}

bool ParseFaultReport(const std::uint8_t* asdu, std::size_t len, FaultReport* out) {
  if (out == nullptr) return false;
  Json root;
  if (!ParseRoot(asdu, len, &root)) return false;
  const Json* items = ItemsOf(root);
  if (items == nullptr) return false;
  auto el = items->find("ErrorList");
  // An EMPTY list is the healthy case and must parse: the measured machine
  // sends "ErrorList":[] twice a second. Treating "no faults" as a parse
  // failure would make a healthy link look broken.
  if (el == items->end() || !el->is_array()) return false;

  FaultReport r;
  for (const Json& e : *el) {
    if (!e.is_object()) continue;
    FaultEntry f;
    f.code = FormatChassisFaultCode(GetInt(e, "Code", 0));
    f.name = GetString(e, "Name");
    // Details has no schema on either side of the link (13 S7.3), so it is
    // carried as text. Rendering a non-string with dump() keeps a structured
    // Details readable instead of silently empty.
    auto det = e.find("Details");
    if (det != e.end()) {
      f.details = det->is_string() ? det->get<std::string>() : det->dump();
    }
    f.grouped = GetBool(e, "Grouped", false);
    f.resources = GetStringArray(e, "Resources");
    f.source = GetStringArray(e, "Source");
    f.source_ids = GetStringArray(e, "SourceIds");
    auto ts = e.find("Timestamp");
    if (ts != e.end() && ts->is_object()) {
      f.since_sec = GetInt(*ts, "Sec", 0);
      f.since_nanosec = GetInt(*ts, "Nanosec", 0);
    }
    f.type = static_cast<int>(GetInt(e, "Type", 0));
    // Severities is per-fault and travels with the report. Presence matters:
    // an absent field and a value of 3 mean different things, and collapsing
    // them would turn every unlabelled fault into a warning.
    auto sev = e.find("Severities");
    bool sev_present = false;
    std::int64_t sev_value = 0;
    if (sev != e.end()) {
      if (sev->is_number()) {
        sev_present = true;
        sev_value = sev->get<std::int64_t>();
      } else if (sev->is_array() && !sev->empty() && sev->front().is_number()) {
        // The guide shows this as an array alongside the code list. The first
        // element is the severity of THIS fault, since the two arrays are
        // parallel and the entry has already been split out.
        sev_present = true;
        sev_value = sev->front().get<std::int64_t>();
      }
    }
    f.level = SeverityToLevel(sev_present, sev_value);
    // CF-3: a cleared fault carries the byte-identical code string it was
    // raised with, so the two lists can be matched without re-formatting.
    if (f.type == 2) {
      r.cleared.push_back(f);
    } else {
      r.faults.push_back(f);
    }
  }
  *out = r;
  return true;
}

}  // namespace chs_a
}  // namespace quadruped
