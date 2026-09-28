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
std::string unknown_label(std::int64_t raw) {
  char buf[40];
  if (raw >= 0 && raw <= 0xFFFF) {
    std::snprintf(buf, sizeof(buf), "unknown_0x%04X", static_cast<unsigned>(raw));
  } else {
    std::snprintf(buf, sizeof(buf), "unknown_%lld", static_cast<long long>(raw));
  }
  return std::string(buf);
}

template <std::size_t N>
OpenSetValue resolve(const CodeName (&table)[N], std::int64_t raw) {
  OpenSetValue v;
  v.raw = raw;
  for (std::size_t i = 0; i < N; ++i) {
    if (table[i].value == raw) {
      v.known = true;
      v.label = table[i].name;
      return v;
    }
  }
  // Ban 1 (13 QD-6): no nearest-match, no default member, and no E_SCHEMA
  // discard either. The caller is told plainly that this value is not
  // modelled, and the raw number rides along.
  v.known = false;
  v.label = unknown_label(raw);
  return v;
}

// ---- type-safe field reads ------------------------------------------------
//
// Each returns `dflt` when the key is absent OR holds a different JSON type.
// The type check is not pedantry: the chassis sends Gait as a number and Name
// as a string, and a firmware that swapped one for the other would otherwise
// throw out of the middle of a parse and cost the entire report.

std::int64_t get_int(const Json& j, const char* key, std::int64_t dflt) {
  auto it = j.find(key);
  if (it == j.end() || !it->is_number()) return dflt;
  return it->get<std::int64_t>();
}

double get_double(const Json& j, const char* key, double dflt) {
  auto it = j.find(key);
  if (it == j.end() || !it->is_number()) return dflt;
  return it->get<double>();
}

std::string get_string(const Json& j, const char* key) {
  auto it = j.find(key);
  if (it == j.end() || !it->is_string()) return std::string();
  return it->get<std::string>();
}

bool get_bool(const Json& j, const char* key, bool dflt) {
  auto it = j.find(key);
  if (it == j.end()) return dflt;
  // The chassis writes booleans both ways: `charge` is a real JSON bool in the
  // battery array, while HES and Sleep arrive as 0/1 integers. Accepting both
  // here is not laxity -- it is what the wire actually carries.
  if (it->is_boolean()) return it->get<bool>();
  if (it->is_number()) return it->get<std::int64_t>() != 0;
  return dflt;
}

// A FIXED-length double array. Returns false unless the key is an array of
// exactly `n` numbers: a short Joint list is a malformed report, not a robot
// with fewer legs, and filling the tail with zeros would publish a leg folded
// flat at the origin. Partial reads are the shape that makes a protocol change
// look like a mechanical fault.
bool get_fixed_double_array(const Json& j, const char* key, double* out,
                         std::size_t n) {
  auto it = j.find(key);
  if (it == j.end() || !it->is_array() || it->size() != n) return false;
  std::size_t i = 0;
  for (const Json& e : *it) {
    if (!e.is_number()) return false;
    out[i++] = e.get<double>();
  }
  return true;
}

// A variable-length int array. Unlike the fixed one this tolerates any length:
// the CPU arrays are per-core and the core count is the chassis's business,
// not ours -- pinning it here would turn a different SoC into a parse failure.
std::vector<int> get_int_array(const Json& j, const char* key) {
  std::vector<int> out;
  auto it = j.find(key);
  if (it == j.end() || !it->is_array()) return out;
  for (const Json& e : *it) {
    if (e.is_number()) out.push_back(static_cast<int>(e.get<std::int64_t>()));
  }
  return out;
}

std::vector<std::string> get_string_array(const Json& j, const char* key) {
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

// One CPU host out of the CPU group. A helper rather than two copies: the
// blocks are identical in shape and the only thing that differs is which key
// they live under, so a copy is two places for a field name to be missed.
void parse_cpu_host(const Json& cpu, const char* key, CpuHostStatus* out) {
  auto h = cpu.find(key);
  if (h == cpu.end() || !h->is_object()) return;
  out->valid = true;
  out->soc_id = get_string(*h, "SocId");
  out->avg_util_pct = static_cast<int>(get_int(*h, "AvgUtil", 0));
  out->package_temp_c = static_cast<int>(get_int(*h, "PackageTemp", 0));
  out->util_pct = get_int_array(*h, "Util");
  out->temps_c = get_int_array(*h, "Temps");
  out->cur_freq_khz = get_int_array(*h, "CurFreqKhz");
  out->hw_max_freq_khz = get_int_array(*h, "HwMaxFreqKhz");
  out->hw_min_freq_khz = get_int_array(*h, "HwMinFreqKhz");
  out->gov_policy = get_string_array(*h, "GovPolicy");
}

// Parse the ASDU and descend to PatrolDevice.Items, the object every report
// puts its payload in. Returns nullptr when the envelope is not what it must
// be -- which is a real failure, unlike a missing leaf field.
const Json* items_of(const Json& root) {
  auto pd = root.find("PatrolDevice");
  if (pd == root.end() || !pd->is_object()) return nullptr;
  auto items = pd->find("Items");
  if (items == pd->end() || !items->is_object()) return nullptr;
  return &(*items);
}

bool parse_root(const std::uint8_t* asdu, std::size_t len, Json* out) {
  if (asdu == nullptr || len == 0) return false;
  // Non-throwing parse: a malformed frame is a link event, not an exception to
  // unwind through the receive thread.
  *out = Json::parse(asdu, asdu + len, nullptr, false);
  // *** EQUIVALENT-MUTANT NOTE (CLAUDE.md 7.2.1). No test can make this line
  // fail, and that is worth stating rather than leaving for the next person to
  // rediscover. Every public parse function calls items_of immediately after
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

OpenSetValue resolve_motion_state(std::int64_t raw) { return resolve(kMotionStates, raw); }
OpenSetValue resolve_gait(std::int64_t raw) { return resolve(kGaits, raw); }

bool gait_value_by_name(const std::string& name, std::int64_t* out) {
  // The SAME table resolve_gait reads, walked the other way. A second name
  // table would be a second place for a gait to be spelled, and the two only
  // have to disagree once -- 13 QC-9's list is configured by name and compared
  // against read-back VALUES, so a mismatch there disarms PR-1 silently.
  for (const CodeName& g : kGaits) {
    if (name == g.name) {
      *out = g.value;
      return true;
    }
  }
  return false;
}

bool is_stair_gait(std::int64_t raw) {
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
OpenSetValue resolve_usage_mode(std::int64_t raw) { return resolve(kUsageModes, raw); }

std::string severity_to_level(bool present, std::int64_t severity) {
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
std::string format_with_prefix(const char* prefix, std::int64_t code) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%s:0x%04X", prefix,
                static_cast<unsigned>(code & 0xFFFF));
  return std::string(buf);
}

}  // namespace

std::string format_chassis_fault_code(std::int64_t code) {
  return format_with_prefix("chs", code);
}

std::string format_charge_fault_code(std::int64_t code) {
  return format_with_prefix("chg", code);
}

bool is_valid_prefixed_fault_code(const std::string& code) {
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

bool parse_basic_status(const std::uint8_t* asdu, std::size_t len, BasicStatus* out) {
  if (out == nullptr) return false;
  Json root;
  if (!parse_root(asdu, len, &root)) return false;
  const Json* items = items_of(root);
  if (items == nullptr) return false;
  auto bs = items->find("BasicStatus");
  if (bs == items->end() || !bs->is_object()) return false;

  BasicStatus s;
  s.motion_state = resolve_motion_state(get_int(*bs, "MotionState", 0));
  s.gait = resolve_gait(get_int(*bs, "Gait", 0));
  s.usage_mode = resolve_usage_mode(get_int(*bs, "ControlUsageMode", 0));
  s.hes = get_bool(*bs, "HES", false);
  s.sleep = get_bool(*bs, "Sleep", false);
  s.charge = static_cast<int>(get_int(*bs, "Charge", 0));
  s.status_code = static_cast<int>(get_int(*bs, "StatusCode", 0));
  s.robot_type = static_cast<int>(get_int(*bs, "RobotType", 0));
  s.direction = static_cast<int>(get_int(*bs, "Direction", 0));
  s.ooa = static_cast<int>(get_int(*bs, "OOA", 0));
  s.ota_status = static_cast<int>(get_int(*bs, "OTAStatus", 0));
  s.power_management = static_cast<int>(get_int(*bs, "PowerManagement", 0));
  s.reset_joints_zero = static_cast<int>(get_int(*bs, "ResetJointsZero", 0));
  s.device_num = get_string(*bs, "DeviceNum");
  s.model = get_string(*bs, "Model");
  s.sn = get_string(*bs, "Sn");
  s.version = get_string(*bs, "Version");
  *out = s;
  return true;
}

bool parse_motion_status(const std::uint8_t* asdu, std::size_t len, MotionStatus* out) {
  if (out == nullptr) return false;
  Json root;
  if (!parse_root(asdu, len, &root)) return false;
  const Json* items = items_of(root);
  if (items == nullptr) return false;
  auto ms = items->find("MotionStatus");
  if (ms == items->end() || !ms->is_object()) return false;

  MotionStatus s;
  s.motion_state = resolve_motion_state(get_int(*ms, "MotionState", 0));
  s.gait = resolve_gait(get_int(*ms, "Gait", 0));
  s.linear_x = get_double(*ms, "LinearX", 0.0);
  s.linear_y = get_double(*ms, "LinearY", 0.0);
  s.angular_z = get_double(*ms, "AngularZ", 0.0);
  s.roll = get_double(*ms, "Roll", 0.0);
  s.pitch = get_double(*ms, "Pitch", 0.0);
  s.yaw = get_double(*ms, "Yaw", 0.0);
  s.height = get_double(*ms, "Height", 0.0);
  // Payload is deliberately NOT read -- see MotionStatus in the header.
  s.remain_mile = get_double(*ms, "RemainMile", 0.0);
  s.acc_x = get_double(*ms, "AccX", 0.0);
  s.acc_y = get_double(*ms, "AccY", 0.0);
  s.acc_z = get_double(*ms, "AccZ", 0.0);
  s.omega_x = get_double(*ms, "OmegaX", 0.0);
  s.omega_y = get_double(*ms, "OmegaY", 0.0);
  s.omega_z = get_double(*ms, "OmegaZ", 0.0);
  // MotorStatus is a SIBLING group of MotionStatus inside Items, not a member
  // of it -- reading it off `ms` finds nothing and leaves joints silently
  // empty, which is exactly the state this file was in before 2026-09-28.
  auto mo = items->find("MotorStatus");
  if (mo != items->end() && mo->is_object()) {
    s.has_joints = get_fixed_double_array(*mo, "Joint", s.joint, 16);
  }
  *out = s;
  return true;
}

bool parse_device_status(const std::uint8_t* asdu, std::size_t len, DeviceStatus* out) {
  if (out == nullptr) return false;
  Json root;
  if (!parse_root(asdu, len, &root)) return false;
  const Json* items = items_of(root);
  if (items == nullptr) return false;
  auto bl = items->find("BatteryList");
  if (bl == items->end() || !bl->is_array()) return false;

  DeviceStatus s;
  for (const Json& e : *bl) {
    if (!e.is_object()) continue;
    BatteryEntry b;
    b.level = static_cast<int>(get_int(e, "BatteryLevel", 0));
    b.voltage = get_double(e, "Voltage", 0.0);
    b.temperature_c = get_double(e, "battery_temperature", 0.0);
    b.charging = get_bool(e, "charge", false);
    b.serial = get_string(e, "serial");
    // An empty slot reports 0 V. A pack that can deliver anything cannot, so
    // voltage is the discriminator; the -273 temperature (the absolute-zero
    // "no sensor" sentinel) and level 0 corroborate it but are not the test --
    // a genuinely flat pack also reads level 0, and telling those two apart is
    // the whole point of this flag.
    b.present = b.voltage > 0.0;
    if (b.charging) s.any_charging = true;
    // The minimum over the PRESENT packs, per 11 S4.2 CHG-10 as corrected on
    // 2026-09-28 (user ruling; 13 V-68). An empty slot reports level 0, and
    // taking it into the minimum makes a legally single-battery machine read
    // 0% forever -- measured on the live chassis the same day: list[0] absent
    // and list[1] at 72%, soc_pct published as 0. Once critical_soc_pct is
    // calibrated that is a machine that refuses to move at 72% charge.
    //
    // The count and the minimum are updated under ONE condition on purpose.
    // present == 0 is what tells write_power_state to publish null rather than a
    // number, and two separate conditions could disagree about whether a
    // minimum exists at all. Reading `present_count == 1` as "this is the
    // first present pack" also removes the separate seed flag: seeding from 0
    // would report a full robot as empty and seeding from 100 would hide a
    // flat pack, and neither failure can happen if the seed IS the first
    // qualifying value.
    if (b.present) {
      ++s.present_count;
      if (s.present_count == 1 || b.level < s.min_level) s.min_level = b.level;
    }
    s.batteries.push_back(b);
  }

  // DeviceTemperature: two readings per joint, same sixteen joints and same
  // order as MotionStatus::joint. Fixed length on purpose -- see
  // get_fixed_double_array.
  auto dt = items->find("DeviceTemperature");
  if (dt != items->end() && dt->is_object()) {
    const bool m = get_fixed_double_array(*dt, "Motor", s.temps.motor, 16);
    const bool d = get_fixed_double_array(*dt, "Driver", s.temps.driver, 16);
    // Both or neither. One of the two alone would publish sixteen zeros under
    // the other name, and 0 degrees is a plausible reading.
    s.temps.valid = m && d;
  }

  // DevEnable. The names are the chassis's own (guide 1.3.1.3), including the
  // nested VoiceControl pair -- 13 S7.2 v1.3 measured every one of them.
  auto de = items->find("DevEnable");
  if (de != items->end() && de->is_object()) {
    s.dev_enable.valid = true;
    s.dev_enable.fan_speed = static_cast<int>(get_int(*de, "FanSpeed", 0));
    s.dev_enable.load_power = static_cast<int>(get_int(*de, "LoadPower", 0));
    s.dev_enable.led_host = static_cast<int>(get_int(*de, "LedHost", 0));
    s.dev_enable.led_ext = static_cast<int>(get_int(*de, "LedExt", 0));
    s.dev_enable.fp = static_cast<int>(get_int(*de, "FP", 0));
    s.dev_enable.lidar = static_cast<int>(get_int(*de, "Lidar", 0));
    s.dev_enable.gps = static_cast<int>(get_int(*de, "GPS", 0));
    s.dev_enable.video = static_cast<int>(get_int(*de, "Video", 0));
    s.dev_enable.gps_mode = static_cast<int>(get_int(*de, "GPSMode", 0));
    s.dev_enable.led = static_cast<int>(get_int(*de, "LED", 0));
    auto vc = de->find("VoiceControl");
    if (vc != de->end() && vc->is_object()) {
      s.dev_enable.voice = static_cast<int>(get_int(*vc, "Voice", 0));
      s.dev_enable.voiceplay = static_cast<int>(get_int(*vc, "Voiceplay", 0));
    }
  }

  // GPS. Forwarded, never consumed: 11 S9.8.3 says positioning runs on our own
  // G90 RTK and this is reference only.
  auto gp = items->find("GPS");
  if (gp != items->end() && gp->is_object()) {
    s.gps.valid = true;
    s.gps.latitude = get_double(*gp, "Latitude", 0.0);
    s.gps.longitude = get_double(*gp, "Longitude", 0.0);
    s.gps.altitude = get_double(*gp, "Altitude", 0.0);
    s.gps.speed = get_double(*gp, "Speed", 0.0);
    s.gps.course = get_double(*gp, "Course", 0.0);
    s.gps.hdop = get_double(*gp, "HDOP", 0.0);
    s.gps.vdop = get_double(*gp, "VDOP", 0.0);
    s.gps.pdop = get_double(*gp, "PDOP", 0.0);
    s.gps.fix_quality = static_cast<int>(get_int(*gp, "FixQuality", 0));
    s.gps.num_satellites = static_cast<int>(get_int(*gp, "NumSatellites", 0));
    s.gps.visible_satellites =
        static_cast<int>(get_int(*gp, "VisibleSatellites", 0));
  }

  // CPU. Two hosts on this machine; 11 S9.8.3 warns that GOS is absent on a
  // STD build and that the parser must tolerate it -- which the find() below
  // does by leaving `valid` false rather than by inventing an empty host.
  auto cp = items->find("CPU");
  if (cp != items->end() && cp->is_object()) {
    parse_cpu_host(*cp, "AOS", &s.cpu_aos);
    parse_cpu_host(*cp, "NOS", &s.cpu_nos);
  }

  *out = s;
  return true;
}

bool parse_fault_report(const std::uint8_t* asdu, std::size_t len, FaultReport* out) {
  if (out == nullptr) return false;
  Json root;
  if (!parse_root(asdu, len, &root)) return false;
  const Json* items = items_of(root);
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
    f.code = format_chassis_fault_code(get_int(e, "Code", 0));
    f.name = get_string(e, "Name");
    // Details has no schema on either side of the link (13 S7.3), so it is
    // carried as text. Rendering a non-string with dump() keeps a structured
    // Details readable instead of silently empty.
    auto det = e.find("Details");
    if (det != e.end()) {
      f.details = det->is_string() ? det->get<std::string>() : det->dump();
    }
    f.grouped = get_bool(e, "Grouped", false);
    f.resources = get_string_array(e, "Resources");
    f.source = get_string_array(e, "Source");
    f.source_ids = get_string_array(e, "SourceIds");
    auto ts = e.find("Timestamp");
    if (ts != e.end() && ts->is_object()) {
      f.since_sec = get_int(*ts, "Sec", 0);
      f.since_nanosec = get_int(*ts, "Nanosec", 0);
      // Presence is recorded, not inferred from the values: a fault whose
      // Timestamp the chassis omitted must reach the wire as since_ts = null,
      // never as the epoch (see FaultEntry::since_valid).
      f.since_valid = true;
    }
    f.type = static_cast<int>(get_int(e, "Type", 0));
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
    f.level = severity_to_level(sev_present, sev_value);
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
