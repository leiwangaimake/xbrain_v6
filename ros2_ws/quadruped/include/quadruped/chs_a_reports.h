/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_a_reports.h
 * Brief: Chassis report ASDUs into internal structs -- open sets, CF-1..CF-5
 *
 * Description:
 * What this solves. The chassis reports its whole state as JSON several times a
 * second, and almost every field in it is something the process must act on or
 * forward. Two properties of that stream decide the shape of this file, and
 * both of them are the opposite of what a first implementation assumes:
 *
 *   * the mode fields are an OPEN SET, not an enum (13 S6.5). The vendor's own
 *     manual says the fault table on the robot is authoritative, the readback
 *     enumeration is missing values the command enumeration accepts, and the
 *     capture this repository ships reports Gait 0 at rest -- a value 13 S5.3
 *     does not list at all. An implementation that rejects what it does not
 *     recognise throws away real chassis state; one that maps it to the nearest
 *     known value is worse, because it invents state that was never reported.
 *   * the fault list is an open set for the same reason, but it arrives WITH a
 *     severity, so an unknown code still has a trustworthy level. That is what
 *     makes the open set safe rather than merely permissive.
 *
 * The three bans from 13 S6.5, restated because each has a failure attached:
 *   1. never map an unregistered value onto a registered one -- fail-open;
 *   2. never drop the whole report because one field is unregistered. HES and
 *      MotionState arrive in the SAME message, so discarding it over a strange
 *      Gait throws away the hardware emergency-stop signal;
 *   3. never report the label without the raw value -- a field office cannot
 *      act on "unknown".
 *
 * Fault codes carry a namespace prefix (13 CF-1..CF-5). The chassis fault space
 * and the charging-dock fault space overlap numerically and mean different
 * things (0x1007 is "no current at the dock" in one and undefined in the
 * other), so a bare 0x1007 cannot be interpreted at all. quadruped is the only
 * process that sees both channels, so it is the only one that can write the
 * prefix -- CF-2. Both branches exist here even though this batch produces only
 * the chassis one; that is CF-2's explicit requirement, not a hook for later.
 *
 * WHERE THIS RUNS, and why it may allocate: the chs_a_rx thread, ordinary
 * priority (13 S9.1). JSON parsing is placed there and NOT in ctrl precisely so
 * that the realtime path never allocates (QD-7 / RTC-5). Nothing in this file
 * may be called from ctrl or from rt_safety.
 *
 * Boundary: this turns bytes into structs. It does not decide anything -- the
 * session state machine (chs_a_session) owns conn state and failure counting,
 * and Tier 1 owns the stop decision.
 *
 * *** This paragraph used to end with "publishing the full-fidelity report onto
 * the RT plane is B4's; the raw ASDU is deliberately NOT copied into these
 * structs -- 13 S7.1 requires every field to be forwarded verbatim, and that is
 * done from the received buffer rather than by rebuilding it from here."
 * It was not true. Nothing forwarded from the received buffer, so for the whole
 * life of the file the three groups this header waved at (DeviceTemperature,
 * DevEnable, CPU -- and the GPS group the real machine adds) reached nobody:
 * 11 S9.8.3 lists every one of them and state/chassis_device carried none.
 * Corrected 2026-09-28 by PARSING them here, which is what 13 S7.1's "verbatim"
 * now means in practice. A comment describing a mechanism that does not exist
 * is worse than no comment: it answers the reader's question wrongly.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_REPORTS_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_REPORTS_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace quadruped {
namespace chs_a {

// One field read back from an open set. Both halves are always present: the
// label is what a person reads, the raw value is what a person acts on, and
// 13 S6.5 ban 3 exists because an early implementation shipped only the label.
struct OpenSetValue {
  // Signed, and that is not incidental: soft_estop is -2 (13 S6.1). An
  // unsigned field would read it as 65534 and the label lookup would miss.
  std::int64_t raw = 0;
  bool known = false;
  std::string label;  // "stand", or "unknown_0x1003" / "unknown_-7"
};

// 13 S5.2 / S5.3 / S6.1 with 11 S9.2.4's names. The numbers come from the
// CURRENT vendor guide plus the 2026-09-15 measurements, NOT from the old
// manual 11 was written against -- soft_estop is -2 there and 2 here, and 2 is
// joint_damp, so copying the old table maps an emergency stop onto a damping
// state (13 S6.1 calls this out as the册's most important correction).
OpenSetValue resolve_motion_state(std::int64_t raw);
OpenSetValue resolve_gait(std::int64_t raw);
OpenSetValue resolve_usage_mode(std::int64_t raw);

// Whether a RAW gait value is one of the two stair gaits (0x1003 / 0x3003).
// 13 S4.4 (4) and 11 S9.9 both hang on this: on a stair gait the wheel odometry
// is published with covariance inflated 3.33x AND with valid = false -- 13 took
// both, saying in so many words that the inflation alone is not enough there.
//
// It takes the RAW value, not an OpenSetValue: the label of an unregistered
// code is unknown_0xNNNN, and matching on labels would make the safety
// behaviour depend on a rendering choice.
// A gait's raw value from its contract name, or false when the name is not one
// of 13 S5.3's five. Used by the config loader, which takes NAMES and has to
// compare them against the read-back VALUES the chassis reports.
bool gait_value_by_name(const std::string& name, std::int64_t* out);

bool is_stair_gait(std::int64_t raw);

// The basic status report (Type 0x00100064 / Command 0x00f00000), 2 Hz.
struct BasicStatus {
  OpenSetValue motion_state;
  OpenSetValue gait;
  OpenSetValue usage_mode;
  // Hardware emergency stop, straight off the wire. The single most important
  // bit in the stream: 13 S6.5 ban 2 exists to protect it.
  bool hes = false;
  // 13 F-21. The chassis lies down and cuts motor power after five idle
  // minutes; every motion command then comes back 0xE008 and there is NO wake
  // command in the protocol. Reported so the process can refuse to send rather
  // than spend five seconds discovering it per command.
  bool sleep = false;
  int charge = 0;
  int status_code = 0;
  int robot_type = 0;
  int direction = 0;
  int ooa = 0;
  int ota_status = 0;
  int power_management = 0;
  int reset_joints_zero = 0;
  std::string device_num;  // "CM20200041"
  std::string model;       // "CA9C"
  std::string sn;
  std::string version;     // "PRO" -- gates the chassis navigation licence
};

// The motion status report (Type 0x00100001 / Command 0x00f00000), 10 Hz.
// Velocities here are the odometry source (13 S4.2); the covariance model in
// B6 needs the sample age, which is why the caller stamps arrival, not this.
struct MotionStatus {
  OpenSetValue motion_state;
  OpenSetValue gait;
  double linear_x = 0.0;   // m/s, body frame
  double linear_y = 0.0;   // m/s
  double angular_z = 0.0;  // rad/s
  double roll = 0.0;       // rad
  double pitch = 0.0;      // rad
  double yaw = 0.0;        // rad
  double height = 0.0;     // m, body height above ground
  // *** No `payload`. The chassis marks MotionStatus.Payload an INVALID
  // parameter and 11 S9.8.2 deleted the field in v0.2 for exactly that
  // reason (v0.1 had mis-mapped it as a load reading). Parsed and
  // published as payload_kg: 0.0 until 2026-09-28. Not kept as an unread
  // member: a number sitting in a struct is how it gets published again.
  double remain_mile = 0.0;  // km, chassis estimate
  double acc_x = 0.0;
  double acc_y = 0.0;
  double acc_z = 0.0;
  double omega_x = 0.0;
  double omega_y = 0.0;
  double omega_z = 0.0;
  // MotorStatus.Joint[16], the report's other parameter group. 11 S9.8.2
  // lists it as `joints` and even gives the leg-prefix table, and it was NOT
  // parsed until 2026-09-28 -- the chassis sends sixteen joint angles ten
  // times a second and nothing in this system could see one of them.
  //
  // *** The ORDER is the vendor's, quoted from the guide 1.3.1.2 note because
  // 13 does not carry it: LeftFrontHipX, LeftFrontHipY, LeftFrontKnee,
  // LeftFrontWheel, then RightFront*, LeftBack*, RightBack* in the same four.
  // A fixed array rather than a vector: the length is the machine's leg count
  // and a short list is a malformed report, not a smaller robot.
  bool has_joints = false;
  double joint[16] = {};
};

// The per-joint temperatures, DeviceTemperature in the device report. Two
// readings per joint (the winding and its driver), in the SAME order as
// MotionStatus::joint -- the guide names the order once, for Joint[16], and
// these two arrays are the same sixteen joints by construction (same length,
// same group, "各关节的电机温度及驱动器温度"). That inference is registered in
// 11 S9.8.3 rather than left implicit: it is the one thing here the vendor
// does not state twice.
struct DeviceTemps {
  bool valid = false;
  double motor[16] = {};
  double driver[16] = {};
};

// DevEnable, the peripheral enable bits. 11 S9.8.3 carries `load_power` into
// the health model (13 V-56 closed on the 2026-09-15 measurement), so this is
// not a diagnostic curiosity -- it is where "the payload bay lost power" would
// first be visible.
struct DevEnableStatus {
  bool valid = false;
  int fan_speed = 0;
  int load_power = 0;
  int led_host = 0;
  int led_ext = 0;
  int fp = 0;
  int lidar = 0;   // 0 off / 1 on / 2 starting (guide 1.3.1.3)
  int gps = 0;
  int video = 0;
  int gps_mode = 0;
  int led = 0;
  int voice = 0;
  int voiceplay = 0;
};

// The chassis's own GPS. 11 S9.8.3 is explicit that this is REFERENCE ONLY --
// positioning runs on our G90 RTK -- so it is forwarded and never consumed.
struct GpsStatus {
  bool valid = false;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0;
  double course = 0.0;
  double hdop = 0.0;
  double vdop = 0.0;
  double pdop = 0.0;
  int fix_quality = 0;
  int num_satellites = 0;
  int visible_satellites = 0;
};

// One of the chassis's control hosts (AOS / NOS; this machine is STD and has
// no GOS, and 11 S9.8.3 says the parser must tolerate that rather than treat
// a missing group as a malformed report).
struct CpuHostStatus {
  bool valid = false;
  std::string soc_id;        // "103" / "106"
  int avg_util_pct = 0;
  int package_temp_c = 0;
  std::vector<int> util_pct;
  std::vector<int> temps_c;
  std::vector<int> cur_freq_khz;
  std::vector<int> hw_max_freq_khz;
  std::vector<int> hw_min_freq_khz;
  std::vector<std::string> gov_policy;
};

// One battery, as the chassis reports it. 13 S7.2: the array is authoritative
// and the left/right named view is a CONSTRUCTED one whose index mapping is
// still unknown (V-55) -- the serial field came back empty on the real machine,
// so the one documented way to tell the two apart does not work.
struct BatteryEntry {
  int level = 0;          // percent
  double voltage = 0.0;   // V
  double temperature_c = 0.0;
  bool charging = false;
  std::string serial;     // empty on the measured machine
  // Whether a pack is physically there. An EMPTY SLOT reports level 0,
  // voltage 0.0 and temperature -273.0 -- the absolute-zero sentinel for "no
  // sensor" -- which is indistinguishable from a flat battery if only the
  // level is read. Measured on 2026-09-15 18:4x with one pack removed, while
  // the chassis simultaneously reported PowerManagement 1 (single_battery).
  //
  // The flag exists so the two cases can be told apart upstream. It does NOT
  // change min_level: 13 BAT-1 says every SOC judgement uses the list and
  // min(level) is unchanged, and quietly excluding a slot would be this file
  // deciding a question 13 V-68 raises for the vendor.
  bool present = false;
};

// The device status report (Type 0x00100002 / Command 0x00f00000), 2 Hz.
// Only the battery view is lifted out: it is what the power state and the
// charging decision need. Everything else (CPU, temperatures, device enables)
// is forwarded verbatim from the raw buffer by B4 and has no consumer here,
// and 9.3 forbids writing the consumer before there is something to consume.
struct DeviceStatus {
  std::vector<BatteryEntry> batteries;
  // 11 S9.8.3 / 13 BAT-1 keep the SOC judgement on the MINIMUM: a pack that is
  // nearly empty decides when the robot must return, regardless of the other.
  //
  // *** The minimum is over the PRESENT packs only (11 S4.2 CHG-10 as corrected
  // 2026-09-28; 13 V-68). An absent slot reports level 0, and until that day
  // this field took it into the minimum "because the contract said min over the
  // list" -- which made a legally single-battery machine read 0% forever.
  // present: false means "there is no pack here", not "this pack is at zero";
  // reading the absence as a zero is CLAUDE.md 3.1's `0.0 masquerading as a
  // calibrated value`, and it arms a refusal that fires at full charge.
  //
  // *** VALID ONLY WHEN present_count > 0. With every slot empty there is no
  // minimum to take and this field keeps its initialiser -- the writer must
  // publish null, never the 0 that sits here. The two are updated under the
  // same condition (see parse_device_status) so they cannot disagree.
  int min_level = 0;
  bool any_charging = false;
  // How many slots hold a pack. Compared against batteries.size() by the
  // consumer: fewer means a slot is empty, which 11 S4.2 models as
  // power_management "single_battery" rather than as a fault. Also the
  // validity flag for min_level above.
  std::size_t present_count = 0;
  // The device report's other parameter groups. The guide (1.3.1.3) lists four
  // -- BatteryList, DeviceTemperature, DevEnable, CPU -- and the real machine
  // sends a fifth, GPS (13 S7.2 v1.3 measured it and closed V-52 on it).
  // Only BatteryList was lifted out until 2026-09-28, and this header said so
  // in as many words: "everything else is forwarded verbatim from the raw
  // buffer". Nothing forwarded it. 11 S9.8's heading requires all of it and
  // 11 S9.8.3 lists every one of these blocks by name.
  DeviceTemps temps;
  DevEnableStatus dev_enable;
  GpsStatus gps;
  CpuHostStatus cpu_aos;
  CpuHostStatus cpu_nos;
};

// One fault, 13 S7.3. `code` already carries its namespace prefix.
struct FaultEntry {
  std::string code;   // "chs:0x8001"
  std::string name;   // "joint_position_over_limit", the only clue for an
                      // unregistered code
  std::string details;  // free-form, forwarded and NOT parsed (no schema)
  bool grouped = false;
  std::vector<std::string> resources;
  std::vector<std::string> source;
  std::vector<std::string> source_ids;
  // The chassis WALL clock. It cannot be compared with our monotonic clock
  // (CLK-C4), so the caller stamps its own arrival time separately rather than
  // converting either one into the other.
  std::int64_t since_sec = 0;
  std::int64_t since_nanosec = 0;
  // True only when the report actually carried a Timestamp object. Without
  // this flag "the chassis sent no time" and "the chassis clock read 0" are
  // the same two integers, and the wire writer would publish since_ts = 0.0 --
  // a NUMBER, which p5 believes, dating the event 1970-01-01 instead of
  // counting it as a fault with no occurrence time (11 S9.8.4 allows the
  // field to be null; p5's no_since_ts counter is that case's own spelling).
  bool since_valid = false;
  // 1 = START (first occurrence, or the severity changed), 2 = STOP (cleared).
  int type = 0;
  // From the closed set kFaultLevel. Derived from the chassis Severities field,
  // never from a local table: the severity travels with each fault, so an
  // unrecognised CODE still has a level we can trust.
  std::string level;
};

// The fault report (Type 0x0010007f / Command 0x00f00000), 2 Hz plus on change.
struct FaultReport {
  std::vector<FaultEntry> faults;   // Type 1
  std::vector<FaultEntry> cleared;  // Type 2
};

// 13 S7.3: 3/4/5 are WARN/ERROR/FATAL. Anything else -- including a missing
// field -- is "degraded", NOT "warn". The chassis fault space is open, so an
// unknown severity is a normal event rather than a defect, and the conservative
// direction is the one that keeps the robot from being reported as merely
// noisy while it is in trouble.
std::string severity_to_level(bool present, std::int64_t severity);

// CF-1 / CF-2. Two spaces, numerically overlapping and semantically disjoint.
// Both exist here because CF-2 requires both branches to be written together,
// so that enabling the charging channel later changes no code in this file.
std::string format_chassis_fault_code(std::int64_t code);  // "chs:0x8001"
std::string format_charge_fault_code(std::int64_t code);   // "chg:0x1007"

// CF-1's regular expression, as a predicate: ^(chs|chg):0x[0-9A-Fa-f]{4}$.
// Used to validate before a code goes up, since a bare or mis-prefixed code
// must be E_SCHEMA rather than guessed into one of the two spaces.
bool is_valid_prefixed_fault_code(const std::string& code);

// Parse one report ASDU. Each returns false when the payload is not the report
// it was asked for or is not parseable at all; a field that is merely MISSING
// leaves its member at the documented default and does not fail the parse --
// 13 S6.5 ban 2, restated: one absent field must not cost the whole report.
//
// `asdu` is the JSON body only (no 16-byte header), as chs_a_framer hands it
// out. It is not retained: everything the caller needs is in the struct, and
// the verbatim forwarding path reads the original buffer.
bool parse_basic_status(const std::uint8_t* asdu, std::size_t len, BasicStatus* out);
bool parse_motion_status(const std::uint8_t* asdu, std::size_t len, MotionStatus* out);
bool parse_device_status(const std::uint8_t* asdu, std::size_t len, DeviceStatus* out);
bool parse_fault_report(const std::uint8_t* asdu, std::size_t len, FaultReport* out);

}  // namespace chs_a
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_REPORTS_H_
