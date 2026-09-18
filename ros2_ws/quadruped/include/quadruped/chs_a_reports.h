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
 * Tier 1 owns the stop decision, and publishing the full-fidelity report onto
 * the RT plane is B4's. The raw ASDU is deliberately NOT copied into these
 * structs: 13 S7.1 requires every field to be forwarded verbatim, and that is
 * done from the received buffer rather than by rebuilding it from here.
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
OpenSetValue ResolveMotionState(std::int64_t raw);
OpenSetValue ResolveGait(std::int64_t raw);
OpenSetValue ResolveUsageMode(std::int64_t raw);

// Whether a RAW gait value is one of the two stair gaits (0x1003 / 0x3003).
// 13 S4.4 (4) and 11 S9.9 both hang on this: on a stair gait the wheel odometry
// is published with covariance inflated 3.33x AND with valid = false -- 13 took
// both, saying in so many words that the inflation alone is not enough there.
//
// It takes the RAW value, not an OpenSetValue: the label of an unregistered
// code is unknown_0xNNNN, and matching on labels would make the safety
// behaviour depend on a rendering choice.
bool IsStairGait(std::int64_t raw);

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
  double payload = 0.0;    // kg
  double remain_mile = 0.0;  // km, chassis estimate
  double acc_x = 0.0;
  double acc_y = 0.0;
  double acc_z = 0.0;
  double omega_x = 0.0;
  double omega_y = 0.0;
  double omega_z = 0.0;
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
  // *** With a slot EMPTY this reads 0, because an absent pack reports level 0.
  // That is the contract's arithmetic, implemented literally and deliberately
  // not "fixed" here -- see 13 V-68. The consumer needs present_count to tell
  // "one pack removed" from "both packs flat", and 11's own battery health item
  // already requires both packs online, so the two facts belong together.
  int min_level = 0;
  bool any_charging = false;
  // How many slots hold a pack. Compared against batteries.size() by the
  // consumer: fewer means a slot is empty, which 11 S4.2 models as
  // power_management "single_battery" rather than as a fault.
  std::size_t present_count = 0;
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
std::string SeverityToLevel(bool present, std::int64_t severity);

// CF-1 / CF-2. Two spaces, numerically overlapping and semantically disjoint.
// Both exist here because CF-2 requires both branches to be written together,
// so that enabling the charging channel later changes no code in this file.
std::string FormatChassisFaultCode(std::int64_t code);  // "chs:0x8001"
std::string FormatChargeFaultCode(std::int64_t code);   // "chg:0x1007"

// CF-1's regular expression, as a predicate: ^(chs|chg):0x[0-9A-Fa-f]{4}$.
// Used to validate before a code goes up, since a bare or mis-prefixed code
// must be E_SCHEMA rather than guessed into one of the two spaces.
bool IsValidPrefixedFaultCode(const std::string& code);

// Parse one report ASDU. Each returns false when the payload is not the report
// it was asked for or is not parseable at all; a field that is merely MISSING
// leaves its member at the documented default and does not fail the parse --
// 13 S6.5 ban 2, restated: one absent field must not cost the whole report.
//
// `asdu` is the JSON body only (no 16-byte header), as chs_a_framer hands it
// out. It is not retained: everything the caller needs is in the struct, and
// the verbatim forwarding path reads the original buffer.
bool ParseBasicStatus(const std::uint8_t* asdu, std::size_t len, BasicStatus* out);
bool ParseMotionStatus(const std::uint8_t* asdu, std::size_t len, MotionStatus* out);
bool ParseDeviceStatus(const std::uint8_t* asdu, std::size_t len, DeviceStatus* out);
bool ParseFaultReport(const std::uint8_t* asdu, std::size_t len, FaultReport* out);

}  // namespace chs_a
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_REPORTS_H_
