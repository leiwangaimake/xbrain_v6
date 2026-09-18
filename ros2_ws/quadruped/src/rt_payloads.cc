/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_payloads.cc
 * Brief: The data objects, assembled into caller buffers (see rt_payloads.h)
 *
 * Description:
 * Every function here follows the same shape: append into a bounded buffer,
 * and the moment anything does not fit, return 0. The Appender below is the one
 * place that bound is enforced, so there is no branch in which a half-written
 * object escapes.
 *
 * Why a small appender rather than one big snprintf per message. RobotState has
 * optional sub-objects and a variable-length fault array; a single format string
 * cannot express "null when there has been no report yet", and building the
 * message with string concatenation would allocate on a path that publishes ten
 * times a second. The appender keeps both properties: no allocation, and an
 * absent sub-object is a branch rather than a placeholder value.
 *
 * On JSON string escaping: the only strings that reach these objects from the
 * chassis are fault names and details, and those come from a vendor firmware
 * rather than from an operator. They are escaped anyway. An unescaped quote in
 * a fault name would produce a payload that fails to parse, and the visible
 * symptom is state/robot going silent -- which reads as the robot having died
 * rather than as a formatting bug.
 */

#include "quadruped/rt_payloads.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace quadruped {
namespace rt {
namespace {

// Bounded appender. Once it has overflowed it stays overflowed, so a caller can
// write the whole object and check once at the end rather than after every
// field -- a per-field check that anyone forgets once is a truncation.
class Appender {
 public:
  Appender(char* out, std::size_t cap) : out_(out), cap_(cap) {
    if (out_ == nullptr || cap_ == 0) overflow_ = true;
  }

  void Raw(const char* s) {
    if (overflow_ || s == nullptr) return;
    const std::size_t n = std::strlen(s);
    if (len_ + n + 1 > cap_) {
      overflow_ = true;
      return;
    }
    std::memcpy(out_ + len_, s, n);
    len_ += n;
  }

  // A JSON string literal, with the six escapes JSON requires. Control
  // characters below 0x20 go out as \u00XX rather than raw: a raw one is
  // invalid JSON and every decoder rejects the whole object.
  void Str(const char* s) {
    Raw("\"");
    if (overflow_ || s == nullptr) {
      Raw("\"");
      return;
    }
    for (const char* p = s; *p != '\0'; ++p) {
      const unsigned char c = static_cast<unsigned char>(*p);
      switch (c) {
        case '"': Raw("\\\""); break;
        case '\\': Raw("\\\\"); break;
        case '\n': Raw("\\n"); break;
        case '\r': Raw("\\r"); break;
        case '\t': Raw("\\t"); break;
        default:
          if (c < 0x20) {
            char esc[8];
            std::snprintf(esc, sizeof(esc), "\\u%04x", c);
            Raw(esc);
          } else {
            char one[2] = {static_cast<char>(c), '\0'};
            Raw(one);
          }
      }
      if (overflow_) return;
    }
    Raw("\"");
  }

  void Num(double v) {
    // A non-finite value is not JSON. It reaches here only if an upstream
    // check was skipped, and writing "null" keeps the object parseable so the
    // rest of the state still arrives -- the same reasoning as 13 S6.5 ban 2.
    if (!std::isfinite(v)) {
      Raw("null");
      return;
    }
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    Raw(buf);
  }

  void Int(long long v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%lld", v);
    Raw(buf);
  }

  void UInt(unsigned long long v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%llu", v);
    Raw(buf);
  }

  void Bool(bool v) { Raw(v ? "true" : "false"); }

  // Finish. Returns 0 on any overflow that happened anywhere along the way.
  std::size_t Finish() {
    if (overflow_) return 0;
    out_[len_] = '\0';
    return len_;
  }

 private:
  char* out_;
  std::size_t cap_;
  std::size_t len_ = 0;
  bool overflow_ = false;
};

// An open-set field goes out as its LABEL, with the raw value beside it. 13 S6.5
// ban 3: a label alone is not actionable, and the raw number is what a field
// engineer matches against the manual.
void OpenSet(Appender* a, const char* name, const chs_a::OpenSetValue& v) {
  a->Raw(",\"");
  a->Raw(name);
  a->Raw("\":");
  a->Str(v.label.c_str());
  a->Raw(",\"");
  a->Raw(name);
  a->Raw("_raw\":");
  a->Int(static_cast<long long>(v.raw));
}

}  // namespace

std::size_t WriteHelloAck(const HelloAckInput& in, char* out,
                          std::size_t cap) {
  Appender a(out, cap);
  a.Raw("{\"type\":\"hello_ack\",\"proto_version\":");
  a.Str(in.proto_version);
  a.Raw(",\"runtime\":{");
  // model / version: absent when no BasicStatus has arrived. 11 S9.7 sources
  // both from the chassis, so a value here means the chassis answered -- which
  // is the one thing the upstream cannot check for itself.
  a.Raw("\"model\":");
  if (in.model != nullptr) { a.Str(in.model); } else { a.Raw("null"); }
  a.Raw(",\"version\":");
  if (in.version != nullptr) { a.Str(in.version); } else { a.Raw("null"); }
  if (in.has_triple) {
    OpenSet(&a, "usage_mode", chs_a::ResolveUsageMode(in.usage_mode_raw));
    OpenSet(&a, "motion_state", chs_a::ResolveMotionState(in.motion_state_raw));
    OpenSet(&a, "gait", chs_a::ResolveGait(in.gait_raw));
  } else {
    a.Raw(",\"usage_mode\":null,\"motion_state\":null,\"gait\":null");
  }
  // 11 S9.7 lists `services`, and 21 V-14 rules its value for this period
  // verbatim: "握手 runtime.services 恒填 [不可查]" -- because the query method
  // itself is unanswered (Q20), which also makes S9.10.1's charge_manager
  // check unimplementable, so that check drops to warn plus a manual entry in
  // the deployment checklist.
  //
  // null, not an omitted key and not an object of guesses. An omitted key
  // reads as an older ack that predates the field; an object would state six
  // service states nobody queried. null says "we cannot see this", which is
  // the true statement.
  a.Raw(",\"services\":null");
  // Same three axes RobotState reports, and for the same reason (13 V-67:
  // spec.* defines no limit for vz / v_roll / v_pitch, so Tier 1 zeroes them
  // unconditionally). Listing an axis as active while it is forced to zero
  // would tell the upstream it can command one.
  a.Raw(",\"active_axes\":[\"vx\",\"vy\",\"wz\"]");
  // 11 S9.7 sources this from S9.11, whose S9.11.3 covers the drdds package.
  // The writer takes it as a parameter and emits what it is given; what the
  // CALLER may pass is settled by 21 V-20 ("恒 false"), and main.cc carries
  // the reasoning. Both polarities are covered by tests so that a writer which
  // hardcoded either one would be red -- the constant belongs at the call
  // site, where the ruling can be cited, not buried in the encoder.
  a.Raw(",\"drdds_available\":");
  a.Bool(in.drdds_available);
  // 13 CB-4 / DDS-9 / TF-1: the EFFECTIVE transport. All three rules give the
  // same reason -- a wrong domain, a wrong codebook or an unannounced frame
  // assignment have no other low-cost way to be noticed, and each of them
  // fails as "connected, no data" rather than as an error.
  a.Raw(",\"transport\":{\"endpoint\":");
  if (in.endpoint != nullptr) { a.Str(in.endpoint); } else { a.Raw("null"); }
  a.Raw(",\"codebook\":");
  if (in.codebook != nullptr) { a.Str(in.codebook); } else { a.Raw("null"); }
  a.Raw(",\"chassis_dds_domain\":");
  a.Int(in.chassis_dds_domain);
  a.Raw(",\"uplink_ros_domain\":");
  a.Int(in.uplink_ros_domain);
  a.Raw(",\"imu_frame_id\":");
  if (in.imu_frame_id != nullptr) { a.Str(in.imu_frame_id); } else { a.Raw("null"); }
  a.Raw("}}");
  // 11 S9.6: static limits, from configs/models/m20s.yaml via the resolved
  // snapshot. They are NOT read from the chassis -- the contract is explicit
  // that the chassis does not provide them, and v0.1's mistake was putting
  // them in the handshake's caps as if it did.
  a.Raw(",\"spec\":{\"holonomic\":");
  a.Bool(in.holonomic);
  a.Raw(",\"max_vx_mps\":");
  a.Num(in.max_vx_mps);
  a.Raw(",\"max_vy_mps\":");
  a.Num(in.max_vy_mps);
  a.Raw(",\"max_wz_radps\":");
  a.Num(in.max_wz_radps);
  a.Raw("}}");
  return a.Finish();
}

std::size_t WriteRobotState(const RobotStateInput& in, char* out,
                            std::size_t cap) {
  Appender a(out, cap);
  a.Raw("{\"conn\":");
  a.Str(chs_a::ConnStateName(in.conn));

  if (in.basic != nullptr) {
    OpenSet(&a, "usage_mode", in.basic->usage_mode);
    OpenSet(&a, "motion_state", in.basic->motion_state);
    OpenSet(&a, "gait", in.basic->gait);
    a.Raw(",\"model\":");
    a.Str(in.basic->model.c_str());
    a.Raw(",\"version\":");
    a.Str(in.basic->version.c_str());
    a.Raw(",\"hes\":");
    a.Bool(in.basic->hes);
    a.Raw(",\"sleep\":");
    a.Bool(in.basic->sleep);
  } else if (in.has_triple) {
    // The triple without the strings. 13 ASM-4 keeps model / version / the
    // full BasicStatus out of this key on purpose (they hold std::string and
    // cannot cross the lock-free slot), but the three NUMBERS are Tier 1's
    // input -- usage_mode is what NAV-111 gates every axis command on -- and a
    // consumer that cannot see them cannot tell "the chassis is in the wrong
    // mode" from "we never learned what mode it is in".
    OpenSet(&a, "usage_mode", chs_a::ResolveUsageMode(in.usage_mode_raw));
    OpenSet(&a, "motion_state", chs_a::ResolveMotionState(in.motion_state_raw));
    OpenSet(&a, "gait", chs_a::ResolveGait(in.gait_raw));
    a.Raw(",\"model\":null,\"version\":null,\"hes\":null,\"sleep\":null");
  } else {
    // Nothing has been heard from the chassis yet. null, not a zeroed struct:
    // a zeroed one reads as "idle, awake, no emergency stop", which is exactly
    // what a healthy standing robot looks like.
    a.Raw(",\"usage_mode\":null,\"motion_state\":null,\"gait\":null");
    a.Raw(",\"model\":null,\"version\":null,\"hes\":null,\"sleep\":null");
  }

  // 11 S9.3.1 / 13 S5.4: the three axes that are always active. vz, v_roll and
  // v_pitch are absent because Tier 1 zeroes them unconditionally (13 V-67 --
  // spec.* defines no limit for them), and listing an axis as active while it
  // is forced to zero would tell the layer above it can command one.
  a.Raw(",\"active_axes\":[\"vx\",\"vy\",\"wz\"]");

  // The three stop-related fields, kept apart. 11's D-08 ruling: hes is the raw
  // level (above), hes_lock is a latch software cannot clear, timeout_lock is a
  // latch a human clears. `locked` is derived so it cannot disagree with them.
  a.Raw(",\"hes_lock\":");
  a.Bool(in.tier1.hes_lock);
  a.Raw(",\"timeout_lock\":");
  a.Bool(in.tier1.timeout_lock);
  a.Raw(",\"locked\":");
  a.Bool(in.tier1.hes_lock || in.tier1.timeout_lock);
  a.Raw(",\"estop_epoch\":");
  a.UInt(in.estop_epoch);
  a.Raw(",\"soft_estop_active\":");
  a.Bool(in.soft_estop_active);
  a.Raw(",\"stop_reason\":");
  {
    // From the shared closed set, never a literal (CLAUDE.md 3.5).
    const std::string_view name = StopReasonName(in.tier1.stop_reason);
    a.Raw("\"");
    for (char c : name) {
      const char one[2] = {c, '\0'};
      a.Raw(one);
    }
    a.Raw("\"");
  }
  a.Raw(",\"mode_switching\":");
  a.Bool(in.mode_switching);

  a.Raw(",\"cmd_age_ms\":");
  if (in.cmd_age_ms < 0.0) {
    // No command has ever arrived. null rather than a huge number: a large age
    // says "late", and "never" is a different fact.
    a.Raw("null");
  } else {
    a.Num(in.cmd_age_ms);
  }

  if (in.motion != nullptr) {
    a.Raw(",\"motion\":{\"vx\":");
    a.Num(in.motion->linear_x);
    a.Raw(",\"vy\":");
    a.Num(in.motion->linear_y);
    a.Raw(",\"wz\":");
    a.Num(in.motion->angular_z);
    a.Raw(",\"roll\":");
    a.Num(in.motion->roll);
    a.Raw(",\"pitch\":");
    a.Num(in.motion->pitch);
    a.Raw(",\"yaw\":");
    a.Num(in.motion->yaw);
    a.Raw("}");
  } else {
    a.Raw(",\"motion\":null");
  }

  // 11 S4.1 RobotState.odom. This block is the ONLY carrier of `valid`: a ROS
  // nav_msgs/Odometry has no field for it, so /odom_quadruped cannot say the
  // pose is untrustworthy. 11 CD-6 and N-2 both refuse relative-displacement
  // delegation on odom.valid == false, and 13 S4.4 (4) clears it on a stair
  // gait -- a rule with no reader until this block exists.
  //
  // cov_xy_m and cov_yaw_rad are SIGMA, not variance: the schema's only
  // definition of them is the unit in the name (_m, _rad), and 13 S4.4's
  // numeric table speaks in sigma throughout (sigma_x(1 s) = 0.081 m, which is
  // also T-ODOM-2's acceptance baseline). Publishing m^2 under a name ending
  // in _m would be off by a square in the safe-looking direction below 1 m and
  // the unsafe direction above it.
  if (in.odom != nullptr) {
    a.Raw(",\"odom\":{\"x\":");
    a.Num(in.odom->x);
    a.Raw(",\"y\":");
    a.Num(in.odom->y);
    a.Raw(",\"yaw_rad\":");
    a.Num(in.odom->yaw);
    a.Raw(",\"vx\":");
    a.Num(in.odom->vx);
    a.Raw(",\"vy\":");
    a.Num(in.odom->vy);
    a.Raw(",\"wz\":");
    a.Num(in.odom->wz);
    a.Raw(",\"cov_xy_m\":");
    a.Num(std::sqrt(in.odom->var_x));
    a.Raw(",\"cov_yaw_rad\":");
    a.Num(std::sqrt(in.odom->var_yaw));
    a.Raw(",\"valid\":");
    a.Bool(in.odom->valid);
    a.Raw("}");
  } else {
    a.Raw(",\"odom\":null");
  }

  a.Raw(",\"faults\":[");
  if (in.faults != nullptr) {
    for (std::size_t i = 0; i < in.faults->faults.size(); ++i) {
      const chs_a::FaultEntry& f = in.faults->faults[i];
      if (i != 0) a.Raw(",");
      // CF-5: the SAME prefixed code the fault stream carries. The 11 S4.1
      // example still shows a bare "0x1007"; copying it is how the two sides
      // stop agreeing about what a code means.
      a.Raw("{\"code\":");
      a.Str(f.code.c_str());
      a.Raw(",\"level\":");
      a.Str(f.level.c_str());
      a.Raw(",\"name\":");
      a.Str(f.name.c_str());
      a.Raw("}");
    }
  }
  a.Raw("]}");
  return a.Finish();
}

std::size_t WritePowerState(const PowerStateInput& in, char* out,
                            std::size_t cap) {
  Appender a(out, cap);
  a.Raw("{");
  if (in.device == nullptr) {
    a.Raw("\"soc_pct\":null,\"batteries\":null,\"battery_mapping\":\"unknown\"");
    a.Raw(",\"present_count\":0,\"remain_mile_km\":null");
    a.Raw(",\"power_management\":null,\"list\":[]}");
    return a.Finish();
  }

  // 11 S4.2 / 13 BAT-1: the MINIMUM over the list. Left unchanged even when a
  // slot is empty (which reports 0) -- see 13 V-68; the honest response is to
  // report present_count beside it, not to quietly exclude a slot.
  a.Raw("\"soc_pct\":");
  a.Int(in.device->min_level);
  a.Raw(",\"present_count\":");
  a.UInt(in.device->present_count);
  a.Raw(",\"remain_mile_km\":");
  a.Num(in.remain_mile_km);

  // 13 BAT-1: the array is authoritative, original indices preserved.
  a.Raw(",\"list\":[");
  for (std::size_t i = 0; i < in.device->batteries.size(); ++i) {
    const chs_a::BatteryEntry& b = in.device->batteries[i];
    if (i != 0) a.Raw(",");
    a.Raw("{\"index\":");
    a.UInt(i);
    a.Raw(",\"level_pct\":");
    a.Int(b.level);
    a.Raw(",\"voltage_v\":");
    a.Num(b.voltage);
    a.Raw(",\"temp_c\":");
    a.Num(b.temperature_c);
    a.Raw(",\"charging\":");
    a.Bool(b.charging);
    a.Raw(",\"present\":");
    a.Bool(b.present);
    a.Raw(",\"serial\":");
    a.Str(b.serial.c_str());
    a.Raw("}");
  }
  a.Raw("]");

  // 13 BAT-2: while the mapping is unknown, left and right are NULL and the
  // mapping field says so. Filling them from array order would be a guess
  // presented as a measurement, and BAT-4 forbids even SAYING "left" in that
  // state -- the HMI must speak of pack #0 and #1.
  a.Raw(",\"battery_mapping\":");
  a.Str(in.index_map_known ? "known" : "unknown");
  a.Raw(",\"batteries\":");
  if (!in.index_map_known) {
    a.Raw("null");
  } else {
    const std::size_t n = in.device->batteries.size();
    const bool ok = in.left_index >= 0 && in.right_index >= 0 &&
                    static_cast<std::size_t>(in.left_index) < n &&
                    static_cast<std::size_t>(in.right_index) < n;
    if (!ok) {
      // A configured mapping that points outside the array is a configuration
      // error, and reporting null is the same answer as "unknown" -- inventing
      // a side here would be worse than saying nothing.
      a.Raw("null");
    } else {
      const chs_a::BatteryEntry& l = in.device->batteries[in.left_index];
      const chs_a::BatteryEntry& r = in.device->batteries[in.right_index];
      a.Raw("{\"left\":{\"level_pct\":");
      a.Int(l.level);
      a.Raw(",\"voltage_v\":");
      a.Num(l.voltage);
      a.Raw("},\"right\":{\"level_pct\":");
      a.Int(r.level);
      a.Raw(",\"voltage_v\":");
      a.Num(r.voltage);
      a.Raw("}}");
    }
  }

  // 11 S9.8.3: 0 normal / 1 single_battery. An unregistered value is reported
  // as itself rather than mapped to either, the same open-set discipline the
  // mode fields follow.
  a.Raw(",\"power_management\":");
  if (in.basic == nullptr) {
    a.Raw("null");
  } else if (in.basic->power_management == 0) {
    a.Str("normal");
  } else if (in.basic->power_management == 1) {
    a.Str("single_battery");
  } else {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "unknown_%d", in.basic->power_management);
    a.Str(buf);
  }
  a.Raw("}");
  return a.Finish();
}

std::size_t WriteEstopAck(const EstopAckInput& in, char* out, std::size_t cap) {
  Appender a(out, cap);
  a.Raw("{\"cmd_id\":");
  a.Str(in.cmd_id);
  a.Raw(",\"result\":");
  a.Str(in.result);
  a.Raw(",\"code\":");
  a.Str(in.code);
  a.Raw(",\"estop_epoch\":");
  a.UInt(in.estop_epoch);
  a.Raw(",\"applied\":[");
  {
    bool first = true;
    if (in.applied_zero_vel) {
      a.Raw("\"zero_vel\"");
      first = false;
    }
    if (in.applied_charge_abort) {
      if (!first) a.Raw(",");
      a.Raw("\"charge_abort\"");
    }
  }
  a.Raw("]");
  // MILLISECONDS, both of them. The envelope around this message carries mono
  // in SECONDS; the two are different fields and 11 names each with its unit.
  a.Raw(",\"recv_mono_ms\":");
  a.UInt(in.recv_mono_ms);
  a.Raw(",\"latency_ms\":");
  a.UInt(in.latency_ms);
  a.Raw(",\"hes\":");
  a.Bool(in.hes);
  a.Raw(",\"timeout_lock\":");
  a.Bool(in.timeout_lock);
  a.Raw("}");
  return a.Finish();
}

std::size_t WriteCtrlAck(const CtrlAckInput& in, char* out, std::size_t cap) {
  Appender a(out, cap);
  a.Raw("{\"cmd_id\":");
  a.Str(in.cmd_id);
  a.Raw(",\"result\":");
  a.Str(in.result);
  a.Raw(",\"code\":");
  a.Str(in.code);
  // 13 Q-2 makes detail.action required, and 11 CR-12 makes the two locks the
  // READ-BACK values: an ack that only said "accepted" would leave the caller
  // unable to tell whether the lock it asked about is actually gone.
  a.Raw(",\"detail\":{\"action\":");
  a.Str(in.action);
  a.Raw(",\"hes_lock\":");
  a.Bool(in.hes_lock);
  a.Raw(",\"timeout_lock\":");
  a.Bool(in.timeout_lock);
  a.Raw("}}");
  return a.Finish();
}

std::size_t WritePong(const PongInput& in, char* out, std::size_t cap) {
  Appender a(out, cap);
  a.Raw("{\"type\":\"pong\",\"seq\":");
  a.UInt(in.seq);
  a.Raw(",\"t_mono_ms\":");
  a.UInt(in.t_mono_ms);
  a.Raw(",\"estop_epoch\":");
  a.UInt(in.estop_epoch);
  a.Raw(",\"hes\":");
  a.Bool(in.hes);
  a.Raw(",\"hes_lock\":");
  a.Bool(in.hes_lock);
  a.Raw(",\"timeout_lock\":");
  a.Bool(in.timeout_lock);
  a.Raw(",\"stop_reason\":");
  {
    const std::string_view name = StopReasonName(in.stop_reason);
    a.Raw("\"");
    for (char c : name) {
      const char one[2] = {c, '\0'};
      a.Raw(one);
    }
    a.Raw("\"");
  }
  a.Raw("}");
  return a.Finish();
}

// ---------------------------------------------------------------------------
// The four report streams. See rt_payloads.h for why they take the parsed
// struct and why open-set values carry the raw number as well as the label.
// ---------------------------------------------------------------------------

std::size_t WriteChassisBasic(const chs_a::BasicStatus& in, char* out,
                              std::size_t cap) {
  Appender a(out, cap);
  // HES first, and not only because it is the field a reader looks for: OpenSet
  // emits its OWN leading comma (it is written for use after an existing
  // field), so something has to precede the first one or the object opens with
  // "{,". Found by parsing the output rather than reading it -- the writer
  // produced text that looked right and was not JSON.
  //
  // Tier 1 latches on HES independently (13 S3.2); this field is the REPORT,
  // never the decision.
  a.Raw("{\"hes\":");
  a.Bool(in.hes);
  // Flat `name` + `name_raw`, which is what OpenSet does and what
  // WriteRobotState already publishes. A second, nested shape for the same
  // values would make every consumer choose which one to read.
  OpenSet(&a, "usage_mode", in.usage_mode);
  OpenSet(&a, "motion_state", in.motion_state);
  OpenSet(&a, "gait", in.gait);
  a.Raw(",\"sleep\":");
  a.Bool(in.sleep);
  a.Raw(",\"charge\":");
  a.Int(in.charge);
  a.Raw(",\"status_code\":");
  a.Int(in.status_code);
  a.Raw(",\"power_management\":");
  a.Int(in.power_management);
  a.Raw(",\"ota_status\":");
  a.Int(in.ota_status);
  a.Raw(",\"direction\":");
  a.Int(in.direction);
  a.Raw(",\"ooa\":");
  a.Int(in.ooa);
  a.Raw(",\"model\":");
  a.Str(in.model.c_str());
  a.Raw(",\"device_num\":");
  a.Str(in.device_num.c_str());
  a.Raw(",\"sn\":");
  a.Str(in.sn.c_str());
  // 13 S5.6 / V-53: "PRO" is what gates the chassis's built-in navigation
  // licence. Forwarded because an operator cannot otherwise tell a STD machine
  // from a PRO one, and the two answer some commands differently.
  a.Raw(",\"version\":");
  a.Str(in.version.c_str());
  a.Raw("}");
  return a.Finish();
}

std::size_t WriteChassisMotion(const chs_a::MotionStatus& in, char* out,
                               std::size_t cap) {
  Appender a(out, cap);
  // The velocity block leads so OpenSet's own leading comma is valid -- see
  // WriteChassisBasic for the same point.
  a.Raw("{\"vel\":{\"x\":");
  a.Num(in.linear_x);
  a.Raw(",\"y\":");
  a.Num(in.linear_y);
  a.Raw(",\"yaw\":");
  a.Num(in.angular_z);
  a.Raw("}");
  OpenSet(&a, "motion_state", in.motion_state);
  OpenSet(&a, "gait", in.gait);
  // Body frame, m/s and rad/s. 13 S5.4 records that the manual's units column
  // says "raw/s" for the angular axis and that this is an error (V-46) -- the
  // value on the wire is rad/s, and forwarding it under any other name would
  // make every downstream consumer wrong by 57.
  a.Raw(",\"rpy\":{\"roll\":");
  a.Num(in.roll);
  a.Raw(",\"pitch\":");
  a.Num(in.pitch);
  a.Raw(",\"yaw\":");
  a.Num(in.yaw);
  a.Raw("},\"imu\":{\"acc\":[");
  a.Num(in.acc_x);
  a.Raw(",");
  a.Num(in.acc_y);
  a.Raw(",");
  a.Num(in.acc_z);
  a.Raw("],\"omega\":[");
  a.Num(in.omega_x);
  a.Raw(",");
  a.Num(in.omega_y);
  a.Raw(",");
  a.Num(in.omega_z);
  a.Raw("]},\"height_m\":");
  a.Num(in.height);
  a.Raw(",\"payload_kg\":");
  a.Num(in.payload);
  a.Raw(",\"remain_mile_km\":");
  a.Num(in.remain_mile);
  a.Raw("}");
  return a.Finish();
}

std::size_t WriteChassisDevice(const chs_a::DeviceStatus& in, char* out,
                               std::size_t cap) {
  Appender a(out, cap);
  a.Raw("{\"min_level_pct\":");
  a.Int(in.min_level);
  // 13 V-68: an empty slot reports 0, so min_level alone cannot tell a flat
  // battery from an absent one. present_count travels beside it for exactly
  // that reason -- the contract's min() rule is left alone and the FACT that
  // would otherwise be missing is supplied.
  a.Raw(",\"present_count\":");
  a.UInt(in.present_count);
  a.Raw(",\"any_charging\":");
  a.Bool(in.any_charging);
  a.Raw(",\"list\":[");
  for (std::size_t i = 0; i < in.batteries.size(); ++i) {
    const chs_a::BatteryEntry& b = in.batteries[i];
    if (i != 0) a.Raw(",");
    // The original index is carried, not the position after any filtering:
    // 13 V-55 records that the left/right mapping is unknown, so an index that
    // moved would destroy the only handle anyone has on which slot is which.
    a.Raw("{\"index\":");
    a.UInt(i);
    a.Raw(",\"level_pct\":");
    a.Int(b.level);
    a.Raw(",\"voltage_v\":");
    a.Num(b.voltage);
    a.Raw(",\"temp_c\":");
    a.Num(b.temperature_c);
    a.Raw(",\"charging\":");
    a.Bool(b.charging);
    a.Raw(",\"present\":");
    a.Bool(b.present);
    a.Raw(",\"serial\":");
    a.Str(b.serial.c_str());
    a.Raw("}");
  }
  a.Raw("]}");
  return a.Finish();
}

namespace {

void WriteFaultList(Appender* a, const std::vector<chs_a::FaultEntry>& list) {
  a->Raw("[");
  for (std::size_t i = 0; i < list.size(); ++i) {
    const chs_a::FaultEntry& f = list[i];
    if (i != 0) a->Raw(",");
    a->Raw("{\"code\":");
    a->Str(f.code.c_str());
    a->Raw(",\"name\":");
    a->Str(f.name.c_str());
    a->Raw(",\"level\":");
    a->Str(f.level.c_str());
    // Free-form and NOT parsed: 13 S7.3 records that the vendor gives no schema
    // for it. Forwarded verbatim because the string is often the only clue an
    // engineer has, and inventing a structure for it would be inventing a
    // contract the vendor has not agreed to.
    a->Raw(",\"details\":");
    a->Str(f.details.c_str());
    a->Raw(",\"grouped\":");
    a->Bool(f.grouped);
    a->Raw(",\"since\":{\"sec\":");
    a->Int(f.since_sec);
    a->Raw(",\"nanosec\":");
    a->Int(f.since_nanosec);
    a->Raw("},\"resources\":[");
    for (std::size_t k = 0; k < f.resources.size(); ++k) {
      if (k != 0) a->Raw(",");
      a->Str(f.resources[k].c_str());
    }
    a->Raw("],\"source\":[");
    for (std::size_t k = 0; k < f.source.size(); ++k) {
      if (k != 0) a->Raw(",");
      a->Str(f.source[k].c_str());
    }
    a->Raw("]}");
  }
  a->Raw("]");
}

}  // namespace

std::size_t WriteChassisFault(const chs_a::FaultReport& in, char* out,
                              std::size_t cap) {
  Appender a(out, cap);
  // BOTH lists, always, including when one is empty. An empty `cleared` and an
  // absent `cleared` are different claims: the first says nothing was cleared
  // this report, the second says nothing was said -- and a consumer that has to
  // guess will keep a fault asserted forever.
  a.Raw("{\"faults\":");
  WriteFaultList(&a, in.faults);
  a.Raw(",\"cleared\":");
  WriteFaultList(&a, in.cleared);
  a.Raw(",\"fault_count\":");
  a.UInt(in.faults.size());
  a.Raw(",\"cleared_count\":");
  a.UInt(in.cleared.size());
  a.Raw("}");
  return a.Finish();
}

}  // namespace rt
}  // namespace quadruped
