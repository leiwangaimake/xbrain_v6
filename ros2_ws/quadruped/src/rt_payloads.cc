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
#include <string_view>

#include "xbrain/enums/closed_sets.h"

namespace quadruped {
namespace rt {
namespace {

namespace sets = hachist::xbrain::enums;

// Lengths taken from the generated tables rather than written out: a value
// added to sets.yaml changes these with no edit here, and 11 S13.6's ban on
// substituting a nearby member is only enforceable if the bound is the real one.
constexpr std::size_t kChargeCount =
    sizeof(sets::kCharge) / sizeof(sets::kCharge[0]);
constexpr std::size_t kPowerManagementCount =
    sizeof(sets::kPowerManagement) / sizeof(sets::kPowerManagement[0]);

// Bounded appender. Once it has overflowed it stays overflowed, so a caller can
// write the whole object and check once at the end rather than after every
// field -- a per-field check that anyone forgets once is a truncation.
class Appender {
 public:
  Appender(char* out, std::size_t cap) : out_(out), cap_(cap) {
    if (out_ == nullptr || cap_ == 0) overflow_ = true;
  }

  void add_raw(const char* s) {
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
  void add_str(const char* s) {
    add_raw("\"");
    if (overflow_ || s == nullptr) {
      add_raw("\"");
      return;
    }
    for (const char* p = s; *p != '\0'; ++p) {
      const unsigned char c = static_cast<unsigned char>(*p);
      switch (c) {
        case '"': add_raw("\\\""); break;
        case '\\': add_raw("\\\\"); break;
        case '\n': add_raw("\\n"); break;
        case '\r': add_raw("\\r"); break;
        case '\t': add_raw("\\t"); break;
        default:
          if (c < 0x20) {
            char esc[8];
            std::snprintf(esc, sizeof(esc), "\\u%04x", c);
            add_raw(esc);
          } else {
            char one[2] = {static_cast<char>(c), '\0'};
            add_raw(one);
          }
      }
      if (overflow_) return;
    }
    add_raw("\"");
  }

  void add_num(double v) {
    // A non-finite value is not JSON. It reaches here only if an upstream
    // check was skipped, and writing "null" keeps the object parseable so the
    // rest of the state still arrives -- the same reasoning as 13 S6.5 ban 2.
    if (!std::isfinite(v)) {
      add_raw("null");
      return;
    }
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    add_raw(buf);
  }

  // Bytes that are ALREADY JSON, embedded verbatim. add_raw() takes a C string
  // and the payloads this wraps come as (pointer, length) from a writer that
  // does not NUL-terminate, so passing them through Raw would read past the
  // end or stop at the first zero byte.
  void add_raw_n(const char* s, std::size_t n) {
    if (overflow_ || s == nullptr) return;
    if (len_ + n + 1 > cap_) {
      overflow_ = true;
      return;
    }
    std::memcpy(out_ + len_, s, n);
    len_ += n;
  }

  // A Unix or monotonic timestamp in seconds. NOT add_num(): that formats with
  // "%.6g", which is six SIGNIFICANT digits -- a wall clock near 1.79e9 comes
  // out as "1.78996e+09" and a monotonic reading near 9.4e5 loses every
  // fractional digit. 11 S3.0 makes `mono` "一切超时与年龄判定的唯一依据",
  // so a one-second resolution there would silently coarsen every age in the
  // system. Fixed six decimals is microseconds, which is what S3.0's own
  // example carries.
  void add_time_sec(double v) {
    if (!std::isfinite(v) || v < 0.0) {
      add_raw("null");
      return;
    }
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.6f", v);
    add_raw(buf);
  }

  // A DURATION in milliseconds. Fixed six decimals for the same reason add_time_sec
  // uses them and add_num() must not be used here: "%.6g" is six SIGNIFICANT digits,
  // so it is lossy at both ends of this field's range -- and the low end is the
  // real one. EstopAck.latency_ms measures an interval that is a fraction of a
  // millisecond on this machine (11 S7.1.1 records 0.293 ms for the relay's own
  // single hop), so anything that rounds toward whole milliseconds publishes 0,
  // which is the value the field carried while it was a hardcoded constant and
  // which passes the contract's 100 ms criterion perfectly.
  void add_milli_sec(double v) {
    // Unreachable from a monotonic difference, and null rather than a number
    // if it ever happens: a required field reading null is a visible defect,
    // while a negative or NaN duration silently poisons whatever the far end
    // computes from it.
    if (!std::isfinite(v) || v < 0.0) {
      add_raw("null");
      return;
    }
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.6f", v);
    add_raw(buf);
  }

  void add_int(long long v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%lld", v);
    add_raw(buf);
  }

  void add_uint(unsigned long long v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%llu", v);
    add_raw(buf);
  }

  void add_bool(bool v) { add_raw(v ? "true" : "false"); }

  // A JSON string from a string_view. Closed-set members arrive this way (the
  // generated tables are string_view, not NUL-terminated char*), and they need
  // no escaping: every member is [a-z_]+ by the generator's own rule. Escaping
  // is still cheap to keep, so this routes through Str's rules rather than
  // growing a second, laxer path that a non-member string could later reach.
  void add_str_view(std::string_view v) {
    add_raw("\"");
    for (const char c : v) {
      const char one[2] = {c, '\0'};
      add_raw(one);
      if (overflow_) return;
    }
    add_raw("\"");
  }

  // Finish. Returns 0 on any overflow that happened anywhere along the way.
  std::size_t finish() {
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
void open_set(Appender* a, const char* name, const chs_a::OpenSetValue& v) {
  a->add_raw(",\"");
  a->add_raw(name);
  a->add_raw("\":");
  a->add_str(v.label.c_str());
  a->add_raw(",\"");
  a->add_raw(name);
  a->add_raw("_raw\":");
  a->add_int(static_cast<long long>(v.raw));
}

// `charge`, as 11 S9.8.1 maps it (idle 0 ... on_dock_no_current 5). ONE
// converter, called from all three places that publish this field --
// RobotState (S4.1), PowerState (S4.2) and ChassisBasic (S9.8.1).
//
// *** Why it is a function and not three copies. Until 2026-09-28 the first
// two emitted the member name while ChassisBasic emitted the raw integer, so
// the SAME chassis field went out in two different shapes from one process and
// a consumer had to pick a side. CF-5 already spells the rule for fault codes
// -- one converter, two sinks -- and this is the same failure one key over.
//
// Out of range is null, never a nearby member (11 S13.6): publishing `idle`
// for a state we do not recognise tells the upper stack the robot is free to
// drive away.
void charge_name(Appender* a, int raw) {
  if (raw < 0 || static_cast<std::size_t>(raw) >= kChargeCount) {
    a->add_raw("null");
    return;
  }
  a->add_str_view(sets::kCharge[static_cast<std::size_t>(raw)]);
}

// `power_management`, 11 S9.8.1: normal 0 / single_battery 1. Same "one
// converter" reason as charge_name -- PowerState published the member name
// while ChassisBasic published the integer.
//
// An unregistered value is reported AS ITSELF (unknown_%d), which differs from
// charge_name's null on purpose: this field has no consumer that acts on it,
// so the open-set discipline the mode fields follow keeps the raw number
// visible, whereas `charge` gates CHG's whole flow and a made-up member there
// is a motion decision.
void power_management_name(Appender* a, int raw) {
  if (raw >= 0 && static_cast<std::size_t>(raw) < kPowerManagementCount) {
    a->add_str_view(sets::kPowerManagement[static_cast<std::size_t>(raw)]);
    return;
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "unknown_%d", raw);
  a->add_str(buf);
}

// 11 S9.8.2's leg prefixes and the four joints per leg, in the vendor's own
// Joint[16] order (guide 1.3.1.2: LeftFront{HipX,HipY,Knee,Wheel}, then
// RightFront, LeftBack, RightBack). ONE table, used by both `joints` on the
// motion report and `motor_temp_c` on the device report -- the two are the
// same sixteen joints and a second copy is how index 9 comes to mean two
// different legs.
constexpr const char* kLegNames[4] = {"lf", "rf", "lb", "rb"};
constexpr const char* kJointNames[4] = {"hip_x_rad", "hip_y_rad", "knee_rad",
                                        "wheel_radps"};
// The same four joints under the names motor_temp_c uses -- no unit suffix,
// because that key's suffix is on the field itself (11 S9.8.3's example is
// lf_hip_x_motor / lf_hip_x_driver).
constexpr const char* kJointStems[4] = {"hip_x", "hip_y", "knee", "wheel"};

// A JSON array of ints from a vector. Empty vector -> [], never null: the
// chassis sends one entry per core, and an empty array says "the group was
// there and carried nothing", which is a different fact from "no group".
void int_array(Appender* a, const std::vector<int>& v) {
  a->add_raw("[");
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i != 0) a->add_raw(",");
    a->add_int(v[i]);
  }
  a->add_raw("]");
}

void str_array(Appender* a, const std::vector<std::string>& v) {
  a->add_raw("[");
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i != 0) a->add_raw(",");
    a->add_str(v[i].c_str());
  }
  a->add_raw("]");
}

// One CPU host. 11 S9.8.3's example names temp_c / freq_int / freq_app; only
// the first has a source (PackageTemp). The other two are published as NULL
// rather than omitted or faked -- see the registration in 11 S9.8.3: the
// vendor guide has no interactive/application frequency split at all, so a
// number there would be invented. Everything the guide DOES define follows,
// under the chassis's own names.
void cpu_host(Appender* a, const char* name, const chs_a::CpuHostStatus& h) {
  // No leading comma: the caller separates the two hosts, so this object does
  // not have to know whether it is first. (write_chassis_basic's open_set does
  // the opposite and needs a field ahead of it -- two conventions in one file
  // is a trap, so this one says which it is.)
  a->add_raw("\"");
  a->add_raw(name);
  a->add_raw("\":");
  if (!h.valid) {
    // A STD machine has no GOS, and a host that did not report is absent --
    // not a host running at 0 degrees (11 S9.8.3 asks for tolerance here).
    a->add_raw("null");
    return;
  }
  a->add_raw("{\"temp_c\":");
  a->add_int(h.package_temp_c);
  a->add_raw(",\"freq_int\":null,\"freq_app\":null,\"soc_id\":");
  a->add_str(h.soc_id.c_str());
  a->add_raw(",\"avg_util_pct\":");
  a->add_int(h.avg_util_pct);
  a->add_raw(",\"util_pct\":");
  int_array(a, h.util_pct);
  a->add_raw(",\"temps_c\":");
  int_array(a, h.temps_c);
  a->add_raw(",\"cur_freq_khz\":");
  int_array(a, h.cur_freq_khz);
  a->add_raw(",\"hw_max_freq_khz\":");
  int_array(a, h.hw_max_freq_khz);
  a->add_raw(",\"hw_min_freq_khz\":");
  int_array(a, h.hw_min_freq_khz);
  a->add_raw(",\"gov_policy\":");
  str_array(a, h.gov_policy);
  a->add_raw("}");
}

}  // namespace

std::size_t write_hello_ack(const HelloAckInput& in, char* out,
                          std::size_t cap) {
  Appender a(out, cap);
  a.add_raw("{\"type\":\"hello_ack\",\"proto_version\":");
  a.add_str(in.proto_version);
  a.add_raw(",\"runtime\":{");
  // model / version: absent when no BasicStatus has arrived. 11 S9.7 sources
  // both from the chassis, so a value here means the chassis answered -- which
  // is the one thing the upstream cannot check for itself.
  a.add_raw("\"model\":");
  if (in.model != nullptr) { a.add_str(in.model); } else { a.add_raw("null"); }
  a.add_raw(",\"version\":");
  if (in.version != nullptr) { a.add_str(in.version); } else { a.add_raw("null"); }
  if (in.has_triple) {
    open_set(&a, "usage_mode", chs_a::resolve_usage_mode(in.usage_mode_raw));
    open_set(&a, "motion_state", chs_a::resolve_motion_state(in.motion_state_raw));
    open_set(&a, "gait", chs_a::resolve_gait(in.gait_raw));
  } else {
    a.add_raw(",\"usage_mode\":null,\"motion_state\":null,\"gait\":null");
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
  a.add_raw(",\"services\":null");
  // Same three axes RobotState reports, and for the same reason (13 V-67:
  // spec.* defines no limit for vz / v_roll / v_pitch, so Tier 1 zeroes them
  // unconditionally). Listing an axis as active while it is forced to zero
  // would tell the upstream it can command one.
  //
  // The literal is NOT dead config: quadruped_config.cc requires
  // motion.axes.always_active to be EXACTLY this triple (13 S5.4) and refuses
  // startup otherwise -- the config key is a checked restatement, and this
  // line is the single wire spelling. (A 2026-09-22 review first read the
  // key as "filled but ignored"; the loader check is why that was wrong.)
  a.add_raw(",\"active_axes\":[\"vx\",\"vy\",\"wz\"]");
  // 11 S9.7 sources this from S9.11, whose S9.11.3 covers the drdds package.
  // The writer takes it as a parameter and emits what it is given; what the
  // CALLER may pass is settled by 21 V-20 ("恒 false"), and main.cc carries
  // the reasoning. Both polarities are covered by tests so that a writer which
  // hardcoded either one would be red -- the constant belongs at the call
  // site, where the ruling can be cited, not buried in the encoder.
  a.add_raw(",\"drdds_available\":");
  a.add_bool(in.drdds_available);
  // 13 CB-4 / DDS-9 / TF-1: the EFFECTIVE transport. All three rules give the
  // same reason -- a wrong domain, a wrong codebook or an unannounced frame
  // assignment have no other low-cost way to be noticed, and each of them
  // fails as "connected, no data" rather than as an error.
  a.add_raw(",\"transport\":{\"endpoint\":");
  if (in.endpoint != nullptr) { a.add_str(in.endpoint); } else { a.add_raw("null"); }
  a.add_raw(",\"codebook\":");
  if (in.codebook != nullptr) { a.add_str(in.codebook); } else { a.add_raw("null"); }
  a.add_raw(",\"chassis_dds_domain\":");
  a.add_int(in.chassis_dds_domain);
  a.add_raw(",\"uplink_ros_domain\":");
  a.add_int(in.uplink_ros_domain);
  a.add_raw(",\"imu_frame_id\":");
  if (in.imu_frame_id != nullptr) { a.add_str(in.imu_frame_id); } else { a.add_raw("null"); }
  a.add_raw("}}");
  // 11 S9.6: static limits, from configs/models/m20s.yaml via the resolved
  // snapshot. They are NOT read from the chassis -- the contract is explicit
  // that the chassis does not provide them, and v0.1's mistake was putting
  // them in the handshake's caps as if it did.
  a.add_raw(",\"spec\":{\"holonomic\":");
  a.add_bool(in.holonomic);
  a.add_raw(",\"max_vx_mps\":");
  a.add_num(in.max_vx_mps);
  a.add_raw(",\"max_vy_mps\":");
  a.add_num(in.max_vy_mps);
  a.add_raw(",\"max_wz_radps\":");
  a.add_num(in.max_wz_radps);
  a.add_raw("}}");
  return a.finish();
}

std::size_t write_robot_state(const RobotStateInput& in, char* out,
                            std::size_t cap) {
  Appender a(out, cap);
  // Already the WIRE name (kChassisConn): the internal->wire mapping lives in
  // rt_bridge's publish_state, the one caller, so it cannot fork. nullptr is a
  // caller defect and goes out as null rather than as a guessed member --
  // the internal names ("probing"/"ok") were what this line published until
  // 2026-09-26, and every consumer of 11 S4.1's closed set had to special-case
  // or drop them.
  a.add_raw("{\"conn\":");
  if (in.conn_wire != nullptr) { a.add_str(in.conn_wire); } else { a.add_raw("null"); }
  // 11 S4.1 proto_version: the handshake's validated peer version. Null until
  // a hello has been answered -- fabricating "1.0" would claim a handshake
  // nobody performed (the same reasoning as the null blocks below).
  a.add_raw(",\"proto_version\":");
  if (in.proto_version != nullptr) {
    a.add_str(in.proto_version);
  } else {
    a.add_raw("null");
  }

  if (in.basic != nullptr) {
    open_set(&a, "usage_mode", in.basic->usage_mode);
    open_set(&a, "motion_state", in.basic->motion_state);
    open_set(&a, "gait", in.basic->gait);
    a.add_raw(",\"model\":");
    a.add_str(in.basic->model.c_str());
    a.add_raw(",\"version\":");
    a.add_str(in.basic->version.c_str());
    a.add_raw(",\"hes\":");
    a.add_bool(in.basic->hes);
    a.add_raw(",\"sleep\":");
    a.add_bool(in.basic->sleep);
  } else if (in.has_triple) {
    // The triple without the strings. 13 ASM-4 keeps model / version / the
    // full BasicStatus out of this key on purpose (they hold std::string and
    // cannot cross the lock-free slot), but the three NUMBERS are Tier 1's
    // input -- usage_mode is what NAV-111 gates every axis command on -- and a
    // consumer that cannot see them cannot tell "the chassis is in the wrong
    // mode" from "we never learned what mode it is in".
    open_set(&a, "usage_mode", chs_a::resolve_usage_mode(in.usage_mode_raw));
    open_set(&a, "motion_state", chs_a::resolve_motion_state(in.motion_state_raw));
    open_set(&a, "gait", chs_a::resolve_gait(in.gait_raw));
    // The identity strings travel through rt_bridge's report-side cache (they
    // hold std::string and cannot cross the slot, 12 RTC-6) -- the same
    // source hello_ack reads. Until the first BasicStatus both are null: a
    // blank model reads as a chassis that answered with an empty name.
    a.add_raw(",\"model\":");
    if (in.model != nullptr) { a.add_str(in.model); } else { a.add_raw("null"); }
    a.add_raw(",\"version\":");
    if (in.version != nullptr) { a.add_str(in.version); } else { a.add_raw("null"); }
    // Same two-source rule as charge below, same reason, same day found: the
    // state path never has `basic`, so sourcing these from it alone published
    // null on every state message while the report path carried them fine.
    if (in.has_charge) {
      a.add_raw(",\"hes\":");
      a.add_bool(in.hes);
      a.add_raw(",\"sleep\":");
      a.add_bool(in.sleep);
    } else {
      a.add_raw(",\"hes\":null,\"sleep\":null");
    }
  } else {
    // Nothing has been read back from the chassis yet. null, not a zeroed
    // struct: a zeroed one reads as "idle, awake, no emergency stop", which is
    // exactly what a healthy standing robot looks like. model/version come
    // from the report-side cache and may already be present here -- the cache
    // fills on the rx thread while the triple waits for the ctrl slot, and
    // suppressing a fact the process holds would not make the null "cleaner".
    a.add_raw(",\"usage_mode\":null,\"motion_state\":null,\"gait\":null");
    a.add_raw(",\"model\":");
    if (in.model != nullptr) { a.add_str(in.model); } else { a.add_raw("null"); }
    a.add_raw(",\"version\":");
    if (in.version != nullptr) { a.add_str(in.version); } else { a.add_raw("null"); }
    a.add_raw(",\"hes\":null,\"sleep\":null");
  }

  // 11 S9.3.1 / 13 S5.4: the three axes that are always active. vz, v_roll and
  // v_pitch are absent because Tier 1 zeroes them unconditionally (13 V-67 --
  // spec.* defines no limit for them), and listing an axis as active while it
  // is forced to zero would tell the layer above it can command one.
  a.add_raw(",\"active_axes\":[\"vx\",\"vy\",\"wz\"]");

  // The three stop-related fields, kept apart. 11's D-08 ruling: hes is the raw
  // level (above), hes_lock is a latch software cannot clear, timeout_lock is a
  // latch a human clears. `locked` is derived so it cannot disagree with them.
  a.add_raw(",\"hes_lock\":");
  a.add_bool(in.tier1.hes_lock);
  a.add_raw(",\"timeout_lock\":");
  a.add_bool(in.tier1.timeout_lock);
  a.add_raw(",\"locked\":");
  a.add_bool(in.tier1.hes_lock || in.tier1.timeout_lock);
  a.add_raw(",\"estop_epoch\":");
  a.add_uint(in.estop_epoch);
  a.add_raw(",\"soft_estop_active\":");
  a.add_bool(in.soft_estop_active);
  // 11 S4.1 last_soft_estop. The KEY is always present: null before the first
  // soft estop of this boot, the object afterwards -- the contract example
  // carries it, and omitting it would read as an older message shape. Inside
  // the object, reason/src_role are null when the stop arrived without them
  // (they are best-effort on that key, 11 S9.12): an empty string would read
  // as a sender that supplied a blank reason.
  a.add_raw(",\"last_soft_estop\":");
  if (!in.has_last_estop) {
    a.add_raw("null");
  } else {
    a.add_raw("{\"epoch\":");
    a.add_uint(in.last_estop_epoch);
    a.add_raw(",\"reason\":");
    if (in.last_estop_reason != nullptr) {
      a.add_str(in.last_estop_reason);
    } else {
      a.add_raw("null");
    }
    a.add_raw(",\"src_role\":");
    if (in.last_estop_src_role != nullptr) {
      a.add_str(in.last_estop_src_role);
    } else {
      a.add_raw("null");
    }
    a.add_raw(",\"age_ms\":");
    a.add_num(in.last_estop_age_ms);
    a.add_raw("}");
  }
  a.add_raw(",\"stop_reason\":");
  {
    // From the shared closed set, never a literal (CLAUDE.md 3.5).
    const std::string_view name = stop_reason_name(in.tier1.stop_reason);
    a.add_raw("\"");
    for (char c : name) {
      const char one[2] = {c, '\0'};
      a.add_raw(one);
    }
    a.add_raw("\"");
  }
  // 11 S4.1 mode_mismatch: present IF AND ONLY IF stop_reason is
  // "mode_mismatch" (the contract says 当且仅当 -- absent is OMITTED, never
  // null), structure {expect, actual}. UM-4's derived event takes its
  // detail.actual from here; without the field, "wrong mode" is known to have
  // happened without knowing what mode the machine is actually in.
  if (in.tier1.stop_reason == StopReason::kModeMismatch) {
    // expect: from the same table Tier 1 compares against (NAV-111 gates on
    // kUsageModeNavigation), not a literal -- one source, or the two drift.
    a.add_raw(",\"mode_mismatch\":{\"expect\":");
    a.add_str(chs_a::resolve_usage_mode(kUsageModeNavigation).label.c_str());
    a.add_raw(",\"actual\":");
    if (in.basic != nullptr) {
      a.add_str(in.basic->usage_mode.label.c_str());
    } else if (in.has_triple) {
      a.add_str(chs_a::resolve_usage_mode(in.usage_mode_raw).label.c_str());
    } else {
      // No mode has ever been read back -- which is itself one of the ways a
      // mismatch happens. null, not a guessed member (11 S13.6).
      a.add_raw("null");
    }
    a.add_raw("}");
  }
  a.add_raw(",\"mode_switching\":");
  a.add_bool(in.mode_switching);
  // TR-4's computed bit, beside mode_switching (11 S4.1, 2026-09-21 unfreeze).
  // Two fields because they answer different questions -- MS-3's "may I send
  // another command" vs "is the machine mid-transition (ours or external)" --
  // and they differ exactly when the factory handset moves the robot.
  a.add_raw(",\"motion_state_transitioning\":");
  a.add_bool(in.motion_state_transitioning);

  // 11 S4.1 `charge`, six states, and S9.8.1's integer mapping. It was absent
  // from RobotState entirely -- state/robot (CR-4 relays this key) therefore
  // could not say whether the robot is on a dock, and 11 S9.11's flow keys off
  // exactly that. Same closed set and the same out-of-range rule as PowerState:
  // null, never a nearby member (11 S13.6), because `idle` for a state we do
  // not recognise tells the upper stack the robot is free to drive away.
  a.add_raw(",\"charge\":");
  {
    // Either source: `basic` when the full report is in hand, the raw int when
    // it is not. The state path only ever has the int -- BasicStatus holds
    // std::string and cannot cross the lock-free slot (12 RTC-6) -- and
    // reading `basic` alone made this field null on every state message while
    // the same value went out correctly on rt/chassis/power.
    const int raw = (in.basic != nullptr) ? in.basic->charge
                                          : (in.has_charge ? in.charge_raw : -1);
    charge_name(&a, raw);
  }
  // 11 S4.1 `services_ok`. NULL, and that is the honest answer rather than a
  // missing field: 21 V-14 rules that runtime.services is 恒填 [不可查] this
  // period because the query method itself is unanswered (Q20), and S9.10.2's
  // check is therefore unimplementable. A `true` here would assert that every
  // required chassis service is healthy on the strength of never having looked.
  a.add_raw(",\"services_ok\":null");

  a.add_raw(",\"cmd_age_ms\":");
  if (in.cmd_age_ms < 0.0) {
    // No command has ever arrived. null rather than a huge number: a large age
    // says "late", and "never" is a different fact.
    a.add_raw("null");
  } else {
    a.add_num(in.cmd_age_ms);
  }

  // *** There is no `motion` block here, and its absence is deliberate.
  // Until 2026-09-28 this writer emitted one ({vx,vy,wz,roll,pitch,yaw} from
  // a MotionStatus pointer) that 11 S4.1 does not register in either its JSON
  // example or its field table, and that publish_state never filled -- so the
  // wire carried "motion": null on every message, forever. User ruling that
  // day: delete it. Nothing is lost, only a duplicate: MotionStatus has its
  // own key (CR-7 -> state/chassis_motion) carrying all six values, and
  // RobotState's own `odom` block below carries vx/vy/wz/yaw. CLAUDE.md 9.3
  // forbids the shape it had -- a field in the schema that no business logic
  // fills or consumes is a reserved hole, and it is removed at review.
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
    a.add_raw(",\"odom\":{\"x\":");
    a.add_num(in.odom->x);
    a.add_raw(",\"y\":");
    a.add_num(in.odom->y);
    a.add_raw(",\"yaw_rad\":");
    a.add_num(in.odom->yaw);
    a.add_raw(",\"vx\":");
    a.add_num(in.odom->vx);
    a.add_raw(",\"vy\":");
    a.add_num(in.odom->vy);
    a.add_raw(",\"wz\":");
    a.add_num(in.odom->wz);
    a.add_raw(",\"cov_xy_m\":");
    a.add_num(std::sqrt(in.odom->var_x));
    a.add_raw(",\"cov_yaw_rad\":");
    a.add_num(std::sqrt(in.odom->var_yaw));
    a.add_raw(",\"valid\":");
    a.add_bool(in.odom->valid);
    // 13 S4.4's detail pair, unfrozen into the contract 2026-09-21 (11 S14.3):
    // the velocity sample's AGE at integration time -- the sample's, not the
    // publish instant's, or the fourth band's whole point (how stale is what
    // we integrated) is lost -- and WHICH source fed it, by the table's own
    // names. Only quadruped knows either value; the P1-derived odom_stale
    // event could not fill its detail until they were carried here.
    a.add_raw(",\"tau_ms\":");
    a.add_num(in.odom->tau_s * 1000.0);
    a.add_raw(",\"source\":");
    switch (in.odom_source) {
      case RobotStateInput::OdomSrc::kDrdds:
        a.add_raw("\"motion_info_20hz\"");
        break;
      case RobotStateInput::OdomSrc::kMonitor:
        a.add_raw("\"monitor_10hz\"");
        break;
      case RobotStateInput::OdomSrc::kNone:
        // An absence, not a third source name: nothing has fed the linear
        // integration yet (13 S6.5 ban 1 -- never map an unknown to a near
        // neighbour).
        a.add_raw("null");
        break;
    }
    a.add_raw("}");
  } else {
    a.add_raw(",\"odom\":null");
  }

  a.add_raw(",\"faults\":[");
  if (in.faults != nullptr) {
    for (std::size_t i = 0; i < in.fault_count; ++i) {
      const RobotStateFault& f = in.faults[i];
      if (i != 0) a.add_raw(",");
      // CF-5: the SAME prefixed code the fault stream carries. The 11 S4.1
      // example carried a bare "0x1007" until 2026-09-27 and now reads
      // "chg:0x1007" -- the correction cites this writer and its test as the
      // evidence, so the two no longer disagree. The rule is unchanged and
      // still the reason this code is not re-derived here: copying a bare
      // number is how the two sides stop agreeing about what a code means,
      // the two vendor spaces overlapping numerically. The entries arrive already
      // formatted because they are CACHED copies of the fault stream's own
      // values (rt_bridge rebuilds the cache per fault report) -- reformatting
      // here would be a second converter, which is exactly what CF-5 forbids.
      a.add_raw("{\"code\":");
      a.add_str(f.code);
      a.add_raw(",\"level\":");
      a.add_str(f.level);
      // `desc` is the key the 11 S4.1 example uses (this writer said "name"
      // until 2026-09-26 -- a consumer coded against the contract found
      // nothing). Content is FaultEntry.name, the human-readable fault name.
      a.add_raw(",\"desc\":");
      a.add_str(f.desc);
      a.add_raw("}");
    }
  }
  a.add_raw("]}");
  return a.finish();
}

std::size_t write_power_state(const PowerStateInput& in, char* out,
                            std::size_t cap) {
  Appender a(out, cap);
  a.add_raw("{");
  if (in.device == nullptr) {
    a.add_raw("\"soc_pct\":null,\"batteries\":null,\"battery_mapping\":\"unknown\"");
    a.add_raw(",\"present_count\":0,\"remain_mile_km\":null");
    a.add_raw(",\"power_management\":null,\"charge\":null,\"list\":[]}");
    return a.finish();
  }

  // 11 S4.2 / 13 BAT-1: the MINIMUM over the PRESENT packs, and null when no
  // pack is present at all.
  //
  // *** This line published the minimum over ALL slots until 2026-09-28, with
  // a comment arguing that the honest response to an empty slot is to report
  // present_count beside it rather than to exclude the slot. The user ruling
  // that day reversed it, and the reason is worth keeping: not hiding an empty
  // slot (right) is a different thing from feeding its fake zero into an
  // aggregate (wrong). present: false means "no pack here", not "this pack is
  // at 0%", and the list[] plus present_count below still report the absence
  // in full -- nothing is hidden by taking the minimum over what is actually
  // there. Measured on the chassis the same day: list[0] absent, list[1] at
  // 72%, soc_pct on the wire 0. CHG-10 reads soc_pct, so once
  // critical_soc_pct is calibrated that machine is judged FAIL and refuses to
  // move on a full battery -- a fuse with the pin already pulled.
  //
  // Null, not 0, when present_count is zero: 0 is a legal SOC and would read
  // as a flat robot (CLAUDE.md 3.1). min_level carries its initialiser in that
  // case and means nothing.
  a.add_raw("\"soc_pct\":");
  if (in.device->present_count == 0) {
    a.add_raw("null");
  } else {
    a.add_int(in.device->min_level);
  }
  a.add_raw(",\"present_count\":");
  a.add_uint(in.device->present_count);
  a.add_raw(",\"remain_mile_km\":");
  a.add_num(in.remain_mile_km);

  // 13 BAT-1: the array is authoritative, original indices preserved.
  a.add_raw(",\"list\":[");
  for (std::size_t i = 0; i < in.device->batteries.size(); ++i) {
    const chs_a::BatteryEntry& b = in.device->batteries[i];
    if (i != 0) a.add_raw(",");
    a.add_raw("{\"index\":");
    a.add_uint(i);
    a.add_raw(",\"level_pct\":");
    a.add_int(b.level);
    a.add_raw(",\"voltage_v\":");
    a.add_num(b.voltage);
    a.add_raw(",\"temp_c\":");
    a.add_num(b.temperature_c);
    a.add_raw(",\"charging\":");
    a.add_bool(b.charging);
    a.add_raw(",\"present\":");
    a.add_bool(b.present);
    a.add_raw(",\"serial\":");
    a.add_str(b.serial.c_str());
    a.add_raw("}");
  }
  a.add_raw("]");

  // 13 BAT-2: while the mapping is unknown, left and right are NULL and the
  // mapping field says so. Filling them from array order would be a guess
  // presented as a measurement, and BAT-4 forbids even SAYING "left" in that
  // state -- the HMI must speak of pack #0 and #1.
  a.add_raw(",\"battery_mapping\":");
  a.add_str(in.index_map_known ? "known" : "unknown");
  a.add_raw(",\"batteries\":");
  if (!in.index_map_known) {
    a.add_raw("null");
  } else {
    const std::size_t n = in.device->batteries.size();
    const bool ok = in.left_index >= 0 && in.right_index >= 0 &&
                    static_cast<std::size_t>(in.left_index) < n &&
                    static_cast<std::size_t>(in.right_index) < n;
    if (!ok) {
      // A configured mapping that points outside the array is a configuration
      // error, and reporting null is the same answer as "unknown" -- inventing
      // a side here would be worse than saying nothing.
      a.add_raw("null");
    } else {
      const chs_a::BatteryEntry& l = in.device->batteries[in.left_index];
      const chs_a::BatteryEntry& r = in.device->batteries[in.right_index];
      a.add_raw("{\"left\":{\"level_pct\":");
      a.add_int(l.level);
      a.add_raw(",\"voltage_v\":");
      a.add_num(l.voltage);
      a.add_raw("},\"right\":{\"level_pct\":");
      a.add_int(r.level);
      a.add_raw(",\"voltage_v\":");
      a.add_num(r.voltage);
      a.add_raw("}}");
    }
  }

  // 11 S9.8.1: 0 normal / 1 single_battery. The two names come from the SHARED
  // closed set, never from literals here (CLAUDE.md S3.5) -- they are the same
  // strings the Python side branches on, and a copy is how the two spellings
  // drift apart until integration. An unregistered value is reported as itself,
  // the same open-set discipline the mode fields follow.
  a.add_raw(",\"power_management\":");
  if (in.basic == nullptr) {
    a.add_raw("null");
  } else {
    power_management_name(&a, in.basic->power_management);
  }
  // 11 S4.2 lists `charge` in PowerState and S9.8.1 gives the mapping
  // (idle 0 ... on_dock_no_current 5). It was absent from this object, which
  // left state/power unable to say whether the robot is on a dock at all --
  // and CHG's whole flow keys off it.
  //
  // An out-of-range value is NULL, not a nearby member: 11 S13.6 forbids
  // degrading to something close, and publishing `idle` for a state we do not
  // recognise would tell the upper stack the robot is free to drive away.
  a.add_raw(",\"charge\":");
  if (in.basic == nullptr) {
    a.add_raw("null");
  } else {
    charge_name(&a, in.basic->charge);
  }
  a.add_raw("}");
  return a.finish();
}

std::size_t write_estop_ack(const EstopAckInput& in, char* out, std::size_t cap) {
  Appender a(out, cap);
  a.add_raw("{\"cmd_id\":");
  a.add_str(in.cmd_id);
  a.add_raw(",\"result\":");
  a.add_str(in.result);
  a.add_raw(",\"code\":");
  a.add_str(in.code);
  a.add_raw(",\"estop_epoch\":");
  a.add_uint(in.estop_epoch);
  a.add_raw(",\"applied\":[");
  {
    bool first = true;
    if (in.applied_zero_vel) {
      a.add_raw("\"zero_vel\"");
      first = false;
    }
    if (in.applied_charge_abort) {
      if (!first) a.add_raw(",");
      a.add_raw("\"charge_abort\"");
    }
  }
  a.add_raw("]");
  // MILLISECONDS, both of them. The envelope around this message carries mono
  // in SECONDS; the two are different fields and 11 names each with its unit.
  // Two WRITERS though: 11 S7.1.1 types recv_mono_ms uint64 (an instant) and
  // latency_ms float (an interval, measured sub-millisecond here), and add_uint on
  // the second one truncated every real measurement to 0.
  a.add_raw(",\"recv_mono_ms\":");
  a.add_uint(in.recv_mono_ms);
  a.add_raw(",\"latency_ms\":");
  a.add_milli_sec(in.latency_ms);
  a.add_raw(",\"hes\":");
  a.add_bool(in.hes);
  a.add_raw(",\"timeout_lock\":");
  a.add_bool(in.timeout_lock);
  a.add_raw("}");
  return a.finish();
}

std::size_t write_envelope(const EnvelopeInput& in, const char* data,
                          std::size_t dlen, char* out, std::size_t cap) {
  if (data == nullptr || dlen == 0) return 0;
  Appender a(out, cap);
  // Field order follows 11 S3.0's own listing. It carries no meaning for a
  // JSON decoder, but a human diffing a capture against the contract reads
  // top to bottom and an arbitrary order costs them a second every time.
  a.add_raw("{\"v\":1,\"rid\":");
  a.add_str(in.rid);
  a.add_raw(",\"ts\":");
  a.add_time_sec(in.ts);
  a.add_raw(",\"mono\":");
  a.add_time_sec(in.mono);
  a.add_raw(",\"boot\":");
  a.add_str(in.boot);
  a.add_raw(",\"seq\":");
  a.add_uint(in.seq);
  a.add_raw(",\"src\":");
  a.add_str(in.src);
  a.add_raw(",\"ts_sync\":");
  a.add_bool(in.ts_sync);
  a.add_raw(",\"data\":");
  // Verbatim: it is already a complete object written by one of the writers
  // above. Re-encoding it would mean parsing it first, and this runs on the
  // publish path.
  a.add_raw_n(data, dlen);
  a.add_raw("}");
  return a.finish();
}

std::size_t write_ctrl_ack(const CtrlAckInput& in, char* out, std::size_t cap) {
  Appender a(out, cap);
  a.add_raw("{\"cmd_id\":");
  a.add_str(in.cmd_id);
  a.add_raw(",\"result\":");
  a.add_str(in.result);
  a.add_raw(",\"code\":");
  a.add_str(in.code);
  // 13 Q-2 makes detail.action required, and 11 CR-12 makes the two locks the
  // READ-BACK values: an ack that only said "accepted" would leave the caller
  // unable to tell whether the lock it asked about is actually gone.
  a.add_raw(",\"detail\":{\"action\":");
  a.add_str(in.action);
  a.add_raw(",\"hes_lock\":");
  a.add_bool(in.hes_lock);
  a.add_raw(",\"timeout_lock\":");
  a.add_bool(in.timeout_lock);
  // 11 S9.3.3: the named item for a refusal that has one. Written only when
  // there IS one -- see CtrlAckInput::item for why the empty case omits the
  // key instead of emitting "".
  if (in.item != nullptr && in.item[0] != '\0') {
    a.add_raw(",\"item\":");
    a.add_str(in.item);
  }
  a.add_raw("}}");
  return a.finish();
}

std::size_t write_pong(const PongInput& in, char* out, std::size_t cap) {
  Appender a(out, cap);
  a.add_raw("{\"type\":\"pong\",\"seq\":");
  a.add_uint(in.seq);
  a.add_raw(",\"t_mono_ms\":");
  a.add_uint(in.t_mono_ms);
  a.add_raw(",\"estop_epoch\":");
  a.add_uint(in.estop_epoch);
  a.add_raw(",\"hes\":");
  a.add_bool(in.hes);
  a.add_raw(",\"hes_lock\":");
  a.add_bool(in.hes_lock);
  a.add_raw(",\"timeout_lock\":");
  a.add_bool(in.timeout_lock);
  a.add_raw(",\"stop_reason\":");
  {
    const std::string_view name = stop_reason_name(in.stop_reason);
    a.add_raw("\"");
    for (char c : name) {
      const char one[2] = {c, '\0'};
      a.add_raw(one);
    }
    a.add_raw("\"");
  }
  a.add_raw("}");
  return a.finish();
}

// ---------------------------------------------------------------------------
// The four report streams. See rt_payloads.h for why they take the parsed
// struct and why open-set values carry the raw number as well as the label.
// ---------------------------------------------------------------------------

std::size_t write_chassis_basic(const chs_a::BasicStatus& in, char* out,
                              std::size_t cap) {
  Appender a(out, cap);
  // HES first, and not only because it is the field a reader looks for: open_set
  // emits its OWN leading comma (it is written for use after an existing
  // field), so something has to precede the first one or the object opens with
  // "{,". Found by parsing the output rather than reading it -- the writer
  // produced text that looked right and was not JSON.
  //
  // Tier 1 latches on HES independently (13 S3.2); this field is the REPORT,
  // never the decision.
  a.add_raw("{\"hes\":");
  a.add_bool(in.hes);
  // Flat `name` + `name_raw`, which is what open_set does and what
  // write_robot_state already publishes. A second, nested shape for the same
  // values would make every consumer choose which one to read.
  open_set(&a, "usage_mode", in.usage_mode);
  open_set(&a, "motion_state", in.motion_state);
  open_set(&a, "gait", in.gait);
  a.add_raw(",\"sleep\":");
  a.add_bool(in.sleep);
  // 11 S9.8.1's field table gives BOTH of these as closed-set NAMES, and until
  // 2026-09-28 this writer put the raw integers on the wire while the very
  // same process published the names on state/robot ("idle") and state/power
  // ("single_battery"). One chassis field, two line shapes, one process --
  // a consumer had to know which key it was reading to know what a value
  // meant. Both now go through one converter each (charge_name /
  // power_management_name above), so the shapes cannot diverge again.
  a.add_raw(",\"charge\":");
  charge_name(&a, in.charge);
  a.add_raw(",\"status_code\":");
  a.add_int(in.status_code);
  a.add_raw(",\"power_management\":");
  power_management_name(&a, in.power_management);
  a.add_raw(",\"ota_status\":");
  a.add_int(in.ota_status);
  a.add_raw(",\"direction\":");
  a.add_int(in.direction);
  a.add_raw(",\"ooa\":");
  a.add_int(in.ooa);
  a.add_raw(",\"model\":");
  a.add_str(in.model.c_str());
  a.add_raw(",\"device_num\":");
  a.add_str(in.device_num.c_str());
  a.add_raw(",\"sn\":");
  a.add_str(in.sn.c_str());
  // 13 S5.6 / V-53: "PRO" is what gates the chassis's built-in navigation
  // licence. Forwarded because an operator cannot otherwise tell a STD machine
  // from a PRO one, and the two answer some commands differently.
  a.add_raw(",\"version\":");
  a.add_str(in.version.c_str());
  a.add_raw("}");
  return a.finish();
}

std::size_t write_chassis_motion(const chs_a::MotionStatus& in, char* out,
                               std::size_t cap) {
  Appender a(out, cap);
  // The velocity block leads so open_set's own leading comma is valid -- see
  // write_chassis_basic for the same point.
  // 11 S9.8.2 spells this block `velocity` with unit-suffixed members. It was
  // `vel{x,y,yaw}` until 2026-09-28: the VALUES were right (13 V-46's rad/s
  // correction is recorded below) and only the names were wrong, which is the
  // shape a consumer coded against the contract cannot work around -- it finds
  // nothing and reports no error.
  a.add_raw("{\"velocity\":{\"vx_mps\":");
  a.add_num(in.linear_x);
  a.add_raw(",\"vy_mps\":");
  a.add_num(in.linear_y);
  a.add_raw(",\"wz_radps\":");
  a.add_num(in.angular_z);
  a.add_raw("}");
  open_set(&a, "motion_state", in.motion_state);
  open_set(&a, "gait", in.gait);
  // Body frame, m/s and rad/s. 13 S5.4 records that the manual's units column
  // says "raw/s" for the angular axis and that this is an error (V-46) -- the
  // value on the wire is rad/s, and forwarding it under any other name would
  // make every downstream consumer wrong by 57.
  // 11 S9.8.2 `attitude`, same story as `velocity` above: was
  // `rpy{roll,pitch,yaw}`.
  a.add_raw(",\"attitude\":{\"roll_rad\":");
  a.add_num(in.roll);
  a.add_raw(",\"pitch_rad\":");
  a.add_num(in.pitch);
  a.add_raw(",\"yaw_rad\":");
  a.add_num(in.yaw);
  a.add_raw("},\"imu\":{\"acc\":[");
  a.add_num(in.acc_x);
  a.add_raw(",");
  a.add_num(in.acc_y);
  a.add_raw(",");
  a.add_num(in.acc_z);
  a.add_raw("],\"omega\":[");
  a.add_num(in.omega_x);
  a.add_raw(",");
  a.add_num(in.omega_y);
  a.add_raw(",");
  a.add_num(in.omega_z);
  a.add_raw("]},\"height_m\":");
  a.add_num(in.height);
  // *** No payload_kg. 11 S9.8.2 deleted it in v0.2 with the reason attached:
  // the chassis marks `Payload` an INVALID parameter, and v0.1 had mis-mapped
  // it as a load reading. This writer kept publishing it (always 0.0) until
  // 2026-09-28 -- a field the contract removed on purpose, sitting on the wire
  // looking like a measurement. Deleted here and in chs_a::MotionStatus, so
  // there is nothing left to publish by accident.
  a.add_raw(",\"remain_mile_km\":");
  a.add_num(in.remain_mile);
  // 11 S9.8.2 `joints`, MotorStatus's sixteen angles grouped by leg. Missing
  // entirely until 2026-09-28: the contract listed the block AND the leg
  // prefix table, and the parser never read the group.
  //
  // Null when the report did not carry a well-formed Joint[16] -- not an
  // object of zeros. A quadruped standing has every knee bent; sixteen zeros
  // is a pose the machine cannot hold, and publishing it would look like a
  // reading rather than like an absence.
  a.add_raw(",\"joints\":");
  if (!in.has_joints) {
    a.add_raw("null");
  } else {
    a.add_raw("{");
    for (std::size_t leg = 0; leg < 4; ++leg) {
      if (leg != 0) a.add_raw(",");
      a.add_raw("\"");
      a.add_raw(kLegNames[leg]);
      a.add_raw("\":{");
      for (std::size_t j = 0; j < 4; ++j) {
        if (j != 0) a.add_raw(",");
        a.add_raw("\"");
        a.add_raw(kJointNames[j]);
        a.add_raw("\":");
        a.add_num(in.joint[leg * 4 + j]);
      }
      a.add_raw("}");
    }
    a.add_raw("}");
  }
  a.add_raw("}");
  return a.finish();
}

std::size_t write_chassis_device(const chs_a::DeviceStatus& in, char* out,
                               std::size_t cap) {
  Appender a(out, cap);
  // The minimum over the PRESENT packs, null when none is (same rule and same
  // reason as write_power_state's soc_pct -- 11 S4.2 CHG-10 as corrected
  // 2026-09-28, 13 V-68). Both keys read the one value parse_device_status
  // computes, so they cannot give two answers for the same chassis report.
  a.add_raw("{\"min_level_pct\":");
  if (in.present_count == 0) {
    a.add_raw("null");
  } else {
    a.add_int(in.min_level);
  }
  // present_count travels beside it: it is what distinguishes "one pack
  // removed" from "both packs flat", and it is also min_level_pct's validity
  // flag (13 V-68).
  a.add_raw(",\"present_count\":");
  a.add_uint(in.present_count);
  a.add_raw(",\"any_charging\":");
  a.add_bool(in.any_charging);
  a.add_raw(",\"list\":[");
  for (std::size_t i = 0; i < in.batteries.size(); ++i) {
    const chs_a::BatteryEntry& b = in.batteries[i];
    if (i != 0) a.add_raw(",");
    // The original index is carried, not the position after any filtering:
    // 13 V-55 records that the left/right mapping is unknown, so an index that
    // moved would destroy the only handle anyone has on which slot is which.
    a.add_raw("{\"index\":");
    a.add_uint(i);
    a.add_raw(",\"level_pct\":");
    a.add_int(b.level);
    a.add_raw(",\"voltage_v\":");
    a.add_num(b.voltage);
    a.add_raw(",\"temp_c\":");
    a.add_num(b.temperature_c);
    a.add_raw(",\"charging\":");
    a.add_bool(b.charging);
    a.add_raw(",\"present\":");
    a.add_bool(b.present);
    a.add_raw(",\"serial\":");
    a.add_str(b.serial.c_str());
    a.add_raw("}");
  }
  a.add_raw("]");

  // 11 S9.8.3 `battery` -- the NAMED view, null while the index mapping is
  // unknown (13 BAT-2, V-55 still open: the serial that was supposed to
  // disambiguate came back empty on the real machine). `list[]` above is the
  // authoritative one (BAT-1) and the mapping field says which state we are
  // in, exactly as PowerState does. Filling left/right from array order would
  // be a guess presented as a measurement, and BAT-4 forbids even SAYING
  // "left" in that state.
  a.add_raw(",\"battery\":null,\"battery_mapping\":\"unknown\"");

  // 11 S9.8.3 `motor_temp_c` -- 32 readings, one motor and one driver per
  // joint, keyed by the SAME leg/joint names `joints` uses on the motion
  // report. Missing entirely until 2026-09-28 although the chassis has been
  // sending DeviceTemperature twice a second all along.
  a.add_raw(",\"motor_temp_c\":");
  if (!in.temps.valid) {
    a.add_raw("null");
  } else {
    a.add_raw("{");
    for (std::size_t leg = 0; leg < 4; ++leg) {
      for (std::size_t j = 0; j < 4; ++j) {
        const std::size_t idx = leg * 4 + j;
        if (idx != 0) a.add_raw(",");
        // "<leg>_<joint>_motor" then "..._driver", the two names 11 S9.8.3's
        // example spells out.
        a.add_raw("\"");
        a.add_raw(kLegNames[leg]);
        a.add_raw("_");
        a.add_raw(kJointStems[j]);
        a.add_raw("_motor\":");
        a.add_num(in.temps.motor[idx]);
        a.add_raw(",\"");
        a.add_raw(kLegNames[leg]);
        a.add_raw("_");
        a.add_raw(kJointStems[j]);
        a.add_raw("_driver\":");
        a.add_num(in.temps.driver[idx]);
      }
    }
    a.add_raw("}");
  }

  // 11 S9.8.3 `led`. NULL, and that is the honest answer rather than a
  // missing key: the device report has no LED group at all. `Led` in the
  // vendor guide is a COMMAND (1.2.7 custom light language, Type 0x00100005),
  // not a report, and the only LED facts the chassis reports are the
  // DevEnable bits below -- which are enable states, not the fill-light
  // readings 11 S9.8.3's example shows. Publishing led_host/led_ext a second
  // time under this name would be the same "one value, two shapes" defect the
  // charge field had. Registered in 11 S9.8.3 as having no source this batch.
  a.add_raw(",\"led\":null");

  // 11 S9.8.3 `gps` -- the chassis's own receiver, REFERENCE ONLY (our G90
  // RTK does the positioning). Forwarded with the contract's own key names.
  a.add_raw(",\"gps\":");
  if (!in.gps.valid) {
    a.add_raw("null");
  } else {
    a.add_raw("{\"lat\":");
    a.add_num(in.gps.latitude);
    a.add_raw(",\"lon\":");
    a.add_num(in.gps.longitude);
    a.add_raw(",\"alt\":");
    a.add_num(in.gps.altitude);
    a.add_raw(",\"speed\":");
    a.add_num(in.gps.speed);
    a.add_raw(",\"course\":");
    a.add_num(in.gps.course);
    a.add_raw(",\"fix_quality\":");
    a.add_int(in.gps.fix_quality);
    a.add_raw(",\"num_satellites\":");
    a.add_int(in.gps.num_satellites);
    a.add_raw(",\"hdop\":");
    a.add_num(in.gps.hdop);
    a.add_raw(",\"vdop\":");
    a.add_num(in.gps.vdop);
    a.add_raw(",\"pdop\":");
    a.add_num(in.gps.pdop);
    a.add_raw(",\"visible_satellites\":");
    a.add_int(in.gps.visible_satellites);
    a.add_raw("}");
  }

  // 11 S9.8.3 `dev_enable`. load_power is the one the health model reads
  // (13 V-56): our payload bay may be fed from it, so "the chassis switched
  // external power off" has to be visible somewhere.
  a.add_raw(",\"dev_enable\":");
  if (!in.dev_enable.valid) {
    a.add_raw("null");
  } else {
    a.add_raw("{\"fan_speed\":");
    a.add_int(in.dev_enable.fan_speed);
    a.add_raw(",\"load_power\":");
    a.add_int(in.dev_enable.load_power);
    a.add_raw(",\"led_host\":");
    a.add_int(in.dev_enable.led_host);
    a.add_raw(",\"led_ext\":");
    a.add_int(in.dev_enable.led_ext);
    a.add_raw(",\"fp\":");
    a.add_int(in.dev_enable.fp);
    // 0 off / 1 on / 2 starting -- three values, so this is NOT a bool
    // (13 S7.2 v1.3 measured the third one).
    a.add_raw(",\"lidar\":");
    a.add_int(in.dev_enable.lidar);
    a.add_raw(",\"gps\":");
    a.add_int(in.dev_enable.gps);
    a.add_raw(",\"video\":");
    a.add_int(in.dev_enable.video);
    a.add_raw(",\"gps_mode\":");
    a.add_int(in.dev_enable.gps_mode);
    a.add_raw(",\"led\":");
    a.add_int(in.dev_enable.led);
    a.add_raw(",\"voice_control\":{\"voice\":");
    a.add_int(in.dev_enable.voice);
    a.add_raw(",\"voiceplay\":");
    a.add_int(in.dev_enable.voiceplay);
    a.add_raw("}}");
  }

  // 11 S9.8.3 `cpu`, two hosts. See cpu_host for why freq_int / freq_app are
  // null and everything else carries the chassis's own names.
  a.add_raw(",\"cpu\":{");
  cpu_host(&a, "aos", in.cpu_aos);
  a.add_raw(",");
  cpu_host(&a, "nos", in.cpu_nos);
  a.add_raw("}");

  a.add_raw("}");
  return a.finish();
}

namespace {

// 11 S9.8.4 `faults[]`: one OBJECT per asserted fault. The contract's four
// keys come first and in its own order (code / level / desc / since_ts); the
// vendor evidence 13 S7.3 requires be forwarded follows. See write_chassis_fault
// for why those extras are here and the two counts are not.
void write_fault_array(Appender* a, const std::vector<chs_a::FaultEntry>& list) {
  a->add_raw("[");
  for (std::size_t i = 0; i < list.size(); ++i) {
    const chs_a::FaultEntry& f = list[i];
    if (i != 0) a->add_raw(",");
    a->add_raw("{\"code\":");
    a->add_str(f.code.c_str());
    a->add_raw(",\"level\":");
    a->add_str(f.level.c_str());
    // `desc`, not `name`: this writer spelled it `name` until 2026-09-27 and a
    // consumer coded against the contract found nothing there. 13 v1.35 had
    // already made the same correction one key over (RobotState.faults[]);
    // CF-5 makes the two keys one conversion, so the CONTENT is the same thing
    // rt_bridge caches for the state side -- FaultEntry.name, the vendor's
    // human-readable fault name, which 13 S7.3 calls the only readable clue an
    // unregistered code has.
    a->add_raw(",\"desc\":");
    a->add_str(f.name.c_str());
    // 13 S7.3 verbatim: Timestamp{Sec,Nanosec} -> since_ts. FLOAT SECONDS, via
    // add_time_sec (fixed six decimals) and never Num: %.6g renders a wall clock
    // near 1.79e9 as "1.78996e+09", which is a different instant by minutes.
    // The clock is the CHASSIS WALL clock, which is what since_ts must be --
    // p5 takes it as detected_at, the occurrence time, and a monotonic reading
    // has no meaning in another process's boot domain (CLK-C3 / CLK-C4). No
    // conversion happens here for exactly that reason.
    a->add_raw(",\"since_ts\":");
    if (f.since_valid) {
      a->add_time_sec(static_cast<double>(f.since_sec) +
                 static_cast<double>(f.since_nanosec) * 1e-9);
    } else {
      // The chassis sent no Timestamp. null is the honest spelling; 0.0 is a
      // number the consumer believes, and it dates the fault to 1970.
      a->add_raw("null");
    }
    // -- beyond 11 S9.8.4's four keys, registered there as our extension.
    // Free-form and NOT parsed: 13 S7.3 records that the vendor gives no schema
    // for it. Forwarded verbatim because the string is often the only clue an
    // engineer has, and inventing a structure for it would be inventing a
    // contract the vendor has not agreed to.
    a->add_raw(",\"details\":");
    a->add_str(f.details.c_str());
    a->add_raw(",\"grouped\":");
    a->add_bool(f.grouped);
    a->add_raw(",\"resources\":[");
    for (std::size_t k = 0; k < f.resources.size(); ++k) {
      if (k != 0) a->add_raw(",");
      a->add_str(f.resources[k].c_str());
    }
    a->add_raw("],\"source\":[");
    for (std::size_t k = 0; k < f.source.size(); ++k) {
      if (k != 0) a->add_raw(",");
      a->add_str(f.source[k].c_str());
    }
    // 13 S7.3 lists Source[] and SourceIds[] on ONE row, both marked upstream
    // ("现场定位靠它"). They are not the same fact: Source is the module
    // ("rl_deploy") and SourceIds is the INSTANCE ("motion_master#0"), so on a
    // chassis running several instances of one module Source alone cannot say
    // which one faulted -- and that is the question a field engineer actually
    // has. Parsed since the ErrorList reader was written (chs_a_reports.cc
    // get_string_array(e, "SourceIds")) and dropped on the floor here until
    // 2026-09-27; nothing else on the wire carries it. Registered as our
    // extension in 11 S9.8.4 alongside the other four, and as an F-5 unfreeze
    // in 11 S14.3 because it adds a field to a frozen schema.
    //
    // NOT added to RobotState.faults[]: 11 S4.1 gives that list exactly
    // {code, level, desc} and CF-5 makes those three one conversion. The five
    // evidence fields are registered on this key only -- the summary view is
    // for "is the machine faulted", the fault stream is for "what and where".
    a->add_raw("],\"source_ids\":[");
    for (std::size_t k = 0; k < f.source_ids.size(); ++k) {
      if (k != 0) a->add_raw(",");
      a->add_str(f.source_ids[k].c_str());
    }
    a->add_raw("]}");
  }
  a->add_raw("]");
}

// 11 S9.8.4 `cleared[]`: bare code STRINGS, not objects. CF-1 puts the regex
// on "每一个元素" of this list, so the element IS the code; CF-3 requires the
// byte-identical spelling the raise used, which is why f.code is forwarded and
// not re-formatted. The rest of the entry is deliberately dropped: a clear
// says one thing ("this code is no longer asserted") and the evidence fields
// were already delivered with the raise.
void write_cleared_array(Appender* a,
                       const std::vector<chs_a::FaultEntry>& list) {
  a->add_raw("[");
  for (std::size_t i = 0; i < list.size(); ++i) {
    if (i != 0) a->add_raw(",");
    a->add_str(list[i].code.c_str());
  }
  a->add_raw("]");
}

}  // namespace

std::size_t write_chassis_fault(const chs_a::FaultReport& in, char* out,
                              std::size_t cap) {
  Appender a(out, cap);
  // BOTH lists, always, including when one is empty. An empty `cleared` and an
  // absent `cleared` are different claims: the first says nothing was cleared
  // this report, the second says nothing was said -- and a consumer that has to
  // guess will keep a fault asserted forever.
  a.add_raw("{\"faults\":");
  write_fault_array(&a, in.faults);
  // NOT the same writer as faults[]. The two lists have different element
  // types in 11 S9.8.4 -- objects here, code strings there -- and calling one
  // routine for both is precisely how this key shipped object-valued cleared[]
  // until 2026-09-27.
  a.add_raw(",\"cleared\":");
  write_cleared_array(&a, in.cleared);
  // No fault_count / cleared_count. They were derivable from the two arrays'
  // own lengths, 11 S9.8.4 does not define them, and nothing ever read them;
  // a redundant copy of a fact is a second place for it to disagree. The
  // details / grouped / resources / source / source_ids fields above are the
  // opposite case and stay: 13 S7.3's disposition table requires each of them
  // be forwarded upstream ("现场定位靠它"), and nothing else on the wire
  // carries them.
  a.add_raw("}");
  return a.finish();
}

}  // namespace rt
}  // namespace quadruped
