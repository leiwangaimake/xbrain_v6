/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: qos_profiles.h
 * Brief: C++17 side of the frozen QoS profile table and the QOS-C1 override
 *
 * Description:
 * What this is for. 11 S2.4.0 (grep "为什么 QoS 必须写进契约") states that a
 * process which does not set QoS explicitly inherits the binding defaults --
 * block, data, reliable, FIFO(256) -- and then names what each of those does
 * here: cmd_vel on block breaks a tick, quadruped sees cmd_age over 200 ms, and
 * Tier 1 locks the machine until an explicit enable. Four of the processes that
 * must set it are C++: chassis_relay and perception cross both planes,
 * quadruped is C++17 and forbidden the general plane by RT-C4, and rtk_driver
 * and teleop_input are still written 建议 C++ / 待定 in 11 S1.1.3. So the C++
 * side needs the same frozen table the Python side has, and two hand-written
 * copies of it is how one language keeps a value the other one changed. The
 * failure is silent: a publisher with the wrong priority still publishes.
 *
 * Where the values come from: 11 S2.4.2 (grep "档位表(冻结)") for the six-row
 * table, and the profiles and rt_override blocks of the json5 in 11 S2.4.7 for
 * the machine form. tests/common/zenoh/test_qos_profiles_cxx.py compiles this,
 * runs it, and compares the emitted table against FROZEN_PROFILES and
 * RT_OVERRIDE in xbrain/common/zenoh/qos.py field by field -- describing the two
 * as equivalent in a comment cannot catch a drift, and a drift here presents on
 * the robot as a subscriber that receives nothing.
 *
 * *** handler.depth 0 means NOT SUPPLIED, never "a queue of length zero". The
 * S2.4.7 field table gives it that meaning verbatim (0 = 无默认值, 必须由部署
 * 配置显式给出, 否则拒绝启动) and 11 S13.15 lists the condition under
 * E_QOS_VIOLATION. Q4_stream carries it today. HasSuppliedDepth exists so a
 * consumer cannot pass the 0 straight into a ring-buffer size: that would size
 * the buffer to nothing, drop every chunk, and log not one word about it.
 * S2.4.2 does now compute N = 10 for Q4 from a 200 ms jitter buffer over 20 ms
 * chunks, and it is deliberately NOT written here, because the same paragraph
 * keeps the value pending QoS-T8 and a number in code cannot later be told apart
 * from a number that was calibrated.
 *
 * What this header does NOT do:
 *   * it does not resolve a key against the S2.4.7 bindings. The bindings come
 *     from deployment configuration, not from the contract text, and a C++ copy
 *     of them would be a second source for the ordering that 11 S2.4.7 makes the
 *     entire safety argument out of. Python's xbrain/common/zenoh/qos.py owns
 *     resolution; a C++ consumer is registered per key in the S1.1.6 whitelist
 *     and applies the profile that resolution assigned it.
 *   * it does not check declarations against the S2.4.8 anti-pattern table. That
 *     is the startup self-check, INF-ZN-5 and INF-ZN-6.
 *   * it links no Zenoh binding and constructs no Zenoh type. 11 S2.4.1 still
 *     records the version as unlocked (grep "Zenoh 版本待锁定"), and the note
 *     there warns that 1.x moved reliability from the subscriber to the
 *     publisher: the SEMANTICS do not move, the API names may. So this header
 *     carries values, and the consumer maps them onto whichever binding it
 *     links.
 *   * it includes no ROS type and no rclcpp header, per CLAUDE.md 5.3. Its
 *     consumers include chassis_relay, which sits on the emergency-stop path
 *     under CRL-4 (no dynamic allocation, no blocking log) and CRL-5 (under
 *     200 microseconds per hop).
 *
 * Why nothing here throws. 13 CPP-2 (grep "禁用异常穿越实时边界") requires the
 * real-time entry points to be noexcept with every throwing point wrapped
 * inside, so a header that threw would push a try/catch into every consumer.
 * Lookups return a null pointer and checks return bool; the caller decides. The
 * caller must actually decide -- a null profile means the name is not in the
 * frozen table, and publishing without a profile is 11 S2.4.8 A-7.
 *
 * Traps that look correct and are not:
 *   * Comparing profile names or knob values with ==. These are const char*, so
 *     == compares addresses. Two identical string literals may or may not share
 *     storage, so the comparison is right on some builds and wrong on others.
 *     Use std::strcmp, as the functions below do.
 *   * Calling TableToJson on a control path. It allocates through std::string.
 *     It exists for the cross-language comparison and for startup dumps, never
 *     for the 20 Hz loop and never inside the CRL-5 relay hop.
 *   * Applying the override on the strength of the profile NAME. It is written
 *     here as the rule S2.4.3 states -- rt/ plus block -- so that a profile that
 *     acquired block later could not slip onto the RT plane by not being called
 *     Q3_cmd.
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_ZENOH_QOS_PROFILES_H_
#define HACHIST_XBRAIN_V6_COMMON_ZENOH_QOS_PROFILES_H_

#include <cstddef>
#include <cstring>
#include <string>

namespace hachist {
namespace xbrain {
// Deliberately NOT named zenoh, for the reason session_config.h states next
// door: zenoh-cpp owns the global namespace ::zenoh, so a consumer writing
// "using namespace hachist::xbrain;" would have two candidates for zenoh:: the
// day that binding is linked, and the ambiguity would be reported in the
// consumer rather than here.
namespace qos {

// The depth value 11 S2.4.7 gives the meaning "no default, deployment must
// supply it, otherwise refuse to start". Named rather than written as a bare 0
// at each comparison, because a bare 0 in this position reads as a queue size
// and that is exactly the misreading the sentinel has to survive.
inline constexpr int kDepthNotSupplied = 0;

// The subscriber-side queue. ring drops the oldest, fifo back-pressures the
// publisher; 11 S2.4.6 is the argument for why that difference decides whether
// lag is bounded or divergent, and why a FIFO under a periodic key also makes
// the frame-age check vacuous.
struct HandlerSpec {
  const char* kind;
  int depth;
};

// One row of the 11 S2.4.2 frozen table.
//
// Plain members and no accessors: the struct carries no invariant of its own.
// The invariants are the frozen values themselves, and they are established by
// the table below plus the cross-language comparison, not by this type.
struct QosProfile {
  const char* name;
  const char* congestion_control;
  const char* priority;
  const char* reliability;
  bool express;
  HandlerSpec handler;
};

// QOS-C1, as the three fields 11 S2.4.7 writes into rt_override.
//
// reliability and express are absent on purpose: the block in S2.4.7 does not
// carry them, so a key the override applies to keeps the reliability and express
// of the profile it matched. Adding them here would silently move Q3-rt away
// from its S2.4.2 row, which reads reliable and express false.
struct RtOverrideSpec {
  const char* congestion_control;
  const char* priority;
  HandlerSpec handler;
};

// The two congestion-control values of 11 S2.4.1, named so the QOS-C1 check
// below reads as the rule it implements rather than as a string comparison.
inline const char* CongestionBlock() { return "block"; }
inline const char* CongestionDrop() { return "drop"; }

// The plane whose keys QOS-C1 governs. It is the third chunk of
// xbrain/{robot_id}/{plane}/{domain}/{name} (11 S2.1).
inline const char* RtPlane() { return "rt"; }

// kProfiles -- the frozen table, one row per profile, in contract order.
//
// An inline constexpr variable, which C++17 added and which 13 CPP-1 therefore
// permits: it is one object across every translation unit, so consumers cannot
// end up with per-TU copies that a later edit updates only some of.
//
// The per-row reasons live with the Python table in
// xbrain/common/zenoh/qos.py, which is the half a reader reaches first; they are
// not repeated here, because two copies of an argument drift the same way two
// copies of a value do.
inline constexpr QosProfile kProfiles[] = {
    {"Q0_safety", "drop", "real_time", "reliable", true, {"ring", 8}},
    {"Q1_rt", "drop", "real_time", "best_effort", true, {"ring", 1}},
    {"Q2_state", "drop", "data_high", "reliable", false, {"ring", 4}},
    {"Q3_cmd", "block", "data", "reliable", false, {"fifo", 256}},
    // Q4's depth is the sentinel, not a size. See the header.
    {"Q4_stream", "drop", "interactive_high", "best_effort", false,
     {"ring", kDepthNotSupplied}},
};

// How many rows the table has.
//
// Derived from the array rather than written as a literal. A literal would be a
// second thing to update when a profile is added, and the failure mode of
// getting it wrong is reading past the end of the table -- which does not crash,
// it returns whatever follows in memory and calls it a profile.
inline constexpr std::size_t kProfileCount =
    sizeof(kProfiles) / sizeof(kProfiles[0]);

// Accessors, so consumers name the table through one pair of functions and a
// later change of storage does not reach them.
inline const QosProfile* ProfileTable() { return kProfiles; }
inline std::size_t ProfileCount() { return kProfileCount; }

// ProfileAt -- row by index, or nullptr past the end.
//
// Returns a pointer and not a reference so that an out-of-range index has an
// answer that is not undefined behaviour. 13 CPP-2 forbids throwing here, and
// clamping the index would be worse than either: it would hand back a real
// profile for a request that was already wrong.
inline const QosProfile* ProfileAt(std::size_t index) {
  if (index >= ProfileCount()) {
    return nullptr;
  }
  return ProfileTable() + index;
}

// FindProfile -- row by name, or nullptr.
//
// std::strcmp and not ==. See the first trap in the header: == on const char*
// compares addresses, and whether two identical literals share storage is up to
// the build.
inline const QosProfile* FindProfile(const char* name) {
  if (name == nullptr) {
    return nullptr;
  }
  for (std::size_t i = 0; i < ProfileCount(); ++i) {
    const QosProfile* profile = ProfileTable() + i;
    if (std::strcmp(profile->name, name) == 0) {
      return profile;
    }
  }
  // Not found is a real answer and the caller must handle it. Falling back to
  // any profile would be 11 S2.4.8 A-7 with extra steps: the publisher would
  // come up carrying QoS nobody chose for it.
  return nullptr;
}

// RtOverride -- QOS-C1, hard-coded.
//
// 11 S2.4.7's field table says so in as many words: 硬编码在实现中, 配置文件里
// 的值仅供审计比对. That is the whole point of the row. A deployment that could
// switch it off would restore block on the RT plane, and S2.4.3 gives block's
// blocking time no upper bound at all -- one blocked put swallows a whole 50 ms
// control period.
inline RtOverrideSpec RtOverride() {
  RtOverrideSpec spec;
  spec.congestion_control = "drop";
  spec.priority = "interactive_high";
  spec.handler.kind = "fifo";
  spec.handler.depth = 32;
  return spec;
}

// RequiresRtOverride -- does QOS-C1 apply to this key's profile.
//
// Written as the rule 11 S2.4.3 states -- a key on the rt/ plane whose
// congestion control would be block -- rather than as "the profile is called
// Q3_cmd". The two select the same rows of today's frozen table, since Q3_cmd is
// the only one carrying block; expressing it as the rule leaves no branch that
// can never be reached, and no way for a future block-carrying profile to reach
// the RT plane by not having that name.
inline bool RequiresRtOverride(const char* plane, const QosProfile& profile) {
  if (plane == nullptr || profile.congestion_control == nullptr) {
    return false;
  }
  return std::strcmp(plane, RtPlane()) == 0 &&
         std::strcmp(profile.congestion_control, CongestionBlock()) == 0;
}

// HasSuppliedDepth -- has deployment given this handler a depth.
//
// The guard that stops the sentinel from becoming a queue size. A consumer that
// reads handler.depth without asking this first sizes a Q4 ring buffer to zero,
// which drops every audio chunk and reports nothing -- the fail-silent direction
// CLAUDE.md 3.1 exists to close.
inline bool HasSuppliedDepth(const HandlerSpec& handler) {
  return handler.depth != kDepthNotSupplied;
}

// AppendJsonHandler -- one handler block, as text.
//
// Split out of TableToJson so that function stays under the 40-line limit of
// CLAUDE.md 5.1 and so the depth sentinel is rendered in exactly one place.
inline void AppendJsonHandler(const HandlerSpec& handler, std::string* out) {
  *out += "{\"kind\":\"";
  *out += handler.kind;
  *out += "\",\"depth\":";
  // null, not 0. The Python half holds this as MISSING, and emitting 0 would
  // make the two sides compare unequal for the right reason but describe the
  // state wrongly for anyone reading the dump: 0 reads as a size.
  if (HasSuppliedDepth(handler)) {
    *out += std::to_string(handler.depth);
  } else {
    *out += "null";
  }
  *out += "}";
}

// TableToJson -- the whole frozen table plus the override, as text.
//
// Exists for one purpose: tests/common/zenoh/test_qos_profiles_cxx.py parses it
// and compares it with the Python table. That comparison is the only thing that
// can catch the two transcriptions drifting, since both are transcribed from the
// same section of 11 and both stay green while one of them is stale.
//
// Allocates. Startup and tooling only -- see the second trap in the header.
inline std::string TableToJson() {
  std::string out = "{\"profiles\":{";
  for (std::size_t i = 0; i < ProfileCount(); ++i) {
    const QosProfile& profile = *(ProfileTable() + i);
    if (i != 0) {
      out += ",";
    }
    out += "\"";
    out += profile.name;
    out += "\":{\"congestion_control\":\"";
    out += profile.congestion_control;
    out += "\",\"priority\":\"";
    out += profile.priority;
    out += "\",\"reliability\":\"";
    out += profile.reliability;
    out += "\",\"express\":";
    out += profile.express ? "true" : "false";
    out += ",\"handler\":";
    AppendJsonHandler(profile.handler, &out);
    out += "}";
  }
  const RtOverrideSpec override_spec = RtOverride();
  out += "},\"rt_override\":{\"congestion_control\":\"";
  out += override_spec.congestion_control;
  out += "\",\"priority\":\"";
  out += override_spec.priority;
  out += "\",\"handler\":";
  AppendJsonHandler(override_spec.handler, &out);
  out += "}}";
  return out;
}

}  // namespace qos
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_ZENOH_QOS_PROFILES_H_
