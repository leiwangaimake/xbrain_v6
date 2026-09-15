/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rt_keys.h
 * Brief: The RT-plane keys quadruped publishes and subscribes (11 S2.2.1)
 *
 * Description:
 * Every key this process touches, declared once as data. The table is the point
 * of the file -- more than the string building around it.
 *
 * Why a table and not a literal at each call site. A mistyped key produces a
 * process that starts, publishes without error, and is heard by nobody: the
 * subscriber's expression simply never matches. That is the failure 13 DDS-9
 * describes as indistinguishable from a dead network, and it is invisible in
 * every log on both sides. With the keys in one table, a test can take each one
 * and require it to appear verbatim in the contract -- which is what
 * test_rt_keys.cc does. A literal scattered across twenty call sites cannot be
 * checked that way, and the twentieth is the one that is wrong.
 *
 * The split between published and subscribed is also load-bearing. 11 S2.2.1
 * assigns exactly one publisher per key (F-1, publisher uniqueness), so a key
 * appearing in both lists here would mean this process talks to itself -- which
 * is either a copy-paste or a design mistake, and the test refuses it either
 * way.
 *
 * Boundary: this file composes key STRINGS. It opens no session, declares no
 * publisher and knows nothing about QoS -- the profiles live in
 * common/include/xbrain/zenoh/qos_profiles.h and the session in B4's transport.
 * Keeping the names separable from the transport is what lets the table be
 * compared against the contract without a Zenoh build.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_RT_KEYS_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_RT_KEYS_H_

#include <cstddef>
#include <string>

namespace quadruped {
namespace rt {

// 11 S2.1 and the Qt-facing spec both fix the first segment as "xbrain", and
// the second as {rid}. Named here so no call site spells either.
inline constexpr const char* kKeyRoot = "xbrain";

// What this process does with a key. Two values and not three: a key is either
// ours to produce or someone else's to produce, and 11 F-1 allows no third
// state.
enum class KeyRole {
  kPublish,
  kSubscribe,
};

// One row of the table. `suffix` is everything after "xbrain/{rid}/".
struct KeySpec {
  const char* suffix;
  KeyRole role;
  // The QoS profile name from common/include/xbrain/zenoh/qos_profiles.h.
  // Carried here so the key and its class of service are declared together:
  // 13 PB-Q1 puts the three Q0 keys on their own thread precisely because
  // mixing them with Q3 traffic destroys the emergency-stop budget, and a
  // reader has to be able to see which keys those are without a second lookup.
  const char* qos;
  // Why this key exists, in one line. Not decoration: the table is what a
  // reviewer compares against 11 S2.2.1, and a row without a reason is a row
  // nobody can check.
  const char* purpose;
};

// The complete set. Order follows 13 S7.1 (the four report streams, then the
// two aggregates) and then S7.1.1 (Q-1..Q-5), so the table reads in the same
// order as the design it implements.
extern const KeySpec kKeys[];
extern const std::size_t kKeyCount;

// Compose "xbrain/{rid}/{suffix}" into a caller buffer. Returns bytes written,
// or 0 when the buffer is too small -- never a truncated key, because a
// truncated key is a well-formed key that matches nothing, which is exactly the
// failure this file exists to prevent.
//
// Allocation-free: the RT publisher builds keys once at declaration time, but
// the same function is reachable from paths that must not allocate, and a
// second "fast" variant would be a second place for the format to drift.
std::size_t BuildKey(const char* rid, const char* suffix, char* out,
                     std::size_t cap);

// Convenience for setup code, where a std::string is what the Zenoh API wants
// anyway. Throws std::length_error rather than returning a truncated key.
std::string BuildKey(const std::string& rid, const std::string& suffix);

// Look up one row by suffix. Returns nullptr when the suffix is not declared --
// which a caller must treat as a defect, not as "use it anyway": publishing on
// an undeclared key is how a key escapes the table the contract is checked
// against.
const KeySpec* FindKey(const char* suffix);

}  // namespace rt
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_RT_KEYS_H_
