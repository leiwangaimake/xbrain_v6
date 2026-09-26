/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: relay_keys.h
 * Brief: The hardcoded CR-1..CR-12 forwarding whitelist (11 S1.1.6 (3), CRL-3)
 *
 * Description:
 * This table IS the process. chassis_relay forwards exactly the twelve key
 * pairs 11 S1.1.6 (3) registers and nothing else; CRL-3 requires the list to
 * be "硬编码于代码, 不可配置, 不读配置文件" -- a configurable whitelist is a
 * general-purpose bridge between the two Zenoh planes, which is the exact
 * thing RT-C3 exists to forbid. The generated audit file
 * configs/generated/whitelist.yaml is COMPARED against this table at startup
 * (mismatch refuses to start, see relay_config.h) but never read as input --
 * the same "配置文件里的值仅供审计比对" stance 11 S2.4.7 takes for rt_override.
 *
 * Key spelling -- the one place this file deliberately differs from the
 * contract text. 11 S2.2 writes every key as xbrain/{rid}/<suffix>, but the
 * DEPLOYED general plane uses BARE keys: p2_core subscribes "state/robot"
 * (xbrain/p2_core/runtime/main_wiring.py, grep STATE_ROBOT_TOPIC), p5_gateway
 * publishes "cmd/estop" and "probe/estop/ping" bare, p1_motion publishes
 * "state/pose" bare, and scripts/dev/zenoh_echo.py documents the convention
 * ("GEN router; BARE keys like state/pose"). The RT plane DOES carry the full
 * prefix: quadruped builds xbrain/{rid}/rt/... (rt_keys.cc BuildKey) and
 * p1_motion subscribes "xbrain/%s/rt/chassis/state". A relay that publishes
 * prefixed keys on the general plane is heard by NOBODY in the running stack,
 * so this table stores gen keys bare and rt keys as suffixes for BuildRtKey.
 * The doc-vs-stack divergence is registered as a finding, not silently fixed.
 *
 * What does NOT belong here: payload knowledge. CRL-1 -- the relay never
 * interprets data, so the table carries no schema, no field list, no rate
 * enforcement. The rate column of the contract table is repeated in the
 * per-row note purely for the human reading a stats line.
 */

#ifndef HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_KEYS_H_
#define HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_KEYS_H_

#include <cstddef>
#include <string>

namespace chassis_relay {

// Direction of one whitelist row. Every row is one-way (RT-C3.c): the probe
// pair that needs both directions is two rows (CR-2 ping, CR-3 pong), never
// one bidirectional entry.
enum class Direction {
  kGenToRt,  // subscribe general plane, publish RT plane (CR-1, CR-2, CR-11)
  kRtToGen,  // subscribe RT plane, publish general plane (the other nine)
};

// One row of the CR table.
//
// qos_profile names a row of the frozen table in
// common/include/xbrain/zenoh/qos_profiles.h; it is the profile of the
// PUBLISH side of the forward. The subscribe side uses a direct callback
// (no channel handler), so the handler spec of the profile does not apply --
// there is no application queue for it to size.
//
// estop_exempt marks the one key whose forwarding is unconditional: 11 S3.0.1
// exempts cmd/estop from envelope validation because every malformed reading
// of it still collapses to "stop" (99 U75). A frame that cannot be re-enveloped
// is forwarded VERBATIM rather than dropped -- stopping must never be blocked
// by formatting. No other row may carry this flag: cmd/chassis/ctrl carries
// "enable", a relaxing action, and raw-forwarding a malformed one would be the
// "构造 payload 直穿对面" path RT-C3.e closes.
struct RelaySpec {
  const char* cr_id;        // "CR-1" .. "CR-12", as 11 S1.1.6 (3) numbers them
  Direction direction;      // which plane is subscribed, which published
  const char* gen_key;      // bare general-plane key (deployed spelling)
  const char* rt_suffix;    // RT-plane suffix under xbrain/{rid}/
  const char* qos_profile;  // publish-side profile name (11 S2.4.2)
  bool estop_exempt;        // CR-1 only; see above
  const char* note;         // contract rate/anchor, for stats lines and humans
};

// Exactly the twelve rows of 11 S1.1.6 (3), v0.6 (9 + CR-10/11/12).
// The count is a compile-time constant so per-row counter arrays can be sized
// from it; relay_keys.cc static_asserts the table length against it.
inline constexpr std::size_t kRelayCount = 12;

// The table itself (defined in relay_keys.cc, checked verbatim against
// docs/11 by test_relay_keys.cc the same way quadruped's test_rt_keys does).
extern const RelaySpec kRelayTable[kRelayCount];

// Root segment of every fully-qualified key (11 S2.1). Shared spelling with
// quadruped's rt_keys.cc on purpose: the two processes must compose the same
// bytes or they silently talk past each other.
inline constexpr const char* kKeyRoot = "xbrain";

// Compose "xbrain/{rid}/{suffix}" into a caller buffer. Returns the length
// written, or 0 when it does not fit -- a truncated key is well formed and
// matches nothing, which is the silent failure this file exists to prevent.
// Allocation-free; safe on any thread.
std::size_t BuildRtKey(const char* rid, const char* suffix, char* out,
                       std::size_t cap);

// Same composition, allocating. Startup only (declarations), never on the
// forward path. Throws std::length_error when the key does not fit.
std::string BuildRtKey(const std::string& rid, const std::string& suffix);

// Row lookup by contract id ("CR-4"), or nullptr. For tests and stats only;
// the forward path indexes the table directly.
const RelaySpec* FindRelaySpec(const char* cr_id);

}  // namespace chassis_relay

#endif  // HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_KEYS_H_
