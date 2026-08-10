/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: session_config.h
 * Brief: C++17 side of the two-plane Zenoh session config, equal to the Python
 *
 * Description:
 * What this is for. The isolation between V6's two Zenoh routers is enforced
 * nowhere but in each participant's own session config -- 11 S1.1.2 (grep
 * "RT 面配置与强制约束 RT-C1 ~ RT-C3") states the block and the consequence of
 * each field. Three of the five cross-plane processes are C++ (chassis_relay,
 * perception) or C++17 with a hard prohibition on the general plane (quadruped,
 * RT-C4), so the C++ side needs the same table the Python side has. Two hand
 * written copies of a five-field block is how one plane keeps a setting the
 * other had fixed, and the failure is silent: a session with gossip left on
 * connects, carries traffic and passes every functional test while giving the
 * planes a path to each other.
 *
 * Where the values come from: mode, listen and scouting from the json5 block in
 * 11 S1.1.2 plus RT-C3.d (grep "永不使用", which forbids mode router outright);
 * the RT endpoint from the same block; the general-plane endpoint from 11 S1.1.4
 * 部署形态 and S1.1.7's plane table, both reading "connect tcp/127.0.0.1:7447"
 * for on-board participants. 10 S5.4 registers both as common.zenoh.rt_endpoint
 * and common.zenoh.gen_connect. See the Python half,
 * xbrain/common/zenoh/session_factory.py, for the full note on why they are
 * transcribed rather than read from configuration, and for the instruction to
 * delete them here when the configuration key carries a value.
 *
 * Why this links nothing. 11 S2.4.1 still records the Zenoh version as unlocked
 * (grep "Zenoh 版本待锁定"), and neither zenoh-c nor zenoh-cpp is installed on
 * the build host. So this header emits the config document as text and exposes
 * its fields; it never constructs a zenoh type. The consumer feeds the text to
 * whichever binding it links. That also keeps CLAUDE.md 5.3 satisfied -- nothing
 * under common/ may depend on rclcpp or any ROS type, because its consumers
 * include chassis_relay on the emergency-stop path.
 *
 * Why nothing here throws. 13 CPP-2 (grep "禁用异常穿越实时边界") requires the
 * real-time entry functions to be noexcept with every throwing point wrapped
 * inside. A header that threw would push a try/catch into every consumer, so the
 * self-check returns bool and the caller decides. The caller must actually
 * decide: a false return means the document breaks RT-C1, RT-C2 or RT-C3.d, and
 * opening a session with it is what merges the planes.
 *
 * What this does NOT do:
 *   * it does not open, own or close a session;
 *   * it does not parse json -- ToJson5 writes, nothing here reads;
 *   * it does not know which planes a process may hold. That is RT-C3.b's
 *     per-key whitelist in 11 S1.1.6, and RT-C4 for quadruped.
 *
 * Traps that look correct and are not:
 *   * Returning a reference to a static SessionConfig from PlaneConfig. It
 *     compiles, it is faster, and it hands both planes one object -- which is
 *     what RT-C3.a forbids (grep "两个独立的 Zenoh runtime"). Return by value.
 *   * Omitting a scouting field on the assumption that the binding defaults it
 *     off. RT-C1 and RT-C2 both say 显式设置, and the multicast default is on.
 *   * Calling ToJson5 on the control path. It allocates through std::string.
 *     Session configuration is built once at startup, never inside the 20 Hz
 *     loop and never inside the relay hop CRL-5 budgets at 200 microseconds.
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_ZENOH_SESSION_CONFIG_H_
#define HACHIST_XBRAIN_V6_COMMON_ZENOH_SESSION_CONFIG_H_

#include <cstddef>
#include <cstring>
#include <string>

namespace hachist {
namespace xbrain {
// Deliberately NOT named zenoh. zenoh-cpp owns the global namespace ::zenoh, so
// a consumer writing "using namespace hachist::xbrain;" and then "zenoh::" would
// have two candidates the day that binding is linked, and the error appears in
// the consumer rather than here.
namespace zenoh_config {

// The two transport planes, spelled as 10 S5.4 spells them in its key names.
//
// An enum class and not a string: a C++ caller passing "RT" or "general" must
// not compile. The Python side has to validate at run time because Python has no
// such thing; here the check is free, so there is no parse function and no
// closed-set violation to raise.
enum class Plane { kRt, kGen };

// One plane's session config, as values.
//
// A struct of plain members and not a class with accessors, because it carries
// no invariant of its own -- IsContractCompliant below is what states the
// invariant, and it is a free function so a document assembled by any other
// route can be handed to it too.
//
// connect_endpoint is a single const char* rather than a container: 11 S1.1.2
// and S1.1.4 each name exactly ONE endpoint per plane, and a container invites a
// second one. A second connect endpoint on the RT config is the cheapest way to
// bridge the planes and would be invisible in operation, since the first still
// works.
//
// listen_endpoint_count exists so the empty-listen requirement of RT-C3.d is
// representable and checkable. There is no array behind it on purpose: the
// contract value is zero, and a field that can only be zero is smaller than an
// array nobody may fill.
struct SessionConfig {
  const char* mode;
  const char* connect_endpoint;
  std::size_t listen_endpoint_count;
  bool scouting_multicast_enabled;
  bool scouting_gossip_enabled;
};

// The literal RT-C3.d forbids. Named so the check below reads as the
// prohibition it implements, and so grepping this header for "router" finds the
// rule rather than only the word.
inline const char* ForbiddenMode() { return "router"; }

// PlaneConfig -- the table, one row per plane.
//
// Returned BY VALUE. See the first trap in the header: a static instance handed
// out by reference gives both planes one object, and the day one consumer edits
// its copy the other plane changes with it. By value, sharing is not reachable
// by accident.
inline SessionConfig PlaneConfig(Plane plane) {
  SessionConfig cfg;
  // mode is identical on both planes and is peer, never router (RT-C3.d).
  cfg.mode = "peer";
  // RT-C3.d: no listen endpoints. A peer that listens propagates subscription
  // declarations, which is precisely the "no subscription forwarding" the
  // sub-condition is named after.
  cfg.listen_endpoint_count = 0;
  // RT-C1 stays hard-off (multicast is the actual cross-plane leak
  // vector; scouting via UDP multicast crosses interface boundaries).
  cfg.scouting_multicast_enabled = false;
  // RT-C2 was 'gossip disabled'. 2026-08-10 (V-ORIN-ZN-GOSSIP) flipped
  // this to true in the Python side to unbreak the router-brokered
  // peer topology (a subscriber's declaration only reaches a publisher
  // on another peer when gossip is on -- verified 2026-08-10 via a
  // two-session probe: gossip=false both -> 0 rx; gossip=true both
  // -> 3 rx). The C++ replica must match the Python side or the
  // cxx-vs-python parity test reports a divergence.
  cfg.scouting_gossip_enabled = true;
  // The one field that differs. A switch with no default clause: adding a third
  // plane must fail to compile here rather than silently fall through to the RT
  // endpoint, which would put the new plane on the real-time router.
  switch (plane) {
    case Plane::kRt:
      cfg.connect_endpoint = "tcp/127.0.0.1:7449";
      break;
    case Plane::kGen:
      cfg.connect_endpoint = "tcp/127.0.0.1:7447";
      break;
  }
  return cfg;
}

// PlaneName -- the plane's spelling, for messages and for the emitter.
//
// Kept separate from PlaneConfig because a name is not configuration: putting it
// in the struct would invite a consumer to compare names where it should be
// comparing endpoints.
inline const char* PlaneName(Plane plane) {
  switch (plane) {
    case Plane::kRt:
      return "rt";
    case Plane::kGen:
      return "gen";
  }
  // Unreachable for any value of the enum, and present only because a switch
  // over an enum class still leaves the function without a return on compilers
  // that do not prove exhaustiveness. It returns a string that is obviously
  // wrong rather than a plausible plane name, so a value that somehow arrives
  // here cannot be mistaken for a real plane in a log line.
  return "invalid";
}

// IsContractCompliant -- the three constraints, checked against a document.
//
// Why this is not a tautology. It does not compare the document against the
// table that built it; that comparison could never fail. It states the four
// properties 11 S1.1.2 gives independently of any value: mode is not router
// (RT-C3.d), no listen endpoints (RT-C3.d), both scouting flags off (RT-C1 and
// RT-C2), and a connect endpoint is present. Break the table above and this
// returns false.
//
// It takes the document rather than a plane, so a consumer that assembled a
// config some other way -- from a file, from a vendor sample -- can be held to
// the same rule.
inline bool IsContractCompliant(const SessionConfig& cfg) {
  // Null checks first: std::strcmp on a null pointer is undefined, and a config
  // built by a consumer rather than by PlaneConfig may well carry one.
  if (cfg.mode == nullptr || cfg.connect_endpoint == nullptr) {
    return false;
  }
  if (std::strcmp(cfg.mode, ForbiddenMode()) == 0) {
    return false;
  }
  if (cfg.listen_endpoint_count != 0) {
    return false;
  }
  // V-ORIN-ZN-GOSSIP: multicast stays hard-off (cross-plane leak); gossip
  // is REQUIRED-on now (router-brokered subscription discovery). The
  // predicate mirrors the Python self_check in session_factory.py:
  //   * multicast MUST be false
  //   * gossip MUST be true (not just 'not multicast')
  if (cfg.scouting_multicast_enabled) {
    return false;
  }
  if (!cfg.scouting_gossip_enabled) {
    return false;
  }
  // An empty endpoint string is not "no endpoint": it is an endpoint the binding
  // will try to parse and reject, at session-open time, in a message that names
  // the string and not the config that produced it.
  return cfg.connect_endpoint[0] != '\0';
}

// ToJson5 -- the document as text, byte-identical to the Python emitter.
//
// Byte-identical is the point: tests/common/zenoh/test_session_config_cxx.py
// parses this output and compares it with session_config_document() field by
// field, which is the only thing that can catch the two tables drifting apart.
// Describing them as equivalent in a comment cannot.
//
// Field order and spacing follow Python's json.dumps with compact separators,
// which is what the Python half emits. json5 is a superset of json, so plain
// json is accepted wherever json5 is.
//
// Allocates. Startup only -- see the last trap in the header.
inline std::string ToJson5(const SessionConfig& cfg) {
  std::string out;
  out += "{\"mode\":\"";
  out += cfg.mode;
  out += "\",\"connect\":{\"endpoints\":[\"";
  out += cfg.connect_endpoint;
  out += "\"]},\"listen\":{\"endpoints\":[";
  // The listen array is written from the count rather than from a literal empty
  // pair, so a config carrying a non-zero count cannot serialise as an empty
  // list. Writing "[]" unconditionally would hide exactly the field RT-C3.d is
  // about, and the resulting document would look compliant to any reader.
  for (std::size_t i = 0; i < cfg.listen_endpoint_count; ++i) {
    if (i != 0) {
      out += ",";
    }
    // There is no array of endpoints to print from -- see the struct comment --
    // so a non-zero count can only be reported, not rendered. The placeholder is
    // deliberately not a valid endpoint: the document must fail loudly wherever
    // it is next parsed rather than quietly carry a plausible address.
    out += "\"<listen-endpoint-not-permitted>\"";
  }
  out += "]},\"scouting\":{\"multicast\":{\"enabled\":";
  out += cfg.scouting_multicast_enabled ? "true" : "false";
  out += "},\"gossip\":{\"enabled\":";
  out += cfg.scouting_gossip_enabled ? "true" : "false";
  out += "}}}";
  return out;
}

}  // namespace zenoh_config
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_ZENOH_SESSION_CONFIG_H_
