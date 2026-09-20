/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: quadruped_config.cc
 * Brief: Resolved quadruped.yaml -> QuadrupedConfig, fail-stop on any null (3.1)
 *
 * Description:
 * Every read goes through yaml_lite's require_* accessors, which throw with the
 * dotted key path on missing / null / unparsable. This file adds nothing but
 * ordering and the three invariants the running process depends on:
 *
 *   1. the odom staleness thresholds are strictly increasing -- they select
 *      four behaviours (publish, warn, invalidate, stop publishing) and an
 *      out-of-order pair makes one of them unreachable, silently;
 *   2. the control period is positive -- it becomes a sleep duration, and a
 *      zero there is a busy loop on a SCHED_FIFO thread, which on an
 *      isolated core is a hang with no log line;
 *   3. at least one endpoint candidate is enabled -- with none, the probe
 *      loop completes "successfully" having contacted nothing, and the
 *      process would sit reporting conn=lost forever with a config that
 *      looks fine.
 *
 * Why only three. Assertion K (10 S5.4.4) already evaluates QC-1..QC-17 at
 * freeze time over this same file; re-checking those here would be a second
 * copy of a rule owned elsewhere, and two copies drift. These three are not in
 * K -- they are process-side preconditions, and they are cheap to state here
 * where the values are already in hand.
 *
 * Ordering is deliberate: identity, then link, then the two domains, then odom,
 * then Tier 1. The first uncalibrated null names itself and stops the process,
 * so the order decides WHICH key an operator sees first -- and the most likely
 * one today is tier1.limits.* (V-01 not measured), which is last so that every
 * cheaper mistake is reported before it.
 */

#include "quadruped/quadruped_config.h"

#include "quadruped/chs_a_reports.h"

#include <sstream>

#include "quadruped/chs_a_codec.h"
#include "xbrain/config/yaml_lite.h"

namespace quadruped {

namespace {

using xbrain::config::YamlNode;

// Prefix every key path with the namespace the resolved file carries, so a
// thrown message names the key exactly as it appears in the file. Writing
// "tier1.limits.max_vx_mps" when the file says "quadruped.tier1.limits.
// max_vx_mps" costs the operator one failed grep.
const char* kNs = "quadruped.";

std::string K(const std::string& rest) { return std::string(kNs) + rest; }

// One endpoint row. Reads through require_* so a malformed row names its field;
// the row index is added by the caller because yaml_lite has no notion of where
// a node came from.
// 11 S2.2.11: [a-z0-9_-]{1,32}. Hand-checked rather than <regex>, which pulls
// in a large amount of code and can throw -- this runs once at load, and the
// pattern is four character classes wide. Written as the shape it accepts so a
// reader can compare it with the contract line directly.
bool IsValidRobotId(const std::string& id) {
  if (id.empty() || id.size() > 32) return false;
  for (char c : id) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                    c == '_' || c == '-';
    if (!ok) return false;
  }
  return true;
}

EndpointCandidate ReadEndpoint(const YamlNode& row) {
  EndpointCandidate ep;
  ep.proto = row.require_string("proto");
  ep.host = row.require_string("host");
  const long port = row.require_int("port");
  ep.port = static_cast<int>(port);
  ep.tls = row.require_bool("tls");
  ep.enabled = row.require_bool("enabled");
  return ep;
}

}  // namespace

const char* DefaultResolvedPath() {
  return "/opt/xbrain_v6/data/run/resolved/quadruped.yaml";
}

QuadrupedConfig LoadQuadrupedConfig(const std::string& path) {
  QuadrupedConfig cfg;
  // yaml_lite throws std::runtime_error; translate at this one boundary so
  // callers catch a single type and main() can map it to EX_CONFIG.
  try {
    const YamlNode root = xbrain::config::LoadYamlFile(path);

    cfg.robot_id = root.require_string(K("robot_id"));
    // 11 S2.2.11 / FLT-02: this value is the {rid} segment of EVERY Zenoh key
    // this process publishes or subscribes. The freeze line already enforces
    // the charset (assertion D-1 in xbrain/boot/freeze/assertions/d_identity.py)
    // and remains the authority; this is a second gate at the point of USE.
    //
    // It earns its place because of how the failure looks. A rid with a stray
    // character produces key names that are perfectly well-formed and simply
    // match nothing: the process starts, publishes happily, and every
    // subscriber stays silent -- the same picture as an unplugged cable, and
    // the same class of failure 13 DDS-9 exists to make distinguishable.
    // Catching it here names the one key path instead.
    if (!IsValidRobotId(cfg.robot_id)) {
      throw ConfigError(
          "quadruped config: robot_id \"" + cfg.robot_id + "\" does not match "
          "[a-z0-9_-]{1,32} (11 S2.2.11 FLT-02, freeze assertion D-1). It is "
          "the {rid} segment of every key this process uses, and a malformed "
          "one produces keys that match nothing while the process reports "
          "itself healthy");
    }

    // ---- channel one -------------------------------------------------
    const YamlNode& eps = root.require_seq(K("chassis_link.endpoint_candidates"));
    for (std::size_t i = 0; i < eps.size(); ++i) {
      cfg.link.endpoints.push_back(ReadEndpoint(eps.at_index(i)));
    }
    cfg.link.probe_timeout_ms =
        static_cast<int>(root.require_int(K("chassis_link.probe_timeout_ms")));
    cfg.link.heartbeat_hz = root.require_double(K("chassis_link.heartbeat_hz"));
    cfg.link.codebook = root.require_string(K("chassis_link.codebook"));
    cfg.link.resync_max_bytes =
        static_cast<int>(root.require_int(K("chassis_link.resync_max_bytes")));
    cfg.link.frame_assembly_timeout_ms = static_cast<int>(
        root.require_int(K("chassis_link.frame_assembly_timeout_ms")));
    cfg.link.partial_send_retry =
        static_cast<int>(root.require_int(K("chassis_link.partial_send_retry")));
    cfg.link.tcp_nodelay = root.require_bool(K("chassis_link.tcp_nodelay"));
    cfg.link.axis_cmd_hz = root.require_double(K("chassis_link.axis_cmd_hz"));
    cfg.link.cmd_fail_threshold =
        static_cast<int>(root.require_int(K("chassis_link.cmd_fail_threshold")));
    cfg.link.state_timeout_degraded_s =
        root.require_double(K("chassis_link.state_timeout_degraded_s"));
    cfg.link.state_timeout_lost_s =
        root.require_double(K("chassis_link.state_timeout_lost_s"));
    // The reconnect ladder. Read through the node-level accessor because a
    // sequence entry has no key of its own; before that accessor existed this
    // list sat in the config unread, which is the quiet half of the same
    // defect 13 S8.2 v1.4 fixed on the tier1 limits.
    const YamlNode& backoff = root.require_seq(K("chassis_link.reconnect_backoff_s"));
    for (std::size_t i = 0; i < backoff.size(); ++i) {
      const std::string label =
          K("chassis_link.reconnect_backoff_s") + "[" + std::to_string(i) + "]";
      cfg.link.reconnect_backoff_s.push_back(backoff.at_index(i).as_double(label));
    }
    cfg.link.axis_cmd_socket_fixed =
        root.require_bool(K("chassis_link.axis_cmd_socket_fixed"));
    cfg.link.single_tx_owner =
        root.require_bool(K("chassis_link.single_tx_owner"));
    cfg.link.proto_version_byte =
        static_cast<int>(root.require_int(K("chassis_link.proto_version_byte")));
    cfg.link.asdu_format = root.require_string(K("chassis_link.asdu_format"));
    // Counted, not copied: nothing consumes the legacy table yet, and 9.3
    // forbids writing the consumer before there is something to consume. The
    // count is what QC-13 needs.
    const YamlNode& legacy =
        root.at(K("chassis_link.codebook_table.legacy_decimal"));
    cfg.link.legacy_decimal_entries = legacy.is_map() ? legacy.items().size() : 0;

    // ---- motion axes: declared here, FIXED by the contract ------------
    //
    // Neither list is loaded as data. Both describe behaviour 11 S9.3.1 and
    // 13 S5.4 fix, and Tier 1 implements it directly -- so the only honest
    // thing to do with them is CHECK them. A key an operator can set and that
    // changes nothing is worse than no key, because the operator who set it
    // believes something changed. That exact defect has already been found
    // twice in this package (13 S8.2 v1.4, and the six link keys of B2).
    {
      const YamlNode& active = root.require_seq(K("motion.axes.always_active"));
      const char* kExpected[] = {"vx", "vy", "wz"};
      bool matches = active.size() == 3;
      for (std::size_t i = 0; matches && i < 3; ++i) {
        const std::string label =
            K("motion.axes.always_active") + "[" + std::to_string(i) + "]";
        matches = active.at_index(i).as_scalar(label) == kExpected[i];
      }
      if (!matches) {
        throw ConfigError(
            "quadruped config: motion.axes.always_active must be exactly "
            "[vx, vy, wz] (11 S9.3.1 / 13 S5.4). Tier 1 implements that axis "
            "set directly, so a different list here would change nothing and "
            "leave whoever edited it believing it had");
      }
      // 13 V-51 leaves "special gait" undefined, and there is a second gap it
      // does not mention: spec.* defines max_vx_mps / max_vy_mps /
      // max_wz_radps and NOTHING for vz / v_roll / v_pitch. Tier 1 exists to
      // hold a command inside a limit, so an axis with no limit cannot be let
      // through -- it zeroes all three unconditionally. Enabling a gait here
      // would therefore do nothing at all, which this refuses rather than
      // performs.
      const YamlNode& special = root.require_seq(K("motion.axes.special_gaits"));
      if (special.size() != 0) {
        throw ConfigError(
            "quadruped config: motion.axes.special_gaits must be empty. "
            "13 V-51 does not define which gaits are 'special', and spec.* "
            "defines no limit for vz / v_roll / v_pitch at all -- Tier 1 zeroes "
            "those three unconditionally because it has nothing to clamp them "
            "against. Filling this list would change no behaviour; the limits "
            "have to exist in spec.* first");
      }
    }

    // ---- 13 PR-1 / QC-9 / MS-2 / TR-1: the motion block --------------
    //
    // *** prone_forbidden_gaits was NEVER READ. The process built its
    // ModeConfig from a lambda that took cfg and threw it away, so the list
    // stayed empty -- and ProneAllowed answers !Contains(list, gait), which is
    // true for every gait when the list is empty. PR-1 therefore never fired:
    // `prone` was accepted on a staircase, and 13 V-54 calls that a safety
    // incident in as many words.
    {
      const YamlNode& gaits = root.require_seq(K("motion.prone_forbidden_gaits"));
      for (std::size_t i = 0; i < gaits.size(); ++i) {
        const std::string label =
            K("motion.prone_forbidden_gaits") + "[" + std::to_string(i) + "]";
        const std::string name = gaits.at_index(i).as_scalar(label);
        std::int64_t value = 0;
        // A name outside 13 S5.3's five is refused rather than skipped. A
        // skipped entry is a gait the operator believes is forbidden and is
        // not -- the same silence PR-1 already suffered from, one layer up.
        if (!chs_a::GaitValueByName(name, &value)) {
          throw ConfigError(
              label + " = \"" + name +
              "\" is not one of the gaits in 13 S5.3 (basic / platform / "
              "stair_standard / flat / stair_agile). A name that resolves to "
              "nothing would leave that gait ALLOWED for prone while the "
              "config says it is forbidden");
        }
        cfg.motion.prone_forbidden_gaits.push_back(value);
      }
      // QC-9, verbatim: the list MUST contain both stair gaits; widening is
      // allowed, narrowing is not, and narrowing refuses startup.
      //
      // Checked against chs_a::IsStairGait rather than against two literals
      // here: that predicate sits next to the gait table, so a stair gait
      // added there is required here without an edit -- which is the whole
      // point of "允许改宽不允许改窄".
      for (const std::int64_t stair : {static_cast<std::int64_t>(0x1003),
                                       static_cast<std::int64_t>(0x3003)}) {
        bool found = false;
        for (const std::int64_t g : cfg.motion.prone_forbidden_gaits) {
          if (g == stair) found = true;
        }
        if (!found) {
          throw ConfigError(
              K("motion.prone_forbidden_gaits") +
              " must contain BOTH stair gaits (stair_agile 0x3003 and "
              "stair_standard 0x1003) -- 13 QC-9 allows widening this list and "
              "forbids narrowing it. 13 GS-3 is why stair_standard belongs "
              "here even though we never command it: the factory handset can "
              "set it, and PR-1 refuses prone on whatever the chassis REPORTS. "
              "13 V-54: there is no anti-rollover path for prone on stairs");
        }
      }
    }
    // 13 GS-1: the gaits this build refuses to COMMAND. Same name resolution
    // and the same refuse-do-not-skip rule as the prone list above.
    {
      const YamlNode& ni = root.require_seq(K("motion.not_implemented.gaits"));
      for (std::size_t i = 0; i < ni.size(); ++i) {
        const std::string label =
            K("motion.not_implemented.gaits") + "[" + std::to_string(i) + "]";
        const std::string name = ni.at_index(i).as_scalar(label);
        std::int64_t value = 0;
        if (!chs_a::GaitValueByName(name, &value)) {
          throw ConfigError(label + " = \"" + name +
                            "\" is not one of the gaits in 13 S5.3. A name "
                            "that resolves to nothing would leave that gait "
                            "COMMANDABLE while the config says it is not");
        }
        cfg.motion.command_forbidden_gaits.push_back(value);
      }
      // GS-1 is a v0.2 定案, not a preference: 0x1003 can be commanded and can
      // NEVER be read back (13 G-02, "读回枚举中无此值"), so commanding it
      // means the read-back check can only time out -- MS-2 必然判超时. A
      // config that removed it would make every stair_standard request end in
      // a five-second failure instead of an immediate, honest refusal.
      bool has_standard = false;
      for (const std::int64_t g : cfg.motion.command_forbidden_gaits) {
        if (g == 0x1003) has_standard = true;
      }
      if (!has_standard) {
        throw ConfigError(
            K("motion.not_implemented.gaits") +
            " must contain stair_standard (0x1003). 13 GS-1 refuses to command "
            "it because 13 G-02 records that it can never be read back: "
            "commanding it leaves the read-back check with nothing to match, "
            "so MS-2 turns every such request into a timeout. Removing it here "
            "does not enable the gait, it only replaces an immediate "
            "E_NOT_IMPLEMENTED with a five-second failure");
      }
    }
    cfg.motion.mode_switch_timeout_s =
        root.require_double(K("motion.mode_switch_timeout_s"));
    cfg.motion.external_transition_hold_s =
        root.require_double(K("motion.external_transition_hold_s"));
    // Both were literals in the process constructor while these keys sat in
    // the config doing nothing -- "填了不生效 = 让设置的人以为改了什么", the
    // same sentence 13 v1.7 used for special_gaits.
    if (!(cfg.motion.mode_switch_timeout_s > 0.0)) {
      throw ConfigError(K("motion.mode_switch_timeout_s") +
                        " must be positive: a zero or negative window makes "
                        "every mode switch fail on the period it is requested");
    }
    if (!(cfg.motion.external_transition_hold_s > 0.0)) {
      throw ConfigError(K("motion.external_transition_hold_s") +
                        " must be positive: 13 TR-1 holds the robot at zero "
                        "for this long after an EXTERNAL triple change, and a "
                        "zero hold releases it on the same period");
    }

    // ---- channel two: chassis DDS domain 0 ---------------------------
    cfg.dds.backend = root.require_string(K("chassis_dds.backend"));
    cfg.dds.domain_id =
        static_cast<int>(root.require_int(K("chassis_dds.domain_id")));
    // require_string, not an optional read: see the field's comment. A missing
    // NIC name is a config error that fails at startup with the key path, and
    // the alternative is a participant that binds the wrong interface and is
    // silent forever.
    //
    // There is deliberately NO extra empty-string check here. yaml_lite maps an
    // empty scalar to null, so `network_interface: ""` is refused by
    // require_string with the CLAUDE.md 3.1 message that names the key path --
    // a better message than one written here, and one behaviour instead of two.
    // A hand-written check was tried and removed: it was unreachable through
    // YAML, which the mutant run showed by surviving. ChassisDds keeps its own
    // guard, because that class can be constructed without this loader.
    cfg.dds.network_interface =
        root.require_string(K("chassis_dds.network_interface"));
    cfg.dds.imu_topic = root.require_string(K("chassis_dds.imu_topic"));
    cfg.dds.imu_expect_hz = root.require_double(K("chassis_dds.imu_expect_hz"));
    cfg.dds.imu_age_warn_ms =
        static_cast<int>(root.require_int(K("chassis_dds.imu_age_warn_ms")));
    cfg.dds.imu_frame_id = root.require_string(K("chassis_dds.imu_frame_id"));
    cfg.dds.forward_imu_to_rt =
        root.require_bool(K("chassis_dds.forward_imu_to_rt"));
    // imu_rt_key is legitimately the empty string while forward_imu_to_rt is
    // false (D-44), and require_string treats "" as null -- so read the node
    // directly and let QC-14 (freeze time) own the paired-value rule.
    {
      const YamlNode& key_node = root.at(K("chassis_dds.imu_rt_key"));
      cfg.dds.imu_rt_key = key_node.is_null() ? std::string()
                                              : key_node.require_string("");
    }

    // ---- channel three: uplink (ROS 2 domain 42 + RT plane) ----------
    cfg.uplink.zenoh_rt_endpoint =
        root.require_string(K("uplink.zenoh_rt_endpoint"));
    cfg.uplink.ros_domain_id =
        static_cast<int>(root.require_int(K("uplink.ros_domain_id")));
    cfg.uplink.rmw = root.require_string(K("uplink.rmw"));
    cfg.uplink.odom_topic = root.require_string(K("uplink.odom_topic"));
    cfg.uplink.odom_frame = root.require_string(K("uplink.odom_frame"));
    cfg.uplink.base_frame = root.require_string(K("uplink.base_frame"));
    cfg.uplink.publish_odom_tf = root.require_bool(K("uplink.publish_odom_tf"));

    // ---- odometry ----------------------------------------------------
    cfg.odom.publish_hz = root.require_double(K("odom.publish_hz"));
    cfg.odom.vel_deadzone_mps = root.require_double(K("odom.vel_deadzone_mps"));
    cfg.odom.gyro_deadzone_radps =
        root.require_double(K("odom.gyro_deadzone_radps"));
    cfg.odom.sigma_v0_mps = root.require_double(K("odom.sigma_v0_mps"));
    cfg.odom.gyro_bias_radps = root.require_double(K("odom.gyro_bias_radps"));
    cfg.odom.arw_rad_sqrt_s = root.require_double(K("odom.arw_rad_sqrt_s"));
    // a_max and trust_by_gait are ${common.spec.*} references (13 S8.2 v1.4).
    // They are read exactly like a local value: by the time this file is read
    // the freeze line has already expanded them, and an unexpanded reference
    // would arrive as the literal "${common...}" and fail require_double with
    // the key path -- which is the correct, loud outcome.
    cfg.odom.a_max_mps2 = root.require_double(K("odom.a_max_mps2"));
    cfg.odom.trust_flat = root.require_double(K("odom.trust_by_gait.flat"));
    cfg.odom.trust_stair = root.require_double(K("odom.trust_by_gait.stair"));
    cfg.odom.stale_warn_ms =
        static_cast<int>(root.require_int(K("odom.stale_warn_ms")));
    cfg.odom.stale_invalid_ms =
        static_cast<int>(root.require_int(K("odom.stale_invalid_ms")));
    cfg.odom.stale_stop_publish_ms =
        static_cast<int>(root.require_int(K("odom.stale_stop_publish_ms")));

    // ---- Tier 1 ------------------------------------------------------
    cfg.tier1.cmd_timeout_ms =
        static_cast<int>(root.require_int(K("tier1.cmd_timeout_ms")));
    cfg.tier1.estop_ack_ms =
        static_cast<int>(root.require_int(K("tier1.estop_ack_ms")));
    cfg.tier1.estop_dedup_ms =
        static_cast<int>(root.require_int(K("tier1.estop_dedup_ms")));
    cfg.tier1.control_loop_hz = root.require_double(K("tier1.control_loop_hz"));
    cfg.tier1.safety_probe_stale_s =
        root.require_double(K("tier1.safety_probe_stale_s"));
    // Last on purpose: these are the values most likely to be null today
    // (common.spec.max_* pending V-01), and stopping here means every cheaper
    // configuration mistake has already been reported.
    cfg.tier1.limits.max_vx_mps = root.require_double(K("tier1.limits.max_vx_mps"));
    cfg.tier1.limits.max_vy_mps = root.require_double(K("tier1.limits.max_vy_mps"));
    cfg.tier1.limits.max_wz_radps =
        root.require_double(K("tier1.limits.max_wz_radps"));
    cfg.tier1.limits.max_accel_mps2 =
        root.require_double(K("tier1.limits.max_accel_mps2"));
    cfg.tier1.limits.max_decel_mps2 =
        root.require_double(K("tier1.limits.max_decel_mps2"));
    cfg.tier1.limits.holonomic = root.require_bool(K("tier1.limits.holonomic"));
  } catch (const std::exception& e) {
    throw ConfigError(std::string("quadruped config: ") + e.what());
  }

  // ---- process-side invariants (see the header for why only these) ----
  if (!(cfg.odom.stale_warn_ms < cfg.odom.stale_invalid_ms &&
        cfg.odom.stale_invalid_ms < cfg.odom.stale_stop_publish_ms)) {
    std::ostringstream m;
    m << "quadruped config: odom staleness thresholds must strictly increase, "
      << "got warn=" << cfg.odom.stale_warn_ms
      << " invalid=" << cfg.odom.stale_invalid_ms
      << " stop=" << cfg.odom.stale_stop_publish_ms
      << " (13 S4.4 four bands; an out-of-order pair makes one band unreachable)";
    throw ConfigError(m.str());
  }
  if (!(cfg.tier1.control_loop_hz > 0.0)) {
    throw ConfigError(
        "quadruped config: tier1.control_loop_hz must be > 0 "
        "(it becomes the control period; zero is a busy loop on a FIFO thread)");
  }
  bool any_enabled = false;
  for (const EndpointCandidate& ep : cfg.link.endpoints) {
    if (ep.enabled) {
      any_enabled = true;
      break;
    }
  }
  if (!any_enabled) {
    throw ConfigError(
        "quadruped config: no enabled endpoint candidate in "
        "chassis_link.endpoint_candidates (the probe would contact nothing and "
        "report conn=lost forever with a config that looks complete)");
  }
  // The reconnect ladder must exist and every rung must be positive. An empty
  // ladder leaves the session with no delay to apply after a drop; a zero rung
  // reconnects as fast as the CPU allows, against a chassis that is already
  // failing -- and the chassis plays a voice prompt and switches its LEDs on
  // every connect (13 CA-6), so the failure is audible in the room.
  if (cfg.link.reconnect_backoff_s.empty()) {
    throw ConfigError(
        "quadruped config: chassis_link.reconnect_backoff_s is empty "
        "(13 S8.2 requires a ladder; with none there is no delay to apply "
        "after a drop and the reconnect becomes a busy loop)");
  }
  for (std::size_t i = 0; i < cfg.link.reconnect_backoff_s.size(); ++i) {
    if (!(cfg.link.reconnect_backoff_s[i] > 0.0)) {
      std::ostringstream m;
      m << "quadruped config: chassis_link.reconnect_backoff_s[" << i
        << "] must be > 0, got " << cfg.link.reconnect_backoff_s[i]
        << " (a zero rung is a busy reconnect loop, and 13 CA-6 makes every "
        << "reconnect play a voice prompt on the chassis)";
      throw ConfigError(m.str());
    }
  }
  // 13 CA-1 and QC-16 both say these are not switchable. Reading them and
  // refusing a false value is the difference between a documented constraint
  // and an enforced one: a key that accepts false and changes nothing tells
  // the operator who set it that something changed.
  if (!cfg.link.axis_cmd_socket_fixed) {
    throw ConfigError(
        "quadruped config: chassis_link.axis_cmd_socket_fixed must be true "
        "(13 CA-1: a new socket reads to the chassis as a NEW CLIENT and axis "
        "commands come back 0xE006 -- the robot accepts commands and does not "
        "move, which looks like a mechanical fault)");
  }
  if (!cfg.link.single_tx_owner) {
    throw ConfigError(
        "quadruped config: chassis_link.single_tx_owner must be true "
        "(13 CA-4 / QC-16: two threads writing the same TCP socket interleave "
        "two APDU frames into one illegal message, seen as random 0xE001)");
  }
  // The header bytes the codec compiles in. Catching a disagreement here costs
  // one clear message at startup; catching it on the wire costs a 0xE002 that
  // 13 S7.5 attributes to the encoder.
  if (cfg.link.proto_version_byte != static_cast<int>(chs_a::kProtoVersion)) {
    std::ostringstream m;
    m << "quadruped config: chassis_link.proto_version_byte = "
      << cfg.link.proto_version_byte << " but the codec writes "
      << static_cast<int>(chs_a::kProtoVersion)
      << " (13 S2.2 header[10]; the two must agree or every frame carries a "
      << "version the chassis did not sanction)";
    throw ConfigError(m.str());
  }
  if (cfg.link.asdu_format != "json") {
    throw ConfigError(
        "quadruped config: chassis_link.asdu_format must be \"json\" "
        "(13 S2.2: the codec writes format byte 0x01 and renders JSON; "
        "\"xml\" would need an encoder that does not exist)");
  }
  // CB-1 / QC-13. Two rules, and they are NOT the same rule: the codebook must
  // be the one the code implements, and the legacy table must be all-or-
  // nothing so a half-filled table can never be selected.
  if (cfg.link.codebook != "hex32") {
    throw ConfigError(
        "quadruped config: chassis_link.codebook must be \"hex32\" "
        "(13 CB-1: it is the only table the vendor documents, and the legacy "
        "decimal codes for the five commands do not exist in any manual)");
  }
  if (cfg.link.legacy_decimal_entries != 0 &&
      cfg.link.legacy_decimal_entries != kLegacyCodebookEntries) {
    std::ostringstream m;
    m << "quadruped config: chassis_link.codebook_table.legacy_decimal holds "
      << cfg.link.legacy_decimal_entries << " entries; it must be empty or "
      << "hold all " << kLegacyCodebookEntries
      << " (13 QC-13: heartbeat, usage mode, motion state, gait, axis -- a "
      << "half-filled table would be selectable and fail on the first command "
      << "it does not cover)";
    throw ConfigError(m.str());
  }
  return cfg;
}

std::string DescribeConfig(const QuadrupedConfig& cfg) {
  // DDS-9 / CB-4: the effective values, printed at startup and carried into
  // hello_ack.runtime.transport. Two domain ids that silently became equal, or
  // a codebook that is not the one the operator believes, both present as "the
  // participant is up and not one packet arrives" -- indistinguishable from a
  // dead cable unless the process says what it is actually using.
  std::ostringstream o;
  o << "quadruped runtime.transport:\n";
  o << "  rid=" << cfg.robot_id << "\n";
  o << "  chassis_dds.domain_id=" << cfg.dds.domain_id
    << " backend=" << cfg.dds.backend << " imu_topic=" << cfg.dds.imu_topic
    << "\n";
  o << "  uplink.ros_domain_id=" << cfg.uplink.ros_domain_id
    << " rmw=" << cfg.uplink.rmw << " odom_topic=" << cfg.uplink.odom_topic
    << "\n";
  o << "  uplink.zenoh_rt_endpoint=" << cfg.uplink.zenoh_rt_endpoint << "\n";
  o << "  chassis_link.codebook=" << cfg.link.codebook
    << " heartbeat_hz=" << cfg.link.heartbeat_hz
    << " axis_cmd_hz=" << cfg.link.axis_cmd_hz << "\n";
  // Every candidate, in probe order, with its two flags: the first enabled one
  // is what the link will actually use, and printing only that one would hide a
  // reordering mistake.
  for (std::size_t i = 0; i < cfg.link.endpoints.size(); ++i) {
    const EndpointCandidate& ep = cfg.link.endpoints[i];
    o << "  endpoint[" << i << "]=" << ep.proto << "://" << ep.host << ":"
      << ep.port << " tls=" << (ep.tls ? "true" : "false")
      << " enabled=" << (ep.enabled ? "true" : "false") << "\n";
  }
  o << "  tier1.cmd_timeout_ms=" << cfg.tier1.cmd_timeout_ms
    << " control_loop_hz=" << cfg.tier1.control_loop_hz << "\n";
  o << "  tier1.limits vx=" << cfg.tier1.limits.max_vx_mps
    << " vy=" << cfg.tier1.limits.max_vy_mps
    << " wz=" << cfg.tier1.limits.max_wz_radps
    << " holonomic=" << (cfg.tier1.limits.holonomic ? "true" : "false") << "\n";
  return o.str();
}

}  // namespace quadruped
