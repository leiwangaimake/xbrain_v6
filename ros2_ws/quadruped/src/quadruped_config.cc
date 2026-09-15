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

#include <sstream>

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

    // ---- channel two: chassis DDS domain 0 ---------------------------
    cfg.dds.backend = root.require_string(K("chassis_dds.backend"));
    cfg.dds.domain_id =
        static_cast<int>(root.require_int(K("chassis_dds.domain_id")));
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
