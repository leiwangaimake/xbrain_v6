/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: relay_config.cc
 * Brief: yaml_lite-backed loader and the set comparison behind the audit gate
 *
 * Description:
 * Uses the shared header-only reader (common/include/xbrain/config/
 * yaml_lite.h) exactly as quadruped's config loader does: require_string
 * throws with the dotted key path on a missing key or a null, which is the
 * CLAUDE.md 3.1 contract -- the operator gets a key name, never a fallback.
 *
 * Two local rules on top of the reader:
 *   * an unexpanded "${...}" value is refused by name. yaml_lite does not
 *     expand references (by design: only the freeze line may), so a
 *     reference in the transitional direct-read file would otherwise flow
 *     into session endpoints and key names as a LITERAL dollar string --
 *     a session that connects nowhere and a key nobody subscribes, both
 *     silent. Refusing here turns that into a one-line startup error.
 *   * the audit comparison is over SETS, not sequences: the generator sorts
 *     its output today, but ordering is its private choice and a reorder
 *     must not read as a whitelist change.
 */

#include "chassis_relay/relay_config.h"

#include <algorithm>
#include <set>
#include <stdexcept>

#include "chassis_relay/relay_keys.h"
#include "xbrain/config/yaml_lite.h"

namespace chassis_relay {

namespace {

// Refuse a value the freeze line should have expanded. See the file header;
// checked on every loaded string because ANY of them flowing through as a
// literal "${...}" fails silently downstream.
std::string RequireExpanded(const std::string& value, const char* key) {
  if (value.size() >= 2 && value[0] == '$' && value[1] == '{') {
    throw std::runtime_error(
        std::string("config key ") + key +
        " holds an unexpanded reference: " + value +
        " -- direct-read files must carry literals (see relay_config.h)");
  }
  return value;
}

}  // namespace

RelayConfig LoadRelayConfig(const std::string& path) {
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("cannot open relay config: " + path);
  }
  std::stringstream buf;
  buf << f.rdbuf();
  const xbrain::config::YamlNode root =
      xbrain::config::ParseYaml(buf.str());

  // All keys live under one top-level map named after the process, the same
  // L6 shape every configs/*.yaml in this repository uses.
  RelayConfig cfg;
  cfg.robot_id = RequireExpanded(
      root.require_string("chassis_relay.robot_id"), "chassis_relay.robot_id");
  cfg.gen_endpoint =
      RequireExpanded(root.require_string("chassis_relay.zenoh_gen_endpoint"),
                      "chassis_relay.zenoh_gen_endpoint");
  cfg.rt_endpoint =
      RequireExpanded(root.require_string("chassis_relay.zenoh_rt_endpoint"),
                      "chassis_relay.zenoh_rt_endpoint");
  cfg.whitelist_audit_path = RequireExpanded(
      root.require_string("chassis_relay.whitelist_audit_path"),
      "chassis_relay.whitelist_audit_path");
  return cfg;
}

WhitelistAudit LoadWhitelistAudit(const std::string& path) {
  std::ifstream f(path);
  if (!f) {
    // Absent registry = deployment defect. The gate exists to catch drift;
    // skipping it when the file is missing would make deleting the file the
    // easiest way to disable the gate.
    throw std::runtime_error("cannot open whitelist audit file: " + path);
  }
  std::stringstream buf;
  buf << f.rdbuf();
  const xbrain::config::YamlNode root =
      xbrain::config::ParseYaml(buf.str());

  WhitelistAudit audit;
  const xbrain::config::YamlNode& pub =
      root.require_seq("processes.chassis_relay.pub");
  for (std::size_t i = 0; i < pub.size(); ++i) {
    audit.pub.push_back(pub.at_index(i).as_scalar("processes.chassis_relay.pub[i]"));
  }
  const xbrain::config::YamlNode& sub =
      root.require_seq("processes.chassis_relay.sub");
  for (std::size_t i = 0; i < sub.size(); ++i) {
    audit.sub.push_back(sub.at_index(i).as_scalar("processes.chassis_relay.sub[i]"));
  }
  return audit;
}

namespace {

// One direction of the comparison, reported symmetrically: a key the code
// table expects but the audit lacks ("missing"), and a key the audit lists
// but the code table does not carry ("extra"). Both directions matter --
// "missing" means the generator or the contract moved, "extra" means
// somebody tried to widen the relay through the registry, which CRL-3
// makes impossible by construction but should still be SAID at startup.
void DiffOneWay(const std::set<std::string>& expected,
                const std::set<std::string>& audited, const char* label,
                std::string* report) {
  for (const std::string& k : expected) {
    if (audited.find(k) == audited.end()) {
      *report += std::string("  ") + label + " missing from audit: " + k + "\n";
    }
  }
  for (const std::string& k : audited) {
    if (expected.find(k) == expected.end()) {
      *report += std::string("  ") + label + " extra in audit: " + k + "\n";
    }
  }
}

}  // namespace

std::string CompareWhitelistAudit(const WhitelistAudit& audit) {
  // The expectation comes from the ONE authoritative table (CRL-3). pub =
  // what this process publishes on the general plane = the RT->GEN rows;
  // sub = what it subscribes there = the GEN->RT rows.
  std::set<std::string> expect_pub;
  std::set<std::string> expect_sub;
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    if (kRelayTable[i].direction == Direction::kRtToGen) {
      expect_pub.insert(kRelayTable[i].gen_key);
    } else {
      expect_sub.insert(kRelayTable[i].gen_key);
    }
  }
  const std::set<std::string> audited_pub(audit.pub.begin(), audit.pub.end());
  const std::set<std::string> audited_sub(audit.sub.begin(), audit.sub.end());

  std::string report;
  DiffOneWay(expect_pub, audited_pub, "pub", &report);
  DiffOneWay(expect_sub, audited_sub, "sub", &report);
  return report;
}

}  // namespace chassis_relay
