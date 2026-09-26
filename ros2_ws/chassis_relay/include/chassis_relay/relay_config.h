/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: relay_config.h
 * Brief: Startup config (rid + two endpoints) and the whitelist audit gate
 *
 * Description:
 * Everything here runs ONCE at startup, before any session opens; it may
 * allocate and it may throw. Nothing here is reachable from a forward path.
 *
 * What is configurable and what is not -- the CRL-3 boundary:
 *   * configurable: robot_id and the two router endpoints. Deployment
 *     parameters, same class as quadruped's zenoh_rt_endpoint.
 *   * NOT configurable: the key whitelist. CRL-3 hardcodes it in
 *     relay_keys.cc. The generated file configs/generated/whitelist.yaml is
 *     read here for ONE purpose: to compare against the code table and
 *     REFUSE TO START on any difference. The comparison can only ever
 *     narrow behaviour (exit 78), never widen it -- the same stance 11
 *     S2.4.7 takes for rt_override ("配置文件里的值仅供审计比对"). Without
 *     this gate the code table and the generated registry would drift apart
 *     silently, which is how the 2026-09-26 column-swap in the contract
 *     table survived until a generator run tripped over it.
 *
 * Config source -- the transitional arrangement, spelled out so it gets
 * removed rather than inherited: this loader reads
 * /opt/xbrain_v6/configs/chassis_relay.yaml DIRECTLY, while 10 S5.4.1 wants
 * every process to read the resolved snapshot under data/run/resolved/. The
 * freeze line does not yet emit a chassis_relay product (it currently stops
 * on unrelated null keys; CLAUDE.md iron rule 3 forbids filling those to get
 * it through), so the source file carries literal values and NO ${common.*}
 * references -- this reader does not expand references, and a literal-only
 * file cannot resolve differently per process, which is the failure S5.4.1
 * exists to prevent. When chassis_relay joins the freeze line: point the
 * default path at data/run/resolved/chassis_relay.yaml and change robot_id
 * in the source back to ${common.robot_id}. Both files carry this note.
 */

#ifndef HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_CONFIG_H_
#define HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_CONFIG_H_

#include <string>
#include <vector>

namespace chassis_relay {

// The effective startup values. All four are REQUIRED in the file: this
// process has no safety parameters, but the no-default rule is kept uniform
// anyway -- a missing endpoint that quietly became 127.0.0.1 would hide a
// deployment mistake behind a value that happens to be right on one machine.
struct RelayConfig {
  std::string robot_id;              // {rid} segment of every RT-plane key
  std::string gen_endpoint;          // general-plane router, tcp/host:port
  std::string rt_endpoint;           // RT-plane router, tcp/host:port
  std::string whitelist_audit_path;  // configs/generated/whitelist.yaml
};

// Load and validate the file at `path`. Throws std::runtime_error naming the
// dotted key on a missing key, a null value, or an unexpanded ${...}
// reference (a reference reaching a direct reader means someone re-added one
// to the transitional file -- refuse rather than forward with a literal
// "${common.robot_id}" in every key).
RelayConfig LoadRelayConfig(const std::string& path);

// The chassis_relay section of the generated whitelist, as two plain lists
// of general-plane keys (the generator writes them bare, matching the
// deployed key spelling).
struct WhitelistAudit {
  std::vector<std::string> pub;
  std::vector<std::string> sub;
};

// Parse processes.chassis_relay.{pub,sub} out of the generated file. Throws
// std::runtime_error when the file or the section is missing -- an absent
// audit registry is a deployment defect, not a license to skip the gate.
WhitelistAudit LoadWhitelistAudit(const std::string& path);

// Compare the audit lists against the hardcoded table (relay_keys.cc):
// pub must equal the general-plane keys of the RT->GEN rows, sub those of
// the GEN->RT rows, both as SETS (order-free, duplicates collapse). Returns
// an empty string when they agree; otherwise a multi-line report naming
// every key missing from or extra in the audit file, for the refusal log.
std::string CompareWhitelistAudit(const WhitelistAudit& audit);

}  // namespace chassis_relay

#endif  // HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_CONFIG_H_
