/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_relay_config.cc
 * Brief: The loader's refusals, the audit gate's diff, the session document
 *
 * Description:
 * Fixtures are written under argv[1] (the build/work dir), never next to the
 * source -- the same argv convention quadruped's config test uses, because
 * ctest and the mutant runner execute from different directories and a
 * relative fixture path finds nothing silently.
 *
 * Three groups:
 *   * loader: values load; a missing key, a null value and an unexpanded
 *     ${...} reference each refuse WITH THE KEY PATH in the message
 *     (CLAUDE.md 3.1 -- the operator gets a name, never a fallback).
 *   * audit gate: a registry matching the code table answers the empty
 *     string; one key missing, one key extra and a pub/sub swap each
 *     produce a non-empty diff naming the key. The gate is the CRL-3
 *     double insurance -- a comparison that cannot go red is decoration,
 *     so every red direction is exercised here.
 *   * session document: the five fields RT-C1/RT-C2/RT-C3.d freeze,
 *     asserted on the emitted BYTES (peer, the endpoint, empty listen,
 *     multicast off, gossip on + multihop off). The wrong values fail on
 *     the robot as silence, which is why they are pinned where no zenoh is
 *     needed (the 2026-08-23 measurement in the file header of
 *     relay_session_config.cc).
 */

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

#include "chassis_relay/relay_config.h"
#include "chassis_relay/relay_session.h"

using namespace chassis_relay;  // NOLINT: test-local

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

std::string g_dir;

std::string WriteFixture(const char* name, const std::string& body) {
  const std::string path = g_dir + "/" + name;
  std::ofstream f(path);
  f << body;
  return path;
}

// One loader failure case: expects a throw whose message contains `needle`
// (the dotted key path -- the whole point of the 3.1 contract).
void ExpectLoadThrow(const std::string& path, const char* needle) {
  try {
    (void)LoadRelayConfig(path);
    std::printf("FAIL no throw for fixture %s\n", path.c_str());
    ++g_failures;
  } catch (const std::exception& e) {
    if (std::strstr(e.what(), needle) == nullptr) {
      std::printf("FAIL message lacks '%s': %s\n", needle, e.what());
      ++g_failures;
    }
  }
}

const char* kGoodYaml =
    "chassis_relay:\n"
    "  robot_id: dev\n"
    "  zenoh_gen_endpoint: \"tcp/127.0.0.1:7447\"\n"
    "  zenoh_rt_endpoint: \"tcp/127.0.0.1:7449\"\n"
    "  whitelist_audit_path: \"/opt/xbrain_v6/configs/generated/whitelist.yaml\"\n";

void LoaderHappyPath() {
  const std::string p = WriteFixture("good.yaml", kGoodYaml);
  const RelayConfig cfg = LoadRelayConfig(p);
  CHECK(cfg.robot_id == "dev");
  CHECK(cfg.gen_endpoint == "tcp/127.0.0.1:7447");
  CHECK(cfg.rt_endpoint == "tcp/127.0.0.1:7449");
  CHECK(cfg.whitelist_audit_path ==
        "/opt/xbrain_v6/configs/generated/whitelist.yaml");
}

void LoaderRefusals() {
  // Missing key: the throw names it.
  ExpectLoadThrow(WriteFixture("missing.yaml",
                               "chassis_relay:\n  robot_id: dev\n"),
                  "chassis_relay.zenoh_gen_endpoint");
  // Null value: uncalibrated per 3.1, named as such by yaml_lite.
  std::string with_null = kGoodYaml;
  with_null.replace(with_null.find("dev"), 3, "null");
  ExpectLoadThrow(WriteFixture("null.yaml", with_null),
                  "chassis_relay.robot_id");
  // Unexpanded reference: the transitional file must carry literals; a
  // ${common.robot_id} reaching a direct reader would flow into every key
  // as a dollar string (relay_config.h).
  std::string with_ref = kGoodYaml;
  with_ref.replace(with_ref.find("dev"), 3, "\"${common.robot_id}\"");
  ExpectLoadThrow(WriteFixture("ref.yaml", with_ref), "unexpanded reference");
  // Absent file.
  ExpectLoadThrow(g_dir + "/does_not_exist.yaml", "cannot open");
}

// The registry as the generator writes it today, matching the code table.
std::string MatchingAudit() {
  return
      "processes:\n"
      "  chassis_relay:\n"
      "    pub:\n"
      "    - cmd/chassis/ctrl/ack\n"
      "    - cmd/estop/ack\n"
      "    - event/fault/chassis\n"
      "    - probe/estop/pong\n"
      "    - state/chassis_basic\n"
      "    - state/chassis_device\n"
      "    - state/chassis_motion\n"
      "    - state/power\n"
      "    - state/robot\n"
      "    sub:\n"
      "    - cmd/chassis/ctrl\n"
      "    - cmd/estop\n"
      "    - probe/estop/ping\n";
}

void AuditAgrees() {
  const std::string p = WriteFixture("wl_ok.yaml", MatchingAudit());
  const WhitelistAudit audit = LoadWhitelistAudit(p);
  CHECK(audit.pub.size() == 9);
  CHECK(audit.sub.size() == 3);
  CHECK(CompareWhitelistAudit(audit).empty());
}

void AuditDisagrees() {
  // One pub missing: the diff names it as missing.
  std::string missing = MatchingAudit();
  missing.erase(missing.find("    - state/robot\n"),
                std::strlen("    - state/robot\n"));
  {
    const WhitelistAudit a =
        LoadWhitelistAudit(WriteFixture("wl_missing.yaml", missing));
    const std::string diff = CompareWhitelistAudit(a);
    CHECK(!diff.empty());
    CHECK(diff.find("pub missing from audit: state/robot") !=
          std::string::npos);
  }
  // One extra sub: somebody widening the relay through the registry -- the
  // gate says so and the startup refuses.
  std::string extra = MatchingAudit();
  extra += "    - cmd/motion/behavior\n";
  {
    const WhitelistAudit a =
        LoadWhitelistAudit(WriteFixture("wl_extra.yaml", extra));
    const std::string diff = CompareWhitelistAudit(a);
    CHECK(!diff.empty());
    CHECK(diff.find("sub extra in audit: cmd/motion/behavior") !=
          std::string::npos);
  }
  // pub/sub swapped wholesale (the exact shape of the 2026-09-26 generator
  // defect): both directions light up.
  std::string swapped =
      "processes:\n  chassis_relay:\n    pub:\n    - cmd/estop\n"
      "    sub:\n    - state/robot\n";
  {
    const WhitelistAudit a =
        LoadWhitelistAudit(WriteFixture("wl_swap.yaml", swapped));
    const std::string diff = CompareWhitelistAudit(a);
    CHECK(diff.find("pub missing from audit: state/robot") !=
          std::string::npos);
    CHECK(diff.find("pub extra in audit: cmd/estop") != std::string::npos);
    CHECK(diff.find("sub missing from audit: cmd/estop") !=
          std::string::npos);
  }
  // Registry without the section: throws, never "gate skipped".
  try {
    (void)LoadWhitelistAudit(WriteFixture("wl_empty.yaml", "processes: {}\n"));
    std::printf("FAIL no throw for empty registry\n");
    ++g_failures;
  } catch (const std::exception&) {
  }
}

void SessionDocument() {
  const std::string doc = RelaySessionConfigJson("tcp/127.0.0.1:7449");
  // The five frozen fields, as bytes. Each one's absence or inversion is a
  // silent-merge or silent-deafness failure on the robot; the header of
  // relay_session_config.cc carries the measurements.
  CHECK(doc.find("mode:\"peer\"") != std::string::npos);
  CHECK(doc.find("connect:{endpoints:[\"tcp/127.0.0.1:7449\"]}") !=
        std::string::npos);
  CHECK(doc.find("listen:{endpoints:[]}") != std::string::npos);
  CHECK(doc.find("multicast:{enabled:false}") != std::string::npos);
  CHECK(doc.find("gossip:{enabled:true,multihop:false}") != std::string::npos);
  // Never router mode (RT-C3.d forbids it outright).
  CHECK(doc.find("router") == std::string::npos);
  // The endpoint parameter is not decoration: a second call with the other
  // plane's endpoint must differ exactly there.
  const std::string gen_doc = RelaySessionConfigJson("tcp/127.0.0.1:7447");
  CHECK(gen_doc.find("7447") != std::string::npos);
  CHECK(gen_doc.find("7449") == std::string::npos);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: test_relay_config <writable work dir>\n");
    return 2;
  }
  g_dir = argv[1];

  LoaderHappyPath();
  LoaderRefusals();
  AuditAgrees();
  AuditDisagrees();
  SessionDocument();

  if (g_failures != 0) {
    std::printf("test_relay_config: %d FAILURES\n", g_failures);
    return 1;
  }
  std::printf("test_relay_config: all checks passed\n");
  return 0;
}
