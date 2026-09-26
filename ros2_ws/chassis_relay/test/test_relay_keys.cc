/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_relay_keys.cc
 * Brief: The CR table against the contract itself, plus key composition
 *
 * Description:
 * Follows quadruped's test_rt_keys.cc: every key in the hardcoded table must
 * appear VERBATIM in docs/11 (argv[1]), because a mistyped key produces a
 * process that publishes happily and is heard by nobody -- there is nothing
 * to observe at run time. On top of that, the twelve (gen, rt) PAIRS are
 * pinned one by one: a table that swapped two mappings (state/robot fed from
 * rt/chassis/power, say) would pass any per-key existence check and deliver
 * every field into the wrong consumer, so the pairing itself is the
 * assertion that matters most here.
 *
 * The whitelist-shape assertions (three subscribes on the general plane,
 * nine publishes, no wildcard anywhere) are W-2 and RT-C3.b as code: the
 * startup audit gate compares against the GENERATED registry, but the
 * generated file is itself derived -- this test pins the table against the
 * numbers the contract freezes, so the two guards fail independently.
 */

#include <cstdio>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

#include "chassis_relay/relay_keys.h"

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

// One expected row: the pairing pinned verbatim, independent of the table's
// own spelling, so a swap or a retarget in relay_keys.cc goes red HERE.
struct ExpectedPair {
  const char* cr;
  Direction dir;
  const char* gen;
  const char* rt;
  const char* qos;
  bool exempt;
};

const ExpectedPair kExpected[] = {
    {"CR-1", Direction::kGenToRt, "cmd/estop", "rt/safety/estop", "Q0_safety",
     true},
    {"CR-2", Direction::kGenToRt, "probe/estop/ping", "rt/safety/probe/ping",
     "Q0_safety", false},
    {"CR-3", Direction::kRtToGen, "probe/estop/pong", "rt/safety/probe/pong",
     "Q0_safety", false},
    {"CR-4", Direction::kRtToGen, "state/robot", "rt/chassis/state",
     "Q2_state", false},
    {"CR-5", Direction::kRtToGen, "state/power", "rt/chassis/power",
     "Q2_state", false},
    {"CR-6", Direction::kRtToGen, "state/chassis_basic", "rt/chassis/basic",
     "Q2_state", false},
    {"CR-7", Direction::kRtToGen, "state/chassis_motion", "rt/chassis/motion",
     "Q2_state", false},
    {"CR-8", Direction::kRtToGen, "state/chassis_device", "rt/chassis/device",
     "Q2_state", false},
    {"CR-9", Direction::kRtToGen, "event/fault/chassis", "rt/chassis/fault",
     "Q3_cmd", false},
    {"CR-10", Direction::kRtToGen, "cmd/estop/ack", "rt/safety/estop/ack",
     "Q0_safety", false},
    {"CR-11", Direction::kGenToRt, "cmd/chassis/ctrl", "rt/chassis/ctrl",
     "Q0_safety", false},
    {"CR-12", Direction::kRtToGen, "cmd/chassis/ctrl/ack",
     "rt/chassis/ctrl/ack", "Q0_safety", false},
};

void CheckTableAgainstExpected() {
  CHECK(kRelayCount == sizeof(kExpected) / sizeof(kExpected[0]));
  for (const ExpectedPair& e : kExpected) {
    const RelaySpec* row = FindRelaySpec(e.cr);
    CHECK(row != nullptr);
    if (row == nullptr) continue;
    CHECK(row->direction == e.dir);
    CHECK(std::strcmp(row->gen_key, e.gen) == 0);
    CHECK(std::strcmp(row->rt_suffix, e.rt) == 0);
    CHECK(std::strcmp(row->qos_profile, e.qos) == 0);
    CHECK(row->estop_exempt == e.exempt);
  }
  // The exemption is a closed set of ONE: any second exempt row would open a
  // raw-forward path on a key whose misreadings do not collapse to "stop".
  std::size_t exempt = 0;
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    if (kRelayTable[i].estop_exempt) ++exempt;
  }
  CHECK(exempt == 1);
}

void CheckWhitelistShape() {
  // The frozen counts: 3 general-plane subscribes (GEN->RT rows), 9
  // publishes -- 11 S1.1.6 (3) "12 条 (v0.6 由 9 增 3)".
  std::size_t gen_to_rt = 0;
  std::set<std::string> gen_keys;
  std::set<std::string> rt_suffixes;
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    const RelaySpec& row = kRelayTable[i];
    if (row.direction == Direction::kGenToRt) ++gen_to_rt;
    gen_keys.insert(row.gen_key);
    rt_suffixes.insert(row.rt_suffix);
    // W-2: no wildcard subscription anywhere on a cross-plane process --
    // a single '*' would be the technical premise of a generic bridge.
    CHECK(std::strchr(row.gen_key, '*') == nullptr);
    CHECK(std::strchr(row.rt_suffix, '*') == nullptr);
    // Every RT suffix must live under rt/ (the plane prefix), every general
    // key must NOT -- a swapped pair of columns lands exactly here, which
    // is the 2026-09-26 contract-table defect turned into a local guard.
    CHECK(std::strncmp(row.rt_suffix, "rt/", 3) == 0);
    CHECK(std::strncmp(row.gen_key, "rt/", 3) != 0);
  }
  CHECK(gen_to_rt == 3);
  CHECK(kRelayCount - gen_to_rt == 9);
  // Uniqueness both sides: a duplicate key would declare two publishers for
  // one destination, which is an F-1 violation waiting for traffic.
  CHECK(gen_keys.size() == kRelayCount);
  CHECK(rt_suffixes.size() == kRelayCount);
}

void CheckAgainstContract(const std::string& contract_text) {
  // Every key must appear verbatim in docs/11. This is the assertion that
  // catches a typo: the key table of the contract spells both columns bare,
  // so a plain substring search is exact enough, and a missing file FAILS
  // (a comparison that cannot run is not a comparison).
  CHECK(!contract_text.empty());
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    const RelaySpec& row = kRelayTable[i];
    const bool gen_found =
        contract_text.find(row.gen_key) != std::string::npos;
    const bool rt_found =
        contract_text.find(row.rt_suffix) != std::string::npos;
    if (!gen_found) {
      std::printf("  gen key not in contract: %s\n", row.gen_key);
    }
    if (!rt_found) {
      std::printf("  rt suffix not in contract: %s\n", row.rt_suffix);
    }
    CHECK(gen_found);
    CHECK(rt_found);
  }
}

void CheckBuildKey() {
  // The composed spelling must match what quadruped composes for the same
  // rid/suffix, byte for byte -- the two processes meet on these keys.
  CHECK(BuildRtKey("dev", "rt/safety/estop") == "xbrain/dev/rt/safety/estop");
  CHECK(BuildRtKey("gj-001", "rt/chassis/state") ==
        "xbrain/gj-001/rt/chassis/state");
  // The bounded form refuses truncation rather than returning a prefix: a
  // truncated key is well formed and matches nothing.
  char small[16];
  CHECK(BuildRtKey("dev", "rt/safety/estop", small, sizeof(small)) == 0);
  char big[128];
  const std::size_t n = BuildRtKey("dev", "rt/safety/estop", big, sizeof(big));
  CHECK(n == std::strlen("xbrain/dev/rt/safety/estop"));
  CHECK(std::strcmp(big, "xbrain/dev/rt/safety/estop") == 0);
  // Null inputs answer zero, never crash.
  CHECK(BuildRtKey(nullptr, "x", big, sizeof(big)) == 0);
  CHECK(BuildRtKey("dev", nullptr, big, sizeof(big)) == 0);
  // Lookup misses answer null, never a nearby row (11 S13.6's ban on
  // substituting a neighbouring member, applied to table rows).
  CHECK(FindRelaySpec("CR-99") == nullptr);
  CHECK(FindRelaySpec(nullptr) == nullptr);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: test_relay_keys <docs/11 path>\n");
    return 2;
  }
  std::ifstream f(argv[1]);
  if (!f) {
    // Missing contract = failing test, not a skip: a comparison that cannot
    // run is not a comparison.
    std::printf("FAIL cannot open contract: %s\n", argv[1]);
    return 1;
  }
  std::stringstream buf;
  buf << f.rdbuf();

  CheckTableAgainstExpected();
  CheckWhitelistShape();
  CheckAgainstContract(buf.str());
  CheckBuildKey();

  if (g_failures != 0) {
    std::printf("test_relay_keys: %d FAILURES\n", g_failures);
    return 1;
  }
  std::printf("test_relay_keys: all checks passed\n");
  return 0;
}
