/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rt_keys.cc
 * Brief: Every declared key must exist verbatim in the contract
 *
 * Description:
 * The strongest case in this file reads docs/11-接口契约.md and requires each
 * suffix in the table to appear in it, character for character. That is the
 * only kind of assertion that catches the failure this table exists to prevent:
 * a mistyped key produces a process that starts, publishes without error, and
 * is heard by nobody, because the subscriber's expression never matches. No
 * amount of unit testing the string builder finds it, and neither side logs
 * anything at all.
 *
 * The contract path is passed in as argv[1] rather than derived here. A test
 * that computes its own path relative to the source file works until the file
 * moves; more importantly, a MISSING contract must fail rather than silently
 * skip the comparison, and that is asserted below.
 *
 * The other cases are structural and cheap:
 *   * no key is declared twice, and none is both published and subscribed --
 *     11 F-1 allows exactly one publisher per key, so a key in both lists means
 *     this process is talking to itself;
 *   * the ten publish and eight subscribe roles are counted, so a row that
 *     silently changes role shows up;
 *   * every QoS name resolves in the shared profile table, so a typo there
 *     cannot leave a safety key running on the command profile;
 *   * the Q0 set is pinned to exactly the six keys of the three request/reply
 *     pairs 13 PB-Q1 puts on the rt_safety thread. Mixing Q3 traffic onto that
 *     thread is what destroys the emergency-stop budget, and which keys are Q0
 *     is the input to that decision.
 */

#include "quadruped/rt_keys.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

#include "xbrain/zenoh/qos_profiles.h"

using quadruped::rt::BuildKey;
using quadruped::rt::FindKey;
using quadruped::rt::KeyRole;
using quadruped::rt::KeySpec;
using quadruped::rt::kKeyCount;
using quadruped::rt::kKeys;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

std::string ReadFile(const std::string& path) {
  std::ifstream f(path);
  if (!f) return std::string();
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

}  // namespace

int main(int argc, char** argv) {
  const std::string contract_path =
      (argc >= 2) ? argv[1] : "docs/11-接口契约.md";

  // ---- every declared key exists verbatim in the contract ----------------
  {
    const std::string contract = ReadFile(contract_path);
    // A missing contract FAILS. Skipping would turn the one assertion that
    // matters into an empty green tick, which is the shape CLAUDE.md 3.2 warns
    // about: a criterion that cannot fail is not a criterion.
    if (contract.empty()) {
      std::printf("FAIL cannot read the contract: %s\n", contract_path.c_str());
      ++g_failures;
    } else {
      // Sanity on the file itself before trusting a search through it: an
      // empty-ish or wrong file would make every key "not found" and the
      // failure would be read as a table problem.
      CHECK(contract.find("2.2.1") != std::string::npos);
      for (std::size_t i = 0; i < kKeyCount; ++i) {
        if (contract.find(kKeys[i].suffix) == std::string::npos) {
          std::printf("FAIL key not in the contract: %s (%s)\n",
                      kKeys[i].suffix, kKeys[i].purpose);
          ++g_failures;
        }
      }
      // ...and the negative control. Without it, a search that always
      // succeeded -- say against a file of every three-letter combination --
      // would make the loop above meaningless.
      CHECK(contract.find("rt/chassis/nonexistent_key") == std::string::npos);
    }
  }

  // ---- no duplicates, and no key both published and subscribed ----------
  {
    std::set<std::string> seen;
    std::set<std::string> published;
    std::set<std::string> subscribed;
    for (std::size_t i = 0; i < kKeyCount; ++i) {
      const std::string s(kKeys[i].suffix);
      if (!seen.insert(s).second) {
        std::printf("FAIL key declared twice: %s\n", kKeys[i].suffix);
        ++g_failures;
      }
      if (kKeys[i].role == KeyRole::kPublish) {
        published.insert(s);
      } else {
        subscribed.insert(s);
      }
    }
    // 11 F-1: one publisher per key. A key in both lists means this process
    // subscribes to itself, which is either a copy-paste or a design error.
    for (const std::string& s : published) {
      CHECK(subscribed.find(s) == subscribed.end());
    }
    // The counts, so a row that silently flips role is visible. Written as two
    // numbers rather than one total: a swap keeps the total unchanged.
    CHECK(published.size() == 10);
    CHECK(subscribed.size() == 8);
    CHECK(kKeyCount == 18);
  }

  // ---- every QoS name resolves in the shared table ----------------------
  {
    // A typo here does not fail to compile and does not fail to publish -- it
    // silently selects nothing, and a safety key ends up on whatever the
    // caller's fallback is.
    for (std::size_t i = 0; i < kKeyCount; ++i) {
      const auto* p = hachist::xbrain::qos::FindProfile(kKeys[i].qos);
      if (p == nullptr) {
        std::printf("FAIL unknown QoS profile %s on key %s\n", kKeys[i].qos,
                    kKeys[i].suffix);
        ++g_failures;
      }
    }
    CHECK(hachist::xbrain::qos::FindProfile("Q9_nonexistent") == nullptr);
  }

  // ---- the Q0 set is exactly the six keys of 13 PB-Q1's three pairs -----
  {
    // PB-Q1 puts these on their own thread and forbids any Q3 traffic there,
    // for the reason 11 CRL-6 gives: the real_time + express budget is
    // emergency-stop specific and mixing destroys it. Which keys are Q0 is the
    // input to that decision, so it is pinned rather than left to the table.
    std::set<std::string> q0;
    for (std::size_t i = 0; i < kKeyCount; ++i) {
      if (std::strcmp(kKeys[i].qos, "Q0_safety") == 0) q0.insert(kKeys[i].suffix);
    }
    const std::set<std::string> expect = {
        "rt/chassis/ctrl", "rt/chassis/ctrl/ack",
        "rt/safety/estop", "rt/safety/estop/ack",
        "rt/safety/probe/ping", "rt/safety/probe/pong"};
    CHECK(q0 == expect);
    // cmd_vel is NOT Q0: it is 20 Hz best-effort state, and putting it on the
    // safety profile would put ordinary traffic on the express budget.
    const KeySpec* cv = FindKey("rt/motion/cmd_vel");
    CHECK(cv != nullptr);
    if (cv != nullptr) CHECK(std::strcmp(cv->qos, "Q0_safety") != 0);
  }

  // ---- key composition ---------------------------------------------------
  {
    char buf[128];
    const std::size_t n = BuildKey("gj-001", "rt/chassis/state", buf, sizeof(buf));
    CHECK(n > 0);
    CHECK(std::string(buf, n) == "xbrain/gj-001/rt/chassis/state");
    // The first segment is fixed and the second is the rid -- both stated by
    // 11 S2.1 and by the Qt-facing spec, which additionally requires the rid to
    // match the second segment of the key character for character.
    CHECK(std::string(buf, n).compare(0, 7, "xbrain/") == 0);

    // A buffer one byte short returns 0 rather than a truncated key. This is
    // the case that matters: "xbrain/gj-001/rt/chassis/stat" is a perfectly
    // well-formed key that nothing subscribes to.
    CHECK(BuildKey("gj-001", "rt/chassis/state", buf, n) == 0);
    CHECK(BuildKey("gj-001", "rt/chassis/state", buf, 0) == 0);
    CHECK(BuildKey(nullptr, "rt/chassis/state", buf, sizeof(buf)) == 0);
    CHECK(BuildKey("gj-001", nullptr, buf, sizeof(buf)) == 0);
    CHECK(BuildKey("gj-001", "rt/chassis/state", nullptr, 16) == 0);

    // The string form agrees with the buffer form. Two builders that disagree
    // is how setup code and the hot path end up publishing on different keys.
    CHECK(BuildKey(std::string("gj-001"), std::string("rt/chassis/state")) ==
          "xbrain/gj-001/rt/chassis/state");
    for (std::size_t i = 0; i < kKeyCount; ++i) {
      char b[256];
      const std::size_t m = BuildKey("gj-001", kKeys[i].suffix, b, sizeof(b));
      CHECK(m > 0);
      CHECK(BuildKey(std::string("gj-001"), std::string(kKeys[i].suffix)) ==
            std::string(b, m));
    }
  }

  // ---- lookup ------------------------------------------------------------
  {
    const KeySpec* k = FindKey("rt/safety/estop/ack");
    CHECK(k != nullptr);
    if (k != nullptr) {
      CHECK(k->role == KeyRole::kPublish);
      CHECK(std::strcmp(k->qos, "Q0_safety") == 0);
    }
    // An undeclared suffix returns nullptr rather than something usable:
    // publishing on a key that is not in the table is how a key escapes the
    // comparison against the contract.
    CHECK(FindKey("rt/chassis/made_up") == nullptr);
    CHECK(FindKey(nullptr) == nullptr);
    // Prefix and suffix near-misses must NOT match: a lookup that accepted
    // them would let "rt/chassis/state" resolve to "rt/chassis/state/extra".
    CHECK(FindKey("rt/chassis/stat") == nullptr);
    CHECK(FindKey("rt/chassis/state/extra") == nullptr);
    // Every row is findable by its own suffix, so the table and the lookup
    // cannot disagree about what is declared.
    for (std::size_t i = 0; i < kKeyCount; ++i) {
      CHECK(FindKey(kKeys[i].suffix) == &kKeys[i]);
    }
  }

  // ---- every row carries a reason ----------------------------------------
  {
    // A row without a purpose is a row nobody can check against 11 S2.2.1, and
    // this table's whole value is that it can be checked.
    for (std::size_t i = 0; i < kKeyCount; ++i) {
      CHECK(kKeys[i].purpose != nullptr && std::strlen(kKeys[i].purpose) > 20);
      CHECK(kKeys[i].suffix != nullptr && std::strlen(kKeys[i].suffix) > 0);
      // Keys are relative: a row that already carried "xbrain/" would compose
      // to "xbrain/gj-001/xbrain/...".
      CHECK(std::string(kKeys[i].suffix).find("xbrain/") == std::string::npos);
      CHECK(kKeys[i].suffix[0] != '/');
    }
  }

  if (g_failures == 0) {
    std::printf("ALL RT_KEYS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RT_KEYS TEST(S) FAILED\n", g_failures);
  return 1;
}
