/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_relay_core.cc
 * Brief: Which frame causes which put, and what the estop exemption covers
 *
 * Description:
 * No router, no zenoh, no threads: the core is driven with bytes and a
 * capturing sink, the seam relay_core.h keeps open on purpose (the same
 * shape as quadruped's rt_bridge test). The cases that carry the most
 * weight, in the order a reviewer should read them:
 *
 *   * "a malformed estop is still forwarded, verbatim, and counted as raw"
 *     -- 11 S3.0.1's exemption is THE safety property of this process; a
 *     relay that drops a truncated stop because it failed to re-envelope it
 *     has inverted the fail-safe direction.
 *   * "a malformed ctrl is dropped, never raw-forwarded" -- the same
 *     exemption NOT applying to the relaxing command is the other half of
 *     the property; raw-forwarding enable-shaped bytes would open the
 *     payload tunnel RT-C3.e closes.
 *   * "seq advances per key, not per process" -- the consumer's gap
 *     detection runs per key (11 S3.0); one shared counter would make
 *     every key's stream look full of holes.
 */

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "chassis_relay/envelope_rebuild.h"
#include "chassis_relay/relay_core.h"
#include "chassis_relay/relay_keys.h"
#include "nlohmann/json.hpp"

using namespace chassis_relay;  // NOLINT: test-local
using nlohmann::json;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

// The capturing sink: every accepted put, in order, with its table row.
struct Sent {
  std::size_t index;
  std::string body;
};

struct Harness {
  std::vector<Sent> sent;
  bool refuse = false;  // when true the sink reports transport refusal
  RelayCore core;

  Harness()
      : core([this](std::size_t i, const char* b, std::size_t n) {
          if (refuse) return false;
          sent.push_back({i, std::string(b, n)});
          return true;
        }) {}
};

// Table indices resolved by cr id so a table reorder cannot silently point
// the cases at the wrong rows.
std::size_t RowOf(const char* cr) {
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    if (std::strcmp(kRelayTable[i].cr_id, cr) == 0) return i;
  }
  std::printf("FAIL no such row: %s\n", cr);
  ++g_failures;
  return 0;
}

const char* kGoodState =
    "{\"v\":1,\"rid\":\"dev\",\"ts\":100.5,\"mono\":42.25,"
    "\"boot\":\"9f2c1a44\",\"seq\":5,\"src\":\"quadruped\","
    "\"ts_sync\":true,\"data\":{\"soc\":0.9}}";

void ForwardRebuilds() {
  Harness h;
  const std::size_t row = RowOf("CR-4");
  CHECK(h.core.OnSample(row, kGoodState, std::strlen(kGoodState), 200.0) ==
        ForwardOutcome::kForwarded);
  CHECK(h.sent.size() == 1);
  CHECK(h.sent[0].index == row);
  json j = json::parse(h.sent[0].body, nullptr, false);
  CHECK(!j.is_discarded());
  // The rebuild happened: src is the relay, the original is preserved, the
  // producer's seq is replaced by this key's counter starting at 1.
  CHECK(j["src"] == "chassis_relay");
  CHECK(j["orig_src"] == "quadruped");
  CHECK(j["seq"] == 1);
  CHECK(h.sent[0].body.find("\"ts\":200.000000") != std::string::npos);
  // data byte-verbatim.
  CHECK(h.sent[0].body.find("{\"soc\":0.9}") != std::string::npos);
  // Counters moved with the forward.
  CHECK(h.core.stats(row).rx.load() == 1);
  CHECK(h.core.stats(row).forwarded.load() == 1);
  CHECK(h.core.stats(row).forwarded_raw.load() == 0);
}

void SeqIsPerKey() {
  Harness h;
  const std::size_t r4 = RowOf("CR-4");
  const std::size_t r5 = RowOf("CR-5");
  h.core.OnSample(r4, kGoodState, std::strlen(kGoodState), 1.0);
  h.core.OnSample(r4, kGoodState, std::strlen(kGoodState), 2.0);
  h.core.OnSample(r5, kGoodState, std::strlen(kGoodState), 3.0);
  CHECK(h.sent.size() == 3);
  // CR-4 counts 1, 2; CR-5 starts back at 1. A process-wide counter would
  // stamp 3 here and every consumer's gap detector would report holes.
  CHECK(json::parse(h.sent[0].body)["seq"] == 1);
  CHECK(json::parse(h.sent[1].body)["seq"] == 2);
  CHECK(json::parse(h.sent[2].body)["seq"] == 1);
}

void MalformedEstopStillForwards() {
  Harness h;
  const std::size_t estop = RowOf("CR-1");
  // Each shape separately: truncated JSON and non-JSON text cannot be
  // re-enveloped at all; 11 S3.0.1 says they go through anyway, VERBATIM.
  // (An object that scans but lacks data is NOT in this list any more: it
  // wraps -- see BareObjectWrapsEverywhere below.)
  const char* shapes[] = {
      "{\"v\":1,\"data\":{\"action\":\"stop\"",  // truncated
      "STOP",                                    // not JSON at all
  };
  std::size_t expected_raw = 0;
  for (const char* s : shapes) {
    CHECK(h.core.OnSample(estop, s, std::strlen(s), 10.0) ==
          ForwardOutcome::kForwardedRaw);
    ++expected_raw;
    CHECK(h.sent.size() == expected_raw);
    // Verbatim means verbatim: byte-identical, no rebuilt wrapper, exactly
    // one put (a doubled put would defeat the 50 ms dedup window).
    CHECK(h.sent.back().body == s);
  }
  CHECK(h.core.stats(estop).forwarded_raw.load() == expected_raw);
  CHECK(h.core.stats(estop).dropped_malformed.load() == 0);
}

void BareObjectWrapsEverywhere() {
  Harness h;
  // The live case this exists for: p5_gateway's probe ping is a BARE object
  // (no S3.0 envelope; measured on the deployed plane 2026-09-26). It must
  // FORWARD -- wrapped, not raw and not dropped -- on its non-exempt row,
  // or the relay black-holes the estop probe it is itself supervised by.
  const char* ping = "{\"seq\":42,\"t_mono_ms\":9,\"type\":\"ping\"}";
  const std::size_t row = RowOf("CR-2");
  CHECK(h.core.OnSample(row, ping, std::strlen(ping), 3.5) ==
        ForwardOutcome::kForwarded);
  CHECK(h.sent.size() == 1);
  json j = json::parse(h.sent[0].body, nullptr, false);
  CHECK(!j.is_discarded());
  CHECK(j["src"] == "chassis_relay");
  CHECK(j["data"]["type"] == "ping");
  CHECK(j["data"]["seq"] == 42);
  // The estop row wraps the same shape too (still a forward, not raw).
  CHECK(h.core.OnSample(RowOf("CR-1"), ping, std::strlen(ping), 3.6) ==
        ForwardOutcome::kForwarded);
  CHECK(h.core.stats(RowOf("CR-1")).forwarded_raw.load() == 0);
}

void WellFormedEstopIsRebuilt() {
  Harness h;
  const std::size_t estop = RowOf("CR-1");
  const char* good =
      "{\"v\":1,\"rid\":\"dev\",\"ts\":50.0,\"seq\":2,\"src\":\"p5_gateway\","
      "\"ts_sync\":true,\"data\":{\"action\":\"stop\",\"cmd_id\":\"c-1\"}}";
  CHECK(h.core.OnSample(estop, good, std::strlen(good), 51.0) ==
        ForwardOutcome::kForwarded);
  json j = json::parse(h.sent[0].body, nullptr, false);
  CHECK(!j.is_discarded());
  // The exemption is a FALLBACK, not the estop path: a well-formed stop is
  // re-enveloped like everything else (orig_src preserved for audit).
  CHECK(j["src"] == "chassis_relay");
  CHECK(j["orig_src"] == "p5_gateway");
  CHECK(j["data"]["action"] == "stop");
}

void MalformedNonExemptDrops() {
  Harness h;
  // The relaxing command is the row this gate protects hardest; the probe
  // ping and a state row are the same policy exercised on both directions.
  const char* garbage = "{\"data\":broken";
  for (const char* cr : {"CR-11", "CR-2", "CR-4"}) {
    const std::size_t row = RowOf(cr);
    CHECK(h.core.OnSample(row, garbage, std::strlen(garbage), 1.0) ==
          ForwardOutcome::kDroppedMalformed);
    CHECK(h.core.stats(row).dropped_malformed.load() == 1);
    CHECK(h.core.stats(row).forwarded_raw.load() == 0);
  }
  // NOTHING was put: no raw fallback outside CR-1.
  CHECK(h.sent.empty());
}

void OversizeDropsEverywhere() {
  Harness h;
  // One byte over the cap: dropped and counted on its own counter, estop
  // included -- the boundary relay_core.h argues (bounded refusal over
  // unbounded allocation on the no-alloc path).
  std::string big(kMaxForwardBytes + 1, 'x');
  for (const char* cr : {"CR-1", "CR-4"}) {
    const std::size_t row = RowOf(cr);
    CHECK(h.core.OnSample(row, big.data(), big.size(), 1.0) ==
          ForwardOutcome::kDroppedOversize);
    CHECK(h.core.stats(row).dropped_oversize.load() == 1);
  }
  CHECK(h.sent.empty());
  // Zero-length is the same refusal (there is nothing to forward).
  CHECK(h.core.OnSample(RowOf("CR-1"), "", 0, 1.0) ==
        ForwardOutcome::kDroppedOversize);
}

void PutRefusalIsCounted() {
  Harness h;
  h.refuse = true;
  const std::size_t row = RowOf("CR-4");
  CHECK(h.core.OnSample(row, kGoodState, std::strlen(kGoodState), 1.0) ==
        ForwardOutcome::kPutFailed);
  CHECK(h.core.stats(row).put_failed.load() == 1);
  CHECK(h.core.stats(row).forwarded.load() == 0);
  // The estop raw fallback reports refusal too rather than pretending.
  h.refuse = true;
  CHECK(h.core.OnSample(RowOf("CR-1"), "junk", 4, 1.0) ==
        ForwardOutcome::kPutFailed);
}

void BadIndexRefused() {
  Harness h;
  CHECK(h.core.OnSample(kRelayCount, kGoodState, std::strlen(kGoodState),
                        1.0) == ForwardOutcome::kBadIndex);
  CHECK(h.core.OnSample(0, nullptr, 4, 1.0) == ForwardOutcome::kBadIndex);
  CHECK(h.sent.empty());
  CHECK(h.core.total_rx() == 0);
}

}  // namespace

int main() {
  ForwardRebuilds();
  SeqIsPerKey();
  MalformedEstopStillForwards();
  WellFormedEstopIsRebuilt();
  BareObjectWrapsEverywhere();
  MalformedNonExemptDrops();
  OversizeDropsEverywhere();
  PutRefusalIsCounted();
  BadIndexRefused();

  if (g_failures != 0) {
    std::printf("test_relay_core: %d FAILURES\n", g_failures);
    return 1;
  }
  std::printf("test_relay_core: all checks passed\n");
  return 0;
}
