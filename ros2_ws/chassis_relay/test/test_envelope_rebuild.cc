/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_envelope_rebuild.cc
 * Brief: RT-C3.e rebuild semantics -- what changes, what stays byte-verbatim
 *
 * Description:
 * Every positive case PARSES the rebuilt text with the vendored json reader
 * rather than substring-matching the whole output: invalid JSON on
 * state/robot reads downstream as "the robot went silent", so
 * well-formedness is itself the property under test. The byte-verbatim
 * claims (data / mono / ts_sync spans copied untouched) are asserted as raw
 * substring presence IN ADDITION to parsed equality, because a rebuild that
 * decoded and re-encoded data could still parse equal while reordering keys
 * and reformatting floats -- the exact corruption CRL-5's no-reencode rule
 * exists to prevent.
 *
 * The negative half mirrors 11 S3.0.1's table: inputs that must fail the
 * scan (truncation, trailing garbage, non-object), and inputs that scan but
 * must fail the rebuild (no data). What the relay DOES with those failures
 * is per-key policy and lives in test_relay_core.cc; this file only pins
 * the primitive's answers.
 */

#include <cstdio>
#include <cstring>
#include <string>

#include "chassis_relay/envelope_rebuild.h"
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

// The canonical inbound frame: quadruped's state aggregate as the producer
// writes it (field order per 11 S3.0), with a data object deep enough to
// exercise the container skipper (nested object, array, string with an
// escaped quote and a brace).
const char* kFull =
    "{\"v\":1,\"rid\":\"dev\",\"ts\":1753660800.123456,"
    "\"mono\":4821.337215,\"boot\":\"9f2c1a44\",\"seq\":777,"
    "\"src\":\"quadruped\",\"ts_sync\":true,"
    "\"data\":{\"soc\":0.81,\"faults\":[{\"name\":\"f{1}\",\"detail\":"
    "\"a\\\"b\"}],\"hes_lock\":false}}";

std::string Rebuild(const char* in, double ts, std::uint64_t seq,
                    bool* ok = nullptr) {
  EnvelopeScan scan;
  const bool scanned = ScanEnvelope(in, std::strlen(in), &scan);
  if (ok != nullptr) *ok = scanned;
  if (!scanned) return std::string();
  char out[8192];
  const std::size_t n = RebuildEnvelope(in, scan, ts, seq, "chassis_relay",
                                        out, sizeof(out));
  return std::string(out, n);
}

void FullRoundTrip() {
  bool scanned = false;
  const std::string out = Rebuild(kFull, 1753660900.5, 42, &scanned);
  CHECK(scanned);
  CHECK(!out.empty());
  json j = json::parse(out, nullptr, false);
  CHECK(!j.is_discarded());
  // Copied verbatim: v, rid, mono, boot, ts_sync.
  CHECK(j["v"] == 1);
  CHECK(j["rid"] == "dev");
  CHECK(j["boot"] == "9f2c1a44");
  CHECK(j["ts_sync"] == true);
  // Rebuilt: ts is the forward instant with six decimals (S3.0's own
  // precision -- a %g here would coarsen every latency statistic).
  CHECK(out.find("\"ts\":1753660900.500000") != std::string::npos);
  // Rebuilt: the relay's own seq, not the producer's 777.
  CHECK(j["seq"] == 42);
  // Rebuilt: the forwarder's name (CRL-2 verbatim).
  CHECK(j["src"] == "chassis_relay");
  // The originals survive in orig_ts / orig_src (RT-C3.e).
  CHECK(j["orig_src"] == "quadruped");
  CHECK(out.find("\"orig_ts\":1753660800.123456") != std::string::npos);
  // mono copied as the same BYTES, not merely the same value.
  CHECK(out.find("\"mono\":4821.337215") != std::string::npos);
  // data: the exact byte span of the original, embedded once.
  const char* data_span =
      "{\"soc\":0.81,\"faults\":[{\"name\":\"f{1}\",\"detail\":\"a\\\"b\"}],"
      "\"hes_lock\":false}";
  CHECK(out.find(data_span) != std::string::npos);
  CHECK(j["data"]["faults"][0]["detail"] == "a\"b");
  // The producer's seq must appear nowhere: a rebuild that leaked it into
  // seq would splice two sequence spaces together.
  CHECK(j["seq"] != 777);
}

void CrossHostShape() {
  // A cloud publisher MUST omit mono/boot (CLK-C4); the rebuild forwards
  // that shape unchanged rather than inventing local values -- re-stamping
  // mono here would make relay-queue time invisible to every age check.
  const char* in =
      "{\"v\":1,\"rid\":\"dev\",\"ts\":1753660800.0,\"seq\":3,"
      "\"src\":\"cloud\",\"ts_sync\":false,\"data\":{\"action\":\"stop\"}}";
  const std::string out = Rebuild(in, 100.25, 1);
  CHECK(!out.empty());
  json j = json::parse(out, nullptr, false);
  CHECK(!j.is_discarded());
  CHECK(!j.contains("mono"));
  CHECK(!j.contains("boot"));
  CHECK(j["orig_src"] == "cloud");
  CHECK(j["src"] == "chassis_relay");
  CHECK(j["ts_sync"] == false);
}

void UnknownFieldsDropped() {
  // S3.0 receivers ignore unknown fields; a FORWARDER that copied them would
  // be a byte tunnel for whatever a sender smuggles at the top level.
  const char* in =
      "{\"v\":1,\"smuggled\":{\"cmd\":\"open\"},\"ts\":5.0,\"src\":\"x\","
      "\"data\":{}}";
  const std::string out = Rebuild(in, 6.0, 1);
  CHECK(!out.empty());
  CHECK(out.find("smuggled") == std::string::npos);
  json j = json::parse(out, nullptr, false);
  CHECK(!j.is_discarded());
  CHECK(j["data"].is_object());
}

void MissingOriginalsOmitted() {
  // No ts / no src in the input -> no orig_ts / no orig_src in the output
  // (writing null would fabricate a claim about the producer).
  const char* in = "{\"v\":1,\"data\":{\"k\":1}}";
  const std::string out = Rebuild(in, 7.5, 9);
  CHECK(!out.empty());
  json j = json::parse(out, nullptr, false);
  CHECK(!j.is_discarded());
  CHECK(!j.contains("orig_ts"));
  CHECK(!j.contains("orig_src"));
  CHECK(j["seq"] == 9);
  CHECK(j["src"] == "chassis_relay");
}

void DuplicateKeyLastWins() {
  // Documented tie-break: the LAST occurrence of a duplicate key is the one
  // forwarded, matching the json decoders used across this stack.
  const char* in = "{\"src\":\"first\",\"src\":\"second\",\"data\":{}}";
  const std::string out = Rebuild(in, 1.0, 1);
  CHECK(!out.empty());
  json j = json::parse(out, nullptr, false);
  CHECK(j["orig_src"] == "second");
}

void ScanRejections() {
  EnvelopeScan s;
  // Truncated: the object never closes.
  const char* cut = "{\"v\":1,\"data\":{\"a\":1}";
  CHECK(!ScanEnvelope(cut, std::strlen(cut), &s));
  // Trailing garbage after the close: not one object.
  const char* trail = "{\"v\":1,\"data\":{}} extra";
  CHECK(!ScanEnvelope(trail, std::strlen(trail), &s));
  // Not an object at all.
  const char* arr = "[1,2,3]";
  CHECK(!ScanEnvelope(arr, std::strlen(arr), &s));
  const char* text = "stop the robot";
  CHECK(!ScanEnvelope(text, std::strlen(text), &s));
  // Unterminated string inside.
  const char* badstr = "{\"src\":\"never closed}";
  CHECK(!ScanEnvelope(badstr, std::strlen(badstr), &s));
  // Empty and null inputs.
  CHECK(!ScanEnvelope("", 0, &s));
  CHECK(!ScanEnvelope(nullptr, 5, &s));
  // Whitespace around one object is fine.
  const char* padded = "  {\"data\":{}}\n";
  CHECK(ScanEnvelope(padded, std::strlen(padded), &s));
  CHECK(s.data.present);
}

void RebuildRejections() {
  // Scans, but carries no data: nothing to forward (S3.0 makes data
  // required), so the rebuild answers 0 and the caller's per-key policy
  // decides what that means.
  const char* no_data = "{\"v\":1,\"ts\":2.0,\"src\":\"y\"}";
  EnvelopeScan s;
  CHECK(ScanEnvelope(no_data, std::strlen(no_data), &s));
  char out[256];
  CHECK(RebuildEnvelope(no_data, s, 1.0, 1, "chassis_relay", out,
                        sizeof(out)) == 0);
  // A buffer too small answers 0, never a prefix: half an envelope is
  // valid-looking JSON that decodes to the wrong thing.
  CHECK(ScanEnvelope(kFull, std::strlen(kFull), &s));
  char tiny[64];
  CHECK(RebuildEnvelope(kFull, s, 1.0, 1, "chassis_relay", tiny,
                        sizeof(tiny)) == 0);
}

void DeepNesting() {
  // 63 nested arrays pass (the cap is 64 levels); 65 must fail the scan
  // rather than spin or misparse. Built programmatically so the two cases
  // cannot drift apart.
  for (int depth : {63, 65}) {
    std::string in = "{\"data\":";
    for (int i = 0; i < depth; ++i) in += "[";
    in += "1";
    for (int i = 0; i < depth; ++i) in += "]";
    in += "}";
    EnvelopeScan s;
    const bool ok = ScanEnvelope(in.c_str(), in.size(), &s);
    CHECK(ok == (depth == 63));
  }
}

}  // namespace

int main() {
  FullRoundTrip();
  CrossHostShape();
  UnknownFieldsDropped();
  MissingOriginalsOmitted();
  DuplicateKeyLastWins();
  ScanRejections();
  RebuildRejections();
  DeepNesting();

  if (g_failures != 0) {
    std::printf("test_envelope_rebuild: %d FAILURES\n", g_failures);
    return 1;
  }
  std::printf("test_envelope_rebuild: all checks passed\n");
  return 0;
}
