/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: envelope_units_probe.cc
 * Brief: The envelope's units and rendered text, as plain assertions
 *
 * Description:
 * test_envelope_units_cross_language.py is the stronger check: it hands this
 * header's output to the real Python decoder, so the two languages are compared
 * rather than described. This file exists alongside it for one reason -- it is a
 * plain program with its own main, so scripts/ci/cxx_mutants.py can mutate
 * envelope_writer.h against it and prove the assertions can turn red.
 *
 * The defect these guard: ts and mono were int64 MILLISECONDS here until
 * 2026-09-15, while 11 S3.0 and the Qt-facing spec both state float64 SECONDS,
 * and while the Python side had already been corrected. No test noticed,
 * because the existing C++ case asserted only ts_sync semantics -- the unit was
 * never an assertion, only an assumption.
 *
 * Note what is NOT asserted here: that the wall clock and the monotonic clock
 * are the ones the process actually reads. Those are the caller's, and injecting
 * them is what makes a five-second window testable in microseconds.
 */

#include "xbrain/envelope/envelope_writer.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using hachist::xbrain::envelope::EnvelopeWriter;
using hachist::xbrain::envelope::StampedEnvelope;
using hachist::xbrain::envelope::WriteEnvelopeJson;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

int main() {
  // Values chosen so a millisecond implementation cannot produce the same
  // digits: the wall time carries six decimals an integer cannot hold, and the
  // monotonic reading is small enough that a stray *1000 is obvious.
  const double kWall = 1789455340.123456;
  const double kMono = 12.5;

  {
    EnvelopeWriter w("gj-001", "quadruped", "9f2c1a44", /*sync_timeout_s=*/5.0);
    w.note_clock_status(true, kMono);
    const StampedEnvelope e = w.stamp(kWall, kMono);

    // *** The unit. A millisecond stamp reads 1789455340123.456 here.
    CHECK(std::fabs(e.ts - kWall) < 1e-6);
    CHECK(std::fabs(e.mono - kMono) < 1e-9);
    // Sub-second precision survives, which an integer field cannot carry.
    CHECK(e.ts != std::floor(e.ts));
    CHECK(e.v == 1);
    CHECK(e.seq == 1);
    CHECK(e.ts_sync == true);
  }

  {
    // The CLK-A3 window is in seconds too. A class that took milliseconds here
    // and stamped seconds elsewhere is the mixed-unit trap itself: 5.0 would
    // mean five milliseconds and every envelope would report ts_sync false.
    EnvelopeWriter w("gj-001", "quadruped", "9f2c1a44", /*sync_timeout_s=*/5.0);
    w.note_clock_status(true, 100.0);
    CHECK(w.ts_sync_at(104.999) == true);   // inside the five seconds
    CHECK(w.ts_sync_at(105.0) == false);    // CLK-A3: >= the window is stale
    CHECK(w.ts_sync_at(105.001) == false);
  }

  {
    // The rendered text: a JSON NUMBER with six decimals, not an integer and
    // not a quoted string. The Qt-facing spec freezes ts as a JSON number and
    // forbids millisecond integers by name.
    EnvelopeWriter w("gj-001", "quadruped", "9f2c1a44", 5.0);
    w.note_clock_status(true, kMono);
    const StampedEnvelope e = w.stamp(kWall, kMono);
    char buf[512];
    const std::size_t n = WriteEnvelopeJson(e, "{\"probe\":1}", buf, sizeof(buf));
    CHECK(n > 0);
    const std::string s(buf, n);
    CHECK(s.find("\"ts\":1789455340.123456") != std::string::npos);
    CHECK(s.find("\"mono\":12.500000") != std::string::npos);
    // Quoted numbers would still parse through a lenient reader, so the absence
    // of the quote is asserted rather than assumed.
    CHECK(s.find("\"ts\":\"") == std::string::npos);
    CHECK(s.find("\"rid\":\"gj-001\"") != std::string::npos);
    CHECK(s.find("\"ts_sync\":true") != std::string::npos);
    CHECK(s.find("\"data\":{\"probe\":1}") != std::string::npos);
    // The object is closed. Truncated JSON is a parse error at the far end, and
    // the length check below is what keeps a partial one off the wire.
    CHECK(!s.empty() && s[s.size() - 1] == '}');
  }

  {
    // A buffer one byte short returns 0, not a partial object.
    EnvelopeWriter w("gj-001", "quadruped", "9f2c1a44", 5.0);
    const StampedEnvelope e = w.stamp(kWall, kMono);
    char big[512];
    const std::size_t full = WriteEnvelopeJson(e, "{\"probe\":1}", big, sizeof(big));
    CHECK(full > 0);
    char small[512];
    CHECK(WriteEnvelopeJson(e, "{\"probe\":1}", small, full) == 0);
    // ...and the degenerate arguments, which a caller can reach by passing a
    // buffer it has not sized yet.
    CHECK(WriteEnvelopeJson(e, "{\"probe\":1}", small, 0) == 0);
    CHECK(WriteEnvelopeJson(e, nullptr, small, sizeof(small)) == 0);
    CHECK(WriteEnvelopeJson(e, "{\"probe\":1}", nullptr, 16) == 0);
  }

  {
    // seq is per-writer and strictly increasing, and it is the ONE source --
    // PB-Q3. Two envelopes from one writer must never share a number.
    EnvelopeWriter w("gj-001", "quadruped", "9f2c1a44", 5.0);
    CHECK(w.stamp(kWall, kMono).seq == 1);
    CHECK(w.stamp(kWall, kMono).seq == 2);
    CHECK(w.stamp(kWall, kMono).seq == 3);
  }

  {
    // ts_sync defaults to FALSE before any ClockStatus. An optimistic default
    // would report a robot as time-synced during the seconds before rtk_driver
    // has said anything at all.
    EnvelopeWriter w("gj-001", "quadruped", "9f2c1a44", 5.0);
    CHECK(w.stamp(kWall, kMono).ts_sync == false);
  }

  if (g_failures == 0) {
    std::printf("ALL ENVELOPE_UNITS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d ENVELOPE_UNITS TEST(S) FAILED\n", g_failures);
  return 1;
}
