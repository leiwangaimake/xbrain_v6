/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_envelope_writer.cc
 * Brief: gtest cases for the CPP-CXX-2 envelope writer, one per named mutation
 *
 * Description:
 * Compiled and run by test_envelope_writer_cxx.py. Each case pins one of the
 * three CPP-CXX-2 mutations against common/include/xbrain/envelope/
 * envelope_writer.h:
 *   (1) before any ClockStatus, ts_sync is false -- a true fallback fails this;
 *   (2) staleness is measured on the monotonic reading -- a wall-clock step does
 *       NOT change ts_sync, only mono does, so a wall-clock impl fails these;
 *   (3) seq comes from one source and increases -- there is no second seq here to
 *       diverge (the "second writer" mutation is a source scan in the .py).
 */

#include <cstdint>

#include "gtest/gtest.h"
#include "xbrain/envelope/envelope_writer.h"

namespace {

using hachist::xbrain::envelope::EnvelopeWriter;
using hachist::xbrain::envelope::StampedEnvelope;

// CLK-A3's 5 s window, in SECONDS. Passed to the writer, never defaulted.
//
// This constant read 5000 and was named ...Ms until 2026-09-15, when the
// envelope's times were corrected to the float64 seconds 11 S3.0 states. The
// rename is not cosmetic: with the new signature, 5000 still COMPILES and every
// case below still passes -- they would just be asserting a window of five
// thousand seconds. A test that passes for the wrong reason is worse than one
// that fails, so the values are restated in seconds here rather than left to
// compile quietly.
constexpr double kSyncTimeoutS = 5.0;

EnvelopeWriter MakeWriter() {
  return EnvelopeWriter("r-1", "quadruped", "0a1b2c3d", kSyncTimeoutS);
}

// *** Mutation 1: ts_sync must be false before any ClockStatus arrives.
// A "default true until we hear otherwise" implementation fails here.
TEST(EnvelopeWriter, TsSyncIsFalseBeforeAnyClockStatus) {
  EnvelopeWriter w = MakeWriter();
  StampedEnvelope env = w.stamp(/*wall=*/1700000000.0, /*mono=*/1.0);
  EXPECT_FALSE(env.ts_sync);
}

// A fresh ClockStatus with sync=true makes ts_sync true...
TEST(EnvelopeWriter, TsSyncCopiesAFreshTrueStatus) {
  EnvelopeWriter w = MakeWriter();
  w.note_clock_status(/*sync=*/true, /*mono=*/1.0);
  EXPECT_TRUE(w.ts_sync_at(/*mono=*/1.0 + 4.999));   // within the 5 s window
}

// ...and a fresh ClockStatus with sync=false is copied as false (CLK-A2: copy,
// never optimistically upgrade).
TEST(EnvelopeWriter, TsSyncCopiesAFreshFalseStatus) {
  EnvelopeWriter w = MakeWriter();
  w.note_clock_status(/*sync=*/false, /*mono=*/1.0);
  EXPECT_FALSE(w.ts_sync_at(/*mono=*/1.0 + 0.010));
}

// *** Mutation 2: at/after the 5 s window ts_sync flips false, measured on mono.
TEST(EnvelopeWriter, TsSyncFlipsFalseAtTheMonotonicWindow) {
  EnvelopeWriter w = MakeWriter();
  w.note_clock_status(/*sync=*/true, /*mono=*/1.0);
  EXPECT_TRUE(w.ts_sync_at(/*mono=*/1.0 + 4.999));    // still fresh
  EXPECT_FALSE(w.ts_sync_at(/*mono=*/1.0 + 5.0));   // exactly 5 s -> stale
  EXPECT_FALSE(w.ts_sync_at(/*mono=*/1.0 + 9.0));   // well past -> stale
}

// *** Mutation 2, the wall-clock step. A huge jump in the wall reading does NOT
// change ts_sync: only the monotonic age governs it. An implementation that
// measured staleness on the wall field would flip here and fail.
TEST(EnvelopeWriter, WallClockStepDoesNotAffectTsSync) {
  EnvelopeWriter w = MakeWriter();
  w.note_clock_status(/*sync=*/true, /*mono=*/1.0);
  // mono says fresh (age 0.1 s); wall jumps BACKWARDS by an hour.
  StampedEnvelope env = w.stamp(/*wall=*/1700000000.0 - 3600.0, /*mono=*/1.1);
  EXPECT_TRUE(env.ts_sync);                           // still fresh by mono
  // mono says stale (age 6 s); wall jumps FORWARDS by an hour.
  StampedEnvelope env2 = w.stamp(/*wall=*/1700000000.0 + 3600.0, /*mono=*/7.0);
  EXPECT_FALSE(env2.ts_sync);                         // stale by mono, wall ignored
}

// *** Mutation 3 (half): seq comes from one source and strictly increases.
TEST(EnvelopeWriter, SeqStartsAtOneAndIncrementsMonotonically) {
  EnvelopeWriter w = MakeWriter();
  EXPECT_EQ(w.stamp(1.0, 1.0).seq, 1u);
  EXPECT_EQ(w.stamp(1.0, 2.0).seq, 2u);
  EXPECT_EQ(w.stamp(1.0, 3.0).seq, 3u);
}

// The wall and mono fields land where they belong -- ts is the wall reading,
// mono is the monotonic one -- so a downstream age computation reads mono.
TEST(EnvelopeWriter, TsHoldsWallAndMonoHoldsMonotonic) {
  EnvelopeWriter w = MakeWriter();
  // Sub-second digits on both, so a field that rounded or rescaled shows here.
  StampedEnvelope env = w.stamp(/*wall=*/1700000000.125, /*mono=*/42.5);
  EXPECT_DOUBLE_EQ(env.ts, 1700000000.125);
  EXPECT_DOUBLE_EQ(env.mono, 42.5);
  EXPECT_EQ(env.v, 1);
  EXPECT_EQ(env.rid, "r-1");
  EXPECT_EQ(env.src, "quadruped");
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
