/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_tick_stats.cc
 * Brief: The T-ODOM-1 instrument -- percentile, max, overflow, degenerate input
 *
 * Description:
 * TickStats is what T-ODOM-1 is measured with, so a defect in it does not
 * produce a wrong number -- it produces a wrong VERDICT on 13 V-69, whose
 * acceptance criterion this is. The cases are therefore built around the two
 * numbers in that criterion (P99 <= 12 ms, max <= 20 ms) and around the ways a
 * histogram can quietly report a pass it did not earn.
 *
 * The case that carries the most weight is
 * test_a_tail_that_a_mean_would_hide: a loop holding 10 ms for 99% of its
 * ticks and stalling for the rest has an unremarkable average and fails
 * T-ODOM-1 outright. An instrument that tracked mean and max would report the
 * same summary for a healthy loop and that one.
 *
 * Every case states the mutation that turns it red. Two are worth naming here:
 * rounding the percentile DOWN to the bucket's lower edge (a marginal failing
 * run starts reporting a pass), and clamping a negative delta to zero instead
 * of dropping it (a caller subtracting in the wrong order gets an
 * improbably perfect result instead of a visible anomaly).
 */

#include "quadruped/tick_stats.h"

#include <cstdio>

using namespace quadruped;  // NOLINT: test-local

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

// Floating-point comparison at the bucket width. Exact equality would be
// asserting the compiler's division, not the class's behaviour.
bool Close(double a, double b) {
  const double d = a - b;
  return (d < 0 ? -d : d) < kTickBucketMs * 0.5;
}

}  // namespace

int main() {
  // ---- empty ------------------------------------------------------------
  {
    // A loop that never ticked has no period. Reporting 0.0 (a pass against
    // any threshold) would let a run that produced NO samples look healthy,
    // which is exactly how a publish loop that never started would be
    // mistaken for one that held its period perfectly.
    // mutant: return 12.0 or any constant -> this goes red.
    TickStats s;
    CHECK(s.count() == 0);
    CHECK(s.max_ms() == 0.0);
    CHECK(s.PercentileMs(0.99) == 0.0);
    CHECK(s.overflow() == 0);
  }

  // ---- a loop that holds its period exactly ------------------------------
  {
    // The healthy case, and the one that must stay quiet: 10 ms at 100 Hz.
    // mutant: make Add ignore its argument -> P99 goes to 0 -> red.
    TickStats s;
    for (int i = 0; i < 1000; ++i) s.Add(10.0);
    CHECK(s.count() == 1000);
    CHECK(Close(s.max_ms(), 10.0));
    // 10.0 exactly, not the bucket edge 10.1: the clamp caps the reported
    // percentile at the true maximum, which here IS 10.0.
    CHECK(Close(s.PercentileMs(0.99), 10.0));
    CHECK(s.overflow() == 0);
  }

  // ---- the tail a mean would hide ---------------------------------------
  {
    // 990 ticks at 10 ms, 10 ticks at 200 ms. The MEAN is 11.9 ms, which reads
    // as a healthy loop; P99 and max both fail T-ODOM-1. This is the whole
    // reason the criterion is a percentile.
    // mutant: report the mean instead of a percentile -> red.
    TickStats s;
    for (int i = 0; i < 980; ++i) s.Add(10.0);
    for (int i = 0; i < 20; ++i) s.Add(200.0);
    CHECK(Close(s.max_ms(), 200.0));
    CHECK(s.max_ms() > 20.0);                 // T-ODOM-1 max criterion: FAIL
    CHECK(s.PercentileMs(0.99) > 12.0);       // T-ODOM-1 P99 criterion: FAIL
    CHECK(s.overflow() == 20);
    // 2% slow, not 1%: a tail of EXACTLY 1% leaves P99 sitting on the fast
    // samples by definition, so it fails only the max criterion. Writing the
    // case with 1% (as a first draft did) asserts a P99 failure the definition
    // does not produce -- the instrument would have looked broken while being
    // right.
  }

  // ---- the percentile rounds AWAY from passing ---------------------------
  {
    // 98 ticks at 5 ms, one at 11.95 ms, one at 50 ms, so the 99th sample
    // (rank ceil(0.99*100) = 99) is the 11.95 ms one. It sits in the
    // [11.9, 12.0) bucket, so the reported value is the UPPER edge 12.0 --
    // at the criterion, not comfortably inside it.
    // mutant: return the bucket's LOWER edge -> 11.9 -> this goes red, and a
    // marginal run would start reporting a pass it did not earn.
    TickStats s;
    for (int i = 0; i < 98; ++i) s.Add(5.0);
    s.Add(11.95);
    // One sample ABOVE the percentile's bucket, so the clamp-to-max does not
    // bind and the rounding is observable. Without it the run's max would be
    // 11.95, the clamp would cap the answer there, and this case would be
    // testing the clamp instead of the rounding -- two different behaviours
    // that happen to agree on this input.
    s.Add(50.0);
    CHECK(Close(s.PercentileMs(0.99), 12.0));
    CHECK(s.PercentileMs(0.99) <= s.max_ms());
  }

  // ---- max stays exact, never quantised ---------------------------------
  {
    // T-ODOM-1 tests max DIRECTLY against 20 ms, so a bucketed max reports a
    // value that never occurred. 11.95 is chosen because quantising it moves
    // it: floor(11.95 / 0.1) * 0.1 = 11.9. A value that survives quantisation
    // (10.0, 50.0) would let the mutant through, which is why this case is
    // separate from the rounding one above -- that one needs a large max for
    // its own reason, and a large round max cannot catch this.
    // mutant: bucket the max in Add -> 11.9 -> red.
    TickStats s;
    s.Add(11.95);
    CHECK(s.max_ms() > 11.94 && s.max_ms() < 11.96);
  }

  // ---- percentile index rounds up ---------------------------------------
  {
    // 101 samples, so 0.99*n = 99.99 is NOT an integer and the two roundings
    // disagree: ceil gives rank 100 (the 8 ms sample), truncation gives 99
    // (still a 1 ms sample). A case built on n=100 would have p*n = 99.0
    // exactly, both roundings agree, and it would prove nothing -- which is
    // how the first draft of this case passed a broken implementation.
    // mutant: truncate instead of ceiling -> reports 1.1 ms -> red.
    TickStats s;
    for (int i = 0; i < 99; ++i) s.Add(1.0);
    s.Add(8.0);
    s.Add(25.0);
    CHECK(s.count() == 101);
    // 8.1 (the bucket edge), not 8.0: the clamp binds against the run's max
    // of 25 ms, which is far above, so the edge survives here.
    CHECK(Close(s.PercentileMs(0.99), 8.0 + kTickBucketMs));
  }

  // ---- overflow is visible, and the percentile says so -------------------
  {
    // Everything past the histogram ceiling. PercentileMs cannot name a
    // bucket edge, so it falls back to the exact max -- and overflow() is how
    // a reader knows that happened rather than having to infer it.
    // mutant: return the last bucket's edge (30.0) instead of max -> red, and
    // a 5-second stall would be reported as 30 ms.
    TickStats s;
    for (int i = 0; i < 10; ++i) s.Add(5000.0);
    CHECK(s.overflow() == 10);
    CHECK(Close(s.PercentileMs(0.99), 5000.0));
    CHECK(Close(s.max_ms(), 5000.0));
  }

  // ---- a negative delta is dropped, not clamped --------------------------
  {
    // steady_clock cannot go backwards, so a negative delta means the caller
    // subtracted in the wrong order. Counting it as a perfect 0 ms tick hides
    // that bug behind an improbably good result.
    // mutant: clamp to 0.0 and count it -> count becomes 3 -> red.
    TickStats s;
    s.Add(10.0);
    s.Add(-1.0);
    s.Add(10.0);
    CHECK(s.count() == 2);
    CHECK(Close(s.max_ms(), 10.0));
  }

  // ---- zero is a real sample -------------------------------------------
  {
    // Two ticks in the same clock granule is 0 ms and IS data -- it means the
    // loop ran twice without sleeping, which is a period defect worth seeing.
    // mutant: drop non-positive instead of negative -> count becomes 0 -> red.
    TickStats s;
    s.Add(0.0);
    CHECK(s.count() == 1);
  }

  // ---- a percentile can never exceed the maximum -------------------------
  {
    // Found on the first live bench run: 3000 ticks whose true max was
    // 10.06 ms reported "p99=10.10 max=10.06" in one line. The percentile was
    // naming its bucket's upper edge, which sits above every sample that fell
    // in it. Arithmetically a percentile cannot exceed the maximum, and an
    // instrument that prints an impossible pair is the first thing a reader
    // stops trusting.
    // mutant: drop the clamp -> p99 comes back 10.1 while max is 10.06 -> red.
    TickStats s;
    for (int i = 0; i < 3000; ++i) s.Add(10.06);
    CHECK(s.PercentileMs(0.99) <= s.max_ms());
    CHECK(Close(s.PercentileMs(0.99), 10.06));
    // The same must hold at every p, not just at 0.99.
    CHECK(s.PercentileMs(0.5) <= s.max_ms());
    CHECK(s.PercentileMs(1.0) <= s.max_ms());
    CHECK(s.PercentileMs(0.0) <= s.max_ms());
  }

  // ---- the T-ODOM-1 verdict shape --------------------------------------
  {
    // A run that PASSES, asserted against the criterion's own two numbers so
    // the pass side is pinned as well as the fail side. Without this, an
    // implementation that always reported a failure would satisfy every case
    // above.
    TickStats s;
    for (int i = 0; i < 60000; ++i) s.Add(10.0);
    for (int i = 0; i < 100; ++i) s.Add(11.5);   // under 1% of samples
    CHECK(s.PercentileMs(0.99) <= 12.0);
    CHECK(s.max_ms() <= 20.0);
  }

  if (g_failures == 0) {
    std::printf("ALL TICK STATS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d TICK STATS TEST(S) FAILED\n", g_failures);
  return 1;
}
