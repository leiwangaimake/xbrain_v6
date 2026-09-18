/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: tick_stats.h
 * Brief: Fixed-bucket period histogram for T-ODOM-1 (13 S11.1)
 *
 * Description:
 * T-ODOM-1 asks a question nothing in this process could answer: does the
 * 100 Hz publish loop actually hold its period? Its criterion is "P99 <= 12 ms,
 * max <= 20 ms over 10 minutes, under load", and before this header the loop
 * kept no timing at all -- the only evidence available was "it did not crash",
 * which every implementation provides.
 *
 * The item matters because V-69 moved odom/TF publishing off the SCHED_FIFO
 * ctrl thread onto rt_pub, which runs at ordinary priority. 13 S12.1 records
 * that trade verbatim and makes T-ODOM-1 its acceptance criterion: "台架未跑
 * 之前不得宣布收口". So this is the instrument that decision is measured with.
 *
 * Why a fixed-bucket histogram and not a sample buffer:
 *   * ZERO allocation and O(1) per tick. rt_pub is not SCHED_FIFO, but it is
 *     still a periodic loop and the measurement must not be the thing that
 *     perturbs the period -- an instrument that changes what it measures gives
 *     a number nobody can act on.
 *   * 10 minutes at 100 Hz is 60000 samples. Keeping them all is 480 kB that
 *     grows without bound if the run goes longer; the histogram is a fixed
 *     2.5 kB whatever the duration.
 *   * A running mean plus a max would NOT do. The criterion is a PERCENTILE,
 *     and a mean cannot produce one: a loop that holds 10 ms for 99% of ticks
 *     and stalls 200 ms for the rest has an unremarkable mean and fails
 *     T-ODOM-1 outright.
 *
 * max_ms is kept EXACTLY, outside the buckets. The criterion tests max against
 * 20 ms, and reading a max off a bucket would report the bucket's edge -- which
 * is either optimistic (lower edge) or a value that never occurred (upper).
 * A percentile may be quantised; a maximum may not.
 *
 * Percentile() returns the bucket's UPPER edge, i.e. it rounds the answer
 * AWAY from passing. A quantised percentile has to err in one direction, and
 * the direction that makes a marginal run look like a pass is the one that
 * gets a bad build shipped.
 *
 * What this does NOT do: it does not read a clock. The caller passes deltas
 * in. A stats class that timed itself would fix the clock source (CLAUDE.md
 * 3.4 requires steady_clock here) in a place where a test cannot drive it, and
 * every case below would then need real elapsed time to run.
 */

#ifndef HACHIST_XBRAIN_V6_QUADRUPED_TICK_STATS_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_TICK_STATS_H_

#include <cstddef>
#include <cstdint>

namespace quadruped {

// Bucket width in milliseconds. 0.1 ms against a 12 ms / 20 ms criterion means
// the quantisation error is under 1% of either threshold -- small enough that
// it cannot move a verdict that was not already on the line.
constexpr double kTickBucketMs = 0.1;
// Buckets span [0, 30) ms. The criterion's ceiling is 20 ms, and the extra
// 10 ms of resolution is there so a run that FAILS still shows its shape
// instead of collapsing into the overflow bin.
constexpr std::size_t kTickBuckets = 300;

// Period histogram for one periodic loop. Not thread-safe: it is written by
// the loop that owns it and read after that loop has stopped. Adding a mutex
// would put a lock in the period being measured.
class TickStats {
 public:
  TickStats() : over_(0), count_(0), max_ms_(0.0) {
    for (std::size_t i = 0; i < kTickBuckets; ++i) hist_[i] = 0;
  }

  // Record one inter-tick interval.
  void Add(double dt_ms) {
    // Negative deltas are dropped rather than clamped to zero. steady_clock
    // cannot go backwards, so a negative value means the caller subtracted in
    // the wrong order -- counting it as a perfect 0 ms tick would hide that bug
    // behind an improbably good result, which is the worst way to lose it.
    if (dt_ms < 0.0) return;
    ++count_;
    if (dt_ms > max_ms_) max_ms_ = dt_ms;
    const std::size_t idx = static_cast<std::size_t>(dt_ms / kTickBucketMs);
    if (idx >= kTickBuckets) {
      ++over_;
      return;
    }
    ++hist_[idx];
  }

  // The p-th percentile in ms, p in [0, 1]. Returns the containing bucket's
  // UPPER edge; see the header note on why it rounds away from passing.
  // An empty histogram returns 0.0 -- a loop that never ticked has no period,
  // and inventing one would let a run that produced no samples report a pass.
  double PercentileMs(double p) const {
    if (count_ == 0) return 0.0;
    // A real ceiling. The rank wanted is ceil(p * n): for p=0.99 and n=101 that
    // is the 100th sample, and truncation gives the 99th -- one sample earlier,
    // which on a distribution whose tail is exactly 1% of samples is the
    // difference between seeing the tail and not seeing it.
    //
    // An earlier draft wrote `want = (uint64)(p*n); if (want < n) ++want;`,
    // which is not a ceiling: for p=0.99 and n=100 the exact rank is 99.0 and
    // that form returned 100, reporting the SLOWEST sample as the 99th
    // percentile. It was caught by test_tick_stats' "index rounds up" case on
    // the first run; the arithmetic is written out here because the wrong form
    // looks like a ceiling and passes a case where p*n is not an integer.
    const double exact = p * static_cast<double>(count_);
    std::uint64_t want = static_cast<std::uint64_t>(exact);
    if (static_cast<double>(want) < exact) ++want;
    // p == 0 asks for the smallest sample, not for "no sample".
    if (want == 0) want = 1;
    if (want > count_) want = count_;
    std::uint64_t seen = 0;
    for (std::size_t i = 0; i < kTickBuckets; ++i) {
      seen += hist_[i];
      if (seen < want) continue;
      // The bucket's upper edge, but never above the exact maximum. Without
      // the clamp a run of identical 10.06 ms ticks reports p99=10.10 and
      // max=10.06 in the same line -- arithmetically impossible (a percentile
      // cannot exceed the maximum) and the first thing a reader would
      // disbelieve about the instrument. Observed on the first live run.
      //
      // The clamp does NOT weaken the "round away from passing" rule: max is a
      // value that actually occurred, so max >= the true percentile always,
      // and the reported figure stays on or above the real one.
      const double edge = static_cast<double>(i + 1) * kTickBucketMs;
      return edge < max_ms_ ? edge : max_ms_;
    }
    // Everything left is in the overflow bin, whose upper edge is unknown --
    // report the exact max instead. That is the only honest answer available
    // and it is on the conservative side, since max >= any percentile.
    return max_ms_;
  }

  double max_ms() const { return max_ms_; }
  std::uint64_t count() const { return count_; }
  // Ticks at or beyond the histogram's ceiling. Exposed separately because a
  // non-zero overflow makes PercentileMs fall back to max_ms, and a reader has
  // to be able to tell that happened rather than infer it.
  std::uint64_t overflow() const { return over_; }

 private:
  std::uint64_t hist_[kTickBuckets];
  std::uint64_t over_;
  std::uint64_t count_;
  double max_ms_;
};

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_TICK_STATS_H_
