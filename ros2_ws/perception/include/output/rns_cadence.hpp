/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_cadence.hpp
 * Brief: Objects-only R4 publisher clock ledger, not a receiver or Hz claim.
 *
 * Description:
 * What problem this solves. The Objects cadence contract (11 section 3.1B.3 v2.1, 19 section 6.3)
 * is judged on the interval between consecutive UNIQUE Objects publications (by capture time),
 * not on an average rate: target every interval <= 50 ms; tier 1: the maximum interval,
 * including the still-open one, <= 100 ms; tier 2: at most 1 % of the intervals in the last 10 s
 * above 50 ms. At run time the same numbers drive two Status reasons: infer_gap (any interval
 * > 50 ms) and infer_rate_low (> 1 % above 50 ms, or any > 100 ms; cleared only after 10 s clean).
 *
 * What is not counted. The first 30 s after start (power-on warm-up, reply 2026-09-11) and every
 * interval inside a declared reconnecting / reconfiguring period. A pause that was NOT declared
 * counts: that is the point of the declaration rule.
 *
 * What this is not. It records the publisher's own put-before times. It says nothing about
 * delivery to a subscriber, and "unique" is by capture time, so a republished frame never looks
 * like a healthy interval.
 */
#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace perception::rns {
// What observe() did with one publication. Only New closes an interval.
enum class CadenceEvent { New, Duplicate, OutOfOrder, Excluded, Invalid };
// The declared exclusion periods of 11 section 3.1B.3; each maps to a Status reason.
enum class Exclusion { None, Reconnecting, Reconfiguring };
// The ledger at one instant: gate inputs, gate results and the runtime reasons.
struct CadenceSnapshot {
    std::int64_t observed_ms = -1;
    Exclusion declared_exclusion = Exclusion::None; // distinct from startup warmup
    bool excluded = false, awaiting_new_segment = false, full_window = false, valid = true;
    // Closed intervals in the 10 s window and how many exceed 50 / 100 ms.
    std::size_t closed_gaps = 0, over50 = 0, over100 = 0;
    // open_gap_ms = now - last publication; max_gap_ms includes it.
    std::int64_t open_gap_ms = 0, max_gap_ms = 0;
    // Nearest-rank P99 over the closed intervals plus the open one.
    std::optional<std::int64_t> p99_with_open_ms;
    double over50_fraction = 0;
    // tier1: max <= 100 ms; tier2: over50 <= 1 % of closed; pass: both over a full window.
    bool tier1 = false, tier2 = false, pass = false;
    // Lifetime counters for the log; invalid_events > 0 makes the snapshot invalid.
    std::uint64_t duplicates = 0, out_of_order = 0, unique_results = 0;
    std::uint64_t excluded_results = 0, invalid_events = 0;
    // Total declared-excluded time, and the largest interval ever seen outside exclusions.
    std::int64_t declared_excluded_ms = 0, historical_max_gap_ms = 0;
    // Runtime reasons are updated on every unique put and explicit open-gap
    // observation, not inferred from 1 Hz average FPS. Invalid/excluded input
    // does not erase reasons. These are not remote delivery/GEN receipts.
    bool infer_gap = false, infer_rate_low = false;
    std::optional<std::int64_t> infer_gap_since_ms, infer_rate_low_since_ms;
};
// Single owning publisher thread. Bounded memory; allocate only at construction.
// The integration must supply actual put-before timestamps. A successful API
// return is not evidence of remote receipt; this class has no receive counter.
class ObjectsCadence {
public:
    // process_start_ms anchors the 30 s warm-up; gap_capacity bounds the 10 s ring.
    ObjectsCadence(std::int64_t process_start_ms, std::size_t gap_capacity);
    // One Objects put: its source capture time (identity) and put-before time.
    CadenceEvent observe(std::int64_t source_capture_ms, std::int64_t put_before_ms);
    // Enter or leave an exclusion period; returns false on a bad value or clock.
    bool declare_exclusion(Exclusion, std::int64_t now_ms);
    // Also advances the window and the open gap to now_ms; times must not go back.
    CadenceSnapshot snapshot(std::int64_t now_ms);
private:
    // One closed interval: when it ended (its second publication) and its length.
    struct Gap { std::int64_t end = 0, value = 0; };
    // Monotonic time check; a rollback counts as invalid.
    bool clock(std::int64_t now);
    // Drop intervals that ended 10 s or more before now.
    void prune(std::int64_t now);
    // Account the open interval up to now (see the .cpp for the two modes).
    void account_open(std::int64_t now, bool evaluate_closed = true);
    // Latch or clear infer_gap / infer_rate_low (19 section 6.3).
    void update_reasons(std::int64_t now);
    // True during the first 30 s after process start.
    bool warmup(std::int64_t now) const;
    // Fixed ring of closed intervals, and a scratch copy for the P99 selection.
    std::vector<Gap> gaps_;
    std::vector<std::int64_t> scratch_;
    std::size_t begin_ = 0, count_ = 0, over50_ = 0, over100_ = 0;
    std::int64_t start_, last_clock_, declared_ms_ = 0, exclusion_start_ = 0;
    std::int64_t historical_max_ = 0;
    // Last unique capture time, last publication, start of the current clean segment.
    std::optional<std::int64_t> last_identity_, last_publish_, segment_start_;
    // When each reason was latched; empty = not active.
    std::optional<std::int64_t> gap_reason_since_, rate_reason_since_;
    Exclusion exclusion_ = Exclusion::None;
    // After an exclusion ends, nothing counts until the next NEW publication.
    bool awaiting_resume_ = false;
    std::uint64_t duplicates_ = 0, out_of_order_ = 0, unique_ = 0, excluded_ = 0, invalid_ = 0;
};
}  // namespace perception::rns
