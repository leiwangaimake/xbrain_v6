/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_cadence.cpp
 * Brief: Explicit closed-gap ratio plus separately reported nearest-rank P99.
 *
 * Description:
 * Two kinds of interval. A CLOSED interval ends with a new unique publication and enters the
 * 10 s ring. The OPEN interval is now minus the last publication; it is never stored as a
 * sample (a stall seen at ten 1 Hz snapshots is one interval, not ten), but it counts toward the
 * maximum and the reasons immediately, so a dead feed cannot hide behind an empty window.
 *
 * Why the ratio uses closed intervals only. Tier 2 is "share of intervals above 50 ms"; the open
 * interval is not finished, so it has no place in a share. The P99 reported beside it includes
 * the open interval as one extra sample, for the operator's view.
 *
 * Where it runs. The service keeps two instances fed with the same puts, each on one thread:
 * one inside the Objects publisher channel (it returns the per-put CadenceEvent), and one on the
 * Status thread, fed from the Objects publication journal, whose snapshot drives the Status
 * reasons and the cadence fields. Both hold 1024 intervals.
 *
 * Tests. tests/test_rns_cadence.cpp and tests/test_rns_declaration.cpp.
 */
#include "output/rns_cadence.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace perception::rns {
namespace {
// 10 s observation window (19 section 6.3); 30 s power-on warm-up (reply 2026-09-11).
constexpr std::int64_t kWindow = 10000, kWarmup = 30000;
// At 30 Hz a 10 s window holds about 300 intervals (the service uses 1024); the
// bound only rejects nonsense. At least 2 so a window can hold a real interval.
std::size_t checked_capacity(std::size_t n) {
    if (n < 2 || n > 1000000) throw std::invalid_argument("cadence capacity");
    return n;
}
}
ObjectsCadence::ObjectsCadence(std::int64_t start, std::size_t capacity)
    : gaps_(checked_capacity(capacity)), scratch_(gaps_.size() + 1), start_(start), last_clock_(start) {
    // start + warm-up must be representable: every "no publication yet" open gap is
    // measured from that instant.
    if (start < 0 || start > std::numeric_limits<std::int64_t>::max() - kWarmup)
        throw std::invalid_argument("cadence start");
}
// Warm-up is measured from process start, not from the first publication.
bool ObjectsCadence::warmup(std::int64_t now) const { return now - start_ < kWarmup; }
// Any time going backwards makes every later interval meaningless: count and refuse.
bool ObjectsCadence::clock(std::int64_t now) {
    if (now < last_clock_) { ++invalid_; return false; }
    last_clock_ = now;
    return true;
}
void ObjectsCadence::prune(std::int64_t now) {
    // Right-closed observation window (now-10s, now]. Old long intervals do
    // not erase the current unfinished gap; it is calculated independently.
    // Intervals are stored in end-time order, so pruning stops at the first live one;
    // the over-threshold counters follow each removed interval.
    while (count_ && now - gaps_[begin_].end >= kWindow) {
        over50_ -= gaps_[begin_].value > 50;
        over100_ -= gaps_[begin_].value > 100;
        begin_ = (begin_ + 1) % gaps_.size(); --count_;
    }
}
void ObjectsCadence::account_open(std::int64_t now, bool evaluate_closed) {
    // Nothing is open while excluded, warming up, or waiting for the first
    // publication after an exclusion.
    if (exclusion_ != Exclusion::None || warmup(now) || awaiting_resume_) return;
    // Before any publication the open interval runs from the end of warm-up.
    const auto base = last_publish_.value_or(start_ + kWarmup);
    // The open interval can already be the longest ever: record it now.
    historical_max_ = std::max(historical_max_, now - base);
    prune(now);
    if (evaluate_closed) update_reasons(now);
    else if (!invalid_) {
        // observe() has not committed its new gap yet. Preserve the real open
        // evidence, but do not latch/clear against a transient closed-count
        // denominator (e.g. 2/199 before appending -> valid 2/200 afterwards).
        if (now - base > 50 && !gap_reason_since_) gap_reason_since_ = now;
        if (now - base > 100 && !rate_reason_since_) rate_reason_since_ = now;
    }
}
// infer_gap: any interval > 50 ms. infer_rate_low: any > 100 ms or > 1 % above 50 ms.
// Both latch at once and clear only after 10 s of clean running (19 section 6.3).
void ObjectsCadence::update_reasons(std::int64_t now) {
    if (invalid_ || exclusion_ != Exclusion::None || warmup(now) || awaiting_resume_) return;
    const auto open = now - last_publish_.value_or(start_ + kWarmup);
    // over50_ * 100 > count_ is "more than 1 %" in exact integer arithmetic.
    const bool gap_bad = over50_ > 0 || open > 50;
    const bool rate_bad = over100_ > 0 || open > 100 || over50_ * 100 > count_;
    // Recovery needs evidence: a full 10 s of the current clean segment.
    const bool complete = segment_start_ && now - *segment_start_ >= kWindow && count_ > 0;
    // Concrete interpretation of 19 A19-RATE-1: a complete *current* healthy
    // ten-second interval plus >=10s since latching proves recovery. Do NOT
    // wait another 10s after the bad interval has already rolled out. Rate may
    // recover with <=1% 51..100ms intervals; gap remains active in that case.
    // Reconnect resets segment_start_ and cannot lend its duration as evidence.
    if (gap_bad) { if (!gap_reason_since_) gap_reason_since_ = now; }
    else if (gap_reason_since_ && complete && now - *gap_reason_since_ >= kWindow)
        gap_reason_since_.reset();
    if (rate_bad) { if (!rate_reason_since_) rate_reason_since_ = now; }
    else if (rate_reason_since_ && complete && now - *rate_reason_since_ >= kWindow)
        rate_reason_since_.reset();
}
CadenceEvent ObjectsCadence::observe(std::int64_t capture, std::int64_t publish) {
    if (!clock(publish)) return CadenceEvent::Invalid;
    // A capture after its own publication is a clock-domain error, not a sample.
    if (capture < 0 || capture > publish) { ++invalid_; return CadenceEvent::Invalid; }
    // Bring the open interval up to this instant first (without judging reasons
    // against a count that is about to change).
    account_open(publish, false);
    // Same capture again: a republish, never a new interval (unique by capture).
    if (last_identity_ && capture == *last_identity_) {
        ++duplicates_; update_reasons(publish); return CadenceEvent::Duplicate;
    }
    // An older capture after a newer one: out of order, also not an interval.
    if (last_identity_ && capture < *last_identity_) {
        ++out_of_order_; update_reasons(publish); return CadenceEvent::OutOfOrder;
    }
    last_identity_ = capture;
    // Unique but not counted: warm-up or a declared exclusion.
    if (warmup(publish) || exclusion_ != Exclusion::None) { ++excluded_; return CadenceEvent::Excluded; }
    prune(publish);
    // A counted publication closes the interval since the previous counted one.
    if (last_publish_) {
        const auto gap = publish - *last_publish_;
        historical_max_ = std::max(historical_max_, gap);
        // Full ring inside 10 s: the capacity was misconfigured; refuse, never drop.
        if (count_ == gaps_.size()) { ++invalid_; return CadenceEvent::Invalid; }
        gaps_[(begin_ + count_) % gaps_.size()] = {publish, gap}; ++count_;
        over50_ += gap > 50; over100_ += gap > 100;
    } else segment_start_ = publish;  // first publication: a new clean segment starts
    last_publish_ = publish; awaiting_resume_ = false; ++unique_;
    // Reasons are judged with the new interval committed.
    update_reasons(publish);
    return CadenceEvent::New;
}
bool ObjectsCadence::declare_exclusion(Exclusion next, std::int64_t now) {
    // Declarations use the same clock as puts; a rollback is refused.
    if (!clock(now)) return false;
    // Only the defined values; anything else is memory damage or a wiring bug.
    if (next != Exclusion::None && next != Exclusion::Reconnecting && next != Exclusion::Reconfiguring) {
        ++invalid_; return false;
    }
    // Repeating the current state is a no-op (idempotent declarations).
    if (next == exclusion_) return true;
    // Close the books on the current state first:
    account_open(now);  // A later declaration cannot retroactively erase a stall.
    // Leaving (or switching) an exclusion adds its duration to the declared total.
    if (exclusion_ != Exclusion::None) declared_ms_ += now - exclusion_start_;
    exclusion_ = next;
    exclusion_start_ = now;
    // Both entering and leaving start a fresh window: intervals never span a boundary.
    begin_ = count_ = over50_ = over100_ = 0;
    last_publish_.reset(); segment_start_.reset();
    awaiting_resume_ = true;
    // After an exclusion, neither old identities nor its time interval count.
    // Await first NEW source frame; historical_max_/invalid_ survive the reset.
    return true;
}
CadenceSnapshot ObjectsCadence::snapshot(std::int64_t now) {
    const bool clock_ok = clock(now);
    // Fill what is known even on a bad clock, so the log shows the counters.
    CadenceSnapshot s;
    s.observed_ms = now;
    s.declared_exclusion = exclusion_;
    // Reasons as latched so far (also reported when the clock is bad).
    s.infer_gap = gap_reason_since_.has_value();
    s.infer_rate_low = rate_reason_since_.has_value();
    s.infer_gap_since_ms = gap_reason_since_;
    s.infer_rate_low_since_ms = rate_reason_since_;
    // Any invalid input ever makes every later snapshot invalid (sticky).
    s.valid = clock_ok && invalid_ == 0;
    // Lifetime counters: diagnostics for the log, they do not enter any gate.
    s.duplicates = duplicates_; s.out_of_order = out_of_order_;
    s.unique_results = unique_; s.excluded_results = excluded_; s.invalid_events = invalid_;
    if (!clock_ok) return s;
    // Advance to now (this may latch a reason for a long open interval), then
    // report the reasons again as they stand after that update.
    prune(now); account_open(now);
    s.infer_gap = gap_reason_since_.has_value();
    s.infer_rate_low = rate_reason_since_.has_value();
    s.infer_gap_since_ms = gap_reason_since_;
    s.infer_rate_low_since_ms = rate_reason_since_;
    // Excluded = warm-up or a declaration; no gate is judged while excluded.
    s.excluded = warmup(now) || exclusion_ != Exclusion::None;
    s.awaiting_new_segment = awaiting_resume_;
    // Declared total includes the period still in progress.
    s.declared_excluded_ms = declared_ms_ + (exclusion_ == Exclusion::None ? 0 : now - exclusion_start_);
    s.historical_max_gap_ms = historical_max_;
    // No gate statistics while nothing is being counted.
    if (s.excluded || s.awaiting_new_segment) return s;
    s.full_window = segment_start_ && now - *segment_start_ >= kWindow;
    s.closed_gaps = count_;
    // Copy the window into scratch for the P99 selection and recount the thresholds
    // from the samples themselves (the running counters are for the reasons).
    for (std::size_t i = 0; i < count_; ++i) {
        const auto gap = gaps_[(begin_ + i) % gaps_.size()].value;
        scratch_[i] = gap;
        s.over50 += gap > 50; s.over100 += gap > 100;
        s.max_gap_ms = std::max(s.max_gap_ms, gap);
    }
    // The open interval joins the maximum and, as one extra sample, the P99.
    s.open_gap_ms = now - last_publish_.value_or(start_ + kWarmup);
    s.max_gap_ms = std::max(s.max_gap_ms, s.open_gap_ms);
    scratch_[count_] = s.open_gap_ms;
    // n >= 1 always (the open interval), so the rank below is at least 1.
    const std::size_t n = count_ + 1;
    // ceil(0.99*n), using exact integer arithmetic, not round() or float.
    const std::size_t rank = (99 * n + 99) / 100;
    // nth_element: O(n) and no allocation (scratch_ was sized at construction).
    std::nth_element(scratch_.begin(), scratch_.begin() + rank - 1, scratch_.begin() + n);
    s.p99_with_open_ms = scratch_[rank - 1];
    // Share for display; the tier-2 verdict below uses exact integers, not this double.
    s.over50_fraction = count_ ? static_cast<double>(s.over50) / static_cast<double>(count_) : 0;
    // Tier 1 includes the open interval: a stall fails it immediately.
    s.tier1 = s.valid && s.max_gap_ms <= 100;
    // Exact integer share of publish intervals (dpub) over 50 ms, at most 1 percent. The open
    // gap is judged by tier 1 and is not counted again here on every tick.
    s.tier2 = s.valid && count_ > 0 && s.over50 * 100 <= count_;
    // A verdict needs a full 10 s window of the current segment.
    s.pass = s.full_window && s.tier1 && s.tier2;
    return s;
}
}  // namespace perception::rns
