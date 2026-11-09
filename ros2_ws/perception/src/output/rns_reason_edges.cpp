/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_reason_edges.cpp
 * Brief: Set difference, per-bit per-direction dedupe, and an atomic one-entry enqueue.
 *
 * Description:
 * update() in four steps: validate the input (known bits, time not backwards); take the set
 * difference against the previous set and store the new set unconditionally (the active set
 * always follows the input, even when no edge can be queued); remove from each direction the
 * bits whose last enqueued edge of that direction is younger than 10 s; enqueue one edge if
 * anything is left.
 *
 * Why anchors move only on enqueue. The 10 s window counts from the last edge that was really
 * queued for sending. A suppressed or lost edge moves nothing, so a flapping reason cannot keep
 * pushing its own window forward and stay silent for ever; the next real change after 10 s is
 * always sent.
 *
 * Why Warn and Info anchors are separate. A warn must never suppress the matching recovery: a
 * reason that appears and clears within 10 s produces both a warn and an info edge.
 *
 * Tests. tests/test_rns_reason_edges.cpp (dedupe boundary at exactly 10000 ms, mixed edges,
 * overflow atomicity, saturation) and tests/test_rns_reason_flow.cpp (host-only flow from
 * injected cadence puts through these edges into Status).
 */
#include "output/rns_reason_edges.hpp"
#include <limits>
#include <stdexcept>

namespace perception::rns {
namespace {
// Configuration check for the member initialiser; throws at startup only.
std::size_t checked_capacity(std::int64_t startup, std::size_t causes, std::size_t capacity) {
    // startup is the first allowed update time; causes must fit one uint64 set.
    if (startup < 0 || startup > ProcessEnvelope::kMaxMonoMs || causes == 0 ||
        causes > ReasonEdges::kMaxCauseCount || capacity == 0 || capacity > ReasonEdges::kMaxQueueCapacity)
        throw std::invalid_argument("invalid reason edge configuration");
    return capacity;
}
// Portable popcount (clears the lowest set bit per iteration, at most 64 rounds).
std::uint64_t bit_count(std::uint64_t bits) noexcept {
    std::uint64_t result = 0;
    while (bits) { bits &= bits - 1; ++result; }
    return result;
}
}  // namespace
ReasonEdges::ReasonEdges(std::int64_t startup, std::size_t causes, std::size_t capacity)
    : queue_(checked_capacity(startup, causes, capacity)), cause_count_(causes),
      // 1 << 64 is undefined behaviour, hence the explicit all-ones case.
      allowed_(causes >= 64 ? std::numeric_limits<std::uint64_t>::max() : (std::uint64_t{1} << causes) - 1),
      last_now_(startup) {
    // -1 = no edge ever queued for this bit: the first change is never suppressed.
    last_warn_.fill(-1); last_info_.fill(-1);
}
void ReasonEdges::add(std::uint64_t& counter, std::uint64_t amount) noexcept {
    constexpr auto max = std::numeric_limits<std::uint64_t>::max();
    // Saturate instead of wrapping; a wrapped counter would read as a clean run.
    if (amount > max - counter) { counter = max; counters_.saturated = true; valid_ = false; }
    else counter += amount;
}
std::uint64_t ReasonEdges::suppressed(std::uint64_t causes,
        const std::array<std::int64_t, 64>& last, std::int64_t now) const noexcept {
    std::uint64_t result = 0;
    // Per bit: suppressed only if it changed in this direction AND its last queued
    // edge of this direction exists and is younger than 10 s (== 10 s is allowed).
    for (std::size_t i = 0; i < cause_count_; ++i) {
        const auto bit = std::uint64_t{1} << i;
        // Both times are nonnegative, ordered, <=kMaxMonoMs; subtraction is safe.
        if ((causes & bit) && last[i] >= 0 && now - last[i] < kDedupeMs) result |= bit;
    }
    return result;
}
// Caller has checked there is room; this cannot fail.
void ReasonEdges::enqueue(const ReasonEdge& edge) noexcept {
    queue_[(begin_ + size_) % queue_.size()] = edge; ++size_;
    // Only eligible bits restart their 10 s window; suppressed bits keep theirs.
    for (std::size_t i = 0; i < cause_count_; ++i) {
        const auto bit = std::uint64_t{1} << i;
        if (edge.eligible_added & bit) last_warn_[i] = edge.observed_ms;
        if (edge.eligible_removed & bit) last_info_[i] = edge.observed_ms;
    }
    // Counted here, the single place an edge enters the queue.
    add(counters_.enqueued_edges, 1);
}
ReasonEdgeUpdate ReasonEdges::update(std::uint64_t next, std::int64_t now) noexcept {
    ReasonEdgeUpdate result;
    result.active_after = active_;
    // Invalid input changes nothing but the fault latch and its counter.
    // Rejected: negative or unrepresentable time, time going backwards (equal is
    // fine: several cuts may fall in one millisecond), or a bit with no name.
    if (now < 0 || now > ProcessEnvelope::kMaxMonoMs || now < last_now_ || (next & ~allowed_)) {
        valid_ = false; add(counters_.invalid_updates, 1);
        result.code = ReasonEdgeCode::Invalid; return result;
    }
    last_now_ = now;
    // Raw edge: what was added and what was removed since the previous update.
    result.added = next & ~active_; result.removed = active_ & ~next;
    active_ = next; result.active_after = active_;
    // Same set as before: Unchanged, no entry (no periodic resend, 19 section 5.2).
    if (!(result.added | result.removed)) return result;
    result.code = ReasonEdgeCode::Accepted;
    // Raw bit counts include what the dedupe will remove, so the log can show both.
    add(counters_.added_bits, bit_count(result.added));
    add(counters_.removed_bits, bit_count(result.removed));
    // Dedupe per direction: additions against warn anchors, removals against info.
    result.suppressed_added = suppressed(result.added, last_warn_, now);
    result.suppressed_removed = suppressed(result.removed, last_info_, now);
    add(counters_.suppressed_added_bits, bit_count(result.suppressed_added));
    add(counters_.suppressed_removed_bits, bit_count(result.suppressed_removed));
    // Everything suppressed: the set change is recorded in the counters only.
    result.eligible_added = result.added & ~result.suppressed_added;
    result.eligible_removed = result.removed & ~result.suppressed_removed;
    if (!(result.eligible_added | result.eligible_removed)) return result;
    // Full queue: queue nothing (never a partial edge), keep the old entries, count
    // the loss and latch; anchors stay where they were.
    if (size_ == queue_.size()) {
        valid_ = false; result.code = ReasonEdgeCode::Overflow; result.lost_edges = 1;
        add(counters_.overflow_updates, 1); add(counters_.lost_edges, 1);
        // lost_bits counts only the bits that would have been sent.
        add(counters_.lost_bits, bit_count(result.eligible_added | result.eligible_removed));
        return result;
    }
    // Any eligible addition makes the whole edge a warn; pure recoveries are info.
    const ReasonEdge edge{result.eligible_added ? ReasonEdgeSeverity::Warn : ReasonEdgeSeverity::Info,
        result.added, result.removed, result.eligible_added, result.eligible_removed,
        result.suppressed_added, result.suppressed_removed, active_, now};
    // The raw and suppressed parts travel with the edge, so the event can say what
    // else changed on this beat even when that part was deduplicated (GenEvents
    // writes raw_added/raw_removed next to the eligible added/removed lists).
    enqueue(edge); result.enqueued_edges = 1;
    return result;
}
// Oldest first; the vacated slot is reset so no stale edge stays in memory.
bool ReasonEdges::pop(ReasonEdge& out) noexcept {
    out = {};
    // Empty queue: out is the default edge and false; faults stay latched.
    if (!size_) return false;
    out = queue_[begin_]; queue_[begin_] = {};
    begin_ = (begin_ + 1) % queue_.size(); --size_;
    // enqueued - popped always equals queued(); the tests check both totals.
    add(counters_.popped_edges, 1);
    return true;
}
}  // namespace perception::rns
