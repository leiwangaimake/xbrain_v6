/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_reason_edges.hpp
 * Brief: Bounded local reason-edge journal; not a GEN/network publisher.
 *
 * Description:
 * What problem this solves. 19 section 5.2: an event is sent on the beat where the
 * degraded_reasons set changes (an addition is warn, a recovery is info), the same cause is
 * deduplicated for 10 s, and nothing is resent periodically (that is Status's job). This class
 * turns the sequence of reason sets into exactly those edges, with the dedupe applied, and
 * queues them for the event sender.
 *
 * How the service uses it. The Status thread builds the active set once per cut as a bitmask
 * over reason_names() in perception_service_main.cpp (14 names on 2026-10-13), calls update(), then
 * pops every queued edge and offers it to GenEvents. The queue holds 256 edges; an overflow or
 * invalid update stops the service ("reason edge journal fault"), because a lost edge would
 * leave the consumer with a wrong picture of what is degraded.
 *
 * Why one entry per update. A beat that adds one reason and clears another is one observed
 * change of the set; splitting it would invent an order between the two that never existed.
 */
#pragma once
#include "output/rns_envelope.hpp"
#include <array>
#include <vector>

namespace perception::rns {
// Warn = something became degraded; Info = only recoveries (19 section 5.2).
enum class ReasonEdgeSeverity { Warn, Info };
// One queued edge. raw_* is the whole change, eligible_* what survives the dedupe,
// suppressed_* what the dedupe removed; active_after is the set after the change.
struct ReasonEdge {
    ReasonEdgeSeverity severity = ReasonEdgeSeverity::Warn;
    std::uint64_t raw_added = 0, raw_removed = 0;
    std::uint64_t eligible_added = 0, eligible_removed = 0;
    std::uint64_t suppressed_added = 0, suppressed_removed = 0;
    std::uint64_t active_after = 0;
    std::int64_t observed_ms = -1;
};
// Accepted: the set changed (an edge may or may not have been queued, see
// enqueued_edges). Overflow/Invalid latch the journal invalid.
enum class ReasonEdgeCode { Accepted, Unchanged, Overflow, Invalid };
// What one update() did, for the caller's log.
struct ReasonEdgeUpdate {
    ReasonEdgeCode code = ReasonEdgeCode::Unchanged;
    std::uint64_t added = 0, removed = 0;
    std::uint64_t eligible_added = 0, eligible_removed = 0;
    std::uint64_t suppressed_added = 0, suppressed_removed = 0;
    std::uint64_t active_after = 0;
    std::size_t enqueued_edges = 0, lost_edges = 0;
};
// Lifetime counters; saturation of any of them latches the journal invalid.
struct ReasonEdgeCounters {
    std::uint64_t added_bits = 0, removed_bits = 0;
    std::uint64_t suppressed_added_bits = 0, suppressed_removed_bits = 0;
    std::uint64_t enqueued_edges = 0, popped_edges = 0, lost_edges = 0, lost_bits = 0;
    std::uint64_t invalid_updates = 0, overflow_updates = 0;
    bool saturated = false;
};

// SINGLE owning thread; callers synchronize status/log handoff. No clock reads,
// network/schema/GEN category/channel/sequence or metric production lives here.
// Startup active set is empty. Bit i names caller-defined reason i, not a wire
// schema. All updates use the startup clock epoch; equal times are permitted.
//
// Each changed-set update emits AT MOST ONE entry. Its explicit directional
// fields preserve raw/eligible/suppressed additions and removals. Local severity
// policy: any eligible addition => Warn; only eligible removals => Info, even
// when some raw additions were suppressed. Mixed edges are never split.
// Dedupe is PER BIT and PER DIRECTION, anchored at the most recently ENQUEUED
// corresponding edge: elapsed <10000 ms suppresses, elapsed ==10000 permits.
// Suppression never postpones the anchor. Warn cannot suppress its recovery.
// Unchanged sets and pop never create periodic/replayed entries.
//
// Entry admission is ATOMIC: when the one eligible entry cannot fit, enqueue
// none, preserve old entries, count its lost entry/bits, return Overflow, and
// still update the active set. Lost edges do not advance dedupe anchors. There
// is no pending retry: only a subsequent real transition can create an edge.
// Overflow/invalid input/counter saturation latch valid=false. Pop and later
// successful updates cannot clear the fault. Unknown bits or invalid/backward
// times do not alter active, anchors, queue or the last accepted input clock.
//
// Constructor allocates one fixed ring; update/pop/getters never allocate.
class ReasonEdges {
public:
    // 19 section 5.2: same cause deduplicated for 10 s.
    static constexpr std::int64_t kDedupeMs = 10000;
    // 64 causes = one bit each in a uint64 set.
    static constexpr std::size_t kMaxCauseCount = 64, kMaxQueueCapacity = 65536;
    // startup in [0,kMaxMonoMs], cause_count in [1,64], queue capacity in
    // [1,kMaxQueueCapacity]. Throws invalid_argument for unsupported inputs.
    ReasonEdges(std::int64_t startup_ms, std::size_t cause_count, std::size_t queue_capacity);
    ReasonEdges(const ReasonEdges&) = delete;
    ReasonEdges& operator=(const ReasonEdges&) = delete;
    // Feed the complete active set observed at now_ms (monotonic ms).
    ReasonEdgeUpdate update(std::uint64_t active, std::int64_t now_ms) noexcept;
    // Oldest first. False clears out to its default value; never clears faults.
    bool pop(ReasonEdge& out) noexcept;
    bool valid() const noexcept { return valid_; }
    std::uint64_t active() const noexcept { return active_; }
    std::size_t queued() const noexcept { return size_; }
    std::size_t capacity() const noexcept { return queue_.size(); }
    ReasonEdgeCounters counters() const noexcept { return counters_; }
private:
    // Host tests seed near-exhaustion counters; no runtime reset API is exposed.
    friend struct ReasonEdgesTestAccess;
    // Saturating counter add; saturation latches invalid.
    void add(std::uint64_t&, std::uint64_t amount) noexcept;
    // Bits of 'causes' whose last enqueued edge in this direction is < 10 s old.
    std::uint64_t suppressed(std::uint64_t causes, const std::array<std::int64_t, 64>&,
                             std::int64_t now) const noexcept;
    // Append one edge and move the dedupe anchors of its eligible bits.
    void enqueue(const ReasonEdge&) noexcept;
    // Fixed ring, allocated once.
    std::vector<ReasonEdge> queue_;
    // Per bit: time of the last enqueued Warn / Info edge carrying it; -1 = never.
    std::array<std::int64_t, 64> last_warn_, last_info_;
    const std::size_t cause_count_;
    // Mask of the cause_count_ low bits; any other bit in an update is invalid.
    const std::uint64_t allowed_;
    std::int64_t last_now_;
    std::uint64_t active_ = 0;
    std::size_t begin_ = 0, size_ = 0;
    bool valid_ = true;
    ReasonEdgeCounters counters_;
};
}  // namespace perception::rns
