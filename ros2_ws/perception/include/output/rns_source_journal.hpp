/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_source_journal.hpp
 * Brief: Single-producer ring of timestamped source events with a cut-safe generation counter.
 *
 * Description:
 * What problem this solves. Three Status numbers come from three different threads: fps_depth
 * (depth frames arriving, capture thread), fps_infer (inference results, inference thread) and
 * invalid_pixel_ratio (processed depth frames, Profile thread). The Status thread must count
 * them over exactly the same window, cut at one instant, without locks on the producer side.
 * Each producer therefore records into its own SourceJournal; the Status thread drains all of
 * them up to one cut time (PublicationHealthHandoff in rns_publication_handoff).
 *
 * The cut problem. A consumer that reads "head" and then "now" could miss an event whose time
 * is before its cut but whose slot was not yet published. The generation counter closes that
 * gap: a producer makes it odd before sampling its clock and even again after publishing the
 * slot. A consumer that sees the same even generation before and after sampling its clock and
 * the head knows no event with an earlier time is still in flight.
 *
 * In the service: three journals of 2048 entries each (DepthArrival, InferenceComplete,
 * DepthProcessed), all on the service monotonic clock.
 */
#pragma once
#include "output/rns_depth_health.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace perception::rns {
// Which producer a journal belongs to; each journal accepts exactly one kind.
enum class SourceKind : std::uint32_t { DepthArrival, InferenceComplete, DepthProcessed };
// One recorded event: the frame's identity and when the producer recorded it.
struct SourceEvent {
    SourceKind kind = SourceKind::DepthArrival;
    native_depth::SourceId source;
    std::int64_t observed_ns = -1;
    // Present evidence only for DepthProcessed. Other kinds retain a default
    // InvalidFrame observation, never an invented zero invalid-pixel ratio.
    DepthHealthObservation depth;
};
// Producer clock: a plain function pointer (no allocation, no exceptions).
struct SourceClock {
    void* context = nullptr;
    std::int64_t (*now_ns)(void*) noexcept = nullptr;
};

// One producer, one consumer. This transfers prepared source observations into
// a fixed ring, not a clock mapper, metrics aggregator or camera owner proof.
// Construct/destruct only after both owners are quiescent. Clock context must
// outlive all record calls; callback must return local MONOTONIC nanoseconds,
// be noexcept and production-nonblocking. No fake/fallback clock is supplied.
class SourceJournal {
public:
    SourceJournal(SourceKind, std::size_t capacity, SourceClock);
    SourceJournal(const SourceJournal&) = delete;
    SourceJournal& operator=(const SourceJournal&) = delete;
    // Source preparation, inference and ROI scans happen BEFORE record().
    // The bound callback is sampled INSIDE an odd seq_cst generation window.
    // Arrival can precede RGB: absent color fields are retained as absent.
    // Inference/processed require both positive device timestamps; SDK index 0
    // is legitimate for a first frame. No cross-device clock subtraction.
    bool record(const native_depth::SourceId&) noexcept; // arrival/inference only
    bool record(const DepthHealthObservation&) noexcept; // processed only
    // Configured and no fault latched yet.
    bool valid() const noexcept;
    SourceKind kind() const noexcept { return kind_; }
    // Odd while a record() is between its clock sample and its publish.
    std::uint64_t generation() const noexcept;
    // Total events published (head) and consumed (tail) since construction.
    std::uint64_t published() const noexcept;
    std::uint64_t consumed() const noexcept;
    // Consumer side: read the oldest event without / with removing it.
    bool peek(SourceEvent&) const noexcept;
    bool pop(SourceEvent&) noexcept;
    // Diagnostics for the log; either being non-zero means valid() is false.
    std::uint64_t overflow_count() const noexcept;
    std::uint64_t fault_count() const noexcept;

    // Same stream: source sequence and depth_index strictly increase, while
    // depth_device_us, host_ready and sampled observation time may tie but may
    // not regress. A changed stream ID is NOT numerically ordered; host time
    // still cannot regress. Actual current stream ownership and rejection of a
    // forged/reused old stream ID belong to the lifecycle owner, not an
    // unbounded history map hidden in this journal.
    //
    // Overflow/invalid calls are sticky faults, not silent sample omission.
    // Previously committed events remain drainable after failure. Failed calls
    // that started timing always restore an EVEN generation; valid()==false
    // must still prevent consumers from labelling that window complete.
    //
    // Consumer must capture a finite head and double-check the same even
    // generation around its clock/head sampling. Millisecond health cutoffs
    // additionally need floor(now)-1ms; queue availability is not a time cut.
    // No disabled/default-mode policy is implemented here.
private:
    friend struct SourceJournalTestAccess;
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free &&
                  std::atomic<std::uint32_t>::is_always_lock_free,
                  "source journal requires fixed-width lock-free atomics");
    // Shared body of both record() overloads.
    bool write(const native_depth::SourceId&, const DepthHealthObservation*, bool) noexcept;
    // Identity and ordering rules (see the comment above the private section).
    bool source_valid(const native_depth::SourceId&, std::int64_t observed_ns) const noexcept;
    // Producer-only saturating increment; saturation latches invalid.
    void increment(std::atomic<std::uint64_t>&) noexcept;
    // Count a fault and latch invalid.
    void fail() noexcept;
    const SourceKind kind_;
    const SourceClock clock_;
    const bool configured_;
    const std::uint64_t capacity_;
    // Fixed ring; positions are monotonic counters, slot = position % capacity.
    std::vector<SourceEvent> events_;
    std::atomic<std::uint64_t> head_{0}, tail_{0}, generation_{0};
    std::atomic<std::uint64_t> overflow_{0}, faults_{0};
    std::atomic<std::uint32_t> invalid_{0};
    // Producer-owned only; the consumer reads copies in committed Event slots.
    native_depth::SourceId last_source_;
    std::int64_t last_observed_ns_ = -1;
    bool ever_ = false;
};
} // namespace perception::rns
