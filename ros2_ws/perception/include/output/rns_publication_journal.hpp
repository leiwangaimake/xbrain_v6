/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_publication_journal.hpp
 * Brief: Per-topic SPSC record of publish attempts, completions and cadence declarations.
 *
 * Description:
 * What problem this solves. Status reports latency (t_publish - t_capture), publication gaps and
 * put outcomes for Objects and Profile (19 section 5.1). Those facts exist only on the publishing
 * thread, at the moment of the put. This journal carries them, without locks, to the Status
 * thread: an Attempt event when the envelope is stamped (capture, put-before time, seq), a
 * Completion event when the native put returns (return time, outcome), and for Objects the
 * reconnecting/reconfiguring declarations that exclude time from the cadence gate.
 *
 * No accounting, no publication. PublisherChannel reserves two slots before it starts a put;
 * if the journal is full the put is refused with TelemetryUnavailable. A message that Status
 * could not account for would make the health numbers silently wrong.
 *
 * Phases. The producer walks Idle -> Reserved -> AttemptTiming -> AttemptCommitted ->
 * CompletionTiming -> Idle (or Idle -> DeclarationTiming -> Idle). Any call out of order is a
 * sticky protocol fault. Generation is odd exactly while a clock sample and its event commit
 * are in progress, which is what lets the Status thread cut a complete window (same protocol as
 * SourceJournal).
 *
 * In the service: one journal per topic (Profile, Objects), 1024 events each.
 */
#pragma once
#include "output/rns_publisher.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace perception::rns {

// One producer (the topic's PublisherChannel owner), one consumer (the health
// owner). This is a bounded event transfer, not a health window or receipt
// oracle. No method starts a thread, reads a clock or calls a transport.
class PublicationJournal {
public:
    // Attempt: stamped and about to put. Completion: put returned. Declaration:
    // Objects cadence exclusion changed.
    enum class Kind : std::uint32_t { Attempt, Completion, Declaration };
    // One journal entry; trivially copyable so a slot copy never allocates.
    struct Event {
        using Kind = PublicationJournal::Kind;
        Kind kind = Kind::Attempt;
        Topic topic = Topic::Profile;
        PublishAudit audit;
        // Objects owner control event, not a put/receipt. Other kinds leave
        // these fields at their defaults; Declaration leaves audit untouched.
        Exclusion declaration = Exclusion::None;
        Millis declared_ms = -1;
    };
    // admission_rejected: puts refused because the journal was full;
    // protocol_faults: out-of-order calls (each also latches valid() false).
    struct SnapshotCounters {
        std::uint64_t admission_rejected = 0, protocol_faults = 0;
    };

    // Profile/Objects only, capacity >= 2 events. Construction may allocate;
    // unsupported topic/capacity yields valid()==false. Allocation failure may
    // throw at construction, never on a producer/consumer hot path.
    PublicationJournal(Topic topic, std::size_t event_capacity);
    PublicationJournal(const PublicationJournal&) = delete;
    PublicationJournal& operator=(const PublicationJournal&) = delete;

    Topic topic() const noexcept { return topic_; }
    // Configured and no protocol fault or counter saturation yet.
    bool valid() const noexcept;
    // Reserves TWO slots before a publish can begin. Full queue is a counted
    // admission rejection, not a protocol fault. Generation remains even.
    bool reserve() noexcept;
    // Call before sampling put-before time. The odd generation covers clock
    // sampling, stamp assembly and Attempt publication, not the native put.
    void begin_attempt() noexcept;
    // Commit the Attempt (must carry a seq, capture <= put-before) and end timing.
    void record_attempt(const PublishAudit&) noexcept;
    // Attempt is visible and generation even while native put is pending.
    // Call before sampling its RETURN clock; completion retains the real audit
    // even when clock_valid=false or the result is Rejected/Unknown/thrown.
    void begin_completion() noexcept;
    // Commit the Completion; its identity must equal the committed Attempt.
    void record_completion(const PublishAudit&) noexcept;
    // Objects owner only, while no publication is in flight. Reserve ONE slot
    // and make generation odd BEFORE the owner reads its actual clock.
    // A full queue is retryable and never changes phase/generation.
    bool begin_declaration() noexcept;
    // Commit the declaration with the owner's clock sample; ends the odd window.
    bool record_declaration(Exclusion, Millis declared_ms) noexcept;
    // Only before record_attempt: cancelled preparation or failed clock/stamp.
    // Cancelling an already begun time sample returns its generation to even.
    // Cancelling a committed Attempt is a fault, never a fabricated completion.
    void cancel() noexcept;

    // Consumer owner only. False means no committed event; output is unchanged.
    // Peek does not release the slot; pop copies then releases it. Only one
    // consumer may call peek/pop/consumed, including after a sticky fault.
    // Committed events remain drainable on fault; no producer recovery/reset.
    bool peek(Event&) const noexcept;
    bool pop(Event&) noexcept;
    std::uint64_t generation() const noexcept; // seq_cst; odd == sampling/commit
    std::uint64_t published() const noexcept;  // acquire logical write position
    std::uint64_t consumed() const noexcept;   // acquire logical read position
    // Each field read atomically; the pair is not one transaction.
    SnapshotCounters counters() const noexcept;

    // Queue transfer alone does NOT define a complete millisecond time cut.
    // Health integration must separately double-check the SAME even generation
    // around its clock/head observations, use floor(now)-1ms as a conservative
    // cutoff, and drain only up to a captured published() head. A producer may
    // otherwise still commit an event from the same truncated clock millisecond.
    // Counter fields are individually atomic, not one transactional pair.
private:
    friend struct PublicationJournalTestAccess;
    // Producer state machine (see the file header).
    enum class Phase : std::uint32_t {
        Idle, Reserved, AttemptTiming, AttemptCommitted, CompletionTiming, DeclarationTiming
    };
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free &&
                  std::atomic<std::uint32_t>::is_always_lock_free,
                  "publication journal requires fixed-width lock-free atomics");
    // Count and latch a protocol fault.
    void fault() noexcept;
    // Single-writer saturating increment; saturation latches the fault.
    void increment(std::atomic<std::uint64_t>&) noexcept;
    // True if valid and in the expected phase; otherwise a fault.
    bool in_phase(Phase) noexcept;
    // Write the slot at head, then publish head (release).
    void commit(Kind, const PublishAudit&) noexcept;
    // Set the next phase and make generation even again.
    void finish_timing(Phase) noexcept;

    const Topic topic_;
    const bool configured_;
    const std::uint64_t capacity_;
    std::vector<Event> events_;
    // Only the producer writes head/generation/phase/counters/fault/attempt.
    // Only the consumer writes tail. Slot ownership uses release/acquire.
    std::atomic<std::uint64_t> head_{0}, tail_{0}, generation_{0};
    std::atomic<std::uint32_t> phase_{static_cast<std::uint32_t>(Phase::Idle)}, fault_{0};
    std::atomic<std::uint64_t> admission_rejected_{0}, protocol_faults_{0};
    // The committed Attempt, to check that the Completion belongs to it.
    PublishAudit attempt_;
};
} // namespace perception::rns
