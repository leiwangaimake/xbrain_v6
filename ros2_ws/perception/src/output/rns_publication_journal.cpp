/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_publication_journal.cpp
 * Brief: Phase-checked producer protocol and a lock-free ring for publication events.
 *
 * Description:
 * The publisher drives one put as: reserve() (two free slots, else admission rejection);
 * begin_attempt() (generation odd); it samples put-before, stamps the envelope and calls
 * record_attempt() (Attempt committed, generation even); the native put runs with no journal
 * state held; begin_completion() (odd); it samples the return time and calls record_completion()
 * (Completion committed, even, back to Idle). cancel() unwinds a put that failed before its
 * Attempt was committed.
 *
 * Why two slots are reserved up front. Once an Attempt is on the wire its Completion must be
 * recordable; reserving both before the put starts means a full ring can only refuse a put,
 * never strand an Attempt without its Completion.
 *
 * Generation values during one put, starting from an even 4:
 *   reserve()            4  Reserved, nothing timed
 *   begin_attempt()       5  odd: put-before sample and stamp in progress
 *   record_attempt()      6  Attempt visible; the native put runs with generation even
 *   begin_completion()    7  odd: return-time sample in progress
 *   record_completion()   8  Completion visible, Idle
 * A Status cut taken while the value is 5 or 7 is Busy and retried 10 ms later; a cut at 6
 * sees the Attempt without its Completion, which the health window treats as a put in flight.
 *
 * Why the generation arithmetic is bounded up front. A put performs four generation steps and
 * two commits; reserve() checks there is room for all of them, so no later step can overflow.
 *
 * Tests. tests/test_rns_publication_journal.cpp and tests/test_rns_publication_handoff.cpp.
 */
#include "output/rns_publication_journal.hpp"
#include <limits>
#include <type_traits>

namespace perception::rns {
namespace {
// Positions (head, tail, generation) are monotonic uint64 counters; this is their end.
constexpr auto max_position = std::numeric_limits<std::uint64_t>::max();
// Only the two topics with per-put health accounting; room for one Attempt+Completion.
bool configuration(Topic topic, std::size_t capacity) noexcept {
    return (topic == Topic::Profile || topic == Topic::Objects) && capacity >= 2 &&
           capacity <= max_position;
}
// A committed Attempt must be a real stamped put: seq issued, capture before put.
bool attempt_identity(const PublishAudit& a) noexcept {
    // seq 0 is never issued by ProcessEnvelope; a negative capture was never stamped.
    return a.attempted && a.seq != 0 && a.capture_ms >= 0 &&
           // latency = put_before - capture must be representable and non-negative
           a.put_before_ms >= a.capture_ms &&
           a.put_before_ms <= ProcessEnvelope::kMaxMonoMs;
}
// The Completion must describe the same put as the Attempt (identity fields only;
// the Completion adds the return time and outcome).
bool same_attempt(const PublishAudit& a, const PublishAudit& b) noexcept {
    // seq alone would do in a correct publisher; the other fields catch a caller
    // that reused or rebuilt the audit between the two records.
    return a.attempted == b.attempted && a.seq == b.seq &&
           a.capture_ms == b.capture_ms && a.put_before_ms == b.put_before_ms &&
           a.packet_bytes == b.packet_bytes;
}
// Slot copies on both sides must be plain memory copies.
static_assert(std::is_trivially_copyable<PublicationJournal::Event>::value,
              "publication event must be a fixed, allocation-free copy");
}
// An invalid configuration does not throw: the journal is empty and valid() is false,
// which PublisherChannel checks when the journal is attached.
PublicationJournal::PublicationJournal(Topic topic, std::size_t capacity)
    : topic_(topic), configured_(configuration(topic, capacity)),
      capacity_(configured_ ? static_cast<std::uint64_t>(capacity) : 0),
      // The only allocation, and only for a valid configuration.
      events_(configured_ ? capacity : 0) {}

// Acquire pairs with the release store in fault(): a reader that sees the fault
// also sees the counters written before it.
bool PublicationJournal::valid() const noexcept {
    return configured_ && fault_.load(std::memory_order_acquire) == 0;
}
void PublicationJournal::increment(std::atomic<std::uint64_t>& counter) noexcept {
    // These counters have exactly one writer, the producer. No CAS retry loop
    // or fetch_add wraparound is necessary; readers only take atomic snapshots.
    const auto old = counter.load(std::memory_order_relaxed);
    // Saturated: keep the value and latch the fault instead of wrapping to 0.
    if (old == max_position) fault_.store(1, std::memory_order_release);
    else counter.store(old + 1, std::memory_order_relaxed);
}
// Sticky: there is no reset. Committed events stay drainable for the consumer.
void PublicationJournal::fault() noexcept {
    increment(protocol_faults_);
    fault_.store(1, std::memory_order_release);
}
// Every producer entry point starts here: a faulted journal refuses silently, a call
// in the wrong phase becomes a fault.
bool PublicationJournal::in_phase(Phase expected) noexcept {
    if (!valid()) return false;
    // phase_ is producer-only, so relaxed is enough; it is atomic for tests/readers.
    if (phase_.load(std::memory_order_relaxed) == static_cast<std::uint32_t>(expected)) return true;
    fault();
    return false;
}
// Called by PublisherChannel after it built the packet and before the put; false
// refuses the put (TelemetryUnavailable), so no message is sent that Status could
// not account for, and no older event is ever overwritten to make room.
bool PublicationJournal::reserve() noexcept {
    // A reserve while another put is in progress is a protocol fault.
    if (!in_phase(Phase::Idle)) return false;
    // head is ours (relaxed); tail comes from the consumer (acquire).
    const auto head = head_.load(std::memory_order_relaxed);
    const auto tail = tail_.load(std::memory_order_acquire);
    const auto g = generation_.load(std::memory_order_seq_cst);
    // Reserve all arithmetic headroom up front: TWO commits, FOUR generation
    // transitions. The last usable generation is the greatest even uint64.
    // Odd generation here means a previous put never finished its timing; tail
    // beyond head or more than capacity in flight means corrupted positions.
    if ((g & 1U) || g > max_position - 5 || head > max_position - 2 ||
        tail > head || head - tail > capacity_) {
        fault();
        return false;
    }
    // Not enough room for Attempt + Completion: refuse the put, not a fault.
    if (capacity_ - (head - tail) < 2) {
        increment(admission_rejected_);
        return false;
    }
    // No generation change yet: reserving takes no clock sample.
    phase_.store(static_cast<std::uint32_t>(Phase::Reserved), std::memory_order_relaxed);
    return true;
}
// Called by the publisher's finalizer immediately before it reads the clocks.
void PublicationJournal::begin_attempt() noexcept {
    if (!in_phase(Phase::Reserved)) return;
    // seq_cst RMW includes acquire ordering: the subsequent clock read may
    // not move ahead of the odd marker. A release-only store is insufficient.
    generation_.fetch_add(1, std::memory_order_seq_cst);
    phase_.store(static_cast<std::uint32_t>(Phase::AttemptTiming), std::memory_order_relaxed);
}
// Shared by Attempt and Completion; declarations build their own event below.
void PublicationJournal::commit(Kind kind, const PublishAudit& audit) noexcept {
    // Room was reserved in reserve(); head is producer-owned.
    const auto head = head_.load(std::memory_order_relaxed);
    // Write the slot completely, then publish it: release makes the slot contents
    // visible to a consumer that acquires the new head.
    events_[static_cast<std::size_t>(head % capacity_)] = {kind, topic_, audit};
    head_.store(head + 1, std::memory_order_release);
}
// Phase first, then the even generation (seq_cst store): a consumer that sees the
// even value also sees the committed head published before it.
void PublicationJournal::finish_timing(Phase next) noexcept {
    phase_.store(static_cast<std::uint32_t>(next), std::memory_order_relaxed);
    // Plain load + store is enough: only the producer ever writes generation_.
    generation_.store(generation_.load(std::memory_order_relaxed) + 1, std::memory_order_seq_cst);
}
// Called after the envelope was stamped, before the native put: the Attempt is
// visible to Status while the put itself is still running.
void PublicationJournal::record_attempt(const PublishAudit& audit) noexcept {
    if (!in_phase(Phase::AttemptTiming)) return;
    // A malformed Attempt is a publisher bug; the odd window then stays open, the
    // Status handoff sees valid() false, reports the cut invalid and the service stops.
    if (!attempt_identity(audit)) { fault(); return; }
    // Keep a copy to match the Completion against.
    attempt_ = audit;
    commit(Kind::Attempt, audit);
    finish_timing(Phase::AttemptCommitted);
}
// Called after the native put returned and before its return time is sampled.
void PublicationJournal::begin_completion() noexcept {
    if (!in_phase(Phase::AttemptCommitted)) return;
    // Same odd-before-clock rule as begin_attempt(), for the return time.
    generation_.fetch_add(1, std::memory_order_seq_cst);
    phase_.store(static_cast<std::uint32_t>(Phase::CompletionTiming), std::memory_order_relaxed);
}
// Rejected, Unknown, thrown and clock-invalid puts are recorded exactly as they are.
void PublicationJournal::record_completion(const PublishAudit& audit) noexcept {
    if (!in_phase(Phase::CompletionTiming)) return;
    if (!same_attempt(audit, attempt_)) { fault(); return; }
    // No normalisation, substituted return timestamp or guessed API receipt.
    commit(Kind::Completion, audit);
    finish_timing(Phase::Idle);
}
// Objects owner declares Reconnecting / Reconfiguring / None between puts; the time
// it then samples is the declared start or end of the excluded interval.
bool PublicationJournal::begin_declaration() noexcept {
    if (!in_phase(Phase::Idle)) return false;
    // Only the Objects cadence has declarations (11 section 3.1B.3).
    if (topic_ != Topic::Objects) { fault(); return false; }
    const auto head = head_.load(std::memory_order_relaxed);
    const auto tail = tail_.load(std::memory_order_acquire);
    const auto g = generation_.load(std::memory_order_seq_cst);
    // Headroom for one commit and two generation steps; positions consistent.
    if ((g & 1U) || g > max_position - 3 || head == max_position ||
        tail > head || head - tail > capacity_) {
        fault(); return false;
    }
    // One slot suffices for a declaration; full is retryable, not a fault.
    if (head - tail == capacity_) {
        increment(admission_rejected_); return false;
    }
    // Odd before the owner samples its clock, exactly as for an Attempt; the
    // Objects owner then reads the clock and calls record_declaration() or cancel().
    generation_.fetch_add(1, std::memory_order_seq_cst);
    phase_.store(static_cast<std::uint32_t>(Phase::DeclarationTiming), std::memory_order_relaxed);
    return true;
}
// Returns false on any fault; PublisherChannel then reports TelemetryUnavailable to
// the Objects owner, and the latched fault stops the Status cuts.
bool PublicationJournal::record_declaration(Exclusion declaration, Millis declared_ms) noexcept {
    if (!in_phase(Phase::DeclarationTiming)) return false;
    // Only the three defined values, at a representable time.
    if ((declaration != Exclusion::None && declaration != Exclusion::Reconnecting &&
         declaration != Exclusion::Reconfiguring) || declared_ms < 0 ||
        declared_ms > ProcessEnvelope::kMaxMonoMs) {
        fault();
        // No event was committed. Restore the time marker without making the
        // sticky protocol fault recoverable.
        finish_timing(Phase::Idle);
        return false;
    }
    // Declarations carry no audit; only kind, topic, value and time.
    Event event;
    event.kind = Kind::Declaration; event.topic = topic_;
    event.declaration = declaration; event.declared_ms = declared_ms;
    // Same write-then-publish order as commit().
    const auto head = head_.load(std::memory_order_relaxed);
    events_[static_cast<std::size_t>(head % capacity_)] = event;
    head_.store(head + 1, std::memory_order_release);
    finish_timing(Phase::Idle);
    return true;
}
// Used by the publisher when a put fails before its Attempt is committed
// (prepare rejected, clock or stamp failure), and by the declaration path.
void PublicationJournal::cancel() noexcept {
    if (!valid()) return;
    const auto p = static_cast<Phase>(phase_.load(std::memory_order_relaxed));
    // Reserved: nothing timed yet, just release. Timing: close the odd window.
    // Any later phase: an Attempt is committed and must get its Completion.
    if (p == Phase::Reserved)
        phase_.store(static_cast<std::uint32_t>(Phase::Idle), std::memory_order_relaxed);
    else if (p == Phase::AttemptTiming || p == Phase::DeclarationTiming) finish_timing(Phase::Idle);
    else fault();
}
// Consumer side, same pattern as SourceJournal: acquire head, copy, release tail.
bool PublicationJournal::peek(Event& out) const noexcept {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    out = events_[static_cast<std::size_t>(tail % capacity_)];
    return true;
}
// Copy first, then release the slot to the producer.
bool PublicationJournal::pop(Event& out) noexcept {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    out = events_[static_cast<std::size_t>(tail % capacity_)];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
}
// seq_cst: ordered against the producer's odd/even steps and the consumer's clock.
std::uint64_t PublicationJournal::generation() const noexcept {
    return generation_.load(std::memory_order_seq_cst);
}
// published() - consumed() = events waiting for the Status thread.
std::uint64_t PublicationJournal::published() const noexcept {
    return head_.load(std::memory_order_acquire);
}
std::uint64_t PublicationJournal::consumed() const noexcept {
    return tail_.load(std::memory_order_acquire);
}
// Diagnostic snapshot for logs and tests.
PublicationJournal::SnapshotCounters PublicationJournal::counters() const noexcept {
    return {admission_rejected_.load(std::memory_order_relaxed),
            protocol_faults_.load(std::memory_order_relaxed)};
}
} // namespace perception::rns
