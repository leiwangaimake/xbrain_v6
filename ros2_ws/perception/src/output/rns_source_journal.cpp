/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_source_journal.cpp
 * Brief: Lock-free SPSC ring; generation made odd around the clock sample and the publish.
 *
 * Description:
 * record() order on the producer: check not faulted and generation even; generation to odd
 * (seq_cst, so the clock read below cannot move before it); sample the clock; validate the
 * source and ordering against the last accepted one; write the slot; publish head (release);
 * generation back to even. The consumer (PublicationHealthHandoff::drain) reads generation,
 * samples its own clock and every head, and re-reads generation; any odd or changed value
 * means "busy, try again", so a cut never excludes an event that is still in flight.
 *
 * Why faults are sticky. A journal that silently dropped a sample would make its window look
 * complete when it is not. Every overflow, invalid source or misuse latches valid() false; the
 * Status side then refuses the whole cut, and the service stops on an invalid cut.
 *
 * Memory order. head_ is released after the slot write and acquired by the consumer before the
 * slot read, which is what makes the slot contents visible. Counters are single-writer and only
 * need atomic observation. SourceEvent is trivially copyable, so a slot copy never allocates.
 *
 * Tests. tests/test_rns_source_journal.cpp and tests/test_rns_source_handoff.cpp.
 */
#include "output/rns_source_journal.hpp"
#include <limits>
#include <type_traits>

namespace perception::rns {
namespace {
constexpr auto max_position = std::numeric_limits<std::uint64_t>::max();
// Known kind, non-empty ring, and a clock. An invalid configuration does not throw:
// the journal is built empty and valid() stays false, which every user checks.
bool configuration(SourceKind kind, std::size_t capacity, SourceClock clock) noexcept {
    return (kind == SourceKind::DepthArrival || kind == SourceKind::InferenceComplete ||
            kind == SourceKind::DepthProcessed) && capacity > 0 &&
           capacity <= max_position && clock.now_ns;
}
// Slot copies must be plain memory copies: no allocation, no exceptions.
static_assert(std::is_trivially_copyable<SourceEvent>::value,
              "source event must be an allocation-free value copy");
}
SourceJournal::SourceJournal(SourceKind kind, std::size_t capacity, SourceClock clock)
    : kind_(kind), clock_(clock), configured_(configuration(kind, capacity, clock)),
      capacity_(configured_ ? static_cast<std::uint64_t>(capacity) : 0),
      // The only allocation, and only for a valid configuration.
      events_(configured_ ? capacity : 0) {}
// Acquire pairs with the release store of invalid_, so a consumer that sees the
// fault also sees the counters written before it.
bool SourceJournal::valid() const noexcept {
    return configured_ && invalid_.load(std::memory_order_acquire) == 0;
}
void SourceJournal::increment(std::atomic<std::uint64_t>& value) noexcept {
    // Exactly one producer writes counters; readers take atomic observations.
    const auto old = value.load(std::memory_order_relaxed);
    // Saturated: keep the value and latch invalid instead of wrapping to 0.
    if (old == max_position) invalid_.store(1, std::memory_order_release);
    else value.store(old + 1, std::memory_order_relaxed);
}
// Latch invalid; faults_ counts how many calls hit a fault.
void SourceJournal::fail() noexcept {
    increment(faults_);
    invalid_.store(1, std::memory_order_release);
}
bool SourceJournal::source_valid(const native_depth::SourceId& s, std::int64_t observed) const noexcept {
    // A real frame: stream, sequence, depth device time, a host ready time not after
    // the record time; color time too, except for arrivals (color may come later).
    if (!s.stream_id || !s.sequence || !s.depth_device_us || s.host_ready_mono_ns <= 0 ||
        observed < s.host_ready_mono_ns ||
        (kind_ != SourceKind::DepthArrival && !s.color_device_us)) return false;
    // First event: nothing to order against.
    if (!ever_) return true;
    // Times may tie but never regress, across streams too.
    if (observed < last_observed_ns_ || s.host_ready_mono_ns < last_source_.host_ready_mono_ns)
        return false;
    // New stream id: accepted as is (ids are not ordered; lifecycle owns that).
    // Same stream: sequence and depth index strictly up, device time not down.
    return s.stream_id != last_source_.stream_id ||
        (s.sequence > last_source_.sequence && s.depth_index > last_source_.depth_index &&
         s.depth_device_us >= last_source_.depth_device_us);
}
bool SourceJournal::write(const native_depth::SourceId& source,
                          const DepthHealthObservation* depth, bool correct_overload) noexcept {
    // A faulted journal accepts nothing more; committed events stay drainable.
    if (!valid()) return false;
    const auto g = generation_.load(std::memory_order_seq_cst);
    // Odd = another record() is in progress (reentrancy or a second producer);
    // near max_position the two increments below could wrap.
    if ((g & 1U) || g > max_position - 3) {
        // Reentrant use cannot take ownership of an outer record's odd marker.
        // The outer call restores it on return; multiple producers unsupported.
        fail();
        return false;
    }
    generation_.fetch_add(1, std::memory_order_seq_cst); // acquire blocks later clock load reordering
    // The record time. Sampled inside the odd window, so no consumer can cut at a
    // time after this sample while the event is still unpublished.
    const auto observed = clock_.now_ns(clock_.context);
    bool accepted = false;
    if (valid()) { // The supplied callback may have triggered a reentrancy fault.
        // head is producer-owned (relaxed); tail comes from the consumer (acquire).
        const auto head = head_.load(std::memory_order_relaxed);
        const auto tail = tail_.load(std::memory_order_acquire);
        // ROI ratio/kind provenance belongs to the downstream evidence checker.
        // Preserve the actual value (including a bad/missing ratio), never
        // recalculate a nicer observation while transferring it.
        const bool depth_ok = !depth ||
            (depth->code == DepthHealthCode::Ready && depth->roi_pixels > 0 &&
             depth->invalid_pixels <= depth->roi_pixels);
        // Wrong overload for this kind, bad source or ordering, malformed health
        // observation, or impossible ring positions: a fault.
        if (!correct_overload || !source_valid(source, observed) || !depth_ok ||
            head == max_position || tail > head || head - tail > capacity_) {
            fail();
        } else if (head - tail == capacity_) {
            // Full: the consumer fell behind by a whole ring. Latch rather than
            // overwrite, so the window can never be reported as complete.
            increment(overflow_);
            invalid_.store(1, std::memory_order_release);
        } else {
            // Build the event, write the slot, then publish it by moving head.
            SourceEvent event;
            event.kind = kind_; event.source = source; event.observed_ns = observed;
            if (depth) event.depth = *depth;
            events_[static_cast<std::size_t>(head % capacity_)] = event;
            head_.store(head + 1, std::memory_order_release);
            // Ordering watermarks advance only for accepted events.
            last_source_ = source; last_observed_ns_ = observed; ever_ = true;
            accepted = true;
        }
    }
    // Back to even on every path that made it odd, accepted or not.
    generation_.fetch_add(1, std::memory_order_seq_cst);
    return accepted;
}
// The bool tells write() whether this overload matches the journal's kind.
bool SourceJournal::record(const native_depth::SourceId& source) noexcept {
    return write(source, nullptr, kind_ != SourceKind::DepthProcessed);
}
bool SourceJournal::record(const DepthHealthObservation& depth) noexcept {
    return write(depth.source, &depth, kind_ == SourceKind::DepthProcessed);
}
// seq_cst to order against the producer's increments and the consumer's clock read.
std::uint64_t SourceJournal::generation() const noexcept {
    return generation_.load(std::memory_order_seq_cst);
}
// Acquire loads: a position seen here comes with the slot contents written before it.
std::uint64_t SourceJournal::published() const noexcept { return head_.load(std::memory_order_acquire); }
// published() - consumed() = events waiting for the Status thread.
std::uint64_t SourceJournal::consumed() const noexcept { return tail_.load(std::memory_order_acquire); }
// Consumer only. tail is consumer-owned (relaxed); head acquired so the slot is visible.
bool SourceJournal::peek(SourceEvent& out) const noexcept {
    const auto tail = tail_.load(std::memory_order_relaxed);
    // Empty ring: nothing to read, out untouched.
    if (tail == head_.load(std::memory_order_acquire)) return false;
    out = events_[static_cast<std::size_t>(tail % capacity_)];
    // peek does not move tail: the handoff pops only after the event was accepted.
    return true;
}
bool SourceJournal::pop(SourceEvent& out) noexcept {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    out = events_[static_cast<std::size_t>(tail % capacity_)];
    // Release: the producer may reuse the slot only after our copy is complete.
    tail_.store(tail + 1, std::memory_order_release);
    return true;
}
// Relaxed: diagnostic counters, read by the tests today; the service decides on
// valid() only.
std::uint64_t SourceJournal::overflow_count() const noexcept {
    return overflow_.load(std::memory_order_relaxed);
}
std::uint64_t SourceJournal::fault_count() const noexcept {
    return faults_.load(std::memory_order_relaxed);
}
} // namespace perception::rns
