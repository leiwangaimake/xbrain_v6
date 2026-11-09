/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_health_window.cpp
 * Brief: Four fixed event rings, strict ordering checks, exact nearest-rank percentiles.
 *
 * Description:
 * Every observe*() call does the same three things: check the ingest clock, check the event
 * against its stream's watermarks (identity, order, window), append it to the ring. snapshot()
 * prunes each ring to the last 10 s and computes the report from what is left.
 *
 * Why so strict. A rate or percentile computed over events that were duplicated, reordered or
 * came from a mixed clock would look plausible and be wrong. Every such event is refused; the
 * serious ones latch the window invalid, and the Status side then refuses to publish the
 * fields (rns_health_fields.cpp) instead of publishing a wrong number.
 *
 * Percentiles. Nearest rank, rank = ceil(p * n / 100) computed in integers, over the exact
 * samples in the window; no interpolation, no histogram. The interval percentile and maximum
 * include the open interval (now - last put) as one extra sample, so a stalled publisher
 * shows up at once (19 section 5.1 v1.7).
 *
 * Tests. tests/test_rns_health_window.cpp, tests/test_rns_health_pending.cpp and
 * tests/test_rns_health_source_identity.cpp.
 */
#include "output/rns_health_window.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace perception::rns {
namespace {
// Configuration errors throw at construction (startup), never at run time.
std::size_t checked_capacity(Millis startup, std::size_t capacity) {
    if (startup < 0 || startup > ProcessEnvelope::kMaxMonoMs || capacity == 0 ||
        capacity > HealthWindow::kMaxCapacityPerStream)
        throw std::invalid_argument("invalid health window configuration");
    return capacity;
}
// The three terminal outcomes a native put can report.
bool known(PutOutcome outcome) noexcept {
    return outcome == PutOutcome::Accepted || outcome == PutOutcome::Rejected || outcome == PutOutcome::Unknown;
}
// Status has no per-put health entry of its own (it is the report).
bool publication_topic(Topic topic) noexcept { return topic == Topic::Profile || topic == Topic::Objects; }
// Every field of a resolution: an exact replay is harmless, a different one is not.
bool same_resolution(const PublishAudit& a, const PublishAudit& b) noexcept {
    return a.seq == b.seq && a.capture_ms == b.capture_ms && a.put_before_ms == b.put_before_ms &&
        a.put_return_ms == b.put_return_ms && a.attempted == b.attempted &&
        a.clock_valid == b.clock_valid && a.threw == b.threw && a.packet_bytes == b.packet_bytes &&
        a.receipt.outcome == b.receipt.outcome && a.receipt.native_code == b.receipt.native_code;
}
// Full source identity, as in the other health modules.
bool same_source(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
}  // namespace
// All four rings and the scratch buffer are allocated here, once.
HealthWindow::HealthWindow(Millis startup, std::size_t capacity)
    : streams_{{Stream(checked_capacity(startup, capacity)), Stream(capacity), Stream(capacity), Stream(capacity)}},
      scratch_(capacity + 1), startup_(startup), last_ingest_(startup) {}
// The ingest/snapshot clock must be representable and never go backwards.
bool HealthWindow::clock(Millis now) noexcept {
    if (now < 0 || now > ProcessEnvelope::kMaxMonoMs || now < last_ingest_) return false;
    // Equal times are fine: several events can be drained in one millisecond.
    last_ingest_ = now;
    return true;
}
// Window (now-10000, now]: drop events whose own time is 10 s or more before now.
void HealthWindow::prune(Stream& stream, Millis now) noexcept {
    // Events are appended in event-time order, so the oldest is always at begin.
    while (stream.size && now - stream.events[stream.begin].time >= kWindowMs) {
        stream.begin = (stream.begin + 1) % stream.events.size(); --stream.size;
    }
}
// Count the rejection; everything except a duplicate or a late event latches invalid.
HealthEventCode HealthWindow::reject(HealthEventCode code) noexcept {
    std::uint64_t* count = nullptr;
    switch (code) {
        // Replays and late deliveries are handoff noise: counted, window stays valid.
        case HealthEventCode::Duplicate: count = &diagnostics_.duplicates; break;
        case HealthEventCode::OutOfOrder: count = &diagnostics_.out_of_order; break;
        // A full ring or anything invalid means the window can no longer be complete.
        case HealthEventCode::CapacityExceeded: count = &diagnostics_.capacity_exceeded; valid_ = false; break;
        default: count = &diagnostics_.invalid; valid_ = false; break;
    }
    // Saturate instead of wrapping to zero.
    if (*count == std::numeric_limits<std::uint64_t>::max()) valid_ = false;
    else ++*count;
    return code;
}
// Terminal outcomes; anything that is not a clear accept or reject is Unknown.
void HealthWindow::count_outcome(HealthPutCounts& counts, PutOutcome outcome) noexcept {
    switch (outcome) {
        case PutOutcome::Accepted: ++counts.accepted; break;
        case PutOutcome::Rejected: ++counts.rejected; break;
        default: ++counts.unknown; break;
    }
}
// A put without an outcome yet is counted as pending, not as unknown.
void HealthWindow::count_put(HealthPutCounts& counts, std::optional<PutOutcome> outcome) noexcept {
    ++counts.attempted;
    if (outcome) count_outcome(counts, *outcome);
    else ++counts.pending;
}
// The append path of a publication attempt (observe_publication_attempt): time, identity,
// order, window and capacity checks, then one ring entry.
HealthEventCode HealthWindow::observe(std::size_t index, Millis capture, Millis event,
        Millis ingest, Millis latency, std::optional<PutOutcome> outcome, std::uint64_t seq) noexcept {
    auto& stream = streams_[index];
    // Prune first, so a full ring only means "full within 10 s".
    prune(stream, ingest);
    // Times in order: capture <= event <= ingest, event not before startup.
    if (capture < 0 || event < startup_ || event > ingest || capture > event)
        return reject(HealthEventCode::Invalid);
    // Identity by capture time for this path: equal = replay, smaller = late.
    if (capture == stream.last_capture) return reject(HealthEventCode::Duplicate);
    if (capture < stream.last_capture) return reject(HealthEventCode::OutOfOrder);
    // A fresh capture with an older event time, or already outside the window on
    // arrival, is a broken handoff.
    if (event < stream.last_event || ingest - event >= kWindowMs)
        return reject(HealthEventCode::Invalid);
    // A full ring within 10 s means the capacity was misconfigured: refuse, never drop.
    if (stream.size == stream.events.size()) return reject(HealthEventCode::CapacityExceeded);
    // Publication streams also store the interval to the previous put (Status gaps).
    const Millis gap = index >= 2 && stream.last_event >= 0 ? event - stream.last_event : -1;
    // Append at the tail and move the stream's watermarks.
    stream.events[(stream.begin + stream.size) % stream.events.size()] = {capture, event, latency, outcome, gap, seq};
    ++stream.size; ++stream.total_events;
    stream.last_capture = capture; stream.last_event = event;
    // Lifetime put counts (publication streams only).
    if (index >= 2) count_put(stream.total_puts, outcome);
    // A strictly increasing nonnegative Millis capture bounds lifetime accepted
    // events by kMaxMonoMs+1, safely below uint64 exhaustion; no counter wrap.
    return HealthEventCode::Accepted;
}
// SourceId rate interface: stream 0 = depth arrivals, stream 1 = inference completions
// (2 and 3 are the Profile/Objects publication streams).
HealthEventCode HealthWindow::observe_depth(const native_depth::SourceId& source,
        Millis event, Millis ingest) noexcept {
    return observe_native_rate(0, source, event, ingest);
}
HealthEventCode HealthWindow::observe_inference(const native_depth::SourceId& source,
        Millis event, Millis ingest) noexcept {
    return observe_native_rate(1, source, event, ingest);
}
HealthEventCode HealthWindow::observe_native_rate(std::size_t index,
        const native_depth::SourceId& source, Millis event, Millis ingest) noexcept {
    if (!clock(ingest)) return reject(HealthEventCode::Invalid);
    auto& stream = streams_[index];
    // Prune at ingest time first, as in observe().
    prune(stream, ingest);
    // A real frame: stream, sequence, depth device time, host ready time; inference
    // results also need the colour frame (arrival may come before RGB).
    if (!source.stream_id || !source.sequence || !source.depth_device_us ||
        source.host_ready_mono_ns <= 0 || (index == 1 && !source.color_device_us))
        return reject(HealthEventCode::Invalid);
    // The host ready time is the frame's time in this window (ms, floored).
    const Millis capture = source.host_ready_mono_ns / 1000000;
    // Frame ready <= event <= ingest, and nothing from before process start.
    if (event < startup_ || event > ingest || capture > event)
        return reject(HealthEventCode::Invalid);
    // Ordering against the last accepted frame of this stream.
    if (stream.last_source) {
        const auto& last = *stream.last_source;
        if (source.stream_id == last.stream_id) {
            // Identity replay precedes event order checks: re-delivering an
            // old event is not proof of a newly observed clock rollback.
            if (source.sequence == last.sequence)
                return reject(same_source(source, last) ? HealthEventCode::Duplicate
                                                       : HealthEventCode::Invalid);
            // An older sequence is a late delivery: skipped and counted.
            if (source.sequence < last.sequence) return reject(HealthEventCode::OutOfOrder);
            // A newer sequence must also have a newer SDK depth index and a device
            // time that did not go back; otherwise the identity is inconsistent.
            if (source.depth_index <= last.depth_index || source.depth_device_us < last.depth_device_us)
                return reject(HealthEventCode::Invalid);
        }
        // Across streams too, host time never goes backwards.
        if (source.host_ready_mono_ns < last.host_ready_mono_ns)
            return reject(HealthEventCode::Invalid);
    }
    // Event time in order and still inside the window on arrival.
    if (event < stream.last_event || ingest - event >= kWindowMs)
        return reject(HealthEventCode::Invalid);
    // A full ring within 10 s means the capacity was misconfigured: refuse, never drop.
    if (stream.size == stream.events.size()) return reject(HealthEventCode::CapacityExceeded);
    // Unlike the strictly increasing Millis capture of publication events (observe()),
    // native identities can be distinct even at one nanosecond; explicitly prevent
    // lifetime overflow.
    if (stream.total_events == std::numeric_limits<std::uint64_t>::max())
        return reject(HealthEventCode::Invalid);
    // Rate samples carry no latency, outcome or interval; seq holds the sequence.
    stream.events[(stream.begin + stream.size) % stream.events.size()] =
        {capture, event, 0, PutOutcome::Unknown, -1, source.sequence};
    ++stream.size; ++stream.total_events;
    stream.last_capture = capture; stream.last_event = event;
    // The next frame of this stream is ordered against this identity.
    stream.last_source = source;
    return HealthEventCode::Accepted;
}
// Split form, first half: the put has been stamped and handed to the native call.
// It counts at once (latency, interval, attempted); its outcome is pending.
HealthEventCode HealthWindow::observe_publication_attempt(Topic topic, std::uint64_t seq,
        Millis capture, Millis before, Millis ingest) noexcept {
    if (!clock(ingest)) return reject(HealthEventCode::Invalid);
    // A publication topic, an issued seq, capture not after the put.
    if (!publication_topic(topic) || !seq || capture < 0 || capture > before)
        return reject(HealthEventCode::Invalid);
    auto& stream = streams_[topic == Topic::Profile ? 2 : 3];
    // Publishing is synchronous per topic: a second attempt before the first
    // resolved is a replay (identical) or a protocol fault (different).
    if (stream.pending) {
        if (stream.pending->seq == seq && stream.pending->capture == capture &&
            stream.pending->put_before == before) return reject(HealthEventCode::Duplicate);
        return reject(HealthEventCode::Invalid);
    }
    // Envelope seq is strictly increasing per process; per topic it only grows.
    if (seq <= stream.last_attempt_seq) return reject(HealthEventCode::Invalid);
    // Counted now with outcome "pending"; the interval uses put-before, like cadence.
    const auto code = observe(topic == Topic::Profile ? 2 : 3, capture, before,
                              ingest, before-capture, std::nullopt, seq);
    // Only an accepted attempt becomes the pending one.
    if (code == HealthEventCode::Accepted) {
        stream.pending = PendingPublication{seq,capture,before};
        stream.last_attempt_seq = seq;
    }
    return code;
}
// Split form, second half: the native call returned. Settles the pending put's
// outcome without adding a sample (the attempt already counted).
HealthEventCode HealthWindow::resolve_publication(Topic topic, const PublishAudit& audit, Millis ingest) noexcept {
    if (!clock(ingest)) return reject(HealthEventCode::Invalid);
    // Coherent audit; a return time is checked only when its clock was valid.
    // A publication topic, an issued seq, a put that was really attempted and a known
    // outcome; capture <= put-before, put-before within [startup, ingest]; a thrown
    // call can only be Unknown.
    if (!publication_topic(topic) || !audit.seq || !audit.attempted ||
        !known(audit.receipt.outcome) || audit.capture_ms < 0 ||
        audit.capture_ms > audit.put_before_ms || audit.put_before_ms < startup_ ||
        audit.put_before_ms > ingest || (audit.threw && audit.receipt.outcome != PutOutcome::Unknown) ||
        (audit.clock_valid && (audit.put_return_ms < audit.put_before_ms || audit.put_return_ms > ingest)))
        return reject(HealthEventCode::Invalid);
    auto& stream = streams_[topic == Topic::Profile ? 2 : 3];
    // Replay of the last resolution: harmless if identical, a fault if it differs.
    if (stream.last_resolved && stream.last_resolved->seq == audit.seq)
        return reject(same_resolution(*stream.last_resolved, audit) ? HealthEventCode::Duplicate
                                                               : HealthEventCode::Invalid);
    // Must match the pending attempt exactly (seq, capture, put-before).
    if (!stream.pending || stream.pending->seq != audit.seq ||
        stream.pending->capture != audit.capture_ms || stream.pending->put_before != audit.put_before_ms ||
        // exactly one pending put exists in the lifetime counts
        stream.total_puts.pending != 1) return reject(HealthEventCode::Invalid);
    prune(stream, ingest);
    // Find the ring entry (if still inside the window) and set its outcome.
    // At most one synchronous in-flight call; its association lives outside
    // the sliding ring, so a return after 10s still settles the lifetime count.
    for (std::size_t i=0; i<stream.size; ++i) {
        auto& event = stream.events[(stream.begin+i)%stream.events.size()];
        if (event.seq == audit.seq) {
            // Only the outcome changes; latency and interval were final at the attempt.
            event.outcome = audit.receipt.outcome;
            break;
        }
    }
    // Lifetime counts move from pending to the real outcome.
    --stream.total_puts.pending;
    count_outcome(stream.total_puts, audit.receipt.outcome);
    // Remember this resolution so an exact replay is recognised as a duplicate.
    stream.pending.reset();
    stream.last_resolved = audit;
    // The outcome is real even if the return clock was not; count it, then latch
    // invalid because the return time cannot be trusted.
    if (!audit.clock_valid) {
        valid_ = false;
        // Saturating count; the invalid latch is already set above.
        if (diagnostics_.clock_faults != std::numeric_limits<std::uint64_t>::max())
            ++diagnostics_.clock_faults;
    }
    return HealthEventCode::Accepted;
}
// fps = frames in the window / 10 s, also during the first 10 s (the rate ramps up
// instead of being extrapolated from a partial window).
HealthRateReport HealthWindow::rate(const Stream& stream) const noexcept {
    // ever_observed, frames in window, lifetime frames, fps over the fixed 10 s.
    return {stream.total_events != 0, stream.size, stream.total_events,
            static_cast<double>(stream.size) / 10.0};
}
HealthPublicationReport HealthWindow::publication(const Stream& stream, bool include_window, Millis now) noexcept {
    HealthPublicationReport report;
    report.ever_observed = stream.total_events != 0;
    report.total = stream.total_puts;
    // Bad snapshot clock: lifetime counts only, no window statistics.
    if (!include_window) return report;
    // Window counts and latencies in one pass over the ring.
    for (std::size_t i = 0; i < stream.size; ++i) {
        const auto& event = stream.events[(stream.begin + i) % stream.events.size()];
        // A pending put counts as attempted/pending; its latency is already known.
        scratch_[i] = event.latency; count_put(report.window, event.outcome);
    }
    // Bounded sort of at most the ring size; no allocation (scratch_ is preallocated).
    std::sort(scratch_.begin(), scratch_.begin() + stream.size);
    // ceil(p*n/100), exact integer nearest rank. n <= 1,000,000; products fit size_t.
    // No put in the window: both latency percentiles stay null (19 section 5.1 v1.7).
    if (stream.size) {
        report.latency_p50_ms = scratch_[(50 * stream.size + 99) / 100 - 1];
        report.latency_p99_ms = scratch_[(99 * stream.size + 99) / 100 - 1];
    }
    // Interval statistics exist once any put was ever made; afterwards the open
    // interval keeps growing even when the window holds no closed interval.
    if (stream.last_event >= 0) {
        std::size_t n = 0;
        for (std::size_t i = 0; i < stream.size; ++i) {
            const auto gap = stream.events[(stream.begin + i) % stream.events.size()].closed_gap;
            // -1 marks the first put of the stream: it closes no interval.
            if (gap >= 0) scratch_[n++] = gap;
        }
        report.closed_gap_samples = n;
        // The open interval is one sample of this snapshot, never stored.
        report.open_gap_ms = now - stream.last_event;
        scratch_[n++] = *report.open_gap_ms;
        std::sort(scratch_.begin(), scratch_.begin() + n);
        // n >= 1 here (the open interval), so the rank is valid.
        report.gap_p99_ms = scratch_[(99 * n + 99) / 100 - 1];
        // Sorted, so the last element is the maximum.
        report.gap_max_ms = scratch_[n - 1];
    }
    return report;
}
HealthWindowReport HealthWindow::snapshot(Millis now) noexcept {
    // now_ms lets the Status side check every snapshot was cut at the same instant.
    HealthWindowReport report;
    report.now_ms = now;
    // A bad snapshot time latches invalid; otherwise bring every ring to now.
    const bool clock_ok = clock(now);
    if (!clock_ok) (void)reject(HealthEventCode::Invalid);
    else for (auto& stream : streams_) prune(stream, now);
    // valid is the sticky latch; full_window says the 10 s window is fully populated.
    report.valid = valid_;
    report.full_window = clock_ok && now - startup_ >= kWindowMs;
    // fps_depth from arrivals, fps_infer from inference completions.
    report.depth = rate(streams_[0]); report.inference = rate(streams_[1]);
    // With a bad clock the window is unknown: report zero window frames, keep totals.
    if (!clock_ok) {
        report.depth.window_frames = report.inference.window_frames = 0;
        report.depth.fps = report.inference.fps = 0;
    }
    // Status uses the Objects report (the slow line, 19 section 5.1); Profile is logged.
    report.profile = publication(streams_[2], clock_ok, now);
    report.objects = publication(streams_[3], clock_ok, now);
    // Lifetime diagnostics are reported whatever the clock state.
    report.diagnostics = diagnostics_;
    return report;
}
}  // namespace perception::rns
