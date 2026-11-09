/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_health_window.hpp
 * Brief: Bounded ten-second health event statistics, no metric estimation.
 *
 * Description:
 * What problem this solves. Status (19 section 5.1) reports, over the last 10 s: fps_depth
 * (depth frames arrived), fps_infer (inference results completed), latency_ms_p50/p99 (Objects
 * capture to put-before) and infer_gap_ms_p99/max (intervals between Objects puts, including
 * the still-open one). HealthWindow keeps the raw events for exactly that window, in four fixed
 * rings: 0 depth arrivals, 1 inference completions, 2 Profile puts, 3 Objects puts; every
 * number is computed from those events at snapshot time, never estimated or smoothed.
 *
 * How it is fed. Only the Status thread touches it, through PublicationHealthHandoff::drain(),
 * which moves certified journal events in and then takes a snapshot at the same cut.
 */
#pragma once
#include "output/rns_publisher.hpp"
#include "common/rgbd_source_identity.hpp"
#include <array>
#include <optional>

namespace perception::rns {
// Duplicate and OutOfOrder are counted and skipped; Invalid and CapacityExceeded latch
// the window invalid (the handoff treats any non-Accepted as fatal).
enum class HealthEventCode { Accepted, Duplicate, OutOfOrder, Invalid, CapacityExceeded };
// A rate stream (depth or inference) in the window and over the lifetime.
struct HealthRateReport {
    bool ever_observed = false;
    std::size_t window_frames = 0;
    std::uint64_t total_frames = 0;
    double fps = 0; // Always window_frames / 10, including startup's partial window.
};
// Put outcomes; pending = attempted but not yet returned from the native call.
struct HealthPutCounts {
    std::uint64_t attempted = 0, accepted = 0, rejected = 0, unknown = 0, pending = 0;
};
// One publication topic: counts, latency percentiles and interval statistics.
struct HealthPublicationReport {
    bool ever_observed = false;
    HealthPutCounts window, total; // Local API outcomes; no received/success-rate claim.
    std::optional<Millis> latency_p50_ms, latency_p99_ms;
    // Health statistics use every actual unique put, including cadence's
    // warmup/exclusions. The open gap is one snapshot-only sample, not inserted
    // repeatedly. No historical publication => all three gap values null.
    std::optional<Millis> gap_p99_ms, gap_max_ms, open_gap_ms;
    std::size_t closed_gap_samples = 0;
};
// Lifetime rejection counters, for the log.
struct HealthDiagnostics {
    std::uint64_t duplicates = 0, out_of_order = 0, invalid = 0, capacity_exceeded = 0;
    std::uint64_t clock_faults = 0;
};
// The whole window at one instant; full_window = at least 10 s since startup.
struct HealthWindowReport {
    bool valid = true, full_window = false;
    Millis now_ms = -1;
    HealthRateReport depth, inference;
    HealthPublicationReport profile, objects;
    HealthDiagnostics diagnostics;
};

// SINGLE owning thread; integration must synchronize event/snapshot handoff.
// This class reads no clock, launches no thread and never observes ObjectsCadence.
// Depth events are arrivals; inference events are completions. Rate events
// identify frames by SourceId. Publication events are actual put-before
// attempts, not API returns or remote receipts. Warmup and reconnect
// exclusions from cadence acceptance do NOT exclude health samples.
//
// All ingest/snapshot times share a monotonic epoch and must not decrease.
// Event times may arrive out of order ACROSS streams, but fresh captures within
// one stream require nondecreasing event times. For publication events,
// equal capture is Duplicate; smaller capture is OutOfOrder, neither counted
// again nor a sticky fault. Native rules are documented on their overloads.
// For a structurally valid duplicate/replay, capture checks precede event-order
// checks so a repeated old event does not masquerade as a fresh clock rollback.
//
// Window is (now-10000, now]. Ingest first prunes its stream at ingest time;
// newly supplied events already expired at ingest are Invalid (never revived).
// Invalid timestamps/audits, real clock rollback or capacity overflow latch
// report.valid=false. Later valid samples may still be counted, but no recovery
// or counter reset is provided in this version. A bad snapshot timestamp returns
// valid=false with empty current-window fields, retaining lifetime diagnostics.
//
// Construction allocates fixed rings and scratch. All event/snapshot calls have
// bounded storage and zero allocation; quantiles use exact nearest rank and
// bounded std::sort. Never-observed and observed-but-now-empty are distinguishable.
class HealthWindow {
public:
    static constexpr Millis kWindowMs = 10000;
    static constexpr std::size_t kMaxCapacityPerStream = 1000000;
    // startup in [0, ProcessEnvelope::kMaxMonoMs], capacity in [1, kMaxCapacity...].
    // Throws invalid_argument for unsupported configuration; allocation may fail.
    HealthWindow(Millis startup_ms, std::size_t capacity_per_stream);
    HealthWindow(const HealthWindow&) = delete;
    HealthWindow& operator=(const HealthWindow&) = delete;
    // Native source rates: real host_ready_mono_ns/1e6 remains the capture time;
    // sequence/index are identity, never fabricated timestamps. Distinct frames
    // may share a millisecond or even the same host nanosecond. Same stream
    // requires increasing sequence/depth_index and nondecreasing device/host
    // timestamps. An exact last SourceId replay is Duplicate; same sequence
    // with conflicting fields is Invalid; an older sequence is OutOfOrder.
    // Rejects never replace the last accepted identity. Arrival may omit RGB;
    // inference requires positive color_device_us. Changed stream IDs are not
    // numerically ordered; the lifecycle owner must authenticate current stream
    // membership (no unbounded history of retired IDs is maintained here).
    HealthEventCode observe_depth(const native_depth::SourceId&, Millis event_ms, Millis ingest_now_ms) noexcept;
    HealthEventCode observe_inference(const native_depth::SourceId&, Millis event_ms, Millis ingest_now_ms) noexcept;
    // Explicit split handoff, at most ONE pending native call per topic.
    // seq must be nonzero and strictly increase for successful split attempts.
    // Attempt counts capture/put-before latency and gap immediately; Pending is
    // not terminal Unknown. Invalid/expired/full attempts never replace pending.
    HealthEventCode observe_publication_attempt(Topic, std::uint64_t seq,
        Millis source_capture_ms, Millis put_before_ms, Millis ingest_now_ms) noexcept;
    // Resolves only the exact pending (topic, seq, capture, put_before) tuple;
    // does NOT add another sample or resurrect an expired window entry. The
    // pending association survives window expiry. Most recent resolved audit
    // is retained for exact idempotent replay; older unknown seq is Invalid.
    // A known API outcome with clock_valid=false is settled (Accepted return)
    // but latches valid=false and clock_faults; actual Attempt is not erased.
    // Duplicate same audit is Duplicate; conflicting audit is Invalid.
    HealthEventCode resolve_publication(Topic, const PublishAudit&, Millis ingest_now_ms) noexcept;
    // Prunes every ring to (now-10000, now] and computes the report; now must not go back.
    HealthWindowReport snapshot(Millis now_ms) noexcept;
    std::size_t capacity_per_stream() const noexcept { return streams_[0].events.size(); }
private:
    friend struct HealthSourceIdentityTestAccess;
    // One ring entry. closed_gap: interval to the previous put (publication streams
    // only, -1 for the first); outcome empty while the put is pending.
    struct Event {
        Millis capture, time, latency;
        std::optional<PutOutcome> outcome;
        Millis closed_gap = -1;
        std::uint64_t seq = 0; // Publication attempt seq (> 0); rate events store the source sequence.
    };
    // The one put whose native call has not returned yet.
    struct PendingPublication { std::uint64_t seq; Millis capture, put_before; };
    // One ring plus the per-stream ordering and identity watermarks.
    struct Stream {
        explicit Stream(std::size_t capacity) : events(capacity) {}
        std::vector<Event> events;
        std::size_t begin = 0, size = 0;
        Millis last_capture = -1, last_event = -1;
        std::uint64_t total_events = 0;
        HealthPutCounts total_puts;
        std::uint64_t last_attempt_seq = 0;
        std::optional<PendingPublication> pending;
        std::optional<PublishAudit> last_resolved;
        std::optional<native_depth::SourceId> last_source;
    };
    bool clock(Millis now) noexcept;
    void prune(Stream&, Millis now) noexcept;
    HealthEventCode reject(HealthEventCode) noexcept;
    HealthEventCode observe(std::size_t stream, Millis capture, Millis event,
        Millis ingest, Millis latency, std::optional<PutOutcome>, std::uint64_t seq = 0) noexcept;
    HealthEventCode observe_native_rate(std::size_t, const native_depth::SourceId&,
        Millis event, Millis ingest) noexcept;
    HealthRateReport rate(const Stream&) const noexcept;
    HealthPublicationReport publication(const Stream&, bool include_window, Millis now) noexcept;
    static void count_put(HealthPutCounts&, std::optional<PutOutcome>) noexcept;
    static void count_outcome(HealthPutCounts&, PutOutcome) noexcept;
    // 0 depth arrivals, 1 inference completions, 2 Profile puts, 3 Objects puts.
    std::array<Stream, 4> streams_;
    // Sort buffer for the percentiles, sized once (capacity + the open gap).
    std::vector<Millis> scratch_;
    const Millis startup_;
    Millis last_ingest_;
    bool valid_ = true;
    HealthDiagnostics diagnostics_;
};
}  // namespace perception::rns
