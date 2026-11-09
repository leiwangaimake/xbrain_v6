/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_unconfigured_profile.cpp
 * Brief: One step = take, identity checks, health scan, capture stamp, put of an UNKNOWN body.
 *
 * Description:
 * The body is prepared once in the constructor (all bins UNKNOWN, wire metadata fixed); a step
 * only sets t_capture_mono_ms and publishes. The work that remains per frame is what makes the
 * UNKNOWN stream trustworthy: the frame must be new and belong to the active stream, its capture
 * time must be derived from its own SourceId in the service clock domain, and the publish must
 * be bracketed by consumer clock samples.
 *
 * Why a frame is consumed before it is published. last_source_ is updated as soon as the
 * identity checks pass. If mapping, encoding or the put then fails, the next step cannot retry
 * the same frame: a retried frame would reach the wire late with an old capture time, which the
 * consumer would take as a real, stale observation.
 *
 * Why the publish bracket. The publisher stamps put_before/put_return on its own clock; this
 * consumer samples build_end and end on the capture clock. Both are meant to be the same
 * monotonic clock; requiring build_end <= put_before <= put_return <= end (in ms) catches a
 * mix-up of clock sources at the first frame instead of in a latency report. It cannot catch
 * a substitution that keeps the order (see the header).
 */
#include "output/rns_unconfigured_profile.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception::rns {
namespace {
// Full source identity: the capture stamp must belong to exactly the taken frame.
bool same(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
// Wire metadata the contract encoder can write: positive z_pass, positive step, range
// in (0, 6] m (the contract validator's bound), bin count within kMaxBins, an epoch.
bool valid(const UnconfiguredProfileDomain& d) noexcept {
    // A zero-initialised domain fails here on purpose: there are no default values.
    if (!std::isfinite(d.z_pass_m) || d.z_pass_m <= 0 ||
        // angles: finite start, finite positive step (bins must fan out)
        !std::isfinite(d.angle_min_rad) || !std::isfinite(d.angle_step_rad) ||
        d.angle_step_rad <= 0 || !std::isfinite(d.range_max_m) ||
        // range and bin count: the same limits the contract encoder enforces
        d.range_max_m <= 0 || d.range_max_m > 6 || !d.n_bins ||
        d.n_bins > kMaxBins || !d.clock_epoch) return false;
    // Only numeric representability, not a calibrated angular field of view.
    // The last bin's angle, computed in double so the check itself cannot overflow.
    const double last = static_cast<double>(d.angle_min_rad) +
        static_cast<double>(d.angle_step_rad) * (d.n_bins - 1);
    return std::isfinite(last) && std::abs(last) <= std::numeric_limits<float>::max();
}
}
UnconfiguredProfileConsumer::UnconfiguredProfileConsumer(native_depth::Mailbox& mailbox,
        std::size_t frame_capacity_pixels, UnconfiguredProfileDomain domain,
        NativeProfileClock clock, PublisherChannel& channel, SourceJournal* processed_depth)
    : mailbox_(mailbox), domain_(domain), clock_(clock), channel_(channel),
      processed_depth_(processed_depth), frame_(frame_capacity_pixels) {
    // Startup checks throw: a wrong domain, clock or channel is a configuration error.
    // The clock epoch must equal the domain epoch so stamps and publish share one domain.
    if (!valid(domain_) || !clock_.context || !clock_.now_ns || !clock_.capture ||
        clock_.epoch != domain_.clock_epoch || !channel_.valid() || channel_.topic() != Topic::Profile)
        throw std::invalid_argument("unconfigured Profile needs explicit wire domain, matching clock epoch and Profile channel");
    // The optional health journal must be the processed-depth kind: arrival or
    // inference journals feed other Status fields and must not receive scans.
    if (processed_depth_ && (!processed_depth_->valid() ||
        processed_depth_->kind() != SourceKind::DepthProcessed))
        throw std::invalid_argument("unconfigured Profile needs a valid DepthProcessed journal");
    // Allocate the body once; the hot path only writes t_capture_mono_ms.
    body_.bins.resize(domain_.n_bins);
    body_.z_pass_m = domain_.z_pass_m;
    body_.angle_min_rad = domain_.angle_min_rad;
    body_.angle_step_rad = domain_.angle_step_rad;
    body_.range_max_m = domain_.range_max_m;
    // Default Bin and Profile fields encode UNKNOWN; they never acquire raw
    // sample attribution or geometry. Only capture time changes on a valid tick.
}
// Every stage time of a step goes through here, so all of them are ordered.
bool UnconfiguredProfileConsumer::mark(std::int64_t& out) noexcept {
    out = clock_.now_ns(clock_.context);
    // Negative = the clock reported an error; smaller = the clock went back. Both
    // make every later interval of this step meaningless, so the step stops.
    if (out < 0 || out < last_clock_ns_) return false;
    last_clock_ns_ = out;
    return true;
}
// Rechecked between stages: a stream restart mid-step abandons the frame.
bool UnconfiguredProfileConsumer::current_source() const noexcept {
    // stream_id 0 = no frame taken yet in this step.
    return report_.source.stream_id != 0 && mailbox_.active_stream() == report_.source.stream_id;
}
// The service counts consecutive non-Idle, non-source failures and stops after 90
// (about 3 s at 30 fps): a persistently broken clock must not look like silence.
const UnconfiguredProfileTick& UnconfiguredProfileConsumer::clock_failure() noexcept {
    report_.status = UnconfiguredProfileStatus::ClockRejected;
    ++counts_.clock_rejected;
    return report_;
}
// A source failure still stamps end_ns; a clock failure there wins as the status.
const UnconfiguredProfileTick& UnconfiguredProfileConsumer::source_failure() noexcept {
    report_.status = UnconfiguredProfileStatus::SourceUnavailable;
    // unavailable counts every tick that ended without a put for source reasons.
    ++counts_.source_rejected;
    ++counts_.unavailable;
    if (!mark(report_.end_ns)) return clock_failure();
    return report_;
}
const UnconfiguredProfileTick& UnconfiguredProfileConsumer::step() noexcept {
    // Fresh tick report; the body loses the previous capture time before anything else.
    report_ = {};
    body_.t_capture_mono_ms = body_.t_publish_mono_ms = -1;
    ++counts_.polls;
    // First sample of the tick; every stage time below is later or equal.
    if (!mark(report_.begin_ns)) return clock_failure();
    // take_called separates "clock failed before the take" from "mailbox empty".
    report_.take_called = true;
    // Non-blocking take of the latest frame into the owned raster.
    report_.take = mailbox_.try_take(frame_);
    if (report_.take != native_depth::Take::Taken) {
        // Empty/Busy are normal (Idle); any other result is a source fault.
        if (report_.take != native_depth::Take::Empty && report_.take != native_depth::Take::Busy)
            return source_failure();
        if (!mark(report_.end_ns)) return clock_failure();
        return report_;
    }
    // From here on a frame is in hand; every exit below accounts for it.
    ++counts_.taken;
    report_.source = frame_.metadata().source;
    if (!mark(report_.taken_ns)) return clock_failure();
    const auto& s = report_.source;
    // Identity must advance: within a stream every index and device time strictly
    // increases; a new stream must have a higher id. Otherwise it is a replay.
    if (!current_source() || !s.sequence || s.stream_id < last_source_.stream_id ||
        // same stream: all four SDK counters/times must move forward together
        (s.stream_id == last_source_.stream_id &&
         (s.sequence <= last_source_.sequence || s.color_index <= last_source_.color_index ||
          s.depth_index <= last_source_.depth_index ||
          s.color_device_us <= last_source_.color_device_us ||
          s.depth_device_us <= last_source_.depth_device_us))) return source_failure();
    // Host ready time: sampled, not after we took the frame, never backwards.
    if (s.host_ready_mono_ns < 0 || s.host_ready_mono_ns > report_.taken_ns ||
        s.host_ready_mono_ns < last_source_.host_ready_mono_ns) return clock_failure();
    last_source_ = s; // consumed even if later mapping, encoding or put fails

    // Health scan with the default ROI (uncalibrated lower half), right after the
    // identity checks and before mapping and publishing, so invalid_pixel_ratio
    // covers every consumed frame, including ones whose publish later fails.
    report_.depth_health_ran = true;
    ++counts_.health_scanned;
    report_.depth_health = observe_depth_health(frame_, {});
    // Journal failure is recorded in the tick but does not stop the Profile: the
    // health path and the Profile path fail independently.
    if (processed_depth_) {
        report_.depth_health_recorded = processed_depth_->record(report_.depth_health);
        counts_.health_recorded += report_.depth_health_recorded;
    }
    if (!mark(report_.health_end_ns)) return clock_failure();
    // A frame the health scan rejects (bad size, no identity) is not published either.
    if (report_.depth_health.code != DepthHealthCode::Ready || !current_source()) return source_failure();

    // Local stamp: a value from a previous tick can never leak into this one.
    geometry::CaptureStamp stamp;
    report_.capture_mapping_ran = true;
    // Capture time from the service clock (the declared link estimate in production).
    const bool mapped = clock_.capture(clock_.context, frame_.metadata(), stamp);
    if (!mark(report_.capture_checked_ns)) return clock_failure();
    // The stamp must be for this frame and domain, not after host ready, and in a
    // later wire millisecond than the previous Profile (no duplicate t_capture).
    if (!mapped || !same(stamp.source, s) || stamp.clock_epoch != domain_.clock_epoch ||
        stamp.capture_mono_ns < 0 || stamp.capture_mono_ns > s.host_ready_mono_ns ||
        stamp.capture_mono_ns / 1000000 <= last_capture_ms_) return clock_failure();
    // The stream may have been replaced while the stamp was computed.
    if (!current_source()) return source_failure();
    // Commit: the body now describes this frame's capture millisecond.
    report_.capture = stamp;
    last_capture_ms_ = stamp.capture_mono_ns / 1000000;
    // "Build" is only this one field: all bins stay UNKNOWN by construction.
    report_.build_ran = true;
    ++counts_.builds;
    body_.t_capture_mono_ms = last_capture_ms_;
    if (!mark(report_.build_end_ns)) return clock_failure();
    // Last source check before the wire: never publish a frame of a dead stream.
    if (!current_source()) return source_failure();
    // The put. dispatched is set first: whatever happens next, this frame is spent.
    report_.put.dispatched = true;
    report_.put.publish = channel_.publish(body_);
    ++counts_.dispatches;
    // Receipt accounting; accepted means the local API accepted it, not delivery.
    const auto& audit = report_.put.publish.audit;
    // attempted = the publisher reached the transport (finalize ran); a prepare
    // failure before that is dispatched but not attempted, and has no receipt.
    counts_.attempted += audit.attempted;
    counts_.accepted += audit.attempted && audit.receipt.outcome == PutOutcome::Accepted;
    counts_.rejected += audit.attempted && audit.receipt.outcome == PutOutcome::Rejected;
    counts_.unknown += audit.attempted && audit.receipt.outcome == PutOutcome::Unknown;
    // PutFinished regardless of the outcome: the tick did its work; the outcome is
    // in put.publish and the counters, and the service loop reads it from there.
    report_.status = UnconfiguredProfileStatus::PutFinished;
    // Preserve actual receipts even if either return clock fails. The frame is
    // consumed, so an idle next tick cannot reattempt the body.
    if (!mark(report_.end_ns)) return clock_failure();
    // The bracket (see the file header), compared in wire milliseconds because the
    // publisher's audit is in milliseconds. Checked only when a put really happened.
    if (audit.attempted) {
        report_.publish_clock_bracket_valid = audit.clock_valid &&
            report_.build_end_ns / 1000000 <= audit.put_before_ms &&
            audit.put_before_ms <= audit.put_return_ms &&
            audit.put_return_ms <= report_.end_ns / 1000000;
        // A broken bracket turns the tick into ClockRejected, but this consumer's
        // counters above keep the receipt: an accepted put is not reported as lost.
        // (The service loop then counts the tick as abnormal, not as accepted.)
        if (!report_.publish_clock_bracket_valid) return clock_failure();
    }
    return report_;
}
// Raster capacity (uint16 depth codes) plus bins, for the memory budget log.
std::size_t UnconfiguredProfileConsumer::storage_bytes() const noexcept {
    return frame_.capacity_pixels() * sizeof(std::uint16_t) + body_.bins.capacity() * sizeof(Bin);
}
} // namespace perception::rns
