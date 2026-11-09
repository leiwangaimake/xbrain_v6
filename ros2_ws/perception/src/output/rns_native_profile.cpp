/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_native_profile.cpp
 * Brief: NativeProfileConsumer: owner validation and the per-frame step() pipeline.
 *
 * Description:
 * What problem this solves. Implements rns_native_profile.hpp. step() is written as one straight
 * pipeline with an early return per failed stage, so every frame ends in exactly one status and
 * a consumed frame is never retried.
 *
 * Why a clock read after every stage. The stamps (taken, geometry, grid, S, build, end) are the
 * Profile timing ledger; each read must not go backwards, and the put audit reported by the
 * publisher must lie inside the build-end .. end bracket. An inconsistent clock is reported as
 * ClockRejected rather than producing a body with untrustworthy times.
 */
#include "output/rns_native_profile.hpp"
#include <stdexcept>

namespace perception::rns {
namespace {
// Full identity equality (every field), as in rns_fused_objects.
bool same(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
}
NativeProfileConsumer::NativeProfileConsumer(native_depth::Mailbox& mailbox,
        geometry::NativeGeometry& geometry, geometry::NormalGrid& grid,
        NativeProfileClock clock, PublisherChannel& channel,
        SemanticProjector* semantic, SemanticFrameSource source,
        SourceJournal* processed_depth, HealthRoiView health_roi, bool extrinsic_calibrated)
    : mailbox_(mailbox), geometry_(geometry), grid_(grid), clock_(clock), channel_(channel),
      processed_depth_(processed_depth), health_roi_(health_roi), semantic_(semantic),
      semantic_source_(source), frame_(geometry::kGeometryPixels), builder_(grid, semantic),
      withdrawn_evidence_(geometry, semantic), withdrawn_builder_(withdrawn_evidence_) {
    // The S owner must describe this geometry and have a snapshot source.
    if (semantic_ && (&semantic_->geometry() != &geometry || !semantic_source_.latest))
        throw std::invalid_argument("native Profile S owner needs the same geometry and a snapshot source");
    validate(clock, channel, extrinsic_calibrated);
}
NativeProfileConsumer::NativeProfileConsumer(native_depth::Mailbox& mailbox,
        geometry::NativeGeometry& geometry, geometry::NormalGrid& grid,
        NativeProfileClock clock, PublisherChannel& channel,
        geometry::NormalTraversability* t, TFrameSource t_source,
        SemanticProjector* semantic, SemanticFrameSource source,
        SourceJournal* processed_depth, HealthRoiView health_roi, bool extrinsic_calibrated)
    : mailbox_(mailbox), geometry_(geometry), grid_(grid), clock_(clock), channel_(channel),
      processed_depth_(processed_depth), health_roi_(health_roi), semantic_(semantic),
      semantic_source_(source), t_(t), t_source_(t_source), frame_(geometry::kGeometryPixels),
      builder_(grid, t, semantic), withdrawn_evidence_(geometry, semantic),
      withdrawn_builder_(withdrawn_evidence_) {
    if (semantic_ && (&semantic_->geometry() != &geometry || !semantic_source_.latest))
        throw std::invalid_argument("native Profile S owner needs the same geometry and a snapshot source");
    // The T owner must describe this geometry and grid and have a mask source.
    if (t_ && (&t_->geometry() != &geometry || &t_->grid() != &grid || !t_source_.latest))
        throw std::invalid_argument("native Profile T owner needs the same geometry/grid and a mask source");
    validate(clock, channel, extrinsic_calibrated);
}
NativeProfileConsumer::NativeProfileConsumer(native_depth::Mailbox& mailbox,
        geometry::NativeGeometry& geometry, geometry::NormalGrid& grid,
        NativeProfileClock clock, PublisherChannel& channel,
        SourceJournal* processed_depth, HealthRoiView health_roi, bool extrinsic_calibrated)
    : mailbox_(mailbox), geometry_(geometry), grid_(grid), clock_(clock), channel_(channel),
      processed_depth_(processed_depth), health_roi_(health_roi), semantic_source_(),
      frame_(geometry::kGeometryPixels), builder_(grid),
      withdrawn_evidence_(geometry), withdrawn_builder_(withdrawn_evidence_) {
    // Geometry-only: no T and no S owner, so the withdrawal evidence is G-only as well.
    validate(clock, channel, extrinsic_calibrated);
}
// Startup checks (throw): owners belong together, the clock mapping is complete,
// the channel is the Profile channel, and the health ROI kind agrees with the
// calibration verdict (the Status depth-health fields depend on it).
void NativeProfileConsumer::validate(const NativeProfileClock& clock, PublisherChannel& channel,
                                     bool extrinsic_calibrated) {
    if (&grid_.geometry() != &geometry_ || !clock.context || !clock.epoch ||
        !clock.now_ns || !clock.capture || !channel.valid() || channel.topic() != Topic::Profile)
        throw std::invalid_argument("native Profile needs exact owners, explicit clock mapping and Profile channel");
    if (processed_depth_ && (!processed_depth_->valid() ||
        processed_depth_->kind() != SourceKind::DepthProcessed))
        throw std::invalid_argument("processed depth needs a valid single-owner journal");
    if (processed_depth_ && extrinsic_calibrated != (health_roi_.kind == HealthRoiKind::CalibratedStaticMask))
        throw std::invalid_argument("health ROI kind must match the extrinsic calibration verdict");
    builder_.set_extrinsic_calibrated(extrinsic_calibrated);
    withdrawn_builder_.set_extrinsic_calibrated(extrinsic_calibrated);
}
// One clock read into a stamp. All stamps of the consumer form one monotonic ledger, so a
// negative or backwards reading is refused and the caller ends the frame as ClockRejected.
bool NativeProfileConsumer::mark(std::int64_t& out) noexcept {
    out = clock_.now_ns(clock_.context);
    if (out < 0 || out < last_clock_ns_) return false;
    last_clock_ns_ = out;
    return true;
}
// Record a clock/capture inconsistency; the frame (if taken) is consumed.
// Names the one refusal the publish guard exists for: the body was built with a Fresh snapshot
// that crossed max age before the publish entry. Other refusals (stream change, owner moved on)
// are not counted here.
bool NativeProfileConsumer::semantic_expired_at(std::int64_t now_ns) const noexcept {
    if (!semantic_ || report_.semantic_status != SemanticStatus::Fresh) return false;
    const auto& objects = semantic_->report().objects;
    return objects && now_ns >= 0 && now_ns - objects->capture_mono_ns > semantic_->max_age_ns();
}
const NativeProfileTick& NativeProfileConsumer::clock_failure() noexcept {
    report_.status = NativeProfileStatus::ClockRejected;
    ++counts_.clock_rejected;
    return report_;
}
const NativeProfileTick& NativeProfileConsumer::step() noexcept {
    report_ = {};
    ++counts_.polls;
    if (!mark(report_.begin_ns)) return clock_failure();
    // Non-blocking take of the newest frame; nothing new is Idle, not an error.
    report_.take_called = true;
    report_.take = mailbox_.try_take(frame_);
    // Empty (nothing new) and Busy (the producer is writing) are the normal idle cases;
    // Capacity (raster size mismatch) and Inactive (no active stream) mean the source cannot
    // serve this consumer.
    if (report_.take != native_depth::Take::Taken) {
        report_.status = (report_.take == native_depth::Take::Empty ||
                          report_.take == native_depth::Take::Busy)
            ? NativeProfileStatus::Idle : NativeProfileStatus::SourceUnavailable;
        if (!mark(report_.end_ns)) return clock_failure();
        return report_;
    }
    ++counts_.taken;
    report_.source = frame_.metadata().source;
    if (!mark(report_.taken_ns)) return clock_failure();
    if (processed_depth_) {
        // The raster is owned here. Scan once, before fit/early returns/put,
        // including all-invalid frames. Timestamp only after this scan; SDK
        // arrival and profile publication are separate observations.
        report_.depth_health_ran = true;
        const auto observation = observe_depth_health(frame_, health_roi_);
        report_.depth_health_recorded = processed_depth_->record(observation);
        // Telemetry overflow/fault stays latched in its own journal. It does
        // not silently suppress the independent geometry/publication path.
    }
    // Capture time of this frame: same identity, same epoch, strictly newer than
    // the last one, and capture <= host-ready <= taken.
    geometry::CaptureStamp stamp;
    const bool mapped = clock_.capture(clock_.context, frame_.metadata(), stamp);
    if (!mark(report_.capture_checked_ns)) return clock_failure();
    if (!mapped || !same(stamp.source, report_.source) || stamp.clock_epoch != clock_.epoch ||
        stamp.capture_mono_ns < 0 || stamp.capture_mono_ns <= last_capture_ns_ ||
        stamp.capture_mono_ns > report_.source.host_ready_mono_ns ||
        report_.source.host_ready_mono_ns > report_.taken_ns) return clock_failure();
    report_.capture = stamp;
    // Kept for the next frame's strictly-newer check.
    last_capture_ns_ = stamp.capture_mono_ns;
    // Ground fit and radial geometry of this frame; its status selects the route.
    report_.geometry_ran = true;
    report_.geometry_status = geometry_.process(frame_, mailbox_).status;
    if (!mark(report_.geometry_end_ns)) return clock_failure();
    if (report_.geometry_status == geometry::GeometryStatus::FitFailed) {
        report_.route = NativeProfileRoute::FitFailedWithdrawal;
        report_.build_ran = true;
        // 19 A19-FIT-3 (reply batch 6, B6-2): the failed fit withdraws FREE,
        // NEG and every plane-dependent obstacle, but an INDEPENDENT S boundary
        // survives. So the S owner is processed here too, not only on the
        // normal path; a footprint that depends on the failed plane is refused
        // by the projector itself (plane_basis).
        std::int64_t withdrawn_now_ns = report_.geometry_end_ns;
        if (semantic_) {
            std::int64_t semantic_now_ns = -1;
            if (!mark(semantic_now_ns)) return clock_failure();
            const auto* snapshot = semantic_source_.latest(semantic_source_.context, stamp);
            report_.semantic_status = semantic_->process(mailbox_, stamp, semantic_now_ns, snapshot).status;
            report_.semantic_check_ns = semantic_now_ns;
            report_.semantic_stale_by_guard = semantic_->report().stale_by_guard;
            if (!mark(report_.semantic_end_ns)) return clock_failure();
            // The S owner was checked at semantic_end_ns: the evidence must be
            // built at or after that instant, never at an earlier one.
            withdrawn_now_ns = report_.semantic_end_ns;
            withdrawn_evidence_.build(mailbox_, withdrawn_now_ns);
        } else {
            withdrawn_evidence_.build(mailbox_, stamp, withdrawn_now_ns);
        }
        // Build the withdrawal body and copy its reasons for the Status adapter.
        const auto& wr = withdrawn_builder_.build(mailbox_, withdrawn_now_ns);
        report_.withdrawn_status = wr.status;
        report_.free_withdrawn = wr.evidence.free_withdrawn;
        report_.reasons = wr.evidence.reasons;
        report_.semantic_bins = wr.evidence.semantic_bins;
        if (!mark(report_.build_end_ns)) return clock_failure();
        if (report_.withdrawn_status != WithdrawnProfileStatus::Ready) {
            report_.status = NativeProfileStatus::ProfileUnavailable;
            ++counts_.unavailable;
            report_.end_ns = report_.build_end_ns;
            return report_;
        }
        // With an S owner: the actual current time at the publish entry, never
        // the earlier build end (an S snapshot inside this body must still be
        // valid now, and the channel rechecks it again at the commit instant).
        // G-only: nothing in the body has a lifetime; the build end is kept.
        std::int64_t publish_entry_ns = report_.build_end_ns;
        if (semantic_ && !mark(publish_entry_ns)) return clock_failure();
        if (semantic_) report_.publish_entry_ns = publish_entry_ns;
        const auto put = withdrawn_builder_.publish(mailbox_, channel_, publish_entry_ns);
        report_.withdrawn_status = put.status;
        report_.put = {put.dispatched, put.publish};
        report_.semantic_expired_at_publish = !put.dispatched && semantic_expired_at(publish_entry_ns);
    // Any other non-normal geometry: nothing is published for this frame.
    } else if (report_.geometry_status != geometry::GeometryStatus::NormalEvidence) {
        report_.status = NativeProfileStatus::GeometryUnavailable;
        ++counts_.unavailable;
        report_.end_ns = report_.geometry_end_ns;
        return report_;
    } else {
        // Normal fit: per-bin cells and radial walks.
        report_.grid_ran = true;
        report_.grid_status = grid_.process(mailbox_).status;
        if (!mark(report_.grid_end_ns)) return clock_failure();
        // A frame without a ground reference is still published (obstacles kept, FREE
        // withdrawn; rns_normal_profile.cpp), so only the other grid outcomes end the frame.
        if (report_.grid_status != geometry::GridStatus::NormalEvidence &&
            report_.grid_status != geometry::GridStatus::NoGroundReference) {
            report_.status = NativeProfileStatus::GridUnavailable;
            ++counts_.unavailable;
            report_.end_ns = report_.grid_end_ns;
            return report_;
        }
        report_.route = report_.grid_status == geometry::GridStatus::NoGroundReference
            ? NativeProfileRoute::NoGroundReference : NativeProfileRoute::Normal;
        // 19 section 3.3 / P19-1: the T owner runs for EVERY frame on these
        // routes with the newest mask the slot may give this frame, or none
        // (Missing); it never waits for inference. Without a ground reference it
        // reports GeometryUnavailable, which the builder accepts (B6-2).
        if (t_) {
            const auto* mask = t_source_.latest(t_source_.context, stamp);
            const auto& tr = t_->process(mailbox_, stamp, mask);
            report_.t_ran = true;
            report_.t_status = tr.status;
            report_.t_age_ns = tr.age_ns;
            report_.t_near_support_missing = tr.near_support_missing;
            if (!mark(report_.t_end_ns)) return clock_failure();
        }
        // 19 section 3.4A: the S channel is processed for EVERY taken frame,
        // including frames with no snapshot or an expired one, so a missing
        // slot degrades to geometry-only instead of skipping the owner.
        if (semantic_) {
            std::int64_t semantic_now_ns = -1;
            if (!mark(semantic_now_ns)) return clock_failure();
            const auto* snapshot = semantic_source_.latest(semantic_source_.context, stamp);
            report_.semantic_status = semantic_->process(mailbox_, stamp, semantic_now_ns, snapshot).status;
            report_.semantic_check_ns = semantic_now_ns;
            report_.semantic_stale_by_guard = semantic_->report().stale_by_guard;
            if (!mark(report_.semantic_end_ns)) return clock_failure();
        }
        report_.build_ran = true;
        report_.normal_status = builder_.build(mailbox_, stamp).status;
        if (!mark(report_.build_end_ns)) return clock_failure();
        if (report_.normal_status != NormalProfileStatus::Ready) {
            report_.status = NativeProfileStatus::ProfileUnavailable;
            ++counts_.unavailable;
            report_.end_ns = report_.build_end_ns;
            return report_;
        }
        // A no-ground body has FREE withdrawn too, but it is not a fit fallback, so no Status
        // reason is set here (11 v2.2: ground_free_withdrawn comes with ground_fit_fallback).
        report_.free_withdrawn = report_.route == NativeProfileRoute::NoGroundReference;
        report_.semantic_bins = builder_.report().semantic_bins;
        report_.t_bins = builder_.report().t_bins;
        // With S configured the publish entry rechecks the snapshot age against
        // the ACTUAL current clock (a fresh sample, not the earlier build end),
        // and the channel rechecks it again at the commit instant; -1 keeps the
        // G-only call unchanged.
        std::int64_t publish_entry_ns = -1;
        if (semantic_ && !mark(publish_entry_ns)) return clock_failure();
        report_.publish_entry_ns = publish_entry_ns;
        const auto put = builder_.publish(mailbox_, channel_, publish_entry_ns);
        report_.normal_status = put.status;
        report_.put = {put.dispatched, put.publish};
        report_.semantic_expired_at_publish = !put.dispatched && semantic_expired_at(publish_entry_ns);
    }
    // Counters: accepted means the local put API accepted it, nothing more.
    counts_.dispatches += report_.put.dispatched;
    counts_.accepted += report_.put.dispatched && report_.put.publish.audit.attempted &&
        report_.put.publish.audit.receipt.outcome == PutOutcome::Accepted;
    report_.status = report_.put.dispatched ? NativeProfileStatus::PutFinished
                                           : NativeProfileStatus::ProfileUnavailable;
    counts_.unavailable += !report_.put.dispatched;
    // A post-put clock failure cannot undo an attempted put. Keep its complete
    // receipt/identity in this report and never retry the consumed frame.
    if (!mark(report_.end_ns)) return clock_failure();
    // The publisher's millisecond put stamps must lie inside our nanosecond
    // bracket (build end .. end); otherwise the two clocks disagree.
    const auto& audit = report_.put.publish.audit;
    if (audit.attempted) {
        report_.publish_clock_bracket_valid = audit.clock_valid &&
            report_.build_end_ns/1000000 <= audit.put_before_ms &&
            audit.put_before_ms <= audit.put_return_ms &&
            audit.put_return_ms <= report_.end_ns/1000000;
        if (!report_.publish_clock_bracket_valid) return clock_failure();
    }
    return report_;
}
// Raster plus builder storage, for the startup memory log.
std::size_t NativeProfileConsumer::storage_bytes() const noexcept {
    return frame_.capacity_pixels()*sizeof(std::uint16_t) + builder_.storage_bytes() +
        withdrawn_evidence_.storage_bytes() + withdrawn_builder_.storage_bytes() +
        (t_ ? t_->storage_bytes() : 0);
}
} // namespace perception::rns
