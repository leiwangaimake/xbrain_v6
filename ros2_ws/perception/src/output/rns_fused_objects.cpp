/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_fused_objects.cpp
 * Brief: FusedObjectsAdapter: frame identity checks and fused/track -> Objects field mapping.
 *
 * Description:
 * What problem this solves. Implements the two-step adapter declared in rns_fused_objects.hpp.
 * prepare() is the gate in front of the tracker: a frame that is stale, reordered, duplicated,
 * from another stream or stamped on another clock never reaches the tracker, because a tracker
 * fed out of order would produce wrong ages and velocities with no visible error. build() maps
 * the fused detections to Objects (19 section 4.1) and cross-checks them against the tracks of
 * the same update.
 *
 * Failure policy. Every inconsistency is a contract fault, not a frame to skip: the adapter
 * latches faulted_ and the Supervisor stops the service (R6_SOURCE_OR_CAPTURE_REJECTED...).
 * Silently dropping frames here would turn an upstream bug into an Objects cadence gap that
 * looks like load.
 *
 * Fixed values stated honestly: velocity_frame "raw" (no ego-motion removal, 19 section 4.2),
 * depth_quality "poor" (three-level mapping not implemented, decision Q10).
 */
#include "output/rns_fused_objects.hpp"
#include <cmath>
#include <limits>

namespace perception::rns {
namespace {
// Full identity equality: every field, so a frame that only reuses a sequence
// number (a stream restart, a replay) is not mistaken for the prepared one.
bool same(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
// SourceId defaults (stream 0 = unavailable, sequence starts at 1, device times
// 0, host_ready -1) mean the identity was never filled.
bool source_valid(const native_depth::SourceId& s) noexcept {
    return s.stream_id && s.sequence && s.color_device_us && s.depth_device_us &&
        s.host_ready_mono_ns >= 0;
}
// Class 92 traversable_area is split off to the map side by the dual-model merge
// (dual_model_merge.cpp) and the Objects contract forbids it (rns_contract valid),
// so seeing it here means that split was bypassed. semantic_status is the
// two-value Objects enum.
bool class_valid(int id, const std::string& name, float confidence,
                const std::string& semantic) noexcept {
    return id >= 0 && !name.empty() && name != "traversable_area" &&
        std::isfinite(confidence) && confidence >= 0 && confidence <= 1 &&
        (semantic == "proxy" || semantic == "confirmed");
}
}
// The two journals must be distinct objects of the right kinds: Status computes
// the arrival rate and the inference-completion rate from them separately
// (PublicationHealthHandoff), and one journal wired twice would report arrivals
// as completions and hide an inference stall.
bool SupervisorRnsContext::valid() const noexcept {
    return native_mailbox && depth_arrivals && inference_completions && clock_epoch &&
        callback_owner && now_ns && capture_stamp && objects_ready &&
        depth_arrivals->valid() && inference_completions->valid() &&
        depth_arrivals->kind() == SourceKind::DepthArrival &&
        inference_completions->kind() == SourceKind::InferenceComplete &&
        depth_arrivals.get() != inference_completions.get();
}
FusedObjectsAdapter::FusedObjectsAdapter(std::uint64_t epoch) : epoch_(epoch) {
    // Reserve once so steady-state build() does not grow the vector.
    body_.objects.reserve(kMaxObjects);
}
FusedObjectsStatus FusedObjectsAdapter::prepare(const yolo::DetectionResult& result,
        const geometry::CaptureStamp& capture, std::int64_t now) noexcept {
    using S = FusedObjectsStatus;
    if (!valid()) return faulted_ ? S::Faulted : S::Configuration;
    // A second prepare before build would lose the first frame's identity.
    if (pending_) return S::Pending;
    if (!result.inference_completed) return S::FailedInference;
    const auto& s = result.native_depth_source;
    // Tracks store frames as int (last_seen_frame), so a frame id above INT_MAX
    // could not be compared in build(); reject it up front.
    if (!source_valid(s) || result.frame_id > static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
        return S::InvalidSource;
    // The capture stamp must describe this very frame.
    if (!same(s, capture.source)) return S::SourceMismatch;
    // Causality on one clock: capture <= host-ready <= now, and now never
    // goes backwards between frames.
    if (capture.clock_epoch != epoch_ || now < 0 || capture.capture_mono_ns < 0 ||
        capture.capture_mono_ns > s.host_ready_mono_ns || s.host_ready_mono_ns > now ||
        now < last_now_) return S::ClockRejected;
    if (has_last_) {
        // One adapter serves one stream; ordering across two streams is
        // meaningless, so a change is a fault (the service stops).
        if (s.stream_id != last_source_.stream_id) return S::StreamChanged;
        if (same(s,last_source_) && result.frame_id == last_frame_ &&
            capture.capture_mono_ns == last_capture_) return S::Duplicate;
        // Everything must move forward. The capture check is at millisecond
        // granularity because the wire carries t_capture_mono_ms: two frames in
        // the same millisecond would publish a non-increasing capture time.
        if (s.sequence <= last_source_.sequence || s.depth_index <= last_source_.depth_index ||
            s.color_device_us < last_source_.color_device_us ||
            s.depth_device_us < last_source_.depth_device_us ||
            s.host_ready_mono_ns < last_source_.host_ready_mono_ns || result.frame_id <= last_frame_ ||
            capture.capture_mono_ns / 1000000 <= last_capture_ / 1000000) return S::OutOfOrder;
    }
    // Detections are checked here, before the tracker, so a malformed one
    // cannot enter track state; the same rules as class_valid plus a real box.
    if (result.detections.size() > kMaxObjects) return S::InvalidDetections;
    for (const auto& d : result.detections) {
        if (d.class_id < 0 || d.class_name.empty() || d.class_name == "traversable_area" ||
            !std::isfinite(d.confidence) || d.confidence < 0 || d.confidence > 1 ||
            d.bbox.width <= 0 || d.bbox.height <= 0) return S::InvalidDetections;
    }
    // Consume the identity now, not in build(): even if tracking or build fails
    // afterwards, this frame can never be offered again.
    has_last_ = pending_ = true;
    last_source_ = s; last_frame_ = result.frame_id;
    last_capture_ = capture.capture_mono_ns; last_now_ = now;
    body_.objects.clear();
    return S::Prepared;
}
FusedObjectsReport FusedObjectsAdapter::build(const yolo::DetectionResult& result,
        const geometry::CaptureStamp& capture, const std::vector<FusedDetection>& fused,
        const std::vector<Track>& tracks) noexcept {
    using S = FusedObjectsStatus;
    FusedObjectsReport report;
    report.source = result.native_depth_source; report.frame_id = result.frame_id;
    report.input_fused = fused.size(); report.tracks = tracks.size();
    // Any failure latches the fault and leaves an empty body, so a partially
    // built list can never be handed on.
    const auto fail = [&](S why) { abort(); body_.objects.clear(); report.status = why; return report; };
    const auto fail_detail = [&](S why, std::string detail) { report.detail = std::move(detail); return fail(why); };
    if (!valid()) { report.status = faulted_ ? S::Faulted : S::Configuration; return report; }
    if (!pending_) return fail(S::NoPreparation);
    pending_ = false; // one invocation even after a malformed body / failed callback
    if (!result.inference_completed || !same(result.native_depth_source,last_source_) ||
        !same(capture.source,last_source_) || result.frame_id != last_frame_ ||
        capture.clock_epoch != epoch_ || capture.capture_mono_ns != last_capture_)
        return fail(S::SourceMismatch);
    if (fused.size() > kMaxObjects || tracks.size() > kMaxObjects) return fail(S::InvalidFused);
    // Track sanity: valid class fields, no frame from the future, unique ids.
    // Quadratic, but both lists are bounded by kMaxObjects and usually small.
    for (std::size_t i=0; i<tracks.size(); ++i) {
        const auto& t=tracks[i];
        if (t.track_id < 0 || t.last_seen_frame < 0 ||
            t.last_seen_frame > static_cast<int>(last_frame_) || t.stable_frames < 0 ||
            !class_valid(t.class_id,t.class_name,t.confidence,t.semantic_status)) return fail(S::InvalidTracks);
        for (std::size_t j=0; j<i; ++j) if(tracks[j].track_id==t.track_id) return fail(S::InvalidTracks);
    }
    body_.t_capture_mono_ms = last_capture_ / 1000000;
    // Stamped by the publisher at the moment of sending, not here.
    body_.t_publish_mono_ms = -1;
    body_.extrinsic_calibrated = calibrated_;
    // String copies and the footprint vector can throw bad_alloc; this function
    // is noexcept, so any throw becomes a latched InvalidFused.
    try {
        for (std::size_t i=0; i<fused.size(); ++i) {
            const auto& f=fused[i];
            for (std::size_t j=0; j<i; ++j) if(fused[j].track_id==f.track_id) return fail(S::InvalidFused);
            // Every fused detection must come from a track of the same update
            // with identical copied fields; a mismatch means fusion and
            // tracking saw different states (exact float compare is intended:
            // the values are copies, not recomputations).
            const Track* matched=nullptr;
            for (const auto& t:tracks) if(t.track_id==f.track_id) matched=&t;
            if (!matched) return fail(S::TrackMismatch);
            const auto& t=*matched;
            if (f.class_id != t.class_id || f.class_name != t.class_name ||
                f.confidence != t.confidence || f.semantic_status != t.semantic_status ||
                f.stable_frames != t.stable_frames || f.last_seen_frame != t.last_seen_frame)
                return fail(S::TrackMismatch);
            // A track not observed in this frame is only predicted (coasting);
            // Objects carries measured objects only, so it is not published.
            if (!observed_in_frame(t.last_seen_frame, last_frame_)) {
                ++report.coast_filtered; continue;
            }
            // Observation times: first seen not after this capture, last seen
            // exactly this capture (the contract requires last == t_capture).
            if (!t.rns_observation_times || t.rns_observation_times->first_seen_mono_ms < 0 ||
                t.rns_observation_times->first_seen_mono_ms > body_.t_capture_mono_ms ||
                t.rns_observation_times->last_seen_mono_ms != body_.t_capture_mono_ms)
                return fail(S::InvalidTracks);
            Object o;
            o.track_id=f.track_id; o.class_id=f.class_id; o.class_name=f.class_name;
            o.confidence=f.confidence; o.semantic_status=f.semantic_status;
            o.stable_frames=static_cast<std::uint64_t>(f.stable_frames);
            o.first_seen_mono_ms=t.rns_observation_times->first_seen_mono_ms;
            o.last_seen_mono_ms=t.rns_observation_times->last_seen_mono_ms;
            // Conservative defaults: no geometry, no velocity. velocity_frame
            // "raw" (19 section 4.2) and depth_quality "poor" (Q10) are fixed.
            o.geometry.reset(); o.velocity_xy.reset(); o.velocity_valid=false;
            o.velocity_frame="raw"; o.velocity_status="warming_up"; o.depth_quality="poor";
            // Velocity (batch-4 reply B4-1): every estimator status is handled
            // explicitly and checked against its validity flag; any other value
            // is an estimator contract fault, reported with its detail.
            // numeric: the estimator produced a vector (motion_unconfirmed has one
            // but velocity_valid=false). gated: the window was closed on a gate.
            // warming_up / timestamp_gap: no estimate yet, passed through.
            const std::string& st = f.velocity_status;
            const bool numeric = st == "moving" || st == "static" || st == "motion_unconfirmed";
            const bool gated = st == "track_unstable" || st == "low_depth_confidence" || st == "depth_unreliable";
            if (!numeric && !gated && st != "warming_up" && st != "timestamp_gap")
                return fail_detail(S::InvalidFused, "unknown velocity_status '" + st + "' track " + std::to_string(f.track_id));
            if ((st == "moving" || st == "static") != f.velocity_valid)
                return fail_detail(S::InvalidFused, "velocity_status '" + st + "' with velocity_valid=" +
                                  (f.velocity_valid ? "true" : "false") + " track " + std::to_string(f.track_id));
            // Only camera-frame estimates can be rotated into base_link as raw.
            if (numeric && f.velocity_frame != FusedDetection::VelocityFrame::Camera)
                return fail_detail(S::InvalidFused, std::string("numeric velocity in ") +
                                  (f.velocity_frame == FusedDetection::VelocityFrame::Odom ? "odom" : "unset") +
                                  " frame track " + std::to_string(f.track_id) + " (only camera-frame raw velocity is supported)");
            if (f.base_geometry_valid) {
                // Measured base_link geometry of this inference frame (19 section 4.1).
                std::vector<Point> footprint;
                footprint.reserve(f.base_footprint_xy.size());
                for (const auto& v : f.base_footprint_xy) footprint.push_back({v[0], v[1]});
                if (!valid_footprint(footprint) || !std::isfinite(f.base_z_min) || !std::isfinite(f.base_z_max) ||
                    f.base_z_min > f.base_z_max) return fail_detail(S::InvalidFused, "invalid measured geometry track " + std::to_string(f.track_id));
                o.geometry.emplace(std::move(footprint), f.base_z_min, f.base_z_max);
                ++report.located;
                if (numeric) {
                    // Raw velocity (no ego-motion removal): the camera-frame window
                    // estimate rotated into base_link axes; motion_unconfirmed keeps
                    // its real candidate with velocity_valid=false.
                    const auto& R = f.base_from_colour_R;
                    const float vx = R[0]*f.velocity_3d[0] + R[1]*f.velocity_3d[1] + R[2]*f.velocity_3d[2];
                    const float vy = R[3]*f.velocity_3d[0] + R[4]*f.velocity_3d[1] + R[5]*f.velocity_3d[2];
                    if (!std::isfinite(vx) || !std::isfinite(vy))
                        return fail_detail(S::InvalidFused, "non-finite velocity track " + std::to_string(f.track_id));
                    o.velocity_xy = std::array<float, 2>{vx, vy};
                    o.velocity_valid = f.velocity_valid;
                    o.velocity_status = st;
                    ++report.velocity_numeric;
                    if (st == "motion_unconfirmed") ++report.velocity_unconfirmed_numeric;
                } else if (gated) {
                    // The estimator ended this track's velocity window on the gate:
                    // no estimate, waiting to re-accumulate an accepted window.
                    o.velocity_status = "warming_up";
                    ++report.velocity_gating_mapped;
                    if (st == "track_unstable") ++report.velocity_gated_track_unstable;
                    else if (st == "low_depth_confidence") ++report.velocity_gated_low_depth_confidence;
                    else ++report.velocity_gated_depth_unreliable;
                } else {
                    o.velocity_status = st;
                }
            } else {
                // Unlocatable: the base_link rotation is filled only together
                // with valid geometry (object geometry hook in the service), so a
                // camera-frame velocity cannot be expressed; the object keeps the
                // conservative defaults (no geometry, warming_up).
                ++report.unlocatable;
            }
            body_.objects.push_back(std::move(o));
        }
    } catch (...) { return fail(S::InvalidFused); }
    report.objects=body_.objects.size(); report.status=S::Ready;
    return report;
}
} // namespace perception::rns
