/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_fused_objects.hpp
 * Brief: Current Supervisor result/tracks -> R6 Objects body. No second tracker.
 *
 * Description:
 * What problem this solves. The Supervisor already tracks, depth-samples and fuses every
 * inference result (the V5 pipeline). FusedObjectsAdapter turns one such frame into the
 * 11 section 3.1B.2 Objects body (field mapping 19 section 4.1) without re-deciding anything:
 * it only checks that the frame is new and self-consistent, maps fields, and states honestly
 * what is not implemented (velocity_frame "raw" because ego-motion removal is not implemented,
 * 19 section 4.2; depth_quality "poor" because the three-level mapping is not implemented,
 * decision Q10).
 *
 * Two calls per frame, in this order, on the Supervisor's process thread:
 *   prepare()  before the tracker update: validates and CONSUMES the frame identity, so a
 *              frame can never be offered twice even if later work fails;
 *   build()    after fusion: builds the body from the fused detections and the same-update
 *              tracks; any inconsistency latches a fault (the service then stops).
 *
 * SupervisorRnsContext is the wiring the R6 service hands to the Supervisor (see below).
 */
#pragma once
#include "common/types.hpp"
#include "modules/native_depth.hpp"
#include "modules/traversability.hpp"
#include "output/rns_contract.hpp"
#include "output/rns_source_journal.hpp"
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>

namespace perception::rns {
// Outcome of prepare()/build(); only Prepared (prepare) and Ready (build) proceed.
enum class FusedObjectsStatus { Configuration, Prepared, Ready, FailedInference,
    InvalidSource, SourceMismatch, ClockRejected, StreamChanged, Duplicate,
    OutOfOrder, InvalidDetections, Pending, NoPreparation, TrackMismatch,
    InvalidTracks, InvalidFused, Faulted };
// A track was observed in a frame when its last_seen_frame is that frame's id; otherwise it is only
// predicted (coasting) and Objects does not carry it. Tracks store frames as int, so a frame id above
// INT_MAX (which prepare() refuses) counts as "not observed". One rule for the adapter's filter and
// for work that only a published object needs (the service's object geometry, 2026-10-13).
inline bool observed_in_frame(int last_seen_frame, std::uint64_t frame_id) noexcept {
    return frame_id <= static_cast<std::uint64_t>(std::numeric_limits<int>::max()) &&
           last_seen_frame == static_cast<int>(frame_id);
}
// What build() did, for the Objects owner's counters and the log.
struct FusedObjectsReport {
    FusedObjectsStatus status = FusedObjectsStatus::Configuration;
    native_depth::SourceId source;
    std::uint64_t frame_id = 0;
    // Inputs, tracks not seen in this frame (coasting, not published), objects out.
    std::size_t input_fused = 0, tracks = 0, coast_filtered = 0, objects = 0;
    // Measured geometry present / unlocatable. Velocity (batch-4 reply B4-1):
    // numeric (moving / static / motion_unconfirmed candidates) and estimator
    // gates, whose window the estimator ended, published as null + warming_up.
    std::size_t located = 0, unlocatable = 0, velocity_numeric = 0, velocity_unconfirmed_numeric = 0,
                velocity_gating_mapped = 0, velocity_gated_track_unstable = 0,
                velocity_gated_low_depth_confidence = 0, velocity_gated_depth_unreliable = 0;
    std::string detail;  // why a build was rejected (empty on Ready)
};

// Single Supervisor/process owner. prepare() validates AND consumes source
// identity before the caller's one timed tracker update. build() uses only the
// final fused population and same-update tracks; no tracking, depth or put work.
// A prepared observation is never replayable, even if downstream work fails.
// build errors latch a fault. Recreate the owner on stream/restart, never reset
// track timestamps from an old wall-clock or a newly published packet.
class FusedObjectsAdapter {
public:
    // clock_epoch identifies the service monotonic domain of every capture stamp.
    explicit FusedObjectsAdapter(std::uint64_t clock_epoch);
    // Same contract as NormalProfileBuilder::set_extrinsic_calibrated: the three
    // topics carry the same verdict (19 section 3.5).
    void set_extrinsic_calibrated(bool calibrated) noexcept { calibrated_ = calibrated; }
    FusedObjectsAdapter(const FusedObjectsAdapter&) = delete;
    FusedObjectsAdapter& operator=(const FusedObjectsAdapter&) = delete;
    // Validate and consume the frame identity (see the file header).
    FusedObjectsStatus prepare(const yolo::DetectionResult&,
        const geometry::CaptureStamp&, std::int64_t now_mono_ns) noexcept;
    // Build the body for the prepared frame; exactly one build per prepare.
    FusedObjectsReport build(const yolo::DetectionResult&,
        const geometry::CaptureStamp&, const std::vector<FusedDetection>&,
        const std::vector<Track>&) noexcept;
    // Latch a fault from outside (the Supervisor calls it when a frame failed).
    void abort() noexcept { faulted_ = true; pending_ = false; }
    // Configured and never faulted; a fault is permanent for this instance.
    bool valid() const noexcept { return epoch_ != 0 && !faulted_; }
    const Objects& last_body() const noexcept { return body_; } // only after Ready
private:
    std::uint64_t epoch_;
    // Watermarks of the last prepared frame (identity, frame id, capture, clock).
    native_depth::SourceId last_source_;
    std::uint64_t last_frame_ = 0;
    std::int64_t last_capture_ = -1, last_now_ = -1;
    // pending_: prepared, build not yet called.
    bool has_last_ = false, pending_ = false, faulted_ = false;
    // Reused body buffer (capacity kMaxObjects reserved once).
    Objects body_;
    bool calibrated_ = false;
};

// Immutable startup wiring copied by Supervisor before init. Shared resources
// and callback_owner are retained through module shutdown/destruction; callbacks
// must capture their dependencies by value or keep them in callback_owner.
// The entry owns Profile/Status/GEN workers and stops Profile before stopping
// the Supervisor (which retires capture's native mailbox). No worker may borrow
// callback arguments beyond its synchronous call. No ROS fallback/double send.
// Callbacks must not call Supervisor lifecycle mutators (init/start/stop/configure).
struct SupervisorRnsContext {
    // Native depth mailbox the capture fills and the Profile thread reads.
    std::shared_ptr<native_depth::Mailbox> native_mailbox;
    // Journals the capture (arrivals) and inference (completions) threads write.
    std::shared_ptr<SourceJournal> depth_arrivals, inference_completions;
    std::uint64_t clock_epoch = 0;
    // service::CalibrationVerdict::calibrated() only; applied to Objects bodies.
    bool extrinsic_calibrated = false;
    // Keeps whatever the callbacks capture alive until the Supervisor is gone.
    std::shared_ptr<void> callback_owner;
    // Service monotonic clock (ns).
    std::function<std::int64_t()> now_ns;
    // Capture time of a frame (the declared link estimate in this delivery).
    std::function<bool(const native_depth::SourceId&, geometry::CaptureStamp&)> capture_stamp;
    // Hand the finished Objects body to its owner, which publishes it, or (paced release) takes
    // a copy that its release thread publishes later; false = failed. The body reference is
    // valid for the call only. A true return therefore means "handed over", not "on the wire".
    std::function<bool(const yolo::DetectionResult&, const geometry::CaptureStamp&,
        const Objects&, const FusedObjectsReport&)> objects_ready;
    // Optional: measured base_link geometry of one object from THIS inference
    // frame's raw depth sample (region = sampled bbox + mask; band = the raw
    // sample's selected [min, max] metres). Fills the FusedDetection base_*
    // fields; leaving base_geometry_valid false means unlocatable. selected_pixels, when not
    // null, is that raw sample's own list of selected pixels (DepthSampler, row-major
    // indices), so the geometry need not scan the region again (2026-10-06).
    std::function<void(const yolo::DetectionResult&, const cv::Rect& sampled_bbox, const cv::Mat& sampled_mask,
        float selected_min_m, float selected_max_m, const std::vector<std::uint32_t>* selected_pixels,
        FusedDetection&)> object_geometry;
    // All required members present, journals of the right kinds and distinct.
    bool valid() const noexcept;
};
} // namespace perception::rns
