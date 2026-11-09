/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_depth_health.hpp
 * Brief: Native-raster ROI health observations, not calibration or FREE evidence.
 *
 * Description:
 * What problem this solves. Status.invalid_pixel_ratio (19 section 5.1) is the share of depth
 * pixels without a measurement inside the ground ROI, over the last 1 s of processed frames.
 * Two pieces produce it: observe_depth_health() counts one native depth frame on the Profile
 * thread, and DepthHealthWindow aggregates those counts on the Status thread.
 *
 * ROI. Calibrated: the static prior-plane ROI of the native geometry table (geometry_setup.cpp).
 * Uncalibrated: the lower half of the raster, and Status then carries roi_fallback
 * (11 section 3.1B.3). The ROI is a fixed mask, not a per-frame ground segmentation, so the
 * ratio measures sensor dropout, not what is in front of the dog.
 *
 * Why this is a health number only. A pixel counted here says nothing about free space: the
 * Profile pipeline makes its own per-pixel decisions. Nothing reads this ratio back into a
 * geometry or FREE decision.
 *
 * Null versus 1. A window without any processed frame reports null (no data), an observed
 * all-invalid window reports 1.0 (19 section 5.1 v1.7: never the old value, 0 or 1 invented).
 */
#pragma once
#include "modules/native_depth.hpp"
#include <optional>
#include <vector>

namespace perception::rns {
// Per-frame result. Only Ready carries counts; the others count nothing.
enum class DepthHealthCode { Ready, InvalidFrame, InvalidRoi, EmptyRoi };
// Which ROI produced the counts; Status cross-checks it against the calibration verdict.
enum class HealthRoiKind { LowerHalfUncalibrated, CalibratedStaticMask };
// Borrowed description of the ROI. A default value is the uncalibrated lower half.
struct HealthRoiView {
    HealthRoiKind kind = HealthRoiKind::LowerHalfUncalibrated;
    // Borrowed binary native-raster mask. Caller establishes calibrated static
    // ray/prior-plane ROI + calibration-record provenance; this is not a
    // current-frame ground segmentation. No pointer survives the call.
    const std::uint8_t* mask = nullptr;
    std::size_t mask_pixels = 0;
    std::uint32_t width = 0, height = 0;
};
// One frame's counts, tagged with the frame's source identity so the window can reject
// duplicates and reordering.
struct DepthHealthObservation {
    DepthHealthCode code = DepthHealthCode::InvalidFrame;
    native_depth::SourceId source;
    HealthRoiKind roi_kind = HealthRoiKind::LowerHalfUncalibrated;
    std::size_t roi_pixels = 0, invalid_pixels = 0;
    std::optional<double> invalid_ratio;
};
// No allocation; zero native depth code is invalid, matching current native
// geometry policy. No metric-distance cutoff or missing mask is silently
// substituted. Lower half = rows [height/2,height), including middle row for
// odd height (implementation convention). Uncalibrated mode rejects a supplied
// mask, so a wrong calibration flag cannot silently hide a stale ROI.
// The caller must serialize frame/lifecycle access and bind this observation
// to its real mailbox/generation and observation age before status publication.
// Frame stores SourceId, not owning mailbox address: no owner proof is claimed.
DepthHealthObservation observe_depth_health(const native_depth::Frame&, HealthRoiView) noexcept;

// Duplicate/OutOfOrder are counted and skipped; Invalid/CapacityExceeded latch the window invalid.
enum class DepthWindowCode { Accepted, Duplicate, OutOfOrder, Invalid, CapacityExceeded };
// Snapshot of the last 1 s. Optional fields are empty exactly when processed_frames == 0.
struct DepthHealthWindowReport {
    bool valid = true, ever_observed = false;
    std::int64_t now_ms = -1;
    std::size_t processed_frames = 0;
    std::uint64_t roi_pixels = 0, invalid_pixels = 0;
    std::optional<double> invalid_ratio;
    std::optional<HealthRoiKind> roi_kind;
    std::optional<native_depth::SourceId> latest_source;
    std::uint64_t duplicates = 0, out_of_order = 0, invalid = 0, overflow = 0;
};
// Single owner, fixed capacity; no allocation after construction. Integration
// serializes observation/snapshot handoff and establishes actual frame owner.
// A sample is timestamped when depth processing completes, NOT at SDK arrival.
// Window (now-1000, now] contains unique processed frames. Ratio is SUM invalid
// pixels / SUM ROI pixels, never the unweighted mean of frame ratios. Empty
// windows are null; all-invalid observed windows are 1. Snapshot is nonconsuming.
// ROI provenance/kind must remain coherent within one window. Invalid samples,
// clock rollback and overflow latch invalid; replay counters do not. No reset
// masks a bad run. Caller creates a separately identified owner after repair.
class DepthHealthWindow {
public:
    // 1 s: the Status heartbeat period, so each beat reports only its own second.
    static constexpr std::int64_t kWindowMs = 1000;
    // Upper bound on the ring size accepted by the constructor (the service uses 2048).
    static constexpr std::size_t kMaxCapacity = 1000000;
    DepthHealthWindow(std::int64_t startup_ms, std::size_t capacity);
    DepthHealthWindow(const DepthHealthWindow&) = delete;
    DepthHealthWindow& operator=(const DepthHealthWindow&) = delete;
    // processed_ms: when depth processing completed (the sample time); ingest_ms: when
    // the Status thread drained it from the journal. Both on the monotonic clock.
    DepthWindowCode observe(const DepthHealthObservation&, std::int64_t processed_ms,
                            std::int64_t ingest_ms) noexcept;
    // Nonconsuming; now_ms must not go backwards (a rollback latches invalid).
    DepthHealthWindowReport snapshot(std::int64_t now_ms) noexcept;
private:
    // One ring entry: the frame's counts and its processing-complete time.
    struct Event { DepthHealthObservation observation; std::int64_t processed_ms = -1; };
    bool clock(std::int64_t now) noexcept;
    void prune(std::int64_t now) noexcept;
    DepthWindowCode reject(DepthWindowCode) noexcept;
    // Fixed-size ring allocated once in the constructor; [begin_, begin_ + size_) is live.
    std::vector<Event> events_;
    std::size_t begin_ = 0, size_ = 0;
    std::int64_t startup_, last_clock_, last_processed_ = -1;
    // Last accepted source, for duplicate / reorder detection.
    native_depth::SourceId last_source_;
    bool valid_ = true, ever_ = false;
    std::uint64_t duplicates_ = 0, out_of_order_ = 0, invalid_ = 0, overflow_ = 0;
};
}  // namespace perception::rns
