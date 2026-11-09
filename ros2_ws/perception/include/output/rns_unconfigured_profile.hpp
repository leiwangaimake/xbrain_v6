/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_unconfigured_profile.hpp
 * Brief: Profile publisher for a service without configured geometry: every bin UNKNOWN.
 *
 * Description:
 * What problem this solves. The camera-to-body extrinsic may legitimately be all null (not yet
 * calibrated, 19 section 8.2: all null = to be calibrated). Without it there are no calibrated
 * rays, no ground and no bin attribution, yet the consumer still needs a live Profile stream at
 * the camera rate so it can tell "perception alive, nothing known" from "perception dead".
 * This consumer publishes exactly that: one Profile per new depth frame, correctly timed, with
 * every bin UNKNOWN.
 *
 * When it runs. The service selects it when perception.extrinsic_base_cam is not in transform
 * mode (perception_service_main.cpp, geometry_configured == false); Status then carries the
 * reason geometry_unconfigured. Wire metadata (z_pass, angles, range, bin count) still comes
 * from the configured perception.profile keys, so the message shape matches the configured
 * mode.
 *
 * What it still measures. The depth health scan (lower-half ROI) runs on every taken frame, so
 * Status.invalid_pixel_ratio stays meaningful without calibration.
 */
#pragma once
#include "output/rns_native_profile.hpp"

namespace perception::rns {

// Caller-selected wire metadata only. These values do not establish calibrated
// rays, extrinsics, ground, sample-to-bin attribution or a production domain.
// Zero initialization is deliberately unusable; there are no deployment values.
struct UnconfiguredProfileDomain {
    float z_pass_m = 0, angle_min_rad = 0, angle_step_rad = 0, range_max_m = 0;
    std::size_t n_bins = 0;
    std::uint64_t clock_epoch = 0;
};
// A single mode, named so logs and the Status adapter cannot confuse it with FitFailed.
enum class UnconfiguredProfileMode { GeometryUnconfigured };
// Outcome of one step(). Idle: no new frame. PutFinished: a put was attempted.
enum class UnconfiguredProfileStatus {
    Idle, SourceUnavailable, ClockRejected, ProfileUnavailable, PutFinished
};
// Everything one step() did, for the service's timing log and for tests.
struct UnconfiguredProfileTick {
    UnconfiguredProfileMode mode = UnconfiguredProfileMode::GeometryUnconfigured;
    UnconfiguredProfileStatus status = UnconfiguredProfileStatus::Idle;
    // A same-mode hint for the separately owned Status adapter. This consumer
    // never publishes Status or labels a missing configuration as FitFailed.
    std::string_view reason = "geometry_unconfigured";
    native_depth::Take take = native_depth::Take::Empty;
    bool take_called = false, depth_health_ran = false, depth_health_recorded = false;
    bool capture_mapping_ran = false, build_ran = false;
    bool geometry_ran = false, grid_ran = false; // always false in this explicit mode
    bool publish_clock_bracket_valid = false;
    native_depth::SourceId source;
    geometry::CaptureStamp capture;
    DepthHealthObservation depth_health;
    NativeProfilePut put;
    // Same-clock absolute samples; do not add these nested intervals. -1 means
    // the stage was not reached. Actual put timing lives in put.publish.audit.
    std::int64_t begin_ns = -1, taken_ns = -1, health_end_ns = -1,
        capture_checked_ns = -1, build_end_ns = -1, end_ns = -1;
};
// Monotonic counters since construction (owner thread only).
struct UnconfiguredProfileCounts {
    std::uint64_t polls = 0, taken = 0, source_rejected = 0, clock_rejected = 0,
        health_scanned = 0, health_recorded = 0, builds = 0, unavailable = 0,
        dispatches = 0, attempted = 0, accepted = 0, rejected = 0, unknown = 0;
    // accepted counts actual local API receipts, including a later clock fault.
    // It does not establish successful delivery, freshness or 20 Hz cadence.
};

// Explicit, isolated diagnostic mode; never a configured geometry substitute.
// Exactly ONE owner thread and ONE Mailbox for the object's whole lifetime.
// step() takes at most one latest real rawdepth frame; it never waits, retries
// a consumed identity, republishes an old body or calls NativeGeometry.
// Every bin is permanently UNKNOWN: all distances/heights/slope null, src/conf/
// terrain zero. blind and segmentation time are null, extrinsics false. Merely
// having valid depth pixels cannot establish G ownership of any angular bin.
//
// Caller supplies exact Mailbox capacity and one explicit MONOTONIC epoch for
// capture mapping and both consumer/publisher clocks. No SDK-ready-as-capture
// fallback, clock formula, TTL or production numeric domain is invented here.
// Capture identity must strictly advance in wire milliseconds across restarts.
// The host-call bracket detects inconsistent publish times, not an invisible
// clock-source substitution. A clock fault after put keeps its receipt.
//
// Raw health scans the owned frame with LowerHalfUncalibrated ROI before mapping
// and publishing, even for all-zero frames and failed capture mappings. Optional
// processed_depth uses the same clock domain, with one producer and one consumer;
// journal failures remain visible without suppressing the independent Profile.
// Serialize lifecycle changes and owner access through the entire synchronous
// put. Producer offers may run concurrently; callbacks and transport must obey
// their own bounded/nonblocking/allocation contract. Setup allocates raster and
// bins once; this consumer's hot path does not allocate.
class UnconfiguredProfileConsumer {
public:
    UnconfiguredProfileConsumer(native_depth::Mailbox&, std::size_t frame_capacity_pixels,
        UnconfiguredProfileDomain, NativeProfileClock, PublisherChannel&,
        SourceJournal* processed_depth = nullptr);
    UnconfiguredProfileConsumer(const UnconfiguredProfileConsumer&) = delete;
    UnconfiguredProfileConsumer& operator=(const UnconfiguredProfileConsumer&) = delete;
    // One poll: take at most one frame, scan it, stamp it, publish it. Never blocks;
    // the service loop waits on the mailbox between Idle ticks.
    const UnconfiguredProfileTick& step() noexcept;
    const UnconfiguredProfileTick& report() const noexcept { return report_; }
    UnconfiguredProfileCounts counts() const noexcept { return counts_; } // owner only
    // Raster plus bins, both allocated once in the constructor.
    std::size_t storage_bytes() const noexcept;
private:
    // Samples the consumer clock into out; false if negative or going backwards.
    bool mark(std::int64_t&) noexcept;
    // The taken frame's stream is still the one the mailbox is delivering.
    bool current_source() const noexcept;
    // Terminal paths of step(): set the status and count it.
    const UnconfiguredProfileTick& clock_failure() noexcept;
    const UnconfiguredProfileTick& source_failure() noexcept;
    native_depth::Mailbox& mailbox_;
    const UnconfiguredProfileDomain domain_;
    NativeProfileClock clock_;
    PublisherChannel& channel_;
    SourceJournal* processed_depth_;
    native_depth::Frame frame_;
    Profile body_;
    UnconfiguredProfileTick report_;
    UnconfiguredProfileCounts counts_;
    // Watermarks: every SourceId field must advance within a stream, the clock must
    // never go back, and each capture must land in a later wire millisecond.
    native_depth::SourceId last_source_;
    std::int64_t last_clock_ns_ = -1, last_capture_ms_ = -1;
};
} // namespace perception::rns
