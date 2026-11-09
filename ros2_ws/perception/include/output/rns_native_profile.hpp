/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_native_profile.hpp
 * Brief: The Profile fast line: one step() = latest depth frame -> geometry -> grid -> S -> build -> publish.
 *
 * Description:
 * What problem this solves. ProfileMsg must follow the depth camera at frame rate and must never
 * wait for inference (19 section 1.1: the fast line). NativeProfileConsumer is the loop body of
 * the service's Profile thread (service::run_profile_steady, profile_steady_loop.cpp): each step() takes
 * the newest depth frame from the mailbox, runs the geometry owners on it, routes it to the
 * normal or the withdrawn builder, and publishes at most one body.
 *
 * Routes. Normal fit -> NormalProfileBuilder. Normal fit without a ground reference -> the
 * same builder, published with FREE/T/slope withdrawn (19 A19-NULL-1, reply B6-2). Fit failed
 * -> WithdrawnProfileBuilder (A19-FIT-3: FREE and plane-dependent claims withdrawn, an
 * independent S block kept). Anything else (invalid source, clock or evidence) publishes
 * nothing; it is never turned into a valid-looking withdrawal frame.
 *
 * Time. Every stage end is stamped on the service monotonic clock; the stamps feed the Profile
 * timing log and check the publisher's put audit for consistency.
 */
#pragma once
#include "output/rns_normal_profile.hpp"
#include "output/rns_withdrawn_profile.hpp"
#include "output/rns_source_journal.hpp"

namespace perception::rns {
// A caller-supplied exposure-midpoint mapper and MONOTONIC clock from one
// explicitly identified epoch. In this delivery the mapper is the declared link
// estimate (EstimatedCaptureClock: host-ready minus latency_fallback_ms,
// 19 section 2.4), not a measured exposure midpoint. No fallback to SDK-ready/wall/device time.
// Both callbacks must be bounded, nonblocking, noexcept and allocation-free.
// Checking these values cannot certify calibration or clock provenance.
struct NativeProfileClock {
    const void* context = nullptr;
    std::uint64_t epoch = 0;
    std::int64_t (*now_ns)(const void*) noexcept = nullptr;
    bool (*capture)(const void*, const native_depth::Metadata&,
                    geometry::CaptureStamp&) noexcept = nullptr;
};
// The measured-footprint snapshot this consumer offers to its S owner for one
// depth frame (19 section 3.4A latest-wins slot). Called on the consumer thread
// between the grid and the build, at most once per taken frame; it must be
// bounded, nonblocking and noexcept, and the returned frame must stay valid
// until the call to step() returns. nullptr = nothing published yet (the S
// owner then reports Missing and the profile stays geometry-only).
struct SemanticFrameSource {
    const SemanticFrame* (*latest)(void*, const geometry::CaptureStamp&) noexcept = nullptr;
    void* context = nullptr;
};
// The raw traversable mask this consumer offers to its T owner for one depth
// frame (19 section 3.1 P19-1: the fast line's only dependency on the slow line
// is reading one latest-wins slot). Called on the consumer thread after the
// grid, at most once per taken frame on the normal and no-ground routes; it
// must be bounded, nonblocking and noexcept, and the returned mask must stay
// valid and unchanged until step() returns. nullptr = no mask this frame may
// use (the T owner then reports Missing and the profile stays G(+S)).
struct TFrameSource {
    const geometry::RawTMask* (*latest)(void*, const geometry::CaptureStamp&) noexcept = nullptr;
    void* context = nullptr;
};
// Outcome of one step(). Idle: no new frame (or the mailbox was busy).
// ClockRejected: a clock read went backwards or the capture mapping was
// inconsistent. PutFinished: a body was handed to the channel.
enum class NativeProfileStatus { Idle, SourceUnavailable, ClockRejected,
    GeometryUnavailable, GridUnavailable, ProfileUnavailable, PutFinished };
// Which builder handled the frame (see the file header).
enum class NativeProfileRoute { None, Normal, NoGroundReference, FitFailedWithdrawal };
struct NativeProfilePut {
    bool dispatched = false;
    PublishResult publish;
};
// Everything one step() did, for the Profile thread's counters and timing log.
struct NativeProfileTick {
    NativeProfileStatus status = NativeProfileStatus::Idle;
    native_depth::Take take = native_depth::Take::Empty;
    bool take_called = false, geometry_ran = false, grid_ran = false, build_ran = false;
    bool publish_clock_bracket_valid = false; // false also when no put attempted
    bool depth_health_ran = false, depth_health_recorded = false;
    native_depth::SourceId source;
    geometry::CaptureStamp capture;
    geometry::GeometryStatus geometry_status = geometry::GeometryStatus::InvalidFrame;
    geometry::GridStatus grid_status = geometry::GridStatus::GeometryUnavailable;
    NativeProfileRoute route = NativeProfileRoute::None;
    NormalProfileStatus normal_status = NormalProfileStatus::NotBuilt;
    WithdrawnProfileStatus withdrawn_status = WithdrawnProfileStatus::NotBuilt;
    // NotProcessed unless an S owner is configured and this frame reached it.
    SemanticStatus semantic_status = SemanticStatus::NotProcessed;
    std::size_t semantic_bins = 0;
    // S freshness bookkeeping: Stale only because of the publish guard; and a snapshot that was
    // Fresh at the check but past max age at the publish entry, so the body was not put.
    bool semantic_stale_by_guard = false, semantic_expired_at_publish = false;
    // T owner outcome; t_ran is false unless a T owner is configured and this
    // frame reached it (normal and no-ground routes). t_bins: bins whose FREE
    // prefix carries the T backing (src bit0) in the built body.
    bool t_ran = false, t_near_support_missing = false;
    geometry::TStatus t_status = geometry::TStatus::Missing;
    std::int64_t t_age_ns = -1;
    std::size_t t_bins = 0;
    bool free_withdrawn = false;
    // Same-frame hints for a separately owned Status reason adapter. These do
    // NOT publish a Status or attest that a heartbeat integration exists.
    std::array<std::string_view, 2> reasons{};
    NativeProfilePut put;
    // Same-clock absolute timestamps, not additive nested durations. -1 means
    // not reached. put.publish.audit separately names actual put API timing.
    std::int64_t begin_ns = -1, taken_ns = -1, capture_checked_ns = -1,
        geometry_end_ns = -1, grid_end_ns = -1, t_end_ns = -1, semantic_end_ns = -1, build_end_ns = -1, end_ns = -1;
    // With an S owner: the instant its age was checked and the publish entry instant (the two
    // ends of the interval the publish guard has to cover).
    std::int64_t semantic_check_ns = -1, publish_entry_ns = -1;
};
struct NativeProfileCounts {
    // accepted = local API receipt, including a later return-clock fault;
    // neither clock-valid success nor delivery/receiver cadence is implied.
    std::uint64_t polls = 0, taken = 0, clock_rejected = 0, unavailable = 0,
        dispatches = 0, accepted = 0;
};

// The fast-path consumer (geometry-only, or with an S owner); it loads no
// configuration, the service constructs it with its owners.
// Exactly one owner thread; this object and geometry/grid must not be shared
// with another consumer. Constructor allocates one raster and bounded body.
// step() tries ONE latest frame, never waits for capture, inference, T or S,
// never sleeps, retries puts or republishes an old body. Lifecycle changes and
// owner access must be externally serialized through the entire put; producers
// may offer newer frames concurrently. The transport has its OWN latency and
// allocation contract: a slow put is not hidden by this component.
// channel's PublishClock MUST share clock.epoch's MONOTONIC domain. The
// millisecond put audit is checked against the nanosecond host-call bracket;
// this detects inconsistent numbers, NOT an unobservable clock-source swap.
//
// Inputs: depth, plus the T mask and the S snapshot when those owners are given
// (plan 2f: the T owner does the colour projection; this consumer only calls it).
// FitFailed -> complete r6 withdrawal, never prior-plane FREE/negative claims.
// Normal fit without ground reference -> retain only valid positive geometry.
// Invalid source/profile/clock is NOT converted into a valid withdrawal frame.
// NoGroundReference preserves a measured blind bound if one exists at range;
// otherwise blind=null. Status/Objects run elsewhere.
class NativeProfileConsumer {
public:
    // extrinsic_calibrated: see NormalProfileBuilder::set_extrinsic_calibrated.
    // With a processed-depth journal the health ROI kind must agree with it
    // (calibrated <=> CalibratedStaticMask), as the Status fields require.
    NativeProfileConsumer(native_depth::Mailbox&, geometry::NativeGeometry&,
        geometry::NormalGrid&, NativeProfileClock, PublisherChannel&,
        SourceJournal* processed_depth = nullptr, HealthRoiView health_roi = {},
        bool extrinsic_calibrated = false);
    // With an S owner (19 section 3.4A). The projector must own the same
    // geometry; this consumer processes it for every taken frame, including
    // frames with no snapshot (Missing) or an expired one (Stale), so the S
    // channel is never silently skipped. A NoGroundReference frame is still
    // published, FREE withdrawn and independent S kept (reply B6-2).
    // A null projector is exactly the constructor above (no S owner).
    NativeProfileConsumer(native_depth::Mailbox&, geometry::NativeGeometry&,
        geometry::NormalGrid&, NativeProfileClock, PublisherChannel&,
        SemanticProjector*, SemanticFrameSource,
        SourceJournal* processed_depth = nullptr, HealthRoiView health_roi = {},
        bool extrinsic_calibrated = false);
    // With a T owner (19 section 3.3, plan 2f) and optionally an S owner. The T
    // owner must own the same geometry and grid; it is processed for every taken
    // frame on the normal and no-ground routes, including frames with no mask
    // (Missing), an old one (Stale) or none yet, so T is never silently skipped.
    // Order per frame: geometry -> grid -> T -> S -> build. The fit-failed
    // withdrawal route does not consult T (its t_seg stays null as 19 v1.7 says; the lead ruled
    // on 2026-10-08, issues.md 3.2(3), that it follows r4 -- t_seg non-null, bit0 = 0 -- with the
    // enable-T contract batch, not before). A null T owner is exactly the
    // S-owner constructor above.
    NativeProfileConsumer(native_depth::Mailbox&, geometry::NativeGeometry&,
        geometry::NormalGrid&, NativeProfileClock, PublisherChannel&,
        geometry::NormalTraversability*, TFrameSource,
        SemanticProjector*, SemanticFrameSource,
        SourceJournal* processed_depth = nullptr, HealthRoiView health_roi = {},
        bool extrinsic_calibrated = false);
    NativeProfileConsumer(const NativeProfileConsumer&) = delete;
    NativeProfileConsumer& operator=(const NativeProfileConsumer&) = delete;
    // One non-blocking iteration; the caller waits for new frames between calls.
    const NativeProfileTick& step() noexcept;
    const NativeProfileTick& report() const noexcept { return report_; }
    NativeProfileCounts counts() const noexcept { return counts_; } // owner only
    std::size_t storage_bytes() const noexcept;
private:
    void validate(const NativeProfileClock&, PublisherChannel&, bool extrinsic_calibrated);
    // Read the monotonic clock into a stage stamp; false if it went backwards.
    bool mark(std::int64_t&) noexcept;
    const NativeProfileTick& clock_failure() noexcept;
    // Was this frame's S snapshot Fresh at its check and past max age at now?
    bool semantic_expired_at(std::int64_t now_ns) const noexcept;
    native_depth::Mailbox& mailbox_;
    geometry::NativeGeometry& geometry_;
    geometry::NormalGrid& grid_;
    NativeProfileClock clock_;
    PublisherChannel& channel_;
    // Same-clock single-producer optional health channel, owned by caller.
    // Calibrated ROI mask must outlive this object; no current-frame fit used.
    SourceJournal* processed_depth_;
    HealthRoiView health_roi_;
    SemanticProjector* semantic_ = nullptr;   // optional S owner, same geometry
    SemanticFrameSource semantic_source_;
    geometry::NormalTraversability* t_ = nullptr;  // optional T owner, same geometry/grid
    TFrameSource t_source_;
    // The one depth raster, reused for every taken frame.
    native_depth::Frame frame_;
    // Normal route and fit-failed route builders (see the file header).
    NormalProfileBuilder builder_;
    WithdrawnSemanticEvidence withdrawn_evidence_;
    WithdrawnProfileBuilder withdrawn_builder_;
    NativeProfileTick report_;
    NativeProfileCounts counts_;
    // Watermarks: last clock read and last accepted capture time.
    std::int64_t last_clock_ns_ = -1, last_capture_ns_ = -1;
};
} // namespace perception::rns
