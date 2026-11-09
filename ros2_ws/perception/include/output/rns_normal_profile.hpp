/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_normal_profile.hpp
 * Brief: Normal-fit geometry (+ optional S) -> one ProfileMsg body per depth frame.
 *
 * Description:
 * What problem this solves. The geometry owners (NativeGeometry, NormalGrid) describe one depth
 * frame as per-bin evidence and a radial walk result. NormalProfileBuilder converts that into
 * the 11 section 3.1B.1 Profile body, bin by bin, optionally tightening blocks with the S
 * channel (rns_semantic), and publishes it at most once. Frames whose ground fit FAILED are not
 * handled here but by the withdrawn builder (rns_withdrawn_profile).
 *
 * Service wiring. NativeProfile (rns_native_profile) owns one builder: with the T owner when the
 * capture session's colour projection is available (plan 2f, wired since 2026-09-30), with the S
 * projector when S is configured, otherwise geometry-only. A fresh T sample puts its
 * segmentation time into t_seg. While the near-side rule (B3-1) is on hold, a fresh sample with
 * a positive age margin cannot support the cells next to the blind range, so FREE ends at
 * blind_near_m and bit0 stays clear; without T, t_seg is null and bit0 clear (19 section 3.3
 * degraded rule).
 *
 * Rounding rule. Every double -> float32 wire conversion goes the safe way: distances toward
 * less FREE and an earlier block, the blind range outward. A frame without a ground reference
 * is still published, with FREE, T backing and slope withdrawn (19 A19-NULL-1, reply B6-2).
 */
#pragma once
#include "modules/traversability.hpp"
#include "output/rns_publisher.hpp"
#include "output/rns_semantic.hpp"

namespace perception::rns {
// Pure bounded conversion. Failure resets output. Count bounds are checked.
// Distances round toward less FREE/earlier BLOCKED; blind rounds outward.
// Measured slope uses ceil degrees, null for no nonempty FREE segment.
// Geometry samples include valid returns and valid expected-plane pierce rays.
// Conf uses valid_returns / expected_rays (saturating floor to uint8).
// semantic_selected accepts an already double-selected, heightless S boundary;
// it is NOT inferred from wire distances. No pose/calibration override here.
bool normal_profile_bin(const geometry::GridBinEvidence& evidence,
    const geometry::BinWalkResult& selected, double blind_m, double range_m,
    bool fresh_t, Bin& output, bool semantic_selected = false) noexcept;
// Normal fit, but no usable ground reference for traversal. Explicit G-only
// conversion: no fabricated blind value and no FREE/T/NEG/slope. Retains
// measured G/conf, an independently classified positive boundary + height, and
// -- with semantic_selected -- an independent S boundary, which does not depend
// on a ground reference and may stand alone in a bin with no geometry sample
// (19 A19-NULL-1 / A19-FIT-3, reply batch 6 B6-2). It does not accept FitFailed
// evidence (that is the withdrawn adapter) or a T-backed prefix.
bool no_ground_profile_bin(const geometry::GridBinEvidence& evidence,
    const geometry::BinWalkResult& selected, double range_m, Bin& output,
    bool semantic_selected = false) noexcept;

// ONE bin exactly as NormalProfileBuilder converts it (reply batch 7 / 19
// A19-SRC-1: the golden runs this very function, not a copy of the loop).
// semantic_m (already fresh, projected and margin-inflated) first restricts
// the prefix -- only a NEARER S boundary is selected, a missing geometric
// boundary counts as +inf (19 section 3.4A, 11 section 3.1B.1 v2.2) -- then
// the wire bin is converted. blind_m == nullopt means no ground reference
// (no_ground_profile_bin); cells may then be null with cell_count 0.
bool profile_bin_with_semantic(const geometry::GridBinEvidence& evidence,
    const geometry::BinWalkResult& prefix, const geometry::CellEvidence* cells, std::size_t cell_count,
    std::optional<double> blind_m, double dr_m, double range_m, bool fresh_t,
    std::optional<double> semantic_m, Bin& output) noexcept;

// Build outcome; only Ready has a body. InactiveOrStale: stream changed, frame not
// newer, or an owner moved on before publish. InvalidEvidence: owner results that
// the wire rules cannot express faithfully.
enum class NormalProfileStatus { Ready, GeometryUnavailable, NoGroundReference,
    InactiveOrStale, InvalidIdentity, InvalidEvidence, NotBuilt };
// Audit of the last build: statuses of the inputs and per-source bin counts.
struct NormalProfileReport {
    NormalProfileStatus status = NormalProfileStatus::NotBuilt;
    geometry::CaptureStamp capture;
    geometry::TStatus t_status = geometry::TStatus::Missing;
    SemanticStatus semantic_status = SemanticStatus::Missing;
    bool near_support_missing = false;
    bool no_ground_reference = false;
    std::size_t geometry_bins = 0, t_bins = 0, semantic_bins = 0, blocked_bins = 0;
};
// Outcome of publish(): dispatched = handed to the channel (at most once per body).
struct NormalProfilePut {
    NormalProfileStatus status = NormalProfileStatus::NotBuilt;
    bool dispatched = false;
    PublishResult publish;
};

class NormalProfileBuilder {
public:
    // Explicit geometry-only mode. No T owner, colour projection, mask wait or
    // invented T sample. Uses NormalGrid's unchanged G-only traversal; t_seg
    // stays null and kT stays clear. This is not a way to ignore a configured
    // but invalid T owner: the other constructors retain their strict checks.
    // Also accepts normal-fit NoGroundReference, preserving null or measured
    // range-edge blind and positive obstacles while explicitly withdrawing FREE.
    explicit NormalProfileBuilder(const geometry::NormalGrid&);
    // Exact immutable owners and mailbox outlive this single-thread consumer.
    // Invoke geometry -> grid -> T.process (also for missing masks) -> build.
    // Calibration verdict is deliberately false; this constructor does not
    // silently discard the T owner. A NoGroundReference frame is published with
    // FREE, the T backing and slope withdrawn (bit0 never set without a
    // verifiable FREE). FitFailed is a separate adapter.
    // Angle min/step MUST already round-trip through float32 exactly; the
    // actual geometry bin domain must equal the serialized domain. Canonicalize
    // configuration before constructing Geometry, never only in this adapter.
    NormalProfileBuilder(const geometry::NormalGrid&,
                         const geometry::NormalTraversability&);
    // Normal-only optional S owner. Process S (including missing/stale) before
    // build. Inputs are measured base footprints; neither RNS class policy nor
    // fallback FREE is introduced. All owners must remain serially owned.
    NormalProfileBuilder(const geometry::NormalGrid&,
                         const geometry::NormalTraversability&, const SemanticProjector&);
    // S owner WITHOUT a T owner (19 section 3.4A: the S channel is carried by
    // src bit2 and is independent of the segmentation channel). Geometry-only
    // traversal as in the G-only constructor: t_seg stays null, kT stays clear,
    // and a geometry-only FREE prefix can still be shortened by a strictly
    // closer S boundary. This is not a way to drop a configured T owner: with T
    // configured the other constructors keep their checks. A NoGroundReference
    // frame is still published (FREE/T/slope withdrawn, measured obstacles and
    // independent S kept) -- reply batch 6, B6-2.
    NormalProfileBuilder(const geometry::NormalGrid&, const SemanticProjector&);
    // Same, for an adapter that decides at construction time whether this
    // deployment has an S owner: nullptr is exactly the G-only constructor.
    NormalProfileBuilder(const geometry::NormalGrid&, const SemanticProjector*);
    // For an adapter that decides at construction time whether this deployment
    // has a T owner and/or an S owner (plan 2f). A null T is exactly the
    // (grid, S*) constructor; a non-null T is (grid, T) or (grid, T, S) with all
    // their checks.
    NormalProfileBuilder(const geometry::NormalGrid&, const geometry::NormalTraversability*,
                         const SemanticProjector*);
    NormalProfileBuilder(const NormalProfileBuilder&) = delete;
    NormalProfileBuilder& operator=(const NormalProfileBuilder&) = delete;
    // Wire extrinsic_calibrated for subsequently built bodies. The ONLY
    // legitimate source is service::CalibrationVerdict::calibrated() (19
    // section 7.3); default false. Not a certification performed here.
    void set_extrinsic_calibrated(bool calibrated) noexcept { calibrated_ = calibrated; }
    // Build the body for this capture from the owners' current results (geometry,
    // grid, T, S must already have processed this frame).
    const NormalProfileReport& build(const native_depth::Mailbox&,
                                     const geometry::CaptureStamp&) noexcept;
    const NormalProfileReport& report() const noexcept { return report_; }
    // nullptr after ANY failed build. Borrowed until next build/publish call;
    // use publish() to revalidate identity after intervening upstream calls.
    const Profile* body() const noexcept;
    // At most once per successfully built frame, even if transport rejects.
    // Revalidates stream + current owner snapshots BEFORE any put. Lifecycle
    // and these owners must be externally serialized through the put call.
    // With S configured, caller supplies actual current MONOTONIC ns from the
    // SAME clock_epoch. A stale built S requires rebuilding, never silently
    // dropping an already selected obstacle. In the service NativeProfile calls
    // it with the Profile PublisherChannel and the service monotonic clock.
    NormalProfilePut publish(const native_depth::Mailbox&, PublisherChannel&,
                             std::int64_t now_mono_ns = -1) noexcept;
    // Heap bytes held (the bin vector), for the startup memory log.
    std::size_t storage_bytes() const noexcept;
private:
    void clear(NormalProfileStatus) noexcept;
    // Are the owner results the body was built from still current? (see .cpp)
    bool current(const native_depth::Mailbox&) const noexcept;
    const geometry::NormalGrid& grid_;
    const geometry::NormalTraversability* t_ = nullptr;
    const SemanticProjector* semantic_ = nullptr;
    Profile body_;
    NormalProfileReport report_;
    // Snapshots of the T report and S revision used by the body, compared again
    // before publishing.
    geometry::TReport t_snapshot_;
    std::uint64_t semantic_revision_ = 0;
    // Order watermarks: last frame identity, clock epoch and capture ms.
    native_depth::SourceId last_;
    std::uint64_t epoch_ = 0;
    Millis last_capture_ms_ = -1;
    // dispatched_: this body was already handed to the channel.
    bool dispatched_ = false;
    bool calibrated_ = false;
};
} // namespace perception::rns
