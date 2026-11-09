/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_semantic.hpp
 * Brief: S channel: project recent object footprints into per-bin semantic blocking distances.
 *
 * Description:
 * What problem this solves. Geometry alone misses obstacles the depth camera cannot see, the
 * classic case being a glass door whose depth is invalid while the detector sees it (19 section
 * 3.4A). The slow line (Objects owner) writes each frame's measured base_link footprints into a
 * latest-wins slot (service/semantic_slot); the fast line (Profile thread) calls
 * SemanticProjector::process once per depth frame to turn them into a semantic distance d_sem
 * per bin. The Profile builders then inject d_sem only where it is strictly nearer than the
 * geometric block (src bit2, 11 section 3.1B.1).
 *
 * Why the inflation. Objects are older than the depth frame and no synchronized pose is used,
 * so each footprint is grown by m(r) = (v_ego_max + omega_max * r) * (t_capture - t_obj) +
 * m_jitter (19 section 3.4A): the robot may have moved that far since the objects were seen.
 * Growth only makes blocks nearer, which is the safe direction for a BLOCKED union.
 *
 * Boundaries. No class policy (what to yield to is RNS's decision), no SDK, no TF: the caller
 * supplies footprints already in base_link. Snapshots older than objects_stale_ms are not
 * injected. When the ground fit fails only plane-independent footprints are kept (19 A19-FIT-3).
 *
 * Why a publish guard. The age is checked here and again when the body is published a few
 * milliseconds later; a body that carries S and has passed objects_stale_ms by then is refused
 * as a whole, so the depth frame is lost. A snapshot whose age at the check is within
 * publish_guard_ns of the bound is therefore treated as Stale (no S, geometry-only body), which
 * is the safe side of the same rule: S is injected only while it is clearly inside its lifetime.
 */
#pragma once
#include "modules/traversability.hpp"
#include "output/rns_contract.hpp"
#include <array>
#include <string_view>

namespace perception::rns {
// Same vertex cap as an Objects footprint (11 section 3.1B.2).
inline constexpr std::size_t kSemanticVertices = 12;
// Storage/numerical support bound, NOT a physical detection or deployment gate.
inline constexpr double kSemanticCoordinateBound = 64.0;
// The Profile bin layout (copied from the geometry options), so d_sem bins line up
// exactly with the geometric bins they are compared with.
struct SemanticLayout {
    double angle_min_rad = 0, angle_step_rad = 0, range_max_m = 0;
    std::size_t bins = 0;
};
// Closed-sector conservative coverage (shared-edge evidence may enter both
// bins), not bin-centre ray sampling. Convex CCW polygon + square [-m,m]^2
// outer Minkowski envelope contains disk-radius-m inflation. Whole-footprint
// uniform margin is supplied by the caller using the farthest vertex.
// Bounded scratch, no allocations. Output optional distances cleared first
// for valid capacity<=kMaxBins. Invalid/overflow geometry returns false.
bool project_semantic_footprint(const Point* vertices, std::size_t count,
    const SemanticLayout&, double margin_m, std::optional<double>* distances,
    std::size_t capacity) noexcept;

// Spatial provenance supplied by the measurement producer, not inferred from
// class, polygon shape or the MeasuredBaseFootprint tag. Independent means the
// footprint's metric construction does not use a ground-plane estimate. The
// adapter must establish this from its actual data path; this enum alone is
// not a calibration/measurement certificate. Unknown fails closed on FitFailed.
enum class SemanticPlaneBasis { Unknown, IndependentMeasurement, GroundPlaneDependent };
// One footprint as the projector sees it; class_name is a view into the
// consumer's snapshot copy, valid only during process().
struct SemanticObject {
    std::int64_t track_id = -1;
    int class_id = -1;
    std::string_view class_name;
    std::array<Point, kSemanticVertices> footprint{};
    std::size_t vertices = 0;
    SemanticPlaneBasis plane_basis = SemanticPlaneBasis::Unknown;
};
// What a footprint was built from. Only MeasuredBaseFootprint is accepted: an image
// box plus depth is a view frustum, not ground occupancy (11 section 3.1B.2).
enum class SemanticMaterial { Unknown, MeasuredBaseFootprint, ImageBox };
// One Objects snapshot: the capture stamp of its inference frame (t_obj), and the
// revision of the base <- camera transform its footprints were built with, which
// must equal the projector's (service::kSemanticBaseRevision).
struct SemanticFrame {
    geometry::CaptureStamp stamp;
    std::uint64_t base_revision = 0;
    SemanticMaterial material = SemanticMaterial::Unknown;
    const SemanticObject* objects = nullptr;
    std::size_t count = 0;
};
// Robot motion envelope for the inflation (v_ego_max, omega_max, m_jitter from the
// profile config block) and the staleness bound (objects_stale_ms, 19 section 3.4A).
// publish_guard_ns: a snapshot older than max_age_ns - publish_guard_ns at the check is Stale
// (see the file header); 0 keeps the plain bound, and it may not exceed max_age_ns.
struct SemanticOptions {
    double v_max_mps = 0, omega_max_rps = 0, jitter_m = 0;
    std::int64_t max_age_ns = -1;
    std::uint64_t base_revision = 0;
    std::int64_t publish_guard_ns = 0;
};
// Result of one process() call. Only Fresh carries distances. Missing (no snapshot
// yet) and Stale (snapshot too old) are honest "no S" results. Future: the snapshot
// was captured after this depth frame. InvalidIdentity: order, clock or identity
// inconsistency. InvalidInput: bad object data or unusable depth. InactiveOrStale:
// the stream changed or this depth frame is not newer than the last one.
enum class SemanticStatus { Fresh, Missing, Stale, Future, InvalidIdentity,
    InvalidInput, InactiveOrStale, NotProcessed };
// Opt-in ONLY for an accepted current native frame whose fit failed. A failed
// frame/profile/clock is never a semantic source. This does not enable Q4 FREE,
// prior-plane classification, or complete degraded Profile publication.
enum class SemanticDepthPolicy { NormalOnly, NormalOrFitFailed };
// Audit of the last process() call: status, identities, ages and counts.
struct SemanticReport {
    SemanticStatus status = SemanticStatus::NotProcessed;
    // The depth frame processed and the Objects snapshot used (if any).
    geometry::CaptureStamp depth;
    std::optional<geometry::CaptureStamp> objects;
    std::int64_t checked_at_ns = -1, age_at_capture_ns = -1, age_at_check_ns = -1;
    std::size_t object_count = 0, projected_objects = 0, excluded_t_objects = 0,
                excluded_plane_basis_objects = 0,
                touched_bins = 0;
    double maximum_margin_m = 0;
    // Stale only because of the publish guard: the snapshot was still within max age at the check.
    bool stale_by_guard = false;
    // process() call counter; builders record it and later check it is unchanged,
    // proving the distances they merged come from the same call.
    std::uint64_t revision = 0;
};
// Owns the per-bin d_sem of the current depth frame. Serial: one thread (the
// Profile thread) calls process() and reads distances().
class SemanticProjector {
public:
    // geometry and one Mailbox owner outlive serial consumer. Input views are
    // borrowed immutable only during process(); resulting distances are owned.
    SemanticProjector(const geometry::NativeGeometry&, const SemanticOptions&,
                      SemanticDepthPolicy = SemanticDepthPolicy::NormalOnly);
    SemanticProjector(const SemanticProjector&) = delete;
    SemanticProjector& operator=(const SemanticProjector&) = delete;
    // Once per depth frame, after the geometry of that frame; frame may be null
    // (no snapshot). All distances are cleared first, so a failed call never
    // leaves the previous frame's boundaries behind.
    const SemanticReport& process(const native_depth::Mailbox&,
        const geometry::CaptureStamp&, std::int64_t now_mono_ns,
        const SemanticFrame*) noexcept;
    const SemanticReport& report() const noexcept { return report_; }
    const geometry::NativeGeometry& geometry() const noexcept { return geometry_; }
    bool bound_to(const native_depth::Mailbox& m) const noexcept { return mailbox_ == &m; }
    // Per-bin d_sem after inflation, nullopt where no footprint covers the bin.
    const std::vector<std::optional<double>>& distances() const noexcept { return distances_; }
    std::int64_t max_age_ns() const noexcept { return options_.max_age_ns; }
    // May the last result still be used at now? Missing/Stale count as usable
    // (S honestly absent); Fresh only while the snapshot is within max age.
    bool usable_at(std::int64_t now_mono_ns) const noexcept;
    // Heap bytes held, for the startup memory log.
    std::size_t storage_bytes() const noexcept;
private:
    bool usable_depth() const noexcept;
    void clear(SemanticStatus) noexcept;
    const geometry::NativeGeometry& geometry_;
    SemanticOptions options_;
    SemanticDepthPolicy depth_policy_;
    SemanticLayout layout_;
    // distances_: union (min) over objects; scratch_: one object's projection.
    std::vector<std::optional<double>> distances_, scratch_;
    SemanticReport report_;
    // Order watermarks: last depth frame, last snapshot, clock and check time.
    native_depth::SourceId last_;
    const native_depth::Mailbox* mailbox_ = nullptr;
    std::optional<geometry::CaptureStamp> last_objects_;
    std::uint64_t clock_epoch_ = 0;
    std::int64_t last_capture_ns_ = -1, last_check_ns_ = -1;
    std::uint64_t revision_ = 0;
};

// Select only a STRICTLY closer semantic boundary. Shorten original complete
// FREE cells using the same double far-edge formula (far<boundary), and recompute
// slope from retained cells. NULL FREE stays NULL. Does not set wire S itself.
bool restrict_semantic_prefix(const geometry::BinWalkResult&, const geometry::CellEvidence*,
    std::size_t cell_count, double blind_m, double dr_m, double semantic_m,
    geometry::BinWalkResult& output, bool& selected) noexcept;
} // namespace perception::rns
