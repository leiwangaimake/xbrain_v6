/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: traversability.hpp
 * Brief: Normal-frame raw traversability evidence.
 *
 * Description:
 * T channel (19 section 3.3): the class-92 traversable mask restricts each bin's geometric FREE
 * prefix. Wired into the service since 2026-09-30: the Profile thread owns one NormalTraversability
 * when the capture session's colour projection passes the registration gate (log line
 * PROFILE_T_READY); the T producer supplies the raw class-92 union of each inference frame. While
 * the near-side rule (B3-1) is on hold, a fresh mask with a positive age margin cannot support the
 * cells next to the blind range, so FREE ends there (near_support_missing).
 */

#pragma once
#include "modules/normal_grid.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace perception::geometry {
struct TOptions {
    double tau = 0, v_max_mps = 0, omega_max_rps = 0, jitter_m = 0;
    std::int64_t max_age_ns = -1;
};
bool valid_t_options(const TOptions&) noexcept;
// Nonnegative binary64 inputs; outward-rounded operation bounds. Invalid
// arguments return +inf (never a falsely zero/negative safety margin).
double t_margin_upper_bound(const TOptions&, std::int64_t age_ns, double far_m) noexcept;
struct TCount { std::uint32_t ground = 0, traversable = 0; };
struct TLayout {
    std::size_t bins = 0, cells = 0, stride = 0;
    double blind_m = 0, dr_m = 0, angle_step_rad = 0;
};
// Conservative rectangle envelope: outward M=(v+omega*far)*age+jitter,
// radial ceil(M/dr), angular ceil(theta_upper/step), where theta_upper uses
// asin(q)<=pi*q/2 for q=M/near with outward arithmetic. M>=near lacks support.
// M==0 uses zero radii. This is an explicit conservative implementation choice.
// Outside observed domain is false; no wrap, edge replication or silent clip.
// With valid bounded buffers, clears all output slots on failure. Unbounded
// capacity or invalid/aliased pointers are rejected without unbounded access.
// Prefix scratch requires
// (bins+1)*(stride+1) uint32 values. All pointers denote independent buffers.
// O(capacity+bins*stride), bounded caller-owned buffers; no allocation.
bool erode_traversability(const TLayout&, const TOptions&, std::int64_t age_ns,
    const TCount*, std::size_t capacity, std::uint8_t* raw, std::uint8_t* eroded,
    std::uint32_t* prefix, std::size_t prefix_capacity) noexcept;

// Restrict an already validated normal geometric prefix with fresh eroded T.
// Never extends it; keeps independent obstruction/height. No T -> original
// geometry with T flag false. Fresh but empty/false T is NOT "no T".
BinWalkResult restrict_normal_prefix(const BinWalkResult&, const CellEvidence*,
    const std::uint8_t* eroded, std::size_t count, double blind_m, double dr_m,
    bool fresh_t) noexcept;

struct CaptureStamp {
    native_depth::SourceId source;
    // Caller-supplied exposure midpoint in one calibrated MONOTONIC epoch.
    // Never inferred from SourceId.host_ready_mono_ns by this component.
    std::uint64_t clock_epoch = 0;
    std::int64_t capture_mono_ns = -1;
};
// The same RGB-D frame set: every identity field equal.
inline bool same_capture_source(const native_depth::SourceId& a, const native_depth::SourceId& b) noexcept {
    return a.stream_id == b.stream_id && a.sequence == b.sequence &&
        a.color_index == b.color_index && a.depth_index == b.depth_index &&
        a.color_device_us == b.color_device_us && a.depth_device_us == b.depth_device_us &&
        a.host_ready_mono_ns == b.host_ready_mono_ns;
}
// The T identity rule: the mask's frame set is the depth frame set itself, or strictly older on
// every index and clock of the same stream. Shared by NormalTraversability and the service's T
// slot, so the slot never hands the owner a mask the owner would call InvalidIdentity.
inline bool source_older_or_same(const native_depth::SourceId& mask, const native_depth::SourceId& depth) noexcept {
    if (mask.stream_id != depth.stream_id || mask.sequence == 0 || mask.sequence > depth.sequence ||
        mask.host_ready_mono_ns < 0) return false;
    if (mask.sequence == depth.sequence) return same_capture_source(mask, depth);
    return mask.color_device_us > 0 && mask.depth_device_us > 0 &&
        mask.color_index < depth.color_index && mask.depth_index < depth.depth_index &&
        mask.color_device_us < depth.color_device_us && mask.depth_device_us < depth.depth_device_us &&
        mask.host_ready_mono_ns <= depth.host_ready_mono_ns;
}
enum class MaskMaterial { Unknown, RawTraversable, FusedOrImageEroded };
struct RawTMask {
    CaptureStamp stamp;
    MaskMaterial material = MaskMaterial::Unknown;
    std::uint64_t projection_revision = 0;
    std::uint32_t width = 0, height = 0;
    std::size_t stride_bytes = 0, bytes = 0;
    const std::uint8_t* pixels = nullptr; // binary 0 or 1; immutable for process()
};
struct ColorProjection {
    std::uint32_t width = 0, height = 0;
    std::uint64_t revision = 0;
    const void* context = nullptr;
    // Projects CURRENT base-frame Ground point to RAW T color raster. Callback
    // must implement its actual distortion/calibration; false is no T sample.
    // Output pixel domain: [0,width) x [0,height), sampled by floor, no clamp.
    bool (*project)(const void*, Vec3, double& u, double& v) noexcept = nullptr;
    // Optional split of `project` into its rigid part and the rest (all three set, or all null):
    // q = rotation * p + translation (row-major), then project_camera(context, q, u, v). With it
    // the owner precomputes rotation * ray for every native pixel once and needs one
    // multiply-add per axis per ground point (optimization round 2, 2026-10-04). Only the
    // floating-point rounding differs from `project` (well below 1e-9 px).
    const std::array<double, 9>* rotation = nullptr;
    const Vec3* translation = nullptr;
    bool (*project_camera)(const void*, Vec3, double& u, double& v) noexcept = nullptr;
};
// Reference projector for explicitly RECTIFIED pinhole rasters only. Not a
// raw-camera distortion model or proof of Orbbec factory-calibration binding.
struct RectifiedColor {
    std::array<double, 9> R_color_base{};
    Vec3 t_color_base;
    double fx = 0, fy = 0, cx = 0, cy = 0;
};
bool valid_rectified_color(const RectifiedColor&) noexcept;
bool project_rectified_color(const void*, Vec3, double&, double&) noexcept;

enum class TStatus { Fresh, Missing, Stale, Future, InvalidMask, InvalidIdentity,
                     GeometryUnavailable, InactiveOrStale, InvalidEvidence };
struct TReport {
    TStatus status = TStatus::Missing;
    CaptureStamp depth;
    std::optional<CaptureStamp> segmentation;
    std::int64_t age_ns = -1;
    std::size_t ground_projected = 0, projection_misses = 0,
                raw_cells = 0, eroded_cells = 0;
    // Fresh T with positive margin needs unobserved near-side halo in the
    // current measured-blind grid. It must not masquerade as missing T.
    bool near_support_missing = false;
};
class NormalTraversability {
public:
    // Exact geometry/grid owners and projection context must outlive this
    // single-thread consumer. Context calibration is immutable for its ENTIRE
    // lifetime; change of calibration/revision requires a new consumer.
    // The borrowed mask remains immutable through each process() call.
    NormalTraversability(const NativeGeometry&, const NormalGrid&,
                         const ColorProjection&, const TOptions&);
    NormalTraversability(const NormalTraversability&) = delete;
    NormalTraversability& operator=(const NormalTraversability&) = delete;
    const TReport& process(const native_depth::Mailbox&, const CaptureStamp&,
                           const RawTMask*) noexcept;
    const TReport& report() const noexcept { return report_; }
    const NativeGeometry& geometry() const noexcept { return geometry_; }
    const NormalGrid& grid() const noexcept { return grid_; }
    const std::vector<TCount>& counts() const noexcept { return counts_; }
    const std::vector<std::uint8_t>& raw() const noexcept { return raw_; }
    const std::vector<std::uint8_t>& eroded() const noexcept { return eroded_; }
    const std::vector<BinWalkResult>& prefixes() const noexcept { return prefixes_; }
    std::size_t storage_bytes() const noexcept;
private:
    void clear() noexcept;
    void reject(TStatus, bool retain_geometry) noexcept;
    const NativeGeometry& geometry_;
    const NormalGrid& grid_;
    ColorProjection projection_;
    TOptions options_;
    std::vector<TCount> counts_;
    std::vector<std::uint8_t> raw_, eroded_;
    std::vector<std::uint32_t> scratch_;
    std::vector<BinWalkResult> prefixes_;
    // With a split projection: rotation * base ray per native pixel, and rotation * t_base + t.
    std::vector<Vec3> camera_rays_;
    Vec3 camera_offset_;
    TReport report_;
    native_depth::SourceId last_;
};
} // namespace perception::geometry
