/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: normal_traversability.cpp
 * Brief: NormalTraversability: T evidence from the class-92 mask restricts each bin's FREE prefix.
 *
 * Description:
 * What problem this solves. Implements the T owner of traversability.hpp. Per Profile frame, with
 * a class-92 mask of the same or an older frame set of the same stream, no older than
 * seg_max_age:
 *   1. every Ground pixel the grid counted is projected into the colour mask (the factory
 *      projection, t_color_projection); a cell's ground count includes projection misses;
 *   2. a cell is raw-traversable when traversable / ground >= tau_T;
 *   3. erosion by the robot's possible motion since the mask (t_margin_upper_bound) keeps a cell
 *      only when its whole margin rectangle is raw-traversable and observed;
 *   4. each bin's geometric FREE prefix is cut at its first non-eroded cell (never extended).
 * Any identity, age, mask or evidence failure returns a TStatus; failures that keep the geometry
 * return the geometric prefixes with the T flag false (no T is never presented as fresh T).
 */
#include "modules/traversability.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace perception::geometry {
namespace {
bool finite(Vec3 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
} // namespace

// Rotation orthonormal with determinant +1 (within 1e-9), finite translation and intrinsics.
bool valid_rectified_color(const RectifiedColor& c) noexcept {
    if (!finite(c.t_color_base) || !std::isfinite(c.fx) || c.fx <= 0 ||
        !std::isfinite(c.fy) || c.fy <= 0 || !std::isfinite(c.cx) || !std::isfinite(c.cy)) return false;
    const auto& r = c.R_color_base;
    for (const auto x : r) if (!std::isfinite(x)) return false;
    for (std::size_t a = 0; a < 3; ++a) for (std::size_t b = 0; b < 3; ++b) {
        double dot = 0;
        for (std::size_t k = 0; k < 3; ++k) dot += r[a*3+k]*r[b*3+k];
        if (std::abs(dot-(a == b ? 1.0 : 0.0)) > 1e-9) return false;
    }
    const double det = r[0]*(r[4]*r[8]-r[5]*r[7]) - r[1]*(r[3]*r[8]-r[5]*r[6]) +
                       r[2]*(r[3]*r[7]-r[4]*r[6]);
    return std::abs(det-1.0) <= 1e-9;
}
// Pinhole projection of a base-frame point (no distortion); false behind the camera or nonfinite.
bool project_rectified_color(const void* context, Vec3 p, double& u, double& v) noexcept {
    u = v = 0;
    if (context == nullptr || !finite(p)) return false;
    const auto& c = *static_cast<const RectifiedColor*>(context);
    // Full binding validation belongs to construction/caller setup. Retain
    // finite output checks here; this callback does NOT model distortion.
    const auto& r = c.R_color_base;
    const Vec3 q{r[0]*p.x+r[1]*p.y+r[2]*p.z+c.t_color_base.x,
                 r[3]*p.x+r[4]*p.y+r[5]*p.z+c.t_color_base.y,
                 r[6]*p.x+r[7]*p.y+r[8]*p.z+c.t_color_base.z};
    if (!finite(q) || q.z <= 0) return false;
    const double x = c.fx*(q.x/q.z)+c.cx, y = c.fy*(q.y/q.z)+c.cy;
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    u = x; v = y;
    return true;
}

NormalTraversability::NormalTraversability(const NativeGeometry& geometry, const NormalGrid& grid,
        const ColorProjection& projection, const TOptions& options)
    : geometry_(geometry), grid_(grid), projection_(projection), options_(options) {
    if (&grid.geometry() != &geometry || !valid_t_options(options) ||
        projection.width != 1280 || projection.height != 800 || projection.revision == 0 ||
        projection.project == nullptr || projection.context == nullptr)
        throw std::invalid_argument("T requires exact geometry owner, explicit 1280x800 projection and thresholds");
    if (projection.project == project_rectified_color &&
        !valid_rectified_color(*static_cast<const RectifiedColor*>(projection.context)))
        throw std::invalid_argument("invalid explicitly rectified projection");
    const bool split = projection.rotation != nullptr || projection.translation != nullptr ||
                       projection.project_camera != nullptr;
    if (split && (projection.rotation == nullptr || projection.translation == nullptr ||
                  projection.project_camera == nullptr))
        throw std::invalid_argument("split projection needs rotation, translation and project_camera");
    if (split) {
        // q = R (ray * z + t_base) + t = z * (R ray) + (R t_base + t): the per-pixel part once here.
        const auto& r = *projection.rotation;
        const auto rotate = [&r](Vec3 a) {
            return Vec3{r[0]*a.x+r[1]*a.y+r[2]*a.z, r[3]*a.x+r[4]*a.y+r[5]*a.z, r[6]*a.x+r[7]*a.y+r[8]*a.z};
        };
        const auto& rays = geometry.base_rays();
        camera_rays_.resize(rays.size());
        for (std::size_t k = 0; k < rays.size(); ++k) camera_rays_[k] = rotate(rays[k]);
        const Vec3 rt = rotate(geometry.base_translation());
        camera_offset_ = {rt.x+projection.translation->x, rt.y+projection.translation->y,
                          rt.z+projection.translation->z};
        for (const auto& a : camera_rays_)
            if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z))
                throw std::invalid_argument("split projection gives a nonfinite camera ray");
        if (!std::isfinite(camera_offset_.x) || !std::isfinite(camera_offset_.y) || !std::isfinite(camera_offset_.z))
            throw std::invalid_argument("split projection gives a nonfinite camera offset");
    }
    const auto n = geometry.options().n_bins, stride = grid.cell_capacity_per_bin();
    counts_.resize(n*stride);
    raw_.resize(n*stride);
    eroded_.resize(n*stride);
    scratch_.resize((n+1)*(stride+1));
    prefixes_.resize(n);
}
void NormalTraversability::clear() noexcept {
    std::fill(counts_.begin(), counts_.end(), TCount{});
    std::fill(raw_.begin(), raw_.end(), 0);
    std::fill(eroded_.begin(), eroded_.end(), 0);
    std::fill(scratch_.begin(), scratch_.end(), 0);
    std::fill(prefixes_.begin(), prefixes_.end(), BinWalkResult{});
}
// Ends a frame without fresh T: retain_geometry keeps each bin's geometric prefix (T flag false),
// otherwise every prefix is cleared.
void NormalTraversability::reject(TStatus status, bool retain_geometry) noexcept {
    clear();
    report_.status = status;
    report_.segmentation.reset();
    report_.age_ns = -1;
    report_.ground_projected = report_.projection_misses = 0;
    report_.raw_cells = report_.eroded_cells = 0;
    report_.near_support_missing = false;
    if (retain_geometry) for (std::size_t b = 0; b < prefixes_.size(); ++b) {
        prefixes_[b] = grid_.bins()[b].traversal;
        prefixes_[b].t_backed_prefix = false;
    }
}
// One Profile frame: identity and freshness checks, mask validation, ground projection and
// counting, erosion, prefix restriction (see the file header). The stream is checked again at
// the end so a retired stream is never reported as Fresh.
const TReport& NormalTraversability::process(const native_depth::Mailbox& mailbox,
        const CaptureStamp& capture, const RawTMask* mask) noexcept {
    clear();
    report_ = {};
    report_.depth = capture;
    const auto active = mailbox.active_stream();
    const auto finish = [&](TStatus status, bool retain) -> const TReport& {
        if (mailbox.active_stream() != active) reject(TStatus::InactiveOrStale, false);
        else reject(status, retain);
        return report_;
    };
    const auto& source = geometry_.result().source;
    if (!geometry_.bound_to(mailbox) || active == 0 || source.stream_id != active || source.sequence == 0 ||
        source.stream_id < last_.stream_id ||
        (source.stream_id == last_.stream_id && source.sequence <= last_.sequence))
        return finish(TStatus::InactiveOrStale, false);
    if (!same_capture_source(source, grid_.result().source) || !same_capture_source(source, capture.source) ||
        capture.clock_epoch == 0 || capture.capture_mono_ns < 0)
        return finish(TStatus::InvalidIdentity, false);
    last_ = source;
    if (!geometry_.result().has_evidence() || grid_.result().status != GridStatus::NormalEvidence ||
        !grid_.result().blind_m)
        return finish(TStatus::GeometryUnavailable, grid_.result().status == GridStatus::NoGroundReference);
    if (mask == nullptr) return finish(TStatus::Missing, true);
    if (mask->stamp.clock_epoch != capture.clock_epoch ||
        !source_older_or_same(mask->stamp.source, source) || mask->projection_revision != projection_.revision)
        return finish(TStatus::InvalidIdentity, true);
    if (mask->stamp.capture_mono_ns < 0) return finish(TStatus::InvalidIdentity, true);
    if (mask->stamp.capture_mono_ns > capture.capture_mono_ns) return finish(TStatus::Future, true);
    const auto age = capture.capture_mono_ns-mask->stamp.capture_mono_ns;
    if (age > options_.max_age_ns) return finish(TStatus::Stale, true);
    // Bound the complete raster walk and all byte-offset arithmetic, including
    // optional per-row padding. Borrowed bytes must be immutable for this call.
    if (mask->material != MaskMaterial::RawTraversable || mask->width != projection_.width ||
        mask->height != projection_.height || mask->pixels == nullptr ||
        mask->stride_bytes < mask->width || mask->stride_bytes > 4096 ||
        mask->bytes != mask->stride_bytes*mask->height)
        return finish(TStatus::InvalidMask, true);
    // Every byte of every row must be 0 or 1: a byte above 1 has a bit other than
    // bit 0, so OR the row eight bytes at a time and test the other seven bits of
    // each byte once per row (same verdict as the byte-by-byte scan, about 8x
    // fewer operations; optimization round 2, 2026-10-03). Row padding is not read.
    for (std::size_t y = 0; y < mask->height; ++y) {
        const std::uint8_t* row = mask->pixels+y*mask->stride_bytes;
        std::uint64_t bits = 0;
        std::size_t x = 0;
        for (; x+8 <= mask->width; x += 8) {
            std::uint64_t word = 0;
            std::memcpy(&word, row+x, sizeof word);
            bits |= word;
        }
        for (; x < mask->width; ++x) bits |= row[x];
        if ((bits & 0xFEFEFEFEFEFEFEFEULL) != 0) return finish(TStatus::InvalidMask, true);
    }
    const auto& gr = grid_.result();
    const auto stride = grid_.cell_capacity_per_bin();
    const auto& pixels = geometry_.pixels();
    // The grid's list of this frame's Ground pixels with the cell each one was
    // counted in (NormalGrid::ground_cells, pixel order): the same pixels and cells
    // the former full-raster scan with full_cell_index produced. It must hold every
    // Ground pixel the geometry classified, and only Ground pixels.
    const auto& ground = grid_.ground_cells();
    if (ground.size() != geometry_.result().classes[static_cast<std::size_t>(SampleClass::Ground)])
        return finish(TStatus::InvalidEvidence, false);
    for (const auto& g : ground) {
        const std::size_t p = g.pixel;
        if (p >= pixels.size() || pixels[p].classification != SampleClass::Ground)
            return finish(TStatus::InvalidEvidence, false);
        const auto& e = pixels[p];
        const int k = g.cell;
        if (k < 0) continue;
        if (e.bin < 0 || static_cast<std::size_t>(e.bin) >= prefixes_.size())
            return finish(TStatus::InvalidEvidence, false);
        const auto index = static_cast<std::size_t>(e.bin)*stride+static_cast<std::size_t>(k);
        auto& count = counts_[index];
        ++count.ground; // projection failure MUST stay in this denominator
        double u = 0, v = 0;
        bool projected = false;
        if (!camera_rays_.empty()) {
            // Split projection: the camera-frame point from the precomputed ray (see constructor).
            double z = 0;
            if (!geometry_.ground_depth(p, z)) return finish(TStatus::InvalidEvidence, false);
            const Vec3& a = camera_rays_[p];
            const Vec3 q{a.x*z+camera_offset_.x, a.y*z+camera_offset_.y, a.z*z+camera_offset_.z};
            projected = projection_.project_camera(projection_.context, q, u, v);
        } else {
            Vec3 point;
            if (!geometry_.ground_point(p, point)) return finish(TStatus::InvalidEvidence, false);
            projected = projection_.project(projection_.context, point, u, v);
        }
        if (!projected ||
            !std::isfinite(u) || !std::isfinite(v) || u < 0 || v < 0 ||
            u >= mask->width || v >= mask->height) {
            ++report_.projection_misses;
            continue;
        }
        ++report_.ground_projected;
        const auto x = static_cast<std::size_t>(std::floor(u));
        const auto y = static_cast<std::size_t>(std::floor(v));
        if (mask->pixels[y*mask->stride_bytes+x] == 1) ++count.traversable;
    }
    for (std::size_t i = 0; i < counts_.size(); ++i)
        if (counts_[i].ground != grid_.cells()[i].ground) return finish(TStatus::InvalidEvidence, false);
    const TLayout layout{prefixes_.size(), gr.cells_per_bin, stride, *gr.blind_m,
                         grid_.options().dr_m, geometry_.options().angle_step_rad};
    if (!erode_traversability(layout, options_, age, counts_.data(), counts_.size(),
                            raw_.data(), eroded_.data(), scratch_.data(), scratch_.size()))
        return finish(TStatus::InvalidEvidence, false);
    for (std::size_t b = 0; b < prefixes_.size(); ++b) {
        prefixes_[b] = restrict_normal_prefix(grid_.bins()[b].traversal,
            grid_.cells().data()+b*stride, eroded_.data()+b*stride, gr.cells_per_bin,
            *gr.blind_m, grid_.options().dr_m, true);
        if (!prefixes_[b].valid) return finish(TStatus::InvalidEvidence, false);
    }
    if (mailbox.active_stream() != active) return finish(TStatus::InactiveOrStale, false);
    report_.status = TStatus::Fresh;
    report_.segmentation = mask->stamp;
    report_.age_ns = age;
    report_.raw_cells = static_cast<std::size_t>(std::count(raw_.begin(), raw_.end(), std::uint8_t{1}));
    report_.eroded_cells = static_cast<std::size_t>(std::count(eroded_.begin(), eroded_.end(), std::uint8_t{1}));
    if (gr.cells_per_bin > 0) {
        const double far = *gr.blind_m+grid_.options().dr_m;
        report_.near_support_missing = t_margin_upper_bound(options_, age, far) > 0;
    }
    return report_;
}
std::size_t NormalTraversability::storage_bytes() const noexcept {
    return counts_.capacity()*sizeof(TCount) + camera_rays_.capacity()*sizeof(Vec3) + raw_.capacity()+eroded_.capacity() +
        scratch_.capacity()*sizeof(std::uint32_t) + prefixes_.capacity()*sizeof(BinWalkResult);
}
} // namespace perception::geometry
