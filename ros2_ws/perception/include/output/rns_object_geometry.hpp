/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_object_geometry.hpp
 * Brief: Measured base_link geometry for one detected object (19 section 4.1).
 *
 * Description:
 * Fields: footprint_xy (at most 12 vertices), the measured z interval and r_near from the
 * footprint; an unlocatable object has all four fields null.
 * What problem this solves. RNS needs a ground footprint and a height interval per object,
 * never an image box (a box plus depth is a frustum) nor a centroid (a bar at 1.8 m would be
 * lost), 11 section 3.1B.2. The service's object geometry hook calls build() once per detection
 * on the Supervisor processing thread; the result fills FusedDetection's base_* fields.
 *
 * Input is the SAME point set DepthSampler selected for this inference frame:
 * the aligned (colour-plane) depth region (mask inside the bbox, or the bbox
 * centre quarter-margin ROI without a mask) whose depth in metres
 * (code * depth_scale, identical float arithmetic) lies in
 * [selected_min_m, selected_max_m] of the RAW sample. Each point is
 * depth * optical_ray(u, v) in the colour optical frame, then base <- colour.
 *
 * Footprint (implementation choices, all conservative except the contract's
 * <= 12 vertex decimation):
 *   - x-buckets of bucket_m keep each bucket's min-y and max-y points; the
 *     convex hull of those, widened by +-bucket_m along x, contains every
 *     selected point's (x, y);
 *   - more than 12 vertices: remove the vertex with the smallest triangle
 *     area, never one of the min/max x/y extreme vertices (19 section 4.1);
 *   - collinear / fewer than 3 distinct points: bounding rectangle inflated by
 *     degenerate_half_width_m;
 *   - the float32 footprint must pass the encoder's own valid_footprint() rule,
 *     else the (inflated) axis-aligned bounding rectangle is used.
 * z interval: exact min / max over ALL selected points, rounded outward to
 * float32 (never the centroid +- half height).
 * One region scan per call (selected points kept for bucketing; scratch grows
 * to the largest region seen, no steady-state allocation). The footprint's work
 * lists (bucket extremes, widened points, hull, decimation flags, float vertices)
 * are builder scratch too (2026-10-11): a Located build allocates once, for the
 * vertices the result owns, and any other build not at all. When the caller hands
 * over DepthSampler's own list of selected pixels (DepthRegion::selected, 2026-10-06)
 * the scan is skipped and the same points are taken from the list, in the same
 * order, so every output is bit-identical; a list that fails the cheap checks in
 * build() is ignored and the region is scanned as before.
 * Single-precision points (options.float_margin_m > 0, 2026-10-08): the same points in float with
 * no division per point, about half the cost. Float error is a few micrometres at most; every
 * output is widened by the margin (footprint: +margin along x on top of the bucket width and
 * +-margin in y at the bucket extremes; z interval: +-margin), so the footprint still contains
 * every exact point and the z interval every exact height. Not bit-identical to the exact path;
 * allowed by the 2026-10-04 accuracy rule (more conservative only, height within 1 cm).
 * Pure computation: no SDK, OpenCV, clock or calibration certification.
 */
#pragma once

#include "output/rns_contract.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace perception::rns {

// Row-major rotation and translation (metres): p_to = R * p_from + t.
struct RigidTransform {
    std::array<double, 9> R{};
    std::array<double, 3> t{};
};
// Validates a proper rotation (orthonormal, det +1 within 1e-6) and finite t.
bool valid_rigid_transform(const RigidTransform&) noexcept;
// a o b: first b, then a.
RigidTransform compose(const RigidTransform& a, const RigidTransform& b) noexcept;
RigidTransform inverse(const RigidTransform&) noexcept;

// Optical rays (x, y, 1) of the colour raster the aligned depth lives on.
struct OpticalRayTable {
    std::uint32_t width = 0, height = 0;
    std::vector<float> x, y;  // row-major, width * height each
};

struct ObjectGeometryOptions {
    std::size_t min_points = 0;         // located only with >= this many selected points
    double bucket_m = 0;                // > 0
    double degenerate_half_width_m = 0; // > 0
    // 0: exact double-precision path (bit-identical outputs). > 0: single-precision points, every
    // output widened by this margin in metres (the service uses 0.001); see the file header.
    double float_margin_m = 0;
};

struct DepthRegion {
    const std::uint16_t* depth = nullptr;  // aligned depth codes, row-major
    std::uint32_t width = 0, height = 0;
    float depth_scale = 0;                 // metres per code
    int bbox_x = 0, bbox_y = 0, bbox_width = 0, bbox_height = 0;
    const std::uint8_t* mask = nullptr;    // bbox-sized binary mask, or nullptr
    // Optional: DepthSampler's selected pixels for this region as row-major indices
    // (y * width + x), strictly increasing, i.e. the same points and order the scan finds.
    // Checked cheaply (not empty, not longer than the region, order, bounds of the region rows,
    // depth code inside the band); the mask and the columns are not re-read, so the list must
    // come from the same sample (see DepthSampler). nullptr = no list, scan the region.
    const std::uint32_t* selected = nullptr;
    std::size_t selected_count = 0;
};

// One base_link (x, y) in double precision while a footprint is built (builder scratch only).
struct FootprintXY { double x, y; };

// Only Located has geometry; every other status leaves the object unlocatable.
enum class ObjectGeometryStatus { Located, InvalidInput, TooFewPoints, NonFinite };
// Result plus the counters the geometry audit log reports.
struct ObjectGeometryResult {
    ObjectGeometryStatus status = ObjectGeometryStatus::InvalidInput;
    std::optional<ObjectGeometry> geometry;
    std::size_t points = 0, hull_vertices = 0;
    bool degenerate = false, decimated = false, rectangle_fallback = false;
    // The points came from DepthRegion::selected; or a list was given but failed a check, so
    // the region was scanned instead (should never happen; counted by the service).
    bool used_selected = false, selected_rejected = false;
    // Built on the single-precision path (options.float_margin_m > 0).
    bool float_points = false;
};

class ObjectGeometryBuilder {
public:
    // Throws std::invalid_argument on an invalid table/transform/options.
    // Allocates bounded scratch once (grows only for a larger region).
    ObjectGeometryBuilder(OpticalRayTable rays, RigidTransform base_from_colour, ObjectGeometryOptions options);
    // selected_min_m/max_m: DepthSampler's selected depth band for this object.
    // One owner, one call at a time (the scratch below is shared by all calls). The result owns
    // its vertices: a later build() never changes a result returned earlier.
    ObjectGeometryResult build(const DepthRegion& region, float selected_min_m, float selected_max_m);

private:
    // The single-precision path of build(); region and code band already validated by build().
    ObjectGeometryResult build_float(const DepthRegion& region, int x0, int y0, int x1, int y1,
                                     std::uint32_t code_lo, std::uint32_t code_hi);
    // The shared tail of both paths: hull, 12-vertex cap, fallbacks, float32 rounding, z interval.
    // `margin` widens every output; 0 on the exact path, whose values are then untouched.
    void finish_geometry(ObjectGeometryResult& out, std::vector<FootprintXY>& widened, double xmin, double xmax,
                         double ymin, double ymax, double zmin, double zmax, double width, double eps, double margin);

    OpticalRayTable rays_;
    RigidTransform base_from_colour_;
    ObjectGeometryOptions options_;
    // Per x-bucket min/max y and the x at which each was seen.
    std::vector<double> bucket_min_y_, bucket_max_y_, bucket_min_y_x_, bucket_max_y_x_;
    std::vector<double> points_x_, points_y_;  // selected base (x, y); grows to the largest region seen
    // Footprint work lists of one build(): two extremes per occupied bucket, their widened
    // copies, the hull (decimated in place), the decimation's keep flags and the float vertices
    // before they are checked. Members only so that their capacity is reused; every one is
    // rewritten from the start by each build() that reaches it and none outlives it as data.
    std::vector<FootprintXY> extremes_, widened_, hull_;
    std::vector<bool> keep_;
    std::vector<Point> footprint_;
    std::vector<float> points_xf_, points_yf_; // the same in single precision (float path)
    std::vector<std::uint32_t> indices_;       // float path: pixel indices found by the region scan
};

}  // namespace perception::rns
