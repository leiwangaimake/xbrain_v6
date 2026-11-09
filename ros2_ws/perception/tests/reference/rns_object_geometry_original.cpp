/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_object_geometry_original.cpp
 * Brief: Frozen copy of ObjectGeometryBuilder as it was before its footprint scratch (test oracle).
 *
 * Description:
 * Built only into test_rns_object_geometry_scratch (namespace rns_object_geometry_original), which
 * compares every output field of the production builder against it, bit for bit. Not production
 * code. The constructor and build() are the text of src/output/rns_object_geometry.cpp at commit
 * 02d2665 (2026-10-10), where the bucket extremes, the widened points, the hull, the decimation
 * lists and the float footprint were locals of build(), allocated in every call. Only the
 * namespace, the include and this header differ; the transform helpers, which the comparison does
 * not concern, are the production ones. Do not edit it together with the production file: a change
 * that alters the outputs must fail against this copy and be decided on its own.
 */
#include "rns_object_geometry_original.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rns_object_geometry_original {
namespace {
// The Objects vertex cap (11 section 3.1B.2, an RNS real-time bound).
constexpr std::size_t kMaxFootprintVertices = 12;
// Bucket count bound; beyond it the bucket width grows instead (memory bound).
constexpr std::size_t kMaxBuckets = 65536;

struct P2 { double x, y; };

// z of (a - o) x (b - o): > 0 when o -> a -> b turns left.
double cross(const P2& o, const P2& a, const P2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Andrew monotone chain; returns CCW hull without repeated end point.
std::vector<P2> convex_hull(std::vector<P2> pts) {
    std::sort(pts.begin(), pts.end(), [](const P2& a, const P2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    pts.erase(std::unique(pts.begin(), pts.end(), [](const P2& a, const P2& b) { return a.x == b.x && a.y == b.y; }),
              pts.end());
    if (pts.size() < 3) return pts;
    // Lower then upper chain; "<= 0" also drops collinear points.
    std::vector<P2> hull(2 * pts.size());
    std::size_t k = 0;
    for (const auto& p : pts) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 0) --k;
        hull[k++] = p;
    }
    for (std::size_t i = pts.size() - 1, lower = k + 1; i-- > 0;) {
        while (k >= lower && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0) --k;
        hull[k++] = pts[i];
    }
    hull.resize(k - 1);
    return hull;
}

// Twice the signed area (positive for CCW); ~0 marks a degenerate hull.
double area2(const std::vector<P2>& poly) {
    double a = 0;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const auto& p = poly[i];
        const auto& q = poly[(i + 1) % poly.size()];
        a += p.x * q.y - q.x * p.y;
    }
    return a;
}

// float32 rounding away from the interior: low ends down, high ends up, so the
// float interval always contains the double one.
float outward_low(double v) {
    auto f = static_cast<float>(v);
    if (static_cast<double>(f) > v) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
    return f;
}
float outward_high(double v) {
    auto f = static_cast<float>(v);
    if (static_cast<double>(f) < v) f = std::nextafter(f, std::numeric_limits<float>::infinity());
    return f;
}

// Nearest float32 vertices; the result is re-checked with valid_footprint().
std::vector<Point> to_float(const std::vector<P2>& poly) {
    std::vector<Point> out;
    out.reserve(poly.size());
    for (const auto& p : poly) out.push_back({static_cast<float>(p.x), static_cast<float>(p.y)});
    return out;
}
// CCW axis-aligned rectangle, rounded outward.
std::vector<Point> rectangle(double x0, double y0, double x1, double y1) {
    return {{outward_low(x0), outward_low(y0)}, {outward_high(x1), outward_low(y0)},
            {outward_high(x1), outward_high(y1)}, {outward_low(x0), outward_high(y1)}};
}
}  // namespace

ObjectGeometryBuilder::ObjectGeometryBuilder(OpticalRayTable rays, RigidTransform base_from_colour,
                                             ObjectGeometryOptions options)
    : rays_(std::move(rays)), base_from_colour_(base_from_colour), options_(options) {
    // Startup checks (throw): a complete finite ray table, a proper rigid
    // transform and usable options.
    const auto pixels = static_cast<std::size_t>(rays_.width) * rays_.height;
    if (rays_.width == 0 || rays_.height == 0 || rays_.x.size() != pixels || rays_.y.size() != pixels)
        throw std::invalid_argument("object geometry: optical ray table size");
    for (std::size_t i = 0; i < pixels; ++i)
        if (!std::isfinite(rays_.x[i]) || !std::isfinite(rays_.y[i]))
            throw std::invalid_argument("object geometry: non-finite optical ray");
    if (!valid_rigid_transform(base_from_colour_)) throw std::invalid_argument("object geometry: invalid base<-colour transform");
    if (options_.min_points < 1 || !(options_.bucket_m > 0) || !std::isfinite(options_.bucket_m) ||
        !(options_.degenerate_half_width_m > 0) || !std::isfinite(options_.degenerate_half_width_m))
        throw std::invalid_argument("object geometry: invalid options");
    // Pre-sized so a real object never reallocates on the hot path (4096 buckets = 41 m at the
    // service's 1 cm width); assign() in build() grows them once for a larger span.
    bucket_min_y_.reserve(4096);
    bucket_max_y_.reserve(4096);
    bucket_min_y_x_.reserve(4096);
    bucket_max_y_x_.reserve(4096);
}

ObjectGeometryResult ObjectGeometryBuilder::build(const DepthRegion& r, float lo, float hi) {
    // Any invalid input: InvalidInput (unlocatable), never an exception.
    ObjectGeometryResult out;
    if (!r.depth || r.width != rays_.width || r.height != rays_.height || !std::isfinite(r.depth_scale) ||
        r.depth_scale <= 0 || r.bbox_width <= 0 || r.bbox_height <= 0 || !std::isfinite(lo) || !std::isfinite(hi) ||
        lo > hi)
        return out;
    // Region: exactly DepthSampler::sample_impl's roi.
    int x0 = std::max(0, r.bbox_x), y0 = std::max(0, r.bbox_y);
    int x1 = std::min(static_cast<int>(r.width), r.bbox_x + r.bbox_width);
    int y1 = std::min(static_cast<int>(r.height), r.bbox_y + r.bbox_height);
    if (x1 <= x0 || y1 <= y0) return out;
    // Without a mask DepthSampler uses the box centre (a quarter margin on each
    // side); the same rectangle here.
    if (!r.mask) {
        int w = x1 - x0, h = y1 - y0;
        const int mx = w / 4, my = h / 4;
        x0 += mx; y0 += my; x1 = x0 + (w - 2 * mx); y1 = y0 + (h - 2 * my);
    }
    // base <- colour transform of this session (see the header).
    const auto& R = base_from_colour_.R;
    const auto& t = base_from_colour_.t;
    // Single region scan: select exactly as DepthSampler, transform each
    // selected point once, keep its base (x, y) for bucketing and track the
    // exact extents. Same per-point expressions as the two-pass reference.
    // Point storage grows only for a region larger than any seen before.
    const auto region_pixels = static_cast<std::size_t>(x1 - x0) * static_cast<std::size_t>(y1 - y0);
    if (points_x_.size() < region_pixels) {
        points_x_.resize(region_pixels);
        points_y_.resize(region_pixels);
    }
    // Selection by code: depth_m = float(code) * depth_scale is non-decreasing
    // in code (exact integer-to-float, positive scale, monotone IEEE rounding;
    // an overflow to +inf still orders above hi), so
    // lo <= depth_m <= hi  <=>  code_lo <= code <= code_hi, found once here.
    const auto metres = [&](std::uint32_t code) { return static_cast<float>(code) * r.depth_scale; };
    std::uint32_t code_lo = 0, code_hi = 65535;
    {
        std::uint32_t a = 0, b = 65536;  // first code with metres >= lo
        while (a < b) { const std::uint32_t m = a + (b - a) / 2; if (metres(m) >= lo) b = m; else a = m + 1; }
        code_lo = a;
        a = 0; b = 65536;  // first code with metres > hi
        while (a < b) { const std::uint32_t m = a + (b - a) / 2; if (metres(m) > hi) b = m; else a = m + 1; }
        if (a == 0) code_lo = 65536;  // nothing <= hi
        else code_hi = a - 1;
    }
    // Exact extents of the selected base_link points.
    double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin, zmin = xmin, zmax = -xmin;
    double ymin = xmin, ymax = -xmin;
    std::size_t n = 0;
    const double* px = points_x_.data();
    const double* py = points_y_.data();
    bool non_finite = false;
    // p_colour = depth * (ray_x, ray_y, 1); p_base = R p_colour + t. One point at pixel index i.
    const auto add_point = [&](std::size_t i, std::uint32_t code) {
        const float depth_m = static_cast<float>(code) * r.depth_scale;
        const double z = depth_m, cx = z * rays_.x[i], cy = z * rays_.y[i];
        const double bx = R[0] * cx + R[1] * cy + R[2] * z + t[0];
        const double by = R[3] * cx + R[4] * cy + R[5] * z + t[1];
        const double bz = R[6] * cx + R[7] * cy + R[8] * z + t[2];
        if (!std::isfinite(bx) || !std::isfinite(by) || !std::isfinite(bz)) {
            non_finite = true;
            return false;
        }
        points_x_[n] = bx;
        points_y_[n] = by;
        ++n;
        xmin = std::min(xmin, bx); xmax = std::max(xmax, bx);
        ymin = std::min(ymin, by); ymax = std::max(ymax, by);
        zmin = std::min(zmin, bz); zmax = std::max(zmax, bz);
        return true;
    };
    // The sampler's list: walk it if it is not empty (a valid sample selects at least one pixel),
    // no longer than the region and every index is in order, inside the region's rows and inside
    // the band; at the first failed check, start over with the scan below.
    bool scanned = false;
    if (r.selected != nullptr) {
        const std::size_t first = static_cast<std::size_t>(y0) * r.width + static_cast<std::size_t>(x0);
        const std::size_t last = static_cast<std::size_t>(y1 - 1) * r.width + static_cast<std::size_t>(x1 - 1);
        bool ok = r.selected_count > 0 && r.selected_count <= region_pixels;
        std::size_t previous = 0;
        for (std::size_t k = 0; ok && k < r.selected_count; ++k) {
            const std::size_t i = r.selected[k];
            const std::uint32_t code = r.depth[i < first || i > last ? first : i];
            if (i < first || i > last || (k > 0 && i <= previous) || code < code_lo || code > code_hi) {
                ok = false;
                break;
            }
            previous = i;
            if (!add_point(i, code)) break;
        }
        if (ok) {
            out.used_selected = true;
            scanned = true;
        } else {
            out.selected_rejected = true;
            n = 0;
            non_finite = false;
            xmin = std::numeric_limits<double>::infinity(); xmax = -xmin; zmin = xmin; zmax = -xmin;
            ymin = xmin; ymax = -xmin;
        }
    }
    for (int y = y0; !scanned && !non_finite && y < y1; ++y) {
        const std::size_t row = static_cast<std::size_t>(y) * r.width;
        const std::uint16_t* depth_row = r.depth + row;
        const std::uint8_t* mask_row =
            r.mask ? r.mask + static_cast<std::size_t>(y - r.bbox_y) * static_cast<std::size_t>(r.bbox_width) : nullptr;
        for (int x = x0; x < x1; ++x) {
            const std::uint32_t code = depth_row[x];
            if (code < code_lo || code > code_hi) continue;
            if (mask_row && mask_row[x - r.bbox_x] == 0) continue;
            if (!add_point(row + static_cast<std::size_t>(x), code)) break;
        }
    }
    if (non_finite) {
        out.status = ObjectGeometryStatus::NonFinite;
        return out;
    }
    out.points = n;
    // Too few measured points: unlocatable, never a guessed box.
    if (n < options_.min_points) {
        out.status = ObjectGeometryStatus::TooFewPoints;
        return out;
    }
    // Per-bucket y extremes over the stored points, in the same scan order.
    double width = options_.bucket_m;
    const double span = xmax - xmin;
    if (!std::isfinite(span) || !std::isfinite(ymax - ymin) || !std::isfinite(zmax - zmin)) {
        out.status = ObjectGeometryStatus::NonFinite;
        return out;
    }
    // Clamp in floating point BEFORE converting (a huge finite span must not
    // overflow the integer conversion); identical count for in-range spans.
    const double whole = std::floor(span / width);
    std::size_t buckets;
    if (whole + 1.0 > static_cast<double>(kMaxBuckets)) {
        width = span / static_cast<double>(kMaxBuckets - 1);
        buckets = kMaxBuckets;
    } else {
        buckets = static_cast<std::size_t>(whole) + 1;
    }
    bucket_min_y_.assign(buckets, std::numeric_limits<double>::infinity());
    bucket_max_y_.assign(buckets, -std::numeric_limits<double>::infinity());
    bucket_min_y_x_.assign(buckets, 0.0);
    bucket_max_y_x_.assign(buckets, 0.0);
    // Each point to its x-bucket; keep the bucket's min/max y and the x there.
    for (std::size_t i = 0; i < n; ++i) {
        const double bx = px[i], by = py[i];
        const double q = std::floor((bx - xmin) / width);  // in [0, kMaxBuckets] by construction
        // With the recomputed width, rounding can put xmax one bucket past the end: clamp it.
        auto b = q < static_cast<double>(buckets) ? static_cast<std::size_t>(q) : buckets - 1;
        if (by < bucket_min_y_[b]) { bucket_min_y_[b] = by; bucket_min_y_x_[b] = bx; }
        if (by > bucket_max_y_[b]) { bucket_max_y_[b] = by; bucket_max_y_x_[b] = bx; }
    }
    // Two points per non-empty bucket.
    std::vector<P2> extremes;
    extremes.reserve(2 * buckets);
    for (std::size_t b = 0; b < buckets; ++b) {
        if (!std::isfinite(bucket_min_y_[b])) continue;
        extremes.push_back({bucket_min_y_x_[b], bucket_min_y_[b]});
        extremes.push_back({bucket_max_y_x_[b], bucket_max_y_[b]});
    }
    // Widen along x by the bucket width: every selected point lies within
    // horizontal distance `width` of the hull of the bucket extremes.
    std::vector<P2> widened;
    widened.reserve(2 * extremes.size());
    for (const auto& p : extremes) {
        widened.push_back({p.x - width, p.y});
        widened.push_back({p.x + width, p.y});
    }
    auto hull = convex_hull(std::move(widened));
    out.hull_vertices = hull.size();
    const double eps = options_.degenerate_half_width_m;
    std::vector<Point> footprint;
    if (hull.size() < 3 || std::abs(area2(hull)) <= 1e-12) {
        // Degenerate (e.g. every point on one line parallel to x, which the x
        // widening cannot open): bounding rectangle of the extent + eps.
        out.degenerate = true;
        footprint = rectangle(xmin - width - eps, ymin - eps, xmax + width + eps, ymax + eps);
    } else {
        // More than 12 vertices: repeatedly drop the non-extreme vertex whose
        // triangle with its neighbours is smallest (least area lost). This cuts
        // corners, the one non-conservative step, demanded by the vertex cap;
        // the four extreme vertices keep the extent.
        if (hull.size() > kMaxFootprintVertices) {
            out.decimated = true;
            std::vector<bool> keep(hull.size(), false);
            std::size_t ixmin = 0, ixmax = 0, iymin = 0, iymax = 0;
            for (std::size_t i = 1; i < hull.size(); ++i) {
                if (hull[i].x < hull[ixmin].x) ixmin = i;
                if (hull[i].x > hull[ixmax].x) ixmax = i;
                if (hull[i].y < hull[iymin].y) iymin = i;
                if (hull[i].y > hull[iymax].y) iymax = i;
            }
            keep[ixmin] = keep[ixmax] = keep[iymin] = keep[iymax] = true;
            std::vector<P2> poly = hull;
            while (poly.size() > kMaxFootprintVertices) {
                std::size_t victim = poly.size();
                double best = std::numeric_limits<double>::infinity();
                for (std::size_t i = 0; i < poly.size(); ++i) {
                    if (keep[i]) continue;
                    const auto& a = poly[(i + poly.size() - 1) % poly.size()];
                    const auto& c = poly[(i + 1) % poly.size()];
                    const double contribution = std::abs(cross(a, poly[i], c));
                    if (contribution < best) { best = contribution; victim = i; }
                }
                // Defensive: only kept extremes left (not reachable above 12).
                if (victim == poly.size()) break;
                poly.erase(poly.begin() + static_cast<long>(victim));
                keep.erase(keep.begin() + static_cast<long>(victim));
            }
            hull = std::move(poly);
        }
        // float32 rounding can make a thin hull non-convex; the encoder's own rule
        // decides, and the outward bounding rectangle is the fallback.
        footprint = to_float(hull);
        if (!valid_footprint(footprint)) {
            out.rectangle_fallback = true;
            // Extent rectangle widened along x like the hull, so it contains the hull it replaces.
            footprint = rectangle(xmin - width, ymin, xmax + width, ymax);
        }
    }
    // Even the rectangle can fail for extreme coordinates: then unlocatable.
    if (!valid_footprint(footprint)) {
        out.status = ObjectGeometryStatus::NonFinite;
        return out;
    }
    // z interval: exact extremes of all selected points, rounded outward.
    const float z_low = outward_low(zmin), z_high = outward_high(zmax);
    if (!std::isfinite(z_low) || !std::isfinite(z_high)) {
        out.status = ObjectGeometryStatus::NonFinite;
        return out;
    }
    out.geometry.emplace(std::move(footprint), z_low, z_high);
    out.status = ObjectGeometryStatus::Located;
    return out;
}

}  // namespace rns_object_geometry_original
