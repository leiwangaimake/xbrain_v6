/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_object_geometry.cpp
 * Brief: ObjectGeometryBuilder: selected depth points -> base_link footprint and z interval.
 *
 * Description:
 * What problem this solves. Implements rns_object_geometry.hpp (algorithm and conservativeness
 * rules are listed there). One scan of the object's region selects the same points as
 * DepthSampler, transforms each to base_link once, and records exact extents; the footprint is
 * then built from per-x-bucket y extremes, widened, hulled and, if needed, decimated to 12
 * vertices. With DepthSampler's list of selected pixels (2026-10-06) the scan is replaced by a
 * walk over that list: the same points in the same order, so the same bits. On 1.8 (core 7,
 * synthetic 15k-point objects) the scan was about 60 % of build(), 0.54-0.71 ms.
 * The footprint's work lists are the builder's scratch (2026-10-11): same values written in the
 * same order as when they were locals, but a build allocates only the result's own vertices.
 *
 * Why buckets instead of a hull of all points. A large object can select tens of thousands of
 * points; the hull of per-bucket extremes (at most two points per bucket) is cheap, and widening
 * it by one bucket width along x provably contains every selected point.
 */
#include "output/rns_object_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception::rns {
namespace {
// The Objects vertex cap (11 section 3.1B.2, an RNS real-time bound).
constexpr std::size_t kMaxFootprintVertices = 12;
// Bucket count bound; beyond it the bucket width grows instead (memory bound).
constexpr std::size_t kMaxBuckets = 65536;

using P2 = FootprintXY;

// z of (a - o) x (b - o): > 0 when o -> a -> b turns left.
double cross(const P2& o, const P2& a, const P2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Andrew monotone chain; leaves the CCW hull without repeated end point in `hull`. `pts` is
// sorted and de-duplicated in place. Both are the caller's scratch: whatever `hull` held before
// is replaced.
void convex_hull(std::vector<P2>& pts, std::vector<P2>& hull) {
    std::sort(pts.begin(), pts.end(), [](const P2& a, const P2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    pts.erase(std::unique(pts.begin(), pts.end(), [](const P2& a, const P2& b) { return a.x == b.x && a.y == b.y; }),
              pts.end());
    if (pts.size() < 3) {
        hull.assign(pts.begin(), pts.end());
        return;
    }
    // Lower then upper chain; "<= 0" also drops collinear points. Only slots below k are read,
    // and each of those was written in this call, so entries left from an earlier hull are inert.
    hull.resize(2 * pts.size());
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

// Nearest float32 vertices; the result is re-checked with valid_footprint(). `out` is replaced.
void to_float(const std::vector<P2>& poly, std::vector<Point>& out) {
    out.clear();
    out.reserve(poly.size());
    for (const auto& p : poly) out.push_back({static_cast<float>(p.x), static_cast<float>(p.y)});
}
// CCW axis-aligned rectangle, rounded outward. `out` is replaced.
void rectangle(double x0, double y0, double x1, double y1, std::vector<Point>& out) {
    out.assign({{outward_low(x0), outward_low(y0)}, {outward_high(x1), outward_low(y0)},
                {outward_high(x1), outward_high(y1)}, {outward_low(x0), outward_high(y1)}});
}

}  // namespace

// R^T R = I and det R = +1 (a reflection would mirror the scene), finite t.
bool valid_rigid_transform(const RigidTransform& T) noexcept {
    for (double v : T.R) if (!std::isfinite(v)) return false;
    for (double v : T.t) if (!std::isfinite(v)) return false;
    const auto& r = T.R;
    for (std::size_t a = 0; a < 3; ++a)
        for (std::size_t c = 0; c < 3; ++c) {
            double d = 0;
            for (std::size_t k = 0; k < 3; ++k) d += r[3 * a + k] * r[3 * c + k];
            if (std::abs(d - (a == c ? 1.0 : 0.0)) > 1e-6) return false;
        }
    const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
                       r[2] * (r[3] * r[7] - r[4] * r[6]);
    return std::abs(det - 1.0) <= 1e-6;
}

// a o b: R = Ra Rb, t = Ra tb + ta.
RigidTransform compose(const RigidTransform& a, const RigidTransform& b) noexcept {
    RigidTransform c;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            double s = 0;
            for (std::size_t k = 0; k < 3; ++k) s += a.R[3 * i + k] * b.R[3 * k + j];
            c.R[3 * i + j] = s;
        }
        double t = a.t[i];
        for (std::size_t k = 0; k < 3; ++k) t += a.R[3 * i + k] * b.t[k];
        c.t[i] = t;
    }
    return c;
}

// Rigid inverse: R^T and -R^T t.
RigidTransform inverse(const RigidTransform& a) noexcept {
    RigidTransform c;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) c.R[3 * i + j] = a.R[3 * j + i];
    for (std::size_t i = 0; i < 3; ++i) {
        double t = 0;
        for (std::size_t k = 0; k < 3; ++k) t -= c.R[3 * i + k] * a.t[k];
        c.t[i] = t;
    }
    return c;
}

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
    // The float path's margin must cover its rounding (micrometres) and stay inside the 1 cm
    // height tolerance of the accuracy rule.
    if (!std::isfinite(options_.float_margin_m) || options_.float_margin_m < 0 ||
        (options_.float_margin_m > 0 && (options_.float_margin_m < 1e-4 || options_.float_margin_m > 0.01)))
        throw std::invalid_argument("object geometry: float margin must be 0 or within [0.1 mm, 1 cm]");
    // Pre-sized so a real object never reallocates on the hot path (4096 buckets = 41 m at the
    // service's 1 cm width); assign() in build() grows them once for a larger span.
    bucket_min_y_.reserve(4096);
    bucket_max_y_.reserve(4096);
    bucket_min_y_x_.reserve(4096);
    bucket_max_y_x_.reserve(4096);
    // The footprint work lists for the same 4096 buckets, every one occupied: two extremes per
    // bucket, two widened points per extreme, the hull's 2n working slots and one keep flag per
    // hull vertex; build() grows them once for more.
    extremes_.reserve(2 * 4096);
    widened_.reserve(4 * 4096);
    hull_.reserve(8 * 4096);
    keep_.reserve(4 * 4096);
    footprint_.reserve(kMaxFootprintVertices);
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
    if (options_.float_margin_m > 0) return build_float(r, x0, y0, x1, y1, code_lo, code_hi);
    if (points_x_.size() < region_pixels) {
        points_x_.resize(region_pixels);
        points_y_.resize(region_pixels);
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
    // Two points per non-empty bucket. From here on the work lists are the builder's scratch
    // under their old local names: each is emptied or overwritten before it is read.
    std::vector<P2>& extremes = extremes_;
    extremes.clear();
    extremes.reserve(2 * buckets);
    for (std::size_t b = 0; b < buckets; ++b) {
        if (!std::isfinite(bucket_min_y_[b])) continue;
        extremes.push_back({bucket_min_y_x_[b], bucket_min_y_[b]});
        extremes.push_back({bucket_max_y_x_[b], bucket_max_y_[b]});
    }
    // Widen along x by the bucket width: every selected point lies within
    // horizontal distance `width` of the hull of the bucket extremes.
    std::vector<P2>& widened = widened_;
    widened.clear();
    widened.reserve(2 * extremes.size());
    for (const auto& p : extremes) {
        widened.push_back({p.x - width, p.y});
        widened.push_back({p.x + width, p.y});
    }
    finish_geometry(out, widened, xmin, xmax, ymin, ymax, zmin, zmax, width, options_.degenerate_half_width_m, 0.0);
    return out;
}

// Footprint and z interval from the widened bucket extremes (`widened` is reordered): convex hull,
// the 12-vertex cap, fallbacks, float32 rounding. `margin` widens every output (the float path);
// with margin 0 every value is the exact path's own one (nothing is added to or taken from it).
void ObjectGeometryBuilder::finish_geometry(ObjectGeometryResult& out, std::vector<FootprintXY>& widened, double xmin,
                                            double xmax, double ymin, double ymax, double zmin, double zmax,
                                            double width, double eps, double margin) {
    const double grow = margin > 0 ? width + margin : width;
    const auto lower = [margin](double v) { return margin > 0 ? v - margin : v; };
    const auto upper = [margin](double v) { return margin > 0 ? v + margin : v; };
    std::vector<P2>& hull = hull_;
    convex_hull(widened, hull);
    out.hull_vertices = hull.size();
    std::vector<Point>& footprint = footprint_;
    if (hull.size() < 3 || std::abs(area2(hull)) <= 1e-12) {
        // Degenerate (e.g. every point on one line parallel to x, which the x
        // widening cannot open): bounding rectangle of the extent + eps.
        out.degenerate = true;
        rectangle(xmin - grow - eps, lower(ymin) - eps, xmax + grow + eps, upper(ymax) + eps, footprint);
    } else {
        // More than 12 vertices: repeatedly drop the non-extreme vertex whose
        // triangle with its neighbours is smallest (least area lost). This cuts
        // corners, the one non-conservative step, demanded by the vertex cap;
        // the four extreme vertices keep the extent.
        if (hull.size() > kMaxFootprintVertices) {
            out.decimated = true;
            std::vector<bool>& keep = keep_;
            keep.assign(hull.size(), false);
            std::size_t ixmin = 0, ixmax = 0, iymin = 0, iymax = 0;
            for (std::size_t i = 1; i < hull.size(); ++i) {
                if (hull[i].x < hull[ixmin].x) ixmin = i;
                if (hull[i].x > hull[ixmax].x) ixmax = i;
                if (hull[i].y < hull[iymin].y) iymin = i;
                if (hull[i].y > hull[iymax].y) iymax = i;
            }
            keep[ixmin] = keep[ixmax] = keep[iymin] = keep[iymax] = true;
            // In place: the hull is not read again before the decimated polygon replaces it.
            std::vector<P2>& poly = hull;
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
        }
        // float32 rounding can make a thin hull non-convex; the encoder's own rule
        // decides, and the outward bounding rectangle is the fallback.
        to_float(hull, footprint);
        if (!valid_footprint(footprint)) {
            out.rectangle_fallback = true;
            // Extent rectangle widened along x like the hull, so it contains the hull it replaces.
            rectangle(xmin - grow, lower(ymin), xmax + grow, upper(ymax), footprint);
        }
    }
    // Even the rectangle can fail for extreme coordinates: then unlocatable.
    if (!valid_footprint(footprint)) {
        out.status = ObjectGeometryStatus::NonFinite;
        return;
    }
    // z interval: exact extremes of all selected points (moved outward by the margin), rounded outward.
    const float z_low = outward_low(lower(zmin)), z_high = outward_high(upper(zmax));
    if (!std::isfinite(z_low) || !std::isfinite(z_high)) {
        out.status = ObjectGeometryStatus::NonFinite;
        return;
    }
    // The result owns its vertices (an exact-size copy, the one allocation of a build), so the
    // next build() can reuse the scratch without touching a result returned earlier.
    out.geometry.emplace(std::vector<Point>(footprint), z_low, z_high);
    out.status = ObjectGeometryStatus::Located;
}

// Single-precision path (see the header): the same selected points, float arithmetic, no division
// per point, every output widened by options_.float_margin_m.
ObjectGeometryResult ObjectGeometryBuilder::build_float(const DepthRegion& r, int x0, int y0, int x1, int y1,
                                                        std::uint32_t code_lo, std::uint32_t code_hi) {
    ObjectGeometryResult out;
    out.float_points = true;
    const auto region_pixels = static_cast<std::size_t>(x1 - x0) * static_cast<std::size_t>(y1 - y0);
    // The sampler's list when it passes the exact path's checks, else a scan of the region in the
    // same row-major order.
    const std::uint32_t* index = nullptr;
    std::size_t count = 0;
    if (r.selected != nullptr) {
        const std::size_t first = static_cast<std::size_t>(y0) * r.width + static_cast<std::size_t>(x0);
        const std::size_t last = static_cast<std::size_t>(y1 - 1) * r.width + static_cast<std::size_t>(x1 - 1);
        bool ok = r.selected_count > 0 && r.selected_count <= region_pixels;
        std::size_t previous = 0;
        for (std::size_t k = 0; ok && k < r.selected_count; ++k) {
            const std::size_t i = r.selected[k];
            if (i < first || i > last || (k > 0 && i <= previous)) { ok = false; break; }
            const std::uint32_t code = r.depth[i];
            if (code < code_lo || code > code_hi) { ok = false; break; }
            previous = i;
        }
        if (ok) {
            index = r.selected;
            count = r.selected_count;
            out.used_selected = true;
        } else {
            out.selected_rejected = true;
        }
    }
    if (index == nullptr) {
        indices_.clear();
        if (indices_.capacity() < region_pixels) indices_.reserve(region_pixels);
        for (int y = y0; y < y1; ++y) {
            const std::size_t row = static_cast<std::size_t>(y) * r.width;
            const std::uint16_t* depth_row = r.depth + row;
            const std::uint8_t* mask_row =
                r.mask ? r.mask + static_cast<std::size_t>(y - r.bbox_y) * static_cast<std::size_t>(r.bbox_width) : nullptr;
            for (int x = x0; x < x1; ++x) {
                const std::uint32_t code = depth_row[x];
                if (code < code_lo || code > code_hi) continue;
                if (mask_row && mask_row[x - r.bbox_x] == 0) continue;
                indices_.push_back(static_cast<std::uint32_t>(row + static_cast<std::size_t>(x)));
            }
        }
        index = indices_.data();
        count = indices_.size();
    }
    if (points_xf_.size() < count) {
        points_xf_.resize(count);
        points_yf_.resize(count);
    }
    // p_colour = depth * (ray_x, ray_y, 1); p_base = R p_colour + t, in float. A non-finite value in
    // any point reaches `check` (NaN and +-inf propagate through the sum).
    const auto& Rd = base_from_colour_.R;
    const auto& td = base_from_colour_.t;
    const float R0 = static_cast<float>(Rd[0]), R1 = static_cast<float>(Rd[1]), R2 = static_cast<float>(Rd[2]);
    const float R3 = static_cast<float>(Rd[3]), R4 = static_cast<float>(Rd[4]), R5 = static_cast<float>(Rd[5]);
    const float R6 = static_cast<float>(Rd[6]), R7 = static_cast<float>(Rd[7]), R8 = static_cast<float>(Rd[8]);
    const float t0 = static_cast<float>(td[0]), t1 = static_cast<float>(td[1]), t2 = static_cast<float>(td[2]);
    const float scale = r.depth_scale;
    const float* ray_x = rays_.x.data();
    const float* ray_y = rays_.y.data();
    float* px = points_xf_.data();
    float* py = points_yf_.data();
    float xmin = std::numeric_limits<float>::infinity(), xmax = -xmin, ymin = xmin, ymax = -xmin, zmin = xmin, zmax = -xmin;
    float check = 0.0F;
    for (std::size_t k = 0; k < count; ++k) {
        const std::uint32_t i = index[k];
        const float z = static_cast<float>(r.depth[i]) * scale;
        const float cx = z * ray_x[i], cy = z * ray_y[i];
        const float bx = R0 * cx + R1 * cy + R2 * z + t0;
        const float by = R3 * cx + R4 * cy + R5 * z + t1;
        const float bz = R6 * cx + R7 * cy + R8 * z + t2;
        px[k] = bx;
        py[k] = by;
        check += bx + by + bz;
        xmin = std::min(xmin, bx); xmax = std::max(xmax, bx);
        ymin = std::min(ymin, by); ymax = std::max(ymax, by);
        zmin = std::min(zmin, bz); zmax = std::max(zmax, bz);
    }
    if (!std::isfinite(check)) {
        out.status = ObjectGeometryStatus::NonFinite;
        return out;
    }
    out.points = count;
    if (count < options_.min_points) {
        out.status = ObjectGeometryStatus::TooFewPoints;
        return out;
    }
    // Buckets exactly as the exact path, from the float extents.
    double width = options_.bucket_m;
    const double span = static_cast<double>(xmax) - static_cast<double>(xmin);
    if (!std::isfinite(span) || !std::isfinite(static_cast<double>(ymax) - static_cast<double>(ymin)) ||
        !std::isfinite(static_cast<double>(zmax) - static_cast<double>(zmin))) {
        out.status = ObjectGeometryStatus::NonFinite;
        return out;
    }
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
    // Multiply instead of divide: a point next to a bucket edge may land in the neighbour bucket,
    // which the margin covers (its x is then within width + micrometres of that bucket).
    const float inverse_width = static_cast<float>(1.0 / width);
    const std::size_t last_bucket = buckets - 1;
    for (std::size_t k = 0; k < count; ++k) {
        const float bx = px[k], by = py[k];
        auto b = static_cast<std::size_t>((bx - xmin) * inverse_width);  // bx >= xmin
        if (b > last_bucket) b = last_bucket;
        if (by < bucket_min_y_[b]) { bucket_min_y_[b] = by; bucket_min_y_x_[b] = bx; }
        if (by > bucket_max_y_[b]) { bucket_max_y_[b] = by; bucket_max_y_x_[b] = bx; }
    }
    // Each bucket's min-y point moves down and its max-y point up by the margin, both widened
    // along x by width + margin: the hull then contains every exact point (float error << margin).
    const double margin = options_.float_margin_m, grow = width + margin;
    std::vector<P2>& widened = widened_;  // the builder's scratch, as on the exact path
    widened.clear();
    widened.reserve(4 * buckets);
    for (std::size_t b = 0; b < buckets; ++b) {
        if (!std::isfinite(bucket_min_y_[b])) continue;
        widened.push_back({bucket_min_y_x_[b] - grow, bucket_min_y_[b] - margin});
        widened.push_back({bucket_min_y_x_[b] + grow, bucket_min_y_[b] - margin});
        widened.push_back({bucket_max_y_x_[b] - grow, bucket_max_y_[b] + margin});
        widened.push_back({bucket_max_y_x_[b] + grow, bucket_max_y_[b] + margin});
    }
    finish_geometry(out, widened, xmin, xmax, ymin, ymax, zmin, zmax, width, options_.degenerate_half_width_m, margin);
    return out;
}

}  // namespace perception::rns
