/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rns_object_geometry.cpp
 * Brief: Unit tests for measured Objects geometry: footprint hull, height band and range from depth points.
 *
 * Description:
 * What it checks. From the depth points of one detection the builder computes the base_link
 * footprint polygon, z band and near range. Fixtures cover a box whose front face and ground edge
 * are visible, a round pillar whose many hull vertices are decimated to 12 with the extremes kept,
 * a degenerate set of collinear points, invalid inputs and construction, depth band selection
 * checked against the float predicate for every 16-bit code, and the transform helpers.
 * Selected-pixel list (2026-10-06): build() with DepthSampler's list of the selected pixels must
 * give bit-identical results to the region scan on random noisy objects (with and without a mask,
 * many bands, including degenerate and too-few-point cases); lists that are empty, unordered,
 * outside the region, too long, or hold a pixel outside the band are refused, and then the scan
 * result is returned.
 */

/* Host test: measured base_link object geometry (19 section 4.1) from the
 * DepthSampler-selected point set. Synthetic pinhole colour rays and scenes;
 * no SDK, OpenCV, camera or calibration certification. */
#include "output/rns_object_geometry.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

using namespace perception::rns;

namespace {
int checks = 0, failures = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)

constexpr std::uint32_t W = 1280, H = 800;
constexpr double F = 600, CX = 639.5, CY = 399.5;

OpticalRayTable pinhole() {
    OpticalRayTable t;
    t.width = W; t.height = H;
    t.x.resize(W * H); t.y.resize(W * H);
    for (std::uint32_t v = 0; v < H; ++v)
        for (std::uint32_t u = 0; u < W; ++u) {
            t.x[v * W + u] = static_cast<float>((u - CX) / F);
            t.y[v * W + u] = static_cast<float>((v - CY) / F);
        }
    return t;
}
// base (x fwd, y left, z up) <- colour optical (x right, y down, z fwd), camera at
// height h pitched down by `pitch` and shifted laterally by `ty`.
RigidTransform camera(double h, double pitch, double ty) {
    const double s = std::sin(pitch), c = std::cos(pitch);
    RigidTransform T;
    T.R = {0, -s, c, -1, 0, 0, 0, -c, -s};
    T.t = {0.1, ty, h};
    return T;
}

struct Scene {
    std::vector<std::uint16_t> depth = std::vector<std::uint16_t>(W * H, 0);
    std::vector<std::uint8_t> mask;
    int bx0 = W, by0 = H, bx1 = -1, by1 = -1;
    float lo = std::numeric_limits<float>::infinity(), hi = 0;
    std::vector<std::array<double, 3>> points;  // true base-frame points actually rendered
};
constexpr float kScale = 0.001F;

// Render an object given a ray-hit function returning optical z (or <= 0).
template <class Hit>
Scene render(const OpticalRayTable& rays, const RigidTransform& T, Hit hit) {
    Scene s;
    for (std::uint32_t v = 0; v < H; ++v)
        for (std::uint32_t u = 0; u < W; ++u) {
            const std::size_t i = v * W + u;
            const double rx = rays.x[i], ry = rays.y[i];
            const std::array<double, 3> dir{T.R[0] * rx + T.R[1] * ry + T.R[2], T.R[3] * rx + T.R[4] * ry + T.R[5],
                                            T.R[6] * rx + T.R[7] * ry + T.R[8]};
            const double z = hit(dir);
            if (!(z > 0.3 && z < 10)) continue;
            const auto code = static_cast<std::uint16_t>(std::lround(z / kScale));
            s.depth[i] = code;
            s.bx0 = std::min<int>(s.bx0, static_cast<int>(u)); s.bx1 = std::max<int>(s.bx1, static_cast<int>(u));
            s.by0 = std::min<int>(s.by0, static_cast<int>(v)); s.by1 = std::max<int>(s.by1, static_cast<int>(v));
        }
    const int bw = s.bx1 - s.bx0 + 1, bh = s.by1 - s.by0 + 1;
    s.mask.assign(static_cast<std::size_t>(bw) * bh, 0);
    for (int v = s.by0; v <= s.by1; ++v)
        for (int u = s.bx0; u <= s.bx1; ++u) {
            const auto code = s.depth[static_cast<std::size_t>(v) * W + u];
            if (!code) continue;
            s.mask[static_cast<std::size_t>(v - s.by0) * bw + (u - s.bx0)] = 1;
            const float d = static_cast<float>(code) * kScale;
            s.lo = std::min(s.lo, d); s.hi = std::max(s.hi, d);
            const std::size_t i = static_cast<std::size_t>(v) * W + u;
            const double cx = d * rays.x[i], cy = d * rays.y[i], cz = d;
            s.points.push_back({T.R[0] * cx + T.R[1] * cy + T.R[2] * cz + T.t[0], T.R[3] * cx + T.R[4] * cy + T.R[5] * cz + T.t[1],
                                T.R[6] * cx + T.R[7] * cy + T.R[8] * cz + T.t[2]});
        }
    return s;
}

DepthRegion region(const Scene& s, bool with_mask) {
    DepthRegion r;
    r.depth = s.depth.data(); r.width = W; r.height = H; r.depth_scale = kScale;
    r.bbox_x = s.bx0; r.bbox_y = s.by0; r.bbox_width = s.bx1 - s.bx0 + 1; r.bbox_height = s.by1 - s.by0 + 1;
    r.mask = with_mask ? s.mask.data() : nullptr;
    return r;
}

bool inside(const std::vector<Point>& poly, double x, double y, double tol) {
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const auto& a = poly[i];
        const auto& b = poly[(i + 1) % poly.size()];
        const double ex = static_cast<double>(b.x) - a.x, ey = static_cast<double>(b.y) - a.y;
        const double c = ex * (y - a.y) - ey * (x - a.x);
        if (c < -tol * std::hypot(ex, ey)) return false;
    }
    return true;
}
}  // namespace

int main() {
    const auto rays = pinhole();
    const ObjectGeometryOptions opt{5, 0.01, 0.02};
    const auto T = camera(0.8, 12.0 * M_PI / 180.0, 0.25);
    ObjectGeometryBuilder builder(rays, T, opt);

    // ---- box: x in [2, 2.5], y in [-0.3, 0.3], z in [0, 1.2] (front face + ground-level edge visible) ----
    {
        const auto hit_box = [&](const std::array<double, 3>& d) {
            double tmin = 0, tmax = 1e9;
            const double o[3] = {T.t[0], T.t[1], T.t[2]}, lo[3] = {2.0, -0.3, 0.0}, hi[3] = {2.5, 0.3, 1.2};
            for (int k = 0; k < 3; ++k) {
                if (std::abs(d[k]) < 1e-12) { if (o[k] < lo[k] || o[k] > hi[k]) return -1.0; continue; }
                double t1 = (lo[k] - o[k]) / d[k], t2 = (hi[k] - o[k]) / d[k];
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1); tmax = std::min(tmax, t2);
            }
            return tmin <= tmax ? tmin : -1.0;  // optical z == ray parameter because ray.z_optical == 1
        };
        auto s = render(rays, T, hit_box);
        CHECK(s.points.size() > 5000);
        const auto r = builder.build(region(s, true), s.lo, s.hi);
        CHECK(r.status == ObjectGeometryStatus::Located && r.geometry && r.points == s.points.size());
        if (r.geometry) {
            const auto& g = *r.geometry;
            CHECK(valid_footprint(g.footprint_xy) && g.footprint_xy.size() <= 12);
            double zmin = 1e9, zmax = -1e9;
            bool all_inside = true;
            for (const auto& p : s.points) {
                zmin = std::min(zmin, p[2]); zmax = std::max(zmax, p[2]);
                all_inside &= inside(g.footprint_xy, p[0], p[1], 1e-6);
            }
            CHECK(all_inside);
            CHECK(g.z_min <= zmin && g.z_max >= zmax && zmax - zmin > 0.5);   // measured interval, outward rounded
            CHECK(static_cast<double>(g.z_min) >= zmin - 1e-6 && static_cast<double>(g.z_max) <= zmax + 1e-6);
            const double r_near = footprint_near(g.footprint_xy);
            CHECK(r_near > 1.9 && r_near < 2.1);
            std::printf("box footprint vertices=%zu z=[%.3f, %.3f] r_near=%.3f points=%zu hull=%zu\n", g.footprint_xy.size(),
                        g.z_min, g.z_max, r_near, r.points, r.hull_vertices);
            // encoder accepts the bundle
            Objects m;
            m.t_capture_mono_ms = 1000; m.t_publish_mono_ms = 1010;
            Object o;
            o.track_id = 1; o.class_id = 0; o.class_name = "box"; o.confidence = 0.9F; o.semantic_status = "confirmed";
            o.geometry = g; o.velocity_frame = "raw"; o.velocity_status = "warming_up"; o.depth_quality = "poor";
            o.stable_frames = 3; o.first_seen_mono_ms = 900; o.last_seen_mono_ms = 1000;
            m.objects.push_back(o);
            char buf[16384];
            const auto e = encode(m, buf, sizeof buf);
            CHECK(static_cast<bool>(e));
            if (e) CHECK(std::string(buf, e.size).find("\"r_near\":") != std::string::npos);
        }
        // band excluding everything -> too few points (unlocatable, never an image-box fallback)
        const auto none = builder.build(region(s, true), 9.0F, 9.5F);
        CHECK(none.status == ObjectGeometryStatus::TooFewPoints && !none.geometry);
        // narrower band -> subset only (the sampler's selected cluster, not the whole mask)
        const auto part = builder.build(region(s, true), s.lo, s.lo + 0.05F);
        CHECK(part.status == ObjectGeometryStatus::Located && part.points < r.points && part.points > 0);
        // no mask: exactly the bbox centre ROI (quarter margins)
        const auto centre = builder.build(region(s, false), s.lo, s.hi);
        const int bw = s.bx1 - s.bx0 + 1, bh = s.by1 - s.by0 + 1;
        const int x0 = s.bx0 + bw / 4, y0 = s.by0 + bh / 4, x1 = x0 + (bw - 2 * (bw / 4)), y1 = y0 + (bh - 2 * (bh / 4));
        std::size_t expected = 0;
        for (int v = y0; v < y1; ++v)
            for (int u = x0; u < x1; ++u) {
                const float d = static_cast<float>(s.depth[static_cast<std::size_t>(v) * W + u]) * kScale;
                expected += d >= s.lo && d <= s.hi;
            }
        CHECK(centre.points == expected && centre.points < r.points);
        // the mask excludes pixels that are inside the band (sampler: mask zero -> not a sample)
        auto half = s;
        std::size_t kept = 0;
        for (int v = 0; v < bh; ++v)
            for (int u = 0; u < bw; ++u) {
                auto& m = half.mask[static_cast<std::size_t>(v) * bw + u];
                if (u < bw / 2) m = 0;
                const float d = static_cast<float>(half.depth[static_cast<std::size_t>(v + s.by0) * W + (u + s.bx0)]) * kScale;
                kept += m && d >= s.lo && d <= s.hi;
            }
        const auto masked = builder.build(region(half, true), s.lo, s.hi);
        CHECK(masked.points == kept && kept > 0 && kept < r.points);
    }

    // ---- round pillar radius 0.35 at (3, 0): many hull vertices -> decimated to 12, extremes kept ----
    {
        const auto hit_pillar = [&](const std::array<double, 3>& d) {
            const double ox = T.t[0] - 3.0, oy = T.t[1];
            const double a = d[0] * d[0] + d[1] * d[1], b = 2 * (ox * d[0] + oy * d[1]), c = ox * ox + oy * oy - 0.35 * 0.35;
            const double disc = b * b - 4 * a * c;
            if (disc < 0) return -1.0;
            const double t = (-b - std::sqrt(disc)) / (2 * a);
            const double z = T.t[2] + t * d[2];
            return (z >= 0 && z <= 1.5) ? t : -1.0;
        };
        auto s = render(rays, T, hit_pillar);
        const auto r = builder.build(region(s, true), s.lo, s.hi);
        CHECK(r.status == ObjectGeometryStatus::Located && r.decimated && r.hull_vertices > 12);
        if (r.geometry) {
            const auto& g = r.geometry->footprint_xy;
            CHECK(valid_footprint(g) && g.size() == 12);
            double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
            for (const auto& p : s.points) {
                xmin = std::min(xmin, p[0]); xmax = std::max(xmax, p[0]);
                ymin = std::min(ymin, p[1]); ymax = std::max(ymax, p[1]);
            }
            double fxmin = 1e9, fxmax = -1e9, fymin = 1e9, fymax = -1e9;
            for (const auto& p : g) {
                fxmin = std::min<double>(fxmin, p.x); fxmax = std::max<double>(fxmax, p.x);
                fymin = std::min<double>(fymin, p.y); fymax = std::max<double>(fymax, p.y);
            }
            // extreme vertices survive decimation (bucket widening adds <= 1 cm in x)
            CHECK(fxmin <= xmin + 1e-6 && fxmax >= xmax - 1e-6 && fymin <= ymin + 1e-6 && fymax >= ymax - 1e-6);
            CHECK(fxmin >= xmin - 0.011 && fxmax <= xmax + 0.011);
            std::printf("pillar hull=%zu -> %zu vertices, r_near=%.3f\n", r.hull_vertices, g.size(), footprint_near(g));
        }
    }

    // ---- degenerate: all points on one line parallel to x (single row of equal y) ----
    {
        Scene s;
        s.bx0 = 600; s.by0 = 700; s.bx1 = 600; s.by1 = 709;
        s.mask.assign(10, 1);
        for (int v = 700; v < 710; ++v) s.depth[static_cast<std::size_t>(v) * W + 600] = static_cast<std::uint16_t>(2000 + (v - 700));
        // choose a transform where the column maps to a line of constant y: camera looking straight down x
        RigidTransform down;
        down.R = {0, 0, 1, -1, 0, 0, 0, -1, 0};
        down.t = {0, 0, 1};
        ObjectGeometryBuilder b2(rays, down, opt);
        const auto r = b2.build(region(s, true), 2.0F, 2.01F);
        CHECK(r.status == ObjectGeometryStatus::Located && r.geometry && valid_footprint(r.geometry->footprint_xy));
    }

    // ---- invalid inputs and construction ----
    {
        Scene s;
        s.bx0 = 10; s.by0 = 10; s.bx1 = 20; s.by1 = 20;
        auto r = region(s, false);
        CHECK(builder.build(r, 2.0F, 1.0F).status == ObjectGeometryStatus::InvalidInput);
        r.width = 640;
        CHECK(builder.build(r, 1.0F, 2.0F).status == ObjectGeometryStatus::InvalidInput);
        r = region(s, false); r.bbox_x = 5000;
        CHECK(builder.build(r, 1.0F, 2.0F).status == ObjectGeometryStatus::InvalidInput);
        r = region(s, false); r.depth_scale = 0;
        CHECK(builder.build(r, 1.0F, 2.0F).status == ObjectGeometryStatus::InvalidInput);
        bool bad_rays = false, bad_T = false, bad_opt = false;
        auto nan_rays = rays; nan_rays.x[5] = std::numeric_limits<float>::quiet_NaN();
        try { ObjectGeometryBuilder b(nan_rays, T, opt); } catch (const std::invalid_argument&) { bad_rays = true; }
        auto scaled = T; scaled.R[0] = 2;
        try { ObjectGeometryBuilder b(rays, scaled, opt); } catch (const std::invalid_argument&) { bad_T = true; }
        try { ObjectGeometryBuilder b(rays, T, {0, 0.01, 0.02}); } catch (const std::invalid_argument&) { bad_opt = true; }
        CHECK(bad_rays && bad_T && bad_opt);
    }

    // ---- depth band selection == float predicate over every code (0..65535) ----
    {
        OpticalRayTable all;
        all.width = 256; all.height = 256;
        all.x.assign(256 * 256, 0.0F); all.y.assign(256 * 256, 0.0F);
        for (std::size_t i = 0; i < all.x.size(); ++i) {
            all.x[i] = (static_cast<int>(i % 256) - 128) / 200.0F;
            all.y[i] = (static_cast<int>(i / 256) - 128) / 200.0F;
        }
        ObjectGeometryBuilder every(all, T, {1, 0.01, 0.02});
        std::vector<std::uint16_t> codes(256 * 256);
        for (std::size_t i = 0; i < codes.size(); ++i) codes[i] = static_cast<std::uint16_t>(i);
        std::vector<std::uint8_t> full(256 * 256, 255);
        DepthRegion r;
        r.depth = codes.data(); r.width = 256; r.height = 256; r.bbox_width = 256; r.bbox_height = 256; r.mask = full.data();
        std::uint64_t seed = 0x2545F4914F6CDD1DULL;
        const auto rnd = [&]() { seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17; return (seed >> 11) * (1.0 / 9007199254740992.0); };
        int mismatches = 0, cases = 0;
        for (int k = 0; k < 400; ++k) {
            const float scale = k < 4 ? std::array<float, 4>{0.001F, 0.00025F, 0.0001F, 1.0F}[k]
                                      : static_cast<float>(std::pow(10.0, -4.5 + 4.0 * rnd()));
            r.depth_scale = scale;
            float lo, hi;
            if (k % 3 == 0) {  // boundaries exactly on representable code values and their neighbours
                const auto c1 = static_cast<std::uint32_t>(rnd() * 65536), c2 = static_cast<std::uint32_t>(rnd() * 65536);
                lo = static_cast<float>(std::min(c1, c2)) * scale; hi = static_cast<float>(std::max(c1, c2)) * scale;
                if (k % 2) { lo = std::nextafter(lo, 1e30F); hi = std::nextafter(hi, -1e30F); }
            } else {
                lo = static_cast<float>(rnd() * 70000.0 * scale); hi = lo + static_cast<float>(rnd() * 30000.0 * scale);
            }
            if (!(lo <= hi)) continue;
            std::size_t expected = 0;
            for (std::uint32_t c = 0; c < 65536; ++c) {
                const float m = static_cast<float>(c) * scale;
                expected += std::isfinite(m) && m >= lo && m <= hi;
            }
            const auto got = every.build(r, lo, hi);
            ++cases;
            mismatches += got.points != expected;
        }
        CHECK(cases > 300 && mismatches == 0);
        // empty bands at both ends and the full code range
        r.depth_scale = 0.001F;
        const auto below = every.build(r, -2.0F, -1.0F);
        CHECK(below.points == 0 && below.status == ObjectGeometryStatus::TooFewPoints);
        CHECK(every.build(r, 70.0F, 80.0F).points == 0);
        CHECK(every.build(r, -1.0F, 1e30F).points == 65536);
        // huge but finite coordinates: no undefined conversion, bounded buckets, a decided status
        auto wide = all;
        for (auto& v : wide.x) v *= 1e15F;
        ObjectGeometryBuilder far(wide, T, {1, 0.01, 0.02});
        r.depth_scale = 1.0F;
        const auto huge = far.build(r, 0.0F, 1e30F);
        CHECK(huge.points == 65536 && (huge.status == ObjectGeometryStatus::Located || huge.status == ObjectGeometryStatus::NonFinite));
        if (huge.geometry) CHECK(valid_footprint(huge.geometry->footprint_xy));
    }

    // ---- DepthSampler's selected-pixel list instead of the region scan ----
    {
        // Bit-identical results: every field, the footprint and z interval by their bits.
        const auto same = [](const ObjectGeometryResult& a, const ObjectGeometryResult& b) {
            bool ok = a.status == b.status && a.points == b.points && a.hull_vertices == b.hull_vertices &&
                      a.degenerate == b.degenerate && a.decimated == b.decimated &&
                      a.rectangle_fallback == b.rectangle_fallback && a.geometry.has_value() == b.geometry.has_value();
            if (ok && a.geometry) {
                const auto& fa = a.geometry->footprint_xy;
                const auto& fb = b.geometry->footprint_xy;
                ok = fa.size() == fb.size() &&
                     (fa.empty() || std::memcmp(fa.data(), fb.data(), fa.size() * sizeof(Point)) == 0) &&
                     std::memcmp(&a.geometry->z_min, &b.geometry->z_min, sizeof(float)) == 0 &&
                     std::memcmp(&a.geometry->z_max, &b.geometry->z_max, sizeof(float)) == 0;
            }
            return ok;
        };
        // The list the sampler hands over: the region's pixels whose depth lies in [lo, hi], row-major.
        const auto listed = [](const DepthRegion& r, float lo, float hi) {
            std::vector<std::uint32_t> out;
            int x0 = std::max(0, r.bbox_x), y0 = std::max(0, r.bbox_y);
            int x1 = std::min(static_cast<int>(r.width), r.bbox_x + r.bbox_width);
            int y1 = std::min(static_cast<int>(r.height), r.bbox_y + r.bbox_height);
            if (!r.mask) {
                const int w = x1 - x0, h = y1 - y0, mx = w / 4, my = h / 4;
                x0 += mx; y0 += my; x1 = x0 + (w - 2 * mx); y1 = y0 + (h - 2 * my);
            }
            for (int y = y0; y < y1; ++y)
                for (int x = x0; x < x1; ++x) {
                    if (r.mask && !r.mask[static_cast<std::size_t>(y - r.bbox_y) * r.bbox_width + (x - r.bbox_x)]) continue;
                    const float d = static_cast<float>(r.depth[static_cast<std::size_t>(y) * r.width + x]) * r.depth_scale;
                    if (d >= lo && d <= hi) out.push_back(static_cast<std::uint32_t>(y) * r.width + static_cast<std::uint32_t>(x));
                }
            return out;
        };
        std::vector<std::uint16_t> depth(W * H);
        std::uint64_t state = 338;
        const auto next = [&state]() { state = state * 6364136223846793005ULL + 1442695040888963407ULL; return state >> 33; };
        int compared = 0, mismatches = 0, located = 0, rejected_wrong = 0;
        for (int scene = 0; scene < 12; ++scene) {
            for (auto& c : depth) c = static_cast<std::uint16_t>(3000 + next() % 4000);
            const int bw = 40 + static_cast<int>(next() % 360), bh = 40 + static_cast<int>(next() % 300);
            const int bx = static_cast<int>(next() % (W - 40)) - 20, by = static_cast<int>(next() % (H - 40)) - 20;
            std::vector<std::uint8_t> mask(static_cast<std::size_t>(bw) * bh);
            const unsigned keep = 30 + next() % 70;
            for (auto& m : mask) m = next() % 100 < keep ? 1 : 0;
            for (int y = std::max(0, by); y < std::min<int>(H, by + bh); ++y)
                for (int x = std::max(0, bx); x < std::min<int>(W, bx + bw); ++x)
                    depth[static_cast<std::size_t>(y) * W + x] =
                        static_cast<std::uint16_t>(scene == 11 ? 2000 : 1500 + next() % 1500);  // 11: one flat depth
            for (const bool with_mask : {true, false}) {
                DepthRegion r;
                r.depth = depth.data(); r.width = W; r.height = H; r.depth_scale = kScale;
                r.bbox_x = bx; r.bbox_y = by; r.bbox_width = bw; r.bbox_height = bh;
                r.mask = with_mask ? mask.data() : nullptr;
                for (const auto& band : std::vector<std::array<float, 2>>{
                         {1.5F, 3.0F}, {1.8F, 2.1F}, {2.0F, 2.0F}, {2.95F, 2.999F}, {0.1F, 0.2F}}) {
                    const auto list = listed(r, band[0], band[1]);
                    const auto scanned = builder.build(r, band[0], band[1]);
                    DepthRegion with_list = r;
                    // An empty vector's data() may be null (= no list); point at a dummy so the
                    // empty list itself is what build() sees and must refuse.
                    static const std::uint32_t dummy = 0;
                    with_list.selected = list.empty() ? &dummy : list.data();
                    with_list.selected_count = list.size();
                    const auto from_list = builder.build(with_list, band[0], band[1]);
                    ++compared;
                    if (list.empty()) {
                        // An empty list is never a valid sample's list: refused, region scanned.
                        mismatches += !(from_list.selected_rejected && same(from_list, scanned));
                        continue;
                    }
                    mismatches += !(from_list.used_selected && !from_list.selected_rejected && same(from_list, scanned));
                    located += from_list.status == ObjectGeometryStatus::Located;
                    if (list.size() < 2) continue;
                    // Corrupted lists: each must be refused and give the scan's result.
                    std::vector<std::vector<std::uint32_t>> bad(4, list);
                    std::swap(bad[0][0], bad[0][1]);                         // out of order
                    bad[1].back() = static_cast<std::uint32_t>(W * H + 5);    // outside the image
                    bad[2].push_back(bad[2].back());                          // repeated index
                    std::uint32_t outside_band = 0;                           // a region pixel outside the band
                    for (std::uint32_t i = list.front(); i <= list.back(); ++i) {
                        const float d = static_cast<float>(depth[i]) * kScale;
                        if (d < band[0] || d > band[1]) { outside_band = i; break; }
                    }
                    if (outside_band) bad[3].insert(std::upper_bound(bad[3].begin(), bad[3].end(), outside_band), outside_band);
                    for (std::size_t k = 0; k < bad.size(); ++k) {
                        if (k == 3 && !outside_band) continue;
                        DepthRegion wrong = r;
                        wrong.selected = bad[k].data();
                        wrong.selected_count = bad[k].size();
                        const auto refused = builder.build(wrong, band[0], band[1]);
                        rejected_wrong += refused.selected_rejected && !refused.used_selected && same(refused, scanned);
                        ++compared;
                        mismatches += !(refused.selected_rejected && same(refused, scanned));
                    }
                }
            }
        }
        // Too long a list (more indices than region pixels) is refused before it is read.
        {
            DepthRegion r;
            r.depth = depth.data(); r.width = W; r.height = H; r.depth_scale = kScale;
            r.bbox_x = 10; r.bbox_y = 10; r.bbox_width = 4; r.bbox_height = 4;
            std::vector<std::uint32_t> many(17, 0);
            for (std::size_t i = 0; i < many.size(); ++i) many[i] = static_cast<std::uint32_t>(i);
            r.selected = many.data();
            r.selected_count = many.size();
            CHECK(builder.build(r, 0.0F, 100.0F).selected_rejected);
        }
        CHECK(compared > 200 && mismatches == 0 && located > 50 && rejected_wrong > 100);
    }

    // ---- single-precision path (float_margin_m > 0): conservative by the margin ----
    {
        const ObjectGeometryOptions fast_opt{5, 0.01, 0.02, 0.001};
        ObjectGeometryBuilder fast(rays, T, fast_opt);
        // Margins outside [0.1 mm, 1 cm] (or not finite, or negative) are refused at construction.
        for (const double bad : {-0.001, 5e-5, 0.02, std::numeric_limits<double>::quiet_NaN()}) {
            bool thrown = false;
            try { ObjectGeometryBuilder b(rays, T, {5, 0.01, 0.02, bad}); } catch (const std::invalid_argument&) { thrown = true; }
            CHECK(thrown);
        }
        // Signed distance of (x, y) outside a convex CCW polygon (<= 0 inside).
        const auto outside = [](const std::vector<Point>& poly, double x, double y) {
            double worst = -std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < poly.size(); ++i) {
                const auto& a = poly[i];
                const auto& b = poly[(i + 1) % poly.size()];
                const double ex = static_cast<double>(b.x) - a.x, ey = static_cast<double>(b.y) - a.y;
                worst = std::max(worst, -(ex * (y - a.y) - ey * (x - a.x)) / std::hypot(ex, ey));
            }
            return worst;
        };
        // The exact (double) points of a region and band, by the exact path's expression.
        const auto exact_points = [&](const DepthRegion& r, float lo, float hi) {
            std::vector<std::array<double, 3>> pts;
            int x0 = std::max(0, r.bbox_x), y0 = std::max(0, r.bbox_y);
            int x1 = std::min(static_cast<int>(r.width), r.bbox_x + r.bbox_width);
            int y1 = std::min(static_cast<int>(r.height), r.bbox_y + r.bbox_height);
            if (!r.mask) {
                const int w = x1 - x0, h = y1 - y0, mx = w / 4, my = h / 4;
                x0 += mx; y0 += my; x1 = x0 + (w - 2 * mx); y1 = y0 + (h - 2 * my);
            }
            for (int y = y0; y < y1; ++y)
                for (int x = x0; x < x1; ++x) {
                    if (r.mask && !r.mask[static_cast<std::size_t>(y - r.bbox_y) * r.bbox_width + (x - r.bbox_x)]) continue;
                    const std::size_t i = static_cast<std::size_t>(y) * r.width + x;
                    const float d = static_cast<float>(r.depth[i]) * r.depth_scale;
                    if (!(d >= lo && d <= hi)) continue;
                    const double z = d, cx = z * rays.x[i], cy = z * rays.y[i];
                    pts.push_back({T.R[0] * cx + T.R[1] * cy + T.R[2] * z + T.t[0], T.R[3] * cx + T.R[4] * cy + T.R[5] * z + T.t[1],
                                   T.R[6] * cx + T.R[7] * cy + T.R[8] * z + T.t[2]});
                }
            return pts;
        };
        // Compare one fast result with the exact one on the same input.
        int cases = 0, located = 0, decimated = 0, failed = 0, farther_than_exact = 0;
        double worst_z = 0, worst_extent = 0, worst_farther = 0, worst_out_fast = -1, worst_out_exact = -1;
        const auto compare = [&](const DepthRegion& r, float lo, float hi, const char* name) {
            const auto exact = builder.build(r, lo, hi);
            const auto f = fast.build(r, lo, hi);
            ++cases;
            bool ok = f.float_points && !exact.float_points && f.status == exact.status && f.points == exact.points;
            if (ok && exact.status == ObjectGeometryStatus::Located && f.geometry && exact.geometry) {
                ++located;
                const auto pts = exact_points(r, lo, hi);
                double zmin = 1e30, zmax = -1e30, out_fast = -1e30, out_exact = -1e30;
                for (const auto& p : pts) {
                    zmin = std::min(zmin, p[2]); zmax = std::max(zmax, p[2]);
                    out_fast = std::max(out_fast, outside(f.geometry->footprint_xy, p[0], p[1]));
                    out_exact = std::max(out_exact, outside(exact.geometry->footprint_xy, p[0], p[1]));
                }
                const auto& g = *f.geometry;
                // z: contains every exact height, at most the margin plus float error wider.
                ok &= g.z_min <= zmin && g.z_max >= zmax && g.z_min >= zmin - 0.0011 && g.z_max <= zmax + 0.0011;
                worst_z = std::max({worst_z, zmin - g.z_min, g.z_max - zmax});
                if (!f.decimated && !exact.decimated) {
                    // No vertex cap involved: every exact point inside, strictly.
                    ok &= out_fast <= 0.0;
                    ok &= footprint_near(g.footprint_xy) <= footprint_near(exact.geometry->footprint_xy) + 1e-6;
                } else {
                    ++decimated;
                    // The 12-vertex cap cuts corners in both paths, but not always the same ones. The
                    // fast footprint may not stick out further than the exact one (plus 2 mm), and its
                    // nearest distance may not exceed the true nearest point, unless it is still no
                    // farther than the exact output's (the safety reading of "never farther").
                    double true_near = 1e30;
                    for (const auto& p : pts) true_near = std::min(true_near, std::hypot(p[0], p[1]));
                    const double near_fast = footprint_near(g.footprint_xy), near_exact = footprint_near(exact.geometry->footprint_xy);
                    ok &= out_fast <= std::max(0.0, out_exact) + 0.002;
                    ok &= near_fast <= true_near + 1e-6 || near_fast <= near_exact + 1e-6;
                    if (near_fast > near_exact + 1e-6) {
                        ++farther_than_exact;
                        worst_farther = std::max(worst_farther, near_fast - near_exact);
                    }
                    worst_out_fast = std::max(worst_out_fast, out_fast);
                    worst_out_exact = std::max(worst_out_exact, out_exact);
                }
                // Not much larger either: extent within margin + bucket rounding of the exact one.
                float fx0 = 1e30F, fx1 = -1e30F, fy0 = 1e30F, fy1 = -1e30F, ex0 = 1e30F, ex1 = -1e30F, ey0 = 1e30F, ey1 = -1e30F;
                for (const auto& v : g.footprint_xy) { fx0 = std::min(fx0, v.x); fx1 = std::max(fx1, v.x); fy0 = std::min(fy0, v.y); fy1 = std::max(fy1, v.y); }
                for (const auto& v : exact.geometry->footprint_xy) { ex0 = std::min(ex0, v.x); ex1 = std::max(ex1, v.x); ey0 = std::min(ey0, v.y); ey1 = std::max(ey1, v.y); }
                const double extent = std::max({static_cast<double>(ex0) - fx0, static_cast<double>(fx1) - ex1,
                                                static_cast<double>(ey0) - fy0, static_cast<double>(fy1) - ey1});
                worst_extent = std::max(worst_extent, extent);
                ok &= extent <= 0.0011;
            }
            if (!ok) {
                ++failed;
                std::cerr << "float path mismatch: " << name << " points=" << exact.points << " status=" << static_cast<int>(f.status)
                          << "/" << static_cast<int>(exact.status) << " decimated=" << f.decimated << "/" << exact.decimated
                          << " degenerate=" << f.degenerate << "/" << exact.degenerate << " fallback=" << f.rectangle_fallback << "/"
                          << exact.rectangle_fallback << "\n";
                if (f.geometry && exact.geometry) {
                    const auto pts = exact_points(r, lo, hi);
                    double out_fast = -1e30, out_exact = -1e30, zmin = 1e30, zmax = -1e30;
                    for (const auto& p : pts) {
                        zmin = std::min(zmin, p[2]); zmax = std::max(zmax, p[2]);
                        out_fast = std::max(out_fast, outside(f.geometry->footprint_xy, p[0], p[1]));
                        out_exact = std::max(out_exact, outside(exact.geometry->footprint_xy, p[0], p[1]));
                    }
                    std::cerr << "  out_fast=" << out_fast << " out_exact=" << out_exact << " near=" << footprint_near(f.geometry->footprint_xy)
                              << "/" << footprint_near(exact.geometry->footprint_xy) << " z=" << f.geometry->z_min << "," << f.geometry->z_max
                              << " exact_z=" << zmin << "," << zmax << " verts=" << f.geometry->footprint_xy.size() << "/"
                              << exact.geometry->footprint_xy.size() << "\n";
                }
            }
        };
        // The box and pillar scenes above, and random regions with several bands, with and without a mask.
        {
            const auto hit_pillar = [&](const std::array<double, 3>& d) {
                // vertical cylinder radius 0.15 at (3, 0.4), z in [0, 2]
                const double ox = T.t[0] - 3.0, oy = T.t[1] - 0.4;
                const double a = d[0] * d[0] + d[1] * d[1], b = 2 * (ox * d[0] + oy * d[1]), c = ox * ox + oy * oy - 0.0225;
                const double disc = b * b - 4 * a * c;
                if (disc < 0 || a < 1e-12) return -1.0;
                const double t = (-b - std::sqrt(disc)) / (2 * a);
                const double z = T.t[2] + t * d[2];
                return (t > 0 && z >= 0 && z <= 2) ? t : -1.0;
            };
            auto s = render(rays, T, hit_pillar);
            compare(region(s, true), s.lo, s.hi, "pillar mask");
            compare(region(s, false), s.lo, s.hi, "pillar box");
        }
        std::vector<std::uint16_t> depth(W * H);
        std::uint64_t state = 4242;
        const auto next = [&state]() { state = state * 6364136223846793005ULL + 1442695040888963407ULL; return state >> 33; };
        for (int scene = 0; scene < 16; ++scene) {
            for (auto& c : depth) c = static_cast<std::uint16_t>(3000 + next() % 4000);
            const int bw = 40 + static_cast<int>(next() % 360), bh = 40 + static_cast<int>(next() % 300);
            const int bx = static_cast<int>(next() % (W - 40)) - 20, by = static_cast<int>(next() % (H - 40)) - 20;
            std::vector<std::uint8_t> mask(static_cast<std::size_t>(bw) * bh);
            const unsigned keep = 30 + next() % 70;
            for (auto& m : mask) m = next() % 100 < keep ? 1 : 0;
            for (int y = std::max(0, by); y < std::min<int>(H, by + bh); ++y)
                for (int x = std::max(0, bx); x < std::min<int>(W, bx + bw); ++x)
                    depth[static_cast<std::size_t>(y) * W + x] =
                        static_cast<std::uint16_t>(scene == 5 ? 2000 : 1500 + next() % 1500);  // 5: one flat depth
            for (const bool with_mask : {true, false}) {
                DepthRegion r;
                r.depth = depth.data(); r.width = W; r.height = H; r.depth_scale = kScale;
                r.bbox_x = bx; r.bbox_y = by; r.bbox_width = bw; r.bbox_height = bh;
                r.mask = with_mask ? mask.data() : nullptr;
                for (const auto& band : std::vector<std::array<float, 2>>{
                         {1.5F, 3.0F}, {1.8F, 2.1F}, {2.0F, 2.0F}, {2.95F, 2.999F}, {0.1F, 0.2F}}) {
                    compare(r, band[0], band[1], "random");
                    // The sampler's list gives the same fast result as the fast scan, bit for bit.
                    std::vector<std::uint32_t> list;
                    for (int y = std::max(0, r.bbox_y); y < std::min<int>(H, r.bbox_y + r.bbox_height); ++y) {
                        if (!r.mask) break;
                        for (int x = std::max(0, r.bbox_x); x < std::min<int>(W, r.bbox_x + r.bbox_width); ++x) {
                            if (!r.mask[static_cast<std::size_t>(y - r.bbox_y) * r.bbox_width + (x - r.bbox_x)]) continue;
                            const float d = static_cast<float>(depth[static_cast<std::size_t>(y) * W + x]) * kScale;
                            if (d >= band[0] && d <= band[1]) list.push_back(static_cast<std::uint32_t>(y) * W + static_cast<std::uint32_t>(x));
                        }
                    }
                    if (with_mask && !list.empty()) {
                        DepthRegion with_list = r;
                        with_list.selected = list.data();
                        with_list.selected_count = list.size();
                        const auto a = fast.build(with_list, band[0], band[1]);
                        const auto b = fast.build(r, band[0], band[1]);
                        bool same_bits = a.used_selected && !b.used_selected && a.status == b.status && a.points == b.points &&
                                         a.geometry.has_value() == b.geometry.has_value();
                        if (same_bits && a.geometry) {
                            const auto& fa = a.geometry->footprint_xy;
                            const auto& fb = b.geometry->footprint_xy;
                            same_bits = fa.size() == fb.size() && std::memcmp(fa.data(), fb.data(), fa.size() * sizeof(Point)) == 0 &&
                                        a.geometry->z_min == b.geometry->z_min && a.geometry->z_max == b.geometry->z_max;
                        }
                        CHECK(same_bits);
                    }
                }
            }
        }
        std::printf("float path: cases=%d located=%d decimated=%d failed=%d worst_z_widening=%.6f worst_extent_growth=%.6f "
                    "capped: nearest_farther_than_exact=%d (worst %.6f m, still not farther than the true nearest point) "
                    "worst_outside fast=%.6f exact=%.6f\n",
                    cases, located, decimated, failed, worst_z, worst_extent, farther_than_exact, worst_farther, worst_out_fast,
                    worst_out_exact);
        CHECK(failed == 0 && located > 60 && decimated > 0);
    }

    // ---- transform helpers ----
    {
        const auto A = camera(0.7, 0.3, -0.2);
        auto B = camera(1.1, -0.2, 0.4);
        B.t = {0.05, -0.02, 0.01};
        const auto I = compose(A, inverse(A));
        bool identity = true;
        for (int i = 0; i < 9; ++i) identity &= std::abs(I.R[i] - (i % 4 == 0 ? 1.0 : 0.0)) < 1e-12;
        for (int i = 0; i < 3; ++i) identity &= std::abs(I.t[i]) < 1e-12;
        CHECK(identity && valid_rigid_transform(compose(A, B)));
        const std::array<double, 3> p{0.3, -0.7, 2.2};
        const auto apply = [](const RigidTransform& X, const std::array<double, 3>& q) {
            return std::array<double, 3>{X.R[0] * q[0] + X.R[1] * q[1] + X.R[2] * q[2] + X.t[0],
                                         X.R[3] * q[0] + X.R[4] * q[1] + X.R[5] * q[2] + X.t[1],
                                         X.R[6] * q[0] + X.R[7] * q[1] + X.R[8] * q[2] + X.t[2]};
        };
        const auto lhs = apply(compose(A, B), p), rhs = apply(A, apply(B, p));
        CHECK(std::abs(lhs[0] - rhs[0]) < 1e-12 && std::abs(lhs[1] - rhs[1]) < 1e-12 && std::abs(lhs[2] - rhs[2]) < 1e-12);
    }
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"scope\":\"object_geometry_host\",\"camera\":false,\"sdk\":false}" << std::endl;
    return failures ? 1 : 0;
}
