/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_native_geometry_pixel_record.cpp
 * Brief: The pixel records of NativeGeometry::process against the frozen parent, bit for bit.
 *
 * Description:
 * Since 2026-10-14 (pixel-record slimming step 1) the fast-path scan writes no pixel record; the
 * classification pass assembles each record once, taking bin and range from the per-pixel tables
 * with the scan's own expressions. Three checks over synthetic scenes (ground, boxes, overhead
 * returns, pits that give pierce candidates and below-plane returns, holes, random invalid depth,
 * one frame per scene whose fit fails), on the fast path (no lateral offset) and on the general
 * path (lateral offset):
 *   1. every output of process() -- the result without its times, every field of every pixel
 *      record, the bins, the raw counts, ground_depth / ground_point of every pixel -- equals, bit
 *      for bit, the frozen parent class (tests/reference/native_geometry_parent.*), frame after
 *      frame on one long-lived object of each;
 *   2. history independence: the same outputs come from a geometry constructed fresh for that one
 *      frame, so nothing of an earlier frame's record survives. The scenes move their pits and
 *      holes from frame to frame, so pixels lose a pierce candidate or turn invalid;
 *   3. coverage: each path saw every class, pierce candidates that vanish in the next frame,
 *      pixels that turn invalid, and a failed fit.
 * Since 2026-10-24 (G-P1 phase 1) the general-path scan starts its bin search from the previous
 * pixel's bin and a bounded scan skips its finiteness checks; a fourth check sweeps that search over
 * random points and every kind of start against the reference bin. One case uses the lateral offset
 * of the .26 vehicle extrinsic (0.345, 0.060 m). Every case has a bounded scan.
 * Since 2026-10-25 (G-P1 phase 2) the general-path scan has its own inline bin lookup, an inline copy of
 * glibc 2.35's hypot and local counters: a fifth check sweeps that lookup (random points and points
 * hugging every bin boundary) against the reference bin, a sixth compares the scan's range with
 * std::hypot over 1.5 M inputs (meaningful under glibc 2.35 on aarch64, where it is the inline copy),
 * and a seventh makes an unbounded scan fail at a middle pixel and compares the counts with the parent.
 * Built with -O2 like the product. Fixture numbers only; nothing here certifies calibration.
 */

#include "modules/native_geometry.hpp"
#include "native_geometry_parent.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

namespace {
using namespace perception::geometry;
namespace nd = perception::native_depth;
namespace parent = perception::geometry::native_geometry_parent;

[[noreturn]] void fail(const char* expression, int line) {
    std::cerr << "native geometry pixel record CHECK failed at line " << line << ": " << expression << '\n';
    std::abort();
}
#define CHECK(expression) do { if (!(expression)) fail(#expression, __LINE__); } while (false)

constexpr double kPi = 3.14159265358979323846;

// Bitwise equality of doubles (tells -0.0 from 0.0 and keeps NaN payloads apart).
bool same_bits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }
bool same_bits(const std::optional<double>& a, const std::optional<double>& b) {
    return a.has_value() == b.has_value() && (!a || same_bits(*a, *b));
}
bool same_bits(Vec3 a, Vec3 b) { return same_bits(a.x, b.x) && same_bits(a.y, b.y) && same_bits(a.z, b.z); }

// Everything process() leaves readable, copied out (the record vector is reused by the next call).
struct Snapshot {
    GeometryResult result;
    std::vector<PixelEvidence> pixels;
    std::vector<BinEvidence> bins;
    std::vector<std::uint32_t> raw;
    std::vector<std::uint8_t> has_depth, has_point;
    std::vector<double> depth;
    std::vector<Vec3> point;
    std::size_t storage = 0;
};

template <class Geometry>
Snapshot snapshot(const Geometry& g) {
    Snapshot s;
    s.result = g.result();
    s.pixels = g.pixels();
    s.bins = g.bins();
    s.raw = g.raw_sample_counts();
    s.has_depth.resize(kGeometryPixels);
    s.has_point.resize(kGeometryPixels);
    s.depth.resize(kGeometryPixels);
    s.point.resize(kGeometryPixels);
    for (std::size_t p = 0; p < kGeometryPixels; ++p) {
        s.has_depth[p] = g.ground_depth(p, s.depth[p]) ? 1 : 0;
        s.has_point[p] = g.ground_point(p, s.point[p]) ? 1 : 0;
    }
    s.storage = g.storage_bytes();
    return s;
}

bool same_source(const nd::SourceId& a, const nd::SourceId& b) {
    return a.stream_id == b.stream_id && a.sequence == b.sequence && a.color_index == b.color_index &&
           a.depth_index == b.depth_index && a.color_device_us == b.color_device_us &&
           a.depth_device_us == b.depth_device_us && a.host_ready_mono_ns == b.host_ready_mono_ns &&
           a.capture_mono_ns == b.capture_mono_ns;
}

bool same_result(const GeometryResult& a, const GeometryResult& b) {
    const auto& fa = a.fit;
    const auto& fb = b.fit;
    if (a.status != b.status || !same_source(a.source, b.source) || fa.status != fb.status ||
        fa.plane.has_value() != fb.plane.has_value() || fa.candidates != fb.candidates || fa.inliers != fb.inliers ||
        fa.hypotheses_tested != fb.hypotheses_tested || !same_bits(fa.inlier_fraction, fb.inlier_fraction) ||
        !same_bits(fa.inlier_rms_m, fb.inlier_rms_m))
        return false;
    if (fa.plane && (!same_bits(fa.plane->normal, fb.plane->normal) || !same_bits(fa.plane->d, fb.plane->d))) return false;
    return a.visited_pixels == b.visited_pixels && a.valid_depth_pixels == b.valid_depth_pixels &&
           a.roi_pixels == b.roi_pixels && a.invalid_roi_pixels == b.invalid_roi_pixels && a.classes == b.classes &&
           same_bits(a.nearest_ground_m, b.nearest_ground_m) &&
           a.raw_sample_evidence_valid == b.raw_sample_evidence_valid;
}

bool same_record(const PixelEvidence& a, const PixelEvidence& b) {
    return a.classification == b.classification && a.bin == b.bin && same_bits(a.range_m, b.range_m) &&
           same_bits(a.height_m, b.height_m) && a.pierce_bin == b.pierce_bin &&
           same_bits(a.pierce_range_m, b.pierce_range_m);
}

bool same_bin(const BinEvidence& a, const BinEvidence& b) {
    return a.valid_samples == b.valid_samples && a.ground_samples == b.ground_samples &&
           same_bits(a.nearest_positive_m, b.nearest_positive_m) &&
           same_bits(a.positive_height_window_max_m, b.positive_height_window_max_m) &&
           same_bits(a.nearest_below_m, b.nearest_below_m) && same_bits(a.below_min_height_m, b.below_min_height_m) &&
           same_bits(a.nearest_pierce_expected_m, b.nearest_pierce_expected_m);
}

// Every field of a against b; names the first difference.
void check_same(const Snapshot& a, const Snapshot& b, const char* against, int frame) {
    const auto report = [&](const char* what, std::size_t index) {
        std::cerr << "frame " << frame << " differs from " << against << ": " << what << " " << index << '\n';
    };
    if (!same_result(a.result, b.result)) { report("result", 0); CHECK(false); }
    CHECK(a.pixels.size() == kGeometryPixels && b.pixels.size() == kGeometryPixels);
    for (std::size_t p = 0; p < kGeometryPixels; ++p)
        if (!same_record(a.pixels[p], b.pixels[p])) { report("pixel record", p); CHECK(false); }
    CHECK(a.bins.size() == b.bins.size());
    for (std::size_t i = 0; i < a.bins.size(); ++i)
        if (!same_bin(a.bins[i], b.bins[i])) { report("bin", i); CHECK(false); }
    if (a.raw != b.raw) { report("raw counts", 0); CHECK(false); }
    for (std::size_t p = 0; p < kGeometryPixels; ++p) {
        if (a.has_depth[p] != b.has_depth[p] || !same_bits(a.depth[p], b.depth[p])) { report("ground_depth", p); CHECK(false); }
        if (a.has_point[p] != b.has_point[p] || !same_bits(a.point[p], b.point[p])) { report("ground_point", p); CHECK(false); }
    }
    CHECK(a.storage == b.storage);
}

// Deterministic generator (splitmix64), as in the other synthetic scene tests.
std::uint64_t next(std::uint64_t& s) {
    s += 0x9e3779b97f4a7c15ULL;
    std::uint64_t z = s;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}
double uniform(std::uint64_t& s) { return static_cast<double>(next(s) >> 11) * (1.0 / 9007199254740992.0); }

// base <- camera: optical (x right, y down, z forward) -> level -> pitch down -> yaw.
std::array<double, 9> pitch_yaw(double pitch, double yaw) {
    const double level[3][3] = {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}};
    const double cp = std::cos(pitch), sp = std::sin(pitch), cy = std::cos(yaw), sy = std::sin(yaw);
    const double p[3][3] = {{cp, 0, sp}, {0, 1, 0}, {-sp, 0, cp}};
    const double y[3][3] = {{cy, -sy, 0}, {sy, cy, 0}, {0, 0, 1}};
    double yp[3][3] = {}, r[3][3] = {};
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
            for (int k = 0; k < 3; ++k) yp[a][b] += y[a][k] * p[k][b];
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
            for (int k = 0; k < 3; ++k) r[a][b] += yp[a][k] * level[k][b];
    return {r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]};
}

struct Case {
    double height_m, pitch_deg, yaw_deg, tx_m, ty_m;
    bool fast;  // expected NativeGeometry::fast_azimuth()
};

// Per-path coverage of the situations the record assembly must handle.
struct Coverage {
    std::array<std::size_t, static_cast<std::size_t>(SampleClass::Count)> classes{};
    std::size_t frames = 0, fit_failed = 0, pierce = 0, pierce_vanished = 0, turned_invalid = 0;
};

GeometryOptions scene_options() {
    GeometryOptions o;
    o.prior = {{0, 0, 1}, 0};
    o.fit.inlier_distance_m = 0.02;
    o.fit.min_inlier_fraction = 0.5;
    o.fit.max_inlier_rms_m = 0.02;
    o.fit.min_xy_stddev_m = 0.1;
    o.fit.min_up_dot = 0.9;
    o.fit.max_prior_angle_rad = 10 * kPi / 180;
    o.fit.min_inliers = 500;
    o.fit.hypotheses = 64;
    o.fit.seed = 1;
    o.fit_near_min_m = 0.6;
    o.fit_near_max_m = 3.0;
    o.fit_prior_band_m = 0.15;
    o.range_max_m = 6;
    o.blind_near_lo_m = 0.59;
    o.h_tol_m = 0.08;
    o.z_pass_m = 0.75;
    o.eps_neg = 0.15;
    o.w_h_m = 0.5;
    o.angle_step_rad = static_cast<double>(0.008727F);
    o.n_bins = 181;
    o.angle_min_rad = static_cast<double>(static_cast<float>(-0.5 * 180 * o.angle_step_rad));
    return o;
}

// One scene frame: a floor with per-frame tilt noise, boxes, a tall block, overhead returns,
// a pit and a hole that move with the frame number, quantization noise, random invalid depth.
// Frame 3 is almost all invalid, so its fit fails.
void render(const Case& c, const ProjectionBinding& b, const std::vector<Vec3>& rays, int frame,
            std::uint64_t& seed, std::vector<std::uint16_t>& raster) {
    const double tilt = (uniform(seed) - 0.5) * 0.004;
    const double shift = 0.35 * static_cast<double>(frame % 3);
    for (std::size_t k = 0; k < kGeometryPixels; ++k) {
        const auto& r = rays[k];
        const auto& m = b.R_base_camera;
        const Vec3 d{m[0] * r.x + m[1] * r.y + m[2] * r.z, m[3] * r.x + m[4] * r.y + m[5] * r.z,
                     m[6] * r.x + m[7] * r.y + m[8] * r.z};
        double depth = 0;
        if (d.z < -1e-6) {
            depth = c.height_m / -(d.z + tilt * d.x);
            const double px = c.tx_m + d.x * depth, py = c.ty_m + d.y * depth;
            if (px > 1.5 && px < 1.8 && py > -0.3 && py < 0.2) depth *= 0.62;                    // box
            if (px > 4.0 && px < 4.3 && py > 0.0 && py < 0.6) depth *= 0.8;                     // low far box
            if (px > 2.0 && px < 2.2 && py > -1.2 && py < -0.6) depth *= 0.35;                  // tall block
            if (px > 2.3 + shift && px < 2.7 + shift && py > 0.4 && py < 0.9) depth *= 1.4;     // pit
            if (px > 3.0 + shift && px < 3.2 + shift && py < -0.5 && py > -1.0) depth = 0;      // hole
            if (std::hypot(px, py) > 7.5) depth = 0;
        } else if (uniform(seed) < 0.02) {
            depth = 2.0 + 3 * uniform(seed);                                                    // overhead
        }
        if (depth > 0) depth += (uniform(seed) - 0.5) * 0.006;
        if (uniform(seed) < 0.03) depth = 0;
        if (frame == 3 && uniform(seed) < 0.998) depth = 0;
        raster[k] = depth > 0 && depth < 60 ? static_cast<std::uint16_t>(std::lround(depth / 0.001)) : 0;
    }
}

// G-P1 phase 2 (2026-10-25): the general-path scan's own bin lookup (the hint and two neighbours
// inline, else bin_of) against the reference bin. Random points from many starts, and points within
// 1e-11 rad of every bin boundary (inside the inline step's band, on both sides) from the starts
// around that boundary. The inline step must decide every random point whose start is its own
// interior bin, so the comparison is not carried by the fallback alone.
void sweep_scan_lookup(const NativeGeometry& g, const GeometryOptions& o, std::uint64_t& seed) {
    const int n = static_cast<int>(o.n_bins);
    std::size_t checked = 0, own_interior = 0, own_inline = 0;
    for (int i = 0; i < 20000; ++i) {
        const double angle = (uniform(seed) - 0.5) * 2.2, radius = 0.05 + uniform(seed) * 7.0;
        const Vec3 p{radius * std::cos(angle), radius * std::sin(angle), uniform(seed) - 0.5};
        const int reference = g.reference_azimuth_bin(p);
        for (const int hint : {-1, 0, 1, n / 2, n - 2, n - 1, n, static_cast<int>(uniform(seed) * n),
                               reference - 2, reference - 1, reference, reference + 1, reference + 2}) {
            bool decided = false;
            CHECK(g.scan_azimuth_bin(p, hint, &decided) == reference);
            ++checked;
            if (hint == reference && reference >= 1 && reference <= n - 2) {
                ++own_interior;
                if (decided) ++own_inline;
            }
        }
    }
    const double lo = o.angle_min_rad - o.angle_step_rad / 2;
    for (int k = 0; k <= n; ++k) {
        const double theta = lo + static_cast<double>(k) * o.angle_step_rad;
        for (const double delta : {-1e-11, -3e-12, -1.2e-12, -8e-13, -2e-13, 0.0, 2e-13, 8e-13, 1.2e-12, 3e-12, 1e-11}) {
            for (const double radius : {0.3, 2.5, 6.5}) {
                const Vec3 p{radius * std::cos(theta + delta), radius * std::sin(theta + delta), 0.1};
                const int reference = g.reference_azimuth_bin(p);
                for (const int hint : {k - 2, k - 1, k, k + 1}) {
                    CHECK(g.scan_azimuth_bin(p, hint) == reference);
                    ++checked;
                }
            }
        }
    }
    CHECK(checked == 20000u * 13u + static_cast<std::size_t>(n + 1) * 11u * 3u * 4u);
    // About 70% of the random angles fall inside the +-45 degree domain.
    if (own_inline != own_interior) std::cerr << "inline step decided " << own_inline << " of " << own_interior << '\n';
    CHECK(own_interior > 12000 && own_inline == own_interior);
}

// G-P1 phase 2: an unbounded scan (one huge ray) that fails at a middle pixel keeps the counts of
// the pixels it visited, as the frozen parent does -- the scan counts in locals and writes them back
// before it reports the failure. Both paths.
void failure_counts(const Case& c, const ProjectionBinding& b, std::vector<Vec3> rays, const GeometryOptions& o,
                    std::uint64_t& seed) {
    constexpr std::size_t kBad = 200 * kNativeWidth + 321;
    rays[kBad].x = std::numeric_limits<double>::max() / 4;
    NativeGeometry production(b, rays.data(), rays.size(), o);
    parent::NativeGeometry frozen(b, rays.data(), rays.size(), o);
    CHECK(!production.bounded_scan());
    nd::Mailbox mailbox(kGeometryPixels);
    const auto stream = mailbox.begin_stream();
    nd::Frame frame(kGeometryPixels);
    std::vector<std::uint16_t> raster(kGeometryPixels);
    render(c, b, rays, 0, seed, raster);
    raster[kBad] = 65535;
    nd::View view;
    view.metadata.intrinsics = b.intrinsics;
    view.metadata.distortion = b.distortion;
    view.metadata.unit_m = b.unit_m;
    view.metadata.source = {stream, 1, 1, 1, 1, 1, 1000};
    view.pixels = raster.data();
    view.bytes = raster.size() * sizeof(std::uint16_t);
    view.pre_d2c = true;
    CHECK(mailbox.try_offer(view) == nd::Offer::Accepted && mailbox.try_take(frame) == nd::Take::Taken);
    const auto& r = production.process(frame, mailbox);
    (void)frozen.process(frame, mailbox);
    CHECK(r.status == GeometryStatus::NumericalFailure);
    CHECK(r.visited_pixels == kBad + 1 && r.valid_depth_pixels > 0);
    check_same(snapshot(production), snapshot(frozen), "the frozen parent (numerical failure)", 0);
}

void run_case(const Case& c, std::uint64_t& seed, Coverage& coverage) {
    ProjectionBinding b;
    b.intrinsics = {308.148F, 308.148F, 317.25F, 202.75F, 640, 400};
    b.distortion.reported = true;
    b.distortion.model = 3;
    b.unit_m = 0.001F;
    b.R_base_camera = pitch_yaw(c.pitch_deg * kPi / 180, c.yaw_deg * kPi / 180);
    b.t_base_camera = {c.tx_m, c.ty_m, c.height_m};
    std::vector<Vec3> rays(kGeometryPixels);
    for (std::size_t v = 0; v < kNativeHeight; ++v)
        for (std::size_t u = 0; u < kNativeWidth; ++u)
            rays[v * kNativeWidth + u] = {(static_cast<double>(u) - b.intrinsics.cx) / b.intrinsics.fx,
                                          (static_cast<double>(v) - b.intrinsics.cy) / b.intrinsics.fy, 1.0};
    const GeometryOptions o = scene_options();
    NativeGeometry production(b, rays.data(), rays.size(), o);
    parent::NativeGeometry frozen(b, rays.data(), rays.size(), o);
    CHECK(production.fast_azimuth() == c.fast && frozen.fast_azimuth() == c.fast);
    CHECK(production.bounded_scan());  // the comparison below runs the unchecked scan
    nd::Mailbox mailbox(kGeometryPixels);
    const auto stream = mailbox.begin_stream();
    nd::Frame frame(kGeometryPixels);
    std::vector<std::uint16_t> raster(kGeometryPixels);
    Snapshot previous;
    for (int f = 0; f < 7; ++f) {
        render(c, b, rays, f, seed, raster);
        nd::View view;
        view.metadata.intrinsics = b.intrinsics;
        view.metadata.distortion = b.distortion;
        view.metadata.unit_m = b.unit_m;
        const auto n = static_cast<std::uint64_t>(f + 1);
        view.metadata.source = {stream, n, n, n, n, n, 1000 + f};
        view.pixels = raster.data();
        view.bytes = raster.size() * sizeof(std::uint16_t);
        view.pre_d2c = true;
        CHECK(mailbox.try_offer(view) == nd::Offer::Accepted && mailbox.try_take(frame) == nd::Take::Taken);
        (void)production.process(frame, mailbox);
        (void)frozen.process(frame, mailbox);
        const Snapshot now = snapshot(production);
        check_same(now, snapshot(frozen), "the frozen parent", f);
        // A geometry that has seen no other frame.
        NativeGeometry fresh(b, rays.data(), rays.size(), o);
        (void)fresh.process(frame, mailbox);
        check_same(now, snapshot(fresh), "a fresh geometry", f);

        ++coverage.frames;
        if (now.result.status == GeometryStatus::FitFailed) ++coverage.fit_failed;
        for (std::size_t i = 0; i < coverage.classes.size(); ++i) coverage.classes[i] += now.result.classes[i];
        const bool both_normal = !previous.pixels.empty() && now.result.has_evidence() && previous.result.has_evidence();
        for (std::size_t p = 0; p < kGeometryPixels; ++p) {
            const auto& e = now.pixels[p];
            if (e.pierce_range_m) ++coverage.pierce;
            if (!both_normal) continue;
            const auto& before = previous.pixels[p];
            if (before.pierce_range_m && !e.pierce_range_m) ++coverage.pierce_vanished;
            if (before.classification != SampleClass::InvalidDepth && e.classification == SampleClass::InvalidDepth)
                ++coverage.turned_invalid;
        }
        previous = now;
    }
    // The bin search from any start (no hint, every bin, beyond the last bin, a random one) gives
    // the reference bin, for points inside, outside and on both sides of the angular domain.
    std::size_t swept = 0;
    for (int i = 0; i < 20000; ++i) {
        const double angle = (uniform(seed) - 0.5) * 2.2, radius = 0.05 + uniform(seed) * 7.0;
        const Vec3 p{radius * std::cos(angle), radius * std::sin(angle), uniform(seed) - 0.5};
        const int reference = production.reference_azimuth_bin(p);
        CHECK(production.azimuth_bin(p) == reference);
        const int n = static_cast<int>(o.n_bins);
        for (const int hint : {-1, 0, n / 2, n - 1, n, static_cast<int>(uniform(seed) * n), reference - 1, reference + 1}) {
            CHECK(production.azimuth_bin(p, hint) == reference);
            ++swept;
        }
    }
    CHECK(swept == 160000);
    sweep_scan_lookup(production, o, seed);
    failure_counts(c, b, rays, o, seed);
}
}  // namespace

// G-P1 phase 2 (2026-10-25): the scan's range equals std::hypot bit for bit -- random coordinates of
// scan size, every ratio from equal to 2^-60 (both branches of the kernel, its ax + ay shortcut and
// the boundaries between them), the exact ax == 2 ay boundary, and the special inputs (zeros, the
// tiny and huge scaling thresholds, subnormals, the largest double, infinities, NaN). Under glibc
// 2.35 on aarch64 scan_range is the inline copy and this compares it with libm; elsewhere it is
// std::hypot itself and the comparison is trivially true.
void sweep_scan_range(std::uint64_t& seed) {
    std::size_t checked = 0;
    const auto check = [&](double x, double y) {
        const double a = NativeGeometry::scan_range(x, y), b = std::hypot(x, y);
        if (!(std::isnan(a) && std::isnan(b)) && !same_bits(a, b)) {
            std::cerr << "scan_range differs from std::hypot at x=" << x << " y=" << y << '\n';
            CHECK(false);
        }
        ++checked;
    };
    for (int i = 0; i < 1000000; ++i) check((uniform(seed) - 0.5) * 16, (uniform(seed) - 0.5) * 16);
    for (int e = 0; e <= 60; ++e)
        for (int i = 0; i < 4000; ++i) {
            const double x = (uniform(seed) + 0.5) * 6, y = std::ldexp(uniform(seed) + 0.5, -e) * x;
            check(x, y);
            check(-y, x);
        }
    for (int i = 0; i < 20000; ++i) {
        const double y = (uniform(seed) + 0.01) * 4, x = 2 * y;
        check(x, y);
        check(std::nextafter(x, 0.0), y);
        check(std::nextafter(x, 100.0), -y);
    }
    const double inf = std::numeric_limits<double>::infinity(), nan = std::numeric_limits<double>::quiet_NaN();
    const double specials[] = {0.0, -0.0, 0x1p-1074, 0x1p-1022, 0x1p-460, 0x1p-459, std::nextafter(0x1p-459, 0.0),
                               0x1p-54, 1.0, 3.0, 0x1p511, std::nextafter(0x1p511, inf), 0x1p600,
                               std::numeric_limits<double>::max(), inf, -inf, nan};
    for (const double x : specials)
        for (const double y : specials) {
            check(x, y);
            check(-x, 0.37 * y);
        }
    CHECK(checked == 1000000u + 61u * 4000u * 2u + 20000u * 3u + 17u * 17u * 2u);
    std::cout << "scan range: " << checked << " inputs equal std::hypot (inline glibc 2.35 copy: "
              << (NativeGeometry::scan_range_inline() ? "yes" : "no, std::hypot itself") << ")\n";
}

int main() {
    // Fast path: no lateral offset (any rotation). General path: a lateral offset, including
    // the 0.30 m forward mount of the device x030 fixture.
    const Case cases[] = {{0.8, 15.7, 0, 0, 0, true},
                          {0.6, 25, 5, 0, 0, true},
                          {0.6, 25, 5, 0.3, -0.1, false},
                          {0.5467, 15.7, 0, 0.30, 0, false},
                          {0.6056, 18, 0, 0.345, 0.060088816533226362, false}};
    std::uint64_t seed = 12345;
    Coverage fast, general;
    for (const auto& c : cases) run_case(c, seed, c.fast ? fast : general);
    sweep_scan_range(seed);
    for (const Coverage* cv : {&fast, &general}) {
        const auto count = [&](SampleClass s) { return cv->classes[static_cast<std::size_t>(s)]; };
        std::cout << (cv == &fast ? "fast" : "general") << ": frames " << cv->frames << " fit_failed " << cv->fit_failed
                  << " ground " << count(SampleClass::Ground) << " obstacle " << count(SampleClass::Obstacle)
                  << " overhead " << count(SampleClass::Overhead) << " below " << count(SampleClass::BelowPlane)
                  << " outside " << count(SampleClass::OutsideDomain) << " invalid " << count(SampleClass::InvalidDepth)
                  << " pierce " << cv->pierce << " pierce_vanished " << cv->pierce_vanished
                  << " turned_invalid " << cv->turned_invalid << '\n';
        CHECK(cv->fit_failed > 0 && cv->pierce > 0 && cv->pierce_vanished > 0 && cv->turned_invalid > 0);
        for (std::size_t i = 1; i < cv->classes.size(); ++i) CHECK(cv->classes[i] > 0);  // all but Unclassified
        CHECK(cv->classes[static_cast<std::size_t>(SampleClass::Unclassified)] == 0);
    }
    std::cout << "native geometry pixel record: OK\n";
    return 0;
}
