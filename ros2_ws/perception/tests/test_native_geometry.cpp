/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_native_geometry.cpp
 * Brief: Synthetic, SDK-independent native-depth evidence checks.
 *
 * Description:
 * These explicitly supplied zero-distortion rays and SE(3) numbers are NOT production calibration. No FREE decision, navigation profile, or deployment acceptance is tested.
 */

#include "modules/native_geometry.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <vector>

namespace {
std::atomic<std::uint64_t> allocations{0};
std::uint64_t checked_process_calls = 0;

void* allocate(std::size_t size) {
    void* p = std::malloc(size == 0 ? 1 : size);
    if (!p) throw std::bad_alloc();
    allocations.fetch_add(1, std::memory_order_relaxed);
    return p;
}
void* allocate_aligned(std::size_t size, std::size_t alignment) {
    void* p = nullptr;
    if (posix_memalign(&p, alignment, size == 0 ? 1 : size) != 0)
        throw std::bad_alloc();
    allocations.fetch_add(1, std::memory_order_relaxed);
    return p;
}
[[noreturn]] void fail(const char* expression, int line) {
    std::cerr << "native geometry CHECK failed at line " << line
              << ": " << expression << '\n';
    std::abort();
}
#define CHECK(expression) do { if (!(expression)) fail(#expression, __LINE__); } while (false)
}  // namespace

// Cover ordinary, array, sized and aligned C++ allocations around every call
// to process. Setup allocations are permitted and happen before measurement.
void* operator new(std::size_t n) { return allocate(n); }
void* operator new[](std::size_t n) { return allocate(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t n, std::align_val_t a) {
    return allocate_aligned(n, static_cast<std::size_t>(a));
}
void* operator new[](std::size_t n, std::align_val_t a) {
    return allocate_aligned(n, static_cast<std::size_t>(a));
}
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {
namespace nd = perception::native_depth;
using namespace perception::geometry;

bool close(double a, double b, double tolerance = 1e-9) {
    return std::abs(a - b) <= tolerance;
}
double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 rotated(const ProjectionBinding& b, Vec3 r) {
    const auto& m = b.R_base_camera;
    return {m[0] * r.x + m[1] * r.y + m[2] * r.z,
            m[3] * r.x + m[4] * r.y + m[5] * r.z,
            m[6] * r.x + m[7] * r.y + m[8] * r.z};
}
std::size_t at(std::size_t u, std::size_t v) { return v * kNativeWidth + u; }

ProjectionBinding synthetic_binding() {
    ProjectionBinding b;
    b.intrinsics = {128.0F, 128.0F, 320.0F, -128.0F, 640, 400};
    b.distortion.reported = true;
    b.distortion.model = 0;  // Fixture provenance only; no SDK model interpretation.
    b.unit_m = 1.0F / 1024.0F;  // Binary exact, including integer-code thresholds.
    b.R_base_camera = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    b.t_base_camera = {0, 0, 1};
    return b;
}

GeometryOptions synthetic_options() {
    GeometryOptions o;
    o.prior = {{0, 0, 1}, 0};
    o.fit.inlier_distance_m = 0.015;
    o.fit.min_inlier_fraction = 0.8;
    o.fit.max_inlier_rms_m = 0.005;
    o.fit.min_xy_stddev_m = 0.01;
    o.fit.min_up_dot = 0.5;
    o.fit.max_prior_angle_rad = 0.3;
    o.fit.min_inliers = 100;
    o.fit.hypotheses = 32;  // Explicit bounded numerical fixture choice.
    o.fit.seed = 0x243f6a8885a308d3ULL;
    o.fit_near_min_m = 0.7;
    o.fit_near_max_m = 2.9;
    o.fit_prior_band_m = 0.25;
    o.range_max_m = 6;
    o.blind_near_lo_m = 0.01;
    o.h_tol_m = 0.125;
    o.z_pass_m = 0.5;
    o.eps_neg = 0.1;
    o.w_h_m = 0.1;
    o.angle_min_rad = -1.55;
    o.angle_step_rad = 0.01;
    o.n_bins = 311;
    return o;
}

std::vector<Vec3> pinhole_rays(const ProjectionBinding& b) {
    std::vector<Vec3> rays(kGeometryPixels);
    for (std::size_t v = 0; v < kNativeHeight; ++v)
        for (std::size_t u = 0; u < kNativeWidth; ++u)
            rays[at(u, v)] = {(static_cast<double>(u) - b.intrinsics.cx) / b.intrinsics.fx,
                             (static_cast<double>(v) - b.intrinsics.cy) / b.intrinsics.fy, 1};
    return rays;
}

struct Fixture {
    ProjectionBinding binding;
    GeometryOptions options;
    std::vector<Vec3> rays;
    std::vector<std::uint16_t> raster;
    nd::Mailbox mailbox{kGeometryPixels};
    nd::Frame frame{kGeometryPixels};
    std::unique_ptr<NativeGeometry> geometry;
    std::uint64_t stream = 0, sequence = 0;

    explicit Fixture(ProjectionBinding b = synthetic_binding(),
                     GeometryOptions o = synthetic_options())
        : binding(b), options(o), rays(pinhole_rays(binding)), raster(kGeometryPixels),
          geometry(std::make_unique<NativeGeometry>(binding, rays.data(), rays.size(), options)) {
        stream = mailbox.begin_stream();
        // Buffer capacities only: excludes Mailbox/Frame, allocator overhead
        // and the rest of the process. This is NOT a peak RSS measurement.
        CHECK(geometry->storage_bytes() <= 40U * 1024U * 1024U);
    }

    void full_plane(Plane plane) {
        for (std::size_t i = 0; i < rays.size(); ++i) {
            const double lambda = -(dot(plane.normal, binding.t_base_camera) + plane.d) /
                                  dot(plane.normal, rotated(binding, rays[i]));
            const auto code = std::llround(lambda / binding.unit_m);
            CHECK(code > 0 && code <= 65535);
            raster[i] = static_cast<std::uint16_t>(code);
        }
    }

    void exact_ground_support() {
        std::fill(raster.begin(), raster.end(), 0);
        // y rays 1, 2, 4 yield exactly z=0, with a two-dimensional XY spread.
        for (std::size_t u = 0; u < kNativeWidth; ++u) {
            raster[at(u, 0)] = 1024;
            raster[at(u, 128)] = 512;
            raster[at(u, 384)] = 256;
        }
    }

    nd::View next_view() {
        nd::View v;
        v.metadata.intrinsics = binding.intrinsics;
        v.metadata.distortion = binding.distortion;
        v.metadata.unit_m = binding.unit_m;
        auto& s = v.metadata.source;
        s.stream_id = stream;
        s.sequence = ++sequence;
        s.color_index = sequence - 1;
        s.depth_index = sequence - 1;
        s.color_device_us = sequence * 1000;
        s.depth_device_us = sequence * 1000 + 1;
        s.host_ready_mono_ns = static_cast<std::int64_t>(sequence * 1000000);
        v.pixels = raster.data();
        v.bytes = raster.size() * sizeof(raster.front());
        v.pre_d2c = true;
        return v;
    }

    void take(const nd::View& v, nd::Frame& output) {
        CHECK(mailbox.try_offer(v) == nd::Offer::Accepted);
        CHECK(mailbox.try_take(output) == nd::Take::Taken);
    }

    const GeometryResult& process_current(const nd::Frame& input) {
        const auto before = allocations.load(std::memory_order_relaxed);
        const auto& result = geometry->process(input, mailbox);
        const auto after = allocations.load(std::memory_order_relaxed);
        CHECK(after == before);
        ++checked_process_calls;
        return result;
    }

    const GeometryResult& process() {
        take(next_view(), frame);
        return process_current(frame);
    }

    Vec3 point(std::size_t i) const {
        const auto q = rotated(binding, rays[i]);
        const double d = raster[i] * static_cast<double>(binding.unit_m);
        return {binding.t_base_camera.x + q.x * d,
                binding.t_base_camera.y + q.y * d,
                binding.t_base_camera.z + q.z * d};
    }

    int expected_bin(Vec3 p) const {
        return static_cast<int>(std::floor((std::atan2(p.y, p.x) - options.angle_min_rad) /
                                          options.angle_step_rad + 0.5));
    }
};

std::size_t class_count(const GeometryResult& r, SampleClass c) {
    return r.classes[static_cast<std::size_t>(c)];
}

void expect_normal(const GeometryResult& r) {
    CHECK(r.status == GeometryStatus::NormalEvidence && r.has_evidence());
    CHECK(r.has_raw_sample_evidence());
    CHECK(r.fit.status == FitStatus::Pass && r.fit.plane.has_value());
    CHECK(r.visited_pixels == kGeometryPixels);
    CHECK(r.fit.inliers >= 100 && r.fit.hypotheses_tested > 0 && r.fit.hypotheses_tested <= 32);
}

std::size_t raw_total(const NativeGeometry& geometry) {
    std::size_t total = 0;
    for (const auto count : geometry.raw_sample_counts()) total += count;
    return total;
}

void expect_no_raw(const Fixture& f) {
    CHECK(!f.geometry->result().has_raw_sample_evidence());
    CHECK(!f.geometry->result().raw_sample_evidence_valid);
    CHECK(raw_total(*f.geometry) == 0);
}

void expect_cleared(const Fixture& f) {
    CHECK(!f.geometry->result().has_evidence());
    CHECK(!f.geometry->result().fit.plane.has_value());
    CHECK(!f.geometry->result().nearest_ground_m.has_value());
    for (const auto count : f.geometry->result().classes) CHECK(count == 0);
    for (const auto& p : f.geometry->pixels()) {
        CHECK(p.classification == SampleClass::Unclassified);
        CHECK(p.bin == -1 && p.pierce_bin == -1 && !p.pierce_range_m);
        CHECK(p.range_m == 0 && p.height_m == 0);
    }
    for (const auto& b : f.geometry->bins()) {
        CHECK(b.valid_samples == 0 && b.ground_samples == 0);
        CHECK(!b.nearest_positive_m && !b.positive_height_window_max_m);
        CHECK(!b.nearest_below_m && !b.below_min_height_m);
        CHECK(!b.nearest_pierce_expected_m);
    }
    if (f.geometry->result().status != GeometryStatus::FitFailed)
        expect_no_raw(f);
}

// Reference height window: the former full scan over every pixel (round 2 keeps a list of the
// Obstacle pixels instead); every bin's window maximum must be the same value or both absent.
void check_height_window(const Fixture& f) {
    std::vector<std::optional<double>> expected(f.geometry->bins().size());
    for (const auto& e : f.geometry->pixels()) {
        if (e.classification != SampleClass::Obstacle) continue;
        const auto& b = f.geometry->bins()[static_cast<std::size_t>(e.bin)];
        auto& w = expected[static_cast<std::size_t>(e.bin)];
        if (b.nearest_positive_m && e.range_m <= *b.nearest_positive_m + f.options.w_h_m && (!w || e.height_m > *w))
            w = e.height_m;
    }
    bool same = true;
    for (std::size_t b = 0; b < expected.size(); ++b)
        same = same && expected[b] == f.geometry->bins()[b].positive_height_window_max_m;
    CHECK(same);
}

// Every pixel is scanned: an all-ground frame fits the same plane twice, one pixel outside the fit
// region still becomes an obstacle, and tilted or offset planes are recovered.
void test_full_frames_and_one_pixel_obstacle() {
    Fixture f;
    f.full_plane({{0, 0, 1}, 0});
    const auto& initial = f.process();
    expect_normal(initial);
    CHECK(initial.valid_depth_pixels == kGeometryPixels);
    CHECK(f.geometry->raw_sample_counts().size() == f.options.n_bins);
    CHECK(raw_total(*f.geometry) == kGeometryPixels);
    for (std::size_t i = 0; i < f.options.n_bins; ++i)
        CHECK(f.geometry->raw_sample_counts()[i] == f.geometry->bins()[i].valid_samples);
    CHECK(initial.roi_pixels == kGeometryPixels && initial.invalid_roi_pixels == 0);
    CHECK(class_count(initial, SampleClass::Ground) == kGeometryPixels);
    CHECK(close(initial.fit.plane->d, 0, 0.002));
    CHECK(close(initial.fit.plane->normal.z, 1, 1e-5));
    const auto initial_candidates = initial.fit.candidates;
    const auto initial_plane = *initial.fit.plane;
    const auto& repeated = f.process();
    expect_normal(repeated);
    CHECK(close(repeated.fit.plane->d, initial_plane.d, 0));
    CHECK(close(repeated.fit.plane->normal.x, initial_plane.normal.x, 0));
    CHECK(close(repeated.fit.plane->normal.y, initial_plane.normal.y, 0));
    CHECK(close(repeated.fit.plane->normal.z, initial_plane.normal.z, 0));

    // The last raster pixel lies outside the near fit. It still MUST be scanned
    // for obstacle evidence; a stride or fit-only processing path loses it.
    const std::size_t thin = kGeometryPixels - 1;
    CHECK(std::hypot(f.point(thin).x, f.point(thin).y) < f.options.fit_near_min_m);
    f.raster[thin] = 187;
    f.raster[at(320, 200)] = 0;
    CHECK(std::hypot(f.point(thin).x, f.point(thin).y) < f.options.fit_near_min_m);
    const auto& obstacle = f.process();
    expect_normal(obstacle);
    CHECK(obstacle.fit.candidates == initial_candidates);
    CHECK(obstacle.valid_depth_pixels == kGeometryPixels - 1);
    CHECK(raw_total(*f.geometry) == kGeometryPixels - 1);
    CHECK(obstacle.roi_pixels == kGeometryPixels && obstacle.invalid_roi_pixels == 1);
    CHECK(class_count(obstacle, SampleClass::InvalidDepth) == 1);
    CHECK(class_count(obstacle, SampleClass::Obstacle) == 1);
    const auto& p = f.geometry->pixels()[thin];
    CHECK(p.classification == SampleClass::Obstacle && p.bin >= 0);
    CHECK(f.geometry->bins()[p.bin].nearest_positive_m.has_value());
    CHECK(close(*f.geometry->bins()[p.bin].nearest_positive_m, p.range_m));
    const auto& zero = f.geometry->pixels()[at(320, 200)];
    CHECK(zero.classification == SampleClass::InvalidDepth && zero.bin == -1);
    CHECK(!zero.pierce_range_m && zero.pierce_bin == -1);

    for (Plane truth : {Plane{{0, 0, 1}, -0.05}, Plane{{-0.08, 0.04, 1}, -0.02}}) {
        f.full_plane(truth);
        const auto& result = f.process();
        expect_normal(result);
        const double norm = std::sqrt(dot(truth.normal, truth.normal));
        CHECK(close(result.fit.plane->normal.x, truth.normal.x / norm, 0.002));
        CHECK(close(result.fit.plane->normal.y, truth.normal.y / norm, 0.002));
        CHECK(close(result.fit.plane->normal.z, truth.normal.z / norm, 0.002));
        CHECK(close(result.fit.plane->d, truth.d / norm, 0.003));
        CHECK(class_count(result, SampleClass::Ground) == kGeometryPixels);
    }
}

// Height limits are inclusive: exactly +-h_tol (0.125 m here) is ground and exactly z_pass (0.5 m)
// is obstacle; one depth code further changes the class; depth 0 is invalid with no bin.
void test_exact_classification_boundaries() {
    auto options = synthetic_options();
    options.fit_prior_band_m = 0.0625;
    Fixture f(synthetic_binding(), options);
    f.exact_ground_support();
    struct Boundary { std::uint16_t code; SampleClass classification; double height; };
    const Boundary cases[] = {
        {449, SampleClass::Ground, 0.123046875},
        {448, SampleClass::Ground, 0.125},
        {447, SampleClass::Obstacle, 0.126953125},
        {257, SampleClass::Obstacle, 0.498046875},
        {256, SampleClass::Obstacle, 0.5},
        {255, SampleClass::Overhead, 0.501953125},
        {575, SampleClass::Ground, -0.123046875},
        {576, SampleClass::Ground, -0.125},
        {577, SampleClass::BelowPlane, -0.126953125},
        {0, SampleClass::InvalidDepth, 0},
    };
    for (std::size_t i = 0; i < std::size(cases); ++i)
        f.raster[at(315 + i, 128)] = cases[i].code;
    const auto& result = f.process();
    expect_normal(result);
    CHECK(close(result.fit.plane->normal.z, 1, 0));
    CHECK(close(result.fit.plane->d, 0, 0));
    for (std::size_t i = 0; i < std::size(cases); ++i) {
        const auto& pixel = f.geometry->pixels()[at(315 + i, 128)];
        CHECK(pixel.classification == cases[i].classification);
        if (cases[i].code != 0) CHECK(close(pixel.height_m, cases[i].height, 0));
        else CHECK(pixel.bin == -1 && !pixel.pierce_range_m);
    }
}

// The same pixel is overhead at 0.75 m but a blocking obstacle at 0.25 m; the bin's height window
// follows the final nearest obstacle, not a running maximum.
void test_paired_crossbar_and_final_height_window() {
    auto options = synthetic_options();
    options.fit_prior_band_m = 0.0625;
    Fixture f(synthetic_binding(), options);
    f.exact_ground_support();
    const auto crossbar = at(320, 128);
    f.raster[crossbar] = 128;  // Same pixel: z=0.75, above the passage threshold.
    expect_normal(f.process());
    auto p = f.geometry->pixels()[crossbar];
    CHECK(p.classification == SampleClass::Overhead);
    CHECK(!f.geometry->bins()[p.bin].nearest_positive_m);
    f.raster[crossbar] = 384;  // Same pixel: z=0.25, a low blocking crossbar.
    expect_normal(f.process());
    p = f.geometry->pixels()[crossbar];
    CHECK(p.classification == SampleClass::Obstacle);
    CHECK(close(*f.geometry->bins()[p.bin].nearest_positive_m, 0.375, 0));
    CHECK(close(*f.geometry->bins()[p.bin].positive_height_window_max_m, 0.25, 0));

    // Earlier raster order: tall/far at r=.5625,h=.4375. Later: short/near at
    // r=.375,h=.25. The final [.375,.475] height window excludes the earlier tall
    // point. A running max that fails to reconsider the window gives .4375.
    f.raster[at(320, 0)] = 576;
    expect_normal(f.process());
    const auto& far = f.geometry->pixels()[at(320, 0)];
    const auto& near = f.geometry->pixels()[crossbar];
    CHECK(far.bin == near.bin && far.classification == SampleClass::Obstacle);
    CHECK(close(far.height_m, 0.4375, 0) && close(far.range_m, 0.5625, 0));
    CHECK(close(*f.geometry->bins()[near.bin].nearest_positive_m, 0.375, 0));
    CHECK(close(*f.geometry->bins()[near.bin].positive_height_window_max_m, 0.25, 0));
    check_height_window(f);
}

// Many obstacles in many bins (a raised, gapped band across a full ground plane): the
// list-based height window equals the full-scan reference, and a following frame without
// obstacles leaves no window behind.
void test_height_window_many_obstacles() {
    Fixture f;
    f.full_plane({{0, 0, 1}, 0});
    expect_normal(f.process());
    check_height_window(f);
    for (std::size_t v = 300; v < 340; ++v)
        for (std::size_t u = 0; u < kNativeWidth; u += 3) f.raster[at(u, v)] = static_cast<std::uint16_t>(f.raster[at(u, v)] / 2);
    expect_normal(f.process());
    CHECK(class_count(f.geometry->result(), SampleClass::Obstacle) > 100);
    std::size_t windows = 0;
    for (const auto& b : f.geometry->bins()) windows += b.positive_height_window_max_m ? 1 : 0;
    CHECK(windows > 10);
    check_height_window(f);
    f.full_plane({{0, 0, 1}, 0});
    expect_normal(f.process());
    for (const auto& b : f.geometry->bins()) CHECK(!b.positive_height_window_max_m);
    check_height_window(f);
}

// With a translated camera a pixel's bin comes from its base-frame point, not the ray direction; a
// below-plane return records its expected ground crossing in the crossing's own bin.
void test_translated_camera_and_expected_pierce_bin() {
    auto binding = synthetic_binding();
    binding.t_base_camera = {0.5, 0.75, 1};
    auto options = synthetic_options();
    options.fit_prior_band_m = 0.0625;
    Fixture f(binding, options);
    f.exact_ground_support();
    const auto sample = at(320, 128);
    f.raster[sample] = 384;
    expect_normal(f.process());
    auto p = f.geometry->pixels()[sample];
    CHECK(p.classification == SampleClass::Obstacle);
    CHECK(p.bin == f.expected_bin({0.875, 0.75, 0.25}));
    CHECK(p.bin != f.expected_bin({1, 0, -2}));  // Direction angle drops translation.
    CHECK(close(p.range_m, std::hypot(0.875, 0.75)));

    f.raster[sample] = 768;  // Return lambda=.75, expected plane lambda=.5.
    expect_normal(f.process());
    p = f.geometry->pixels()[sample];
    const Vec3 expected_intersection{1, 0.75, 0};
    const Vec3 actual_return{1.25, 0.75, -0.5};
    CHECK(p.classification == SampleClass::BelowPlane);
    CHECK(p.bin == f.expected_bin(actual_return));
    CHECK(p.pierce_bin == f.expected_bin(expected_intersection));
    CHECK(p.pierce_bin != p.bin && p.pierce_range_m.has_value());
    CHECK(close(*p.pierce_range_m, 1.25));
    CHECK(close(*f.geometry->bins()[p.pierce_bin].nearest_pierce_expected_m, 1.25));
    CHECK(!f.geometry->bins()[p.bin].nearest_pierce_expected_m);
    CHECK(close(*f.geometry->bins()[p.bin].nearest_below_m,
                std::hypot(actual_return.x, actual_return.y)));
}

// Ground 0.15 m below the prior: pierce is judged against the current fitted plane, so no ray
// falsely pierces.
void test_current_plane_rejects_prior_false_pierce() {
    Fixture f;
    const Plane truth{{0, 0, 1}, 0.15};  // Actual ground is 0.15 m below the prior.
    f.full_plane(truth);
    double maximum_quantization_error = 0;
    for (std::size_t i = 0; i < kGeometryPixels; ++i) {
        const double error = std::abs(dot(truth.normal, f.point(i)) + truth.d);
        maximum_quantization_error = std::max(maximum_quantization_error, error);
        // Quantized ground still lies inside the fit prior band, but every ray
        // would incorrectly pierce if the expected intersection used the prior.
        CHECK(std::abs(f.point(i).z) < f.options.fit_prior_band_m);
        const double prior_depth = -f.binding.t_base_camera.z /
                                   rotated(f.binding, f.rays[i]).z;
        const double actual_depth = f.raster[i] * static_cast<double>(f.binding.unit_m);
        CHECK(actual_depth > prior_depth * (1.0 + f.options.eps_neg));
    }
    // Half a depth code times the largest downward ray component bounds the
    // vertical quantization error: 0.5/1024 * 527/128 < 0.002011 m.
    CHECK(maximum_quantization_error <= 0.002011);
    const auto& result = f.process();
    expect_normal(result);
    CHECK(close(result.fit.plane->d, truth.d, 0.003));
    CHECK(class_count(result, SampleClass::Ground) == kGeometryPixels);
    for (const auto& p : f.geometry->pixels())
        CHECK(p.pierce_bin == -1 && !p.pierce_range_m);
    for (const auto& b : f.geometry->bins())
        CHECK(!b.nearest_pierce_expected_m);
}

// An all-invalid frame fails the fit and clears every bin's evidence (raw counts valid but zero);
// the next good frame recovers.
void test_failure_clearing_and_recovery() {
    Fixture f;
    f.exact_ground_support();
    f.raster[at(320, 0)] = 576;    // Positive height and window evidence.
    f.raster[at(320, 128)] = 768;  // Below-plane and raw crossing evidence.
    expect_normal(f.process());
    const auto bin = f.geometry->pixels()[at(320, 0)].bin;
    CHECK(bin == f.geometry->pixels()[at(320, 128)].bin);
    CHECK(f.geometry->bins()[bin].nearest_positive_m);
    CHECK(f.geometry->bins()[bin].positive_height_window_max_m);
    CHECK(f.geometry->bins()[bin].nearest_below_m && f.geometry->bins()[bin].below_min_height_m);
    CHECK(f.geometry->bins()[bin].nearest_pierce_expected_m);
    std::fill(f.raster.begin(), f.raster.end(), 0);
    const auto& invalid = f.process();
    CHECK(invalid.status == GeometryStatus::FitFailed);
    CHECK(invalid.fit.candidates == 0 && invalid.valid_depth_pixels == 0);
    CHECK(invalid.has_raw_sample_evidence() && raw_total(*f.geometry) == 0);
    CHECK(invalid.roi_pixels == kGeometryPixels && invalid.invalid_roi_pixels == kGeometryPixels);
    expect_cleared(f);
    f.exact_ground_support();
    expect_normal(f.process());
    CHECK(f.geometry->result().source.sequence == 3);
}

// Raw sample counts do not depend on the fit: a failed fit keeps the same counts but no other
// evidence, and a newer sparse frame counts only its own samples.
void test_raw_ledger_independent_of_fit_and_classifications() {
    Fixture normal;
    normal.full_plane({{0, 0, 1}, 0});
    expect_normal(normal.process());
    const auto expected = normal.geometry->raw_sample_counts();
    auto options = synthetic_options();
    options.fit.min_inliers = kGeometryPixels;
    Fixture failed(synthetic_binding(), options);
    failed.full_plane({{0, 0, 1}, 0});
    const auto& result = failed.process();
    CHECK(result.status == GeometryStatus::FitFailed);
    CHECK(result.fit.status == FitStatus::InsufficientCandidates);
    CHECK(result.has_raw_sample_evidence() && !result.has_evidence());
    CHECK(result.valid_depth_pixels == kGeometryPixels);
    CHECK(result.fit.candidates < kGeometryPixels);
    CHECK(failed.geometry->raw_sample_counts() == expected);
    expect_cleared(failed);
    Vec3 point{1, 2, 3};
    CHECK(!failed.geometry->ground_point(at(320, 128), point));
    CHECK(point.x == 0 && point.y == 0 && point.z == 0);

    // Failure on a newer sparse frame must describe only that frame. It must
    // neither retain the previous dense counts nor require any fitted support.
    std::fill(failed.raster.begin(), failed.raster.end(), 0);
    failed.raster[at(320, 128)] = 128;  // Valid return above the prior ground.
    const auto& sparse = failed.process();
    CHECK(sparse.status == GeometryStatus::FitFailed && sparse.has_raw_sample_evidence());
    CHECK(sparse.fit.candidates == 0 && raw_total(*failed.geometry) == 1);
    expect_cleared(failed);
    CHECK(failed.process_current(failed.frame).status == GeometryStatus::InactiveOrStale);
    expect_cleared(failed);
}

// Raw counts use the actual return's range and angle: both range limits are inclusive, and returns
// outside the range or angular domain are not counted.
void test_raw_ledger_uses_actual_return_domain() {
    auto options = synthetic_options();
    options.angle_min_rad = 0;
    options.angle_step_rad = 0.2;
    options.n_bins = 1;
    options.blind_near_lo_m = 0.5;
    Fixture f(synthetic_binding(), options);
    f.raster[at(320, 0)] = 512;   // Inclusive near boundary: r=.5.
    f.raster[at(320, 1)] = 511;   // Below the unchanged near domain.
    f.raster[at(320, 2)] = 6144;  // Inclusive far boundary: r=6.
    f.raster[at(320, 3)] = 6145;  // Outside the range domain.
    f.raster[at(0, 4)] = 512;     // Outside this angular domain.
    const auto& result = f.process();
    CHECK(result.status == GeometryStatus::FitFailed && result.has_raw_sample_evidence());
    CHECK(result.valid_depth_pixels == 5);
    CHECK(f.geometry->raw_sample_counts().size() == 1);
    CHECK(f.geometry->raw_sample_counts()[0] == 2);
    expect_cleared(f);
}

// Raw evidence needs the complete bound 640x400 frame of the current stream; an invalid frame, an
// ended stream or an old-stream frame clears it, and a restarted stream recovers.
void test_raw_ledger_invalid_frame_and_generation_clear() {
    Fixture f;
    f.raster[at(320, 128)] = 128;
    nd::Frame held(kGeometryPixels);
    f.take(f.next_view(), held);
    CHECK(f.process_current(held).has_raw_sample_evidence());
    CHECK(raw_total(*f.geometry) == 1);

    // The Mailbox accepts this self-consistent smaller raster; NativeGeometry
    // requires the complete bound 640x400 frame before accepting raw evidence.
    auto smaller = f.next_view();
    smaller.metadata.intrinsics.width = 320;
    smaller.bytes /= 2;
    f.take(smaller, f.frame);
    CHECK(f.process_current(f.frame).status == GeometryStatus::InvalidFrame);
    expect_cleared(f);

    CHECK(f.process().has_raw_sample_evidence());
    f.mailbox.end_stream();
    CHECK(f.process_current(f.frame).status == GeometryStatus::InactiveOrStale);
    expect_cleared(f);
    const auto old_stream = f.stream;
    f.stream = f.mailbox.begin_stream();
    f.sequence = 0;
    CHECK(f.stream != old_stream);
    const auto& restarted = f.process();
    CHECK(restarted.status == GeometryStatus::FitFailed && restarted.has_raw_sample_evidence());
    CHECK(restarted.source.stream_id == f.stream && raw_total(*f.geometry) == 1);
    CHECK(f.process_current(held).status == GeometryStatus::InactiveOrStale);
    expect_cleared(f);
    const auto& recovered = f.process();
    CHECK(recovered.has_raw_sample_evidence() && recovered.source.stream_id == f.stream);
}

// A numerical failure at the last pixel clears the whole scan, including the counts already made;
// a good frame recovers.
void test_raw_ledger_numerical_failure_clears_partial_scan() {
    Fixture f;
    f.rays.back().x = std::numeric_limits<double>::max()/4;
    f.geometry = std::make_unique<NativeGeometry>(f.binding, f.rays.data(), f.rays.size(), f.options);
    CHECK(!f.geometry->bounded_scan());  // such a ray keeps the checks (G-P1 phase 1)
    f.exact_ground_support();
    expect_normal(f.process());
    CHECK(raw_total(*f.geometry) > 0);
    // The final pixel overflows its projected coordinate after many earlier
    // valid samples were counted. No prefix of that invalid scan may escape.
    f.raster.back() = 65535;
    CHECK(f.process().status == GeometryStatus::NumericalFailure);
    expect_cleared(f);
    f.raster.back() = 0;
    expect_normal(f.process());
    CHECK(raw_total(*f.geometry) > 0);
}

// Raw evidence is valid only with status NormalEvidence or FitFailed and the validity flag set.
void test_raw_validity_needs_allowed_status() {
    GeometryResult result;
    result.raw_sample_evidence_valid = true;
    for (const auto status : {GeometryStatus::InvalidFrame, GeometryStatus::InactiveOrStale,
            GeometryStatus::ProfileMismatch, GeometryStatus::NumericalFailure}) {
        result.status = status;
        CHECK(!result.has_raw_sample_evidence());
    }
    for (const auto status : {GeometryStatus::NormalEvidence, GeometryStatus::FitFailed}) {
        result.status = status;
        CHECK(result.has_raw_sample_evidence());
        result.raw_sample_evidence_valid = false;
        CHECK(!result.has_raw_sample_evidence());
        result.raw_sample_evidence_valid = true;
    }
}

// A frame from another mailbox is refused even with the same stream id and sequence, without
// allocating or disturbing the owner's next frame.
void test_mailbox_owner_rejects_matching_numeric_identity() {
    Fixture f;
    f.exact_ground_support();
    expect_normal(f.process());  // Bind the first owner at source sequence 1.
    nd::Mailbox foreign(kGeometryPixels);
    nd::Frame foreign_frame(kGeometryPixels);
    CHECK(foreign.begin_stream() == f.stream);
    const auto same_source = f.next_view();
    f.take(same_source, f.frame);
    CHECK(foreign.try_offer(same_source) == nd::Offer::Accepted);
    CHECK(foreign.try_take(foreign_frame) == nd::Take::Taken);
    CHECK(foreign_frame.metadata().source.stream_id == f.frame.metadata().source.stream_id);
    CHECK(foreign_frame.metadata().source.sequence == f.frame.metadata().source.sequence);
    CHECK(same_source.metadata.source.sequence == 2);
    const auto before = allocations.load(std::memory_order_relaxed);
    const auto& rejected = f.geometry->process(foreign_frame, foreign);
    const auto after = allocations.load(std::memory_order_relaxed);
    CHECK(after == before);
    ++checked_process_calls;
    CHECK(rejected.status == GeometryStatus::InactiveOrStale);
    expect_cleared(f);
    // Rejecting the foreign owner must not consume the original owner's
    // matching sequence or change the lifetime binding.
    expect_normal(f.process_current(f.frame));
    CHECK(f.geometry->result().source.sequence == 2);
    CHECK(raw_total(*f.geometry) > 0);
}

// Replayed, older and pre-restart frames are refused as stale and clear the output; a restarted
// stream starts again at sequence 1.
void test_stop_restart_replay_and_old_frame() {
    Fixture f;
    f.exact_ground_support();
    auto first = f.next_view();
    first.metadata.source.host_ready_mono_ns = 0;
    f.take(first, f.frame);
    expect_normal(f.process_current(f.frame));
    CHECK(f.geometry->result().source.host_ready_mono_ns == 0);
    CHECK(f.process_current(f.frame).status == GeometryStatus::InactiveOrStale);
    expect_cleared(f);
    nd::Frame newer(kGeometryPixels);
    f.take(f.next_view(), newer);
    expect_normal(f.process_current(newer));
    CHECK(f.process_current(f.frame).status == GeometryStatus::InactiveOrStale);
    expect_cleared(f);
    f.mailbox.end_stream();
    CHECK(f.process_current(newer).status == GeometryStatus::InactiveOrStale);
    expect_cleared(f);
    const auto previous_stream = f.stream;
    f.stream = f.mailbox.begin_stream();
    f.sequence = 0;
    CHECK(f.stream != previous_stream);
    CHECK(f.process_current(newer).status == GeometryStatus::InactiveOrStale);
    expect_cleared(f);
    const auto& restarted = f.process();
    expect_normal(restarted);
    CHECK(restarted.source.stream_id == f.stream && restarted.source.sequence == 1);
}

// Any change of intrinsics, size, depth unit or distortion (even -0.0 for 0) is refused as
// ProfileMismatch and clears the output; the mailbox refuses a frame without distortion.
void test_profile_metadata_changes() {
    Fixture f;
    f.exact_ground_support();
    expect_normal(f.process());
    const auto reject = [&](auto change) {
        auto view = f.next_view();
        change(view.metadata);
        f.take(view, f.frame);
        CHECK(f.process_current(f.frame).status == GeometryStatus::ProfileMismatch);
        expect_cleared(f);
    };
    reject([](nd::Metadata& m) { m.intrinsics.fx = std::nextafter(m.intrinsics.fx, 1000.0F); });
    reject([](nd::Metadata& m) { m.intrinsics.fy = std::nextafter(m.intrinsics.fy, 1000.0F); });
    reject([](nd::Metadata& m) { m.intrinsics.cx += 1; });
    reject([](nd::Metadata& m) { m.intrinsics.cy += 1; });
    reject([](nd::Metadata& m) { m.intrinsics.width = 800; m.intrinsics.height = 320; });
    reject([](nd::Metadata& m) { m.unit_m *= 2; });
    reject([](nd::Metadata& m) { m.distortion.model += 1; });
    for (std::size_t i = 0; i < f.binding.distortion.coefficients.size(); ++i)
        reject([i](nd::Metadata& m) { m.distortion.coefficients[i] = 0.01F; });
    reject([](nd::Metadata& m) {
        CHECK(m.distortion.coefficients[0] == 0 && !std::signbit(m.distortion.coefficients[0]));
        m.distortion.coefficients[0] = -0.0F;
        CHECK(m.distortion.coefficients[0] == 0 && std::signbit(m.distortion.coefficients[0]));
    });
    auto missing_distortion = f.next_view();
    missing_distortion.metadata.distortion.reported = false;
    CHECK(f.mailbox.try_offer(missing_distortion) == nd::Offer::Invalid);
    expect_normal(f.process());
}

// The constructor refuses bad ray tables, non-rotation or non-finite extrinsics, mismatched
// intrinsics, a camera below the prior ground plane, and out-of-range options.
void test_invalid_construction() {
    const auto binding = synthetic_binding();
    const auto options = synthetic_options();
    auto rays = pinhole_rays(binding);
    const auto reject = [&](const ProjectionBinding& b, const Vec3* r, std::size_t n,
                            const GeometryOptions& o) {
        bool rejected = false;
        try { NativeGeometry invalid(b, r, n, o); }
        catch (const std::invalid_argument&) { rejected = true; }
        CHECK(rejected);
    };
    reject(binding, nullptr, rays.size(), options);
    reject(binding, rays.data(), rays.size() - 1, options);
    rays.back().z = 2;
    reject(binding, rays.data(), rays.size(), options);
    rays.back().z = 1;
    rays.back().x = std::numeric_limits<double>::quiet_NaN();
    reject(binding, rays.data(), rays.size(), options);
    rays = pinhole_rays(binding);
    auto bad_binding = binding;
    bad_binding.R_base_camera[0] = 0.1;
    reject(bad_binding, rays.data(), rays.size(), options);
    bad_binding = binding;
    bad_binding.R_base_camera[3] = 1;  // Orthogonal reflection, determinant -1.
    reject(bad_binding, rays.data(), rays.size(), options);
    bad_binding = binding;
    bad_binding.t_base_camera.x = std::numeric_limits<double>::infinity();
    reject(bad_binding, rays.data(), rays.size(), options);
    bad_binding = binding;
    bad_binding.intrinsics.width = 1280;
    reject(bad_binding, rays.data(), rays.size(), options);
    bad_binding = binding;
    bad_binding.unit_m = 0;
    reject(bad_binding, rays.data(), rays.size(), options);
    bad_binding = binding;
    bad_binding.distortion.reported = false;
    reject(bad_binding, rays.data(), rays.size(), options);
    bad_binding = binding;
    bad_binding.t_base_camera.z = -1;  // All prior intersections have negative depth.
    reject(bad_binding, rays.data(), rays.size(), options);
    const auto reject_option = [&](auto change) {
        auto bad = options;
        change(bad);
        reject(binding, rays.data(), rays.size(), bad);
    };
    reject_option([](GeometryOptions& o) { o.fit.hypotheses = 0; });
    reject_option([](GeometryOptions& o) { o.fit.min_inlier_fraction = 1.1; });
    reject_option([](GeometryOptions& o) { o.prior.normal = {0, 0, 0}; });
    reject_option([](GeometryOptions& o) { o.fit_near_max_m = o.fit_near_min_m; });
    reject_option([](GeometryOptions& o) { o.fit_prior_band_m = 0; });
    reject_option([](GeometryOptions& o) { o.range_max_m = 0; });
    reject_option([](GeometryOptions& o) { o.range_max_m = 6.5; });
    reject_option([](GeometryOptions& o) { o.blind_near_lo_m = -1; });
    reject_option([](GeometryOptions& o) { o.h_tol_m = -1; });
    reject_option([](GeometryOptions& o) { o.z_pass_m = o.h_tol_m; });
    reject_option([](GeometryOptions& o) { o.eps_neg = -1; });
    reject_option([](GeometryOptions& o) { o.w_h_m = -1; });
    reject_option([](GeometryOptions& o) { o.angle_step_rad = 0; });
    reject_option([](GeometryOptions& o) { o.n_bins = 0; });
    reject_option([](GeometryOptions& o) { o.n_bins = kGeometryMaxBins + 1; });
}

}  // namespace

int main() {
    const struct Test { const char* name; void (*run)(); } tests[] = {
        {"height_window_many_obstacles", test_height_window_many_obstacles},
        {"full_frames_all_pixels_thin_obstacle", test_full_frames_and_one_pixel_obstacle},
        {"exact_height_classification_boundaries", test_exact_classification_boundaries},
        {"paired_crossbar_final_closest_height_window", test_paired_crossbar_and_final_height_window},
        {"translated_camera_expected_pierce_bin", test_translated_camera_and_expected_pierce_bin},
        {"current_plane_rejects_prior_false_pierce", test_current_plane_rejects_prior_false_pierce},
        {"all_invalid_clears_and_recovers", test_failure_clearing_and_recovery},
        {"raw_ledger_fit_independent_unclassified", test_raw_ledger_independent_of_fit_and_classifications},
        {"raw_ledger_actual_return_domain", test_raw_ledger_uses_actual_return_domain},
        {"raw_ledger_invalid_frame_generation_clear", test_raw_ledger_invalid_frame_and_generation_clear},
        {"raw_ledger_numerical_prefix_cleared", test_raw_ledger_numerical_failure_clears_partial_scan},
        {"raw_validity_requires_allowed_status", test_raw_validity_needs_allowed_status},
        {"mailbox_owner_numeric_identity_collision", test_mailbox_owner_rejects_matching_numeric_identity},
        {"stop_restart_replay_old_frame", test_stop_restart_replay_and_old_frame},
        {"profile_metadata_changes", test_profile_metadata_changes},
        {"invalid_construction", test_invalid_construction},
    };
    for (const auto& test : tests) {
        test.run();
        std::cout << "PASS " << test.name << '\n';
    }
    const Fixture storage_fixture;
    std::cout << "{\"suite\":\"synthetic_native_geometry\",\"tests_passed\":"
              << std::size(tests) << ",\"zero_cpp_allocation_process_calls\":"
              << checked_process_calls
              << ",\"geometry_buffer_capacity_bytes\":"
              << storage_fixture.geometry->storage_bytes()
              << ",\"production_calibration_claim\":false,\"profile_or_free_claim\":false}\n";
}
