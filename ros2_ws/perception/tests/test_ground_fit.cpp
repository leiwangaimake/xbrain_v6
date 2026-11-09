/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_ground_fit.cpp
 * Brief: Unit tests for the bounded robust ground-plane fit (19 section 3.2A).
 *
 * Description:
 * What it checks. The fit returns a plane only when its gates pass; no prior or fallback is ever
 * returned as success. Cases cover exact planes, outliers and determinism with a fixed seed, input
 * and option validation, each rejection gate, boundary values and the final metrics, and large and
 * centred inputs. Built with PERCEPTION_FIT_TEST_HOOKS it also checks (2026-10-24, G-P1 phase 1)
 * that the bounded-input passes give the checked passes' result bit for bit, on realistic noisy
 * ground scenes, at both sides of the bound, and on inputs large enough to overflow.
 */

#include "modules/ground_fit.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <vector>

namespace {
bool count_allocations = false;
std::size_t allocations = 0;
std::size_t checked_fit_calls = 0;
int failures = 0;
}

void* operator new(std::size_t size) {
    if (count_allocations) ++allocations;
    if (void* pointer = std::malloc(size == 0 ? 1 : size)) return pointer;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void* operator new(std::size_t size, std::align_val_t alignment) {
    if (count_allocations) ++allocations;
    void* pointer = nullptr;
    if (posix_memalign(&pointer, static_cast<std::size_t>(alignment), size == 0 ? 1 : size) != 0) {
        throw std::bad_alloc();
    }
    return pointer;
}
void* operator new[](std::size_t size, std::align_val_t alignment) {
    return ::operator new(size, alignment);
}
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::align_val_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::align_val_t) noexcept { std::free(pointer); }
#if defined(__cpp_sized_deallocation)
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t, std::align_val_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t, std::align_val_t) noexcept { std::free(pointer); }
#endif

namespace {
using perception::geometry::FitOptions;
using perception::geometry::FitResult;
using perception::geometry::FitStatus;
using perception::geometry::Plane;
using perception::geometry::Vec3;
using perception::geometry::fit_ground;
using perception::geometry::kMaxFitPoints;
using perception::geometry::valid_fit_options;

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        ++failures; \
    } \
} while (false)

FitOptions options() {
    FitOptions value;
    value.inlier_distance_m = 0.025;
    value.min_inlier_fraction = 0.65;
    value.max_inlier_rms_m = 0.01;
    value.min_xy_stddev_m = 0.05;
    value.min_up_dot = 0.8;
    value.max_prior_angle_rad = 0.6;
    value.min_inliers = 12;
    value.hypotheses = 128;
    value.seed = UINT64_C(0x625cbf261358);
    return value;
}

Plane horizontal(double height = 0.0) { return {{0.0, 0.0, 1.0}, -height}; }

Plane equation(double slope_x, double slope_y, double height) {
    const double length = std::hypot(slope_x, slope_y, 1.0);
    return {{-slope_x / length, -slope_y / length, 1.0 / length}, -height / length};
}

std::vector<Vec3> grid(double slope_x = 0, double slope_y = 0, double height = 0,
                       int side = 13) {
    std::vector<Vec3> points;
    points.reserve(static_cast<std::size_t>(side * side));
    for (int x = 0; x < side; ++x) {
        for (int y = 0; y < side; ++y) {
            const double px = static_cast<double>(x - side / 2) * 0.1;
            const double py = static_cast<double>(y - side / 2) * 0.1;
            points.push_back({px, py, slope_x * px + slope_y * py + height});
        }
    }
    return points;
}

// Every fit invocation, including invalid input/options/prior, uses this
// measured entry point. Vector setup and assertion reporting stay outside it.
FitResult run(const Vec3* points, std::size_t count, const Plane& prior, const FitOptions& fit_options) {
    const std::size_t before = allocations;
    count_allocations = true;
    const FitResult result = fit_ground(points, count, prior, fit_options);
    count_allocations = false;
    ++checked_fit_calls;
    CHECK(allocations == before);
    CHECK(result.candidates == count);
    CHECK(result.hypotheses_tested <= fit_options.hypotheses);
    CHECK((result.status == FitStatus::Pass) == result.plane.has_value());
    return result;
}

FitResult run(const std::vector<Vec3>& points, const Plane& prior, const FitOptions& fit_options) {
    return run(points.data(), points.size(), prior, fit_options);
}

void expect_plane(const FitResult& result, const Plane& expected, double tolerance = 1e-10) {
    CHECK(result.status == FitStatus::Pass);
    CHECK(result.plane.has_value());
    if (!result.plane) return;
    CHECK(std::abs(result.plane->normal.x - expected.normal.x) <= tolerance);
    CHECK(std::abs(result.plane->normal.y - expected.normal.y) <= tolerance);
    CHECK(std::abs(result.plane->normal.z - expected.normal.z) <= tolerance);
    CHECK(std::abs(result.plane->d - expected.d) <= tolerance);
    CHECK(result.plane->normal.z > 0.0);
    CHECK(std::abs(std::hypot(result.plane->normal.x, result.plane->normal.y,
                              result.plane->normal.z) - 1.0) < 1e-12);
}

void expect_failure(const FitResult& result, FitStatus status) {
    CHECK(result.status == status);
    CHECK(!result.plane.has_value());
}

// Exact flat, offset and tilted planes are recovered with every point an inlier, even when the
// prior has none; with a single hypothesis and a zero RMS limit the offset plane fails.
void test_exact_planes() {
    auto fit_options = options();
    const auto flat = grid();
    const auto first = run(flat, horizontal(), fit_options);
    expect_plane(first, horizontal());
    CHECK(first.inliers == flat.size());
    CHECK(first.inlier_fraction == 1.0);
    CHECK(first.inlier_rms_m == 0.0);

    const auto offset = grid(0, 0, 0.28);
    // A prior with zero inliers must not prevent recovery of an offset plane.
    const auto shifted = run(offset, horizontal(), fit_options);
    expect_plane(shifted, horizontal(0.28));
    CHECK(shifted.inliers == offset.size());

    const auto tilted = grid(0.12, -0.19, 0.08);
    const auto sloped = run(tilted, horizontal(), fit_options);
    expect_plane(sloped, equation(0.12, -0.19, 0.08));
    CHECK(sloped.inliers == tilted.size());
    CHECK(sloped.inlier_rms_m < 1e-12);

    fit_options.hypotheses = 1;
    fit_options.max_inlier_rms_m = 0;
    expect_plane(run(flat, horizontal(), fit_options), horizontal());
    expect_failure(run(offset, horizontal(), fit_options), FitStatus::InsufficientInliers);
}

// Raised outliers are excluded and the plane is still exact; a fixed seed repeats the result bit
// for bit.
void test_outliers_and_determinism() {
    auto points = grid(0.08, -0.04, 0.015);
    const std::size_t ground_count = points.size();
    for (int i = 0; i < 64; ++i) {
        points.push_back({static_cast<double>(i % 8) * 0.09 - 0.4,
                          static_cast<double>(i / 8) * 0.11 - 0.4,
                          0.3 + static_cast<double>((i * 17) % 31) * 0.02});
    }
    auto fit_options = options();
    fit_options.seed = 0;
    const auto first = run(points, horizontal(), fit_options);
    expect_plane(first, equation(0.08, -0.04, 0.015));
    CHECK(first.inliers == ground_count);
    CHECK(first.inlier_fraction == static_cast<double>(ground_count) / static_cast<double>(points.size()));
    for (int repeat = 0; repeat < 8; ++repeat) {
        const auto next = run(points, horizontal(), fit_options);
        CHECK(next.status == first.status);
        CHECK(next.inliers == first.inliers);
        CHECK(next.hypotheses_tested == first.hypotheses_tested);
        CHECK(next.inlier_fraction == first.inlier_fraction);
        CHECK(next.inlier_rms_m == first.inlier_rms_m);
        if (next.plane && first.plane) {
            CHECK(next.plane->normal.x == first.plane->normal.x);
            CHECK(next.plane->normal.y == first.plane->normal.y);
            CHECK(next.plane->normal.z == first.plane->normal.z);
            CHECK(next.plane->d == first.plane->d);
        }
    }
}

// Null, oversize or non-finite input and an invalid prior are InvalidInput; too few points is
// InsufficientCandidates; a prior normal within 1e-10 of unit length is accepted.
void test_input_validation() {
    const auto fit_options = options();
    expect_failure(run(nullptr, 0, horizontal(), fit_options), FitStatus::InsufficientCandidates);
    expect_failure(run(nullptr, 50, horizontal(), fit_options), FitStatus::InvalidInput);
    const Vec3 one{};
    expect_failure(run(&one, kMaxFitPoints + 1, horizontal(), fit_options), FitStatus::InvalidInput);
    expect_failure(run(&one, 1, horizontal(), fit_options), FitStatus::InsufficientCandidates);

    auto points = grid();
    points[5].x = std::numeric_limits<double>::quiet_NaN();
    expect_failure(run(points, horizontal(), fit_options), FitStatus::InvalidInput);
    points[5].x = 0;
    points[5].y = std::numeric_limits<double>::infinity();
    expect_failure(run(points, horizontal(), fit_options), FitStatus::InvalidInput);
    points[5].y = 0;
    points[5].z = -std::numeric_limits<double>::infinity();
    expect_failure(run(points, horizontal(), fit_options), FitStatus::InvalidInput);

    points = grid();
    expect_failure(run(points, {{0, 0, 2}, 0}, fit_options), FitStatus::InvalidInput);
    expect_failure(run(points, {{0, 0, 0}, 0}, fit_options), FitStatus::InvalidInput);
    expect_failure(run(points, {{0, 0, -1}, 0}, fit_options), FitStatus::InvalidInput);
    expect_failure(run(points, {{0, 0, 1}, std::numeric_limits<double>::quiet_NaN()}, fit_options), FitStatus::InvalidInput);
    expect_failure(run(points, {{std::numeric_limits<double>::quiet_NaN(), 0, 1}, 0}, fit_options), FitStatus::InvalidInput);
    expect_plane(run(points, {{0, 0, 1.0 + 1e-10}, 0}, fit_options), horizontal());
}

// Every option is range-checked with inclusive limits (256 hypotheses pass, 257 fail); invalid
// options fail the fit as InvalidInput.
void test_options_validation() {
    const auto base = options();
    CHECK(valid_fit_options(base));
    CHECK(!valid_fit_options(FitOptions{}));
    const std::array<double, 3> nonfinite{{std::numeric_limits<double>::quiet_NaN(),
                                        std::numeric_limits<double>::infinity(),
                                        -std::numeric_limits<double>::infinity()}};
    const std::array<double FitOptions::*, 6> fields{{&FitOptions::inlier_distance_m,
        &FitOptions::min_inlier_fraction, &FitOptions::max_inlier_rms_m,
        &FitOptions::min_xy_stddev_m, &FitOptions::min_up_dot, &FitOptions::max_prior_angle_rad}};
    for (const auto field : fields) {
        for (const double value : nonfinite) {
            auto candidate = base;
            candidate.*field = value;
            CHECK(!valid_fit_options(candidate));
        }
    }
    auto value = base;
    value.inlier_distance_m = 0; CHECK(!valid_fit_options(value));
    value = base; value.inlier_distance_m = -1; CHECK(!valid_fit_options(value));
    value = base; value.min_inlier_fraction = 0; CHECK(!valid_fit_options(value));
    value = base; value.min_inlier_fraction = 1.001; CHECK(!valid_fit_options(value));
    value = base; value.max_inlier_rms_m = -1; CHECK(!valid_fit_options(value));
    value = base; value.min_xy_stddev_m = 0; CHECK(!valid_fit_options(value));
    value = base; value.min_up_dot = -0.001; CHECK(!valid_fit_options(value));
    value = base; value.min_up_dot = 1.001; CHECK(!valid_fit_options(value));
    value = base; value.max_prior_angle_rad = -0.001; CHECK(!valid_fit_options(value));
    value = base; value.max_prior_angle_rad = 3.142; CHECK(!valid_fit_options(value));
    value = base; value.min_inliers = 2; CHECK(!valid_fit_options(value));
    value = base; value.min_inliers = kMaxFitPoints + 1; CHECK(!valid_fit_options(value));
    value = base; value.hypotheses = 0; CHECK(!valid_fit_options(value));
    value = base; value.hypotheses = 257; CHECK(!valid_fit_options(value));
    value = base; value.hypotheses = 256; CHECK(valid_fit_options(value));
    value = base; value.min_inliers = kMaxFitPoints; CHECK(valid_fit_options(value));
    value = base; value.max_inlier_rms_m = 0; CHECK(valid_fit_options(value));
    value = base; value.min_up_dot = 0; CHECK(valid_fit_options(value));
    value = base; value.min_up_dot = 1; CHECK(valid_fit_options(value));
    value = base; value.max_prior_angle_rad = 0; CHECK(valid_fit_options(value));
    value = base; value.max_prior_angle_rad = std::acos(-1.0); CHECK(valid_fit_options(value));
    value = base; value.hypotheses = 0;
    expect_failure(run(grid(), horizontal(), value), FitStatus::InvalidInput);
}

// Each gate rejects with its own status: degenerate support, too few inliers, residual too high,
// and a normal too far from the prior or from up.
void test_rejection_gates() {
    auto fit_options = options();
    const std::vector<Vec3> identical(30, {0.1, -0.2, 0});
    expect_failure(run(identical, horizontal(), fit_options), FitStatus::DegenerateSupport);
    std::vector<Vec3> line;
    for (int i = -30; i <= 30; ++i) {
        line.push_back({static_cast<double>(i) * 0.1, static_cast<double>(i) * 0.2, 0});
    }
    expect_failure(run(line, horizontal(), fit_options), FitStatus::DegenerateSupport);

    auto points = grid();
    for (int i = 0; i < 40; ++i) points.push_back({static_cast<double>(i) * 0.01, 0.5, 1.0});
    fit_options.min_inlier_fraction = 0.9;
    expect_failure(run(points, horizontal(), fit_options), FitStatus::InsufficientInliers);

    points = grid();
    for (std::size_t i = 0; i < points.size(); ++i) points[i].z = (i % 2 == 0) ? 0.012 : -0.012;
    fit_options = options();
    fit_options.inlier_distance_m = 0.05;
    fit_options.max_inlier_rms_m = 0.003;
    const auto noisy = run(points, horizontal(), fit_options);
    expect_failure(noisy, FitStatus::ResidualExceeded);
    CHECK(noisy.inliers == points.size());
    CHECK(noisy.inlier_rms_m > 0.01);

    points = grid(0.4, 0, 0);
    fit_options = options();
    fit_options.max_prior_angle_rad = 0.2;
    expect_failure(run(points, horizontal(), fit_options), FitStatus::NormalRejected);
    fit_options.max_prior_angle_rad = 1.0;
    fit_options.min_up_dot = 0.99;
    expect_failure(run(points, horizontal(), fit_options), FitStatus::NormalRejected);
    // The same slope is allowed against its matching prior with a permissive up bound.
    fit_options.min_up_dot = 0.8;
    fit_options.max_prior_angle_rad = 0;
    expect_plane(run(points, equation(0.4, 0, 0), fit_options), equation(0.4, 0, 0));
    fit_options.max_prior_angle_rad = std::acos(equation(0.4, 0, 0).normal.z);
    fit_options.min_up_dot = equation(0.4, 0, 0).normal.z;
    expect_plane(run(points, horizontal(), fit_options), equation(0.4, 0, 0));
}

// Inlier fraction, support spread, inlier distance and RMS limits are inclusive; the reported
// inlier count and RMS match a recomputation against the returned plane.
void test_boundaries_and_final_metrics() {
    std::vector<Vec3> points{{-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0}, {0, 0, 3}};
    auto fit_options = options();
    fit_options.min_inliers = 4;
    fit_options.min_inlier_fraction = 0.8;
    fit_options.min_xy_stddev_m = 1;
    const auto exact = run(points, horizontal(), fit_options);
    expect_plane(exact, horizontal());
    CHECK(exact.inliers == 4);
    CHECK(exact.inlier_fraction == 0.8);
    fit_options.min_inlier_fraction = std::nextafter(0.8, 1.0);
    expect_failure(run(points, horizontal(), fit_options), FitStatus::InsufficientInliers);
    fit_options.min_inlier_fraction = 0.8;
    fit_options.min_xy_stddev_m = std::nextafter(1.0, 2.0);
    expect_failure(run(points, horizontal(), fit_options), FitStatus::DegenerateSupport);
    fit_options.min_xy_stddev_m = 1;
    fit_options.min_inliers = 5;
    expect_failure(run(points, horizontal(), fit_options), FitStatus::InsufficientInliers);

    // Inclusive threshold: balanced points on both +/- threshold belong to
    // the same plane; the final RMS reaches its inclusive acceptance bound.
    points = {{-1, -1, -0.125}, {-1, 1, 0.125}, {1, -1, 0.125}, {1, 1, -0.125}};
    fit_options = options();
    fit_options.min_inliers = 4;
    fit_options.min_inlier_fraction = 1;
    fit_options.hypotheses = 1;
    fit_options.inlier_distance_m = 0.125;
    fit_options.max_inlier_rms_m = 0.125;
    const auto inclusive = run(points, horizontal(), fit_options);
    expect_plane(inclusive, horizontal());
    CHECK(inclusive.inliers == 4);
    CHECK(inclusive.inlier_rms_m == 0.125);
    fit_options.max_inlier_rms_m = std::nextafter(0.125, 0.0);
    expect_failure(run(points, horizontal(), fit_options), FitStatus::ResidualExceeded);

    // Verify final reported membership and RMS directly against returned plane.
    points = grid(0.17, 0.03, 0.015);
    for (std::size_t i = 0; i < points.size(); ++i) {
        points[i].z += static_cast<double>(static_cast<int>(i % 7) - 3) * 0.001;
    }
    points.push_back({0, 0, 0.5});
    fit_options = options();
    const auto result = run(points, horizontal(), fit_options);
    CHECK(result.status == FitStatus::Pass);
    if (result.plane) {
        std::size_t inliers = 0;
        double sum = 0;
        for (const auto& point : points) {
            const auto& plane = *result.plane;
            const double residual = plane.normal.x * point.x + plane.normal.y * point.y +
                                    plane.normal.z * point.z + plane.d;
            if (std::abs(residual) <= fit_options.inlier_distance_m) {
                ++inliers;
                sum += residual * residual;
            }
        }
        CHECK(result.inliers == inliers);
        CHECK(std::abs(result.inlier_rms_m - std::sqrt(sum / static_cast<double>(inliers))) < 1e-15);
    }
}

// The maximum point count fits, and a plane 1e8 m from the origin is still exact (inputs are
// centred).
void test_large_and_centered_input() {
    auto fit_options = options();
    fit_options.hypotheses = 1;
    std::vector<Vec3> maximum;
    maximum.reserve(kMaxFitPoints);
    for (std::size_t i = 0; i < kMaxFitPoints; ++i) {
        maximum.push_back({static_cast<double>(i % 640U) * 0.01,
                           static_cast<double>(i / 640U) * 0.01, 0});
    }
    fit_options.min_inliers = kMaxFitPoints;
    fit_options.min_inlier_fraction = 1;
    const auto full = run(maximum, horizontal(), fit_options);
    expect_plane(full, horizontal());
    CHECK(full.inliers == kMaxFitPoints);
    CHECK(full.hypotheses_tested == 1);

    auto translated = grid();
    for (auto& point : translated) {
        point.x += 1e8;
        point.y -= 1e8;
        point.z = 0.3;
    }
    fit_options = options();
    expect_plane(run(translated, horizontal(0.3), fit_options), horizontal(0.3));
}
#ifdef PERCEPTION_FIT_TEST_HOOKS
// Bit-for-bit equality of two fit results (status, counts, fraction, RMS, plane).
bool same_bits(const FitResult& a, const FitResult& b) {
    const auto same = [](double x, double y) { return std::memcmp(&x, &y, sizeof x) == 0; };
    if (a.status != b.status || a.candidates != b.candidates || a.inliers != b.inliers ||
        a.hypotheses_tested != b.hypotheses_tested || !same(a.inlier_fraction, b.inlier_fraction) ||
        !same(a.inlier_rms_m, b.inlier_rms_m) || a.plane.has_value() != b.plane.has_value()) return false;
    if (!a.plane) return true;
    return same(a.plane->normal.x, b.plane->normal.x) && same(a.plane->normal.y, b.plane->normal.y) &&
           same(a.plane->normal.z, b.plane->normal.z) && same(a.plane->d, b.plane->d);
}

// The same fit with the bounded-input passes and with the checked passes forced.
bool bounded_equals_checked(const std::vector<Vec3>& points, const Plane& prior, const FitOptions& fit_options,
                            FitResult* bounded_result = nullptr) {
    perception::geometry::g_fit_force_checked = false;
    const auto bounded = run(points, prior, fit_options);
    perception::geometry::g_fit_force_checked = true;
    const auto checked = run(points, prior, fit_options);
    perception::geometry::g_fit_force_checked = false;
    if (bounded_result) *bounded_result = bounded;
    return same_bits(bounded, checked);
}

// Realistic ground scenes like the near-field fit candidates (0.6-3 m ahead, +-1.5 m aside):
// a tilted floor with millimetre noise and a share of points up to 15 cm off it (curbs, clutter),
// in random order, and the same scene shifted to just inside / just outside the bound; then points
// spread over 1e200 m, whose covariance overflows (the checked pass reports it as InvalidInput).
void test_bounded_passes_bit_identical() {
    std::uint64_t state = UINT64_C(0x5eed20261024);
    const auto uniform = [&state]() {
        state = state * UINT64_C(6364136223846793005) + UINT64_C(1442695040888963407);
        return static_cast<double>(state >> 11U) * (1.0 / 9007199254740992.0);
    };
    auto fit_options = options();
    fit_options.inlier_distance_m = 0.02;
    fit_options.min_inlier_fraction = 0.3;
    fit_options.max_inlier_rms_m = 0.02;
    fit_options.min_xy_stddev_m = 0.1;
    fit_options.min_up_dot = 0.9;
    fit_options.max_prior_angle_rad = 10.0 * std::acos(-1.0) / 180.0;
    fit_options.min_inliers = 500;
    fit_options.hypotheses = 64;
    fit_options.seed = 1;
    std::size_t passed = 0;
    for (int scene = 0; scene < 24; ++scene) {
        const double slope_x = (uniform() - 0.5) * 0.12, slope_y = (uniform() - 0.5) * 0.12;
        const double height = (uniform() - 0.5) * 0.1, noise = 0.002 + uniform() * 0.006;
        const double off_share = 0.05 + uniform() * 0.35;
        const std::size_t count = 2000 + static_cast<std::size_t>(uniform() * 18000.0);
        std::vector<Vec3> points(count);
        for (auto& p : points) {
            p.x = 0.6 + uniform() * 2.4;
            p.y = (uniform() - 0.5) * 3.0;
            p.z = slope_x * p.x + slope_y * p.y + height + (uniform() + uniform() + uniform() - 1.5) * 2.0 * noise;
            if (uniform() < off_share) p.z += (uniform() - 0.5) * 0.3;
        }
        FitResult result;
        CHECK(bounded_equals_checked(points, horizontal(), fit_options, &result));
        passed += result.status == FitStatus::Pass;
        // An unreachable inlier share stops after the hypotheses: the reported RMS is then the
        // hypothesis score's own sum (score_plane), not the refinement's.
        auto strict = fit_options;
        strict.min_inlier_fraction = 0.995;
        FitResult stopped;
        CHECK(bounded_equals_checked(points, horizontal(), strict, &stopped));
        CHECK(stopped.status == FitStatus::InsufficientInliers && stopped.inlier_rms_m > 0);
        // Just inside and just outside the bound (1e6 m): both sides agree with the checked passes.
        if (scene % 6 == 0) {
            for (const double shift : {1e6 - 4.0, 1e6 + 4.0}) {
                auto shifted = points;
                for (auto& p : shifted) p.x += shift;
                CHECK(bounded_equals_checked(shifted, horizontal(), fit_options));
            }
        }
    }
    CHECK(passed >= 18);  // the scenes are mostly fittable, so the refinement passes really ran
    // Spread over 1e200 m: the Welford sums overflow; both runs must report it the same way.
    std::vector<Vec3> huge;
    for (int x = 0; x < 40; ++x)
        for (int y = 0; y < 40; ++y) huge.push_back({1e200 * (1.0 + x * 0.02), 1e200 * (y * 0.02 - 0.4), 0.0});
    fit_options = options();
    FitResult overflow;
    CHECK(bounded_equals_checked(huge, horizontal(), fit_options, &overflow));
    CHECK(overflow.status == FitStatus::InvalidInput);
}
#endif
}  // namespace

int main() {
    test_exact_planes();
    test_outliers_and_determinism();
    test_input_validation();
    test_options_validation();
    test_rejection_gates();
    test_boundaries_and_final_metrics();
    test_large_and_centered_input();
#ifdef PERCEPTION_FIT_TEST_HOOKS
    test_bounded_passes_bit_identical();
#endif
    if (failures != 0) {
        std::fprintf(stderr, "ground-fit tests: %d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("ground-fit tests: PASS (bounded synthetic numerical/support gates; "
                "zero ordinary/aligned C++ new allocations across %zu fitGround calls)\n", checked_fit_calls);
    return EXIT_SUCCESS;
}
