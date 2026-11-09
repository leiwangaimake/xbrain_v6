/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: ground_fit.hpp
 * Brief: Pure bounded normal-fit component.
 *
 * Description:
 * No prior/fallback is returned as success. R4 19 section 3.2A permits RANSAC/IRLS; numerical implementation choices below require explicit caller values, not invented deployment defaults.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace perception::geometry {

struct Vec3 { double x = 0, y = 0, z = 0; };
struct Plane { Vec3 normal; double d = 0; };
inline constexpr std::size_t kMaxFitPoints = 640U * 400U;

struct FitOptions {
    // Orthogonal point-to-plane distance, metres; inlier test <= threshold.
    double inlier_distance_m = 0;
    double min_inlier_fraction = 0;
    double max_inlier_rms_m = 0;
    // Both inlier XY covariance principal standard deviations must be >=
    // this value. Guards point/line support, not semantic ground identity.
    double min_xy_stddev_m = 0;
    // Unit normal must face base +Z and obey BOTH these constraints.
    double min_up_dot = 0;
    double max_prior_angle_rad = 0;
    std::size_t min_inliers = 0;
    // Fixed bounded hypotheses, no convergence-based unbounded retries.
    std::size_t hypotheses = 0;
    std::uint64_t seed = 0;
};

enum class FitStatus {
    Pass, InvalidInput, InsufficientCandidates, InsufficientInliers,
    DegenerateSupport, ResidualExceeded, NormalRejected
};
struct FitResult {
    FitStatus status = FitStatus::InvalidInput;
    std::optional<Plane> plane;
    std::size_t candidates = 0, inliers = 0, hypotheses_tested = 0;
    double inlier_fraction = 0, inlier_rms_m = 0;
};

bool valid_fit_options(const FitOptions&) noexcept;
// All input candidates must be finite, near-region/prior-band selected by the
// caller. Fraction denominator is exactly count (not all image pixels).
// No allocation, retained input pointer, time dependence, or shared RNG.
FitResult fit_ground(const Vec3* candidates, std::size_t count,
                    const Plane& prior, const FitOptions&) noexcept;

#ifdef PERCEPTION_FIT_TEST_HOOKS
// Host tests only (neither the product nor the bench defines the macro): true makes every fit use
// the checked scoring and covariance passes, so a test can compare them bit for bit with the
// bounded-input passes on the same input.
extern bool g_fit_force_checked;
#endif

#ifdef PERCEPTION_FIT_DIAGNOSTICS
// Development bench only (perception_geometry_bench defines the macro; the product never does):
// CLOCK_MONOTONIC stamps of the last fit_ground call -- entry, after the prior plane's score,
// after the hypothesis loop -- and how many hypotheses needed a full score_plane pass.
struct FitDiagnostics {
    std::int64_t entry_ns = -1, prior_scored_ns = -1, hypotheses_done_ns = -1;
    std::size_t full_scores = 0, refine_rounds = 0;
};
extern FitDiagnostics g_fit_diagnostics;
#endif

}  // namespace perception::geometry
