/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: ground_fit.cpp
 * Brief: Bounded RANSAC + total-least-squares ground plane fit.
 *
 * Description:
 * What problem this solves. Implements fit_ground() (19 section 3.2A permits RANSAC/IRLS). The
 * ground plane is the reference every height class and FREE distance is measured against, so a
 * wrong plane silently turns ground into obstacles or the reverse. The fit therefore:
 *   1. scores the prior plane and a fixed number of random 3-point hypotheses (deterministic
 *      seed, no retries: bounded time on the fast line), keeping the one with most inliers,
 *      ties broken by the smaller residual;
 *   2. refines it three times by total least squares on its inliers (smallest-eigenvalue
 *      normal of the inlier covariance);
 *   3. accepts it only with enough inliers, a real 2-D spread in x/y (not a line of points),
 *      a normal close to up and to the prior, and a small inlier RMS.
 * Any failed gate returns a status and no plane; the prior is never returned as a success.
 * Bounded input (2026-10-24, G-P1 phase 1): when every coordinate is within kBoundedCoordinate the
 * scoring and covariance passes run without their per-point finiteness checks (they provably
 * cannot fire there) and keep their sums in locals; same operations in the same order, so the
 * same results bit for bit. Any other input keeps the checked passes.
 */
#include "modules/ground_fit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#ifdef PERCEPTION_FIT_DIAGNOSTICS
#include <cstdlib>
#include <ctime>
#endif

namespace perception::geometry {
namespace {

// Upper bound of the configured hypothesis budget, and the TLS refinement rounds.
constexpr std::size_t kMaxHypotheses = 256;
constexpr std::size_t kRefinementRounds = 3;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kRoundoff = 64.0 * std::numeric_limits<double>::epsilon();
// |x*nx + y*ny + z*nz + d| <= 3*max|coordinate| + |d| for a unit normal; below
// this bound no residual can overflow, so early rejection cannot hide a
// non-finite residual that the full scan would have reported.
constexpr double kOverflowFreeCoordinate = 0.125 * std::numeric_limits<double>::max();
// Bounded input (G-P1 phase 1). With every |coordinate| <= kBoundedCoordinate and a plane whose
// normal is unit length and |d| <= kBoundedOffset, nothing in score_plane / inlier_covariance /
// score_and_covariance can leave the finite range: |residual| <= 3e6 + 1e7; the Welford mean is a
// running average of the points (rounding growth <= (1 + 2 eps)^kMaxFitPoints < 1 + 1e-9), so every
// |delta| <= 2.1e6, every covariance increment <= 4.5e12 and their sum over kMaxFitPoints points
// <= 1.2e18. Their finiteness checks are then always true and those passes skip them. Every plane
// they score has a unit normal (the normalised prior, triple_plane, tls_plane) and, from bounded
// points, |d| <= sqrt(3) * kBoundedCoordinate; only a prior d outside kBoundedOffset is checked.
constexpr double kBoundedCoordinate = 1e6;  // metres; a depth return is within tens of metres
constexpr double kBoundedOffset = 1e7;

// 1.0 / n for n in [1, kMaxFitPoints], computed with the exact expression the
// Welford update used, once at load (never on the fit path).
const std::vector<double>& reciprocals() noexcept {
    static const std::vector<double> table = [] {
        std::vector<double> t(kMaxFitPoints + 1, 0.0);
        for (std::size_t n = 1; n <= kMaxFitPoints; ++n) t[n] = 1.0 / static_cast<double>(n);
        return t;
    }();
    return table;
}
[[maybe_unused]] const bool kReciprocalsReady = !reciprocals().empty();  // build the table at load

bool finite(const Vec3& p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

double dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Normalise in place; false for zero or non-finite length.
bool unit(Vec3& v) noexcept {
    const double length = std::hypot(v.x, v.y, v.z);
    if (!std::isfinite(length) || length == 0.0) return false;
    v.x /= length;
    v.y /= length;
    v.z /= length;
    return finite(v);
}

// Orient the plane so its normal points up (+z); heights are then positive above.
void face_up(Plane& plane) noexcept {
    if (plane.normal.z < 0.0) {
        plane.normal.x = -plane.normal.x;
        plane.normal.y = -plane.normal.y;
        plane.normal.z = -plane.normal.z;
        plane.d = -plane.d;
    }
}

// Signed point-to-plane distance (the plane normal is unit length).
double distance(const Vec3& p, const Plane& plane) noexcept {
    return dot(p, plane.normal) + plane.d;
}

// All state belongs to this invocation. A zero seed is also well-defined.
// (splitmix64: deterministic, so a frame always gets the same hypotheses.)
std::uint64_t random_word(std::uint64_t& state) noexcept {
    state += UINT64_C(0x9e3779b97f4a7c15);
    std::uint64_t value = state;
    value = (value ^ (value >> 30U)) * UINT64_C(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27U)) * UINT64_C(0x94d049bb133111eb);
    return value ^ (value >> 31U);
}

// Plane through three points (normal from the cross product of the two edges).
bool triple_plane(const Vec3& a, const Vec3& b, const Vec3& c,
                 Plane& plane) noexcept {
    Vec3 ab{b.x - a.x, b.y - a.y, b.z - a.z};
    Vec3 ac{c.x - a.x, c.y - a.y, c.z - a.z};
    if (!unit(ab) || !unit(ac)) return false;
    Vec3 normal{ab.y * ac.z - ab.z * ac.y,
                ab.z * ac.x - ab.x * ac.z,
                ab.x * ac.y - ab.y * ac.x};
    // Reject nearly collinear triples before amplifying their roundoff.
    if (std::hypot(normal.x, normal.y, normal.z) <= kRoundoff || !unit(normal)) {
        return false;
    }
    plane = {normal, -dot(normal, a)};
    face_up(plane);
    return std::isfinite(plane.d);
}

struct Score {
    std::size_t count = 0;
    // Scale residuals by the inlier threshold to avoid squared-distance
    // overflow. Each contribution is <= 1, count is bounded by kMaxFitPoints.
    double scaled_square_sum = 0.0;
};

// True when the unchecked passes are exact for this plane (see kBoundedCoordinate).
bool bounded_plane(bool bounded_input, const Plane& plane) noexcept {
    return bounded_input && std::abs(plane.d) <= kBoundedOffset;
}

// Inlier count and scaled squared residual sum of one plane. kChecked = false only for a bounded
// plane (bounded_plane): the residual check can never fail there. The sums are kept in locals and
// stored once (the same additions in the same order).
template <bool kChecked>
bool score_plane_pass(const Vec3* points, std::size_t count, const Plane& plane,
                     double threshold, Score& score) noexcept {
    score = {};
    double scaled_square_sum = 0.0;
    std::size_t inliers = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const double residual = distance(points[i], plane);
        if constexpr (kChecked) {
            if (!std::isfinite(residual)) return false;
        }
        if (std::abs(residual) <= threshold) {
            const double scaled = residual / threshold;
            scaled_square_sum += scaled * scaled;
            ++inliers;
        }
    }
    score.count = inliers;
    score.scaled_square_sum = scaled_square_sum;
    return true;
}
bool score_plane(const Vec3* points, std::size_t count, const Plane& plane,
                double threshold, bool bounded_input, Score& score) noexcept {
    return bounded_plane(bounded_input, plane) ? score_plane_pass<false>(points, count, plane, threshold, score)
                                               : score_plane_pass<true>(points, count, plane, threshold, score);
}

// count_inliers is the hypothesis loop's hot spot (every candidate for every
// hypothesis). Each residual is computed independently and only an integer count
// is accumulated, so vectorizing keeps every residual's operations and the exact
// count; GCC 11 (the target compiler) does not auto-vectorize at -O2, so it is
// enabled for this function only (as for the mask kernels in postprocess.cpp).
// score_plane's floating-point sum is outside this section and keeps its order.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("tree-vectorize")
#endif
// Inlier count only, with the SAME residual expression as score_plane. A
// non-finite residual makes score_plane fail; it is reported identically here.
// When `overflow_free` (every |coordinate| and |d| are small enough that no
// residual can overflow; see fit_ground), a hypothesis that can no longer reach
// `needed` inliers stops early: it is rejected either way, so selection is
// unchanged. Otherwise every point is visited so a non-finite residual is seen.
bool count_inliers(const Vec3* points, std::size_t count, const Plane& plane,
                  double threshold, std::size_t needed, bool overflow_free,
                  std::size_t& inliers) noexcept {
    std::size_t n = 0;
    bool nonfinite = false;
    constexpr std::size_t kBlock = 4096;
    for (std::size_t begin = 0; begin < count; begin += kBlock) {
        const std::size_t end = std::min(count, begin + kBlock);
        if (overflow_free) {
            // No residual can be non-finite here (see kOverflowFreeCoordinate).
            const double nx = plane.normal.x, ny = plane.normal.y, nz = plane.normal.z, d = plane.d;
            for (std::size_t i = begin; i < end; ++i) {
                const double residual = points[i].x * nx + points[i].y * ny + points[i].z * nz + d;
                n += std::abs(residual) <= threshold;
            }
        } else {
            for (std::size_t i = begin; i < end; ++i) {
                const double residual = distance(points[i], plane);
                nonfinite |= !std::isfinite(residual);
                n += std::abs(residual) <= threshold;
            }
        }
        if (overflow_free && n + (count - end) < needed) {
            inliers = n;  // below `needed`: rejected by the caller
            return !nonfinite;
        }
    }
    inliers = n;
    return !nonfinite;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

// Inlier count, fraction of all candidates, and RMS in metres.
void set_score(FitResult& result, const Score& score, double threshold) noexcept {
    result.inliers = score.count;
    result.inlier_fraction = static_cast<double>(score.count) /
                             static_cast<double>(result.candidates);
    result.inlier_rms_m = score.count == 0 ? 0.0 :
        threshold * std::sqrt(score.scaled_square_sum / static_cast<double>(score.count));
}

// Both inlier gates: absolute count and fraction.
bool enough(const FitResult& result, const FitOptions& options) noexcept {
    return result.inliers >= options.min_inliers &&
           result.inlier_fraction >= options.min_inlier_fraction;
}

struct Covariance {
    Vec3 mean;
    double matrix[3][3]{};
    std::size_t count = 0;
};

// Mean and covariance of the plane's inliers (population covariance). kChecked as score_plane_pass;
// the running mean, matrix and count live in a local Covariance (registers, not stores through the
// reference) and are copied out once. Nothing reads `covariance` when this returns false.
template <bool kChecked>
bool inlier_covariance_pass(const Vec3* points, std::size_t count, const Plane& plane,
                           double threshold, Covariance& covariance) noexcept {
    covariance = {};
    Covariance running;
    const double* reciprocal_table = reciprocals().data();
    // Welford accumulation is centered throughout; do not subtract two large
    // uncentered second moments to obtain a small spatial variance.
    for (std::size_t i = 0; i < count; ++i) {
        const double residual = distance(points[i], plane);
        if constexpr (kChecked) {
            if (!std::isfinite(residual)) return false;
        }
        if (std::abs(residual) > threshold) continue;
        ++running.count;
        const Vec3& point = points[i];
        if (running.count == 1) {
            running.mean = point;
            continue;
        }
        const double delta[3]{point.x - running.mean.x,
                              point.y - running.mean.y,
                              point.z - running.mean.z};
        const double inverse_count = reciprocal_table[running.count];
        running.mean.x += delta[0] * inverse_count;
        running.mean.y += delta[1] * inverse_count;
        running.mean.z += delta[2] * inverse_count;
        if constexpr (kChecked) {
            if (!finite(running.mean)) return false;
        }
        const double weight = static_cast<double>(running.count - 1) * inverse_count;
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = row; column < 3; ++column) {
                running.matrix[row][column] += delta[row] * (delta[column] * weight);
                if constexpr (kChecked) {
                    if (!std::isfinite(running.matrix[row][column])) return false;
                }
            }
        }
    }
    covariance = running;
    if (covariance.count < 3) return false;
    const double inverse_count = 1.0 / static_cast<double>(covariance.count);
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = row; column < 3; ++column) {
            covariance.matrix[row][column] *= inverse_count;
            covariance.matrix[column][row] = covariance.matrix[row][column];
        }
    }
    return true;
}
bool inlier_covariance(const Vec3* points, std::size_t count, const Plane& plane,
                      double threshold, bool bounded_input, Covariance& covariance) noexcept {
    return bounded_plane(bounded_input, plane)
        ? inlier_covariance_pass<false>(points, count, plane, threshold, covariance)
        : inlier_covariance_pass<true>(points, count, plane, threshold, covariance);
}

// One pass computing exactly what score_plane() and inlier_covariance() compute
// for the SAME plane (each accumulator keeps its own order and arithmetic).
// Returns false on a non-finite residual (score_plane's failure). A non-finite
// covariance accumulator does NOT stop the score: it only clears
// `covariance_ok`, which mirrors inlier_covariance()'s success, so the caller
// keeps the original order (score gates first, covariance failure after).
// kChecked as score_plane_pass: unchecked, covariance_failed stays false (no accumulator can become
// non-finite). Sums, mean, matrix and counts live in locals and are copied out once.
template <bool kChecked>
bool score_and_covariance_pass(const Vec3* points, std::size_t count, const Plane& plane, double threshold,
                              Score& score, Covariance& covariance, bool& covariance_ok) noexcept {
    score = {};
    covariance = {};
    covariance_ok = false;
    bool covariance_failed = false;
    Score running_score;
    Covariance running;
    const double* reciprocal_table = reciprocals().data();
    for (std::size_t i = 0; i < count; ++i) {
        const double residual = distance(points[i], plane);
        if constexpr (kChecked) {
            if (!std::isfinite(residual)) return false;
        }
        if (std::abs(residual) > threshold) continue;
        const double scaled = residual / threshold;
        running_score.scaled_square_sum += scaled * scaled;
        ++running_score.count;
        if constexpr (kChecked) {
            if (covariance_failed) continue;
        }
        ++running.count;
        const Vec3& point = points[i];
        if (running.count == 1) {
            running.mean = point;
            continue;
        }
        const double delta[3]{point.x - running.mean.x,
                              point.y - running.mean.y,
                              point.z - running.mean.z};
        const double inverse_count = reciprocal_table[running.count];
        running.mean.x += delta[0] * inverse_count;
        running.mean.y += delta[1] * inverse_count;
        running.mean.z += delta[2] * inverse_count;
        if constexpr (kChecked) {
            if (!finite(running.mean)) { covariance_failed = true; continue; }
        }
        const double weight = static_cast<double>(running.count - 1) * inverse_count;
        for (std::size_t row = 0; row < 3 && !covariance_failed; ++row) {
            for (std::size_t column = row; column < 3; ++column) {
                running.matrix[row][column] += delta[row] * (delta[column] * weight);
                if constexpr (kChecked) {
                    if (!std::isfinite(running.matrix[row][column])) { covariance_failed = true; break; }
                }
            }
        }
    }
    score = running_score;
    covariance = running;
    if (covariance_failed || covariance.count < 3) return true;
    const double inverse_count = 1.0 / static_cast<double>(covariance.count);
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = row; column < 3; ++column) {
            covariance.matrix[row][column] *= inverse_count;
            covariance.matrix[column][row] = covariance.matrix[row][column];
        }
    }
    covariance_ok = true;
    return true;
}
bool score_and_covariance(const Vec3* points, std::size_t count, const Plane& plane, double threshold,
                        bool bounded_input, Score& score, Covariance& covariance, bool& covariance_ok) noexcept {
    return bounded_plane(bounded_input, plane)
        ? score_and_covariance_pass<false>(points, count, plane, threshold, score, covariance, covariance_ok)
        : score_and_covariance_pass<true>(points, count, plane, threshold, score, covariance, covariance_ok);
}

// Both principal standard deviations of the inliers' x/y spread must reach the
// minimum: a thin strip of points could fit any tilted plane.
bool xy_support(const Covariance& covariance, double minimum_stddev) noexcept {
    const double xx = covariance.matrix[0][0];
    const double xy = covariance.matrix[0][1];
    const double yy = covariance.matrix[1][1];
    const double scale = std::max({std::abs(xx), std::abs(xy), std::abs(yy)});
    if (!std::isfinite(scale) || scale <= 0.0 || xx < 0.0 || yy < 0.0) return false;
    const double a = xx / scale;
    const double b = xy / scale;
    const double c = yy / scale;
    const double large = 0.5 * (a + c + std::hypot(a - c, 2.0 * b));
    if (large <= 0.0) return false;
    // determinant / large avoids subtracting two almost equal eigenvalues.
    const double small = std::max(0.0, std::fma(a, c, -b * b) / large);
    const double root_scale = std::sqrt(scale);
    return root_scale * std::sqrt(small) >= minimum_stddev &&
           root_scale * std::sqrt(large) >= minimum_stddev;
}

// Total least squares: the plane normal is the eigenvector of the smallest
// covariance eigenvalue, through the inlier mean.
bool tls_plane(const Covariance& covariance, Plane& plane) noexcept {
    double a[3][3]{};
    double eigenvectors[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double scale = 0.0;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            scale = std::max(scale, std::abs(covariance.matrix[row][column]));
        }
    }
    if (!std::isfinite(scale) || scale <= 0.0) return false;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            a[row][column] = covariance.matrix[row][column] / scale;
        }
    }

    // A symmetric 3x3 Jacobi solver: at most 24 sweeps / 72 rotations.
    // Matrix scaling keeps both very small and large supported patches stable.
    for (std::size_t sweep = 0; sweep < 24; ++sweep) {
        for (std::size_t p = 0; p < 2; ++p) {
            for (std::size_t q = p + 1; q < 3; ++q) {
                if (std::abs(a[p][q]) <= kRoundoff) continue;
                const double angle = 0.5 * std::atan2(2.0 * a[p][q], a[q][q] - a[p][p]);
                const double cosine = std::cos(angle);
                const double sine = std::sin(angle);
                const double pp = a[p][p];
                const double qq = a[q][q];
                const double pq = a[p][q];
                a[p][p] = cosine * cosine * pp - 2.0 * cosine * sine * pq + sine * sine * qq;
                a[q][q] = sine * sine * pp + 2.0 * cosine * sine * pq + cosine * cosine * qq;
                a[p][q] = a[q][p] = 0.0;
                for (std::size_t k = 0; k < 3; ++k) {
                    if (k != p && k != q) {
                        const double kp = a[k][p];
                        const double kq = a[k][q];
                        a[k][p] = a[p][k] = cosine * kp - sine * kq;
                        a[k][q] = a[q][k] = sine * kp + cosine * kq;
                    }
                    const double vp = eigenvectors[k][p];
                    const double vq = eigenvectors[k][q];
                    eigenvectors[k][p] = cosine * vp - sine * vq;
                    eigenvectors[k][q] = sine * vp + cosine * vq;
                }
            }
        }
    }
    if (std::max({std::abs(a[0][1]), std::abs(a[0][2]), std::abs(a[1][2])}) >
        4.0 * kRoundoff) return false;
    std::size_t smallest = 0;
    for (std::size_t i = 0; i < 3; ++i) {
        if (!std::isfinite(a[i][i]) || a[i][i] < -kRoundoff) return false;
        if (a[i][i] < a[smallest][smallest]) smallest = i;
    }
    Vec3 normal{eigenvectors[0][smallest], eigenvectors[1][smallest], eigenvectors[2][smallest]};
    if (!unit(normal)) return false;
    plane = {normal, -dot(normal, covariance.mean)};
    face_up(plane);
    return std::isfinite(plane.d);
}

// The fitted normal must point up enough and stay within the allowed angle of
// the prior normal (a wall or a steep slope fitted as ground is rejected).
bool admissible_normal(const Plane& plane, const Plane& prior,
                      const FitOptions& options) noexcept {
    // The tolerance only absorbs normalized-vector/trigonometric roundoff.
    return finite(plane.normal) && plane.normal.z > 0.0 &&
           plane.normal.z + kRoundoff >= options.min_up_dot &&
           dot(plane.normal, prior.normal) + kRoundoff >= std::cos(options.max_prior_angle_rad);
}

}  // namespace

bool valid_fit_options(const FitOptions& options) noexcept {
    return std::isfinite(options.inlier_distance_m) && options.inlier_distance_m > 0.0 &&
           std::isfinite(options.min_inlier_fraction) && options.min_inlier_fraction > 0.0 &&
           options.min_inlier_fraction <= 1.0 &&
           std::isfinite(options.max_inlier_rms_m) && options.max_inlier_rms_m >= 0.0 &&
           std::isfinite(options.min_xy_stddev_m) && options.min_xy_stddev_m > 0.0 &&
           std::isfinite(options.min_up_dot) && options.min_up_dot >= 0.0 && options.min_up_dot <= 1.0 &&
           std::isfinite(options.max_prior_angle_rad) && options.max_prior_angle_rad >= 0.0 &&
           options.max_prior_angle_rad <= kPi && options.min_inliers >= 3 &&
           options.min_inliers <= kMaxFitPoints && options.hypotheses >= 1 &&
           options.hypotheses <= kMaxHypotheses;
}

#ifdef PERCEPTION_FIT_TEST_HOOKS
bool g_fit_force_checked = false;
#endif
#ifdef PERCEPTION_FIT_DIAGNOSTICS
FitDiagnostics g_fit_diagnostics;
namespace {
std::int64_t diagnostic_clock_ns() noexcept {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
}  // namespace
#define FIT_DIAGNOSTIC(statement) statement
#else
#define FIT_DIAGNOSTIC(statement)
#endif

FitResult fit_ground(const Vec3* candidates, std::size_t count,
                    const Plane& prior, const FitOptions& options) noexcept {
    FIT_DIAGNOSTIC(g_fit_diagnostics = FitDiagnostics{}; g_fit_diagnostics.entry_ns = diagnostic_clock_ns();)
    FitResult result;
    result.candidates = count;
    const double prior_length = std::hypot(prior.normal.x, prior.normal.y, prior.normal.z);
    if (!valid_fit_options(options) || count > kMaxFitPoints ||
        (count != 0 && candidates == nullptr) || !finite(prior.normal) ||
        !std::isfinite(prior.d) || !std::isfinite(prior_length) ||
        std::abs(prior_length - 1.0) > 1e-9 || prior.normal.z <= 0.0) {
        return result;
    }
    // Every candidate must be finite; the largest coordinate decides whether the
    // early-reject inlier count is overflow-safe.
    double max_abs_coordinate = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        if (!finite(candidates[i])) return result;
        max_abs_coordinate = std::max({max_abs_coordinate, std::abs(candidates[i].x), std::abs(candidates[i].y),
                                       std::abs(candidates[i].z)});
    }
    if (count < options.min_inliers) {
        result.status = FitStatus::InsufficientCandidates;
        return result;
    }
    // Bounded input: the scoring and covariance passes may skip their finiteness checks (see
    // kBoundedCoordinate); with larger coordinates every pass keeps them.
#ifdef PERCEPTION_FIT_TEST_HOOKS
    const bool bounded_input = !g_fit_force_checked && max_abs_coordinate <= kBoundedCoordinate;
#else
    const bool bounded_input = max_abs_coordinate <= kBoundedCoordinate;
#endif
    // The prior plane is scored first (hypothesis 0); a random hypothesis replaces
    // it only with more inliers, or as many with a smaller residual.
    Plane best{{prior.normal.x / prior_length, prior.normal.y / prior_length,
                prior.normal.z / prior_length}, prior.d / prior_length};
    const Plane normalized_prior = best;
    Score best_score;
    if (!score_plane(candidates, count, best, options.inlier_distance_m, bounded_input, best_score)) return result;
    FIT_DIAGNOSTIC(g_fit_diagnostics.prior_scored_ns = diagnostic_clock_ns();)
    result.hypotheses_tested = 1;
    std::uint64_t state = options.seed;
    // The budget includes the prior. Duplicate/degenerate draws are skipped
    // without retries, so work remains bounded even for identical inputs.
    for (std::size_t attempt = 1; attempt < options.hypotheses; ++attempt) {
        const std::size_t a = random_word(state) % count;
        const std::size_t b = random_word(state) % count;
        const std::size_t c = random_word(state) % count;
        if (a == b || a == c || b == c) continue;
        Plane hypothesis;
        if (!triple_plane(candidates[a], candidates[b], candidates[c], hypothesis)) continue;
        // A hypothesis with fewer inliers than the best can never be selected, so
        // its residual sum is not needed (selection identical to scoring all).
        std::size_t inliers = 0;
        const bool overflow_free = max_abs_coordinate < kOverflowFreeCoordinate && std::abs(hypothesis.d) < kOverflowFreeCoordinate;
        if (!count_inliers(candidates, count, hypothesis, options.inlier_distance_m, best_score.count,
                          overflow_free, inliers)) return result;
        if (inliers < best_score.count) {
            ++result.hypotheses_tested;
            continue;
        }
        Score score;
        FIT_DIAGNOSTIC(++g_fit_diagnostics.full_scores;)
        if (!score_plane(candidates, count, hypothesis, options.inlier_distance_m, bounded_input, score)) return result;
        ++result.hypotheses_tested;
        if (score.count > best_score.count ||
            (score.count == best_score.count && score.scaled_square_sum < best_score.scaled_square_sum)) {
            best = hypothesis;
            best_score = score;
        }
    }
    FIT_DIAGNOSTIC(g_fit_diagnostics.hypotheses_done_ns = diagnostic_clock_ns();)
    set_score(result, best_score, options.inlier_distance_m);
    if (!enough(result, options)) {
        result.status = FitStatus::InsufficientInliers;
        return result;
    }

    // Each round: covariance of the current plane -> TLS plane -> score of the
    // new plane. The score of a plane and the next round's (or the final)
    // covariance of the SAME plane come from one merged pass; results and the
    // order of every check/return are unchanged.
    Covariance covariance;
    if (!inlier_covariance(candidates, count, best, options.inlier_distance_m, bounded_input, covariance)) return result;
#ifdef PERCEPTION_FIT_DIAGNOSTICS
    // Development bench only (the product never defines the macro): another round count, read
    // from PERCEPTION_FIT_ROUNDS_OVERRIDE, to evaluate it against the approximation rule.
    static const std::size_t refinement_rounds = [] {
        const char* value = std::getenv("PERCEPTION_FIT_ROUNDS_OVERRIDE");
        return value ? static_cast<std::size_t>(std::strtoul(value, nullptr, 10)) : kRefinementRounds;
    }();
#else
    constexpr std::size_t refinement_rounds = kRefinementRounds;
#endif
    for (std::size_t round = 0; round < refinement_rounds; ++round) {
        if (!xy_support(covariance, options.min_xy_stddev_m) || !tls_plane(covariance, best)) {
            result.status = FitStatus::DegenerateSupport;
            return result;
        }
        bool covariance_ok = false;
        FIT_DIAGNOSTIC(++g_fit_diagnostics.refine_rounds;)
        if (!score_and_covariance(candidates, count, best, options.inlier_distance_m, bounded_input, best_score,
                                covariance, covariance_ok)) return result;
        set_score(result, best_score, options.inlier_distance_m);
        if (!enough(result, options)) {
            result.status = FitStatus::InsufficientInliers;
            return result;
        }
        if (!covariance_ok) return result;  // inlier_covariance would fail on this plane
    }
    // Gate the exact final membership, not a previous hypothesis's inliers.
    const Covariance& final_covariance = covariance;
    if (!xy_support(final_covariance, options.min_xy_stddev_m)) {
        result.status = FitStatus::DegenerateSupport;
        return result;
    }
    if (!admissible_normal(best, normalized_prior, options)) {
        result.status = FitStatus::NormalRejected;
        return result;
    }
    if (result.inlier_rms_m > options.max_inlier_rms_m) {
        result.status = FitStatus::ResidualExceeded;
        return result;
    }
    result.plane = best;
    result.status = FitStatus::Pass;
    return result;
}

}  // namespace perception::geometry
