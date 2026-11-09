/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: native_geometry.cpp
 * Brief: Per-frame native depth geometry: raw sample ledger, ground fit, pixel classification.
 *
 * Description:
 * What problem this solves. Implements native_geometry.hpp, the first stage of the Profile fast
 * line (19 section 3.1 precompute, 3.2 per frame). Setup (constructor) rotates every optical ray
 * into base_link once and marks the prior-plane ROI. process() then, per 640x400 depth frame:
 *   1. checks the frame identity, order and camera profile (the rays belong to one profile);
 *   2. scans every pixel: base_link point, azimuth bin, horizontal range; counts raw samples
 *      per bin and collects near-field points close to the prior plane as fit candidates;
 *   3. fits the ground plane (ground_fit);
 *   4. classifies every pixel against the fitted plane by height: Ground (|h| <= h_tol),
 *      Obstacle (h_tol < h <= z_pass), Overhead (> z_pass), BelowPlane (< -h_tol), and notes
 *      pierce candidates (a return beyond the expected ground intersection); the pixel records
 *      are completed here (2026-10-14: on the fast path the scan no longer writes them, so each
 *      is written once, in this pass);
 *   5. records per bin the nearest positive obstacle and the highest obstacle point within w_h
 *      behind it (the visible-height input of h_block), reading only the Obstacle pixels the
 *      classification noted (2026-10-06; before, every pixel record was read again).
 * General path (lateral offset; the vehicle mount), 2026-10-24 G-P1 phase 1: the scan's bin search
 * starts from the previous pixel's bin instead of an atan2 estimate (two divisions per pixel).
 * Both paths: when the setup proves every point the scan can form is bounded (scan_bounded_), the
 * scan's finiteness checks, which then cannot fire, are skipped. Same values, bit for bit
 * (tests/test_native_geometry_pixel_record.cpp). The classification pass reads each pixel's class
 * back from its record for the class count (2026-10-25): after the scan changes GCC 11 held the
 * class in a register instead, the pass lost its one load per pixel at the record's fixed 48-byte
 * stride and took about twice the L1 data refills (PMU on .26: 122k against 58k per frame,
 * 3.5 against 2.5 ms; notes/PROFILE_GP1_PHASE1_20261024.md section 9).
 * General-path scan, 2026-10-25 (G-P1 phase 2, same note section 10): the bin lookup first tries the
 * previous pixel's bin and two neighbours inline (bin_near_hint; bin_of only when they do not decide),
 * the range is an inline copy of glibc 2.35's hypot (scan_hypot; std::hypot on any other libm), the
 * counters, option values and table pointers are locals, and each pixel record is prefetched for write
 * 16 pixels ahead. Same values, bit for bit (the scan sweeps in tests/test_native_geometry_pixel_record.cpp).
 * The classification pass reads its arrays, binding and option values through locals too (same day).
 * The stream is checked again after the heavy parts: a result for a retired stream is never
 * reported as current evidence.
 */
#include "modules/native_geometry.hpp"
#include <time.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace perception::geometry {
namespace {
// Stage timestamps for the timing ledger (CLOCK_MONOTONIC ns, -1 on error).
std::int64_t stage_clock_ns() noexcept {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
}  // namespace

namespace {
constexpr double pi = 3.1415926535897932384626433832795;
static_assert(kGeometryPixels <= std::numeric_limits<std::uint32_t>::max(),
              "one raw bin count must hold a complete native frame");
bool finite(Vec3 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
// Bounded scan (G-P1 phase 1): a depth code is at most 65535, so z <= 65535 * unit and every scan
// point has |p_i| <= max|ray| * z + |t_i|. With that bound and |prior d| within kBoundedPoint,
// |range| <= 1.5e9 and |prior height| <= 4e9 (a unit prior normal, on both paths: the fast path's
// tables are the same factors), all finite.
constexpr double kBoundedPoint = 1e9;  // metres
double dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 position(Vec3 ray, double z, Vec3 t) { return {ray.x*z+t.x, ray.y*z+t.y, ray.z*z+t.z}; }
Vec3 rotate(const std::array<double, 9>& r, Vec3 p) {
    return {r[0]*p.x+r[1]*p.y+r[2]*p.z,
            r[3]*p.x+r[4]*p.y+r[5]*p.z,
            r[6]*p.x+r[7]*p.y+r[8]*p.z};
}
// Bitwise equality: the camera profile must be exactly the bound one.
bool same_float(float a, float b) {
    return std::memcmp(&a, &b, sizeof(float)) == 0;
}
}  // namespace

bool valid_projection_binding(const ProjectionBinding& b) noexcept {
    const auto& i = b.intrinsics;
    if (i.width != kNativeWidth || i.height != kNativeHeight || !std::isfinite(i.fx) || i.fx <= 0 ||
        !std::isfinite(i.fy) || i.fy <= 0 || !std::isfinite(i.cx) || !std::isfinite(i.cy) ||
        !b.distortion.reported || !std::isfinite(b.unit_m) || b.unit_m <= 0 || !finite(b.t_base_camera)) return false;
    for (const auto v : b.distortion.coefficients) if (!std::isfinite(v)) return false;
    for (const auto v : b.R_base_camera) if (!std::isfinite(v)) return false;
    const auto& r = b.R_base_camera;
    // SE(3) validation tolerance is an implementation numerical check, not
    // an external calibration accuracy allowance.
    for (std::size_t a = 0; a < 3; ++a) {
        for (std::size_t c = 0; c < 3; ++c) {
            double d = 0;
            for (std::size_t k = 0; k < 3; ++k) d += r[3*a+k]*r[3*c+k];
            if (std::abs(d-(a == c ? 1.0 : 0.0)) > 1e-9) return false;
        }
    }
    const double det = r[0]*(r[4]*r[8]-r[5]*r[7]) - r[1]*(r[3]*r[8]-r[5]*r[6]) +
                       r[2]*(r[3]*r[7]-r[4]*r[6]);
    return std::abs(det-1.0) <= 1e-9;
}
// Options sanity: a unit prior normal pointing up, finite values, range within
// 6 m, fit band inside the range, h_tol < z_pass, bins covering at most [-pi, pi].
bool valid_geometry_options(const GeometryOptions& o) noexcept {
    if (!valid_fit_options(o.fit) || !finite(o.prior.normal) || !std::isfinite(o.prior.d) ||
        std::abs(dot(o.prior.normal, o.prior.normal)-1.0) > 1e-9 || o.prior.normal.z <= 0) return false;
    const double values[] = {o.fit_near_min_m, o.fit_near_max_m, o.fit_prior_band_m, o.range_max_m,
        o.blind_near_lo_m, o.h_tol_m, o.z_pass_m, o.eps_neg, o.w_h_m, o.angle_min_rad, o.angle_step_rad};
    for (const auto v : values) if (!std::isfinite(v)) return false;
    if (o.range_max_m <= 0 || o.range_max_m > 6.0 || o.blind_near_lo_m < 0 || o.blind_near_lo_m >= o.range_max_m ||
        o.fit_near_min_m < 0 || o.fit_near_max_m <= o.fit_near_min_m || o.fit_near_max_m > o.range_max_m ||
        o.fit_prior_band_m <= 0 || o.h_tol_m <= 0 || o.z_pass_m <= o.h_tol_m || o.eps_neg < 0 ||
        !std::isfinite(1.0+o.eps_neg) || o.w_h_m < 0 || o.n_bins == 0 || o.n_bins > kGeometryMaxBins ||
        o.angle_step_rad <= 0) return false;
    const auto lo = o.angle_min_rad-o.angle_step_rad/2;
    const auto hi = o.angle_min_rad+(static_cast<double>(o.n_bins)-0.5)*o.angle_step_rad;
    return lo >= -pi && hi <= pi && hi > lo;
}

namespace {
// Where a ray from the camera centre meets a plane, in front of the camera
// (z > 0); false for parallel or behind.
bool intersection(const Plane& plane, Vec3 ray, Vec3 translation, Vec3& p, double& z) {
    const double denominator = dot(plane.normal, ray);
    if (!std::isfinite(denominator) || denominator == 0) return false;
    z = -(dot(plane.normal, translation)+plane.d)/denominator;
    if (!std::isfinite(z) || z <= 0) return false;
    p = position(ray, z, translation);
    return finite(p);
}
void nearest(std::optional<double>& result, double value) {
    if (!result || value < *result) result = value;
}
void highest(std::optional<double>& result, double value) {
    if (!result || value > *result) result = value;
}
}  // namespace

NativeGeometry::NativeGeometry(const ProjectionBinding& binding, const Vec3* optical_rays,
                               std::size_t ray_count, const GeometryOptions& options)
    : binding_(binding), options_(options) {
    if (!valid_projection_binding(binding) || !valid_geometry_options(options) || optical_rays == nullptr || ray_count != kGeometryPixels)
        throw std::invalid_argument("native geometry requires explicit valid 640x400 projection and fit options");
    // Bin boundary directions for the fast azimuth search (bin_of); only for a
    // span below pi, where the cross-product sign test is unambiguous.
    {
        const double lo = options.angle_min_rad - options.angle_step_rad / 2;
        const double span = static_cast<double>(options.n_bins) * options.angle_step_rad;
        if (span < 3.0) {
            boundary_cos_.resize(options.n_bins + 1);
            boundary_sin_.resize(options.n_bins + 1);
            for (std::size_t k = 0; k <= options.n_bins; ++k) {
                const double theta = lo + static_cast<double>(k) * options.angle_step_rad;
                boundary_cos_[k] = std::cos(theta);
                boundary_sin_[k] = std::sin(theta);
            }
        }
    }
    rays_.resize(kGeometryPixels);
    codes_.resize(kGeometryPixels);
    candidates_.resize(kGeometryPixels);
    pixels_.resize(kGeometryPixels);
    roi_.resize(kGeometryPixels);
    bins_.resize(options.n_bins);
    raw_sample_counts_.resize(options.n_bins);
    obstacle_pixels_.reserve(kGeometryPixels);  // never reallocates on the per-frame path
    // Rotate every ray into base_link once, and mark the pixels whose ray meets
    // the prior plane within range (the ROI; none at all fails PSC-7).
    std::size_t support = 0;
    for (std::size_t k = 0; k < kGeometryPixels; ++k) {
        const Vec3 ray = optical_rays[k];
        if (!finite(ray) || ray.z != 1.0)
            throw std::invalid_argument("optical rays must be finite with optical z exactly one");
        rays_[k] = rotate(binding.R_base_camera, ray);
        if (!finite(rays_[k])) throw std::invalid_argument("projected ray is nonfinite");
        Vec3 expected;
        double z = 0;
        if (intersection(options.prior, rays_[k], binding.t_base_camera, expected, z) &&
            std::hypot(expected.x, expected.y) <= options.range_max_m) {
            roi_[k] = 1;
            ++support;
        }
    }
    if (support == 0) throw std::invalid_argument("prior plane has no positive in-range ray intersections");
    roi_pixels_ = support;

    // Depth-independent azimuth (B6-1). p = (ray.x*z + t.x, ray.y*z + t.y, ...).
    // With t.x = t.y = 0 and z > 0 the azimuth of p is the azimuth of the ray,
    // so the bin is a property of the PIXEL, not of the frame; the radial factor
    // and the prior-plane dot product factor the same way:
    //   range        = hypot(ray.x*z, ray.y*z) = z * hypot(ray.x, ray.y)
    //   prior height = z * (n . ray) + (n.z * t.z + d)
    // These are algebraic identities in exact arithmetic and differ from the
    // general path only by rounding; the measured difference over 60 real frames
    // and 5.7 M boundary-forced pixels is <= 3e-14 m with ZERO changed
    // classifications, bins, counts or bin evidence, against a measured decision
    // margin of 3.7e-8 m (evidence/segment2/b61_proto).
    fast_azimuth_ = binding.t_base_camera.x == 0.0 && binding.t_base_camera.y == 0.0;
    if (fast_azimuth_) {
        pixel_bin_.resize(kGeometryPixels);
        pixel_rho_.resize(kGeometryPixels);
        pixel_prior_.resize(kGeometryPixels);
        prior_const_ = options.prior.normal.z * binding.t_base_camera.z + options.prior.d;
        for (std::size_t k = 0; k < kGeometryPixels; ++k) {
            const Vec3 r = rays_[k];
            pixel_bin_[k] = bin_of(r);          // any positive depth gives this azimuth
            pixel_rho_[k] = std::hypot(r.x, r.y);
            pixel_prior_[k] = dot(options.prior.normal, r);
            if (!std::isfinite(pixel_rho_[k]) || !std::isfinite(pixel_prior_[k])) {
                fast_azimuth_ = false;         // fall back to the general path
                break;
            }
        }
        if (!std::isfinite(prior_const_)) fast_azimuth_ = false;
    }
    double max_ray = 0;
    for (const Vec3& r : rays_) max_ray = std::max({max_ray, std::abs(r.x), std::abs(r.y), std::abs(r.z)});
    const Vec3& t = binding.t_base_camera;
    const double point_bound = max_ray * (65535.0 * static_cast<double>(binding.unit_m)) +
                               std::max({std::abs(t.x), std::abs(t.y), std::abs(t.z)});
    scan_bounded_ = std::isfinite(point_bound) && point_bound <= kBoundedPoint &&
                    std::abs(options.prior.d) <= kBoundedPoint;
}


namespace {
// G-P1 phase 2 (2026-10-25): the general-path scan's bin lookup tries the previous pixel's bin and at
// most two neighbours inline before the full search (bin_of). Returns a bin only when that bin's two
// boundaries bracket p beyond the band, on an interior bin (1 .. n-2): the same condition under which
// bin_of's own walk returns a bin, and it implies the point lies strictly inside the angular domain,
// so bin_of's domain checks pass and its walk from the same start returns the same bin. -2: undecided
// here, bin_of decides. kHintBand must equal bin_of's kBinCrossBand.
// Plain inline on purpose, like scan_hypot: with [[gnu::always_inline]] on both, GCC 11 kept four loop
// invariants of scan() on the stack and the scan took 3.89 instead of 3.61 ms (1.8, same values).
constexpr double kHintBand = 1e-12;
inline int bin_near_hint(const double* c, const double* s, std::size_t n, double px, double py, int hint) noexcept {
    if (hint < 1) return -2;
    std::size_t k = static_cast<std::size_t>(hint);
    const double band = kHintBand * (std::abs(px) + std::abs(py));
    for (int tries = 0; tries < 3 && k >= 1 && k + 2 <= n; ++tries) {
        const double below = c[k] * py - s[k] * px;
        const double above = c[k + 1] * py - s[k + 1] * px;
        if (!(std::abs(below) > band) || !(std::abs(above) > band)) return -2;
        if (below > 0 && above < 0) return static_cast<int>(k);
        if (below <= 0) --k; else ++k;
    }
    return -2;
}
// std::hypot without the call, for the scan (G-P1 phase 2): the operations of glibc 2.35's __hypot
// (sysdeps/ieee754/dbl-64/e_hypot.c, FMA kernel; disassembled from the .26 libm.so.6) on its common
// path -- finite, max <= 0x1p511, min >= 0x1p-459. Anything else calls std::hypot itself. Only
// compiled for glibc 2.35 on aarch64 (kInlineHypot); elsewhere scan_hypot is std::hypot.
#if defined(__aarch64__) && defined(__GLIBC__) && __GLIBC__ == 2 && __GLIBC_MINOR__ == 35 && defined(__FP_FAST_FMA)
constexpr bool kInlineHypot = true;
#else
constexpr bool kInlineHypot = false;
#endif
inline double scan_hypot(double x, double y) noexcept {
    if constexpr (kInlineHypot) {
        const double fx = std::abs(x), fy = std::abs(y);
        const double ax = fx < fy ? fy : fx, ay = fx < fy ? fx : fy;  // fmaxnm / fminnm
        if (ax <= 0x1p511 && ay >= 0x1p-459) {
            if (ax * 0x1p-54 >= ay) return ax + ay;
            const double t1 = ay + ay;
            if (ax <= t1) {
                const double t2 = ax - ay;
                return std::sqrt(std::fma(t1, ax, t2 * t2));
            }
            return std::sqrt(std::fma(ax, ax, ay * ay));
        }
    }
    return std::hypot(x, y);
}
}  // namespace

// Classified evidence depends on the fitted plane; the raw ledger does not.
void NativeGeometry::clear_classified_evidence() noexcept {
    std::fill(pixels_.begin(), pixels_.end(), PixelEvidence{});
    std::fill(bins_.begin(), bins_.end(), BinEvidence{});
    result_.classes.fill(0);
    result_.nearest_ground_m.reset();
}
void NativeGeometry::clear_evidence() noexcept {
    clear_classified_evidence();
    std::fill(raw_sample_counts_.begin(), raw_sample_counts_.end(), 0);
    result_.raw_sample_evidence_valid = false;
}
// A failed fit keeps the raw ledger (the withdrawal route and S still use it);
// every other failure clears everything.
void NativeGeometry::fail(GeometryStatus status) noexcept {
    if (status == GeometryStatus::FitFailed && result_.raw_sample_evidence_valid)
        clear_classified_evidence();
    else
        clear_evidence();
    result_.fit.plane.reset();
    result_.status = status;
}
// Azimuth bin without atan2 on the common path, provably identical to
// reference_bin_of(): cross(b_k, p) = |p| sin(theta - theta_k) has the sign of
// theta - theta_k inside a wedge narrower than pi. Every sign used is required
// to exceed kBinCrossBand * (|x| + |y|) >= 1e-12 rad * |p|, far above the
// roundoff of atan2, of the boundary vectors and of the reference floor
// formula's own transition (~1e-15 rad); otherwise the reference is used.
// A hint in [0, n) replaces only the starting guess: the walk still returns only a bin whose
// two boundaries bracket p beyond the band, which is the reference bin whatever the start.
int NativeGeometry::bin_of(Vec3 p, int hint) const noexcept {
    // No directional evidence can be assigned to an undefined azimuth.
    if (p.x == 0 && p.y == 0) return -1;
    if (!boundary_cos_.empty()) {
        constexpr double kBinCrossBand = 1e-12;
        const double band = kBinCrossBand * (std::abs(p.x) + std::abs(p.y));
        const std::size_t n = options_.n_bins;
        const double first = boundary_cos_[0] * p.y - boundary_sin_[0] * p.x;
        const double last = boundary_cos_[n] * p.y - boundary_sin_[n] * p.x;
        if (std::isfinite(band) && std::abs(first) > band && std::abs(last) > band) {
            if (first < 0 || last > 0) return -1;  // outside [lo, hi)
            // Guess from a cheap atan2 approximation (|error| < 0.005 rad), then
            // walk to the bin whose boundaries bracket p: cross(b_k) > 0 and
            // cross(b_{k+1}) < 0, each beyond the band. The guess only chooses
            // where to start; the bracket decides the bin.
            std::size_t k = 0;
            if (hint >= 0 && static_cast<std::size_t>(hint) < n) {
                k = static_cast<std::size_t>(hint);
            } else {
                const double ax = std::abs(p.x), ay = std::abs(p.y);
                double t;
                if (ax >= ay) {
                    const double a = ay / ax;
                    t = 0.7853981633974483 * a + 0.273 * a * (1.0 - a);
                } else {
                    const double a = ax / ay;
                    t = 1.5707963267948966 - (0.7853981633974483 * a + 0.273 * a * (1.0 - a));
                }
                if (p.x < 0) t = 3.141592653589793 - t;
                if (p.y < 0) t = -t;
                const double lo = options_.angle_min_rad - options_.angle_step_rad / 2;
                double guess = std::floor((t - lo) / options_.angle_step_rad);
                if (!(guess >= 0)) guess = 0;
                if (guess > static_cast<double>(n - 1)) guess = static_cast<double>(n - 1);
                k = static_cast<std::size_t>(guess);
            }
            for (std::size_t steps = 0; steps <= n; ++steps) {
                const double below = boundary_cos_[k] * p.y - boundary_sin_[k] * p.x;
                const double above = boundary_cos_[k + 1] * p.y - boundary_sin_[k + 1] * p.x;
                if (std::abs(below) <= band || std::abs(above) <= band) break;  // reference decides
                if (below > 0 && above < 0) return static_cast<int>(k);
                if (below <= 0) {
                    if (k == 0) break;
                    --k;
                } else {
                    if (k + 1 >= n) break;
                    ++k;
                }
            }
        }
    }
    return reference_bin_of(p);
}
// Reference definition: floor((atan2(y, x) - lo) / step) within [lo, hi).
int NativeGeometry::reference_bin_of(Vec3 p) const noexcept {
    if (p.x == 0 && p.y == 0) return -1;
    const double theta = std::atan2(p.y, p.x);
    const double lo = options_.angle_min_rad-options_.angle_step_rad/2;
    const double hi = options_.angle_min_rad+(static_cast<double>(options_.n_bins)-0.5)*options_.angle_step_rad;
    if (!std::isfinite(theta) || theta < lo || theta >= hi) return -1;
    const double index = std::floor((theta-lo)/options_.angle_step_rad);
    if (index < 0 || index >= static_cast<double>(options_.n_bins)) return -1;
    return static_cast<int>(index);
}
// The frame's intrinsics, unit and distortion must equal the bound profile
// exactly; otherwise the precomputed rays would be wrong for this frame.
bool NativeGeometry::profile_matches(const native_depth::Metadata& m) const noexcept {
    const auto& a = m.intrinsics;
    const auto& b = binding_.intrinsics;
    if (a.width != b.width || a.height != b.height || !same_float(a.fx,b.fx) || !same_float(a.fy,b.fy) ||
        !same_float(a.cx,b.cx) || !same_float(a.cy,b.cy) || !same_float(m.unit_m,binding_.unit_m) ||
        m.distortion.reported != binding_.distortion.reported || m.distortion.model != binding_.distortion.model) return false;
    for (std::size_t k = 0; k < m.distortion.coefficients.size(); ++k)
        if (!same_float(m.distortion.coefficients[k], binding_.distortion.coefficients[k])) return false;
    return true;
}

// The scan pass of process() (step 2 of the file header), in its own function since 2026-10-24
// (G-P1 phase 1; the scan itself measured 5.43 -> 5.31 ms on .26 against the inline version).
// False after a numerical failure (already reported through fail()).
[[gnu::noinline]] bool NativeGeometry::scan(const native_depth::Frame& frame, std::size_t& candidate_count) noexcept {
    const std::uint16_t* const code = frame.pixels();
    std::memcpy(codes_.data(), code, kGeometryPixels * sizeof(std::uint16_t));
    candidate_count = 0;
    // G-P1 phase 2 (2026-10-25): the per-pixel counters, the option values and the table pointers are
    // locals; members, the stores of the loop could alias them, so GCC reloaded and stored them for
    // every pixel. Same values, same expressions.
    std::size_t visited = 0, roi_sum = 0, invalid_roi = 0, valid = 0, count = 0;
    const auto flush = [&] {
        result_.visited_pixels += visited;
        result_.roi_pixels += roi_sum;
        result_.invalid_roi_pixels += invalid_roi;
        result_.valid_depth_pixels += valid;
        candidate_count = count;
    };
    const double unit = static_cast<double>(binding_.unit_m);
    const Vec3 t = binding_.t_base_camera;
    const Vec3 normal = options_.prior.normal;
    const double prior_d = options_.prior.d, prior_band = options_.fit_prior_band_m;
    const double near_lo = options_.blind_near_lo_m, range_max = options_.range_max_m;
    const double fit_lo = options_.fit_near_min_m, fit_hi = options_.fit_near_max_m;
    const bool fast = fast_azimuth_, bounded = scan_bounded_;
    const Vec3* const rays = rays_.data();
    const std::uint8_t* const roi = roi_.data();
    PixelEvidence* const records = pixels_.data();
    std::uint32_t* const raw = raw_sample_counts_.data();
    Vec3* const candidates = candidates_.data();
    const double* const rho = pixel_rho_.data();
    const double* const prior = pixel_prior_.data();
    const int* const pbin = pixel_bin_.data();
    const double prior_c = prior_const_;
    const double* const bc = boundary_cos_.empty() ? nullptr : boundary_cos_.data();
    const double* const bs = boundary_sin_.empty() ? nullptr : boundary_sin_.data();
    const std::size_t n_bins = options_.n_bins;
    int previous_bin = -1;  // general path: the last valid pixel's bin, the next search's start
    for (std::size_t k = 0; k < kGeometryPixels; ++k) {
        ++visited;
        roi_sum += roi[k];
        if (code[k] == 0) {
            invalid_roi += roi[k];
            continue;
        }
        ++valid;
        const double z = static_cast<double>(code[k])*unit;
        double range = 0, prior_height = 0;
        int bin = -1;
        Vec3 p{};
        if (fast) {
            p = {rays[k].x * z, rays[k].y * z, rays[k].z * z + t.z};
            range = z * rho[k];
            prior_height = z * prior[k] + prior_c;
            bin = pbin[k];
            if (!bounded && (!finite(p) || !std::isfinite(range) || !std::isfinite(prior_height))) {
                flush(); fail(GeometryStatus::NumericalFailure); return false;
            }
        } else {
            p = position(rays[k], z, t);
            if (!bounded && !finite(p)) { flush(); fail(GeometryStatus::NumericalFailure); return false; }
            range = scan_hypot(p.x,p.y);
            prior_height = dot(normal,p)+prior_d;
            if (!bounded && (!std::isfinite(range) || !std::isfinite(prior_height))) {
                flush(); fail(GeometryStatus::NumericalFailure); return false;
            }
            bin = bc == nullptr ? -2 : bin_near_hint(bc, bs, n_bins, p.x, p.y, previous_bin);
            if (bin == -2) bin = bin_of(p, previous_bin);
            previous_bin = bin;
            // Cache bin/range for the later normal pass, in a cleared record (its
            // pierce fields stay clear unless that pass finds a candidate). Inside
            // this existing branch, not under a new one: GCC contracts the scan's
            // products into FMAs according to the surrounding control flow.
            // Prefetch for write the record 16 pixels ahead (G-P1 phase 2): without it each record
            // line was a write-allocate miss the faster loop waited for (PMU on 1.8: 144k against 56k
            // L1 data refills per frame, scan 4.18 -> 3.61 ms). A hint only, never a load or store;
            // the address is formed as an integer, so it may lie past the end of the array.
            __builtin_prefetch(reinterpret_cast<const void*>(reinterpret_cast<std::uintptr_t>(records + k) +
                                                             16 * sizeof(PixelEvidence)), 1, 3);
            records[k] = PixelEvidence{};
            records[k].bin = bin;
            records[k].range_m = range;
        }
        // This ledger depends only on the actual valid return, projection and
        // the unchanged angular/range domain. Fit ROI and height never select
        // which raw samples exist.
        if (bin >= 0 && range >= near_lo && range <= range_max)
            ++raw[static_cast<std::size_t>(bin)];
        // Fit candidates: near field only, and close to the prior plane, so the
        // fit sees ground and not walls or furniture.
        if (range >= fit_lo && range <= fit_hi && std::abs(prior_height) <= prior_band)
            candidates[count++] = p;
    }
    flush();
    return true;
}

int NativeGeometry::scan_azimuth_bin(Vec3 p, int hint, bool* inline_decided) const noexcept {
    const int bin = boundary_cos_.empty() ? -2 :
        bin_near_hint(boundary_cos_.data(), boundary_sin_.data(), options_.n_bins, p.x, p.y, hint);
    if (inline_decided != nullptr) *inline_decided = bin != -2;
    return bin != -2 ? bin : bin_of(p, hint);
}
double NativeGeometry::scan_range(double x, double y) noexcept { return scan_hypot(x, y); }
bool NativeGeometry::scan_range_inline() noexcept { return kInlineHypot; }

const GeometryResult& NativeGeometry::process(const native_depth::Frame& frame,
                                             const native_depth::Mailbox& mailbox) noexcept {
    result_ = {};
    result_.stage_ns[0] = stage_clock_ns();
    result_.source = frame.metadata().source;
    // Bins, raw counts and classes are cleared here. Every field of every per-pixel
    // record is written this frame, by the classification pass below (on the
    // general path partly by the scan just before it), and every return before that
    // pass goes through fail(), which clears every record, so no stale pixel
    // evidence can survive into this frame.
    std::fill(bins_.begin(), bins_.end(), BinEvidence{});
    std::fill(raw_sample_counts_.begin(), raw_sample_counts_.end(), 0);
    obstacle_pixels_.clear();
    result_.stage_ns[1] = stage_clock_ns();
    if (mailbox_ == nullptr) mailbox_ = &mailbox;
    if (mailbox_ != &mailbox) {
        fail(GeometryStatus::InactiveOrStale);
        return result_;
    }
    // Order: the active stream, strictly newer than the last processed frame.
    const auto& source = result_.source;
    const auto active = mailbox.active_stream();
    if (active == 0 || active != source.stream_id || source.stream_id < last_.stream_id ||
        (source.stream_id == last_.stream_id && source.sequence <= last_.sequence)) {
        fail(GeometryStatus::InactiveOrStale);
        return result_;
    }
    if (source.sequence == 0 || source.host_ready_mono_ns < 0 || source.color_device_us == 0 ||
        source.depth_device_us == 0 || frame.pixel_count() != kGeometryPixels) {
        fail(GeometryStatus::InvalidFrame);
        return result_;
    }
    if (!profile_matches(frame.metadata())) {
        fail(GeometryStatus::ProfileMismatch);
        return result_;
    }
    last_ = source;  // A failed fit is still a consumed frame, not a retry target.
    // Scan: keep the depth codes (ground_point recomputes from them), build the
    // raw ledger and the fit candidates. Code 0 is an invalid depth. Only the
    // general path writes pixel records here (pixel-record slimming step 1,
    // 2026-10-14): the classification pass writes every invalid pixel's record and,
    // on the fast path, takes bin and range from the same per-pixel tables.
    std::size_t candidate_count = 0;
    if (!scan(frame, candidate_count)) return result_;
    result_.stage_ns[2] = stage_clock_ns();
    result_.fit = fit_ground(candidates_.data(),candidate_count,options_.prior,options_.fit);
    result_.stage_ns[3] = stage_clock_ns();
    // The fit-failure return must obey the same generation barrier as normal
    // geometry; a completed raw scan alone cannot keep a retired stream alive.
    if (mailbox.active_stream() != active) {
        fail(GeometryStatus::InactiveOrStale);
        return result_;
    }
    // InvalidInput also reports fit arithmetic failures. Do not preserve a raw
    // ledger across that numerical/integrity failure as an ordinary failed fit.
    if (result_.fit.status == FitStatus::InvalidInput) {
        fail(GeometryStatus::NumericalFailure);
        return result_;
    }
    result_.raw_sample_evidence_valid = true;
    if (result_.fit.status != FitStatus::Pass || !result_.fit.plane) {
        fail(GeometryStatus::FitFailed);
        if (mailbox.active_stream() != active)
            fail(GeometryStatus::InactiveOrStale);
        return result_;
    }
    const Plane plane = *result_.fit.plane;
    const double plane_offset = dot(plane.normal, binding_.t_base_camera)+plane.d;
    // Same factoring as the prior plane above, for the fitted plane.
    const double plane_const = plane.normal.z * binding_.t_base_camera.z + plane.d;
    // Every native pixel is evaluated; near-region filtering applies ONLY to
    // fitting. This pass must not be replaced with the fit candidate subset.
    // G-P1 phase 2 (2026-10-25): the depth codes, record / ray / ROI / bin arrays, the binding and the
    // option values the pass reads are locals, as in scan(): the record and bin stores could alias the
    // members, so GCC reloaded them per pixel. Same expressions (1 + eps_neg is the same sum, formed once).
    const std::uint16_t* const code = frame.pixels();
    PixelEvidence* const records = pixels_.data();
    const Vec3* const rays = rays_.data();
    const std::uint8_t* const roi = roi_.data();
    BinEvidence* const bins = bins_.data();
    const double unit = static_cast<double>(binding_.unit_m);
    const Vec3 t = binding_.t_base_camera;
    const bool fast = fast_azimuth_;
    const double eps1 = 1.0 + options_.eps_neg, near_lo = options_.blind_near_lo_m, range_max = options_.range_max_m;
    const double z_pass = options_.z_pass_m, h_tol = options_.h_tol_m;
    for (std::size_t k = 0; k < kGeometryPixels; ++k) {
        auto& e = records[k];
        if (code[k] == 0) {
            // Written in full: the scan left this record untouched.
            e = PixelEvidence{};
            e.classification = SampleClass::InvalidDepth;
        } else {
            // Same expression as the scan: recomputed instead of stored. One dot
            // product serves both the height and the pierce denominator below.
            const double z = static_cast<double>(code[k])*unit;
            const double plane_ray = dot(plane.normal, rays[k]);
            if (fast) {
                // The scan left this record untouched: every field is written here (the
                // class below). Bin and range are the scan's own: the same table entry, and
                // the same single multiplication of the same z (no addition to fuse), so the
                // same double. Written after plane_ray and inside this existing branch on
                // purpose: GCC -O2 contracts the three products of that dot product into
                // FMAs in an order that follows the surrounding code, and an extra branch or
                // a local record before it changed which product was fused (1 ulp in some
                // heights, found by test_native_geometry_pixel_record built with GCC).
                e.bin = pixel_bin_[k];
                e.range_m = z * pixel_rho_[k];
                e.pierce_bin = -1;
                e.pierce_range_m.reset();
                e.height_m = z * plane_ray + plane_const;
            } else {
                const Vec3 p = position(rays[k], z, t);
                e.height_m = dot(plane.normal,p)+plane.d;
            }
            if (!std::isfinite(e.height_m)) { fail(GeometryStatus::NumericalFailure); return result_; }
            // Separate expected intersection: do not assign a pit to a distant
            // return's azimuth when the camera has a nonzero translation.
            if (roi[k]) {
                // Same intersection as intersection(): the expected range/bin are
                // only needed when the actual return lies beyond the expected depth.
                const double denominator = plane_ray;
                if (std::isfinite(denominator) && denominator != 0) {
                    const double expected_z = -plane_offset/denominator;
                    const double actual_z = static_cast<double>(code[k])*unit;
                    // A return clearly beyond where the ground should be (by the
                    // relative margin eps_neg): a pierce candidate at the
                    // expected ground point, a possible pit or drop.
                    if (std::isfinite(expected_z) && expected_z > 0 && actual_z > expected_z*eps1) {
                        const Vec3 expected = position(rays[k], expected_z, t);
                        if (finite(expected)) {
                            const double expected_r = std::hypot(expected.x,expected.y);
                            const int expected_bin = bin_of(expected);
                            if (expected_bin >= 0 && expected_r >= near_lo && expected_r <= range_max) {
                                e.pierce_bin = expected_bin;
                                e.pierce_range_m = expected_r;
                                nearest(bins[static_cast<std::size_t>(expected_bin)].nearest_pierce_expected_m,expected_r);
                            }
                        }
                    }
                }
            }
            if (e.bin < 0 || e.range_m < near_lo || e.range_m > range_max) {
                e.classification = SampleClass::OutsideDomain;
            } else {
                // Height classes relative to the fitted plane (see file header).
                auto& bin = bins[static_cast<std::size_t>(e.bin)];
                ++bin.valid_samples;
                if (e.height_m > z_pass) {
                    e.classification = SampleClass::Overhead;
                } else if (e.height_m > h_tol) {
                    e.classification = SampleClass::Obstacle;
                    nearest(bin.nearest_positive_m,e.range_m);
                    obstacle_pixels_.push_back(static_cast<std::uint32_t>(k));
                } else if (e.height_m >= -h_tol) {
                    e.classification = SampleClass::Ground;
                    ++bin.ground_samples;
                    nearest(result_.nearest_ground_m,e.range_m);
                } else {
                    e.classification = SampleClass::BelowPlane;
                    nearest(bin.nearest_below_m,e.range_m);
                    nearest(bin.below_min_height_m,e.height_m);
                }
            }
        }
        // Read back through a volatile lvalue (2026-10-25): the value just written, but always
        // a load, one per pixel at the record stride, as before the scan changes. Held in a
        // register instead (GCC 11), this pass took about twice the L1 data refills, +1.0 ms on
        // .26 (file header). Do not simplify to e.classification without measuring on the target.
        ++result_.classes[static_cast<std::size_t>(*static_cast<const volatile SampleClass*>(&e.classification))];
    }
    result_.stage_ns[4] = stage_clock_ns();
    // Height window is evaluated against the FINAL closest positive distance.
    // Negative evidence remains separate; this kernel never selects h_block.
    // Only the Obstacle pixels noted above, in the same pixel order (a maximum, so the order
    // would not matter anyway): the other records are never read here.
    for (const std::uint32_t k : obstacle_pixels_) {
        const auto& e = pixels_[k];
        auto& b = bins_[static_cast<std::size_t>(e.bin)];
        if (b.nearest_positive_m && e.range_m <= *b.nearest_positive_m+options_.w_h_m)
            highest(b.positive_height_window_max_m,e.height_m);
    }
    if (mailbox.active_stream() != active) {
        fail(GeometryStatus::InactiveOrStale);
        return result_;
    }
    result_.stage_ns[5] = stage_clock_ns();
    result_.status = GeometryStatus::NormalEvidence;
    return result_;
}

// Heap bytes of all per-pixel and per-bin tables, for the startup log.
std::size_t NativeGeometry::storage_bytes() const noexcept {
    return (rays_.capacity()+candidates_.capacity())*sizeof(Vec3) + codes_.capacity()*sizeof(std::uint16_t) +
        pixel_bin_.capacity()*sizeof(int) + (pixel_rho_.capacity()+pixel_prior_.capacity())*sizeof(double) +
        pixels_.capacity()*sizeof(PixelEvidence) + bins_.capacity()*sizeof(BinEvidence) +
        roi_.capacity()*sizeof(std::uint8_t) + raw_sample_counts_.capacity()*sizeof(std::uint32_t) +
        obstacle_pixels_.capacity()*sizeof(std::uint32_t);
}
// Where this pixel's ray meets `plane` (2026-10-27, fitted tickets), for prior-ROI pixels only: the
// inline ticket_on, as NormalGrid's per-frame pass runs it, with its range sqrt(xh^2 + yh^2) / dh
// (the pass itself compares squares and never forms it).
bool NativeGeometry::plane_ground_projection(const Plane& plane, std::size_t pixel, int hint, int& bin,
                                             double& range_m) const noexcept {
    bin = -1;
    range_m = 0;
    if (pixel >= rays_.size() || !roi_[pixel]) return false;
    double xh = 0, yh = 0, dh = 0;
    if (!ticket_on(ticket_plane(plane), pixel, hint, bin, xh, yh, dh)) {
        bin = -1;
        return false;
    }
    range_m = std::sqrt(xh * xh + yh * yh) / dh;
    return true;
}
// Where this pixel's ray meets the PRIOR plane (setup geometry, any frame).
bool NativeGeometry::prior_ground_projection(std::size_t pixel, int& bin, double& range_m) const noexcept {
    bin = -1;
    range_m = 0;
    if (pixel >= rays_.size() || !roi_[pixel]) return false;
    Vec3 expected;
    double z = 0;
    if (!intersection(options_.prior, rays_[pixel], binding_.t_base_camera, expected, z)) return false;
    const int i = bin_of(expected);
    const double r = std::hypot(expected.x, expected.y);
    if (i < 0 || !std::isfinite(r) || r < options_.blind_near_lo_m || r > options_.range_max_m) return false;
    bin = i;
    range_m = r;
    return true;
}
bool NativeGeometry::ground_depth(std::size_t pixel, double& z) const noexcept {
    z = 0;
    if (!result_.has_evidence() || pixel >= pixels_.size() ||
        pixels_[pixel].classification != SampleClass::Ground || pixel >= codes_.size())
        return false;
    const double depth = static_cast<double>(codes_[pixel])*static_cast<double>(binding_.unit_m);
    if (!std::isfinite(depth)) return false;
    z = depth;
    return true;
}
bool NativeGeometry::ground_point(std::size_t pixel, Vec3& out) const noexcept {
    out = {};
    if (!result_.has_evidence() || pixel >= pixels_.size() ||
        pixels_[pixel].classification != SampleClass::Ground || pixel >= codes_.size())
        return false;
    // Same expression as the scan, from this frame's stored depth code.
    const double z = static_cast<double>(codes_[pixel])*static_cast<double>(binding_.unit_m);
    const Vec3 p = position(rays_[pixel], z, binding_.t_base_camera);
    if (!finite(p)) return false;
    out = p;
    return true;
}
}  // namespace perception::geometry
