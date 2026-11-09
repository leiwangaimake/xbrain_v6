/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: native_geometry_parent.cpp
 * Brief: Frozen copy of the parent NativeGeometry, the oracle for the pixel-record tests.
 *
 * Description:
 * Built only into test_native_geometry_pixel_record (namespace perception::geometry::native_geometry_parent),
 * which compares every output of the production NativeGeometry::process against it. Not production
 * code. The body below is src/modules/native_geometry.cpp of commit 33a3032 unchanged except for the
 * include, the namespace, and the two free functions valid_projection_binding / valid_geometry_options
 * with the constant pi only they used (the production functions are used); do not edit it together
 * with the production file.
 */
#include "native_geometry_parent.hpp"
#include <time.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace perception::geometry::native_geometry_parent {
namespace {
// Stage timestamps for the timing ledger (CLOCK_MONOTONIC ns, -1 on error).
std::int64_t stage_clock_ns() noexcept {
    timespec ts{};
    if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return -1;
    return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
}  // namespace

namespace {
static_assert(kGeometryPixels <= std::numeric_limits<std::uint32_t>::max(),
              "one raw bin count must hold a complete native frame");
bool finite(Vec3 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
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
}

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
int NativeGeometry::bin_of(Vec3 p) const noexcept {
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
            std::size_t k = static_cast<std::size_t>(guess);
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

const GeometryResult& NativeGeometry::process(const native_depth::Frame& frame,
                                             const native_depth::Mailbox& mailbox) noexcept {
    result_ = {};
    result_.stage_ns[0] = stage_clock_ns();
    result_.source = frame.metadata().source;
    // Bins, raw counts and classes are cleared here. Every per-pixel record is
    // rewritten in full by the scan below (fail() still clears everything), so
    // no stale pixel evidence can survive into this frame.
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
    // raw ledger and the fit candidates. Code 0 is an invalid depth.
    std::memcpy(codes_.data(), frame.pixels(), kGeometryPixels * sizeof(std::uint16_t));
    std::size_t candidate_count = 0;
    for (std::size_t k = 0; k < kGeometryPixels; ++k) {
        ++result_.visited_pixels;
        result_.roi_pixels += roi_[k];
        pixels_[k] = PixelEvidence{};
        if (frame.pixels()[k] == 0) {
            result_.invalid_roi_pixels += roi_[k];
            continue;
        }
        ++result_.valid_depth_pixels;
        const double z = static_cast<double>(frame.pixels()[k])*static_cast<double>(binding_.unit_m);
        double range = 0, prior_height = 0;
        int bin = -1;
        Vec3 p{};
        if (fast_azimuth_) {
            p = {rays_[k].x * z, rays_[k].y * z, rays_[k].z * z + binding_.t_base_camera.z};
            range = z * pixel_rho_[k];
            prior_height = z * pixel_prior_[k] + prior_const_;
            bin = pixel_bin_[k];
            if (!finite(p) || !std::isfinite(range) || !std::isfinite(prior_height)) {
                fail(GeometryStatus::NumericalFailure); return result_;
            }
        } else {
            p = position(rays_[k], z, binding_.t_base_camera);
            if (!finite(p)) { fail(GeometryStatus::NumericalFailure); return result_; }
            range = std::hypot(p.x,p.y);
            prior_height = dot(options_.prior.normal,p)+options_.prior.d;
            if (!std::isfinite(range) || !std::isfinite(prior_height)) {
                fail(GeometryStatus::NumericalFailure); return result_;
            }
            bin = bin_of(p);
        }
        // This ledger depends only on the actual valid return, projection and
        // the unchanged angular/range domain. Fit ROI and height never select
        // which raw samples exist. Cache bin/range for the later normal pass.
        pixels_[k].bin = bin;
        pixels_[k].range_m = range;
        if (bin >= 0 && range >= options_.blind_near_lo_m && range <= options_.range_max_m)
            ++raw_sample_counts_[static_cast<std::size_t>(bin)];
        // Fit candidates: near field only, and close to the prior plane, so the
        // fit sees ground and not walls or furniture.
        if (range >= options_.fit_near_min_m && range <= options_.fit_near_max_m &&
            std::abs(prior_height) <= options_.fit_prior_band_m)
            candidates_[candidate_count++] = p;
    }
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
    for (std::size_t k = 0; k < kGeometryPixels; ++k) {
        auto& e = pixels_[k];
        if (frame.pixels()[k] == 0) {
            e.classification = SampleClass::InvalidDepth;
        } else {
            // Same expression as the scan: recomputed instead of stored. One dot
            // product serves both the height and the pierce denominator below.
            const double z = static_cast<double>(frame.pixels()[k])*static_cast<double>(binding_.unit_m);
            const double plane_ray = dot(plane.normal, rays_[k]);
            if (fast_azimuth_) {
                e.height_m = z * plane_ray + plane_const;
            } else {
                const Vec3 p = position(rays_[k], z, binding_.t_base_camera);
                e.height_m = dot(plane.normal,p)+plane.d;
            }
            if (!std::isfinite(e.height_m)) { fail(GeometryStatus::NumericalFailure); return result_; }
            // Separate expected intersection: do not assign a pit to a distant
            // return's azimuth when the camera has a nonzero translation.
            if (roi_[k]) {
                // Same intersection as intersection(): the expected range/bin are
                // only needed when the actual return lies beyond the expected depth.
                const double denominator = plane_ray;
                if (std::isfinite(denominator) && denominator != 0) {
                    const double expected_z = -plane_offset/denominator;
                    const double actual_z = static_cast<double>(frame.pixels()[k])*static_cast<double>(binding_.unit_m);
                    // A return clearly beyond where the ground should be (by the
                    // relative margin eps_neg): a pierce candidate at the
                    // expected ground point, a possible pit or drop.
                    if (std::isfinite(expected_z) && expected_z > 0 && actual_z > expected_z*(1.0+options_.eps_neg)) {
                        const Vec3 expected = position(rays_[k], expected_z, binding_.t_base_camera);
                        if (finite(expected)) {
                            const double expected_r = std::hypot(expected.x,expected.y);
                            const int expected_bin = bin_of(expected);
                            if (expected_bin >= 0 && expected_r >= options_.blind_near_lo_m && expected_r <= options_.range_max_m) {
                                e.pierce_bin = expected_bin;
                                e.pierce_range_m = expected_r;
                                nearest(bins_[static_cast<std::size_t>(expected_bin)].nearest_pierce_expected_m,expected_r);
                            }
                        }
                    }
                }
            }
            if (e.bin < 0 || e.range_m < options_.blind_near_lo_m || e.range_m > options_.range_max_m) {
                e.classification = SampleClass::OutsideDomain;
            } else {
                // Height classes relative to the fitted plane (see file header).
                auto& bin = bins_[static_cast<std::size_t>(e.bin)];
                ++bin.valid_samples;
                if (e.height_m > options_.z_pass_m) {
                    e.classification = SampleClass::Overhead;
                } else if (e.height_m > options_.h_tol_m) {
                    e.classification = SampleClass::Obstacle;
                    nearest(bin.nearest_positive_m,e.range_m);
                    obstacle_pixels_.push_back(static_cast<std::uint32_t>(k));
                } else if (e.height_m >= -options_.h_tol_m) {
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
        ++result_.classes[static_cast<std::size_t>(e.classification)];
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
}  // namespace perception::geometry::native_geometry_parent
