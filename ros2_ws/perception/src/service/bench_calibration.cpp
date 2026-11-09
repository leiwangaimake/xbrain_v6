/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: bench_calibration.cpp
 * Brief: BenchCalibrator: camera roll, pitch and height above the floor from depth frames.
 *
 * Description:
 * Implements bench_calibration.hpp (the bench frame and the per-frame pipeline are defined there).
 * Each frame is fitted on its own with the production ground fit; the result is accepted only
 * when every passing frame agrees within the configured spreads, and it reports the worst
 * per-frame residuals, so a calibration is never better on paper than its worst frame.
 */
#include "service/bench_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace perception::service {
namespace {
using geometry::Vec3;
constexpr double kPi = 3.14159265358979323846;

Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 scale(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
double norm(Vec3 a) { return std::sqrt(dot(a, a)); }
bool finite(Vec3 a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }

// provisional (x fwd, y left, z up) <- camera optical (x right, y down, z fwd). Assumes zero
// pitch and roll; it only selects candidates below the camera, the fit finds the real tilt.
Vec3 to_provisional(Vec3 c) { return {c.z, -c.x, -c.y}; }

// Rotation matrix -> unit quaternion (x, y, z, w), the extrinsic_base_cam format. Branches on the
// largest diagonal term for numerical stability; w >= 0 picks one of the two equivalent signs.
std::array<double, 4> quaternion(const double R[3][3]) {
    const double trace = R[0][0] + R[1][1] + R[2][2];
    std::array<double, 4> q{};
    if (trace > 0) {
        const double t = std::sqrt(1.0 + trace) * 2;
        q = {(R[2][1] - R[1][2]) / t, (R[0][2] - R[2][0]) / t, (R[1][0] - R[0][1]) / t, 0.25 * t};
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        const double t = std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2;
        q = {0.25 * t, (R[1][0] + R[0][1]) / t, (R[0][2] + R[2][0]) / t, (R[2][1] - R[1][2]) / t};
    } else if (R[1][1] > R[2][2]) {
        const double t = std::sqrt(1.0 - R[0][0] + R[1][1] - R[2][2]) * 2;
        q = {(R[1][0] + R[0][1]) / t, 0.25 * t, (R[2][1] + R[1][2]) / t, (R[0][2] - R[2][0]) / t};
    } else {
        const double t = std::sqrt(1.0 - R[0][0] - R[1][1] + R[2][2]) * 2;
        q = {(R[0][2] + R[2][0]) / t, (R[2][1] + R[1][2]) / t, 0.25 * t, (R[1][0] - R[0][1]) / t};
    }
    if (q[3] < 0)
        for (auto& v : q) v = -v;
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (auto& v : q) v /= n;
    return q;
}
}  // namespace

BenchCalibrator::BenchCalibrator(const std::vector<Vec3>& optical_rays, const BenchCalibrationOptions& o)
    : rays_(optical_rays), options_(o) {
    // The native 640x400 depth raster with undistorted optical rays (z == 1, so depth * ray is the
    // camera-frame point), and options that can actually accept a frame.
    if (rays_.size() != kPixels) throw std::invalid_argument("bench calibration needs 640x400 optical rays");
    for (const auto& r : rays_)
        if (!finite(r) || r.z != 1.0) throw std::invalid_argument("optical rays must be finite with z == 1");
    if (!geometry::valid_fit_options(o.fit) || !(o.near_min_m >= 0) || !(o.near_max_m > o.near_min_m) ||
        !(o.min_below_m >= 0) || !(o.max_normal_spread_rad > 0) || !(o.max_height_spread_m > 0) || o.min_frames == 0)
        throw std::invalid_argument("invalid bench calibration options");
    candidates_.reserve(kPixels);
}

geometry::FitResult BenchCalibrator::add_frame(const std::uint16_t* pixels, std::size_t count, float unit_m) {
    ++frames_seen_;
    geometry::FitResult failed;
    if (!pixels || count != kPixels || !std::isfinite(unit_m) || unit_m <= 0) return failed;
    // Candidates: valid depth in a horizontal ring around the camera (near_min..near_max) and at
    // least min_below under it, i.e. where the floor is expected. `below` keeps up to 4096 of their
    // depths below the camera to estimate the prior plane height.
    candidates_.clear();
    std::vector<double> below;
    below.reserve(4096);
    for (std::size_t i = 0; i < kPixels; ++i) {
        if (pixels[i] == 0) continue;
        const Vec3 p = to_provisional(scale(rays_[i], static_cast<double>(pixels[i]) * unit_m));
        const double r = std::hypot(p.x, p.y);
        if (r < options_.near_min_m || r > options_.near_max_m || -p.z < options_.min_below_m) continue;
        candidates_.push_back(p);
        if (below.size() < below.capacity()) below.push_back(-p.z);
    }
    if (candidates_.size() < options_.fit.min_inliers) {
        failed.status = geometry::FitStatus::InsufficientCandidates;
        failed.candidates = candidates_.size();
        return failed;
    }
    // Prior: level plane at the median depth-below of the sampled candidates.
    std::nth_element(below.begin(), below.begin() + static_cast<long>(below.size() / 2), below.end());
    geometry::Plane prior{{0, 0, 1}, below[below.size() / 2]};
    const auto fit = geometry::fit_ground(candidates_.data(), candidates_.size(), prior, options_.fit);
    // Only passing frames are aggregated; the worst inlier fraction and RMS over them are kept.
    if (fit.status == geometry::FitStatus::Pass && fit.plane) {
        planes_.push_back(*fit.plane);
        inliers_total_ += static_cast<long>(fit.inliers);
        worst_fraction_ = std::min(worst_fraction_, fit.inlier_fraction);
        worst_rms_ = std::max(worst_rms_, fit.inlier_rms_m);
    }
    return fit;
}

BenchCalibrationResult BenchCalibrator::finish() const {
    BenchCalibrationResult out;
    out.frames_seen = frames_seen_;
    out.frames_passed = planes_.size();
    const auto fail = [&out](const std::string& why) {
        out.ok = false;
        out.failure = why;
        return out;
    };
    if (planes_.size() < options_.min_frames) return fail("too few frames with a passing floor fit");
    // Mean plane (normalised mean normal, mean offset), then the largest deviation of any frame
    // from it: frames that disagree mean the camera or the floor moved, and nothing is recorded.
    Vec3 n{};
    double d = 0;
    for (const auto& p : planes_) {
        n = {n.x + p.normal.x, n.y + p.normal.y, n.z + p.normal.z};
        d += p.d;
    }
    n = scale(n, 1.0 / norm(n));
    d /= static_cast<double>(planes_.size());
    for (const auto& p : planes_) {
        const double c = std::clamp(dot(p.normal, n) / norm(p.normal), -1.0, 1.0);
        out.normal_spread_rad = std::max(out.normal_spread_rad, std::acos(c));
        out.height_spread_m = std::max(out.height_spread_m, std::abs(p.d - d));
    }
    if (out.normal_spread_rad > options_.max_normal_spread_rad) return fail("floor normal differs between frames");
    if (out.height_spread_m > options_.max_height_spread_m) return fail("floor height differs between frames");
    // Plane n.p + d = 0 with n up: the optical centre (origin) is d above the floor.
    if (!(d > 0) || !(n.z > 0)) return fail("camera is not above an upward floor plane");
    // Bench axes (see the header): z = floor normal, x = optical axis (provisional x) projected
    // onto the floor, y = z x x. Yaw is zero by this definition.
    const Vec3 zb = n;
    Vec3 xb = sub({1, 0, 0}, scale(zb, zb.x));
    if (norm(xb) < 1e-6) return fail("optical axis is perpendicular to the floor");
    xb = scale(xb, 1.0 / norm(xb));
    const Vec3 yb = cross(zb, xb);
    // R_bench<-camera = R_bench<-provisional * R_provisional<-camera
    const double Rbp[3][3] = {{xb.x, xb.y, xb.z}, {yb.x, yb.y, yb.z}, {zb.x, zb.y, zb.z}};
    const double Rpc[3][3] = {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}};
    double R[3][3] = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) R[i][j] += Rbp[i][k] * Rpc[k][j];
    // The origin is the optical centre's foot point on the floor, so the translation is only the
    // height d; roll and pitch are read back from R for the record.
    out.rotation_xyzw = quaternion(R);
    out.translation = {0.0, 0.0, d};
    out.height_m = d;
    // camera optical axis in bench: column 2 of R; pitch down = -asin(z component)
    out.pitch_down_deg = -std::asin(std::clamp(R[2][2], -1.0, 1.0)) * 180.0 / kPi;
    // camera x (right) in bench: column 0; roll = asin(z component) (right side down -> negative)
    out.roll_deg = std::asin(std::clamp(R[2][0], -1.0, 1.0)) * 180.0 / kPi;
    out.inliers_total = inliers_total_;
    out.worst_inlier_fraction = worst_fraction_;
    out.worst_inlier_rms_m = worst_rms_;
    out.ok = true;
    return out;
}

}  // namespace perception::service
