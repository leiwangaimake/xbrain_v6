/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_bench_calibration.cpp
 * Brief: Host test of the bench extrinsic from a synthetic depth floor.
 *
 * Description:
 * What it checks. A floor seen from a known mounting (height, pitch, roll) is recovered across
 * frames; the same frame fed through NativeGeometry with the result closes, the fitted floor
 * coming back at bench z = 0; inconsistent or unusable frames are rejected. Synthetic rays only.
 */

#include "service/bench_calibration.hpp"

#include "modules/native_geometry.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

using namespace perception;
using namespace perception::service;
using geometry::Vec3;

namespace {
int checks = 0, failures = 0, rejections = 0;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; std::cerr << "FAIL " << __LINE__ << ": " #cond "\n"; } } while (0)
constexpr double kPi = 3.14159265358979323846;
constexpr float kUnit = 0.001F;

std::vector<Vec3> pinhole(double f = 320, double cx = 320, double cy = 200) {
    std::vector<Vec3> r(BenchCalibrator::kPixels);
    for (std::size_t v = 0; v < 400; ++v)
        for (std::size_t u = 0; u < 640; ++u) r[v * 640 + u] = {(u - cx) / f, (v - cy) / f, 1.0};
    return r;
}

// World (x fwd, y left, z up), floor z = 0, camera at height h, pitched down by
// pitch, rolled by roll (right side down positive -> reported roll negative).
// Returns camera->world rotation columns via R_wc.
void camera_to_world(double pitch, double roll, double R[3][3]) {
    // base optical-to-level mapping
    const double B[3][3] = {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}};
    // pitch down about world y (left): rotating +x toward -z
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double P[3][3] = {{cp, 0, sp}, {0, 1, 0}, {-sp, 0, cp}};
    // roll about world x (forward)
    const double cr = std::cos(roll), sr = std::sin(roll);
    const double Rr[3][3] = {{1, 0, 0}, {0, cr, -sr}, {0, sr, cr}};
    double T[3][3] = {}, U[3][3] = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) T[i][j] += P[i][k] * Rr[k][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) U[i][j] += T[i][k] * B[k][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R[i][j] = U[i][j];
}

std::vector<std::uint16_t> floor_frame(const std::vector<Vec3>& rays, double h, double pitch, double roll,
                                      double max_range = 5.0) {
    double R[3][3];
    camera_to_world(pitch, roll, R);
    std::vector<std::uint16_t> px(BenchCalibrator::kPixels, 0);
    for (std::size_t i = 0; i < rays.size(); ++i) {
        const auto& r = rays[i];
        const double wz = R[2][0] * r.x + R[2][1] * r.y + R[2][2] * r.z;  // world z of the ray (z_cam = 1)
        if (wz >= -1e-6) continue;                                         // never reaches the floor
        const double depth = h / -wz;                                      // optical z distance
        const double wx = depth * (R[0][0] * r.x + R[0][1] * r.y + R[0][2] * r.z);
        const double wy = depth * (R[1][0] * r.x + R[1][1] * r.y + R[1][2] * r.z);
        if (std::hypot(wx, wy) > max_range) continue;
        px[i] = static_cast<std::uint16_t>(std::lround(depth / kUnit));
    }
    return px;
}

BenchCalibrationOptions options() {
    BenchCalibrationOptions o;
    o.near_min_m = 0.4;
    o.near_max_m = 3.0;
    o.min_below_m = 0.1;
    o.fit.inlier_distance_m = 0.01;
    o.fit.min_inlier_fraction = 0.8;
    o.fit.max_inlier_rms_m = 0.005;
    o.fit.min_xy_stddev_m = 0.1;
    o.fit.min_up_dot = 0.7;
    o.fit.max_prior_angle_rad = 45.0 * kPi / 180.0;
    o.fit.min_inliers = 1000;
    o.fit.hypotheses = 64;
    o.fit.seed = 0x9e3779b97f4a7c15ULL;
    o.max_normal_spread_rad = 0.5 * kPi / 180.0;
    o.max_height_spread_m = 0.005;
    o.min_frames = 5;
    return o;
}

double quat_to_axis_z(const std::array<double, 4>& q, int column) {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double R[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                            {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                            {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
    return R[2][column];
}
}  // namespace

int main() {
    const auto rays = pinhole();
    // ---- recovery of a known mounting ----
    for (const auto& c : std::vector<std::array<double, 3>>{{0.62, 25.0, 3.0}, {0.35, 10.0, 0.0}, {1.10, 40.0, -5.0}}) {
        const double h = c[0], pitch = c[1] * kPi / 180.0, roll = c[2] * kPi / 180.0;
        BenchCalibrator cal(rays, options());
        const auto frame = floor_frame(rays, h, pitch, roll);
        for (int k = 0; k < 6; ++k) {
            const auto fit = cal.add_frame(frame.data(), frame.size(), kUnit);
            CHECK(fit.status == geometry::FitStatus::Pass);
        }
        const auto r = cal.finish();
        CHECK(r.ok && r.frames_seen == 6 && r.frames_passed == 6 && r.inliers_total > 6000);
        CHECK(std::abs(r.height_m - h) < 0.002 && r.translation[0] == 0 && r.translation[1] == 0 && r.translation[2] == r.height_m);
        CHECK(std::abs(r.pitch_down_deg - c[1]) < 0.2);
        // reported roll = elevation of the camera right axis; the synthetic roll is
        // about the world forward axis, applied before pitch: asin(-cos(pitch)*sin(roll))
        CHECK(std::abs(r.roll_deg - std::asin(-std::cos(pitch) * std::sin(roll)) * 180.0 / kPi) < 0.2);
        const auto& q = r.rotation_xyzw;
        CHECK(std::abs(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1.0) <= 1e-12);
        // camera optical axis points down by the pitch in the bench frame
        CHECK(std::abs(-std::asin(quat_to_axis_z(q, 2)) * 180.0 / kPi - c[1]) < 0.2);
        CHECK(r.worst_inlier_rms_m < 0.002 && r.worst_inlier_fraction > 0.8);

        // ---- closure: the same frame through NativeGeometry with the result ----
        geometry::ProjectionBinding b;
        b.intrinsics = {320, 320, 320, 200, 640, 400};
        b.distortion.reported = true;
        b.distortion.model = 3;
        b.unit_m = kUnit;
        const double x = q[0], y = q[1], z = q[2], w = q[3];
        b.R_base_camera = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                           2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                           2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
        b.t_base_camera = {r.translation[0], r.translation[1], r.translation[2]};
        geometry::GeometryOptions go;
        go.prior = {{0, 0, 1}, 0};
        go.fit = options().fit;
        go.fit_near_min_m = 0.4; go.fit_near_max_m = 3.0; go.fit_prior_band_m = 0.05;
        go.range_max_m = 6; go.blind_near_lo_m = 0.1; go.h_tol_m = 0.08; go.z_pass_m = 1.5;
        go.eps_neg = 0.05; go.w_h_m = 0.1; go.angle_min_rad = -1.0; go.angle_step_rad = 0.25; go.n_bins = 9;
        geometry::NativeGeometry geom(b, rays.data(), rays.size(), go);
        native_depth::Mailbox mailbox(BenchCalibrator::kPixels);
        const auto stream = mailbox.begin_stream();
        native_depth::View view;
        view.metadata.intrinsics = b.intrinsics;
        view.metadata.distortion = b.distortion;
        view.metadata.unit_m = kUnit;
        view.metadata.source.stream_id = stream;
        view.metadata.source.sequence = 1;
        view.metadata.source.depth_index = 1;
        view.metadata.source.depth_device_us = 1;
        view.metadata.source.color_index = 1;
        view.metadata.source.color_device_us = 1;
        view.metadata.source.host_ready_mono_ns = 1;
        view.pixels = frame.data();
        view.bytes = frame.size() * sizeof(std::uint16_t);
        view.pre_d2c = true;
        CHECK(mailbox.try_offer(view) == native_depth::Offer::Accepted);
        native_depth::Frame taken(BenchCalibrator::kPixels);
        CHECK(mailbox.try_take(taken) == native_depth::Take::Taken);
        const auto& g = geom.process(taken, mailbox);
        CHECK(g.status == geometry::GeometryStatus::NormalEvidence && g.fit.plane);
        if (g.fit.plane) {
            CHECK(std::abs(g.fit.plane->d) < 0.003 && g.fit.plane->normal.z > 0.9999);
        }
        CHECK(g.classes[static_cast<std::size_t>(geometry::SampleClass::Obstacle)] == 0);
    }

    // ---- rejections ----
    {
        BenchCalibrator cal(rays, options());
        for (int k = 0; k < 6; ++k) {
            const auto f = floor_frame(rays, 0.60 + 0.01 * k, 25.0 * kPi / 180.0, 0);  // floor moving 1 cm per frame
            (void)cal.add_frame(f.data(), f.size(), kUnit);
        }
        const auto r = cal.finish();
        CHECK(!r.ok && r.failure.find("height") != std::string::npos);
        ++rejections;
    }
    {
        BenchCalibrator cal(rays, options());
        for (int k = 0; k < 6; ++k) {
            const auto f = floor_frame(rays, 0.6, (20.0 + k) * kPi / 180.0, 0);  // tilting 1 degree per frame
            (void)cal.add_frame(f.data(), f.size(), kUnit);
        }
        const auto r = cal.finish();
        CHECK(!r.ok && r.failure.find("normal") != std::string::npos);
        ++rejections;
    }
    {
        BenchCalibrator cal(rays, options());
        std::vector<std::uint16_t> empty(BenchCalibrator::kPixels, 0);
        for (int k = 0; k < 6; ++k) {
            const auto fit = cal.add_frame(empty.data(), empty.size(), kUnit);
            CHECK(fit.status == geometry::FitStatus::InsufficientCandidates);
        }
        const auto r = cal.finish();
        CHECK(!r.ok && r.frames_passed == 0);
        ++rejections;
    }
    {
        BenchCalibrator cal(rays, options());
        const auto sky = floor_frame(rays, 0.6, -60.0 * kPi / 180.0, 0);  // camera looking up: no floor pixels
        for (int k = 0; k < 6; ++k) (void)cal.add_frame(sky.data(), sky.size(), kUnit);
        CHECK(!cal.finish().ok);
        ++rejections;
    }
    {
        BenchCalibrator cal(rays, options());
        const auto f = floor_frame(rays, 0.6, 25.0 * kPi / 180.0, 0);
        for (int k = 0; k < 4; ++k) (void)cal.add_frame(f.data(), f.size(), kUnit);  // fewer than min_frames
        const auto r = cal.finish();
        CHECK(!r.ok && r.frames_passed == 4);
        ++rejections;
        CHECK(cal.add_frame(nullptr, 0, kUnit).status != geometry::FitStatus::Pass);
        CHECK(cal.add_frame(f.data(), f.size(), 0.0F).status != geometry::FitStatus::Pass);
    }
    {
        bool bad_rays = false, bad_options = false;
        try { BenchCalibrator cal(std::vector<Vec3>(10), options()); } catch (const std::invalid_argument&) { bad_rays = true; }
        auto o = options();
        o.near_max_m = o.near_min_m;
        try { BenchCalibrator cal(rays, o); } catch (const std::invalid_argument&) { bad_options = true; }
        CHECK(bad_rays && bad_options);
    }
    std::cout << "{\"result\":\"" << (failures ? "FAIL" : "PASS") << "\",\"checks\":" << checks << ",\"failures\":" << failures
              << ",\"rejections\":" << rejections << ",\"scope\":\"bench_calibration_host\",\"camera\":false}" << std::endl;
    return failures ? 1 : 0;
}
