/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: bench_calibration.hpp
 * Brief: Bench extrinsic from the depth floor plane (19 section 7.1 step 1, reply 2026-09-17 Q13).
 *
 * Description:
 * Plane fit -> roll, pitch, camera height.
 * Bench frame definition (a component-verification frame, NOT base_link):
 *   z = fitted floor normal (up); origin = foot point of the camera optical
 *   centre on the fitted plane; x = camera optical axis projected on the plane;
 *   y = z x x. Hence yaw and x/y translation are zero BY DEFINITION
 *   (record method yaw/xy = bench_axis_definition) and only roll, pitch and
 *   height are measured.
 *
 * Per frame: native pixels -> camera points (depth * optical ray, z == 1) ->
 * provisional level frame (x fwd, y left, z up assuming zero pitch/roll) ->
 * candidates in a horizontal ring below the camera -> geometry::fit_ground.
 * Frames are aggregated only when every passing frame agrees (normal spread,
 * height spread); the record reports the WORST per-frame inlier RMS and inlier
 * fraction, never a best-case or averaged residual.
 * Pure computation: no SDK, device, clock or file access. Setup allocates.
 */
#pragma once

#include "modules/ground_fit.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace perception::service {

struct BenchCalibrationOptions {
    double near_min_m = 0, near_max_m = 0;  // horizontal radius of candidates (provisional frame)
    double min_below_m = 0;                 // candidates at least this far below the optical centre
    geometry::FitOptions fit;               // prior normal is +Z of the provisional frame
    double max_normal_spread_rad = 0;       // between any passing frame and the mean normal
    double max_height_spread_m = 0;         // between any passing frame and the mean height
    std::size_t min_frames = 0;             // passing frames required
};

struct BenchCalibrationResult {
    bool ok = false;
    std::string failure;
    std::size_t frames_seen = 0, frames_passed = 0;
    long inliers_total = 0;
    double worst_inlier_fraction = 0, worst_inlier_rms_m = 0;
    double height_m = 0, pitch_down_deg = 0, roll_deg = 0;
    double normal_spread_rad = 0, height_spread_m = 0;
    std::array<double, 3> translation{};    // bench <- camera, metres
    std::array<double, 4> rotation_xyzw{};  // bench <- camera, unit quaternion
};

class BenchCalibrator {
public:
    static constexpr std::size_t kPixels = 640U * 400U;
    // optical_rays: kPixels distortion-corrected rays with z == 1 for the bound
    // native profile (e.g. OrbbecNativeRays::rays()). Throws std::invalid_argument.
    BenchCalibrator(const std::vector<geometry::Vec3>& optical_rays, const BenchCalibrationOptions& options);
    // Fits one frame. unit_m converts raw codes to metres; code 0 is invalid.
    geometry::FitResult add_frame(const std::uint16_t* pixels, std::size_t count, float unit_m);
    BenchCalibrationResult finish() const;

private:
    std::vector<geometry::Vec3> rays_, candidates_;
    BenchCalibrationOptions options_;
    std::vector<geometry::Plane> planes_;
    std::size_t frames_seen_ = 0;
    long inliers_total_ = 0;
    double worst_fraction_ = 1, worst_rms_ = 0;
};

}  // namespace perception::service
