/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rgb_depth_registration.hpp
 * Brief: Registers the software-D2C aligned depth raster onto the decoded RGB
 *        image (batch-4 reply B4-2, option 1).
 *
 * Description:
 * Measured on .23 (evidence/segment2/alignprobe, b42_samples):
 *   - the SDK 2.9.3 software aligner lays depth out on a PINHOLE raster with the
 *     colour intrinsics (no distortion);
 *   - the decoded RGB image follows the reported standard Brown-Conrady model
 *     (k1 k2 k3 p1 p2; model 4 with k4..k6 = 0) far better than pinhole.
 * Hence the RGB pixel p and the aligned pixel q of the same 3D point relate by
 *   q = K * undistort(p).
 * This class precomputes, for every RGB pixel centre, the NEAREST aligned pixel
 * (no interpolation: a depth value is never blended across an edge) and the
 * undistorted optical ray, so depth sampling, object geometry and centroid
 * back-projection all use one explicit mapping. RGB pixels whose mapped
 * position leaves the aligned raster read 0 (invalid depth).
 *
 * Residual limits (measured, not removed here): after this mapping about
 * 1.1-1.4 px board-dependent offset and ~+1.2 px per unit normalized radius
 * remain between RGB corners and the SDK-parameter prediction.
 * Pure computation: no SDK, OpenCV, clock or device access.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace perception {

struct StandardDistortion {
    double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;
};

// Standard Brown-Conrady (OpenCV convention) on normalized coordinates.
void distort_normalized(const StandardDistortion& d, double x, double y, double& xd, double& yd) noexcept;

class RgbDepthRegistration {
public:
    // Throws std::invalid_argument on invalid intrinsics, a raster outside
    // [1, 8192], or a model that is not invertible at any pixel centre
    // (non-positive Jacobian determinant, no convergence, or forward residual
    // above 1e-6 px).
    RgbDepthRegistration(int width, int height, double fx, double fy, double cx, double cy, const StandardDistortion& d);

    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }

    // out[i] = aligned[index(i)], or 0 where the mapped position is outside.
    // Both buffers are width*height, row-major; they must not overlap.
    // Executes precomputed runs of consecutive indices (memcpy / zero fill);
    // byte-identical to the per-pixel gather over aligned_index().
    void apply(const std::uint16_t* aligned, std::uint16_t* out) const noexcept;
    std::size_t run_count() const noexcept { return runs_.size(); }

    // Aligned-raster index of RGB pixel i, or -1 when outside.
    std::int32_t aligned_index(std::size_t rgb_index) const noexcept { return index_[rgb_index]; }

    // Undistorted optical ray (x, y, 1) of every RGB pixel centre, row-major.
    const std::vector<float>& ray_x() const noexcept { return ray_x_; }
    const std::vector<float>& ray_y() const noexcept { return ray_y_; }

    // Undistorts an arbitrary RGB (sub)pixel coordinate to normalized (x, y).
    // Returns false when the iteration does not converge.
    bool undistort_pixel(double u, double v, double& x, double& y) const noexcept;

    std::size_t outside_pixels() const noexcept { return outside_; }
    double max_shift_px() const noexcept { return max_shift_px_; }
    double max_forward_residual_px() const noexcept { return max_residual_px_; }

private:
    int width_, height_;
    double fx_, fy_, cx_, cy_;
    StandardDistortion d_;
    std::vector<std::int32_t> index_;
    struct Run {
        std::uint32_t dst, length;
        std::int32_t src;  // -1: zero fill
    };
    std::vector<Run> runs_;
    std::vector<float> ray_x_, ray_y_;
    std::size_t outside_ = 0;
    double max_shift_px_ = 0, max_residual_px_ = 0;
};

// A point back-projected with the pinhole intrinsics from an RGB pixel,
// x = (u - cx) z / fx, y = (v - cy) z / fy, is re-derived onto that pixel and
// back-projected through the registration's undistortion instead (float
// round-off of the recovered pixel ~1e-4 px). Returns false, leaving x and y
// unchanged, when z is not positive or the undistortion does not converge.
bool undistort_pinhole_point(const RgbDepthRegistration& registration, double fx, double fy, double cx, double cy,
                           float& x, float& y, float z) noexcept;

}  // namespace perception
