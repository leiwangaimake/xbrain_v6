/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rgb_depth_registration.cpp
 * Brief: Per-pixel RGB -> aligned-depth index table, undistorted rays and the run-based gather.
 *
 * Description:
 * What problem this solves. Implements rgb_depth_registration.hpp. The table is built once per
 * capture session: for every RGB pixel centre, undistort it (Newton iteration on the
 * Brown-Conrady model), project it with the colour pinhole K onto the aligned raster, and
 * keep the nearest aligned pixel. Per frame only the precomputed gather runs.
 *
 * Why the model must be invertible everywhere. A pixel where the undistortion fails would get
 * no depth, silently; so construction refuses the whole model instead (the capture then reports
 * the registration as unavailable and objects stay unlocatable), and every accepted pixel has a
 * forward residual below 1e-6 px.
 */
#include "modules/rgb_depth_registration.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace perception {
namespace {
// Newton stops at a relative step of 1e-15 (double precision) or 50 iterations.
constexpr int kMaxIterations = 50;
constexpr double kStep = 1e-15;

// Jacobian of distort_normalized at (x, y).
void jacobian(const StandardDistortion& d, double x, double y, double& a, double& b, double& c, double& e) noexcept {
    const double r2 = x * x + y * y;
    const double radial = 1 + d.k1 * r2 + d.k2 * r2 * r2 + d.k3 * r2 * r2 * r2;
    const double dradial = d.k1 + 2 * d.k2 * r2 + 3 * d.k3 * r2 * r2;  // d(radial)/d(r2)
    // xd = x*radial + 2 p1 x y + p2 (r2 + 2 x^2); yd = y*radial + p1 (r2 + 2 y^2) + 2 p2 x y
    a = radial + x * dradial * 2 * x + 2 * d.p1 * y + d.p2 * (2 * x + 4 * x);          // dxd/dx
    b = x * dradial * 2 * y + 2 * d.p1 * x + d.p2 * (2 * y);                            // dxd/dy
    c = y * dradial * 2 * x + d.p1 * (2 * x) + 2 * d.p2 * y;                            // dyd/dx
    e = radial + y * dradial * 2 * y + d.p1 * (2 * y + 4 * y) + 2 * d.p2 * x;          // dyd/dy
}

// Invert the distortion: find undistorted (x, y) with distort(x, y) = (xd, yd)
// by Newton's method from (xd, yd). A non-positive Jacobian means the model
// folds over here and has no unique inverse: failure.
bool solve(const StandardDistortion& d, double xd, double yd, double& x, double& y, double& det_out) noexcept {
    x = xd;
    y = yd;
    for (int it = 0; it < kMaxIterations; ++it) {
        double fx, fy;
        distort_normalized(d, x, y, fx, fy);
        const double rx = fx - xd, ry = fy - yd;
        double a, b, c, e;
        jacobian(d, x, y, a, b, c, e);
        const double det = a * e - b * c;
        det_out = det;
        if (!(det > 0) || !std::isfinite(det)) return false;
        const double dx = (e * rx - b * ry) / det, dy = (-c * rx + a * ry) / det;
        x -= dx;
        y -= dy;
        if (!std::isfinite(x) || !std::isfinite(y)) return false;
        if (std::fabs(dx) + std::fabs(dy) <= kStep * (1 + std::fabs(x) + std::fabs(y))) break;
    }
    double a, b, c, e;
    jacobian(d, x, y, a, b, c, e);
    det_out = a * e - b * c;
    return det_out > 0 && std::isfinite(det_out);
}
}  // namespace

void distort_normalized(const StandardDistortion& d, double x, double y, double& xd, double& yd) noexcept {
    const double r2 = x * x + y * y;
    const double radial = 1 + d.k1 * r2 + d.k2 * r2 * r2 + d.k3 * r2 * r2 * r2;
    xd = x * radial + 2 * d.p1 * x * y + d.p2 * (r2 + 2 * x * x);
    yd = y * radial + d.p1 * (r2 + 2 * y * y) + 2 * d.p2 * x * y;
}

RgbDepthRegistration::RgbDepthRegistration(int width, int height, double fx, double fy, double cx, double cy,
                                           const StandardDistortion& d)
    : width_(width), height_(height), fx_(fx), fy_(fy), cx_(cx), cy_(cy), d_(d) {
    if (width < 1 || height < 1 || width > 8192 || height > 8192 || !(fx > 0) || !(fy > 0) || !std::isfinite(fx) ||
        !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy))
        throw std::invalid_argument("rgb depth registration: invalid raster or intrinsics");
    for (double k : {d.k1, d.k2, d.k3, d.p1, d.p2})
        if (!std::isfinite(k)) throw std::invalid_argument("rgb depth registration: non-finite distortion");
    const auto pixels = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    index_.resize(pixels);
    ray_x_.resize(pixels);
    ray_y_.resize(pixels);
    // Per RGB pixel centre: undistort, verify by re-distorting, keep the ray,
    // then the nearest aligned pixel of q = K * undistort(p) (or outside).
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            const std::size_t i = static_cast<std::size_t>(v) * static_cast<std::size_t>(width) + static_cast<std::size_t>(u);
            const double xd = (u - cx) / fx, yd = (v - cy) / fy;
            double x, y, det;
            if (!solve(d, xd, yd, x, y, det))
                throw std::invalid_argument("rgb depth registration: model not invertible at pixel " + std::to_string(u) +
                                            "," + std::to_string(v));
            double fxd, fyd;
            distort_normalized(d, x, y, fxd, fyd);
            const double residual = std::max(std::fabs(fxd - xd) * fx, std::fabs(fyd - yd) * fy);
            if (!(residual <= 1e-6))
                throw std::invalid_argument("rgb depth registration: forward residual " + std::to_string(residual) +
                                            " px at pixel " + std::to_string(u) + "," + std::to_string(v));
            max_residual_px_ = std::max(max_residual_px_, residual);
            ray_x_[i] = static_cast<float>(x);
            ray_y_[i] = static_cast<float>(y);
            const double qu = fx * x + cx, qv = fy * y + cy;
            max_shift_px_ = std::max(max_shift_px_, std::hypot(qu - u, qv - v));
            const long iu = std::lround(qu), iv = std::lround(qv);
            if (iu < 0 || iv < 0 || iu >= width || iv >= height) {
                index_[i] = -1;
                ++outside_;
            } else {
                index_[i] = static_cast<std::int32_t>(iv * width + iu);
            }
        }
    }
    // Runs: maximal stretches where the index advances by exactly one (or all
    // outside). Rows are kept separate so a run never wraps a raster row.
    for (int v = 0; v < height; ++v) {
        const std::size_t row = static_cast<std::size_t>(v) * static_cast<std::size_t>(width);
        std::size_t start = 0;
        while (start < static_cast<std::size_t>(width)) {
            const std::int32_t first = index_[row + start];
            std::size_t len = 1;
            while (start + len < static_cast<std::size_t>(width)) {
                const std::int32_t next = index_[row + start + len];
                if (first < 0 ? next >= 0 : next != first + static_cast<std::int32_t>(len)) break;
                ++len;
            }
            runs_.push_back({static_cast<std::uint32_t>(row + start), static_cast<std::uint32_t>(len), first < 0 ? -1 : first});
            start += len;
        }
    }
}

// Per frame: copy each run with one memcpy (or zero-fill an outside run).
void RgbDepthRegistration::apply(const std::uint16_t* aligned, std::uint16_t* out) const noexcept {
    for (const Run& r : runs_) {
        if (r.src < 0)
            std::memset(out + r.dst, 0, r.length * sizeof(std::uint16_t));
        else
            std::memcpy(out + r.dst, aligned + r.src, r.length * sizeof(std::uint16_t));
    }
}

bool RgbDepthRegistration::undistort_pixel(double u, double v, double& x, double& y) const noexcept {
    if (!std::isfinite(u) || !std::isfinite(v)) return false;
    double det;
    return solve(d_, (u - cx_) / fx_, (v - cy_) / fy_, x, y, det);
}

// Re-derive the RGB pixel of a pinhole back-projection and back-project it
// again through the real (distorted) model (see the header).
bool undistort_pinhole_point(const RgbDepthRegistration& registration, double fx, double fy, double cx, double cy,
                           float& x, float& y, float z) noexcept {
    if (!(z > 0) || !(fx > 0) || !(fy > 0)) return false;
    const double u = static_cast<double>(x) * fx / z + cx;
    const double v = static_cast<double>(y) * fy / z + cy;
    double ux = 0, uy = 0;
    if (!registration.undistort_pixel(u, v, ux, uy)) return false;
    x = static_cast<float>(ux * z);
    y = static_cast<float>(uy * z);
    return true;
}

}  // namespace perception
