/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: t_color_projection.cpp
 * Brief: CalibratedColor validation and projection (see t_color_projection.hpp).
 *
 * Description:
 * The projection is the forward direction of RgbDepthRegistration's model: rigid transform,
 * perspective division, distort_normalized (shared code), intrinsics, then the +0.5 that turns the
 * pixel-centre convention into NormalTraversability's floor sampling.
 */
#include "modules/t_color_projection.hpp"

#include <algorithm>
#include <cmath>

namespace perception::geometry {

namespace {

bool finite(Vec3 p) noexcept { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }

}  // namespace

bool valid_calibrated_color(const CalibratedColor& c) noexcept {
    if (!finite(c.t_color_base) || !std::isfinite(c.fx) || !(c.fx > 0) || !std::isfinite(c.fy) || !(c.fy > 0) ||
        !std::isfinite(c.cx) || !std::isfinite(c.cy) || !std::isfinite(c.max_r2) || !(c.max_r2 > 0))
        return false;
    for (const double k : {c.distortion.k1, c.distortion.k2, c.distortion.k3, c.distortion.p1, c.distortion.p2})
        if (!std::isfinite(k)) return false;
    const auto& r = c.R_color_base;
    for (const double x : r)
        if (!std::isfinite(x)) return false;
    // Orthonormal rows and det +1, with the tolerance rns::valid_rigid_transform uses (the SDK
    // extrinsic arrives as float).
    for (std::size_t a = 0; a < 3; ++a)
        for (std::size_t b = 0; b < 3; ++b) {
            double dot = 0;
            for (std::size_t k = 0; k < 3; ++k) dot += r[a * 3 + k] * r[b * 3 + k];
            if (std::abs(dot - (a == b ? 1.0 : 0.0)) > 1e-6) return false;
        }
    const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
                       r[2] * (r[3] * r[7] - r[4] * r[6]);
    return std::abs(det - 1.0) <= 1e-6;
}

bool project_calibrated_color(const void* context, Vec3 p, double& u, double& v) noexcept {
    u = v = 0;
    if (context == nullptr || !finite(p)) return false;
    const auto& c = *static_cast<const CalibratedColor*>(context);
    const auto& r = c.R_color_base;
    const Vec3 q{r[0] * p.x + r[1] * p.y + r[2] * p.z + c.t_color_base.x,
                 r[3] * p.x + r[4] * p.y + r[5] * p.z + c.t_color_base.y,
                 r[6] * p.x + r[7] * p.y + r[8] * p.z + c.t_color_base.z};
    return project_calibrated_camera(context, q, u, v);
}

bool project_calibrated_camera(const void* context, Vec3 q, double& u, double& v) noexcept {
    u = v = 0;
    if (context == nullptr) return false;
    const auto& c = *static_cast<const CalibratedColor*>(context);
    if (!finite(q) || !(q.z > 0)) return false;
    const double x = q.x / q.z, y = q.y / q.z;
    if (!(x * x + y * y <= c.max_r2)) return false;  // beyond the image corners (or NaN)
    double xd = 0, yd = 0;
    distort_normalized(c.distortion, x, y, xd, yd);
    const double pu = c.fx * xd + c.cx + 0.5, pv = c.fy * yd + c.cy + 0.5;
    if (!std::isfinite(pu) || !std::isfinite(pv)) return false;
    u = pu;
    v = pv;
    return true;
}

bool corner_max_r2(const RgbDepthRegistration& registration, double& max_r2) noexcept {
    max_r2 = 0;
    const double w = registration.width(), h = registration.height();
    const double corners[4][2] = {{-0.5, -0.5}, {w - 0.5, -0.5}, {-0.5, h - 0.5}, {w - 0.5, h - 0.5}};
    for (const auto& corner : corners) {
        double x = 0, y = 0;
        if (!registration.undistort_pixel(corner[0], corner[1], x, y)) return false;
        max_r2 = std::max(max_r2, x * x + y * y);
    }
    return std::isfinite(max_r2) && max_r2 > 0;
}

}  // namespace perception::geometry
