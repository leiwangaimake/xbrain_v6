/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: t_color_projection.hpp
 * Brief: The factory-calibrated colour projection T uses to read the class-92 mask at a ground point.
 *
 * Description:
 * 19 section 3.3: project with the factory depth<->colour intrinsics and extrinsics, never by
 * aspect-ratio scaling.
 * What it computes. A current base-frame ground point is moved into the colour camera frame
 * (colour <- base, rigid), divided by depth, distorted with the colour camera's Brown-Conrady
 * model (k1 k2 k3 p1 p2, OpenCV convention, the same model and code RgbDepthRegistration inverts
 * for the Objects path) and scaled with the colour intrinsics. So T and Objects use one pixel
 * correspondence.
 *
 * Pixel-centre convention. RgbDepthRegistration treats integer coordinates as pixel centres
 * (pixel i covers [i - 0.5, i + 0.5)); NormalTraversability samples with floor(u). The projector
 * therefore returns u + 0.5, v + 0.5, so floor() picks the nearest pixel. Which convention the
 * SDK's cx, cy were calibrated in is not verified; T follows the registration, so both paths agree.
 *
 * Folding guard. The polynomial model is only trusted up to the image corners: a point whose
 * normalized radius exceeds the corners' largest one is outside the image and is "no sample",
 * even if the polynomial would fold it back inside.
 *
 * Pure computation: no SDK, OpenCV, clock or allocation.
 */

#pragma once

#include "modules/rgb_depth_registration.hpp"
#include "modules/traversability.hpp"

#include <array>

namespace perception::geometry {

struct CalibratedColor {
    // colour <- base: p_colour = R * p_base + t.
    std::array<double, 9> R_color_base{};
    Vec3 t_color_base;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    StandardDistortion distortion;
    // Largest squared normalized radius the image reaches (its corners), > 0.
    double max_r2 = 0;
};

// Proper rotation (orthonormal, det +1 within 1e-6, as rns::valid_rigid_transform), finite t,
// positive finite focal lengths, finite centre and coefficients, positive finite max_r2.
bool valid_calibrated_color(const CalibratedColor&) noexcept;

// ColorProjection callback (context: const CalibratedColor*). False when the point is not finite,
// at or behind the camera plane, or beyond max_r2.
bool project_calibrated_color(const void* context, Vec3 base_point, double& u, double& v) noexcept;
// The part of project_calibrated_color after the rigid transform, for the split form of
// ColorProjection: camera_point = R_color_base * base_point + t_color_base. Same checks and
// arithmetic as project_calibrated_color from that point on.
bool project_calibrated_camera(const void* context, Vec3 camera_point, double& u, double& v) noexcept;

// Largest squared normalized radius over the four image corners (outer pixel edges) of a
// registration's raster, through its own undistortion. False if a corner does not converge.
bool corner_max_r2(const RgbDepthRegistration& registration, double& max_r2) noexcept;

}  // namespace perception::geometry
