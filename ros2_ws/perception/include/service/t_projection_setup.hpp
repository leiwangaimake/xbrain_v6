/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: t_projection_setup.hpp
 * Brief: Build T's colour projection for one capture session.
 *
 * Description:
 * What it builds. colour <- base = (colour <- depth, the SDK's measured extrinsic of this
 * session) o (depth <- base, the inverse of the configured extrinsic_base_cam), with the colour
 * intrinsics and distortion of the same session, on the 1280x800 colour raster the mask lives on.
 * The configured base <- depth cancels in which colour pixel a depth sample lands on; it only
 * decides which ground cell the sample belongs to.
 *
 * Failure split (user decision 2026-09-30, following B4-3 for Objects). T reuses the Objects
 * registration gate: no session calibration, a registration that did not pass the gate, a colour
 * raster other than 1280x800, a distortion model this projection does not implement, or an invalid
 * SDK extrinsic is a capability gap (TProjectionCapabilityError): the service runs on without T
 * and reports it. Anything else (an extrinsic_base_cam that no longer parses) is a configuration
 * or program fault and propagates.
 */

#pragma once

#include "modules/rgb_depth_registration.hpp"
#include "modules/t_color_projection.hpp"
#include "modules/traversability.hpp"

#include <array>
#include <memory>
#include <stdexcept>
#include <string>

namespace perception::service {

class TProjectionCapabilityError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// The part of the capture session calibration T needs (copied from yolo::RgbdCalibration by the
// service; plain types only).
struct TProjectionSession {
    bool present = false;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    int width = 0, height = 0;
    int distortion_model = 0;                     // raw SDK OBCameraDistortionModel
    std::array<float, 8> distortion{};            // k1 k2 k3 k4 k5 k6 p1 p2
    std::array<float, 9> depth_to_colour_R{};     // p_c = R p_d + t
    std::array<float, 3> depth_to_colour_t_m{};
    std::shared_ptr<const RgbDepthRegistration> registration;
    std::string registration_status;              // "active" or "unavailable: ..."
};

// The projection and the context it points to, kept together (ColorProjection borrows `color`).
struct TProjectionReady {
    std::unique_ptr<geometry::CalibratedColor> color;
    geometry::ColorProjection projection;
};

// Throws TProjectionCapabilityError for a capability gap, anything else for a fault (see above).
TProjectionReady setup_t_projection(const TProjectionSession& session, const std::string& resolved_yaml,
                                    std::uint64_t projection_revision);

}  // namespace perception::service
