/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: object_geometry_setup.hpp
 * Brief: Objects measured-geometry capability setup (batch-4 reply B4-3).
 *
 * Description:
 * Three outcomes, kept apart:
 *   - Ready: builder + registration for this capture session.
 *   - ObjectGeometryCapabilityError: the Objects coordinate-adaptation
 *     capability is not available for this session (no session RGB-D
 *     calibration, RGB <- aligned depth registration not established for an
 *     unmeasured configuration, or an invalid SDK depth->colour extrinsic). The
 *     service keeps publishing; objects stay unlocatable and Status carries the
 *     free diagnostic reason object_geometry_unavailable.
 *   - any other exception: a configuration / program fault handled by the
 *     existing error strategy (never downgraded to "unsupported").
 * No SDK, OpenCV or device access.
 */

#pragma once

#include "common/rns_geometry_config.hpp"
#include "modules/rgb_depth_registration.hpp"
#include "output/rns_object_geometry.hpp"

#include <array>
#include <memory>
#include <stdexcept>
#include <string>

namespace perception::service {

class ObjectGeometryCapabilityError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// The part of the capture session calibration the setup needs (copied from
// yolo::RgbdCalibration by the service; plain types only).
struct ObjectGeometrySession {
    bool present = false;                         // the capture reported a session calibration
    std::array<float, 9> depth_to_colour_R{};     // p_c = R p_d + t (measured direction)
    std::array<float, 3> depth_to_colour_t_m{};
    std::shared_ptr<const RgbDepthRegistration> registration;
    std::string registration_status;              // "active" or "unavailable: ..."
};

struct ObjectGeometryReady {
    std::unique_ptr<rns::ObjectGeometryBuilder> builder;
    std::shared_ptr<const RgbDepthRegistration> registration;
    rns::RigidTransform base_from_colour;
    std::array<float, 9> base_from_colour_R{};
};

// Throws ObjectGeometryCapabilityError (capability not available), or any
// other exception for configuration / program faults (for example an
// extrinsic_base_cam that no longer parses after startup prevalidation, or
// invalid builder options).
ObjectGeometryReady setup_object_geometry(const ObjectGeometrySession& session, const std::string& resolved_yaml,
                                        const rns::ObjectGeometryOptions& options);

}  // namespace perception::service
