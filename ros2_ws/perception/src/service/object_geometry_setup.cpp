/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: object_geometry_setup.cpp
 * Brief: Build the Objects geometry builder for one capture session.
 *
 * Description:
 * What problem this solves. Implements object_geometry_setup.hpp. Object points are measured on
 * the colour raster (the depth is aligned to colour), but the configured extrinsic places the
 * depth camera in base_link. This function chains the two into base <- colour and builds the
 * ray table of the registered colour raster, once per session.
 *
 * Failure split (B4-3). Missing session calibration or registration, or an invalid SDK extrinsic,
 * is a capability gap: ObjectGeometryCapabilityError, the service runs on with unlocatable
 * objects. Anything else is a configuration or program fault and propagates.
 */
#include "service/object_geometry_setup.hpp"

#include <utility>

namespace perception::service {

ObjectGeometryReady setup_object_geometry(const ObjectGeometrySession& session, const std::string& resolved_yaml,
                                        const rns::ObjectGeometryOptions& options) {
    // Capability checks for this session.
    if (!session.present)
        throw ObjectGeometryCapabilityError("no session RGB-D calibration (software D2C not active)");
    if (!session.registration)
        throw ObjectGeometryCapabilityError("rgb-depth registration " +
                                            (session.registration_status.empty() ? std::string("absent")
                                                                                 : session.registration_status));
    // The SDK's measured depth -> colour extrinsic of this session.
    rns::RigidTransform colour_from_depth;
    for (std::size_t i = 0; i < 9; ++i) colour_from_depth.R[i] = session.depth_to_colour_R[i];
    for (std::size_t i = 0; i < 3; ++i) colour_from_depth.t[i] = session.depth_to_colour_t_m[i];
    if (!rns::valid_rigid_transform(colour_from_depth))
        throw ObjectGeometryCapabilityError("invalid SDK depth->colour extrinsic for this session");

    // Configuration and program faults propagate unchanged: extrinsic_base_cam
    // was prevalidated at startup, and builder options are constants.
    const auto base = config::parse_base_from_camera(resolved_yaml);
    rns::RigidTransform base_from_depth;
    base_from_depth.R = base.R;
    base_from_depth.t = {base.t.x, base.t.y, base.t.z};
    // base <- colour = (base <- depth) o (depth <- colour).
    ObjectGeometryReady ready;
    ready.base_from_colour = rns::compose(base_from_depth, rns::inverse(colour_from_depth));
    // Optical rays of the registered colour raster the aligned depth lives on.
    rns::OpticalRayTable rays;
    rays.width = static_cast<std::uint32_t>(session.registration->width());
    rays.height = static_cast<std::uint32_t>(session.registration->height());
    rays.x = session.registration->ray_x();
    rays.y = session.registration->ray_y();
    try {
        ready.builder = std::make_unique<rns::ObjectGeometryBuilder>(std::move(rays), ready.base_from_colour, options);
    } catch (const std::invalid_argument& e) {
        // Not a capability question: the inputs were validated above.
        throw std::logic_error(std::string("object geometry builder rejected validated inputs: ") + e.what());
    }
    // float copy of the rotation for the raw velocity rotation in Objects.
    for (std::size_t i = 0; i < 9; ++i) ready.base_from_colour_R[i] = static_cast<float>(ready.base_from_colour.R[i]);
    ready.registration = session.registration;
    return ready;
}

}  // namespace perception::service
