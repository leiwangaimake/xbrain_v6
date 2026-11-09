/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: t_projection_setup.cpp
 * Brief: setup_t_projection (see t_projection_setup.hpp).
 *
 * Description:
 * Capability checks first (each names the missing piece), then the transform chain with the same
 * rns::compose / rns::inverse the Objects path uses, the corner radius bound from the session's
 * own registration, and a final validation of the whole projection.
 */
#include "service/t_projection_setup.hpp"

#include "common/rns_geometry_config.hpp"
#include "output/rns_object_geometry.hpp"

#include <cmath>
#include <string>

namespace perception::service {

namespace {

// SDK OBCameraDistortionModel value of BROWN_CONRADY_K6, the model the registration gate accepts
// (with k4..k6 = 0, i.e. the standard k1 k2 k3 p1 p2 model).
constexpr int kBrownConradyK6 = 4;

}  // namespace

TProjectionReady setup_t_projection(const TProjectionSession& session, const std::string& resolved_yaml,
                                    std::uint64_t projection_revision) {
    // Without the capture session's own calibration there is no colour model at all (the
    // capture only stores it when software D2C is active); T is then unavailable, not faulty.
    if (!session.present) throw TProjectionCapabilityError("no session RGB-D calibration (software D2C not active)");
    // The Objects registration gate: T uses the colour model only where it was verified.
    if (!session.registration)
        throw TProjectionCapabilityError("rgb-depth registration " +
                                         (session.registration_status.empty() ? std::string("absent")
                                                                              : session.registration_status));
    // The class-92 mask lives on the 1280x800 colour raster (19 section 13.1); the session
    // intrinsics and the registration must describe exactly that raster, or the projected
    // pixel would index a different image.
    if (session.width != 1280 || session.height != 800 || session.registration->width() != 1280 ||
        session.registration->height() != 800)
        throw TProjectionCapabilityError("colour raster is not 1280x800");
    // Only the k1 k2 k3 p1 p2 polynomial is implemented (and verified by the registration gate);
    // a model with k4..k6 would be evaluated wrongly, so it is a capability gap.
    const auto& k = session.distortion;
    if (session.distortion_model != kBrownConradyK6 || k[3] != 0.0F || k[4] != 0.0F || k[5] != 0.0F)
        throw TProjectionCapabilityError("colour distortion model " + std::to_string(session.distortion_model) +
                                         " with k4..k6 set is not implemented");
    // The SDK's measured depth -> colour extrinsic of this session (float, metres). A rotation
    // that is not orthonormal would distort every projection, so it is checked like the
    // Objects path checks its transforms.
    rns::RigidTransform colour_from_depth;
    for (std::size_t i = 0; i < 9; ++i) colour_from_depth.R[i] = session.depth_to_colour_R[i];
    for (std::size_t i = 0; i < 3; ++i) colour_from_depth.t[i] = session.depth_to_colour_t_m[i];
    if (!rns::valid_rigid_transform(colour_from_depth))
        throw TProjectionCapabilityError("invalid SDK depth->colour extrinsic for this session");

    // Configuration faults propagate unchanged (extrinsic_base_cam was prevalidated at startup).
    const auto base = config::parse_base_from_camera(resolved_yaml);
    rns::RigidTransform base_from_depth;
    base_from_depth.R = base.R;
    base_from_depth.t = {base.t.x, base.t.y, base.t.z};
    // colour <- base = (colour <- depth) o (depth <- base).
    const rns::RigidTransform colour_from_base = rns::compose(colour_from_depth, rns::inverse(base_from_depth));

    // The projection context lives on the heap so the ColorProjection pointers below stay
    // valid when TProjectionReady is moved to its owner.
    TProjectionReady ready;
    ready.color = std::make_unique<geometry::CalibratedColor>();
    auto& c = *ready.color;
    c.R_color_base = colour_from_base.R;
    c.t_color_base = {colour_from_base.t[0], colour_from_base.t[1], colour_from_base.t[2]};
    c.fx = session.fx;
    c.fy = session.fy;
    c.cx = session.cx;
    c.cy = session.cy;
    // SDK order is k1 k2 k3 k4 k5 k6 p1 p2: the tangential terms are entries 6 and 7.
    c.distortion = StandardDistortion{k[0], k[1], k[2], k[6], k[7]};
    // Folding guard: the polynomial is trusted only up to the image corners' normalized radius
    // (t_color_projection.hpp); a model that cannot invert at the corners gives no bound.
    if (!geometry::corner_max_r2(*session.registration, c.max_r2))
        throw TProjectionCapabilityError("colour model does not invert at the image corners");
    if (!geometry::valid_calibrated_color(c))
        throw std::logic_error("T projection rejected validated inputs");  // not a capability question
    // The ColorProjection the T owner borrows: raster size, the revision every mask must carry
    // (a mask built for another projection is refused), and the callback with its context.
    ready.projection.width = 1280;
    ready.projection.height = 800;
    ready.projection.revision = projection_revision;
    ready.projection.context = ready.color.get();
    ready.projection.project = geometry::project_calibrated_color;
    // The same mapping split at its rigid part, so the T owner can precompute it per pixel ray.
    ready.projection.rotation = &c.R_color_base;
    ready.projection.translation = &c.t_color_base;
    ready.projection.project_camera = geometry::project_calibrated_camera;
    return ready;
}

}  // namespace perception::service
