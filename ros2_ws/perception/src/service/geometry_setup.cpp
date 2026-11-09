/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: geometry_setup.cpp
 * Brief: Startup prevalidation and first-frame construction of the Profile geometry owners.
 *
 * Description:
 * What problem this solves. Implements geometry_setup.hpp. Every failure is turned into a
 * StartupCheckError naming the check (PSC-2/3/7, GEOM-PRIOR-HEIGHT) and the config path, so the
 * service exits with an actionable message instead of a bare exception text.
 */
#include "service/geometry_setup.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace perception::service {
namespace {
// Geometry parser messages are "dotted.path: why"; the pinned yaml_lite reader
// uses "missing config key: dotted.path". Report whichever side is a path.
StartupCheckError as_check(const char* check, const std::exception& e) {
    const std::string what = e.what();
    const auto colon = what.find(": ");
    if (colon != std::string::npos && colon > 0) {
        const auto head = what.substr(0, colon), tail = what.substr(colon + 2);
        if (head.find(' ') == std::string::npos) return StartupCheckError(check, head, tail);
        if (!tail.empty() && tail.find(' ') == std::string::npos) return StartupCheckError(check, tail, head);
    }
    return StartupCheckError(check, "perception", what);
}

// Plausible 640x400 metadata used ONLY to surface key errors before the camera
// is opened; nothing derived from it reaches the output.
native_depth::Metadata synthetic_native() {
    native_depth::Metadata m;
    m.intrinsics = {300.0F, 300.0F, 320.0F, 200.0F, 640U, 400U};
    m.distortion.reported = true;
    m.distortion.model = 3;
    m.unit_m = 0.001F;
    return m;
}
}  // namespace

// Startup, before the camera is opened: a configuration error should stop the service at once
// with its key path, not after the capture session is up. The real metadata is only known from
// the first frame, so this parse uses synthetic metadata and only surfaces key/value errors.
void prevalidate_geometry_setup(const ServiceConfig& config) {
    // Without a configured transform the service runs the unconfigured Profile.
    if (config.contract.extrinsic.mode != ExtrinsicMode::TransformConfigured) return;
    try {
        (void)config::parse_native_geometry_setup(config.contract_yaml, synthetic_native());
    } catch (const std::exception& e) {
        throw as_check("PSC-2", e);
    }
}

std::unique_ptr<ProfileGeometry> build_profile_geometry(const ServiceConfig& config,
                                                      const native_depth::Metadata& native,
                                                      const CalibrationVerdict& verdict,
                                                      const GeometryFactory& factory) {
    // Calling this without a configured transform, or with an incomplete factory, is a program
    // error of the caller (logic_error), not a configuration problem to report to the operator.
    if (config.contract.extrinsic.mode != ExtrinsicMode::TransformConfigured ||
        verdict.code() == CalibrationCode::NoTransform)
        throw std::logic_error("configured-geometry setup without a configured transform");
    // The factory injects the camera-specific ray model (OrbbecNativeRays in the service), so the
    // same construction runs in host tests with synthetic rays.
    if (!factory.prepare_rays || !factory.make_geometry) throw std::logic_error("geometry factory incomplete");
    // PSC-2 with the real metadata of the bound stream.
    auto out = std::make_unique<ProfileGeometry>();
    try {
        out->setup = config::parse_native_geometry_setup(config.contract_yaml, native);
    } catch (const std::exception& e) {
        throw as_check("PSC-2", e);
    }
    // PSC-3: the camera's projection model must be supported to build rays.
    try {
        out->rays = factory.prepare_rays(out->setup.projection);
    } catch (const std::exception& e) {
        throw StartupCheckError("PSC-3", "native_depth.metadata",
                                std::string("camera projection metadata unsupported: ") + e.what());
    }
    if (!out->rays) throw StartupCheckError("PSC-3", "native_depth.metadata", "ray table not built");
    // EXTENSION self-check (reply batch 3, B3-4), NOT the literal PSC-7 text:
    // the signed height of the optical centre above the prior plane,
    // n . t + d, must be positive. A camera at/below the plane still gets
    // positive z_exp for upward rays. A failure may come from the prior, units,
    // the coordinate origin or the transform direction, so all are reported.
    const auto& prior = out->setup.geometry.prior;
    const auto& t = out->setup.projection.t_base_camera;
    const double height = prior.normal.x * t.x + prior.normal.y * t.y + prior.normal.z * t.z + prior.d;
    if (!(height > 0)) {
        char why[256];
        std::snprintf(why, sizeof(why),
                      "optical centre not above the prior plane: n=(%.9g,%.9g,%.9g) d=%.9g t=(%.9g,%.9g,%.9g) signed_height=%.9g",
                      prior.normal.x, prior.normal.y, prior.normal.z, prior.d, t.x, t.y, t.z, height);
        throw StartupCheckError("GEOM-PRIOR-HEIGHT", "perception.extrinsic_base_cam.translation", why);
    }
    // Coverage tickets (runtime switch coverage_tickets, 2026-10-27): "fitted" makes the grid draw
    // each frame's tickets from its fitted plane; unset or "prior" keeps the static prior tickets.
    if (const char* tickets = std::getenv("PERCEPTION_COVERAGE_TICKETS")) {
        const std::string value(tickets);
        if (value != "prior" && value != "fitted")
            throw StartupCheckError("ENV", "PERCEPTION_COVERAGE_TICKETS", "must be prior or fitted");
        out->setup.grid.fitted_tickets = value == "fitted";
    }
    // PSC-7: the precomputed tables and grid; their constructors self-check.
    try {
        out->geometry = factory.make_geometry(out->rays, out->setup.geometry);
        if (!out->geometry) throw std::runtime_error("no geometry");
        out->grid = std::make_unique<geometry::NormalGrid>(*out->geometry, out->setup.grid);
    } catch (const std::exception& e) {
        throw StartupCheckError("PSC-7", "perception.extrinsic_base_cam",
                                std::string("precomputed table self-check failed: ") + e.what());
    }
    out->roi_pixels = out->geometry->roi_pixels();  // > 0: NativeGeometry rejects an empty table
    // Depth-health ROI: the prior-plane mask only when calibrated; otherwise the
    // lower-half fallback (Status then reports roi_fallback).
    out->calibrated = verdict.calibrated();
    if (out->calibrated) {
        out->health_mask = out->geometry->prior_roi();
        out->health_roi.kind = rns::HealthRoiKind::CalibratedStaticMask;
        out->health_roi.mask = out->health_mask.data();
        out->health_roi.mask_pixels = out->health_mask.size();
        out->health_roi.width = static_cast<std::uint32_t>(geometry::kNativeWidth);
        out->health_roi.height = static_cast<std::uint32_t>(geometry::kNativeHeight);
    } else {
        out->health_roi = rns::HealthRoiView{};
    }
    return out;
}

}  // namespace perception::service
