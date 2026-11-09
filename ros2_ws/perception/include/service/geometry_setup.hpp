/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: geometry_setup.hpp
 * Brief: Configured-geometry Profile setup for the formal service (19 section
 *        3.1 precompute, section 11 PSC-7, reply 2026-09-17 Q12).
 *
 * Description:
 * Only used when perception.extrinsic_base_cam carries a transform. The
 * transform may be uncertified (calibrated=false, Q12 "compute and publish");
 * certification is solely the CalibrationVerdict.
 *
 * Startup order:
 *   1. prevalidate_geometry_setup() before any device access: parses
 *      perception.profile + perception.native_geometry_setup against a
 *      synthetic 640x400 metadata purely to surface key errors as PSC-2.
 *      Nothing from that synthetic parse is used for output.
 *   2. build_profile_geometry() on the FIRST native depth frame's metadata (the
 *      actual intrinsics/distortion/unit of the bound stream):
 *        PSC-2  configuration rejected with the real metadata
 *        PSC-3  camera projection metadata unsupported (ray table not built)
 *        PSC-7  precomputed table self-check: no prior-plane ray intersection
 *               within range_max_m (e.g. a transform written upside down),
 *               or the table/grid could not be constructed.
 *        GEOM-PRIOR-HEIGHT  extension self-check beyond the PSC-7 text: the
 *               optical centre's signed height n.t + d above the prior plane
 *               must be positive (reply batch 3, B3-4).
 * Health ROI: calibrated -> the precomputed prior-plane ROI mask
 * (CalibratedStaticMask); otherwise LowerHalfUncalibrated, which the Status
 * fields pair with degraded_reasons roi_fallback (11 section 3.1B.3).
 */
#pragma once

#include "modules/native_geometry.hpp"
#include "modules/normal_grid.hpp"
#include "output/rns_depth_health.hpp"
#include "common/rns_geometry_config.hpp"
#include "service/calibration_record.hpp"
#include "service/service_config.hpp"

#include <functional>
#include <memory>
#include <vector>

namespace perception::service {

struct GeometryFactory {
    // Builds the ray table for the exact binding. Throw on unsupported camera
    // projection metadata (reported as PSC-3). The returned owner is opaque.
    std::function<std::shared_ptr<void>(const geometry::ProjectionBinding&)> prepare_rays;
    // Builds NativeGeometry from the prepared rays (failures reported as PSC-7).
    std::function<std::unique_ptr<geometry::NativeGeometry>(const std::shared_ptr<void>&,
                                                            const geometry::GeometryOptions&)> make_geometry;
};

struct ProfileGeometry {
    config::NativeGeometrySetup setup;
    std::shared_ptr<void> rays;
    std::unique_ptr<geometry::NativeGeometry> geometry;
    std::unique_ptr<geometry::NormalGrid> grid;
    std::vector<std::uint8_t> health_mask;  // non-empty only when calibrated
    rns::HealthRoiView health_roi;          // points into health_mask when calibrated
    bool calibrated = false;
    std::size_t roi_pixels = 0;
};

// Throws StartupCheckError("PSC-2", key, why). No device access.
void prevalidate_geometry_setup(const ServiceConfig& config);

// Throws StartupCheckError (PSC-2 / PSC-3 / PSC-7) as documented above, or
// std::logic_error when called without a configured transform.
std::unique_ptr<ProfileGeometry> build_profile_geometry(const ServiceConfig& config,
                                                      const native_depth::Metadata& native,
                                                      const CalibrationVerdict& verdict,
                                                      const GeometryFactory& factory);

}  // namespace perception::service
