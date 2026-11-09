/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: orbbec_native_rays.hpp
 * Brief: Setup-only native-depth projection adapter.
 *
 * Description:
 * No Context, device, frame, Pipeline, calibration acquisition, or production configuration defaults.
 */

#pragma once

#include "modules/native_geometry.hpp"
#include <memory>

namespace perception::geometry {

// Builds optical-Z rays from the supplied *native depth* parameters with the
// pinned Orbbec SDK. This is numerical binding, NOT an extrinsic calibration
// certificate. The owner must re-create it on camera/installation/calibration
// changes even when the camera reports numerically identical intrinsics.
// The current scope is 640x400 Brown-Conrady (SDK enum 3, k4..k6 zero).
// Unsupported models and a single invalid SDK ray reject the entire table;
// there is no pinhole fallback, invalid-pixel filling or model inference.
class OrbbecNativeRays {
public:
    explicit OrbbecNativeRays(const ProjectionBinding&);
    OrbbecNativeRays(const OrbbecNativeRays&) = delete;
    OrbbecNativeRays& operator=(const OrbbecNativeRays&) = delete;

    const ProjectionBinding& binding() const noexcept { return binding_; }
    const std::vector<Vec3>& rays() const noexcept { return rays_; }
    // Only this factory path prevents independently substituting a binding or
    // a ray table. NativeGeometry's lower-level injected-test API still exists.
    // NativeGeometry copies both inputs; it outlives this setup object safely.
    std::unique_ptr<NativeGeometry> make_geometry(const GeometryOptions&) const;

private:
    const ProjectionBinding binding_;
    std::vector<Vec3> rays_;
};
}  // namespace perception::geometry
