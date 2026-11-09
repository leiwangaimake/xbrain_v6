/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_geometry_config.hpp
 * Brief: Resolved-config -> NativeGeometry/NormalGrid setup and the base <- camera transform.
 *
 * Description:
 * What problem this solves. The geometry owners take plain option structs; this is the one
 * place that maps configuration keys onto them (contract keys in perception.profile and
 * perception.extrinsic_base_cam, internal fit knobs in perception.native_geometry_setup), with
 * the camera metadata taken from the real bound stream. Used by the service's geometry setup
 * (PSC-2 prevalidation and first-frame build) and by the Objects geometry setup.
 */
#pragma once

#include "modules/native_geometry.hpp"
#include "modules/normal_grid.hpp"

#include <array>
#include <string>

namespace perception::config {

// Explicit normal-fit, G-only setup. No fallback policy, transport, clock,
// segmentation, owner thread or calibrated=true is created by this loader.
struct NativeGeometrySetup {
    geometry::ProjectionBinding projection;
    geometry::GeometryOptions geometry;
    geometry::GridOptions grid;
};

// Parse an already-resolved YAML product with the pinned upstream common
// yaml_lite reader. Numeric flow/block vectors at the three documented local
// boundaries are adapted before parsing. No defaults, ${...} resolution,
// file reads, SDK access or device access. Throws with a dotted key path.
//
// Metadata comes from the bound native stream/record, never generic camera
// defaults. R/t are read only from perception.extrinsic_base_cam. Internal
// fitting knobs live in perception.native_geometry_setup (schema_version: 1),
// not in the upstream profile contract. Angle min/step are explicitly rounded
// once to float32 for the existing wire domain, then widened for geometry.
//
// Successful parsing is numeric configuration validation only. The SDK ray
// factory + NativeGeometry + NormalGrid still perform their own admission
// checks, including actual prior-plane/ray intersections. The calib_record,
// calib_sha256 and residual_max_m keys are accepted here but not read: in the
// service, ServiceConfig (read_extrinsic) has already validated them and
// service::CalibrationVerifier reads them. Nothing here sets
// extrinsic_calibrated=true.
NativeGeometrySetup parse_native_geometry_setup(
    const std::string& resolved_yaml, const native_depth::Metadata& native_profile);

// perception.extrinsic_base_cam alone: base_link <- native depth optical, with
// the same key and unit-quaternion rules as parse_native_geometry_setup (used by
// the object geometry path, which has no native ray binding). Throws with a
// dotted key path.
struct BaseFromCamera {
    std::array<double, 9> R{};
    geometry::Vec3 t;
};
BaseFromCamera parse_base_from_camera(const std::string& resolved_yaml);

}  // namespace perception::config
