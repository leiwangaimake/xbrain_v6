/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: orbbec_native_rays.cpp
 * Brief: Undistorted optical ray table of the native depth raster via the Orbbec SDK.
 *
 * Description:
 * What problem this solves. Implements orbbec_native_rays.hpp. Each native depth pixel needs the
 * direction it looks in, with lens distortion removed; the SDK's XY table gives, per pixel, the
 * x/y of that direction at optical z = 1. Using the SDK's own undistortion (instead of a local
 * formula) keeps the geometry consistent with how the camera itself maps pixels. Runs once at
 * setup, on the first frame's real metadata.
 */
#include "modules/orbbec_native_rays.hpp"
#include <libobsensor/hpp/Utils.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace perception::geometry {
namespace {
// Only the profile this code has been verified for: 640x400, Brown-Conrady.
void validate_profile(const ProjectionBinding& b) {
    const auto& k = b.intrinsics;
    if (k.width != kNativeWidth || k.height != kNativeHeight ||
        !std::isfinite(k.fx) || !std::isfinite(k.fy) || k.fx <= 0 || k.fy <= 0 ||
        !std::isfinite(k.cx) || !std::isfinite(k.cy) ||
        !std::isfinite(b.unit_m) || b.unit_m <= 0 || !b.distortion.reported ||
        b.distortion.model != static_cast<int>(OB_DISTORTION_BROWN_CONRADY))
        throw std::invalid_argument("native rays: unsupported or invalid depth profile");
    for (float c : b.distortion.coefficients)
        if (!std::isfinite(c)) throw std::invalid_argument("native rays: nonfinite distortion");
    // Do not silently interpret K6 coefficients under the observed K3 model.
    for (std::size_t i = 3; i < 6; ++i)
        if (b.distortion.coefficients[i] != 0)
            throw std::invalid_argument("native rays: K6 terms under Brown-Conrady K3 model");
}
}  // namespace

OrbbecNativeRays::OrbbecNativeRays(const ProjectionBinding& binding) : binding_(binding) {
    validate_profile(binding_);
    const auto& k = binding_.intrinsics;
    const auto& c = binding_.distortion.coefficients;
    // Hand the depth intrinsics and distortion to the SDK as its calibration.
    OBCalibrationParam calibration{};
    calibration.intrinsics[OB_SENSOR_DEPTH] = {
        k.fx, k.fy, k.cx, k.cy, static_cast<std::int16_t>(k.width), static_cast<std::int16_t>(k.height)};
    calibration.distortion[OB_SENSOR_DEPTH] = {
        c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], OB_DISTORTION_BROWN_CONRADY};
    // SDK 2.9.3 takes a count of FLOATS, not bytes. Its public wrapper rejects
    // null data, so no null-size query is made. This fixed layout is verified
    // against the pinned implementation and by offline real-SDK regression.
    static_assert(kGeometryPixels * 2 < std::numeric_limits<std::uint32_t>::max());
    const auto count = static_cast<std::uint32_t>(kGeometryPixels * 2);
    std::vector<float> storage(count, std::numeric_limits<float>::quiet_NaN());
    std::uint32_t capacity = count;
    OBXYTables table{};
    if (!ob::CoordinateTransformHelper::transformationInitXYTables(
            calibration, OB_SENSOR_DEPTH, storage.data(), &capacity, &table) ||
        capacity != count || table.width != static_cast<int>(k.width) ||
        table.height != static_cast<int>(k.height) || table.xTable != storage.data() ||
        table.yTable != storage.data() + kGeometryPixels)
        throw std::runtime_error("native rays: SDK table failure or unexpected layout");

    // Overall SDK success does not imply every pixel is valid: a failed
    // undistortion may be encoded as x=NaN,y=0. Reject before exposing a table.
    rays_.reserve(kGeometryPixels);
    for (std::size_t i = 0; i < kGeometryPixels; ++i) {
        if (!std::isfinite(table.xTable[i]) || !std::isfinite(table.yTable[i]))
            throw std::runtime_error("native rays: invalid SDK pixel at " + std::to_string(i));
        // The table entries multiply optical Z. Do not normalize to unit norm
        // or apply the depth unit here (NativeGeometry applies raw*unit_m once).
        rays_.push_back({static_cast<double>(table.xTable[i]),
                         static_cast<double>(table.yTable[i]), 1.0});
    }
}

// The geometry owner bound to exactly this binding and ray table.
std::unique_ptr<NativeGeometry> OrbbecNativeRays::make_geometry(const GeometryOptions& options) const {
    return std::make_unique<NativeGeometry>(binding_, rays_.data(), rays_.size(), options);
}
}  // namespace perception::geometry
