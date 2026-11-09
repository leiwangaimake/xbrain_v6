/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: odom_transform.cpp
 * Brief: Odometry coordinate transform implementation.
 *
 * Description:
 * Implements odom_transform.hpp; read its Pitfalls before enabling the TF path.
 */

#include "common/odom_transform.hpp"

#include <cmath>

namespace perception {
namespace {

constexpr float QUAT_EPSILON = 1e-6F;

// A near-zero quaternion becomes the identity instead of a division by zero.
std::array<float, 4> normalized_quat(const std::array<float, 4>& q) {
    const float norm = std::sqrt((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
    if (norm < QUAT_EPSILON) {
        return {0.0F, 0.0F, 0.0F, 1.0F};
    }
    return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

// The rotation matrix of the unit quaternion, applied to the point.
std::array<float, 3> rotate_point(const std::array<float, 4>& rotation_xyzw, const std::array<float, 3>& point) {
    const std::array<float, 4> q = normalized_quat(rotation_xyzw);
    const float x = q[0];
    const float y = q[1];
    const float z = q[2];
    const float w = q[3];

    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;

    return {
        ((1.0F - (2.0F * (yy + zz))) * point[0]) + ((2.0F * (xy - wz)) * point[1]) +
            ((2.0F * (xz + wy)) * point[2]),
        ((2.0F * (xy + wz)) * point[0]) + ((1.0F - (2.0F * (xx + zz))) * point[1]) +
            ((2.0F * (yz - wx)) * point[2]),
        ((2.0F * (xz - wy)) * point[0]) + ((2.0F * (yz + wx)) * point[1]) +
            ((1.0F - (2.0F * (xx + yy))) * point[2])};
}

RigidTransform odom_pose_to_transform(const OdomPose& pose) {
    return {
        {pose.position[0], pose.position[1], pose.position[2]},
        {pose.orientation_xyzw[0], pose.orientation_xyzw[1], pose.orientation_xyzw[2], pose.orientation_xyzw[3]}};
}

}  // namespace

RigidTransform extrinsic_camera_to_base_from_config(const yolo::Config& config) {
    return {config.extrinsic_base_lidar_translation(), config.extrinsic_base_lidar_rotation_xyzw()};
}

std::array<float, 3> transform_point(const RigidTransform& transform, const std::array<float, 3>& point) {
    std::array<float, 3> rotated = rotate_point(transform.rotation_xyzw, point);
    rotated[0] += transform.translation[0];
    rotated[1] += transform.translation[1];
    rotated[2] += transform.translation[2];
    return rotated;
}

void transform_detections_to_odom(
    std::vector<FusedDetection>& detections,
    const OdomPose& odom_pose,
    const RigidTransform& camera_to_base) {
    for (auto& det : detections) {
        const std::array<float, 3> centroid_camera = {det.centroid_3d[0], det.centroid_3d[1], det.centroid_3d[2]};
        const std::array<float, 3> centroid_base = transform_point(camera_to_base, centroid_camera);
        const std::array<float, 3> centroid_odom = transform_point(odom_pose_to_transform(odom_pose), centroid_base);
        det.centroid_3d[0] = centroid_odom[0];
        det.centroid_3d[1] = centroid_odom[1];
        det.centroid_3d[2] = centroid_odom[2];

        // The legacy footprint is taken as camera points with z = 0 (see the header).
        for (auto& vertex : det.ground_footprint_xy) {
            const std::array<float, 3> point_camera = {vertex[0], vertex[1], 0.0F};
            const std::array<float, 3> point_base = transform_point(camera_to_base, point_camera);
            const std::array<float, 3> point_odom = transform_point(odom_pose_to_transform(odom_pose), point_base);
            vertex[0] = point_odom[0];
            vertex[1] = point_odom[1];
        }
    }
}

}  // namespace perception
