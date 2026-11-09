/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: odom_transform.hpp
 * Brief: Odometry coordinate transform helpers.
 *
 * Description:
 * What problem this solves. Part of the pending TF / odom capability (19 section 1.1, "TF 监听";
 * 19 section 4.2, ego removal), kept by decision although the delivered configuration refuses
 * tf_enable (B4-1): it moves detection centroids and the legacy ground_footprint_xy into odom
 * with a camera -> base transform and the odom <- base pose of the frame.
 *
 * Boundaries. No pose lookup (odom_buffer), no velocity (track_velocity), no wire output.
 *
 * Pitfalls. This is NOT yet the 19 section 4.2 design and must not be enabled as it is:
 *   - the camera -> base transform comes from the legacy LiDAR key extrinsic_base_lidar (identity
 *     when absent). The fix is not a key rename: the centroids are in the colour optical frame,
 *     so the validated base <- colour chain of setup_object_geometry() has to be reused;
 *   - ground_footprint_xy is transformed as camera points with z = 0, although older text calls
 *     it a LiDAR-frame polygon; R6 geometry is base_footprint_xy and never comes through here;
 *   - the result stays on odom axes. Ego removal also needs the pose at the exposure time,
 *     interpolated, the rotation back into the current base_link and a calibrated extrinsic
 *     before velocity_frame may say ego_removed.
 */

#pragma once

#include <array>
#include <vector>

#include "common/config.hpp"
#include "common/types.hpp"
#include "modules/odom_buffer.hpp"

namespace perception {

// Translation plus a quaternion (x, y, z, w); the quaternion is normalised at use.
struct RigidTransform {
    std::array<float, 3> translation = {0.0F, 0.0F, 0.0F};
    std::array<float, 4> rotation_xyzw = {0.0F, 0.0F, 0.0F, 1.0F};
};

// Legacy source of the camera -> base transform: the extrinsic_base_lidar key (see Pitfalls).
RigidTransform extrinsic_camera_to_base_from_config(const yolo::Config& config);
// R * p + t.
std::array<float, 3> transform_point(const RigidTransform& transform, const std::array<float, 3>& point);
// In place: centroid_3d and ground_footprint_xy of every detection become odom coordinates.
void transform_detections_to_odom(
    std::vector<FusedDetection>& detections,
    const OdomPose& odom_pose,
    const RigidTransform& camera_to_base);

}  // namespace perception
