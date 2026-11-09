/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: odom_buffer.hpp
 * Brief: Odometry pose ring buffer for perception-to-odom transforms.
 *
 * Description:
 * What problem this solves. Expressing object centroids in odom needs the robot pose at the
 * frame's time (19 section 4.2, ego removal; a pending capability). Two sources, without any
 * ROS type in this interface:
 *   - a pose provider: the ROS bridge answers exact-time TF lookups. It is attached only with
 *     tf_enable, which the delivered configuration refuses (B4-1), so on R6 the lookup fails and
 *     the velocity path stays in the camera frame;
 *   - without a provider, a bounded ring of pushed poses, searched for the nearest stamp.
 *
 * Boundaries. Pose lookup only; transforming detections is odom_transform. The ring returns the
 * nearest sample and does not interpolate, whereas 19 section 4.2 asks for the pose at the
 * exposure time.
 *
 * Pitfalls. The requested stamp must be on the poses' clock (the service passes the frame's wall
 * stamp). A lookup farther than stale_ms from every sample fails instead of returning an old pose.
 */

#pragma once

#include <deque>
#include <functional>
#include <mutex>

#include "common/config.hpp"

namespace perception {

/**
 * @brief One odom<-base_link pose sample.
 */
struct OdomPose {
    // The time the pose was asked for (provider) or pushed at (ring).
    double stamp_sec = 0.0;
    // The stamp the pose source itself reported (the TF header); its distance from the request
    // is the lookup error reported as diff_ms.
    double source_stamp_sec = 0.0;
    float position[3] = {0.0F, 0.0F, 0.0F};
    float orientation_xyzw[4] = {0.0F, 0.0F, 0.0F, 1.0F};
};

/**
 * @brief Thread-safe ring buffer for odometry pose lookup.
 */
class OdomBuffer {
public:
    // Answers one lookup; false when no pose is available for that time.
    using PoseProvider = std::function<bool(double target_stamp_sec, OdomPose& out_pose, float& out_diff_ms)>;

    struct Params {
        // Largest accepted distance between the request and the nearest ring sample.
        float stale_ms = 200.0F;
        // Ring capacity; the oldest pose is dropped when it is full.
        int max_frames = 50;
    };

    explicit OdomBuffer(const Params& params);
    explicit OdomBuffer(const yolo::Config& config);

    void push(OdomPose pose);
    // An installed provider takes precedence over the ring; an empty one detaches it.
    void set_pose_provider(PoseProvider provider);
    // Pose for target_stamp_sec; out_diff_ms is the distance to the pose actually used.
    bool get_nearest(double target_stamp_sec, OdomPose& out_pose, float& out_diff_ms) const;
    int size() const;
    bool available() const;

private:
    mutable std::mutex mutex_;
    std::deque<OdomPose> buffer_;
    Params params_;
    PoseProvider pose_provider_;
};

}  // namespace perception
