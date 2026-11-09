/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: odom_buffer.cpp
 * Brief: Odometry pose ring buffer implementation.
 *
 * Description:
 * Implements odom_buffer.hpp. A mutex guards the ring and the provider: poses may be pushed from
 * a ROS callback thread while the service thread looks them up.
 */

#include "modules/odom_buffer.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace perception {
namespace {

constexpr int DEFAULT_MAX_FRAMES = 50;
constexpr float MS_PER_SECOND = 1000.0F;

int sanitize_max_frames(int max_frames) {
    return max_frames > 0 ? max_frames : DEFAULT_MAX_FRAMES;
}

float sanitize_stale_ms(float stale_ms) {
    return stale_ms > 0.0F ? stale_ms : 200.0F;
}

}  // namespace

OdomBuffer::OdomBuffer(const Params& params) : params_(params) {
    params_.max_frames = sanitize_max_frames(params_.max_frames);
    params_.stale_ms = sanitize_stale_ms(params_.stale_ms);
}

OdomBuffer::OdomBuffer(const yolo::Config& config)
    : params_{config.odom_stale_ms(), DEFAULT_MAX_FRAMES} {
    params_.stale_ms = sanitize_stale_ms(params_.stale_ms);
}

void OdomBuffer::push(OdomPose pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (static_cast<int>(buffer_.size()) >= params_.max_frames) {
        buffer_.pop_front();
    }
    buffer_.push_back(std::move(pose));
}

void OdomBuffer::set_pose_provider(PoseProvider provider) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_provider_ = std::move(provider);
}

bool OdomBuffer::get_nearest(double target_stamp_sec, OdomPose& out_pose, float& out_diff_ms) const {
    // Copy the provider and call it outside the lock: a TF lookup may wait up to its timeout, and
    // pushes must not block behind it.
    PoseProvider provider;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        provider = pose_provider_;
    }
    if (provider) {
        return provider(target_stamp_sec, out_pose, out_diff_ms);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (buffer_.empty()) {
        out_diff_ms = 0.0F;
        return false;
    }

    // Linear scan: the ring holds at most max_frames poses.
    auto nearest = buffer_.begin();
    double best_diff_sec = std::numeric_limits<double>::max();
    for (auto it = buffer_.begin(); it != buffer_.end(); ++it) {
        const double diff_sec = std::fabs(it->stamp_sec - target_stamp_sec);
        if (diff_sec < best_diff_sec) {
            best_diff_sec = diff_sec;
            nearest = it;
        }
    }

    out_diff_ms = static_cast<float>(best_diff_sec * static_cast<double>(MS_PER_SECOND));
    if (out_diff_ms > params_.stale_ms) {
        return false;
    }

    out_pose = *nearest;
    return true;
}

int OdomBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(buffer_.size());
}

bool OdomBuffer::available() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !buffer_.empty();
}

}  // namespace perception
