/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_odom_buffer.cpp
 * Brief: Unit tests for odometry pose buffering.
 *
 * Description:
 *   1. Verifies bounded buffering and nearest timestamp matching.
 *   2. Verifies stale odom rejection for lidar-frame degradation.
 */

#include "modules/odom_buffer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

perception::OdomPose make_pose(double stamp_sec, float x) {
    perception::OdomPose pose;
    pose.stamp_sec = stamp_sec;
    pose.position[0] = x;
    pose.orientation_xyzw[3] = 1.0F;
    return pose;
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

// The buffer keeps only the newest max_size poses and returns the nearest one with its time
// difference.
void test_nearest_and_capacity() {
    perception::OdomBuffer buffer({200.0F, 3});
    buffer.push(make_pose(1.0, 1.0F));
    buffer.push(make_pose(2.0, 2.0F));
    buffer.push(make_pose(3.0, 3.0F));
    buffer.push(make_pose(4.0, 4.0F));
    require(buffer.size() == 3, "OdomBuffer did not keep max size");

    perception::OdomPose out;
    float diff_ms = 0.0F;
    require(buffer.get_nearest(3.05, out, diff_ms), "nearest odom pose was not found");
    require(std::fabs(out.position[0] - 3.0F) < 1e-6F, "nearest odom pose value mismatch");
    require(std::fabs(diff_ms - 50.0F) < 1e-3F, "nearest odom diff mismatch");
}

// A nearest pose farther than the tolerance is refused, still reporting the difference.
void test_stale_rejects() {
    perception::OdomBuffer buffer({200.0F, 10});
    buffer.push(make_pose(10.0, 1.0F));

    perception::OdomPose out;
    float diff_ms = 0.0F;
    require(!buffer.get_nearest(10.250, out, diff_ms), "stale odom pose was accepted");
    require(std::fabs(diff_ms - 250.0F) < 1e-3F, "stale odom diff mismatch");
}

// An empty buffer returns no pose and resets the difference to 0.
void test_empty_rejects() {
    perception::OdomBuffer buffer({200.0F, 10});
    perception::OdomPose out;
    float diff_ms = -1.0F;
    require(!buffer.get_nearest(1.0, out, diff_ms), "empty odom buffer returned a pose");
    require(diff_ms == 0.0F, "empty odom diff was not reset");
}

}  // namespace

int main() {
    test_nearest_and_capacity();
    test_stale_rejects();
    test_empty_rejects();
    std::cout << "test_odom_buffer passed\n";
    return 0;
}
