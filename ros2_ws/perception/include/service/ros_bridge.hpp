/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: ros_bridge.hpp
 * Brief: ROS executor of the formal R6 service (19 section 1.1 item 5).
 *
 * Description:
 * Only two duties (P19-4): TF odom<-base_link for the Supervisor odom buffer
 * (optional, perception.pipeline.ros.tf_enable) and the evidence capture
 * command. Topics and the TF lookup/back-off policy are carried over from the
 * legacy node (src/node/perception_node.cpp; reply 2026-09-17 Q15):
 *   /perception/capture_cmd_json  -> Supervisor::handle_capture_command
 *   /perception/capture_done_json <- its receipt
 * Perception data (profile/objects/status) is never published to ROS.
 * rclcpp is initialized WITHOUT its signal handlers so the service keeps its
 * own ordered SIGTERM/SIGINT stop. TF missing -> the Supervisor keeps its
 * camera-frame path; R6 wire velocity stays raw/null (not ego_removed).
 */

#pragma once

#include "service/service_config.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

namespace perception {
class Supervisor;
}

namespace perception::service {

class RosBridge {
public:
    static constexpr const char* kCaptureCommandTopic = "/perception/capture_cmd_json";
    static constexpr const char* kCaptureDoneTopic = "/perception/capture_done_json";

    struct Counters {
        std::uint64_t capture_commands = 0, capture_replies = 0;
        std::uint64_t tf_lookups = 0, tf_successes = 0, tf_unavailable = 0;
    };

    // Throws std::runtime_error when rclcpp cannot be initialized.
    RosBridge(const RosConfig&, Supervisor&);
    ~RosBridge();
    RosBridge(const RosBridge&) = delete;
    RosBridge& operator=(const RosBridge&) = delete;

    // After Supervisor::init, before start (no-op unless tf_enable).
    void attach_pose_provider();
    // After Supervisor::start: capture subscription + executor thread.
    void start();
    // Detach provider, stop executor, join, rclcpp::shutdown. Idempotent.
    void stop() noexcept;

private:
    class Node;
    RosConfig config_;
    Supervisor& supervisor_;
    std::shared_ptr<Node> node_;
    std::thread thread_;
    std::atomic<bool> stopping_{false};
    bool provider_attached_ = false, stopped_ = false;
};

}  // namespace perception::service
