/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: ros_bridge.cpp
 * Brief: RosBridge: capture command subscription and the optional TF pose provider.
 *
 * Description:
 * What problem this solves. Implements ros_bridge.hpp. One rclcpp node on its own executor
 * thread (svc_ros) serves the evidence capture command; with tf_enable it also answers the
 * Supervisor's exact-time odom <- base_link lookups.
 *
 * Note on TF in this delivery. The service configuration refuses tf_enable=true (reply B4-1:
 * odom-frame velocity is not supported by the Objects path), so in the delivered service only
 * the capture-command duty is active; the TF code path is kept but not reachable.
 */
#include "service/ros_bridge.hpp"

#include "common/logger.hpp"
#include "common/time_utils.hpp"
#include "modules/odom_buffer.hpp"
#include "supervisor/supervisor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace perception::service {
namespace {
constexpr const char* kModule = "RosBridge";
constexpr double kTfLookupTimeoutSec = 0.002;  // legacy node value
// After a failed lookup, TF is not asked again for 1 s: the lookup runs on the
// Supervisor thread, which must not keep paying for a TF tree that is absent.
constexpr auto kTfUnavailableRetry = std::chrono::seconds(1);
}  // namespace

class RosBridge::Node : public rclcpp::Node {
public:
    Node(const RosConfig& config, Supervisor& supervisor)
        : rclcpp::Node("perception"), config_(config), supervisor_(supervisor) {
        // LINT-ALLOW(ros): capture-command REPLY on the control plane (std_msgs/String). P19-4 forbids publishing
        // perception data on ROS topics; no detection, image, profile or status data is published here.
        done_pub_ = create_publisher<std_msgs::msg::String>(kCaptureDoneTopic, rclcpp::QoS(10));
        if (config_.tf_enable) {
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, true);
        }
    }
    // Subscribed only after the pipeline started, so a command always finds a
    // running Supervisor.
    void subscribe_capture() {
        capture_sub_ = create_subscription<std_msgs::msg::String>(
            kCaptureCommandTopic, rclcpp::QoS(10),
            [this](const std_msgs::msg::String::SharedPtr msg) { on_capture(msg); });
    }
    // Exact-time odom <- base_link pose for the Supervisor's odom buffer.
    bool lookup_odom_pose(double stamp_sec, OdomPose& out_pose, float& out_diff_ms) {
        out_diff_ms = 0.0F;
        if (!tf_buffer_) return false;
        const auto now = std::chrono::steady_clock::now();
        if (now < retry_after_) return false;
        tf_lookups_.fetch_add(1, std::memory_order_relaxed);
        // Cheap existence check first (latest transform, no wait): when the TF tree has no
        // odom <- base_link at all, the timed lookup below would cost its full timeout.
        std::string why;
        if (!tf_buffer_->canTransform(config_.odom_frame, config_.base_frame, tf2::TimePointZero,
                                      tf2::durationFromSec(0.0), &why)) {
            retry_after_ = now + kTfUnavailableRetry;
            tf_unavailable_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        // Exact-time lookup with a 2 ms bound: the Supervisor waits for it inline.
        try {
            const auto t = tf_buffer_->lookupTransform(config_.odom_frame, config_.base_frame,
                                                       tf2::timeFromSec(stamp_sec),
                                                       tf2::durationFromSec(kTfLookupTimeoutSec));
            out_pose = convert(t, stamp_sec);
            out_diff_ms = static_cast<float>(std::fabs(out_pose.source_stamp_sec - stamp_sec) * 1000.0);
            tf_successes_.fetch_add(1, std::memory_order_relaxed);
            return true;
        } catch (const tf2::TransformException&) {
            // Exact-time TF missing: no latest-pose substitution here. A pose from
            // another time would violate the exact-time ego_removed condition.
            retry_after_ = std::chrono::steady_clock::now() + kTfUnavailableRetry;
            tf_unavailable_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
    }
    Counters counters() const noexcept {
        Counters c;
        c.capture_commands = commands_.load(std::memory_order_relaxed);
        c.capture_replies = replies_.load(std::memory_order_relaxed);
        c.tf_lookups = tf_lookups_.load(std::memory_order_relaxed);
        c.tf_successes = tf_successes_.load(std::memory_order_relaxed);
        c.tf_unavailable = tf_unavailable_.load(std::memory_order_relaxed);
        return c;
    }

private:
    // One reply per command, whatever the Supervisor answered.
    void on_capture(const std_msgs::msg::String::SharedPtr& msg) {
        if (!msg) return;
        commands_.fetch_add(1, std::memory_order_relaxed);
        std_msgs::msg::String done;
        done.data = supervisor_.handle_capture_command(msg->data);
        done_pub_->publish(std::move(done));
        replies_.fetch_add(1, std::memory_order_relaxed);
    }
    // TF message -> OdomPose; keeps both the requested and the TF source stamp.
    static OdomPose convert(const geometry_msgs::msg::TransformStamped& t, double requested_stamp_sec) {
        OdomPose pose;
        pose.stamp_sec = requested_stamp_sec;
        pose.source_stamp_sec = stamp_to_sec(t.header.stamp.sec, t.header.stamp.nanosec);
        pose.position[0] = static_cast<float>(t.transform.translation.x);
        pose.position[1] = static_cast<float>(t.transform.translation.y);
        pose.position[2] = static_cast<float>(t.transform.translation.z);
        pose.orientation_xyzw[0] = static_cast<float>(t.transform.rotation.x);
        pose.orientation_xyzw[1] = static_cast<float>(t.transform.rotation.y);
        pose.orientation_xyzw[2] = static_cast<float>(t.transform.rotation.z);
        pose.orientation_xyzw[3] = static_cast<float>(t.transform.rotation.w);
        return pose;
    }

    RosConfig config_;
    Supervisor& supervisor_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr done_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr capture_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::chrono::steady_clock::time_point retry_after_{};
    std::atomic<std::uint64_t> commands_{0}, replies_{0}, tf_lookups_{0}, tf_successes_{0}, tf_unavailable_{0};
};

RosBridge::RosBridge(const RosConfig& config, Supervisor& supervisor) : config_(config), supervisor_(supervisor) {
    // The service only builds a bridge when ROS is enabled; anything else is a caller bug.
    if (!config_.enable) throw std::logic_error("RosBridge constructed with ros.enable=false");
    // No external (file) or rosout logging: the service logs through its own
    // logger, needs no HOME/ROS_LOG_DIR under systemd, and publishes nothing but
    // the capture receipt on ROS.
    // No rclcpp signal handlers: SIGTERM/SIGINT stay with the service's own
    // ordered stop (see the header).
    const char* args[] = {"perception_node", "--ros-args", "--disable-external-lib-logs", "--disable-rosout-logs"};
    rclcpp::init(4, args, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    node_ = std::make_shared<Node>(config_, supervisor_);
    LOG_INFO_M(kModule, "ROS_EXECUTOR node=perception tf_enable=" << config_.tf_enable << " odom_frame=" << config_.odom_frame
        << " base_frame=" << config_.base_frame << " capture_cmd=" << kCaptureCommandTopic << " capture_done=" << kCaptureDoneTopic);
}

RosBridge::~RosBridge() { stop(); }

void RosBridge::attach_pose_provider() {
    // Only with tf_enable (refused by the delivered configuration, B4-1): the Supervisor's odom
    // buffer then asks this node for exact-time poses.
    if (!config_.tf_enable) return;
    auto* odom = supervisor_.odom_buffer();
    if (!odom) throw std::runtime_error("Supervisor did not expose an OdomBuffer for TF");
    auto node = node_;
    odom->set_pose_provider([node](double stamp_sec, OdomPose& pose, float& diff_ms) {
        return node->lookup_odom_pose(stamp_sec, pose, diff_ms);
    });
    provider_attached_ = true;
}

void RosBridge::start() {
    node_->subscribe_capture();
    // Spin in 100 ms slices so a stop request is seen within that bound.
    thread_ = std::thread([this] {
        pthread_setname_np(pthread_self(), "svc_ros");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node_);
        while (!stopping_.load(std::memory_order_acquire) && rclcpp::ok())
            executor.spin_once(std::chrono::milliseconds(100));
        executor.remove_node(node_);
    });
}

void RosBridge::stop() noexcept {
    if (stopped_) return;
    stopped_ = true;
    // Order: detach the pose provider first (no TF call during pipeline stop),
    // then stop the executor, log the counters and shut rclcpp down.
    try {
        if (provider_attached_) {
            if (auto* odom = supervisor_.odom_buffer()) odom->set_pose_provider({});
        }
        stopping_.store(true, std::memory_order_release);
        if (thread_.joinable()) thread_.join();
        const auto c = node_ ? node_->counters() : Counters{};
        LOG_INFO_M(kModule, "ROS_EXECUTOR_STOPPED capture_commands=" << c.capture_commands << " capture_replies="
            << c.capture_replies << " tf_lookups=" << c.tf_lookups << " tf_successes=" << c.tf_successes
            << " tf_unavailable=" << c.tf_unavailable);
        node_.reset();
        if (rclcpp::ok()) rclcpp::shutdown();
    } catch (...) {
        // stop() also runs from the destructor: an exception from rclcpp during shutdown must
        // not escape and terminate the process during the service's ordered stop.
    }
}

}  // namespace perception::service
