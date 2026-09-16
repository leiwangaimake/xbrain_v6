/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: uplink.cc
 * Brief: The three rclcpp call sites, and nothing else (13 PB-5)
 *
 * Description:
 * The three sites PB-5 permits, in order: create the context, publish an
 * Odometry, publish a tf. They are the three whose API is identical in humble
 * and jazzy, which is why the platform decision (D-45) can still land without
 * rewriting this file.
 *
 * Two details that are easy to get wrong and expensive to find:
 *
 *   * the domain is set on the CONTEXT, through rclcpp::InitOptions, and never
 *     read from ROS_DOMAIN_ID. This process also holds a BARE domain-0
 *     participant for the chassis DDS (13 DDS-1), and the environment variable
 *     is process-wide: one read would put both on the same domain. The symptom
 *     is "the participant is up and not one packet arrives", which 13 DDS-9
 *     records as indistinguishable from a dead network.
 *   * the header stamp is built from the wall time passed IN. A bare
 *     rclcpp::Clock() is the ROS WALL clock and looks identical at the call
 *     site to the steady one, which is why CLK-C1 names it explicitly and why
 *     scripts/lint/clock_scan.py greps for it.
 *
 * Nothing here decides anything. Whether to publish, whether the sample is
 * valid and how large the covariance is were all settled by Odometry against
 * 13 S4.4; this file transcribes that decision onto the wire.
 */

#include "quadruped/uplink.h"

#include <stdexcept>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace quadruped {

struct Uplink::Impl {
  UplinkConfig cfg;
  std::string rid;
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bc;
  std::uint64_t odom_count = 0;
  std::uint64_t tf_count = 0;
  std::uint64_t suppressed_count = 0;
};

Uplink::Uplink(const UplinkConfig& cfg, const std::string& rid)
    : impl_(new Impl()) {
  impl_->cfg = cfg;
  impl_->rid = rid;

  // ---- rclcpp call site 1 of 3: create the context ----------------------
  impl_->context = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions init_options;
  // Explicit, never from the environment. See the file comment: this process
  // holds a bare domain-0 participant at the same time, and one environment
  // read collapses the two domains with no error anywhere.
  init_options.set_domain_id(static_cast<std::size_t>(cfg.ros_domain_id));
  // Leave signal handling to the process: a library that installs handlers
  // takes the shutdown path away from the owner, and main.cc needs SIGTERM to
  // reach it so the control thread is stopped and joined.
  //
  // NO TEST CAN DISTINGUISH THIS VALUE (CLAUDE.md 7.2.1), and it is still the
  // right one. The handlers are installed by rclcpp::init() or
  // install_signal_handlers(), and this file calls neither; the flag only
  // decides whether THIS context is shut down by a handler something else
  // installed. What IS defended, by an assertion in test_uplink.cc, is the
  // thing that would actually go wrong here: building an Uplink must not
  // replace the process's SIGTERM disposition -- which the documented way of
  // starting rclcpp would do. Registered as a declared-equivalent mutant.
  init_options.shutdown_on_signal = false;
  impl_->context->init(0, nullptr, init_options);

  rclcpp::NodeOptions node_options;
  node_options.context(impl_->context);
  // 13 S5.7: publishers are declared ONCE here, never created at run time.
  impl_->node = std::make_shared<rclcpp::Node>("quadruped_uplink", node_options);

  // Depth 10, and the default (reliable) history: odometry is state, and a
  // best-effort odom would drop exactly when the network is busiest -- which
  // is when the planner most needs a pose.
  impl_->odom_pub = impl_->node->create_publisher<nav_msgs::msg::Odometry>(
      cfg.odom_topic, rclcpp::QoS(10));
  impl_->tf_bc = std::make_unique<tf2_ros::TransformBroadcaster>(impl_->node);

  if (!impl_->odom_pub) {
    throw std::runtime_error(
        "quadruped uplink: could not create the odometry publisher on " +
        cfg.odom_topic);
  }
}

Uplink::~Uplink() {
  if (impl_ && impl_->context) {
    impl_->context->shutdown("quadruped uplink closing");
  }
}

void Uplink::Publish(const OdomSample& s, double wall_ts_s) {
  if (!impl_) return;
  if (!s.publish) {
    // 13 S4.4 (4): past one second of staleness NOTHING goes out, TF included.
    // Counted rather than logged: the interesting signal is a rate, and a log
    // line per tick at 100 Hz buries it.
    ++impl_->suppressed_count;
    return;
  }

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(wall_ts_s);
  stamp.nanosec = static_cast<std::uint32_t>(
      (wall_ts_s - static_cast<double>(stamp.sec)) * 1e9);

  const Quaternion q = YawToQuaternion(s.yaw);

  // ---- rclcpp call site 2 of 3: publish the Odometry --------------------
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = impl_->cfg.odom_frame;
  msg.child_frame_id = impl_->cfg.base_frame;
  msg.pose.pose.position.x = s.x;
  msg.pose.pose.position.y = s.y;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation.x = q.x;
  msg.pose.pose.orientation.y = q.y;
  msg.pose.pose.orientation.z = q.z;
  msg.pose.pose.orientation.w = q.w;
  msg.twist.twist.linear.x = s.vx;
  msg.twist.twist.linear.y = s.vy;
  msg.twist.twist.angular.z = s.wz;
  // The index constants live in the ROS-free core and are tested there: [35]
  // is yaw, and an implementation that used [5] leaves the yaw variance zero
  // while filling a correlation nobody reads.
  FillCovariance36(s.var_x, s.var_y, s.var_yaw, msg.pose.covariance.data());
  FillCovariance36(s.var_vx, s.var_vy, s.var_wz, msg.twist.covariance.data());
  impl_->odom_pub->publish(msg);
  ++impl_->odom_count;

  // ---- rclcpp call site 3 of 3: publish the tf --------------------------
  if (impl_->cfg.publish_odom_tf) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = impl_->cfg.odom_frame;
    tf.child_frame_id = impl_->cfg.base_frame;
    tf.transform.translation.x = s.x;
    tf.transform.translation.y = s.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x;
    tf.transform.rotation.y = q.y;
    tf.transform.rotation.z = q.z;
    tf.transform.rotation.w = q.w;
    impl_->tf_bc->sendTransform(tf);
    ++impl_->tf_count;
  }
}

int Uplink::actual_domain_id() const {
  if (!impl_ || !impl_->context) return -1;
  // Through the rcl context, which is the entity that actually joined. The
  // rclcpp::Context object only remembers what it was asked for, so reading
  // that would pass on the broken implementation too.
  std::size_t id = 0;
  const rcl_context_t* ctx = impl_->context->get_rcl_context().get();
  if (rcl_context_get_domain_id(const_cast<rcl_context_t*>(ctx), &id) !=
      RCL_RET_OK) {
    return -1;
  }
  return static_cast<int>(id);
}

std::uint64_t Uplink::odom_published() const {
  return impl_ ? impl_->odom_count : 0;
}
std::uint64_t Uplink::tf_published() const { return impl_ ? impl_->tf_count : 0; }
std::uint64_t Uplink::suppressed() const {
  return impl_ ? impl_->suppressed_count : 0;
}

}  // namespace quadruped
