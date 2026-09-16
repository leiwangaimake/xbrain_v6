/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: uplink.h
 * Brief: Channel three -- rclcpp odom + TF on ROS domain 42 (13 DDS-1, PB-5)
 *
 * Description:
 * The ONLY part of this package that touches rclcpp, and the header does not:
 * every ROS type is behind a pimpl so the core library, its tests, and
 * chassis_relay all keep building on a machine with no ROS at all. That is not
 * tidiness -- 13 S5.3 forbids the shared library from seeing a ROS type because
 * chassis_relay is on the emergency-stop path, and a header that leaked one
 * would take the whole dependency with it.
 *
 * 13 PB-5 limits the rclcpp coupling to THREE call sites: create the context,
 * publish an Odometry, publish a tf. The limit exists because the platform
 * baseline (D-45, humble against jazzy) is not settled, and those three APIs
 * are the ones that are identical in both. Anything else here would have to be
 * revisited the day that decision lands -- which is also why there is no
 * distribution macro anywhere in this package (PB-5b, audited by
 * scripts/ci/cxx_discipline_audit.py).
 *
 * The domain is passed explicitly and never read from the environment.
 * 13 DDS-1/DDS-7: this process holds a bare domain-0 participant for the
 * chassis DDS at the same time, and ROS_DOMAIN_ID is a process-wide variable --
 * one environment read is all it takes for the two domains to collapse into
 * one, which presents as "the participant is up and not one packet arrives".
 *
 * What it does NOT do: decide. The band, the validity and the covariance all
 * come from Odometry (13 S4.4), and this file publishes or does not publish
 * according to what that decided. In particular a kStop band publishes NOTHING,
 * TF included -- a frozen TF makes Nav2 believe the robot is stationary and
 * keep commanding rotation, and stopping it makes Nav2 abort inside its 200 ms
 * transform tolerance instead.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_UPLINK_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_UPLINK_H_

#include <memory>
#include <string>

#include "quadruped/odometry.h"
#include "quadruped/quadruped_config.h"

namespace quadruped {

class Uplink {
 public:
  // Creates the rclcpp context on cfg.ros_domain_id. Throws on failure rather
  // than leaving a half-initialised object: a publisher that silently does
  // nothing is the failure this whole package is built to avoid.
  Uplink(const UplinkConfig& cfg, const std::string& rid);
  ~Uplink();

  Uplink(const Uplink&) = delete;
  Uplink& operator=(const Uplink&) = delete;

  // One publish tick. `wall_ts_s` stamps the ROS header -- ROS message stamps
  // are wall clock by convention, and it is passed IN rather than read here
  // (CLK-C1 bans a bare rclcpp::Clock(), whose default constructor is the wall
  // clock and looks identical to the steady one at the call site).
  //
  // Publishes nothing at all when the sample says not to.
  void Publish(const OdomSample& s, double wall_ts_s);

  // The domain the context actually joined, READ BACK from it rather than
  // echoed from the config. Echoing would report 42 on an implementation that
  // took the domain from ROS_DOMAIN_ID -- which is the one failure 13 DDS-1 is
  // written to prevent, and the one whose symptom (a publisher that is up and
  // heard by nobody) is indistinguishable from a dead network.
  int actual_domain_id() const;

  // Counters, so "it is publishing" is a number rather than an impression.
  std::uint64_t odom_published() const;
  std::uint64_t tf_published() const;
  std::uint64_t suppressed() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_UPLINK_H_
