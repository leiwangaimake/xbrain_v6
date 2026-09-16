/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_uplink.cc
 * Brief: The uplink constructs on its own domain and honours the odom bands
 *
 * Description:
 * What this DOES establish: the rclcpp context comes up on the configured
 * domain without touching the environment, the publisher and the broadcaster
 * are created once at construction (13 S5.7), and the suppression rules of
 * 13 S4.4 (4) are obeyed -- a stopped band publishes nothing at all, TF
 * included, and a configuration with publish_odom_tf false publishes the
 * odometry alone.
 *
 * *** What it does NOT establish, stated so the pass is not read as more than
 * it is: that any subscriber RECEIVES these messages. That is T-ODOM-1, it
 * needs a second process on the same domain, and it belongs to the bench batch.
 * A counter incrementing proves the publish call returned, not that a packet
 * left the machine -- and 13 DDS-9 exists precisely because those two are
 * indistinguishable from inside one process.
 *
 * The domain used here is the configured one (42), not a test-only domain. A
 * test that quietly used a different domain would pass on an implementation
 * that reads ROS_DOMAIN_ID from the environment, which is the mistake DDS-1
 * cares about most: this process holds a bare domain-0 participant as well, and
 * one environment read collapses the two.
 */

#include "quadruped/uplink.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

#include "nav_msgs/msg/odometry.hpp"
#include "quadruped/odometry.h"
#include "rclcpp/rclcpp.hpp"

using namespace quadruped;  // NOLINT: test-local

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

namespace {

// Somewhere for the process's own SIGTERM disposition to point, so the test can
// tell "still ours" from "replaced". Never raised.
volatile std::sig_atomic_t g_signalled = 0;
void OurHandler(int) { g_signalled = 1; }

UplinkConfig Cfg(bool with_tf) {
  UplinkConfig c;
  c.ros_domain_id = 42;          // 13 DDS-1, the configured value
  c.rmw = "rmw_cyclonedds_cpp";
  c.odom_topic = "/odom_quadruped";
  c.odom_frame = "odom";
  c.base_frame = "base_link";
  c.publish_odom_tf = with_tf;
  c.zenoh_rt_endpoint = "tcp/127.0.0.1:7449";
  return c;
}

OdomSample Fresh() {
  OdomSample s;
  s.x = 1.25;
  s.y = -0.5;
  s.yaw = 0.3;
  s.vx = 0.4;
  s.var_x = 0.01;
  s.var_y = 0.01;
  s.var_yaw = 1e-4;
  s.var_vx = 0.0025;
  s.var_vy = 0.0025;
  s.var_wz = 6.4e-5;
  s.band = OdomBand::kFresh;
  s.valid = true;
  s.publish = true;
  return s;
}

}  // namespace

int main() {
  // *** The WRONG domain in the environment, set before anything is built.
  // rclcpp reads ROS_DOMAIN_ID by default, and this process is also supposed to
  // hold a bare domain-0 participant for the chassis -- one environment read
  // puts both on the same domain, and the symptom is a publisher nobody hears
  // (13 DDS-1 / DDS-9).
  setenv("ROS_DOMAIN_ID", "7", 1);

  // *** The process's shutdown path, installed BEFORE rclcpp exists.
  //
  // rclcpp::init() -- the documented way to start rclcpp, and what anyone
  // touching uplink.cc would reach for -- installs process-wide SIGINT and
  // SIGTERM handlers. This process needs those signals to reach main.cc: that
  // is where the control thread is stopped and joined. Taken by rclcpp, a
  // SIGTERM shuts the ROS context down and leaves the chassis link and the
  // control loop running, until systemd SIGKILLs a process still holding a
  // socket to a robot.
  //
  // uplink.cc avoids it by building a Context directly, which installs nothing
  // (checked against the humble headers: the handlers come from rclcpp::init
  // and install_signal_handlers, and InitOptions::shutdown_on_signal only says
  // whether a handler someone ELSE installed shuts this context down). So this
  // assertion is not about that flag -- it is about the call, and the mutant
  // that adds rclcpp::install_signal_handlers() is what proves it can fail.
  struct sigaction ours;
  std::memset(&ours, 0, sizeof(ours));
  ours.sa_handler = OurHandler;
  CHECK(sigaction(SIGTERM, &ours, nullptr) == 0);

  {
    Uplink up(Cfg(/*with_tf=*/true), "gj-001");
    // Read BACK from the rcl context, not echoed from the config.
    CHECK(up.actual_domain_id() == 42);
    CHECK(up.odom_published() == 0);

    up.Publish(Fresh(), 1789455340.125);
    CHECK(up.odom_published() == 1);
    CHECK(up.tf_published() == 1);
    CHECK(up.suppressed() == 0);

    // 13 S4.4 (4): past one second of staleness NOTHING goes out, TF included.
    // A frozen TF makes Nav2 believe the robot is stationary and keep
    // commanding rotation; stopping it makes Nav2 abort inside its 200 ms
    // transform tolerance, which is the fail-safe we want.
    OdomSample stopped = Fresh();
    stopped.band = OdomBand::kStop;
    stopped.publish = false;
    stopped.valid = false;
    up.Publish(stopped, 1789455341.125);
    CHECK(up.odom_published() == 1);      // unchanged
    CHECK(up.tf_published() == 1);        // unchanged -- the TF stopped too
    CHECK(up.suppressed() == 1);

    // ...and it resumes when the band does, so the suppression is a state and
    // not a latch.
    up.Publish(Fresh(), 1789455342.125);
    CHECK(up.odom_published() == 2);
    CHECK(up.tf_published() == 2);
  }

  {
    // publish_odom_tf false: the odometry still goes out, the transform does
    // not. A deployment where something else owns odom->base_link would
    // otherwise get two publishers for one edge, and tf2 resolves that by
    // whichever arrived last.
    Uplink up(Cfg(/*with_tf=*/false), "gj-001");
    up.Publish(Fresh(), 1789455340.125);
    CHECK(up.odom_published() == 1);
    CHECK(up.tf_published() == 0);
  }

  // The disposition must still be ours after an Uplink has been built and
  // destroyed. Compared as a POINTER, because "a handler is installed" is true
  // either way -- rclcpp installs one too.
  {
    struct sigaction now;
    std::memset(&now, 0, sizeof(now));
    CHECK(sigaction(SIGTERM, nullptr, &now) == 0);
    CHECK(now.sa_handler == OurHandler);
  }

  // ---- what actually went on the wire -----------------------------------
  //
  // Every assertion above counts publishes. A publisher that sent an all-zero
  // Odometry on the right topic at the right rate would satisfy all of them,
  // and downstream that reads as a robot sitting at the origin -- the one pose
  // a planner will happily accept. This block subscribes and looks.
  {
    Uplink up(Cfg(/*with_tf=*/true), "gj-001");

    // A subscriber on the SAME domain, built with its own context so it is a
    // genuinely separate participant rather than an intra-node shortcut.
    auto ctx = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions io;
    io.set_domain_id(42);
    io.shutdown_on_signal = false;
    ctx->init(0, nullptr, io);
    rclcpp::NodeOptions no;
    no.context(ctx);
    auto sub_node = std::make_shared<rclcpp::Node>("uplink_probe", no);

    nav_msgs::msg::Odometry got;
    bool have = false;
    auto sub = sub_node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_quadruped", rclcpp::QoS(10),
        [&got, &have](nav_msgs::msg::Odometry::SharedPtr m) {
          got = *m;
          have = true;
        });

    rclcpp::ExecutorOptions eo;
    eo.context = ctx;
    rclcpp::executors::SingleThreadedExecutor exec(eo);
    exec.add_node(sub_node);

    // Published repeatedly while spinning: discovery between two participants
    // takes time, and a single publish before it completes is dropped with no
    // error. Bounded, and the failure is asserted rather than waited out.
    const OdomSample sample = Fresh();
    for (int i = 0; i < 200 && !have; ++i) {
      up.Publish(sample, 1789455340.125);
      exec.spin_some(std::chrono::milliseconds(10));
    }
    CHECK(have);

    if (have) {
      // The pose, by value. 13 S4.4 is the acceptance basis for these numbers
      // and "something arrived" is not a check against it.
      CHECK(got.pose.pose.position.x == 1.25);
      CHECK(got.pose.pose.position.y == -0.5);
      CHECK(got.twist.twist.linear.x == 0.4);
      CHECK(got.header.frame_id == "odom");
      CHECK(got.child_frame_id == "base_link");

      // The yaw, through the quaternion. 0.3 rad must come back as 0.3 rad --
      // a half-angle error here is invisible in every other assertion and
      // makes the robot's heading wrong by a factor of two.
      const double yaw = 2.0 * std::atan2(got.pose.pose.orientation.z,
                                          got.pose.pose.orientation.w);
      CHECK(std::fabs(yaw - 0.3) < 1e-9);

      // The covariance INDEX. [35] is yaw-yaw in a row-major 6x6; an
      // implementation using [5] fills a correlation term nobody reads and
      // leaves the yaw variance at zero -- which downstream reads as a
      // perfectly known heading.
      CHECK(got.pose.covariance[0] == 0.01);
      CHECK(got.pose.covariance[7] == 0.01);
      CHECK(got.pose.covariance[35] == 1e-4);
      CHECK(got.twist.covariance[35] == 6.4e-5);

      // The stamp is the wall time passed IN, not a clock read here.
      CHECK(got.header.stamp.sec == 1789455340);
      CHECK(got.header.stamp.nanosec > 124000000u);
      CHECK(got.header.stamp.nanosec < 126000000u);
    }

    exec.remove_node(sub_node);
    ctx->shutdown("test done");
  }

  if (g_failures == 0) {
    std::printf("ALL UPLINK TESTS PASSED\n");
    return 0;
  }
  std::printf("%d UPLINK TEST(S) FAILED\n", g_failures);
  return 1;
}
