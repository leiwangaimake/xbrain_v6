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

#include <cstdio>

#include "quadruped/odometry.h"

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
  {
    Uplink up(Cfg(/*with_tf=*/true), "gj-001");
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

  if (g_failures == 0) {
    std::printf("ALL UPLINK TESTS PASSED\n");
    return 0;
  }
  std::printf("%d UPLINK TEST(S) FAILED\n", g_failures);
  return 1;
}
