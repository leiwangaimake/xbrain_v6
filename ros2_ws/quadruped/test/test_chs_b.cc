/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_chs_b.cc
 * Brief: DDS-1 -- the domain is the configured one even when the environment disagrees
 *
 * Description:
 * The case that matters here sets ROS_DOMAIN_ID to 42 and then requires the
 * participant to have joined domain 0 anyway.
 *
 * That is the only way to test DDS-1. This process holds an rclcpp context on
 * domain 42 at the same time as this participant on domain 0, and both
 * ROS_DOMAIN_ID and CYCLONEDDS_URI are process-wide: an implementation that
 * read either one would put both entities on the same domain. The result is a
 * participant that comes up, reports no error, and never receives anything --
 * which 13 DDS-9 records as indistinguishable from a dead network, on a system
 * where nobody would think to suspect an environment variable.
 *
 * The domain is read BACK from the entity rather than echoed from the config.
 * Echoing would pass on the broken implementation too.
 *
 * *** What this does NOT establish: that the chassis's IMU is received. That
 * needs the chassis on the wire and is T-CHS-1a/b, on the bench. What is
 * asserted here is that with nothing publishing, the reader reports "never
 * seen" rather than a zeroed sample -- the distinction that tells an engineer
 * whether to look at the name mapping or at the cable.
 */

#include "quadruped/chs_b.h"

#include <cstdio>
#include <cstdlib>
#include <string>

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
ChassisDdsConfig Cfg() {
  ChassisDdsConfig c;
  c.backend = "cyclone_raw";
  c.domain_id = 0;               // 13 DDS-1: the chassis domain
  c.imu_topic = "/IMU";
  c.imu_expect_hz = 200.0;
  c.imu_age_warn_ms = 50;
  c.imu_frame_id = "imu_link";
  c.forward_imu_to_rt = false;
  return c;
}
}  // namespace

int main() {
  // *** Set the environment to the WRONG domain before constructing anything.
  // An implementation that read it would join 42 and never hear the chassis.
  setenv("ROS_DOMAIN_ID", "42", 1);

  ChassisDds dds(Cfg());

  // Read back from the entity. Echoing the config would pass on the broken
  // implementation as well, which is the whole reason this is a read-back.
  CHECK(dds.actual_domain_id() == 0);

  // Nothing is publishing, so nothing arrives. The distinction asserted here is
  // "never seen" versus a zeroed sample: a zeroed one reads as a stationary,
  // level robot, and an engineer shown that would look at the chassis rather
  // than at the topic name.
  ImuSample s;
  CHECK(dds.latest_imu(&s) == false);
  CHECK(dds.imu_age_s(1.0) < 0.0);
  CHECK(ClassifyImuAge(dds.imu_age_s(1.0), 50) == ImuFreshness::kNeverSeen);
  CHECK(dds.imu_count() == 0);

  MotionInfoSample m;
  CHECK(dds.latest_motion_info(&m) == false);
  CHECK(dds.motion_info_age_s(1.0) < 0.0);

  // Polling an empty reader is a no-op, not an error and not a block. The
  // chs_b thread calls this at 200 Hz and must never stall there (DDS-6).
  for (int i = 0; i < 20; ++i) CHECK(dds.Poll(0.001 * i) == 0);
  CHECK(dds.imu_count() == 0);

  if (g_failures == 0) {
    std::printf("ALL CHS_B TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CHS_B TEST(S) FAILED\n", g_failures);
  return 1;
}
