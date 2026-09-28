/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_dds_names.cc
 * Brief: The name mapping, which is otherwise only visible as silence
 *
 * Description:
 * A wrong ROS-to-DDS topic mapping produces a participant that comes up, a
 * topic that exists and not one sample. CLAUDE.md 5.4 calls it the easiest trap
 * in this package and records that 13 does not mention it; 13 DDS-9 records
 * that it is indistinguishable from a dead network. There is nothing to observe
 * at run time, so it has to be pinned here.
 *
 * The degenerate inputs matter as much as the happy one. A mapping that
 * accepted "" would create a reader on "rt/" -- a perfectly valid topic that
 * nothing publishes -- and the operator would be told nothing at all.
 */

#include "quadruped/dds_names.h"

#include <cstdio>
#include <stdexcept>
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
template <typename F>
bool throws(F f) {
  try {
    f();
  } catch (const std::exception&) {
    return true;
  }
  return false;
}
}  // namespace

int main() {
  // ---- the mapping itself ------------------------------------------------
  {
    // The two topics this process actually subscribes. Written out rather than
    // generated, so a reviewer can compare them with what `ros2 topic list`
    // shows on the chassis.
    CHECK(ros_topic_to_dds_topic("/IMU") == "rt/IMU");
    CHECK(ros_topic_to_dds_topic("/MOTION_INFO") == "rt/MOTION_INFO");
    // A nested name keeps its inner slashes: only the leading one goes.
    CHECK(ros_topic_to_dds_topic("/LIDAR/POINTS") == "rt/LIDAR/POINTS");
  }

  // ---- the degenerate forms are refused, not mapped ---------------------
  {
    // Each of these would produce a VALID DDS topic that nothing publishes, so
    // the failure would arrive as silence rather than as a message.
    CHECK(throws([] { ros_topic_to_dds_topic(""); }));
    CHECK(throws([] { ros_topic_to_dds_topic("/"); }));
    // Relative, which is what a config edit that dropped the slash produces.
    CHECK(throws([] { ros_topic_to_dds_topic("IMU"); }));
    CHECK(throws([] { ros_topic_to_dds_topic("rt/IMU"); }));
    CHECK(throws([] { ros_topic_to_dds_topic("/IMU/"); }));
    // *** A doubled slash is REFUSED rather than normalised. Normalising would
    // map "//IMU" and "/IMU" to the same place and hide the typo; refusing
    // tells whoever wrote it.
    CHECK(ros_topic_to_dds_topic("//IMU") == "rt//IMU");
  }

  // ---- the freshness bands ----------------------------------------------
  {
    // 13 S2.5: an age over ten sample periods (50 ms at 200 Hz) degrades the
    // odometry onto the 10 Hz attitude from the monitor protocol.
    CHECK(classify_imu_age(-1.0, 50) == ImuFreshness::kNeverSeen);
    CHECK(classify_imu_age(0.0, 50) == ImuFreshness::kFresh);
    CHECK(classify_imu_age(0.049, 50) == ImuFreshness::kFresh);
    CHECK(classify_imu_age(0.050, 50) == ImuFreshness::kFresh);   // == is not over
    CHECK(classify_imu_age(0.051, 50) == ImuFreshness::kStale);
    CHECK(classify_imu_age(5.0, 50) == ImuFreshness::kStale);
    // *** never-seen is NOT folded into stale. The two have different causes --
    // never-seen is almost always the name mapping above, stale is almost
    // always the link -- and an operator sent to the wrong one loses an hour.
    CHECK(classify_imu_age(-0.001, 50) != ImuFreshness::kStale);
  }

  // ---- the band names are distinct ---------------------------------------
  {
    const std::string n[] = {imu_freshness_name(ImuFreshness::kNeverSeen),
                             imu_freshness_name(ImuFreshness::kFresh),
                             imu_freshness_name(ImuFreshness::kStale)};
    for (int i = 0; i < 3; ++i) {
      CHECK(!n[i].empty());
      for (int j = i + 1; j < 3; ++j) CHECK(n[i] != n[j]);
    }
  }

  if (g_failures == 0) {
    std::printf("ALL DDS_NAMES TESTS PASSED\n");
    return 0;
  }
  std::printf("%d DDS_NAMES TEST(S) FAILED\n", g_failures);
  return 1;
}
