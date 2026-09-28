/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_gnss_heading.cc
 * Brief: Offline unit test for GnssHeading consistency + JSON (no ROS, no gtest)
 *
 * Description:
 * Self-contained CTest executable: compiles with plain g++ against
 * gnss_heading.cc + the common closed-set header, verifying the 11 S3.3 message
 * WITHOUT ROS / zenoh / hardware. The load-bearing case is the L3 null cov_rad:
 * it must serialise as JSON null, never 0.0 (a 0 covariance reads as a perfect
 * heading, NAV-02 over-trust). Each assertion is paired with the mutation that
 * turns it red (CLAUDE.md 3.3): opt_num -> Num on the nullopt path, or dropping
 * the level match in gnss_heading_consistent.
 */

#include "sensor/gnss_heading.h"

#include <cstdio>
#include <string>

using sensor::GnssHeading;
using sensor::gnss_heading_consistent;
using sensor::to_json_data;

static int g_failures = 0;

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
      ++g_failures;                                                       \
    }                                                                     \
  } while (0)

static bool has(const std::string& s, const std::string& sub) {
  return s.find(sub) != std::string::npos;
}

// ---- source <-> level one-to-one + closed-set membership -----------------
static void test_consistent() {
  GnssHeading h;
  h.source = "dual_antenna"; h.level = 1; CHECK(gnss_heading_consistent(h));
  h.source = "cog";          h.level = 2; CHECK(gnss_heading_consistent(h));
  h.source = "none";         h.level = 3; CHECK(gnss_heading_consistent(h));
  // source/level mismatch rejected (mutation: drop the level compare -> passes).
  h.source = "dual_antenna"; h.level = 2; CHECK(!gnss_heading_consistent(h));
  h.source = "none";         h.level = 1; CHECK(!gnss_heading_consistent(h));
  // out-of-set source rejected (11 S13.6: no silent degrade to a nearby value).
  h.source = "gyro";         h.level = 1; CHECK(!gnss_heading_consistent(h));
  h.source = "";             h.level = 3; CHECK(!gnss_heading_consistent(h));
}

// ---- L1 JSON: values present, correct field names ------------------------
static void test_json_l1() {
  GnssHeading h;
  h.heading_rad = 1.5708;
  h.heading_true_north_rad = 0.1234;
  h.heading_valid = true;
  h.source = "dual_antenna"; h.level = 1;
  h.cov_rad = 0.005;
  h.speed_mps = 1.2;
  h.cog_rad = 1.56;
  h.baseline_m = 0.62;
  h.baseline_valid = true;
  h.yaw_capable = true;
  h.i_heading = 1.0;
  h.age_s = 0.05;
  h.t_mono = 128300.1;
  const std::string j = to_json_data(h);
  CHECK(has(j, "\"heading_rad\":1.570800"));
  CHECK(has(j, "\"heading_true_north_rad\":0.123400"));
  CHECK(has(j, "\"heading_valid\":true"));
  CHECK(has(j, "\"source\":\"dual_antenna\""));
  CHECK(has(j, "\"level\":1"));
  CHECK(has(j, "\"cov_rad\":0.005000"));
  CHECK(has(j, "\"speed_mps\":1.200000"));
  CHECK(has(j, "\"baseline_valid\":true"));
  CHECK(has(j, "\"yaw_capable\":true"));
  CHECK(has(j, "\"i_heading\":1.000000"));
  CHECK(has(j, "\"t_mono\":128300.100000"));
}

// ---- L3 JSON: cov_rad + optionals are null, NOT 0 (the load-bearing case) -
static void test_json_l3_null_not_zero() {
  GnssHeading h;   // defaults: source "none", level 3, valid false, all opt empty
  const std::string j = to_json_data(h);
  // cov_rad MUST be null. Mutation (opt_num -> Num on nullopt) makes it 0.000000,
  // which reads downstream as a perfect heading -> both these turn red.
  CHECK(has(j, "\"cov_rad\":null"));
  CHECK(!has(j, "\"cov_rad\":0"));
  CHECK(has(j, "\"heading_valid\":false"));
  CHECK(has(j, "\"source\":\"none\""));
  CHECK(has(j, "\"level\":3"));
  CHECK(has(j, "\"heading_true_north_rad\":null"));
  CHECK(has(j, "\"cog_rad\":null"));
  CHECK(has(j, "\"baseline_m\":null"));
  CHECK(has(j, "\"baseline_valid\":null"));
  CHECK(has(j, "\"yaw_capable\":false"));
}

int main() {
  test_consistent();
  test_json_l1();
  test_json_l3_null_not_zero();
  if (g_failures == 0) {
    std::printf("ALL GNSS HEADING TESTS PASSED\n");
    return 0;
  }
  std::printf("%d GNSS HEADING TEST(S) FAILED\n", g_failures);
  return 1;
}
