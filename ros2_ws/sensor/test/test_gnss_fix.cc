/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_gnss_fix.cc
 * Brief: Offline unit test for GnssFix mapping + null-safe JSON (11 S3.2)
 *
 * Description:
 * The load-bearing case is the no-position mutant (CLAUDE.md 3.3): when
 * has_position is false, lat/lon/cov_h_m MUST serialise as JSON null, never 0.0 --
 * 0,0 is a real coordinate a consumer would plot and 0 m cov reads as a perfect
 * fix (NAV-02). The quality->fix_type table is pinned to the 11 S4.5 closed set.
 */

#include "sensor/gnss_fix.h"

#include <cstdio>
#include <string>

using sensor::fix_type_from_gga_quality;
using sensor::fix_type_valid;
using sensor::GnssFix;
using sensor::to_json_data;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

static bool has(const std::string& s, const std::string& sub) {
  return s.find(sub) != std::string::npos;
}

int main() {
  // quality -> fix_type (11 S4.5 closed set).
  CHECK(fix_type_from_gga_quality(4) == "rtk_fixed");
  CHECK(fix_type_from_gga_quality(5) == "rtk_float");
  CHECK(fix_type_from_gga_quality(2) == "dgps");
  CHECK(fix_type_from_gga_quality(1) == "single");
  CHECK(fix_type_from_gga_quality(0) == "no_fix");
  CHECK(fix_type_from_gga_quality(6) == "no_fix");   // dead-reckoning is not a fix
  CHECK(fix_type_valid("rtk_fixed"));
  CHECK(!fix_type_valid("garbage"));

  // has_position = true -> real numbers.
  GnssFix f;
  f.has_position = true;
  f.lat = 34.7; f.lon = 135.5; f.alt = 40.0;
  f.fix_type = "rtk_fixed"; f.cov_h_m = 0.02; f.sats = 24;
  const std::string j = to_json_data(f);
  CHECK(has(j, "\"fix_type\":\"rtk_fixed\""));
  CHECK(has(j, "\"sats\":24"));
  CHECK(!has(j, "\"lat\":null"));

  // lat/lon keep 9 decimals: 6 (std::to_string) is ~0.11 m and erased the RTK
  // fixed centimetres on the bus (2026-11-03 field test, 11 S3.2).
  GnssFix cm = f;
  cm.lat = 31.158372345;
  cm.lon = 121.391281678;
  const std::string jc = to_json_data(cm);
  CHECK(has(jc, "\"lat\":31.158372345,"));
  CHECK(has(jc, "\"lon\":121.391281678,"));
  CHECK(has(jc, "\"cov_h_m\":0.020000,"));

  // 3.3 mutant: no position -> lat/lon/cov are JSON null, never 0.
  GnssFix nf;   // defaults: has_position=false, fix_type=no_fix
  const std::string jn = to_json_data(nf);
  CHECK(has(jn, "\"lat\":null"));
  CHECK(has(jn, "\"lon\":null"));
  CHECK(has(jn, "\"cov_h_m\":null"));
  CHECK(has(jn, "\"fix_type\":\"no_fix\""));

  // age_s is the differential age: null without corrections, never 0 (a 0 would
  // read as "corrections applied this instant"); a number when present.
  CHECK(has(j, "\"age_s\":null"));          // f has a position but no corrections
  GnssFix df = f;
  df.has_age = true;
  df.age_s = 1.5;
  const std::string jd = to_json_data(df);
  CHECK(has(jd, "\"age_s\":1.500000"));
  CHECK(has(jn, "\"age_s\":null"));

  // RTG-10 differential-age cap (thresholds 10 s / 60 s; strictly greater).
  using sensor::fix_type_after_diff_age;
  CHECK(fix_type_after_diff_age("rtk_fixed", true, 2.0, 10.0, 60.0) == "rtk_fixed");
  CHECK(fix_type_after_diff_age("rtk_fixed", true, 10.0, 10.0, 60.0) == "rtk_fixed");
  CHECK(fix_type_after_diff_age("rtk_fixed", true, 10.5, 10.0, 60.0) == "rtk_float");
  CHECK(fix_type_after_diff_age("rtk_float", true, 30.0, 10.0, 60.0) == "rtk_float");
  CHECK(fix_type_after_diff_age("rtk_float", true, 60.0, 10.0, 60.0) == "rtk_float");
  CHECK(fix_type_after_diff_age("rtk_fixed", true, 61.0, 10.0, 60.0) == "dgps");
  CHECK(fix_type_after_diff_age("rtk_float", true, 61.0, 10.0, 60.0) == "dgps");
  // An RTK class with no age at all is not trusted as RTK.
  CHECK(fix_type_after_diff_age("rtk_fixed", false, 0.0, 10.0, 60.0) == "dgps");
  CHECK(fix_type_after_diff_age("rtk_float", false, 0.0, 10.0, 60.0) == "dgps");
  // Never touches the non-RTK classes, and never lowers below dgps.
  CHECK(fix_type_after_diff_age("dgps", true, 999.0, 10.0, 60.0) == "dgps");
  CHECK(fix_type_after_diff_age("single", false, 0.0, 10.0, 60.0) == "single");
  CHECK(fix_type_after_diff_age("no_fix", false, 0.0, 10.0, 60.0) == "no_fix");

  if (g_failures == 0) {
    std::printf("ALL GNSS FIX TESTS PASSED\n");
    return 0;
  }
  std::printf("%d GNSS FIX TEST(S) FAILED\n", g_failures);
  return 1;
}
