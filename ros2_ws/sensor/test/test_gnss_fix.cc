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

using sensor::FixTypeFromGgaQuality;
using sensor::FixTypeValid;
using sensor::GnssFix;
using sensor::ToJsonData;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

static bool Has(const std::string& s, const std::string& sub) {
  return s.find(sub) != std::string::npos;
}

int main() {
  // quality -> fix_type (11 S4.5 closed set).
  CHECK(FixTypeFromGgaQuality(4) == "rtk_fixed");
  CHECK(FixTypeFromGgaQuality(5) == "rtk_float");
  CHECK(FixTypeFromGgaQuality(2) == "dgps");
  CHECK(FixTypeFromGgaQuality(1) == "single");
  CHECK(FixTypeFromGgaQuality(0) == "no_fix");
  CHECK(FixTypeFromGgaQuality(6) == "no_fix");   // dead-reckoning is not a fix
  CHECK(FixTypeValid("rtk_fixed"));
  CHECK(!FixTypeValid("garbage"));

  // has_position = true -> real numbers.
  GnssFix f;
  f.has_position = true;
  f.lat = 34.7; f.lon = 135.5; f.alt = 40.0;
  f.fix_type = "rtk_fixed"; f.cov_h_m = 0.02; f.sats = 24;
  const std::string j = ToJsonData(f);
  CHECK(Has(j, "\"fix_type\":\"rtk_fixed\""));
  CHECK(Has(j, "\"sats\":24"));
  CHECK(!Has(j, "\"lat\":null"));

  // 3.3 mutant: no position -> lat/lon/cov are JSON null, never 0.
  GnssFix nf;   // defaults: has_position=false, fix_type=no_fix
  const std::string jn = ToJsonData(nf);
  CHECK(Has(jn, "\"lat\":null"));
  CHECK(Has(jn, "\"lon\":null"));
  CHECK(Has(jn, "\"cov_h_m\":null"));
  CHECK(Has(jn, "\"fix_type\":\"no_fix\""));

  if (g_failures == 0) {
    std::printf("ALL GNSS FIX TESTS PASSED\n");
    return 0;
  }
  std::printf("%d GNSS FIX TEST(S) FAILED\n", g_failures);
  return 1;
}
