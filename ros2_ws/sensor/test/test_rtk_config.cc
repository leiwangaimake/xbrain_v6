/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rtk_config.cc
 * Brief: Offline unit test for LoadRtkConfig (3.1 fail-stop on null/missing)
 *
 * Description:
 * Golden case: a full valid flat config loads into DriverConfig with every field
 * mapped and heading_stddev converted deg->rad. Mutation cases (CLAUDE.md 3.3):
 * a null resolver threshold and a missing driver timeout must each make
 * LoadRtkConfig THROW -- proving the 3.1 no-default contract is actually
 * enforced, not just intended. Without the null-throws mutant, a loader that
 * quietly substituted 0.0 would pass a positive-only test and ship a machine
 * that limits itself to a stop with no error.
 */

#include "sensor/rtk_config.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

using sensor::LoadRtkConfig;
using sensor::RtkConfig;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

template <class F>
static bool Throws(F f) {
  try {
    f();
    return false;
  } catch (const std::exception&) {
    return true;
  }
}

static void WriteFile(const std::string& path, const std::string& body) {
  std::ofstream f(path, std::ios::binary);
  f << body;
}

static const char* kValid =
    "serial:\n"
    "  port: \"/dev/ttyACM0\"\n"
    "  baudrate: 115200\n"
    "heading_stddev_deg: 1.0\n"
    "gga_timeout_s: 1.0\n"
    "tra_timeout_s: 1.0\n"
    "rmc_timeout_s: 1.0\n"
    "sync_timeout_ms: 5000\n"
    "resolver:\n"
    "  cov_thresh_rad: 0.02\n"
    "  age_thresh_s: 0.2\n"
    "  cog_speed_thresh_mps: 0.5\n"
    "  degrade_sustain_s: 0.5\n"
    "  recover_sustain_s: 2.0\n"
    "  fix_lost_sustain_s: 1.0\n"
    "  blind_rise_sustain_s: 0.5\n"
    "  blind_timeout_s: 30.0\n"
    "  cov_h_m: 0.05\n"
    "  cog_diff_dt_s: 0.9\n"
    "  i_heading_l1: 1.0\n"
    "  i_heading_l2: 0.4\n"
    "  i_heading_l3: 0.0\n"
    "fix_cov:\n"
    "  rtk_fixed_h_m: 0.02\n"
    "  rtk_float_h_m: 0.30\n"
    "  dgps_h_m: 1.50\n"
    "  single_h_m: 3.00\n"
    "  vertical_factor: 1.5\n"
    "clock:\n"
    "  offset_threshold_ms: 20.0\n"
    "  ref_max_age_s: 5.0\n"
    "  rtc_trusted: false\n";

int main() {
  const std::string p = "test_rtk_cfg_tmp.yaml";

  // Golden: full valid config maps every field.
  WriteFile(p, kValid);
  const RtkConfig c = LoadRtkConfig(p, "robot1", "rtk_driver", "abc12345");
  CHECK(c.driver.rid == "robot1");
  CHECK(c.driver.src == "rtk_driver");
  CHECK(c.driver.boot == "abc12345");
  CHECK(c.serial_port == "/dev/ttyACM0");
  CHECK(c.serial_baud == 115200);
  CHECK(c.driver.sync_timeout_ms == 5000);
  CHECK(std::fabs(c.driver.heading_stddev_rad - 1.0 * 3.14159265358979323846 / 180.0) < 1e-12);
  CHECK(c.driver.gga_timeout_s == 1.0);
  CHECK(c.driver.resolver.cov_thresh_rad == 0.02);
  CHECK(c.driver.resolver.blind_timeout_s == 30.0);
  CHECK(c.driver.resolver.i_heading_l1 == 1.0);
  CHECK(c.driver.resolver.i_heading_l3 == 0.0);
  CHECK(c.driver.fix_cov.rtk_fixed_h_m == 0.02);
  CHECK(c.driver.fix_cov.vertical_factor == 1.5);

  // 3.3 mutant A: a null safety threshold must stop the load.
  std::string null_cov = kValid;
  {
    const std::string from = "cov_thresh_rad: 0.02";
    null_cov.replace(null_cov.find(from), from.size(), "cov_thresh_rad: null");
  }
  WriteFile(p, null_cov);
  CHECK(Throws([&] { LoadRtkConfig(p, "r", "rtk_driver", "b"); }));

  // 3.3 mutant B: a missing driver timeout must stop the load.
  std::string no_gga = kValid;
  {
    const std::string from = "gga_timeout_s: 1.0\n";
    no_gga.replace(no_gga.find(from), from.size(), "");
  }
  WriteFile(p, no_gga);
  CHECK(Throws([&] { LoadRtkConfig(p, "r", "rtk_driver", "b"); }));

  std::remove(p.c_str());
  if (g_failures == 0) {
    std::printf("ALL RTK_CONFIG TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RTK_CONFIG TEST(S) FAILED\n", g_failures);
  return 1;
}
