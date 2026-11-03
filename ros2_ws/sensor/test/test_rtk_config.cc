/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rtk_config.cc
 * Brief: Offline unit test for load_rtk_config (3.1 fail-stop on null/missing)
 *
 * Description:
 * Golden case: a full valid flat config loads into DriverConfig with every field
 * mapped and heading_stddev converted deg->rad. Mutation cases (CLAUDE.md 3.3):
 * a null resolver threshold and a missing driver timeout must each make
 * load_rtk_config THROW -- proving the 3.1 no-default contract is actually
 * enforced, not just intended. Without the null-throws mutant, a loader that
 * quietly substituted 0.0 would pass a positive-only test and ship a machine
 * that limits itself to a stop with no error.
 */

#include "sensor/rtk_config.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

using sensor::load_rtk_config;
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
static bool throws(F f) {
  try {
    f();
    return false;
  } catch (const std::exception&) {
    return true;
  }
}

static void write_file(const std::string& path, const std::string& body) {
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
    "diff_age:\n"
    "  fixed_to_float_s: 10.0\n"
    "  to_dgps_s: 60.0\n"
    "clock:\n"
    "  offset_threshold_ms: 20.0\n"
    "  ref_max_age_s: 5.0\n"
    "  rtc_trusted: false\n"
    "rtcm:\n"
    "  sink:\n"
    "    queue_max_bytes: 16384\n"
    "    frame_max_age_s: 2.0\n"
    "  lan:\n"
    "    enabled: true\n"
    "    host: \"192.168.10.5\"\n"
    "    port: 9100\n"
    "    connect_timeout_s: 3.0\n"
    "    read_timeout_s: 5.0\n"
    "    backoff_min_s: 1.0\n"
    "    backoff_max_s: 30.0\n"
    "  ntrip:\n"
    "    enabled: false\n"
    "    host: \"rtk.example.com\"\n"
    "    port: null\n"
    "    mountpoint: null\n"
    "    version: 1\n"
    "    user: \"qxuser\"\n"
    "    password: \"secret\"\n"
    "    connect_timeout_s: 5.0\n"
    "    read_timeout_s: 10.0\n"
    "    backoff_min_s: 2.0\n"
    "    backoff_max_s: 60.0\n"
    "    backoff_auth_s: 300.0\n"
    "    linger_s: 60.0\n"
    "    gga_period_s: 5.0\n"
    "    gga_max_pos_age_s: 2.0\n"
    "    vrs_rebase_km: 10.0\n"
    "  health:\n"
    "    obs_window_s: 3.0\n"
    "    station_window_s: 30.0\n"
    "  selector:\n"
    "    hold_up_s: 10.0\n"
    "    backup_after_s: 15.0\n";

static std::string with(const std::string& base, const std::string& from, const std::string& to) {
  std::string s = base;
  s.replace(s.find(from), from.size(), to);
  return s;
}

int main() {
  const std::string p = "test_rtk_cfg_tmp.yaml";

  // Golden: full valid config maps every field.
  write_file(p, kValid);
  const RtkConfig c = load_rtk_config(p, "robot1", "rtk_driver", "abc12345");
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
  CHECK(c.driver.diff_age.fixed_to_float_s == 10.0);
  CHECK(c.driver.diff_age.to_dgps_s == 60.0);

  // 3.3 mutant A: a null safety threshold must stop the load.
  std::string null_cov = kValid;
  {
    const std::string from = "cov_thresh_rad: 0.02";
    null_cov.replace(null_cov.find(from), from.size(), "cov_thresh_rad: null");
  }
  write_file(p, null_cov);
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));

  // 3.3 mutant B: a missing driver timeout must stop the load.
  std::string no_gga = kValid;
  {
    const std::string from = "gga_timeout_s: 1.0\n";
    no_gga.replace(no_gga.find(from), from.size(), "");
  }
  write_file(p, no_gga);
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));

  // rtcm (11 S3.2A): the enabled LAN source is read in full; the disabled NTRIP
  // source may keep null port / mountpoint (V-RTCM-2 still open).
  CHECK(c.sink.queue_max_bytes == 16384);
  CHECK(c.sink.frame_max_age_s == 2.0);
  CHECK(c.corrections.queue_max_bytes == 16384);
  CHECK(c.corrections.lan.enabled);
  CHECK(c.corrections.lan.host == "192.168.10.5");
  CHECK(c.corrections.lan.port == 9100);
  CHECK(c.corrections.lan.backoff_max_s == 30.0);
  CHECK(!c.corrections.ntrip.enabled);
  CHECK(c.corrections.health.station_window_s == 30.0);
  CHECK(c.corrections.selector.hold_up_s == 10.0);
  CHECK(c.corrections.selector.backup_after_s == 15.0);

  // Enabling NTRIP with the port still null must stop the load (3.1).
  const std::string nt_on = with(kValid, "    enabled: false\n", "    enabled: true\n");
  write_file(p, nt_on);
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));
  // ... and with every key present it loads, linger_s included.
  const std::string nt_full = with(with(nt_on, "    port: null\n", "    port: 8002\n"),
                                   "    mountpoint: null\n", "    mountpoint: \"AUTO\"\n");
  write_file(p, nt_full);
  const RtkConfig cn = load_rtk_config(p, "r", "rtk_driver", "b");
  CHECK(cn.corrections.ntrip.enabled);
  CHECK(cn.corrections.ntrip.port == 8002);
  CHECK(cn.corrections.ntrip.mountpoint == "AUTO");
  CHECK(cn.corrections.ntrip.password == "secret");
  CHECK(cn.corrections.ntrip.vrs_rebase_km == 10.0);
  CHECK(cn.corrections.selector.linger_s == 60.0);
  write_file(p, with(nt_full, "    version: 1\n", "    version: 3\n"));
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));

  // An enabled LAN source with a missing key stops the load.
  write_file(p, with(kValid, "    read_timeout_s: 5.0\n", ""));
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));
  // A queue smaller than one maximum frame is refused.
  write_file(p, with(kValid, "queue_max_bytes: 16384", "queue_max_bytes: 1000"));
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));
  // RTG-10 thresholds: required, and the float step must come first.
  write_file(p, with(kValid, "  to_dgps_s: 60.0\n", ""));
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));
  write_file(p, with(kValid, "  to_dgps_s: 60.0\n", "  to_dgps_s: 10.0\n"));
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));
  write_file(p, with(kValid, "  fixed_to_float_s: 10.0\n", "  fixed_to_float_s: 0.0\n"));
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));
  // The selector keys are required whatever is enabled.
  write_file(p, with(kValid, "    backup_after_s: 15.0\n", ""));
  CHECK(throws([&] { load_rtk_config(p, "r", "rtk_driver", "b"); }));

  std::remove(p.c_str());
  if (g_failures == 0) {
    std::printf("ALL RTK_CONFIG TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RTK_CONFIG TEST(S) FAILED\n", g_failures);
  return 1;
}
