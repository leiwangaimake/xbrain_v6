/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rtk_driver.cc
 * Brief: Offline unit test for the rtk_driver core (no ROS, no zenoh, no gtest)
 *
 * Description:
 * Feeds synthetic NMEA into RtkDriver and captures the published messages with a
 * fake PublishSink, so the whole feed -> parse -> resolve -> GnssHeading ->
 * envelope -> publish path is verified WITHOUT a serial port or a zenoh binding.
 * Time is injected, so the S3.3.3 recovery dwell is exercised. Each assertion is
 * paired with a mutation (CLAUDE.md 3.3): the load-bearing ones are the keyexpr
 * (a wrong key publishes the heading nowhere the nav subscribes) and ts_sync
 * defaulting false with no ClockStatus (CLK-A3 fail-safe).
 */

#include "sensor/rtk_driver.h"

#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using sensor::DriverConfig;
using sensor::PublishSink;
using sensor::ResolverConfig;
using sensor::RtkDriver;

static int g_failures = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
      ++g_failures;                                                       \
    }                                                                     \
  } while (0)
static bool Has(const std::string& s, const std::string& sub) {
  return s.find(sub) != std::string::npos;
}

class CaptureSink : public PublishSink {
 public:
  std::vector<std::pair<std::string, std::string>> msgs;
  void publish(const std::string& key, const std::string& payload) override {
    msgs.emplace_back(key, payload);
  }
};

static DriverConfig Cfg() {
  DriverConfig c;
  c.rid = "r1";
  c.src = "rtk_driver";
  c.boot = "9f2c1a44";
  c.sync_timeout_ms = 5000;
  c.heading_stddev_rad = 0.0175;   // ~1 deg, <= the 0.02 L1 gate
  c.gga_timeout_s = 1.0;
  c.tra_timeout_s = 1.0;
  c.rmc_timeout_s = 1.0;
  c.resolver = {0.02, 0.2, 0.5, 0.5, 2.0, 1.0, 0.5, 30.0, 0.05, 0.9, 1.0, 0.4, 0.0};
  return c;
}

// L1-good sentences (checksum-less -> tolerant parse): RTK-fixed GGA, NARROW_INT
// TRA at 90 deg (east), moving RMC.
static const char* kGga =
    "$GNGGA,015540.00,3113.31552,N,12121.24700,E,4,18,0.6,12.5,M,8.2,M,,\n";
static const char* kTra = "$GNTRA,015540.00,90.00,-2.00,0.00,4,18,0.0,0000\n";
static const char* kRmc =
    "$GNRMC,015540.00,A,3113.31552,N,12121.24700,E,2.0,90.0,140826,,,A,C\n";

static void feedAll(RtkDriver& d, double now) {
  d.feed(kGga, std::char_traits<char>::length(kGga), now);
  d.feed(kTra, std::char_traits<char>::length(kTra), now);
  d.feed(kRmc, std::char_traits<char>::length(kRmc), now);
}

static void TestPublishL1() {
  CaptureSink sink;
  RtkDriver d(Cfg(), &sink);
  double now = 100.0;
  for (int i = 0; i < 30; ++i) {   // 3.0 s at 0.1 -> past the 2.0 s recovery dwell
    feedAll(d, now);
    d.tick(now, 1700000000000LL);
    now += 0.1;
  }
  CHECK(d.level() == 1);
  CHECK(!sink.msgs.empty());
  const auto& last = sink.msgs.back();
  CHECK(last.first == "xbrain/r1/rt/gnss/heading");   // keyexpr (mutation target)
  const std::string& j = last.second;
  // envelope (11 S3.0)
  CHECK(Has(j, "\"v\":1"));
  CHECK(Has(j, "\"rid\":\"r1\""));
  CHECK(Has(j, "\"src\":\"rtk_driver\""));
  CHECK(Has(j, "\"boot\":\"9f2c1a44\""));
  CHECK(Has(j, "\"ts_sync\":false"));   // no ClockStatus -> CLK-A3 fail-safe false
  CHECK(Has(j, "\"seq\":"));
  CHECK(Has(j, "\"data\":{"));
  // data (11 S3.3): reached L1, heading = 90 deg true -> ENU 0.
  CHECK(Has(j, "\"source\":\"dual_antenna\""));
  CHECK(Has(j, "\"heading_valid\":true"));
  CHECK(Has(j, "\"level\":1"));
  CHECK(Has(j, "\"heading_rad\":0.000000"));
}

static void TestPublishL3Startup() {
  CaptureSink sink;
  RtkDriver d(Cfg(), &sink);
  d.tick(50.0, 1700000000000LL);   // no data fed -> L3 (startup / safe)
  CHECK(d.level() == 3);
  CHECK(sink.msgs.size() == 1);
  const std::string& j = sink.msgs.back().second;
  CHECK(Has(j, "\"source\":\"none\""));
  CHECK(Has(j, "\"heading_valid\":false"));
  CHECK(Has(j, "\"cov_rad\":null"));    // null, NOT 0 (NAV-02)
  CHECK(Has(j, "\"level\":3"));
}

static void TestSeqIncrements() {
  CaptureSink sink;
  RtkDriver d(Cfg(), &sink);
  d.tick(1.0, 1000LL);
  d.tick(2.0, 1000LL);
  CHECK(sink.msgs.size() == 2);
  CHECK(Has(sink.msgs[0].second, "\"seq\":1"));   // per-producer seq starts at 1
  CHECK(Has(sink.msgs[1].second, "\"seq\":2"));   // and strictly increases
}

int main() {
  TestPublishL1();
  TestPublishL3Startup();
  TestSeqIncrements();
  if (g_failures == 0) {
    std::printf("ALL RTK DRIVER TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RTK DRIVER TEST(S) FAILED\n", g_failures);
  return 1;
}
