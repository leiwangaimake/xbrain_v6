/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_gga_vrs.cc
 * Brief: Offline unit test for GGA formatting and the VRS anchor (RTG-7 / RTG-12)
 *
 * Description:
 * Expected GGA strings and checksums were computed outside this code (python,
 * XOR over the body) so a formatter and its own checksum cannot agree on a
 * wrong answer. The anchor cases encode the location requirement: no position
 * or a stale one is not reportable, Shanghai -> Nanjing (about 270 km) asks for
 * a rebuilt virtual station, moving around a site does not.
 */

#include "sensor/gga_vrs.h"

#include <cmath>
#include <cstdio>
#include <string>

using sensor::build_gga;
using sensor::GgaPosition;
using sensor::gga_quality_from_fix_type;
using sensor::great_circle_km;
using sensor::PositionSample;
using sensor::VrsAnchor;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

static GgaPosition pos(double lat, double lon, double alt, int q, int sats, double hdop) {
  GgaPosition p;
  p.lat_deg = lat; p.lon_deg = lon; p.alt_m = alt;
  p.quality = q; p.sats = sats; p.hdop = hdop;
  return p;
}

static void test_quality_digits() {
  CHECK(gga_quality_from_fix_type("rtk_fixed") == 4);
  CHECK(gga_quality_from_fix_type("rtk_float") == 5);
  CHECK(gga_quality_from_fix_type("dgps") == 2);
  CHECK(gga_quality_from_fix_type("single") == 1);
  CHECK(gga_quality_from_fix_type("no_fix") == 0);
}

static void test_gga_format() {
  const GgaPosition sh = pos(31.2219253, 121.4541167, 12.5, 1, 8, 0.9);
  CHECK(build_gga(sh, 3 * 3600 + 4 * 60 + 5.5) ==
        "$GPGGA,030405.50,3113.31552,N,12127.24700,E,1,08,0.9,12.500,M,,M,,*44\r\n");
}

static void test_gga_minute_carry() {
  // 30.99999999999 deg is 59.9999999994' -> rounds to 60' -> must carry to 31 deg.
  const GgaPosition p = pos(30.99999999999, 120.99999999999, 5.0, 4, 12, 0.6);
  CHECK(build_gga(p, 0.0) ==
        "$GPGGA,000000.00,3100.00000,N,12100.00000,E,4,12,0.6,5.000,M,,M,,*77\r\n");
}

static void test_gga_south_west_and_day_end() {
  const GgaPosition p = pos(-33.5, -70.25, -3.25, 5, 10, 1.2);
  CHECK(build_gga(p, 86399.99) ==
        "$GPGGA,235959.99,3330.00000,S,07015.00000,W,5,10,1.2,-3.250,M,,M,,*53\r\n");
}

static void test_great_circle() {
  // Shanghai -> Nanjing city centres: 269.66 km (reference value, python haversine).
  CHECK(std::fabs(great_circle_km(31.2304, 121.4737, 32.0603, 118.7969) - 269.66) < 0.05);
  CHECK(std::fabs(great_circle_km(31.2304, 121.4737, 31.2394, 121.4737) - 1.0008) < 0.001);
  CHECK(great_circle_km(10.0, 179.9, 10.0, -179.9) < 25.0);   // across the antimeridian
}

static void test_reportable() {
  VrsAnchor a(5.0, 10.0);
  PositionSample s;
  CHECK(!a.reportable(s, 100.0));             // no position at all
  s.has_position = true;
  s.pos = pos(31.2, 121.4, 10.0, 1, 8, 0.9);
  s.t_mono = 96.0;
  CHECK(!a.reportable(s, 100.0));             // no receiver time to stamp the GGA with
  s.has_utc = true;
  s.utc_s_of_day = 3600.0;
  CHECK(a.reportable(s, 100.0));
  CHECK(a.reportable(s, 101.0));              // exactly max_pos_age_s old
  CHECK(!a.reportable(s, 101.01));            // too old: wait, do not report it
  s.has_position = false;
  CHECK(!a.reportable(s, 96.0));              // a fix_type without position
}

static void test_rebase() {
  VrsAnchor a(5.0, 10.0);
  const GgaPosition shanghai = pos(31.2304, 121.4737, 5.0, 1, 8, 0.9);
  const GgaPosition nanjing = pos(32.0603, 118.7969, 15.0, 1, 8, 0.9);
  CHECK(!a.needsRebase(nanjing));             // no anchor yet: nothing to compare
  a.setAnchor(shanghai);
  CHECK(a.needsRebase(nanjing));
  CHECK(!a.needsRebase(pos(31.2394, 121.4737, 5.0, 4, 20, 0.6)));   // 1 km across a site
  CHECK(!a.needsRebase(pos(31.2304 + 9.99 / 111.195, 121.4737, 5.0, 4, 20, 0.6)));
  CHECK(a.needsRebase(pos(31.2304 + 10.01 / 111.195, 121.4737, 5.0, 4, 20, 0.6)));
  a.clear();
  CHECK(!a.hasAnchor());
  CHECK(!a.needsRebase(nanjing));
}

int main() {
  test_quality_digits();
  test_gga_format();
  test_gga_minute_carry();
  test_gga_south_west_and_day_end();
  test_great_circle();
  test_reportable();
  test_rebase();
  if (g_failures == 0) {
    std::printf("ALL GGA VRS TESTS PASSED\n");
    return 0;
  }
  std::printf("%d GGA VRS TEST(S) FAILED\n", g_failures);
  return 1;
}
