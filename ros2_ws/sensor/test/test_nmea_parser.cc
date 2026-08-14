/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_nmea_parser.cc
 * Brief: Offline unit test for the NMEA parser (no ROS, no gtest dependency)
 *
 * Description:
 * Self-contained CTest executable (registered in CMakeLists via add_test):
 * compiles with plain g++ against nmea_parser.cc alone, so GGA / HDT / TRA / RMC
 * parsing is verified WITHOUT ROS / hardware. Uses a minimal CHECK macro
 * (non-zero exit on first failure) instead of gtest so it needs no extra
 * dependency on the build host. Includes lines captured verbatim from the bench
 * Unicore module (2026-08-14) so the field layout is checked against real output,
 * not only hand-built sentences.
 */

#include "sensor/nmea_parser.h"

#include <cmath>
#include <cstdio>
#include <string>

using sensor::GgaFix;
using sensor::HdtHeading;
using sensor::NmeaChecksumOk;
using sensor::NmeaSentenceType;
using sensor::ParseGga;
using sensor::ParseHdt;
using sensor::ParseRmc;
using sensor::ParseTra;
using sensor::RmcData;
using sensor::TraHeading;

static int g_failures = 0;

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
      ++g_failures;                                                       \
    }                                                                     \
  } while (0)

static bool Near(double a, double b, double eps) { return std::fabs(a - b) < eps; }

// ---- checksum (well-known canonical sentences) ---------------------------
static void TestChecksum() {
  // Classic NMEA example, checksum 0x47.
  CHECK(NmeaChecksumOk(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"));
  CHECK(NmeaChecksumOk("$GPHDT,274.07,T*03"));
  // Corrupted checksum must fail.
  CHECK(!NmeaChecksumOk(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00"));
  // No checksum present -> tolerant accept.
  CHECK(NmeaChecksumOk("$GNGGA,015540.00,3113.31552,N,12121.24700,E,4,18,0.6,12.5,M,8.2,M,,"));
  // Non-NMEA junk.
  CHECK(!NmeaChecksumOk("random noise"));
}

// ---- GGA single-fix (checksum-validated) ---------------------------------
static void TestGgaSingle() {
  GgaFix fix;
  const bool ok = ParseGga(
      "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
      &fix);
  CHECK(ok);
  CHECK(fix.valid);
  CHECK(Near(fix.latitude_deg, 48.1173, 1e-3));   // 48 + 7.038/60
  CHECK(Near(fix.longitude_deg, 11.51667, 1e-3));  // 11 + 31/60
  CHECK(Near(fix.altitude_m, 545.4, 1e-3));
  CHECK(fix.quality == 1);
  CHECK(fix.num_satellites == 8);
  CHECK(Near(fix.hdop, 0.9, 1e-6));
}

// ---- GGA RTK-fixed (no checksum, tolerant) -------------------------------
static void TestGgaRtkFixed() {
  GgaFix fix;
  const bool ok = ParseGga(
      "$GNGGA,015540.00,3113.31552,N,12121.24700,E,4,18,0.6,12.5,M,8.2,M,,",
      &fix);
  CHECK(ok);
  CHECK(fix.valid);
  CHECK(fix.quality == 4);                          // RTK fixed
  CHECK(Near(fix.latitude_deg, 31.221925, 1e-4));
  CHECK(Near(fix.longitude_deg, 121.354117, 1e-4));
  CHECK(fix.num_satellites == 18);
}

// ---- GGA no-fix (empty lat/lon) ------------------------------------------
static void TestGgaNoFix() {
  GgaFix fix;
  // recognised GGA, quality 0, but no position -> ok==true, valid==false.
  const bool ok = ParseGga("$GNGGA,015540.00,,,,,0,00,99.9,,M,,M,,", &fix);
  CHECK(ok);
  CHECK(!fix.valid);
  CHECK(fix.quality == 0);
}

// ---- HDT dual-antenna heading --------------------------------------------
static void TestHdt() {
  HdtHeading hdg;
  CHECK(ParseHdt("$GPHDT,274.07,T*03", &hdg));
  CHECK(hdg.valid);
  CHECK(Near(hdg.heading_true_deg, 274.07, 1e-3));
  // Wrong sentence type rejected.
  HdtHeading hdg2;
  CHECK(!ParseHdt("$GPHDT,,T", &hdg2));  // empty heading
  CHECK(!hdg2.valid);
}

// ---- TRA dual-antenna heading + QF quality -------------------------------
static void TestTra() {
  // NARROW_INT (QF=4): heading + pitch + quality + sats parsed.
  TraHeading tra;
  CHECK(ParseTra("$GPTRA,032725.00,175.85,-2.50,0.00,4,28,1.0,0000", &tra));
  CHECK(tra.valid);
  CHECK(Near(tra.heading_true_deg, 175.85, 1e-3));
  CHECK(Near(tra.pitch_deg, -2.50, 1e-3));
  CHECK(tra.quality == 4);            // NARROW_INT
  CHECK(tra.num_satellites == 28);
  // No heading solution (QF=0, real module all-zero line): parsed valid==true
  // but quality==0 signals "no heading" (downstream gates it out).
  TraHeading tra0;
  CHECK(ParseTra("$GPTRA,030158.40,0.00,0.00,0.00,0,00,0.00,0000", &tra0));
  CHECK(tra0.valid);
  CHECK(tra0.quality == 0);
  // Too few fields (cannot reach the QF column) rejected.
  TraHeading tra2;
  CHECK(!ParseTra("$GPTRA,032725.00,175.85", &tra2));
  CHECK(!tra2.valid);
  // Cross-type: TRA parser rejects HDT, HDT parser rejects TRA.
  TraHeading tra3;
  CHECK(!ParseTra("$GPHDT,274.07,T*03", &tra3));
  HdtHeading hdg;
  CHECK(!ParseHdt("$GPTRA,032725.00,175.85,-2.50,0.00,4,28,1.0,0000", &hdg));
}

// ---- TRA on REAL bench output (locks the QF 4/5 mapping) -----------------
static void TestTraBench() {
  // Captured verbatim 2026-08-14: QF=5 <-> NARROW_FLOAT (checksum stripped for a
  // tolerant parse). Confirms field 5 is the QF and the layout matches.
  TraHeading tra;
  CHECK(ParseTra("$GNTRA,092703.05,176.88,-6.72,0.00,5,12,0.00,0000", &tra));
  CHECK(tra.valid);
  CHECK(Near(tra.heading_true_deg, 176.88, 1e-3));
  CHECK(tra.quality == 5);            // NARROW_FLOAT (bench-verified)
  CHECK(tra.num_satellites == 12);
}

// ---- RMC course + speed over ground (L2 COG source) ----------------------
static void TestRmc() {
  constexpr double kKn = 1852.0 / 3600.0;
  // Moving: course + speed present. 1.9438 kn -> ~1.0 m/s.
  RmcData rmc;
  CHECK(ParseRmc(
      "$GNRMC,092751.00,A,3441.8343,N,13530.3198,E,1.9438,87.5,140826,8.1,W,A,C",
      &rmc));
  CHECK(rmc.valid);
  CHECK(rmc.status_active);
  CHECK(Near(rmc.speed_mps, 1.9438 * kKn, 1e-6));
  CHECK(Near(rmc.cog_deg, 87.5, 1e-3));
  CHECK(rmc.cog_present);
  // Standstill with an EMPTY course field: cog_present false, NOT read as 0 deg.
  RmcData rmc2;
  CHECK(ParseRmc(
      "$GNRMC,092751.00,A,3441.8343,N,13530.3198,E,0.010,,140826,8.1,W,A,C",
      &rmc2));
  CHECK(rmc2.valid);
  CHECK(!rmc2.cog_present);
  CHECK(Near(rmc2.speed_mps, 0.010 * kKn, 1e-9));
  // Void status ('V'): parsed, but status_active false.
  RmcData rmc3;
  CHECK(ParseRmc("$GNRMC,092751.00,V,,,,,,,140826,,,N,V", &rmc3));
  CHECK(rmc3.valid);
  CHECK(!rmc3.status_active);
  CHECK(!rmc3.cog_present);
  // Real bench line (course present but garbage at a standstill -- the parser
  // still reports it present; the resolver's SPEED gate, not this, withholds L2).
  RmcData rmc4;
  CHECK(ParseRmc(
      "$GNRMC,092829.55,A,3441.83986627,N,13530.32137003,E,0.050,303.3,140826,8.1,W,A,C",
      &rmc4));
  CHECK(rmc4.valid);
  CHECK(rmc4.cog_present);
  CHECK(Near(rmc4.cog_deg, 303.3, 1e-3));
  // Too few fields (cannot reach the course column) rejected.
  RmcData rmc5;
  CHECK(!ParseRmc("$GNRMC,092751.00,A,3441.8343,N", &rmc5));
  // Cross-reject: RMC parser rejects a GGA.
  RmcData rmc6;
  CHECK(!ParseRmc(
      "$GNGGA,015540.00,3113.31552,N,12121.24700,E,4,18,0.6,12.5,M,8.2,M,,",
      &rmc6));
}

// ---- cross: GGA parser rejects a non-GGA / type detection ----------------
static void TestTypeAndCrossReject() {
  CHECK(NmeaSentenceType("$GNGGA,...") == "GGA");
  CHECK(NmeaSentenceType("$GPHDT,...") == "HDT");
  CHECK(NmeaSentenceType("garbage").empty());
  GgaFix fix;
  CHECK(!ParseGga("$GPHDT,274.07,T*03", &fix));  // HDT is not a GGA
}

int main() {
  TestChecksum();
  TestGgaSingle();
  TestGgaRtkFixed();
  TestGgaNoFix();
  TestHdt();
  TestTra();
  TestTraBench();
  TestRmc();
  TestTypeAndCrossReject();
  if (g_failures == 0) {
    std::printf("ALL NMEA PARSER TESTS PASSED\n");
    return 0;
  }
  std::printf("%d NMEA PARSER TEST(S) FAILED\n", g_failures);
  return 1;
}
