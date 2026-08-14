/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: nmea_parser.h
 * Brief: Pure NMEA-0183 parser for the Unicore dual-antenna RTK module (rtk_driver)
 *
 * Description:
 * The problem this solves. rtk_driver must turn the module's raw serial lines
 * into the four facts the 11 S3.3 heading resolver needs -- position + fix
 * quality (GGA), dual-antenna heading + its fix quality (TRA), and course/speed
 * over ground (RMC, the L2 COG fallback). This file is ONLY that parse: stateless
 * and ROS-free so it compiles with plain g++ and is unit-tested offline with no
 * rclcpp, no zenoh, no hardware.
 *
 * Which sentences and why. The bench module (Unicore UM982-class, WCH CH343 USB
 * serial, 115200 8N1, 20 Hz) emits GN-talker NMEA:
 *   GGA -- lat/lon/alt + fix quality (1 single .. 4 RTK-fixed .. 5 float);
 *   TRA -- dual-antenna heading + pitch + QF (4 = NARROW_INT, 5 = NARROW_FLOAT),
 *          VERIFIED on the bench 2026-08-14 (QF 4 <-> INT, 5 <-> FLOAT);
 *   RMC -- course-over-ground + speed-over-ground, the L2 source when the
 *          dual-antenna solution is unavailable AND the platform is moving.
 * Talker id is ignored ($GP / $GN / $BD all accepted) -- this device sends $GN.
 *
 * What it does NOT do, and the boundary. It does NOT decide heading_valid /
 * level / source -- that is the 11 S3.3.1 degradation chain, a stateful decision
 * (COG speed gate, hysteresis) that belongs to the resolver, not the parser. It
 * does NOT convert angles to the ENU convention or radians -- it returns the raw
 * true-north-clockwise DEGREES the module sends; the resolver applies
 * heading_enu = wrap(pi/2 - heading_ned). It has no clock and no I/O.
 *
 * Traps this exists to avoid. NMEA leaves optional numeric fields EMPTY (not 0),
 * so an empty field must not read as a real zero heading -- callers gate on the
 * per-struct `valid`. The checksum is tolerant of a missing *HH (some configs
 * omit it) but rejects a present-but-wrong one, so garbage mid-line cannot be
 * mistaken for a fix. No exception is ever thrown: every entry point returns a
 * bool and fills an out struct; malformed input yields valid == false.
 */

#ifndef SENSOR__NMEA_PARSER_H_
#define SENSOR__NMEA_PARSER_H_

#include <string>
#include <vector>

namespace sensor {

// GGA-derived position fix. quality is the raw NMEA GGA fix-quality field:
//   0 = invalid, 1 = single GPS, 2 = DGPS, 4 = RTK fixed, 5 = RTK float,
//   6 = dead-reckoning. The node maps this to NavSatStatus (see node).
struct GgaFix {
  bool valid = false;          // true iff a position was parsed
  double latitude_deg = 0.0;   // decimal degrees, + = North
  double longitude_deg = 0.0;  // decimal degrees, + = East
  double altitude_m = 0.0;     // metres above mean sea level
  int quality = 0;             // raw GGA fix-quality field
  int num_satellites = 0;
  double hdop = 0.0;           // horizontal dilution of precision
};

// HDT-derived dual-antenna heading. heading_true_deg is degrees from TRUE
// north, clockwise (compass convention) -- the node converts to the RNS
// kinematic convention (radians, 0 = East, ccw positive).
struct HdtHeading {
  bool valid = false;
  double heading_true_deg = 0.0;
};

// TRA-derived dual-antenna attitude + quality. Unlike HDT (heading only),
// the Unicore $GPTRA sentence carries heading/pitch AND a quality flag (QF),
// letting the node gate on the heading FIX quality (feat/dual-source-heading
// NARROW_INT gate). QF (mirrors the GGA fix-quality convention):
//   0 = no heading solution, 4 = fixed-integer (NARROW_INT, cm-class heading,
//   TRUST), 5 = float (NARROW_FLOAT, weak). heading_true_deg is TRUE-north
//   clockwise degrees (same convention as HDT).
struct TraHeading {
  bool valid = false;          // true iff a TRA line was parsed (any QF)
  double heading_true_deg = 0.0;
  double pitch_deg = 0.0;
  int quality = 0;             // QF: 0 none / 4 fixed(NARROW_INT) / 5 float
  int num_satellites = 0;
};

// RMC-derived course + speed over ground: the L2 (COG) heading source. cog_deg
// is TRUE-north clockwise degrees (same convention as HDT/TRA). speed_mps is the
// GNSS-Doppler speed over ground (RMC reports it in KNOTS; this struct carries
// the SI value). status_active mirrors the RMC A/V flag. cog_present is false
// when the module leaves the course field EMPTY -- which it does at a standstill,
// where COG is undefined; the resolver, not the parser, applies the speed gate
// (11 S3.3.1) that decides whether COG may be trusted.
struct RmcData {
  bool valid = false;          // true iff an RMC line was parsed
  bool status_active = false;  // RMC field 2: 'A' active / 'V' void
  double speed_mps = 0.0;      // speed over ground, m/s (converted from knots)
  double cog_deg = 0.0;        // course over ground, true-north clockwise degrees
  bool cog_present = false;    // false when the RMC course field was empty
};

// XOR checksum of every char between '$' and '*'. Returns true when the
// trailing *HH matches, OR when no '*' is present (tolerant: some configs
// emit checksum-less lines). Returns false only on a present-but-wrong sum.
bool NmeaChecksumOk(const std::string& sentence);

// The 3-char sentence type after the talker id, e.g. "GGA" / "HDT". Returns
// "" when the string is not a recognisable NMEA sentence ($ttTYP,...).
std::string NmeaSentenceType(const std::string& sentence);

// Split a sentence into comma fields: field[0] is the address ($GNGGA),
// subsequent fields are the data columns (the trailing *HH is stripped off
// the final field). Returns {} for a non-NMEA line.
std::vector<std::string> NmeaSplitFields(const std::string& sentence);

// Convert an NMEA ddmm.mmmm / dddmm.mmmm magnitude + hemisphere to signed
// decimal degrees. Empty magnitude returns 0.0. S / W hemispheres negate.
double NmeaLatLonToDegrees(const std::string& magnitude,
                           const std::string& hemisphere);

// Parse a GGA sentence. Returns false (and out->valid == false) when the
// sentence is not a GGA, the checksum is wrong, or it carries no position.
bool ParseGga(const std::string& sentence, GgaFix* out);

// Parse an HDT sentence. Returns false on non-HDT / bad checksum / empty.
bool ParseHdt(const std::string& sentence, HdtHeading* out);

// Parse a TRA ($GPTRA) sentence. Returns false on non-TRA / bad checksum /
// too-few fields. out->quality carries the QF flag (0/4/5) so the caller can
// gate on NARROW_INT (QF==4). A parsed line with QF==0 still returns true
// (valid==true) but signals "no heading solution" via quality.
bool ParseTra(const std::string& sentence, TraHeading* out);

// Parse an RMC sentence (position/velocity/time). Returns false on non-RMC / bad
// checksum / too-few fields. out->speed_mps is converted from the knots field;
// out->cog_present is false when the course field is empty (standstill), so the
// resolver can withhold the L2 COG source rather than trust an undefined course.
bool ParseRmc(const std::string& sentence, RmcData* out);

}  // namespace sensor

#endif  // SENSOR__NMEA_PARSER_H_
