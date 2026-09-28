/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: nmea_parser.cc
 * Brief: Pure NMEA-0183 parser implementation (see nmea_parser.h)
 *
 * Description:
 * Implements GGA / HDT / TRA / RMC parsing for the Unicore dual-antenna RTK
 * module (rtk_driver). No ROS, no zenoh, no clock, no exceptions: every entry
 * point returns a bool + fills an out struct. Compiled stand-alone (plain g++)
 * for offline unit tests. The heading/level DECISION is the resolver's, not
 * here (see nmea_parser.h "what it does NOT do").
 */

#include "sensor/nmea_parser.h"

#include <cctype>
#include <cmath>
#include <cstdlib>

namespace sensor {

namespace {

// Parse a hex nibble; returns -1 on a non-hex char.
int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Find the start ('$') so leading serial noise before the sentence is
// tolerated. Returns std::string::npos when absent.
size_t find_start(const std::string& s) { return s.find('$'); }

// strtod that treats an empty / blank field as 0.0 (NMEA leaves optional
// numeric fields empty rather than zero).
double field_to_double(const std::string& f) {
  if (f.empty()) return 0.0;
  return std::strtod(f.c_str(), nullptr);
}

int field_to_int(const std::string& f) {
  if (f.empty()) return 0;
  return static_cast<int>(std::strtol(f.c_str(), nullptr, 10));
}

}  // namespace

bool nmea_checksum_ok(const std::string& sentence) {
  const size_t start = find_start(sentence);
  if (start == std::string::npos) return false;
  const size_t star = sentence.find('*', start);
  if (star == std::string::npos) {
    // No checksum present -> tolerant accept (some configs omit it).
    return true;
  }
  // XOR everything strictly between '$' and '*'.
  unsigned char xsum = 0;
  for (size_t i = start + 1; i < star; ++i) {
    xsum ^= static_cast<unsigned char>(sentence[i]);
  }
  // Need two hex chars after '*'.
  if (star + 2 >= sentence.size()) return false;
  const int hi = hex_nibble(sentence[star + 1]);
  const int lo = hex_nibble(sentence[star + 2]);
  if (hi < 0 || lo < 0) return false;
  return xsum == static_cast<unsigned char>((hi << 4) | lo);
}

std::vector<std::string> nmea_split_fields(const std::string& sentence) {
  std::vector<std::string> fields;
  const size_t start = find_start(sentence);
  if (start == std::string::npos) return fields;
  // Stop at '*' (checksum) or end of line.
  size_t end = sentence.find('*', start);
  if (end == std::string::npos) end = sentence.size();
  // Also stop at CR/LF if present before '*'.
  for (size_t i = start; i < end; ++i) {
    if (sentence[i] == '\r' || sentence[i] == '\n') {
      end = i;
      break;
    }
  }
  std::string cur;
  for (size_t i = start; i < end; ++i) {
    if (sentence[i] == ',') {
      fields.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(sentence[i]);
    }
  }
  fields.push_back(cur);
  return fields;
}

std::string nmea_sentence_type(const std::string& sentence) {
  const size_t start = find_start(sentence);
  if (start == std::string::npos) return "";
  // Address is "$ttTYP" -> talker (2) + type (3). Need at least 6 chars.
  if (sentence.size() < start + 6) return "";
  return sentence.substr(start + 3, 3);
}

double nmea_lat_lon_to_degrees(const std::string& magnitude,
                           const std::string& hemisphere) {
  if (magnitude.empty()) return 0.0;
  const double raw = std::strtod(magnitude.c_str(), nullptr);
  // ddmm.mmmm: degrees are the value / 100 (integer part), minutes the rest.
  const double deg = std::floor(raw / 100.0);
  const double minutes = raw - deg * 100.0;
  double result = deg + minutes / 60.0;
  if (!hemisphere.empty() && (hemisphere[0] == 'S' || hemisphere[0] == 'W')) {
    result = -result;
  }
  return result;
}

bool parse_gga(const std::string& sentence, GgaFix* out) {
  if (out == nullptr) return false;
  *out = GgaFix();
  if (nmea_sentence_type(sentence) != "GGA") return false;
  if (!nmea_checksum_ok(sentence)) return false;
  const std::vector<std::string> f = nmea_split_fields(sentence);
  // GGA columns: 0 addr,1 utc,2 lat,3 N/S,4 lon,5 E/W,6 quality,7 numSV,
  // 8 HDOP,9 alt,10 M,11 geoidSep,...  -> need at least 10 fields.
  if (f.size() < 10) return false;
  // No position fix yet -> report a parsed-but-invalid GGA (quality kept so
  // the node can still publish a NONE-status fix for the R88 grace timer).
  out->quality = field_to_int(f[6]);
  out->num_satellites = field_to_int(f[7]);
  out->hdop = field_to_double(f[8]);
  if (f[2].empty() || f[4].empty()) {
    out->valid = false;
    return true;  // recognised GGA, just no lat/lon
  }
  out->latitude_deg = nmea_lat_lon_to_degrees(f[2], f[3]);
  out->longitude_deg = nmea_lat_lon_to_degrees(f[4], f[5]);
  out->altitude_m = field_to_double(f[9]);
  out->valid = true;
  return true;
}

bool parse_hdt(const std::string& sentence, HdtHeading* out) {
  if (out == nullptr) return false;
  *out = HdtHeading();
  if (nmea_sentence_type(sentence) != "HDT") return false;
  if (!nmea_checksum_ok(sentence)) return false;
  const std::vector<std::string> f = nmea_split_fields(sentence);
  // HDT columns: 0 addr, 1 heading_deg, 2 'T'. Need >= 2 fields + a value.
  if (f.size() < 2 || f[1].empty()) return false;
  out->heading_true_deg = field_to_double(f[1]);
  out->valid = true;
  return true;
}

bool parse_tra(const std::string& sentence, TraHeading* out) {
  if (out == nullptr) return false;
  *out = TraHeading();
  if (nmea_sentence_type(sentence) != "TRA") return false;
  if (!nmea_checksum_ok(sentence)) return false;
  const std::vector<std::string> f = nmea_split_fields(sentence);
  // $GPTRA columns: 0 addr, 1 utc, 2 heading, 3 pitch, 4 roll, 5 QF,
  // 6 numSV, 7 age, 8 stnID. Need >= 6 fields to reach the QF column.
  if (f.size() < 6) return false;
  out->heading_true_deg = field_to_double(f[2]);
  out->pitch_deg = field_to_double(f[3]);
  out->quality = field_to_int(f[5]);
  out->num_satellites = (f.size() > 6) ? field_to_int(f[6]) : 0;
  out->valid = true;  // parsed OK; quality field says if the heading is usable
  return true;
}

bool parse_rmc(const std::string& sentence, RmcData* out) {
  if (out == nullptr) return false;
  *out = RmcData();
  if (nmea_sentence_type(sentence) != "RMC") return false;
  if (!nmea_checksum_ok(sentence)) return false;
  const std::vector<std::string> f = nmea_split_fields(sentence);
  // RMC columns: 0 addr, 1 utc, 2 status(A/V), 3 lat, 4 N/S, 5 lon, 6 E/W,
  // 7 speed(knots), 8 cog(deg true), 9 date, ...  Need >= 9 to reach cog.
  if (f.size() < 9) return false;
  out->status_active = (!f[2].empty() && f[2][0] == 'A');
  // RMC reports speed over ground in KNOTS; convert to m/s (1 kn = 1852 m/3600 s).
  constexpr double kKnotToMps = 1852.0 / 3600.0;
  out->speed_mps = field_to_double(f[7]) * kKnotToMps;
  // The module leaves the course field EMPTY at a standstill (COG undefined).
  // Report the absence so the resolver withholds L2 rather than reading 0 as
  // "heading due North" -- the exact fabricated-heading the resolver must avoid.
  out->cog_present = !f[8].empty();
  out->cog_deg = field_to_double(f[8]);
  out->valid = true;
  return true;
}

}  // namespace sensor
