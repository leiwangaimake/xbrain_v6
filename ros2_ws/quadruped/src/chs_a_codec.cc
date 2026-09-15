/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_a_codec.cc
 * Brief: CHS-A codec implementation (see chs_a_codec.h)
 *
 * Description:
 * Every encoder follows the same three steps -- render the ASDU into the space
 * AFTER the header, then write the header now that the length is known, then
 * return the total. Rendering first is what makes the length field correct by
 * construction rather than by a second calculation that can drift.
 *
 * snprintf is the whole JSON writer. That is not a shortcut: the five payloads
 * we send are fixed shapes with numeric leaves, and a library would add an
 * allocation on the 100 Hz path for no expressive gain (QD-7). The receive
 * side is a different question and gets a real parser in B2, where the shapes
 * are the chassis's and change with its firmware.
 *
 * WALL-CLOCK-OK(align): the ASDU "Time" field is a protocol field the chassis
 * requires in local wall-clock form; it is never used for an age, a timeout or
 * a period, all of which read CLOCK_MONOTONIC (CLK-C1). The value is passed in
 * by the caller so this file performs no clock read at all.
 */

#include "quadruped/chs_a_codec.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace quadruped {
namespace chs_a {

namespace {

// "YYYY-MM-DD HH:MM:SS" plus the terminator.
constexpr std::size_t kTimeBufBytes = 24;

// Render the mandatory Time field. localtime_r rather than localtime: the
// latter returns a pointer to shared state, and this function is called from
// the ctrl thread and the non-realtime tx thread.
// * Falls back to a fixed, obviously-wrong-looking string if the conversion
//   fails, instead of leaving the buffer uninitialised. A garbage Time is
//   rejected by the chassis with 0xE002, which is loud; uninitialised stack
//   bytes would sometimes parse and sometimes not.
void FormatTime(std::int64_t now_wall, char* out, std::size_t cap) {
  std::time_t t = static_cast<std::time_t>(now_wall);
  std::tm tm_buf;
  if (localtime_r(&t, &tm_buf) == nullptr ||
      std::strftime(out, cap, "%Y-%m-%d %H:%M:%S", &tm_buf) == 0) {
    std::snprintf(out, cap, "1970-01-01 00:00:00");
  }
}

// Finish a frame: write the header in front of an already-rendered ASDU.
// Returns the total frame size, or 0 if the ASDU did not fit or is too large
// for the 16-bit length field.
std::size_t Finish(std::uint8_t* buf, std::size_t cap, std::uint16_t msg_id,
                   int asdu_written) {
  if (asdu_written <= 0) return 0;
  const std::size_t asdu_len = static_cast<std::size_t>(asdu_written);
  // snprintf returns what it WOULD have written; a value that reaches the end
  // of the available space means the payload was truncated, and a truncated
  // JSON object is a parse error at the far end.
  if (asdu_len > kMaxAsduBytes) return 0;
  if (kHeaderBytes + asdu_len > cap) return 0;
  Header h;
  h.asdu_len = static_cast<std::uint16_t>(asdu_len);
  h.msg_id = msg_id;
  if (!WriteHeader(h, buf, cap)) return 0;
  return kHeaderBytes + asdu_len;
}

// Space available for the ASDU given the whole-frame capacity.
std::size_t AsduCap(std::size_t cap) {
  return cap > kHeaderBytes ? cap - kHeaderBytes : 0;
}

char* AsduStart(std::uint8_t* buf) {
  return reinterpret_cast<char*>(buf + kHeaderBytes);
}

// Guard for every float that goes on the wire. Non-finite values would render
// as "nan"/"inf", which is not JSON and which the chassis rejects as OUR bug.
bool AllFinite(const AxisCommand& c) {
  return std::isfinite(c.vx) && std::isfinite(c.vy) && std::isfinite(c.vz) &&
         std::isfinite(c.roll) && std::isfinite(c.pitch) && std::isfinite(c.yaw);
}

// Find "\"name\":" inside an ASDU and return the position just past the colon,
// or npos-equivalent (len) when absent. A hand-rolled scan rather than a JSON
// parse because B1 needs three integers for routing and B2 replaces this with
// a real parser for the payloads.
// * Deliberately does NOT skip nested objects: the three fields it looks for
//   are at the top level of PatrolDevice, and a match inside a nested object
//   would be a different key with the same name -- none exists in this
//   protocol, and B2's parser removes the question entirely.
std::size_t FindValue(const char* s, std::size_t len, const char* name) {
  const std::size_t nlen = std::strlen(name);
  for (std::size_t i = 0; i + nlen + 2 < len; ++i) {
    if (s[i] != '"') continue;
    if (std::strncmp(s + i + 1, name, nlen) != 0) continue;
    std::size_t j = i + 1 + nlen;
    if (j >= len || s[j] != '"') continue;
    ++j;
    while (j < len && (s[j] == ' ' || s[j] == '\t')) ++j;
    if (j >= len || s[j] != ':') continue;
    ++j;
    while (j < len && (s[j] == ' ' || s[j] == '\t')) ++j;
    return j;
  }
  return len;
}

// Read an unsigned decimal at position p. Returns false when there is no digit
// there, which is how a field that exists but holds an object or a string is
// rejected rather than read as 0.
bool ReadU32(const char* s, std::size_t len, std::size_t p, std::uint32_t* out) {
  if (p >= len || s[p] < '0' || s[p] > '9') return false;
  std::uint64_t v = 0;
  while (p < len && s[p] >= '0' && s[p] <= '9') {
    v = v * 10u + static_cast<std::uint64_t>(s[p] - '0');
    if (v > 0xFFFFFFFFull) return false;  // not a 32-bit code; refuse to wrap
    ++p;
  }
  *out = static_cast<std::uint32_t>(v);
  return true;
}

}  // namespace

bool WriteHeader(const Header& h, std::uint8_t* buf, std::size_t cap) {
  if (buf == nullptr || cap < kHeaderBytes) return false;
  buf[0] = kSync0;
  buf[1] = kSync1;
  buf[2] = kSync2;
  buf[3] = kSync3;
  // Little-endian, written byte by byte rather than by memcpy of a uint16 so
  // the file reads the same on any host endianness.
  buf[4] = static_cast<std::uint8_t>(h.asdu_len & 0xFFu);
  buf[5] = static_cast<std::uint8_t>((h.asdu_len >> 8) & 0xFFu);
  buf[6] = static_cast<std::uint8_t>(h.msg_id & 0xFFu);
  buf[7] = static_cast<std::uint8_t>((h.msg_id >> 8) & 0xFFu);
  buf[8] = h.format;
  buf[9] = h.packet_no;
  buf[10] = h.version;
  // The five reserved bytes MUST be zero. The vendor's own example leaves them
  // uninitialised on the stack (13 V-42), so anything that works there is
  // accidental; zero is the only value the field table sanctions.
  std::memset(buf + 11, 0, 5);
  return true;
}

bool ReadHeader(const std::uint8_t* buf, std::size_t len, Header* out) {
  if (buf == nullptr || out == nullptr || len < kHeaderBytes) return false;
  if (buf[0] != kSync0 || buf[1] != kSync1 || buf[2] != kSync2 ||
      buf[3] != kSync3) {
    return false;
  }
  out->asdu_len = static_cast<std::uint16_t>(buf[4] |
                                             (static_cast<std::uint16_t>(buf[5]) << 8));
  out->msg_id = static_cast<std::uint16_t>(buf[6] |
                                           (static_cast<std::uint16_t>(buf[7]) << 8));
  out->format = buf[8];
  out->packet_no = buf[9];
  out->version = buf[10];
  return true;
}

std::size_t EncodeHeartbeat(std::uint8_t* buf, std::size_t cap,
                            std::uint16_t msg_id, std::int64_t now_wall) {
  if (buf == nullptr) return 0;
  char ts[kTimeBufBytes];
  FormatTime(now_wall, ts, sizeof(ts));
  const int n = std::snprintf(
      AsduStart(buf), AsduCap(cap),
      "{\"PatrolDevice\": {\"Type\": %u, \"Command\": %u, \"Time\": \"%s\", "
      "\"Items\": {}}}",
      kHeartbeat.type, kHeartbeat.command, ts);
  return (n > 0 && static_cast<std::size_t>(n) < AsduCap(cap))
             ? Finish(buf, cap, msg_id, n)
             : 0;
}

std::size_t EncodeUsageMode(std::uint8_t* buf, std::size_t cap,
                            std::uint16_t msg_id, std::int64_t now_wall,
                            int mode) {
  if (buf == nullptr) return 0;
  char ts[kTimeBufBytes];
  FormatTime(now_wall, ts, sizeof(ts));
  const int n = std::snprintf(
      AsduStart(buf), AsduCap(cap),
      "{\"PatrolDevice\": {\"Type\": %u, \"Command\": %u, \"Time\": \"%s\", "
      "\"Items\": {\"Mode\": %d}}}",
      kUsageModeSwitch.type, kUsageModeSwitch.command, ts, mode);
  return (n > 0 && static_cast<std::size_t>(n) < AsduCap(cap))
             ? Finish(buf, cap, msg_id, n)
             : 0;
}

std::size_t EncodeMotionState(std::uint8_t* buf, std::size_t cap,
                              std::uint16_t msg_id, std::int64_t now_wall,
                              int motion_param) {
  if (buf == nullptr) return 0;
  char ts[kTimeBufBytes];
  FormatTime(now_wall, ts, sizeof(ts));
  const int n = std::snprintf(
      AsduStart(buf), AsduCap(cap),
      "{\"PatrolDevice\": {\"Type\": %u, \"Command\": %u, \"Time\": \"%s\", "
      "\"Items\": {\"MotionParam\": %d}}}",
      kMotionStateSwitch.type, kMotionStateSwitch.command, ts, motion_param);
  return (n > 0 && static_cast<std::size_t>(n) < AsduCap(cap))
             ? Finish(buf, cap, msg_id, n)
             : 0;
}

std::size_t EncodeGait(std::uint8_t* buf, std::size_t cap, std::uint16_t msg_id,
                       std::int64_t now_wall, std::uint32_t gait_param) {
  if (buf == nullptr) return 0;
  char ts[kTimeBufBytes];
  FormatTime(now_wall, ts, sizeof(ts));
  const int n = std::snprintf(
      AsduStart(buf), AsduCap(cap),
      "{\"PatrolDevice\": {\"Type\": %u, \"Command\": %u, \"Time\": \"%s\", "
      "\"Items\": {\"GaitParam\": %u}}}",
      kGaitSwitch.type, kGaitSwitch.command, ts, gait_param);
  return (n > 0 && static_cast<std::size_t>(n) < AsduCap(cap))
             ? Finish(buf, cap, msg_id, n)
             : 0;
}

std::size_t EncodeRealAxis(std::uint8_t* buf, std::size_t cap,
                           std::uint16_t msg_id, std::int64_t now_wall,
                           const AxisCommand& cmd) {
  if (buf == nullptr) return 0;
  // Checked before anything is rendered: a partially written buffer that is
  // then rejected is harder to reason about than one that was never touched.
  if (!AllFinite(cmd)) return 0;
  char ts[kTimeBufBytes];
  FormatTime(now_wall, ts, sizeof(ts));
  // Six decimals: the chassis limit is 2 m/s and 1 rad/s, so 1e-6 is far below
  // any actuator resolution, and a fixed format keeps the frame length stable
  // (which %g would not -- it switches to exponent form for small values, and
  // 1e-07 is valid JSON but changes the byte count tick to tick).
  const int n = std::snprintf(
      AsduStart(buf), AsduCap(cap),
      "{\"PatrolDevice\": {\"Type\": %u, \"Command\": %u, \"Time\": \"%s\", "
      "\"Items\": {\"X\": %.6f, \"Y\": %.6f, \"Z\": %.6f, \"Roll\": %.6f, "
      "\"Pitch\": %.6f, \"Yaw\": %.6f}}}",
      kRealAxis.type, kRealAxis.command, ts, cmd.vx, cmd.vy, cmd.vz, cmd.roll,
      cmd.pitch, cmd.yaw);
  return (n > 0 && static_cast<std::size_t>(n) < AsduCap(cap))
             ? Finish(buf, cap, msg_id, n)
             : 0;
}

std::size_t EncodeSdkMode(std::uint8_t* buf, std::size_t cap,
                          std::uint16_t msg_id, std::int64_t now_wall,
                          bool enable, int joint_rate_hz) {
  if (buf == nullptr) return 0;
  char ts[kTimeBufBytes];
  FormatTime(now_wall, ts, sizeof(ts));
  const int n = std::snprintf(
      AsduStart(buf), AsduCap(cap),
      "{\"PatrolDevice\": {\"Type\": %u, \"Command\": %u, \"Time\": \"%s\", "
      "\"Items\": {\"SDKEnable\": %s, \"Frequency\": %d}}}",
      kSdkMode.type, kSdkMode.command, ts, enable ? "true" : "false",
      joint_rate_hz);
  return (n > 0 && static_cast<std::size_t>(n) < AsduCap(cap))
             ? Finish(buf, cap, msg_id, n)
             : 0;
}

bool ParseAsduRouting(const std::uint8_t* asdu, std::size_t len,
                      AsduRouting* out) {
  if (asdu == nullptr || out == nullptr || len == 0) return false;
  const char* s = reinterpret_cast<const char*>(asdu);
  // The wrapper must be there. Checking it here as well as trusting the fields
  // means a payload from some other protocol that happened to contain a "Type"
  // key is rejected instead of routed.
  if (FindValue(s, len, "PatrolDevice") == len) return false;
  const std::size_t tp = FindValue(s, len, "Type");
  const std::size_t cp = FindValue(s, len, "Command");
  if (tp == len || cp == len) return false;
  AsduRouting r;
  if (!ReadU32(s, len, tp, &r.type)) return false;
  if (!ReadU32(s, len, cp, &r.command)) return false;
  const std::size_t ep = FindValue(s, len, "ErrorCode");
  if (ep != len) {
    std::uint32_t code = 0;
    if (ReadU32(s, len, ep, &code)) {
      r.has_error_code = true;
      r.error_code = code;
    }
    // A present-but-unreadable ErrorCode leaves has_error_code false, so the
    // caller sees "no code" rather than "code 0" -- and 0 means SUCCESS here.
  }
  *out = r;
  return true;
}

}  // namespace chs_a
}  // namespace quadruped
