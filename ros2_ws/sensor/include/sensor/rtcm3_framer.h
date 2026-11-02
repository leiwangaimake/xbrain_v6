/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtcm3_framer.h
 * Brief: RTCM 3.x byte stream -> CRC-checked whole frames (11 S3.2A RTG-3)
 *
 * Description:
 * Both correction sources deliver a bare byte stream: the RPi bridge forwards the
 * base station UART 1:1 and starts mid-stream (RTB-2), the NTRIP caster sends
 * frames after its reply header. This framer finds frame boundaries in that
 * stream and hands out only frames whose CRC-24Q checks, unchanged, so the bytes
 * written to the receiver are always whole frames (RTG-6).
 *
 * Frame: 0xD3 | 6 reserved bits (0) + 10-bit length L | L payload bytes | CRC-24Q
 * over header + payload. The message number is the first 12 payload bits; for
 * station (1005/1006), MSM and legacy observation messages the reference station
 * id is the next 12 bits.
 *
 * The one rule that matters for resync: on a CRC failure only the 0xD3 byte is
 * dropped and the search restarts at the next byte. Skipping L bytes instead
 * would trust a length read from what is, by then, known to be garbage, and can
 * jump over the start of the next real frame.
 *
 * Pure: no I/O, no clock. Not thread-safe; each source owns one instance.
 */

#ifndef SENSOR__RTCM3_FRAMER_H_
#define SENSOR__RTCM3_FRAMER_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sensor {

// One CRC-checked frame, bytes exactly as received (header + payload + CRC).
struct Rtcm3Frame {
  std::string bytes;
  int msg_type = 0;      // first 12 payload bits; 0 for an empty payload
  int station_id = -1;   // reference station id when the type carries one, else -1
};

// CRC-24Q (Qualcomm), polynomial 0x1864CFB, initial value 0.
uint32_t rtcm_crc24q(const uint8_t* data, std::size_t n);

// Observation messages: MSM1-7 of GPS / GLONASS / Galileo / SBAS / QZSS / BDS
// (107x / 108x / 109x / 110x / 111x / 112x) and legacy 1001-1004 / 1009-1012.
bool rtcm_is_observation(int msg_type);

// Reference station position: 1005 / 1006.
bool rtcm_is_station(int msg_type);

class Rtcm3Framer {
 public:
  static constexpr std::size_t kMaxPayload = 1023;
  static constexpr std::size_t kMaxFrame = 3 + kMaxPayload + 3;

  // Append bytes and move every complete, CRC-valid frame into *out (appended).
  void feed(const char* data, std::size_t n, std::vector<Rtcm3Frame>* out);

  // Drop buffered bytes (a reconnect starts a new stream). Counters are kept.
  void reset() { buf_.clear(); }

  uint64_t frames_ok() const { return frames_ok_; }
  uint64_t crc_errors() const { return crc_errors_; }
  uint64_t garbage_bytes() const { return garbage_bytes_; }

 private:
  std::string buf_;
  uint64_t frames_ok_ = 0;
  uint64_t crc_errors_ = 0;
  uint64_t garbage_bytes_ = 0;
};

}  // namespace sensor

#endif  // SENSOR__RTCM3_FRAMER_H_
