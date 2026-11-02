/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtcm3_framer.cc
 * Brief: RTCM 3.x framer + CRC-24Q (see rtcm3_framer.h)
 *
 * Description:
 * The scan loop works on an offset into buf_ and erases the consumed prefix once
 * per feed(), so a burst of frames costs one erase, not one per frame. Garbage
 * is counted byte by byte as it is skipped. The buffer needs no separate size
 * cap: the loop only stops at a preamble still waiting for its frame, and a
 * frame is at most kMaxFrame bytes, so what is kept is always under kMaxFrame.
 */

#include "sensor/rtcm3_framer.h"

namespace sensor {

namespace {
constexpr uint8_t kPreamble = 0xD3;

// Read `bits` bits starting at bit `pos` of the payload (MSB first).
uint32_t get_bits(const uint8_t* p, std::size_t pos, std::size_t bits) {
  uint32_t v = 0;
  for (std::size_t i = 0; i < bits; ++i) {
    const std::size_t b = pos + i;
    v = (v << 1) | ((p[b / 8] >> (7 - (b % 8))) & 1u);
  }
  return v;
}

bool type_carries_station(int t) {
  return rtcm_is_observation(t) || rtcm_is_station(t);
}
}  // namespace

uint32_t rtcm_crc24q(const uint8_t* data, std::size_t n) {
  uint32_t crc = 0;
  for (std::size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (int k = 0; k < 8; ++k) {
      crc <<= 1;
      if (crc & 0x1000000u) crc ^= 0x1864CFBu;
    }
  }
  return crc & 0xFFFFFFu;
}

bool rtcm_is_observation(int t) {
  if (t >= 1001 && t <= 1004) return true;   // legacy GPS
  if (t >= 1009 && t <= 1012) return true;   // legacy GLONASS
  // MSM1..MSM7: 1071-1077 GPS, 1081-1087 GLO, 1091-1097 GAL, 1101-1107 SBAS,
  // 1111-1117 QZSS, 1121-1127 BDS.
  if (t >= 1071 && t <= 1127) {
    const int unit = t % 10;
    return unit >= 1 && unit <= 7;
  }
  return false;
}

bool rtcm_is_station(int t) { return t == 1005 || t == 1006; }

void Rtcm3Framer::feed(const char* data, std::size_t n, std::vector<Rtcm3Frame>* out) {
  buf_.append(data, n);
  const uint8_t* b = reinterpret_cast<const uint8_t*>(buf_.data());
  const std::size_t size = buf_.size();
  std::size_t i = 0;
  while (i < size) {
    if (b[i] != kPreamble) {
      ++garbage_bytes_;
      ++i;
      continue;
    }
    if (size - i < 3) break;                         // need the length header
    if ((b[i + 1] & 0xFC) != 0) {                    // reserved bits set: not a frame
      ++garbage_bytes_;
      ++i;
      continue;
    }
    const std::size_t len = (static_cast<std::size_t>(b[i + 1] & 0x03) << 8) | b[i + 2];
    const std::size_t total = 3 + len + 3;
    if (size - i < total) break;                     // wait for the rest
    const uint32_t want = (static_cast<uint32_t>(b[i + 3 + len]) << 16) |
                          (static_cast<uint32_t>(b[i + 3 + len + 1]) << 8) |
                          static_cast<uint32_t>(b[i + 3 + len + 2]);
    if (rtcm_crc24q(b + i, 3 + len) != want) {
      // Drop ONLY the preamble byte: the length came from bytes now known to be
      // bad, so jumping by it could skip the start of the next real frame.
      ++crc_errors_;
      ++garbage_bytes_;
      ++i;
      continue;
    }
    Rtcm3Frame f;
    f.bytes.assign(buf_, i, total);
    const uint8_t* payload = b + i + 3;
    if (len >= 2) f.msg_type = static_cast<int>(get_bits(payload, 0, 12));
    if (len >= 3 && type_carries_station(f.msg_type)) {
      f.station_id = static_cast<int>(get_bits(payload, 12, 12));
    }
    out->push_back(std::move(f));
    ++frames_ok_;
    i += total;
  }
  buf_.erase(0, i);
}

}  // namespace sensor
