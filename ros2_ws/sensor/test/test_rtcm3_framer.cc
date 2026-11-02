/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rtcm3_framer.cc
 * Brief: Offline unit test for the RTCM 3.x framer (11 S3.2A RTG-3 / RTG-6)
 *
 * Description:
 * The CRC is pinned to the published CRC-24Q check value first, so the frames
 * this test builds with rtcm_crc24q() cannot agree with a wrong CRC by
 * construction. The load-bearing case is the resync after a bad CRC: a fake
 * header whose length spans the start of the next real frame must cost one
 * byte, not the real frame.
 */

#include "sensor/rtcm3_framer.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using sensor::Rtcm3Frame;
using sensor::Rtcm3Framer;
using sensor::rtcm_crc24q;
using sensor::rtcm_is_observation;
using sensor::rtcm_is_station;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

// Build a frame: 12-bit type, 12-bit station id, then `extra` filler bytes.
static std::string make_frame(int type, int station, std::size_t extra) {
  std::string payload(3 + extra, '\0');
  payload[0] = static_cast<char>((type >> 4) & 0xFF);
  payload[1] = static_cast<char>(((type & 0x0F) << 4) | ((station >> 8) & 0x0F));
  payload[2] = static_cast<char>(station & 0xFF);
  for (std::size_t i = 0; i < extra; ++i) payload[3 + i] = static_cast<char>(0x40 + i);
  std::string f;
  f.push_back(static_cast<char>(0xD3));
  f.push_back(static_cast<char>((payload.size() >> 8) & 0x03));
  f.push_back(static_cast<char>(payload.size() & 0xFF));
  f += payload;
  const uint32_t crc =
      rtcm_crc24q(reinterpret_cast<const uint8_t*>(f.data()), f.size());
  f.push_back(static_cast<char>((crc >> 16) & 0xFF));
  f.push_back(static_cast<char>((crc >> 8) & 0xFF));
  f.push_back(static_cast<char>(crc & 0xFF));
  return f;
}

static std::vector<Rtcm3Frame> feed_all(Rtcm3Framer& fr, const std::string& s) {
  std::vector<Rtcm3Frame> out;
  fr.feed(s.data(), s.size(), &out);
  return out;
}

static void test_crc_check_value() {
  const char* v = "123456789";
  CHECK(rtcm_crc24q(reinterpret_cast<const uint8_t*>(v), std::strlen(v)) == 0xCDE703u);
}

static void test_message_classes() {
  CHECK(rtcm_is_station(1005) && rtcm_is_station(1006));
  CHECK(!rtcm_is_station(1007));
  CHECK(rtcm_is_observation(1074) && rtcm_is_observation(1077));
  CHECK(rtcm_is_observation(1124) && rtcm_is_observation(1094));
  CHECK(rtcm_is_observation(1004) && rtcm_is_observation(1012));
  CHECK(!rtcm_is_observation(1070) && !rtcm_is_observation(1078));
  CHECK(!rtcm_is_observation(1033) && !rtcm_is_observation(1005));
  CHECK(!rtcm_is_observation(1230));
}

static void test_single_frame_whole() {
  Rtcm3Framer fr;
  const std::string f = make_frame(1005, 1234, 16);
  const auto out = feed_all(fr, f);
  CHECK(out.size() == 1);
  if (out.size() == 1) {
    CHECK(out[0].bytes == f);                  // unchanged, header + CRC included
    CHECK(out[0].msg_type == 1005);
    CHECK(out[0].station_id == 1234);
  }
  CHECK(fr.frames_ok() == 1);
  CHECK(fr.garbage_bytes() == 0);
}

static void test_byte_by_byte() {
  Rtcm3Framer fr;
  const std::string s = make_frame(1074, 7, 40) + make_frame(1005, 7, 16);
  std::vector<Rtcm3Frame> out;
  for (char c : s) fr.feed(&c, 1, &out);
  CHECK(out.size() == 2);
  if (out.size() == 2) {
    CHECK(out[0].msg_type == 1074 && out[0].station_id == 7);
    CHECK(out[1].msg_type == 1005);
  }
}

static void test_garbage_around_frames() {
  Rtcm3Framer fr;
  const std::string junk = "hello\r\n$GNGGA,junk\r\n";
  const std::string s = junk + make_frame(1124, 3, 30) + "xyz" + make_frame(1006, 3, 18);
  const auto out = feed_all(fr, s);
  CHECK(out.size() == 2);
  CHECK(fr.garbage_bytes() == junk.size() + 3);
}

static void test_crc_error_rejected() {
  Rtcm3Framer fr;
  std::string bad = make_frame(1005, 1, 16);
  bad[10] ^= 0x01;                             // one bit in the payload
  const auto out = feed_all(fr, bad + make_frame(1005, 1, 16));
  CHECK(out.size() == 1);                       // the corrupted one is not handed out
  CHECK(fr.crc_errors() >= 1);
}

// A fake header whose length runs past the start of the next real frame. Skipping
// by that length would swallow the real frame; dropping one byte finds it.
static void test_resync_drops_one_byte_only() {
  Rtcm3Framer fr;
  const std::string real = make_frame(1074, 9, 20);
  std::string s;
  s.push_back(static_cast<char>(0xD3));
  s.push_back(static_cast<char>(0x00));
  s.push_back(static_cast<char>(0x14));         // claims 20 payload bytes
  s += "AB";                                    // two bytes, then the real frame
  s += real;
  s += std::string(40, 'Z');                    // enough bytes to complete the fake length
  const auto out = feed_all(fr, s);
  CHECK(out.size() == 1);
  if (out.size() == 1) CHECK(out[0].bytes == real);
  CHECK(fr.crc_errors() == 1);
}

static void test_reserved_bits_not_a_frame() {
  Rtcm3Framer fr;
  std::string s;
  s.push_back(static_cast<char>(0xD3));
  s.push_back(static_cast<char>(0x40));         // reserved bit set
  s += make_frame(1005, 2, 16);
  const auto out = feed_all(fr, s);
  CHECK(out.size() == 1);
  CHECK(fr.crc_errors() == 0);                  // rejected before any CRC was tried
}

static void test_no_station_for_other_types() {
  Rtcm3Framer fr;
  const auto out = feed_all(fr, make_frame(1033, 55, 20));
  CHECK(out.size() == 1);
  if (out.size() == 1) {
    CHECK(out[0].msg_type == 1033);
    CHECK(out[0].station_id == -1);
  }
}

static void test_partial_then_reset() {
  Rtcm3Framer fr;
  const std::string f = make_frame(1005, 4, 16);
  std::vector<Rtcm3Frame> out;
  fr.feed(f.data(), 10, &out);                  // half a frame
  CHECK(out.empty());
  fr.reset();                                   // reconnect: a new stream
  fr.feed(f.data() + 10, f.size() - 10, &out);  // the tail alone is not a frame
  CHECK(out.empty());
  fr.feed(f.data(), f.size(), &out);
  CHECK(out.size() == 1);
}

// A fake maximum-length header holds the framer until 1029 bytes have arrived;
// a real frame inside that span must still come out once the fake one fails.
static void test_pending_max_length_header() {
  Rtcm3Framer fr;
  std::vector<Rtcm3Frame> out;
  std::string s;
  s.push_back(static_cast<char>(0xD3));
  s.push_back(static_cast<char>(0x03));
  s.push_back(static_cast<char>(0xFF));         // 1023-byte payload claimed
  fr.feed(s.data(), s.size(), &out);
  const std::string f = make_frame(1005, 4, 16);
  fr.feed(f.data(), f.size(), &out);
  CHECK(out.empty());                           // still waiting for the fake frame
  const std::string filler(1100, 'q');
  fr.feed(filler.data(), filler.size(), &out);
  CHECK(out.size() == 1);
  if (out.size() == 1) CHECK(out[0].bytes == f);
}

int main() {
  test_crc_check_value();
  test_message_classes();
  test_single_frame_whole();
  test_byte_by_byte();
  test_garbage_around_frames();
  test_crc_error_rejected();
  test_resync_drops_one_byte_only();
  test_reserved_bits_not_a_frame();
  test_no_station_for_other_types();
  test_partial_then_reset();
  test_pending_max_length_header();
  if (g_failures == 0) {
    std::printf("ALL RTCM3 FRAMER TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RTCM3 FRAMER TEST(S) FAILED\n", g_failures);
  return 1;
}
