/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: test_rtcm_sink.cc
 * Brief: Offline unit test for the main-thread RTCM write queue (RTG-6 / RTG-4)
 *
 * Description:
 * A fake fd records what reaches the "receiver" and can be told to take only so
 * many bytes per call, to answer EAGAIN, or to fail. The load-bearing property:
 * whatever the fd does, the recorded byte stream is a concatenation of whole
 * frames -- a half-written frame is finished before anything else goes out,
 * and nothing (age, overflow, source switch) cuts it.
 */

#include "sensor/rtcm_sink.h"

#include <cerrno>
#include <cstdio>
#include <string>

using sensor::DrainResult;
using sensor::RtcmSink;
using sensor::SinkConfig;
using sensor::TaggedFrame;

static int g_failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
      ++g_failures;                                                \
    }                                                              \
  } while (0)

struct FakeFd {
  std::string wire;
  long budget = -1;        // bytes per call; -1 = unlimited
  int fail_errno = 0;      // non-zero: every call fails with this errno
  ssize_t write(const char* d, std::size_t n) {
    if (fail_errno != 0) {
      errno = fail_errno;
      return -1;
    }
    std::size_t k = n;
    if (budget >= 0 && static_cast<std::size_t>(budget) < k) k = static_cast<std::size_t>(budget);
    if (k == 0) {
      errno = EAGAIN;
      return -1;
    }
    wire.append(d, k);
    return static_cast<ssize_t>(k);
  }
};

static TaggedFrame tf(const std::string& b, int source, double t) {
  TaggedFrame f;
  f.bytes = b;
  f.source = source;
  f.t_mono = t;
  return f;
}

static SinkConfig cfg() { return SinkConfig{64, 2.0}; }

static void test_whole_frames_in_order() {
  RtcmSink s(cfg());
  FakeFd fd;
  s.push(tf("AAAA", 1, 0.0));
  s.push(tf("BBBBBB", 1, 0.0));
  CHECK(s.drain([&](const char* d, std::size_t n) { return fd.write(d, n); }, 0.1) ==
        DrainResult::kOk);
  CHECK(fd.wire == "AAAABBBBBB");
  CHECK(s.framesWritten() == 2);
  CHECK(s.queuedBytes() == 0);
}

static void test_partial_write_finishes_first() {
  RtcmSink s(cfg());
  FakeFd fd;
  auto w = [&](const char* d, std::size_t n) { return fd.write(d, n); };
  fd.budget = 3;
  s.push(tf("AAAAAAAA", 1, 0.0));
  s.drain(w, 0.0);                             // 3 of 8 out (budget per call)
  s.push(tf("BBBB", 1, 0.0));
  fd.budget = 0;                               // fd full now: EAGAIN
  CHECK(s.drain(w, 0.05) == DrainResult::kOk);
  fd.budget = -1;
  s.drain(w, 0.1);
  CHECK(fd.wire == "AAAAAAAABBBB");
}

static void test_stale_dropped_but_not_the_started_one() {
  RtcmSink s(cfg());
  FakeFd fd;
  auto w = [&](const char* d, std::size_t n) { return fd.write(d, n); };
  fd.budget = 2;
  s.push(tf("AAAAAA", 1, 0.0));
  s.push(tf("BBBB", 1, 0.0));
  s.drain(w, 0.0);                             // A started
  fd.budget = -1;
  s.drain(w, 5.0);                             // both "stale" by now
  CHECK(fd.wire == "AAAAAA");                  // A finished, B dropped
  CHECK(s.droppedStale() == 1);
}

static void test_overflow_drops_oldest_whole_frame() {
  RtcmSink s(SinkConfig{10, 2.0});
  s.push(tf("AAAA", 1, 0.0));
  s.push(tf("BBBB", 1, 0.0));
  s.push(tf("CCCC", 1, 0.0));                  // 12 > 10: A goes
  CHECK(s.droppedOverflow() == 1);
  CHECK(s.queuedBytes() == 8);
  FakeFd fd;
  s.drain([&](const char* d, std::size_t n) { return fd.write(d, n); }, 0.0);
  CHECK(fd.wire == "BBBBCCCC");
}

static void test_overflow_keeps_started_frame() {
  RtcmSink s(SinkConfig{10, 2.0});
  FakeFd fd;
  auto w = [&](const char* d, std::size_t n) { return fd.write(d, n); };
  fd.budget = 1;
  s.push(tf("AAAA", 1, 0.0));
  s.drain(w, 0.0);                             // A started
  s.push(tf("BBBB", 1, 0.0));
  s.push(tf("CCCC", 1, 0.0));                  // over: B goes, not A
  fd.budget = -1;
  s.drain(w, 0.0);
  CHECK(fd.wire == "AAAACCCC");
}

static void test_source_switch_flushes() {
  RtcmSink s(cfg());
  FakeFd fd;
  auto w = [&](const char* d, std::size_t n) { return fd.write(d, n); };
  fd.budget = 2;
  s.push(tf("AAAA", 1, 0.0));
  s.drain(w, 0.0);                             // A (lan) started
  s.push(tf("BBBB", 1, 0.0));                  // lan, queued
  s.push(tf("XXXX", 2, 0.0));                  // ntrip: B is flushed, A finishes
  CHECK(s.droppedSwitch() == 1);
  fd.budget = -1;
  s.drain(w, 0.0);
  CHECK(fd.wire == "AAAAXXXX");
}

static void test_error_and_clear() {
  RtcmSink s(cfg());
  FakeFd fd;
  auto w = [&](const char* d, std::size_t n) { return fd.write(d, n); };
  s.push(tf("AAAA", 1, 0.0));
  fd.fail_errno = EIO;
  CHECK(s.drain(w, 0.0) == DrainResult::kError);
  s.clear();
  CHECK(s.queuedBytes() == 0);
  fd.fail_errno = 0;
  s.push(tf("BBBB", 1, 0.0));
  s.drain(w, 0.0);
  CHECK(fd.wire == "BBBB");                    // nothing of A survives the reopen
}

int main() {
  test_whole_frames_in_order();
  test_partial_write_finishes_first();
  test_stale_dropped_but_not_the_started_one();
  test_overflow_drops_oldest_whole_frame();
  test_overflow_keeps_started_frame();
  test_source_switch_flushes();
  test_error_and_clear();
  if (g_failures == 0) {
    std::printf("ALL RTCM SINK TESTS PASSED\n");
    return 0;
  }
  std::printf("%d RTCM SINK TEST(S) FAILED\n", g_failures);
  return 1;
}
