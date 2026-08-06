/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: slot_semantics.cc
 * Brief: Deterministic single-thread probe of LockfreeSlot, the queue detector
 *
 * Description:
 * What this establishes. 13 S9.1 requires the cross-thread carrier to be a
 * single slot whose latest value overwrites, precisely because a queue lets a
 * consumer read data that has already been superseded -- QD-5. This program is
 * the deterministic half of that check: it publishes a known sequence with no
 * consumer running, then takes once, and reports what came out. A slot answers
 * with the last value published and then reports nothing new; a FIFO queue
 * answers with the FIRST value and then has a backlog to hand over. The two are
 * distinguished by the numbers printed here, with no timing involved.
 *
 * It also pins the two properties that keep an absent sample from turning into
 * a plausible one: before anything is published, TakeFresh must return false
 * AND must leave the caller's variable untouched. The second half of that is
 * the fail-silent guard from CLAUDE.md 3.1 -- an implementation that helpfully
 * zero-fills the output hands its caller a perfectly believable "not moving,
 * nothing in range" reading that no downstream check can question.
 *
 * What this program does NOT establish. Nothing about concurrency: there is one
 * thread here. Torn reads and staleness under a real producer are
 * slot_staleness.cc. Nothing about the guard or about scheduling either.
 *
 * Output contract. One key=value line per fact, parsed by
 * tests/common/test_rtcomm.py. The assertions live in the Python side so that a
 * failure prints the expected and actual values rather than an abort with no
 * numbers, and so the thresholds sit in one reviewable place.
 */

#include "xbrain/rtcomm/lockfree_slot.h"

#include <cstdint>
#include <cstdio>

namespace {

// A payload with two fields whose bytes differ, so a half-copied value is
// visible. Trivially copyable, as the slot's static_assert requires.
struct Sample {
  int32_t seq;
  double value;
};

// The sentinel the untouched-output check uses. Any value works as long as it
// is one no publish ever produces, so that "still the sentinel" cannot be
// confused with "a published value happened to match".
const int32_t kSentinelSeq = -424242;

}  // namespace

int main() {
  hachist::xbrain::rtcomm::LockfreeSlot<Sample> slot;

  // 1. Nothing published yet. The take must fail and must not write through the
  //    pointer -- the sentinel has to survive intact.
  Sample out;
  out.seq = kSentinelSeq;
  out.value = -1.0;
  const bool empty_take = slot.TakeFresh(&out);
  std::printf("empty_take=%d\n", empty_take ? 1 : 0);
  std::printf("sentinel_intact=%d\n", out.seq == kSentinelSeq ? 1 : 0);

  // 2. Publish a run of values with no consumer in between. This is the shape
  //    that separates a slot from a queue: 13 S9.1's chs_b publishes at 200 Hz
  //    while ctrl consumes at 100 Hz, so roughly every other sample is meant to
  //    die unread.
  const int32_t kPublishes = 100;
  for (int32_t i = 1; i <= kPublishes; ++i) {
    Sample s;
    s.seq = i;
    s.value = static_cast<double>(i) * 0.5;
    slot.Publish(s);
  }

  // 3. One take. A slot yields the newest; a queue yields the oldest.
  const bool first_take = slot.TakeFresh(&out);
  std::printf("first_take=%d\n", first_take ? 1 : 0);
  std::printf("first_seq=%d\n", static_cast<int>(out.seq));
  // The payload must be internally consistent: value is a fixed function of
  // seq, so a mismatch means the two fields came from different publishes.
  std::printf("first_value_consistent=%d\n",
              out.value == static_cast<double>(out.seq) * 0.5 ? 1 : 0);

  // 4. A second take with nothing published in between. A slot reports nothing
  //    new. A queue still has ninety-nine entries queued up, every one of them
  //    older than what was already consumed -- QD-5's "stale data being
  //    consumed" in its purest form.
  Sample second;
  second.seq = kSentinelSeq;
  second.value = -1.0;
  const bool second_take = slot.TakeFresh(&second);
  std::printf("second_take=%d\n", second_take ? 1 : 0);
  std::printf("second_seq=%d\n", static_cast<int>(second.seq));

  // 5. The counters. published far above taken is the intended behaviour of
  //    this container and the proof that samples really were dropped -- without
  //    it, a test that "shows the slot drops stale data" might simply never
  //    have had any to drop.
  std::printf("publish_count=%llu\n",
              static_cast<unsigned long long>(slot.publish_count()));
  std::printf("take_count=%llu\n",
              static_cast<unsigned long long>(slot.take_count()));
  std::printf("publishes_requested=%d\n", static_cast<int>(kPublishes));

  return 0;
}
