/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: slot_staleness.cc
 * Brief: Fast producer, slow consumer, the QD-5 stale-data injection case
 *
 * Description:
 * What this establishes. The failure QD-5 names is a consumer that receives
 * data which was already superseded and cannot tell. 13 S9.1 blames the queue
 * for it: a queue absorbs the difference between a 200 Hz producer and a slower
 * consumer as backlog, and every item drawn from that backlog is a reading from
 * the past presented as the present. This program builds exactly that
 * situation -- a producer roughly ten times faster than its consumer -- and
 * measures how far behind the consumer's last value is when the dust settles.
 *
 * With the single slot, the answer is "one sample at most, because everything
 * older was overwritten". With a queue the answer is "as far behind as the
 * consumer is slow", and the number printed here is the difference. That is why
 * the criterion is stated as a gap and not as a pass/fail flag: the gap is a
 * measurement that a queue cannot fake.
 *
 * The two secondary properties checked in the same run, both of which a
 * concurrent implementation can get wrong while passing the single-threaded
 * probe in slot_semantics.cc:
 *   * no torn value. Every payload carries the sequence number in eight places
 *     plus a checksum; if the consumer ever reads a buffer the producer is
 *     writing, the copies disagree. This is what would happen with two buffers
 *     instead of three, or with a seqlock whose retry was dropped
 *   * no going backwards. A taken sequence number that is not greater than the
 *     previous one means the consumer was handed an older sample than one it
 *     already had, which is QD-5 again in a different costume
 *
 * What this does NOT establish. It cannot prove the absence of a memory-order
 * bug: on x86_64 the hardware supplies most of the ordering, so a weakened
 * release/acquire pair can pass here and fail on the aarch64 ORIN. Stating that
 * plainly is the point -- a green run of this file is evidence about buffer
 * ownership and staleness, not about the memory model.
 *
 * Output contract. key=value lines, asserted by tests/common/test_rtcomm.py.
 * The run duration comes from argv[1] in milliseconds and has no default: the
 * runner decides how long, and a program that picked its own duration would
 * quietly turn a short smoke run into the evidence for a long soak.
 */

#include "xbrain/rtcomm/lockfree_slot.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>

namespace {

// Eight repeats of the sequence number plus a checksum over them. Redundancy is
// the only way a copy can be caught mid-flight: a single field always looks
// self-consistent no matter which producer wrote it.
struct Sample {
  int32_t seq;
  int32_t repeat[8];
  int64_t checksum;
};

void FillSample(Sample* s, int32_t seq) {
  s->seq = seq;
  int64_t sum = 0;
  for (int i = 0; i < 8; ++i) {
    // Each slot gets a different function of seq so that a partially copied
    // value cannot pass by accident -- eight identical fields would all still
    // agree if a torn read happened to land between two equal values.
    s->repeat[i] = seq * (i + 1);
    sum += s->repeat[i];
  }
  s->checksum = sum;
}

bool SampleIsConsistent(const Sample& s) {
  int64_t sum = 0;
  for (int i = 0; i < 8; ++i) {
    if (s.repeat[i] != s.seq * (i + 1)) {
      return false;
    }
    sum += s.repeat[i];
  }
  return sum == s.checksum;
}

// Shared state. The producer publishes into the slot and records how far it
// got; the consumer reads until told to stop.
hachist::xbrain::rtcomm::LockfreeSlot<Sample> g_slot;
std::atomic<bool> g_producer_running(true);
std::atomic<int> g_published_total(0);

// Consumer bookkeeping, owned by the consumer thread alone.
int g_taken_total = 0;
int g_last_taken_seq = 0;
int g_torn_count = 0;
int g_backwards_count = 0;

void ConsumerLoop() {
  Sample got;
  while (g_producer_running.load(std::memory_order_acquire)) {
    if (g_slot.TakeFresh(&got)) {
      ++g_taken_total;
      if (!SampleIsConsistent(got)) {
        ++g_torn_count;
      }
      if (got.seq <= g_last_taken_seq) {
        ++g_backwards_count;
      }
      g_last_taken_seq = got.seq;
    }
    // Deliberately an order of magnitude slower than the producer. This is the
    // whole experiment: without a consumer that falls behind there is no stale
    // data for the container to mishandle.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // One last take after the producer has stopped. On a slot this returns the
  // final published value regardless of how far behind the consumer was; on a
  // queue it returns whatever is next in the backlog.
  if (g_slot.TakeFresh(&got)) {
    ++g_taken_total;
    if (!SampleIsConsistent(got)) {
      ++g_torn_count;
    }
    g_last_taken_seq = got.seq;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    // No default duration on purpose; see the file comment.
    std::fprintf(stderr, "usage: slot_staleness <milliseconds>\n");
    return 2;
  }
  const int duration_ms = std::atoi(argv[1]);
  if (duration_ms <= 0) {
    std::fprintf(stderr, "duration must be positive\n");
    return 2;
  }

  std::thread consumer(ConsumerLoop);

  // steady_clock, not system_clock: CLAUDE.md 3.4 and 11 CLK-C1 put every
  // duration and every deadline on the monotonic clock. A wall-clock step from
  // an NTP correction mid-run would end this loop early or late for a reason
  // that has nothing to do with the code under test.
  const std::chrono::steady_clock::time_point deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
  int32_t seq = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    ++seq;
    Sample s;
    FillSample(&s, seq);
    g_slot.Publish(s);
    g_published_total.store(seq, std::memory_order_relaxed);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  g_producer_running.store(false, std::memory_order_release);
  consumer.join();

  const int published = g_published_total.load(std::memory_order_relaxed);
  std::printf("published_total=%d\n", published);
  std::printf("taken_total=%d\n", g_taken_total);
  std::printf("last_taken_seq=%d\n", g_last_taken_seq);
  // The headline number: how many publishes the consumer's final value is
  // behind the producer's final value.
  std::printf("final_gap=%d\n", published - g_last_taken_seq);
  std::printf("torn_count=%d\n", g_torn_count);
  std::printf("backwards_count=%d\n", g_backwards_count);
  std::printf("slot_publish_count=%llu\n",
              static_cast<unsigned long long>(g_slot.publish_count()));
  std::printf("slot_take_count=%llu\n",
              static_cast<unsigned long long>(g_slot.take_count()));
  return 0;
}
