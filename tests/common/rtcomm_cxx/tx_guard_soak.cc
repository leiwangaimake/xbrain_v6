/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: tx_guard_soak.cc
 * Brief: 100 Hz realtime side against hammering non-realtime contenders
 *
 * Description:
 * What this establishes. The stress case for CPP-CXX-3: a realtime side ticking
 * at 100 Hz -- ctrl's rate in 13 S9.1 -- against non-realtime threads competing
 * for the same guard, run for a caller-supplied span. Over that span three
 * things have to hold, and each is reported as a number rather than a verdict:
 *   * the realtime side never waits. Its per-attempt latency is measured on
 *     every single tick and the maximum is reported
 *   * the accounting is exact. Ticks equal acquisitions plus skips; a guard
 *     that forgot to count a skip, or counted one twice, shows up here and
 *     nowhere else
 *   * tx_skip_count stays a small fraction and does not run away. It is
 *     reported for the first and the second half of the run separately, because
 *     the failure that matters -- a critical section leaked by an early return,
 *     so every later tick skips -- is invisible in a total and obvious in a
 *     split. CPP-4's remark that a non-zero count is meant to be seen is the
 *     whole reason the counter exists
 *
 * Which design sections. 13 S9.1 TX-6 (the asymmetry), 13 S9.3 CPP-4 (the
 * counter and what it is for), 13 S9.1's thread table (ctrl at 100 Hz), 13 S9.2
 * RTC-7 via rt_thread.h for the scheduling attempt.
 *
 * On SCHED_FIFO. This program asks for it and reports whether it was granted.
 * It does not require it: an unprivileged user with RLIMIT_RTPRIO at zero gets
 * EPERM, and the properties above are about the guard, not about scheduling.
 * What it must never do is pretend -- so the errno is printed and the runner
 * says out loud which part of the run was unverified. A silent fallback to
 * SCHED_OTHER would let a soak "prove" realtime behaviour on a box where
 * realtime was never granted.
 *
 * What this does NOT establish. It is not a cadence measurement of the real
 * ctrl loop: there is no chassis, no socket and no framing here, and on a
 * shared build machine without FIFO the tick count reflects the machine's load
 * as much as the code's. The sharp latency discrimination lives in
 * tx_guard_asymmetry.cc, which forces the contended case deterministically.
 *
 * Output contract. key=value lines, asserted by tests/common/test_rtcomm.py.
 * Duration comes from argv[1] in seconds and has no default -- the runner
 * chooses, so that a short smoke run can never be mistaken for the long soak.
 */

#include "xbrain/rtcomm/rt_thread.h"
#include "xbrain/rtcomm/tx_guard.h"

#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {

hachist::xbrain::rtcomm::TxGuard g_guard;

// Set once the realtime loop is finished so the contenders stop too.
std::atomic<bool> g_running(true);

// Contender bookkeeping. Relaxed throughout: these are counters nobody
// branches on, and making them sequentially consistent would add a barrier to
// the loop that is meant to be hammering the guard.
std::atomic<uint64_t> g_nonrt_iters(0);

// Realtime loop results, written by the realtime thread and read after join.
uint64_t g_ticks = 0;
uint64_t g_acquires = 0;
int64_t g_max_try_ns = 0;
uint64_t g_skips_at_half = 0;

// One control period. 13 S9.1 puts ctrl at 10 ms; this mirrors it so the
// contention pattern matches the real one rather than an arbitrary rate.
const long kTickNs = 10L * 1000L * 1000L;

// How long the holder stays inside the critical section. Chosen to stand in for
// one whole-frame send (TX-7 requires the whole frame inside one critical
// section) without pretending to be a measurement of the real thing.
const int64_t kRealtimeHoldUs = 20;
const int64_t kNonRealtimeHoldUs = 10;

void BusyMicros(int64_t micros) {
  // A spin, not a sleep: a sleep would hand the CPU back and the guard would be
  // held across a scheduling gap, which is exactly the situation TX-7 forbids
  // in the real sender and would flatter the contention numbers here.
  const std::chrono::steady_clock::time_point end =
      std::chrono::steady_clock::now() + std::chrono::microseconds(micros);
  while (std::chrono::steady_clock::now() < end) {
  }
}

void AddNs(struct timespec* ts, long ns) {
  ts->tv_nsec += ns;
  while (ts->tv_nsec >= 1000L * 1000L * 1000L) {
    ts->tv_nsec -= 1000L * 1000L * 1000L;
    ts->tv_sec += 1;
  }
}

// Realtime side. Absolute-deadline sleeping on CLOCK_MONOTONIC: a relative
// sleep accumulates the loop's own execution time as drift, so the tick count
// would fall short of the expected one for a reason that has nothing to do with
// the guard. CLOCK_MONOTONIC and not CLOCK_REALTIME, per CLAUDE.md 3.4 and
// 11 CLK-C1 -- a wall-clock step would move every remaining deadline at once.
void* RealtimeLoop(void* arg) {
  const int duration_s = *static_cast<const int*>(arg);

  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  const std::chrono::steady_clock::time_point started =
      std::chrono::steady_clock::now();
  const std::chrono::steady_clock::time_point half =
      started + std::chrono::seconds(duration_s) / 2;
  const std::chrono::steady_clock::time_point finish =
      started + std::chrono::seconds(duration_s);
  bool half_recorded = false;

  while (std::chrono::steady_clock::now() < finish) {
    AddNs(&next, kTickNs);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

    ++g_ticks;
    const std::chrono::steady_clock::time_point t0 =
        std::chrono::steady_clock::now();
    {
      // The RAII form, because it is the shape callers are meant to copy: the
      // release cannot be skipped by an early return out of the send.
      hachist::xbrain::rtcomm::RealtimeTxScope scope(&g_guard);
      const std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
      const int64_t try_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
      if (try_ns > g_max_try_ns) {
        g_max_try_ns = try_ns;
      }
      if (scope.acquired()) {
        ++g_acquires;
        BusyMicros(kRealtimeHoldUs);
      }
      // No log, no allocation and no branch on failure beyond the counter:
      // QD-7 forbids blocking work here, and a skipped tick is normal.
    }

    if (!half_recorded && std::chrono::steady_clock::now() >= half) {
      g_skips_at_half = g_guard.tx_skip_count();
      half_recorded = true;
    }
  }
  if (!half_recorded) {
    g_skips_at_half = g_guard.tx_skip_count();
  }
  return nullptr;
}

void ContenderLoop() {
  while (g_running.load(std::memory_order_acquire)) {
    {
      hachist::xbrain::rtcomm::NonRealtimeTxScope scope(&g_guard);
      BusyMicros(kNonRealtimeHoldUs);
    }
    g_nonrt_iters.fetch_add(1, std::memory_order_relaxed);
    // Roughly 5 kHz per contender, fifty times the realtime rate. High enough
    // to be the "high frequency contention" the criterion asks for, spaced
    // enough that two contenders cannot own the guard continuously and starve
    // the realtime side by construction -- which would make the skip-ratio
    // assertion a statement about this test rather than about the guard.
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: tx_guard_soak <seconds>\n");
    return 2;
  }
  int duration_s = std::atoi(argv[1]);
  if (duration_s <= 0) {
    std::fprintf(stderr, "duration must be positive\n");
    return 2;
  }
  std::printf("duration_s=%d\n", duration_s);

  std::thread contender_a(ContenderLoop);
  std::thread contender_b(ContenderLoop);

  // The priority is queried from the kernel rather than written down. Any legal
  // value exercises the wrapper equally, and naming a number here would put a
  // priority into the repository that D-42 has not decided -- see rt_thread.h.
  const int priority = hachist::xbrain::rtcomm::FifoPriorityMin();
  pthread_t rt_thread;
  const int fifo_rc = hachist::xbrain::rtcomm::StartFifoThread(
      &rt_thread, priority, RealtimeLoop, &duration_s);
  std::printf("fifo_rc=%d\n", fifo_rc);

  if (fifo_rc != 0) {
    // Reported, never hidden. The run continues under ordinary scheduling
    // because the guard's properties do not depend on the policy, but the
    // runner has to be able to say which claim went unverified.
    std::printf("fifo_applied=0\n");
    const int rc = pthread_create(&rt_thread, nullptr, RealtimeLoop, &duration_s);
    if (rc != 0) {
      std::fprintf(stderr, "could not start the realtime loop at all\n");
      return 3;
    }
  } else {
    std::printf("fifo_applied=1\n");
  }

  pthread_join(rt_thread, nullptr);
  g_running.store(false, std::memory_order_release);
  contender_a.join();
  contender_b.join();

  const uint64_t skips = g_guard.tx_skip_count();
  std::printf("ticks=%llu\n", static_cast<unsigned long long>(g_ticks));
  std::printf("rt_acquires=%llu\n", static_cast<unsigned long long>(g_acquires));
  std::printf("guard_rt_acquires=%llu\n",
              static_cast<unsigned long long>(g_guard.rt_acquire_count()));
  std::printf("tx_skip_count=%llu\n", static_cast<unsigned long long>(skips));
  std::printf("skips_first_half=%llu\n",
              static_cast<unsigned long long>(g_skips_at_half));
  std::printf("skips_second_half=%llu\n",
              static_cast<unsigned long long>(skips - g_skips_at_half));
  std::printf("max_try_us=%lld\n",
              static_cast<long long>(g_max_try_ns / 1000));
  std::printf("max_try_ns=%lld\n", static_cast<long long>(g_max_try_ns));
  std::printf("nonrt_iters=%llu\n",
              static_cast<unsigned long long>(
                  g_nonrt_iters.load(std::memory_order_relaxed)));
  std::printf("expected_ticks=%d\n", duration_s * 100);
  return 0;
}
