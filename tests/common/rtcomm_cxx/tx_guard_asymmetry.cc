/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: tx_guard_asymmetry.cc
 * Brief: The realtime side never waits, the non-realtime side does -- measured
 *
 * Description:
 * What this establishes. 13 S9.1 TX-6 makes the guard deliberately asymmetric:
 * ctrl does one test_and_set and abandons the tick if it fails, never blocking,
 * while chs_a_tx spins until it gets in. CPP-3 explains why -- a realtime
 * thread that waits on a lock a non-realtime thread can hold is priority
 * inversion on the 100 Hz cadence. Both halves of that asymmetry are claims
 * about time, so both are measured here rather than asserted.
 *
 * How the measurement is made sharp. A contender takes the guard and holds it
 * for a caller-supplied span that is orders of magnitude longer than any real
 * frame send. The realtime attempt is then timed against that span: an
 * implementation that skips returns in well under a microsecond, one that waits
 * returns after the whole hold. There is no threshold-tuning question here,
 * because the two outcomes differ by roughly five orders of magnitude.
 *
 * The second phase inverts the roles and times the non-realtime acquire while
 * the realtime side holds. It exists because the first phase alone is passed by
 * a guard that never acquires at all: if TryAcquireRealtime simply returned
 * false forever, phase one would look perfect. Phase two requires the
 * non-realtime path to actually wait and actually get in, which that shell
 * implementation cannot do.
 *
 * What this does NOT establish. Nothing about sustained behaviour, cadence or
 * counter growth -- that is tx_guard_soak.cc. Nothing about what the caller
 * does inside the critical section either; TX-7 (one frame per critical
 * section, short writes topped up inside it) is not enforceable from here.
 *
 * A note on why the realtime attempt releases the guard if it unexpectedly
 * succeeds. Under the mutation this file is meant to catch (a realtime side
 * that spins), the attempt eventually succeeds; without the release, the next
 * phase would spin forever against a guard this thread already holds and the
 * test would hang. A hanging test is a worse signal than a failing one -- it
 * gets attributed to the machine.
 *
 * Output contract. key=value lines, asserted by tests/common/test_rtcomm.py.
 * The hold duration comes from argv[1] in milliseconds; there is no default.
 */

#include "xbrain/rtcomm/tx_guard.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {

hachist::xbrain::rtcomm::TxGuard g_guard;

// Set by the contender once it is inside the critical section, so the realtime
// attempt is timed against a guard that is definitely held. Polling this rather
// than sleeping a guessed amount removes the one race that would make this test
// flaky on a loaded machine.
std::atomic<bool> g_contender_holds(false);

// How long the realtime side may poll for the contender before giving up. This
// is a test harness safety net, not a property of the code under test: if the
// contender never announces itself something is badly wrong and the program
// should say so rather than hang.
const int kSetupTimeoutMs = 5000;

int64_t ElapsedUs(std::chrono::steady_clock::time_point start,
                  std::chrono::steady_clock::time_point end) {
  // steady_clock throughout, per CLAUDE.md 3.4 and 11 CLK-C1: this is a
  // duration measurement, and a wall clock adjusted mid-measurement produces a
  // number that is wrong in either direction with nothing to indicate it.
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start)
      .count();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: tx_guard_asymmetry <hold_milliseconds>\n");
    return 2;
  }
  const int hold_ms = std::atoi(argv[1]);
  if (hold_ms <= 0) {
    std::fprintf(stderr, "hold must be positive\n");
    return 2;
  }
  std::printf("hold_ms=%d\n", hold_ms);

  // ---------------------------------------------------------------------
  // Phase 1. Non-realtime side holds; the realtime side must skip at once.
  // ---------------------------------------------------------------------
  std::thread contender([hold_ms]() {
    g_guard.AcquireNonRealtime();
    g_contender_holds.store(true, std::memory_order_release);
    std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
    g_guard.Release();
  });

  const std::chrono::steady_clock::time_point setup_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(kSetupTimeoutMs);
  while (!g_contender_holds.load(std::memory_order_acquire)) {
    if (std::chrono::steady_clock::now() > setup_deadline) {
      std::fprintf(stderr, "contender never took the guard\n");
      return 3;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }

  const uint64_t skips_before = g_guard.tx_skip_count();
  const std::chrono::steady_clock::time_point t0 =
      std::chrono::steady_clock::now();
  const bool rt_got_it = g_guard.TryAcquireRealtime();
  const std::chrono::steady_clock::time_point t1 =
      std::chrono::steady_clock::now();
  const uint64_t skips_after = g_guard.tx_skip_count();

  std::printf("phase1_rt_acquired=%d\n", rt_got_it ? 1 : 0);
  std::printf("phase1_rt_try_us=%lld\n",
              static_cast<long long>(ElapsedUs(t0, t1)));
  std::printf("phase1_skip_delta=%llu\n",
              static_cast<unsigned long long>(skips_after - skips_before));

  // See the file comment: this keeps a spinning mutant from deadlocking the
  // next phase instead of failing the assertions above.
  if (rt_got_it) {
    g_guard.Release();
  }
  contender.join();

  // ---------------------------------------------------------------------
  // Phase 2. Realtime side holds; the non-realtime side must wait it out.
  // ---------------------------------------------------------------------
  const bool rt_owns = g_guard.TryAcquireRealtime();
  std::printf("phase2_rt_acquired=%d\n", rt_owns ? 1 : 0);
  if (!rt_owns) {
    std::fprintf(stderr, "guard was still held after phase 1\n");
    return 3;
  }

  // Started only after the realtime side owns the guard, otherwise the waiter
  // could win the race and measure a wait of zero.
  std::atomic<long long> wait_us(-1);
  std::thread waiter([&wait_us]() {
    const std::chrono::steady_clock::time_point w0 =
        std::chrono::steady_clock::now();
    g_guard.AcquireNonRealtime();
    const std::chrono::steady_clock::time_point w1 =
        std::chrono::steady_clock::now();
    wait_us.store(ElapsedUs(w0, w1), std::memory_order_release);
    g_guard.Release();
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(hold_ms));
  g_guard.Release();
  waiter.join();

  std::printf("phase2_nonrt_wait_us=%lld\n",
              static_cast<long long>(wait_us.load(std::memory_order_acquire)));
  std::printf("total_rt_acquires=%llu\n",
              static_cast<unsigned long long>(g_guard.rt_acquire_count()));
  std::printf("total_tx_skips=%llu\n",
              static_cast<unsigned long long>(g_guard.tx_skip_count()));
  return 0;
}
