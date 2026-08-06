/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: tx_guard.h
 * Brief: Asymmetric transmit guard, atomic_flag, realtime side never waits
 *
 * Description:
 * What problem this solves. 13 S9.1 TX-4 gives the outbound socket to a single
 * function, and three threads call it: ctrl (axis commands plus the heartbeat
 * it carries, SCHED_FIFO), rt_safety (the zero-velocity frame on estop) and
 * chs_a_tx (aperiodic commands, ordinary priority). Two threads writing a TCP
 * socket without synchronisation interleave the bytes of two APDU frames into
 * one illegal frame; 13 S9.1 records the observed symptom as random 0xE001 /
 * 0xE002 replies that S7.5 then classifies as "our bug, do not retry", and it
 * is extremely hard to locate. The obvious fix, a mutex, is banned: CPP-3
 * forbids the realtime thread from blocking on a lock a non-realtime thread can
 * hold, because that is priority inversion on the 100 Hz tick.
 *
 * Which design section this implements, verbatim. 13 S9.1 TX-6 requires the
 * synchronisation to be a tx_guard built on std::atomic_flag and not a
 * std::mutex; the realtime side does test_and_set and skips the tick on
 * failure, never blocking, while the non-realtime side spins with an upper
 * bound of one whole-frame send() by ctrl. 13 S9.3 CPP-4 pins the
 * implementation further: ATOMIC_FLAG_INIT, test_and_set with
 * memory_order_acquire on the realtime side, tx_skip_count incremented on
 * failure, spin plus a pause instruction on the non-realtime side, and
 * std::mutex::try_lock explicitly forbidden as a substitute because glibc's
 * pthread_mutex can enter the kernel through futex on the contended path and
 * wreck the 100 Hz cadence.
 *
 * What this class does NOT do, so the logic lands in the right place:
 *   * it does not own or even know about the socket. TX-4 and CA-4 put that in
 *     chs_a_send(); this is only the mutual exclusion around it
 *   * it does not send, retry, frame or count bytes. TX-7 ("one frame, one
 *     critical section", short writes topped up INSIDE the same critical
 *     section per FR-4) is the caller's obligation and cannot be enforced from
 *     here -- the scopes below exist to make the correct shape the easy one
 *   * it does not publish tx_skip_count anywhere. CPP-4 wants that counter in
 *     hello_ack.runtime and in the periodic log; both of those are quadruped's
 *     data paths, and a logging call in here would be a blocking write on the
 *     realtime path (QD-7)
 *   * it does not decide what "the realtime side" is. The caller picks the
 *     method that matches its thread, and picking the wrong one is the failure
 *     mode named below
 *
 * The three ways to get this wrong that all look reasonable:
 *
 * 1. Calling AcquireNonRealtime from the ctrl thread "because it is only a few
 *    microseconds". That is a realtime thread waiting on a lock a non-realtime
 *    thread holds, which is the exact wording of what CPP-3 forbids; the
 *    latency does not show up in testing because the non-realtime holder is
 *    usually fast, and it shows up on the robot when the machine is loaded.
 *
 * 2. Retrying TryAcquireRealtime in a loop until it succeeds. Same defect,
 *    dressed as a retry policy. TX-6 says failure means the tick is skipped,
 *    and skipping one 10 ms axis command is harmless -- missing the tick
 *    deadline is not.
 *
 * 3. Releasing on the success path only. A caller that returns early on a write
 *    error and forgets Release leaves the flag set forever; every later ctrl
 *    tick then skips and tx_skip_count climbs without bound while the robot
 *    quietly stops receiving axis commands. Use the scope classes, and watch
 *    the counter -- CPP-4's note that a value above zero should be visible is
 *    what turns this from a silent stall into something an operator can see.
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_RTCOMM_TX_GUARD_H_
#define HACHIST_XBRAIN_V6_COMMON_RTCOMM_TX_GUARD_H_

#include <atomic>
#include <cstdint>

namespace hachist {
namespace xbrain {
namespace rtcomm {

// One iteration of a contended spin. The instruction tells the core that this
// is a spin loop: on x86 it drains the pipeline and cuts the memory-order
// mis-speculation penalty on exit, on aarch64 it hints the same to the
// scheduler. CPP-4 names both spellings, which is why both appear here.
//
// This is a CPU architecture test, not the distribution test PB-5 forbids. The
// two differ in kind: the humble-versus-jazzy question is an undecided design
// point (D-45) whose answer would rewrite this file, whereas x86_64 (the
// development box) and aarch64 (the ORIN) are both real targets today and the
// intrinsic simply does not exist on the other one. The final branch compiles
// to nothing, which costs correctness nothing -- the spin is still correct, it
// merely burns more power.
inline void CpuRelax() noexcept {
#if defined(__x86_64__) || defined(__i386__)
  __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
  __asm__ __volatile__("yield" ::: "memory");
#else
  // No hint available. Deliberately empty rather than a std::this_thread::yield
  // call: yield is a syscall, and a syscall inside the non-realtime spin would
  // make the wait unbounded instead of bounded by one frame send (TX-6).
#endif
}

class TxGuard {
 public:
  // ATOMIC_FLAG_INIT is CPP-4's wording. In C++17 an atomic_flag that has not
  // been initialised with it is in an unspecified state until clear() is
  // called, so this is not decoration -- constructing without it and relying on
  // zero-initialised storage is a portability bug that happens to work.
  TxGuard() noexcept
      : flag_(ATOMIC_FLAG_INIT), tx_skip_count_(0), rt_acquire_count_(0) {}

  // A guard is tied to one socket and one call site. Copying it would produce a
  // second flag guarding the same socket, which is worse than no guard at all
  // because it looks synchronised.
  TxGuard(const TxGuard&) = delete;
  TxGuard& operator=(const TxGuard&) = delete;

  // Realtime side. Exactly one attempt, ever. Returns false when the guard is
  // held, and the caller must then skip this tick -- not retry, not wait.
  //
  // memory_order_acquire is CPP-4's wording and it is load-acquire semantics on
  // the successful path: everything the previous holder did before its Release
  // (which is a release store) is visible to this thread before it starts
  // writing the socket.
  bool TryAcquireRealtime() noexcept {
    if (flag_.test_and_set(std::memory_order_acquire)) {
      // Counting the skip is not optional bookkeeping. Without it a guard held
      // by a stuck non-realtime thread stops every axis command and the only
      // externally visible symptom is a robot that does not move -- with it,
      // the count in hello_ack.runtime says why. Relaxed because nothing
      // branches on the value.
      tx_skip_count_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    rt_acquire_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
  }

  // Non-realtime side. Spins until the guard is free. TX-6 bounds this by one
  // whole-frame send() from ctrl, which is what makes an unbounded-looking loop
  // acceptable here and nowhere else.
  //
  // The load before the retry is the standard test-and-test-and-set: repeated
  // test_and_set calls are read-modify-write operations that take the cache
  // line exclusively on every iteration, so a plain spin on test_and_set slows
  // down the holder it is waiting for -- measurably, and worst when contention
  // is highest.
  void AcquireNonRealtime() noexcept {
    while (flag_.test_and_set(std::memory_order_acquire)) {
      // std::atomic_flag has no load() in C++17 (test() arrived in C++20, and
      // CPP-1 pins the standard at exactly 17), so the read-only part of the
      // test-and-test-and-set is not expressible. The pause hint is what keeps
      // the contended loop from starving the holder.
      CpuRelax();
    }
  }

  // Release is symmetric for both sides on purpose: a separate ReleaseRealtime
  // would only create a way to pair the wrong two calls.
  //
  // memory_order_release pairs with the acquire in both acquire paths, so the
  // bytes this holder wrote to the socket are ordered before the next holder's
  // first write.
  void Release() noexcept { flag_.clear(std::memory_order_release); }

  // CPP-4: this count goes into hello_ack.runtime and the periodic log, and a
  // value above zero is meant to be seen. Reading it is racy by nature and that
  // is fine -- it is a diagnostic, and a diagnostic that needed a lock to read
  // would defeat the purpose of the class.
  uint64_t tx_skip_count() const noexcept {
    return tx_skip_count_.load(std::memory_order_relaxed);
  }

  // The other half of the accounting. attempts == acquires + skips is the only
  // way a test can tell "the realtime side won every time" apart from "the
  // realtime side was never asked", and those two produce the same skip count.
  uint64_t rt_acquire_count() const noexcept {
    return rt_acquire_count_.load(std::memory_order_relaxed);
  }

 private:
  std::atomic_flag flag_;
  std::atomic<uint64_t> tx_skip_count_;
  std::atomic<uint64_t> rt_acquire_count_;
};

// RAII for the realtime side. Construct, ask acquired(), send the whole frame,
// let the destructor release. The point is failure mode 3 in the file comment:
// an early return out of a short-write or error path cannot forget the release.
//
// acquired() == false means skip this tick. It is not an error and must not be
// logged from the realtime thread (QD-7 forbids blocking logs there); the
// counter already records it.
class RealtimeTxScope {
 public:
  explicit RealtimeTxScope(TxGuard* guard) noexcept
      : guard_(guard), acquired_(guard->TryAcquireRealtime()) {}

  ~RealtimeTxScope() noexcept {
    if (acquired_) {
      guard_->Release();
    }
  }

  RealtimeTxScope(const RealtimeTxScope&) = delete;
  RealtimeTxScope& operator=(const RealtimeTxScope&) = delete;

  bool acquired() const noexcept { return acquired_; }

 private:
  TxGuard* guard_;
  bool acquired_;
};

// RAII for the non-realtime side. There is no acquired() because there is no
// failure case -- it waits. Using this type from ctrl or chs_b is the CPP-3
// violation described in the file comment, and the type name is the only
// warning the compiler can give.
class NonRealtimeTxScope {
 public:
  explicit NonRealtimeTxScope(TxGuard* guard) noexcept : guard_(guard) {
    guard_->AcquireNonRealtime();
  }

  ~NonRealtimeTxScope() noexcept { guard_->Release(); }

  NonRealtimeTxScope(const NonRealtimeTxScope&) = delete;
  NonRealtimeTxScope& operator=(const NonRealtimeTxScope&) = delete;

 private:
  TxGuard* guard_;
};

}  // namespace rtcomm
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_RTCOMM_TX_GUARD_H_
