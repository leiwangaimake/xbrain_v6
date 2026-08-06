/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: lockfree_slot.h
 * Brief: Single producer / single consumer latest-value slot, triple buffered
 *
 * Description:
 * What problem this solves. 13 S9.1 states that the lock-free single slot
 * (latest value overwrites) is the ONLY means of moving data between the
 * threads of quadruped, and 12 RTC-6 says the same for P1. The reason is not
 * elegance: a queue accumulates, and a consumer that drains a queue consumes
 * data that was already superseded. In an odometry integrator that is a
 * verbatim violation of QD-5 ("do not resend stale data as if it were new"),
 * and the failure is silent -- the numbers look plausible, they are simply from
 * the past, and the robot drives on a pose that lags further behind the more
 * loaded the machine gets.
 *
 * Which design section this implements. 13 S9.1 (thread table, the sentence
 * under it about the single slot), 13 S9.2 RTC-6, and QD-7 (no dynamic
 * allocation, no mutex that a non-realtime thread could hold). The producer of
 * the IMU slot is chs_b at 200 Hz and its consumer is ctrl at 100 Hz, so the
 * design point is exactly "publish faster than you consume and let the old
 * values die".
 *
 * How it works, and why three buffers rather than a seqlock. The classic
 * seqlock has the reader retry while the writer is mid-update; on the realtime
 * side that is a wait on a non-realtime writer, which is what CPP-3 forbids,
 * and the payload copy raced against a concurrent write is formally undefined
 * behaviour. This is a triple buffer instead:
 *
 *   invariant: {write_index_, read_index_, latest_ & kIndexMask} is always a
 *              permutation of {0, 1, 2}
 *
 *   Publish:   write into buffer_[write_index_]        (producer owns it alone)
 *              old = latest_.exchange(write_index_ | kFreshBit)
 *              write_index_ = old & kIndexMask
 *
 *   TakeFresh: if fresh bit is clear -> report "nothing new", touch nothing
 *              old = latest_.exchange(read_index_)
 *              read_index_ = old & kIndexMask
 *              copy out of buffer_[read_index_]        (consumer owns it alone)
 *
 * Both sides perform exactly one atomic exchange and never loop. Because the
 * three indices stay a permutation, the buffer the producer writes is never the
 * buffer the consumer reads, so the payload copy is an ordinary non-atomic
 * access to memory nobody else touches -- no data race, no torn value, no
 * retry. The release/acquire pair on the exchange is what publishes the payload
 * bytes to the consumer.
 *
 * What this class deliberately does NOT do:
 *   * it is not a queue and will never grow one; if the consumer wants every
 *     sample, this is the wrong container and the requirement is wrong (13 S9.1)
 *   * it is not multi-producer or multi-consumer; two producers would both
 *     mutate write_index_ and the permutation invariant collapses. The
 *     assignment of one producer thread and one consumer thread per slot is the
 *     caller's, because only the caller knows its thread model (13 S9.1)
 *   * it does not timestamp anything. Freshness in the CLK-C1 sense (age
 *     against a monotonic clock) belongs to the payload struct the caller
 *     defines; a clock in here would tempt callers to treat "the slot says
 *     fresh" as "the datum is recent", which are different claims
 *   * it does not carry a default or zero value
 *
 * The trap that looks right and is not. The obvious API is `T Read()` returning
 * the last value, with a zero-initialised T before the first publish. That
 * turns "no data has ever arrived" into a fully plausible reading -- zero
 * velocity, zero range, identity attitude -- which is the exact fail-silent
 * shape CLAUDE.md 3.1 was written about. So TakeFresh returns false and does
 * not write through the out pointer at all: a caller that ignores the return
 * value gets whatever it already had, never a fabricated zero.
 *
 * Second trap: calling TakeFresh twice per control tick. The second call
 * reports "nothing new" and the caller that treats that as an error will drop
 * a perfectly good sample. One take per tick, keep the value.
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_RTCOMM_LOCKFREE_SLOT_H_
#define HACHIST_XBRAIN_V6_COMMON_RTCOMM_LOCKFREE_SLOT_H_

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace hachist {
namespace xbrain {
namespace rtcomm {

// The claim in the type name has to be true, so it is checked rather than
// asserted in prose. std::atomic<unsigned> is permitted by the standard to be
// implemented with a hidden lock; on such a target every Publish would take a
// lock that a non-realtime thread can hold, which is precisely CPP-3's
// prohibition, and the class name would be a lie. ATOMIC_INT_LOCK_FREE == 2
// means "always lock free" (1 means "sometimes", which is not good enough).
static_assert(ATOMIC_INT_LOCK_FREE == 2,
              "std::atomic<unsigned> is not always lock free on this target; "
              "the single slot cannot be used on the realtime path here");

template <typename T>
class LockfreeSlot {
 public:
  // Trivially copyable is required, not merely convenient. A payload with a
  // heap member (std::string, std::vector) would allocate inside Publish, and
  // Publish runs in the 200 Hz chs_b thread where QD-7 and 13 S9.2 RTC-5 forbid
  // dynamic allocation. Rejecting it at compile time is the only way to keep
  // that true, because the allocation is invisible at the call site.
  static_assert(std::is_trivially_copyable<T>::value,
                "LockfreeSlot payload must be trivially copyable; a heap-owning "
                "payload would allocate on the realtime publish path (QD-7)");

  // Nothing is allocated and the payload buffers are left uninitialised on
  // purpose. Zero-filling them would create a value that is indistinguishable
  // from a real reading if a future bug ever handed one out; leaving them alone
  // means such a bug produces obvious garbage instead of a plausible zero.
  // No buffer is ever readable before the first Publish -- the fresh bit gates
  // that -- so the uninitialised bytes are never read.
  LockfreeSlot() noexcept
      : latest_(kInitialLatestIndex),
        write_index_(kInitialWriteIndex),
        read_index_(kInitialReadIndex),
        publish_count_(0),
        take_count_(0) {}

  // Copying would duplicate the indices while the buffers stay distinct, which
  // breaks the permutation invariant in a way no test would notice until the
  // two copies were used concurrently. There is no legitimate reason to copy a
  // slot, so the compiler refuses.
  LockfreeSlot(const LockfreeSlot&) = delete;
  LockfreeSlot& operator=(const LockfreeSlot&) = delete;

  // Producer side. Callable from a realtime thread: one memory copy plus one
  // atomic exchange, no loop, no syscall, no allocation, no failure mode.
  void Publish(const T& value) noexcept {
    // The producer owns buffer_[write_index_] outright. The consumer cannot be
    // reading it because the three indices are a permutation.
    buffer_[write_index_] = value;

    // acq_rel, not release: the release half publishes the bytes just written;
    // the acquire half is needed because the index this exchange RETURNS was
    // last written by the consumer, and the producer is about to take ownership
    // of that buffer. With plain release the producer could reorder its next
    // write ahead of learning which buffer it now owns.
    const unsigned old =
        latest_.exchange(write_index_ | kFreshBit, std::memory_order_acq_rel);
    write_index_ = old & kIndexMask;

    // Diagnostics only, hence relaxed: nothing branches on this counter, it
    // exists so that "the consumer is missing samples" can be told apart from
    // "the producer stopped" during a soak. Relaxed keeps it off the critical
    // path in both senses.
    publish_count_.fetch_add(1, std::memory_order_relaxed);
  }

  // Consumer side. Returns true and writes *out only when a value published
  // since the previous successful take is available.
  //
  // The false case is the whole point of the API: it is the truthful answer to
  // "is there something new", which QD-5 requires the caller to be able to ask.
  // The alternative -- returning the previous value again -- is what makes
  // stale data indistinguishable from fresh data downstream.
  bool TakeFresh(T* out) noexcept {
    // Reading the flag before exchanging costs one load and avoids churning the
    // indices when nothing was published; more importantly it means a consumer
    // polling faster than the producer does not keep swapping buffers under a
    // producer that is mid-Publish.
    if ((latest_.load(std::memory_order_acquire) & kFreshBit) == 0u) {
      return false;
    }

    // If the producer published again between the load above and this exchange,
    // the exchange returns the newer index. That is the desired behaviour: the
    // consumer always ends up holding the most recent published buffer, never
    // the one it happened to observe a moment ago.
    const unsigned old = latest_.exchange(read_index_, std::memory_order_acq_rel);
    read_index_ = old & kIndexMask;

    // Now exclusively the consumer's buffer, so this is an ordinary copy.
    *out = buffer_[read_index_];
    take_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
  }

  // How many values were published and how many were taken. published - taken
  // is the number of samples that were overwritten before anyone looked at
  // them, which for this container is normal operation and not an error -- see
  // the class comment. Exposed because a soak needs to see that the producer
  // really did outrun the consumer, otherwise a test that "proves" the slot
  // drops stale data may simply never have had any to drop.
  uint64_t publish_count() const noexcept {
    return publish_count_.load(std::memory_order_relaxed);
  }

  uint64_t take_count() const noexcept {
    return take_count_.load(std::memory_order_relaxed);
  }

 private:
  // Three buffers is the minimum that works, and the reason is worth stating
  // because two looks sufficient. With two, the moment the consumer takes the
  // buffer the producer is not writing, the producer's next Publish has nowhere
  // to go but the buffer the consumer is reading -- a torn value under exactly
  // the load this container exists for. The third buffer is what lets both
  // sides always own one while a third waits.
  static constexpr unsigned kBufferCount = 3u;

  // Two bits of index plus one fresh bit, packed into the single atomic word so
  // that "which buffer is newest" and "has anyone taken it yet" change together
  // in one indivisible step. Two separate atomics would admit a window where
  // the index has moved but the flag has not.
  //
  // The mask admits the value 3, which is not a valid buffer index. It is
  // unreachable by construction: the three indices start as a permutation of
  // {0, 1, 2} and every exchange swaps two of them, so no fourth value can
  // enter. A runtime check for it would be dead code on the 200 Hz path and
  // would suggest the invariant is in doubt, which it is not.
  static constexpr unsigned kIndexMask = 0x3u;
  static constexpr unsigned kFreshBit = 0x4u;

  // The initial permutation. No fresh bit is set, so the first TakeFresh
  // reports "nothing new" as it must.
  static constexpr unsigned kInitialWriteIndex = 0u;
  static constexpr unsigned kInitialReadIndex = 1u;
  static constexpr unsigned kInitialLatestIndex = 2u;

  T buffer_[kBufferCount];

  // Written by both sides, always with exchange.
  std::atomic<unsigned> latest_;

  // Touched by the producer only. Not atomic on purpose: making it atomic would
  // suggest the consumer may read it, and a consumer that read it would break
  // the ownership argument this class rests on.
  unsigned write_index_;

  // Touched by the consumer only, for the same reason.
  unsigned read_index_;

  std::atomic<uint64_t> publish_count_;
  std::atomic<uint64_t> take_count_;
};

}  // namespace rtcomm
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_RTCOMM_LOCKFREE_SLOT_H_
