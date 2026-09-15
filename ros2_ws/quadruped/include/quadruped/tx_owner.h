/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: tx_owner.h
 * Brief: The single outbound owner for channel one -- CA-4 / TX-4..TX-7 (13 S2.2, S9.1)
 *
 * Description:
 * What problem this solves. Three constraints from 13 meet here and each one
 * alone is easy, while the three together are what this class exists to make
 * simultaneously true:
 *   CA-1/CA-2  every outbound frame (axis command, heartbeat, mode, light)
 *              leaves through ONE socket whose source port never changes --
 *              the chassis rejects a second client with 0xE006 for 2 s, and
 *              the symptom is "commands are sent and the robot does not move";
 *   CA-4/QD-1  therefore exactly ONE function owns that socket, so Tier 1
 *              cannot be bypassed by a second send path;
 *   TX-1..TX-7 the estop callback must put a zero-velocity frame on the wire
 *              inside its own callback (11 T-1: <= 5 ms, not "set a flag"),
 *              which means a non-realtime thread and the 100 Hz ctrl thread
 *              write the same socket -- while CPP-3 forbids the realtime side
 *              from ever blocking on a lock.
 *
 * The resolution is asymmetric, and the asymmetry IS the design: the realtime
 * caller makes exactly one attempt and SKIPS its tick on failure; the
 * non-realtime caller spins, bounded by one whole-frame send. Neither waits on
 * a mutex, so there is no priority inversion to reason about. The primitive is
 * common/rtcomm/tx_guard.h (std::atomic_flag, CPP-4); this class is what binds
 * it to the one socket and to the whole-frame rule.
 *
 * Why the writer is injected rather than owned. TX-7 ("one frame, one critical
 * section, short writes completed inside it") is a property of the SEQUENCING,
 * not of the socket, so it can be established and tested before any socket
 * exists -- and it must be, because the failure it prevents (two frames
 * interleaved into one illegal APDU on TCP) shows up as random 0xE001/0xE002
 * that 13 S7.5 tells the reader to blame on our own encoder. A test with a
 * fake writer that records interleaving is the only cheap way to prove it.
 *
 * Boundary. This class does NOT encode frames (B1 codec), does NOT open or
 * reconnect the socket (B2), and does NOT decide what to send (Tier 1 owns
 * that). It decides only WHO may write and WHEN, which is the part that must be
 * right before any of the others exist.
 *
 * The trap worth naming. A "retry the realtime send next tick" helper looks
 * harmless and would defeat the whole thing: the frame the realtime side skips
 * is, by construction, the one the estop callback is writing at that moment
 * (TX-3) -- the correct behaviour is to lose it, not to queue it. A queued axis
 * command is a stale command, and 13 QD-5 forbids presenting stale data as
 * fresh. There is deliberately no queue here.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_TX_OWNER_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_TX_OWNER_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>
#include <utility>

#include "xbrain/rtcomm/tx_guard.h"

namespace quadruped {

// ---------------------------------------------------------------------------
// 13 CPP-4, checked at compile time rather than asserted in prose.
//
// The realistic defect is not someone writing std::mutex here on purpose; it
// is someone rewriting common/rtcomm/tx_guard.h onto a mutex because "the spin
// looks wasteful", which compiles everywhere and puts the 100 Hz ctrl thread
// on a blocking lock held by a non-realtime thread. The symptom is a control
// loop that misses its deadline only under estop load -- the worst possible
// time -- and no error anywhere.
//
// Two checks, because either alone is defeatable:
//   1. the guard must still be BUILT on std::atomic_flag (the alias is
//      published by TxGuard for exactly this);
//   2. both acquire paths must still be noexcept -- std::mutex::lock() is not,
//      so a mutex rewrite either drops noexcept (this fires) or keeps it and
//      terminates on throw (visible in review, not silent).
// ---------------------------------------------------------------------------
static_assert(
    std::is_same<hachist::xbrain::rtcomm::TxGuard::FlagType,
                 std::atomic_flag>::value,
    "13 CPP-4: the tx guard must be built on std::atomic_flag, not a mutex -- "
    "a blocking lock on the realtime side is the priority inversion CPP-3 "
    "forbids");
static_assert(
    noexcept(std::declval<hachist::xbrain::rtcomm::TxGuard&>()
                 .TryAcquireRealtime()),
    "13 CPP-4: the realtime acquire must be noexcept (a try, not a lock)");
static_assert(
    noexcept(std::declval<hachist::xbrain::rtcomm::TxGuard&>()
                 .AcquireNonRealtime()),
    "13 CPP-4: the non-realtime acquire must be noexcept");

// How the caller reached the socket, which decides the wait policy. Named
// rather than a bool because "true means realtime" is exactly the kind of
// call-site ambiguity that puts the 100 Hz loop on the spinning path.
enum class TxCaller {
  kRealtime,     // ctrl thread: one attempt, skip the tick on contention
  kNonRealtime,  // rt_safety / chs_a_tx: spin, bounded by one frame send
};

// Outcome of one attempt. Distinguishes "did not write" from "wrote less than
// the frame", because they demand opposite responses: the first is normal
// contention (skip), the second means the stream now carries half a frame and
// the connection must be dropped (FR-4).
enum class TxResult {
  kSent,          // whole frame handed to the writer
  kSkipped,       // realtime caller found the guard held; frame deliberately lost
  kShortWrite,    // writer could not place the whole frame: caller must reconnect
  kWriterFailed,  // writer reported an error (socket closed, EPIPE, ...)
};

// The seam. Returns bytes written (>= 0) or a negative value on error, i.e.
// the shape of a POSIX send() so the real implementation is a thin wrapper and
// the test double is trivial.
// * Takes (data, len) rather than a container so the caller can hand it a
//   statically preallocated frame buffer: QD-7 forbids allocation on the
//   realtime path, and a std::vector parameter invites exactly that.
using FrameWriter = std::function<long(const std::uint8_t* data, std::size_t len)>;

class TxOwner {
 public:
  // partial_send_retry comes from the config (13 S8.2 chassis_link), not from a
  // constant here: CLAUDE.md 3.1 keeps tunables out of the code, and this one
  // bounds a loop on the estop path.
  TxOwner(FrameWriter writer, int partial_send_retry);

  TxOwner(const TxOwner&) = delete;
  TxOwner& operator=(const TxOwner&) = delete;

  // Send one whole frame. The ONLY way a byte reaches the chassis (CA-4).
  //
  // * noexcept because the realtime caller is a noexcept thread entry (CPP-2):
  //   an exception crossing this boundary would terminate the process, and
  //   while stopping is safe (QD-8), stopping for a transient write error is
  //   an unnecessary outage. Errors are returned, never thrown.
  TxResult Send(TxCaller caller, const std::uint8_t* data,
                std::size_t len) noexcept;

  // Diagnostics that must be visible, not just counted: a guard held by a stuck
  // non-realtime thread stops every axis command, and the only external symptom
  // is a robot that does not move. CPP-4 requires this number to reach
  // hello_ack.runtime, so it is exposed here rather than logged and forgotten.
  std::uint64_t tx_skip_count() const noexcept { return guard_.tx_skip_count(); }
  std::uint64_t rt_acquire_count() const noexcept {
    return guard_.rt_acquire_count();
  }
  // Frames the writer accepted whole. Paired with the two counters above it
  // answers "is the realtime path actually getting through", which no single
  // counter can.
  std::uint64_t sent_count() const noexcept { return sent_count_; }

 private:
  // Write the whole frame, completing a short write inside the SAME critical
  // section (FR-4 / TX-7). Bounded by partial_send_retry_; on exhaustion the
  // caller gets kShortWrite and must close the connection -- leaving half a
  // frame in a TCP stream makes the peer parse the next bytes as a header.
  TxResult WriteWhole(const std::uint8_t* data, std::size_t len) noexcept;

  FrameWriter writer_;
  int partial_send_retry_;
  hachist::xbrain::rtcomm::TxGuard guard_;
  std::uint64_t sent_count_;
};

}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_TX_OWNER_H_
