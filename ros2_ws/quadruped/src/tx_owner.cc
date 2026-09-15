/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: tx_owner.cc
 * Brief: TxOwner implementation -- one socket, two wait policies (see tx_owner.h)
 *
 * Description:
 * The whole file is thirty lines of logic guarding one invariant: between the
 * first and last byte of a frame, no other thread writes the socket. Everything
 * else -- the counters, the two enums -- exists so that a violation is visible
 * rather than inferred from a chassis error code hours later.
 *
 * Reading order: Send() picks the wait policy, WriteWhole() owns the bytes. The
 * split is deliberate -- the retry loop must not be reachable without holding
 * the guard, and a single function would let a later edit move the loop above
 * the acquire without anything failing.
 */

#include "quadruped/tx_owner.h"

namespace quadruped {

namespace {
// A writer that returns 0 forever would spin the retry loop to exhaustion and
// report a short write, which is the right outcome but a confusing one to read
// in a log. Nothing enforces progress here on purpose: bounding the loop by
// count (not by "did it progress") is what keeps it terminating even when the
// writer misbehaves.
constexpr long kWriterError = -1;
}  // namespace

TxOwner::TxOwner(FrameWriter writer, int partial_send_retry)
    : writer_(std::move(writer)),
      partial_send_retry_(partial_send_retry),
      guard_(),
      sent_count_(0) {}

TxResult TxOwner::WriteWhole(const std::uint8_t* data,
                             std::size_t len) noexcept {
  std::size_t done = 0;
  // attempts counts COMPLETION attempts after the first write, which is what
  // partial_send_retry bounds (13 S8.2: "TCP short write bounded completion").
  // Counting the first write as a retry would silently halve the configured
  // budget.
  int attempts = 0;
  while (done < len) {
    const long n = writer_(data + done, len - done);
    if (n <= kWriterError) {
      // The socket is gone or refused. Not a short write: there is nothing on
      // the wire to complete, so the caller reconnects rather than closes.
      return TxResult::kWriterFailed;
    }
    done += static_cast<std::size_t>(n);
    if (done >= len) break;
    if (++attempts > partial_send_retry_) {
      // Half a frame is now in the stream. The caller MUST close the
      // connection (FR-4): the peer will read the remaining bytes as a frame
      // header and every subsequent frame is garbage, which presents as random
      // 0xE001/0xE002 and gets misattributed to our encoder.
      return TxResult::kShortWrite;
    }
  }
  ++sent_count_;
  return TxResult::kSent;
}

TxResult TxOwner::Send(TxCaller caller, const std::uint8_t* data,
                       std::size_t len) noexcept {
  // A zero-length frame is a caller bug, not a wire condition. Reporting it as
  // "sent" would make an encoder defect invisible; reporting it as a writer
  // failure would make the caller reconnect a healthy socket. Treat it as a
  // short write: loud, and the caller's recovery path (close + reconnect) is
  // harmless if it ever fires.
  if (data == nullptr || len == 0) {
    return TxResult::kShortWrite;
  }

  if (caller == TxCaller::kRealtime) {
    // One attempt, ever. RealtimeTxScope does the try-and-release; on failure
    // the tick is skipped and the frame is DROPPED, never queued -- see the
    // header for why a queue here would be a defect rather than a feature.
    hachist::xbrain::rtcomm::RealtimeTxScope scope(&guard_);
    if (!scope.acquired()) {
      return TxResult::kSkipped;
    }
    return WriteWhole(data, len);
  }

  // Non-realtime: spin until the guard is free. Bounded by one whole-frame
  // send from ctrl (TX-6), which is the only reason an unbounded-looking wait
  // is acceptable on the estop path.
  hachist::xbrain::rtcomm::NonRealtimeTxScope scope(&guard_);
  return WriteWhole(data, len);
}

}  // namespace quadruped
