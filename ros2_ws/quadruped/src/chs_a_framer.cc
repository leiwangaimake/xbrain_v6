/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_a_framer.cc
 * Brief: CHS-A stream reassembly implementation (see chs_a_framer.h)
 *
 * Description:
 * The state is four numbers: how many bytes are buffered, how many of them the
 * caller has already been handed (deferred consume), when the current
 * incomplete frame was first seen, and how many bytes have been slid since the
 * last good frame. Everything else is derived per call, which keeps "where am I
 * in the stream" from having two answers that can disagree.
 *
 * *** The deferred consume is the one non-obvious mechanism here, and it exists
 * to avoid copying every frame out. Next() hands back a pointer INTO the
 * buffer; if the bytes were compacted away immediately that pointer would
 * dangle before the caller could read it, and if they were never compacted the
 * buffer would fill. So the consume is RECORDED on the way out and APPLIED at
 * the head of the next entry point -- which is exactly the "valid until the
 * next call" contract in the header, stated there as a rule and implemented
 * here as the reason for it. The alternative, a read cursor, means every bound
 * check has to pick the right one of two indices, and that is the class of
 * mistake this class exists to prevent.
 *
 * Compaction by memmove rather than by cursor is affordable: the largest report
 * is 2.4 KB at 2 Hz.
 */

#include "quadruped/chs_a_framer.h"

#include <cstring>

namespace quadruped {
namespace chs_a {

// The header size is part of the wire format, not a tuning knob; a change here
// without a change to the field table would silently mis-slice every frame.
static_assert(kHeaderBytes == 16, "CHS-A header is 16 bytes (guide 1.1.5)");

Framer::Framer(std::size_t resync_max_bytes, int frame_assembly_timeout_ms)
    : resync_max_bytes_(resync_max_bytes),
      frame_assembly_timeout_s_(static_cast<double>(frame_assembly_timeout_ms) /
                                1000.0) {}

void Framer::ApplyPendingConsume() {
  if (pending_consume_ == 0) return;
  // The frame handed out by the previous call ends here. Shift the remainder
  // down; the caller's pointer is invalid from this moment, which is what the
  // header promises.
  std::memmove(buf_, buf_ + pending_consume_, used_ - pending_consume_);
  used_ -= pending_consume_;
  pending_consume_ = 0;
  frame_ = nullptr;
  frame_len_ = 0;
}

void Framer::Reset() {
  used_ = 0;
  pending_consume_ = 0;
  partial_since_s_ = -1.0;
  frame_ = nullptr;
  frame_len_ = 0;
  resync_run_ = 0;
  // The totals deliberately survive a Reset: they answer "how bad has this link
  // been", and zeroing them on every reconnect would hide a peer that forces a
  // reconnect every few seconds -- the exact pattern worth seeing.
}

bool Framer::Push(const std::uint8_t* data, std::size_t len) {
  if (data == nullptr) return false;
  ApplyPendingConsume();
  if (len > capacity()) return false;
  std::memcpy(buf_ + used_, data, len);
  used_ += len;
  return true;
}

bool Framer::Resync() {
  // Start at offset 1: offset 0 has just been rejected by the caller, and
  // searching from 0 would match nothing new and spin.
  std::size_t i = 1;
  while (i + 4 <= used_) {
    if (buf_[i] == kSync0 && buf_[i + 1] == kSync1 && buf_[i + 2] == kSync2 &&
        buf_[i + 3] == kSync3) {
      break;
    }
    ++i;
  }
  std::size_t skip = i;
  if (i + 4 > used_) {
    // No candidate in what we hold. Keep the last three bytes: a sync word
    // split across two reads is the COMMON case at a frame boundary, and
    // dropping the prefix would make it unrecoverable.
    skip = (used_ >= 3) ? used_ - 3 : 0;
  }
  if (skip > 0) {
    std::memmove(buf_, buf_ + skip, used_ - skip);
    used_ -= skip;
    resync_bytes_total_ += skip;
    resync_run_ += skip;
  }
  // FR-2: a stream that will not resync is not a noisy stream, it is a stream
  // carrying something else. Say so and let the caller close, rather than
  // sliding forever while the robot sits with no state updates and no error.
  return resync_run_ <= resync_max_bytes_;
}

FrameStatus Framer::Next(double now_mono_s) {
  ApplyPendingConsume();

  for (;;) {
    if (used_ < kHeaderBytes) {
      return FrameStatus::kNeedMore;
    }
    if (buf_[0] != kSync0 || buf_[1] != kSync1 || buf_[2] != kSync2 ||
        buf_[3] != kSync3) {
      if (!Resync()) {
        // Poisoned. Clear the buffer so a caller that ignores the status and
        // keeps pushing does not re-report forever on the same bytes.
        used_ = 0;
        partial_since_s_ = -1.0;
        resync_run_ = 0;
        return FrameStatus::kPoisoned;
      }
      continue;  // re-examine from the new offset
    }
    Header h;
    if (!ReadHeader(buf_, used_, &h)) {
      // Unreachable while used_ >= 16 and the sync word matched, but returning
      // rather than asserting keeps a future header change from turning a
      // parse problem into a crash on the receive path.
      return FrameStatus::kNeedMore;
    }
    const std::size_t need = kHeaderBytes + h.asdu_len;
    if (used_ < need) {
      // FR-3. The timer starts on the FIRST sight of this incomplete frame and
      // is NOT refreshed on later calls: refreshing would make the timeout
      // unreachable on a link delivering one byte per poll, which is precisely
      // the stall it guards against.
      if (partial_since_s_ < 0.0) {
        partial_since_s_ = now_mono_s;
        return FrameStatus::kNeedMore;
      }
      if (now_mono_s - partial_since_s_ <= frame_assembly_timeout_s_) {
        return FrameStatus::kNeedMore;
      }
      // Aged out: drop the header we were waiting on and hunt for the next
      // sync word. Keeping it would mean waiting forever for bytes that are
      // not coming.
      ++dropped_frames_;
      partial_since_s_ = -1.0;
      if (!Resync()) {
        used_ = 0;
        resync_run_ = 0;
        return FrameStatus::kPoisoned;
      }
      return FrameStatus::kDropped;
    }
    // A whole frame is present at the front of the buffer.
    frame_ = buf_;
    frame_len_ = need;
    header_ = h;
    partial_since_s_ = -1.0;
    resync_run_ = 0;  // a good frame ends the current noise run
    ++frames_out_;
    pending_consume_ = need;  // applied on the caller's next entry
    return FrameStatus::kFrame;
  }
}

FrameStatus Framer::PushDatagram(const std::uint8_t* data, std::size_t len) {
  // One Framer serves one transport. Clearing the stream state here is not a
  // mode switch -- it is a statement that mixing the two on one instance is
  // not supported, made loudly in code rather than only in the header.
  used_ = 0;
  pending_consume_ = 0;
  partial_since_s_ = -1.0;
  frame_ = nullptr;
  frame_len_ = 0;

  if (data == nullptr || len < kHeaderBytes || len > kMaxFrameBytes) {
    ++dropped_frames_;
    return FrameStatus::kDropped;
  }
  Header h;
  if (!ReadHeader(data, len, &h)) {
    ++dropped_frames_;
    return FrameStatus::kDropped;
  }
  // FR-5: exactly one frame per datagram. A datagram carrying more than one
  // frame, or a truncated one, is dropped rather than partially consumed --
  // UDP gives no ordering, so a leftover has no defined continuation.
  if (kHeaderBytes + static_cast<std::size_t>(h.asdu_len) != len) {
    ++dropped_frames_;
    return FrameStatus::kDropped;
  }
  std::memcpy(buf_, data, len);
  frame_ = buf_;
  frame_len_ = len;
  header_ = h;
  ++frames_out_;
  return FrameStatus::kFrame;
}

}  // namespace chs_a
}  // namespace quadruped
