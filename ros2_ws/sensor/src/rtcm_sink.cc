/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtcm_sink.cc
 * Brief: RTCM write queue (see rtcm_sink.h)
 *
 * Description:
 * The started frame (partial_ > 0) is pinned at the front: overflow, staleness
 * and a source switch only ever drop frames behind it, and a switch lets it
 * finish -- cutting it would hand the receiver a broken frame, which is exactly
 * what the whole-frame rule is there to prevent.
 */

#include "sensor/rtcm_sink.h"

#include <cerrno>
#include <utility>

namespace sensor {

void RtcmSink::dropFront(uint64_t* counter) {
  queued_bytes_ -= q_.front().bytes.size();
  q_.pop_front();
  ++*counter;
}

void RtcmSink::push(TaggedFrame f) {
  if (!q_.empty() && q_.back().source != f.source) {
    // Source switched: keep only a frame already on the wire.
    while (q_.size() > (partial_ > 0 ? 1u : 0u)) {
      queued_bytes_ -= q_.back().bytes.size();
      q_.pop_back();
      ++dropped_switch_;
    }
  }
  queued_bytes_ += f.bytes.size();
  q_.push_back(std::move(f));
  while (queued_bytes_ > cfg_.queue_max_bytes && q_.size() > (partial_ > 0 ? 1u : 0u)) {
    if (partial_ > 0) {
      // Front is on the wire: drop the oldest one behind it.
      auto it = q_.begin() + 1;
      queued_bytes_ -= it->bytes.size();
      q_.erase(it);
      ++dropped_overflow_;
    } else {
      dropFront(&dropped_overflow_);
    }
  }
}

DrainResult RtcmSink::drain(const SerialWriteFn& write_fn, double now_mono_s) {
  while (!q_.empty()) {
    if (partial_ == 0 && (now_mono_s - q_.front().t_mono) > cfg_.frame_max_age_s) {
      dropFront(&dropped_stale_);
      continue;
    }
    const std::string& b = q_.front().bytes;
    const ssize_t n = write_fn(b.data() + partial_, b.size() - partial_);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) return DrainResult::kOk;
      return DrainResult::kError;
    }
    partial_ += static_cast<std::size_t>(n);
    bytes_written_ += static_cast<uint64_t>(n);
    if (partial_ < b.size()) return DrainResult::kOk;   // fd full: finish next tick
    partial_ = 0;
    queued_bytes_ -= b.size();
    q_.pop_front();
    ++frames_written_;
  }
  return DrainResult::kOk;
}

void RtcmSink::clear() {
  q_.clear();
  queued_bytes_ = 0;
  partial_ = 0;
}

}  // namespace sensor
