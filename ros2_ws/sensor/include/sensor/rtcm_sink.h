/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rtcm_sink.h
 * Brief: Main-thread RTCM write queue onto the receiver serial port (RTG-6)
 *
 * Description:
 * The 20 Hz loop owns the serial fd (RTG-2). Corrections reach it as whole,
 * CRC-checked frames from the network thread; this queue writes them out without
 * ever blocking the loop and without ever interleaving two frames:
 *   - a frame that has started to go out is finished before the next one starts,
 *     across as many ticks as the non-blocking fd needs -- the receiver only ever
 *     sees a sequence of whole frames;
 *   - frames waiting longer than frame_max_age_s are dropped (stale corrections
 *     are worse than none), but never the one already half written;
 *   - the queue holds at most queue_max_bytes; overflow drops the oldest whole
 *     frame that has not started;
 *   - every frame carries its source tag; a frame from a different source than
 *     the queued ones flushes the queue first (RTG-4: never two references in a
 *     row), the network thread having flushed on its side already.
 *
 * Pure: the write goes through an injected function with write(2) semantics, so
 * partial writes, EAGAIN and errors are tested with a fake. Main-thread only.
 */

#ifndef SENSOR__RTCM_SINK_H_
#define SENSOR__RTCM_SINK_H_

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

namespace sensor {

// configs rtcm.sink.*. No defaults (CLAUDE.md 3.1).
struct SinkConfig {
  std::size_t queue_max_bytes;
  double frame_max_age_s;
};

// One frame handed over by the network thread.
struct TaggedFrame {
  std::string bytes;
  int source = 0;         // ActiveSource as int: which source produced it
  double t_mono = 0.0;    // when the network thread received it
};

// write(2)-like: returns bytes written, or -1 with errno set.
using SerialWriteFn = std::function<ssize_t(const char* data, std::size_t n)>;

enum class DrainResult { kOk, kError };

class RtcmSink {
 public:
  explicit RtcmSink(SinkConfig cfg) : cfg_(cfg) {}

  void push(TaggedFrame f);

  // Write as much as the fd takes now. kError means a write failed with
  // something other than EAGAIN; the caller hands the fd to its reopen logic.
  DrainResult drain(const SerialWriteFn& write_fn, double now_mono_s);

  // Serial reopened: the receiver lost any half frame, the rest is stale.
  void clear();

  std::size_t queuedBytes() const { return queued_bytes_; }
  uint64_t framesWritten() const { return frames_written_; }
  uint64_t bytesWritten() const { return bytes_written_; }
  uint64_t droppedOverflow() const { return dropped_overflow_; }
  uint64_t droppedStale() const { return dropped_stale_; }
  uint64_t droppedSwitch() const { return dropped_switch_; }

 private:
  void dropFront(uint64_t* counter);

  SinkConfig cfg_;
  std::deque<TaggedFrame> q_;
  std::size_t queued_bytes_ = 0;
  std::size_t partial_ = 0;   // bytes of q_.front() already written; > 0 = started
  uint64_t frames_written_ = 0;
  uint64_t bytes_written_ = 0;
  uint64_t dropped_overflow_ = 0;
  uint64_t dropped_stale_ = 0;
  uint64_t dropped_switch_ = 0;
};

}  // namespace sensor

#endif  // SENSOR__RTCM_SINK_H_
