/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: chs_a_framer.h
 * Brief: CHS-A stream reassembly -- FR-1..FR-5 (13 S2.2)
 *
 * Description:
 * What problem this solves. TCP is a byte stream: it has no message boundary
 * and one read() can deliver half a frame, three frames, or two and a half.
 * The vendor guide describes the frame and says nothing about reassembly, so a
 * first implementation that calls recv() and parses what it got works on a
 * quiet desk and fails on a loaded link -- as random 0xE001/0xE002, which
 * 13 S7.5 tells the reader to blame on our own encoder. This class is the
 * reason that misdiagnosis cannot happen.
 *
 * The rules it implements, verbatim from 13 S2.2:
 *   FR-1  read 16 header bytes, check the sync word, take the LE16 length --
 *         which counts the ASDU ALONE -- then wait for exactly that many more
 *   FR-2  on a sync mismatch slide ONE byte and look again; after
 *         resync_max_bytes slid without a match the link is poisoned and the
 *         caller must reconnect (a stream that never resyncs is not a stream
 *         with noise in it, it is a stream carrying something else)
 *   FR-3  a partial frame older than frame_assembly_timeout_ms is dropped and
 *         resync starts again; the buffer is statically sized at one maximum
 *         frame, so a peer that sends a huge length cannot make us allocate
 *   FR-5  in UDP mode one datagram is one frame: the length and sync are still
 *         validated, but a datagram that does not contain exactly one whole
 *         frame is dropped rather than carried over into the next one
 *
 * Time is injected, never read here. Every deadline in this class is monotonic
 * (CLK-C1), and passing `now_mono_s` in makes the timeout testable without
 * sleeping -- a test that sleeps is a test that is flaky on a loaded machine
 * and slow on a quiet one.
 *
 * Boundary: this yields ASDU byte ranges. It does not parse them (chs_a_codec
 * for routing, B2 for payloads) and it does not own the socket (B2). It never
 * copies a frame out: Next() hands back a pointer INTO the buffer, valid until
 * the next call, which is what keeps a 10 Hz stream of 2.4 KB device reports
 * from allocating on every frame.
 */
#ifndef HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_FRAMER_H_
#define HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_FRAMER_H_

#include <cstddef>
#include <cstdint>

#include "quadruped/chs_a_codec.h"

namespace quadruped {
namespace chs_a {

// What one Next() call found. Four outcomes, not a bool, because three of them
// demand different handling and collapsing any two of them loses the
// distinction the caller needs.
enum class FrameStatus {
  kNeedMore,   // no complete frame yet; feed more bytes
  kFrame,      // a whole frame is available via frame()/asdu()
  kDropped,    // a partial frame aged out (FR-3) or a datagram was malformed
  kPoisoned,   // resync budget exhausted (FR-2): CLOSE the connection
};

class Framer {
 public:
  // resync_max_bytes and frame_assembly_timeout_ms come from the config
  // (13 S8.2 chassis_link), not from constants here -- both bound a recovery
  // path, and CLAUDE.md 3.1 keeps that kind of number out of the code.
  Framer(std::size_t resync_max_bytes, int frame_assembly_timeout_ms);

  Framer(const Framer&) = delete;
  Framer& operator=(const Framer&) = delete;

  // Append bytes read from the socket. Returns false when the data does not
  // fit, which can only happen if the caller ignored capacity() -- the buffer
  // holds one maximum-size frame, so a correct caller never overflows it.
  bool Push(const std::uint8_t* data, std::size_t len);

  // Try to extract the next frame. Call in a loop until it stops returning
  // kFrame: one read() commonly carries several reports.
  //
  // now_mono_s is CLOCK_MONOTONIC seconds, used ONLY for the FR-3 partial
  // timeout. A jump in this value cannot corrupt a frame; it can only cause an
  // early or late discard of an incomplete one.
  FrameStatus Next(double now_mono_s);

  // Valid until the next Next()/Push(). Pointer into the internal buffer.
  const std::uint8_t* frame() const { return frame_; }
  std::size_t frame_len() const { return frame_len_; }
  // The payload alone, which is what a parser wants.
  const std::uint8_t* asdu() const { return frame_ + kHeaderBytes; }
  std::size_t asdu_len() const { return frame_len_ - kHeaderBytes; }
  const Header& header() const { return header_; }

  // How many bytes the caller may still Push before it must drain with Next().
  std::size_t capacity() const { return kMaxFrameBytes - used_; }

  // Diagnostics. Counters rather than log lines because the interesting signal
  // is a RATE: one resync after a reconnect is normal, one per second means
  // the peer and we disagree about the frame format, and only a counter shows
  // the difference.
  std::uint64_t resync_bytes_total() const { return resync_bytes_total_; }
  std::uint64_t dropped_frames() const { return dropped_frames_; }
  std::uint64_t frames_out() const { return frames_out_; }

  // Drop everything buffered. Called on reconnect: bytes from the old
  // connection must never be parsed as the beginning of the new one.
  void Reset();

  // FR-5: one datagram is one frame. Returns kFrame when the datagram holds
  // exactly one valid frame, kDropped otherwise. Deliberately does NOT share
  // the stream buffer -- carrying a partial datagram into the next one is the
  // bug this separate entry point exists to prevent, since UDP has no ordering
  // guarantee that would make the continuation meaningful.
  FrameStatus PushDatagram(const std::uint8_t* data, std::size_t len);

 private:
  // Slide to the next sync-word candidate, counting the skipped bytes against
  // the FR-2 budget. Returns false when the budget is exhausted.
  bool Resync();

  // Drop the frame handed out by the previous call. Deferred rather than done
  // on the way out so the caller's pointer stays valid until it calls again --
  // see the .cc file comment for why that beats a read cursor.
  void ApplyPendingConsume();

  std::size_t resync_max_bytes_;
  double frame_assembly_timeout_s_;

  std::uint8_t buf_[kMaxFrameBytes];
  std::size_t used_ = 0;
  // Bytes at the front already handed to the caller, removed on the next entry.
  std::size_t pending_consume_ = 0;

  // When the current incomplete frame was first seen, for FR-3. Negative means
  // "no partial frame in flight", which is distinct from 0.0 (a legitimate
  // monotonic reading right after boot).
  double partial_since_s_ = -1.0;

  const std::uint8_t* frame_ = nullptr;
  std::size_t frame_len_ = 0;
  Header header_;

  std::uint64_t resync_bytes_total_ = 0;
  std::uint64_t dropped_frames_ = 0;
  std::uint64_t frames_out_ = 0;
  // Bytes slid since the last successfully framed message. Reset on success so
  // that noise spread over hours does not eventually trip the budget.
  std::size_t resync_run_ = 0;
};

}  // namespace chs_a
}  // namespace quadruped

#endif  // HACHIST_XBRAIN_V6_QUADRUPED_CHS_A_FRAMER_H_
