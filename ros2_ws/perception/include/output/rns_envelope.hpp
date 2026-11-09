/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: rns_envelope.hpp
 * Brief: Bounded process-wide envelope stamps; serialization stays at the caller.
 *
 * Description:
 * What problem this solves. Every r6 message (Objects, Profile, Status, events and notices)
 * carries the 11 section 3.0 outer envelope: v, rid, ts, mono, boot, seq, src, ts_sync. Two of
 * those fields are easy to get wrong: seq must come from ONE counter per producer process
 * (a second counter is a second sequence space the consumer's gap detection cannot reconcile),
 * and ts_sync must be the copied ClockStatus.sync, forced false when none arrived for the
 * sync timeout (CLK-A2/CLK-A3). This class owns exactly those two decisions.
 *
 * Relation to the platform. common/include/xbrain/envelope/envelope_writer.h is the platform's
 * writer. This class keeps its rules (single seq starting at 1, no optimistic ts_sync, 5 s
 * window measured on the monotonic clock, ">= timeout is stale") but differs on purpose:
 *   - Times are int64 milliseconds, not double seconds. Integer ages make the ">= 5000 ms"
 *     boundary exact; a double subtraction of two second counts can land either side of it.
 *     The callers render seconds from the integer parts (e.g. 12345 -> 12.345), so the wire
 *     still carries seconds as 11 section 3.0 requires, only with 3 instead of 6 decimals.
 *   - It is thread-safe (atomics), because Objects, Profile, Status and the event channels
 *     are published from different threads and must share one seq space.
 *   - It rejects what the platform accepts silently: invalid rid/boot, negative times, a
 *     ClockStatus receipt older than the last one, and a receipt in the future relative to the
 *     stamp (the platform would treat that negative age as fresh).
 *
 * Current use. Perception does not subscribe to ClockStatus in this delivery (reply 2026-09-17,
 * Q1: the key is not registered for perception), so note_clock_status() is never
 * called by the service and every envelope carries ts_sync=false by the CLK-A3 rule. The
 * receive/expire logic is implemented and tested so that a later subscription only has to
 * call note_clock_status().
 */
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>

namespace perception::rns {

// The per-message part of the envelope. v, rid, boot and src are per-process constants and are
// read from ProcessEnvelope directly, so this stays a small trivially copyable value.
struct EnvelopeStamp {
    // 1 for the first stamped message of the process; never 0 on a successful stamp.
    std::uint64_t seq = 0;
    // Milliseconds as passed to stamp(); -1 means "not stamped" in a default value.
    std::int64_t wall_ms = -1, mono_ms = -1;
    // Copied ClockStatus.sync if fresh at mono_ms, otherwise false (CLK-A2/CLK-A3).
    bool ts_sync = false;
};

// Create exactly ONE instance for the producer process and share it across all
// topics. Sequence allocation is thread-safe; network transmission order is not
// guaranteed by a sequence allocator. v=1 and src="perception" are constants.
//
// Same ts_sync and seq rules as the platform EnvelopeWriter for valid
// configuration, nondecreasing input time and calls before sequence exhaustion;
// the unit differs (milliseconds here, seconds there, see the file header). The
// caller converts wall_ms/mono_ms to seconds for docs11 section 3.0 JSON
// serialization. This implementation is stricter on invalid identities/times and
// never treats a ClockStatus received in the future as fresh. It does not read
// any clock: the caller passes the times in, which keeps the staleness rule
// testable with injected values.
//
// Only construction may allocate. Calls have bounded storage and at most
// kMaxCasAttempts strong-CAS attempts; false means invalid input, sequence
// exhaustion, or excessive contention. On stamp failure, out is unchanged.
// Callers must not fabricate a replacement sequence after failure.
class ProcessEnvelope {
public:
    // docs11 rid length limit; the fixed buffer below is sized from it.
    static constexpr std::size_t kMaxRidLength = 32;
    // INT64_MAX is excluded so that (ms << 1 | sync) always differs from kNoClock.
    // The other r6 output modules validate monotonic times against this bound too.
    static constexpr std::int64_t kMaxMonoMs =
        std::numeric_limits<std::int64_t>::max() - 1;
    // Bounded retry keeps a stamp's worst-case time finite on the publish path.
    // The handful of publishing threads (Objects callback, Profile, Status,
    // events) cannot plausibly make one caller lose 64 CAS races in a row; if
    // it happens, that message is not published rather than blocking a thread.
    static constexpr unsigned kMaxCasAttempts = 64;
    static constexpr int kVersion = 1;
    static constexpr std::string_view kSource = "perception";

    // rid: docs11 [a-z0-9_-]{1,32}; boot: exactly 8 lowercase hex digits.
    // sync_timeout_ms must be explicitly supplied and strictly positive: it is a
    // safety parameter (CLK-A3, 5 s) and a code default would hide a wrong value.
    // Invalid arguments do not throw; valid() reports them and every call fails.
    ProcessEnvelope(std::string rid, std::string boot, std::int64_t sync_timeout_ms);
    // Not copyable or movable: a copy would be a second seq source (see above).
    ProcessEnvelope(const ProcessEnvelope&) = delete;
    ProcessEnvelope& operator=(const ProcessEnvelope&) = delete;
    ProcessEnvelope(ProcessEnvelope&&) = delete;
    ProcessEnvelope& operator=(ProcessEnvelope&&) = delete;

    // False when the constructor rejected rid, boot or sync_timeout_ms.
    bool valid() const noexcept;
    // received_mono_ms is the caller's LOCAL receipt time, not remote payload
    // time. Reject negative/unrepresentable times and receipts older than the
    // last accepted receipt. Equal times are permitted (millisecond resolution).
    // No "now" is available here: stamp enforces receipt <= its now_mono_ms.
    bool note_clock_status(bool sync, std::int64_t received_mono_ms) noexcept;
    // wall_ms >= 0; mono_ms in [0, kMaxMonoMs]. Positive wall-clock steps in
    // either direction are allowed and do not affect ClockStatus freshness.
    bool stamp(std::int64_t wall_ms, std::int64_t mono_ms, EnvelopeStamp& out) noexcept;
    // Views into this object's fixed storage; valid for its lifetime. Both are
    // empty when the configuration was rejected.
    std::string_view rid() const noexcept;
    std::string_view boot() const noexcept;

private:
    // Allows the host test to exercise uint64 exhaustion without a public
    // sequence-reset API; production callers cannot reset the sequence.
    friend struct ProcessEnvelopeTestAccess;

    // A single uint64 publishes a coherent (receipt_ms, sync) snapshot. Values
    // encode receipt_ms << 1 | sync; UINT64_MAX denotes "never received".
    // Excluding INT64_MAX leaves that sentinel unambiguous (a ~292 Myr bound).
    static constexpr std::uint64_t kNoClock = std::numeric_limits<std::uint64_t>::max();
    // A lock-based atomic could block a publisher thread; refuse to build instead.
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
                  "Envelope stamping requires lock-free 64-bit atomics");

    // Fixed storage cannot inherit an oversized input string's capacity.
    std::array<char, kMaxRidLength> rid_{};
    std::array<char, 8> boot_{};
    std::size_t rid_length_ = 0;
    const std::int64_t sync_timeout_ms_;
    bool valid_ = false;
    // Last issued seq; 0 before the first stamp, so the first message gets 1.
    std::atomic<std::uint64_t> seq_{0};
    // Encoded ClockStatus snapshot, see kNoClock.
    std::atomic<std::uint64_t> clock_{kNoClock};
};

}  // namespace perception::rns
