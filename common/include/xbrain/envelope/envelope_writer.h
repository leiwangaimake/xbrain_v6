/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: envelope_writer.h
 * Brief: C++17 RT-plane envelope writer -- stamps the 11 S3.0 outer fields and
 *        copies ts_sync from ClockStatus with the CLK-A3 5 s fail-safe
 *
 * Description:
 * CPP-CXX-2. The RT-plane C++ processes (quadruped, and any future RT producer)
 * must stamp the outer envelope (11 S3.0: v / rid / ts / mono / boot / seq / src
 * / ts_sync) on every message they publish. The one field that is easy to get
 * wrong is ts_sync, and getting it wrong is a safety fault: a stale or optimistic
 * ts_sync tells the rest of the system the clock is synced when it is not.
 *
 * The ts_sync rule, verbatim from 11 S1.5.3:
 *   CLK-A2: a process must NOT judge sync itself. It COPIES the most recent
 *           ClockStatus.sync (rtk_driver is the only judge, CLK-A1).
 *   CLK-A3: if no ClockStatus has arrived for >= sync_timeout (5 s, measured on
 *           the MONOTONIC clock, CLK-C1), ts_sync is forced false (fail-safe).
 * So ts_sync = (a ClockStatus was received AND it is fresher than sync_timeout)
 *              ? that ClockStatus's sync value : false.
 *
 * What this writer does NOT do, and why:
 *   * It does not read a clock. Like message_age.h in this same package, it takes
 *     the monotonic reading as a parameter, so the staleness maths is testable
 *     with injected values and there is no CLOCK_REALTIME to read the wrong one
 *     from (CLK-C1). The caller reads steady_clock and passes the millisecond
 *     count in.
 *   * It does not encode JSON. It fills a plain struct; serialising it to the
 *     wire is the caller's transport concern, kept out so this header pulls in no
 *     JSON dependency and stays usable on the constrained RT-plane processes.
 *   * It has NO default for sync_timeout_ms. It is a safety parameter
 *     (common.safety.clock.sync_timeout_s, 11 T-11 = 5 s) and CLAUDE.md 3.1
 *     forbids a code default; a wrong-but-plausible 0 here would force ts_sync
 *     false forever, or a large one would trust a dead clock.
 *
 * Traps -- things that look right and are not:
 *   1. Defaulting ts_sync true "until we hear otherwise". Before the first
 *      ClockStatus, clock_received_ is false and ts_sync is false. A true default
 *      is the exact fail-silent this guards (test mutation 1).
 *   2. Measuring the 5 s on the wall clock (the ts field) instead of mono. A
 *      backward wall-clock step would then make a fresh ClockStatus look stale,
 *      or a forward one make a dead clock look fresh. The staleness compare uses
 *      now_mono_ms only (test mutation 2 injects a wall step).
 *   3. A second place that hand-builds an envelope and increments its own seq.
 *      There must be exactly one seq source per producer; a second is a second
 *      sequence space the consumer's gap detection cannot reconcile
 *      (test mutation 3 is a scan for a second seq++).
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_ENVELOPE_ENVELOPE_WRITER_H_
#define HACHIST_XBRAIN_V6_COMMON_ENVELOPE_ENVELOPE_WRITER_H_

#include <cstdint>
#include <string>

namespace hachist {
namespace xbrain {
namespace envelope {

// The stamped outer envelope (11 S3.0). data (the payload) is deliberately NOT a
// member: the writer owns the outer fields, the caller attaches its own data.
struct StampedEnvelope {
  int v;              // envelope version (11 S3.0)
  std::string rid;    // robot id
  int64_t ts;         // wall-clock ms -- cross-machine align / logging only (CLK-C1)
  int64_t mono;       // monotonic ms -- what age / timeout maths use
  std::string boot;   // boot id, first 8 hex (11 S3.0)
  uint64_t seq;       // per-writer, strictly increasing
  std::string src;    // this process's source id
  bool ts_sync;       // copied ClockStatus.sync, forced false if stale / never
};

// One envelope writer per producer process. Not thread-safe: the owning process
// serialises publishing onto one thread (same discipline as the arbiter), so the
// seq counter needs no lock.
class EnvelopeWriter {
 public:
  // rid / src / boot identify this process's envelopes. sync_timeout_ms is
  // CLK-A3's window (5 s); it is injected, never defaulted (trap 3 in the file
  // header / CLAUDE.md 3.1).
  EnvelopeWriter(std::string rid, std::string src, std::string boot,
                 int64_t sync_timeout_ms)
      : rid_(std::move(rid)),
        src_(std::move(src)),
        boot_(std::move(boot)),
        sync_timeout_ms_(sync_timeout_ms),
        seq_(0),
        clock_received_(false),
        last_sync_(false),
        last_clock_mono_(0) {}

  // CLK-A2: record the latest ClockStatus. We COPY sync; we never judge it.
  // mono_ms is the steady_clock reading when it arrived.
  void note_clock_status(bool sync, int64_t mono_ms) {
    clock_received_ = true;         // we have heard from rtk_driver at least once
    last_sync_ = sync;              // copied verbatim -- no local judgement (CLK-A2)
    last_clock_mono_ = mono_ms;     // monotonic time of this ClockStatus
  }

  // Stamp the next envelope. wall_ts_ms goes into ts (align/log only);
  // now_mono_ms is the steady reading used both for mono and for the CLK-A3
  // staleness check. seq increments by one.
  StampedEnvelope stamp(int64_t wall_ts_ms, int64_t now_mono_ms) {
    StampedEnvelope env;
    env.v = kEnvelopeVersion;
    env.rid = rid_;
    env.ts = wall_ts_ms;            // wall clock: cross-machine align only
    env.mono = now_mono_ms;         // monotonic: the field age maths trust
    env.boot = boot_;
    env.seq = ++seq_;               // the ONE seq source for this producer
    env.src = src_;
    env.ts_sync = compute_ts_sync(now_mono_ms);
    return env;
  }

  // Exposed for the test's benefit and for a caller that wants the flag without
  // stamping. Same rule as stamp() uses.
  bool ts_sync_at(int64_t now_mono_ms) const {
    return compute_ts_sync(now_mono_ms);
  }

 private:
  // 11 S3.0 envelope version. Named once so no branch spells the literal.
  static constexpr int kEnvelopeVersion = 1;

  // CLK-A2 + CLK-A3. Never received -> false. Older than the window (measured on
  // the MONOTONIC now, trap 2) -> false. Otherwise the copied sync value.
  bool compute_ts_sync(int64_t now_mono_ms) const {
    if (!clock_received_) {
      return false;                 // trap 1: no optimistic default before first status
    }
    // Strictly-less: >= sync_timeout is stale (CLK-A3 says ">= 5 s -> false").
    const int64_t age_ms = now_mono_ms - last_clock_mono_;   // MONOTONIC age
    if (age_ms >= sync_timeout_ms_) {
      return false;                 // fail-safe: a dead clock is not a synced clock
    }
    return last_sync_;              // fresh: copy the last judged sync (CLK-A2)
  }

  std::string rid_;
  std::string src_;
  std::string boot_;
  int64_t sync_timeout_ms_;         // CLK-A3 window, injected (no default)
  uint64_t seq_;                    // strictly increasing, the one seq source
  bool clock_received_;             // false until the first note_clock_status
  bool last_sync_;                  // the last ClockStatus.sync value (copied)
  int64_t last_clock_mono_;         // monotonic time of the last ClockStatus
};

}  // namespace envelope
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_ENVELOPE_ENVELOPE_WRITER_H_
