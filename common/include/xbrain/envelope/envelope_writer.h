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

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

namespace hachist {
namespace xbrain {
namespace envelope {

// The stamped outer envelope (11 S3.0). data (the payload) is deliberately NOT a
// member: the writer owns the outer fields, the caller attaches its own data.
struct StampedEnvelope {
  int v;              // envelope version (11 S3.0)
  std::string rid;    // robot id
  // *** SECONDS, float64, both of them. 11 S3.0 states ts as "Unix 秒 UTC" and
  // mono as "CLOCK_MONOTONIC 读数, 单位秒", and the Qt-facing spec
  // (docs/MISSON/任务枚举_qt端v2.0.md S1) freezes ts as a float64 second count
  // and forbids millisecond integers outright.
  //
  // These were int64 MILLISECONDS until 2026-09-15, and nothing caught it: the
  // C++ test only ever asserted ts_sync semantics, so the unit was an untested
  // assumption. The Python side was corrected on 2026-09-13 (commit 5ea2dc0),
  // leaving the two languages a factor of a thousand apart on `mono` -- which
  // 11 S3.0 calls the ONLY basis for every timeout and age judgement.
  double ts;          // wall clock, Unix seconds -- align / record / latency ONLY
  double mono;        // CLOCK_MONOTONIC seconds -- what every age computation uses (CLK-C1)
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
  // rid / src / boot identify this process's envelopes. sync_timeout_s is
  // CLK-A3's window (5 s); it is injected, never defaulted (trap 3 in the file
  // header / CLAUDE.md 3.1). SECONDS, like every other time in this class --
  // a class that took a window in milliseconds and stamps in seconds is the
  // mixed-unit trap this whole correction exists to remove.
  EnvelopeWriter(std::string rid, std::string src, std::string boot,
                 double sync_timeout_s)
      : rid_(std::move(rid)),
        src_(std::move(src)),
        boot_(std::move(boot)),
        sync_timeout_s_(sync_timeout_s),
        seq_(0),
        clock_received_(false),
        last_sync_(false),
        last_clock_mono_s_(0.0) {}

  // CLK-A2: record the latest ClockStatus. We COPY sync; we never judge it.
  // mono_ms is the steady_clock reading when it arrived.
  void note_clock_status(bool sync, double mono_s) {
    clock_received_ = true;         // we have heard from rtk_driver at least once
    last_sync_ = sync;              // copied verbatim -- no local judgement (CLK-A2)
    last_clock_mono_s_ = mono_s;    // monotonic time of this ClockStatus
  }

  // Stamp the next envelope. wall_ts_s goes into ts (align/log only);
  // now_mono_s is the steady reading used both for mono and for the CLK-A3
  // staleness check. Both in SECONDS. seq increments by one.
  StampedEnvelope stamp(double wall_ts_s, double now_mono_s) {
    StampedEnvelope env;
    env.v = kEnvelopeVersion;
    env.rid = rid_;
    env.ts = wall_ts_s;             // wall clock: cross-machine align only
    env.mono = now_mono_s;          // monotonic: the field age maths trust
    env.boot = boot_;
    env.seq = ++seq_;               // the ONE seq source for this producer
    env.src = src_;
    env.ts_sync = compute_ts_sync(now_mono_s);
    return env;
  }

  // Exposed for the test's benefit and for a caller that wants the flag without
  // stamping. Same rule as stamp() uses.
  bool ts_sync_at(double now_mono_s) const {
    return compute_ts_sync(now_mono_s);
  }

 private:
  // 11 S3.0 envelope version. Named once so no branch spells the literal.
  static constexpr int kEnvelopeVersion = 1;

  // CLK-A2 + CLK-A3. Never received -> false. Older than the window (measured on
  // the MONOTONIC now, trap 2) -> false. Otherwise the copied sync value.
  bool compute_ts_sync(double now_mono_s) const {
    if (!clock_received_) {
      return false;                 // trap 1: no optimistic default before first status
    }
    // Strictly-less: >= sync_timeout is stale (CLK-A3 says ">= 5 s -> false").
    const double age_s = now_mono_s - last_clock_mono_s_;    // MONOTONIC age
    if (age_s >= sync_timeout_s_) {
      return false;                 // fail-safe: a dead clock is not a synced clock
    }
    return last_sync_;              // fresh: copy the last judged sync (CLK-A2)
  }

  std::string rid_;
  std::string src_;
  std::string boot_;
  double sync_timeout_s_;         // CLK-A3 window, injected (no default)
  uint64_t seq_;                    // strictly increasing, the one seq source
  bool clock_received_;             // false until the first note_clock_status
  bool last_sync_;                  // the last ClockStatus.sync value (copied)
  double last_clock_mono_s_;         // monotonic time of the last ClockStatus
};

// Serialise the outer envelope around an already-rendered data object, into a
// caller-supplied buffer. Returns the number of bytes written, or 0 when the
// buffer is too small -- never a partial object, because half an envelope is
// valid-looking JSON that decodes to the wrong thing.
//
// Why this lives HERE rather than in each publisher. PB-Q3 requires the eight
// envelope fields to be written in exactly one place per process; the same
// argument applies across processes, because the fields that drift are the ones
// each publisher spells for itself. rtk_driver had its own copy, and that copy
// is where the millisecond/second divergence lived for two days without any
// test noticing.
//
// Allocation-free, so a non-realtime publisher can call it from a thread with a
// fixed buffer (13 QD-7 keeps allocation off ctrl; this is usable from either).
//
// ts and mono are rendered with six decimals, matching the precision 11 S3.0's
// own example shows (1753660800.123456). snprintf with an explicit format
// rather than std::to_string: the latter is locale-sensitive in principle, and
// a decimal comma would produce JSON that no decoder accepts.
inline std::size_t WriteEnvelopeJson(const StampedEnvelope& e,
                                     const char* data_json, char* out,
                                     std::size_t cap) {
  if (out == nullptr || data_json == nullptr || cap == 0) return 0;
  const int n = std::snprintf(
      out, cap,
      "{\"v\":%d,\"rid\":\"%s\",\"ts\":%.6f,\"mono\":%.6f,"
      "\"boot\":\"%s\",\"seq\":%llu,\"src\":\"%s\","
      "\"ts_sync\":%s,\"data\":%s}",
      e.v, e.rid.c_str(), e.ts, e.mono, e.boot.c_str(),
      static_cast<unsigned long long>(e.seq), e.src.c_str(),
      e.ts_sync ? "true" : "false", data_json);
  // snprintf returns what it WOULD have written. A value that reaches the end
  // of the buffer means the object was truncated, and truncated JSON is a parse
  // error at the far end -- reported here as 0 rather than sent.
  if (n < 0 || static_cast<std::size_t>(n) >= cap) return 0;
  return static_cast<std::size_t>(n);
}

}  // namespace envelope
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_ENVELOPE_ENVELOPE_WRITER_H_
