/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: relay_core.h
 * Brief: The per-key forward step: rebuild, publish through an injected sink,
 *        count. No zenoh, no threads -- the testable half of the process
 *
 * Description:
 * One call per inbound sample: OnSample(row, bytes, len, wall_ts). It rebuilds
 * the envelope (RT-C3.e, envelope_rebuild.h), hands the result to the sink the
 * process wired at startup, and keeps per-row counters. The zenoh sessions,
 * the threads and the QoS options all live OUTSIDE this class, which is what
 * lets test_relay_core.cc drive every branch with bytes and a capturing sink
 * -- the same seam quadruped's rt_bridge test uses.
 *
 * The one behavioural fork, and its exact boundary (11 S3.0.1):
 *   * cmd/estop (the single estop_exempt row): a frame that cannot be
 *     re-enveloped -- scan failure, missing data, rebuild overflow -- is
 *     forwarded VERBATIM and counted as raw. "停" may misfire, "放行" must
 *     not (the S3.0.1 one-liner): the collapse
 *     direction of every misreading is the safe one, so formatting must never
 *     block it. This is the relay-side twin of quadruped's rule that every
 *     malformed estop still stops.
 *   * every other row DROPS a frame that cannot be re-enveloped, and counts
 *     why. Raw-forwarding there would reopen the "构造 payload 直穿对面" path
 *     RT-C3.e closes -- worst on cmd/chassis/ctrl, whose "enable" is the one
 *     action that makes the machine MORE able to move (放松型: a malformed
 *     frame is rejected, never guessed at).
 *
 * Threading and CRL-6, argued once here because this class is where the hot
 * path lives (main.cc wires it, relay_session.cc carries the same argument in
 * short form):
 *   * OnSample is called INLINE from zenoh subscriber callbacks. There is no
 *     application queue anywhere on a forward: one hop = one callback = one
 *     rebuild on the stack = one put. The general-plane session subscribes
 *     ONLY the three Q0 command keys (CR-1/2/11), so the threads that carry
 *     the stop direction carry nothing else -- no Q3, no state batch
 *     (anti-pattern A-1 and CRL-6 both satisfied by construction).
 *   * The RT session's callbacks forward Q0 acks/pong and Q2 states with
 *     congestion "drop" puts, which cannot block. The ONE Q3 publish
 *     (CR-9 -> event/fault/chassis, "block" per A-4) is NOT executed on a
 *     callback thread: main.cc's sink for that row writes a lock-free
 *     latest-value slot and a dedicated event thread does the blocking put.
 *     So no thread that handles Q0 traffic ever executes a blocking Q3 put --
 *     the property CRL-6's "该线程不得承载任何 Q3 流量" protects. CRL-6's
 *     "同一线程内处理" is read as "no handoff, no shared queue with lower
 *     classes": with two independent sessions the GEN->RT and RT->GEN halves
 *     necessarily run on their own runtimes' threads, and what the clause
 *     defends (Q0 never waiting behind Q3) holds exactly.
 *   * Latest-value semantics on CR-9 can coalesce a fault burst. Deliberate:
 *     rt/chassis/fault carries the FULL current fault list every time
 *     (2 Hz + on change), so the newest frame supersedes the missed one
 *     within 500 ms; the drop is visible as rx > forwarded in the stats line.
 *
 * Size cap: kMaxForwardBytes bounds the stack buffers this class and the
 * session layer use (CRL-4 forbids heap on the safety path, so the buffers
 * are stack arrays and must have a fixed size). A larger frame is dropped
 * AND COUNTED -- including on cmd/estop, where the boundary deserves its
 * note: a real estop is ~200 bytes, 11 S9.12.6 has the sender repeat it at
 * 10 Hz for up to 1 s, and quadruped's own decoder would reject a 64 KiB
 * blob anyway; accepting unbounded input on the no-allocation path would
 * trade a bounded, counted refusal for an unbounded allocation.
 */

#ifndef HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_CORE_H_
#define HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_CORE_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "chassis_relay/relay_keys.h"

namespace chassis_relay {

// Upper bound on one forwarded message, input side. The largest real frame on
// these twelve keys is a RobotState with a long fault list -- single-digit
// KiB; 64 KiB is an order of magnitude of headroom, not a tuned value.
inline constexpr std::size_t kMaxForwardBytes = 64 * 1024;

// What one OnSample call did, for tests and for the audit trail. The numeric
// values are stable only within a build; nothing serialises them.
enum class ForwardOutcome {
  kForwarded,         // rebuilt and handed to the sink, sink accepted
  kForwardedRaw,      // estop only: rebuild impossible, original bytes sent
  kDroppedMalformed,  // scan/rebuild failed on a non-exempt row
  kDroppedOversize,   // len > kMaxForwardBytes (all rows, estop included)
  kPutFailed,         // sink returned false (transport refused the bytes)
  kBadIndex,          // row index outside the table -- a wiring bug
};

// Per-row counters. Atomics because zenoh may run callbacks for different
// subscribers on different threads; relaxed order is enough for counters that
// only ever feed logs and tests.
struct RelayRowStats {
  std::atomic<std::uint64_t> rx{0};             // samples entering OnSample
  std::atomic<std::uint64_t> forwarded{0};      // rebuilt forwards
  std::atomic<std::uint64_t> forwarded_raw{0};  // estop verbatim forwards
  std::atomic<std::uint64_t> dropped_malformed{0};
  std::atomic<std::uint64_t> dropped_oversize{0};
  std::atomic<std::uint64_t> put_failed{0};
  std::atomic<std::uint64_t> seq{0};            // last seq stamped on this row
};

class RelayCore {
 public:
  // The sink publishes `len` bytes for table row `index` on that row's
  // OUTPUT plane. Returning false means the transport refused; the core
  // counts it and does not retry (a retry loop inside a callback is a stall).
  // Wired once at startup; calls through a bound std::function do not
  // allocate.
  using PutFn = std::function<bool(std::size_t index, const char* bytes,
                                   std::size_t len)>;

  explicit RelayCore(PutFn put);

  // Forward one inbound sample for table row `index`. Called inline from
  // subscriber callbacks; allocation-free (one stack buffer); noexcept
  // because an exception escaping into the zenoh runtime aborts the process.
  ForwardOutcome OnSample(std::size_t index, const char* bytes,
                          std::size_t len, double wall_ts_s) noexcept;

  // Counters for row `index`; used by the stats line and by every test.
  const RelayRowStats& stats(std::size_t index) const {
    return stats_[index];
  }

  // Sum of rx over all rows -- the cheapest liveness figure for the 60 s line.
  std::uint64_t total_rx() const;

 private:
  // The rebuild-failed fork: drop (eleven rows) or verbatim raw forward
  // (cmd/estop alone). Split out of OnSample for the 40-line function rule
  // and so the exemption boundary reads as one block.
  ForwardOutcome OnRebuildFailed(std::size_t index, const char* bytes,
                                 std::size_t len) noexcept;

  PutFn put_;
  RelayRowStats stats_[kRelayCount];
};

}  // namespace chassis_relay

#endif  // HACHIST_XBRAIN_V6_CHASSIS_RELAY_RELAY_CORE_H_
