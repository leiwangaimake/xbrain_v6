/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: relay_core.cc
 * Brief: OnSample -- one bounded rebuild-and-put per inbound frame
 *
 * Description:
 * The body of the forward step the header argues for. Two properties carry
 * all the weight and both are structural rather than checked:
 *
 *   * No heap. The rebuild target is one stack array sized
 *     kMaxForwardBytes + kRebuildSlack; every helper it calls is
 *     allocation-free by construction (envelope_rebuild.cc), and the sink is
 *     a std::function bound once at startup, whose CALL does not allocate.
 *     CRL-4 is thereby a property of the code shape, not of a code path
 *     being lucky.
 *
 *   * At most one sink call per sample. The estop fallback REPLACES the
 *     rebuilt put, it never doubles it -- two puts for one frame would make
 *     the destination's dedup window (11 S9.12.6) see a burst the sender
 *     never sent.
 *
 * Order of checks: index, size, then rebuild. Size before rebuild so an
 * oversized frame is refused before any work is spent on it, and so the
 * refusal is counted under its own name rather than as "malformed" -- the
 * operator reading the stats line needs to know which of the two happened,
 * because one means a broken producer and the other means a producer that
 * outgrew this relay's buffer (a deployment decision, not a bug).
 */

#include "chassis_relay/relay_core.h"

#include "chassis_relay/envelope_rebuild.h"

namespace chassis_relay {

namespace {
// The src every rebuilt envelope carries (RT-C3.e: the forwarder's process
// name). One spelling, used by the rebuild call below and asserted verbatim
// by the tests; the contract writes it in CRL-2.
constexpr const char* kSrcName = "chassis_relay";
}  // namespace

RelayCore::RelayCore(PutFn put) : put_(std::move(put)) {}

ForwardOutcome RelayCore::OnSample(std::size_t index, const char* bytes,
                                   std::size_t len,
                                   double wall_ts_s) noexcept {
  // A wiring bug, not traffic: no counter row exists for it, so it is the one
  // outcome reported before rx is counted.
  if (index >= kRelayCount || bytes == nullptr || put_ == nullptr) {
    return ForwardOutcome::kBadIndex;
  }
  RelayRowStats& st = stats_[index];
  st.rx.fetch_add(1, std::memory_order_relaxed);

  // The size boundary, estop included -- see the header's note on why a
  // bounded, counted refusal beats an unbounded allocation on this path.
  if (len > kMaxForwardBytes || len == 0) {
    st.dropped_oversize.fetch_add(1, std::memory_order_relaxed);
    return ForwardOutcome::kDroppedOversize;
  }

  // Rebuild into the stack. The seq is claimed BEFORE the put so two threads
  // can never stamp the same value; a failed put therefore leaves a gap,
  // which is the honest reading (the message existed and was lost).
  EnvelopeScan scan;
  const bool scanned = ScanEnvelope(bytes, len, &scan);
  char out[kMaxForwardBytes + kRebuildSlack];
  std::size_t out_len = 0;
  if (scanned) {
    const std::uint64_t seq =
        st.seq.fetch_add(1, std::memory_order_relaxed) + 1;
    out_len = RebuildEnvelope(bytes, scan, wall_ts_s, seq, kSrcName, out,
                              sizeof(out));
  }

  if (out_len == 0) return OnRebuildFailed(index, bytes, len);

  if (!put_(index, out, out_len)) {
    st.put_failed.fetch_add(1, std::memory_order_relaxed);
    return ForwardOutcome::kPutFailed;
  }
  st.forwarded.fetch_add(1, std::memory_order_relaxed);
  return ForwardOutcome::kForwarded;
}

ForwardOutcome RelayCore::OnRebuildFailed(std::size_t index,
                                          const char* bytes,
                                          std::size_t len) noexcept {
  RelayRowStats& st = stats_[index];
  // Rebuild impossible. For eleven rows that is a drop; for cmd/estop the
  // original bytes go through verbatim -- 11 S3.0.1's exemption, held to
  // exactly the one key whose every misreading collapses to "stop". The
  // relaxing command cmd/chassis/ctrl is the row this gate protects hardest:
  // raw-forwarding a malformed "enable" would be a payload tunnel into the
  // RT plane on the one action that unlocks the machine.
  if (!kRelayTable[index].estop_exempt) {
    st.dropped_malformed.fetch_add(1, std::memory_order_relaxed);
    return ForwardOutcome::kDroppedMalformed;
  }
  if (!put_(index, bytes, len)) {
    st.put_failed.fetch_add(1, std::memory_order_relaxed);
    return ForwardOutcome::kPutFailed;
  }
  st.forwarded_raw.fetch_add(1, std::memory_order_relaxed);
  return ForwardOutcome::kForwardedRaw;
}

std::uint64_t RelayCore::total_rx() const {
  std::uint64_t sum = 0;
  for (std::size_t i = 0; i < kRelayCount; ++i) {
    sum += stats_[i].rx.load(std::memory_order_relaxed);
  }
  return sum;
}

}  // namespace chassis_relay
