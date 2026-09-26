/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: envelope_rebuild.h
 * Brief: RT-C3.e envelope rebuild -- scan the 11 S3.0 outer object, re-stamp
 *        seq/ts/src, keep everything else byte-verbatim
 *
 * Description:
 * RT-C3.e: forwarding must rebuild the S3.0 envelope -- seq from the
 * forwarder's own counter, ts at the forward instant, src renamed to the
 * forwarder, the original values kept in orig_ts / orig_src, and never a
 * verbatim byte relay of the far plane (grep the contract for the anchor
 * "重新封装而非透传"). This header is that sentence as code, and nothing
 * more (CRL-1: the relay interprets no payload).
 *
 * What is rebuilt and what is copied -- and WHY the split is exactly here:
 *   * seq / ts / src are REBUILT: they are the three fields RT-C3.e names.
 *     A fresh per-key seq keeps the two planes' loss statistics from
 *     polluting each other; a fresh ts marks the forward instant; src names
 *     the forwarder. The original ts/src land in orig_ts / orig_src.
 *   * v / rid / mono / boot / ts_sync / data are COPIED BYTE-VERBATIM:
 *     RT-C3.e names none of them, and each has a reason of its own.
 *       - mono/boot: 11 S3.0 makes mono "一切超时与年龄判定的唯一依据", and
 *         both planes live on ONE host, so the copied mono still matches the
 *         consumer's boot id and yields the TRUE production age -- including
 *         time spent queued in this relay. Re-stamping mono would make a
 *         stale message look fresh, which is the fail-silent direction.
 *         A message with no mono (a cross-host publisher MUST omit it,
 *         CLK-C4) is forwarded without one; the consumer falls back to
 *         receive-time age, exactly as S3.0.1 prescribes.
 *       - ts_sync: judgement belongs to rtk_driver alone (CLK-A1/A2), and
 *         the relay does not even subscribe rt/clock/status (not in its
 *         twelve rows), so it COPIES -- same rule P1-13 states for the clock
 *         mirror ("不得改写 sync/source 任何字段").
 *       - data: copied as a raw byte span, never decoded and re-encoded.
 *         Re-encoding would mean parsing arbitrary payloads on a hop with a
 *         200 us budget (CRL-5), and any serializer difference (float
 *         formatting, key order) would corrupt digests downstream.
 *   * unknown top-level fields are DROPPED. S3.0's receiver rule is "未知字段
 *     必须忽略"; a forwarder that copied fields it cannot name would be a
 *     byte tunnel for whatever a sender smuggles at the top level.
 *
 * The scanner is a single forward pass over the top-level object: no
 * recursion, no allocation, no library. Nested values are skipped by brace
 * depth with in-string tracking; it EXTRACTS spans, it does not validate JSON
 * (a mismatched bracket pair deep inside data will pass the scan and fail at
 * the consumer, which is where schema validation lives -- CRL-1). Failure
 * answers are uniform: ScanEnvelope returns false, RebuildEnvelope returns 0,
 * and the caller (relay_core) decides what a failure means per key -- drop
 * for eleven rows, verbatim raw forward for cmd/estop (11 S3.0.1).
 */

#ifndef HACHIST_XBRAIN_V6_CHASSIS_RELAY_ENVELOPE_REBUILD_H_
#define HACHIST_XBRAIN_V6_CHASSIS_RELAY_ENVELOPE_REBUILD_H_

#include <cstddef>
#include <cstdint>

namespace chassis_relay {

// One extracted value: a byte range inside the INPUT buffer. Offsets rather
// than pointers so a scan result can be asserted against fixture strings in
// tests without pointer arithmetic.
struct Span {
  std::size_t off = 0;
  std::size_t len = 0;
  bool present = false;
};

// The nine S3.0 fields, as found (or not) at the top level of one message.
// Duplicate keys: the LAST occurrence wins, matching what json decoders in
// this stack do; the case cannot arise from our own writers.
struct EnvelopeScan {
  Span v;
  Span rid;
  Span ts;
  Span mono;
  Span boot;
  Span seq;
  Span src;
  Span ts_sync;
  Span data;
  // The scanned input's length, recorded so the wrap path (a bare payload,
  // data absent) can embed the WHOLE object without the caller re-supplying
  // len -- a second len parameter is a second chance for the two to disagree.
  std::size_t input_len = 0;
};

// Scan the top-level JSON object in [in, in+len). Returns true when the input
// is one complete object (optionally wrapped in whitespace) whose top level
// could be walked to the closing brace; field spans are filled for the keys
// that were seen. Allocation-free, single pass, bounded by len.
bool ScanEnvelope(const char* in, std::size_t len, EnvelopeScan* out);

// Serialise the rebuilt envelope into [out, out+cap):
//   { v?, rid?, ts = fwd_ts_s, mono?, boot?, seq = fwd_seq, src = fwd_src,
//     ts_sync?, orig_ts = old ts?, orig_src = old src?, data }
// where ? marks fields emitted only when present in the scan.
//
// A scan with NO data field is a BARE payload (the deployed general plane
// carries them; p5_gateway's probe ping is one) and takes the WRAP form
// instead: { v:1, ts, seq, src, data = the whole original object } -- no
// copied fields, no orig_*, no fabricated rid/mono/boot/ts_sync (the
// receivers' fallbacks for those are the fail-safe directions). See the
// WrapBare note in the .cc for the full argument.
//
// Returns bytes written, or 0 when the buffer is too small -- never a
// partial object (half an envelope is valid-looking JSON that decodes to the
// wrong thing). fwd_ts_s is rendered with six decimals, the precision S3.0's
// own example carries. Allocation-free.
std::size_t RebuildEnvelope(const char* in, const EnvelopeScan& scan,
                            double fwd_ts_s, std::uint64_t fwd_seq,
                            const char* fwd_src, char* out, std::size_t cap);

// Worst-case growth of a rebuild over its input: the fixed field names and
// punctuation plus a re-rendered ts/seq, with orig_ts/orig_src duplicating
// the two original spans. Callers size their output buffer as
// input_cap + kRebuildSlack; RebuildEnvelope still checks every write, this
// constant only makes "big enough" writable as one expression.
inline constexpr std::size_t kRebuildSlack = 256;

}  // namespace chassis_relay

#endif  // HACHIST_XBRAIN_V6_CHASSIS_RELAY_ENVELOPE_REBUILD_H_
