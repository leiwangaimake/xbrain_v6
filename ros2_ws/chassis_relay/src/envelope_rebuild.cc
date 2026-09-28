/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: envelope_rebuild.cc
 * Brief: The top-level scan and the bounded re-serialisation (see the header)
 *
 * Description:
 * Two halves, both allocation-free because they run inside zenoh callback
 * threads on the emergency-stop path (CRL-4, CRL-5):
 *
 *   * scan_envelope walks the ONE top level of a JSON object and records the
 *     byte span of each S3.0 field it recognises. Values it does not need are
 *     skipped structurally: strings by escape-aware quote matching, objects
 *     and arrays by a single depth counter with in-string tracking, scalars
 *     by delimiter. The scanner does NOT validate JSON -- 11 CRL-1 forbids
 *     this process from judging payloads, and the consumers already reject
 *     malformed data with their own schema rules. What the scan DOES
 *     guarantee is that the spans it returns lie inside the input and that
 *     the object closed before the input ended, which is all the rebuild
 *     needs to compose a well-formed output.
 *
 *   * rebuild_envelope emits the new envelope through a bounded appender that
 *     latches on overflow, so there is no branch in which a half-written
 *     object escapes (the same shape as quadruped's rt_payloads.cc Appender,
 *     and for the same reason: truncated JSON at a consumer reads as "the
 *     robot went silent", not as a relay bug).
 *
 * Things that look wrong and are not:
 *   * A mismatched '['/'}' pair deep inside data passes the scan. Deliberate:
 *     matching bracket TYPES would buy nothing (the consumer's decoder is the
 *     validator) and cost a type stack on a 200 us budget.
 *   * isspace is not used. It is locale-sensitive and takes int; the four
 *     JSON whitespace bytes are spelled out instead.
 *   * The literal scanner accepts any run of non-delimiter bytes (so "tru3"
 *     scans fine). The rebuild copies it verbatim and the consumer rejects
 *     it -- rejecting here would make the relay a second validator (CRL-1).
 */

#include "chassis_relay/envelope_rebuild.h"

#include <cstdio>
#include <cstring>

namespace chassis_relay {
namespace {

// The four whitespace bytes JSON permits between tokens (RFC 8259 s2).
inline bool is_ws(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

inline void skip_ws(const char* in, std::size_t len, std::size_t* i) {
  while (*i < len && is_ws(in[*i])) ++(*i);
}

// Advance past one JSON string, *i at the opening quote on entry, one past
// the closing quote on exit. Escapes are skipped pairwise -- \" never closes.
bool scan_string(const char* in, std::size_t len, std::size_t* i) {
  ++(*i);  // the opening quote
  while (*i < len) {
    const char c = in[*i];
    if (c == '\\') {
      // An escape consumes the next byte whatever it is; a truncated escape
      // at the end of input falls out of the loop and fails below.
      *i += 2;
      continue;
    }
    ++(*i);
    if (c == '"') return true;
  }
  return false;  // unterminated string: the input ended inside it
}

// Advance past one object or array, *i at the opener on entry. One depth
// counter covers both bracket kinds; in-string state keeps braces inside
// string values from counting. Depth is capped so a hostile input cannot
// turn the scan into an effectively unbounded loop of nested openers.
bool scan_container(const char* in, std::size_t len, std::size_t* i) {
  constexpr int kMaxDepth = 64;
  int depth = 0;
  bool in_string = false;
  while (*i < len) {
    const char c = in[*i];
    if (in_string) {
      if (c == '\\') {
        *i += 2;
        continue;
      }
      if (c == '"') in_string = false;
      ++(*i);
      continue;
    }
    if (c == '"') {
      in_string = true;
    } else if (c == '{' || c == '[') {
      if (++depth > kMaxDepth) return false;
    } else if (c == '}' || c == ']') {
      --depth;
      if (depth == 0) {
        ++(*i);  // include the closer in the span
        return true;
      }
      if (depth < 0) return false;  // closer with no opener
    }
    ++(*i);
  }
  return false;  // input ended inside the container
}

// Advance past one scalar (number / true / false / null / any bare token).
// Stops at the delimiters that can follow a value at the scanned level.
bool scan_literal(const char* in, std::size_t len, std::size_t* i) {
  const std::size_t start = *i;
  while (*i < len) {
    const char c = in[*i];
    if (c == ',' || c == '}' || c == ']' || is_ws(c)) break;
    ++(*i);
  }
  return *i > start;  // an empty literal ("key: ,") is malformed
}

// Advance past one value of any shape, recording its span.
bool scan_value(const char* in, std::size_t len, std::size_t* i, Span* out) {
  const std::size_t start = *i;
  bool ok = false;
  if (*i >= len) return false;
  const char c = in[*i];
  if (c == '"') {
    ok = scan_string(in, len, i);
  } else if (c == '{' || c == '[') {
    ok = scan_container(in, len, i);
  } else {
    ok = scan_literal(in, len, i);
  }
  if (!ok) return false;
  out->off = start;
  out->len = *i - start;
  out->present = true;
  return true;
}

// Does the key span (quotes included) spell exactly `name`? memcmp over the
// interior bytes; an escaped spelling of a known name ("v" for v) will
// not match and the field lands in the ignored set -- our own writers never
// escape these seven-bit names, so nothing real is lost.
bool key_is(const char* in, const Span& key, const char* name) {
  const std::size_t n = std::strlen(name);
  if (key.len != n + 2) return false;  // interior length plus the two quotes
  return std::memcmp(in + key.off + 1, name, n) == 0;
}

// Route one scanned key/value pair into the result struct. Last occurrence
// wins on a duplicate key, matching the json decoders used in this stack.
void assign(const char* in, const Span& key, const Span& val,
            EnvelopeScan* out) {
  if (key_is(in, key, "v")) out->v = val;
  else if (key_is(in, key, "rid")) out->rid = val;
  else if (key_is(in, key, "ts")) out->ts = val;
  else if (key_is(in, key, "mono")) out->mono = val;
  else if (key_is(in, key, "boot")) out->boot = val;
  else if (key_is(in, key, "seq")) out->seq = val;
  else if (key_is(in, key, "src")) out->src = val;
  else if (key_is(in, key, "ts_sync")) out->ts_sync = val;
  else if (key_is(in, key, "data")) out->data = val;
  // Every other key is dropped by the rebuild -- see the header on why a
  // forwarder must not copy fields it cannot name.
}

// Bounded appender: latches overflowed and stays overflowed, so the caller
// writes the whole object and checks once (a per-write check someone forgets
// once is a truncation).
struct Out {
  char* p;
  std::size_t cap;
  std::size_t n = 0;
  bool overflow = false;

  void add_bytes(const char* s, std::size_t k) {
    if (overflow) return;
    if (n + k > cap) {
      overflow = true;
      return;
    }
    std::memcpy(p + n, s, k);
    n += k;
  }
  void add_lit(const char* s) { add_bytes(s, std::strlen(s)); }
  void add_span_of(const char* in, const Span& s) { add_bytes(in + s.off, s.len); }
};

// The key/value walk of the top-level object, *i one past the opening brace
// on entry (and NOT at a closing one), one past the closing brace on success.
bool walk_pairs(const char* in, std::size_t len, std::size_t* i,
               EnvelopeScan* out) {
  while (true) {
    if (*i >= len || in[*i] != '"') return false;
    Span key{*i, 0, false};
    if (!scan_string(in, len, i)) return false;
    key.len = *i - key.off;
    skip_ws(in, len, i);
    if (*i >= len || in[*i] != ':') return false;
    ++(*i);
    skip_ws(in, len, i);
    Span val;
    if (!scan_value(in, len, i, &val)) return false;
    assign(in, key, val, out);
    skip_ws(in, len, i);
    if (*i >= len) return false;  // object never closed
    if (in[*i] == ',') {
      ++(*i);
      skip_ws(in, len, i);
      continue;
    }
    if (in[*i] == '}') {
      ++(*i);
      return true;
    }
    return false;  // something other than , or } after a value
  }
}

}  // namespace

bool scan_envelope(const char* in, std::size_t len, EnvelopeScan* out) {
  if (in == nullptr || out == nullptr || len == 0) return false;
  *out = EnvelopeScan{};
  out->input_len = len;
  std::size_t i = 0;
  skip_ws(in, len, &i);
  if (i >= len || in[i] != '{') return false;
  ++i;
  skip_ws(in, len, &i);
  // The empty object closes immediately; otherwise walk key/value pairs.
  if (i < len && in[i] == '}') {
    ++i;
  } else if (!walk_pairs(in, len, &i, out)) {
    return false;
  }
  // Strictly one object: trailing bytes other than whitespace mean this was
  // not the message it claims to be, and a rebuild from a prefix of garbage
  // would launder that garbage into a well-formed envelope.
  skip_ws(in, len, &i);
  return i == len;
}

namespace {

// One optionally-copied field: emitted only when the scan saw it, with the
// comma bookkeeping shared through `first`.
void emit_copied(Out* o, bool* first, const char* in, const char* name,
                const Span& s) {
  if (!s.present) return;
  if (!*first) o->add_lit(",");
  *first = false;
  o->add_lit("\"");
  o->add_lit(name);
  o->add_lit("\":");
  o->add_span_of(in, s);
}

// The three REBUILT fields, in S3.0 order relative to each other:
// ts = the forward instant (wall clock, six decimals -- the precision S3.0's
// own example carries; %g would coarsen it to whole seconds), then seq = the
// relay's own per-key counter (RT-C3.e: gap detection on the destination
// plane must measure THIS hop, not the far one), then src = the forwarder.
void emit_rebuilt_ts(Out* o, bool* first, double fwd_ts_s) {
  char num[48];
  std::snprintf(num, sizeof(num), "%s\"ts\":%.6f", *first ? "" : ",",
                fwd_ts_s);
  *first = false;
  o->add_lit(num);
}

void emit_rebuilt_seq_src(Out* o, std::uint64_t fwd_seq, const char* fwd_src) {
  char num[48];
  std::snprintf(num, sizeof(num), ",\"seq\":%llu",
                static_cast<unsigned long long>(fwd_seq));
  o->add_lit(num);
  o->add_lit(",\"src\":\"");
  o->add_lit(fwd_src);
  o->add_lit("\"");
}

}  // namespace

// The wrap half of the rebuild: an input with NO data field is a BARE
// payload (the deployed general plane carries them -- p5_gateway's 1 Hz
// probe/estop/ping was `{"seq":N,"t_mono_ms":M,"type":"ping"}`, no envelope
// at all), and the S3.0-faithful forward is to AUTHOR a fresh envelope with
// the whole original object as data.
//
// *** 2026-09-27: p5_gateway now sends a full S3.0 envelope on that key, so
// THIS PATH IS NOW PURELY DEFENSIVE -- it is no longer the one the estop
// probe takes. Kept, and kept exactly as it is, because CRL-1 is what it
// implements: a bare payload is still a payload, and refusing to carry one
// would make the relay decide what a message MEANS. Worth knowing why the
// probe stopped using it: the wrap path cannot write `rid` (it has no
// original envelope to copy one from, see below), and quadruped's envelope
// reader rejects a ping whose rid is absent -- so a bare ping was answered
// with seq 0 forever. 11 S8.5 PRB-1..3 and the ruling there fixed the seq
// half; p5 sending the envelope fixes this half.
//
// What is deliberately NOT written:
//   * rid / mono / boot / ts_sync / orig_* -- there is no original envelope
//     to copy them from, and fabricating provenance or a production time
//     would let a message that sat somewhere look fresh (receivers fall
//     back to receive-time age and to ts_sync=false, both the fail-safe
//     directions of S3.0/S3.0.1);
//   * any interpretation of the object -- an input that has SOME envelope
//     fields but no data ({"v":9,"ts":..}) wraps the same way; deciding it
//     was "a broken envelope" rather than "a payload" would be the payload
//     judgement CRL-1 forbids, and the consumer's schema rejects it either
//     way.
// v IS written: it versions THIS envelope, which the relay is the author of.
static std::size_t wrap_bare(const char* in, std::size_t len, double fwd_ts_s,
                            std::uint64_t fwd_seq, const char* fwd_src,
                            char* out, std::size_t cap) {
  Out o{out, cap};
  char num[48];
  std::snprintf(num, sizeof(num), "{\"v\":1,\"ts\":%.6f", fwd_ts_s);
  o.add_lit(num);
  emit_rebuilt_seq_src(&o, fwd_seq, fwd_src);
  o.add_lit(",\"data\":");
  // The whole input, verbatim -- scan_envelope already proved it is exactly
  // one object in optional whitespace, and surrounding whitespace is legal
  // inside a JSON value position.
  o.add_bytes(in, len);
  o.add_lit("}");
  if (o.overflow) return 0;
  return o.n;
}

std::size_t rebuild_envelope(const char* in, const EnvelopeScan& scan,
                            double fwd_ts_s, std::uint64_t fwd_seq,
                            const char* fwd_src, char* out, std::size_t cap) {
  if (in == nullptr || fwd_src == nullptr || out == nullptr || cap == 0) {
    return 0;
  }
  // No data field = a bare payload: wrap it whole (see wrap_bare above).
  // scan.data carries no offsets to recover the input length from, so the
  // caller-visible contract stays "the scan plus the same in/len".
  if (!scan.data.present) {
    return wrap_bare(in, scan.input_len, fwd_ts_s, fwd_seq, fwd_src, out, cap);
  }

  Out o{out, cap};
  o.add_lit("{");
  // Copied fields go out only when they came in; the emit order follows the
  // S3.0 listing so a human diffing a capture against the contract reads top
  // to bottom (same argument as quadruped's write_envelope).
  bool first = true;
  emit_copied(&o, &first, in, "v", scan.v);
  emit_copied(&o, &first, in, "rid", scan.rid);
  emit_rebuilt_ts(&o, &first, fwd_ts_s);
  emit_copied(&o, &first, in, "mono", scan.mono);
  emit_copied(&o, &first, in, "boot", scan.boot);
  emit_rebuilt_seq_src(&o, fwd_seq, fwd_src);
  emit_copied(&o, &first, in, "ts_sync", scan.ts_sync);
  // The originals, kept as RT-C3.e requires. Spans verbatim: orig_src keeps
  // its quotes, orig_ts its number formatting.
  emit_copied(&o, &first, in, "orig_ts", scan.ts);
  emit_copied(&o, &first, in, "orig_src", scan.src);
  o.add_lit(",\"data\":");
  o.add_span_of(in, scan.data);
  o.add_lit("}");
  if (o.overflow) return 0;
  return o.n;
}

}  // namespace chassis_relay
