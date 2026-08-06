/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: message_age.h
 * Brief: C++17 side of message_age_s (11 S3.0.1), byte-equal to the Python impl
 *
 * Description:
 * What this is for. 11 S3.0.1 defines ONE age computation for the whole system,
 * and INF-CM-2 criterion four requires the Python implementation
 * (xbrain/common/envelope/age.py) and this C++ one to produce the same age for
 * the same envelope. This header is the C++ half;
 * tests/common/envelope/test_age_cross_language.py compiles it and compares the
 * two against one shared set of golden vectors -- the same measured-not-asserted
 * pattern common/include/xbrain/digest/canonical_digest.h follows.
 *
 * Why header-only. Its RT-plane consumers (p1_motion is Python, but chassis_relay
 * and quadruped compute ages in C++) sit on the emergency-stop and 20 Hz paths,
 * and CLAUDE.md 5.3 forbids anything in common/ from pulling in rclcpp or any ROS
 * type so those processes can use it. A header with no link step is the cheapest
 * way to keep that true. Nothing here is outside the C++17 standard library: no
 * C++20 features per CPP-1, and no distribution-detection macros per PB-5,
 * because the humble/jazzy baseline D-45 is settled on Humble (99 U74) but this
 * header must not encode a platform assumption regardless.
 *
 * The four branches, identical to age.py and to the S3.0.1 pseudocode:
 *   1. has_mono AND boot == local_boot_id  -> now_mono - mono  (produced age)
 *   2. boot present but != local_boot_id    -> now_mono - rx_mono (fallback)
 *   3. mono absent (cloud, CLK-C4)          -> now_mono - rx_mono (fallback)
 *   4. age < 0                              -> was_negative = true, age clamped 0
 *
 * The one trap worth stating up front. The produced-age branch subtracts `mono`,
 * the MONOTONIC field, never `ts`. ts is the wall clock (S3.0), and a subtraction
 * against it goes wrong by seconds at the exact moment RTK first locks and steps
 * the clock. There is no `ts` parameter on ComputeAge at all, so that mistake
 * cannot be made on this side -- the Python side, where the whole Envelope is in
 * scope, is where INF-CM-2 mutation one lives.
 *
 * This header does NOT emit the CLK-C5 negative-age event. Event emission is a
 * side effect that belongs in one place, and that place is the Python caller with
 * an injected sink (age.py). This header reports was_negative so a C++ caller can
 * emit the event through its own event path; it does not invent an event bus.
 * And it does NOT decode JSON: like the digest header, it takes already-extracted
 * scalar fields, so the only code under cross-language test is the arithmetic and
 * the boot comparison -- a JSON parser here could mask or invent a disagreement.
 */

#ifndef HACHIST_XBRAIN_V6_COMMON_ENVELOPE_MESSAGE_AGE_H_
#define HACHIST_XBRAIN_V6_COMMON_ENVELOPE_MESSAGE_AGE_H_

#include <cstdio>
#include <string>

namespace hachist {
namespace xbrain {
namespace envelope {

// The two branch labels. They must be byte-identical to age.py's
// BRANCH_PRODUCED / BRANCH_RX_FALLBACK, because the cross-language harness prints
// and compares them: a disagreement on the branch is diagnosed separately from a
// disagreement on the number, exactly as the digest harness separates the
// canonical string from the hash. constexpr char arrays rather than std::string
// so they carry no static-init cost on a hot path.
constexpr const char* kBranchProduced = "produced";
constexpr const char* kBranchRxFallback = "rx_fallback";

// The outcome of the S3.0.1 computation, mirroring age.py's AgeResult field for
// field: the clamped age a timeout compares against, the raw age the event would
// carry, the branch taken, and whether the CLK-C5 negative path applied.
struct AgeResult {
  double age_s;         // clamped to 0 when negative; what a timeout uses
  double raw_age_s;     // pre-clamp value; what a negative-age event reports
  const char* branch;   // kBranchProduced or kBranchRxFallback
  bool was_negative;    // true iff raw_age_s < 0
};

// ComputeAge -- the pure S3.0.1 branch/clamp, the C++ twin of age.py compute_age.
//
// has_mono models the Optional[float] mono field: false is the cloud case
// (CLK-C4 requires cross-host publishers to omit mono), and when it is false the
// value of `mono` is not read. boot is compared to local_boot_id ONLY when
// has_mono is true; a mono from another boot is not comparable to this host's
// now_mono (CLK-C4), so a mismatch drops to the receive-time fallback.
//
// rx_mono and now_mono are both this host's CLOCK_MONOTONIC in this boot, passed
// in by the caller -- this header reads no clock itself, for the same reason
// age.py does not: an age must be a pure function of its inputs to be comparable
// across languages, and reading a clock here would also be a place a wall clock
// could slip in against S3.0.1.
inline AgeResult ComputeAge(bool has_mono, double mono,
                            const std::string& boot, double rx_mono,
                            double now_mono, const std::string& local_boot_id) {
  double raw_age;
  const char* branch;
  // Branch 1 requires BOTH a monotonic reading AND that it belongs to this boot.
  // Dropping the boot half (using mono whenever has_mono) is INF-CM-2 mutation
  // three on the Python side; the condition is spelled the same here so the two
  // implementations cannot diverge on it.
  if (has_mono && boot == local_boot_id) {
    raw_age = now_mono - mono;
    branch = kBranchProduced;
  } else {
    // Branches 2 and 3 both land here: boot mismatch, or mono absent. The
    // difference of two readings from the same clock and boot is always
    // comparable, which is the whole reason the fallback exists.
    raw_age = now_mono - rx_mono;
    branch = kBranchRxFallback;
  }
  // CLK-C5: a negative age is clamped to 0 and flagged. The raw value is kept so
  // a caller emitting the event reports how far negative it went.
  if (raw_age < 0.0) {
    return AgeResult{0.0, raw_age, branch, true};
  }
  return AgeResult{raw_age, raw_age, branch, false};
}

// FormatAge -- render a double the way the cross-language harness compares it.
//
// "%.17g" is the shortest format that round-trips every IEEE 754 double, and it
// is the SAME rule canonical.py's format_number uses, so the Python side ("%.17g"
// % v) and this side print byte-identical text for identical bits. That is what
// lets the harness assert string equality on the age rather than a tolerance:
// the two languages are printing the same bits under the same rule. %g already
// strips trailing zeros (C99 7.21.6.1), so no extra stripping is needed to match
// Python's %.17g.
inline std::string FormatAge(double v) {
  char buf[64];
  // snprintf, not std::to_string: to_string is fixed at 6 decimals and locale
  // sensitive, either of which would break the byte-for-byte match. The buffer is
  // far larger than the longest %.17g rendering of a double (~24 chars).
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return std::string(buf);
}

}  // namespace envelope
}  // namespace xbrain
}  // namespace hachist

#endif  // HACHIST_XBRAIN_V6_COMMON_ENVELOPE_MESSAGE_AGE_H_
