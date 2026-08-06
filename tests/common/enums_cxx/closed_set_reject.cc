/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: wanglei@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: closed_set_reject.cc
 * Brief: Probe of the C++ Contains/IndexOf rejection semantics for CFG-CM-5
 *
 * Description:
 * What this establishes. CFG-CM-5 requires the closed-set validator to reject an
 * off-contract value on BOTH language sides -- Python raises ClosedSetViolation,
 * and the deployed header answers the same question without throwing, because its
 * nearest consumer chassis_relay runs under CRL-4 (no dynamic allocation, and a
 * throw allocates). So common/include/xbrain/enums/closed_sets.h answers with a
 * bool from Contains() and a sentinel kNotAMember from IndexOf(), and a caller
 * MUST branch on that answer rather than use a substituted nearby value. That
 * rejection contract is a property of runtime behaviour, so reading the header is
 * not evidence it holds; this program exercises it and prints one key=value line
 * per fact for tests/common/test_closed_set_reject.py to assert on.
 *
 * The two forbidden repairs this pins, both named by 11 S13.6:
 *   * silent pass-through -- Contains() answering true for a value not in the
 *     set. contains_rejects_foo catches it.
 *   * degrade-to-something-close -- IndexOf() answering a real position for a
 *     miss. Reported as 0 it would read as the STRONGEST member: gate_limiter
 *     holds estop at position 0, so a miss read as 0 becomes the highest-priority
 *     limiter. indexof_rejects_foo and miss_not_read_as_strongest catch it, and
 *     the second is why kNotAMember is deliberately not zero.
 *
 * Why the positive facts are here too. A Contains() that always returned false,
 * or an IndexOf() that always returned kNotAMember, would satisfy every rejection
 * fact above while rejecting legal values as well -- the empty-shell pass
 * CLAUDE.md 3.2 form 1 warns about. contains_accepts_system and indexof_finds_estop
 * are the guard against it, and they double as the control for the mutation runs:
 * when the Python driver compiles this same source against a header mutated only
 * in the miss branch, the positive facts stay 1, so a red rejection fact is the
 * mutation and not a broken build.
 *
 * What this program does NOT establish. Nothing about the VALUES in the sets --
 * that they are the right members is held by the symmetric-difference cases in
 * test_closed_sets.py against the design volumes, and repeating them here would
 * create a second place to update. Nothing about the Python side either: that is
 * the parse_enum assertions in the Python driver. And nothing about whole-key
 * legality -- pairing a plane name with a domain name is the transport layer's
 * to judge, not this header's.
 *
 * Output contract. One key=value line per fact, every value an integer 0 or 1
 * (or a position), parsed by the Python driver. Assertions live on the Python
 * side so a failure prints expected against actual and the thresholds sit in one
 * reviewable place, matching tests/common/rtcomm_cxx.
 */

#include "xbrain/enums/closed_sets.h"

#include <cstdio>

// A short alias for the deployed namespace. This is a translation unit, not a
// header, and the name aliased is our own, so CLAUDE.md 5.1 (no using-directive
// for std in a header) does not reach it; the alias just keeps the fact lines
// below readable.
namespace e = hachist::xbrain::enums;

int main() {
  // An off-contract value used against two sets. "foo" is in neither
  // event_category nor gate_limiter, which is the whole point: a peer that sent
  // it has broken the contract, and the header must not launder that into an
  // ordinary-looking answer.
  const std::string_view kOffContract = "foo";

  // 1. Silent pass-through. Contains() must answer false for a value not in the
  //    set. Printed as 1 when it correctly rejects (returns false), so the
  //    Contains-miss mutation return true flips this to 0.
  std::printf("contains_rejects_foo=%d\n",
              e::Contains(e::kEventCategory, kOffContract) ? 0 : 1);

  // 2. Degrade-to-something-close. IndexOf() must answer kNotAMember for a miss,
  //    never a real position. Printed as 1 when it correctly returns the
  //    sentinel, so the IndexOf-miss mutation return 0 flips this to 0.
  std::printf("indexof_rejects_foo=%d\n",
              e::IndexOf(e::kEventCategory, kOffContract) == e::kNotAMember ? 1 : 0);

  // 3. The sharp form of the degrade case. gate_limiter is ORDERED and estop is
  //    at position 0, so a miss reported as 0 would read as the highest-priority
  //    limiter -- a fail-silent that is worse than a fail-safe. This checks the
  //    miss does NOT collide with the strongest real position. Under the
  //    IndexOf-miss mutation return 0 it flips to 0, which is exactly the
  //    dangerous reading the sentinel exists to prevent.
  std::printf("miss_not_read_as_strongest=%d\n",
              e::IndexOf(e::kGateLimiter, kOffContract) != 0u ? 1 : 0);

  // 4. Positive guard for Contains. "system" is a real member of event_category,
  //    so Contains() must answer true. Without this, a Contains() rewritten to
  //    always return false would pass fact 1 while rejecting everything. It is
  //    unaffected by the miss-branch mutation (the hit branch is untouched), so
  //    it stays 1 and serves as the control for the Contains mutation run.
  std::printf("contains_accepts_system=%d\n",
              e::Contains(e::kEventCategory, "system") ? 1 : 0);

  // 5. Positive guard for IndexOf, and the anchor the strongest-position check
  //    leans on. estop is first in gate_limiter, so its index is 0; an IndexOf()
  //    that always returned kNotAMember would pass fact 2 while finding nothing.
  //    Unaffected by the miss-branch mutation, so it is the control for the
  //    IndexOf mutation run.
  std::printf("indexof_finds_estop=%d\n",
              e::IndexOf(e::kGateLimiter, "estop") == 0u ? 1 : 0);

  // 6. The sentinel's own property, stated so a reader need not trust the header
  //    comment. kNotAMember must not be a real position; if it ever became 0 the
  //    miss-vs-strongest distinction in fact 3 would collapse silently.
  std::printf("not_a_member_sentinel_nonzero=%d\n",
              e::kNotAMember != 0u ? 1 : 0);

  return 0;
}
