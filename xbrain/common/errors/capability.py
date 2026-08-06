"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: capability.py
Brief: The capability-unavailable guard -- one place that turns an adjudicated-unavailable function into a rejection

Description:
The problem this solves. Several M20S functions have been adjudicated permanently
unavailable this period: the vendor either never shipped the interface, left its
semantics undocumented, or the interface is present but unsafe to exercise. If
each caller (P4 intent handling, quadruped command translation, teleop) decides
on its own whether to reject one of these and which code to send, the codes drift
-- one caller sends E_CAPABILITY, another sends E_NOT_IMPLEMENTED for the same
function -- and a cloud client that branches on the code takes the wrong arm. This
module is the single point that answers "is this function adjudicated unavailable,
and if so how is it rejected", so the answer is the same everywhere.

Where the table comes from. 21 (measurement and third-party debt) sections 2 and
3 carry, in the fourth column of each V-row ("unclosed default behaviour"), the
disposition for every debt item. The rows whose disposition names a rejection code
(E_CAPABILITY or E_NOT_IMPLEMENTED) are exactly the adjudicated-unavailable
functions. The done-criterion for this item (XBRAIN_V6_TODO.md INF-DB-4, clause 1)
is "for EVERY function adjudicated unavailable in 21 S3 a call returns rejected
plus the specified code". _UNAVAILABLE below is the hand-written single source of
truth, and tests/common/test_capability_guard.py diffs it against 21 in BOTH
directions -- so a row added to 21 that this table misses, and a debt id here that
21 does not adjudicate, both fail. That bidirectional diff is the only thing
keeping the two equal (the same shape as test_error_codes.py against 11 S13).

Why keyed by the 21 debt id and not a coined capability name. The debt ids
(V-06, V-45, ...) are stable, documented identifiers that 21's rows are already
keyed by, so the bidirectional diff is mechanical. Coining English names for the
functions (there is no documented "illumination_capability" identifier for most
rows) would be inventing a name the contract does not carry, which is exactly the
fabrication CLAUDE.md 3.1/9.1 forbids. The caller maps its own semantic intent to
a debt id from the design; this module owns the debt-id -> rejection mapping.

What this module deliberately does NOT do, so nothing gets bolted onto it:
  * it does not decide the AVAILABLE path. capability_guard returns None for a
    debt id not in the table, which means "this guard has no opinion" -- NOT
    "available". Whether an available function can run today depends on calibrated
    values and cloud answers (V-45/V-59 block the available gait/action paths),
    and that decision lives with the caller, not here. This guard only owns the
    REJECTED path (INF-DB-4 triage: V-45/V-59 block the available path, not the
    guard's rejected path).
  * it does not build an Ack. The Ack.result = rejected wrapper, the cmd_id echo
    and the transport belong to the gateway/pipeline. This hands back the code and
    detail; the caller assembles the reply.
  * it does not invent detail keys. 11 S13.13 (group J) gives E_CAPABILITY and
    E_NOT_IMPLEMENTED no detail column, and codes.yaml marks both detail
    unspecified, so the contract mandates no required detail key for either. Where
    a design volume DOES name a detail.item it does so inconsistently across
    volumes (13 writes prone_on_stair for the stair-prone refusal while 18 writes
    gait; 13 pairs gait_readback_gap with E_NOT_IMPLEMENTED while 21 S3 pairs the
    same row with E_CAPABILITY), so pinning a detail.item here would be picking a
    side in an unresolved cross-volume conflict. detail is therefore left empty
    and the vacuity is asserted against the contract (see the metatest), not
    papered over -- it becomes a real key requirement the day 11 grows one.

A trap this module is built to refuse. The tempting shape is a guard that returns
None (proceed) for a capability it does not recognise AND for one it knows is
unavailable, so a caller reads both as "go ahead". That collapses "unavailable"
into "unknown", and an unavailable function then runs silently. The guard returns
a rejection object for the unavailable case and None only for the not-in-table
case; the two are distinguishable, and the mutation test injects the collapse
(drop a row -> the function it named is now "accepted") and watches it go red.

Naming note. On disk this is xbrain/common/errors/capability.py; CLAUDE.md 3.5
writes the shared error library as common/errors/, and 0.2 reserves the top-level
common/ for deployed artifacts, so the Python source sits under xbrain/ -- the
same split the package __init__ explains.
"""

from typing import Dict, FrozenSet, NamedTuple, Optional

# The two rejection codes, imported as NAMES from the shared library rather than
# spelled as string literals. no_literal_ecode.py forbids the literal spelling
# outside codes.yaml (a code spelled twice is a code that drifts); a bare imported
# name is not a literal and cannot drift from the table it came from. The import
# resolves because the package __init__ binds these names (globals().update over
# codes.yaml) BEFORE it imports this submodule at the end of its own body -- so by
# the time this line runs the names exist on the partially-initialised package.
from . import E_CAPABILITY, E_NOT_IMPLEMENTED


# The unavailable-capability table. Hand-written single source of truth, keyed by
# 21 debt id, valued by the rejection code (an imported name, never a literal).
# Every entry is diffed against 21 sections 2 and 3 in both directions by
# tests/common/test_capability_guard.py; do not edit one side without the other.
#
# The per-row comment records the 21 anchor and the one-line reason, so a reader
# does not have to open 21 to see why a row is here. The code on the right is what
# 21's fourth column literally says for that row -- NOT necessarily what a design
# volume says, because the criterion (INF-DB-4 clause 1) names 21 S3 as the source.
# The one place 21 and 13 disagree is called out on its row.
_UNAVAILABLE: Dict[str, str] = {
    # 21 S2 V-06: on-board teleop accepts the keyboard device only; any other
    # TeleopInput.device value is rejected. The gamepad mapping table is not
    # enabled this period (which of the two teleop inputs wins is unmeasured).
    "V-06": E_CAPABILITY,
    # 21 S3 V-45: stunt/trick actions -- the action-parameter value table is
    # absent (the vendor example value is not even in the parameter table), so the
    # supported action set is unknown and stunt actions are not implemented.
    "V-45": E_NOT_IMPLEMENTED,
    # 21 S3 V-47: an independent lighting command (front/rear) exists only in the
    # retired manual, not in the current guide; the illumination field is rejected
    # rather than faked as success.
    "V-47": E_CAPABILITY,
    # 21 S3 V-48: whether the startup damping gear may be commanded is
    # self-contradictory in the guide; conservatively it is never sent, and a
    # request for it is rejected and logged warn.
    "V-48": E_CAPABILITY,
    # 21 S3 V-49: the "zero" (5) and "damped prone" (0x1001) actions have no
    # documented semantics, preconditions, duration or failure behaviour, so they
    # are not implemented.
    "V-49": E_NOT_IMPLEMENTED,
    # 21 S3 V-54: prone under a stair gait is refused (PR-1 / QC-9) -- the vendor
    # only guaranteed anti-rollover for the navigation gait, and prone on stairs is
    # a rollover risk. A stale or missing gait field is treated as stair, i.e. also
    # refused.
    "V-54": E_CAPABILITY,
    # 21 S3 V-59: sending a gait outside the send/readback intersection. 21 S3's
    # fourth column reads E_CAPABILITY for this row and the criterion names 21 S3
    # as the source, so E_CAPABILITY is what this table carries.
    # *** CROSS-VOLUME CONFLICT, recorded not resolved: 13 S12.1 V-59 and 13 GS-1
    # give E_NOT_IMPLEMENTED (with detail.item gait_readback_gap) for the very same
    # row, while 21 S3 -- which 21's own note claims is a verbatim copy of 13 S12.1
    # -- says E_CAPABILITY. That disagreement is a human ruling (CLAUDE.md 9.1), so
    # this follows the criterion's named source (21 S3) and the ruling is reported
    # upward rather than decided here.
    "V-59": E_CAPABILITY,
}

#: The debt ids this guard rejects. A frozenset so a caller can test membership and
#: so nothing can add a row at run time (an unavailable set an operator could widen
#: is not one). !! Never write its size into code or a comment (CLAUDE.md 3.7): the
#: authoritative membership is this object, and a copied count rots.
CAPABILITY_DEBTS: FrozenSet[str] = frozenset(_UNAVAILABLE)


class CapabilityRejection(NamedTuple):
    """How an adjudicated-unavailable function is rejected.

    debt   the 21 debt id that adjudicated it (e.g. V-47), for the log and for the
           caller to cite; it is the stable identifier, not a coined name.
    code   E_CAPABILITY or E_NOT_IMPLEMENTED -- a shared-library value, so the
           wire form and this field cannot diverge.
    detail the EC-3 structured payload. Empty for these codes: the contract marks
           both detail unspecified (11 S13.13 group J has no detail column), so
           there is no required key to carry, and the design volumes name detail
           items inconsistently (see the module docstring). A fresh dict is built
           per call so no two rejections share one mutable object.
    """

    debt: str
    code: str
    detail: dict


def capability_guard(debt_id: str) -> Optional[CapabilityRejection]:
    """The rejection for an adjudicated-unavailable function, or None.

    None means this guard has NO opinion on debt_id -- it is not in the
    unavailable table. That is deliberately NOT the same as "available": whether an
    available function can run today is the caller's decision (calibration, cloud
    answers), and this guard owns only the rejected path. A caller must not read
    None as a green light for a function it has not otherwise checked.

    A non-None result means the function IS adjudicated unavailable and the caller
    must reply rejected with the returned code and detail. Because the result is a
    fresh object each call, the caller may add its own context to detail without
    mutating a shared row.
    """
    # .get with NO default. This is not the CLAUDE.md 3.1 fallback pattern it can
    # resemble -- there is no substitute value -- it is a membership lookup that
    # returns None for "not in the table", which is precisely the "no opinion"
    # answer this guard is documented to give. A .get(debt_id, SOMETHING) here
    # would be a fabricated disposition for an unrecognised id, the fail-silent
    # shape that section forbids.
    code = _UNAVAILABLE.get(debt_id)
    if code is None:
        return None
    # A fresh empty detail per call. Empty because the contract mandates no
    # required detail key for E_CAPABILITY / E_NOT_IMPLEMENTED (11 S13.13 group J,
    # detail unspecified in codes.yaml); the metatest asserts that vacuity against
    # the contract rather than letting it pass unstated, so if 11 ever adds a
    # required key the assertion fires and this line has to grow to fill it.
    return CapabilityRejection(debt=debt_id, code=code, detail={})
