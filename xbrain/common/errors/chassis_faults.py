"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: chassis_faults.py
Brief: Chassis fault-code validation -- CF-1 format gate plus the open-set discipline that never drops the message

Description:
The problem this solves. Chassis fault codes are an OPEN set (13 S7.3, ruling 99
U68): the vendor guide's 44-code table is a "common faults" quick reference, not
the complete enumeration, and the complete one is still owed (11 V-08). Two
opposite failures have to be prevented at once, and a naive validator commits one
while avoiding the other:

  * treat the codes as a closed set and reject anything unregistered with E_SCHEMA
    -- and a real fault the robot IS reporting gets swallowed, HMI shows all-green
    while the machine is faulted (13 S7.3, the outcome-to-avoid diagram; QD-6).
  * accept anything at all -- and a malformed code (no space prefix, so its
    meaning is ambiguous between the two vendor code spaces) travels on as if it
    named a real fault.

U68's resolution, which this module implements:
  * a code that is MALFORMED -- fails the CF-1 shape ^(chs|chg):0x[0-9A-Fa-f]{4}$
    (11 S9.8.4 CF-1, 13 S7.3 CF-1) -- is rejected with E_SCHEMA. This is a format
    contract, not a registration one: no prefix, a prefix outside {chs, chg}, or a
    mixed-case prefix all fail, and MUST NOT be degraded to "assume one space".
  * a code that is WELL-FORMED but not in the caller's registered set is marked
    UNKNOWN and reported -- it is kept, not rejected. This is the open set (QD-6):
    the raw code is preserved and the severity still comes from the vendor's own
    Severities field, so an unrecognised code is still actionable.
  * NEITHER of those may drop the whole message. The hardware e-stop / HES bit
    rides in the same report as the codes (13 S6.5 forbid #2, criterion 2's
    "the e-stop bit is in the same message"); if one bad code aborted parsing, the
    safety bit would be lost with it. read_fault_report extracts the safety bit
    first and processes every entry independently, so no single entry can drop it.

Which criterion each entry point serves (XBRAIN_V6_TODO.md INF-DB-4):
  clause 2  malformed code -> E_SCHEMA, reported, message not dropped.
            require_wellformed_fault_code raises (the reject); read_fault_report
            records E_SCHEMA per entry (the not-dropped).
  clause 3  open-set readback of an out-of-set value -> mark unknown, message not
            dropped (T-MODE-1). read_fault_report returns status UNKNOWN with the
            raw value preserved, and the safety bit survives.

What this module deliberately does NOT do, so nothing gets bolted onto it:
  * it does not own the registered code set. The 44-code table (with names and
    descriptions, for human display) belongs to quadruped, which decodes the wire.
    The registered set is a PARAMETER here, so this module never hardcodes an
    incomplete enumeration (V-08 is open) and never becomes a closed-set gate on
    registration -- which is the exact trap 13 S7.3 warns against.
  * it does not reformat the unknown value. 13 S6.5 forbid #3 says report the raw
    code so the field is locatable; UNKNOWN keeps the raw string. The "unknown_0x
    %04X" display string of 13 S6.5 is a quadruped presentation concern, not a
    validation one.
  * it does not emit the event. Marking a code UNKNOWN or MALFORMED is a
    classification; emitting event/fault/chassis is the owning process's job, and
    this layer has no event bus. The caller reads the outcome and emits.
  * it does not derive severity. faults[].level comes from the vendor Severities
    field (13 S7.3), never from the code, so it is not this module's to compute.

A trap this module refuses. The obvious "defensive" shape validates the whole
report and raises on the first bad code, so a caller wraps the parse in one
try/except and, on any bad field, drops the report. That is precisely the
message-drop failure: the safety bit dies with the rejected code. read_fault_report
therefore NEVER raises -- it records the rejection per entry and keeps going -- and
its mutation test injects the drop-on-bad shape and watches the safety bit vanish.
"""

import re
from typing import FrozenSet, NamedTuple, Optional, Sequence, Tuple

# ClosedSetViolation carries E_SCHEMA in its .code (see exceptions.py): its meaning
# in codes.yaml lists 枚举值越界 / malformed structure, which is exactly a code that
# fails the CF-1 shape. Raising it -- rather than referencing an E_SCHEMA literal --
# is how the malformed path reports E_SCHEMA without spelling the code here.
from .exceptions import ClosedSetViolation

# The E_SCHEMA name for the per-entry outcome of read_fault_report. Imported (a
# bare name, not a literal) from the shared library, which the package __init__
# has already bound by the time it imports this submodule at the end of its body.
# It is the SAME code ClosedSetViolation carries; sourcing both from the package
# keeps the batch outcome and the raised exception naming one value.
from . import E_SCHEMA


# CF-1, verbatim from 11 S9.8.4 (grep CF-1) and 13 S7.3 (grep CF-1). The prefix is
# lowercase and closed to {chs, chg}; the hex body is four digits, either case, to
# match the regex the contract prints. Anchored at both ends: without the anchors
# "xchs:0x8001y" would match, and a code embedded in noise is exactly the malformed
# input this gate exists to reject.
FAULT_CODE_RE = re.compile(r"^(chs|chg):0x[0-9A-Fa-f]{4}$")

# The set name reported when a malformed code raises. It is a format set, not a
# value set: what failed is the shape, so the name says shape, and a reader of the
# log is not sent hunting for an out-of-range VALUE that does not exist.
_FAULT_FORMAT_SET = "chassis_fault_code_format"

# The classification a well-formed code lands in, plus the malformed verdict. A
# small closed set of internal status strings (not wire values), so callers branch
# on a name rather than re-deriving the decision. Kept as three distinct members on
# purpose: collapsing REGISTERED and UNKNOWN would lose the open-set marking
# (criterion 3), and collapsing MALFORMED into either would lose the format gate
# (criterion 2).
FAULT_REGISTERED = "registered"   # well-formed and in the caller's known set
FAULT_UNKNOWN = "unknown"         # well-formed, not registered -- open set keeps it
FAULT_MALFORMED = "malformed"     # fails CF-1 -- E_SCHEMA, never "assume a space"
FAULT_STATUSES: FrozenSet[str] = frozenset(
    {FAULT_REGISTERED, FAULT_UNKNOWN, FAULT_MALFORMED})


def is_wellformed_fault_code(code: str) -> bool:
    """True when code matches the CF-1 shape. The single format decision.

    Every other function here routes its format question through this one, so
    there is exactly one place the CF-1 regex is applied and one place a mutation
    that loosens it would land.
    """
    # match, not search: FAULT_CODE_RE is already anchored, so the two are
    # equivalent here, and match states the intent -- the whole string is the code.
    return FAULT_CODE_RE.match(code) is not None


def require_wellformed_fault_code(code: str) -> str:
    """Return code if it is CF-1 well-formed, else raise ClosedSetViolation.

    This is criterion 2's reject half: a malformed code raises, carrying E_SCHEMA,
    with NO degrade-to-a-space (CF-1 forbids interpreting a prefixless or
    mixed-case code as one of the two spaces). It mirrors enums.ClosedSet.parse --
    a boundary validator that returns its input so the check can be welded to the
    decode site -- and the caller that must NOT drop the surrounding message uses
    read_fault_report instead, which catches this per entry.
    """
    if not is_wellformed_fault_code(code):
        # ClosedSetViolation sets code E_SCHEMA and records the set name and the
        # bad value, so the log says which shape rejected which string. from-nothing
        # is unnecessary here: this is the originating raise, not a re-wrap.
        raise ClosedSetViolation(_FAULT_FORMAT_SET, code)
    return code


def classify_fault_code(code: str, known: FrozenSet[str]) -> str:
    """One of FAULT_REGISTERED / FAULT_UNKNOWN / FAULT_MALFORMED. Never raises.

    The pure decision, for callers that want to branch without exception control
    flow. known is the caller's registered set (quadruped owns the real one); it is
    a parameter so this module never hardcodes the incomplete fault enumeration and
    never turns registration into a closed-set gate -- an unregistered but
    well-formed code is UNKNOWN, which is kept, not rejected (QD-6).
    """
    # Format first. A malformed code is malformed regardless of what is in known,
    # so the CF-1 gate precedes the membership test; reversing them would let a
    # malformed string that happened to be listed in known slip through as
    # registered.
    if not is_wellformed_fault_code(code):
        return FAULT_MALFORMED
    # Well-formed. Registration is a display distinction, not a gate: both arms are
    # kept and reported. UNKNOWN is not an error -- it is the open set working.
    if code in known:
        return FAULT_REGISTERED
    return FAULT_UNKNOWN


class FaultOutcome(NamedTuple):
    """One fault code's verdict.

    raw     the code exactly as received, always preserved (13 S6.5 forbid #3: a
            string without its raw value cannot be located in the field).
    status  one of the FAULT_* constants.
    code    E_SCHEMA when status is MALFORMED -- the value the caller reports on
            event/fault/chassis; None otherwise, because REGISTERED and UNKNOWN are
            not errors (the open set keeps them). Optional, not "OK", so a handler
            testing `outcome.code is not None` sees only the real rejections.
    """

    raw: str
    status: str
    code: Optional[str]


class FaultReport(NamedTuple):
    """A whole chassis fault report, parsed without ever dropping it.

    safety_bit  the HES / e-stop bit that rides in the same message as the codes
                (13 S6.5 forbid #2). Preserved unconditionally -- it is copied here
                before any code is looked at, so no code can cost it.
    outcomes    one FaultOutcome per input entry, in order. Same length as the
                input: a bad entry becomes a MALFORMED outcome, it is never
                silently dropped, which would shorten the list and hide that a
                fault arrived at all.
    """

    safety_bit: bool
    outcomes: Tuple[FaultOutcome, ...]


def read_fault_report(safety_bit: bool, entries: Sequence[str],
                      known: FrozenSet[str]) -> FaultReport:
    """Classify every code in a report without ever dropping the report.

    This is the not-dropped half of criterion 2 and the whole of criterion 3. The
    safety bit is taken first and carried through untouched; each entry is
    classified independently; a malformed entry is recorded as an E_SCHEMA outcome
    rather than raised, so it cannot abort the loop and take the safety bit or the
    other entries with it.

    known is the caller's registered set, applies to the well-formed entries, and
    has no default: an empty frozenset is a legitimate explicit value (it is
    today's real state -- the complete enumeration is owed, V-08 -- and it makes
    every well-formed code UNKNOWN, which is the correct open-set answer), so a
    default would only hide a caller that forgot to pass one.

    *** This function NEVER raises. That is the load-bearing property, not an
    incidental one: the moment it can raise on a bad entry, a caller's single
    try/except drops the whole message and the safety bit with it -- the exact
    failure 13 S6.5 forbid #2 describes. The mutation test injects a version that
    drops the report on a bad field and asserts the safety bit is lost.
    """
    outcomes = []
    for raw in entries:
        # classify_fault_code, not require_wellformed_fault_code: the batch path
        # must not raise (see the docstring), so it uses the non-raising classifier
        # and turns the malformed verdict into a recorded E_SCHEMA outcome. The
        # single-code raising validator exists for the OTHER context -- a caller
        # validating one code that is entitled to abort on it.
        status = classify_fault_code(raw, known)
        # E_SCHEMA rides on the outcome only for the malformed verdict. REGISTERED
        # and UNKNOWN carry None: they are not rejections, and stamping a code on
        # them would make the open-set path look like an error path.
        code = E_SCHEMA if status == FAULT_MALFORMED else None
        outcomes.append(FaultOutcome(raw=raw, status=status, code=code))
    # safety_bit is passed straight through. It was never touched by the loop, which
    # is the point: its survival does not depend on any entry being well-formed.
    return FaultReport(safety_bit=safety_bit, outcomes=tuple(outcomes))
