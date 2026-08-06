"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chassis_faults.py
Brief: INF-DB-4 clauses 2 and 3 -- the CF-1 format gate rejects, the open set marks unknown, and neither drops the message

Description:
This pins the two fault-code contracts INF-DB-4 leaves to the shared library after
ruling 99 U68, and performs the two mutations that row names.

Clause 2 (format gate). A chassis fault code that fails the CF-1 shape
^(chs|chg):0x[0-9A-Fa-f]{4}$ (11 S9.8.4 CF-1, 13 S7.3 CF-1) is rejected with
E_SCHEMA and MUST NOT be degraded to "assume a code space". The reject half is
require_wellformed_fault_code raising; the not-dropped half is read_fault_report
recording E_SCHEMA per entry. The named mutation -- let an out-of-set value pass
silently -- is injected by removing the format gate, after which a malformed code
is classified as merely unknown instead of rejected.

Clause 3 (open set, T-MODE-1). A well-formed code that is not in the caller's
registered set is marked UNKNOWN and kept, not rejected (QD-6), and its raw value
is preserved (13 S6.5 forbid #3). And no bad field may drop the whole report,
because the HES / e-stop bit rides in the same message (13 S6.5 forbid #2). The
named mutation -- drop the whole message because of one unknown field -- is
injected by wrapping read_fault_report so any non-registered field discards the
report, after which the safety bit is observed to vanish.

Why the mutations monkeypatch the module attributes rather than run a subprocess.
These are pure functions with no process-wide singleton behind them, so an
in-process patch that pytest restores after the test cannot leak the way patching
enums.ClosedSet.parse would. read_fault_report and classify_fault_code are looked
up in the module globals on each call, so replacing the module attribute drives the
real code paths (the batch still calls the patched classifier) rather than a
transcription of them.

The positive assertions are here for a reason (CLAUDE.md 3.2 form 1). A classifier
that answered MALFORMED for everything would satisfy every rejection case while
rejecting legal codes too, and a report parser that returned an empty report would
satisfy "did not raise" while dropping everything. So legal codes are asserted to
survive: a registered code stays registered, a well-formed unknown one is kept with
its raw value, and the safety bit and every entry come through.
"""

import os
import sys

import pytest

# ROOT three levels up, on sys.path, no conftest -- the pattern the sibling tests
# use so "from xbrain.common ..." resolves.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common import errors  # noqa: E402
from xbrain.common.errors import chassis_faults  # noqa: E402
from xbrain.common.errors.exceptions import ClosedSetViolation  # noqa: E402

# A small registered set for the tests. NOT the real 44-code table -- that belongs
# to quadruped and is still incomplete (V-08). Supplying our own here is the point:
# the module takes the registered set as a parameter precisely so a test (or a
# caller) provides it, and the open-set behaviour does not depend on the real one.
KNOWN = frozenset({"chs:0x8001", "chs:0x8002", "chg:0x1007"})  # both spaces, so chg is exercised too

# Codes that MUST pass the CF-1 shape: both prefixes, hex body in either case.
WELLFORMED = ("chs:0x8001", "chg:0x1007", "chs:0xABCD", "chs:0xabcd")  # upper + lower hex both legal

# Codes that MUST fail CF-1, one per way it can be malformed. The mixed-case and
# wrong-prefix cases are the ones CF-1 calls out as "do not degrade to a space".
MALFORMED = (  # one entry per distinct malformation, so no single class is untested
    "0x8001",       # no space prefix at all
    "chs:8001",     # missing the 0x
    "CHS:0x8001",   # upper-case prefix -- must not be read as chs
    "chx:0x8001",   # prefix outside the closed {chs, chg}
    "chs:0x800G",   # G is not a hex digit
    "chs:0x801",    # three hex digits, not four
    "chs:0x80011",  # five hex digits, not four
)


def test_wellformed_codes_pass():
    """The positive half of the format gate: legal codes are accepted unchanged.

    Without this an is_wellformed that returned False for everything would satisfy
    every malformed-rejection case below while rejecting real codes -- the empty
    shell CLAUDE.md 3.2 form 1 warns of.
    """
    for code in WELLFORMED:  # every shape the CF-1 regex must accept
        assert chassis_faults.is_wellformed_fault_code(code), code  # predicate says yes
        # require returns its input, so the check can be welded to a decode site.
        assert chassis_faults.require_wellformed_fault_code(code) == code  # returned as-is


def test_malformed_codes_raise_e_schema():
    """*** Clause 2 reject half: a malformed code raises, carrying E_SCHEMA.

    Every MALFORMED shape must raise ClosedSetViolation, and the raised code must be
    E_SCHEMA (read off the exception, never a hardcoded string), so a handler
    branches on the same value the contract names. The set name says the failure is
    a format one, not an out-of-range value.
    """
    for code in MALFORMED:  # each malformation must raise, none may slip through
        with pytest.raises(ClosedSetViolation) as caught:  # the reject is a raise
            chassis_faults.require_wellformed_fault_code(code)  # under test
        exc = caught.value  # the raised instance, so its fields can be inspected
        # E_SCHEMA from the shared library, compared against the exception's code.
        assert exc.code == errors.E_SCHEMA, (  # the code the contract names for bad shape
            "%r rejected with %r, not E_SCHEMA" % (code, exc.code))
        # The offending value is in the message so a log reader sees what failed.
        assert code in str(exc), str(exc)  # the bad value is named in the log form


def test_classify_three_ways():
    """Registered, unknown and malformed are three distinct verdicts.

    Registered and unknown are BOTH kept (the open set); malformed is the only
    rejection. Collapsing any pair would lose either the format gate or the
    unknown-marking, so all three are asserted here as the pure decision.
    """
    reg = chassis_faults.classify_fault_code("chs:0x8001", KNOWN)  # in the known set
    assert reg == chassis_faults.FAULT_REGISTERED, reg  # a listed code is registered
    # Well-formed but not registered -> unknown. This is the open set: kept, not
    # rejected, even though the complete enumeration is still owed (V-08).
    unk = chassis_faults.classify_fault_code("chs:0x800F", KNOWN)  # well-formed, unlisted
    assert unk == chassis_faults.FAULT_UNKNOWN, unk  # kept as unknown, not thrown away
    # Fails CF-1 -> malformed, regardless of the known set.
    mal = chassis_faults.classify_fault_code("0x800F", KNOWN)  # no prefix -> bad shape
    assert mal == chassis_faults.FAULT_MALFORMED, mal  # the only rejection of the three
    # And with an EMPTY known set -- today's real state -- a well-formed code is
    # unknown, never malformed: registration and format are independent axes.
    empt = chassis_faults.classify_fault_code("chs:0x8001", frozenset())  # nothing registered
    assert empt == chassis_faults.FAULT_UNKNOWN, empt  # well-formed stays well-formed


def test_open_set_keeps_the_unknown_code():
    """Clause 3 marking: a well-formed unregistered code is kept with its raw value.

    QD-6 and 13 S7.3: an unregistered code is marked unknown and reported, never
    dropped and never rejected. read_fault_report must return status UNKNOWN, no
    error code, and the raw string unchanged (13 S6.5 forbid #3 -- report the raw
    value or the fault cannot be located).
    """
    rep = chassis_faults.read_fault_report(False, ["chs:0x9999"], KNOWN)  # one unlisted code
    assert len(rep.outcomes) == 1  # kept, so exactly one outcome comes back
    out = rep.outcomes[0]  # the single verdict to inspect
    assert out.status == chassis_faults.FAULT_UNKNOWN  # marked unknown, not rejected
    assert out.raw == "chs:0x9999", "the raw code must be preserved, not reformatted"
    assert out.code is None, "an unknown code is not a rejection; it carries no code"


# A fixed mixed report used by the message-preservation control and its mutation.
# One registered, one malformed, one well-formed-unknown, so all three arms are
# exercised in the same call and the safety bit rides alongside them.
BAD_MIX = ["chs:0x8001", "0x800F", "chs:0x9999"]


def _assert_message_preserved():
    """The clause-3 property: the safety bit and every entry survive a bad field.

    Factored out so the SAME assertion runs against the real function and against
    the mutant, which is what makes "the mutation reddens this" a claim about this
    exact check rather than a separate one.
    """
    # safety_bit True is the HES bit set; it must come through whatever the codes do.
    rep = chassis_faults.read_fault_report(True, BAD_MIX, KNOWN)
    assert rep.safety_bit is True, "the e-stop bit was dropped with a bad code"
    # Same length as the input: a bad entry becomes an outcome, it is not removed.
    assert len(rep.outcomes) == len(BAD_MIX), "an entry was dropped from the report"


def test_message_preserved_under_bad_field():
    """*** Clause 2 not-dropped half plus clause 3: control on the real function.

    The registered, malformed and unknown entries all come back, the malformed one
    stamped E_SCHEMA, and the safety bit survives. This is the baseline the
    drop-message mutation breaks.
    """
    _assert_message_preserved()
    # Also pin each verdict, so "length 3" cannot pass with the wrong statuses.
    rep = chassis_faults.read_fault_report(True, BAD_MIX, KNOWN)
    statuses = [o.status for o in rep.outcomes]
    assert statuses == [chassis_faults.FAULT_REGISTERED,
                        chassis_faults.FAULT_MALFORMED,
                        chassis_faults.FAULT_UNKNOWN], statuses
    # The malformed entry, and ONLY it, carries E_SCHEMA -- reported on
    # event/fault/chassis by the caller, without dropping the rest.
    assert rep.outcomes[1].code == errors.E_SCHEMA
    assert rep.outcomes[0].code is None and rep.outcomes[2].code is None


def test_silent_passthrough_mutation_is_caught(monkeypatch):
    """*** Clause 2 mutation: remove the format gate and a malformed code passes.

    Control (real): classify says MALFORMED and the batch stamps E_SCHEMA. Mutant:
    a classifier with no CF-1 check treats a malformed code as merely unknown, so it
    travels on as a valid-but-unregistered code -- the silent pass-through the
    clause names. The batch calls the patched classifier (it is a module-global
    lookup), so both the pure and batch assertions redden together.
    """
    def _assert_malformed_rejected():
        # The pure verdict and the batch outcome must both say rejected.
        assert chassis_faults.classify_fault_code("0x800F", KNOWN) == chassis_faults.FAULT_MALFORMED
        rep = chassis_faults.read_fault_report(True, ["0x800F"], KNOWN)
        assert rep.outcomes[0].status == chassis_faults.FAULT_MALFORMED
        assert rep.outcomes[0].code == errors.E_SCHEMA

    # Control: the real classifier rejects the malformed code.
    _assert_malformed_rejected()

    # Mutant: skip the format gate. A code is now registered-or-unknown by
    # membership alone, with no shape check -- so "0x800F" becomes unknown, accepted.
    def _no_format_gate(code, known):
        return (chassis_faults.FAULT_REGISTERED if code in known
                else chassis_faults.FAULT_UNKNOWN)

    monkeypatch.setattr(chassis_faults, "classify_fault_code", _no_format_gate)
    with pytest.raises(AssertionError):
        # The malformed code now passes as unknown; the rejection assertion fails,
        # which is the mutation being caught.
        _assert_malformed_rejected()


def test_drop_message_mutation_is_caught(monkeypatch):
    """*** Clause 3 mutation: drop the whole report on one bad field.

    Control (real): the safety bit and all entries survive. Mutant: wrap
    read_fault_report so any non-registered field discards the report -- the
    "defensive" shape that validates the whole message and rejects on any bad code,
    taking the e-stop bit down with it. The safety bit then vanishes, reddening the
    preservation assertion, which is exactly what 13 S6.5 forbid #2 exists to
    prevent.
    """
    # Control: the real function preserves the message.
    _assert_message_preserved()

    real = chassis_faults.read_fault_report

    def _drop_on_bad(safety_bit, entries, known):
        # Run the real classification, then throw the report away if anything was
        # not registered. This is the message-drop bug in its most plausible form.
        rep = real(safety_bit, entries, known)
        if any(o.status != chassis_faults.FAULT_REGISTERED for o in rep.outcomes):
            # safety bit lost, outcomes gone -- the HES bit dropped with the codes.
            return chassis_faults.FaultReport(safety_bit=False, outcomes=())
        return rep

    monkeypatch.setattr(chassis_faults, "read_fault_report", _drop_on_bad)
    with pytest.raises(AssertionError):
        # BAD_MIX has a malformed and an unknown entry, so the mutant drops the
        # report; the safety-bit assertion fails, catching the mutation.
        _assert_message_preserved()
