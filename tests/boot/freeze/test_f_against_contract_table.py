"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_f_against_contract_table.py
Brief: freeze assertion F evaluated against the REAL 11 S2.4.7 table (T8 / B15)

Description:
Closes the gap NEXT.md S11 T8 names verbatim: "assertion F and the CONTRACT
table, both directions (new criterion)" and, in the same cell, "the existing
tests/common/zenoh/ group CANNOT serve as evidence -- it compares the contract
json5 against the golden fixture only, it never evaluates F and never reads
common.yaml".

So this file is the one place where the contract's own QoS document is fed to
the freeze-line assertion that governs it. Three facts are pinned, and the
point of pinning all three is that no single one of them is the finding:

  (1) The contract table is REFUSED by F today, and the refusal is exactly
      one row: rt/** -> Q3_cmd, caught by a5_block_on_rt.
  (2) Every OTHER row passes. Swap that one row for a drop profile and F
      goes green -- so (1) is a single-row conflict, not a broken table.
  (3) The refused row resolves, at runtime, to drop/FIFO(32) -- because
      QOS-C1 rewrites it unconditionally. So the state A-5 refuses is one
      the configuration layer cannot produce.

Taken together: the conflict is real, narrow, and A-5 is refusing an
unreachable state. That is CLAUDE.md S3.2 shape (2), an always-red
criterion. Which way to resolve it -- evaluate A-5 after the override, or
keep it as-is and accept that the full contract table never passes F -- is
a judgement call with no right answer, so it is a user ruling (CLAUDE.md
S9.1) and is registered in docs/NEXT.md. 11 S2.4.7's v1.8 correction block
carries the same three facts in prose.

WHY THIS FILE DOES NOT JUST SAY "known red": a comment saying "F refuses
the contract table" rots the moment either side changes, and nobody finds
out. These cases fail the day the ruling lands -- which is exactly when
somebody has to come back and update the contract, the fixture and this
file together.

Where the "contract table" comes from: the qos block of
tests/common/zenoh/golden/qos_bindings_vectors.json, which is a verbatim
transcription of the 11 S2.4.7 json5 block. It is NOT a second source --
test_qos_resolve.py::test_fixture_bindings_are_verbatim and
::test_frozen_profiles_match_contract hold it equal to the contract, row by
row and profile by profile. Parsing the json5 out of markdown here instead
would be a second parser whose failure mode (silently matching nothing)
makes the whole file vacuous; that reasoning is already written out in
test_qos_resolve.py's own header.

What this file does NOT cover: whether configs/common.yaml carries a table
at all (its qos.profiles / qos.bindings are both null today, so F has no
object in production -- that is a config-landing item, not an F defect),
and whether the four-row dev set in common_sim.yaml matches the contract
(it deliberately does not; that file says so itself).

MUTANTS, all three run 2026-10-08 against f_qos_and_port.py, all three
red (CLAUDE.md S3.3):

  A-5 evaluates the POST-override profile
      ("... == BLOCK and not resolution.rt_override_applied")
      -> 2 of 4 cases red. This is the shape option (a) of the ruling
         would produce, so these cases are the notification.
  A-5's scan surface widened to every binding ("rt_covered = True")
      -> 1 red. The global fallback then enters the scan and F names it.
  A-5 skipped entirely ("if True: continue")
      -> 2 red. Guards against the whole check quietly becoming dead.

READING THE GREEN CORRECTLY. These four cases passing does NOT mean the
QoS table is sound. It means: the table is in the state described above,
and A-5 behaves as described above. Three of the four cases assert that
something is BROKEN. If they all go green after someone "fixes" A-5,
that is the bug -- they are supposed to go red then.
"""

# json to read the golden document; copy.deepcopy so a case that edits the
# table cannot leak the edit into the next case through the shared dict.
import copy
import json
import os

import pytest

from tests.boot.freeze.test_f_qos_and_port import (
    _FakeOverlay,
    _stub_port_probe,
)
from xbrain.boot.freeze.assertions.f_qos_and_port import run as f_run
from xbrain.common.errors.exceptions import XbrainError
from xbrain.common.zenoh.qos import load_qos_table

#: Every case reads committed files and evaluates in-process. No chassis,
#: no camera, no socket -- the port probe is the stubbed one F accepts
#: through ctx.
pytestmark = pytest.mark.no_device

#: The golden document. Path derived from this file so the suite can run
#: from any cwd; three dirname() calls reach the repo root from
#: tests/boot/freeze/<name>.py -- a fourth would silently read a sibling
#: checkout on the sshfs mount and every case would pass against the
#: wrong table.
_GOLDEN = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    "tests", "common", "zenoh", "golden", "qos_bindings_vectors.json")

#: The row this whole file is about, spelled once. Written as a constant
#: rather than inline in four places because three of the cases assert
#: that F names THIS row and not another -- four separate string
#: literals would let a half-done edit leave two cases checking the old
#: expression while the other two checked the new one, and both halves
#: would still be green.
_REFUSED_MATCH = "xbrain/*/rt/**"


def _contract_qos():
    """The 11 S2.4.7 document, as a fresh dict every call."""
    # Fresh copy per call, not a module-level constant: two cases below
    # mutate the bindings array, and a shared dict would make the second
    # case depend on whether the first ran.
    # ["qos"] and not the whole document: the file also carries the
    # expected resolutions ("vectors"), which are test_qos_resolve.py's
    # business. Feeding those to F would be feeding it expectations.
    with open(_GOLDEN, encoding="utf-8") as handle:
        return copy.deepcopy(json.load(handle)["qos"])


def _ctx(qos_doc):
    """ctx for F carrying this qos document and the stub port probe."""
    # config_root points nowhere on purpose: F takes the tree from
    # ctx["overlay"] when present, and a real root would make the case
    # depend on configs/ (whose qos keys are null -- F would skip).
    # The path is still required (F asserts on its absence, deliberately:
    # a caller that forgot to populate it is a wiring bug, not a config
    # problem), so it is present and meaningless rather than omitted.
    #
    # _stub_port_probe comes from the existing F test module rather than
    # being redefined here: F' (the port-identity half) runs in the same
    # pass as F and would otherwise try to open real sockets. Reusing
    # the stub keeps one definition of "what a healthy probe returns".
    return {
        "config_root": "/tmp/nonexistent-contract-table",
        "overlay": _FakeOverlay({"common": {"qos": qos_doc}}),
        "port_probe_fn": _stub_port_probe,
    }


def test_the_contract_table_is_refused_by_f_and_by_exactly_one_row():
    """(1) F refuses 11 S2.4.7, with a5_block_on_rt on rt/** -> Q3_cmd.

    Pinned down to the binding match and the profile name, not just "it
    raises": a table that failed F for some unrelated reason (a bad depth,
    an ordering violation) would satisfy a bare pytest.raises and hide
    the fact that the conflict had moved.
    """
    with pytest.raises(XbrainError) as ei:
        f_run(_ctx(_contract_qos()))
    # detail, not str(exception): the message is prose and would match
    # several different failures. The kind is the closed-set field F
    # commits to (its module docstring lists the kinds it raises).
    detail = ei.value.detail
    assert detail["kind"] == "a5_block_on_rt", detail
    assert detail["binding_match"] == _REFUSED_MATCH, detail
    assert detail["profile_name"] == "Q3_cmd", detail
    # The congestion control is in the detail so an operator can see WHY
    # without opening the profile table.
    assert detail["congestion"] == "block", detail


def test_every_other_contract_binding_passes_f():
    """(2) Swap that one row for a drop profile -> F goes green.

    This is the half that makes (1) a finding about ONE row. Without it,
    "F refuses the contract table" is compatible with the table being
    wrong in five places, and the fix for the real conflict would leave
    four behind.

    Q1_rt is the substitute because it is drop and it is already in the
    frozen profile set -- inventing a profile would make this case about
    a table that does not exist.
    """
    qos = _contract_qos()
    for binding in qos["bindings"]:
        if binding["match"] == _REFUSED_MATCH:
            binding["profile"] = "Q1_rt"
            break
    else:
        raise AssertionError(
            "the %s row is gone from the contract table; if it was removed "
            "deliberately, this whole file needs rewriting, not patching"
            % _REFUSED_MATCH)
    # qos_checked is asserted alongside status: F SKIPS the QoS half
    # when common.qos is absent from the tree (M is the required-key
    # check, not F). A skip also reports status "pass", so without this
    # second assertion a ctx that lost its qos document would satisfy
    # the case while proving nothing.
    result = f_run(_ctx(qos))
    assert result["status"] == "pass"
    assert result["qos_checked"] is True


def test_the_refused_row_resolves_to_drop_at_runtime():
    """(3) QOS-C1 rewrites that row to drop, so A-5 refuses an
    unreachable state.

    This is the measurement behind 11 S2.4.7's v1.8 correction block and
    behind the ruling registered in NEXT.md. It is a criterion rather
    than a note because the argument depends on QOS-C1 being
    unconditional: the moment someone makes the override configurable,
    A-5's refusal becomes correct and this case goes red, which is the
    notification that the ruling has to be revisited.

    rt_override_applied is asserted alongside the knobs because the knobs
    alone would also match a table that simply bound rt/** to a drop
    profile -- that is a different world, and this case must not pass in
    it.
    """
    table = load_qos_table(_contract_qos())
    resolution = table.resolve("xbrain/gj-001/rt/probe/test")
    # It really is the row under discussion that matched.
    assert resolution.binding_match == _REFUSED_MATCH
    assert resolution.profile_name == "Q3_cmd"
    # And QOS-C1 fired, turning block into drop.
    assert resolution.rt_override_applied is True
    assert resolution.congestion_control == "drop"
    assert resolution.priority == "interactive_high"
    assert (resolution.handler.kind, resolution.handler.depth) == ("fifo", 32)
    # The same profile on the GENERAL plane is NOT rewritten -- that is
    # what makes the override plane-scoped rather than a blanket "Q3 is
    # always drop", and it is why the contract can keep one Q3_cmd.
    general = table.resolve("xbrain/gj-001/cmd/motion/behavior")
    assert general.rt_override_applied is False
    assert general.congestion_control == "block"


def test_a5_only_ever_examines_the_rt_fallback_row():
    """A-5's scan surface is ONE row: whichever rt/ binding a single
    synthetic probe key happens to hit.

    *** Measured 2026-10-08 while writing the case this replaces. The
    case originally asserted that repointing rt/audio/gate at a block
    profile would make F name GATE. It does not. F still names
    rt/** -> Q3_cmd, and gate goes unexamined.

    Mechanism, from a5_block_on_rt's own body: for each binding whose
    match string contains "rt/", it resolves ONE synthetic key --
    "xbrain/gj-001/rt/probe/test" -- and skips the binding unless that
    key resolved to this very binding ("if resolution.binding_match !=
    match: continue"). Only the rt fallback can ever match a probe key
    nobody registered, so every SPECIFIC rt row -- rt/safety/**,
    rt/chassis/**, both v1.8 audio rows, every cmd_vel rule -- is
    skipped unconditionally.

    Consequence, stated plainly because it is the finding: pointing
    rt/safety/estop at a block profile would NOT be refused by A-5.
    The thing that saves the system today is QOS-C1, which rewrites
    block to drop on the whole rt plane regardless of binding -- i.e.
    the guard that actually holds is the override, and A-5 adds nothing
    on top of it. That pairs with the other A-5 finding (it evaluates
    the PRE-override profile, so the one row it does examine is refused
    for a state the config layer cannot produce): the two together are
    why "fix A-5" is a ruling and not a one-line edit. Registered in
    docs/NEXT.md; 11 S2.4.7's v1.8 block carries both.

    This case is written to go RED when A-5 is fixed. That is the
    intent -- it is the notification that the contract, the fixture and
    this file have to be revisited together.
    """
    qos = _contract_qos()
    rows = {b["match"]: b for b in qos["bindings"]}
    # Both v1.8 rows exist and carry the profile 11 S2.4.7 gives them.
    for match in ("xbrain/*/rt/audio/gate", "xbrain/*/rt/audio/lease"):
        assert rows[match]["profile"] == "Q2_state", match
    # Repoint gate at the block profile. A fixed A-5 would name gate.
    rows["xbrain/*/rt/audio/gate"]["profile"] = "Q3_cmd"
    with pytest.raises(XbrainError) as ei:
        f_run(_ctx(qos))
    assert ei.value.detail["kind"] == "a5_block_on_rt"
    assert ei.value.detail["binding_match"] == _REFUSED_MATCH, (
        "A-5 named %r -- if it now names the gate row, A-5's scan "
        "surface was widened and this file's premise changed"
        % ei.value.detail["binding_match"])
    # And the proof that gate really is unexamined rather than merely
    # outranked: remove the fallback row entirely, and F goes GREEN with
    # gate still pointing at a block profile.
    qos["bindings"] = [b for b in qos["bindings"]
                       if b["match"] != _REFUSED_MATCH]
    result = f_run(_ctx(qos))
    assert result["status"] == "pass", (
        "with the fallback gone, a block-bound rt/audio/gate passed F -- "
        "this is the scan-surface hole, not an accident of ordering")
