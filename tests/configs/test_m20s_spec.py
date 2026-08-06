"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_m20s_spec.py
Brief: configs/models/m20s.yaml checked against 11 S9.6, S9.6.1.1 and S9.2.4

Description:
CFG-CF-3. The L2 model file is the ONLY place the whole stack is allowed to
learn what this chassis physically cannot exceed, so two different mistakes in
it are silent and expensive:

  * A limit written 0.0 instead of null. 0.0 passes "is this key assigned",
    reaches the run time, and makes v_max = min(..., 0) = 0. The robot then does
    not move and nothing is logged. null is the spelling that means "declared,
    not yet calibrated": 10 S5.4.4 assertion A refuses to boot and prints the key
    path. The same trap has a sharper edge on odom.yaw_drift_dps, where 0.0 reads
    as "no drift" -- 11 S9.6 records the measured failure chain, 12 CD-1 tests
    `== null` and finds it false, so the forced outer-loop heading recheck never
    runs and CD-7 computes err_pred = 0.0 * t_spin = 0, skipping the standstill
    recheck too. A 180 degree spin then gets NO heading verification at all.

  * A gait missing from, or zeroed in, spec.gait_limits. 11 S9.6.1.1 GT-3 turns a
    missing gait name into "unknown gait forever" plus a permanent warn, and a
    zero turns that gait into one that cannot move with nothing reported.

WHAT THIS FILE IS AND IS NOT. It is a configuration-tree check that runs today.
It is NOT the startup assertion executor: SP-1 / SP-3 / SP-9 / SP-10 are
evaluated by xbrain-config-freeze.service (11 S9.6, the note under the assertion
table: "全部断言由 xbrain-config-freeze.service 执行"), which is CFG-FZ-7 and
does not exist yet. The mutations named in the CFG-CF-3 criterion say "SP-9 must
go red" and "SP-10 must go red"; until that executor exists the same mutations
are made to go red here instead, which is the honest half of the claim. When
CFG-FZ-7 lands, these cases stay -- they cover the file, the executor covers the
merged tree, and they fail for different reasons.

WHY THE CONTRACT IS PARSED INSTEAD OF TRANSCRIBED. The gait closed set and the
per-gait limits are read out of 11 at test time. Typing them here would compare
the contract against one person's copy of the contract, and a shared typo would
read as agreement -- the same reason xbrain/common/enums keeps its values in
sets.yaml and diffs them back against 11.

*** A closed set for `gait` does NOT yet exist in xbrain/common/enums. That is
why this file extracts it from 11 S9.2.4 rather than importing it, and it is a
gap worth closing when the freeze executor needs the set at run time: an
executor cannot parse markdown. It is recorded here rather than fixed here
because xbrain/common/enums/sets.yaml belongs to BIZ-CM-0, and a second session
editing it would collide.

WHICH STARTUP ASSERTION CATCHES WHAT, because three of them look interchangeable
from a distance and are not (10 S5.4.4, 10 S5.4.5):

  A  reports a key that is present and null. It prints the KEY PATH, which is
     what a field engineer needs -- the fix is to fill that key.
  M  reports a key that is ABSENT from the four namespaces L0 may not default
     (common.safety., common.spec., common.motion.profiles, common.fence.).
     A cannot see an absent key, so deleting a line is not a quieter way of
     saying "uncalibrated": it is the way to make the omission unreportable.
  G  reports a value that is present but out of range -- SP-1, SP-5, SP-9,
     SP-10, SP-11. It prints a RANGE violation, and the field's first instinct
     on reading one is to go and edit the assertion.

They fail in the same direction (refuse to boot) and say different things, which
is why the cases below check both presence and nullness rather than either one.

WHY THIS LIVES IN tests/configs/ AND NOT tests/common/. tests/common/ holds the
xbrain/common/ library -- the merger, the reference expander, the closed sets --
and every case there passes on an empty configs/ tree. These cases are about the
CONTENT of one deployed file, so they fail when the repository's configuration
is wrong even though every library test is green. Filing them next to the
library would blur that line, and CFG-CF-3 names tests/configs/ for this reason.
"""

import os
import re
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common.config import layers, merge  # noqa: E402

# The three paths this file reads. CONFIG_ROOT is the SOURCE tree, not
# /run/xbrain/resolved/. That is deliberate and is the same exception 10 S5.4.4
# assertions J and B take: they evaluate on the source because they exist to
# catch things -- a forbidden key, a wrong layer -- that no longer exist once the
# references have been expanded. scripts/lint/no_config_source_read.py scans
# xbrain/, ros2_ws/ and services/ for exactly this and deliberately does not scan
# tests/, so nothing here weakens the run-time rule.
CONFIG_ROOT = os.path.join(ROOT, "configs")
MODELS_DIR = os.path.join(CONFIG_ROOT, "models")
M20S = os.path.join(MODELS_DIR, "m20s.yaml")
CONTRACT = os.path.join(ROOT, "docs", "11-接口契约.md")

# The five whole-vehicle limits of 11 S9.6 SP-1, as leaf paths under the L2
# namespace. Written as key paths, not as a nested shape, because that is what
# assertion A prints and what a reader has to grep for.
FIVE_LIMITS = (
    "common.spec.max_vx_mps",
    "common.spec.max_vy_mps",
    "common.spec.max_wz_radps",
    "common.spec.max_accel_mps2",
    "common.spec.max_decel_mps2",
)

# 11 S9.6 terrain block. Called out separately from the five because their reason
# for being null is stronger, not weaker: zero slope and zero step height are
# meaningful values (climbs nothing, steps over nothing), so 0.0 here is more
# likely than elsewhere to be read as a real measurement.
TERRAIN = (
    "common.spec.terrain.max_slope_deg",
    "common.spec.terrain.max_step_height_m",
    "common.spec.terrain.max_gap_m",
)


# --- reading the file under test ------------------------------------------

def _flat_config(path):
    """Flatten one yaml file to dotted leaf paths. A comment-only file is {}."""
    # safe_load returns None for a file that holds only comments, which every
    # other skeleton under configs/ currently is. Mapping that to {} rather than
    # letting it through keeps the callers free of a None branch that would
    # otherwise be duplicated at four sites.
    #
    # A malformed file is NOT swallowed here: safe_load raises and the case fails
    # naming the file. A try/except returning {} would be the comfortable version
    # and would turn "this file no longer parses" into "this file defines no
    # spec keys", which is the same silent-shrink failure the closed-set loader
    # in xbrain/common/enums was rewritten to avoid.
    #
    # flatten comes from xbrain/common/config so the key paths here are the ones
    # the loader itself produces. Re-walking the tree locally would let this file
    # and the loader disagree about what counts as a leaf -- an empty map, for
    # instance, which merge.flatten deliberately keeps as a leaf so the branch
    # cannot vanish from the key set.
    with open(path, encoding="utf-8") as fh:
        tree = yaml.safe_load(fh)
    return merge.flatten(tree) if isinstance(tree, dict) else {}


@pytest.fixture(scope="module")
def flat():
    """configs/models/m20s.yaml as dotted leaf path -> value."""
    return _flat_config(M20S)


# --- reading the contract --------------------------------------------------

def _contract_lines():
    """11 as a list of lines. Read once per call; the file is read three times."""
    with open(CONTRACT, encoding="utf-8") as fh:
        return fh.read().split("\n")


def _section(*fragments):
    """Lines of the first heading containing every fragment, up to the next one.

    Anchored on the section NUMBER plus a verbatim fragment of its title, never
    on a line number: CLAUDE.md 1.1 NUM-4 exists because a review round once
    found all three of its line references had drifted onto blank lines while the
    citing side believed it had checked them. Bounding the search to one section
    also matters here -- "| `gait` |" appears twice in 11, and the second one is a
    field-type table whose cell holds no member names at all.
    """
    lines = _contract_lines()
    for i, line in enumerate(lines):
        if not re.match(r"^#{1,6}\s", line):
            continue
        if all(f in line for f in fragments):
            out = []
            for row in lines[i + 1:]:
                if re.match(r"^#{1,6}\s", row):
                    break
                out.append(row)
            return out
    raise AssertionError(
        f"11: no heading contains all of {fragments!r} -- the section was "
        "renamed or renumbered, so every check keyed off it is now blind"
    )


def contract_gait_closed_set():
    """The five gait values of 11 S9.2.4, in contract order."""
    # The row is | `gait` | `basic` / ... / `stair_standard` | 1 / 2 / 12 / 13 / 14 |
    # so the members are the backticked identifiers of the SECOND cell. The third
    # cell holds the chassis numbers, which are the vendor's encoding and not part
    # of this contract's closed set.
    for row in _section("9.2.4", "模式切换指令"):
        if row.startswith("| `gait` |"):
            cells = row.split("|")
            values = re.findall(r"`([a-z_]+)`", cells[2])
            assert values, "11 S9.2.4: gait row matched but no members parsed"
            return values
    raise AssertionError("11 S9.2.4: no row starts with the gait cell")


def contract_gait_limits():
    """gait -> max_vx_mps from the config-carrier block of 11 S9.6.1.1."""
    # Parsed from the yaml block the contract labels as the configuration
    # carrier, not from the prose table above it. The two agree today; the block
    # is the one that says what this file must contain, and pinning the check to
    # it is what makes a contract edit surface here as a red test rather than as
    # a silent divergence between docs and configs.
    out = {}
    seen_block = False
    for row in _section("9.6.1.1", "按步态的速度上限表"):
        if row.strip().startswith("gait_limits:"):
            seen_block = True
            continue
        if not seen_block:
            continue
        m = re.match(r"^\s+([a-z_]+):\s*\{\s*max_vx_mps:\s*([0-9]+(?:\.[0-9]+)?)\s*\}", row)
        if m:
            out[m.group(1)] = float(m.group(2))
        elif row.strip().startswith("```"):
            break
    assert out, "11 S9.6.1.1: gait_limits block found but no entries parsed"
    return out


# --- the two checks that a future freeze executor will need ----------------

def sp10_upper_half(spec_max_vx, gait_limits):
    """SP-10's `<= spec.max_vx_mps` half. Returns (state, offending gaits).

    Split out as a function so the branch below can be tested on synthetic input.
    On the shipped file spec.max_vx_mps is null, so the comparison must NOT run,
    and a check that only ever takes the unevaluable branch proves nothing about
    the comparison it claims to make (CLAUDE.md 3.2 form 1).

    The guard is not defensive style, it is written into the contract. 11 S9.6
    SP-1 states the precondition in so many words: decide "key present and
    numeric" FIRST, and only then compare. In Python `2.0 <= None` raises
    TypeError, which would abort xbrain-config-freeze.service with a traceback
    instead of the key path assertion A exists to print. The refusal direction is
    the same either way, but a stack trace sends the field to the wrong place.
    """
    if not isinstance(spec_max_vx, (int, float)) or isinstance(spec_max_vx, bool):
        return "unevaluable_assertion_a_first", []
    return "evaluated", sorted(g for g, v in gait_limits.items() if v > spec_max_vx)


def spec_definitions(flat_keys):
    """Keys that DEFINE something under spec, in either spelling.

    Both spellings are checked on purpose. The contract's own sample block in 11
    S9.6 is rooted at a bare `spec:`, while the deployed shape is
    `common.spec.*` (10 S5.4.3 gives L2 the common.spec. / common.motion.
    allowance). Somebody copying the sample into a site file produces the bare
    form, and a check that knew only the dotted common form would wave it
    through -- which is the exact edit SP-3 exists to stop.

    References are values, never keys, so `${common.spec.max_vx_mps}` on the
    right-hand side of an L6 key is not reported. 13 S8.3 refines QC-1 to say
    precisely this: assert that a process file does not DEFINE a spec. key, and
    keep allowing it to REFERENCE one, because otherwise the reference in 13 S8.2
    would be rejected by its own assertion.
    """
    return sorted(k for k in flat_keys
                  if k == "spec" or k.startswith("spec.")
                  or k == "common.spec" or k.startswith("common.spec."))


def spec_offences(rel_path, flat):
    """spec.* keys this particular file is not allowed to carry.

    One exemption, and it is narrow. 10 S5.4.5 gives the single definition site
    for common.spec.* as models/m20s.yaml and adds, in parentheses on the same
    row, that common.yaml only DECLARES the key positions as null. SP-3 names the
    two layers it refuses -- the site layer and the process-private layer -- and
    the L1 shared file is not one of them. So a null in configs/common.yaml is
    the contract's own shape, and reporting it would make this case permanently
    red against a correct tree.

    A key position is null and nothing else. The moment configs/common.yaml
    carries a VALUE under spec it has become a second definition site, which is
    precisely what SP-3 exists to prevent, and 10 S5.4.3 makes the consequence
    concrete: L1 sits below L2, so a value there is silently overridden by the
    model file and the operator who edited it sees no effect at all. That is why
    the exemption is written against the value and not against the file name.
    """
    keys = spec_definitions(flat)
    if rel_path == os.path.join("configs", "common.yaml"):
        return [k for k in keys if flat[k] is not None]
    return keys


# --- projecting the model file's gait block --------------------------------
#
# Two projections rather than one, and the difference between them is the point.
# GAIT_PREFIX is shared so the key shape is written once: three cases below read
# this block, and a shape spelled three times is a shape that gets half-changed.
GAIT_PREFIX = "common.spec.gait_limits."


def configured_gait_names(flat):
    """Every gait NAME appearing under gait_limits, whatever leaf it carries.

    Deliberately blind to the leaf key. SP-9 is a question about names, and a
    file that spells one entry `stair_agil: { max_vy_mps: 2.0 }` has two defects
    -- a typo and a wrong leaf. Projecting through max_vx_mps first would make
    the typo invisible as an EXTRA name and report only the missing one, which
    sends the reader looking for a deletion that never happened.
    """
    return {k[len(GAIT_PREFIX):].split(".")[0] for k in flat if k.startswith(GAIT_PREFIX)}


def configured_gait_limits(flat):
    """gait -> max_vx_mps, the projection SP-10 and the value check both need.

    Restricted to the max_vx_mps leaf because SP-10 is a question about that one
    number. An entry missing it does not appear here at all, which is correct for
    a limit check and is why the name check above exists separately rather than
    being derived from this dict.
    """
    return {k[len(GAIT_PREFIX):].rsplit(".", 1)[0]: v
            for k, v in flat.items()
            if k.startswith(GAIT_PREFIX) and k.endswith(".max_vx_mps")}


# --- SP-3: which file is allowed to define spec.* --------------------------

def test_spec_definitions_flags_a_definition_and_not_a_reference():
    """The SP-3 checker itself: a definition is caught, a reference is not.

    Mutation that reddens it: drop the `spec.` arm from spec_definitions and the
    bare-root case stops being reported. Without this case the whole-tree scan
    below would still pass, because no file in the tree carries either shape
    today -- so the scan alone cannot tell a working check from a broken one.

    WHEN THIS GOES RED. Red means the SP-3 checker itself is broken, so the tree scan below is
    reporting nothing for the wrong reason. Fix this before believing that
    scan's green.
    """
    assert spec_definitions({"common.spec.max_vx_mps": 2.0}) == ["common.spec.max_vx_mps"]
    assert spec_definitions({"spec.max_vx_mps": 2.0}) == ["spec.max_vx_mps"]
    # A reference lives in the VALUE. Reporting it would reject a correct
    # p1_motion.yaml, and a permanently red assertion gets loosened until it
    # passes -- CLAUDE.md 3.2 form 2, which is how a real check becomes a
    # rubber stamp.
    assert spec_definitions({"p1_motion.limits.v_max": "${common.spec.max_vx_mps}"}) == []


def test_common_yaml_may_declare_a_null_key_position_but_not_a_value():
    """The L1 exemption, both directions.

    Only the null direction is exercised by the tree today, so without the second
    half the exemption would be a hole nobody could see: a value quietly added to
    configs/common.yaml would ride through on a file-name match.

    Mutation: widen the exemption to the whole file (drop the `is not None`
    filter) and the second assertion goes red.

    WHEN THIS GOES RED. Red means the exemption drifted. Widen it and a value in the L1 file rides
    through unseen; narrow it and the tree scan turns permanently red against
    a correct configs/common.yaml, which ends with somebody deleting the scan.
    """
    common = os.path.join("configs", "common.yaml")
    assert spec_offences(common, {"common.spec.max_vx_mps": None}) == []
    assert spec_offences(common, {"common.spec.max_vx_mps": 2.0}) == ["common.spec.max_vx_mps"]
    # The exemption is keyed to that one file. The same null anywhere else is
    # still a second definition site, because no other layer is allowed to
    # declare a spec key position at all.
    site = os.path.join("configs", "sites", "demo.yaml")
    assert spec_offences(site, {"common.spec.max_vx_mps": None}) == ["common.spec.max_vx_mps"]


def test_no_config_file_outside_models_defines_spec():
    """SP-3 / 12 S12.1 S-3 (1): spec.* may only be written in the L2 model file.

    What this stops is the most realistic wrong edit there is -- somebody raising
    the hard limit at the site, in a file nobody re-reviews, so the vehicle
    exceeds a bound the model layer was supposed to own.

    Mutation (3) of the CFG-CF-3 criterion: add common.spec.max_vx_mps to
    configs/sites/<site>.yaml or to configs/p1_motion.yaml and this goes red. A
    non-null value under spec in configs/common.yaml reddens it too, through the
    narrow L1 exemption in spec_offences.

    Scope, stated because the pass is weaker than it looks: every yaml under
    configs/ except models/ is scanned, but most of those files are still
    comment-only skeletons, so the surface has few keys in it today. The two
    cases above are what establish that the checker works.

    WHEN THIS GOES RED. Red names the file and the keys. The fix is never to relax this: move the key
    into configs/models/m20s.yaml, or -- if the intent was to READ the value --
    write ${common.spec.<key>} on the right-hand side, which is a reference and
    is not reported.
    """
    offenders = {}
    for dirpath, dirnames, filenames in os.walk(CONFIG_ROOT):
        # models/ is the one legal home. Pruned by directory rather than by file
        # name so a second model file added later is covered without an edit.
        dirnames[:] = [d for d in dirnames if os.path.join(dirpath, d) != MODELS_DIR]
        for name in sorted(filenames):
            if not name.endswith((".yaml", ".yml")):
                continue
            path = os.path.join(dirpath, name)
            rel = os.path.relpath(path, ROOT)
            found = spec_offences(rel, _flat_config(path))
            if found:
                offenders[rel] = found
    assert not offenders, (
        f"spec.* defined outside configs/models/: {offenders} -- 11 S9.6 SP-3 "
        "allows it only in the L2 model file"
    )


# --- the model file itself -------------------------------------------------
#
# The cases below read the file as it sits in the repository, before any merge.
# That is the only place two of these properties exist at all: after the overlay
# has run, "which layer wrote this key" and "was this spelled null or omitted"
# are both gone, and they are exactly what SP-3 and assertion A / M are about.
#
# None of them asserts a numeric limit, and none of them may ever start to. The
# five values are blocked on 11 S15 V-01 and inventing one here would be worse
# than useless: the test would then encode a number nobody measured, the config
# would be changed to match it, and the vehicle would run to a bound that came
# out of a test file. What is assertable today is the SHAPE -- which keys exist,
# which layer owns them, and that an uncalibrated one says so out loud.

def test_model_file_writes_only_the_l2_namespace(flat):
    """10 S5.4.3: L2 may write common.spec.* and common.motion.* and nothing else.

    Reused from xbrain/common/config rather than reimplemented, so the file and
    the loader cannot disagree about what L2 means.

    Mutation: add common.safety.t_lat_s to the model file -- it belongs to L3,
    where the whole directory is automatically locked -- and this goes red.

    WHEN THIS GOES RED. Red names the offending key and the allowance it broke. The fix is to move the
    key to the layer that owns it -- common.safety.* to configs/safety/,
    common.geo.* to configs/sites/ -- not to widen L2.
    """
    l2 = [layer for layer in layers.LAYERS if layer.name == "L2"][0]
    # check_namespace raises rather than returning a verdict, which is what the
    # loader does at boot; the test asserts the absence of that raise.
    layers.check_namespace(l2, flat.keys())


@pytest.mark.parametrize("key", FIVE_LIMITS + TERRAIN + ("common.spec.odom.yaw_drift_dps",))
def test_uncalibrated_keys_are_declared_and_null(flat, key):
    """The nine uncalibrated keys are present and explicitly null, never 0.0.

    PRESENT and null, which are two requirements, not one. 10 S5.4.3 makes the
    difference load-bearing: null means "declared, unassigned" and assertion A
    reports it; an absent key does not merge at all and A has nothing to say
    about it. Deleting the key is therefore NOT an equivalent way of saying "not
    calibrated" -- it is the way to make the omission invisible.

    Values are blocked on 11 S15 V-01, which the vendor answered on 2026-07-30
    with "we have not tested the maximum sustained braking deceleration; test it
    yourself if you care about that figure". So there is nothing to write here,
    and writing something anyway is the failure this case exists to catch.

    Mutation: set any one of these to 0.0 and this goes red. It is the mutation
    that matters most on odom.yaw_drift_dps -- see the module docstring for the
    measured failure chain that 0.0 unlocks there.

    WHEN THIS GOES RED. Red on 'absent' means a key was deleted: restore it as null. Red on a value
    means somebody supplied a number that V-01 has not produced: remove it. Do
    not resolve either by editing this list of keys.
    """
    assert key in flat, (
        f"{key} is absent from configs/models/m20s.yaml -- an absent key is not "
        "an unassigned key; assertion A can only report the latter"
    )
    assert flat[key] is None, (
        f"{key} = {flat[key]!r}, expected null. 0.0 reads as 'calibrated to "
        "zero' and passes the assigned check; null is what 10 S5.4.5 requires "
        "until the value exists"
    )


def test_no_numeric_leaf_in_the_model_file_is_zero(flat):
    """Whole-file scan: no numeric leaf equals zero.

    Wider than the parametrised case above on purpose. That one names the keys
    that are uncalibrated TODAY; this one holds for the file as a whole, so a key
    added later -- by a session that never read this test -- cannot arrive
    carrying a zero placeholder. There is no key in this file whose correct value
    is zero: a zero limit, a zero terrain capability and a zero gait speed all
    mean "cannot move", and none of them is a thing the M20S is.

    bool is excluded explicitly because in Python False == 0 and
    isinstance(False, int) is True, so spec.holonomic would otherwise be reported
    the day somebody sets it False for a tracked chassis -- a legal value, and a
    false positive here would get the whole case deleted.

    WHEN THIS GOES RED. Red names every zero-valued leaf. The fix is null, always -- a zero here is
    either an uncalibrated value in disguise or a capability the machine does
    not have, and only the second is worth writing down, in the contract first.
    """
    zeros = sorted(k for k, v in flat.items()
                   if isinstance(v, (int, float)) and not isinstance(v, bool) and v == 0)
    assert not zeros, (
        f"zero-valued leaves in configs/models/m20s.yaml: {zeros} -- write null "
        "for an uncalibrated value; 0.0 passes the assigned check and then "
        "limits the vehicle silently"
    )


def test_holonomic_is_declared(flat):
    """spec.holonomic exists and is a bool, because two gates read it.

    Not a value check. 11 S9.6 gives true for the M20S and a tracked chassis must
    be false, so the value is a model fact and not something to assert; what has
    to hold is that the key is there and is a boolean. 12 S4.7.3 TS-4 and 20
    RNS-ROT-3 both zero the vy component when it is false, and a missing or
    non-boolean value would make that gate decide on a truthiness accident.

    WHEN THIS GOES RED. Red means the key vanished or stopped being boolean. Restore it from 11 S9.6;
    do not let the gate fall back to a truthiness accident on a missing key.
    """
    assert isinstance(flat.get("common.spec.holonomic"), bool), (
        "common.spec.holonomic must be present and boolean -- 12 TS-4 and 20 "
        "RNS-ROT-3 gate lateral motion on it"
    )


# --- SP-9 and SP-10 --------------------------------------------------------
#
# These two were dead letters in the contract for several revisions: 11 S9.6.1.1
# said "folded into freeze" while every one of the five copies of the executor's
# evaluation list stopped at SP-1..SP-7. A misspelled gait name or a gait pinned
# at zero went unreported the whole time. The lists were reconciled in the
# document; the executor is still to be written, so the cases below are for now
# the only thing that actually evaluates either assertion.
#
# They are kept as three cases, not one, because the three failures need
# different fixes: a wrong NAME is edited against 11 S9.2.4, a wrong NUMBER
# against 11 S9.6.1.1, and a non-positive number is not a transcription question
# at all.

def test_gait_limits_key_set_equals_the_contract_closed_set(flat):
    """SP-9: the gait_limits key set equals the gait closed set of 11 S9.2.4.

    Both directions of the difference, because the two failures are different and
    only one of them is obvious. Missing a gait means GT-3 marks that gait
    unknown forever and parks a warn on the HMI; having an extra one means the
    file names a gait the chassis does not have, which is nearly always a typo --
    and a containment-only check would pass one of the two while looking like it
    checked both.

    Mutation (1) of the CFG-CF-3 criterion: delete stair_standard from
    configs/models/m20s.yaml and this goes red. Adding a sixth name reddens it in
    the other direction.

    WHEN THIS GOES RED. Red names the difference in whichever direction it points. If the contract
    genuinely changed, 11 S9.2.4 moves first and this file follows -- never the
    other way round, or configs/ becomes a second source of truth for the set.
    """
    contract = set(contract_gait_closed_set())
    configured = configured_gait_names(flat)
    # Direction one: a name the chassis does not have. Nearly always a typo, and
    # the typo costs twice -- the real gait is now missing as well.
    assert configured - contract == set(), (
        f"gait_limits names not in the 11 S9.2.4 closed set: "
        f"{sorted(configured - contract)}"
    )
    # Direction two: a gait the file forgot. This is the one that fails quietly
    # in the field, because four gaits still work and only the fifth is capped by
    # GT-3's unknown-gait fallback.
    assert contract - configured == set(), (
        f"gaits missing from gait_limits: {sorted(contract - configured)} -- "
        "GT-3 would mark each of them unknown forever"
    )


def test_gait_limits_match_the_contract_carrier_block(flat):
    """The five values are the contract's, digit for digit.

    11 S9.6.1.1 is explicit that 2.0 across the board is not a placeholder but the
    vendor's own answer, and equally explicit that it must not be lowered because
    stair gaits at 2 m/s feel wrong: that would produce a speed limit stricter
    than the machine with no traceable reason, which is the failure mode this
    project keeps rediscovering.

    So the direction this guards is transcription drift in either file. If M-26
    measures the stair gaits and 11 changes, this goes red and the config is
    updated -- rather than the two quietly disagreeing about how fast the robot
    may climb stairs.

    WHEN THIS GOES RED. Red prints both sides. Decide which one is stale before touching either: if
    M-26 measured the stair gaits, the contract changes first and this file is
    re-transcribed from it.
    """
    contract = contract_gait_limits()
    configured = configured_gait_limits(flat)
    # Whole-dict equality, not per-key containment: it is the only form that
    # fails on all three ways the two can drift apart -- a changed number, a gait
    # the contract dropped, and a gait the contract added.
    assert configured == contract, (
        f"gait_limits differ from the 11 S9.6.1.1 carrier block: "
        f"config={configured} contract={contract}"
    )


def test_every_gait_limit_is_a_positive_number(flat):
    """SP-10, lower half: 0 < gait_limits[g].max_vx_mps for every gait.

    This is the half that can be evaluated today, and it is the half that catches
    the bad edit. 11 S9.6.1.1 SP-10 spells out the consequence of a zero: that
    gait cannot move at all and nothing reports it -- the same fail-silent shape
    as a zeroed spec limit, one level down where it is even easier to miss
    because the other four gaits still work.

    Mutation (2) of the CFG-CF-3 criterion: set one gait to 0 and this goes red.
    Setting it to null reddens it too, through the isinstance branch: unlike the
    five whole-vehicle limits, a gait limit has a value today, so null here would
    mean somebody deleted a known number rather than declined to invent one.

    WHEN THIS GOES RED. Red names the gait and the value it carries. There is no legitimate zero, null
    or negative here; if a gait's real limit is unknown, that is a contract
    question for 11 S9.6.1.1 and not something to encode as zero.
    """
    # Three rejections in one condition, because all three arrive as "this gait
    # cannot move": a non-number (null, or a string somebody quoted), a bool
    # (True would otherwise pass `> 0` and read as 1 m/s), and anything <= 0.
    bad = {gait: value for gait, value in sorted(configured_gait_limits(flat).items())
           if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0}
    assert not bad, (
        f"gait limits that are not a positive number: {bad} -- a gait limited to "
        "zero cannot move and reports nothing"
    )


def test_sp10_upper_half_catches_a_gait_above_spec():
    """SP-10, upper half, on synthetic input: a gait faster than spec is caught.

    Kept separate from the file because the file cannot exercise it: with
    spec.max_vx_mps null the comparison must not run at all. Without this case
    the upper half would be a branch that has never once evaluated, which is an
    assertion that has never been red (CLAUDE.md 3.3).

    Mutation: change the offending 2.5 below to 1.5 and the expected list stops
    matching. Change the comparison in sp10_upper_half from > to >= and the
    equality case at the end goes red -- equality is legal, since a gait limit
    equal to the vehicle limit is exactly what the contract records today.

    WHEN THIS GOES RED. Red means the comparison is wrong, not that the configuration is: this case
    never reads configs/. It is the only place the comparison runs at all while
    common.spec.max_vx_mps stays null.
    """
    state, bad = sp10_upper_half(2.0, {"flat": 2.0, "stair_agile": 2.5})
    assert state == "evaluated"
    assert bad == ["stair_agile"]
    # Equality is not a violation: every gait sits at 2.0 in the contract, and
    # rejecting it would refuse to boot on the contract's own values.
    assert sp10_upper_half(2.0, {"flat": 2.0}) == ("evaluated", [])


def test_sp10_upper_half_on_the_shipped_file(flat):
    """SP-10's upper half against the real file, whichever branch it lands in.

    Today it lands in the unevaluable branch, and that is a real requirement
    rather than bookkeeping. 11 S9.6 SP-1 requires the "present and numeric" test
    before any comparison, and the reason is concrete: `2.0 <= None` raises
    TypeError, which would kill xbrain-config-freeze.service with a traceback
    instead of the key path assertion A exists to print. Refusing to boot is
    right either way; only one of the two tells the field which key to fill.

    The expected branch is DERIVED from the file rather than hard-coded, so this
    case keeps meaning something after V-01 lands. Pinning it to "unevaluable"
    would make it fail on the day the value is finally calibrated -- which reads
    as the calibration breaking the test, and gets the case deleted rather than
    read.

    Mutation: drop the isinstance guard in sp10_upper_half and this raises
    TypeError instead of returning, so the case goes red rather than passing on a
    crash.

    WHEN THIS GOES RED. Red on the state means the guard changed behaviour; red on the list means a
    gait is configured faster than the vehicle limit, which hides a filling
    mistake behind a bound that can never be reached.
    """
    spec_max = flat["common.spec.max_vx_mps"]
    state, bad = sp10_upper_half(spec_max, configured_gait_limits(flat))
    expected = "unevaluable_assertion_a_first" if spec_max is None else "evaluated"
    assert state == expected, (
        f"common.spec.max_vx_mps is {spec_max!r} so SP-10's upper half should be "
        f"{expected!r}, got {state!r}"
    )
    # Holds in both branches, for two different reasons: while the right operand
    # is null nothing can violate it, and once it is calibrated every gait limit
    # has to fit underneath it or the vehicle limit is not the ceiling it claims
    # to be.
    assert bad == [], f"gait limits above common.spec.max_vx_mps: {bad}"


# --- the extractors, so a silent contract change cannot go unnoticed -------

def test_contract_extractors_return_something(flat):
    """Both contract extractors find their anchors and agree on the gait names.

    An extractor that quietly returns nothing turns every check above it green,
    which is the failure the enums loader in xbrain/common/enums was rewritten to
    avoid after a row filter dropped nine of the twelve task states. The
    functions assert non-emptiness internally; this case pins the third property
    the checks above depend on but never state -- that 11 S9.2.4 and 11 S9.6.1.1
    name the SAME five gaits. If they ever diverge, both SP-9 and the value check
    would be measuring against different contracts and only this case would say
    so.

    Mutation: point _section at a heading fragment that does not exist and the
    AssertionError names the missing anchor instead of the set silently emptying.

    WHEN THIS GOES RED. Red means 11 changed under the extractors. Re-read the two sections before
    adjusting anything here: an extractor bent until it parses again is how a
    contract change gets absorbed instead of noticed.
    """
    closed_set = contract_gait_closed_set()
    carrier = contract_gait_limits()
    assert set(closed_set) == set(carrier), (
        f"11 S9.2.4 gait set {sorted(closed_set)} does not match the S9.6.1.1 "
        f"carrier block {sorted(carrier)} -- the contract disagrees with itself"
    )
