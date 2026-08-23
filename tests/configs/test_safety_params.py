"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_safety_params.py
Brief: CFG-CF-4 -- L3 safety values (brake.yaml d_safe_m + clock.yaml) landed and guarded

Description:
WHAT THIS GUARDS. CFG-CF-4 lands the L3 safety-layer VALUES that CFG-CF-2 could
only declare as null positions: the seven clock keys of 11 S1.5.5 into
configs/safety/clock.yaml, and common.safety.d_safe_m = 1.00 into
configs/safety/brake.yaml (99 U67 promoted the key path, 99 U54 rule 3 fixed the
value). Every value here is transcribed from 11 S1.5.5 or the 10 S5.4.5 table; no
number is invented (CLAUDE.md 3.1). This file is the mutation-tested acceptance
for that config content -- each case pairs a positive assertion with a mutant that
turns it red, because a config that only ever passes is a config nobody proved
(CLAUDE.md 3.3).

WHY THE ASSERTIONS ARE EVALUATED IN ISOLATION HERE, not through the freeze
executor. The startup assertions themselves are owned by other Phase 0 items:
assertion G / SP-5 by CFG-FZ-7, assertion E by CFG-FZ-5, assertion N and MR-1 by
CFG-FZ-15, assertions A/M by CFG-FZ-3. None of those executors exist yet, and
building one here would collide with those items. So this file evaluates the SP-5
predicate and the assertion-E intersection as pure, self-contained checks over the
deployed config -- which is exactly what the CFG-CF-4 row asks for when it says the
SP-5 clauses "are evaluable on assertion G in isolation". The registry-driven
versions remain CFG-FZ-5/7's deliverable and source their operands from the docs.

TWO THINGS ARE BLOCKED AND ARE NOT FAKED GREEN HERE.

  * SP-5 clause 3 (0 < a_mps2 <= common.spec.max_decel_mps2). max_decel_mps2 is
    uncalibrated (null, pending T-DECEL / 11 M-22). On the real config this clause
    is NOT evaluable, so the case for it SKIPS rather than inventing a bound, and
    enforces the clause automatically once max_decel lands. The predicate itself is
    still proved complete in isolation, with an injected bound that is a test input,
    never written into any config.

  * The PROVENANCE value-domain validation named by the CFG-CF-4 mutation 3
    ("change a key's PROVENANCE to an off-table value => validation must go red").
    The domain's only cited source is "SET-01 gate 3", and SET-01 has ZERO hits in
    the formal volumes (grep-confirmed); the domain has no authoritative definition
    point anywhere. CFG-DC-5 records this as a dangling convention that needs a USER
    ruling and forbids the implementer from self-drafting it, and configs/models/
    m20s.yaml already set the precedent of not propagating the unratified tag.
    Writing a domain checker here would either self-draft the domain or hardcode a
    second source of truth -- both forbidden. So the domain half of CFG-CF-4
    sub-task 1 is BLOCKED. What this file DOES do is the "keep" half: it proves the
    existing PROVENANCE tags survive this edit (mutant: delete one => red). Off-table
    membership is not decidable until CFG-DC-5 lands the domain.

DECLARED SCAN FACE. Every case reads the SOURCE files under configs/, never the
/run/xbrain/resolved snapshot (which does not exist before freeze). tests/ sits
outside scripts/lint/no_config_source_read.py's SCAN_DIRS (xbrain, ros2_ws,
services) precisely so a test may name the config source; the same exemption that
tests/configs/test_m20s_spec.py relies on.
"""

import os
import sys

import pytest
import yaml

# ROOT is three levels up from tests/configs/<this file>. It goes on sys.path so
# the shared config library imports exactly as the runtime imports it, not through
# a test-only shim that could drift from what the freeze service actually loads.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

# merge.flatten is the loader's OWN flattener, reused rather than reimplemented:
# it treats an empty map as a leaf, which is the behaviour the freeze pass sees, so
# a branch someone emptied out still shows up here instead of vanishing silently.
from xbrain.common.config import merge  # noqa: E402

# INF-TS-1 三档 marker. 本文件是纯静态/元检查(读文件与仓库状态),
# 不碰任何硬件, 故 no_device -- 2026-08-23 从 legacy 未标记名单迁出.
pytestmark = pytest.mark.no_device

# The three source files under test. Sources, not the resolved snapshot -- see the
# module docstring for why a test is allowed to name them.
CONFIGS = os.path.join(ROOT, "configs")
BRAKE_YAML = os.path.join(CONFIGS, "safety", "brake.yaml")
CLOCK_YAML = os.path.join(CONFIGS, "safety", "clock.yaml")
COMMON_YAML = os.path.join(CONFIGS, "common.yaml")


# --- reading the files under test ------------------------------------------

def _flat(path):
    """One yaml file flattened to dotted leaf paths.

    safe_load, so a tag in the file cannot construct a Python object: the freeze
    service parses these same files and a loader that could be talked into
    instantiating something would make the config root an execution surface. A
    comment-only file maps to {} rather than None so callers need no None branch.
    """
    with open(path, encoding="utf-8") as handle:
        tree = yaml.safe_load(handle)
    return merge.flatten(tree if tree is not None else {})


def brake_leaf(path):
    """One leaf value out of brake.yaml, or MISSING if the key is absent.

    MISSING is a distinct sentinel, not None: 3.1 draws a hard line between a key
    that is absent and a key present with a null value, and a helper that returned
    None for both would erase exactly the distinction the safety rules turn on.
    """
    return _flat(BRAKE_YAML).get(path, MISSING)


def clock_leaf(path):
    """One leaf value out of clock.yaml, or MISSING if absent (see brake_leaf)."""
    return _flat(CLOCK_YAML).get(path, MISSING)


def common_leaf(path):
    """One leaf value out of common.yaml, or MISSING if absent (see brake_leaf)."""
    return _flat(COMMON_YAML).get(path, MISSING)


#: Absent-key sentinel. A plain object() so it can never be equal to a real value
#: and can never be confused with a null a file legitimately wrote.
MISSING = object()


# ---------------------------------------------------------------------------
# clock.yaml -- the seven keys of 11 S1.5.5, transcribed with their values
# ---------------------------------------------------------------------------

#: The clock block of 11 S1.5.5, verbatim. Held here as the criterion the deployed
#: file must match, the same way test_m20s_spec.py holds the five SP-1 limits: the
#: mutant is applied to the FILE and this constant is what notices the divergence.
#: Anchor for a hand check: 11 S1.5.5, the fenced yaml under "配置项
#: (/opt/xbrain_v6/configs/safety/clock.yaml)" (NUM-4: section anchor, no line no).
CLOCK_CONTRACT = {
    "common.safety.clock.sync_timeout_s": 5.0,
    "common.safety.clock.offset_threshold_ms": 20.0,
    "common.safety.clock.ref_max_age_s": 5.0,
    "common.safety.clock.unsynced_max_speed_mps": 0.5,
    # rtc_trusted: 11 S1.5.5 gives the literal false, annotated hardware-pending.
    # false is the documented fail-safe (an unconfirmed RTC is not trusted), not an
    # invented value; writing true would loosen toward trusting an unverified clock.
    "common.safety.clock.rtc_trusted": False,
    "common.safety.clock.step_notify_ms": 100.0,
    # allow_unsynced_motion: field-forced false; a hot-updatable true would be a
    # remote channel to lift the clock safety constraint (S1.5.5 gate 1).
    "common.safety.clock.allow_unsynced_motion": False,
}


def test_clock_declares_exactly_the_seven_contract_keys():
    """clock.yaml declares the S1.5.5 seven -- no key missing, none invented.

    Set equality, not a subset test, on purpose. A subset test would pass a file
    that grew an eighth clock key nobody authorised; a superset test would pass a
    file that dropped one. The layer only earns its "landed" status when the two
    sets are identical.

    Mutation run: delete any clock line from clock.yaml, or add an eighth key =>
    this goes red naming the symmetric difference.
    """
    got = {k for k in _flat(CLOCK_YAML) if k.startswith("common.safety.clock.")}
    want = set(CLOCK_CONTRACT)
    assert got == want, "clock.yaml key set diverges from 11 S1.5.5: %s" % (
        sorted(got ^ want))


@pytest.mark.parametrize("path,expected", sorted(CLOCK_CONTRACT.items()))
def test_clock_value_matches_contract(path, expected):
    """Every clock key carries exactly its 11 S1.5.5 value.

    One case per key so pytest names the offending key in the test id, readable
    from the summary line without opening the report.

    Mutation run: set common.safety.clock.unsynced_max_speed_mps to 0.6 (or flip
    allow_unsynced_motion to true) => the matching case goes red naming the path.
    """
    # sorted() on the parametrize source keeps the case ids stable between runs: an
    # unordered report of the same defect reads as a new defect each run, and the
    # first thing doubted is then the checker rather than the config.
    value = clock_leaf(path)
    assert value is not MISSING, "clock.yaml is missing %s (11 S1.5.5)" % path
    # Exact match. For the two booleans this also catches a truthy 1 sneaking in as
    # a boolean; for the floats the S1.5.5 values are exact decimals, so == is right
    # and no tolerance is wanted -- a tolerance would let a wrong-but-close value in.
    assert value == expected, "%s = %r, 11 S1.5.5 says %r" % (path, value, expected)
    # bool is an int subclass in Python, so 0.5 == False would be False but
    # True == 1 is True; guard the two flags so a numeric 1/0 cannot pass as the
    # boolean the contract means.
    if isinstance(expected, bool):
        assert isinstance(value, bool), "%s must be a real boolean, got %r" % (path, value)


# ---------------------------------------------------------------------------
# d_safe_m -- L3 value 1.00 in brake.yaml, L1 null slot in common.yaml
# ---------------------------------------------------------------------------

def test_d_safe_m_value_is_one_meter_in_brake_yaml():
    """common.safety.d_safe_m = 1.00 lands in the L3 safety file (99 U67 / U54).

    The 10 S5.4.5 table names safety/brake.yaml as the single source for this leaf
    and fixes the value at 1.00 m (U54 rule 3). This is the right operand of
    assertion N and the value MR-1 pins margin_rot_m to.

    Mutation run: change d_safe_m to 0.90 in brake.yaml => red. (A change to 0.30
    alongside margin_rot_m is MR-1's own M-MR1-c mutant, owned by CFG-FZ-15.)
    """
    value = brake_leaf("common.safety.d_safe_m")
    assert value is not MISSING, "brake.yaml must define common.safety.d_safe_m (10 S5.4.5)"
    # Exact 1.00, no tolerance: this is a fixed decision (U54 rule 3), not a measured
    # quantity with error bars, so any drift from it is a defect, not rounding. yaml
    # parses 1.00 as the float 1.0, and 1.0 == 1.00 holds, so the compare is safe.
    assert value == 1.00, "common.safety.d_safe_m = %r, U54 rule 3 fixes it at 1.00 m" % value


def test_d_safe_m_is_split_l1_null_slot_over_l3_value():
    """The layering itself: L1 declares the slot null, L3 supplies the value.

    This is the CFG-CF-4-specific shape and is not what test_common_skeleton checks
    (that file only proves the L1 slot exists). If common.yaml ever LANDED a value
    for d_safe_m it would become a second source for the same leaf, which is the
    exact duplication assertion B exists to refuse.

    Mutation run: put 1.00 on the common.yaml d_safe_m line => red (L1 must stay a
    null position); or delete the brake.yaml value => red (L3 must supply it).
    """
    l1 = common_leaf("common.safety.d_safe_m")
    assert l1 is not MISSING, "common.yaml must declare the d_safe_m slot (CFG-CF-2)"
    assert l1 is None, "L1 common.yaml must leave d_safe_m null; the value belongs to L3"
    assert brake_leaf("common.safety.d_safe_m") == 1.00


# ---------------------------------------------------------------------------
# SP-5 / S-4 -- evaluated in isolation (assertion G executor is CFG-FZ-7)
# ---------------------------------------------------------------------------

def sp5_holds(t_lat_s, k, a_mps2, max_decel_mps2):
    """The SP-5 predicate: the three clauses 11 S9.6 SP-5 and 12 S12.1 S-4 share.

    S-4 is a superset of SP-5 (12 S12.1: only t_lat_s>=0.4 is verbatim identical,
    the other two are semantically shared, and both assertions must hold at once).
    This function is exactly the shared conjunction, so a mutant that violates it
    turns BOTH assertions red -- which is what CFG-CF-4 mutation 1 requires.

      clause 1: t_lat_s      >= 0.4
      clause 2: k            >= 1.0
      clause 3: 0 < a_mps2   <= max_decel_mps2
    """
    return (t_lat_s >= 0.4) and (k >= 1.0) and (0 < a_mps2 <= max_decel_mps2)


def test_sp5_predicate_is_complete_each_clause_has_a_red_mutant():
    """The predicate passes on a valid tuple and fails on every single-clause break.

    Evaluated in isolation with injected operands. The 3.0 fed as max_decel_mps2
    here is a TEST INPUT that exercises clause 3 -- it is never written into any
    config, so this does not calibrate the uncalibrated max_decel (see the clause-3
    case below). A predicate proved only on the positive tuple is the shell-
    implementation failure of CLAUDE.md 3.3: a three-line "return True" would pass
    it. Each mutant below is the necessary counterexample for one clause.
    """
    # Positive: a tuple that satisfies all three clauses.
    assert sp5_holds(0.4, 1.5, 2.5, 3.0)
    # Mutant for clause 1 -- and the exact value CFG-CF-4 mutation 1 names.
    assert not sp5_holds(0.2, 1.5, 2.5, 3.0)
    # Mutant for clause 2: k below 1.0.
    assert not sp5_holds(0.4, 0.9, 2.5, 3.0)
    # Mutant for clause 3, lower bound: a_mps2 == 0 is the fail-silent zero 3.1 warns
    # of -- it must NOT pass as "assigned".
    assert not sp5_holds(0.4, 1.5, 0.0, 3.0)
    # Mutant for clause 3, upper bound: braking harder than the chassis maximum.
    assert not sp5_holds(0.4, 1.5, 3.5, 3.0)


def test_sp5_and_s4_shared_clauses_hold_on_the_real_config():
    """The two determinable clauses hold on the deployed brake.yaml.

    Clauses 1 and 2 read only t_lat_s and k, both landed, so they are decided today;
    clause 3 needs max_decel and is handled by the next case. This is the case
    CFG-CF-4 mutation 1 drives: setting common.safety.t_lat_s to 0.2 in brake.yaml
    turns it red, and because sp5_holds is the shared conjunction of SP-5 and S-4,
    both assertions go red together, which is what the row demands.
    """
    # Read from brake.yaml directly rather than from a merged product: this item
    # owns the L3 source, and reading the source is what lets the t_lat_s->0.2
    # mutation on the file surface here. A merged tree would also work today but
    # would couple this case to the whole layer stack for no added coverage.
    t_lat_s = brake_leaf("common.safety.t_lat_s")
    k = brake_leaf("common.safety.brake.k")
    assert t_lat_s is not MISSING and k is not MISSING, "brake.yaml must define t_lat_s and k"
    # Clause 1 and clause 2 only. Writing them out rather than calling sp5_holds so
    # the failure message names WHICH clause broke; sp5_holds returns a bare bool.
    assert t_lat_s >= 0.4, "SP-5/S-4 clause 1: t_lat_s = %r must be >= 0.4" % t_lat_s
    assert k >= 1.0, "SP-5/S-4 clause 2: k = %r must be >= 1.0" % k


def test_sp5_clause3_holds_when_max_decel_lands_else_records_it_blocked():
    """Clause 3 on the real config, or a recorded skip while max_decel is null.

    max_decel_mps2 is uncalibrated (null, pending T-DECEL / 11 M-22). Inventing a
    bound to make clause 3 evaluable is the single thing 3.1 forbids most sharply,
    so this case SKIPS while the operand is null and enforces the clause the moment
    it lands -- it never passes vacuously and it never fabricates the bound.

    Mutation run (future, once max_decel lands): set a_mps2 above max_decel => red.
    Mutation run (today): the skip is the honest state; a_mps2 is still checked > 0.
    """
    a = brake_leaf("common.safety.brake.a_mps2")
    # a_mps2 is a working assumption (2.5, PROVENANCE assumed) but it is present and
    # positive today, so the "0 < a_mps2" half of clause 3 is already meaningful; it
    # is only the upper bound that waits on max_decel.
    assert a is not MISSING and a > 0, "brake.yaml a_mps2 must be a positive value"
    max_decel = common_leaf("common.spec.max_decel_mps2")
    # None is the declared-but-null case; MISSING would be the key-absent case. Either
    # way the upper bound is unknown, so skip rather than compare against a non-number.
    if max_decel is None or max_decel is MISSING:
        pytest.skip("common.spec.max_decel_mps2 uncalibrated (null); SP-5 clause 3 blocked, 11 M-22")
    assert 0 < a <= max_decel, "SP-5/S-4 clause 3: a_mps2 %r must be in (0, max_decel %r]" % (a, max_decel)


# ---------------------------------------------------------------------------
# Assertion E -- safety namespaces intersect the hot-update whitelist == empty
# (registry-driven executor is CFG-FZ-5; evaluated in isolation here)
# ---------------------------------------------------------------------------

#: The five safety namespaces of 10 S5.4.4 assertion E. A key falls "under" one of
#: these when it equals the namespace or is a dotted child of it. Held local to this
#: isolation check; CFG-FZ-5 is the executor that sources the left operand from the
#: 10 S5.4.5 table and the right operand from the 11 S7.6 whitelist.
SAFETY_NAMESPACES = (
    "common.safety",
    "common.spec",
    "common.motion.profiles",
    "common.qos",
    "common.fence",
)

#: A representative hot-update whitelist. 10 S5.4.4 / 11 S7.6 name only log level,
#: debug switches, the ASR dictionary, speech presets and suspicion rules -- none of
#: which is under a safety namespace. These stand in for the S7.6 table so the
#: intersection has something real to be empty against; the authoritative parse is
#: CFG-FZ-5's. The point of the test is that a safety key can never be added here.
REPRESENTATIVE_WHITELIST = (
    "common.log.level",
    "debug.flags",
    "asr_dict",
    "speech_presets",
    "suspicion_rules",
)


def _safety_keys_in(whitelist):
    """Every whitelist entry that falls under a safety namespace.

    Membership is prefix-based on dotted paths, so common.safety.clock.rtc_trusted
    is caught by the common.safety namespace while a lookalike such as
    common.safetybelt (no dot) is not -- the boundary is a path separator, not a
    string prefix, which is why the child test uses namespace + ".".
    """
    hits = []
    for key in whitelist:
        for ns in SAFETY_NAMESPACES:
            if key == ns or key.startswith(ns + "."):
                hits.append(key)
                break
    return hits


def test_assertion_E_holds_no_declared_safety_key_is_whitelisted():
    """The real safety keys and the whitelist are disjoint (assertion E, green).

    The left operand is the ACTUAL set of safety keys this item deploys -- every
    leaf under common.safety in brake.yaml and clock.yaml -- so this is not a
    hypothetical: it proves the keys CFG-CF-4 just landed are all outside the
    whitelist. A detector that returned [] unconditionally would pass this, which is
    why the next case feeds it a key it MUST flag.

    Mutation run: add "common.safety.clock.allow_unsynced_motion" to
    REPRESENTATIVE_WHITELIST => this goes red naming the intersection.
    """
    # The left operand is built from the files this item deploys, so a passing
    # result is a statement about the real keys, not a hypothetical set. Both L3
    # safety files feed it because a safety key that slipped into the whitelist
    # could originate in either one.
    declared_safety = [k for k in list(_flat(BRAKE_YAML)) + list(_flat(CLOCK_YAML))
                       if _safety_keys_in([k])]
    # Sanity: the config really did declare safety keys, else "disjoint" is vacuous.
    # Without this line a config that declared NO safety keys would pass the
    # intersection test trivially -- the empty-set-passes-everything shape.
    assert declared_safety, "expected common.safety.* keys in the L3 files"
    intersection = set(declared_safety) & set(REPRESENTATIVE_WHITELIST)
    assert not intersection, "assertion E: safety keys in the whitelist: %s" % sorted(intersection)


def test_assertion_E_catches_allow_unsynced_motion_in_the_whitelist():
    """CFG-CF-4 mutation 2, executed: whitelisting allow_unsynced_motion is caught.

    allow_unsynced_motion is common.safety.clock.allow_unsynced_motion, a key
    clock.yaml actually declares, so it is genuinely under a safety namespace and
    assertion E must reject it from any whitelist. The mutation is the whitelist
    operand (11 S7.6, not a file this item owns), so it is applied as an input here
    rather than as a file edit: a whitelist WITH the key must produce a non-empty
    intersection. This is the necessary red-making counterexample for the detector.
    """
    allow_key = "common.safety.clock.allow_unsynced_motion"
    # It is really one of our declared safety keys, not a straw path.
    assert allow_key in _flat(CLOCK_YAML), "clock.yaml must declare allow_unsynced_motion"
    mutated = REPRESENTATIVE_WHITELIST + (allow_key,)
    hits = _safety_keys_in(mutated)
    assert allow_key in hits, "assertion E failed to catch a safety key in the whitelist"


# ---------------------------------------------------------------------------
# PROVENANCE -- the "keep" half; the domain half is BLOCKED (CFG-DC-5)
# ---------------------------------------------------------------------------

#: The three PROVENANCE tags brake.yaml already carried before CFG-CF-4, plus the
#: one this item adds for d_safe_m. This case guards that the edit did not drop a
#: tag (sub-task 1, the "keep" half). It deliberately does NOT check the tag VALUE
#: against a domain: the domain's cited source SET-01 has zero hits in the formal
#: volumes and CFG-DC-5 requires a user ruling before any domain checker exists, so
#: an off-table-value check (CFG-CF-4 mutation 3) is not decidable and is reported
#: BLOCKED rather than faked.
EXPECTED_PROVENANCE_LINES = (
    "t_lat_s: 0.4",
    "a_mps2: 2.5",
    "k: 1.5",
    "d_safe_m: 1.00",
)


def test_brake_provenance_tags_are_preserved():
    """Each landed brake.yaml value still carries a PROVENANCE tag (the keep half).

    Raw-text scan, because the tag lives in a trailing comment that yaml.safe_load
    discards. Falsifiable: deleting the "# PROVENANCE:" from any of the four value
    lines turns this red. What it cannot do -- and does not pretend to -- is judge
    whether the tag's value is inside the allowed domain; that is BLOCKED on the
    dangling SET-01 reference and CFG-DC-5's pending user ruling.

    Mutation run (the keep half): strip "# PROVENANCE: assumed" from the a_mps2
    line => red. Mutation run named by the row (the domain half, CFG-CF-4 mutation
    3): NOT achievable here -- there is no authoritative domain to be off of.
    """
    with open(BRAKE_YAML, encoding="utf-8") as handle:
        raw = handle.read()
    for value_line in EXPECTED_PROVENANCE_LINES:
        # Find the value line, then require a PROVENANCE marker on the same physical
        # line. Assembling the marker from parts so this criterion is not inside its
        # own scan face -- a check whose text contains the string it greps for can
        # never reach zero hits (CLAUDE.md 3.2 form 3).
        marker = "# " + "PROVENANCE" + ":"
        line = next((ln for ln in raw.splitlines() if value_line in ln), None)
        assert line is not None, "brake.yaml no longer has the %r line" % value_line
        assert marker in line, "brake.yaml %r lost its PROVENANCE tag" % value_line
