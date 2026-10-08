"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_charge_trigger.py
Brief: 15 S8.2 charge criteria A / B and S8.2A, with the contract's own numbers

Description:
xbrain/p3_task/charge/trigger.py had NO criteria at all before 2026-10-09 --
no test file named it, and tests/meta/test_unwired_modules.py did not catch
that because the module stem "trigger" is a common word that appears all over
the tree (NEXT T12 records that blind spot). An unwired module with no
criteria is how a file ends up implementing eight conditions that the contract
never defined, which is exactly what had happened.

So these cases pin the two criteria 15 S8.2 actually defines, their OR, the
S8.2A tier above them, and the three absent-input states.

VALUES USED HERE AND WHY THEY ARE NOT "test data pulled from nowhere": every
number comes from 15 S12's charge: block (soc_threshold_pct 30 /
reserve_ratio 0.20 / detour_factor 1.3 / soc_critical_pct 15, the last one
commented-out there because the key is omitted by default). They are passed
in explicitly rather than read from configs/ because the module takes them at
construction (CLAUDE.md S3.1) and because a criterion that read the live
config would start failing the day someone re-tunes a site -- it would be
testing the config, not the logic.

WHAT THESE CASES DO NOT PROVE:
  - that anything calls evaluate(). It still has no production caller; that
    is BIZ-P3-15's remaining half and is registered in docs/NEXT.md.
  - that "unknown SoC" (no seated battery, CHG-12b) is handled CORRECTLY.
    15 S8.2 does not define that case. The cases below pin what the code
    DOES, and say so; they are not evidence that the behaviour is right.
  - that the thresholds in configs/ are these values. Nothing here reads
    configs/; 15 S12 is the source and tests/configs/ is where landed
    values get checked.

MUTANTS, all eight run 2026-10-09, all eight red (CLAUDE.md S3.3; caches
cleared before every run per S7.2.1):

  criterion B hard-codes 0.8 instead of (1 - reserve_ratio)
  criterion B hard-codes 1.3 instead of detour_factor
  the critical tier uses `if thr.soc_critical_pct:` (truthiness)
  criterion A uses `<=` instead of `<`
  criterion B uses `<=` instead of `<`
  S8.2A returns stop_in_place=False
  criterion B's whole branch is skipped
  criterion B reports "CR-11" instead of "CHG-11"

The `<=` mutant on B SURVIVED the first round, and the reason is worth
recording: none of the cases stood on B's boundary. Every one of them was
comfortably on one side, so flipping the operator changed no outcome. The
fix was not a new assertion on an existing case but a new case built AT
the boundary -- and built with neutralised coefficients, because with the
real 0.20 / 1.3 the boundary point is 8.0/1.3, which binary floating point
cannot represent; the comparison would land on one side by an epsilon
nobody chose and both operators would pass. See that case's own docstring.

WHAT THIS FILE REPLACED: six cases in test_batch_e.py (test_cr1_low,
test_cr2_urgent, test_cr3_*, test_cr4_*, test_cr8_*, and a no-trigger
case). They were green for as long as they existed and were the only
criteria that had ever touched trigger.py -- over a specification that
does not exist. That file now carries a block explaining why they were
deleted rather than updated.
"""

# dataclasses for building threshold sets; pytest for parametrize.
import pytest

from xbrain.p3_task.charge.trigger import ChargeThresholds, evaluate

#: Pure computation over frozen dataclasses. No device, no clock, no IO.
pytestmark = pytest.mark.no_device

#: 15 S12 charge: block, with the optional critical tier OFF -- that is the
#: default state the volume rules for it (key omitted, not null).
_THR_NO_CRITICAL = ChargeThresholds(
    soc_threshold_pct=30.0,
    reserve_ratio=0.20,
    detour_factor=1.3,
)

#: Same, with the critical tier configured at the value 15 S12 shows in its
#: commented-out line. 15 S8.2A CR-2 requires 0 < critical < threshold.
_THR_WITH_CRITICAL = ChargeThresholds(
    soc_threshold_pct=30.0,
    reserve_ratio=0.20,
    detour_factor=1.3,
    soc_critical_pct=15.0,
)

#: A dock far enough away that criterion B cannot fire, used whenever a case
#: is about criterion A. 0.1 km of detoured distance against any plausible
#: RemainMile keeps B silent -- stated explicitly because a case that fired
#: BOTH criteria and asserted "triggered" would pass without testing either.
#:
#: Named rather than inlined for one reason: if criterion B's arithmetic ever
#: changes so that 20 km of range no longer covers a 0.1 km dock, every case
#: that depends on B being silent breaks at once and the fix is in one place.
#: Four separate inline literals would instead leave some cases accidentally
#: testing B while claiming to test A, and they would still be green.
_B_SILENT = {"remain_mile_km": 20.0, "d_to_nearest_dock_km": 0.1}


def test_criterion_a_fires_below_the_single_threshold():
    """A: soc_pct < soc_threshold_pct -> CHG-10, and NOT stop_in_place.

    The code is asserted, not just the boolean: an implementation that
    reported every trigger as the critical tier would satisfy a bare
    `triggered is True` while telling the caller not to drive.
    """
    # 29.9 and not 29: one tick below the threshold is where an off-by-one
    # in the comparison shows up, and 15 S12 lands soc_threshold_pct as an
    # integer 30 while the chassis reports a float.
    d = evaluate(soc_pct=29.9, thr=_THR_NO_CRITICAL, **_B_SILENT)
    assert d.triggered is True
    assert d.trigger_code == "CHG-10"
    assert d.stop_in_place is False


def test_criterion_a_does_not_fire_at_the_threshold_itself():
    """Strict `<`: exactly 30.0 does not trigger.

    15 S8.2 writes `soc_pct < 30`. The boundary matters because this is the
    value an operator sets, and a `<=` would make a robot parked at exactly
    30% request charging on every tick.
    """
    d = evaluate(soc_pct=30.0, thr=_THR_NO_CRITICAL, **_B_SILENT)
    assert d.triggered is False
    assert d.trigger_code == ""


def test_criterion_b_fires_when_the_dock_is_too_far_even_at_high_soc():
    """B: usable range < detoured distance -> CHG-11, at 80% SoC.

    This is the criterion the previous implementation lacked entirely, and
    the case is built at a HIGH SoC on purpose: it is the situation 15 S8.2's
    closing paragraph describes verbatim, "30% 够不够, 取决于最近的桩有多远".
    At 80% SoC criterion A is nowhere near firing, so a trigger here can only
    be B.

    Arithmetic, spelled out so the next reader does not have to trust it:
      usable = 10.0 * (1 - 0.20) = 8.0 km
      needed =  7.0 * 1.3        = 9.1 km
      8.0 < 9.1  -> fires
    """
    d = evaluate(soc_pct=80.0, remain_mile_km=10.0,
                 d_to_nearest_dock_km=7.0, thr=_THR_NO_CRITICAL)
    assert d.triggered is True
    assert d.trigger_code == "CHG-11"
    assert d.stop_in_place is False


def test_criterion_b_stays_silent_when_the_range_is_enough():
    """The reverse of the case above, one step the other way.

      usable = 10.0 * 0.8 = 8.0 km
      needed =  6.0 * 1.3 = 7.8 km
      8.0 < 7.8 is false -> silent

    Without this case, "B fires" is satisfied by an implementation that
    always fires B.
    """
    d = evaluate(soc_pct=80.0, remain_mile_km=10.0,
                 d_to_nearest_dock_km=6.0, thr=_THR_NO_CRITICAL)
    assert d.triggered is False


def test_criterion_b_does_not_fire_at_exact_equality():
    """Strict `<` on criterion B: usable == needed does NOT trigger.

    15 S8.2 writes the comparison as `RemainMile * 0.8 < d * 1.3`. At exact
    equality the remaining range is precisely the detoured distance, which
    the contract treats as still reachable -- the margin for "precisely" is
    what reserve_ratio already is, so a `<=` here would be a second reserve
    stacked on the first.

    *** The threshold values here are NOT 15 S12's (reserve_ratio 0.0 and
    detour_factor 1.0 instead of 0.20 / 1.3). That is deliberate and is the
    point of the case: with the real coefficients, landing exactly on the
    boundary needs d = 8.0 / 1.3, which is not representable in binary
    floating point -- the comparison would come out strictly one side or the
    other by an epsilon nobody chose, and the case would pass for both `<`
    and `<=`. With the coefficients neutralised, usable and needed are both
    exactly 5.0 and the operator is the only thing being tested.

    This case was added after a mutant survived: changing `<` to `<=` left
    every other case in this file green, because none of them stood on the
    boundary (CLAUDE.md S7.2.1 -- before concluding a mutant is equivalent,
    check whether the cases actually construct the situation being tested).
    """
    thr = ChargeThresholds(soc_threshold_pct=30.0, reserve_ratio=0.0,
                           detour_factor=1.0)
    # usable = 5.0 * (1 - 0.0) = 5.0 ; needed = 5.0 * 1.0 = 5.0
    d = evaluate(soc_pct=80.0, remain_mile_km=5.0,
                 d_to_nearest_dock_km=5.0, thr=thr)
    assert d.triggered is False, (
        "criterion B fired at exact equality; 15 S8.2 writes strict `<`")
    # One epsilon further and it must fire -- otherwise "does not fire at
    # equality" would also be satisfied by a B that never fires at all.
    d2 = evaluate(soc_pct=80.0, remain_mile_km=5.0,
                  d_to_nearest_dock_km=5.000001, thr=thr)
    assert d2.triggered is True
    assert d2.trigger_code == "CHG-11"


def test_criterion_b_reads_the_keys_and_not_the_literals():
    """reserve_ratio and detour_factor must move the comparison.

    15 S8.2 prints 0.8 and 1.3 as numbers and annotates them as
    (1 - reserve_ratio) and detour_factor. An implementation that hard-coded
    the two numbers would pass every other case in this file and then ignore
    a site that re-tuned either key.

    Same geometry as the silent case above (usable 8.0 vs needed 7.8), but
    with detour_factor raised to 1.4: needed becomes 8.4 and B must fire.
    """
    # Only detour_factor differs from _THR_NO_CRITICAL. Changing one key at
    # a time is what makes the case attributable: a threshold set with two
    # edits would fire for either reason.
    thr = ChargeThresholds(soc_threshold_pct=30.0, reserve_ratio=0.20,
                           detour_factor=1.4)
    d = evaluate(soc_pct=80.0, remain_mile_km=10.0,
                 d_to_nearest_dock_km=6.0, thr=thr)
    assert d.triggered is True
    assert d.trigger_code == "CHG-11"
    # And the other key, independently: reserve_ratio 0.30 shrinks usable to
    # 7.0 against the original needed 7.8.
    thr2 = ChargeThresholds(soc_threshold_pct=30.0, reserve_ratio=0.30,
                            detour_factor=1.3)
    d2 = evaluate(soc_pct=80.0, remain_mile_km=10.0,
                  d_to_nearest_dock_km=6.0, thr=thr2)
    assert d2.triggered is True
    assert d2.trigger_code == "CHG-11"


def test_s82a_takes_precedence_and_forbids_driving():
    """Below soc_critical_pct: S8.2A, stop_in_place True.

    Precedence over A is the point. At 10% SoC criterion A is ALSO true, so
    an implementation that evaluated A first would report CHG-10 with
    stop_in_place False -- and the caller would drive a robot at 10% toward a
    dock, which is the outcome S8.2A CR-1 exists to prevent
    ("就地停车优先于回桩").
    """
    # _B_SILENT keeps criterion B out of this: with a dock 0.1 km away the
    # energy check cannot fire, so the reported code can only come from the
    # SoC path. Without that, a case at 10% SoC near a far dock would be
    # satisfied by either criterion and prove neither.
    d = evaluate(soc_pct=10.0, thr=_THR_WITH_CRITICAL, **_B_SILENT)
    assert d.triggered is True
    assert d.trigger_code == "S8.2A"
    # stop_in_place is the half with field consequences: a caller that only
    # read `triggered` would drive a robot at 10% SoC toward a dock.
    assert d.stop_in_place is True


def test_s82a_is_off_when_the_key_is_omitted():
    """Same SoC, critical tier not configured -> plain CHG-10.

    15 S12's per-key ruling makes the omitted key mean "tier off with a
    standing warn". So the identical input must produce a DRIVABLE decision
    when the dial is absent. Both halves are needed: the case above alone
    would pass for an implementation that ignored the config and always
    treated low SoC as critical.
    """
    d = evaluate(soc_pct=10.0, thr=_THR_NO_CRITICAL, **_B_SILENT)
    assert d.triggered is True
    assert d.trigger_code == "CHG-10"
    assert d.stop_in_place is False


def test_a_configured_zero_critical_is_not_read_as_tier_off():
    """soc_critical_pct=0.0 is a configured value, not an absent one.

    `if thr.soc_critical_pct:` instead of `is not None` would treat 0.0 as
    off. 0.0 is nonsensical and 15 S8.2A CR-2 refuses it at startup
    (0 < critical < threshold), but this module must not be the thing that
    silently absorbs it -- that is CLAUDE.md S3.1's "0.0 passing for
    assigned" in the other direction.

    At soc_pct 0.0 with critical 0.0 the strict `<` does not fire, so the
    observable difference is the CODE: tier-on-and-not-below reports CHG-10
    via criterion A, which is what the assertions below pin. A truthiness
    test would produce the same code here, so the case is built at a SoC
    BELOW zero as well, where the two implementations diverge.
    """
    thr = ChargeThresholds(soc_threshold_pct=30.0, reserve_ratio=0.20,
                           detour_factor=1.3, soc_critical_pct=0.0)
    # Negative SoC is not physical, but it is the only input that separates
    # "tier configured at 0" from "tier off" -- and a reading of -1 off a
    # faulty BMS is not impossible.
    d = evaluate(soc_pct=-1.0, thr=thr, **_B_SILENT)
    assert d.trigger_code == "S8.2A", (
        "critical tier configured at 0.0 was treated as absent")
    assert d.stop_in_place is True


@pytest.mark.parametrize("soc,mile,dock,why", [
    (None, None, None, "no seated battery AND no chassis range AND no dock"),
    (None, 10.0, 6.0, "no seated battery, B evaluable and silent"),
    (50.0, None, 6.0, "no RemainMile from the chassis"),
    (50.0, 10.0, None, "no dock known"),
])
def test_absent_inputs_do_not_trigger_and_do_not_raise(soc, mile, dock, why):
    """Each documented absent state: no trigger, no exception.

    *** This pins BEHAVIOUR, not correctness. 15 S8.2 defines soc_pct as
    null when no pack is seated (CHG-12b) and says nothing about what to
    conclude from it. "Unknown SoC" is not "SoC is fine" -- but it is also
    not a reason to drive to a dock, since charging a robot with no pack
    achieves nothing. The module declines to invent a fail-safe; this case
    records that choice so a future ruling has something to contradict.

    What it DOES guarantee is the part that is not a judgement call: no
    AttributeError / TypeError from comparing None with a float. The previous
    version typed soc_pct as plain float and would have raised on the very
    state its own source volume defines.
    """
    d = evaluate(soc_pct=soc, remain_mile_km=mile,
                 d_to_nearest_dock_km=dock, thr=_THR_WITH_CRITICAL)
    assert d.triggered is False, why
    assert d.stop_in_place is False, why


def test_no_invented_cr_codes_remain():
    """No trigger_code may be a bare CR-n.

    15 S1027 registers CR-1..CR-8 inside that volume as single-meaning (the
    S8.2A critical rules) and verbatim refuses to rename them, because
    S12 / S13 / S14 already cite them by number. The codes this module used
    to emit were a second meaning for the same prefix, reachable by grep and
    indistinguishable from the real one.

    Checked over every reachable decision rather than by reading the source,
    so a re-introduction through a new branch is caught too.
    """
    # One decision per reachable return statement in evaluate(): critical
    # tier, criterion A, criterion B firing, and nothing firing. If a branch
    # is added without a line here, the last assertion (the closed set of
    # emitted codes) is what will not cover it -- that is a known limit of
    # this case, not something it silently handles.
    decisions = [
        evaluate(soc_pct=10.0, thr=_THR_WITH_CRITICAL, **_B_SILENT),
        evaluate(soc_pct=29.0, thr=_THR_NO_CRITICAL, **_B_SILENT),
        evaluate(soc_pct=80.0, remain_mile_km=10.0,
                 d_to_nearest_dock_km=7.0, thr=_THR_NO_CRITICAL),
        evaluate(soc_pct=80.0, remain_mile_km=10.0,
                 d_to_nearest_dock_km=1.0, thr=_THR_NO_CRITICAL),
    ]
    # startswith and not equality against a list: the point is that NO code
    # carries that prefix, including one nobody has invented yet. An
    # allow-list of known-good codes would pass a seventh code added later.
    for d in decisions:
        assert not d.trigger_code.startswith("CR-"), d
    # And the codes that ARE emitted must be greppable into 15.
    emitted = {d.trigger_code for d in decisions if d.triggered}
    assert emitted <= {"CHG-10", "CHG-11", "S8.2A"}, emitted
