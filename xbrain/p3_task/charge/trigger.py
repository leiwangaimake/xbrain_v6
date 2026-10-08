"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: trigger.py
Brief: BIZ-P3-15 charge trigger -- 15 S8.2 criteria A or B, plus S8.2A critical

Description:
Decides whether P3 should request charging ON ITS OWN. 15 S8.2 gives exactly
two criteria and joins them with OR, verbatim "两个判据取'或'":

  A (fixed threshold)   soc_pct < soc_threshold_pct                 CHG-10
  B (energy reachable)  RemainMile * (1 - reserve_ratio)
                          < d_to_nearest_dock * detour_factor       CHG-11
  trigger <=> A or B

S8.2A adds one more state ABOVE those two, and it is not a third trigger --
it changes what the robot is allowed to DO: below soc_critical_pct the robot
must stop in place and must NOT drive to any dock, because driving from a
critical SoC might strand it somewhere further from rescue (S8.2A CR-1
verbatim "就地停车优先于回桩"). soc_critical_pct is an OPTIONAL dial: 15 S12
rules its key is OMITTED by default, and omission means the whole tier is off
with a standing warn. So None here is a legal, configured state, not a hole.

*** WHAT THIS MODULE DELIBERATELY DOES NOT DO (2026-10-09 rewrite, T11)

It does not implement "the user asked for charging". That request exists and
is already specified -- 11 S9.11.2 verbatim "下发: TaskCommand.action =
'submit', task.type = 'charge', task.params.dock_id 可选" -- but it arrives as
a SUBMITTED TASK, not as a trigger criterion. A submitted charge task does not
need P3 to decide anything; it needs P3 to run it. Putting a `user_requested`
branch here would make the same request reachable by two different code paths
with two different downstream shapes, and the dock-selection / admission logic
hangs off the task path only.
(The other charge entry point, the `cmd/charge` KEY, is 11 S9.11.1 "保留但本期
不实现" -- it answers rejected + E_NOT_IMPLEMENTED. So it is not an input here
either.)

*** WHAT THE PREVIOUS VERSION OF THIS FILE GOT WRONG, recorded because the
shape is worth recognising, not to narrate history

It declared "15 S8.2 defines eight independent conditions" and implemented
eight, coded CR-1..CR-8. Every part of that was wrong:
  - S8.2 defines TWO criteria, not eight.
  - Criterion B was absent entirely. The one criterion that answers "is 30%
    even enough from HERE" -- the whole point of S8.2's closing paragraph,
    verbatim "30% 够不够, 取决于最近的桩有多远" -- was not implemented.
  - It used two SoC tiers (soc_low_pct, soc_urgent_pct) plus a third
    (soc_recover_pct). The contract has ONE trigger threshold and 15 S12 CD-4
    says so in as many words: "不要再加第三个阈值" (the hysteresis comes free
    from the 30/90 gap between soc_threshold_pct and resume_soc_pct).
  - Four conditions (patrol block ended / return_home auto-charge / dock
    admission clearing a suspended task / idle timer) have no source in any
    volume. They are scheduling policy, and the contract's two criteria
    already answer "should we charge". Removed per CLAUDE.md S9.3.
  - CR-1..CR-8 collides with 15's OWN CR-1..CR-8, which are the S8.2A critical
    rules -- a different set of eight things. 15 S1027 registers that prefix as
    single-meaning within the volume and verbatim refuses to rename it, so the
    invented codes are the ones that had to go.

Trigger codes here are therefore the contract's own criterion numbers, which
can be grepped: "CHG-10", "CHG-11", "S8.2A".

All thresholds are injected at construction, no defaults (CLAUDE.md S3.1).
"""

from __future__ import annotations

# dataclass for the two frozen value objects; Optional because three inputs
# have a documented absent state (see each field).
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ChargeDecision:
    """What one evaluation concluded.

    stop_in_place is separate from triggered on purpose: S8.2A is a state in
    which the robot both WANTS to charge and MUST NOT drive, so a single
    boolean cannot carry it. A caller that reads only `triggered` and drives
    to a dock is the failure S8.2A exists to prevent, which is why the flag
    is on the decision rather than left for the caller to re-derive from the
    SoC.
    """

    triggered: bool
    #: "CHG-10" | "CHG-11" | "S8.2A" | "" -- greppable into 15. Empty only
    #: when triggered is False.
    trigger_code: str = ""
    stop_in_place: bool = False


@dataclass(frozen=True)
class ChargeThresholds:
    """Config values this decision needs. No defaults (CLAUDE.md S3.1).

    Names match 15 S12's charge: block key for key, so a reader can grep one
    spelling across config, contract and code. The previous version invented
    four names that appear in no volume.
    """

    #: 15 S12 charge.soc_threshold_pct (CHG-10). The ONE trigger threshold.
    soc_threshold_pct: float
    #: 15 S12 charge.reserve_ratio. Criterion B uses (1 - this) as the
    #: usable fraction of RemainMile -- S8.2 writes the 0.8 directly and
    #: annotates it "保留系数 (= 1 - reserve_ratio)", so the key is the
    #: source and the 0.8 is derived, never the other way round.
    reserve_ratio: float
    #: 15 S12 charge.detour_factor. Straight-line distance to real path.
    detour_factor: float
    #: 15 S12 charge.soc_critical_pct -- OPTIONAL. The key is omitted by
    #: default (15 S12's S6 per-key ruling: omitted, NOT null, because
    #: 10 S5.4.3 gives those two different meanings and null would make
    #: assertion A refuse the boot, rendering S8.2A's "key absent -> tier
    #: off" branch unreachable). None here IS that configured state.
    #: When present, 15 S8.2A CR-2 requires 0 < critical < threshold and
    #: refuses the boot otherwise -- enforced at startup, not here.
    soc_critical_pct: Optional[float] = None


def evaluate(soc_pct: Optional[float],
             remain_mile_km: Optional[float],
             d_to_nearest_dock_km: Optional[float],
             thr: ChargeThresholds) -> ChargeDecision:
    """15 S8.2 criterion A or B, with S8.2A taking precedence.

    Order is critical-first because S8.2A changes the permitted ACTION, not
    just the request. Then A, then B -- A and B are OR'd so the order between
    them only decides which code is reported, and A is reported first because
    it is the one an operator can check against a number on the HMI.

    Absent inputs, each with a documented reason:

      soc_pct is None       No battery is seated. 15 S8.2 CHG-12b verbatim:
                            take the smaller level_pct of the SEATED packs,
                            "一块在位的都没有 => null". Criteria A and S8.2A
                            cannot be evaluated; B still can.
      remain_mile_km None   The chassis did not supply RemainMile (CHG-12).
      d_to_nearest_dock     No dock is known (no geo data, or none reachable).
        is None

    *** When an input is absent this returns NOT-triggered for that criterion,
    and that is a gap in the contract rather than a decision this module is
    entitled to make. 15 S8.2 does not say what "soc_pct is null" should do:
    "no seated battery" means the SoC is unknown, which is not the same as
    "SoC is fine", but it is also not a reason to drive to a dock (charging a
    robot with no pack in it achieves nothing). So this module declines to
    invent a fail-safe and reports the criteria it could evaluate. Registered
    in docs/NEXT.md; a caller that needs "unknown SoC" to mean something must
    get that ruling first, NOT read it out of this function's behaviour.
    """
    # -- S8.2A: critical tier. Precedence, and only when the dial is on. ----
    # `is not None` and not a truthiness test: 0.0 would be a configured
    # value (a nonsensical one, which CR-2 refuses at startup) and must not
    # be read as "tier off". That distinction is the whole reason 15 S12
    # ruled the key OMITTED rather than null.
    if (thr.soc_critical_pct is not None
            and soc_pct is not None
            and soc_pct < thr.soc_critical_pct):
        # Both flags: it wants to charge AND must not drive there.
        return ChargeDecision(triggered=True, trigger_code="S8.2A",
                              stop_in_place=True)

    # -- Criterion A: fixed threshold (CHG-10) ------------------------------
    if soc_pct is not None and soc_pct < thr.soc_threshold_pct:
        return ChargeDecision(triggered=True, trigger_code="CHG-10")

    # -- Criterion B: energy reachability (CHG-11) -------------------------
    # 15 S8.2 verbatim: RemainMile * 0.8 < d_to_nearest_dock * 1.3, where
    # the 0.8 is annotated as (1 - reserve_ratio) and the 1.3 as
    # detour_factor. Written from the keys so that changing reserve_ratio
    # in config moves this comparison -- a literal 0.8 here would be a
    # second definition of the same quantity.
    #
    # Strict `<`, matching the contract's own operator: at exact equality
    # the remaining range is precisely the detoured distance, which S8.2
    # treats as still reachable. That is the boundary the reserve_ratio
    # already covers; adding a margin here would be a second reserve.
    if remain_mile_km is not None and d_to_nearest_dock_km is not None:
        usable_km = remain_mile_km * (1.0 - thr.reserve_ratio)
        needed_km = d_to_nearest_dock_km * thr.detour_factor
        if usable_km < needed_km:
            return ChargeDecision(triggered=True, trigger_code="CHG-11")

    # Neither criterion fired (or neither could be evaluated -- see the
    # docstring; those two outcomes are deliberately NOT distinguished here,
    # because distinguishing them would require inventing a third state the
    # contract does not define).
    return ChargeDecision(triggered=False)
