"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_types.py
Brief: RNS closed-set and transition-table assertions (20 S9.0.1/S9.0.2)

Description:
Guards the closed sets in rns/types.py against drift and the transition table
against illegal moves (A-ST-1). Each test states the mutant that reddens it
(CLAUDE.md 3.3): the closed-set tests redden if a member is added/removed
without updating both the enum and this test; the transition tests redden if
is_legal_transition() stops rejecting a table-external move.
"""

from __future__ import annotations

import pytest

from xbrain.p1_motion.rns.types import (
    Cell, MissionKind, NavFailReason, NavState, Origin, SrcBit,
    TRANSITIONS, VelocityCandidate, is_legal_transition,
)
from xbrain.common.types.units import Mps


def test_cell_closed_set_exact():
    # mutant: add/remove a Cell member -> this frozenset compare reddens.
    assert {c.value for c in Cell} == {"free", "blocked", "unknown"}


def test_fail_reason_closed_set_exact():
    # mutant: drop watchdog_no_progress (v1.8 addition) -> reddens. This is the
    # 3.7 hand-count trap made into a machine check.
    assert {r.value for r in NavFailReason} == {
        "max_deviation", "wall_closed_loop", "wall_no_progress", "wall_budget",
        "watchdog_no_progress", "blocked_by_dynamic", "rtk_unreliable",
        "target_lost",
    }


def test_navstate_has_eight_resident_plus_three_exits():
    residents = {"idle", "follow", "thread", "detour", "wall_follow",
                 "wait_dynamic", "suspended"}
    exits = {"arrived", "failed", "cancelled"}
    assert {s.value for s in NavState} == residents | exits


def test_src_bits_are_distinct_powers_of_two():
    bits = [SrcBit.SEG, SrcBit.GEOM, SrcBit.SEMANTIC, SrcBit.NEGATIVE]
    assert bits == [1, 2, 4, 8]
    # they OR without collision (a cell can have several sources)
    assert SrcBit.SEG | SrcBit.GEOM == 3


def test_mission_kind_carries_reserved_follow_target():
    # follow_target is a reserved schema value (20 #20-13); it must exist so
    # messages round-trip, even though no consumer branches on it this phase.
    assert MissionKind.FOLLOW_TARGET.value == "follow_target"


def test_origin_is_route_or_relmove():
    # the whole point of Origin (12 S4.2c.1): exactly these two carriers.
    assert {o.value for o in Origin} == {"route", "relmove"}


# ── transition table (A-ST-1) ─────────────────────────────────────────────────
def test_wall_follow_cannot_jump_to_thread():
    # mutant: the classic table-external move. 20 S9.0.1 has no WALL_FOLLOW ->
    # THREAD edge (you leave wall-follow to FOLLOW, then re-thread). Must be
    # illegal; if is_legal_transition stops rejecting it, this reddens.
    assert not is_legal_transition(NavState.WALL_FOLLOW, NavState.THREAD)


def test_idle_only_starts_follow():
    assert is_legal_transition(NavState.IDLE, NavState.FOLLOW)
    assert not is_legal_transition(NavState.IDLE, NavState.WALL_FOLLOW)


def test_any_motion_state_may_go_idle():
    # arrive/fail/cancel collapse any state to IDLE (20 S9.0.1).
    for s in (NavState.FOLLOW, NavState.THREAD, NavState.DETOUR,
              NavState.WALL_FOLLOW, NavState.WAIT_DYNAMIC, NavState.SUSPENDED):
        assert is_legal_transition(s, NavState.IDLE)


def test_suspended_resumes_only_to_follow():
    # RNS-M-7 c: on release, re-evaluate from FOLLOW; SUSPENDED never resumes
    # straight back into WALL_FOLLOW/THREAD (that is a re-evaluation, not a
    # restore of the frozen intent).
    assert is_legal_transition(NavState.SUSPENDED, NavState.FOLLOW)
    assert not is_legal_transition(NavState.SUSPENDED, NavState.WALL_FOLLOW)


def test_unknown_source_state_has_no_transitions():
    # is_legal_transition must not KeyError on an event-exit state; it has no
    # outgoing edges (it collapses to IDLE outside the table).
    assert not is_legal_transition(NavState.ARRIVED, NavState.FOLLOW)


def test_velocity_candidate_holds_units():
    c = VelocityCandidate(vx=Mps(1.5), vy=Mps(0.0), wz=0.3)
    assert c.vx == Mps(1.5)
    assert c.wz == 0.3
