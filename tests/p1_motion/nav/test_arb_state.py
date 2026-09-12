"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_arb_state.py
Brief: domain-1 arbitration visibility -- 7A.5.1 body, gen-on-change, 1 Hz heartbeat, 7A.7 events

Description:
Pins the three rules 11 S7A.8 states for domain 1 (gen only on winner
change; state on change + 1 Hz; audit event on change) and the outward
schema (7A.5.1 field set, 7A.7 fixed detail keys), driven through a real
P1Arbiter so alive flags come from actual note()/tick() calls.
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.nav.arb_state import ArbVisibility
from xbrain.p1_motion.sources.arbiter_p1 import BehaviorSource, P1Arbiter

pytestmark = pytest.mark.no_device

STATE_KEYS = {"domain", "gen", "suspended", "holder", "waiting", "last_change", "sources"}
HOLDER_KEYS = {"source_id", "req_id", "priority", "since_mono_ms", "held_ms", "note"}
EVENT_KEYS = {"domain", "action", "from", "to", "reason", "policy", "held_ms", "wait_ms",
              "overdue_ms", "forced", "gen", "code", "count"}


def _obs(vis, arb, now, suspended=None):
    return vis.observe(holder=arb.holder(), snapshot=arb.snapshot(),
                       suspended=suspended, now_mono_ms=now)


def test_first_tick_grants_hold_with_the_full_schema():
    arb, vis = P1Arbiter(), ArbVisibility()
    arb.note(BehaviorSource.HOLD, 1000)
    arb.tick(1000)
    body, events = _obs(vis, arb, 1000)
    assert body is not None and set(body) == STATE_KEYS
    assert body["domain"] == "motion" and body["gen"] == 1 and body["suspended"] is None
    assert set(body["holder"]) == HOLDER_KEYS and body["holder"]["source_id"] == "hold"
    assert body["holder"]["priority"] == 100 and body["waiting"] == []
    assert {s["source_id"] for s in body["sources"]} == {s.value for s in BehaviorSource}
    alive = {s["source_id"]: s["alive"] for s in body["sources"]}
    assert alive["hold"] is True and alive["rns_avoid"] is False
    assert [e.action for e in events] == ["grant"] and set(events[0].detail) == EVENT_KEYS
    assert events[0].dedup_key == "arb:motion:grant"


def test_gen_moves_only_with_the_winner():
    """mutant: gen += 1 per tick -> red."""
    arb, vis = P1Arbiter(), ArbVisibility()
    for t in range(1000, 1500, 50):
        arb.note(BehaviorSource.HOLD, t)
        arb.tick(t)
        _obs(vis, arb, t)
    assert vis.gen == 1
    arb.note(BehaviorSource.RNS_AVOID, 1500)
    arb.note(BehaviorSource.HOLD, 1500)
    arb.tick(1500)
    body, events = _obs(vis, arb, 1500)
    assert vis.gen == 2 and body["holder"]["source_id"] == "rns_avoid"
    assert body["last_change"]["action"] == "preempt" and body["last_change"]["from"] == "hold"
    assert events[0].action == "preempt" and events[0].detail["held_ms"] == 500


def test_release_when_the_holder_goes_stale():
    arb, vis = P1Arbiter(dwell_ms=200), ArbVisibility()
    arb.note(BehaviorSource.RNS_AVOID, 1000)
    arb.note(BehaviorSource.HOLD, 1000)
    arb.tick(1000)
    _obs(vis, arb, 1000)
    for t in (1100, 1300):
        arb.note(BehaviorSource.HOLD, t)
        arb.tick(t)
        body, events = _obs(vis, arb, t)
    assert body["holder"]["source_id"] == "hold" and events[0].action == "release"
    assert events[0].detail["reason"] == "source_deactivated"


def test_heartbeat_every_second_without_change():
    arb, vis = P1Arbiter(), ArbVisibility()
    arb.note(BehaviorSource.HOLD, 0)
    arb.tick(0)
    assert _obs(vis, arb, 0)[0] is not None
    seen = [t for t in range(50, 2100, 50) if _obs(vis, arb, t)[0] is not None]
    assert seen == [1000, 2000]


def test_suspend_and_rearm_are_audited():
    arb, vis = P1Arbiter(), ArbVisibility()
    arb.note(BehaviorSource.HOLD, 0)
    arb.tick(0)
    _obs(vis, arb, 0)
    body, events = _obs(vis, arb, 50, suspended="soft_estop")
    assert body["suspended"] == "soft_estop" and [e.action for e in events] == ["suspend"]
    body, events = _obs(vis, arb, 100, suspended=None)
    assert body["suspended"] is None and [e.action for e in events] == ["rearm"]
    assert vis.gen == 1                                # not a winner change
