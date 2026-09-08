"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_audit_report.py
Brief: report-once + ring buffer + superseded (P6 -- 20 S9.0/S9.3)

Description:
Guards report-once (A-FAIL-1), no-auto-retry (A-FAIL-2), superseded-not-failure
(A-FAIL-3), and the bounded ring buffer (S9.3). Each names its mutant.
"""

from __future__ import annotations

from xbrain.p1_motion.rns.audit import (
    AuditRecord, Outcome, RingAudit, TerminalReporter, on_route_superseded,
)
from xbrain.p1_motion.rns.types import NavFailReason, NavFailure


def _fail(reason=NavFailReason.MAX_DEVIATION):
    return NavFailure(reason=reason, detail={})


def test_failure_reported_exactly_once():
    # A-FAIL-1: one terminal event -> one report. A second call returns None.
    # mutant: drop the spent gate -> two reports -> reddens.
    tr = TerminalReporter()
    assert tr.report_failure(_fail()) is not None
    assert tr.report_failure(_fail()) is None   # already reported


def test_spent_after_report_no_retry():
    # A-FAIL-2: after a terminal report, is_spent -> is_active false, no output.
    tr = TerminalReporter()
    assert tr.is_spent() is False
    tr.report_failure(_fail())
    assert tr.is_spent() is True


def test_new_mission_resets_reporter():
    # only a NEW mission un-spends the reporter (no auto-retry path does).
    tr = TerminalReporter()
    tr.report_failure(_fail())
    tr.reset_for_new_mission()
    assert tr.is_spent() is False
    assert tr.report_failure(_fail()) is not None


def test_arrival_also_reports_once():
    tr = TerminalReporter()
    assert tr.report_arrival() is Outcome.ARRIVED
    assert tr.report_arrival() is None


def test_superseded_is_not_a_failure():
    # A-FAIL-3: route_rev replacement -> SUPERSEDED outcome, audit record, NO
    # failure. mutant: report deviation on the swap -> false failure -> reddens.
    ra = RingAudit(capacity=8)
    out = on_route_superseded(ra, t_mono_ms=1000, old_rev=7, new_rev=8)
    assert out is Outcome.SUPERSEDED
    assert len(ra) == 1
    rec = ra.drain()[0]
    assert rec.kind == "superseded"
    assert rec.detail == {"old_rev": 7, "new_rev": 8}


def test_ring_buffer_is_bounded():
    # S9.3: fixed capacity, oldest evicted -- never unbounded growth in the loop.
    ra = RingAudit(capacity=3)
    for i in range(10):
        ra.append(AuditRecord(t_mono_ms=i, kind="x", detail={}))
    assert len(ra) == 3          # capped
    recs = ra.drain()
    assert [r.t_mono_ms for r in recs] == [7, 8, 9]   # newest three


def test_drain_clears():
    ra = RingAudit(capacity=8)
    ra.append(AuditRecord(t_mono_ms=1, kind="x", detail={}))
    ra.drain()
    assert len(ra) == 0
