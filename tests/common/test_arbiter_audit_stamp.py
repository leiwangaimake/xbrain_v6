"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_arbiter_audit_stamp.py
Brief: Arbitration audit events carry a producer eid (11 S6.1) and survive p5's event pipeline step 1

Description:
From 99 U99 C1 (2026-10-28) p2 published event/{sev}/arbitration straight from
merge_audit_window, and audit.py trap 3 said p5 would stamp the eid. It does
not: p5's pipeline drops an eid-less event (missing_field:eid) and its cloud
projector raises "event missing eid (v2.0 S5.1)", so not one arbitration event
reached record.db or the cloud (00 EVT-16 "every grant / preempt / release is
audited" was dead on arrival). These cases drive the REAL renderer through
stamp_for_publish into the REAL p5 normaliser and pipeline validator:
  * a rendered record is dropped by p5 without the stamp, kept with it;
  * the stamp adds eid / src / ts and leaves the caller's record untouched;
  * an empty eid is refused at the producer, not discovered at p5.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import pytest  # raises, marks

from xbrain.common.arbiter.audit import render_audit_event, stamp_for_publish  # under test
from xbrain.common.arbiter.model import ArbAction, ArbEvent  # a real arbitration event
from xbrain.p5_gateway.event.pipeline import EventPipeline  # p5 step 1
from xbrain.p5_gateway.runtime.main_wiring import _normalise_event  # p5 ingest

pytestmark = pytest.mark.no_device  # pure in-process


def _record():  # one rendered release on the speaker
    """A rendered audit record exactly as p2 had published it."""
    ev = ArbEvent(domain="speaker", action=ArbAction.RELEASE.value, gen=3,
                  from_source="tts_local", to_source=None, reason="done",
                  forced=False, mono_ms=1000, detail={})  # tts let go
    return render_audit_event(ev)  # what merge_audit_window yields per event


def _p5_verdict(body, monkeypatch):  # through p5's ingest + step 1
    """p5's drop reason for this body on event/{sev}/arbitration, None = kept."""
    monkeypatch.setenv("XBRAIN_ROBOT_ID", "gj-001")  # rid for a relative key
    norm = _normalise_event("event/%s/arbitration" % body["sev"], body)  # p5 ingest
    if norm is None:  # the normaliser already refuses it
        return "normalise:none"  # dropped before the pipeline
    return EventPipeline(dao=None)._validate(norm)  # pipeline step 1


def test_p5_drops_the_unstamped_record_and_keeps_the_stamped_one(monkeypatch):
    """The on-robot defect, end to end. MUTATION: stamp_for_publish leaves eid
    out -> p5 drops it -> the second assert fails."""
    rec = _record()  # as published before the fix
    assert _p5_verdict(rec, monkeypatch) is not None  # dropped: the defect
    out = stamp_for_publish(rec, eid="arb-p2-1-1", src="p2_core", ts=1.0e9)  # the fix
    assert _p5_verdict(out, monkeypatch) is None  # kept


def test_the_stamp_adds_three_fields_and_copies():
    """MUTATION: update the caller's dict in place -> rec gains eid -> fails."""
    rec = _record()  # rendered
    out = stamp_for_publish(rec, eid="arb-p2-1-2", src="p2_core", ts=2.0e9)  # stamped
    assert (out["eid"], out["src"], out["ts"]) == ("arb-p2-1-2", "p2_core", 2.0e9)  # the three
    assert "eid" not in rec and out["title"] == rec["title"]  # copy, body kept


def test_an_empty_eid_is_refused_at_the_producer():
    """MUTATION: drop the check -> no raise -> p5 would drop it silently."""
    with pytest.raises(ValueError):
        stamp_for_publish(_record(), eid="", src="p2_core", ts=1.0)
