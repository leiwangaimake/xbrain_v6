"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_event_sev_cat.py
Brief: 11 S6.2 event sev/cat come off the KEY -- HMI stream + cloud relay

Description:
The defect these cases exist for: p5's event callback read severity/category out
of the message BODY, while every on-board producer puts them only in the key
(event/{sev}/{cat}). Both came back None for every event ever published, which
blanked them in the HMI stream and -- because the cloud relay is gated on exactly
those two values -- meant not one event ever reached Qt.

Why it survived the test suite: the relay had a test, and that test asserted an
ast.Call to publish_event exists in main_wiring. A call sitting behind a guard
that is never true satisfies that assertion perfectly (3.2 form 1). The cases
here assert the guard's INPUTS on the real producer message shape instead.
"""

from __future__ import annotations

import inspect

from xbrain.p5_gateway.runtime import main_wiring
from xbrain.p5_gateway.runtime.main_wiring import (
    _event_sev_cat, _normalise_event, run_voice_loop_wiring,
)


# INF-TS-1: 纯单测, 不碰设备(无 zenohd / 无底盘 / 无 ORIN 专属硬件).
import pytest

pytestmark = pytest.mark.no_device


#: What p3/p2/p1 actually put on the bus: no sev, no cat, no severity, no
#: category -- those two live in the key and nowhere else.
_PRODUCER_BODY = {"eid": "geo-abc-1", "title": "geo w-main_gate geo.renamed",
                  "detail": {"geo_id": "w-main_gate", "type": "geo.renamed"},
                  "src": "p3_task", "ts": 0.0}


def test_the_real_producer_message_yields_both_values():
    """THE defect, stated as the shape that broke. Reading the body gives
    (None, None) here, and the relay gate turns both into "drop it".
    MUTATION: return (d.get("sev"), d.get("cat")) -> red."""
    sev, cat = _event_sev_cat("event/info/geo", dict(_PRODUCER_BODY))
    assert sev == "info"
    assert cat == "geo"


def test_the_key_wins_over_a_body_that_disagrees():
    """_normalise_event calls the key authoritative; a body that carries stale
    or wrong values must not override it.
    MUTATION: prefer the body when present -> red."""
    body = dict(_PRODUCER_BODY, sev="fault", category="chassis")
    sev, cat = _event_sev_cat("event/warn/fence", body)
    assert (sev, cat) == ("warn", "fence")


def test_the_absolute_cloud_key_parses_too():
    """xbrain/{rid}/event/{sev}/{cat} differs from the bare key by a two-segment
    prefix, so a fixed offset reads the rid as the severity.
    MUTATION: sev = segs[1], cat = segs[2] -> red."""
    sev, cat = _event_sev_cat("xbrain/gj-001/event/alarm/intrusion",
                              dict(_PRODUCER_BODY))
    assert (sev, cat) == ("alarm", "intrusion")


def test_the_body_is_still_the_fallback_when_the_key_carries_nothing():
    """Kept so a message that does carry them and arrives on an unparsable key
    is not dropped. MUTATION: return (None, None) in the no-event-segment case
    -> red."""
    sev, cat = _event_sev_cat("some/other/key",
                              {"sev": "warn", "category": "comm"})
    assert (sev, cat) == ("warn", "comm")


def test_a_key_with_no_category_segment_yields_no_category():
    """event/info alone is not an S6.2 key; inventing a category would put an
    unclassifiable row in the audit trail (3.5).
    MUTATION: default cat to "system" -> red."""
    sev, cat = _event_sev_cat("event/info", dict(_PRODUCER_BODY))
    assert sev == "info"
    assert cat is None


def test_normalise_event_persists_the_producer_shape():
    """The persist path already read the key, which is why record.db has correct
    sev/cat while the cloud got nothing -- the two paths had two copies of this
    parse and only one was right. Now they share one.
    MUTATION: have _normalise_event read sev/cat from the body -> red."""
    ev = _normalise_event("event/info/geo", dict(_PRODUCER_BODY, rid="gj-001"))
    assert ev is not None, "a well-formed producer event must not normalise away"
    assert ev["sev"] == "info"
    assert ev["cat"] == "geo"


def test_the_hmi_stream_and_the_relay_gate_read_the_key_not_the_body():
    """Both consumers must use the shared parse. _on_event is a closure inside
    the wiring, so this reads p5's real source: a fake callback would just do
    the right thing and prove nothing (3.2 form 1).
    MUTATION: restore "sev": d.get("severity") or d.get("sev") -> red."""
    src = inspect.getsource(run_voice_loop_wiring)
    body = src[src.index("def _on_event("):]
    body = body[:body.index("cloud_bridge.publish_event")]
    assert "_event_sev_cat(key, d)" in body, (
        "the event callback does not derive sev/cat from the key")
    assert '"sev": d.get(' not in body and '"cat": d.get(' not in body, (
        "the event callback still reads sev/cat out of the body")


def test_the_relay_gate_is_fed_by_what_the_key_parse_produced():
    """The gate itself is what silently disabled the relay. It must test the
    values the shared parse produced, not fields of the raw message.
    MUTATION: gate on d.get("sev") and d.get("cat") -> red."""
    src = inspect.getsource(run_voice_loop_wiring)
    gate = src[src.index("def _on_event("):]
    gate = gate[gate.index("cloud_bridge is not None"):]
    gate = gate[:gate.index("\n")]
    assert 'ev["sev"]' in gate and 'ev["cat"]' in gate, gate


def test_the_two_parses_are_one_implementation():
    """They drifted apart once already, and the copy that was wrong was the one
    nothing checked. MUTATION: inline the segs parse back into _normalise_event
    -> red."""
    src = inspect.getsource(_normalise_event)
    assert "_event_sev_cat(key, d)" in src
    assert "segs[ei + 1]" not in src, (
        "_normalise_event kept its own copy of the sev/cat parse")


def test_main_wiring_still_has_exactly_one_relay_call():
    """Kept from the original relay test -- a second call site would double every
    event on the cloud face. The cases above cover what it could not: that the
    single call is reachable."""
    src = inspect.getsource(main_wiring)
    assert src.count("cloud_bridge.publish_event(") == 1
