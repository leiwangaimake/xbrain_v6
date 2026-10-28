"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptz_wiring.py
Brief: cmd/ptz consumer -- config load, both frame shapes, ordered worker, acks and events

Description:
Tests the p2 PTZ wiring (runtime/ptz_wiring.py):
  * a missing / bad credential file disables PTZ (does not raise);
  * read_ptz_config names every missing or unassigned key (CLAUDE.md 3.6);
  * a voice intent frame becomes controller legs, a PtzCommand frame goes to
    the controller and its ack is published on cmd/ptz/ack (Qt contract v0.2
    S7: the in-robot ack p5 translates);
  * frames are handled IN ARRIVAL ORDER by one worker -- a stop submitted
    after a jog leaves the head stopped (the 2026-08-11 thread-per-frame shape
    could run the stop first);
  * events carry the Qt contract S6 code.
ONVIF and LAPI are stubbed; no device. Mutation guards per CLAUDE.md 3.3.
"""
from __future__ import annotations

import json
import time

import pytest

import xbrain.p2_core.ptz.onvif_client as oc
from xbrain.common.config.resolved import MISSING
from xbrain.p2_core.ptz import lapi_focus
from xbrain.p2_core.runtime import ptz_wiring as pw
from xbrain.p2_core.runtime.ptz_wiring import (
    CMD_PTZ_ACK_TOPIC,
    EVENT_PTZ_TOPIC,
    OnvifConfig,
    PtzDomain,
    load_onvif_config,
    read_ptz_config,
)

pytestmark = pytest.mark.no_device


# -- config load: missing/bad file disables PTZ, never raises ------------

def test_load_missing_file_returns_none():
    assert load_onvif_config("/no/such/onvif.json") is None


def test_load_bad_json_returns_none(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{not json", encoding="utf-8")
    assert load_onvif_config(str(p)) is None


def test_load_missing_key_returns_none(tmp_path):
    p = tmp_path / "partial.json"
    p.write_text(json.dumps({"host": "h"}), encoding="utf-8")   # no user/pwd
    assert load_onvif_config(str(p)) is None


def test_load_good_file(tmp_path):
    p = tmp_path / "ok.json"
    p.write_text(json.dumps({"host": "h", "user": "u", "pwd": "p"}),
                 encoding="utf-8")
    cfg = load_onvif_config(str(p))
    assert cfg == OnvifConfig(host="h", user="u", pwd="p")


# -- read_ptz_config ---------------------------------------------------------

_CFG = {"ptz.hold_max_ms": 2000, "ptz.min_pulse_ms": 100, "ptz.max_pulse_ms": 2800,
        "ptz.max_cmd_hz": 17, "ptz.lapi_timeout_s": 0.5, "ptz.focus_mode": 2,
        "ptz.shield_move_ptz": 1}


def test_read_ptz_config_full():
    """Every key present -> a dict keyed without the 'ptz.' prefix."""
    cfg = read_ptz_config(lambda k: _CFG.get(k, MISSING), MISSING)
    assert cfg["hold_max_ms"] == 2000 and cfg["max_cmd_hz"] == 17


def test_read_ptz_config_names_every_missing_key():
    """Missing AND unassigned keys are both reported, by path. MUTATION:
    report only the first -> the second path absent from the message -> red."""
    snap = dict(_CFG)
    del snap["ptz.max_cmd_hz"]
    snap["ptz.lapi_timeout_s"] = None
    with pytest.raises(ValueError) as ei:
        read_ptz_config(lambda k: snap.get(k, MISSING), MISSING)
    assert "ptz.max_cmd_hz" in str(ei.value) and "ptz.lapi_timeout_s" in str(ei.value)


# -- the domain, with the camera stubbed ----------------------------------

@pytest.fixture
def domain(monkeypatch):
    """A PtzDomain whose ONVIF calls are recorded and whose LAPI session is a
    no-op; published (key, body) pairs are collected in dom.out."""
    calls = []
    monkeypatch.setattr(oc, "get_profile_token", lambda s, *a, **k: "tok")
    monkeypatch.setattr(oc, "ptz_continuous",
                        lambda s, p, t, **kw: calls.append(("move", kw)))
    monkeypatch.setattr(oc, "ptz_stop",
                        lambda s, p, t, **kw: calls.append(("stop", kw)))
    monkeypatch.setattr(lapi_focus.LapiFocusClient, "session_start", lambda self: True)
    monkeypatch.setattr(lapi_focus.LapiFocusClient, "session_end", lambda self: True)
    out = []
    dom = PtzDomain(OnvifConfig(host="h", user="u", pwd="p"),
                    read_ptz_config(lambda k: _CFG.get(k, MISSING), MISSING),
                    {"manual_cloud": 700, "auto_track": 500},
                    publish=lambda key, data: out.append((key, json.loads(data))))
    dom.calls = calls  # recorded camera calls
    dom.out = out      # published frames
    yield dom
    dom._stop.set()


def _jog(cmd_id, pan=0.5):
    """A cloud jog PtzCommand frame."""
    return json.dumps({"cmd_id": cmd_id, "action": "jog", "source": "cloud",
                       "vel": {"pan": pan}, "hold_ms": 800}).encode("utf-8")


def test_voice_intent_becomes_controller_legs(domain):
    """A voice E01 frame starts the left pulse through the controller.
    MUTATION: drop the slots -> no direction -> nothing moves -> red."""
    domain.handle_envelope(json.dumps({"intent_id": "E01", "direction": "left",
                                       "amount": "small"}).encode("utf-8"))
    assert domain.calls and domain.calls[-1][1]["pan"] < 0
    assert domain.controller.active          # the pulse runs in the tick now


def test_ptz_command_acks_on_cmd_ptz_ack(domain):
    """A PtzCommand frame reaches the controller and its ack goes out on the
    in-robot ack key. MUTATION: publish on another key -> red."""
    domain.handle_envelope(_jog("c-1"))
    assert domain.out[-1][0] == CMD_PTZ_ACK_TOPIC
    assert domain.out[-1][1]["cmd_id"] == "c-1" and domain.out[-1][1]["result"] == "accepted"


def test_bad_envelope_does_not_raise(domain):
    """Malformed bytes and non-object JSON are dropped, never raised."""
    domain.handle_envelope(b"{not json")
    domain.handle_envelope(b"[1, 2]")
    assert domain.calls == [] and domain.out == []


def test_worker_keeps_arrival_order(domain):
    """jog then stop, submitted back to back, end with the head stopped.
    MUTATION: a thread per frame / LIFO queue -> stop may run first -> the
    head stays moving -> red (the queue is drained by ONE worker)."""
    domain.submit(_jog("c-1"))
    domain.submit(json.dumps({"cmd_id": "c-2", "action": "stop"}).encode("utf-8"))
    domain.start()
    for _ in range(100):
        if len([o for o in domain.out if o[0] == CMD_PTZ_ACK_TOPIC]) >= 2:
            break
        time.sleep(0.02)
    acks = [o[1]["cmd_id"] for o in domain.out if o[0] == CMD_PTZ_ACK_TOPIC]
    assert acks == ["c-1", "c-2"]
    assert not domain.controller.active
    assert domain.calls[-1][0] == "stop"


def test_event_carries_contract_code(domain):
    """event/warn/ptz carries code PTZ_DEADMAN_STOP (Qt contract S6).
    MUTATION: omit 'code' -> p5 would derive one from the title -> red."""
    domain._publish_event("PTZ_DEADMAN_STOP", {"cmd_id": "c-1"})
    key, body = domain.out[-1]
    assert key == EVENT_PTZ_TOPIC and body["code"] == "PTZ_DEADMAN_STOP"
    assert body["src"] == "p2_core" and body["detail"]["cmd_id"] == "c-1"


def test_queue_overflow_is_counted_not_raised(domain):
    """A full queue drops and counts (the sender's dead-man covers a lost
    renewal); the Zenoh callback must never raise or block. The worker is
    not started, so the queue fills at QUEUE_DEPTH."""
    for _ in range(pw.QUEUE_DEPTH + 5):
        domain.submit(_jog("c-x"))
    assert domain.dropped == 5
