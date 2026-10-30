"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_payload_state.py
Brief: state/mode.payload (11 S4.3) is built from the device readback, left out when the device is silent, and carried by ModeFace

Description:
Until 2026-10-30 P2's state/mode had no payload object, so the G25-G30 voice
queries could not answer (field test R004-R006). These cases pin
mode/payload_state.PayloadStateTracker and its ModeFace wiring:
  * light / strobe come from the GET /status lights readback (searchlight,
    redblue), volume / siren / speaking from the inputs P2 holds;
  * no answer or a malformed readback -> None (the field is left out, never a
    last-known value);
  * updated_ts moves only when a field changes;
  * ModeFace puts the object into state/mode, and leaves it out on None.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import json  # decode the published frame

import pytest  # parametrize, marks

from xbrain.p2_core.mode.payload_state import PayloadStateTracker  # under test
from xbrain.p2_core.runtime.mode_wiring import ModeFace  # the publisher

pytestmark = pytest.mark.no_device  # pure in-process

STATUS = {"mode": "func1", "lights": {"searchlight": True, "bright": 30, "redblue": 1}}  # as /status sends it


def test_fields_come_from_the_readback_and_p2_inputs():  # one query answer: test fields come from the readback and p2 inputs
    """MUTATION: read strobe from searchlight -> strobe follows the wrong lamp -> fails."""
    p = PayloadStateTracker().build(STATUS, volume_pct=30, siren_on=False, speaking=True, now_wall=100.0)  # local read of the state field
    assert p == {"siren": False, "strobe": True, "light": "on", "light_effective": True,  # the property this case pins
                 "light_src": "manual", "volume_pct": 30, "speaking": True, "updated_ts": 100.0}  # id -> spoken label


def test_each_lamp_reads_its_own_bit():  # one query answer: test each lamp reads its own bit
    """Searchlight on, red/blue off: the two must not follow each other."""
    s = {"mode": "func1", "lights": {"searchlight": True, "bright": 30, "redblue": 0}}  # the lamps disagree
    p = PayloadStateTracker().build(s, volume_pct=30, siren_on=True, speaking=False, now_wall=1.0)  # local read of the state field
    assert (p["light_effective"], p["strobe"], p["siren"]) == (True, False, True)  # the property this case pins


@pytest.mark.parametrize("status", [None, {}, {"lights": None},
                                    {"lights": {"searchlight": "yes", "redblue": 0}},
                                    {"lights": {"searchlight": True, "redblue": True}}])
def test_no_or_bad_readback_gives_none(status):  # one query answer: test no or bad readback gives none
    """MUTATION: default a missing readback to off -> a guess is published -> fails."""
    assert PayloadStateTracker().build(status, volume_pct=30, siren_on=False,  # the property this case pins
                                       speaking=False, now_wall=1.0) is None


def test_updated_ts_moves_only_on_change():  # one query answer: test updated ts moves only on change
    """MUTATION: stamp every build -> updated_ts says 'just changed' forever -> fails."""
    t = PayloadStateTracker()  # local read of the state field
    t.build(STATUS, volume_pct=30, siren_on=False, speaking=False, now_wall=100.0)
    same = t.build(STATUS, volume_pct=30, siren_on=False, speaking=False, now_wall=105.0)  # value used below
    assert same["updated_ts"] == 100.0  # nothing changed
    moved = t.build(STATUS, volume_pct=75, siren_on=False, speaking=False, now_wall=110.0)  # value used below
    assert moved["updated_ts"] == 110.0  # volume changed


def _published(payload_state_of):  # ModeFace -> the decoded state/mode body
    sent = []  # (key, bytes)
    face = ModeFace(publish=lambda k, d: sent.append((k, d)), payload_state_of=payload_state_of)  # value used below
    face.republish(now_mono_ms=1000)  # the 1 Hz floor publishes
    return json.loads(sent[-1][1].decode("utf-8"))  # result to the caller


def test_modeface_carries_the_payload_and_leaves_it_out_on_none():  # one query answer: test modeface carries the payload and leaves it out on none
    """MUTATION: drop the payload line in ModeFace -> no field -> fails."""
    obj = {"siren": False, "volume_pct": 30}  # any object
    assert _published(lambda: obj)["payload"] == obj  # carried
    assert "payload" not in _published(lambda: None)  # device silent -> left out
