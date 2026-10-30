"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_turn_speed.py
Brief: The in-place turn speed defaults to max, steps by 0.1 inside [0.4, 0.6], keeps an explicit value across restarts, and reaches every relmove as max_yaw_rate_radps
Description:
User ruling 2026-10-30 (99 U104): turn at the maximum by default; "机器人转速
快一点 / 慢一点 / 最快 / 最慢" adjusts it, never below the twist threshold; an
explicit value survives restarts like the volume. These cases cover:
  * TurnSpeedSetting: default, steps, bounds, at_limit, persistence, a stored
    value clamped after the range moved, an unreadable file, a bad config;
  * the dispatcher refuses an unknown level; ModeFace applies the action,
    acks the RESULT, publishes state/mode.turn_speed, and refuses when the
    setting is not wired;
  * the relmove P2 forwards carries max_yaw_rate_radps = the current value.
The bounds used here are the production p2_core.yaml turn_speed block.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import json  # frames and the state file

import pytest  # fixtures, marks
import yaml  # production config

from xbrain.p2_core.mode.turn_speed import TurnSpeedSetting  # under test
from xbrain.p2_core.mode_actions.dispatch import dispatch  # the closed-set gate
from xbrain.p2_core.runtime.mode_wiring import ModeFace  # applies set_turn_speed

pytestmark = pytest.mark.no_device  # pure in-process

with open("/opt/xbrain_v6/configs/p2_core.yaml", encoding="utf-8") as _fh:  # production config
    TS = yaml.safe_load(_fh)["turn_speed"]  # the U104 block


def _setting(tmp_path, name="ts.json"):  # a setting on the production bounds
    return TurnSpeedSetting(str(tmp_path / name), TS["min_radps"], TS["max_radps"], TS["step_radps"])  # fresh


def test_production_bounds_are_the_ruling():  # the ruled numbers
    """The user's numbers: max 0.6 (interim), min 0.4, step 0.1."""
    assert (TS["max_radps"], TS["min_radps"], TS["step_radps"]) == (0.6, 0.4, 0.1)  # max 0.6 interim, min 0.4, step 0.1


def test_default_is_max(tmp_path):  # no file yet
    s = _setting(tmp_path)  # nothing stored
    assert s.effective() == 0.6 and s.origin() == "default"  # default = max
    assert s.snapshot() == {"radps": 0.6, "min_radps": 0.4, "max_radps": 0.6, "origin": "default"}  # state/mode.turn_speed shape


def test_steps_bounds_and_at_limit(tmp_path):  # faster / slower / max / min
    """MUTATION: drop the min() clamp on faster -> 0.7 -> fails."""
    s = _setting(tmp_path)  # starts at max
    assert s.apply("faster") == {"turn_speed_radps": 0.6, "changed": False, "at_limit": "max"}  # already max
    assert s.apply("slower") == {"turn_speed_radps": 0.5, "changed": True, "at_limit": None}  # one step
    assert s.apply("slower") == {"turn_speed_radps": 0.4, "changed": True, "at_limit": "min"}  # floor
    assert s.apply("slower") == {"turn_speed_radps": 0.4, "changed": False, "at_limit": "min"}  # never below
    assert s.apply("max")["turn_speed_radps"] == 0.6 and s.apply("min")["turn_speed_radps"] == 0.4  # jumps


def test_repeated_steps_land_exactly_on_the_bounds(tmp_path):  # no float drift
    """Integer milli-rad/s: two "快一点" from 0.4 is exactly 0.6, not 0.6000000001."""
    s = _setting(tmp_path)  # fresh
    s.apply("min")  # 0.4
    s.apply("faster")  # 0.5
    assert s.apply("faster") == {"turn_speed_radps": 0.6, "changed": True, "at_limit": "max"}  # exactly max


def test_values_live_on_the_milli_grid(tmp_path):
    """MUTATION: keep float milli-units (drop int(round())) -> a hand-edited
    0.5004 stays 0.5004 and five steps never land on the bounds -> fails.
    (For 3-decimal configs float * 1000 happens to be exact; the grid is what
    keeps an odd stored value from drifting off it.)"""
    (tmp_path / "ts.json").write_text(json.dumps({"turn_speed_radps": 0.5004}), encoding="utf-8")  # off-grid
    s = _setting(tmp_path)  # loads it
    assert s.effective() == 0.5  # snapped to the milli grid
    assert s.apply("faster") == {"turn_speed_radps": 0.6, "changed": True, "at_limit": "max"}  # lands on max


def test_explicit_value_survives_a_restart(tmp_path):  # persistence
    """MUTATION: skip the file write -> the new instance starts at max -> fails."""
    _setting(tmp_path).apply("slower")  # 0.5, persisted
    again = _setting(tmp_path)  # "restart"
    assert again.effective() == 0.5 and again.origin() == "explicit"  # restored, not reset to max
    assert json.loads((tmp_path / "ts.json").read_text(encoding="utf-8")) == {"turn_speed_radps": 0.5}  # the file on disk


def test_stored_value_outside_the_range_is_clamped(tmp_path):  # config range moved
    """The config max dropped below a stored value: never turn faster than max."""
    (tmp_path / "ts.json").write_text(json.dumps({"turn_speed_radps": 0.9}), encoding="utf-8")  # above max
    assert _setting(tmp_path).effective() == 0.6  # clamped down to max
    (tmp_path / "ts.json").write_text(json.dumps({"turn_speed_radps": 0.1}), encoding="utf-8")  # below min
    assert _setting(tmp_path).effective() == 0.4  # never into the twist band


@pytest.mark.parametrize("content", ["{", "[1]", json.dumps({"turn_speed_radps": "fast"}),  # truncated / list / string
                                     json.dumps({"turn_speed_radps": True})])  # bool is not a number
def test_unusable_file_falls_back_to_max(tmp_path, content):  # default on bad file
    (tmp_path / "ts.json").write_text(content, encoding="utf-8")  # truncated / foreign
    s = _setting(tmp_path)  # loads the bad file
    assert s.effective() == 0.6 and s.origin() == "default"  # max, origin default


@pytest.mark.parametrize("lo, hi, st", [(0.6, 0.4, 0.1), (0.0, 0.6, 0.1), (0.4, 0.6, 0.0)])  # inverted / zero min / zero step
def test_bad_config_raises(tmp_path, lo, hi, st):  # config errors
    with pytest.raises(ValueError):  # loud at start
        TurnSpeedSetting(str(tmp_path / "x.json"), lo, hi, st)  # must raise


def test_dispatcher_refuses_an_unknown_level():  # closed set
    """MUTATION: drop the closed-set check -> "fastest" is accepted -> fails."""
    assert dispatch({"action": "set_turn_speed", "turn_speed": "fastest"}, {}).accepted is False  # unknown level refused
    assert dispatch({"action": "set_turn_speed"}, {}).accepted is False  # missing
    assert dispatch({"action": "set_turn_speed", "turn_speed": "faster"}, {}).accepted is True  # known level passes


def _face(tmp_path, wired=True):  # ModeFace with a recorded publisher
    sent = []  # (key, bytes)
    face = ModeFace(publish=lambda k, d: sent.append((k, d)),  # recorder
                    turn_speed=_setting(tmp_path) if wired else None)  # the setting
    return face, sent  # face + what it published


def _frame(level):  # one set_turn_speed ModeCommand
    return json.dumps({"cmd_id": "c-ts", "action": "set_turn_speed", "turn_speed": level,  # ModeCommand body
                       "source": "cloud"}).encode("utf-8")  # bus bytes


def test_modeface_applies_and_acks_the_result(tmp_path):  # ack carries the result
    """MUTATION: ack the request instead of apply() -> no turn_speed_radps -> fails."""
    face, sent = _face(tmp_path)  # wired face
    ack = face.handle_frame(_frame("slower"), now_mono_ms=1000)  # 0.6 -> 0.5
    assert ack["result"] == "accepted"  # accepted
    assert ack["detail"]["applied"] == {"turn_speed_radps": 0.5, "changed": True, "at_limit": None}  # value after the step
    body = json.loads(sent[-1][1].decode("utf-8"))  # the state/mode it published
    assert body["turn_speed"]["radps"] == 0.5 and body["turn_speed"]["origin"] == "explicit"  # state/mode follows
    assert body["voice_mode"] == "idle"  # no mode change


def test_state_mode_carries_turn_speed_every_beat(tmp_path):  # 1 Hz republish
    """MUTATION: drop the turn_speed line in _publish_state -> fails."""
    face, sent = _face(tmp_path)  # wired face
    face.republish(now_mono_ms=1000)  # the 1 Hz floor
    assert json.loads(sent[-1][1].decode("utf-8"))["turn_speed"]["radps"] == 0.6  # default max


def test_unwired_setting_is_refused_not_acked(tmp_path):  # no setting wired
    face, _sent = _face(tmp_path, wired=False)  # face without a setting
    ack = face.handle_frame(_frame("faster"), now_mono_ms=1000)  # try to set
    assert ack["result"] == "rejected"  # never "done" without a setting


def test_relmove_carries_the_turn_speed(tmp_path, monkeypatch):  # MO-2 param
    """MUTATION: drop params["max_yaw_rate_radps"] in _handle_motion_intent -> fails."""
    from xbrain.p2_core.runtime import main_wiring as mw  # under test
    from xbrain.p2_core.runtime import motion_intent_wiring as miw  # its gates

    class _Pass:  # every gate passed
        passed = True  # verdict.passed

    monkeypatch.setattr(miw, "parse_intent_envelope",  # a well-formed turn_left intent
                        lambda body: {"cmd_id": "mi-1", "intent": "turn_left",  # turn_left 90
                                      "slots": {"angle_deg": 90}, "channel": "cloud"})  # its slots
    monkeypatch.setattr(miw, "evaluate", lambda cmd, **kw: _Pass())  # gates are not the subject

    class _Health:  # build_summary stub
        def build_summary(self, cfg):  # health summary stub
            return {}  # empty summary

    class _Pub:  # the relmove publisher
        def __init__(self):  # recorder
            self.sent = []  # forwarded relmoves

        def put(self, data):  # publisher.put
            self.sent.append(json.loads(data.decode("utf-8")))  # decode what P2 sent

    s = _setting(tmp_path)  # 0.6 default
    s.apply("slower")  # the operator said slower: 0.5
    pub = _Pub()  # relmove publisher
    ack = mw._handle_motion_intent(b"{}", None, {}, _Health(), None, pub, s)  # forward one intent
    assert ack["result"] == "accepted"  # gates passed
    assert pub.sent[-1]["max_yaw_rate_radps"] == 0.5 and pub.sent[-1]["dyaw_rad"] > 0  # left turn at 0.5
