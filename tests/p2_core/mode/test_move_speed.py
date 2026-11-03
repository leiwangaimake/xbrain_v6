"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_move_speed.py
Brief: The move speed (forward / back / strafe) defaults to 0.5 (also the slowest), steps by 0.1 up to 1.0, keeps an explicit value across restarts, and reaches every relmove as max_speed_mps
Description:
User ruling 2026-11-03 (99 U106): forward, back and both strafes use one move
speed, set by cloud voice / text like the volume and the turn speed; an
explicit value persists and can be set again; the numbers live in p2_core.yaml
move_speed; the floor must clear the body's linear control deadzones. Same day,
after the field test: 0.5 m/s is the default AND the slowest, 1.0 the fastest,
step 0.1 (at 0.2 the chassis strafed right at a third of the command). These cases cover:
  * MoveSpeedSetting: default, steps, bounds, persistence, re-setting, the
    deadzone check;
  * the dispatcher refuses an unknown level; ModeFace applies the action, acks
    the RESULT and publishes state/mode.move_speed;
  * the relmove P2 forwards carries max_speed_mps = the current value.
The bounds used here are the production p2_core.yaml move_speed block.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

import json  # frames and the state file

import pytest  # fixtures, marks
import yaml  # production config

from xbrain.p2_core.mode.move_speed import MoveSpeedSetting  # under test
from xbrain.p2_core.mode_actions.dispatch import dispatch  # the closed-set gate
from xbrain.p2_core.runtime.mode_wiring import ModeFace  # applies set_move_speed

pytestmark = pytest.mark.no_device  # pure in-process

with open("/opt/xbrain_v6/configs/p2_core.yaml", encoding="utf-8") as _fh:  # production config
    MS = yaml.safe_load(_fh)["move_speed"]  # the U106 block
with open("/opt/xbrain_v6/configs/models/m20s.yaml", encoding="utf-8") as _fh:  # body spec
    _SPEC = yaml.safe_load(_fh)["common"]["spec"]  # control deadzones (2026-11-03)
DZ = max(_SPEC["min_cmd_vx_mps"], _SPEC["min_cmd_vy_mps"])  # the wiring takes the larger


def _setting(tmp_path, name="ms.json"):  # a setting on the production bounds
    return MoveSpeedSetting(str(tmp_path / name), MS["min_mps"], MS["max_mps"],
                            MS["step_mps"], MS["default_mps"], DZ)  # fresh


def test_production_numbers_are_the_ruling():  # the ruled numbers
    """The user's numbers: default = slowest = 0.5, fastest 1.0, step 0.1; deadzones by reference."""
    assert (MS["default_mps"], MS["min_mps"], MS["max_mps"], MS["step_mps"]) == (0.5, 0.5, 1.0, 0.1)  # U106
    assert MS["deadzone_vx_mps"] == "${common.spec.min_cmd_vx_mps}"  # body spec, not a copy
    assert MS["deadzone_vy_mps"] == "${common.spec.min_cmd_vy_mps}"  # body spec, not a copy
    assert MS["min_mps"] > DZ  # the floor clears both deadzones


def test_default_is_the_configured_default_not_max(tmp_path):  # unlike the turn speed
    """MUTATION: default to max like TurnSpeedSetting -> 1.0 -> fails."""
    s = _setting(tmp_path)  # nothing stored
    assert s.effective() == 0.5 and s.origin() == "default"  # default 0.5
    assert s.snapshot() == {"mps": 0.5, "min_mps": 0.5, "max_mps": 1.0, "origin": "default"}  # state/mode.move_speed shape


def test_steps_bounds_and_at_limit(tmp_path):  # faster / slower / max / min
    s = _setting(tmp_path)  # 0.5 (default = slowest)
    assert s.apply("slower") == {"move_speed_mps": 0.5, "changed": False, "at_limit": "min"}  # already slowest
    assert s.apply("faster") == {"move_speed_mps": 0.6, "changed": True, "at_limit": None}  # one step up
    assert s.apply("max") == {"move_speed_mps": 1.0, "changed": True, "at_limit": "max"}  # "最快" = 1.0
    assert s.apply("faster") == {"move_speed_mps": 1.0, "changed": False, "at_limit": "max"}  # never above
    assert s.apply("min") == {"move_speed_mps": 0.5, "changed": True, "at_limit": "min"}  # floor


def test_explicit_value_survives_a_restart_and_can_be_set_again(tmp_path):  # persistence
    """MUTATION: skip the file write -> the new instance starts at 0.5 -> fails."""
    _setting(tmp_path).apply("faster")  # 0.6, persisted
    again = _setting(tmp_path)  # "restart"
    assert again.effective() == 0.6 and again.origin() == "explicit"  # restored
    assert again.apply("faster")["move_speed_mps"] == 0.7  # set again after the restart
    assert json.loads((tmp_path / "ms.json").read_text(encoding="utf-8")) == {"move_speed_mps": 0.7}  # on disk


@pytest.mark.parametrize("lo, dz", [(0.08, 0.08), (0.05, 0.08)])  # floor AT / below the deadzone
def test_floor_must_clear_the_deadzone(tmp_path, lo, dz):  # 2026-11-03 "死区抑制"
    """A move speed the chassis does not step at is a config error, not a setting."""
    with pytest.raises(ValueError):  # loud at start
        MoveSpeedSetting(str(tmp_path / "x.json"), lo, 0.5, 0.1, 0.2, dz)  # must raise


@pytest.mark.parametrize("dflt", [0.05, 0.6])  # default outside [min, max]
def test_default_outside_the_range_raises(tmp_path, dflt):  # config error
    with pytest.raises(ValueError):  # loud at start
        MoveSpeedSetting(str(tmp_path / "x.json"), 0.1, 0.5, 0.1, dflt, 0.08)  # must raise


def test_dispatcher_refuses_an_unknown_level():  # closed set
    """MUTATION: drop the closed-set check in _handle_set_move_speed -> fails."""
    assert dispatch({"action": "set_move_speed", "move_speed": "fastest"}, {}).accepted is False  # unknown
    assert dispatch({"action": "set_move_speed"}, {}).accepted is False  # missing
    assert dispatch({"action": "set_move_speed", "move_speed": "slower"}, {}).accepted is True  # known


def test_modeface_applies_acks_and_publishes(tmp_path):  # ack carries the result
    """MUTATION: drop the move_speed line in _publish_state -> fails."""
    sent = []  # (key, bytes)
    face = ModeFace(publish=lambda k, d: sent.append((k, d)), move_speed=_setting(tmp_path))  # wired face
    frame = json.dumps({"cmd_id": "c-ms", "action": "set_move_speed", "move_speed": "faster",
                        "source": "cloud"}).encode("utf-8")  # one ModeCommand
    ack = face.handle_frame(frame, now_mono_ms=1000)  # 0.5 -> 0.6
    assert ack["result"] == "accepted"  # accepted
    assert ack["detail"]["applied"] == {"move_speed_mps": 0.6, "changed": True, "at_limit": None}  # result
    body = json.loads(sent[-1][1].decode("utf-8"))  # the state/mode it published
    assert body["move_speed"]["mps"] == 0.6 and body["move_speed"]["origin"] == "explicit"  # state follows


def test_unwired_setting_is_refused_not_acked():  # no setting wired
    face = ModeFace(publish=lambda k, d: None)  # no move speed
    frame = json.dumps({"cmd_id": "c-ms", "action": "set_move_speed", "move_speed": "faster",
                        "source": "cloud"}).encode("utf-8")  # one ModeCommand
    assert face.handle_frame(frame, now_mono_ms=1000)["result"] == "rejected"  # never "done"


def test_relmove_carries_the_move_speed(tmp_path, monkeypatch):  # MO-2 param
    """MUTATION: drop params["max_speed_mps"] in _handle_motion_intent -> fails."""
    from xbrain.p2_core.runtime import main_wiring as mw  # under test
    from xbrain.p2_core.runtime import motion_intent_wiring as miw  # its gates

    class _Pass:  # every gate passed
        passed = True  # verdict.passed

    monkeypatch.setattr(miw, "parse_intent_envelope",  # a well-formed move_left intent
                        lambda body: {"cmd_id": "mi-1", "intent": "move_left",
                                      "slots": {"distance_m": 1.0}, "channel": "cloud"})  # its slots
    monkeypatch.setattr(miw, "evaluate", lambda cmd, **kw: _Pass())  # gates are not the subject

    class _Health:  # build_summary stub
        def build_summary(self, cfg):  # health summary stub
            return {}  # empty summary

    class _Pub:  # the relmove publisher
        def __init__(self):  # recorder
            self.sent = []  # forwarded relmoves

        def put(self, data):  # publisher.put
            self.sent.append(json.loads(data.decode("utf-8")))  # decode what P2 sent

    s = _setting(tmp_path)  # 0.5 default
    s.apply("faster")  # the operator said faster: 0.6
    pub = _Pub()  # relmove publisher
    ack = mw._handle_motion_intent(b"{}", None, {}, _Health(), None, pub, None, move_speed=s)  # forward one
    assert ack["result"] == "accepted"  # gates passed
    assert pub.sent[-1]["max_speed_mps"] == 0.6 and pub.sent[-1]["dy_m"] > 0  # left at 0.6
