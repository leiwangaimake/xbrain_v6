"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_chassis_ctrl_stamp.py
Brief: p4's cmd/chassis/ctrl leaves as a full S3.0 envelope that quadruped's reader accepts

Description:
2026-10-26 on .1.26: a bare cmd/chassis/ctrl body went through chassis_relay
(which wraps bare bodies but cannot add a rid) and quadruped refused it as
E_SCHEMA with cmd_id "anonymous" -- so the voice "站起来" / "趴下" that
dcd44c7 had moved to this key was still never executed. The publish point
now wraps the body with ChassisCtrlStamper. Checked here:

  * the stamped frame carries every field quadruped's read_envelope
    requires (v=1, rid, src, unsigned seq, numeric ts, boolean ts_sync,
    object data), decodes with the shared S3.0 decoder, and keeps the body
    as data unchanged;
  * seq starts at 0 and increases per frame (S3.0 per-key seq);
  * mono and boot travel as a pair (CLK-C4);
  * no rid, or a body that is not a JSON object, gives None (not sent);
  * main_wiring's _publish routes cmd/chassis/ctrl through the stamper and
    drops the frame when it returns None (read from the source: the closure
    cannot be called without Zenoh).
"""
from __future__ import annotations

import ast  # read main_wiring's _publish
import json  # frames
import pathlib  # source path

import pytest  # marker

from xbrain.common.envelope.envelope import decode  # the shared S3.0 decoder
from xbrain.p4_agent.runtime.chassis_ctrl_stamp import SRC, ChassisCtrlStamper

pytestmark = pytest.mark.no_device  # runs anywhere

#: A body exactly as decision_to_publishes builds it.
_BODY = {"cmd_id": "cc-0123456789ab", "action": "stand", "origin": "voice"}


def _stamp(stamper, body=_BODY):
    """Stamp one body; decoded dict or None."""
    out = stamper.stamp(json.dumps(body).encode("utf-8"))  # wire bytes
    return None if out is None else json.loads(out)  # decoded


def test_the_frame_has_every_field_quadruped_requires():
    """rt_parse read_envelope: v, rid, src, seq (uint), ts, ts_sync, data."""
    f = _stamp(ChassisCtrlStamper("gj-001", "513b59e6"))  # one frame
    assert f["v"] == 1 and f["rid"] == "gj-001" and f["src"] == SRC  # identity
    assert isinstance(f["seq"], int) and f["seq"] >= 0  # unsigned
    assert isinstance(f["ts"], float)  # seconds
    assert f["ts_sync"] is False  # present, boolean, fail-safe value
    assert f["data"] == _BODY  # the body, unchanged
    env = decode(f)  # the shared decoder accepts it too
    assert (env.rid, env.data["action"]) == ("gj-001", "stand")


def test_seq_counts_from_zero():
    """S3.0 per-key seq: 0, 1, 2 for one stamper."""
    s = ChassisCtrlStamper("gj-001", "513b59e6")  # one key, one counter
    assert [_stamp(s)["seq"] for _ in range(3)] == [0, 1, 2]  # from 0, +1


def test_mono_travels_with_boot():
    """CLK-C4: mono only together with its boot domain."""
    f = _stamp(ChassisCtrlStamper("gj-001", "513b59e6"))  # one frame
    assert f["boot"] == "513b59e6" and isinstance(f["mono"], float)  # the pair


@pytest.mark.parametrize("rid,raw", [
    (None, json.dumps(_BODY)), ("", json.dumps(_BODY)),  # no robot id
    ("gj-001", "[1, 2]"), ("gj-001", "not json")])  # not an object
def test_unsendable_frames_give_none(rid, raw):
    """Nothing to send rather than a frame quadruped would refuse."""
    assert ChassisCtrlStamper(rid, "513b59e6").stamp(raw.encode("utf-8")) is None  # not sent


def test_publish_routes_chassis_ctrl_through_the_stamper():
    """_publish: stamp for CMD_CHASSIS_CTRL, return without put on None."""
    src = (pathlib.Path(__file__).resolve().parents[3] / "xbrain" / "p4_agent"  # repo path ...
           / "runtime" / "main_wiring.py").read_text(encoding="utf-8")  # ... of the wiring
    fn = [f for f in ast.walk(ast.parse(src))  # every function ...
          if isinstance(f, ast.FunctionDef) and f.name == "_publish"]  # ... named _publish
    assert len(fn) == 1  # the one publish point
    ifs = [n for n in ast.walk(fn[0]) if isinstance(n, ast.If)  # branches ...
           and isinstance(n.test, ast.Compare)  # ... on a comparison ...
           and getattr(n.test.comparators[0], "id", "") == "CMD_CHASSIS_CTRL"]  # ... with the key
    assert ifs, "_publish no longer special-cases CMD_CHASSIS_CTRL"
    body = ifs[0].body  # the branch
    calls = [getattr(n.func, "attr", "") for n in ast.walk(ast.Module(body=body, type_ignores=[]))  # methods ...
             if isinstance(n, ast.Call)]  # ... called in the branch
    assert "stamp" in calls  # wrapped
    returns = [n for n in ast.walk(ast.Module(body=body, type_ignores=[]))  # returns ...
               if isinstance(n, ast.Return)]  # ... in the branch
    assert returns  # and dropped when the stamper says None
    assigned = [t.id for n in ast.walk(ast.Module(body=body, type_ignores=[]))  # names ...
                if isinstance(n, ast.Assign) for t in n.targets  # ... assigned ...
                if isinstance(t, ast.Name)]  # ... in the branch
    assert "payload_bytes" in assigned  # the stamped bytes are what gets put
