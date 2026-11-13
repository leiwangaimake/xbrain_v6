"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_stall_diag.py
Brief: 99 U195 diagnostics -- a route run standing still logs what it saw, once per window

Description:
User report 2026-11-13: frequent mid-route stalls the P1 log could not
explain. runtime/stall_diag decides when to write a line (start after
START_AFTER_S still, repeat every REPEAT_S, end when it moves) and what goes
in it (hold reason, limiter, pose, front / left / right blocked distances,
nearest objects, newest RNS audit records). Each case names the mutant it
kills.
"""
from __future__ import annotations  # postponed annotations

import math  # profile bin angles
from pathlib import Path  # the wiring source, read statically
from types import SimpleNamespace  # duck-typed out / pose / snapshot

import pytest  # parametrize

from xbrain.p1_motion.rns.audit import AuditRecord  # the audit record shape
from xbrain.p1_motion.runtime.stall_diag import (  # under test
    REPEAT_S, START_AFTER_S, StallDiag, describe_stall)  # the three timings and the two entry points

pytestmark = pytest.mark.no_device  # pure logic


def test_a_stall_logs_start_then_repeat_then_end():
    """MUTATION: log on the first still tick -> a normal stop logs -> red;
    no repeat -> a 4 min stall writes one line -> red; no end -> red."""
    d = StallDiag()  # a fresh tracker
    assert d.observe(0.0, True, True) is None  # the stall begins
    assert d.observe(START_AFTER_S - 0.1, True, True) is None  # still a normal stop
    assert d.observe(START_AFTER_S, True, True) == "start"  # 3 s still: the first line
    assert d.observe(START_AFTER_S + REPEAT_S - 0.1, True, True) is None  # inside the repeat window: quiet
    assert d.observe(START_AFTER_S + REPEAT_S, True, True) == "repeat"  # one window later: the stall is still on
    assert d.duration_s(START_AFTER_S + REPEAT_S) == pytest.approx(START_AFTER_S + REPEAT_S)
    assert d.observe(20.0, True, False) == "end"  # it moves again
    assert d.observe(20.1, True, False) is None and d.duration_s(20.1) is None


def test_a_short_stop_or_no_route_writes_nothing():
    """MUTATION: end without a start -> every short stop logs "end" -> red;
    count a stall outside a route run -> red."""
    d = StallDiag()  # a fresh tracker
    d.observe(0.0, True, True)  # the stop begins
    assert d.observe(1.0, True, False) is None  # 1 s stop: not a stall
    assert d.observe(10.0, False, True) is None  # idle robot: not a route stall
    assert d.observe(20.0, False, True) is None  # still idle later: still nothing


def _profile(blocked):
    """A 181-bin profile from -90 to +90 deg; blocked: {deg: distance}."""
    n = 181  # one bin per degree
    d_block = [None] * n  # nothing blocked by default
    for deg, dist in blocked.items():  # the requested hits
        d_block[deg + 90] = dist  # bin index = angle + 90
    return SimpleNamespace(angle_min_rad=math.radians(-90.0), angle_step_rad=math.radians(1.0),  # -90 deg start, 1 deg bins
                           d_block=tuple(d_block), d_free=tuple([5.0] * n))  # every ray observed: unk=0


def _obj(cls, conf, x, y):
    return SimpleNamespace(class_name=cls, confidence=conf, localized=True,  # a placed object, footprint around (x, y)
                           footprint_xy=((x - 0.1, y - 0.1), (x + 0.1, y - 0.1), (x, y + 0.1)))  # triangle centred near (x, y)


def test_the_line_names_the_hold_the_view_the_objects_and_the_audit():
    """MUTATION: wrong sector bounds -> the 0.6 m front hit reads under left
    -> red; objects unsorted -> the far car first -> red; drop the audit
    tail -> red."""
    out = SimpleNamespace(rns_hold="no_way_through", limiter="free_space", nav_state="follow",  # the U195 hold as P1 reports it
                          raw_vx=0.0, raw_vy=0.0, raw_wz=0.0, v_max=0.0)  # nothing commanded, cap zero
    pose = SimpleNamespace(xy=(3.0, -4.0), yaw=0.0, fix_type="rtk_fixed",  # a pose with fix and heading
                           heading_source="dual_antenna")  # the dual antenna, as on the robot
    snap = SimpleNamespace(profile=_profile({5: 0.6, 60: 2.0}),  # front hit 0.6 m, left hit 2 m
                           objects=SimpleNamespace(objects=(_obj("car", 0.9, 8.0, 0.0),  # a car 8 m ahead
                                                            _obj("person", 0.8, 1.0, 1.0))))  # a person 1.4 m to the front-left
    audit = [AuditRecord(1, "detour_enter", {"via": "x"}), AuditRecord(2, "objects_unlocalized", {"n": 1})]  # two RNS decisions
    line = describe_stall(out, snap, pose, audit, 12.5)  # the line under test
    assert "dur=12.5s hold=no_way_through limiter=free_space nav=follow" in line  # the head fields
    assert "front=0.60" in line and "left=2.00" in line and "right=-" in line  # sector bounds put each hit where it belongs
    assert line.index("person:0.80") < line.index("car:0.90")  # nearest first
    assert "objects_unlocalized" in line and "detour_enter" in line  # the audit tail is drained in
    assert "pose=(3.0,-4.0)" in line and "fix=rtk_fixed" in line  # pose and fix


def test_missing_view_and_objects_say_so():
    """MUTATION: crash on a None snapshot -> red."""
    out = SimpleNamespace(rns_hold=None, limiter="none", nav_state="follow",  # no hold, gate idle
                          raw_vx=0.0, raw_vy=0.0, raw_wz=0.0, v_max=0.0)  # all zero
    line = describe_stall(out, None, SimpleNamespace(xy=None, yaw=None), [], None)  # no snapshot, no pose, no audit
    assert "profile=none" in line and "objects=none" in line and "audit=none" in line  # each missing part named


def test_the_wiring_logs_route_stalls_with_the_rns_audit():
    """MUTATION: drop the hook, or stop draining the RNS audit -> red."""
    src = (Path(__file__).resolve().parents[2] / "xbrain" / "p1_motion" / "runtime"  # the wiring source, read statically
           / "nav_wiring.py").read_text(encoding="utf-8")  # utf-8: the file has Chinese comments
    assert 'self._host.route_state == "running" and out.source == "rns_avoid"' in src  # the hook condition
    assert "self._src.rns.audit.drain()" in src  # the RNS audit reaches the log
    assert '_logger.warning("p1 stall %s: %s"' in src  # the log line itself
