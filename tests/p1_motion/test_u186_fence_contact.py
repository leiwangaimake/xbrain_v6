"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u186_fence_contact.py
Brief: 99 U186 -- boundary contact repeats, speed-zone entry / exit, route recording stops at the line, keyboard only while recording

Description:
The user's ruling of 2026-11-13 (99 U186), each rule pinned by a case that
names the mutant it kills:
  * hitting the activity area / a forbid zone (撞线) is reported at once, then
    every repeat_s while it lasts, and ends with hysteresis (distance or time);
    a fence recording tells it once; no geometry freezes the state;
  * entering / leaving a 限速区域 is reported like a 报警区域, with its limit;
  * a ROUTE recording stops the cloud keyboard at the effective boundary, with
    no slow-down band before it; a FENCE recording is not stopped;
  * the cloud keyboard drives only while a recording runs; a refusal is
    reported once per episode.
"""
from __future__ import annotations  # postponed annotations

import json  # event bodies
import threading  # the tracker lock
from dataclasses import replace  # frozen NavInputs
from types import SimpleNamespace  # a minimal NavRuntime self

import pytest  # marker, approx

from tests.p1_motion.nav.test_nav_tick_fence import FENCE, OK, _stack  # the real tick with a fence
from tests.p1_motion.rns.scenes import healthy_status, snapshot, uniform_free  # perception
from xbrain.p1_motion.fence.clip import PolyDistance  # one polygon's numbers
from xbrain.p1_motion.fence.contact import ContactTracker  # under test
from xbrain.p1_motion.fence.zones import ZoneTracker  # under test
from xbrain.p1_motion.nav.nav_tick import NavInputs  # the tick snapshot
from xbrain.p1_motion.runtime.nav_wiring import NavRuntime  # _teleop_drive / _on_teach
from xbrain.p1_motion.sources.arbiter_p1 import BehaviorSource  # teleop source
from xbrain.p1_motion.teleop.state import TeleopTracker  # arbitration
from xbrain.p1_motion.teleop.velocity import TeleopSpeeds  # voice speeds

pytestmark = pytest.mark.no_device  # pure logic


def _fev(*polys):  # a FenceEval-shaped value: only per_poly is read
    return SimpleNamespace(per_poly=tuple(polys))  # the tracker reads nothing else


def _pd(pid, role, hard, d_eff, name="北门禁止区域围栏"):  # one PolyDistance
    return PolyDistance(pid, name, role, True, d_eff + 0.3, d_eff, (1.0, 0.0), hard)


# -- contact ------------------------------------------------------------------
def test_a_contact_starts_repeats_every_period_and_ends_by_distance():
    """MUTATION: hold on every tick -> 20 rows a second -> red; never hold ->
    red; end without the distance rule -> stays open at 0.6 m -> red."""
    ct = ContactTracker(3.0, 0.5, 2.0)  # the configured 3 s / 0.5 m / 2 s
    on = _fev(_pd("f-pit", "forbid", True, -0.1))  # held at the line
    assert [e.kind for e in ct.observe(on, now_s=10.0, rev=1, recording=None)] == ["contact_start"]
    assert ct.observe(on, now_s=11.0, rev=1, recording=None) == []  # within the period
    hold = ct.observe(on, now_s=13.0, rev=1, recording=None)  # one period later
    assert [(e.kind, e.episode_id) for e in hold] == [("contact_hold", 0)]  # same contact
    off = ct.observe(_fev(_pd("f-pit", "forbid", False, 0.6)), now_s=13.5, rev=1, recording=None)
    assert [(e.kind, e.severity, e.episode_id) for e in off] == [("contact_end", "info", 0)]
    again = ct.observe(on, now_s=20.0, rev=1, recording=None)  # a new contact
    assert [(e.kind, e.episode_id) for e in again] == [("contact_start", 1)]  # next id


def test_a_contact_ends_after_the_release_time_close_to_the_line():
    """Off the line but within 0.5 m: ends only after 2 s of not touching it.
    MUTATION: end on the first clear tick -> toggling on the line -> red."""
    ct = ContactTracker(3.0, 0.5, 2.0)  # configured
    ct.observe(_fev(_pd("f-pit", "forbid", True, 0.0)), now_s=1.0, rev=1, recording=None)  # start
    near = _fev(_pd("f-pit", "forbid", False, 0.2))  # just off, still close
    assert ct.observe(near, now_s=1.1, rev=1, recording=None) == []  # clock starts
    assert ct.observe(near, now_s=2.5, rev=1, recording=None) == []  # 1.4 s: not yet
    assert [e.kind for e in ct.observe(near, now_s=3.2, rev=1, recording=None)] == ["contact_end"]


def test_a_fence_recording_tells_a_contact_once():
    """Driving along a boundary to record it is the point. MUTATION: repeat
    during a fence recording -> red; drop the recording tag -> P4 words it as
    autonomous -> red."""
    ct = ContactTracker(3.0, 0.5, 2.0)  # configured
    on = _fev(_pd("f-pit", "forbid", True, -0.2))  # past the line
    first = ct.observe(on, now_s=1.0, rev=1, recording="fence")  # told once
    assert [(e.kind, e.recording) for e in first] == [("contact_start", "fence")]
    assert ct.observe(on, now_s=10.0, rev=1, recording="fence") == []  # never repeated


def test_rev_change_or_a_vanished_polygon_ends_the_contact_and_no_geometry_freezes():
    """MUTATION: keep contacts across a new set -> an end never comes -> red;
    end on a degraded tick (no per_poly) -> red."""
    ct = ContactTracker(3.0, 0.5, 2.0)  # configured
    ct.observe(_fev(_pd("f-pit", "forbid", True, 0.0)), now_s=1.0, rev=1, recording=None)
    assert ct.observe(_fev(), now_s=9.0, rev=1, recording=None) == []  # degraded: frozen
    ended = ct.observe(_fev(_pd("f-new", "allow", False, 5.0)), now_s=9.1, rev=2, recording=None)
    assert [(e.kind, e.poly_id) for e in ended] == [("contact_end", "f-pit")]  # closed with the set
    assert ct.observe(_fev(_pd("w", "warning", True, -1.0)), now_s=9.2, rev=2, recording=None) == []


# -- speed zones ----------------------------------------------------------------
def test_entering_and_leaving_a_speed_zone_is_reported_with_its_limit():
    """MUTATION: speed zones dropped from the tracker -> no event -> red; the
    role hard-coded warning -> P4 says 报警区域 -> red."""
    zt = ZoneTracker()  # fresh
    ring = ((31.0, 121.0), (31.001, 121.0), (31.001, 121.001), (31.0, 121.001))  # lat/lon
    zone = SimpleNamespace(poly_id="f-yard", name="路口限速区域围栏", vertices=ring,
                           role="speed_limit", speed_limit_mps=0.6)  # a HeldPolygon shape
    enter = zt.observe(31.0005, 121.0005, [zone])  # inside
    assert [(e.kind, e.severity, e.role, e.speed_limit_mps) for e in enter] == [
        ("zone_enter", "info", "speed_limit", 0.6)]  # a hint, not an alarm
    leave = zt.observe(31.01, 121.0005, [zone])  # outside
    assert [(e.kind, e.role) for e in leave] == [("zone_exit", "speed_limit")]


# -- route recording stops at the line ------------------------------------------
def _kbd(now, vx, pose, recording):  # the cloud keyboard driving forward
    perc = snapshot(uniform_free(6.0, t_capture_mono_ms=now, t_seg_mono_ms=now - 10),
                    None, healthy_status(t_publish_mono_ms=now))  # perception ok
    inp = NavInputs(now_mono_ms=now, pose_xy=pose, yaw_rad=0.0, heading_valid=True,
                    i_fix=1.0, i_heading=1.0, perception=perc, health=OK, estop=False,
                    teleop_active=True, fix_type="rtk_fixed", fence=FENCE)  # inside an allow square
    return replace(inp, teleop_cmd=(vx, 0.0, 0.0), teleop_source=BehaviorSource.TELEOP_KEYBOARD,
                   teleop_unconditional=True, recording_kind=recording)  # 99 U150 / U186


def test_a_route_recording_stops_at_the_boundary_with_no_slowdown_before_it():
    """The square's east edge is x = 8 (effective 7.7 with the rtk_fixed
    inset). MUTATION: no clip for a route recording -> 1.0 at the line -> red;
    the soft band applied -> slower than 1.0 at x = 5 -> red; clip a fence
    recording too -> red."""
    tick = _stack()  # real tick with fence constants
    # x = 7.2: d_eff 0.5 m, inside the soft band, where the autonomous clip
    # would cap the speed near 0.63 m/s; not yet at the line (predicted 7.48)
    band = tick.run(_kbd(5000, 1.0, (7.2, 0.0), "route"))
    assert band.vx == pytest.approx(1.0)  # no slow-down band before the line
    at = tick.run(_kbd(5100, 1.0, (7.8, 0.0), "route"))  # at the effective line
    assert at.vx == pytest.approx(0.0) and at.limiter == "fence"  # stopped there
    fence_rec = tick.run(_kbd(5200, 1.0, (7.8, 0.0), "fence"))  # recording a fence
    assert fence_rec.vx == pytest.approx(1.0)  # not stopped (only told)


# -- the keyboard only while recording -------------------------------------------
def _me(teach, kind):  # a minimal NavRuntime self for _teleop_drive
    puts = []  # what it published
    me = SimpleNamespace(_teleop_lock=threading.Lock(), _teleop=TeleopTracker(),
                         _teleop_speeds=TeleopSpeeds(move_mps=0.5, turn_radps=0.6),
                         _cfg=SimpleNamespace(holonomic=True, recording_state_stale_ms=3000),
                         _teleop_unconditional=False, _teach=teach, _teach_kind=kind,
                         _kbd_refused_ms=None, _seq={"event": 0}, _event_boot="b0",
                         _counts={"publish_fail": 0},
                         _gen=SimpleNamespace(put=lambda k, v: puts.append((k, json.loads(v)))))
    me._recording_kind_now = lambda now: NavRuntime._recording_kind_now(me, now)  # bound
    me._note_keyboard_refused = lambda now: NavRuntime._note_keyboard_refused(me, now)
    me._teleop.observe("keyboard_cloud", now_mono_ms=1000, deadman=True, axes={"wz": 1.0})
    return me, puts


def test_the_cloud_keyboard_drives_only_while_a_recording_runs():
    """"其他情况下键盘开车是被系统禁止的". MUTATION: drop the gate -> it
    drives at the voice speed -> red; report every refused frame -> red."""
    me, puts = _me((False, 0.0), None)  # no recording
    assert NavRuntime._teleop_drive(me, 1100) == (None, None)  # refused
    assert NavRuntime._teleop_drive(me, 1150) == (None, None)  # still refused, same episode
    assert [(k, b["detail"]["kind"]) for k, b in puts] == [("event/warn/teleop", "keyboard_refused")]
    rec, _ = _me((True, 1.0), "route")  # a recording heard at t = 1.0 s
    cmd, _src = NavRuntime._teleop_drive(rec, 1100)  # within the freshness window
    assert cmd == pytest.approx((0.0, 0.0, 0.6))  # drives


def test_on_teach_keeps_the_recording_kind():
    """MUTATION: kind not read -> a fence recording is clipped like a route
    -> red (the recording_kind feeds nav_tick)."""
    me = SimpleNamespace(_teach=(False, 0.0), _teach_kind=None, _teach_seen=(None, None))
    frame = {"session": {"session_id": "ts-1", "state": "recording", "kind": "fence"}}
    NavRuntime._on_teach(me, SimpleNamespace(payload=json.dumps(frame).encode("utf-8")))
    assert me._teach[0] is True and me._teach_kind == "fence"  # kept


def test_main_wiring_feeds_speed_zones_to_the_zone_tracker():
    """The zone loop runs inside p1's main thread (no harness), so its one
    U186 change is checked in the source. MUTATION: only warning polygons ->
    a speed zone is never announced -> red."""
    from pathlib import Path  # static read
    src = (Path(__file__).resolve().parents[2] / "xbrain" / "p1_motion" / "runtime"
           / "main_wiring.py").read_text(encoding="utf-8")  # the wiring
    assert "active.warning_polygons() + active.speed_limit_polygons()" in src  # both kinds
    assert '"role": ev.role' in src  # the role travels, not a hard-coded warning
