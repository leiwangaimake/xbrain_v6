"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mission_host.py
Brief: MissionHost over the real RnsSource -- both origins, every shell rule, report routing

Description:
A tiny closed loop (NavTick -> integrate pose -> MissionHost.after_tick) drives
the shell end to end with the production RnsSource: a relative_move is
accepted, reported at 2 Hz with the ACTUAL displacement and ends succeeded; a
route arrives; estop aborts a relmove (soft_estop) but only suspends a route
(aborted -> running on release, mission kept); teleop / timeout / a newer
command each abort a relmove with the 12 S4.5.1 reason; an RNS failure is
routed by ORIGIN -- path_progress failed + verbatim reason for a route,
abort_reason input_lost/extrinsic for a relmove, an event either way; clear
cancels with no failure.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import List

import pytest
import yaml

from tests.p1_motion.rns.scenes import (healthy_status, one_object, snapshot,
                                        uniform_free)
from xbrain.common.errors import E_CAPABILITY
from xbrain.p1_motion.nav.health_factor import HealthView
from xbrain.p1_motion.nav.mission_host import (CH_EVENT, CH_PROGRESS,
                                               CH_RELMOVE, Emit, MissionHost)
from xbrain.p1_motion.nav.nav_tick import NavInputs, NavTick
from xbrain.p1_motion.nav.relmove_intake import RelMoveLimits
from xbrain.p1_motion.nav.route_intake import RouteClear, RouteSet
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import NavState
from xbrain.p1_motion.sources.arbiter_p1 import P1Arbiter
from xbrain.p1_motion.sources.rns_avoid import RnsAvoidSource

pytestmark = pytest.mark.no_device

_ROOT = Path(__file__).resolve().parents[3]
_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))
OK = HealthView(1.0, True, "patrol", "ok", 100)
LIM = RelMoveLimits(20.0, 6.2832, 0.02, 20.0, True)
DT_MS = 50


class Sim:
    """NavTick + MissionHost + a unicycle integrator (the SIL kinematics)."""

    def __init__(self):
        cfg = copy.deepcopy(_CFG)
        self.src = RnsAvoidSource(RnsSource(cfg=cfg, r_eff_m=0.5), cfg["rns"])
        self.tick = NavTick(self.src, P1Arbiter(dwell_ms=200), v_nom_mps=1.0,
                            wz_max_rps=1.2, spec_max_vx_mps=2.0, holonomic=True)
        self.now = 5000
        self.host = MissionHost(self.src, relmove_limits=LIM, holonomic=True,
                                now_mono_ms=self.now)
        self.x = self.y = self.yaw = 0.0

    person_ahead = False     # a confirmed person closing at 1 m -> WAIT_DYNAMIC

    def snap(self, extrinsic=True):
        objs = None
        if self.person_ahead:
            objs = one_object(class_name="person", r_near=1.0, velocity_xy=(-1.0, 0.0),
                              velocity_status="moving", t_capture_mono_ms=self.now - 20)
        return snapshot(uniform_free(6.0, t_capture_mono_ms=self.now, t_seg_mono_ms=self.now - 10,
                                     extrinsic_calibrated=extrinsic), objs,
                        healthy_status(t_publish_mono_ms=self.now, extrinsic_calibrated=extrinsic))

    def step(self, n=1, *, estop=False, teleop=False, extrinsic=True) -> List[Emit]:
        emits: List[Emit] = []
        for _ in range(n):
            self.now += DT_MS
            inp = NavInputs(now_mono_ms=self.now, pose_xy=(self.x, self.y), yaw_rad=self.yaw,
                            heading_valid=True, i_fix=1.0, i_heading=1.0,
                            perception=self.snap(extrinsic), health=OK, estop=estop,
                            teleop_active=teleop, ts_wall_s=1.5)
            out = self.tick.run(inp)
            emits += self.host.after_tick(inp, out)
            dt = DT_MS / 1000.0
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += (out.vx * c - out.vy * s) * dt
            self.y += (out.vx * s + out.vy * c) * dt
            self.yaw = math.atan2(math.sin(self.yaw + out.wz * dt), math.cos(self.yaw + out.wz * dt))
        return emits

    def relmove(self, dx=2.0, dy=0.0, dyaw=0.0, **kw) -> List[Emit]:
        body = {"cmd_id": "rm-1", "dx_m": dx, "dy_m": dy, "dyaw_rad": dyaw}
        body.update(kw)
        return self.host.on_relmove(body, now=self.now, pose=(self.x, self.y),
                                    yaw_rad=self.yaw, heading_valid=True)

    def route(self, points, rev=1) -> List[Emit]:
        rs = RouteSet(cmd_id="rg-%d" % rev, route_id="r-1", route_rev=rev, loop_mode="oneway",
                      total_len_m=0.0, points_xy=tuple(points),
                      arrive_radius_m=(0.8,) * len(points))
        return self.host.on_route(rs, self.now, (self.x, self.y))

    def run_until(self, pred, max_ticks=600, **kw) -> List[Emit]:
        seen: List[Emit] = []
        for _ in range(max_ticks):
            seen += self.step(**kw)
            if any(pred(e) for e in seen):
                return seen
        return seen


def _ch(emits, ch):
    return [e for e in emits if e.channel == ch]


def test_relmove_accepted_running_succeeded_with_actual_displacement():
    sim = Sim()
    acc = sim.relmove(dx=2.0)
    assert [e.body["state"] for e in _ch(acc, CH_RELMOVE)] == ["accepted"]
    seen = sim.run_until(lambda e: e.channel == CH_RELMOVE and e.body["state"] == "succeeded")
    states = [e.body["state"] for e in _ch(seen, CH_RELMOVE)]
    assert states[-1] == "succeeded" and "running" in states
    done = _ch(seen, CH_RELMOVE)[-1].body["progress"]["dx_done_m"]
    assert 1.0 < done <= 2.2            # arrival radius 0.8 around the 2 m goal
    assert sim.host.goal is None and sim.src.nav_state() is NavState.IDLE


def test_relmove_reject_is_a_status_with_code():
    sim = Sim()
    out = sim.relmove(dx=0.0, dyaw=1.57)
    assert len(out) == 1 and out[0].channel == CH_RELMOVE
    assert out[0].body["state"] == "rejected" and out[0].body["code"] == E_CAPABILITY
    assert out[0].body["detail"] == {"item": "nav2_spin"}
    assert sim.host.goal is None


def test_estop_aborts_relmove_soft_estop():
    """mutant: skip the estop branch -> the relmove keeps 'running' under
    estop -> red."""
    sim = Sim()
    sim.relmove(dx=5.0)
    sim.step(10)
    ems = _ch(sim.step(estop=True), CH_RELMOVE)
    assert [e.body["state"] for e in ems] == ["aborted"]
    assert ems[0].body["abort_reason"] == "soft_estop"
    assert sim.host.goal is None and sim.src.nav_state() is NavState.IDLE


def test_teleop_aborts_relmove_preempted():
    sim = Sim()
    sim.relmove(dx=5.0)
    sim.step(5)
    ems = _ch(sim.step(teleop=True), CH_RELMOVE)
    assert ems[0].body["abort_reason"] == "preempted" and ems[0].body["detail"]["item"] == "teleop"


def test_relmove_timeout_aborts():
    """mutant: timeout compared in the wrong unit -> never fires -> red."""
    sim = Sim()
    sim.relmove(dx=15.0, timeout_s=1.0)
    seen = sim.run_until(lambda e: e.channel == CH_RELMOVE and e.body["state"] == "aborted",
                         max_ticks=60)
    ab = [e for e in _ch(seen, CH_RELMOVE) if e.body["state"] == "aborted"]
    assert ab and ab[0].body["abort_reason"] == "timeout"
    assert 1000 <= sim.now - 5000 <= 1200


def test_route_set_preempts_relmove():
    """mutant: route set leaves the relmove untouched -> red."""
    sim = Sim()
    sim.relmove(dx=5.0)
    sim.step(5)
    ems = sim.route([(0.0, 0.0), (3.0, 0.0)])
    rm = _ch(ems, CH_RELMOVE)
    assert rm and rm[0].body["state"] == "aborted" and rm[0].body["detail"]["item"] == "route"
    pg = _ch(ems, CH_PROGRESS)
    assert pg and pg[-1].body["state"] == "running" and pg[-1].body["route_rev"] == 1
    assert sim.host.goal is None


def test_route_runs_reports_progress_and_arrives():
    """mutant: no host tracker -> dist_done_m never grows -> red."""
    sim = Sim()
    sim.route([(0.0, 0.0), (2.5, 0.0)])
    seen = sim.run_until(lambda e: e.channel == CH_PROGRESS and e.body["state"] == "arrived")
    pg = _ch(seen, CH_PROGRESS)
    assert pg[-1].body["state"] == "arrived" and pg[-1].body["waypoint_index"] == 1
    running = [e.body for e in pg if e.body["state"] == "running"]
    assert running and max(b["dist_done_m"] for b in running) > 0.5
    assert all(b["waypoint_index"] == 0 for b in running)
    assert sim.src.nav_state() is NavState.IDLE


def test_route_is_suspended_not_cancelled_by_estop():
    """mutant: never mark the route aborted on suspension -> red."""
    sim = Sim()
    sim.route([(0.0, 0.0), (6.0, 0.0)])
    sim.step(5)
    pg = _ch(sim.step(estop=True), CH_PROGRESS)
    assert pg and pg[-1].body["state"] == "aborted" and pg[-1].body["fail_reason"] is None
    assert sim.src.mission_loaded() and sim.src.nav_state() is NavState.SUSPENDED
    pg = _ch(sim.step(), CH_PROGRESS)
    assert pg and pg[-1].body["state"] == "running"
    assert sim.src.nav_state() is NavState.FOLLOW


def test_route_failure_is_failed_with_verbatim_reason_and_event():
    """mutant: write a generic word instead of the 20 S9.0.2 reason -> red."""
    sim = Sim()
    sim.route([(0.0, 0.0), (6.0, 0.0)])
    sim.step(3)
    ems = sim.step(extrinsic=False)
    pg = [e for e in _ch(ems, CH_PROGRESS) if e.body["state"] == "failed"]
    assert pg and pg[0].body["fail_reason"] == "extrinsic_uncalibrated"
    ev = _ch(ems, CH_EVENT)
    assert ev and ev[0].severity == "fault" and ev[0].body["detail"]["origin"] == "route"
    assert sim.src.nav_state() is NavState.IDLE


def test_relmove_failure_maps_to_abort_reason_and_event():
    sim = Sim()
    sim.relmove(dx=6.0)
    sim.step(3)
    ems = sim.step(extrinsic=False)
    rm = [e for e in _ch(ems, CH_RELMOVE) if e.body["state"] == "aborted"]
    assert rm and rm[0].body["abort_reason"] == "input_lost"
    assert rm[0].body["detail"]["item"] == "extrinsic"
    assert _ch(ems, CH_EVENT)[0].severity == "fault"


def test_clear_cancels_without_failure():
    sim = Sim()
    sim.route([(0.0, 0.0), (6.0, 0.0)])
    sim.step(3)
    ems = sim.host.on_route(RouteClear("rg-9", "r-1", 1), sim.now, (sim.x, sim.y))
    assert _ch(ems, CH_PROGRESS)[-1].body["state"] == "idle"
    assert sim.src.nav_state() is NavState.IDLE
    assert not _ch(sim.step(3), CH_EVENT)
    assert "cancelled" in [r.kind for r in sim.src.rns.audit.drain()]


def test_periodic_stream_while_idle():
    sim = Sim()
    pg = _ch(sim.step(21), CH_PROGRESS)
    assert len(pg) >= 2 and all(b.body["state"] == "idle" for b in pg)
    assert pg[0].body["ts"] == 1.5


def test_abort_on_obstacle_stops_and_tells():
    """12 S4.5.6 true row: RNS enters an avoidance state (here WAIT_DYNAMIC for
    a person closing in) -> the relmove is aborted obstacle with the state in
    detail.item, NOT detoured. mutant: drop the rule -> the relmove stays
    running while RNS waits -> red."""
    sim = Sim()
    sim.relmove(dx=6.0)
    sim.step(3)
    sim.person_ahead = True
    seen = sim.run_until(lambda e: e.channel == CH_RELMOVE and e.body["state"] == "aborted",
                         max_ticks=40)
    ab = [e for e in _ch(seen, CH_RELMOVE) if e.body["state"] == "aborted"]
    assert ab and ab[0].body["abort_reason"] == "obstacle"
    assert ab[0].body["detail"]["item"] == NavState.WAIT_DYNAMIC.value
    assert sim.host.goal is None and sim.src.nav_state() is NavState.IDLE


def test_abort_on_obstacle_false_lets_rns_handle_it():
    sim = Sim()
    sim.relmove(dx=6.0, abort_on_obstacle=False)
    sim.step(3)
    sim.person_ahead = True
    seen = sim.step(20)
    assert not [e for e in _ch(seen, CH_RELMOVE) if e.body["state"] == "aborted"]
    assert sim.src.nav_state() is NavState.WAIT_DYNAMIC and sim.host.goal is not None
