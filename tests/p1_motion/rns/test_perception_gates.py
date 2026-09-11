"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_perception_gates.py
Brief: source-level perception consume gates -- class dispatch v1.35 (20 S5.1.1)

Description:
Drives RnsSource.compute with a real PerceptionSnapshot (profile + objects +
status) so the CONSUMED effect of the 20 S5.1.1 v1.35 rules is what is
asserted, not the helper in isolation: a T-class "object" must not stop the
robot (A-CLS-5), an ignore-class bird must not (A-CLS-6), a low-confidence
bird MUST (A-CLS-7, block is the fail-safe direction) and a low-confidence
person MUST (A-CLS-7 reverse). The stop is observable as vx == 0 with the
source in WAIT_DYNAMIC; the drop is observable in the audit ring.

Why source-level: classify.effective_behavior can be right while source.py
still calls the old behavior_class -- the P7 wiring is the thing under test.
Each test names the mutant that reddens it (CLAUDE.md 3.3).
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import pytest
import yaml

from tests.p1_motion.rns.scenes import healthy_status, one_object, snapshot, uniform_free
from xbrain.p1_motion.rns.inputs import PerceptionSnapshot
from xbrain.p1_motion.rns.route import Mission
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind, NavState, Origin

pytestmark = pytest.mark.no_device

_ROOT = Path(__file__).resolve().parents[3]
_REAL_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))
R_EFF = 0.5
NOW = 5000


@dataclass
class Ctx:
    """The tick context the SIL / P7.1 host hands to compute(): pose, heading,
    speed caps, the perception snapshot and the monotonic tick time."""
    pose_xy: Optional[Tuple[float, float]] = (0.0, 0.0)
    yaw_rad: Optional[float] = 0.0
    v_nom_mps: Optional[float] = 1.0
    wz_max_rps: Optional[float] = 1.0
    perception: Optional[PerceptionSnapshot] = None
    now_mono_ms: Optional[int] = NOW
    holonomic: bool = False


def _cfg():
    return copy.deepcopy(_REAL_CFG)


def _mission():
    pts = [(i * 0.5, 0.0) for i in range(41)]   # straight path 0..20 m, +x
    return Mission(MissionKind.PATH, Origin.ROUTE, pts,
                   search_window=8, arrival_radius_m=1.0, max_deviation_m=10.0)


def _snap(objects):
    return snapshot(profile=uniform_free(t_capture_mono_ms=NOW - 20,
                                         t_seg_mono_ms=NOW - 30),
                    objects=objects,
                    status=healthy_status(t_publish_mono_ms=NOW - 100))


def _run(objects):
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    out = s.compute(Ctx(perception=_snap(objects)))
    return s, out


def _moving_toward_robot(class_name, **kw):
    # 1 m ahead, closing at 1 m/s, well inside stop_dist (3.0) and the corridor
    return one_object(class_name=class_name, r_near=1.0, velocity_xy=(-1.0, 0.0),
                      velocity_status="moving", t_capture_mono_ms=NOW - 20, **kw)


def test_confirmed_person_ahead_stops_the_robot():
    # baseline for every negative case below: the stop rule DOES fire for a
    # real dynamic object in the corridor (else the "must not stop" tests
    # would pass vacuously -- CLAUDE.md 3.2 form 1).
    s, out = _run(_moving_toward_robot("person"))
    assert out is not None and out.vx.value == 0.0
    assert s.nav_state() == NavState.WAIT_DYNAMIC


def test_t_class_object_is_dropped_not_consumed_as_block():
    # A-CLS-5: traversable_area is the T channel, never an object. mutant:
    # remove the `beh is None -> continue` branch in source.py -> the object
    # falls through unmapped->block -> dynamic rule -> STOP -> reddens.
    s, out = _run(_moving_toward_robot("traversable_area"))
    assert out is not None and out.vx.value > 0.0
    assert s.nav_state() == NavState.FOLLOW
    kinds = [r.kind for r in s.audit.drain()]
    assert "objects_t_class_dropped" in kinds


def test_ignore_class_bird_does_not_stop_the_robot():
    # A-CLS-6: an airborne target carries no yield rule. mutant: treat ignore
    # as block -> bird 1 m ahead stops the robot -> reddens.
    s, out = _run(_moving_toward_robot("bird"))
    assert out is not None and out.vx.value > 0.0
    assert s.nav_state() == NavState.FOLLOW


def test_low_confidence_bird_is_block_not_ignore():
    # A-CLS-7: below min_confidence the class is UNKNOWN -> block, so the
    # ignore mapping must NOT apply. mutant: skip the confidence gate ->
    # the 0.1-confidence bird is ignored -> robot keeps driving -> reddens.
    s, out = _run(_moving_toward_robot("bird", confidence=0.1))
    assert out is not None and out.vx.value == 0.0
    assert s.nav_state() == NavState.WAIT_DYNAMIC


def test_low_confidence_person_still_stops():
    # A-CLS-7 reverse: person is immune to the confidence gate -- anything
    # that might be a person stops the robot. mutant: gate person too ->
    # a 0.05-confidence person is "block" (still dynamic here) -- so this
    # reverse is pinned on the helper's contract in test_classify_dispatch
    # AND here on the consumed effect.
    s, out = _run(_moving_toward_robot("person", confidence=0.05))
    assert out is not None and out.vx.value == 0.0


def test_proxy_car_never_enters_static_pile():
    # 20 S5.1.1 v1.35: a proxy (maybe-)car that has "stopped" for longer than
    # the dwell must NOT become a static-pile detour candidate; it stays under
    # the dynamic rule. Observable: a static proxy car 1 m ahead still STOPS
    # the robot (dynamic rule), whereas a confirmed static car would be left
    # to geometry. mutant: ignore allow_static -> proxy car goes static ->
    # no WAIT_DYNAMIC -> reddens.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    car = one_object(class_name="car", r_near=1.0, velocity_xy=(0.0, 0.0),
                     velocity_status="static", semantic_status="proxy",
                     t_capture_mono_ms=NOW - 20)
    # dwell: feed the same static car for longer than t_static_dwell_s (2 s)
    out = None
    for k in range(60):
        now = NOW + k * 50
        car_k = one_object(class_name="car", r_near=1.0, velocity_xy=(0.0, 0.0),
                           velocity_status="static", semantic_status="proxy",
                           t_capture_mono_ms=now - 20)
        snap = snapshot(profile=uniform_free(t_capture_mono_ms=now - 20,
                                             t_seg_mono_ms=now - 30),
                        objects=car_k,
                        status=healthy_status(t_publish_mono_ms=now - 100))
        out = s.compute(Ctx(perception=snap, now_mono_ms=now))
    assert car is not None
    assert out is not None and out.vx.value == 0.0
    assert s.nav_state() == NavState.WAIT_DYNAMIC
