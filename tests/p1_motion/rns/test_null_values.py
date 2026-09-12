"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_null_values.py
Brief: source-level consumption of the 11 v2.3 legal nulls (velocity unknown, unlocalized, status unknown)

Description:
Drives RnsSource.compute with DTOs built by the scene helpers so the CONSUMED
effect of the three v2.3 null rules is what is asserted: a car with
velocity_valid=false is never the static pile (STOP in the corridor, where a
valid static car is handed to geometry and driven past); a detected-but-
unlocalizable person caps the tick and is audited on edge; a null
invalid_pixel_ratio caps like an over-limit one. Each test names the mutant
that reddens it (CLAUDE.md 3.3).
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import pytest
import yaml

from tests.p1_motion.rns.scenes import healthy_status, one_object, snapshot, uniform_free
from xbrain.p1_motion.rns.inputs import PerceptionSnapshot, StatusMsg
from xbrain.p1_motion.rns.route import Mission
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind, NavState, Origin

pytestmark = pytest.mark.no_device

_ROOT = Path(__file__).resolve().parents[3]
_REAL_CFG = yaml.safe_load((_ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))
NOW = 5000
V_NOM = 1.0


@dataclass
class Ctx:
    pose_xy: Optional[Tuple[float, float]] = (0.0, 0.0)
    yaw_rad: Optional[float] = 0.0
    v_nom_mps: Optional[float] = V_NOM
    wz_max_rps: Optional[float] = 1.0
    perception: Optional[PerceptionSnapshot] = None
    now_mono_ms: Optional[int] = NOW
    holonomic: bool = False


def _source():
    cfg = copy.deepcopy(_REAL_CFG)
    s = RnsSource(cfg=cfg, r_eff_m=0.5)
    s.load_mission(Mission(MissionKind.PATH, Origin.ROUTE,
                           [(i * 0.5, 0.0) for i in range(41)],
                           search_window=cfg["rns"]["route"]["search_window"],
                           arrival_radius_m=cfg["rns"]["route"]["arrival_radius_m"],
                           max_deviation_m=cfg["rns"]["route"]["max_deviation_m"]))
    return s, cfg


def _drive(s, objects_fn, ticks=8, status=None):
    out = None
    for k in range(ticks):
        now = NOW + 50 * k
        objects = objects_fn(now)
        snap = snapshot(uniform_free(6.0, t_capture_mono_ms=now, t_seg_mono_ms=now - 10),
                        objects,
                        status if status is not None else healthy_status(t_publish_mono_ms=now))
        out = s.compute(Ctx(perception=snap, now_mono_ms=now))
    return out


def test_unknown_velocity_car_stops_where_a_static_car_is_driven_past():
    """11 v2.3 row 2. mutant: read velocity_xy regardless of velocity_valid
    -> the warming-up car reads 0 m/s -> static pile after the dwell -> red."""
    s, cfg = _source()
    dwell_ticks = int(cfg["rns"]["dynamic"]["t_static_dwell_s"] * 20) + 4
    out = _drive(s, lambda now: one_object(track_id=7, class_name="car", r_near=2.3,
                                           velocity_valid=True, velocity_status="static",
                                           t_capture_mono_ms=now), ticks=dwell_ticks)
    assert s.nav_state() != NavState.WAIT_DYNAMIC and out.vx.value > 0.0   # geometry covers it
    s2, _ = _source()
    out2 = _drive(s2, lambda now: one_object(track_id=8, class_name="car", r_near=2.3,
                                             velocity_valid=False, velocity_status="warming_up",
                                             t_capture_mono_ms=now), ticks=dwell_ticks)
    assert s2.nav_state() == NavState.WAIT_DYNAMIC and out2.vx.value == 0.0


def test_unlocalized_person_caps_and_audits():
    """11 v2.3 row 4. mutant: `continue` past unlocalized objects without the
    cap / audit -> full speed, no record -> red."""
    s, cfg = _source()
    out = _drive(s, lambda now: one_object(track_id=9, class_name="person", unlocalized=True,
                                           t_capture_mono_ms=now), ticks=4)
    cap = cfg["rns"]["perception"]["unlocalized_speed_cap_mps"]
    assert 0.0 < out.vx.value <= cap + 1e-9
    assert s.nav_state() == NavState.FOLLOW
    recs = [r for r in s.audit.drain() if r.kind == "objects_unlocalized"]
    assert len(recs) == 1 and recs[0].detail == {"n": 1}        # edge-audited, not per tick


def test_unlocalized_ignore_class_does_not_cap():
    s, cfg = _source()
    out = _drive(s, lambda now: one_object(track_id=10, class_name="bird", unlocalized=True,
                                           t_capture_mono_ms=now), ticks=4)
    cap = cfg["rns"]["perception"]["unlocalized_speed_cap_mps"]
    assert out.vx.value > cap


def test_null_invalid_ratio_caps_like_over_limit():
    """11 v2.3 row 3 / RNS-I-2. mutant: health_speed_capped(None) False -> red."""
    s, cfg = _source()
    st = StatusMsg(t_publish_mono_ms=NOW, fps_depth=29.0, fps_infer=20.0,
                   invalid_pixel_ratio=None, extrinsic_calibrated=True,
                   traversable_seg_available=True)
    out = _drive(s, lambda now: None, ticks=3, status=st)
    cap = cfg["rns"]["perception"]["no_seg_speed_cap_mps"]
    assert 0.0 < out.vx.value <= cap + 1e-9
