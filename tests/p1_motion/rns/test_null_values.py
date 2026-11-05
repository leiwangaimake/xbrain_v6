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
unlocalizable person caps the tick and is audited on edge. The third v2.3
rule (a null invalid_pixel_ratio caps like an over-limit one) went with the
whole RNS-I-2 cap in 99 U116: neither a null nor a high ratio caps any more.
Each test names the mutant that reddens it (CLAUDE.md 3.3).
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import pytest
import yaml

from tests.p1_motion.rns.scenes import (
    healthy_status,
    one_object,
    snapshot,
    uniform_free,
)
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


def _source(world_v_max=None, unloc_cap=None):
    # 99 U114: the shipped world ceiling (1.0) equals the shipped unlocalized
    # cap (1.0) and v_nom here; the unlocalized tests pin the MECHANISM, so
    # they lower the cap / lift the ceiling to keep it observable.
    cfg = copy.deepcopy(_REAL_CFG)
    if world_v_max is not None:
        cfg["rns"]["world"]["v_max_mps"] = world_v_max
    if unloc_cap is not None:
        cfg["rns"]["perception"]["unlocalized_speed_cap_mps"] = unloc_cap
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


def test_unknown_velocity_car_is_left_to_geometry_person_still_waits():
    """11 v2.3 row 2 under 99 U112: velocity unknown no longer decides
    anything for a car -- a warming-up car 2.3 m ahead is the static pile
    like a measured-static one (geometry, open profile -> drives). A person
    with the same unknown velocity still waits. mutant: keep the old
    'unknown speed -> dynamic' rule -> the car WAITs -> red; let the person
    through the same door -> red."""
    s, _ = _source()
    out = _drive(s, lambda now: one_object(track_id=8, class_name="car", r_near=2.3,
                                           velocity_valid=False, velocity_status="warming_up",
                                           t_capture_mono_ms=now), ticks=60)
    assert s.nav_state() != NavState.WAIT_DYNAMIC and out.vx.value > 0.0   # geometry covers it
    s2, _ = _source()
    out2 = _drive(s2, lambda now: one_object(track_id=9, class_name="person", r_near=2.3,
                                             velocity_valid=False, velocity_status="warming_up",
                                             t_capture_mono_ms=now), ticks=60)
    assert s2.nav_state() == NavState.WAIT_DYNAMIC and out2.vx.value == 0.0


def test_unlocalized_person_caps_and_audits():
    """11 v2.3 row 4. mutant: `continue` past unlocalized objects without the
    cap / audit -> full speed, no record -> red."""
    s, cfg = _source(world_v_max=2.0, unloc_cap=0.5)
    out = _drive(s, lambda now: one_object(track_id=9, class_name="person", unlocalized=True,
                                           t_capture_mono_ms=now), ticks=4)
    cap = cfg["rns"]["perception"]["unlocalized_speed_cap_mps"]
    assert 0.0 < out.vx.value <= cap + 1e-9
    assert s.nav_state() == NavState.FOLLOW
    recs = [r for r in s.audit.drain() if r.kind == "objects_unlocalized"]
    assert len(recs) == 1 and recs[0].detail == {"n": 1}        # edge-audited, not per tick


def test_unlocalized_ignore_class_does_not_cap():
    s, cfg = _source(world_v_max=2.0, unloc_cap=0.5)
    out = _drive(s, lambda now: one_object(track_id=10, class_name="bird", unlocalized=True,
                                           t_capture_mono_ms=now), ticks=4)
    cap = cfg["rns"]["perception"]["unlocalized_speed_cap_mps"]
    assert out.vx.value > cap


@pytest.mark.parametrize("ratio", [None, 0.55, 0.95])
def test_invalid_ratio_no_longer_caps(ratio):
    """99 U116 (20 S3.0A NM-7, A-WLD-10): RNS-I-2 is retired -- a fresh status
    whose ground-mask invalid ratio is null, the 2026-11-04 field value 0.55,
    or 0.95 no longer pins the speed at no_seg_speed_cap_mps. (A status that
    has gone SILENT past T-53 still caps: A-AGE-3 in test_perception_gates.)
    mutant: put the health_speed_capped arm back in source.py -> vx == cap
    -> red."""
    s, cfg = _source()
    st = StatusMsg(t_publish_mono_ms=NOW, fps_depth=29.0, fps_infer=20.0,
                   invalid_pixel_ratio=ratio, extrinsic_calibrated=True,
                   traversable_seg_available=True)
    out = _drive(s, lambda now: None, ticks=3, status=st)
    cap = cfg["rns"]["perception"]["no_seg_speed_cap_mps"]
    assert out.vx.value > cap + 1e-6
