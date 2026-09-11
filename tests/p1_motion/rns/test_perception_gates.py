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

from tests.p1_motion.rns.scenes import (healthy_status, one_object, snapshot,
                                        uniform_free, with_block)
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


# ── 11 S3.1B.5 v2.1 acceptance + 20 S3.1.8 age tiers + S3.1B.4 gate (B2) ─────
from xbrain.p1_motion.rns.types import NavFailReason  # noqa: E402


def _tick(s, now, profile=None, objects=None, status=None, yaw=0.0):
    snap = snapshot(profile=profile, objects=objects, status=status)
    return s.compute(Ctx(perception=snap, now_mono_ms=now, yaw_rad=yaw))


def test_duplicate_profile_never_refreshes_age():
    # A-ACC-1: the SAME profile (one t_capture) re-carried every tick must age
    # out to T-51 zero speed. mutant: accept the dup as new (refresh age) ->
    # the robot keeps driving after 1 s -> reddens.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    prof = uniform_free(t_capture_mono_ms=NOW - 20, t_seg_mono_ms=NOW - 30)
    out = None
    for k in range(26):                      # 0 .. 1250 ms of the same frame
        out = _tick(s, NOW + k * 50, profile=prof,
                    status=healthy_status(t_publish_mono_ms=NOW + k * 50))
    assert out is not None and out.vx.value == 0.0     # T-51 hit on a dup stream
    kinds = [r.kind for r in s.audit.drain()]
    assert "perception_dup" in kinds


def test_epoch_reset_clears_memory_and_accepts_frame():
    # A-ACC-2: a t_capture more than 1 s BEHIND the last accepted one is an
    # epoch reset (machine restart): memory cleared, frame accepted, audited.
    # mutant: classify it as out_of_order (drop) -> no audit, grid keeps its
    # cells -> reddens.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    blocked = with_block(uniform_free(t_capture_mono_ms=NOW - 20,
                                      t_seg_mono_ms=NOW - 30), 90, 3.0)
    _tick(s, NOW, profile=blocked, status=healthy_status(t_publish_mono_ms=NOW))
    import math
    from xbrain.p1_motion.rns.types import Cell
    assert s._grid.read(3.0, 0.0, NOW) == Cell.BLOCKED   # the wall is remembered
    new_epoch_now = 500
    fresh = uniform_free(t_capture_mono_ms=480, t_seg_mono_ms=470)
    # the new-epoch frame looks the OTHER way (yaw pi): the old wall's cell is
    # outside its FOV, so only clear_all() can remove it -- an uncleared grid
    # keeps a BLOCKED whose t_seen (5000) is "in the future" on the new epoch
    # and never expires: the false wall the rule exists to prevent.
    out = _tick(s, new_epoch_now, profile=fresh,
                status=healthy_status(t_publish_mono_ms=490), yaw=math.pi)
    kinds = [r.kind for r in s.audit.drain()]
    assert "perception_epoch_reset" in kinds
    assert s._last_t["profile"] == 480                # accepted as the new baseline
    assert s._grid.read(3.0, 0.0, new_epoch_now) != Cell.BLOCKED
    assert out is not None and out.vx.value > 0.0


def test_out_of_order_frame_is_dropped_and_audited():
    # 11 S3.1B.5 v2.1: an older-but-within-1s frame is late; dropped, audited,
    # and the age keeps counting from the last ACCEPTED frame.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    _tick(s, NOW, profile=uniform_free(t_capture_mono_ms=NOW - 20, t_seg_mono_ms=NOW - 30),
          status=healthy_status(t_publish_mono_ms=NOW))
    _tick(s, NOW + 50, profile=uniform_free(t_capture_mono_ms=NOW - 400, t_seg_mono_ms=NOW - 410),
          status=healthy_status(t_publish_mono_ms=NOW + 50))
    assert s._last_t["profile"] == NOW - 20
    assert "perception_out_of_order" in [r.kind for r in s.audit.drain()]


def _static_car_stream(s, age_ms, ticks=60):
    """Feed a confirmed STATIC car 1 m ahead for `ticks` ticks; each objects
    message is `age_ms` old at consumption (fresh profile every tick)."""
    out = None
    for k in range(ticks):
        now = NOW + k * 50
        car = one_object(class_name="car", r_near=1.0, velocity_xy=(0.0, 0.0),
                         velocity_status="static", t_capture_mono_ms=now - age_ms)
        out = _tick(s, now,
                    profile=uniform_free(t_capture_mono_ms=now - 20, t_seg_mono_ms=now - 30),
                    objects=car, status=healthy_status(t_publish_mono_ms=now - 100))
    return out


def test_fresh_static_car_goes_to_static_pile_baseline():
    # baseline for A-AGE-1: with FRESH objects (age 20 ms) the dwelled static
    # car leaves the dynamic pile -> geometry only (open profile) -> drives.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    out = _static_car_stream(s, age_ms=20)
    assert out is not None and out.vx.value > 0.0
    assert s.nav_state() == NavState.FOLLOW


def test_tier2_age_refuses_velocity_judgement():
    # A-AGE-1: objects 300 ms old (ok=200 < age <= T-52=500): the velocity
    # judgement is refused -> the "static" car stays dynamic -> WAIT. mutant:
    # drop tier 2 -> car enters static pile -> drives -> reddens.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    out = _static_car_stream(s, age_ms=300)
    assert out is not None and out.vx.value == 0.0
    assert s.nav_state() == NavState.WAIT_DYNAMIC


def test_tier3_objects_lost_caps_speed_and_ignores_stale_yield():
    # A-AGE-2: objects 600 ms old (> T-52): channel lost -> speed capped to the
    # no-seg cap (0.5) and the stale person does NOT drive a WAIT. mutant:
    # drop the cap -> vx == v_nom -> reddens.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    person = one_object(class_name="person", r_near=1.0, velocity_xy=(-1.0, 0.0),
                        velocity_status="moving", t_capture_mono_ms=NOW - 600)
    out = _tick(s, NOW, profile=uniform_free(t_capture_mono_ms=NOW - 20, t_seg_mono_ms=NOW - 30),
                objects=person, status=healthy_status(t_publish_mono_ms=NOW - 100))
    assert out is not None
    assert 0.0 < out.vx.value <= 0.5 + 1e-9
    assert s.nav_state() == NavState.FOLLOW
    assert "objects_lost" in [r.kind for r in s.audit.drain()]


def test_status_older_than_t53_caps_speed():
    # A-AGE-3: a status 4 s old is a lost health channel -> worst-case cap.
    # mutant: drop the T-53 term -> full speed -> reddens.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    out = _tick(s, NOW, profile=uniform_free(t_capture_mono_ms=NOW - 20, t_seg_mono_ms=NOW - 30),
                status=healthy_status(t_publish_mono_ms=NOW - 4000))
    assert out is not None and 0.0 < out.vx.value <= 0.5 + 1e-9


def test_uncalibrated_extrinsics_refuse_mission_at_load():
    # A-CAL-1 (11 S3.1B.4): once perception declared the extrinsics uncalibrated,
    # a new mission is refused up front with extrinsic_uncalibrated. mutant:
    # ignore the flag -> mission runs -> reddens.
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    _tick(s, NOW, profile=uniform_free(t_capture_mono_ms=NOW - 20, t_seg_mono_ms=NOW - 30,
                                       extrinsic_calibrated=False),
          status=healthy_status(t_publish_mono_ms=NOW, extrinsic_calibrated=False))
    f = s.take_failure()
    assert f is not None and f.reason is NavFailReason.EXTRINSIC_UNCALIBRATED
    assert s.nav_state() == NavState.IDLE
    s.load_mission(_mission())                      # refused at load, too
    f2 = s.take_failure()
    assert f2 is not None and f2.reason is NavFailReason.EXTRINSIC_UNCALIBRATED
    assert not s.is_active(Ctx())


def test_calibrated_extrinsics_do_not_refuse():
    # reverse of A-CAL-1: the calibrated case runs (guards a "refuse always").
    s = RnsSource(cfg=_cfg(), r_eff_m=R_EFF)
    s.load_mission(_mission())
    out = _tick(s, NOW, profile=uniform_free(t_capture_mono_ms=NOW - 20, t_seg_mono_ms=NOW - 30),
                status=healthy_status(t_publish_mono_ms=NOW))
    assert out is not None and out.vx.value > 0.0
    assert s.take_failure() is None
