"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_inputs.py
Brief: RNS perception DTO + replay tests (P0.2 -- 20 S3.1 / 11 S3.1B)

Description:
Guards the three-key DTOs and the replay intake. Focus: null-preserving fields
(RNS-I-1), profile/objects separate timestamps (TIME-2), has_seg from t_seg
(20 S3.1.11), and replay latest-wins that grows age rather than vanishing.
Each test names its mutant (CLAUDE.md 3.3).
"""

from __future__ import annotations

from xbrain.p1_motion.rns.inputs import (
    ObjectsMsg, PerceptionSnapshot, ProfileMsg, ReplayPerceptionInput, StatusMsg,
    TrackedObject,
)


def _profile(t_cap=1000, t_seg=None):
    return ProfileMsg(
        t_capture_mono_ms=t_cap, t_publish_mono_ms=t_cap + 10,
        extrinsic_calibrated=False, angle_min_rad=-0.785, angle_step_rad=0.0087,
        n_bins=181, range_max_m=6.0, blind_near_m=0.59, z_pass_m=0.75,
        t_seg_mono_ms=t_seg,
        d_free=(0.6, None, 3.0), d_block=(None, 0.8, None),
        h_block=(None, 0.4, None), src=(2, 3, 1), conf=(200, 180, 90))


def test_profile_preserves_null_entries():
    # RNS-I-1: None stays None, never coerced to 0 or range_max. mutant: a DTO
    # that fills None -> some number reddens this.
    p = _profile()
    assert p.d_free[1] is None
    assert p.d_block[0] is None


def test_has_seg_false_when_t_seg_none():
    # 20 S3.1.11: t_seg None IS the no-segmentation truth.
    assert _profile(t_seg=None).has_seg() is False
    assert _profile(t_seg=990).has_seg() is True


def test_profile_and_objects_carry_own_timestamps():
    # TIME-2: not same-frame. Distinct t_capture fields prove they age apart.
    prof = _profile(t_cap=1000)
    objs = ObjectsMsg(t_capture_mono_ms=850, extrinsic_calibrated=False)
    assert prof.t_capture_mono_ms != objs.t_capture_mono_ms


def test_snapshot_allows_missing_keys():
    # any of the three may be None (that key dropped); consumer handles via
    # timeout, not default.
    snap = PerceptionSnapshot(profile=_profile())
    assert snap.objects is None
    assert snap.status is None


def test_replay_yields_then_repeats_last():
    # latest-wins: exhausted -> last snapshot repeats (a real sensor keeps
    # publishing its last frame; tests see AGE grow, not the stream vanish).
    # mutant: replay returning empty after exhaustion reddens this.
    s1 = PerceptionSnapshot(profile=_profile(t_cap=1000))
    s2 = PerceptionSnapshot(profile=_profile(t_cap=1050))
    r = ReplayPerceptionInput(snapshots=[s1, s2])
    assert r.latest(1000).profile.t_capture_mono_ms == 1000
    assert r.latest(1050).profile.t_capture_mono_ms == 1050
    assert r.latest(1100).profile.t_capture_mono_ms == 1050  # repeats last


def test_replay_empty_gives_empty_snapshot():
    r = ReplayPerceptionInput(snapshots=[])
    snap = r.latest(1000)
    assert snap.profile is None and snap.objects is None and snap.status is None


def test_tracked_object_holds_velocity_frame():
    # velocity_frame is the field 20 S3.1.5 keys the raw-refusal on.
    o = TrackedObject(
        track_id=41, class_name="person", class_id=0, confidence=0.9,
        semantic_status="confirmed", footprint_xy=((0.0, 0.0), (1.0, 0.0), (0.5, 1.0)),
        z_min=0.02, z_max=1.78, r_near=2.3, velocity_xy=(0.8, -0.1),
        velocity_frame="raw", velocity_valid=True, velocity_status="moving",
        stable_frames=37)
    assert o.velocity_frame == "raw"
