"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: scenes.py
Brief: Synthetic perception scene builders (P0.6 -- RNS_TODO M-3 / 20 S13)

Description:
The intake for every RNS golden scene (RNS_TODO M-3). Builds ProfileMsg /
ObjectsMsg / PerceptionSnapshot from a compact spec so P1..P6 tests read like the
scene they describe (straight corridor, glass hole, thin pole, U-trap, ...) not
like array literals. This is a TEST asset (CLAUDE.md 7.2.1: no real hardware).

Design choices that keep scenes honest:
  - a bin's value is None unless the spec sets it (RNS-I-1: unobserved stays
    UNKNOWN; the builder never fills a None with 0 or range_max);
  - the builder does NOT enforce PROF-1 (d_free<=d_block) -- a scene that
    violates it is a legitimate MUTANT input for A-FUS-4, so validation is the
    consumer's job, not the fixture's.

Not a scene library yet: this file is the BUILDER. Named scenes (U-trap, glass
hole, grazing) are assembled in the test that needs them, each documenting the
assertion id it serves (M-3 requires the scene-file header to name it).
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from xbrain.p1_motion.rns.inputs import (
    ObjectsMsg, PerceptionSnapshot, ProfileMsg, StatusMsg, TrackedObject,
)

# Default sector geometry (11 S3.1B.1 example values: +/-45 deg, 0.5 deg, 181).
DEF_ANGLE_MIN = -math.pi / 4
DEF_ANGLE_STEP = math.radians(0.5)
DEF_N_BINS = 181
DEF_RANGE_MAX = 6.0
DEF_BLIND_NEAR = 0.59
DEF_Z_PASS = 0.75


def uniform_free(
    d_free_m: float = 6.0,
    *,
    n_bins: int = DEF_N_BINS,
    t_capture_mono_ms: int = 1000,
    t_seg_mono_ms: Optional[int] = 990,
    extrinsic_calibrated: bool = True,
    range_max_m: float = DEF_RANGE_MAX,
) -> ProfileMsg:
    """Open ground: every bin FREE to d_free_m, no obstacle. The baseline scene;
    other builders start here and punch obstacles in. src bit0|bit1 set (T and G
    both back it) when t_seg is present, else bit1 only (geometry-only)."""
    seg = t_seg_mono_ms is not None
    src_val = (0b11 if seg else 0b10)
    return ProfileMsg(
        t_capture_mono_ms=t_capture_mono_ms,
        t_publish_mono_ms=t_capture_mono_ms + 10,
        extrinsic_calibrated=extrinsic_calibrated,
        angle_min_rad=DEF_ANGLE_MIN, angle_step_rad=DEF_ANGLE_STEP,
        n_bins=n_bins, range_max_m=range_max_m, blind_near_m=DEF_BLIND_NEAR,
        z_pass_m=DEF_Z_PASS, t_seg_mono_ms=t_seg_mono_ms,
        d_free=tuple([d_free_m] * n_bins),
        d_block=tuple([None] * n_bins),
        h_block=tuple([None] * n_bins),
        src=tuple([src_val] * n_bins),
        conf=tuple([220] * n_bins),
    )


def with_block(
    base: ProfileMsg,
    bin_index: int,
    d_block_m: float,
    *,
    h_block_m: Optional[float] = 0.4,
    d_free_m: Optional[float] = None,
) -> ProfileMsg:
    """Punch a blocked bin. d_free for that bin defaults to just short of the
    block (kept < d_block so PROF-1 holds unless a test wants to break it)."""
    df = list(base.d_free)
    db = list(base.d_block)
    hb = list(base.h_block)
    db[bin_index] = d_block_m
    hb[bin_index] = h_block_m
    df[bin_index] = d_block_m - 0.1 if d_free_m is None else d_free_m
    return _replace(base, d_free=tuple(df), d_block=tuple(db), h_block=tuple(hb))


def with_unknown(base: ProfileMsg, bin_index: int) -> ProfileMsg:
    """Mark a bin unobserved: d_free/d_block both None (RNS-I-1). Used for glass
    holes, blind sectors, occlusion."""
    df = list(base.d_free)
    db = list(base.d_block)
    df[bin_index] = None
    db[bin_index] = None
    return _replace(base, d_free=tuple(df), d_block=tuple(db))


def _replace(p: ProfileMsg, **kw) -> ProfileMsg:
    import dataclasses
    return dataclasses.replace(p, **kw)


def one_object(
    *,
    track_id: int = 1,
    class_name: str = "person",
    r_near: float = 3.0,
    velocity_xy: Tuple[float, float] = (0.0, 0.0),
    velocity_frame: str = "ego_removed",
    velocity_status: str = "static",
    stable_frames: int = 30,
    t_capture_mono_ms: int = 1000,
    footprint: Optional[Sequence[Tuple[float, float]]] = None,
    confidence: float = 0.9,
    semantic_status: str = "confirmed",
) -> ObjectsMsg:
    """One tracked object. Defaults to a static person at 3 m (the stop-distance
    boundary). footprint defaults to a small triangle in front of the robot."""
    fp = tuple(footprint) if footprint is not None else (
        (r_near, -0.3), (r_near + 0.6, 0.0), (r_near, 0.3))
    obj = TrackedObject(
        track_id=track_id, class_name=class_name, class_id=0,
        confidence=confidence, semantic_status=semantic_status, footprint_xy=fp,
        z_min=0.02, z_max=1.75, r_near=r_near, velocity_xy=velocity_xy,
        velocity_frame=velocity_frame, velocity_valid=True,
        velocity_status=velocity_status, stable_frames=stable_frames)
    return ObjectsMsg(t_capture_mono_ms=t_capture_mono_ms,
                      t_publish_mono_ms=t_capture_mono_ms + 10,
                      extrinsic_calibrated=True, objects=(obj,))


def healthy_status(
    *, t_publish_mono_ms: int = 1010, invalid_pixel_ratio: float = 0.05,
    traversable_seg_available: bool = True, extrinsic_calibrated: bool = True,
    fps_infer: float = 20.5,
) -> StatusMsg:
    return StatusMsg(
        t_publish_mono_ms=t_publish_mono_ms, fps_depth=29.0, fps_infer=fps_infer,
        invalid_pixel_ratio=invalid_pixel_ratio,
        extrinsic_calibrated=extrinsic_calibrated,
        traversable_seg_available=traversable_seg_available,
        degraded_reasons=())


def snapshot(
    profile: Optional[ProfileMsg] = None,
    objects: Optional[ObjectsMsg] = None,
    status: Optional[StatusMsg] = None,
) -> PerceptionSnapshot:
    return PerceptionSnapshot(profile=profile, objects=objects, status=status)
