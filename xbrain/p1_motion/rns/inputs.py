"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: inputs.py
Brief: RNS perception consume face -- three-key DTOs + replay (P0.2/P2 -- 20 S3.1)

Description:
The RNS module's view over the three perception keys (11 S3.1B): profile (geometry
three-state), objects (semantic), status (health). Fed by the tick snapshot
(RNS-M-4: RNS never reads a Zenoh session itself, 20 S1.1).

P0.2 scope (this slice): the DTOs and the replay injection point. The tick-time
CONSUME logic -- per-message ageing (T-50..T-53), bit0/no-seg degrade, the
RNS-I-1..4 discipline -- lands in P2. The replay source is the intake for every
synthetic golden scene (W-11 / RNS_TODO M-3); getting it right now is what lets
P1..P6 test against deterministic frames.

Why a DTO here and not the raw 11 schema dict: the module must never consume a
None as a number (RNS-I-1: null stays UNKNOWN). Modelling d_free/d_block as
Optional[float] arrays with a null-preserving accessor is how that invariant is
carried in code, not left to each caller to remember.

Trap this shape guards:
  - 12 S4.2c / 20 TIME-2: profile and objects are NOT same-frame. Each DTO
    carries its OWN t_capture_mono_ms; the consumer ages them separately. A
    single shared timestamp field would silently re-introduce the same-frame
    coupling that 11 S3.1B.0 spent a whole ruling removing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple


# ── profile (11 S3.1B.1) ──────────────────────────────────────────────────────
@dataclass(frozen=True)
class ProfileMsg:
    """Geometric three-state profile, one depth frame. Arrays are per-bin, all
    length n_bins. Optional[float] entries are None for "not observed" (RNS-I-1:
    null stays UNKNOWN, never 0 / never range_max)."""
    t_capture_mono_ms: int
    t_publish_mono_ms: int
    extrinsic_calibrated: bool
    angle_min_rad: float
    angle_step_rad: float
    n_bins: int
    range_max_m: float
    blind_near_m: float
    z_pass_m: float
    t_seg_mono_ms: Optional[int]              # None = no T evidence this frame
    d_free: Tuple[Optional[float], ...] = ()  # per-bin; None = unobserved
    d_block: Tuple[Optional[float], ...] = ()  # per-bin; None = no obstacle in range
    h_block: Tuple[Optional[float], ...] = ()
    src: Tuple[int, ...] = ()                  # per-bin bitmask (SrcBit)
    conf: Tuple[int, ...] = ()

    def has_seg(self) -> bool:
        """20 S3.1.11: no T evidence -> geometry-only FREE, RNS limits speed.
        t_seg None is the single source of that truth."""
        return self.t_seg_mono_ms is not None


# ── objects (11 S3.1B.2) ──────────────────────────────────────────────────────
@dataclass(frozen=True)
class TrackedObject:
    track_id: int
    class_name: str
    class_id: int
    confidence: float
    semantic_status: str                       # confirmed | proxy
    footprint_xy: Tuple[Tuple[float, float], ...]  # ground hull, CCW, 3<=N<=12
    z_min: float
    z_max: float
    r_near: float
    velocity_xy: Tuple[float, float]
    velocity_frame: str                        # ego_removed | raw
    velocity_valid: bool
    velocity_status: str
    stable_frames: int


@dataclass(frozen=True)
class ObjectsMsg:
    t_capture_mono_ms: int                     # own timestamp (TIME-2)
    extrinsic_calibrated: bool
    objects: Tuple[TrackedObject, ...] = ()


# ── status (11 S3.1B.3) ───────────────────────────────────────────────────────
@dataclass(frozen=True)
class StatusMsg:
    t_publish_mono_ms: int
    fps_depth: float
    fps_infer: float
    invalid_pixel_ratio: float
    extrinsic_calibrated: bool
    traversable_seg_available: bool
    degraded_reasons: Tuple[str, ...] = ()


@dataclass(frozen=True)
class PerceptionSnapshot:
    """The three keys as one tick's snapshot (RNS-M-4). Any of the three may be
    None (that key not yet received / dropped); the consumer treats a missing
    key via its timeout (T-50..T-53), NOT by defaulting."""
    profile: Optional[ProfileMsg] = None
    objects: Optional[ObjectsMsg] = None
    status: Optional[StatusMsg] = None


class PerceptionInput:
    """Abstract intake. Production wires a Zenoh subscriber (three keys, strong
    refs -- CLAUDE.md 4.3); tests wire ReplayPerceptionInput. The tick loop calls
    latest() once per tick; it never blocks (RNS-M-1/M-5)."""
    def latest(self, now_mono_ms: int) -> PerceptionSnapshot:
        raise NotImplementedError


@dataclass
class ReplayPerceptionInput(PerceptionInput):
    """Test intake: a deterministic snapshot sequence. THE intake for every
    synthetic golden scene (W-11). Latest-wins per tick; exhausted -> repeats the
    last snapshot (a real sensor keeps publishing its last frame until a new one,
    so tests see age grow, not the stream vanish)."""
    snapshots: List[PerceptionSnapshot] = field(default_factory=list)
    _idx: int = 0

    def latest(self, now_mono_ms: int) -> PerceptionSnapshot:
        if not self.snapshots:
            return PerceptionSnapshot()
        if self._idx < len(self.snapshots):
            snap = self.snapshots[self._idx]
            self._idx += 1
            return snap
        return self.snapshots[-1]
