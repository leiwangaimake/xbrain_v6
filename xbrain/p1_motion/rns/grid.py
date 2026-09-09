"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: grid.py
Brief: Three-state fusion + ageing + memory grid (P2 -- 20 S3.1.2/S3.1.11/S4)

Description:
Turns the perception snapshot (inputs.py DTOs) into the module's world view: the
asymmetric three-state fusion (RNS-N-5), the per-message ageing (T-50..T-53), the
T-channel degrade (S3.1.11), and the RTK-anchored memory grid (S4.2.1).

The fusion is ASYMMETRIC on purpose (RNS-N-5, S3.1.2), and its two halves are a
kill-pair -- writing only one lets a one-sided shell pass:
  BLOCKED  <=  ANY channel says obstacle        (union;  A-FUS-1)
  FREE     <=  T says traversable AND G is flat  (intersection; A-FUS-2)
  UNKNOWN  <=  everything else
The union catches an unlabelled object only geometry sees; the intersection
refuses a "grass is free" that geometry knows hides a rock.

This slice (P2.1/P2.2/P2.3/P2.4): ageing, per-bin fusion, T-degrade, invalid->
UNKNOWN. The memory grid write rules (S4.2.1) are P2.5; velocity_frame==raw
refusal (A-FUS-7) is classify.py (P3), not here.

Trap this file's shape guards: "FREE = T or G" (union instead of intersection)
is the fail-open one-sided shell A-FUS-2 exists to kill. And a d_free bin whose
src bit0 is unset is geometry-only FREE (no T backing) -- it must NOT be treated
as full FREE (S3.1.11, A-FUS-6); it caps speed.
"""

from __future__ import annotations

from typing import Optional

from .inputs import ProfileMsg
from .types import Cell, SrcBit


# ── ageing (T-50..T-53) ───────────────────────────────────────────────────────
def profile_age_ms(profile: ProfileMsg, now_mono_ms: int) -> int:
    """Age of the profile geometry (11 S1.6 T-50/T-51): now - t_capture. RNS ages
    profile and objects SEPARATELY (RNS-I-4 / TIME-2); this never uses one key's
    timestamp for the other."""
    return now_mono_ms - profile.t_capture_mono_ms


def profile_zero_speed(age_ms: int, t51_ms: int) -> bool:
    """T-51 (1 s default): profile too old -> zero speed (not locked; recovers
    when a fresh frame arrives)."""
    return age_ms > t51_ms


def profile_speed_limited(age_ms: int, t50_ms: int) -> bool:
    """T-50 (300 ms default): profile aged -> limit speed to obstacle_avoid."""
    return age_ms > t50_ms


# ── T-channel staleness (S3.1.11) ─────────────────────────────────────────────
def seg_stale(profile: ProfileMsg, seg_stale_ms: Optional[int]) -> bool:
    """T evidence stale/absent (S3.1.11 triggers): no t_seg, or the seg frame is
    older than seg_stale_ms behind the depth frame. seg_stale_ms null -> the
    degrade branch is unusable, so any staleness question is answered
    conservatively True (treat as no T, CLAUDE.md 3.1)."""
    if not profile.has_seg():
        return True
    if seg_stale_ms is None:
        return True
    assert profile.t_seg_mono_ms is not None  # has_seg() guaranteed it
    return (profile.t_capture_mono_ms - profile.t_seg_mono_ms) > seg_stale_ms


# ── per-bin three-state fusion (RNS-N-5, S3.1.2) ──────────────────────────────
def fuse_bin(
    d_free: Optional[float],
    d_block: Optional[float],
    src: int,
    r_query_m: float,
) -> Cell:
    """The state of one bin at query range r_query_m (RNS-N-5 asymmetric).

    - BLOCKED: an obstacle is at/inside r_query (d_block is not None and
      d_block <= r_query). ANY channel that produced d_block counts (union).
    - FREE: r_query is within confirmed-free distance AND that free has BOTH T
      and G backing (src has SEG and GEOM). Intersection: geometry-only free
      (bit0 unset) is NOT full FREE (S3.1.11) -- it returns UNKNOWN here so the
      caller applies the no-seg speed cap rather than treating it as clear.
    - UNKNOWN: d_free is None (invalid/unobserved, RNS-I-1), or r_query is beyond
      d_free, or free lacks T backing.

    invalid stays UNKNOWN (A-FUS-3): d_free None is never coerced to a distance."""
    # BLOCKED wins (union): an obstacle at or before the query range.
    if d_block is not None and d_block <= r_query_m:
        return Cell.BLOCKED
    # d_free None = unobserved -> UNKNOWN, never coerced (RNS-I-1 / A-FUS-3).
    if d_free is None:
        return Cell.UNKNOWN
    if r_query_m > d_free:
        return Cell.UNKNOWN
    # within free distance: FREE requires BOTH T and G (intersection, A-FUS-2).
    has_t = bool(src & SrcBit.SEG)
    has_g = bool(src & SrcBit.GEOM)
    if has_t and has_g:
        return Cell.FREE
    # geometry-only free (no T backing): not full FREE (S3.1.11 / A-FUS-6).
    return Cell.UNKNOWN


def bin_needs_seg_speed_cap(d_free: Optional[float], src: int,
                            r_query_m: float) -> bool:
    """A-FUS-6 / S3.1.11: a bin that is geometry-flat-and-within-d_free but has
    NO T backing (src bit0 unset) is passable only under the no-seg speed cap.
    This is the fail-silent shortcut the assertion targets -- returning full FREE
    here (never capping) is the exact defect."""
    if d_free is None or r_query_m > d_free:
        return False
    return not bool(src & SrcBit.SEG) and bool(src & SrcBit.GEOM)


def forward_min_d_free(profile: ProfileMsg) -> Optional[float]:
    """The speed gate's input (20 S8.1 / 12 S6.2): the minimum d_free over the
    forward sector. None entries (unobserved) are the binding constraint -- a
    None means UNKNOWN ahead, which the gate treats as an obstacle boundary. So
    the min is taken over observed bins, and if any forward bin is None the
    caller must treat forward as UNKNOWN-limited (returned as None here)."""
    if not profile.d_free:
        return None
    # if any forward bin is unobserved, forward is not confirmed clear.
    if any(v is None for v in profile.d_free):
        return None
    return min(v for v in profile.d_free if v is not None)


# ── memory grid (P2.5 -- 20 S4.2 / S4.2.1) ────────────────────────────────────
class MemoryGrid:
    """RTK-anchored local memory grid (20 S4.2). Cells hold a three-state value,
    an optional class, and t_seen (monotonic). Anchored in the LOCALIZATION frame
    (RTK/ENU), not the body frame -- with cm-level absolute pose, memory duration
    is bounded by how fast the world changes, not by odometry drift, and turning
    in place does not corrupt the map (S4.2).

    Write rules (S4.2.1) this class enforces, each a real failure mode:
      - only observed cells are written (out-of-view cells keep their memory --
        that IS the memory);
      - BLOCKED/FREE overwrite any older value and refresh t_seen; an OBSERVED
        UNKNOWN does NOT overwrite a non-UNKNOWN older value (else occlusion
        wipes the just-seen memory each tick and wall-follow, S7.4, dies);
      - class conflict takes the MORE dangerous (hazard>block>traverse);
      - a cell older than ttl_{class} reverts to UNKNOWN;
      - a single-tick pose jump > reset_jump_m (RTK repair/reconverge) clears the
        WHOLE grid to UNKNOWN + warn -- a jumped grid's BLOCKED cells are phantom
        walls and its FREE cells phantom paths; clearing is the only safe answer
        (A-MEM-3).
    """

    def __init__(self, cell_m: float, ttl_static_s: float, ttl_dynamic_s: float,
                 reset_jump_m: float) -> None:
        if cell_m <= 0.0:
            raise ValueError("cell_m must be > 0 (20 S4.2.1)")
        self._cell_m = cell_m
        self._ttl_static_ms = int(ttl_static_s * 1000)
        self._ttl_dynamic_ms = int(ttl_dynamic_s * 1000)
        self._reset_jump_m = reset_jump_m
        # cell key (ix, iy) in the localization frame -> (state, cls, t_seen_ms)
        self._cells: dict = {}
        self._last_pose: Optional[tuple] = None
        self.cleared_by_jump = False  # observable for the warn event / A-MEM-3

    def _key(self, x: float, y: float) -> tuple:
        return (int(x // self._cell_m), int(y // self._cell_m))

    def on_pose(self, pose_xy: tuple) -> bool:
        """Call once per tick with the current localization pose. Returns True and
        clears the grid if the single-tick jump exceeds reset_jump_m (A-MEM-3)."""
        self.cleared_by_jump = False
        if self._last_pose is not None:
            dx = pose_xy[0] - self._last_pose[0]
            dy = pose_xy[1] - self._last_pose[1]
            if (dx * dx + dy * dy) ** 0.5 > self._reset_jump_m:
                self._cells.clear()
                self.cleared_by_jump = True
        self._last_pose = pose_xy
        return self.cleared_by_jump

    def write(self, x: float, y: float, state: Cell, now_ms: int,
              cls: Optional[str] = None) -> None:
        """Write one observed cell (S4.2.1 covering rules). An observed UNKNOWN
        does not overwrite a non-UNKNOWN older value."""
        key = self._key(x, y)
        old = self._cells.get(key)
        # REVIEW-FIX B6 (2026-09-09): an entry past its TTL is already UNKNOWN
        # on the read side; the write side must agree, or the expired entry (a)
        # blocks a legitimate observed-UNKNOWN write and (b) pollutes the class
        # merge with a class that has officially reverted. Treat expired as
        # absent and drop the stale entry.
        if old is not None:
            _ostate, ocls, ot_seen = old
            ottl = (self._ttl_dynamic_ms if _is_dynamic_class(ocls)
                    else self._ttl_static_ms)
            if now_ms - ot_seen > ottl:
                del self._cells[key]
                old = None
        if state == Cell.UNKNOWN and old is not None and old[0] != Cell.UNKNOWN:
            # observed-UNKNOWN must not erase a just-seen BLOCKED/FREE (S4.2.1).
            return
        # class conflict: the LATEST observation with a class wins (user ruling
        # 2026-09-09, 20 S4.2.1 v1.16). The old danger-rank merge let a fresh
        # person inherit an unexpired hazard's STATIC ttl -> phantom wall after
        # the person left. "What occupies this cell NOW" is the latest classed
        # observation; a class-less write keeps the known class (this tick just
        # had no semantic info, which is not evidence the class changed).
        new_cls = cls if cls is not None else (old[1] if old is not None else None)
        self._cells[key] = (state, new_cls, now_ms)

    def read(self, x: float, y: float, now_ms: int) -> Cell:
        """Read a cell, applying TTL expiry (S4.2.1): a cell older than its
        class ttl reverts to UNKNOWN."""
        key = self._key(x, y)
        entry = self._cells.get(key)
        if entry is None:
            return Cell.UNKNOWN
        state, cls, t_seen = entry
        ttl = self._ttl_dynamic_ms if _is_dynamic_class(cls) else self._ttl_static_ms
        if now_ms - t_seen > ttl:
            return Cell.UNKNOWN
        return state

    def cell_count(self) -> int:
        return len(self._cells)


# (the v1.15-era danger-rank merge -- _DANGER_RANK / _more_dangerous -- was
# retired by the 2026-09-09 ruling: the latest classed observation wins, see
# write(). Kept as a note so the next reader knows the merge was DELIBERATELY
# removed, not forgotten.)


def _is_dynamic_class(cls: Optional[str]) -> bool:
    # dynamic classes get the short TTL (phantom-obstacle guard, S4.2.1). person
    # and vehicles are dynamic; static structure/hazard get the long TTL.
    return cls in ("person", "car", "truck", "bus", "motorcycle")
