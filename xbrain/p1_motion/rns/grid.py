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

    def ingest_profile(self, profile, pose_xy, yaw, now_ms: int,
                       free_step_m: float = 0.5,
                       foot_r_m: float = 0.5) -> None:
        """S2b assembly: write one tick's profile into the grid (world frame).
        Every blocked bin writes its hit point BLOCKED; the free run before it
        (or the full free ray) writes sparse FREE samples. This is what makes
        wall-follow possible at all on a 90 deg FOV: the wall being hugged sits
        at ~90 deg to the side -- OUT of view -- and lives only here (RNS-I-7).

        The robot FOOTPRINT is stamped FREE too (v2, with the conservative
        side query): the body standing there IS the traversability
        observation -- the sensor cannot see its own feet (0.59 m blind
        zone), so without the stamp the first cells of every side ray stay
        UNKNOWN forever and the conservative d_side would read a phantom
        wall at 0.25 m on open ground.
        """
        import math as _m
        # footprint disc, foot_r_m around the pose
        span = int(foot_r_m / self._cell_m)
        for fi in range(-span, span + 1):
            for fj in range(-span, span + 1):
                fx = pose_xy[0] + fi * self._cell_m
                fy = pose_xy[1] + fj * self._cell_m
                if _m.hypot(fx - pose_xy[0], fy - pose_xy[1]) <= foot_r_m:
                    self.write(fx, fy, Cell.FREE, now_ms)
        for i in range(profile.n_bins):
            ang = yaw + profile.angle_min_rad + i * profile.angle_step_rad
            c, si = _m.cos(ang), _m.sin(ang)
            db = profile.d_block[i]
            df = profile.d_free[i]
            if df is not None:
                r = free_step_m
                while r < df:
                    self.write(pose_xy[0] + r * c, pose_xy[1] + r * si,
                               Cell.FREE, now_ms)
                    r += free_step_m
            if db is not None:
                self.write(pose_xy[0] + db * c, pose_xy[1] + db * si,
                           Cell.BLOCKED, now_ms)

    def nearest_blocked_full(self, pose_xy, now_ms: int,
                             r_max_m: float = 6.0):
        """Omnidirectional nearest remembered BLOCKED: (dist, world bearing)
        or None. The convex-corner re-acquire uses it -- past a wall's end the
        wall leaves the side sector AND the robot's FOV, but not the memory."""
        import math as _m
        best = None
        for k in range(24):
            ang = k * _m.pi / 12.0
            c, si = _m.cos(ang), _m.sin(ang)
            r = self._cell_m
            while r <= r_max_m:
                if self.read(pose_xy[0] + r * c, pose_xy[1] + r * si,
                             now_ms) == Cell.BLOCKED:
                    if best is None or r < best[0]:
                        best = (r, ang)
                    break
                r += self._cell_m
        return best

    def wall_end_dist(self, anchor_xy, walk_bear: float, now_ms: int,
                      r_max_m: float = 8.0, gap_bridge_m: float = 1.25):
        """Distance from anchor (a point ON the wall) to the wall's END along
        walk_bear, or None if no end is CONFIRMED within r_max_m. Feeds the
        side-pick rules 1/2 of 20 S7.2 ("the side that sees the wall's end";
        both see -> nearer end) -- before this the assembly passed sees_end =
        False,False and the goal-side heuristic could send the robot the LONG
        way around (field bug 2026-09-10: 1 m short of the south end, re-entry
        flipped north and re-walked the whole 13 m wall).

        Method -- WALL-SNAKE probe (v2, field bug 2026-09-10 "45 m reverse
        lap"): step along the probe heading; at each step scan the
        PERPENDICULAR band +/- gap_bridge_m for BLOCKED cells (the band
        bridges intra-wall gaps like parked-car spacing ~1 m). While BLOCKED
        is present, RE-CENTER the probe onto the band's blocked centroid and
        BLEND the heading toward the actual drift -- the probe line follows
        the wall's true run even when walk_bear starts diagonal. This is the
        v2 core: v1 probed a straight line, and entering at a wall CORNER the
        nearest-blocked bearing points at the corner cell, so both
        perpendicular tangents cut the wall diagonally -- one step walked off
        the wall onto just-walked FREE ground and a tiny FAKE end (live
        audit: end_r=0.75) flipped nearer-end-wins into a 45 m reverse lap.
        A run of gap_bridge_m with no BLOCKED is the end -- but ONLY when the
        run carries FREE evidence: an all-UNKNOWN band means "never looked",
        and calling that an end would let the un-observed side always win.
        Returns the ARC length walked to the end (real detour cost), else
        None."""
        import math as _m
        hx, hy = _m.cos(walk_bear), _m.sin(walk_bear)
        px, py = anchor_xy
        step = self._cell_m
        n_perp = int(gap_bridge_m / step)
        gap_run = 0.0
        walked = 0.0
        while walked < r_max_m:
            # advance one step along the current heading
            px += hx * step
            py += hy * step
            walked += step
            nc, ns = -hy, hx                 # perpendicular unit vector
            hits = []
            has_free = False
            for j in range(-n_perp, n_perp + 1):
                st = self.read(px + nc * step * j, py + ns * step * j, now_ms)
                if st == Cell.BLOCKED:
                    hits.append(j)
                elif st == Cell.FREE and abs(j) <= 2:
                    # FREE evidence counts only near the probe LINE (+/-0.5 m):
                    # the band is gap_bridge wide to bridge wall gaps, but a
                    # FREE corridor BESIDE the wall must not vouch for an
                    # un-written wall segment (field collision 2026-09-10:
                    # the blind-zone wall face stayed UNKNOWN while the
                    # walked corridor 1 m west was all FREE -- the wide-band
                    # evidence called a fake end 0.8 m early and the corner
                    # cut went through the still-standing wall).
                    has_free = True
            if hits:
                gap_run = 0.0                # wall continues; keep following
                # re-center onto the blocked centroid (capped to half the
                # band) and blend the heading toward the observed drift --
                # the snake part. Cap + blend keep one noisy cell from
                # yanking the probe off line.
                off = sum(hits) / len(hits) * step
                off = max(-gap_bridge_m / 2, min(gap_bridge_m / 2, off))
                px += nc * off * 0.5
                py += ns * off * 0.5
                if abs(off) > 1e-9:
                    dhx = hx + nc * (off / step) * 0.3
                    dhy = hy + ns * (off / step) * 0.3
                    n = _m.hypot(dhx, dhy)
                    hx, hy = dhx / n, dhy / n
                continue
            if not has_free:
                # un-observed inside the gap. If the confirmed-FREE run is
                # already substantial (>= 0.6 * gap_bridge), the end IS
                # confirmed -- the far side merely fades into unexplored
                # ground. Field data 2026-09-10 (corr->east_mid): the probe
                # rode the wall 4.5 m to its true south end, banked a 1.0 m
                # FREE run past it, then hit one un-observed band 0.25 m
                # short of gap_bridge and the whole probe was voided -- the
                # side pick fell to the goal heuristic and walked a 19.4 m
                # lap the wrong way. A thin run (< 0.6 * bridge) still
                # returns None: never call an end on wishful thinking.
                if gap_run >= 0.6 * gap_bridge_m:
                    return walked - gap_run
                return None
            gap_run += step
            if gap_run >= gap_bridge_m:
                # end sits where the confirmed-free run began
                return walked - gap_run + step
        return None

    def nearest_blocked_in_sector(self, pose_xy, yaw, ang_lo: float,
                                  ang_hi: float, now_ms: int,
                                  r_max_m: float = 4.0,
                                  n_rays: int = 7) -> Optional[float]:
        """Min distance to a remembered BLOCKED cell inside a body-frame angular
        sector (the wall-follow d_side query, 20 S7.7: perception U memory --
        here memory IS the union, since ingest_profile wrote perception in).
        Returns None when the sector holds no remembered wall.

        UNKNOWN is a WALL CANDIDATE, not vacuum (RNS-I-1; field collision
        2026-09-10, corr->east_mid -0.152 m): hugging a wall, the wall face
        sits at ~90 deg -- outside the FOV -- and once the robot drifts close
        it falls inside the 0.59 m sensor blind zone, so the face is NEVER
        written and stays UNKNOWN forever. The old query skipped through that
        UNKNOWN and returned a STALE far hit (d_side 2.0 while the true face
        was 0.4 m away); the PD then steered TOWARD the phantom and dragged
        the hull through the wall's south corner. Rule: a ray that meets a
        run of un-observed cells (>= unk_wall_m with no FREE in between)
        stops there and reports the run's START as a conservative distance --
        keep-distance then holds d_wall off the un-observed region, which
        also brings the real face back out of the blind zone so the memory
        heals itself. Sparse FREE paint (0.5 m spacing) never trips the
        0.5 m run: every other cell reads FREE and resets it."""
        import math as _m
        unk_wall_m = 0.5
        best: Optional[float] = None
        for k in range(n_rays):
            ang = yaw + ang_lo + (ang_hi - ang_lo) * k / max(1, n_rays - 1)
            c, si = _m.cos(ang), _m.sin(ang)
            r = self._cell_m
            unk_run = 0.0
            unk_start = None
            while r <= r_max_m:
                st = self.read(pose_xy[0] + r * c, pose_xy[1] + r * si, now_ms)
                if st == Cell.BLOCKED:
                    if best is None or r < best:
                        best = r
                    break
                if st == Cell.FREE:
                    unk_run = 0.0
                    unk_start = None
                else:
                    if unk_start is None:
                        unk_start = r
                    unk_run += self._cell_m
                    if unk_run >= unk_wall_m:
                        # un-observed run: conservative wall candidate at its
                        # start. Do NOT read through it to a farther hit.
                        if best is None or unk_start < best:
                            best = unk_start
                        break
                r += self._cell_m
        return best


# (the v1.15-era danger-rank merge -- _DANGER_RANK / _more_dangerous -- was
# retired by the 2026-09-09 ruling: the latest classed observation wins, see
# write(). Kept as a note so the next reader knows the merge was DELIBERATELY
# removed, not forgotten.)


def _is_dynamic_class(cls: Optional[str]) -> bool:
    # dynamic classes get the short TTL (phantom-obstacle guard, S4.2.1). person
    # and vehicles are dynamic; static structure/hazard get the long TTL.
    return cls in ("person", "car", "truck", "bus", "motorcycle")
