"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: stall_diag.py
Brief: Log what the robot saw when a route run stands still (99 U195 diagnostics)

Description:
User report 2026-11-13: a goto to 4号线终点 stood still for about 4.5 min,
and several later runs stalled too. The P1 log could not say why: RNS kept
nav_state "follow", the gate limiter read "none", and RNS's own audit ring
(rns/audit.RingAudit) is drained by nobody in production, so its decisions
never reached a file.

This module decides WHEN to write one diagnostic line, and WHAT goes in it:
  * StallDiag: a route run (route running, rns_avoid holding the output)
    whose published vx, vy and wz are all zero for START_AFTER_S logs
    "start"; while it lasts it logs "repeat" every REPEAT_S; when it moves
    again after a "start" it logs "end" with the duration. A stop shorter
    than START_AFTER_S (normal braking, a 1 s wait) writes nothing.
  * describe_stall: one line -- the RNS hold reason and limiter, the raw
    candidate, pose / fix / heading source, the nearest blocked distance
    and the unknown share in the front / left / right sectors of the
    profile, the nearest perception objects, and the newest RNS audit
    records (drained from the ring at that moment).

What it does NOT do: it never changes the output, and it writes at most one
line per REPEAT_S, so the 20 Hz loop pays nothing outside a stall. The two
timings are log cadence, not safety parameters, so they are module constants
(CLAUDE.md 3.1 covers common.spec / common.safety only).
"""
from __future__ import annotations  # postponed annotations

import json  # compact audit details
import math  # bearings and distances
from typing import Any, Iterable, List, Optional, Tuple  # type hints

#: A route run standing still this long is a stall worth one log line. Normal
#: stops (braking, a short wait for a person) stay below it.
START_AFTER_S = 3.0  # about one braking stop plus margin
#: While the stall lasts, one line every this many seconds.
REPEAT_S = 10.0  # a 4.5 min stall = ~27 lines, not 5400
#: Output magnitude below which an axis counts as zero.
STILL_EPS = 1e-3  # float noise, not motion
#: Body-frame sectors (deg) of the profile summary: front / left / right.
SECTORS = (("front", -30.0, 30.0), ("left", 30.0, 90.0), ("right", -90.0, -30.0))  # +-30 deg ahead; 30-90 each side (left = +y)
#: How many objects / audit records go into one line.
MAX_OBJECTS = 4  # enough to name the blocker
MAX_AUDIT = 6  # the decisions just before the stall


class StallDiag:
    """Edge / cadence logic only; the caller writes the line."""

    def __init__(self) -> None:
        self._since: Optional[float] = None   # monotonic s the stall began
        self._last_log: Optional[float] = None  # monotonic s of the last line

    def observe(self, now_s: float, route_run: bool, still: bool) -> Optional[str]:
        """"start" / "repeat" / "end" when a line is due, else None."""
        if not (route_run and still):  # moving, or not a route run
            logged = self._last_log is not None  # a start was written
            self._since, self._last_log = None, None  # reset either way
            return "end" if logged else None  # end only closes a logged stall
        if self._since is None:  # first still tick
            self._since = now_s  # the stall begins (nothing logged yet)
            return None  # nothing due on the first tick
        if self._last_log is None:  # no start written yet
            if now_s - self._since >= START_AFTER_S:  # long enough to be a stall
                self._last_log = now_s  # the repeat window starts here
                return "start"  # first line of this stall
            return None  # still inside a normal short stop
        if now_s - self._last_log >= REPEAT_S:  # one window since the last line
            self._last_log = now_s  # the next window starts here
            return "repeat"  # the stall is still on
        return None  # inside a window: stay quiet

    def duration_s(self, now_s: float) -> Optional[float]:
        """Seconds since the stall began, or None outside one."""
        return None if self._since is None else now_s - self._since  # the caller prints it in the line


def _sector_summary(profile: Any) -> List[str]:
    """Per sector: nearest blocked distance (m) and unknown share of d_free."""
    if profile is None or not getattr(profile, "d_block", None):
        return ["profile=none"]  # no geometry this tick
    out = []  # one string per sector
    for name, lo, hi in SECTORS:  # front, left, right
        blocked, unknown, n = [], 0, 0  # blocked distances, unseen rays, rays in sector
        for i, db in enumerate(profile.d_block):  # every profile bin
            a = math.degrees(profile.angle_min_rad + i * profile.angle_step_rad)  # bin angle, body frame
            if not lo <= a <= hi:
                continue  # outside this sector
            n += 1  # the bin belongs to this sector
            if db is not None:
                blocked.append(db)  # an obstacle in range on this ray
            free = profile.d_free[i] if i < len(profile.d_free) else None  # d_free may be shorter than d_block
            if free is None:
                unknown += 1  # not observed on this ray
        near = "%.2f" % min(blocked) if blocked else "-"  # "-" = nothing blocked
        out.append("%s=%s unk=%d/%d" % (name, near, unknown, n))  # e.g. front=0.60 unk=0/61
    return out  # front / left / right


def _object_summary(objects: Any) -> List[str]:
    """Nearest objects: class, confidence, distance (m) and bearing (deg)."""
    items: List[Tuple[float, str]] = []  # (distance, text) pairs for sorting
    for o in getattr(objects, "objects", ()) or ():  # perception objects this tick
        if not o.localized:  # seen but no footprint
            items.append((float("inf"), "%s:%.2f@unloc" % (o.class_name, o.confidence)))  # sorted last
            continue  # in view but not placeable
        fp = o.footprint_xy  # body-frame polygon
        cx = sum(p[0] for p in fp) / len(fp)  # body-frame centroid
        cy = sum(p[1] for p in fp) / len(fp)  # lateral, +y is left
        d = math.hypot(cx, cy)  # range from the robot
        items.append((d, "%s:%.2f@%.1fm/%+.0fdeg" % (o.class_name, o.confidence, d,  # class:conf@dist/bearing
                                                     math.degrees(math.atan2(cy, cx)))))  # bearing in degrees, left positive
    items.sort(key=lambda t: t[0])  # nearest first
    return [s for _, s in items[:MAX_OBJECTS]] or ["objects=none"]  # the placeholder keeps the field visible


def _audit_summary(records: Iterable[Any]) -> List[str]:
    """The newest RNS audit records: kind and a short detail."""
    recs = list(records)[-MAX_AUDIT:]  # the newest only
    return ["%s%s" % (r.kind, json.dumps(r.detail, ensure_ascii=False,  # kind plus the first 60 chars of detail
                                         separators=(",", ":"))[:60])  # compact JSON, no spaces
            for r in recs] or ["audit=none"]  # an empty ring says so


def describe_stall(out: Any, snap: Any, pose: Any, audit: Iterable[Any],
                   duration_s: Optional[float]) -> str:  # duration None outside a stall
    """One line for the P1 log (see the module note for the fields)."""
    head = ("dur=%s hold=%s limiter=%s nav=%s raw=(%.2f,%.2f,%.2f) vmax=%.2f"  # what P1 decided this tick
            % ("-" if duration_s is None else "%.1fs" % duration_s, out.rns_hold,  # '-' outside a stall
               out.limiter, out.nav_state, out.raw_vx, out.raw_vy, out.raw_wz, out.v_max))  # raw candidate against the gate cap
    xy = getattr(pose, "xy", None)  # None before the first fix
    yaw = getattr(pose, "yaw", None)  # None without a heading
    where = ("pose=%s yaw=%s fix=%s hdg=%s"  # where it stood and how sure
             % ("-" if xy is None else "(%.1f,%.1f)" % xy,  # metres in the P1 frame
                "-" if yaw is None else "%.0fdeg" % math.degrees(yaw),  # degrees for a human
                getattr(pose, "fix_type", None), getattr(pose, "heading_source", None)))  # fix level and heading source
    profile = getattr(snap, "profile", None) if snap is not None else None  # None when no snapshot this tick
    objects = getattr(snap, "objects", None) if snap is not None else None  # same: no snapshot, no objects
    return " | ".join([head, where, " ".join(_sector_summary(profile)),  # fields split by ' | ' for grep
                       "near: " + ", ".join(_object_summary(objects)),  # nearest objects first
                       "audit: " + "; ".join(_audit_summary(audit))])  # what RNS did just before
