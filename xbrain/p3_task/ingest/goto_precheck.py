"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: goto_precheck.py
Brief: 11 S12.1.2 W2 -- the P3 half of an HMI goto: positioning failed / target outside the fence

Description:
The problem this solves. 11 S12.1.2's W2 table splits the checks on an HMI goto:
P5 (gate G4) refuses what it can see in the frame, and P3 refuses what needs
the live world -- E_NOT_FOUND for a missing waypoint, E_OUT_OF_FENCE for a
target outside the fence, E_DEGRADED for degraded positioning. Until
2026-11-10 none of P3's three existed: an HMI goto was stored and acknowledged
"accepted" whatever the fix and wherever the point lay, and the operator read
"sent" for a task that could not, or should not, run.

The three rules, each decided at SUBMIT so the answer reaches the browser in
the ack rather than as a task that fails later:
  * positioning failed -> E_DEGRADED. "Failed" is 99 U142's line, reused, not
    redrawn: only single and no fix fail; rtk_float / dgps take tasks at a
    reduced speed (lifecycle/rtk_loss.LOST_FIXES). No pose at all also fails.
  * waypoint_id that names no live keypoint -> E_NOT_FOUND (P5 checked its own
    copy of the layer; this is the authoritative one, geo.db).
  * fence: the target must lie inside the allow polygon when one exists, and
    outside every forbid polygon -> else E_OUT_OF_FENCE, detail.subject =
    "target" (11 S12.1.2 verbatim). warning / speed_limit polygons constrain no
    motion (11 S9A.10A) and are ignored. No active fence set -> nothing to be
    outside of, so no fence refusal.

What it does NOT do: it opens nothing and reads no clock. The wiring passes
the pose it already caches, the active FenceSet (11 S9A.2 shape, WGS84
vertices) and an async keypoint lookup. Scope: called for HMI-originated gotos
only -- the W2 table is the HMI's. Other channels are unchanged.

Trap: point_in_polygon is planar. Fence vertices and the target are both WGS84,
so (x, y) = (lon, lat) is used on both -- consistent, and the distortion is
irrelevant at fence scale (the same convention P1 uses for its alarm regions).
"""

from __future__ import annotations

from typing import Any, Awaitable, Callable, Dict, Optional, Tuple

from xbrain.common.errors import E_DEGRADED, E_NOT_FOUND, E_OUT_OF_FENCE
from xbrain.common.fence.geom import Polygon, point_in_polygon
from xbrain.p3_task.lifecycle.rtk_loss import LOST_FIXES

#: (code, message, detail) -- what TaskCommandError is built from.
Refusal = Tuple[str, str, Dict[str, Any]]

#: async (waypoint_id) -> (lat, lon) or None for a missing / tombstoned row.
KeypointLookup = Callable[[str], Awaitable[Optional[Tuple[float, float]]]]


def _polygon(poly: Dict[str, Any]) -> Optional[Polygon]:
    pts = []
    for v in poly.get("vertices") or []:
        if not isinstance(v, dict):
            return None
        la, lo = v.get("lat"), v.get("lon")
        if not isinstance(la, (int, float)) or not isinstance(lo, (int, float)):
            return None
        pts.append((float(lo), float(la)))
    return Polygon(points=tuple(pts)) if len(pts) >= 3 else None


def fence_refusal(lat: float, lon: float,
                  fence_set: Optional[Dict[str, Any]]) -> Optional[Refusal]:
    """E_OUT_OF_FENCE when (lat, lon) breaks the active fence, else None."""
    if not isinstance(fence_set, dict):
        return None
    allow, forbid = [], []
    for poly in fence_set.get("polygons") or []:
        if not isinstance(poly, dict):
            continue
        shape = _polygon(poly)
        if shape is None:
            continue
        if poly.get("role") == "allow":
            allow.append((poly, shape))
        elif poly.get("role") == "forbid":
            forbid.append((poly, shape))
    if allow and not any(point_in_polygon(lon, lat, s) for _p, s in allow):
        return (E_OUT_OF_FENCE, "goto target is outside the allow fence",
                {"reason": "target_outside_allow_fence", "subject": "target",
                 "role": "allow"})
    for poly, shape in forbid:
        if point_in_polygon(lon, lat, shape):
            return (E_OUT_OF_FENCE, "goto target is inside a forbid zone",
                    {"reason": "target_inside_forbid_zone", "subject": "target",
                     "role": "forbid", "poly_id": poly.get("poly_id")})
    return None


async def hmi_goto_refusal(task: Dict[str, Any], *,
                           pose: Optional[Dict[str, Any]],
                           fence_set: Optional[Dict[str, Any]],
                           keypoint: KeypointLookup) -> Optional[Refusal]:
    """The first W2 rule an HMI goto breaks, or None when it may be stored."""
    fix = (pose or {}).get("fix_type")
    if not pose or fix is None or fix in LOST_FIXES:
        return (E_DEGRADED, "positioning has failed (single or no fix); a goto "
                "cannot be planned", {"reason": "positioning_failed",
                                      "fix_type": fix})
    params = task.get("params") or {}
    wid = params.get("waypoint_id")
    if isinstance(wid, str) and wid:
        point = await keypoint(wid)
        if point is None:
            return (E_NOT_FOUND, "keypoint %r not found" % wid,
                    {"reason": "keypoint_not_found", "subject": "target",
                     "waypoint_id": wid})
        lat, lon = point
    else:
        lat, lon = params.get("lat"), params.get("lon")
        if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
            # parse_task_command already insists on a destination; a goto that
            # still has no usable coordinate here is not ours to place.
            return None
    return fence_refusal(float(lat), float(lon), fence_set)
