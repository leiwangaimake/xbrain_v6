"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: fence_set.py
Brief: Build the 11 S9A.2 FenceSet (with crc32) from fence.db active rows

Description:
The GEOMETRY-BROADCAST half of the fence runtime (11 S9A.3). P3 is the sole
cmd/fence publisher; this module turns the active fence.db rows into a FenceSet
message (11 S9A.2) so P3 can broadcast the real stored geometry instead of the
demo injector. It:
  * runs validate_active_fence_set (11 S9A.1A FS-5A: exactly 1 allow, <= 5 total)
    -- FV-3 of the S9A.3 compile checklist -- BEFORE building, so an invalid set
    never goes on the wire;
  * shapes each row into a polygon {poly_id, role, name, winding, hard_enforce,
    vertices:[{lat,lon}]} (winding declared per role: allow->ccw, keep-out->cw;
    the receiver re-normalises per 11 S9A.2 and does not trust the sender);
  * computes crc32 by the EXACT 11 S9A.2 normalisation (fence_set_id|rev| then per
    polygon poly_id|role|winding|hard_enforce(1/0)| then each vertex %.8f,%.8f;),
    CRC-32 IEEE 802.3 == zlib.crc32, 8 lowercase hex -- so the C++/Python receiver
    self-computes the SAME value (FV-8).

Scope (honest, 2026-08-20): this is the geometry broadcast only. The two-stage
FenceCommand handshake of 11 S9A.3 (op=stage -> cmd/fence/ack -> op=commit +
require_ack_from + P1 state/fence) is DEFERRED until P1 fence clipping lands --
there is no consumer to ack yet. Until then P3 broadcasts the FenceSet directly
(the demo injector already did the flat shape; this upgrades it to come from
fence.db with proper metadata). Do NOT read this as the commit protocol.
"""
from __future__ import annotations

import json
import zlib
from typing import Any, Dict, List, Optional, Sequence

from xbrain.p3_task.fence.geom import validate_active_fence_set

# Declared winding per role (11 S9A.2: receiver normalises, this is a hint).
_WINDING = {"allow": "ccw", "forbid": "cw", "speed_limit": "cw", "warning": "cw"}


def _vertices(geom_json: str) -> List[Dict[str, float]]:
    """fence.db geom_json {"points": [[lat,lon],...]} -> [{lat,lon},...]."""
    pts = json.loads(geom_json).get("points", [])
    return [{"lat": float(la), "lon": float(lo)} for la, lo in pts]


def fence_set_crc32(fence_set_id: str, rev: int,
                    polygons: Sequence[Dict[str, Any]]) -> str:
    """11 S9A.2 crc32: fence_set_id|rev| then per polygon (array order)
    poly_id|role|winding|hard_enforce(1/0)| then each vertex '%.8f,%.8f;'.
    CRC-32 IEEE 802.3 (== zlib.crc32), 8 lowercase hex. The receiver self-computes
    and compares (FV-8); a byte-identical set MUST yield the same crc32."""
    parts = ["%s|%d|" % (fence_set_id, rev)]
    for p in polygons:
        parts.append("%s|%s|%s|%s|" % (
            p["poly_id"], p["role"], p["winding"],
            "1" if p["hard_enforce"] else "0"))
        for v in p["vertices"]:
            parts.append("%.8f,%.8f;" % (v["lat"], v["lon"]))
    data = "".join(parts).encode("utf-8")
    return "%08x" % (zlib.crc32(data) & 0xFFFFFFFF)


def build_fence_set(active_rows: Sequence, *, fence_set_id: str, rev: int,
                    enu_origin: Optional[Dict[str, float]] = None,
                    soft_margin_min_m: Optional[float] = None) -> Dict[str, Any]:
    """Build a 11 S9A.2 FenceSet from FencesDAO.list_active() rows. Each row is
    (fence_id, name, role, kind, geom_json, hard_enforce, rev). Raises
    InvalidFenceSet (FV-3) if the active set is not exactly-1-allow / <= 5."""
    roles = [r[2] for r in active_rows]
    validate_active_fence_set(roles)               # FV-3 / S9A.1A -- raises on bad set
    polygons: List[Dict[str, Any]] = []
    for (fence_id, name, role, _kind, geom_json, hard_enforce, _rev) in active_rows:
        polygons.append({
            "poly_id": fence_id,
            "role": role,
            "name": name,
            "winding": _WINDING.get(role, "ccw"),
            "hard_enforce": bool(hard_enforce),
            "vertices": _vertices(geom_json),
        })
    return {
        "fence_set_id": fence_set_id,
        "rev": rev,
        "crc32": fence_set_crc32(fence_set_id, rev, polygons),
        "enu_origin": enu_origin,
        "soft_margin_min_m": soft_margin_min_m,
        "polygons": polygons,
    }
