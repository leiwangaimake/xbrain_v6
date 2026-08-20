"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_object.py
Brief: GeoObject body (11 S7.8) -> validated column set for the geo/fence tables

Description:
The upsert applier receives a GeoObject (11 S7.8.2 metadata + S7.8.3 geometry)
and has to turn it into one row. This module is that translation, kept apart
from the applier because it is pure: given a dict, produce either a validated
ParsedObject or a GeoCommandError. No connection, no clock, no transaction.

What it validates, and why each one is here rather than left to the DB:

  * the geometry SHAPE per type (S7.8.3). A route with one point, a fence ring
    with two vertices, a waypoint with no lat -- the DB would take all three
    (they violate no column constraint) and the robot would find out later, at
    the point where the geometry is used to move.
  * lat/lon RANGE. An ENU metre value (say 12.5, 40.2) is a perfectly valid
    float in a lat/lon column and lands the object a few hundred kilometres
    away. This is not hypothetical here: the geo model was WGS84-migrated on
    2026-08-20 and the teach path still holds ENU samples internally, so a
    missed projection produces exactly those numbers.
  * name presence. routes/waypoints/docks have NOT NULL UNIQUE names because
    voice navigation resolves "go to <name>" to one object.
  * num range and alias shape, the two S7.8.2 fields rename may also change.

What it deliberately does NOT do: assign rev / content_hash / state /
created_by / updated_by / updated_ms. Those are the single writer's to set
(S7.9.2 step 6 says rev from the sender is IGNORED), so they are not even read
from the incoming object -- a sender that puts rev: 99 in its body gets the rev
P3 decides, with no branch anywhere that could honour the sender's number.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from xbrain.common.enums import FENCE_ROLE
from xbrain.common.errors import E_GEO_INVALID, E_GEO_TOO_LARGE
from xbrain.p3_task.ingest.geo_command import GeoCommandError

# 11 S7.8.3: a route carries at most 5000 vertices; beyond that the answer is
# E_GEO_TOO_LARGE rather than a truncated route (a silently shortened patrol
# path would be walked to its new, wrong end).
_MAX_PATH_POINTS = 5000
_MIN_ROUTE_POINTS = 2
# 11 S7.8.3 / CMD-18: a polygon needs three distinct vertices to bound an area.
_MIN_RING_POINTS = 3

_EARTH_R_M = 6371000.0     # spherical model; total_len_m is a display/replay
#                            length, not a survey figure (same as geo_commit).

#: type -> (table, primary-key column, geo_id prefix). The prefix is 15 S9.3
#: ID-2 and is also enforced by a GLOB CHECK in the DDL; it is repeated here so
#: the refusal names the rule instead of surfacing as an opaque IntegrityError.
TABLE_FOR_TYPE: Dict[str, Tuple[str, str, str]] = {
    "route": ("routes", "geo_id", "r-"),
    "waypoint": ("waypoints", "geo_id", "w-"),
    "dock": ("docks", "geo_id", "d-"),
    "fence": ("fences", "fence_id", "f-"),
}


@dataclass(frozen=True)
class ParsedObject:
    """One validated row-to-be. `columns` holds only sender-owned values; the
    writer adds the lifecycle and sync columns. `content` is what the
    content_hash is taken over -- geometry plus name, per S7.8.2."""
    table: str
    pk_col: str
    columns: Dict[str, Any]
    content: Dict[str, Any]


def _bad(msg: str) -> GeoCommandError:
    return GeoCommandError(E_GEO_INVALID, msg)


def _latlon(point: Any, what: str) -> Tuple[float, float]:
    """One WGS84 coordinate from either {lat, lon} or [lat, lon].

    Both spellings are accepted because both appear in the system already: the
    contract's S7.8.3 examples use objects, and the stored path_points column is
    an array of pairs. What is NOT accepted is a value outside the WGS84 range
    -- see the module docstring on ENU metres arriving as coordinates.
    """
    if isinstance(point, dict):
        lat, lon = point.get("lat"), point.get("lon")
    elif isinstance(point, (list, tuple)) and len(point) >= 2:
        lat, lon = point[0], point[1]
    else:
        raise _bad(f"{what}: point is neither an object nor a [lat, lon] pair")
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
        raise _bad(f"{what}: lat/lon must be numbers")
    if isinstance(lat, bool) or isinstance(lon, bool):
        # bool is an int subclass in Python; True would read as latitude 1.0.
        raise _bad(f"{what}: lat/lon must be numbers, not booleans")
    if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
        raise _bad(f"{what}: lat/lon {lat},{lon} outside WGS84 range "
                   "(ENU metres reaching a WGS84 field look exactly like this)")
    return float(lat), float(lon)


def _haversine_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Great-circle distance in metres between two (lat, lon) degree pairs."""
    la1, lo1, la2, lo2 = (math.radians(a[0]), math.radians(a[1]),
                          math.radians(b[0]), math.radians(b[1]))
    h = (math.sin((la2 - la1) / 2.0) ** 2
         + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2.0) ** 2)
    return 2.0 * _EARTH_R_M * math.asin(min(1.0, math.sqrt(h)))


def polyline_len_m(pts: Sequence[Tuple[float, float]]) -> float:
    """Sum of segment lengths over an ordered (lat, lon) list (11 S7.8.3)."""
    return sum(_haversine_m(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _name(obj: Dict[str, Any], required: bool) -> Optional[str]:
    """S7.8.2 name. Required for the three types whose column is NOT NULL."""
    name = obj.get("name")
    if name is None and not required:
        return None
    if not isinstance(name, str) or not name.strip():
        raise _bad("name is required and must be a non-empty string")
    return name.strip()


def _num(obj: Dict[str, Any]) -> Optional[int]:
    """S7.8.2 num: 1..99, optional. This is what voice "3 hao lu jing" resolves
    on, so 0 and 100 are refused rather than stored and never matched."""
    num = obj.get("num")
    if num is None:
        return None
    if not isinstance(num, int) or isinstance(num, bool) or not 1 <= num <= 99:
        raise _bad(f"num must be an integer 1..99, got {num!r}")
    return num


def _alias_json(obj: Dict[str, Any]) -> str:
    """S7.8.2 alias[]: ASR L3 synonyms. Stored as JSON text; an entry that is
    not a string would come back out of the column and into the ASR hint list."""
    alias = obj.get("alias")
    if alias is None:
        return "[]"
    if not isinstance(alias, list) or any(not isinstance(a, str)
                                          for a in alias):
        raise _bad("alias must be a list of strings")
    return json.dumps(alias, ensure_ascii=False, separators=(",", ":"))


def _geom(obj: Dict[str, Any]) -> Dict[str, Any]:
    geom = obj.get("geom")
    if not isinstance(geom, dict):
        raise _bad("obj.geom is required and must be an object (11 S7.8.3)")
    return geom


def _route(obj: Dict[str, Any]) -> ParsedObject:
    """S7.8.3 route: an inline vertex list (mode B) or named anchors (mode A).

    total_len_m is computed HERE and not taken from the sender even though
    S7.8.3 lists it as a field: it is the input to break-point remapping
    (S7.12), and a sender-supplied length that disagrees with the geometry
    would make a resumed patrol restart at the wrong fraction of the route.
    """
    geom = _geom(obj)
    name = _name(obj, required=True)
    loop_mode = geom.get("loop_mode", "oneway")
    if loop_mode not in ("oneway", "pingpong", "closed"):
        raise _bad(f"bad loop_mode {loop_mode!r}")
    direction = geom.get("direction", "forward")
    if direction not in ("forward", "reverse"):
        raise _bad(f"bad direction {direction!r}")
    raw_points = geom.get("points")
    anchors = geom.get("waypoint_ids")
    if (raw_points is None) == (anchors is None):
        # The routes table enforces this XOR too; refusing here names which of
        # the two geometries was expected instead of raising an SQL CHECK.
        raise _bad("route needs exactly one of geom.points / geom.waypoint_ids")
    columns: Dict[str, Any] = {
        "name": name,
        "loop_mode": loop_mode,
        "direction": direction,
        "max_speed": geom.get("max_speed"),
        "description": obj.get("description"),
        "num": _num(obj),
        "alias_json": _alias_json(obj),
    }
    if raw_points is not None:
        if not isinstance(raw_points, list):
            raise _bad("geom.points must be a list")
        if len(raw_points) > _MAX_PATH_POINTS:
            raise GeoCommandError(
                E_GEO_TOO_LARGE,
                f"route has {len(raw_points)} points, cap is "
                f"{_MAX_PATH_POINTS} (11 S7.8.3)")
        if len(raw_points) < _MIN_ROUTE_POINTS:
            raise _bad(f"route needs >= {_MIN_ROUTE_POINTS} points")
        pts = [_latlon(p, f"geom.points[{i}]")
               for i, p in enumerate(raw_points)]
        total = polyline_len_m(pts)
        if total <= 0.0:
            # The routes CHECK would abort on this; refusing here says WHY a
            # route of coincident points is not a route.
            raise _bad("route has zero length (all points coincide)")
        columns["path_points"] = json.dumps([[la, lo] for la, lo in pts],
                                            separators=(",", ":"))
        columns["waypoint_ids"] = None
        columns["total_len_m"] = total
        content = {"pp": [[la, lo] for la, lo in pts], "name": name}
    else:
        if (not isinstance(anchors, list)
                or any(not isinstance(a, str) for a in anchors)):
            raise _bad("geom.waypoint_ids must be a list of geo_id strings")
        if len(anchors) < _MIN_ROUTE_POINTS:
            raise _bad(f"route needs >= {_MIN_ROUTE_POINTS} anchors")
        columns["waypoint_ids"] = json.dumps(list(anchors),
                                             separators=(",", ":"))
        columns["path_points"] = None
        # total_len_m over anchors needs their coordinates, which means a db
        # read; the writer fills it in (it holds the connection). None here is
        # the "not computed yet" marker, allowed by the column and by its CHECK.
        columns["total_len_m"] = None
        content = {"wi": list(anchors), "name": name}
    return ParsedObject("routes", "geo_id", columns, content)


def _waypoint(obj: Dict[str, Any]) -> ParsedObject:
    """S7.8.3 waypoint: one WGS84 keypoint plus its arrival radius. This is the
    F06 "record this spot as the east gate" object."""
    geom = _geom(obj)
    name = _name(obj, required=True)
    lat, lon = _latlon(geom, "geom")
    radius = geom.get("arrive_radius_m", 1.0)
    if not isinstance(radius, (int, float)) or radius <= 0.0:
        raise _bad("arrive_radius_m must be a positive number")
    columns = {
        "name": name,
        # S7.8.1 has no closed set for the waypoint TYPE column (poi/dock/home/
        # gate/...); it is descriptive, so an unrecognised value is stored as
        # given rather than refused. poi is the neutral default for a spot the
        # operator just named.
        "type": obj.get("wtype") or geom.get("type") or "poi",
        "rtk_lat": lat, "rtk_lon": lon, "rtk_alt": geom.get("alt"),
        "yaw_deg": geom.get("yaw_deg"),
        "arrival_radius": float(radius),
        "description": obj.get("description"),
        "num": _num(obj), "alias_json": _alias_json(obj),
    }
    content = {"name": name, "lat": lat, "lon": lon,
               "yaw": geom.get("yaw_deg")}
    return ParsedObject("waypoints", "geo_id", columns, content)


def _fence(obj: Dict[str, Any]) -> ParsedObject:
    """S9A.2 fence: a WGS84 vertex ring plus its ROLE.

    On role vs the S7.8.3 "kind": S7.8.3's fence example predates the S9A.2
    role model and writes geom.kind = keep_in | keep_out. The stored model --
    and the closed set every consumer classifies on (FENCE_ROLE, S9A.2) -- is
    role = allow | forbid | speed_limit | warning, and the kind COLUMN means the
    geometry shape (polygon | circle). The two keep_* values have no way to
    express speed_limit or warning, so they are not accepted as a role: an
    aliasing table here would be "degrade the unknown value to something close",
    which 11 S13.6 forbids by name. See the 2026-08-20 note added to S7.8.3.
    """
    geom = _geom(obj)
    role = geom.get("role") or obj.get("role")
    if role not in FENCE_ROLE:
        raise _bad(f"fence role {role!r} is not in the 11 S9A.2 closed set "
                   f"{sorted(FENCE_ROLE)}")
    ring = geom.get("outer")
    if ring is None:
        ring = geom.get("points")
    if not isinstance(ring, list):
        raise _bad("fence needs geom.outer (or geom.points), a vertex list")
    if len(ring) < _MIN_RING_POINTS:
        raise _bad(f"fence ring needs >= {_MIN_RING_POINTS} vertices (CMD-18)")
    verts = [_latlon(p, f"geom.outer[{i}]") for i, p in enumerate(ring)]
    margin = geom.get("soft_margin_m")
    if margin is not None and (not isinstance(margin, (int, float))
                               or margin < 0.0):
        raise _bad("soft_margin_m must be a non-negative number")
    columns = {
        # name is nullable on fences (unlike the other three): S9A.2 lists it as
        # a display field, and a fence recorded by voice may be saved before it
        # is named (F08 then F09).
        "name": _name(obj, required=False),
        "role": role,
        "kind": "polygon",
        "geom_json": json.dumps(
            {"points": [[la, lo] for la, lo in verts]},
            separators=(",", ":")),
        # S9A.2: a warning fence never hard-enforces. Derived, never taken from
        # the sender -- a sender-set hard_enforce on a warning fence would make
        # P1 clip against an advisory zone.
        "hard_enforce": 0 if role == "warning" else 1,
        "soft_margin_m": margin,
        "num": _num(obj), "alias_json": _alias_json(obj),
    }
    content = {"points": [[la, lo] for la, lo in verts], "role": role,
               "name": columns["name"]}
    return ParsedObject("fences", "fence_id", columns, content)


def _dock(obj: Dict[str, Any]) -> ParsedObject:
    """S7.8.3 dock: body pose + handover point. Charging is not wired yet, so
    this exists to make F10 ("record this as a charging dock") storable and
    renderable; nothing consumes the handover tolerances at run time today."""
    geom = _geom(obj)
    name = _name(obj, required=True)
    lat, lon = _latlon(geom, "geom")
    heading = geom.get("dock_heading_rad")
    if not isinstance(heading, (int, float)):
        raise _bad("dock needs geom.dock_heading_rad (the approach direction)")
    handover = geom.get("handover")
    if not isinstance(handover, dict):
        raise _bad("dock needs geom.handover (the stage-1 nav target)")
    h_lat, h_lon = _latlon(handover, "geom.handover")
    h_heading = handover.get("heading_rad")
    if not isinstance(h_heading, (int, float)):
        raise _bad("dock needs geom.handover.heading_rad")
    on_route = geom.get("on_route", [])
    if (not isinstance(on_route, list)
            or any(not isinstance(r, str) for r in on_route)):
        raise _bad("geom.on_route must be a list of route geo_ids")
    columns = {
        "name": name, "num": _num(obj),
        "rtk_lat": lat, "rtk_lon": lon, "rtk_alt": geom.get("alt"),
        "dock_heading_rad": float(heading),
        "handover_lat": h_lat, "handover_lon": h_lon,
        "handover_heading_rad": float(h_heading),
        "on_route_json": json.dumps(list(on_route), separators=(",", ":")),
        # enabled is the CHG-side "disable != delete" flag (G-3) and is separate
        # from the S7.8.2 lifecycle state; both default to in-service.
        "enabled": 1 if geom.get("enabled", True) else 0,
        "description": obj.get("description"),
        "alias_json": _alias_json(obj),
    }
    content = {"name": name, "lat": lat, "lon": lon,
               "handover": [h_lat, h_lon]}
    return ParsedObject("docks", "geo_id", columns, content)


_PARSERS = {"route": _route, "waypoint": _waypoint, "fence": _fence,
            "dock": _dock}


def parse_geo_object(gtype: str, geo_id: str,
                     obj: Dict[str, Any]) -> ParsedObject:
    """Validate a GeoObject body of type `gtype` into a ParsedObject.

    geo_id is checked against its S9.3 ID-2 prefix here so a mistyped id fails
    with the rule quoted, rather than as an SQL CHECK abort during the write.
    """
    entry = TABLE_FOR_TYPE.get(gtype)
    if entry is None:
        # parse_geo_command already refused off-set types; reaching this means
        # TABLE_FOR_TYPE and GEO_TYPE disagree, which is a build error.
        raise _bad(f"no table mapping for geo type {gtype!r}")
    table, pk_col, prefix = entry
    if not geo_id.startswith(prefix):
        raise _bad(f"geo_id {geo_id!r} must start with {prefix!r} "
                   f"for type {gtype!r} (15 S9.3 ID-2)")
    parsed = _PARSERS[gtype](obj)
    return ParsedObject(table, pk_col, parsed.columns, parsed.content)


def resolvable_anchor_ids(parsed: ParsedObject) -> List[str]:
    """The anchor geo_ids of a mode-A route, or an empty list.

    Exposed so the writer can resolve them to coordinates (for total_len_m)
    without re-parsing the JSON it just wrote.
    """
    raw = parsed.columns.get("waypoint_ids")
    if not raw:
        return []
    return list(json.loads(raw))
