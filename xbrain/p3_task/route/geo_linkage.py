"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_linkage.py
Brief: BIZ-P3-14 geo change x task state linkage GC-1..7 + GX-1..3

Description:
15 S9 defines seven linkage rules between edits to geographic
objects (waypoints / routes / docks / fences) and the states of
tasks that reference them:

  GC-1 delete-referenced-waypoint  -> tasks 'suspended'(kind='system',
                                       reason='waypoint_deleted')
  GC-2 modify-referenced-waypoint  -> patrol_progress may need remap
  GC-3 delete-referenced-route     -> tasks 'aborted' immediately
  GC-4 replace-route (same id)     -> if resume_policy=='exact' remap;
                                       else nearest_wp / restart
  GC-5 delete-referenced-dock      -> charging tasks aborted; ongoing
                                       charge sessions kept (no self-eject)
  GC-6 fence add/modify            -> re-evaluate V-4 for every
                                       pending / queued / running task
  GC-7 fence delete                -> re-evaluate V-4 as above; may
                                       ADMIT previously-blocked tasks

GX-1..3 are the RECOVERY paths (repair, ignore, cancel).

This module holds the pure classification: given an edit description,
what response is required. The actual mutation runs from the
scheduler using the DAOs.
"""

from __future__ import annotations

from dataclasses import dataclass


class LinkageResponse(str):
    pass


RESPONSES = frozenset({
    "suspend_system", "abort_immediately", "remap_required",
    "re_evaluate_v4", "no_action",
})


@dataclass(frozen=True)
class GeoChange:
    kind: str        # 'waypoint' / 'route' / 'dock' / 'fence'
    op: str          # 'delete' / 'modify' / 'add' / 'replace'
    object_id: str


class UnknownGeoChange(Exception):
    pass


def classify(change: GeoChange, referenced: bool) -> str:
    """Return one of RESPONSES. Raises on an unknown (kind, op).
    `referenced`: does any active (pending/queued/running/suspended)
    task refer to this object? If not, no_action is fine even for
    a delete."""
    key = (change.kind, change.op)
    if not referenced:
        # Fences are the exception -- they must always trigger a
        # V-4 sweep regardless of current references (V-4 might
        # ADMIT previously-blocked candidates).
        if change.kind == "fence":
            return "re_evaluate_v4"
        return "no_action"

    if key == ("waypoint", "delete"):
        return "suspend_system"           # GC-1
    if key == ("waypoint", "modify"):
        return "remap_required"           # GC-2
    if key == ("route", "delete"):
        return "abort_immediately"        # GC-3
    if key == ("route", "replace"):
        return "remap_required"           # GC-4
    if key == ("dock", "delete"):
        return "abort_immediately"        # GC-5
    if key == ("fence", "add"):
        return "re_evaluate_v4"           # GC-6
    if key == ("fence", "modify"):
        return "re_evaluate_v4"           # GC-6
    if key == ("fence", "delete"):
        return "re_evaluate_v4"           # GC-7
    raise UnknownGeoChange(f"kind/op combo not classified: {key}")
