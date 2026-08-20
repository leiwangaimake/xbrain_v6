"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_linkage.py
Brief: 15 S7.6 GC-1..GC-7 -- what a geo edit does to the tasks that use it

Description:
P3 is the only writer of the geo tables, so every commit it makes must be
followed by this table (15 S7.6 trigger note). This module is the pure
classification: given the change and one referencing task, what must happen to
THAT task. The mutation runs in the delete/upsert applier.

*** This file was REWRITTEN on 2026-08-20 against 15 S7.6. The previous version
classified from an earlier reading and disagreed with the specification in the
dangerous direction on its two most important rows:

  * route deleted, task running -> it returned "abort_immediately". 15 S7.6 GC-3
    says the opposite in bold: do NOT interrupt the motion. Finish the current
    lap, clamp the remaining laps, end normally. The stated reason is physical:
    stopping a patrol in the middle of a camp roadway is more dangerous than
    finishing it.
  * waypoint deleted, task running -> it returned "suspend_system". GC-5 says a
    goto already under way finishes its snapshot, and patrol is unaffected
    entirely because route vertices are inline in the snapshot and do not
    reference the waypoints table at all.

Both would have produced a robot that stops in the road when somebody tidies up
the map from the HMI. The rows below are transcribed from 15 S7.6 with the
task's STATE as an input, because every row of that table answers differently
for running / suspended / queued.

Two constraints from the same section shape the interface:

  GX-1 the classification runs AFTER the commit, serialised on the same db
       thread -- otherwise a task can be read against a row that is already
       deleted. So this is a pure function called by the applier, never a
       background sweep.
  GX-2 no row may touch P1's behaviour source. P3 changes its own task state and
       patrol_progress only. The single exception is GC-6 (dock reselect), which
       issues a new goto.
  GX-3 speed_limit / warning fences produce NO task-side linkage at all.
"""

from __future__ import annotations

from dataclasses import dataclass

# The actions a row can call for. Deliberately small, and deliberately
# containing DO NOTHING as a first-class value: three of the five rows resolve
# to "leave the task alone", and that is a decision the specification argues
# for, not an absence of one.
ACTIONS = frozenset({
    "none",             # leave the task exactly as it is
    "fail",             # terminal: -> failed, with a fail_reason
    "clamp_laps",       # GC-3 running patrol: finish this lap, drop the rest
    "reselect_dock",    # GC-6: pick another dock and issue a new goto
    "check_on_resume",  # nothing now; the resume path re-validates
    "check_on_dequeue",  # nothing now; the dequeue precondition re-validates
})

#: Task states, grouped the way 15 S7.6's three answer columns are.
_RUNNING = "running"
_SUSPENDED = "suspended"
_QUEUED = frozenset({"pending", "scheduled", "blocked", "ready"})


@dataclass(frozen=True)
class GeoChange:
    """One committed edit. `op` is delete | modify | rename; `kind` is the
    geo type. rename is a distinct op because GC-2 is the one row that is
    unconditionally "no effect" -- that is the value of separating geo_id from
    name (G-5), and collapsing rename into modify would throw it away."""
    kind: str
    op: str
    object_id: str


@dataclass(frozen=True)
class LinkageOutcome:
    """What to do with one task, and why. `reason` lands in
    error_context_json.fail_reason for a fail, and in the log otherwise."""
    action: str
    reason: str = ""


class UnknownGeoChange(Exception):
    """A (kind, op) combination 15 S7.6 does not describe. Raised rather than
    defaulted to 'none': a silent no-op on an unclassified edit is how a task
    keeps driving a route that no longer exists."""


def classify(change: GeoChange, task_state: str,
             task_type: str) -> LinkageOutcome:
    """The 15 S7.6 cell for (change, task state, task type).

    task_type matters for two rows: GC-3 clamps laps only for a patrol (nothing
    else has laps), and GC-6 applies only to a charge task.
    """
    if change.op == "rename":
        # GC-2: renaming or renumbering affects nothing, in any state. Tasks
        # reference geo_id, which rename cannot touch (S7.9.1).
        return LinkageOutcome("none", "rename does not affect references")
    if change.kind == "fence":
        # GC-4 + GX-3: a fence change takes effect in P1 immediately and P3
        # does nothing to tasks. The suspended and queued columns are
        # re-validations at resume/dequeue time, not actions now.
        if task_state == _SUSPENDED:
            return LinkageOutcome("check_on_resume",
                                  "fence changed; re-validate on resume")
        if task_state in _QUEUED:
            return LinkageOutcome("check_on_dequeue",
                                  "fence changed; re-validate on dequeue")
        return LinkageOutcome("none", "fence enforcement is P1's, not P3's")
    if change.kind == "route":
        return _route(change, task_state, task_type)
    if change.kind == "waypoint":
        return _waypoint(change, task_state)
    if change.kind == "dock":
        return _dock(change, task_state, task_type)
    raise UnknownGeoChange(f"no 15 S7.6 row for kind {change.kind!r}")


def _route(change: GeoChange, task_state: str,
           task_type: str) -> LinkageOutcome:
    """GC-1 (modified) and GC-3 (deleted)."""
    if change.op == "delete":
        if task_state == _RUNNING:
            # GC-3: NOT an abort. A patrol finishes the lap it is on with the
            # remaining laps clamped; anything else simply runs its snapshot
            # out. The snapshot is self-contained, so a deleted route row does
            # not leave the running task without geometry.
            if task_type == "patrol":
                return LinkageOutcome(
                    "clamp_laps", "route deleted; finish this lap then done")
            return LinkageOutcome("none", "route deleted; finish the snapshot")
        if task_state == _SUSPENDED:
            # The one place a delete terminates a task. It cannot be resumed
            # onto a route that no longer exists, and 15 S7.6 requires the
            # progress and snapshot rows to SURVIVE for the audit.
            return LinkageOutcome("fail", "route_deleted")
        if task_state in _QUEUED:
            # Deliberately not failed here: GC-3's queued column says the
            # DEQUEUE precondition fails (-> failed + E_NOT_FOUND). Failing it
            # now would also kill tasks whose route is re-created before they
            # ever run.
            return LinkageOutcome("check_on_dequeue",
                                  "route deleted; dequeue check will fail")
        return LinkageOutcome("none", "terminal task, nothing to do")
    if change.op == "modify":
        if task_state == _RUNNING:
            # GC-1: no hot swap. The running task keeps its snapshot; the only
            # output is the warn event telling the operator the change lands
            # next time.
            return LinkageOutcome("none", "route changed; effective next run")
        if task_state == _SUSPENDED:
            return LinkageOutcome("check_on_resume",
                                  "route changed; remap on resume (S7.3A)")
        if task_state in _QUEUED:
            return LinkageOutcome("check_on_dequeue",
                                  "route changed; snapshot rebuilt on dequeue")
        return LinkageOutcome("none", "terminal task, nothing to do")
    raise UnknownGeoChange(f"no 15 S7.6 row for route op {change.op!r}")


def _waypoint(change: GeoChange, task_state: str) -> LinkageOutcome:
    """GC-5. Note what is NOT here: patrol is unaffected by a waypoint edit in
    every state, because route geometry is inline on the route (15 S9.3 PLAN A)
    and does not reference the waypoints table."""
    if task_state == _RUNNING:
        return LinkageOutcome("none", "goto in flight finishes its snapshot")
    if task_state == _SUSPENDED:
        return LinkageOutcome("check_on_resume",
                              "waypoint edited; verified on resume")
    if task_state in _QUEUED:
        return LinkageOutcome("check_on_dequeue",
                              "waypoint edited; verified on dequeue")
    return LinkageOutcome("none", "terminal task, nothing to do")


def _dock(change: GeoChange, task_state: str,
          task_type: str) -> LinkageOutcome:
    """GC-6 / GC-7. Only a charge task heading for THAT dock is affected."""
    if task_type != "charge":
        return LinkageOutcome("none", "not a charging task")
    if task_state == _RUNNING:
        return LinkageOutcome("reselect_dock", "dock removed while returning")
    if task_state in _QUEUED or task_state == _SUSPENDED:
        return LinkageOutcome("check_on_dequeue",
                              "dock removed; dock selection re-runs")
    return LinkageOutcome("none", "terminal task, nothing to do")
