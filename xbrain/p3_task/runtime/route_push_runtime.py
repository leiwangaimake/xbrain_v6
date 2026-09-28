"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: route_push_runtime.py
Brief: dispatch -> snapshot -> cmd/motion/route -> RA-1 window (EX-2)

Description:
The piece that makes a task drive the chassis. 15 S2.4 is blunt about why it
has to exist: "按 v0.3 实现, path_follow 没有几何来源, patrol 无法实现".
P1 has been able to receive the geometry since P7.2 -- it subscribes
cmd/motion/route (P1-11, nav_wiring.py CMD_ROUTE_TOPIC), parses it
(nav/route_intake.py) and starts the RNS mission on it (nav/mission_host.py
on_route) -- and P3 has had the route rows all along. Nothing connected the
two: before 2026-09-28 `grep -rn "cmd/motion/route" xbrain/p3_task/` matched
one comment.

Order of operations, and every one of them is load-bearing:

  SN-1 (15 S9.3A)   write and COMMIT task_route_snapshot first, THEN push.
                    Reversed, a power cut in between leaves P1 holding
                    geometry P3 has no snapshot of, and S7.3A remaps against
                    the wrong T0.
  chunks in order   every frame carries the same cmd_id; P1 assembles by it
                    and swaps the mission pointer only when all `total`
                    indices are in (RG-1/RG-3).
  RA-1 window       opened AFTER the last chunk (15 S2.4.4), and only then.
                    While it is open the task must receive no motion command.

*** WHY NO BehaviorCommand{path_follow} IS SENT, although 15 S2.4.1 says
"收齐 ack 后才发". 11 registers cmd/motion/behavior with p3_task among its
publishers and p1_motion as subscriber (P1-3), but P1 HAS NO SUBSCRIBER for
it: `grep -n "cmd/motion/behavior" xbrain/p1_motion/runtime/nav_wiring.py`
is empty -- the wiring declares cmd/motion/route, /relative_move and /factor
and nothing else. Publishing into a key nobody reads would be a line of code
that looks like the mission being started and is not (CLAUDE.md 3.2 form 1),
and it is not needed for motion in this build: MissionHost.on_route starts
the mission from the RouteSet itself. Adding the P1-3 subscriber is real
12-domain work (route_rev cross-check -> E_GEO_CONFLICT, the LP-1..LP-8 loop
state machine) and is reported as the remaining half of EX-2 rather than
faked here.

What this module does NOT do: it does not decide WHEN to dispatch (the
scheduler tick does), does not build frames or judge acks (route/push.py),
and does not read geo.db beyond the two queries below.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from xbrain.p3_task.persistence.schema_task import iso_from_wall_ms
from xbrain.p3_task.route.push import (
    RouteAck,
    RouteAckWindow,
    RoutePushError,
    RoutePushTrigger,
    build_route_frames,
    new_cmd_id,
)
from xbrain.p3_task.route.snapshot_build import SnapshotBuildError, build_snapshot

_logger = logging.getLogger("xbrain.p3.route_push")

#: 11 S2.2: cmd/motion/route is a general-plane key, QoS Q3_cmd (reliable).
CMD_ROUTE_TOPIC = "cmd/motion/route"


@dataclass(frozen=True)
class PushResult:
    """Outcome of one push attempt, for the caller's log and for tests."""
    pushed: bool
    frames: int
    cmd_id: str
    reason: str          # '' on success, else why nothing was sent


async def load_route_for_task(geo_conn, route_geo_id: str):
    """Read the routes row plus, for mode A, its anchors.

    Returns (row, anchor_lookup) where row is
    (geo_id, name, waypoint_ids, path_points, loop_mode, direction,
     total_len_m, rev) and anchor_lookup maps geo_id ->
    (rtk_lat, rtk_lon, arrival_radius). Raises LookupError when the route is
    gone -- the dequeue precondition V-8 should already have caught that, so
    reaching here means the route was deleted between validation and
    dispatch, which is a real race and not a reason to push a partial path.
    """
    cur = await geo_conn.execute(
        "SELECT geo_id, name, waypoint_ids, path_points, loop_mode,"
        " direction, total_len_m, rev FROM routes"
        " WHERE geo_id=? AND tombstone=0", (route_geo_id,))
    row = await cur.fetchone()
    if row is None:
        raise LookupError("route %r not found or tombstoned" % route_geo_id)
    anchors: Dict[str, Tuple[float, float, float]] = {}
    if row[2]:
        for wid in json.loads(row[2]):
            c = await geo_conn.execute(
                "SELECT rtk_lat, rtk_lon, arrival_radius FROM waypoints"
                " WHERE geo_id=? AND tombstone=0", (str(wid),))
            got = await c.fetchone()
            if got is not None:
                anchors[str(wid)] = (got[0], got[1], got[2])
    return row, anchors


async def push_route_for_task(*, task_id: str, route_geo_id: str,
                              task_conn, geo_conn, snapshot_dao,
                              progress_dao, route_pub, ack_window,
                              trigger: RoutePushTrigger,
                              now_mono_ms: int, now_wall_ms: int,
                              recorded_arrive_radius_m: Optional[float] = None,
                              ) -> PushResult:
    """RP-1 / RP-2 / RP-3: snapshot the route, push it, open the ack window.

    RP-4 never reaches here: a task entering a terminal pushes nothing and
    revokes nothing (15 S2.4.1), and the geometry staying resident in P1 is
    what 15 S1.3 T-1 relies on.

    Returns a PushResult rather than raising for the expected refusals (route
    missing, mode-B radius not configured, over the point cap). The caller
    turns those into the task failure the contract asks for; raising would
    put that decision in the wrong place and would take the p3 loop with it.
    """
    if trigger is RoutePushTrigger.RP4_TASK_TERMINAL:
        # Stated as a guard rather than left to the caller: "do nothing" is a
        # rule that disappears the moment somebody adds an else branch.
        return PushResult(False, 0, "", "rp4_pushes_nothing")
    try:
        row, anchors = await load_route_for_task(geo_conn, route_geo_id)
    except LookupError as exc:
        return PushResult(False, 0, "", "route_missing: %s" % exc)
    _gid, name, waypoint_ids, path_points, loop_mode, direction, \
        total_len_m, rev = row
    try:
        snap = build_snapshot(
            task_id=task_id, route_id=route_geo_id, rev=int(rev),
            loop_mode=loop_mode, path_points=path_points,
            waypoint_ids=waypoint_ids, anchor_lookup=anchors,
            recorded_arrive_radius_m=recorded_arrive_radius_m,
            total_len_m=total_len_m)
        frames = build_route_frames(
            route_id=snap.route_id, route_rev=snap.rev,
            loop_mode=snap.loop_mode, total_len_m=snap.total_len_m,
            points=snap.points, cmd_id=new_cmd_id())
    except (SnapshotBuildError, RoutePushError) as exc:
        # Named, not swallowed: "the patrol did not start" with no reason is
        # the report this whole batch exists to stop producing.
        _logger.error("p3 route push refused for task %s (%s): %s",
                      task_id, route_geo_id, exc)
        return PushResult(False, 0, "", "refused: %s" % exc)

    now_iso = iso_from_wall_ms(now_wall_ms)
    # SN-1: snapshot first, and COMMITTED before the first frame leaves.
    await snapshot_dao.replace(snap, now_iso=now_iso)
    # The progress run is opened in the same transaction as the snapshot: a
    # snapshot with no progress row would leave the breakpoint columns with
    # nowhere to land, and EX-3's writer would report "no active row" for the
    # whole patrol.
    if trigger is RoutePushTrigger.RP1_DISPATCH:
        await progress_dao.start_run(
            task_id=task_id, route_name=name, route_geo_id=snap.route_id,
            route_rev=snap.rev, direction=direction or "forward",
            loop_mode=snap.loop_mode, waypoint_total=snap.point_count,
            route_total_m=snap.total_len_m,
            # loop_total comes from the task's mission (loops). Not wired
            # yet -- mission expansion is EX-1 -- so the DDL default of 1
            # stands and is NOT overwritten with a guess here. A wrong
            # loop_total would make LP-3 judge completion on the wrong lap.
            loop_total=1, now_iso=now_iso)
    await task_conn.commit()

    for frame in frames:
        route_pub.put(json.dumps(frame, ensure_ascii=False).encode("utf-8"))
    # 15 S2.4.4: the window opens after the LAST chunk, not the first. One
    # that starts early can expire while frames are still going out.
    ack_window.open(snap.route_id, snap.rev, now_mono_ms=now_mono_ms)
    _logger.info(
        "p3 %s push task=%s route=%s rev=%d points=%d frames=%d cmd_id=%s",
        trigger.value, task_id, snap.route_id, snap.rev, snap.point_count,
        len(frames), frames[0]["cmd_id"])
    return PushResult(True, len(frames), frames[0]["cmd_id"], "")


def ack_outcome_is_fatal(outcome) -> bool:
    """15 S2.4.2 last row: retries exhausted -> the task goes to `failed`
    with error_context_json.code = E_GEO_INCOMPLETE.

    `failed` and not `suspended`, because 11 S4.4's suspend_reason is a
    closed set of 11 values with no entry for "geometry push failed", and
    15 does not invent enum values (Q-P3-24). The failure direction is safe
    either way: a failed task raises a dialog, a suspended one is grey text.
    """
    return outcome is not None and not outcome.retry_allowed \
        and outcome.ack in (RouteAck.RA2_REPORTED_INCOMPLETE,
                            RouteAck.RA3_TIMEOUT)


def make_ack_window(**kw: Any) -> RouteAckWindow:
    """One window per process: 15 S6.1 allows one running task at a time, so
    there is never more than one push in flight."""
    return RouteAckWindow(**kw)
