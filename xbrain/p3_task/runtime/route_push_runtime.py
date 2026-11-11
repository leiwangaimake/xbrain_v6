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
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from xbrain.p3_task.persistence.schema_task import iso_from_wall_ms
from xbrain.p3_task.route.push import (
    RouteAck,
    RouteAckWindow,
    RoutePushError,
    RoutePushTrigger,
    build_route_frames,
    new_cmd_id,
)
from xbrain.p3_task.route.laps import LapSpec
from xbrain.p3_task.route.route_plan import RouteGeom, plan_goto, plan_patrol
from xbrain.p3_task.route.snapshot_build import (
    SnapshotBuildError,
    build_composite_snapshot,
    build_goto_snapshot,
    build_snapshot,
)

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
    # 11 S6.2 (99 U140): the closed cause a failed task carries in
    # error_context_json and so in its terminal event -- '' on success, else
    # one of PUSH_FAIL_CAUSES. reason above is free text for the log; P4
    # voices the failure from THIS field, never by parsing reason.
    cause: str


#: 11 S6.2 error.cause for a geometry-delivery failure (99 U140). P3 also
#: writes 'route_unconfirmed' (RA-2 / RA-3 fatal, main_wiring), which is not a
#: push outcome and so never appears in a PushResult.
PUSH_FAIL_CAUSES = ("no_position", "target_missing", "plan_refused")


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
                              start: Optional[Tuple[float, float]],
                              lap_spec: Optional[LapSpec],
                              ) -> PushResult:
    """RP-1 / RP-2 / RP-3: snapshot the route, push it, open the ack window.

    start is the robot's (lat, lon) from state/pose (start_from_pose). With
    it the pushed polyline is the U127 patrol plan (route/route_plan.py
    plan_patrol: entry leg to the nearest route point, then the far end and
    back on an open route, once round on a closed one) -- without the entry
    leg a robot more than rns.route.max_deviation_m from the first point
    fails max_deviation on the first tick (20 S2.7). None (no fix yet)
    pushes the recorded route unchanged, as before 2026-11-05.

    lap_spec (99 U131) is what the task asked for in laps; RP-1 opens the
    progress run with loop_total = lap_spec.passes(loop_mode) (route/laps.py:
    a "来回" doubles the count on an open route). The other triggers find the
    run already open and pass None; RP-1 with None is a caller bug and is
    raised, not defaulted to one lap -- a wrong loop_total makes the progress
    sink end the patrol on the wrong pass.

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
        return PushResult(False, 0, "", "rp4_pushes_nothing", "plan_refused")
    try:
        row, anchors = await load_route_for_task(geo_conn, route_geo_id)
    except LookupError as exc:
        return PushResult(False, 0, "", "route_missing: %s" % exc,
                          "target_missing")
    _gid, name, waypoint_ids, path_points, loop_mode, direction, \
        total_len_m, rev = row
    try:
        snap = build_snapshot(
            task_id=task_id, route_id=route_geo_id, rev=int(rev),
            loop_mode=loop_mode, path_points=path_points,
            waypoint_ids=waypoint_ids, anchor_lookup=anchors,
            recorded_arrive_radius_m=recorded_arrive_radius_m,
            total_len_m=total_len_m)
        if start is not None:  # U127: plan the entry leg and the pass
            snap = build_composite_snapshot(
                task_id=task_id, route_id=snap.route_id, rev=snap.rev,
                points=plan_patrol(
                    start, [(p.lat, p.lon) for p in snap.points],
                    closed=(snap.loop_mode == "closed"),
                    # the return leg is dropped when the near side is within
                    # TWO arrival radii -- P1's own "at the end" tolerance
                    # (A-RT-2a: remaining arc <= 2r). One radius was too
                    # tight: the next pass is planned from the pose P3 holds
                    # when P1 reports arrived, already ~r from the end and a
                    # tick stale, so a pass of "来回巡逻N次" became a full
                    # out-and-back (bench 2026-11-06, 99 U147 nav audit).
                    near_skip_m=2.0 * snap.points[-1].arrive_radius_m),
                # the strictest per-point radius: P1 judges arrival at the
                # last point only, and which recorded point ends the plan
                # depends on where the robot entered
                arrive_radius_m=min(p.arrive_radius_m for p in snap.points))
        frames = build_route_frames(
            route_id=snap.route_id, route_rev=snap.rev,
            loop_mode=snap.loop_mode, total_len_m=snap.total_len_m,
            points=snap.points, cmd_id=new_cmd_id())
    except (SnapshotBuildError, RoutePushError) as exc:
        # Named, not swallowed: "the patrol did not start" with no reason is
        # the report this whole batch exists to stop producing.
        _logger.error("p3 route push refused for task %s (%s): %s",
                      task_id, route_geo_id, exc)
        return PushResult(False, 0, "", "refused: %s" % exc, "plan_refused")

    now_iso = iso_from_wall_ms(now_wall_ms)
    # SN-1: snapshot first, and COMMITTED before the first frame leaves.
    await snapshot_dao.replace(snap, now_iso=now_iso)
    # The progress run is opened in the same transaction as the snapshot: a
    # snapshot with no progress row would leave the breakpoint columns with
    # nowhere to land, and EX-3's writer would report "no active row" for the
    # whole patrol.
    if trigger is RoutePushTrigger.RP1_DISPATCH:
        if lap_spec is None:
            raise ValueError("RP-1 push for %s needs the task's lap_spec"
                             % (task_id,))
        await progress_dao.start_run(
            task_id=task_id, route_name=name, route_geo_id=snap.route_id,
            route_rev=snap.rev, direction=direction or "forward",
            loop_mode=snap.loop_mode, waypoint_total=snap.point_count,
            route_total_m=snap.total_len_m,
            # 99 U131: the pass count, from the RECORDED route's loop_mode
            # (the composite snapshot above is a one-way polyline whatever
            # the route is). Until 2026-11-05 this was a fixed 1 and every
            # patrol ran one pass.
            loop_total=lap_spec.passes(loop_mode), now_iso=now_iso)
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
    return PushResult(True, len(frames), frames[0]["cmd_id"], "", "")


#: The route_id a goto's single-point geometry carries (11 S3.5A needs one; a
#: goto has no routes row). Prefix + the waypoint geo_id, so path_progress and
#: the snapshot name the target the operator asked for.
GOTO_ROUTE_PREFIX = "goto-"


class GotoTargetError(LookupError):
    """The goto's destination cannot be resolved to one point to push. The
    message names what is missing -- 'goto failed' alone sends the reader to
    the wrong process."""


def start_from_pose(pose: Optional[Dict[str, Any]]) -> Optional[Tuple[float, float]]:
    """(lat, lon) from the cached state/pose body, or None when there is no
    usable position (absent, null, non-numeric). Never a guessed point."""
    if not isinstance(pose, dict):
        return None
    lat, lon = pose.get("lat"), pose.get("lon")
    for v in (lat, lon):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return None
    return float(lat), float(lon)


def _radius_override(params: Dict[str, Any]) -> Optional[float]:
    """params.arrive_radius_m (15 S9.5A.1 goto row: optional, overrides the
    point's own value), or None when absent. Present but not a positive
    finite number raises GotoTargetError -- a bad override is never ignored
    in favour of the waypoint's value."""
    v = params.get("arrive_radius_m")
    if v is None:
        return None
    if isinstance(v, bool) or not isinstance(v, (int, float)) \
            or not math.isfinite(v) or v <= 0.0:
        raise GotoTargetError("params.arrive_radius_m must be a positive "
                              "number, got %r" % (v,))
    return float(v)


def _coordinate(params: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    """(lat, lon) of a coordinate goto: params.lat + params.lon, or
    params.target{lat, lon} (the forms task_command accepts, 15 S2.4)."""
    src = params.get("target") if isinstance(params.get("target"), dict) else params
    lat, lon = src.get("lat"), src.get("lon")
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           for v in (lat, lon)):
        return None
    return float(lat), float(lon)


async def load_goto_target(geo_conn, params: Dict[str, Any], task_id: str):
    """Resolve a goto's destination to (route_id, rev, lat, lon, radius, name).

    * params.waypoint_id: the waypoint row (15 S9.3.1 G-2); its own
      arrival_radius unless params.arrive_radius_m overrides it (15 S9.5A.1).
    * params.lat/lon or params.target (HMI W2): accepted ONLY with an
      explicit params.arrive_radius_m. Without one there is no radius source
      -- no config key exists for it -- so it is REFUSED rather than given an
      invented radius (CLAUDE.md 3.1). route_id is "goto-" + task_id, rev 0.
    Raises GotoTargetError.
    """
    if not isinstance(params, dict):
        raise GotoTargetError("goto carries no params")
    override = _radius_override(params)
    wid = params.get("waypoint_id")
    if isinstance(wid, str) and wid.strip():
        cur = await geo_conn.execute(
            "SELECT rtk_lat, rtk_lon, arrival_radius, rev, name FROM waypoints"
            " WHERE geo_id=? AND tombstone=0", (wid.strip(),))
        row = await cur.fetchone()
        if row is None:
            raise GotoTargetError("waypoint %r not found or tombstoned" % wid)
        lat, lon, radius, rev, name = row
        if override is not None:
            radius = override
        if radius is None:
            raise GotoTargetError("waypoint %r has no arrival_radius" % wid)
        return (GOTO_ROUTE_PREFIX + wid.strip(), int(rev or 0), float(lat),
                float(lon), float(radius), name or wid)
    point = _coordinate(params)
    if point is not None:
        if override is None:
            raise GotoTargetError(
                "coordinate goto (params.lat/lon or params.target) carries no "
                "params.arrive_radius_m and no config key supplies one; an "
                "invented radius would be indistinguishable from a calibrated value")
        return (GOTO_ROUTE_PREFIX + task_id, 0, point[0], point[1], override,
                "%.7f,%.7f" % point)
    raise GotoTargetError("goto carries no waypoint_id or coordinate to resolve")


#: One resolved stop of a cloud keypoint plan: (geo_id, lat, lon, radius, name, rev).
Keypoint = Tuple[str, float, float, float, str, int]


def is_keypoint_plan(params: Any) -> bool:
    """True for the cloud GOTO_KEYPOINT plan (99 U164): params.waypoints is a
    non-empty list. The gateway passes v2.0 S2.1 waypoints[] through
    unchanged (p5 inbound/task_router._goto); no other producer sends it."""
    return (isinstance(params, dict) and isinstance(params.get("waypoints"), list)
            and bool(params["waypoints"]))


async def load_keypoint_targets(geo_conn, params: Dict[str, Any]) -> List[Keypoint]:
    """The cloud plan's waypoints, in order, resolved against geo.db.

    The coordinate is the waypoint ROW's (the manifest's authority, v2.0
    S4.4), not the latitude / longitude the plan echoes; the radius is the
    plan's own arrival_radius_m (v2.0 S2.1: required, 0.5..10, "不得在缺失时
    静默补默认值" -- the gateway refused a missing or out-of-range one, so a
    missing one here is a caller bug and raises). Raises GotoTargetError
    naming the item, so a waypoint deleted after admission fails the task
    with the index the operator can find."""
    out: List[Keypoint] = []
    for i, item in enumerate(params.get("waypoints") or []):
        wid = item.get("id") if isinstance(item, dict) else None
        if not isinstance(wid, str) or not wid.strip():
            raise GotoTargetError("waypoints[%d] carries no id" % i)
        radius = item.get("arrival_radius_m")
        if isinstance(radius, bool) or not isinstance(radius, (int, float)) \
                or not math.isfinite(radius) or radius <= 0.0:
            raise GotoTargetError("waypoints[%d] arrival_radius_m must be a positive "
                                  "number, got %r" % (i, radius))
        cur = await geo_conn.execute(
            "SELECT rtk_lat, rtk_lon, rev, name FROM waypoints"
            " WHERE geo_id=? AND tombstone=0", (wid.strip(),))
        row = await cur.fetchone()
        if row is None:
            raise GotoTargetError("waypoints[%d] %r not found or tombstoned" % (i, wid))
        out.append((wid.strip(), float(row[0]), float(row[1]), float(radius),
                    row[3] or wid, int(row[2] or 0)))
    return out


def plan_keypoints(start: Tuple[float, float], targets: List[Keypoint],
                   routes: List[RouteGeom],
                   membership_m: Optional[float]) -> Tuple[List[Tuple[float, float]], bool]:
    """99 U164 (user ruling 2026-11-11): the waypoints strung into ONE line in
    order -- robot -> 1st -> 2nd ... -> last, each leg planned by U127 over
    `routes` (the plan's recorded_path_id only), straight when membership_m
    is None. Returns (points, followed_a_route). Intermediate waypoints are
    passed through, not stopped at: P1 judges arrival at the last point only
    (build_composite_snapshot)."""
    pts: List[Tuple[float, float]] = [start]
    cur, followed = start, False
    for t in targets:
        target = (t[1], t[2])
        if membership_m is not None:
            leg = plan_goto(cur, target, routes, membership_m)
            followed = followed or leg.route_id is not None
            pts.extend(leg.points[1:])  # leg.points[0] is cur, already in
        else:
            pts.append(target)
        cur = target
    return pts, followed


async def push_goto_for_task(*, task_id: str, params: Dict[str, Any],
                             start: Optional[Tuple[float, float]],
                             task_conn, geo_conn, snapshot_dao, progress_dao,
                             route_pub, ack_window,
                             trigger: RoutePushTrigger,
                             now_mono_ms: int, now_wall_ms: int,
                             route_membership_m: Optional[float]) -> PushResult:
    """A goto task's push, through the same SN-1 snapshot, frame builder and
    RA-1 window as a route.

    99 U127: with route_membership_m set, the live routes are loaded and the
    goto follows the cheapest route the target belongs to (route_plan.
    plan_goto); with no member route, or when the robot is closer to the
    target than to that route, it is ONE point (11 S3.5A v2.0, 12 S4.2c.3)
    as before. None (tests that predate U127) keeps the single point.

    99 U164: params.waypoints[] (the cloud GOTO_KEYPOINT plan) is strung into
    one line in order (plan_keypoints) along params.via_route_id only, and
    arrives at the last waypoint by ITS arrival_radius_m. A re-push (RA retry,
    resume) re-plans the whole line from where the robot is then.

    Returns PushResult(pushed=False, reason) -- never raises -- for an
    unresolvable target, no current position, or a robot already at the
    target; the caller fails the task with that reason (15 S2.4.2).
    """
    if trigger is RoutePushTrigger.RP4_TASK_TERMINAL:
        return PushResult(False, 0, "", "rp4_pushes_nothing", "plan_refused")
    keypoints = is_keypoint_plan(params)
    try:
        if keypoints:
            # 99 U164: the cloud GOTO_KEYPOINT plan. Until 2026-11-11 it was
            # dispatched as a PATROL of recorded_path_id (route_geo_id set ->
            # push_route_for_task): the whole path driven, waypoints[] and
            # arrival_radius_m stored and never read.
            targets = await load_keypoint_targets(geo_conn, params)
            last = targets[-1]
            route_id = GOTO_ROUTE_PREFIX + (last[0] if len(targets) == 1 else task_id)
            rev = last[5] if len(targets) == 1 else 0
            lat, lon, radius = last[1], last[2], last[3]
            name = " -> ".join(t[4] for t in targets)
        else:
            route_id, rev, lat, lon, radius, name = await load_goto_target(
                geo_conn, params, task_id)
    except GotoTargetError as exc:
        return PushResult(False, 0, "", "goto_target: %s" % exc, "target_missing")
    if start is None:
        # P1 cannot drive to a point without knowing where it is either; the
        # snapshot needs the leg length. Refused, not pushed with a guess.
        return PushResult(False, 0, "", "goto_target: no current position "
                                        "(state/pose has no lat/lon)", "no_position")
    plan = None  # U127 route-following plan, when one applies
    chain = None  # U164 keypoint line, when it is more than one straight point
    if keypoints:
        routes = await load_route_geoms(geo_conn)
        via = params.get("via_route_id")
        if via:
            # user ruling 2026-11-11: follow recorded_path_id ONLY, never the
            # cheapest of all routes (that is the voice B01 rule, U127)
            routes = [r for r in routes if r.geo_id == via]
            if not routes:
                return PushResult(False, 0, "", "goto_target: route %r not found, "
                                  "tombstoned or not active" % via, "target_missing")
        pts, followed = plan_keypoints(start, targets, routes, route_membership_m)
        if len(targets) > 1 or followed:
            chain = pts
    elif route_membership_m is not None:  # route-aware goto configured
        plan = plan_goto(start, (lat, lon), await load_route_geoms(geo_conn),
                         route_membership_m)
        if plan.route_id is None:  # straight is the plan: the single point below
            plan = None
    try:
        if chain is not None:  # the keypoint line: robot, legs, last waypoint
            snap = build_composite_snapshot(
                task_id=task_id, route_id=route_id, rev=rev,
                points=chain, arrive_radius_m=radius)
            _logger.info("p3 goto %s keypoint plan %s via %s (%d stops)",
                         task_id, name, params.get("via_route_id"), len(targets))
        elif plan is not None:  # follow the route: robot, entry, route leg, exit, target
            snap = build_composite_snapshot(
                task_id=task_id, route_id=route_id, rev=rev,
                points=plan.points, arrive_radius_m=radius)
            _logger.info("p3 goto %s follows route %s (%.1f m planned)",
                         task_id, plan.route_id, plan.cost_m)
        else:
            snap = build_goto_snapshot(
                task_id=task_id, route_id=route_id, rev=rev, target_lat=lat,
                target_lon=lon, arrive_radius_m=radius, start_lat=start[0],
                start_lon=start[1])
        frames = build_route_frames(
            route_id=snap.route_id, route_rev=snap.rev,
            loop_mode=snap.loop_mode, total_len_m=snap.total_len_m,
            points=snap.points, cmd_id=new_cmd_id())
    except (SnapshotBuildError, RoutePushError) as exc:
        _logger.error("p3 goto push refused for task %s (%s): %s",
                      task_id, route_id, exc)
        return PushResult(False, 0, "", "refused: %s" % exc, "plan_refused")
    now_iso = iso_from_wall_ms(now_wall_ms)
    # SN-1, same order as a route: snapshot (and progress run) committed
    # before the first frame leaves.
    await snapshot_dao.replace(snap, now_iso=now_iso)
    if trigger is RoutePushTrigger.RP1_DISPATCH:
        await progress_dao.start_run(
            task_id=task_id, route_name=name, route_geo_id=snap.route_id,
            route_rev=snap.rev, direction="forward",
            loop_mode=snap.loop_mode, waypoint_total=snap.point_count,
            route_total_m=snap.total_len_m, loop_total=1, now_iso=now_iso)
    await task_conn.commit()
    for frame in frames:
        route_pub.put(json.dumps(frame, ensure_ascii=False).encode("utf-8"))
    ack_window.open(snap.route_id, snap.rev, now_mono_ms=now_mono_ms)
    _logger.info(
        "p3 %s goto push task=%s target=%s (%s) rev=%d leg=%.1fm radius=%.2fm "
        "cmd_id=%s", trigger.value, task_id, snap.route_id, name, snap.rev,
        snap.total_len_m, radius, frames[0]["cmd_id"])
    return PushResult(True, len(frames), frames[0]["cmd_id"], "", "")


async def load_route_geoms(geo_conn) -> list:
    """Every live route as a RouteGeom (99 U127 membership is computed
    against these, never stored). Mode B: routes.path_points; mode A: the
    anchors' coordinates in waypoint_ids order. A row that cannot be read
    is skipped with a warning -- one malformed route must not stop every
    goto."""
    cur = await geo_conn.execute(
        "SELECT geo_id, name, waypoint_ids, path_points, loop_mode FROM routes"
        " WHERE tombstone=0 AND state='active'")
    rows = list(await cur.fetchall())
    cur = await geo_conn.execute(
        "SELECT geo_id, rtk_lat, rtk_lon FROM waypoints WHERE tombstone=0")
    coords = {str(r[0]): (float(r[1]), float(r[2])) for r in await cur.fetchall()}
    out = []
    for gid, name, wids, pp, loop_mode in rows:
        try:
            if pp:  # mode B dense polyline
                pts = [(float(p[0]), float(p[1])) for p in json.loads(pp)]
            else:  # mode A named anchors
                pts = [coords[str(w)] for w in json.loads(wids or "[]")]
        except (ValueError, TypeError, KeyError, IndexError) as exc:
            _logger.warning("p3 route %s skipped for goto planning: %s", gid, exc)
            continue
        if pts:
            out.append(RouteGeom(geo_id=str(gid), name=str(name), points=tuple(pts),
                                 closed=(loop_mode == "closed")))
    return out


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
