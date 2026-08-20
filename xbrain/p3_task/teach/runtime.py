"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: runtime.py
Brief: The live cmd/teach session -- command handling, sampling, commit (11 S12A)

Description:
Everything stateful about recording. It owns at most one session (S12A.3 arming
check 1 makes that a global invariant), applies the S12A.6 rule to the pose
stream, and turns a save into a real geo object through the existing
commit_route / commit_fence writers.

Three design points worth reading before changing anything here:

*** 1. Sampled points are written to task.db's memory table, not to geo.db.
S12A.6.1 gives four reasons and the second is the one that bites: appending
directly to `routes` would make a half-recorded path appear in "which routes are
there", and a scheduler could dispatch a task onto it. The other three: the
buffer survives a crash mid-recording, the 1 Hz write stays off the database
P1/RNS have open read-only, and the geometry invariants only have to hold at
commit rather than at every intermediate point.

*** 2. The arming inputs come from live state caches, and a MISSING cache is a
refusal, not a pass. Recording suppresses lateral avoidance and voice e-stop
(S12A.3), so "I could not determine whether an e-stop is reachable" has to mean
no. The consequence is visible and deliberate: on a stack where nothing
publishes health/summary, state/robot, state/power or state/teleop, arming is
refused and the ack names which source was missing. That is the honest state of
the system -- those publishers do not exist yet -- and it is reported rather
than defaulted away. There is deliberately NO switch to bypass it (CLAUDE.md
3.6: a "skip the safety assertions" flag is a remote disable for every safety
constraint at once).

*** 3. Nothing here reads a clock. now_mono_s and now_ms are injected by the
wiring loop, which is also what makes the 1 Hz gate testable without sleeping.
"""
from __future__ import annotations

import json
import logging
import re
from typing import Any, Dict, List, Optional, Tuple

from xbrain.common.errors import (
    E_NAME_CONFLICT, E_TEACH_GEOMETRY, E_TEACH_QUALITY, E_TEACH_STATE,
)
from xbrain.p3_task.ingest.geo_commit import (
    GeoCommitError, commit_fence, commit_route, commit_waypoint,
)
from xbrain.p3_task.teach.command import (
    TeachCommand, TeachCommandError, parse_teach_command, teach_ack,
)
from xbrain.p3_task.teach.sampling import PoseSample, Recorder
from xbrain.p3_task.teach.session import (
    ArmingInputs, TeachSession, TeachStateError, check_arming, clamp_limits,
)
from xbrain.p3_task.teach.validate import (
    merge_degenerate, validate_fence, validate_route,
)

_logger = logging.getLogger("xbrain.p3.teach")

#: The memory-table key a buffered vertex lands under (S12A.6.1). The session id
#: is embedded so the cleanup contract ("delete_by_task on finish / cancel /
#: abort, and sweep orphans at startup") is one prefix delete.
_BUF_PREFIX = "teach:"

#: How stale a pose may be and still be used, in seconds of MONOTONIC time. A
#: recording that keeps sampling a frozen pose would lay every remaining vertex
#: on the last known position -- the route then looks complete and is wrong.
POSE_MAX_AGE_S = 2.0

#: S12A.8: a dock capture requires the robot to have been still for this long.
#: The handover point has to be precise, and a dock recorded while drifting is
#: a dock the robot will keep missing.
DOCK_STILL_REQUIRED_S = 1.0
DOCK_STILL_SPEED_MPS = 0.1


class TeachRuntime:
    """One recording session at a time, plus the caches its gates need."""

    def __init__(self, task_conn, geo_conn, fence_conn, *,
                 boot_id: str) -> None:
        self._task_conn = task_conn
        self._geo_conn = geo_conn
        self._fence_conn = fence_conn
        self._boot_id = boot_id
        self._session: Optional[TeachSession] = None
        self._recorder: Optional[Recorder] = None
        # cmd_id -> ack, for the S12A.4 duplicate rule. In memory on purpose:
        # the session it refers to is in memory too, so a table would outlive
        # the only thing that gives the replayed answer meaning.
        self._acks: Dict[str, Dict[str, Any]] = {}
        self._seq = 0
        # Live state caches. None means "never received", which every gate
        # treats as a refusal -- see design point 2.
        self._pose: Optional[Dict[str, Any]] = None
        self._pose_mono_s: float = -1e9
        self._health: Optional[Dict[str, Any]] = None
        self._robot: Optional[Dict[str, Any]] = None
        self._power: Optional[Dict[str, Any]] = None
        self._teleop: Optional[Dict[str, Any]] = None
        self._last_moving_mono_s: float = -1e9

    # ------------------------------------------------------------ caches --

    def update_pose(self, data: Dict[str, Any], now_mono_s: float) -> None:
        """Latest state/pose. Called from the loop thread, never from the Zenoh
        callback (CLAUDE.md 4.2)."""
        self._pose = data
        self._pose_mono_s = now_mono_s
        speed = data.get("speed_mps")
        if not isinstance(speed, (int, float)) or abs(speed) >= DOCK_STILL_SPEED_MPS:
            # Unknown speed counts as MOVING: a dock captured on a pose that
            # never reported speed would be accepted with no stillness evidence
            # at all, which is the assertion-that-cannot-fail shape.
            self._last_moving_mono_s = now_mono_s

    def update_health(self, data: Dict[str, Any]) -> None:
        self._health = data

    def update_robot(self, data: Dict[str, Any]) -> None:
        self._robot = data

    def update_power(self, data: Dict[str, Any]) -> None:
        self._power = data

    def update_teleop(self, data: Dict[str, Any]) -> None:
        self._teleop = data

    # ------------------------------------------------------------- gates --

    def _pose_fresh(self, now_mono_s: float) -> Optional[Dict[str, Any]]:
        if self._pose is None:
            return None
        if now_mono_s - self._pose_mono_s > POSE_MAX_AGE_S:
            return None
        return self._pose

    def missing_sources(self) -> List[str]:
        """Which state caches have never been filled. Reported in the refusal so
        the operator is told what to start, instead of "unhealthy"."""
        missing = []
        for name, value in (("state/pose", self._pose),
                            ("health/summary", self._health),
                            ("state/robot", self._robot),
                            ("state/power", self._power)):
            if value is None:
                missing.append(name)
        return missing

    def _arming_inputs(self, now_mono_s: float,
                       require_fix: str) -> ArmingInputs:
        pose = self._pose_fresh(now_mono_s) or {}
        health = self._health or {}
        robot = self._robot or {}
        power = self._power or {}
        teleop = self._teleop or {}
        sources = teleop.get("sources") or []
        # Criterion 1 of check 7: a gamepad/keyboard source that is alive. Its
        # dedicated e-stop key is the fallback the whole check is about.
        nonvoice = any(
            isinstance(s, dict) and s.get("alive")
            and s.get("device") in ("gamepad", "keyboard")
            for s in sources)
        # Criterion 2: state/robot reachable and its e-stop path not down.
        estop_ok = bool(robot) and robot.get("estop_path") not in (None, "down")
        return ArmingInputs(
            has_active_session=self._session is not None
            and self._session.state not in ("idle", "closed"),
            running_task_types=(),        # filled by the caller via with_tasks
            fix_type=pose.get("fix_type"),
            allow_motion=bool(health.get("allow_motion")),
            hes_engaged=bool(robot.get("hes")),
            soc_pct=power.get("soc_pct"),
            nonvoice_estop_source=nonvoice,
            estop_path_ok=estop_ok,
            teleop_driver_online=bool(sources),
            require_fix=require_fix)

    # ---------------------------------------------------------- sampling --

    async def offer_pose(self, now_mono_s: float, now_ms: int) -> bool:
        """Offer the cached pose to an active recording. True if a point landed.

        Called every loop pass; the 1 Hz gate lives in the Recorder, so the loop
        does not need its own timer and the rate is testable without sleeping.
        """
        if self._session is None or not self._session.is_sampling():
            return False
        pose = self._pose_fresh(now_mono_s)
        if pose is None or pose.get("lat") is None or pose.get("lon") is None:
            return False
        sample = PoseSample(lat=float(pose["lat"]), lon=float(pose["lon"]),
                            mono_s=now_mono_s, fix_type=pose.get("fix_type"),
                            alt=pose.get("alt"),
                            heading_rad=pose.get("heading_rad"))
        kept, _reason = self._recorder.offer(sample)
        self._sync_stats()
        if kept:
            await self._persist_point(sample, now_ms)
        if self._recorder.point_count >= self._recorder.max_points:
            # S12A.6: reaching max_points auto-finishes with a warn rather than
            # silently dropping every later point.
            if "max_points" not in self._session.warn:
                self._session.warn.append("max_points")
        return kept

    async def _persist_point(self, sample: PoseSample, now_ms: int) -> None:
        """One buffered vertex into task.db memory (S12A.6.1)."""
        self._seq += 1
        key = "%s%s:%05d" % (_BUF_PREFIX, self._session.session_id, self._seq)
        body = json.dumps({"lat": sample.lat, "lon": sample.lon,
                           "alt": sample.alt, "heading_rad": sample.heading_rad,
                           "manual": sample.manual,
                           "kind": ("fence_vertex" if self._session.kind == "fence"
                                    else "route_vertex")},
                          ensure_ascii=False).encode("utf-8")
        await self._task_conn.execute(
            "INSERT INTO memory (key, value, updated_ms) VALUES (?, ?, ?) "
            "ON CONFLICT(key) DO UPDATE SET value=excluded.value, "
            " updated_ms=excluded.updated_ms", (key, body, now_ms))
        await self._task_conn.commit()

    async def _clear_buffer(self, session_id: str) -> None:
        """The S12A.6.1 cleanup contract: drop the buffer on finish / discard.
        Left behind, the vertices of every past session accumulate in task.db
        and the startup orphan sweep has to guess which belong to what."""
        await self._task_conn.execute(
            "DELETE FROM memory WHERE key LIKE ?",
            ("%s%s:%%" % (_BUF_PREFIX, session_id),))
        await self._task_conn.commit()

    def _sync_stats(self) -> None:
        if self._session is None or self._recorder is None:
            return
        self._session.point_count = self._recorder.point_count
        self._session.manual_count = self._recorder.manual_count
        self._session.dropped_by_quality = self._recorder.dropped_by_quality
        self._session.length_m = self._recorder.length_m

    # ------------------------------------------------------------ command --

    async def handle(self, payload: Dict[str, Any], *, now_mono_s: float,
                     now_ms: int,
                     running_task_types: Tuple[str, ...] = ()) -> Dict[str, Any]:
        """One cmd/teach payload -> the ack body. Never raises."""
        raw_id = payload.get("cmd_id") if isinstance(payload, dict) else None
        cmd_id = raw_id if isinstance(raw_id, str) else ""
        if cmd_id and cmd_id in self._acks:
            # S12A.4 duplicate: replay the first outcome, do not re-apply.
            replay = dict(self._acks[cmd_id])
            replay["result"] = "duplicate"
            return replay
        try:
            cmd = parse_teach_command(payload)
        except TeachCommandError as exc:
            return teach_ack(cmd_id, "rejected", exc.code, {"reason": str(exc)})
        try:
            ack = await self._dispatch(cmd, now_mono_s, now_ms,
                                       running_task_types)
        except (TeachStateError, TeachCommandError) as exc:
            ack = teach_ack(cmd.cmd_id, "rejected", exc.code,
                            exc.detail if exc.detail is not None
                            else {"reason": str(exc)})
        except GeoCommitError as exc:
            ack = teach_ack(cmd.cmd_id, "rejected", E_TEACH_GEOMETRY,
                            {"reason": str(exc)})
        except Exception as exc:              # noqa: BLE001
            _logger.error("p3 teach %s failed: %s", cmd.action, exc)
            ack = teach_ack(cmd.cmd_id, "rejected", E_TEACH_STATE,
                            {"reason": str(exc)})
        if cmd_id:
            self._acks[cmd_id] = ack
        return ack

    async def _dispatch(self, cmd: TeachCommand, now_mono_s: float,
                        now_ms: int,
                        running_task_types: Tuple[str, ...]) -> Dict[str, Any]:
        if cmd.action == "start":
            return await self._start(cmd, now_mono_s, running_task_types)
        if cmd.action == "mark_once":
            return await self._mark_once(cmd, now_mono_s, now_ms)
        if cmd.action == "query":
            return teach_ack(cmd.cmd_id, "accepted", "OK",
                             self.teach_state_payload(now_mono_s)["session"])
        session = self._require_session(cmd)
        if cmd.action == "mark":
            return await self._mark(cmd, session, now_mono_s, now_ms)
        if cmd.action == "undo":
            session.apply("undo")
            removed = self._recorder.undo(cmd.undo_count)
            self._sync_stats()
            await self._clear_buffer(session.session_id)
            for point in self._recorder.points:
                await self._persist_point(point, now_ms)
            return teach_ack(cmd.cmd_id, "accepted", "OK",
                             {"session_id": session.session_id,
                              "removed": removed,
                              "point_count": self._recorder.point_count})
        if cmd.action in ("pause", "resume"):
            session.apply(cmd.action)
            return teach_ack(cmd.cmd_id, "accepted", "OK",
                             {"session_id": session.session_id,
                              "state": session.state})
        if cmd.action == "finish":
            return self._finish(cmd, session)
        if cmd.action == "save":
            return await self._save(cmd, session, now_ms)
        if cmd.action == "discard":
            session.apply("discard")
            session.close_reason = cmd.reason or "operator_cancel"
            await self._clear_buffer(session.session_id)
            detail = {"session_id": session.session_id, "state": "closed",
                      "close_reason": session.close_reason,
                      "discarded_points": self._recorder.point_count}
            self._session = None
            self._recorder = None
            return teach_ack(cmd.cmd_id, "accepted", "OK", detail)
        if cmd.action == "takeover":
            # S12A.11 O-4: re-attach an orphaned session to a new issuer. The
            # orphan DETECTION (issuer heartbeat) is not wired, so this only
            # re-labels the issuer -- said plainly rather than pretended.
            session.issuer_src = cmd.issuer_src
            session.issuer_channel = cmd.issuer_channel
            return teach_ack(cmd.cmd_id, "accepted", "OK",
                             {"session_id": session.session_id,
                              "issuer": cmd.issuer_src})
        raise TeachStateError(E_TEACH_STATE,
                              f"action {cmd.action!r} has no handler")

    def _require_session(self, cmd: TeachCommand) -> TeachSession:
        """The session this command names, or E_TEACH_STATE.

        A mismatched session_id is refused rather than applied to whatever is
        current -- that is the whole point of carrying the id (S12A.4).
        """
        if self._session is None:
            raise TeachStateError(E_TEACH_STATE, "no recording session is open")
        if cmd.session_id and cmd.session_id != self._session.session_id:
            raise TeachStateError(
                E_TEACH_STATE,
                f"session {cmd.session_id!r} is not the open one",
                {"open_session": self._session.session_id})
        return self._session

    async def _start(self, cmd: TeachCommand, now_mono_s: float,
                     running_task_types: Tuple[str, ...]) -> Dict[str, Any]:
        missing = self.missing_sources()
        if missing:
            # Design point 2: an undeterminable gate is a refusal that names
            # what it could not read.
            return teach_ack(
                cmd.cmd_id, "rejected", E_TEACH_QUALITY,
                {"reason": "state_unavailable", "missing": missing})
        inputs = self._arming_inputs(now_mono_s, cmd.start["require_fix"])
        inputs = ArmingInputs(**{**inputs.__dict__,
                                 "running_task_types": running_task_types})
        result = check_arming(inputs)
        if not result.ok:
            return teach_ack(cmd.cmd_id, "rejected", result.code,
                             result.detail or {"reason": result.reason})
        limits, applied = clamp_limits(cmd.start["sample"],
                                       cmd.start["max_duration_s"])
        limits.require_fix = cmd.start["require_fix"]
        session_id = "ts-%s-%04d" % (self._boot_id, len(self._acks) + 1)
        self._session = TeachSession(
            session_id=session_id, kind=cmd.start["kind"], state="arming",
            name_hint=cmd.start["name_hint"] or "",
            issuer_src=cmd.issuer_src, issuer_channel=cmd.issuer_channel,
            boot_id=self._boot_id, started_mono_s=now_mono_s,
            deadline_mono_s=now_mono_s + limits.max_duration_s,
            limits=limits, warn=list(result.warn))
        # arming resolves immediately once the checks passed: the contract puts
        # it at "typically < 200 ms" and there is nothing left to wait for.
        self._session.state = "recording"
        self._recorder = Recorder(dedup_min_dist_m=limits.dedup_min_dist_m,
                                  sample_hz=limits.sample_hz,
                                  require_fix=limits.require_fix,
                                  max_points=limits.max_points)
        self._seq = 0
        return teach_ack(cmd.cmd_id, "accepted", "OK",
                         {"session_id": session_id, "state": "recording",
                          "point_count": 0, "sample_applied": applied,
                          "warn": list(self._session.warn)})

    async def _mark(self, cmd: TeachCommand, session: TeachSession,
                    now_mono_s: float, now_ms: int) -> Dict[str, Any]:
        """F05: force a point in, exempt from the interval and distance gates."""
        session.apply("mark")
        pose = self._pose_fresh(now_mono_s)
        if pose is None or pose.get("lat") is None:
            raise TeachStateError(E_TEACH_QUALITY, "no fresh pose to mark")
        sample = PoseSample(lat=float(pose["lat"]), lon=float(pose["lon"]),
                            mono_s=now_mono_s, fix_type=pose.get("fix_type"),
                            alt=pose.get("alt"),
                            heading_rad=pose.get("heading_rad"), manual=True)
        kept, reason = self._recorder.offer(sample)
        self._sync_stats()
        if not kept:
            raise TeachStateError(E_TEACH_QUALITY,
                                  f"mark refused: {reason}")
        await self._persist_point(sample, now_ms)
        return teach_ack(cmd.cmd_id, "accepted", "OK",
                         {"session_id": session.session_id,
                          "point_count": self._recorder.point_count,
                          "manual_count": self._recorder.manual_count})

    def _finish(self, cmd: TeachCommand,
                session: TeachSession) -> Dict[str, Any]:
        """Stop sampling and validate (S12A.7). The buffer is KEPT: finish is
        not a discard, and the operator still has to name it."""
        session.apply("finish")
        validation = self._validate(session, activate=False)
        return teach_ack(cmd.cmd_id, "accepted", "OK",
                         {"session_id": session.session_id,
                          "state": session.state,
                          "point_count": self._recorder.point_count,
                          "length_m": round(self._recorder.length_m, 1),
                          "validation": {"ok": validation.ok,
                                         "issues": validation.issues}})

    def _validate(self, session: TeachSession, *, activate: bool):
        points = self._recorder.latlon_points()
        if session.kind == "fence":
            robot_at = None
            pose = self._pose or {}
            if pose.get("lat") is not None and pose.get("lon") is not None:
                robot_at = (float(pose["lat"]), float(pose["lon"]))
            return validate_fence(
                points, dropped_by_quality=self._recorder.dropped_by_quality,
                robot_at=robot_at, activate=activate)
        return validate_route(
            points, dropped_by_quality=self._recorder.dropped_by_quality)

    async def _save(self, cmd: TeachCommand, session: TeachSession,
                    now_ms: int) -> Dict[str, Any]:
        """S12A.7: validate AGAIN (the object library may have changed since
        finish), then commit through the existing writers."""
        activate = bool(cmd.save["activate"]) and session.kind == "fence"
        validation = self._validate(session, activate=activate)
        if not validation.ok:
            raise TeachStateError(
                E_TEACH_GEOMETRY, "geometry validation failed",
                {"session_id": session.session_id,
                 "issues": validation.issues})
        name = cmd.save["name"]
        points = merge_degenerate(self._recorder.latlon_points())
        if session.kind == "route":
            geo_id = await self._mint_id(self._geo_conn, "routes", "r-", name)
            await self._ensure_name_free(self._geo_conn, "routes", name,
                                         cmd.save["overwrite"])
            await commit_route(self._geo_conn, route_id=geo_id, name=name,
                               path_points=points, now_ms=now_ms,
                               state="active", created_by="teach")
            total_len = self._recorder.length_m
        else:
            geo_id = await self._mint_id(self._fence_conn, "fences", "f-", name)
            await self._ensure_name_free(self._fence_conn, "fences", name,
                                         cmd.save["overwrite"])
            # A recorded fence is a forbid zone by default: the operator drove
            # around something to exclude it. An allow (keep-in) fence is the
            # camp boundary and is set deliberately through cmd/geo, where the
            # S9A.1A "exactly one allow" invariant is easier to reason about.
            # state='draft': saving is not enabling (S12A.7 constraint 1).
            await commit_fence(self._fence_conn, fence_id=geo_id, role="forbid",
                               points=points, now_ms=now_ms, name=name,
                               state="draft", created_by="teach")
            total_len = self._recorder.length_m
        session.apply("save")
        session.close_reason = "saved"
        await self._clear_buffer(session.session_id)
        detail = {"session_id": session.session_id, "state": "closed",
                  "applied": {"geo_id": geo_id, "name": name, "rev": 1,
                              "point_count": len(points),
                              "total_len_m": round(total_len, 1)},
                  "validation": {"ok": True, "issues": validation.issues},
                  # S12A.7 constraint 1, echoed so the caller cannot mistake a
                  # save for an activation: a recorded fence is stored as a
                  # DRAFT and F15 is what puts it in force.
                  "activated": False}
        self._session = None
        self._recorder = None
        return teach_ack(cmd.cmd_id, "accepted", "OK", detail)

    async def _ensure_name_free(self, conn, table: str, name: str,
                                overwrite: bool) -> None:
        """E_NAME_CONFLICT unless overwrite was requested (and confirmed
        upstream at L2). The name column is UNIQUE, so without this the sqlite
        IntegrityError would surface as an internal failure instead of the
        answer the operator can act on."""
        cur = await conn.execute(
            f"SELECT 1 FROM {table} WHERE name=? AND tombstone=0", (name,))
        if await cur.fetchone() is not None and not overwrite:
            raise TeachStateError(E_NAME_CONFLICT,
                                  f"the name {name!r} is already in use",
                                  {"name": name})

    async def _mint_id(self, conn, table: str, prefix: str,
                       name: str) -> str:
        """A geo_id of the S9.3 ID-2 form: prefix + slug.

        The slug is taken from the ASCII of the name when there is any, and
        falls back to a sequence number -- operator names are usually Chinese,
        which has no ASCII slug, and a transliteration table would be a second
        source of truth for what an object is called.
        """
        slug = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
        if not slug:
            cur = await conn.execute(f"SELECT COUNT(*) FROM {table}")
            slug = "%03d" % ((await cur.fetchone())[0] + 1)
        candidate = prefix + slug
        pk = "fence_id" if table == "fences" else "geo_id"
        n = 1
        while True:
            cur = await conn.execute(
                f"SELECT 1 FROM {table} WHERE {pk}=?", (candidate,))
            if await cur.fetchone() is None:
                return candidate
            n += 1
            candidate = "%s%s_%d" % (prefix, slug, n)

    async def _mark_once(self, cmd: TeachCommand, now_mono_s: float,
                         now_ms: int) -> Dict[str, Any]:
        """S12A.8 F06 / F10: capture one point. No session, no state change."""
        pose = self._pose_fresh(now_mono_s)
        if pose is None or pose.get("lat") is None:
            raise TeachStateError(E_TEACH_QUALITY, "no fresh pose")
        if pose.get("fix_type") != "rtk_fixed":
            raise TeachStateError(
                E_TEACH_QUALITY,
                f"mark_once needs rtk_fixed, have {pose.get('fix_type')!r}")
        kind = cmd.mark_once["kind"]
        name = cmd.mark_once["name"]
        if kind == "dock":
            # S12A.8 adds two requirements a waypoint does not have, and both
            # are about precision of the handover pose rather than of the fix.
            if pose.get("heading_level") != 1:
                raise TeachStateError(
                    E_TEACH_QUALITY,
                    "dock capture needs L1 (dual-antenna) heading")
            still_for = now_mono_s - self._last_moving_mono_s
            if still_for < DOCK_STILL_REQUIRED_S:
                raise TeachStateError(
                    E_TEACH_QUALITY,
                    f"dock capture needs {DOCK_STILL_REQUIRED_S}s stationary, "
                    f"had {max(0.0, still_for):.1f}s")
            raise TeachStateError(
                E_TEACH_STATE,
                "dock capture is not wired: the charging subsystem has no "
                "handover-point model in this build")
        await self._ensure_name_free(self._geo_conn, "waypoints", name,
                                     cmd.mark_once["overwrite"])
        geo_id = await self._mint_id(self._geo_conn, "waypoints", "w-", name)
        yaw = None
        if cmd.mark_once["capture_heading"]:
            heading = pose.get("heading_rad")
            if isinstance(heading, (int, float)) and pose.get("heading_valid"):
                import math
                yaw = math.degrees(float(heading))
            else:
                # U34: with heading degraded, the point is still worth keeping
                # -- it just carries no orientation, and the caller is told.
                yaw = None
        await commit_waypoint(self._geo_conn, geo_id=geo_id, name=name,
                              wtype="poi", rtk_lat=float(pose["lat"]),
                              rtk_lon=float(pose["lon"]),
                              rtk_alt=pose.get("alt"), yaw_deg=yaw,
                              now_ms=now_ms, state="active",
                              created_by="teach")
        return teach_ack(cmd.cmd_id, "accepted", "OK",
                         {"applied": {"geo_id": geo_id, "name": name,
                                      "kind": kind, "yaw_deg": yaw},
                          "warn": [] if yaw is not None else ["no_heading"]})

    # ---------------------------------------------------------- broadcast --

    def teach_state_payload(self, now_mono_s: float) -> Dict[str, Any]:
        """The S12A.5 TeachState body. With no session it carries only the
        state, exactly as the contract specifies -- an idle payload full of
        null stats reads like a session that lost its data."""
        if self._session is None:
            return {"schema": "teach_state_v1", "session": {"state": "idle"}}
        s = self._session
        last = self._recorder.points[-1] if self._recorder.points else None
        return {
            "schema": "teach_state_v1",
            "session": {"session_id": s.session_id, "state": s.state,
                        "kind": s.kind, "name_hint": s.name_hint,
                        "issuer": {"src": s.issuer_src,
                                   "channel": s.issuer_channel},
                        "orphan": False, "recovered": s.recovered,
                        "started_mono": s.started_mono_s,
                        "boot_id": s.boot_id,
                        "elapsed_s": round(now_mono_s - s.started_mono_s, 1),
                        "deadline_mono": s.deadline_mono_s},
            "stats": {"point_count": s.point_count,
                      "manual_count": s.manual_count,
                      "length_m": round(s.length_m, 1),
                      "dropped_by_quality": s.dropped_by_quality,
                      # S12A.5: constant false since v0.7 -- rns_avoid emits no
                      # detour candidates while recording. The field stays (its
                      # removal is a structural change) but nothing may derive
                      # "N metres were recorded around an obstacle" from it.
                      "rns_intervened": False,
                      "last_point": None if last is None else {
                          "seq": s.point_count, "lat": last.lat,
                          "lon": last.lon, "alt": last.alt,
                          "heading_rad": last.heading_rad,
                          "quality": last.fix_type, "manual": last.manual,
                          "flags": ["manual"] if last.manual else []}},
            "control": {"driver": self._driver_name(), "teleop_active": False,
                        "teleop_age_ms": None},
            "warn": list(s.warn),
            "limits": {"max_duration_s": s.limits.max_duration_s,
                       "max_points": s.limits.max_points,
                       "finalize_timeout_s": s.limits.finalize_timeout_s},
        }

    def _driver_name(self) -> str:
        for src in (self._teleop or {}).get("sources") or []:
            if isinstance(src, dict) and src.get("alive"):
                return str(src.get("device"))
        return "none"

    def expire(self, now_mono_s: float) -> Optional[str]:
        """Auto-finish a session past its deadline (S12A.3). Returns the reason
        when it fired. Called every loop pass; the deadline is monotonic, so a
        wall-clock step at RTK lock cannot end a recording early."""
        if self._session is None or not self._session.is_sampling():
            return None
        if now_mono_s < self._session.deadline_mono_s:
            return None
        self._session.apply("finish")
        self._session.warn.append("max_duration")
        return "max_duration"
