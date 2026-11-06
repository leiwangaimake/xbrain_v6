"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: push.py
Brief: cmd/motion/route (11 S3.5A RouteGeometry) frames, chunking and the 15 S2.4.4 ack window

Description:
15 S2.4 is the third P3 -> P1 path and the one patrol cannot run without:
P1 never opens geo.db (11 S3.5A rules that out -- 12 has no SQLite anywhere
and RTC-4 forbids blocking I/O in the loop), so the geometry has to be PUSHED.
This module builds those frames and owns the confirmation window; the sending
and the database are the runtime's (runtime/route_push_runtime.py).

*** THIS FILE WAS REWRITTEN ON 2026-09-28. The previous version compiled and
had zero callers, and it did not implement this protocol. It disagreed with
the contract on every field that matters, in ways a reader would not spot
because the RP-/RA- code names looked right:

  * it built a `RouteChunk(task_id, route_seq, chunk_ix, total_chunks,
    waypoints=((x, y, heading), ...))`. 11 S3.5A's RouteGeometry has none of
    those names: it is {v, op, cmd_id, route_id, route_rev, loop_mode,
    total_len_m, frame, points[{lat,lon,seq,arrive_radius_m}], chunk{index,
    total}}. Not one field lined up, and P1's parser
    (nav/route_intake.py) would have raised RouteIntakeError on the first
    frame -- "participant is up, nothing gets through", 13 DDS-9's failure
    mode.
  * CHUNK_SIZE was 32. 15 S2.4.2: split above 1000 points (or 256 KB,
    whichever comes first). At 32 a 2400-point route becomes 75 frames
    instead of the 3 that 15 TC-36 asserts.
  * RP-2/RP-3/RP-4 meant different things ("resume", "geo change",
    "suspend-cancel required to restart"). 15 S2.4.1: RP-2 is the re-push
    after a route revision and remap, RP-3 is the P1 RESTART re-push
    (path_progress.state back to "idle" while a task is running), and RP-4 is
    a task reaching a terminal P1 reported -- where the rule is DO NOTHING,
    neither push nor revoke, because the geometry is meant to stay resident
    in P1. (An operator cancel / pause is not RP-4: it sends op=clear,
    route_clear_needed below.)
  * classify_ack() parsed an "ack code" off the wire. THERE IS NO ACK KEY.
    15 S2.4.4 opens with that sentence precisely so an implementer does not
    go looking for one: 11 S2.2.4 registers neither cmd/motion/route/ack nor
    cmd/motion/behavior/ack. Confirmation is the three-criteria read of
    state/motion/path_progress below.

So the codes RP-1..4 / RA-1..3 here are the ones 15 S2.4 defines, and they
are NOT the same objects the old file called by those names.

What this module does NOT do: it does not build the snapshot (route/
snapshot_build.py), does not touch a database or a session, and does not
send BehaviorCommand -- see runtime/route_push_runtime.py for why that last
one is currently not sent at all.
"""

from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Tuple

# CLAUDE.md 3.5: an E_* value is imported from the shared exporter, never
# spelled as a literal -- a hand-typed code drifts in case or prefix and the
# mismatch only shows up in integration.
from xbrain.common.errors import E_GEO_TOO_LARGE

#: 15 S2.4.2 / 11 S3.5A `chunk` row: split above 1000 points. NOT 32 -- see
#: the header. 15 TC-36 pins the consequence: 2400 points must become 3
#: frames, and it fails at any other value.
CHUNK_POINTS = 1000

#: 15 S2.4.2 "另有 256 KB 上限, 二者取先". A dense recorded polyline can carry
#: long floats, so the point count alone does not bound the frame size.
CHUNK_BYTES = 256 * 1024

#: 11 S3.5A RG-2 / 14.2. Above this the task is refused at the precondition
#: (E_GEO_TOO_LARGE) and NOTHING is pushed -- P1 preallocates and refuses
#: rather than growing.
MAX_POINTS = 5000

#: 11 S3.5A: the only frame value. P1 projects to site metres itself
#: (nav/route_intake.py), so P3 must not pre-project.
FRAME_WGS84 = "wgs84"

#: 15 S12. The contract states both defaults in as many words ("缺省 2",
#: route_ack_timeout_s "3.0"). Protocol cadences, not common.spec.* /
#: common.safety.* calibration, so CLAUDE.md 3.1's no-defaults rule does not
#: reach them -- same standing as the other period constants in p3's wiring.
#: They are module constants rather than dataclass defaults so a caller who
#: overrides one does it visibly at the call site.
ROUTE_PUSH_RETRY = 2
ROUTE_ACK_TIMEOUT_S = 3.0


class RoutePushTrigger(str, Enum):
    """15 S2.4.1. The value is the code so a log line and an event carry the
    same token the contract uses."""

    #: task ready -> running (patrol / goto): push the whole route.
    RP1_DISPATCH = "RP-1"
    #: route revised, S7.3A remap done, task back to running: push the new rev.
    RP2_REMAP = "RP-2"
    #: P1 restarted (path_progress.state == "idle" while a task is running):
    #: re-push the current snapshot. Idempotent by (route_id, route_rev).
    RP3_P1_RESTART = "RP-3"
    #: task reached a terminal that P1 itself reported (arrived / failed).
    #: PUSH NOTHING AND REVOKE NOTHING -- the geometry stays resident in P1,
    #: which is what 15 S1.3 T-1 depends on (P1 finishes the lap if P3 dies).
    #: A terminal P3 or the operator causes (cancel, an unconfirmed push) is
    #: NOT this row: it withdraws the mission, see route_clear_needed (15
    #: S2.4.1 RP-4 narrowed 2026-11-06).
    RP4_TASK_TERMINAL = "RP-4"
    #: 99 U131: one pass of a multi-pass patrol arrived and passes remain --
    #: push the route again, re-planned from where the robot is (P1 is
    #: single-pass; the progress run stays open and only loop_index moves).
    RP5_NEXT_PASS = "RP-5"


class RouteAck(str, Enum):
    """15 S2.4.4 outcomes of the confirmation WINDOW. Not wire values: there
    is no ack key, so nothing ever carries these strings on a topic."""

    RA1_CONFIRMED = "RA-1"      # the three criteria all held inside the window
    RA2_REPORTED_INCOMPLETE = "RA-2"   # event/warn/motion carried E_GEO_INCOMPLETE
    RA3_TIMEOUT = "RA-3"        # window elapsed with the criteria unmet


class RoutePushError(ValueError):
    """The route cannot be turned into a legal RouteGeometry. Raised rather
    than trimmed: a silently shortened route is a robot driving a path nobody
    approved."""


@dataclass(frozen=True)
class RoutePoint:
    """One 11 S3.5A point. arrive_radius_m is PER POINT (the gate and the
    alley do not share an arrival test, 15 S9.3.1 G-2) -- there is no global
    fallback in this module."""
    lat: float
    lon: float
    arrive_radius_m: float


def _chunk_points(points: Sequence[RoutePoint]) -> List[List[Tuple[int, RoutePoint]]]:
    """Split into frames by 15 S2.4.2: above CHUNK_POINTS, or above
    CHUNK_BYTES, whichever comes first. Returns lists of (seq, point) so the
    wire `seq` stays the index in the WHOLE route -- P1 checks that seq runs
    0..n-1 across the assembled set (route_intake.py), so a per-chunk restart
    would look to it like a lost chunk.
    """
    out: List[List[Tuple[int, RoutePoint]]] = []
    cur: List[Tuple[int, RoutePoint]] = []
    cur_bytes = 0
    for seq, p in enumerate(points):
        # Measured on the encoded point, not estimated: a lat/lon carries as
        # many digits as json.dumps gives it, and an estimate that is slightly
        # low produces a frame slightly over the cap -- which fails at the
        # transport, far from here.
        size = len(json.dumps(_point_body(seq, p), ensure_ascii=False)
                   .encode("utf-8")) + 1
        if cur and (len(cur) >= CHUNK_POINTS or cur_bytes + size > CHUNK_BYTES):
            out.append(cur)
            cur, cur_bytes = [], 0
        cur.append((seq, p))
        cur_bytes += size
    if cur:
        out.append(cur)
    return out


def _point_body(seq: int, p: RoutePoint) -> Dict[str, Any]:
    return {"lat": p.lat, "lon": p.lon, "seq": seq,
            "arrive_radius_m": p.arrive_radius_m}


def new_cmd_id() -> str:
    """15 S2.4.3: 'rg-' + uuid4()[:4]. One id for the WHOLE push -- P1
    assembles chunks by cmd_id and abandons a pending set when a different
    one arrives, so a per-chunk id would make every multi-chunk route an
    endless series of abandoned single-chunk sets."""
    return "rg-" + uuid.uuid4().hex[:4]


def build_route_frames(*, route_id: str, route_rev: int, loop_mode: str,
                       total_len_m: float, points: Sequence[RoutePoint],
                       cmd_id: Optional[str] = None) -> List[Dict[str, Any]]:
    """Build the ordered RouteGeometry frames for one push (11 S3.5A).

    Every frame carries the full envelope-level fields and its own chunk
    {index, total}; P1 swaps the mission pointer only once all `total`
    indices are in (RG-1/RG-3), so a partial push leaves the OLD geometry
    running rather than emptying it.
    """
    if not points:
        raise RoutePushError("refusing to push an empty route %r" % route_id)
    if len(points) > MAX_POINTS:
        # RG-2 / 15 S2.4.2: over the cap the task is failed at the
        # precondition with E_GEO_TOO_LARGE and nothing is sent. Trimming
        # here would push a route that stops short of where it was told to go.
        raise RoutePushError(
            "route %r has %d points, over the 11 S3.5A RG-2 cap of %d; the "
            "task must fail with %s and push nothing"
            % (route_id, len(points), MAX_POINTS, E_GEO_TOO_LARGE))
    if total_len_m <= 0.0:
        # It is S7.3A's T0. A zero divides in the remap's alpha; a null means
        # the route was committed without its length and the remap cannot run
        # at all (11 S7.12.3 step 1).
        raise RoutePushError(
            "route %r has total_len_m %r; it is the remap's T0 and must be "
            "positive (11 S7.12.3 step 1)" % (route_id, total_len_m))
    cid = cmd_id or new_cmd_id()
    chunks = _chunk_points(points)
    total = len(chunks)
    return [
        {"v": 1,
         "op": "set",
         "cmd_id": cid,
         "route_id": route_id,
         "route_rev": route_rev,
         "loop_mode": loop_mode,
         "total_len_m": total_len_m,
         "frame": FRAME_WGS84,
         "points": [_point_body(seq, p) for seq, p in chunk],
         "chunk": {"index": i, "total": total}}
        for i, chunk in enumerate(chunks)
    ]


#: The states a RUNNING task can leave to on which P1 must be told to stop
#: (15 S7.2 step 1, 12 S4.2c.4). `done` is not here: P1 itself reported
#: the arrival (12 S4.3.1 LP-3), so it is already idle and a clear would only
#: preempt whatever comes next (a relative_move, the next task's push).
#: `failed` IS here even though P1 may have reported it, because P3 also
#: fails tasks on its own -- an unconfirmed push (RA fatal), a refused plan --
#: and in those cases P1 may hold, or later finish loading, the geometry of a
#: task that no longer exists; the callers that relay a P1 terminal report
#: simply do not pass the stop callback (runtime/main_wiring.py).
ROUTE_CLEAR_ON = frozenset({"suspended", "cancelled", "failed"})


def route_clear_needed(from_state: str, to_state: str) -> bool:
    """True when this task transition must withdraw P1's navigation mission.

    Why this exists (2026-11-06 audit, measured on the robot): a voice or HMI
    cancel / pause only rewrote tasks.state. Nothing told P1, so RNS kept
    following the route -- 20 s after "取消任务" P1 still reported nav_state
    follow / route_state running, and outdoors (perception up) the robot
    drives on to the old target. 15 S7.2 step 1 named
    cmd/motion/behavior=hold for this, but 11 S2.2 struck p3_task from that
    key's publishers on 2026-10-08 (99 U90) and P1 refuses a behavior frame
    whose src is not p2_core; the stop P3 owns is cmd/motion/route op=clear
    (11 S3.5A v2.0, 12 S4.2c.4).
    """
    # Only a task that WAS running has a mission in P1; a queued task being
    # cancelled must never touch the robot (it may be running another task).
    return from_state == "running" and to_state in ROUTE_CLEAR_ON


def build_route_clear(cmd_id: Optional[str] = None) -> Dict[str, Any]:
    """One cmd/motion/route op=clear frame (11 S3.5A v2.0).

    P1 answers it by cancelling the RNS mission: back to IDLE, audited as
    cancelled, NO failure report (20 S9.0.3, nav/mission_host.py on_route).
    route_id / route_rev / chunk are omitted on purpose: the contract makes
    them optional for clear and P1 cancels whatever mission it holds, so
    naming one would only invite a mismatch check nobody implements. points
    is the empty array the contract allows for clear.
    """
    # Same id family as a push, so P1's log lines read as one protocol.
    return {"v": 1, "op": "clear", "cmd_id": cmd_id or new_cmd_id(),
            "points": []}


@dataclass(frozen=True)
class AckOutcome:
    """What the window concluded, and whether the caller may push again."""
    ack: RouteAck
    retry_allowed: bool
    attempts: int


class RouteAckWindow:
    """15 S2.4.4. cmd/motion/route has NO ack key, so a push is confirmed by
    reading state/motion/path_progress.

    RA-1 is the part that is easy to get wrong and the reason this is a class
    rather than a boolean: confirmation requires loading == false AND
    route_id == the pushed one AND route_rev == the pushed one, ALL THREE.
    `loading == false` on its own is also the steady state of the OLD
    geometry, so a push that never arrived at all reads as confirmed; P3 then
    sends a path_follow carrying the new route_rev and P1 answers
    E_GEO_CONFLICT. 15 TC-37a is exactly that regression.

    RA-2: the window uses CLOCK_MONOTONIC (DBF-3 / CLK-C1). No clock is read
    here -- the caller injects now_mono_ms, which is also what lets the
    timeout be tested without sleeping.

    RA-3: while the window is open NO motion command may be sent and the task
    stays at ready (first start) or suspended (resume). This class does not
    enforce that -- it cannot, it sends nothing -- but `is_open` is what the
    caller gates on.
    """

    def __init__(self, *, timeout_s: float = ROUTE_ACK_TIMEOUT_S,
                 max_retry: int = ROUTE_PUSH_RETRY) -> None:
        if timeout_s <= 0.0:
            raise ValueError("route ack timeout must be positive, got %r"
                             % (timeout_s,))
        if max_retry < 0:
            raise ValueError("route push retry must be >= 0, got %r"
                             % (max_retry,))
        self._timeout_ms = int(timeout_s * 1000.0)
        self._max_retry = max_retry
        self._route: Optional[Tuple[str, int]] = None
        self._opened_ms = 0
        self._attempts = 0
        # Monotonic ms of the latest push; kept after confirm / reset so the
        # progress intake can drop frames P1 produced before it (pushed_ms).
        self._pushed_ms = 0

    @property
    def is_open(self) -> bool:
        return self._route is not None

    @property
    def attempts(self) -> int:
        return self._attempts

    @property
    def pushed_ms(self) -> int:
        """Monotonic ms of the latest open() (0 before the first push).

        *** A path_progress frame carries route_id / route_rev but nothing
        that names the PUSH, and P3 re-pushes the same route at the same rev
        (patrol repeat, the next pass, resume, a second goto to one point).
        P1 keeps re-sending `arrived` for the finished run at 2 Hz until it
        loads the new one, so those frames matched the new run on id and rev
        and closed it at once -- "再巡一圈" ended before the robot moved, and
        the stale frame also satisfied RA-1 (found on the bench 2026-11-06,
        99 U147 nav audit). P1 stamps `mono` at the start of the tick that
        builds the frame and loads a new route inside the tick, so a frame of
        the old run always has mono < the push. Same CLOCK_MONOTONIC on one
        machine (state/path_progress.py header)."""
        return self._pushed_ms

    def open(self, route_id: str, route_rev: int, *, now_mono_ms: int) -> None:
        """Start the window after the LAST chunk went out (15 S2.4.4: the
        window opens at chunk.index == total-1, not at the first chunk -- a
        window that starts early can expire while frames are still being
        sent)."""
        self._route = (route_id, route_rev)
        self._opened_ms = now_mono_ms
        self._pushed_ms = now_mono_ms  # frames stamped before this are the old run's
        self._attempts += 1

    def observe(self, *, loading: bool, route_id: str, route_rev: int,
                ) -> Optional[AckOutcome]:
        """Feed one path_progress frame. Returns an outcome only on RA-1."""
        if self._route is None:
            return None
        want_id, want_rev = self._route
        # All three, in one expression, so no refactor can drop one of them
        # and leave a test that only exercises the happy path green.
        if not (loading is False and route_id == want_id
                and route_rev == want_rev):
            return None
        self._route = None
        return AckOutcome(RouteAck.RA1_CONFIRMED, False, self._attempts)

    def report_incomplete(self) -> Optional[AckOutcome]:
        """event/warn/motion carried E_GEO_INCOMPLETE: re-push at once rather
        than waiting out the window (15 S2.4.4). The whole route is re-sent,
        NOT the missing chunks -- 11 S3.5A discards the whole set on a gap, so
        P1 holds no partial copy to complete. (11 S13.10's generic "resend
        only detail.missing[]" is written for cause 1, cmd/geo; registered as
        Q-P3-26.)"""
        if self._route is None:
            return None
        self._route = None
        return AckOutcome(RouteAck.RA2_REPORTED_INCOMPLETE,
                          self._attempts <= self._max_retry, self._attempts)

    def tick(self, *, now_mono_ms: int) -> Optional[AckOutcome]:
        """Returns an outcome once the window has elapsed unconfirmed."""
        if self._route is None:
            return None
        if now_mono_ms - self._opened_ms < self._timeout_ms:
            return None
        self._route = None
        return AckOutcome(RouteAck.RA3_TIMEOUT,
                          self._attempts <= self._max_retry, self._attempts)

    def reset(self) -> None:
        self._route = None
        self._attempts = 0
