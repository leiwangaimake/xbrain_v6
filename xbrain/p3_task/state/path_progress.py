"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: path_progress.py
Brief: state/motion/path_progress intake -- PP-1 flush decision + terminal mapping (EX-3 / EX-4)

Description:
P1-12 publishes state/motion/path_progress at 2 Hz plus one extra frame at
every waypoint (11 S3.5B). 15 S2.1 calls it "the ONLY source of the
breakpoint" -- waypoint_index / seg_done_m / dist_done_m / dir_sign -- and
15 S9.5 makes patrol_progress its landing place. Until 2026-09-28 nothing in
p3_task subscribed to it: `grep -rn path_progress xbrain/p3_task/` matched one
COMMENT in schedule/driver.py and nothing else, so U07a resume, the S7.3A
remap and every progress figure the HMI shows had no input at all
(docs/NEXT.md EX-3 / EX-4).

This module is the pure half: parse the frame, decide whether it must hit the
disk, and say what a terminal state means for the task. It performs no I/O and
holds no database handle -- the writing is PatrolProgressDAO's and the wiring's
(runtime/main_wiring.py), so every rule below is testable without a database.

*** A NAME COLLISION YOU MUST KNOW ABOUT BEFORE READING EITHER FILE.
There are two unrelated things called PP-1..PP-3b in this tree:
  * 15 S9.5B PP-1..PP-3b -- the patrol_progress FLUSH rules. That is what this
    module implements, and it is the numbering the contract actually defines.
  * xbrain/p3_task/state/progress.py PP-1..PP-3b -- a DIFFERENT set invented in
    that file ("step boundary crossed" / "suspend or resume" / "route push
    accepted"), attributed there to "15 S5". 15 S5 defines no PP-* codes;
    `grep -n 'PP-1' docs/15-*.md` returns only the S9.5B table and references
    to it. The collision is registered as a finding and is NOT fixed here
    (renaming touches that file's callers, which is a separate batch).
Read a PP-* code in p3 code as belonging to whichever of the two files it sits
in, never by number alone.

The three rules this file owns:

  PP-1 (15 S9.5B, quoting 11 S4.4.1 PG-2)  write to disk when the waypoint
       index CHANGED, or when progress_flush_s has elapsed since the last
       write -- WHICHEVER COMES FIRST. Not index alone: a single 300 m
       segment would then lose the whole segment on a power cut. Not the
       timer alone either: 2 Hz all day is ~170k pointless fsyncs.
  3 s staleness (11 S3.5B)  a progress frame older than 3 s makes
       TaskState.progress.persisted false. That bit is RUNTIME ONLY and is
       computed here in memory off the monotonic clock; 15 S9.5B says in as
       many words that it is not a column.
  terminal mapping (12 S4.3.1 LP-3)  P1 only ever says
       path_progress.state == "arrived"; "把它映射成任务 done 的是 P3".

What it does NOT do: it does not compute pct (PP-3 says pct is derived at read
time and never stored), does not decide the resume direction (S9.5B owns that
and it is fixed at first entry), and does not push geometry (route/push.py).

Every interval here is CLOCK_MONOTONIC in milliseconds (CLK-C1 / DBF-3). The
frame's own `mono` field is P1's clock, not ours: two processes' monotonic
clocks share an epoch only because both are CLOCK_MONOTONIC on the same
machine, and 11 S3.5B still says timeouts use `mono` rather than `ts`. The
receive-side ages below are measured on OUR reads so a frame that sat in a
queue is aged from when we saw it, not from when P1 stamped it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

#: 11 S3.5B `state` closed set. An off-set value RAISES (CLAUDE.md 3.5: no
#: silent pass-through, no "interpret the unknown value as the nearest known
#: one"). The two v2.0 additions matter: `failed` is an RNS navigation failure
#: and carries fail_reason, `aborted` is preemption or e-stop and does not.
PATH_STATES = ("idle", "route_loading", "running", "arrived", "aborted",
               "failed")

#: The states after which P1 will send nothing further for this mission.
TERMINAL_PATH_STATES = ("arrived", "aborted", "failed")

#: 11 S3.5B: "P3 侧 3 s 未更新 -> 进度视为不可信, TaskState.progress.persisted
#: = false". Seconds, held here because it is a contract constant rather than a
#: tuning knob.
PROGRESS_STALE_S = 3.0

#: 15 S12 `progress_flush_s`, the other half of PG-2's "whichever comes first".
#: The contract states the default in as many words ("缺省 5 s"). It is a
#: protocol cadence, NOT a common.spec.* / common.safety.* calibration value,
#: so CLAUDE.md 3.1's no-defaults rule does not reach it -- the same standing
#: as GEO_PUBLISH_PERIOD_S and TASK_STATE_PERIOD_S in runtime/main_wiring.py.
#: It is a module constant rather than a dataclass default so that a caller
#: who wants another value passes it explicitly and it shows up at the call
#: site.
PROGRESS_FLUSH_S = 5.0


class PathProgressError(ValueError):
    """Off-contract path_progress frame. The frame is dropped and counted;
    it never half-updates the tracker."""


@dataclass(frozen=True)
class PathProgress:
    """One 11 S3.5B frame, validated.

    Field names are the wire names verbatim (CFG-40 forbids an alias here, and
    15 S9.5 stores them under the same names) so a reader can diff this
    against the json5 block in 11 S3.5A/B without a translation table.
    """
    route_id: str
    route_rev: int
    loading: bool
    waypoint_index: int
    waypoint_total: int
    seg_done_m: float
    dist_done_m: float
    loop_index: int
    loop_total: int
    dir_sign: int
    state: str
    fail_reason: Optional[str]

    @property
    def is_terminal(self) -> bool:
        return self.state in TERMINAL_PATH_STATES


def _require(body: Dict[str, Any], key: str) -> Any:
    if key not in body:
        raise PathProgressError("path_progress missing required key %r" % key)
    return body[key]


def _as_int(body: Dict[str, Any], key: str, *, floor: int) -> int:
    v = _require(body, key)
    # bool is an int subclass in Python and json true/false decodes to bool;
    # letting it through would silently read `true` as waypoint_index 1.
    if isinstance(v, bool) or not isinstance(v, int):
        raise PathProgressError("%s must be an int, got %r" % (key, v))
    if v < floor:
        raise PathProgressError("%s must be >= %d, got %d" % (key, floor, v))
    return v


def _as_float(body: Dict[str, Any], key: str) -> float:
    v = _require(body, key)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise PathProgressError("%s must be a number, got %r" % (key, v))
    f = float(v)
    if f != f:                                   # NaN; f != f is the only test
        raise PathProgressError("%s must not be NaN" % key)
    return f


def parse_path_progress(body: Any) -> PathProgress:
    """Validate one frame body (envelope already unwrapped) against 11 S3.5B.

    Strict on purpose. A progress frame drives what P3 believes about where
    the robot is; a half-parsed one produces a breakpoint that resumes the
    patrol somewhere it never was, and there is no later check that would
    catch it.
    """
    if not isinstance(body, dict):
        raise PathProgressError("path_progress body must be an object, got %r"
                                % type(body).__name__)
    state = _require(body, "state")
    if state not in PATH_STATES:
        raise PathProgressError(
            "path_progress.state %r outside the 11 S3.5B closed set %s"
            % (state, list(PATH_STATES)))
    route_id = _require(body, "route_id")
    if not isinstance(route_id, str) or not route_id:
        raise PathProgressError("route_id must be a non-empty string, got %r"
                                % (route_id,))
    loading = _require(body, "loading")
    if not isinstance(loading, bool):
        raise PathProgressError("loading must be a bool, got %r" % (loading,))
    dir_sign = _require(body, "dir_sign")
    # 11 S3.5B calls dir_sign mandatory and notes the contract once omitted it,
    # leaving 15 S9.5's NOT NULL column with no source. Accepting a missing
    # one here would put that hole back.
    if dir_sign not in (1, -1) or isinstance(dir_sign, bool):
        raise PathProgressError("dir_sign must be +1 or -1, got %r"
                                % (dir_sign,))
    fail_reason = body.get("fail_reason")
    if state == "failed":
        # 11 S3.5B v2.0 (#20-1) verbatim: "state == \"failed\" 时必填". Without
        # it the whole point of splitting failed out of aborted is lost -- P3
        # would again be unable to say WHY the navigation died.
        if not isinstance(fail_reason, str) or not fail_reason:
            raise PathProgressError(
                "state 'failed' requires a non-empty fail_reason (11 S3.5B)")
    elif fail_reason is not None:
        raise PathProgressError(
            "fail_reason is only legal with state 'failed', got state %r"
            % (state,))
    waypoint_total = _as_int(body, "waypoint_total", floor=0)
    waypoint_index = _as_int(body, "waypoint_index", floor=-1)
    if waypoint_index >= waypoint_total:
        # Same bound as the 15 S9.5 CHECK. Catching it here names the frame
        # that was wrong; catching it at the UPDATE gives an IntegrityError
        # with no way back to the sender.
        raise PathProgressError(
            "waypoint_index %d is not < waypoint_total %d"
            % (waypoint_index, waypoint_total))
    return PathProgress(
        route_id=route_id,
        route_rev=_as_int(body, "route_rev", floor=0),
        loading=loading,
        waypoint_index=waypoint_index,
        waypoint_total=waypoint_total,
        seg_done_m=_as_float(body, "seg_done_m"),
        dist_done_m=_as_float(body, "dist_done_m"),
        loop_index=_as_int(body, "loop_index", floor=0),
        loop_total=_as_int(body, "loop_total", floor=0),
        dir_sign=int(dir_sign),
        state=state,
        fail_reason=fail_reason if state == "failed" else None)


# 12 S4.3.1 LP-3 verbatim: "P1 只发 path_progress.state='arrived', 把它映射成
# 任务 done 的是 P3". This table is that mapping, expressed in the vocabulary
# schedule/driver.apply_motion_result already speaks.
#
# *** READ THIS BEFORE EDITING: the word "aborted" appears on BOTH sides of
# this table with DIFFERENT meanings, and they are not each other.
#   * the KEY 'aborted' is a path_progress state (11 S3.5B): preempted or
#     e-stopped.
#   * the VALUE 'aborted' is a relative_move status (11 S3.5, the closed set
#     apply_motion_result takes): the motion did not complete.
# They are deliberately NOT paired here -- see the omission below.
#
# Why path_progress 'aborted' maps to NOTHING. 11 S3.5B's own note on adding
# `failed` says why the two exist separately: before it, "state 只有
# aborted(=被抢占/estop), 没有'这条导航自己栽了'的位, p3 无从与抢占区分".
# Preemption and e-stop are transitions P3 (15 S7.2) and ES-2 have ALREADY
# applied to the task -- it is sitting at `suspended`, resumable through U07.
# Failing it here would overwrite that suspend with a terminal state and
# destroy the breakpoint, i.e. it would undo the thing the distinction was
# added to make possible. So an 'aborted' frame moves no task.
#
# NOT in the contract as an explicit P3-side table: 15 states what P3 does for
# `arrived` (via 12 LP-3) but has no row for `aborted` / `failed`. The two
# entries below are read off 11 S3.5B's stated PURPOSE for the split, and the
# omission is the conservative direction (it leaves the task where P3's own
# logic put it). Flagged for confirmation.
MOTION_RESULT_FOR_PATH_STATE = {
    "arrived": "succeeded",
    "failed": "aborted",
}


@dataclass(frozen=True)
class FlushDecision:
    """Why (or why not) this frame must reach the disk."""
    flush: bool
    reason: str          # 'waypoint' | 'period' | 'memory_only' | 'first'


class ProgressTracker:
    """Per-task in-memory progress state and the PP-1 decision over it.

    One instance per P3 process. It is keyed by task_id even though 15 S6.1
    allows only one running task at a time, so that a late frame arriving
    after a handover cannot be mistaken for the new task's progress.

    All times are monotonic milliseconds supplied by the caller: this class
    reads no clock, which is what lets the period rule be tested without
    sleeping (and keeps CLK-C1 a property of one call site instead of many).
    """

    def __init__(self, *, flush_period_s: float = PROGRESS_FLUSH_S,
                 stale_after_s: float = PROGRESS_STALE_S) -> None:
        if flush_period_s <= 0.0 or stale_after_s <= 0.0:
            # A zero period would make every 2 Hz frame an fsync; a zero
            # staleness would mark every frame untrustworthy the instant it
            # landed. Both look like "extra safety" and are neither.
            raise ValueError(
                "flush_period_s and stale_after_s must be positive, got "
                "%r / %r" % (flush_period_s, stale_after_s))
        self._flush_period_ms = int(flush_period_s * 1000.0)
        self._stale_after_ms = int(stale_after_s * 1000.0)
        #: task_id -> (last flushed waypoint_index, last flush mono ms)
        self._flushed: Dict[str, Tuple[int, int]] = {}
        #: task_id -> mono ms of the last frame accepted
        self._seen: Dict[str, int] = {}

    def decide(self, task_id: str, progress: PathProgress, *,
               now_mono_ms: int) -> FlushDecision:
        """PP-1. Record the frame as seen and say whether it must be written.

        Terminal frames always flush: they are the last one, so "wait for the
        period" would mean never.
        """
        self._seen[task_id] = now_mono_ms
        prev = self._flushed.get(task_id)
        if prev is None:
            self._flushed[task_id] = (progress.waypoint_index, now_mono_ms)
            return FlushDecision(True, "first")
        prev_index, prev_ms = prev
        if progress.waypoint_index != prev_index:
            self._flushed[task_id] = (progress.waypoint_index, now_mono_ms)
            return FlushDecision(True, "waypoint")
        if progress.is_terminal:
            self._flushed[task_id] = (progress.waypoint_index, now_mono_ms)
            return FlushDecision(True, "waypoint")
        if now_mono_ms - prev_ms >= self._flush_period_ms:
            self._flushed[task_id] = (progress.waypoint_index, now_mono_ms)
            return FlushDecision(True, "period")
        # PP-2: seg_done_m advancing inside a segment is NOT its own write.
        # The accepted worst-case loss is one waypoint (U38 / PWR-4).
        return FlushDecision(False, "memory_only")

    def is_stale(self, task_id: str, *, now_mono_ms: int) -> bool:
        """11 S3.5B: no frame for 3 s -> progress.persisted is false. A task
        never seen is stale, not fresh -- 'we have heard nothing' and 'all is
        well' must not be the same answer."""
        last = self._seen.get(task_id)
        if last is None:
            return True
        return now_mono_ms - last >= self._stale_after_ms

    def forget(self, task_id: str) -> None:
        """Drop a finished task. Without this the two dicts grow for the life
        of the process -- slowly, which is the kind of leak that surfaces as
        an unexplained restart after a fortnight of patrols."""
        self._flushed.pop(task_id, None)
        self._seen.pop(task_id, None)
