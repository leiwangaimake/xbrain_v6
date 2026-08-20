"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: session.py
Brief: Teach recording session machine + arming preconditions (11 S12A.3)

Description:
The recording session lives in P3 (11 S12A.2: P3 is the only writer of the geo
tables, so the buffer and the commit must be on the same side of the process
boundary). This module is the machine itself, pure: states, the legal
transitions between them, and the seven arming checks. It holds no connection,
reads no clock (monotonic values are passed in), and publishes nothing.

*** The arming checks are the safety-critical part of this file, and check 7 is
the one to understand before touching anything here.

Recording weakens the robot in three ways at once (11 S12A.3, quoting Q-U53-6):
lateral obstacle avoidance is suppressed (rns_avoid keeps only its longitudinal
slow/stop half), voice e-stop is suppressed for the duration (U45: A01 does not
apply while recording), and the operator is driving manually. What remains is
the dedicated e-stop key on a gamepad/keyboard, or the HMI/cloud e-stop path.

Check 7 refuses to arm when NEITHER of those is alive. It is a hard admission
gate and not a confirmation level, because a confirmation proves somebody
clicked, not that somebody can REACH an e-stop. The failure directions are
deliberately asymmetric: refusing wrongly means "cannot record until the gamepad
is plugged in", accepting wrongly means "nobody can stop it".

Note what check 6 does NOT do: it permits arming with no teleop driver online
(you may start the session and then pick up the controller). That relaxation
applies to the DRIVE channel only. Relaxing it for the e-stop channel would be
the same sentence with a completely different consequence.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from xbrain.common.enums import TEACH_STATE
from xbrain.common.errors import (
    E_BUSY, E_LOCKED, E_LOW_BATTERY, E_TEACH_BUSY, E_TEACH_QUALITY,
    E_TEACH_STATE, E_UNHEALTHY,
)

#: The S12A.3 transition graph: (state, action) -> next state. Exhaustive by
#: construction -- an action not listed for a state is E_TEACH_STATE, never a
#: silent no-op, because a dropped teach command reads to the operator as "it
#: did not hear me" while the session carries on recording.
TRANSITIONS: Dict[Tuple[str, str], str] = {
    ("idle", "start"): "arming",
    # arming resolves without an external action: pass -> recording, fail ->
    # closed. Both are driven by arm_result() below rather than by a command.
    ("recording", "mark"): "recording",       # self-loop, forced point
    ("recording", "undo"): "recording",       # self-loop, remove last point(s)
    ("recording", "pause"): "paused",
    ("paused", "resume"): "recording",
    ("recording", "finish"): "finalizing",
    ("paused", "finish"): "finalizing",
    ("recording", "discard"): "closed",
    ("paused", "discard"): "closed",
    ("finalizing", "save"): "closed",
    ("finalizing", "discard"): "closed",
}

#: Actions that are legal in ANY state because they do not change it: query
#: reads the session, takeover re-attaches an orphaned session to a new issuer.
_STATELESS_ACTIONS = frozenset({"query", "takeover"})

#: 11 S12A.6 defaults. sample_hz and dedup_min_dist_m are the U42 decision
#: (1 Hz timer + 0.5 m dedup, matching V5) -- NOT the retired v0.2 triple of
#: distance + heading + interval. Kept here beside the machine because the
#: session carries them per-session (start.sample may override).
DEFAULT_SAMPLE_HZ = 1.0
DEFAULT_DEDUP_MIN_DIST_M = 0.5
DEFAULT_MAX_POINTS = 2000
DEFAULT_MAX_DURATION_S = 1800          # CMD-17
MAX_DURATION_CAP_S = 7200
DEFAULT_REQUIRE_FIX = "rtk_fixed"
DEFAULT_FINALIZE_TIMEOUT_S = 600
#: Below this SoC arming is refused (S12A.6 teach_min_soc). This is a policy
#: threshold on a recording session, not a common.safety.* parameter, so a
#: default here is not the CLAUDE.md 3.1 case -- and its failure direction is
#: "cannot start recording", never motion.
DEFAULT_TEACH_MIN_SOC = 25.0


class TeachStateError(Exception):
    """An action is not legal in the current session state (11 S12A.3). Carries
    the closed-set code so the ack does not invent one."""

    def __init__(self, code: str, message: str,
                 detail: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(message)
        self.code = code
        self.detail = detail


@dataclass
class SessionLimits:
    """Per-session limits, clamped at start (S12A.4: an out-of-range value is
    clamped and REPORTED in the ack detail, never silently taken)."""
    max_duration_s: float = DEFAULT_MAX_DURATION_S
    max_points: int = DEFAULT_MAX_POINTS
    finalize_timeout_s: float = DEFAULT_FINALIZE_TIMEOUT_S
    sample_hz: float = DEFAULT_SAMPLE_HZ
    dedup_min_dist_m: float = DEFAULT_DEDUP_MIN_DIST_M
    require_fix: str = DEFAULT_REQUIRE_FIX


@dataclass
class TeachSession:
    """The live session. One at a time, globally (arming check 1)."""
    session_id: str
    kind: str                       # route | fence
    state: str = "idle"
    name_hint: str = ""
    issuer_src: str = ""
    issuer_channel: str = ""
    boot_id: str = ""
    started_mono_s: float = 0.0
    deadline_mono_s: float = 0.0
    limits: SessionLimits = field(default_factory=SessionLimits)
    point_count: int = 0
    manual_count: int = 0
    dropped_by_quality: int = 0
    length_m: float = 0.0
    warn: List[str] = field(default_factory=list)
    close_reason: str = ""
    recovered: bool = False

    def can(self, action: str) -> bool:
        """Is `action` legal right now?"""
        return (action in _STATELESS_ACTIONS
                or (self.state, action) in TRANSITIONS)

    def apply(self, action: str) -> str:
        """Move the machine, or raise E_TEACH_STATE. Returns the new state."""
        if action in _STATELESS_ACTIONS:
            return self.state
        nxt = TRANSITIONS.get((self.state, action))
        if nxt is None:
            raise TeachStateError(
                E_TEACH_STATE,
                f"action {action!r} is not legal in state {self.state!r}",
                {"session_id": self.session_id, "state": self.state})
        self.state = nxt
        return nxt

    def is_sampling(self) -> bool:
        """Only `recording` samples (S12A.3 state table). paused keeps the
        buffer intact but takes no points -- that difference is the whole
        purpose of pause, so it is asserted rather than assumed."""
        return self.state == "recording"


@dataclass(frozen=True)
class ArmingInputs:
    """Everything the seven checks need, gathered by the caller from the live
    state caches. Passed as one frozen object so a check cannot be evaluated
    against a value that changed halfway through the sequence."""
    has_active_session: bool
    running_task_types: Tuple[str, ...]      # running patrol/goto/charge
    fix_type: Optional[str]
    allow_motion: bool
    hes_engaged: bool
    soc_pct: Optional[float]
    #: Check 7, criterion 1: a gamepad/keyboard teleop source that is alive.
    nonvoice_estop_source: bool
    #: Check 7, criterion 2: state/robot reachable and estop_path not down.
    estop_path_ok: bool
    teleop_driver_online: bool = True
    teach_min_soc: float = DEFAULT_TEACH_MIN_SOC
    require_fix: str = DEFAULT_REQUIRE_FIX


@dataclass(frozen=True)
class ArmingResult:
    ok: bool
    code: str = "OK"
    reason: str = ""
    detail: Optional[Dict[str, Any]] = None
    warn: Tuple[str, ...] = ()


#: Tasks whose presence blocks arming (S12A.3 check 2). standby/teach/follow are
#: absent on purpose: the seven task types of 15 include ones that do not
#: drive the robot, and blocking on those would refuse a recording for no reason.
_BLOCKING_TASK_TYPES = frozenset({"patrol", "goto", "charge"})


def check_arming(inp: ArmingInputs) -> ArmingResult:
    """The seven S12A.3 preconditions, in table order. First failure wins.

    Order matters for the message the operator hears, not for correctness: the
    checks are independent, and reporting "no e-stop" when the real problem is
    "a patrol is running" would send them to fix the wrong thing.
    """
    warn: List[str] = []
    if inp.has_active_session:
        return ArmingResult(False, E_TEACH_BUSY,
                            "another recording session is already open")
    blocking = [t for t in inp.running_task_types if t in _BLOCKING_TASK_TYPES]
    if blocking:
        return ArmingResult(False, E_BUSY, "task_running",
                            {"reason": "task_running", "types": blocking})
    if inp.fix_type != inp.require_fix:
        # A route recorded at rtk_float carries decimetre error into every
        # future patrol that drives it; the check is on the SESSION, not just
        # per sample, so a session cannot start in a place with no fix at all.
        return ArmingResult(False, E_TEACH_QUALITY,
                            f"fix_type is {inp.fix_type!r}, "
                            f"need {inp.require_fix!r}",
                            {"fix_type": inp.fix_type})
    if not inp.allow_motion:
        return ArmingResult(False, E_UNHEALTHY, "health forbids motion")
    if inp.hes_engaged:
        return ArmingResult(False, E_LOCKED, "hardware e-stop is engaged")
    if inp.soc_pct is None or inp.soc_pct < inp.teach_min_soc:
        return ArmingResult(False, E_LOW_BATTERY,
                            f"soc {inp.soc_pct} below {inp.teach_min_soc}",
                            {"soc_pct": inp.soc_pct})
    if not inp.teleop_driver_online:
        # Check 6: permitted, with a standing warning. "Start the session then
        # pick up the controller" is a real workflow.
        warn.append("no_driver")
    if not (inp.nonvoice_estop_source or inp.estop_path_ok):
        # *** Check 7. See the module docstring. Both criteria are reported so
        # the operator is told WHICH channel to restore.
        return ArmingResult(
            False, E_UNHEALTHY, "no_nonvoice_estop",
            {"reason": "no_nonvoice_estop", "item": "estop_path",
             "checked": {"teleop_estop_source": inp.nonvoice_estop_source,
                         "estop_path_ok": inp.estop_path_ok}})
    return ArmingResult(True, "OK", "", None, tuple(warn))


def clamp_limits(raw: Optional[Dict[str, Any]],
                 max_duration_s: Optional[float]) -> Tuple[SessionLimits,
                                                           Dict[str, Any]]:
    """Build the per-session limits from start.sample, clamping out-of-range
    values and reporting what was applied (S12A.4: "越界值被钳到合法区间并在 ack
    的 detail 中回报").

    Clamping rather than rejecting is the contract's choice, and reporting is
    what keeps it honest: a silently clamped sample rate would have the operator
    believe they are recording at 5 Hz while the buffer fills at 1 Hz.
    """
    raw = raw or {}
    limits = SessionLimits()
    hz = raw.get("min_interval_ms")
    if isinstance(raw.get("sample_hz"), (int, float)):
        # 0.2 .. 5 Hz. Below 0.2 the route is a set of disconnected guesses;
        # above 5 the 0.5 m dedup discards nearly everything anyway.
        limits.sample_hz = min(5.0, max(0.2, float(raw["sample_hz"])))
    elif isinstance(hz, (int, float)) and hz > 0:
        # S12A.4 still spells the sample block with min_interval_ms (the v0.2
        # shape). Accepted and converted rather than ignored -- a sender using
        # the documented field name must not be silently overridden.
        limits.sample_hz = min(5.0, max(0.2, 1000.0 / float(hz)))
    if isinstance(raw.get("min_dist_m"), (int, float)):
        limits.dedup_min_dist_m = min(10.0, max(0.05,
                                                float(raw["min_dist_m"])))
    if isinstance(max_duration_s, (int, float)) and max_duration_s > 0:
        limits.max_duration_s = min(MAX_DURATION_CAP_S, float(max_duration_s))
    applied = {"sample_hz": limits.sample_hz,
               "dedup_min_dist_m": limits.dedup_min_dist_m,
               "max_duration_s": limits.max_duration_s,
               "max_points": limits.max_points}
    return limits, applied


def session_state_names() -> frozenset:
    """The closed set, for callers that validate a stored/received state."""
    return frozenset(TEACH_STATE.values)
