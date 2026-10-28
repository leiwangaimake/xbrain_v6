"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptz_controller.py
Brief: p2 PTZ domain controller -- PtzCommand jog / nudge / stop / refocus with dead-man, 17 Hz budget and acks

Description:
What problem this solves
------------------------
11 S7.4 defines the in-robot PtzCommand (jog / nudge / stop / refocus ...)
and the Qt contract docs/MISSON/PTZ控制契约_qt端v0.2草案.md maps the cloud's
keyboard control onto it (99 U98). Before this module p2 only understood the
voice intents (E01 / E06 ...), and drove them as BLOCKING pulses:
ContinuousMove, time.sleep(pulse) under the session lock, Stop. Two things
follow from that shape and both break the contract:
  * there is no open-ended jog, so "hold the arrow key, release to stop"
    cannot be expressed at all (the cloud would have to fake it with pulses,
    which keep moving after the key is released -- Qt contract R-2);
  * a running pulse holds the lock, so a Stop waits behind it: up to 2.8 s
    for a large nudge and 12 s for the voice "look around" orbit. 11 PTZ-C6
    says pan has no mechanical limit, so a Stop that cannot get in is the
    exact failure the dead-man exists to prevent.

What it does
------------
One controller owns every motion of the head (11 PTZ-C9: one shared head):
  * jog      -- open-ended motion, held alive by renewals; a renewal with the
                same velocity only pushes the deadline, it sends NOTHING to
                the camera (11 PTZ-D2); the deadline running out sends Stop
                and an event (PTZ-D1, Qt contract S6 PTZ_DEADMAN_STOP);
  * nudge    -- a bounded pulse, and the voice intents' pulse sequences
                ("legs"), run by the tick instead of by sleeping, so a Stop
                or a new command interrupts them at once (Qt contract S3.4);
  * stop     -- always executed, never coalesced, never refused for budget;
  * refocus  -- the one-shot LAPI refocus of 11 S7.4.4 / S7.4.7 F4, run on
                its own worker so it neither blocks motion nor is blocked by
                it; a second refocus while one runs is merged (coalesced).
The 17 Hz camera budget (11 S7.4.5) is enforced here: a velocity change that
would exceed it is merged last-wins into a pending velocity that the tick
applies as soon as the budget allows -- never queued (queueing = the head
keeps moving after release).

Domain 5 admission (11 S7A): the full arbiter does not run in this build, so
the controller does the part that matters for the head itself -- the holder
of the current motion and its priority. A command from a lower-priority
source while a higher one moves the head is refused with E_BUSY and the
holder named; an equal or higher one takes over, and when the source changes
the head is stopped first (11 PTZ-D4) and a PTZ_PREEMPTED event is emitted.
Source -> priority comes from the resolved config (arbiter.domains.ptz.
sources); the PtzCommand source names map onto those ids (see SOURCE_IDS).

What it does NOT do (boundaries)
--------------------------------
  * No camera I/O of its own: ONVIF and LAPI calls are injected (PtzOps /
    FocusOps), so every rule here is testable without a camera.
  * No positions, no angles, no closed loops (11 PTZ-C1..C3): deleted fields
    are refused with E_CAPABILITY, never translated.
  * No Zenoh: acks and events go out through injected callables; the wiring
    (runtime/ptz_wiring.py) owns keys and threads.
  * goto_preset / set_track / patrol are not implemented in this build and
    answer E_NOT_IMPLEMENTED -- never a fake success on the head.

Pitfalls this shape avoids (each one is a real bug class in this repo)
---------------------------------------------------------------------
  * Re-sending ContinuousMove on every renewal: 4 Hz of renewals would eat a
    quarter of the 17 Hz budget for nothing and make the head stutter.
  * Treating a renewal with the same cmd_id as valid: the p5 / HMI layer must
    send a NEW cmd_id per renewal (11 PTZ-D3); this controller does not
    deduplicate by cmd_id at all, so a duplicate id cannot swallow a renewal.
  * Handing the head to a new holder while the old ContinuousMove still runs:
    without position read-back nobody could notice (PTZ-D4) -- hence Stop
    first on every source change.
"""
from __future__ import annotations  # postponed annotations (forward refs in type hints)

import logging  # module logger, English messages only (CLAUDE.md 2.1)
import math  # isfinite: NaN / inf velocities are refused, not clamped
import threading  # one lock for state, one thread per refocus
from dataclasses import dataclass, field  # limits are frozen; the motion record is mutable
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple  # type hints only

from xbrain.common.errors import (  # closed error set, never string literals (CLAUDE.md 3.5)
    E_BUSY,  # domain 5 held by a higher priority source
    E_CAPABILITY,  # deleted field / action (no absolute positioning)
    E_NOT_IMPLEMENTED,  # action exists in 11 but is not driven in this build
    E_SCHEMA,  # malformed command / unknown enum value
    E_UNHEALTHY,  # camera or LAPI failure (11 S13.9 item ptz)
)

_logger = logging.getLogger("xbrain.p2.ptz_controller")  # one logger name per module, grep-able in journald

#: Lower bound of hold_ms, fixed by the contract itself (11 S7.4.4 clamp table
#: "hold_ms [200, ptz.hold_max_ms]"); only the upper bound is a config value.
HOLD_MIN_MS = 200  # contract constant, deliberately not a config key

#: Ack schema tag, same family as cmd/mode/ack (mode_ack_v1).
ACK_SCHEMA = "ptz_ack_v1"  # lets p5 tell a PTZ ack from a mode ack

#: The three axes a stop may name (11 S7.4.4 "axes"; Qt contract S3.6).
AXES = ("pan", "tilt", "zoom")  # order matters: vel tuples are (pan, tilt, zoom)

#: PtzCommand.source (11 S7.4.4 closed set) -> domain-5 source id in the
#: resolved arbiter config. 11 S7A row 5 names the manual holder as
#: "cloud / HMI (manual)"; the voice and WeCom paths are operator-initiated
#: manual control too, so they share that id (and that priority). "bit" has no
#: domain-5 source configured in this build and is therefore refused.
SOURCE_IDS = {"cloud": "manual_cloud", "hmi": "manual_cloud",  # operator-driven sources share the manual id
              "wecom": "manual_cloud", "voice_local": "manual_cloud",  # voice / WeCom are operator-driven as well
              "auto_track": "auto_track"}  # the autonomous tracker keeps its own lower id

#: Fields 11 S7.4.4 deleted: refused with E_CAPABILITY, never translated
#: (a silent translation would make "turn to 0 degrees" mean something else,
#: and no read-back could reveal it).
DELETED_FIELDS = ("pan_deg", "tilt_deg", "d_pan_deg", "d_tilt_deg", "d_zoom",  # absolute angles / increments / magnification
                  "zoom")   # top-level "zoom" = the deleted absolute magnification
#: Deleted actions (same table) and the hint the contract tells us to give.
DELETED_ACTIONS = {"move_abs": "use action=jog", "move_rel": "use action=nudge",  # move_abs / move_rel: absolute or angle-based
                   "home": "use action=goto_preset"}  # home: the only way back is a preset (T-PTZ-1)
#: Actions that exist in 11 S7.4.4 but are not driven by this build.
NOT_IMPLEMENTED_ACTIONS = ("goto_preset", "set_track", "patrol")  # answered E_NOT_IMPLEMENTED, never faked


class PtzOps:  # interface the ONVIF driver implements
    """Camera motion primitives the controller needs (ONVIF, 11 S7.4.3).

    Implemented by runtime/ptz_wiring.py on top of the ONVIF session. Both
    methods may raise; the controller turns any exception into an
    E_UNHEALTHY answer and clears its motion record.
    """

    def move(self, pan: float, tilt: float, zoom: float) -> None:  # pan / tilt / zoom in [-1, 1], signs per 11 S7.4.4
        """One ContinuousMove carrying all three axes (11 PTZ-C5)."""
        raise NotImplementedError  # abstract: PtzDriver provides it

    def stop(self, pantilt: bool, zoom: bool) -> None:  # pantilt / zoom map to the ONVIF Stop flags
        """One ONVIF Stop for the named axis groups."""
        raise NotImplementedError  # abstract: PtzDriver provides it


class FocusOps:  # interface the LAPI client implements
    """The LAPI refocus primitive (11 S7.4.7 F4), implemented by lapi_focus."""

    def refocus(self) -> Tuple[bool, bool]:  # (ok, focus_mode_restored) feeds the ack detail
        """Run FocusMode 2 -> 0 -> dwell -> 2; return (ok, focus_mode_restored)."""
        raise NotImplementedError  # abstract: LapiFocusClient provides it


@dataclass(frozen=True)  # frozen: limits cannot drift at runtime
class PtzLimits:  # built once by the wiring from read_ptz_config
    """Every numeric limit, injected from the resolved config (CLAUDE.md 3.1:
    no code-side defaults). Units are in the names."""
    hold_max_ms: int        # 11 S7.4.4 ptz.hold_max_ms (Qt contract U-1)
    min_pulse_ms: int       # 11 S7.4.4 ptz.min_pulse_ms
    max_pulse_ms: int       # 11 S7.4.4 ptz.max_pulse_ms
    max_cmd_hz: int         # 11 S7.4.5 ptz.max_cmd_hz (= 17, never higher)

    def __post_init__(self) -> None:  # runs right after the generated __init__
        """Refuse limits that would disable a guard (fail-silent forms)."""
        if self.hold_max_ms < HOLD_MIN_MS:  # a max below the contract minimum disables clamping
            raise ValueError("hold_max_ms %r < %d" % (self.hold_max_ms, HOLD_MIN_MS))  # refused at startup, not at the first jog
        if not (0 < self.min_pulse_ms <= self.max_pulse_ms):  # zero / inverted range would make every nudge invalid
            raise ValueError("bad pulse range %r..%r"  # message names both bounds
                             % (self.min_pulse_ms, self.max_pulse_ms))  # values as configured
        if not (0 < self.max_cmd_hz <= 17):  # 17 Hz is the camera's real ceiling (58 ms per call)
            # 11 S7.4.5: "ptz.max_cmd_hz = 17 (not to be configured higher)".
            raise ValueError("max_cmd_hz must be in 1..17, got %r" % self.max_cmd_hz)  # refused rather than silently capped


@dataclass  # mutable: deadline and vel change while it runs
class _Motion:  # private: only the controller creates it
    """The one motion the head is doing now (there is never more than one)."""
    kind: str                         # "jog" or "legs"
    source: str                       # PtzCommand.source of the holder
    priority: int                     # domain-5 priority of that source
    cmd_id: str                       # the command that started it (events)
    vel: Tuple[float, float, float]   # velocity last commanded to the camera
    deadline_ms: int = 0              # jog: dead-man; legs: end of this leg
    hold_ms: int = 0                  # jog only, reported in the event
    legs: List[Tuple[Tuple[float, float, float], int]] = field(default_factory=list)  # remaining pulse legs after the current one


def ack(cmd_id: str, result: str, code: str = "OK",  # used for every answer, sync and async
        detail: Optional[Dict[str, Any]] = None, message: str = "") -> Dict[str, Any]:  # message stays empty unless a caller has prose
    """cmd/ptz/ack body (11 S7.7 Ack, same shape as cmd/mode/ack)."""
    body: Dict[str, Any] = {"schema": ACK_SCHEMA, "cmd_id": cmd_id,  # schema tag + the cmd_id the requester matches on
                            "result": result, "code": code}  # result: accepted / rejected; code: OK or E_*
    if message:  # optional human text
        body["message"] = message  # kept out of the body when empty
    if detail is not None:  # detail carries applied / clamped / holder
        body["detail"] = detail  # attached only when given
    return body  # plain dict: the wiring serialises it


def _clamp_vel(raw: Any) -> Tuple[Optional[Tuple[float, float, float]], List[str]]:  # used by jog and nudge
    """vel dict -> (pan, tilt, zoom) clamped to [-1, 1] plus the clamped axis
    names; None when vel is not a mapping or carries a non-finite number
    (11 S7.4.4: vel.* clamp, but NaN / non-number is a schema error)."""
    if not isinstance(raw, Mapping):  # vel must be an object, not a list or number
        return None, []  # None = schema error for the caller
    out: List[float] = []  # collected in AXES order
    clamped: List[str] = []  # axis names that were clamped (ack detail.clamped)
    for axis in AXES:  # pan, tilt, zoom
        v = raw.get(axis, 0.0)  # an absent axis does not move (11 S7.4.4)
        if v is None:  # explicit null means the same as absent
            v = 0.0                    # "0 or absent = this axis does not move"
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):  # bool is an int subclass: refuse True / False too
            return None, []  # one bad axis invalidates the whole command
        if v > 1.0 or v < -1.0:  # outside the ONVIF normalised range
            clamped.append(axis)  # reported, not hidden
            v = max(-1.0, min(1.0, float(v)))  # clamp into [-1, 1]
        out.append(float(v))  # always float for the ONVIF body
    return (out[0], out[1], out[2]), clamped  # fixed-order tuple + clamp report


def _clamp_int(raw: Any, lo: int, hi: int) -> Tuple[Optional[int], bool]:  # used for hold_ms and pulse_ms
    """An integer field clamped to [lo, hi]; (None, False) if not an integer."""
    if isinstance(raw, bool) or not isinstance(raw, int):  # integers only (bool and float refused)
        return None, False  # caller answers E_SCHEMA
    if raw < lo:  # below range: lift to the bound
        return lo, True  # clamped flag goes into the ack
    if raw > hi:  # above range: cut to the bound
        return hi, True  # clamped flag goes into the ack
    return raw, False  # in range: unchanged


class PtzController:  # one instance per p2 process
    """Owns the head: one motion at a time, dead-man, budget, admission.

    Thread model: handle_command / handle_legs are called from the cmd/ptz
    handler threads, tick from the wiring's 50 Hz scheduler thread; every
    state change happens under self._lock. Camera calls are made under the
    lock too -- they are ~58 ms each (11 PTZ-C4) and serialising them is what
    the single ONVIF session requires anyway. The refocus worker does NOT take
    the lock while it talks to LAPI, only to publish its ack.
    """

    def __init__(self, ops: PtzOps, focus: Optional[FocusOps], limits: PtzLimits,  # everything injected: no camera, no Zenoh, no clock here
                 priorities: Mapping[str, int], *,  # priorities from arbiter.domains.ptz.sources
                 now_ms: Callable[[], int],  # monotonic clock only (CLAUDE.md 3.4)
                 publish_ack: Callable[[Dict[str, Any]], None],  # ack body -> cmd/ptz/ack
                 publish_event: Callable[[str, Dict[str, Any]], None],  # (code, detail) -> event/warn/ptz
                 spawn: Optional[Callable[[Callable[[], None]], None]] = None) -> None:  # defaults to a daemon thread per refocus
        """priorities: domain-5 source id -> priority (resolved config).
        now_ms: monotonic milliseconds (CLAUDE.md 3.4). spawn: runs a callable
        on another thread (injected so tests can run refocus synchronously)."""
        self._ops = ops  # ONVIF primitives
        self._focus = focus  # LAPI refocus, may be None if unavailable
        self._lim = limits  # frozen limits
        self._prio = dict(priorities)  # copied: the caller's mapping may change
        self._now = now_ms  # monotonic ms source
        self._publish_ack = publish_ack  # ack sink
        self._publish_event = publish_event  # event sink
        self._spawn = spawn or (lambda fn: threading.Thread(  # production spawn: a daemon thread
            target=fn, name="p2.ptz_refocus", daemon=True).start())  # named for py-spy / journald
        self._lock = threading.Lock()  # guards motion, pending, budget and refocus flag
        self._motion: Optional[_Motion] = None  # None = head idle
        self._pending_vel: Optional[Tuple[float, float, float]] = None  # velocity merged over budget, applied by tick
        self._cmd_times: List[int] = []     # camera commands in the last second
        self._refocus_busy = False  # True while a refocus worker runs
        self._deferred: Optional[Callable[[], None]] = None   # refocus to start
        self.stats: Dict[str, int] = {"camera_cmds": 0, "renewals": 0,  # observability counters (log / tests)
                                      "deadman_stops": 0, "coalesced": 0,  # deadman_stops: how often the operator link failed
                                      "preempted": 0, "errors": 0}  # errors: camera calls that raised

    # -- public entry points -------------------------------------------------

    def handle_command(self, cmd: Mapping[str, Any]) -> Optional[Dict[str, Any]]:  # PtzCommand from cloud / HMI via cmd/ptz
        """Run one PtzCommand; publish and return its ack. Returns None only
        for a refocus that started (its ack is published when it finishes)."""
        cmd_id = cmd.get("cmd_id") if isinstance(cmd.get("cmd_id"), str) else ""  # missing / non-string cmd_id answers with an empty id
        with self._lock:  # all state work under the lock
            body = self._dispatch(cmd, cmd_id)  # returns the ack, or None for an async refocus
            deferred, self._deferred = self._deferred, None  # take the deferred refocus start, if any
        if deferred is not None:  # a refocus was accepted for start
            # Started AFTER the lock is released: the refocus worker takes the
            # lock to publish its ack, so starting it inside would deadlock
            # whenever spawn runs inline (and holds the lock needlessly when
            # it does not).
            self._spawn(deferred)  # worker publishes its own ack later
        if body is not None:  # sync answer exists
            self._publish_ack(body)  # published outside the lock
        return body  # returned for tests and for the caller's log

    def handle_legs(self, legs: List[Tuple[Tuple[float, float, float], int]],  # voice pulse plans from ptz_driver.plan
                    source: str, cmd_id: str) -> Optional[Dict[str, Any]]:  # source is voice_local for the voice path
        """A voice pulse sequence (E01 / E06 / E07) as a bounded motion: each
        leg is (vel, ms). Interruptible like a nudge. Voice has no ack key, so
        the returned verdict is only logged by the caller."""
        with self._lock:  # same lock as every other entry point
            return self._start_legs(legs, source, cmd_id)  # voice has no ack key; the verdict is logged

    def stop_all(self, cmd_id: str = "voice-stop") -> Dict[str, Any]:  # voice E05 / E08 and shutdown
        """Stop every axis (voice E05 / E08, shutdown). Never refused."""
        with self._lock:  # a stop is never refused
            return self._stop(cmd_id, list(AXES))  # all three axes

    def tick(self) -> None:  # driven by the wiring's 50 Hz thread
        """Scheduler step: apply a merged velocity, fire the dead-man, advance
        or finish pulse legs. Called every ~20 ms by the wiring."""
        with self._lock:  # same lock as the command path
            now = self._now()  # one clock read per tick
            self._flush_pending(now)  # first apply a merged velocity if budget allows
            m = self._motion  # the current motion, if any
            if m is None or now < m.deadline_ms:  # idle, or deadline not reached yet
                return  # nothing else to do this tick
            if m.kind == "jog":  # a jog that was not renewed in time
                self._deadman(m)  # PTZ-D1: Stop + event
            else:  # a pulse leg ran out
                self._next_leg(m, now)  # next leg or final Stop

    @property  # read-only view
    def active(self) -> bool:  # used by tests and the wiring
        """True while the head is commanded to move (jog or pulse legs)."""
        return self._motion is not None  # any motion record = head commanded to move

    # -- dispatch ------------------------------------------------------------

    def _dispatch(self, cmd: Mapping[str, Any], cmd_id: str) -> Optional[Dict[str, Any]]:  # under the lock, from handle_command
        """Shape checks shared by every action, then the per-action handler."""
        action = cmd.get("action")  # closed set, checked below
        if action == "stop":  # stop is handled before every other check
            # Before ANY other check, the source included: a stop from a
            # malformed or unknown sender still stops the head (stopping is
            # always safe; refusing it first would leave the head moving).
            return self._stop_cmd(cmd, cmd_id)  # axes validated inside
        for name in DELETED_FIELDS:  # 11 S7.4.4 deleted fields
            if name in cmd:  # presence alone is refused
                return ack(cmd_id, "rejected", E_CAPABILITY,  # never translated to a nudge / preset
                           {"reason": "no_absolute_positioning", "field": name,  # field named so the sender can fix it
                            "hint": "use action=nudge"})  # pointer to the replacement
        if isinstance(action, str) and action in DELETED_ACTIONS:  # deleted actions
            return ack(cmd_id, "rejected", E_CAPABILITY,  # same refusal family
                       {"reason": "no_absolute_positioning",  # positioning does not exist on this head
                        "hint": DELETED_ACTIONS[action]})  # per-action replacement hint
        if action in NOT_IMPLEMENTED_ACTIONS:  # exists in 11, not driven here
            return ack(cmd_id, "rejected", E_NOT_IMPLEMENTED, {"action": action})  # never a fake success
        source = cmd.get("source")  # who is asking (11 S7.4.4 closed set)
        if source not in SOURCE_IDS or SOURCE_IDS[source] not in self._prio:  # unknown or not configured in domain 5
            # Closed set (CLAUDE.md 3.5): an unknown source is refused, never
            # given some priority by guesswork.
            return ack(cmd_id, "rejected", E_SCHEMA, {"field": "source"})  # refused, not guessed
        ignored = ["speed_pct"] if "speed_pct" in cmd else []  # deprecated field: ignored and reported
        if action == "clear_track":  # 11 S7.4.4 clear_track
            # 11 S7.4.4: "exit tracking and Stop all axes at once". No
            # tracking runs in this build, so only the Stop half applies.
            return self._stop(cmd_id, list(AXES))  # tracking does not run: only its Stop half
        if action == "jog":  # open-ended, held by renewals
            return self._jog(cmd, cmd_id, source, ignored)  # per-action handler
        if action == "nudge":  # bounded pulse
            return self._nudge(cmd, cmd_id, source, ignored)  # per-action handler
        if action == "refocus":  # LAPI one-shot refocus
            return self._refocus(cmd_id, source)  # async: may return None
        return ack(cmd_id, "rejected", E_SCHEMA, {"field": "action"})  # action outside the closed set

    def _admit(self, source: str) -> Optional[Dict[str, Any]]:  # domain 5 admission (11 S7A)
        """Domain-5 admission: refuse when a higher-priority source moves the
        head; otherwise None (proceed). Equal priority takes over (last-wins
        among manual controllers, Qt contract R-7)."""
        m = self._motion  # current holder, if any
        prio = self._prio[SOURCE_IDS[source]]  # priority of the requesting source
        if m is not None and m.priority > prio:  # strictly higher holder wins; equal takes over
            return {"domain": "ptz", "holder": m.source, "holder_priority": m.priority}  # 11 S7A.0 E_BUSY detail shape
        return None  # admitted

    # -- jog -----------------------------------------------------------------

    def _jog(self, cmd: Mapping[str, Any], cmd_id: str, source: str,  # 11 S7.4.4 jog
             ignored: List[str]) -> Dict[str, Any]:  # ignored: deprecated fields to report
        """Open-ended motion held by renewals (11 S7.4.5 PTZ-D1..D3)."""
        vel, clamped = _clamp_vel(cmd.get("vel"))  # normalise velocities first
        if vel is None:  # not an object / non-finite number
            return ack(cmd_id, "rejected", E_SCHEMA, {"field": "vel"})  # schema error, nothing sent
        if vel == (0.0, 0.0, 0.0):  # a jog that moves nothing is meaningless
            return ack(cmd_id, "rejected", E_SCHEMA,  # refused rather than treated as stop
                       {"field": "vel", "reason": "all_axes_zero"})  # reason distinguishes it from a bad type
        hold, hold_clamped = _clamp_int(cmd.get("hold_ms"), HOLD_MIN_MS,  # hold_ms clamped to [200, hold_max_ms]
                                        self._lim.hold_max_ms)  # upper bound from config
        if hold is None:  # absent or not an integer
            # PTZ-D1: a jog without hold_ms would have no dead-man at all.
            return ack(cmd_id, "rejected", E_SCHEMA, {"field": "hold_ms"})  # no dead-man = refused (PTZ-C6)
        if hold_clamped:  # clamp is reported, not hidden
            clamped.append("hold_ms")  # named in detail.clamped
        busy = self._admit(source)  # domain 5 admission
        if busy is not None:  # a higher priority source moves the head
            return ack(cmd_id, "rejected", E_BUSY, busy)  # holder named in detail
        now = self._now()  # one clock read for this command
        detail: Dict[str, Any] = {"action": "jog", "renewed": False,  # ack detail for both renewal and start
                                  "clamped": clamped, "hold_ms": hold}  # hold_ms actually in force
        if ignored:  # speed_pct seen
            detail["ignored"] = ignored  # reported per 11 S7.4.4
        m = self._motion  # current motion, if any
        if (m is not None and m.kind == "jog" and m.source == source  # same holder still jogging
                and self._target_vel(m) == vel):  # and the request equals what it asked for
            # PTZ-D2: same velocity = renewal. Push the deadline, send NOTHING.
            m.deadline_ms = now + hold  # push the dead-man only
            m.hold_ms = hold  # the latest hold wins
            self.stats["renewals"] += 1  # observability
            detail["renewed"] = True  # tells p5 / Qt no camera command was sent
            return ack(cmd_id, "accepted", "OK", detail)  # renewal done: zero ONVIF commands
        try:  # camera calls below may raise
            cam = self._take_head(source, cmd_id, now)  # stop first if the holder changes
            # vel of the record = what the CAMERA is doing now; the request is
            # applied by _command_vel (sent now, or merged and sent by tick).
            self._motion = _Motion(kind="jog", source=source,  # record before sending: _send_move updates vel
                                   priority=self._prio[SOURCE_IDS[source]],  # holder priority for later admissions
                                   cmd_id=cmd_id, vel=cam,  # vel = what the camera runs now
                                   deadline_ms=now + hold, hold_ms=hold)  # dead-man armed
            coalesced = self._command_vel(vel, now)  # sent now or merged for the tick
        except Exception as exc:          # noqa: BLE001 -- camera fault -> answer
            return self._camera_failed(cmd_id, exc)  # E_UNHEALTHY, motion cleared
        if coalesced:  # merged over budget
            detail["coalesced"] = True  # tells the sender it was merged
        return ack(cmd_id, "accepted", "OK", detail)  # accepted: the dead-man now guards the head

    # -- nudge / legs --------------------------------------------------------

    def _nudge(self, cmd: Mapping[str, Any], cmd_id: str, source: str,  # 11 S7.4.4 nudge
               ignored: List[str]) -> Dict[str, Any]:  # same deprecated-field reporting as jog
        """Bounded pulse: start now, the tick stops it after pulse_ms. The ack
        goes out at the START (Qt contract S2), never after the pulse."""
        vel, clamped = _clamp_vel(cmd.get("vel"))  # same normalisation as jog
        if vel is None or vel == (0.0, 0.0, 0.0):  # a pulse must move something
            return ack(cmd_id, "rejected", E_SCHEMA, {"field": "vel"})  # nothing sent
        pulse, pulse_clamped = _clamp_int(cmd.get("pulse_ms"),  # pulse length clamped to config range
                                          self._lim.min_pulse_ms,  # lower bound
                                          self._lim.max_pulse_ms)  # upper bound
        if pulse is None:  # absent or not an integer
            return ack(cmd_id, "rejected", E_SCHEMA, {"field": "pulse_ms"})  # nothing sent
        if pulse_clamped:  # clamp is reported
            clamped.append("pulse_ms")  # named in detail.clamped
        verdict = self._start_legs([(vel, pulse)], source, cmd_id)  # a nudge is a one-leg pulse sequence
        if verdict["result"] == "accepted":  # only an accepted pulse gets the extra detail
            verdict["detail"].update({"action": "nudge", "clamped": clamped,  # echo what is in force
                                      "pulse_ms": pulse})  # the pulse actually used
            if ignored:  # speed_pct seen
                verdict["detail"]["ignored"] = ignored  # reported per 11 S7.4.4
        return verdict  # ack at the START of the pulse

    def _start_legs(self, legs: List[Tuple[Tuple[float, float, float], int]],  # shared by nudge and voice legs
                    source: str, cmd_id: str) -> Dict[str, Any]:  # source already validated by the caller
        """Start the first leg now; the tick runs the rest. A running motion
        is replaced (interrupted) -- never queued behind (Qt contract S3.4)."""
        if not legs:  # an empty plan moves nothing
            return ack(cmd_id, "rejected", E_SCHEMA, {"field": "legs"})  # refused, nothing sent
        if source not in SOURCE_IDS or SOURCE_IDS[source] not in self._prio:  # voice path enters here directly
            return ack(cmd_id, "rejected", E_SCHEMA, {"field": "source"})  # refused, not guessed
        busy = self._admit(source)  # domain 5 admission
        if busy is not None:  # a higher priority source moves the head
            return ack(cmd_id, "rejected", E_BUSY, busy)  # holder named in detail
        now = self._now()  # one clock read for this command
        (vel, ms), rest = legs[0], list(legs[1:])  # first leg now, the rest via tick
        try:  # camera calls below may raise
            cam = self._take_head(source, cmd_id, now)  # stop first if the holder changes
            self._motion = _Motion(kind="legs", source=source,  # a pulse is bounded by its legs, not a dead-man
                                   priority=self._prio[SOURCE_IDS[source]],  # holder priority for later admissions
                                   cmd_id=cmd_id, vel=cam,  # vel = what the camera runs now
                                   deadline_ms=now + ms, legs=rest)  # deadline = end of this leg
            coalesced = self._command_vel(vel, now)  # sent now or merged for the tick
        except Exception as exc:          # noqa: BLE001 -- camera fault -> answer
            return self._camera_failed(cmd_id, exc)  # E_UNHEALTHY, motion cleared
        detail: Dict[str, Any] = {}  # nudge adds its own fields on top
        if coalesced:  # merged over budget
            detail["coalesced"] = True  # tells the sender it was merged
        return ack(cmd_id, "accepted", "OK", detail)  # accepted at the start of the pulse

    def _next_leg(self, m: _Motion, now: int) -> None:  # from tick, under the lock
        """Leg time is up: start the next leg, or stop when none is left."""
        try:  # camera calls below may raise
            if m.legs:  # more legs to run
                vel, ms = m.legs.pop(0)  # next (vel, ms)
                m.deadline_ms = now + ms  # new leg deadline
                self._command_vel(vel, now)  # budget-aware send
                return  # motion continues
            self._send_stop(True, True, now)  # all legs done: stop every axis
        except Exception as exc:          # noqa: BLE001 -- logged, motion dropped
            self.stats["errors"] += 1  # observability
            _logger.error("ptz leg step failed: %s", exc)  # the head may still move: logged loudly
        self._motion = None  # pulse finished (or failed)
        self._pending_vel = None  # nothing left to merge

    # -- stop ----------------------------------------------------------------

    def _stop_cmd(self, cmd: Mapping[str, Any], cmd_id: str) -> Dict[str, Any]:  # stop with optional axes
        """stop with an optional axes list. A bad list is refused, but the
        head is stopped on every axis first: stopping is always safe and a
        malformed stop must never leave the head moving."""
        axes = cmd.get("axes")  # absent = all axes
        if axes is None:  # the common case
            return self._stop(cmd_id, list(AXES))  # all three axes
        if (not isinstance(axes, list) or not axes  # must be a non-empty list
                or any(a not in AXES for a in axes)):  # of known axis names only
            self._stop(cmd_id, list(AXES))  # stop everything FIRST
            return ack(cmd_id, "rejected", E_SCHEMA, {"field": "axes"})  # then report the bad field
        return self._stop(cmd_id, list(axes))  # valid subset

    def _stop(self, cmd_id: str, axes: List[str]) -> Dict[str, Any]:  # used by stop / clear_track / voice stop
        """Stop the named axes. Never coalesced, never budget-refused."""
        now = self._now()  # one clock read
        detail: Dict[str, Any] = {"action": "stop", "axes": axes}  # echoed in the ack
        m = self._motion  # may be None
        if m is None:  # nothing was moving
            detail["note"] = "no_active_motion"  # Qt contract S3.6 note
        try:  # the camera call may raise
            self._send_stop("pan" in axes or "tilt" in axes, "zoom" in axes, now)  # pan or tilt -> PanTilt flag; zoom -> Zoom flag
        except Exception as exc:          # noqa: BLE001 -- camera fault -> answer
            return self._camera_failed(cmd_id, exc)  # E_UNHEALTHY, motion cleared
        if m is not None:  # something was moving
            pan, tilt, zoom = self._target_vel(m)  # what the holder asked for
            if "pan" in axes:  # zero the stopped axes
                pan = 0.0  # pan stopped
            if "tilt" in axes:  # tilt named
                tilt = 0.0  # tilt stopped
            if "zoom" in axes:  # zoom named
                zoom = 0.0  # zoom stopped
            if (pan, tilt, zoom) == (0.0, 0.0, 0.0) or m.kind == "legs":  # nothing left moving, or a pulse was cut
                # A pulse sequence is one bounded motion: stopping any of it
                # ends it (resuming a half-cut voice sweep would surprise).
                self._motion = None  # motion over
                self._pending_vel = None  # drop any merged velocity
            else:  # a jog with axes still moving
                # Partial stop of a jog: the remaining axes keep moving under
                # the same dead-man; the camera already stopped the named ones
                # and the others need their velocity re-issued (ONVIF Stop of
                # PanTilt stops both pan and tilt).
                m.vel = (0.0, 0.0, zoom) if ("pan" in axes or "tilt" in axes) \
                    else (m.vel[0], m.vel[1], 0.0)   # what the camera does now
                try:  # the re-send may raise
                    if m.vel != (pan, tilt, zoom):   # e.g. zoom-only stop: nothing to re-send
                        self._command_vel((pan, tilt, zoom), now)  # restore the axes that must keep moving
                    else:  # camera already runs the target
                        self._pending_vel = None  # nothing pending for this motion
                except Exception as exc:  # noqa: BLE001 -- head is stopped anyway
                    self.stats["errors"] += 1  # observability
                    _logger.error("ptz re-issue after partial stop failed: %s", exc)  # head is stopped on the named axes anyway
                    self._motion = None  # drop the record: no motion to guard
        return ack(cmd_id, "accepted", "OK", detail)  # a stop is always accepted

    # -- refocus -------------------------------------------------------------

    def _refocus(self, cmd_id: str, source: str) -> Optional[Dict[str, Any]]:  # 11 S7.4.4 refocus
        """11 S7.4.4 refocus: admission only (not a holder, no dead-man), run
        on a worker, ack when it finishes. A refocus already running absorbs
        this one: accepted + coalesced at once (Qt contract S3.5)."""
        if self._focus is None:  # LAPI client could not be built
            return ack(cmd_id, "rejected", E_UNHEALTHY,  # answered, never faked
                       {"item": "ptz", "reason": "lapi_unavailable"})  # item ptz per 11 S13.9
        busy = self._admit(source)  # admission only, not a holder
        if busy is not None:  # a higher priority source moves the head
            return ack(cmd_id, "rejected", E_BUSY, busy)  # holder named in detail
        if self._refocus_busy:  # a refocus is already running
            self.stats["coalesced"] += 1  # observability
            return ack(cmd_id, "accepted", "OK",  # merged into the running one, answered now
                       {"action": "refocus", "coalesced": True})  # Qt contract S3.5
        self._refocus_busy = True  # mark busy before the worker starts
        self._deferred = lambda: self._run_refocus(cmd_id)   # see handle_command
        return None  # ack comes from the worker

    def _run_refocus(self, cmd_id: str) -> None:  # runs on its own thread
        """Worker body: LAPI calls WITHOUT the lock (motion stays live)."""
        focus = self._focus  # may be None only in tests
        try:  # LAPI errors must not escape the worker
            ok, restored = focus.refocus() if focus is not None else (False, False)  # slow part, outside the lock
        except Exception as exc:          # noqa: BLE001 -- reported in the ack
            _logger.error("ptz refocus crashed: %s", exc)  # anything unexpected is a failed refocus
            ok, restored = False, False  # restored unknown -> False (honest)
        with self._lock:  # short critical section
            self._refocus_busy = False  # the next refocus may start
        if ok:  # camera back in auto, refocused
            body = ack(cmd_id, "accepted", "OK", {"action": "refocus"})  # plain success
        else:  # failed somewhere in the sequence
            body = ack(cmd_id, "rejected", E_UNHEALTHY,  # E_UNHEALTHY per 11 S7.4.4 row
                       {"item": "ptz", "reason": "refocus_failed",  # reason fixed by the contract
                        "focus_mode_restored": restored})  # tells whether auto focus is back
        self._publish_ack(body)  # published from the worker

    # -- camera helpers (all called under the lock) -------------------------

    def _target_vel(self, m: _Motion) -> Tuple[float, float, float]:  # used by renewal and partial stop
        """The velocity the holder asked for: a merged (pending) one wins over
        the one the camera last got, so a renewal is compared to the request."""
        return self._pending_vel if self._pending_vel is not None else m.vel  # request beats camera state

    def _take_head(self, source: str, cmd_id: str,  # before every new motion
                   now: int) -> Tuple[float, float, float]:  # returns the camera's velocity afterwards
        """Before a new motion: when the holder changes, Stop first and tell
        (11 PTZ-D4, Qt contract S6 PTZ_PREEMPTED). Returns the velocity the
        camera is running at afterwards: zero after a Stop or when idle, the
        old holder's velocity when the same source just changes its mind."""
        m = self._motion  # current holder, if any
        if m is not None and m.source == source:  # same source: no handover
            return m.vel  # camera keeps running its velocity
        if m is not None:  # a different source takes over
            self._send_stop(True, True, now)  # PTZ-D4: Stop before handover
            self._pending_vel = None  # old merged velocity is void
            self.stats["preempted"] += 1  # observability
            self._publish_event("PTZ_PREEMPTED", {  # Qt contract S6 event
                "cmd_id": m.cmd_id, "source": m.source, "new_holder": source,  # who lost the head, who took it
                "new_cmd_id": cmd_id})  # the command that took it
            self._motion = None  # old motion gone
        return (0.0, 0.0, 0.0)  # camera stopped or idle

    def _command_vel(self, vel: Tuple[float, float, float], now: int) -> bool:  # the only path to a ContinuousMove
        """Send vel now if the 17 Hz budget allows, else merge it last-wins
        into the pending slot (11 S7.4.5 "merge by axis, last-wins, never
        queue"). Returns True when it was merged."""
        if not self._budget_ok(now):  # over 17 Hz
            self._pending_vel = vel  # last-wins: replaces an older merge
            self.stats["coalesced"] += 1  # observability
            return True  # merged
        self._pending_vel = None  # sending now supersedes any merge
        self._send_move(vel, now)  # one ContinuousMove
        return False  # sent

    def _flush_pending(self, now: int) -> None:  # first step of every tick
        """Tick half of the merge: apply the newest merged velocity."""
        if self._pending_vel is None or self._motion is None:  # nothing merged, or nothing moving
            self._pending_vel = None  # a merge without a motion is void
            return  # nothing to send
        if not self._budget_ok(now):  # still over budget
            return  # try again next tick
        vel, self._pending_vel = self._pending_vel, None  # take the newest merge
        try:  # the send may raise
            self._send_move(vel, now)  # one ContinuousMove
        except Exception as exc:          # noqa: BLE001 -- motion dropped
            self.stats["errors"] += 1  # observability
            _logger.error("ptz merged move failed: %s", exc)  # logged loudly
            self._motion = None  # no motion to guard any more

    def _budget_ok(self, now: int) -> bool:  # sliding one-second window
        """True if one more camera command fits in the last 1000 ms window."""
        self._cmd_times = [t for t in self._cmd_times if now - t < 1000]  # forget commands older than one second
        return len(self._cmd_times) < self._lim.max_cmd_hz  # room for one more

    def _send_move(self, vel: Tuple[float, float, float], now: int) -> None:  # every move goes through here
        """One ContinuousMove (counted against the budget)."""
        self._ops.move(*vel)  # the actual ONVIF call
        self._cmd_times.append(now)  # counted against the budget
        self.stats["camera_cmds"] += 1  # observability
        if self._motion is not None:  # keep the record in step with the camera
            self._motion.vel = vel  # camera now runs this velocity

    def _send_stop(self, pantilt: bool, zoom: bool, now: int) -> None:  # every stop goes through here
        """One ONVIF Stop. Counted, but never refused for budget: a Stop that
        waits for budget is the failure this whole module exists to avoid."""
        self._ops.stop(pantilt, zoom)  # the actual ONVIF call
        self._cmd_times.append(now)  # counted, but never gated
        self.stats["camera_cmds"] += 1  # observability

    def _deadman(self, m: _Motion) -> None:  # PTZ-D1
        """PTZ-D1: no renewal before the deadline -> Stop all axes + event."""
        try:  # the stop may raise
            self._send_stop(True, True, self._now())  # all axes
        except Exception as exc:          # noqa: BLE001 -- still clear + report
            self.stats["errors"] += 1  # observability
            _logger.error("ptz dead-man stop failed: %s", exc)  # logged loudly
        self.stats["deadman_stops"] += 1  # observability
        self._motion = None  # motion over either way
        self._pending_vel = None  # drop any merged velocity
        self._publish_event("PTZ_DEADMAN_STOP", {  # Qt contract S6 code
            "cmd_id": m.cmd_id, "source": m.source, "hold_ms": m.hold_ms})  # who held it and the hold in force

    def _camera_failed(self, cmd_id: str, exc: Exception) -> Dict[str, Any]:  # any camera exception lands here
        """A camera call raised: try one Stop, drop the motion, answer
        E_UNHEALTHY{item:ptz} (11 S7.4.4 / S13.9)."""
        self.stats["errors"] += 1  # observability
        _logger.error("ptz camera call failed: %s", exc)  # logged loudly
        try:  # best-effort stop
            self._ops.stop(True, True)
        except Exception:                 # noqa: BLE001 -- best effort only
            pass  # the original error is what we report
        self._motion = None  # nothing to guard any more
        self._pending_vel = None  # drop any merged velocity
        return ack(cmd_id, "rejected", E_UNHEALTHY,  # E_UNHEALTHY per 11 S13.9
                   {"item": "ptz", "reason": "camera_error"})  # reason distinguishes camera from LAPI
