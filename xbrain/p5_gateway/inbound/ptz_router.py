"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptz_router.py
Brief: Cloud xbrain/{rid}/cmd/ptz -> validate, map to the in-robot PtzCommand, translate p2's ack (Qt PTZ contract v0.2)

Description:
What problem this solves
------------------------
The cloud's keyboard control of the PTZ head (docs/MISSON/PTZ控制契约_qt端
v0.2草案.md, 99 U98) arrives on xbrain/{rid}/cmd/ptz. Before this module p5
did not subscribe to it at all: every press went nowhere (contract S9).
This is the gateway half; p2_core/ptz/ptz_controller.py is the robot half.

What it does (pure logic -- no Zenoh, no clock of its own)
---------------------------------------------------------
  * validate the v2.0 data object against contract S3 (fields, closed sets,
    ranges, forbidden fields) and reject at the gateway with the contract's
    codes -- in particular "absolute zoom / angle -> 1003" and "IR / camera
    config -> 1006" are decided HERE, because letting them reach p2 would
    come back as E_CAPABILITY and map to 2002 "busy" (contract S7);
  * map an accepted payload onto 11 S7.4 PtzCommand (contract S7 table):
    move / zoom -> jog, zoom_step -> nudge, refocus -> refocus, stop -> stop,
    always source "cloud", cmd_id "c-" + msg_id (CLOUD_CMD_PREFIX, the same
    prefix p5 uses to recognise its own commands on in-robot acks);
  * track each task_id (one press of a key): a later move / zoom with the
    same parameters is a renewal; different parameters inside one task are
    refused AND the head is stopped (contract S3.2 / S3.3);
  * R-6 rate limit: at most 8 forwarded commands per second; over it a
    renewal is answered "accepted, coalesced" without being forwarded
    (the previous renewal already holds the dead-man for >= 600 ms); a stop
    is never coalesced; a start / step / refocus over the limit is still
    forwarded -- p2's own 17 Hz merge (11 S7.4.5) bounds the camera side, and
    dropping a start would make a press do nothing;
  * translate p2's in-robot ack (cmd/ptz/ack, ptz_ack_v1) into the contract
    S5 v2.0 ack: E_BUSY -> 2002 with holder, E_UNHEALTHY -> 2001 E_NOT_READY
    (refocus failure keeps reason + focus_mode_restored), E_NOT_IMPLEMENTED ->
    1006 E_TASK_UNSUPPORTED, E_SCHEMA / E_CAPABILITY -> 1003 E_INVALID_FIELD;
  * decide whether a rejection gets an audit event: once per task_id
    (contract S5 last note) -- 4 Hz renewals refused one after another must
    not become a 4 Hz event storm.

What it does NOT do
-------------------
  * no envelope parsing, no msg_id dedup, no publishing: runtime/cloud_wiring.py
    does those with the same helpers it uses for cmd/task;
  * no dead-man, no budget, no arbitration: p2 owns the head.

Pitfalls
--------
  * Renewals MUST carry a new msg_id (contract R-3); the bridge's dedup
    window answers a repeated msg_id with "duplicate" before this router
    sees it -- that is the contract's intended behaviour, not a bug here.
  * The internal jog of a renewal gets the renewal's OWN cmd_id ("c-" + its
    msg_id): p2 recognises a renewal by source + velocity, not by id, and a
    shared id would let p2-side caches collapse two different commands.
"""
from __future__ import annotations  # postponed annotations

import math  # isfinite: NaN / inf speeds refused
from dataclasses import dataclass, field  # plain records
from typing import Any, Callable, Dict, List, Optional, Tuple  # type hints only

from ...common import errors  # closed E_* set (CLAUDE.md 3.5)
from ..outbound.error_map import (  # Qt integer codes and v2.0 wire strings
    CODE_REQUIRED_FIELD,  # 1002
    V2_DETAIL_INVALID_FIELD,  # 1003 detail.code
    V2_DETAIL_NOT_READY,  # 2001 detail.code
    V2_DETAIL_REQUIRED_FIELD,  # 1002 detail.code
    V2_DETAIL_TASK_UNSUPPORTED,  # 1006 detail.code
    to_qt_code,  # E_* -> Qt integer (one mapping table for all keys)
)  # end of imports
from ..outbound.task_ack import RESULT_ACCEPTED, RESULT_REJECTED  # ack result words

#: Contract S3.1: the one task_type this key carries.
TASK_TYPE = "PTZ_CONTROL"  # contract S3.1
#: In-robot command source (11 S7.4.4) for everything from the cloud.
SOURCE = "cloud"  # 11 S7.4.4 source of every cloud command
#: Contract S3.1 action closed set.
ACTIONS = ("move", "zoom", "zoom_step", "refocus", "stop")  # contract S3.1 closed set
#: Contract S3.7: actions named as "not opened in this phase" -> 1006.
UNOPENED_ACTIONS = ("preset", "goto_preset", "patrol", "track", "set_track",  # contract S3.7: named as not opened
                    "clear_track", "scan", "home")  # answered 1006, never forwarded
#: Contract S3.2: move direction -> (pan sign, tilt sign); 11 S7.4.4 signs:
#: pan + = right, tilt + = up.
MOVE_DIRECTIONS = {"up": (0, 1), "down": (0, -1), "left": (-1, 0),  # (pan, tilt) signs per 11 S7.4.4
                   "right": (1, 0), "up_left": (-1, 1), "up_right": (1, 1),  # pan + = right, tilt + = up
                   "down_left": (-1, -1), "down_right": (1, -1)}  # diagonals drive both axes
#: Contract S3.3 / S3.4: zoom direction -> zoom sign (in = narrower view).
ZOOM_DIRECTIONS = {"in": 1, "out": -1}  # in = narrower view = zoom +
#: Contract S3.6 stop axes.
AXES = ("pan", "tilt", "zoom")  # contract S3.6
#: Contract ranges (S3.2 - S3.4). Protocol constants of the frozen-to-be
#: contract, not tunables: changing one is a contract change, not a config one.
SPEED_MIN, SPEED_MAX = 0.1, 1.0  # contract S3.2 / S3.3
HOLD_MIN_MS, HOLD_MAX_MS = 600, 2000  # contract S3.2: >= 2 renewal periods
PULSE_MIN_MS, PULSE_MAX_MS = 100, 2800  # contract S3.4
#: Contract S3.1 / S3.7 camera ids.
CAM_VIS, CAM_IR = "cam_ptz_vis", "cam_ptz_ir"  # contract S3.1 / S3.7
#: Contract R-6: forwarded commands per second.
RATE_LIMIT_PER_S = 8  # contract R-6
#: Contract S2 ack time limits: 500 ms for move / zoom / stop / zoom_step,
#: 2 s for refocus (it acks after the LAPI sequence ran).
ACK_TIMEOUT_S = {"refocus": 2.0}  # contract S2: refocus acks after it ran
ACK_TIMEOUT_DEFAULT_S = 0.5  # contract S2: 500 ms
#: A task_id idle this long is forgotten (a press lasts at most a few
#: seconds; renewals arrive every 250 ms).
TASK_IDLE_S = 10.0  # one press lasts seconds at most
#: Field names that express an absolute zoom / an angle (contract S3.7).
ABS_ZOOM_FIELDS = ("zoom_level", "zoom_x", "target_zoom", "zoom_ratio")  # contract S3.7 row 1
ANGLE_FIELDS = ("pan_deg", "tilt_deg", "angle", "d_pan_deg", "d_tilt_deg",  # contract S3.7 row 2
                "target_heading", "heading_deg")  # heading words too: no compass on this head
#: Camera configuration the robot owns (contract S3.7; 11 S7.4.7 B0 / F1 / F2).
CAMERA_CONFIG_FIELDS = ("bitrate", "BitRate", "i_interval", "IInterval",  # contract S3.7 last row
                        "focus_mode", "FocusMode", "shield_move_ptz",  # focus settings are F1 / F2, robot-owned
                        "ShieldTrigger")  # the LAPI sub-object name


@dataclass  # plain record
class PtzOutcome:  # returned by handle()
    """What the bridge must do with one cloud frame.

    forward: in-robot PtzCommands to publish on cmd/ptz, in order.
    ack: v2.0 ack fields to answer NOW (result, error_code, reason, detail),
         or None when the answer comes from p2's ack (pending_cmd_id set).
    pending_cmd_id: the cmd_id whose p2 ack completes this frame.
    reject_event: True when a rejection must be audited (first per task).
    """
    forward: List[Dict[str, Any]] = field(default_factory=list)  # commands to put on cmd/ptz
    ack: Optional[Dict[str, Any]] = None  # immediate answer, if any
    pending_cmd_id: Optional[str] = None  # p2 ack that completes the frame
    reject_event: bool = False  # audit this rejection


@dataclass  # plain record
class _Pending:  # private: only the router creates it
    """A forwarded command waiting for p2's ack."""
    msg_id: str  # the frame's msg_id (ref_msg_id)
    task_id: str  # the press (task_id)
    action: str  # Qt action name, echoed in the ack
    clamped: List[str]  # gateway-side clamps
    hold_ms: Optional[int]  # move / zoom only
    deadline: float  # monotonic deadline for p2's ack


def _reject(qt_code: int, detail_code: str, text: str,  # every rejection goes through here
            **detail: Any) -> Dict[str, Any]:  # extra detail fields
    """v2.0 rejection fields: top-level reason = human text (text), detail
    carries detail.code plus any machine fields -- including detail.reason,
    the machine-readable cause the contract names (S3.7 / S5)."""
    body = dict(detail)  # copy: callers may reuse dicts
    body["code"] = detail_code  # the contract detail.code
    return {"result": RESULT_REJECTED, "error_code": qt_code,  # v2.0 S3.1 ack fields
            "reason": text, "detail": body}  # human text + machine detail


def _invalid(field_name: str, text: str, **detail: Any) -> Dict[str, Any]:  # field-level 1003
    """1003 E_INVALID_FIELD naming the field."""
    return _reject(to_qt_code(errors.E_SCHEMA), V2_DETAIL_INVALID_FIELD,  # E_SCHEMA -> 1003
                   text, field=field_name, **detail)  # field named for the sender


def _missing(field_name: str) -> Dict[str, Any]:  # field-level 1002
    """1002 E_REQUIRED_FIELD naming the field."""
    return _reject(CODE_REQUIRED_FIELD, V2_DETAIL_REQUIRED_FIELD,  # no E_* for 1002: the v2.0 code itself
                   "missing required field: %s" % field_name, field=field_name)  # field named for the sender


def _unsupported(reason: str) -> Dict[str, Any]:  # 1006 with a contract reason
    """1006 E_TASK_UNSUPPORTED with the contract reason."""
    return _reject(to_qt_code(errors.E_NOT_IMPLEMENTED),  # E_NOT_IMPLEMENTED -> 1006
                   V2_DETAIL_TASK_UNSUPPORTED, "not supported: %s" % reason,  # detail.code E_TASK_UNSUPPORTED
                   reason=reason)  # detail.reason = the machine cause


def _number(v: Any) -> Optional[float]:  # speed parser
    """A finite real number, or None (bool, NaN, inf, strings refused)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):  # bool is an int subclass
        return None  # not a number
    return float(v) if math.isfinite(v) else None  # finite float or None


class CloudPtzRouter:  # one per bridge (per robot)
    """Per-robot state: open tasks, pending acks, the R-6 window, and which
    tasks already produced a reject event."""

    def __init__(self, *, now_mono: Callable[[], float],  # injected clock (CLAUDE.md 3.4)
                 cmd_prefix: str) -> None:  # the bridge's c- prefix
        """now_mono: monotonic seconds (CLAUDE.md 3.4). cmd_prefix: the
        bridge's CLOUD_CMD_PREFIX ("c-")."""
        self._now = now_mono  # monotonic seconds
        self._prefix = cmd_prefix  # prefix of in-robot cmd_ids
        self._tasks: Dict[str, Dict[str, Any]] = {}  # task_id -> {params, last}
        self._pending: Dict[str, _Pending] = {}  # cmd_id -> waiting frame
        self._sent: List[float] = []  # R-6 window timestamps
        self._rejected_tasks: Dict[str, float] = {}  # task_id -> first reject time

    # -- inbound -------------------------------------------------------------

    def handle(self, data: Dict[str, Any]) -> PtzOutcome:  # the only inbound entry
        """One cloud frame's data object (envelope already validated, msg_id
        already deduplicated by the bridge)."""
        now = self._now()  # one clock read per frame
        self._expire(now)  # drop idle state first
        msg_id = data.get("msg_id")  # required (v2.0 S1.2)
        task_id = data.get("task_id")  # required: one press
        for name, value in (("msg_id", msg_id), ("task_id", task_id),  # three required strings
                            ("task_type", data.get("task_type"))):  # task_type checked below
            if not isinstance(value, str) or not value:  # missing or empty
                return self._rejected(_missing(name), task_id)  # 1002
        if data["task_type"] != TASK_TYPE:  # this key carries one type
            return self._rejected(_invalid("task_type", "task_type must be %s" % TASK_TYPE),  # 1003
                                  task_id)  # the press is still known
        payload = data.get("payload")  # the action object
        if not isinstance(payload, dict):  # must be an object
            return self._rejected(_missing("payload"), task_id)  # 1002
        action = payload.get("action")  # closed set, checked in _forbidden
        if action == "stop":  # stop before any other check
            return self._stop(payload, msg_id, task_id, now)  # always forwarded
        early = self._forbidden(payload, action)  # contract S3.7 table
        if early is not None:  # refused at the gateway
            return self._rejected(early, task_id)  # refused, nothing forwarded
        if action in ("move", "zoom"):  # held actions
            return self._hold(payload, action, msg_id, task_id, now)  # start / renew / refuse change
        if action == "zoom_step":  # single step
            return self._zoom_step(payload, msg_id, task_id, now)  # nudge
        return self._refocus(payload, msg_id, task_id, now)  # only refocus is left

    def _forbidden(self, payload: Dict[str, Any], action: Any) -> Optional[Dict[str, Any]]:  # contract S3.7 + action + camera
        """Contract S3.7 table, decided at the gateway (see module doc)."""
        for name in ANGLE_FIELDS:  # angles first
            if name in payload:  # presence alone is refused
                return _invalid(name, "no absolute or angle positioning on this head",  # 1003
                                reason="no_absolute_positioning")  # contract reason
        for name in ABS_ZOOM_FIELDS:  # absolute zoom
            if name in payload:  # presence alone is refused
                return _invalid(name, "no absolute zoom on this head",  # 1003
                                reason="no_absolute_zoom")  # contract reason
        for name in CAMERA_CONFIG_FIELDS:  # camera configuration
            if name in payload:  # presence alone is refused
                return _unsupported("camera_config_robot_managed")  # 1006
        if action in UNOPENED_ACTIONS:  # named as not opened
            return _unsupported("action_not_open")  # 1006
        if action not in ACTIONS:  # anything else unknown
            return _invalid("action", "action must be one of %s" % ", ".join(ACTIONS))  # 1003 (v2.0 S1.3)
        if "camera" in payload:  # optional camera field
            cam = payload["camera"]  # its value
            if action == "move":  # move turns the shared head
                return _invalid("camera", "move turns the shared head; no camera field",  # 1003
                                reason="shared_head")  # contract reason
            if cam == CAM_IR:  # IR has a fixed focal length
                return _unsupported("ir_fixed_focal")  # 1006
            if cam != CAM_VIS:  # only the visible camera
                return _invalid("camera", "camera must be %s" % CAM_VIS)  # 1003
        return None  # nothing forbidden

    # -- move / zoom (held, renewed) ------------------------------------------

    def _hold(self, payload: Dict[str, Any], action: str, msg_id: str,  # move and zoom
              task_id: str, now: float) -> PtzOutcome:  # now: this frame's clock read
        """move / zoom: validate, then start, renew, or refuse a change."""
        directions = MOVE_DIRECTIONS if action == "move" else ZOOM_DIRECTIONS  # 8 or 2 directions
        direction = payload.get("direction")  # required
        if direction is None:  # absent
            return self._rejected(_missing("direction"), task_id)  # 1002
        if direction not in directions:  # outside the closed set
            return self._rejected(_invalid("direction", "direction outside the closed set"),  # 1003
                                  task_id)  # task kept for its reject mark
        speed, clamped, bad = self._speed(payload)  # speed with clamp report
        if bad is not None:  # bad speed
            return self._rejected(bad, task_id)  # refused, nothing forwarded
        hold, hold_clamped, bad = self._int(payload, "hold_ms", HOLD_MIN_MS, HOLD_MAX_MS)  # hold_ms with clamp report
        if bad is not None:  # bad hold
            return self._rejected(bad, task_id)  # refused, nothing forwarded
        if hold_clamped:  # clamped
            clamped.append("hold_ms")  # reported in the ack
        params = (action, direction, speed)  # what defines a renewal
        task = self._tasks.get(task_id)  # the open press, if any
        if task is not None and task["params"] != params:  # same press, different parameters
            # Contract S3.2 / S3.3: a change inside one task is refused and
            # the task is stopped (stopping is always safe).
            self._tasks.pop(task_id, None)  # the press is over
            out = self._rejected(_invalid("payload", "direction / speed changed within a task",  # 1003 per contract S3.2
                                          reason="params_changed_within_task"), task_id)  # contract reason
            out.forward.append(self._internal_stop(msg_id, None))  # stopping is always safe
            self._count(now)  # counts against R-6
            return out  # refusal + stop
        renewal = task is not None  # a known press = renewal
        if renewal and not self._room(now):  # R-6 full
            # R-6: the previous renewal holds the dead-man; merge this one.
            task["last"] = now  # keep the press alive in our books
            return PtzOutcome(ack={"result": RESULT_ACCEPTED, "error_code": 0, "reason": "",  # accepted without forwarding
                                   "detail": {"action": action, "renewed": True,  # renewed
                                              "clamped": clamped, "hold_ms": hold,  # same fields as a forwarded renewal
                                              "coalesced": True}})  # tells Qt it was merged
        self._tasks[task_id] = {"params": params, "last": now}  # open or refresh the press
        cmd = self._jog(msg_id, action, directions[direction], speed, hold)  # contract S7 jog
        return self._forward(cmd, msg_id, task_id, action, clamped, hold, now)  # forward + pending

    def _jog(self, msg_id: str, action: str, signs: Any, speed: float,  # contract S7 rows 1-2
             hold: int) -> Dict[str, Any]:  # dead-man length
        """Contract S7: move -> jog vel pan / tilt, zoom -> jog vel zoom."""
        if action == "move":  # pan / tilt
            pan_sign, tilt_sign = signs  # signs from the table
            vel = {"pan": pan_sign * speed, "tilt": tilt_sign * speed, "zoom": 0.0}  # zoom stays 0
        else:  # zoom
            vel = {"pan": 0.0, "tilt": 0.0, "zoom": signs * speed}  # pan / tilt stay 0
        return {"cmd_id": self._prefix + msg_id, "action": "jog", "vel": vel,  # own cmd_id per frame (R-3)
                "hold_ms": hold, "source": SOURCE}  # dead-man + cloud source

    # -- zoom_step / refocus / stop -------------------------------------------

    def _zoom_step(self, payload: Dict[str, Any], msg_id: str, task_id: str,  # contract S3.4
                   now: float) -> PtzOutcome:  # now: this frame's clock read
        """zoom_step -> nudge on the zoom axis (contract S3.4 / S7)."""
        direction = payload.get("direction")  # required
        if direction is None:  # absent
            return self._rejected(_missing("direction"), task_id)  # 1002
        if direction not in ZOOM_DIRECTIONS:  # in / out only
            return self._rejected(_invalid("direction", "direction must be in or out"), task_id)  # 1003
        speed, clamped, bad = self._speed(payload)  # speed
        if bad is not None:  # bad speed
            return self._rejected(bad, task_id)  # refused, nothing forwarded
        pulse, pulse_clamped, bad = self._int(payload, "pulse_ms", PULSE_MIN_MS, PULSE_MAX_MS)  # pulse length
        if bad is not None:  # bad pulse
            return self._rejected(bad, task_id)  # refused, nothing forwarded
        if pulse_clamped:  # clamped
            clamped.append("pulse_ms")  # reported in the ack
        cmd = {"cmd_id": self._prefix + msg_id, "action": "nudge",  # contract S7 nudge
               "vel": {"pan": 0.0, "tilt": 0.0, "zoom": ZOOM_DIRECTIONS[direction] * speed},  # zoom axis only
               "pulse_ms": pulse, "source": SOURCE}  # bounded by the pulse
        return self._forward(cmd, msg_id, task_id, "zoom_step", clamped, None, now)  # forward + pending

    def _refocus(self, payload: Dict[str, Any], msg_id: str, task_id: str,  # contract S3.5
                 now: float) -> PtzOutcome:  # now: this frame's clock read
        """refocus -> refocus (contract S3.5); camera already checked."""
        cmd = {"cmd_id": self._prefix + msg_id, "action": "refocus", "source": SOURCE}  # contract S7 refocus
        return self._forward(cmd, msg_id, task_id, "refocus", [], None, now)  # acks after it ran (2 s)

    def _stop(self, payload: Dict[str, Any], msg_id: str, task_id: str,  # contract S3.6
              now: float) -> PtzOutcome:  # now: this frame's clock read
        """stop is always forwarded (contract S3.6): a bad axes list or a
        camera field is refused, but a FULL stop still goes out first."""
        self._tasks.pop(task_id, None)  # a stop ends the press
        axes = payload.get("axes")  # optional
        problem = None  # nothing wrong yet
        if "camera" in payload:  # stop acts on the shared head
            problem = _invalid("camera", "stop acts on the shared head; no camera field",  # 1003
                               reason="shared_head")  # contract reason
        elif axes is not None and (not isinstance(axes, list) or not axes  # bad axes list
                                   or any(a not in AXES for a in axes)):  # non-empty subset only
            problem = _invalid("axes", "axes must be a non-empty subset of pan / tilt / zoom")  # 1003
        if problem is not None:  # refuse, but
            out = self._rejected(problem, task_id)  # the refusal
            out.forward.append(self._internal_stop(msg_id, None))  # full stop anyway
            self._count(now)  # counts against R-6
            return out  # refusal + stop
        cmd = self._internal_stop(msg_id, axes)  # valid stop
        return self._forward(cmd, msg_id, task_id, "stop", [], None, now)  # forward + pending

    def _internal_stop(self, msg_id: str, axes: Optional[List[str]]) -> Dict[str, Any]:  # in-robot stop
        """In-robot stop; axes omitted = all axes."""
        cmd: Dict[str, Any] = {"cmd_id": self._prefix + msg_id + ":stop"  # safety stops get their own id
                               if axes is None else self._prefix + msg_id,  # so p2's ack is not mistaken
                               "action": "stop", "source": SOURCE}  # cloud source
        if axes is not None:  # partial stop
            cmd["axes"] = list(axes)  # copy
        return cmd  # ready

    # -- field helpers ---------------------------------------------------------

    def _speed(self, payload: Dict[str, Any]) -> Tuple[float, List[str], Optional[Dict[str, Any]]]:  # speed parser
        """speed in (0, +inf) clamped to [0.1, 1.0]; <= 0 / NaN / non-number
        refused (direction alone carries the sign, contract S3.3)."""
        if "speed" not in payload:  # required
            return 0.0, [], _missing("speed")  # 1002
        v = _number(payload["speed"])  # finite or None
        if v is None or v <= 0:  # sign lives in direction
            return 0.0, [], _invalid("speed", "speed must be a positive finite number")  # 1003
        if v < SPEED_MIN or v > SPEED_MAX:  # outside 0.1 .. 1.0
            return min(SPEED_MAX, max(SPEED_MIN, v)), ["speed"], None  # clamped and reported
        return v, [], None  # in range

    def _int(self, payload: Dict[str, Any], name: str, lo: int,  # integer parser
             hi: int) -> Tuple[int, bool, Optional[Dict[str, Any]]]:  # lo / hi from the contract
        """An integer field clamped to [lo, hi] (contract: out of range is
        clamped and reported, a non-integer is refused)."""
        if name not in payload:  # required
            return 0, False, _missing(name)  # 1002
        v = payload[name]  # the value
        if isinstance(v, bool) or not isinstance(v, int):  # integers only
            return 0, False, _invalid(name, "%s must be an integer" % name)  # 1003
        if v < lo or v > hi:  # out of range
            return min(hi, max(lo, v)), True, None  # clamped and reported
        return v, False, None  # in range

    # -- forwarding, rate window, rejections -----------------------------------

    def _forward(self, cmd: Dict[str, Any], msg_id: str, task_id: str, action: str,  # every forwarded command
                 clamped: List[str], hold: Optional[int], now: float) -> PtzOutcome:  # ack-relevant extras
        """Forward one command and register its pending ack."""
        self._count(now)  # R-6 bookkeeping
        timeout = ACK_TIMEOUT_S.get(action, ACK_TIMEOUT_DEFAULT_S)  # contract S2 per action
        self._pending[cmd["cmd_id"]] = _Pending(msg_id, task_id, action, clamped,  # remember the frame
                                                hold, now + timeout)  # deadline for p2's ack
        return PtzOutcome(forward=[cmd], pending_cmd_id=cmd["cmd_id"])  # one command

    def _room(self, now: float) -> bool:  # R-6
        """R-6: is there room for one more forwarded command this second?"""
        self._sent = [t for t in self._sent if now - t < 1.0]  # one-second window
        return len(self._sent) < RATE_LIMIT_PER_S  # room left

    def _count(self, now: float) -> None:  # R-6 bookkeeping
        """Record one forwarded command in the R-6 window."""
        self._room(now)  # trim the window
        self._sent.append(now)  # one more

    def _rejected(self, fields: Dict[str, Any], task_id: Any) -> PtzOutcome:  # every gateway refusal
        """A gateway rejection; audited once per task_id."""
        return PtzOutcome(ack=fields, reject_event=self._first_reject(task_id))  # audit only the first per press

    def _first_reject(self, task_id: Any) -> bool:  # contract S5 last note
        """True the first time a task_id is rejected (contract S5)."""
        if not isinstance(task_id, str) or not task_id:  # no task to dedup on
            return True  # audit it
        if task_id in self._rejected_tasks:  # already audited
            return False  # silent
        self._rejected_tasks[task_id] = self._now()  # remember
        return True  # audit it

    def _expire(self, now: float) -> None:  # housekeeping per frame
        """Forget idle tasks and old reject marks."""
        for tid in [t for t, v in self._tasks.items() if now - v["last"] > TASK_IDLE_S]:  # idle presses
            del self._tasks[tid]  # forgotten
        for tid in [t for t, ts in self._rejected_tasks.items() if now - ts > TASK_IDLE_S]:  # old reject marks
            del self._rejected_tasks[tid]  # forgotten

    # -- p2's answers --------------------------------------------------------

    def is_pending(self, cmd_id: Any) -> bool:  # used by the bridge
        """True if this in-robot ack belongs to a cloud frame."""
        return isinstance(cmd_id, str) and cmd_id in self._pending  # ours or not

    def translate(self, internal: Dict[str, Any]) -> Tuple[Dict[str, Any], str, str, bool]:  # p2's answer
        """p2's cmd/ptz/ack -> (v2.0 ack fields, ref msg_id, task_id,
        reject_event). Only call for is_pending(cmd_id)."""
        p = self._pending.pop(internal["cmd_id"])  # the waiting frame
        code = internal.get("code") or "OK"  # internal code
        detail = dict(internal.get("detail") or {})  # copy
        if internal.get("result") == "accepted":  # p2 accepted
            out: Dict[str, Any] = {"action": p.action}  # Qt action name
            if p.action in ("move", "zoom"):  # held actions only
                out["renewed"] = bool(detail.get("renewed"))  # from p2
                out["hold_ms"] = p.hold_ms  # the gateway-clamped hold
            out["clamped"] = sorted(set(p.clamped))  # gateway clamps
            for key in ("coalesced", "note"):  # contract S5 optional fields
                if key in detail:  # present only when set
                    out[key] = detail[key]  # passed through
            fields = {"result": RESULT_ACCEPTED, "error_code": 0, "reason": "",  # v2.0 accepted
                      "detail": out}  # contract S5 detail
            return fields, p.msg_id, p.task_id, False  # never audited
        fields = self._map_error(code, detail, p.action)  # contract S5 mapping
        if p.action in ("move", "zoom"):  # a refused press is over
            # A refused renewal ends the press on the Qt side (contract S3.3).
            self._tasks.pop(p.task_id, None)  # Qt stops renewing it
        return fields, p.msg_id, p.task_id, self._first_reject(p.task_id)  # audited once per press

    def _map_error(self, code: str, detail: Dict[str, Any], action: str) -> Dict[str, Any]:  # contract S5 table
        """Internal E_* -> contract S5 (qt code, detail.code, extras)."""
        if code == errors.E_BUSY:  # domain 5 refusal
            return _reject(to_qt_code(code), errors.E_BUSY,  # 2002 E_BUSY
                           "PTZ is held by a higher-priority source",  # human text
                           holder=detail.get("holder"),  # who holds the head
                           holder_priority=detail.get("holder_priority"))  # and its priority
        if code == errors.E_UNHEALTHY:  # camera / LAPI unavailable
            extra = {"item": detail.get("item", "ptz")}  # item ptz
            for key in ("reason", "focus_mode_restored"):  # refocus details
                if key in detail:  # present only when set
                    extra[key] = detail[key]  # passed through
            return _reject(to_qt_code(code), V2_DETAIL_NOT_READY,  # 2001 E_NOT_READY
                           "PTZ not ready (%s)" % detail.get("reason", "unhealthy"), **extra)  # reason in the text too
        if code == errors.E_NOT_IMPLEMENTED:  # not driven by p2
            return _unsupported("action_not_driven")  # 1006
        if code in (errors.E_SCHEMA, errors.E_CAPABILITY):  # should have been caught here
            return _invalid(str(detail.get("field", "payload")),  # 1003 naming p2's field
                            "rejected by the PTZ domain: %s" % code)  # internal code in the text
        return _reject(to_qt_code(code), code, "PTZ command failed: %s" % code)  # anything else: mapped integer + internal code

    def expired(self) -> List[Tuple[Dict[str, Any], str, str]]:  # contract S2 time limits
        """Pending commands whose ack did not come in time (contract S2):
        answered 2001 E_NOT_READY -- p2 not answering means the PTZ path is
        down (no credentials, p2 restarting)."""
        now = self._now()  # one clock read
        out = []  # collected answers
        for cid in [c for c, p in self._pending.items() if now >= p.deadline]:  # deadline passed
            p = self._pending.pop(cid)  # no longer pending
            if p.action in ("move", "zoom"):  # held actions
                self._tasks.pop(p.task_id, None)  # the press is over
            out.append((_reject(to_qt_code(errors.E_UNHEALTHY), V2_DETAIL_NOT_READY,  # 2001 E_NOT_READY
                                "PTZ did not answer in time", item="ptz",  # item ptz
                                reason="no_ack_from_ptz_domain"),  # machine cause
                        p.msg_id, p.task_id))  # routed to the frame
        return out  # answered by the bridge's tick
