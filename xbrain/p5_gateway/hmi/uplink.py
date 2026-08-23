"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: uplink.py
Brief: HMI -> P5 upstream frames -- the 11 S12.1.1 whitelist (W2/W3/W4/W7)

Description:
The browser's writable surface. 11 S12.1.1 calls its table "the only
authoritative list of HMI upstream write operations in the whole document set",
makes it frozen item F-8, and says in as many words that an implementer may not
wire one more type in code. So the closed set here IS that table:

    W1 estop | W2 goto | W3 exit_broadcast | W4 geo | W7 task

W5 and W6 are tombstones (never reused, so an old reference points at "removed"
rather than silently at another class), and W8 is reserved-unopened.

This module implements W2 (goto), W3 (exit_broadcast), W4 (geo) and W7 (task),
and refuses the rest with E_NOT_IMPLEMENTED -- a refusal that names the class,
rather than an unknown-type error that reads as a frontend bug. That leaves W1
estop as the only whitelist class not served here, and it is served by its own
<=10 ms REST path instead (S6.4).

*** W2 and W7 both land on cmd/task, and neither writes anything itself.
S12.1.1 requires goto to become a TASK rather than a BehaviorCommand (the fence
pre-check and the U07a ledger live on the task path), and the W7 row notes that
cmd/task already listed the HMI as a publisher -- so opening these two classes
adds no key to S2.2.3.

*** Of the five classes, only W1 / W2 / W7 can make the robot MOVE, and
S12.1.1's own self-check is that all three are "stop, or go with a check": W1
stops, W2 is L1 with a fence pre-check, W7 pauses/cancels. None of them drives
continuously -- that is why W6 teleop was removed rather than narrowed.

*** W4-F: no fence object is writable from the HMI, at all.

The rule is by geo.type, NOT by op. 00 HMI-03a requires that the safety
constraint (the electronic fence) never enters the HMI's writable surface, and
S12.1.1 spells out why the narrower "the HMI cannot DELETE a fence" is not
enough: disabling an allow fence is equivalent to deleting it (S9A.1 admits
exactly one), and changing its geometry is worse. Both of those are set_state
and upsert -- so a delete-only rule leaves the two more dangerous doors open.

*** On confirm.level, which is required and is NOT what protects anything.

S12.1.1 is explicit: confirm.level is an AUDIT credential, not an authorisation
one. U23 leaves the HMI unauthenticated, so the browser can put "L2" in the
field itself. The thing that actually stops a dangerous operation is its absence
from the whitelist (W4-F, W6 removed, W8 unopened). This module still REQUIRES
the field on an L2 op -- 12A.9's W4 row says a missing one is
E_CONFIRM_REQUIRED -- because the audit trail is worth having; it just must not
be mistaken for a permission check.

Boundary: builds frames, decides refusals. It opens no socket, publishes
nothing, and reads no clock (the rate limiter takes now_ms).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from xbrain.common.enums import GEO_TYPE
from xbrain.common.errors import (
    E_BUSY, E_CHANNEL_DENIED, E_CONFIRM_REQUIRED, E_NOT_IMPLEMENTED, E_SCHEMA,
)

#: 11 S12.1.1 upstream `type` closed set. Five values; W5/W6 are tombstones and
#: W8 is reserved. Adding a member here without amending S12.1.1 first is the
#: thing F-8 exists to prevent.
UPSTREAM_TYPES = frozenset({"estop", "goto", "exit_broadcast", "geo", "task"})

#: The W4 ops the HMI may ask for, and the confirmation level each carries
#: (S12.1.1 W4 row). refs is read-only. Any geo action outside this map -- force,
#: resync, get, list -- is not on the HMI surface at all: S7.9.5 makes force and
#: resync cloud-only, and get/list are served by the snapshot the browser
#: already receives.
W4_OPS: Dict[str, str] = {
    "rename": "L0",
    "set_state": "L0",
    "upsert": "L1",
    "delete": "L2",
    "refs": "L0",
}

#: The ops that write. Read against geo.type for W4-F.
_W4_WRITE_OPS = frozenset({"upsert", "delete", "rename", "set_state"})

#: W7 actions and their confirmation level (S12.1.1 W7 row). pause/resume are
#: L0 because "hold on a second" is the most frequent field intervention and a
#: dialog on it would cost the seconds it exists to save; cancel is L2 per 18
#: B07, and clear_queue is L2 because it acts on tasks the operator cannot all
#: see at once.
W7_ACTIONS: Dict[str, str] = {
    "pause": "L0",
    "resume": "L0",
    "cancel": "L2",
    "clear_queue": "L2",
}

#: W2 speed_profile closed set. U33 DELETED cruise and transit; they are not
#: legacy spellings to be mapped onto patrol, they are gone (see the refusal in
#: build_goto_command).
SPEED_PROFILES = frozenset({"obstacle_avoid", "patrol"})

#: S12.1.1: the downstream cmd_id is "h-" + req_id. Keeping the prefix means a
#: cmd/geo/ack can be routed back to the browser session that asked, and it
#: makes an HMI-originated command identifiable in P3's log without consulting
#: the origin field.
CMD_ID_PREFIX = "h-"


@dataclass(frozen=True)
class UplinkRefusal:
    """A frame that will not be forwarded, and the ack to send instead."""
    code: str
    reason: str
    detail: Optional[Dict[str, Any]] = None


@dataclass(frozen=True)
class UplinkCommand:
    """A frame that will be forwarded: the key and the payload to publish."""
    key: str
    payload: Dict[str, Any]
    req_id: str
    req_type: str


def parse_envelope(msg: Any) -> Dict[str, Any]:
    """Common S12.1.1 envelope checks, or raise ValueError with the reason.

    estop is deliberately NOT routed through here -- S12.1.1 makes it the one
    exception that bypasses schema validation, rate limiting and the restricted
    downgrade, because a malformed stop must still stop (S3.0.1). The caller
    checks for it before calling this.
    """
    if not isinstance(msg, dict):
        raise ValueError("frame is not an object")
    mtype = msg.get("type")
    if mtype not in UPSTREAM_TYPES:
        raise ValueError("type %r is not in the S12.1.1 whitelist" % (mtype,))
    req_id = msg.get("req_id")
    if not isinstance(req_id, str) or not req_id:
        raise ValueError("req_id is required")
    return {"type": mtype, "req_id": req_id}


def build_geo_command(msg: Dict[str, Any]) -> Any:
    """W4: an HMI geo frame -> a cmd/geo payload, or an UplinkRefusal.

    Returns rather than raises, because every outcome here ends in an ack to
    the browser and the refusals carry structure the operator's dialog uses.
    """
    req_id = msg.get("req_id")
    op = msg.get("op")
    geo = msg.get("geo")
    if op not in W4_OPS:
        return UplinkRefusal(
            E_SCHEMA, "op %r is not one of %s" % (op, sorted(W4_OPS)),
            {"op": op})
    if not isinstance(geo, dict):
        return UplinkRefusal(E_SCHEMA, "geo object is required")
    gtype = geo.get("type")
    if gtype not in GEO_TYPE:
        return UplinkRefusal(E_SCHEMA, "geo.type %r is unknown" % (gtype,),
                             {"type": gtype})
    # *** W4-F, before anything else about this frame is considered.
    if gtype == "fence" and op in _W4_WRITE_OPS:
        return UplinkRefusal(
            E_CHANNEL_DENIED,
            "fence objects are not writable from the HMI (11 S12.1.1 W4-F)",
            {"reason": "fence_not_writable_from_hmi", "op": op})
    geo_id = geo.get("geo_id")
    if not isinstance(geo_id, str) or not geo_id:
        return UplinkRefusal(E_SCHEMA, "geo.geo_id is required")
    level = W4_OPS[op]
    if level == "L2" and not _has_confirm(msg, "L2"):
        # 12A.9 W4: a missing confirm on an L2 op is E_CONFIRM_REQUIRED. See the
        # module docstring on what this does and does not protect.
        return UplinkRefusal(
            E_CONFIRM_REQUIRED,
            "op %r requires confirm.level L2" % (op,), {"op": op})
    payload: Dict[str, Any] = {
        "cmd_id": CMD_ID_PREFIX + req_id,
        "action": op,
        "type": gtype,
        "geo_id": geo_id,
        # CH-2: P5 stamps origin "hmi" and must never relabel it as cloud. That
        # single field is the whole permission boundary under U23, and S7.9.5
        # denies the cloud-only cells on the strength of it.
        "origin": "hmi",
        "base_rev": geo.get("base_rev", 0),
    }
    if op == "upsert":
        obj = geo.get("obj")
        if not isinstance(obj, dict):
            return UplinkRefusal(E_SCHEMA, "upsert needs geo.obj")
        payload["obj"] = obj
    elif op == "rename":
        name = geo.get("name")
        if not isinstance(name, str) or not name.strip():
            return UplinkRefusal(E_SCHEMA, "rename needs geo.name")
        payload["obj"] = {"name": name.strip()}
        for optional in ("num", "alias"):
            if optional in geo:
                payload["obj"][optional] = geo[optional]
    elif op == "set_state":
        state = geo.get("state")
        if not isinstance(state, str):
            return UplinkRefusal(E_SCHEMA, "set_state needs geo.state")
        payload["obj"] = {"state": state}
    return UplinkCommand(key="cmd/geo", payload=payload, req_id=req_id,
                         req_type="geo")


def build_task_command(msg: Dict[str, Any]) -> Any:
    """W7: an HMI task frame -> a cmd/task payload (11 S7.2), or a refusal.

    *** Why the HMI gets this at all, from S12.1.1's W7 row: after a link loss
    (U36) the robot KEEPS RUNNING its current task. The cloud is unreachable by
    definition at that moment, so if the HMI cannot pause either, there is no
    soft way to stop a task that is merely running WRONG -- only the e-stop.
    Using an e-stop for that costs a zero-velocity slam and an estop audit
    event, which is not what it is for.

    That is also the W1/W7 split: W1 stops MOTION and does not end the task
    (after U35 nothing latches, so the next motion command resumes), W7 changes
    the TASK state machine. S12.1.1 requires the HMI to present them apart and
    explicitly forbids putting cancel next to the e-stop button.
    """
    req_id = msg.get("req_id")
    action = msg.get("action")
    if action not in W7_ACTIONS:
        return UplinkRefusal(
            E_SCHEMA,
            "action %r is not one of %s" % (action, sorted(W7_ACTIONS)),
            {"action": action})
    task_id = msg.get("task_id")
    if action != "clear_queue":
        # S12.1.1 W7 and S7.2 agree, and both say it in the same words: there is
        # no "omit = the current task". The queue is live, so between the
        # operator reading "A is running" off the panel and this frame arriving,
        # A may have ended and B started -- the shorthand would pause B and
        # nothing in the record would show it.
        if not isinstance(task_id, str) or not task_id:
            return UplinkRefusal(
                E_SCHEMA,
                "action %r requires task_id (S12.1.1 W7 forbids "
                "'omit = the current task')" % (action,), {"action": action})
    else:
        task_id = None
    level = W7_ACTIONS[action]
    if level == "L2" and not _has_confirm(msg, "L2"):
        return UplinkRefusal(
            E_CONFIRM_REQUIRED,
            "action %r requires confirm.level L2" % (action,),
            {"action": action})
    payload: Dict[str, Any] = {
        "cmd_id": CMD_ID_PREFIX + str(req_id),
        "action": action,
        # CH-2 again: "hmi" is stamped here and is never taken from the frame.
        "source": "hmi",
    }
    if task_id is not None:
        payload["task_id"] = task_id
    return UplinkCommand(key="cmd/task", payload=payload, req_id=str(req_id),
                         req_type="task")


def build_exit_broadcast_command(msg: Dict[str, Any]) -> Any:
    """W3: the HMI's "force exit broadcast" button -> a cmd/mode ModeCommand.

    *** Why this class exists at all, from S12.1.1's W3 row: while B mode is
    running the local mic is closed by the half-duplex gate (S8.9), and a
    cloud operator saying "stop broadcasting" into a microphone triggers the
    self-trigger loop S8.7.4 / S8.9.1 already argued through. So in B mode this
    button is the ONLY exit that does not go through voice.

    *** It carries no preconditions and no confirm.
    S12.1.1 is explicit: W3 is subject to no task state, no mode state and no
    L2 confirm -- it does exactly one thing, leave B mode. Adding a confirm
    here would be actively harmful: the situation it exists for is one where
    the operator cannot use their voice and the robot is broadcasting.

    * It does NOT bypass `restricted` though -- W1 estop is the only class that
    does. `restricted` only appears after 20 consecutive violations on one
    connection, by which point that client is not trustworthy.
    """
    req_id = msg.get("req_id")
    payload: Dict[str, Any] = {
        "cmd_id": CMD_ID_PREFIX + str(req_id),
        # 11 S7.3's action for leaving B mode. P2's ModeFace maps it to IDLE.
        "action": "exit_broadcast",
        # CH-2 again: stamped, never read from the frame.
        "source": "hmi",
    }
    return UplinkCommand(key="cmd/mode", payload=payload, req_id=str(req_id),
                         req_type="exit_broadcast")


def build_goto_command(msg: Dict[str, Any]) -> Any:
    """W2: an HMI goto frame -> a cmd/task submit of a goto task, or a refusal.

    *** Why this becomes a TASK and not a BehaviorCommand (S12.1.1 W2, verbatim
    reasoning): BehaviorCommand's publisher closed set is p2_core / p3_task
    only, so a P5-published one is caught by S2.2.14's startup self-check. More
    importantly it would bypass P3's fence pre-check and the U07a breakpoint
    ledger -- the operator would get motion with no fence validation and no
    record of where it interrupted. "The HMI's goto MUST land as a task."

    *** The validation split matters here. P5 (gate G4) rejects what it can see
    from the frame alone -- both target forms missing, a non-numeric or
    out-of-range coordinate, a retired speed_profile. P3 rejects what needs the
    world: waypoint not found, target outside the fence, positioning degraded.
    Doing P3's half here would need P5 to read geo.db, which it must not
    (S7.8.4.3).
    """
    req_id = msg.get("req_id")
    waypoint_id = msg.get("waypoint_id")
    lat, lon = msg.get("lat"), msg.get("lon")
    params: Dict[str, Any] = {}
    if isinstance(waypoint_id, str) and waypoint_id:
        # S12.1.1 W2: when both forms are present waypoint_id WINS. Written as
        # a precedence rule the table states, not as "reject the ambiguity" --
        # a frontend that sends the tapped point alongside the snapped waypoint
        # is being helpful, not confused.
        params["waypoint_id"] = waypoint_id
    else:
        ok, refusal = _valid_coords(lat, lon)
        if not ok:
            return refusal
        params["lat"], params["lon"] = float(lat), float(lon)
    profile = msg.get("speed_profile")
    if profile is not None:
        if profile not in SPEED_PROFILES:
            # *** The retired values cruise / transit are refused, NOT read as
            # patrol. S13.6 (3) forbids interpreting an off-set value as the
            # nearest one, and U33 deleted these two: a stale frontend still
            # asking for "transit" has to be told, because silently giving it
            # patrol makes the bug invisible on both sides.
            return UplinkRefusal(
                E_SCHEMA,
                "speed_profile %r is not one of %s (U33 removed cruise and "
                "transit)" % (profile, sorted(SPEED_PROFILES)),
                {"speed_profile": profile})
        params["speed_profile"] = profile
    payload: Dict[str, Any] = {
        "cmd_id": CMD_ID_PREFIX + str(req_id),
        "action": "submit",
        # No task_id: the form is t-YYYYMMDD-NNN and only P3 holds the per-day
        # sequence (S7.2, corrected 2026-08-20). It comes back in the ack.
        "task": {"type": "goto", "params": params},
        "source": "hmi",
    }
    return UplinkCommand(key="cmd/task", payload=payload, req_id=str(req_id),
                         req_type="goto")


def _valid_coords(lat: Any, lon: Any):
    """(True, None) when lat/lon are usable WGS84, else (False, refusal).

    bool is excluded explicitly: in Python `True` is an int, so a frontend bug
    that sends lat: true would otherwise pass the numeric check and travel on
    as latitude 1.0 -- a real place, roughly 110 km off the equator.
    """
    if lat is None and lon is None:
        return False, UplinkRefusal(
            E_SCHEMA, "goto needs either waypoint_id or lat+lon")
    for name, value in (("lat", lat), ("lon", lon)):
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return False, UplinkRefusal(
                E_SCHEMA, "%s %r is not a number" % (name, value),
                {"field": name})
    if not -90.0 <= float(lat) <= 90.0:
        return False, UplinkRefusal(
            E_SCHEMA, "lat %r is out of range" % (lat,), {"field": "lat"})
    if not -180.0 <= float(lon) <= 180.0:
        return False, UplinkRefusal(
            E_SCHEMA, "lon %r is out of range" % (lon,), {"field": "lon"})
    return True, None


def _has_confirm(msg: Dict[str, Any], level: str) -> bool:
    confirm = msg.get("confirm")
    return isinstance(confirm, dict) and confirm.get("level") == level


def ack_frame(req_id: str, req_type: str, result: str, code: str = "OK",
              detail: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """The downstream ack. 11 S12.1.1 names the W4 answer `geo_ack`, defined as
    an alias of `ack` with req_type="geo" -- so one shape is emitted with the
    type spelled out, rather than two shapes that can drift."""
    frame: Dict[str, Any] = {"kind": "ack", "req_type": req_type,
                             "req_id": req_id, "result": result, "code": code}
    if detail is not None:
        frame["detail"] = detail
    return frame


def not_implemented(req_type: str) -> UplinkRefusal:
    """A whitelisted class this build does not serve yet.

    Distinct from an unknown type on purpose: "W7 task is not wired here" is a
    backlog item, while "type 'flyaway' is not in the whitelist" is a frontend
    sending something nobody reviewed. Collapsing them would hide the second.
    """
    return UplinkRefusal(E_NOT_IMPLEMENTED,
                         "upstream class %r is not wired in this build"
                         % (req_type,), {"type": req_type})


def rate_limited(req_type: str) -> UplinkRefusal:
    """SS-4: over the bucket. Never queued -- queueing would let an operator
    build back-pressure by hammering a button.

    The code is E_BUSY, not E_RATE_LIMIT. 17 S11's SS-4 names the latter, but
    11 S13 -- which is the closed set, and the only one -- does not define it,
    and CLAUDE.md 3.5 forbids inventing a code. E_BUSY is the nearest member
    ("the resource is taken"), and its retryability is `conditional`, which is
    the right advice here: wait and send it again. detail.reason carries the
    actual cause so a frontend can tell this apart from a genuine busy. The
    mismatch is recorded in 17 S11 rather than papered over.
    """
    return UplinkRefusal(E_BUSY, "rate limit exceeded",
                         {"reason": "rate_limited", "type": req_type})
