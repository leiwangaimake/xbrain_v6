"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: translate.py
Brief: CHK-0-38 cmd/task/ext 甲方云端任务入站翻译层

Description:
Every message that arrives on `xbrain/{rid}/cmd/task` from Qt goes
through this module before P3 sees it. The mapping is DEFENSIVE by
default: any field the contract does not name is dropped, any value
outside the closed set raises. Nothing is inferred, nothing is
defaulted; a shape defect surfaces here (E_SCHEMA) so it never reaches
the task queue.

Rules cross-referenced from docs/MISSON/任务枚举_qt端v2.0.md
(v2.0 evaluated 2026-08-09):

  * R1.4  rid MUST match ^[a-z0-9_-]{1,32}$ AND equal the second key
          segment
  * R1.5  outer envelope is EXACTLY {v, rid, ts, seq, src, data}; v==1
  * R1.5  ts is JSON number, float64 UTC seconds (NEVER ISO string)
  * R1.7  seq is uint64; new-connection-window resets are OK,
          in-window rewinds are DROP
  * R3.1  GOTO_KEYPOINT: coordinate_system=='WGS84', arrival_radius_m
          in [0.5, 10.0], waypoints IDs match ^w-[a-z0-9_]{1,40}$,
          route ID matches ^r-[a-z0-9_]{1,40}$
  * R3.2  STOP_TASK.action in {'pause','resume','cancel'}
  * R3.4  SET_ALARM_CONFIG.alarm_level in {1, 2}; siren_level in
          [0, 100]; duration_sec in [1, 20]; cooldown_sec in
          [0.5, 600.0]
  * R3.4  regions[] kind restricted to 'alarm_region'; 'keep_in' HARD
          REJECT with E_GEO_INVALID
  * R11.2 ts must be finite float, monotonically-plausible against
          receiver's monotonic clock (the receiver's window is
          checked in dedupe.py, not here)

Any translation failure returns a TranslateFailure named tuple with
the exact E_* closed-set code + detail; the caller emits it back on
`cmd/task/ack` with rejected=True.

R10.4 task_id 分域: the returned InboundTask carries an
`internal_task_id` prefixed with 'ext:' so P3 can never confuse a
cloud-issued task with a locally-generated one; the caller writes
BOTH the internal id and the original client task_id to the DB.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Dict, Optional

from xbrain.common.errors import (
    E_CONFIG_INVALID, E_GEO_INVALID, E_SCHEMA,
)


RID_RE = re.compile(r"^[a-z0-9_-]{1,32}$")
WAYPOINT_ID_RE = re.compile(r"^w-[a-z0-9_]{1,40}$")
ROUTE_ID_RE = re.compile(r"^r-[a-z0-9_]{1,40}$")


ENVELOPE_REQUIRED = frozenset({"v", "rid", "ts", "seq", "src", "data"})


ALLOWED_TASK_TYPES = frozenset({
    "GOTO_KEYPOINT",
    "STOP_TASK",
    "SET_ALARM_CONFIG",
    "AUDIO_CONTROL",
})


LEGACY_TASK_TYPES = frozenset({
    "INSPECTION_ROUTE", "FOLLOW_RECORDED_PATH", "PAUSE_TASK",
    "RESUME_TASK", "RETURN_HOME", "SET_GEOFENCE", "SET_KEYPOINTS",
    "SET_RECORDED_PATHS", "START_RECORDING", "STOP_RECORDING",
})


STOP_TASK_ACTIONS = frozenset({"pause", "resume", "cancel"})


SET_ALARM_REGION_KINDS = frozenset({"alarm_region"})
FORBIDDEN_REGION_KINDS = frozenset({"keep_in"})


@dataclass(frozen=True)
class InboundTask:
    """Translated task ready for the P3 admission queue. Carries a
    prefixed internal id so P3 code never has to know the id was
    external -- R10.4 分域 discipline."""
    internal_task_id: str
    client_task_id: str
    rid: str
    task_type: str
    payload: dict
    ts_utc_sec: float
    seq: int
    src: str
    msg_id: str


@dataclass(frozen=True)
class TranslateFailure:
    """A rejection produced by translate(); the caller renders it into
    cmd/task/ack {rejected: true, code, detail}."""
    code: str
    detail: dict


def _fail(code: str, **detail) -> TranslateFailure:
    return TranslateFailure(code=code, detail=dict(detail))


def _validate_envelope(msg: Any) -> Optional[TranslateFailure]:
    if not isinstance(msg, dict):
        return _fail(E_SCHEMA, kind="envelope_not_object")
    missing = ENVELOPE_REQUIRED - set(msg)
    if missing:
        return _fail(E_SCHEMA, kind="envelope_missing",
                      missing=sorted(missing))
    extras = set(msg) - ENVELOPE_REQUIRED - {"msg_id", "task_id"}
    if extras:
        return _fail(E_SCHEMA, kind="envelope_extras",
                      extras=sorted(extras))
    if msg.get("v") != 1:
        return _fail(E_SCHEMA, kind="envelope_v_mismatch",
                      got=msg.get("v"))
    return None


def _validate_rid(rid: Any, key_second_segment: str) -> Optional[TranslateFailure]:
    if not isinstance(rid, str) or not RID_RE.match(rid):
        return _fail(E_SCHEMA, kind="rid_shape", got=rid)
    if rid != key_second_segment:
        return _fail(E_SCHEMA, kind="rid_key_mismatch",
                      rid=rid, key_segment=key_second_segment)
    return None


def _validate_ts(ts: Any) -> Optional[TranslateFailure]:
    """R11.2: ts must be JSON number (float64), finite. NO ISO
    strings, NO integer milliseconds."""
    if isinstance(ts, bool) or not isinstance(ts, (int, float)):
        return _fail(E_SCHEMA, kind="ts_not_number", got=repr(ts))
    if not math.isfinite(float(ts)):
        return _fail(E_SCHEMA, kind="ts_not_finite", got=ts)
    return None


def _validate_seq(seq: Any) -> Optional[TranslateFailure]:
    """R1.7: seq is uint64; caller (dedupe layer) tracks rewinds --
    here we only validate shape."""
    if isinstance(seq, bool) or not isinstance(seq, int):
        return _fail(E_SCHEMA, kind="seq_not_int", got=repr(seq))
    if seq < 0 or seq > (1 << 64) - 1:
        return _fail(E_SCHEMA, kind="seq_out_of_range", got=seq)
    return None


def _validate_task_type(task_type: Any) -> Optional[TranslateFailure]:
    if not isinstance(task_type, str):
        return _fail(E_SCHEMA, kind="task_type_not_string", got=repr(task_type))
    if task_type in LEGACY_TASK_TYPES:
        return _fail(E_SCHEMA, kind="legacy_task_type", got=task_type)
    if task_type not in ALLOWED_TASK_TYPES:
        return _fail(E_SCHEMA, kind="task_type_unknown", got=task_type)
    return None


def _validate_goto_keypoint(payload: dict) -> Optional[TranslateFailure]:
    if payload.get("coordinate_system") != "WGS84":
        return _fail(E_SCHEMA, kind="coordinate_system_mismatch",
                      got=payload.get("coordinate_system"))
    ar = payload.get("arrival_radius_m")
    if not isinstance(ar, (int, float)) or isinstance(ar, bool):
        return _fail(E_SCHEMA, kind="arrival_radius_m_type", got=repr(ar))
    if not (0.5 <= float(ar) <= 10.0):
        return _fail(E_SCHEMA, kind="arrival_radius_m_range", got=ar)
    route_id = payload.get("recorded_path_id")
    if route_id is not None:
        if not isinstance(route_id, str) or not ROUTE_ID_RE.match(route_id):
            return _fail(E_SCHEMA, kind="route_id_shape", got=route_id)
    wps = payload.get("waypoints") or []
    if not isinstance(wps, list) or not wps:
        return _fail(E_SCHEMA, kind="waypoints_empty_or_bad_type")
    for i, wp in enumerate(wps):
        wid = wp.get("id") if isinstance(wp, dict) else None
        if not isinstance(wid, str) or not WAYPOINT_ID_RE.match(wid):
            return _fail(E_SCHEMA, kind="waypoint_id_shape",
                          index=i, got=wid)
    return None


def _validate_stop_task(payload: dict) -> Optional[TranslateFailure]:
    if "target_task_id" not in payload or not payload["target_task_id"]:
        return _fail(E_SCHEMA, kind="target_task_id_required")
    if payload.get("action") not in STOP_TASK_ACTIONS:
        return _fail(E_SCHEMA, kind="stop_action_closed_set",
                      got=payload.get("action"))
    return None


def _validate_set_alarm_config(payload: dict) -> Optional[TranslateFailure]:
    lvl = payload.get("alarm_level")
    if lvl not in (1, 2):
        return _fail(E_CONFIG_INVALID, kind="alarm_level_closed_set",
                      got=lvl)
    siren = payload.get("siren_level")
    if not isinstance(siren, (int, float)) or isinstance(siren, bool):
        return _fail(E_CONFIG_INVALID, kind="siren_level_type", got=repr(siren))
    if not (0 <= float(siren) <= 100):
        return _fail(E_CONFIG_INVALID, kind="siren_level_range", got=siren)
    dur = payload.get("duration_sec")
    if not isinstance(dur, (int, float)) or isinstance(dur, bool):
        return _fail(E_CONFIG_INVALID, kind="duration_sec_type", got=repr(dur))
    if not (1 <= float(dur) <= 20):
        return _fail(E_CONFIG_INVALID, kind="duration_sec_range", got=dur)
    cd = payload.get("cooldown_sec")
    if not isinstance(cd, (int, float)) or isinstance(cd, bool):
        return _fail(E_CONFIG_INVALID, kind="cooldown_sec_type", got=repr(cd))
    if not (0.5 <= float(cd) <= 600.0):
        return _fail(E_CONFIG_INVALID, kind="cooldown_sec_range", got=cd)
    for i, r in enumerate(payload.get("regions") or []):
        kind = r.get("kind") if isinstance(r, dict) else None
        if kind in FORBIDDEN_REGION_KINDS:
            return _fail(E_GEO_INVALID, kind="keep_in_forbidden",
                          index=i)
        if kind not in SET_ALARM_REGION_KINDS:
            return _fail(E_GEO_INVALID, kind="region_kind_closed_set",
                          index=i, got=kind)
    return None


_PAYLOAD_VALIDATORS = {
    "GOTO_KEYPOINT": _validate_goto_keypoint,
    "STOP_TASK":     _validate_stop_task,
    "SET_ALARM_CONFIG": _validate_set_alarm_config,
    "AUDIO_CONTROL": lambda p: None,   # shape gated by audio subsystem
}


def translate(msg: Any,
                key_second_segment: str) -> Any:
    """Return either an InboundTask (success) or a TranslateFailure.
    Never raises for a defective message -- the caller must be able to
    render every failure as an ack payload."""
    fail = _validate_envelope(msg)
    if fail is not None:
        return fail
    fail = _validate_rid(msg["rid"], key_second_segment)
    if fail is not None:
        return fail
    fail = _validate_ts(msg["ts"])
    if fail is not None:
        return fail
    fail = _validate_seq(msg["seq"])
    if fail is not None:
        return fail

    data = msg["data"]
    if not isinstance(data, dict):
        return _fail(E_SCHEMA, kind="data_not_object")
    task_type = data.get("task_type")
    fail = _validate_task_type(task_type)
    if fail is not None:
        return fail
    payload = data.get("payload") or {}
    if not isinstance(payload, dict):
        return _fail(E_SCHEMA, kind="payload_not_object")
    fail = _PAYLOAD_VALIDATORS[task_type](payload)
    if fail is not None:
        return fail

    client_task_id = msg.get("task_id") or data.get("task_id") or ""
    msg_id = msg.get("msg_id") or data.get("msg_id") or ""
    if not client_task_id:
        return _fail(E_SCHEMA, kind="client_task_id_required")
    return InboundTask(
        internal_task_id="ext:" + client_task_id,
        client_task_id=client_task_id,
        rid=msg["rid"],
        task_type=task_type,
        payload=payload,
        ts_utc_sec=float(msg["ts"]),
        seq=int(msg["seq"]),
        src=str(msg["src"]),
        msg_id=msg_id,
    )
