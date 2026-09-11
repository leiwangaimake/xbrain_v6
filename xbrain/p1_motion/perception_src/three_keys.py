"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: three_keys.py
Brief: p1 intake for the three perception keys -- envelope -> body -> DTO slots

Description:
The production side of the RNS consume face (11 S3.1B; 20 S3.1, #20-24). p1
subscribes xbrain/{rid}/rt/perception/{profile,objects,status} on the RT plane,
decodes each sample (11 S3.0 envelope, then the S3.1B body), turns it into the
frozen DTOs of rns/inputs.py and keeps the latest per key in a lock-guarded
slot. The 20 Hz tick reads latest() -> PerceptionSnapshot; RNS itself never
touches a session (RNS-M-4/M-5 -- that is why this file lives in
perception_src/, outside rns/, and why tests/p1_motion/rns/test_meta_no_zenoh
would redden if it were moved in).

What is decided here and why:
  * STRICT parsing, whole-message rejection. A body whose schema string, frame,
    array lengths, closed-set values, footprint size or PROF-1 (d_free <=
    d_block) is wrong is DROPPED and counted, never patched. The safe
    direction is absence: a missing profile ages into T-50/T-51 (limit, then
    zero speed) and missing objects into T-52 (all BLOCKED forbidden), whereas
    a "repaired" message would drive the robot on a lie (CLAUDE.md 3.5: an
    out-of-set value throws, it is never reinterpreted).
  * null stays null. d_free / d_block / h_block entries and t_seg_mono_ms are
    Optional in the DTO precisely so RNS-I-1 (unobserved = UNKNOWN, never 0 or
    range_max) is carried by the type; a parser that coerced None to a number
    would be the fail-silent shape 20 S3.1.3 warns about.
  * Latest-wins, no queue. Newer > complete for a control loop (19 P19-2 has
    the producer-side twin of this). Acceptance (strictly newer t_capture,
    duplicates never refreshing the age, epoch reset) is RNS's job
    (11 S3.1B.5 v2.1, source._accept); this slot only stores what arrived.
  * Rust-thread callbacks store only (CLAUDE.md 4.2): a threading.Lock around
    a dict assignment, no asyncio, no publish. Handles are held in a list
    (CLAUDE.md 4.3: a dropped declare_subscriber return unsubscribes on GC).

What looks right but is wrong: reading the S3.1B fields off the top-level
JSON. Every RT key rides in the S3.0 envelope, the body is under "data" --
the gnss bridge in runtime/main_wiring.py reads msg.get("data") for the same
reason. Also: do NOT compute ages here from envelope.mono; the contract ages
on the body's own t_capture_mono_ms (11 S3.1B.0 TIME-2), and that is done by
RNS per tick.
"""
from __future__ import annotations

import functools
import json
import logging
import threading
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from xbrain.common.envelope import EnvelopeSchemaError, decode
from xbrain.p1_motion.rns.inputs import (ObjectsMsg, PerceptionInput,
                                         PerceptionSnapshot, ProfileMsg,
                                         StatusMsg, TrackedObject)

_logger = logging.getLogger("xbrain.p1_motion.perception_src")

# 11 S3.1B schema strings: field add/remove bumps the version and the producer
# follows; an unknown string is rejected, never guessed (11 S3.0 rule).
PROFILE_SCHEMA = "perception_profile_v1"
OBJECTS_SCHEMA = "perception_objects_v1"
STATUS_SCHEMA = "perception_status_v1"
FRAME = "base_link"                       # 11 S3.1B.1/.2: the only legal frame
KEYS: Tuple[str, str, str] = ("profile", "objects", "status")
# closed sets of 11 S3.1B.2 (velocity_status five values adopted from the
# producer's implementation; the other two are two-valued by contract).
VELOCITY_STATUS = frozenset({"warming_up", "motion_unconfirmed", "moving",
                             "static", "timestamp_gap"})
VELOCITY_FRAME = frozenset({"ego_removed", "raw"})
SEMANTIC_STATUS = frozenset({"confirmed", "proxy"})
FOOTPRINT_MIN = 3                         # 11 S3.1B.2: 3 <= N <= 12, never less
FOOTPRINT_MAX = 12
SRC_MAX = 0b1111                          # four src bits (11 S3.1B.1)
CONF_MAX = 255


class PerceptionSchemaError(EnvelopeSchemaError):
    """A perception body that violates 11 S3.1B (wrong schema string, frame,
    length, closed-set value, footprint size or PROF-1). Same E_SCHEMA class
    as an envelope failure; a named subclass so an audit can tell body from
    envelope, and so the whole message is dropped (never repaired)."""


def key_expr(rid: str, key: str) -> str:
    """The full RT key for one of the three perception keys (11 S2.2.1)."""
    if key not in KEYS:
        raise ValueError("unknown perception key %r" % key)
    return "xbrain/%s/rt/perception/%s" % (rid, key)


# ── field readers: every violation raises PerceptionSchemaError ───────────────
def _is_num(v: Any) -> bool:
    # bool is an int in Python; "d_free": true must not parse as 1.0
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _num(body: Dict[str, Any], name: str, *, allow_null: bool = False,
         non_negative: bool = False) -> Optional[float]:
    if name not in body:
        raise PerceptionSchemaError("missing field %r (11 S3.1B)" % name)
    v = body[name]
    if v is None:
        if allow_null:
            return None
        raise PerceptionSchemaError("field %r must not be null" % name)
    if not _is_num(v):
        raise PerceptionSchemaError("field %r must be a number, got %r" % (name, v))
    if non_negative and v < 0:
        raise PerceptionSchemaError("field %r must be >= 0, got %r" % (name, v))
    return float(v)


def _int(body: Dict[str, Any], name: str, *, allow_null: bool = False) -> Optional[int]:
    if name not in body:
        raise PerceptionSchemaError("missing field %r (11 S3.1B)" % name)
    v = body[name]
    if v is None:
        if allow_null:
            return None
        raise PerceptionSchemaError("field %r must not be null" % name)
    if isinstance(v, bool) or not isinstance(v, int):
        raise PerceptionSchemaError("field %r must be an integer, got %r" % (name, v))
    return v


def _bool(body: Dict[str, Any], name: str) -> bool:
    if name not in body or not isinstance(body[name], bool):
        raise PerceptionSchemaError("field %r must be a boolean (11 S3.1B)" % name)
    return body[name]


def _str_in(body: Dict[str, Any], name: str, closed: Optional[frozenset] = None) -> str:
    v = body.get(name)
    if not isinstance(v, str):
        raise PerceptionSchemaError("field %r must be a string (11 S3.1B)" % name)
    if closed is not None and v not in closed:
        # CLAUDE.md 3.5: out-of-set -> throw, never "interpret" a degraded value
        raise PerceptionSchemaError(
            "field %r = %r outside the closed set %s" % (name, v, sorted(closed)))
    return v


def _array(body: Dict[str, Any], name: str, n: int) -> list:
    v = body.get(name)
    if not isinstance(v, list):
        raise PerceptionSchemaError("field %r must be an array (11 S3.1B.1)" % name)
    if len(v) != n:
        # TIME-1 / definedness: every per-bin array is exactly n_bins long; a
        # short array would silently shift bins against each other.
        raise PerceptionSchemaError(
            "array %r has %d entries, expected n_bins=%d" % (name, len(v), n))
    return v


def _num_or_null_array(body: Dict[str, Any], name: str, n: int) -> Tuple[Optional[float], ...]:
    out: List[Optional[float]] = []
    for i, e in enumerate(_array(body, name, n)):
        if e is None:
            out.append(None)                # RNS-I-1: null stays null
        elif _is_num(e):
            out.append(float(e))
        else:
            raise PerceptionSchemaError(
                "array %r[%d] must be a number or null, got %r" % (name, i, e))
    return tuple(out)


def _uint_array(body: Dict[str, Any], name: str, n: int, hi: int) -> Tuple[int, ...]:
    out: List[int] = []
    for i, e in enumerate(_array(body, name, n)):
        if isinstance(e, bool) or not isinstance(e, int) or not 0 <= e <= hi:
            raise PerceptionSchemaError(
                "array %r[%d] must be an integer in [0, %d], got %r" % (name, i, hi, e))
        out.append(e)
    return tuple(out)


def _schema(body: Any, expected: str) -> Dict[str, Any]:
    if not isinstance(body, dict):
        raise PerceptionSchemaError("body must be a JSON object (11 S3.1B)")
    got = body.get("schema")
    if got != expected:
        raise PerceptionSchemaError(
            "schema %r is not %r (11 S3.1B; unknown -> reject, never guess)"
            % (got, expected))
    return body


# ── body parsers (11 S3.1B.1 / .2 / .3) ──────────────────────────────────────
def parse_profile(body: Any) -> ProfileMsg:
    """11 S3.1B.1 ProfileMsg body -> DTO. Rejects (raises) on any structural or
    invariant violation; null entries are preserved (RNS-I-1); PROF-1 is
    checked per bin (a d_free beyond d_block declares UNKNOWN space FREE --
    the one error the whole contract exists to prevent)."""
    b = _schema(body, PROFILE_SCHEMA)
    if _str_in(b, "frame") != FRAME:
        raise PerceptionSchemaError("profile frame %r is not %r" % (b["frame"], FRAME))
    n = _int(b, "n_bins")
    if n is None or n <= 0:
        raise PerceptionSchemaError("n_bins must be a positive integer")
    d_free = _num_or_null_array(b, "d_free", n)
    d_block = _num_or_null_array(b, "d_block", n)
    h_block = _num_or_null_array(b, "h_block", n)
    for i in range(n):
        df, db = d_free[i], d_block[i]
        if df is not None and df < 0:
            raise PerceptionSchemaError("d_free[%d] < 0" % i)
        if df is not None and db is not None and df > db:
            raise PerceptionSchemaError(
                "PROF-1 violated at bin %d: d_free %.3f > d_block %.3f" % (i, df, db))
    return ProfileMsg(
        t_capture_mono_ms=_int(b, "t_capture_mono_ms"),
        t_publish_mono_ms=_int(b, "t_publish_mono_ms"),
        extrinsic_calibrated=_bool(b, "extrinsic_calibrated"),
        angle_min_rad=_num(b, "angle_min_rad"),
        angle_step_rad=_num(b, "angle_step_rad"),
        n_bins=n,
        range_max_m=_num(b, "range_max_m", non_negative=True),
        blind_near_m=_num(b, "blind_near_m", non_negative=True),
        z_pass_m=_num(b, "z_pass_m", non_negative=True),
        t_seg_mono_ms=_int(b, "t_seg_mono_ms", allow_null=True),
        d_free=d_free, d_block=d_block, h_block=h_block,
        src=_uint_array(b, "src", n, SRC_MAX),
        conf=_uint_array(b, "conf", n, CONF_MAX),
    )


def _parse_object(o: Any, idx: int) -> TrackedObject:
    if not isinstance(o, dict):
        raise PerceptionSchemaError("objects[%d] must be an object" % idx)
    fp_raw = o.get("footprint_xy")
    if not isinstance(fp_raw, list) or not FOOTPRINT_MIN <= len(fp_raw) <= FOOTPRINT_MAX:
        # 11 S3.1B.2: a hull is 3..12 points; the producer must send the
        # degenerate-case rectangle, never fewer points (nor an image box).
        raise PerceptionSchemaError(
            "objects[%d].footprint_xy must have %d..%d points"
            % (idx, FOOTPRINT_MIN, FOOTPRINT_MAX))
    fp: List[Tuple[float, float]] = []
    for p in fp_raw:
        if (not isinstance(p, (list, tuple)) or len(p) != 2
                or not _is_num(p[0]) or not _is_num(p[1])):
            raise PerceptionSchemaError("objects[%d].footprint_xy point malformed" % idx)
        fp.append((float(p[0]), float(p[1])))
    vel = o.get("velocity_xy")
    if (not isinstance(vel, (list, tuple)) or len(vel) != 2
            or not _is_num(vel[0]) or not _is_num(vel[1])):
        raise PerceptionSchemaError("objects[%d].velocity_xy malformed" % idx)
    z_min = _num(o, "z_min")
    z_max = _num(o, "z_max")
    if z_min is not None and z_max is not None and z_min > z_max:
        raise PerceptionSchemaError("objects[%d]: z_min > z_max" % idx)
    return TrackedObject(
        track_id=_int(o, "track_id"),
        class_name=_str_in(o, "class_name"),
        class_id=_int(o, "class_id"),
        confidence=_num(o, "confidence"),
        semantic_status=_str_in(o, "semantic_status", SEMANTIC_STATUS),
        footprint_xy=tuple(fp),
        z_min=z_min, z_max=z_max,
        r_near=_num(o, "r_near", non_negative=True),
        velocity_xy=(float(vel[0]), float(vel[1])),
        velocity_frame=_str_in(o, "velocity_frame", VELOCITY_FRAME),
        velocity_valid=_bool(o, "velocity_valid"),
        velocity_status=_str_in(o, "velocity_status", VELOCITY_STATUS),
        stable_frames=_int(o, "stable_frames"),
    )


def parse_objects(body: Any) -> ObjectsMsg:
    """11 S3.1B.2 ObjectsMsg body -> DTO. Any malformed object rejects the
    WHOLE message (a dropped person is the unsafe direction; an absent
    message ages into T-52's all-forbidden handling, the safe one)."""
    b = _schema(body, OBJECTS_SCHEMA)
    if _str_in(b, "frame") != FRAME:
        raise PerceptionSchemaError("objects frame %r is not %r" % (b["frame"], FRAME))
    objs = b.get("objects")
    if not isinstance(objs, list):
        raise PerceptionSchemaError("objects must be an array (empty is legal)")
    return ObjectsMsg(
        t_capture_mono_ms=_int(b, "t_capture_mono_ms"),
        extrinsic_calibrated=_bool(b, "extrinsic_calibrated"),
        objects=tuple(_parse_object(o, i) for i, o in enumerate(objs)),
    )


def parse_status(body: Any) -> StatusMsg:
    """11 S3.1B.3 StatusMsg body -> DTO (the fields RNS consumes; the
    statistics fields are read by the rx audit tool, not here)."""
    b = _schema(body, STATUS_SCHEMA)
    reasons = b.get("degraded_reasons")
    if not isinstance(reasons, list) or not all(isinstance(r, str) for r in reasons):
        raise PerceptionSchemaError("degraded_reasons must be an array of strings")
    ratio = _num(b, "invalid_pixel_ratio", non_negative=True)
    if ratio is not None and ratio > 1.0:
        raise PerceptionSchemaError("invalid_pixel_ratio > 1.0")
    return StatusMsg(
        t_publish_mono_ms=_int(b, "t_publish_mono_ms"),
        fps_depth=_num(b, "fps_depth", non_negative=True),
        fps_infer=_num(b, "fps_infer", non_negative=True),
        invalid_pixel_ratio=ratio,
        extrinsic_calibrated=_bool(b, "extrinsic_calibrated"),
        traversable_seg_available=_bool(b, "traversable_seg_available"),
        degraded_reasons=tuple(reasons),
    )


_PARSERS: Dict[str, Callable[[Any], Any]] = {
    "profile": parse_profile, "objects": parse_objects, "status": parse_status,
}


def parse_payload(key: str, raw: bytes) -> Any:
    """One wire sample -> DTO: JSON -> 11 S3.0 envelope -> body under "data"
    -> the key's body parser. Raises EnvelopeSchemaError (or its
    PerceptionSchemaError subclass) / ValueError; the caller drops + counts."""
    doc = json.loads(raw.decode("utf-8"))
    env = decode(doc)                       # 11 S3.0: v / rid / ts / seq / src / data
    return _PARSERS[key](env.data)


# ── the live intake ──────────────────────────────────────────────────────────
class ZenohPerceptionInput(PerceptionInput):
    """Production PerceptionInput: three RT subscriptions, latest-wins slots.
    Construct, then declare(session) once the RT session exists; the tick loop
    calls latest(now). stats() exposes received/rejected counts per key for
    the heartbeat log and the rx audit -- a rejected stream is visible as a
    rising count, not as silence."""

    def __init__(self, rid: str) -> None:
        if not rid:
            raise ValueError("rid required to form xbrain/{rid}/rt/perception/* keys")
        self._rid = rid
        self._lock = threading.Lock()
        self._slots: Dict[str, Any] = {k: None for k in KEYS}
        self._subs: List[Any] = []            # strong refs (CLAUDE.md 4.3)
        self._stats: Dict[str, Dict[str, Any]] = {
            k: {"received": 0, "rejected": 0, "last_error": None} for k in KEYS}

    def declare(self, session: Any) -> None:
        """Declare the three subscribers on the RT session. Handles land in a
        list (the CI static rule requires list.append / self.x = / registry)."""
        for key in KEYS:
            self._subs.append(session.declare_subscriber(
                key_expr(self._rid, key), functools.partial(self.on_sample, key)))

    def on_sample(self, key: str, sample: Any) -> None:
        """RUST THREAD (CLAUDE.md 4.2): decode, store, count. No await, no
        publish, no queue. A schema violation drops the message and is counted
        (first and every 100th logged so a bad producer is visible without
        flooding the log at 30 Hz)."""
        try:
            dto = parse_payload(key, bytes(sample.payload))
        except (EnvelopeSchemaError, ValueError, UnicodeDecodeError) as exc:
            with self._lock:
                st = self._stats[key]
                st["rejected"] += 1
                st["last_error"] = str(exc)
                n = st["rejected"]
            if n == 1 or n % 100 == 0:
                _logger.warning("p1 perception %s rejected (n=%d): %s", key, n, exc)
            return
        with self._lock:
            self._slots[key] = dto
            self._stats[key]["received"] += 1

    def latest(self, now_mono_ms: int) -> PerceptionSnapshot:
        """The tick's view: whatever arrived last per key (None = never).
        Ageing / acceptance is RNS's (source._accept), not this slot's."""
        with self._lock:
            return PerceptionSnapshot(profile=self._slots["profile"],
                                      objects=self._slots["objects"],
                                      status=self._slots["status"])

    def stats(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            return {k: dict(v) for k, v in self._stats.items()}

    def undeclare(self) -> None:
        for s in self._subs:
            try:
                s.undeclare()
            except Exception:      # noqa: BLE001 -- teardown must not raise
                pass
        self._subs = []


__all__ = ["KEYS", "PROFILE_SCHEMA", "OBJECTS_SCHEMA", "STATUS_SCHEMA",
           "PerceptionSchemaError", "ZenohPerceptionInput", "key_expr",
           "parse_payload", "parse_profile", "parse_objects", "parse_status"]
