"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_three_keys.py
Brief: three-key intake -- strict body parsing + latest-wins slots (11 S3.1B)

Description:
Guards perception_src/three_keys.py, the production intake of the RNS consume
face (#20-24): a valid wire sample decodes into the frozen DTOs with nulls
preserved (RNS-I-1); a wrong schema string, wrong frame, short array, out-of-set
closed value, undersized footprint or a PROF-1 violation rejects the WHOLE
message and is counted (never repaired); handles are held (CLAUDE.md 4.3);
slots are latest-wins. Each test names the mutant that reddens it.
"""
from __future__ import annotations

import json

import pytest

from xbrain.p1_motion.perception_src.three_keys import (
    OBJECTS_SCHEMA, PROFILE_SCHEMA, STATUS_SCHEMA, PerceptionSchemaError,
    ZenohPerceptionInput, key_expr, parse_objects, parse_payload, parse_profile,
    parse_status,
)

pytestmark = pytest.mark.no_device

N = 5


def profile_body(**over):
    b = {
        "schema": PROFILE_SCHEMA, "t_capture_mono_ms": 1000,
        "t_publish_mono_ms": 1030, "frame": "base_link",
        "extrinsic_calibrated": True, "pose_used": None,
        "t_seg_mono_ms": 990, "z_pass_m": 0.75, "angle_min_rad": -0.785,
        "angle_step_rad": 0.3925, "n_bins": N, "range_max_m": 6.0,
        "blind_near_m": 0.59,
        "d_free": [6.0, None, 2.75, 0.59, 6.0],
        "d_block": [None, None, 3.0, 2.5, None],
        "h_block": [None, None, 0.4, None, None],
        "src": [3, 0, 3, 6, 3], "conf": [220, 0, 220, 90, 220],
        "terrain": [0] * N, "slope_deg": [None] * N,
    }
    b.update(over)
    return b


def object_body(**over):
    o = {
        "track_id": 41, "class_name": "person", "class_id": 0,
        "confidence": 0.91, "semantic_status": "confirmed",
        "footprint_xy": [[2.0, -0.3], [2.6, 0.0], [2.0, 0.3]],
        "z_min": 0.02, "z_max": 1.78, "r_near": 1.9,
        "velocity_xy": [0.8, -0.1], "velocity_frame": "ego_removed",
        "velocity_valid": True, "velocity_status": "moving",
        "stable_frames": 37, "first_seen_mono_ms": 900,
        "last_seen_mono_ms": 1000, "depth_quality": "good",
    }
    o.update(over)
    return o


def objects_body(objs=None, **over):
    b = {"schema": OBJECTS_SCHEMA, "t_capture_mono_ms": 1000,
         "t_publish_mono_ms": 1030,       # 11 S3.1B.2 v2.2: the cadence stamp
         "frame": "base_link", "extrinsic_calibrated": True,
         "objects": [object_body()] if objs is None else objs}
    b.update(over)
    return b


def status_body(**over):
    b = {"schema": STATUS_SCHEMA, "t_publish_mono_ms": 1050,
         "fps_depth": 29.4, "fps_infer": 20.1, "latency_ms_p50": 120.0,
         "latency_ms_p99": 219.5, "infer_gap_ms_p99": 41.0,
         "infer_gap_ms_max": 62.0, "invalid_pixel_ratio": 0.07,
         "ground_seg_level": 2, "extrinsic_calibrated": True,
         "traversable_seg_available": True, "degraded_reasons": []}
    b.update(over)
    return b


def wire(body, seq=1):
    """A wire sample: the 11 S3.0 envelope around the S3.1B body."""
    env = {"v": 1, "rid": "dev", "ts": 1.0, "seq": seq, "src": "perception",
           "data": body}
    return json.dumps(env).encode("utf-8")


# ── parsers ──────────────────────────────────────────────────────────────────
def test_profile_parses_and_preserves_nulls():
    # RNS-I-1: None entries stay None. mutant: coerce None -> 0.0 in
    # _num_or_null_array -> reddens.
    p = parse_profile(profile_body())
    assert p.n_bins == N and p.d_free[1] is None and p.d_block[0] is None
    assert p.d_free[0] == 6.0 and p.h_block[2] == 0.4 and p.src[3] == 6
    assert p.t_seg_mono_ms == 990 and p.extrinsic_calibrated is True


def test_profile_wrong_schema_string_is_rejected():
    # 11 S3.0/S3.1B: unknown schema -> reject, never guess. mutant: drop the
    # schema comparison -> parses -> reddens.
    with pytest.raises(PerceptionSchemaError):
        parse_profile(profile_body(schema="perception_profile_v9"))


def test_profile_wrong_frame_is_rejected():
    with pytest.raises(PerceptionSchemaError):
        parse_profile(profile_body(frame="camera_link"))


def test_profile_short_array_is_rejected():
    # every per-bin array is exactly n_bins long; a short one shifts bins.
    # mutant: drop the length check -> reddens.
    with pytest.raises(PerceptionSchemaError):
        parse_profile(profile_body(d_block=[None, None, 3.0]))


def test_profile_prof1_violation_is_rejected():
    # PROF-1 / RNS-I-3: d_free > d_block declares UNKNOWN space FREE.
    # mutant: drop the per-bin check -> reddens.
    with pytest.raises(PerceptionSchemaError):
        parse_profile(profile_body(d_free=[6.0, None, 3.2, 0.59, 6.0]))


def test_profile_sentinel_types_are_rejected():
    # a boolean is not a number ("d_free": true must not read as 1.0); src
    # outside 0..15 is out of set.
    with pytest.raises(PerceptionSchemaError):
        parse_profile(profile_body(d_free=[True, None, 2.75, 0.59, 6.0]))
    with pytest.raises(PerceptionSchemaError):
        parse_profile(profile_body(src=[3, 0, 3, 16, 3]))


def test_objects_parses_and_empty_list_is_legal():
    m = parse_objects(objects_body())
    assert len(m.objects) == 1 and m.objects[0].class_name == "person"
    assert m.objects[0].footprint_xy[1] == (2.6, 0.0)
    assert parse_objects(objects_body(objs=[])).objects == ()


def test_objects_without_publish_stamp_is_rejected():
    # 11 S3.1B.2 v2.2: t_publish_mono_ms is the cadence measurement point and
    # the ledger join field; a producer omitting it cannot be audited. mutant:
    # default it in the parser -> parses -> reddens.
    body = objects_body()
    del body["t_publish_mono_ms"]
    with pytest.raises(PerceptionSchemaError):
        parse_objects(body)
    assert parse_objects(objects_body()).t_publish_mono_ms == 1030


def test_objects_undersized_footprint_rejects_whole_message():
    # 11 S3.1B.2: 3 <= N <= 12; a 2-point hull is the degenerate case the
    # producer must have turned into a rectangle. mutant: drop the size check
    # -> reddens.
    bad = object_body(footprint_xy=[[2.0, -0.3], [2.6, 0.0]])
    with pytest.raises(PerceptionSchemaError):
        parse_objects(objects_body(objs=[object_body(), bad]))


def test_objects_closed_sets_are_enforced():
    # CLAUDE.md 3.5: out-of-set throws. mutant: accept any string -> reddens.
    with pytest.raises(PerceptionSchemaError):
        parse_objects(objects_body(objs=[object_body(velocity_status="unknown")]))
    with pytest.raises(PerceptionSchemaError):
        parse_objects(objects_body(objs=[object_body(velocity_frame="odom")]))
    with pytest.raises(PerceptionSchemaError):
        parse_objects(objects_body(objs=[object_body(semantic_status="guess")]))


def test_status_parses_required_fields_and_ignores_stats():
    s = parse_status(status_body())
    assert s.invalid_pixel_ratio == 0.07 and s.traversable_seg_available is True
    assert s.degraded_reasons == ()
    with pytest.raises(PerceptionSchemaError):
        parse_status(status_body(invalid_pixel_ratio=1.5))
    with pytest.raises(PerceptionSchemaError):
        parse_status(status_body(degraded_reasons="infer_gap"))


def test_payload_needs_the_envelope():
    # every RT key rides in the 11 S3.0 envelope; a bare body is rejected.
    from xbrain.common.envelope import EnvelopeSchemaError
    with pytest.raises(EnvelopeSchemaError):
        parse_payload("profile", json.dumps(profile_body()).encode("utf-8"))
    p = parse_payload("profile", wire(profile_body()))
    assert p.t_capture_mono_ms == 1000


# ── the live intake with a fake session ──────────────────────────────────────
class _Sample:
    def __init__(self, payload: bytes) -> None:
        self.payload = payload


class _Handle:
    def __init__(self, key, cb) -> None:
        self.key, self.cb, self.undeclared = key, cb, False

    def undeclare(self) -> None:
        self.undeclared = True


class _FakeSession:
    def __init__(self) -> None:
        self.handles = []

    def declare_subscriber(self, key_expr_, cb):
        h = _Handle(key_expr_, cb)
        self.handles.append(h)
        return h

    def deliver(self, key, payload: bytes) -> None:
        for h in self.handles:
            if h.key.endswith("/" + key):
                h.cb(_Sample(payload))


def test_intake_declares_three_full_keys_and_holds_handles():
    # CLAUDE.md 4.3: a dropped declare_subscriber return unsubscribes on GC.
    # mutant: not appending the handle -> len(_subs) == 0 -> reddens.
    sess = _FakeSession()
    inp = ZenohPerceptionInput("gj-001")
    inp.declare(sess)
    assert [h.key for h in sess.handles] == [
        key_expr("gj-001", k) for k in ("profile", "objects", "status")]
    assert key_expr("gj-001", "profile") == "xbrain/gj-001/rt/perception/profile"
    assert len(inp._subs) == 3
    inp.undeclare()
    assert all(h.undeclared for h in sess.handles) and inp._subs == []


def test_intake_latest_wins_and_counts():
    # latest-wins: the second profile replaces the first. mutant: first-wins
    # (store only when the slot is empty) -> reddens.
    sess = _FakeSession()
    inp = ZenohPerceptionInput("dev")
    inp.declare(sess)
    assert inp.latest(0).profile is None
    sess.deliver("profile", wire(profile_body(t_capture_mono_ms=1000), seq=1))
    sess.deliver("profile", wire(profile_body(t_capture_mono_ms=1033), seq=2))
    sess.deliver("objects", wire(objects_body()))
    sess.deliver("status", wire(status_body()))
    snap = inp.latest(2000)
    assert snap.profile.t_capture_mono_ms == 1033
    assert snap.objects.t_capture_mono_ms == 1000
    assert snap.status.t_publish_mono_ms == 1050
    st = inp.stats()
    assert st["profile"]["received"] == 2 and st["profile"]["rejected"] == 0


def test_intake_rejects_and_counts_bad_samples_keeping_last_good():
    # a rejected sample never touches the slot (the last good frame stays and
    # AGES -- absence is the safe direction); the rejection is visible in
    # stats. mutant: store the DTO before validating -> reddens.
    sess = _FakeSession()
    inp = ZenohPerceptionInput("dev")
    inp.declare(sess)
    sess.deliver("profile", wire(profile_body(t_capture_mono_ms=1000)))
    sess.deliver("profile", b"not json at all")
    sess.deliver("profile", wire(profile_body(t_capture_mono_ms=1033,
                                              d_free=[9.0, None, 9.0, 9.0, 9.0])))  # PROF-1
    sess.deliver("objects", wire({"schema": "perception_objects_v1"}))
    snap = inp.latest(2000)
    assert snap.profile.t_capture_mono_ms == 1000
    assert snap.objects is None
    st = inp.stats()
    assert st["profile"]["rejected"] == 2 and "PROF-1" in st["profile"]["last_error"]
    assert st["objects"]["rejected"] == 1
