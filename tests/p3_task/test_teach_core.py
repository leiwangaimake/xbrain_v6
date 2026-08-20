"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_teach_core.py
Brief: Teach session machine + sampling + geometry validation (11 S12A, batch 4a)

Description:
The pure half of the recording subsystem: the S12A.3 machine, the seven arming
checks, the S12A.6 sampling rule and the S12A.7 geometry validation. All four
are decided before anything is stored, so they are tested without a database.

The cases that carry the batch:

  * arming check 7 (a non-voice e-stop channel must exist). Recording suppresses
    lateral avoidance AND voice e-stop, so this is the last remaining way to
    stop the robot; the test asserts the refusal, not just the pass.
  * sampling is WGS84 end to end. A degree-scale dedup threshold or an ENU
    metre leaking into a latitude is what this replaces, so there is a case
    pinning that 0.5 m means half a metre on the ground.
  * the (robot_outside, activate=true) pair is the one warn that becomes a
    refusal.

Each assertion names the mutation that reddens it.
"""
from __future__ import annotations

import pytest

from xbrain.common.enums import TEACH_ACTION, TEACH_STATE
from xbrain.common.errors import (
    E_BUSY, E_LOCKED, E_LOW_BATTERY, E_SCHEMA, E_TEACH_BUSY, E_TEACH_QUALITY,
    E_TEACH_STATE, E_UNHEALTHY,
)
from xbrain.p3_task.teach.command import (
    TeachCommandError, parse_teach_command, teach_ack,
)
from xbrain.p3_task.teach.sampling import (
    PoseSample, Recorder, fix_is_good_enough, haversine_m,
)
from xbrain.p3_task.teach.session import (
    ArmingInputs, TRANSITIONS, TeachSession, TeachStateError, check_arming,
    clamp_limits,
)
from xbrain.p3_task.teach.validate import (
    BLOCKING_ISSUES, merge_degenerate, ring_area_m2, ring_self_intersects,
    validate_fence, validate_route,
)

pytestmark = pytest.mark.no_device

# Reference point near the reseeded site origin. 1e-5 deg of latitude is about
# 1.11 m, which is what the sampling cases use to step a known distance.
_LAT, _LON = 34.6970, 135.5050
_DEG_PER_M_LAT = 1.0 / 111320.0


def _ok_arming(**over):
    base = dict(has_active_session=False, running_task_types=(),
                fix_type="rtk_fixed", allow_motion=True, hes_engaged=False,
                soc_pct=80.0, nonvoice_estop_source=True, estop_path_ok=True)
    base.update(over)
    return ArmingInputs(**base)


# ----------------------------------------------------------- arming gates ---

def test_arming_passes_when_everything_is_healthy():
    res = check_arming(_ok_arming())
    assert res.ok and res.code == "OK" and res.warn == ()


@pytest.mark.parametrize("over,code", [
    ({"has_active_session": True}, E_TEACH_BUSY),
    ({"running_task_types": ("patrol",)}, E_BUSY),
    ({"fix_type": "rtk_float"}, E_TEACH_QUALITY),
    ({"fix_type": None}, E_TEACH_QUALITY),
    ({"allow_motion": False}, E_UNHEALTHY),
    ({"hes_engaged": True}, E_LOCKED),
    ({"soc_pct": 10.0}, E_LOW_BATTERY),
    ({"soc_pct": None}, E_LOW_BATTERY),
])
def test_each_arming_check_refuses_with_its_own_code(over, code):
    """One case per S12A.3 row. MUTATION: drop any single check and its case
    goes green-to-red -- notably the fix_type one, whose absence lets a route be
    recorded at float accuracy and then driven as if it were surveyed."""
    res = check_arming(_ok_arming(**over))
    assert not res.ok and res.code == code


def test_arming_refuses_when_no_nonvoice_estop_channel_exists():
    """*** Check 7, the safety gate of this batch.

    While recording, lateral avoidance is suppressed and voice e-stop does not
    apply (U45), so the gamepad key or the HMI/cloud e-stop path is the only way
    left to stop the robot. MUTATION: delete this check (or demote it to a
    warning) and arming succeeds into a state with no reachable e-stop at all.
    """
    res = check_arming(_ok_arming(nonvoice_estop_source=False,
                                  estop_path_ok=False))
    assert not res.ok and res.code == E_UNHEALTHY
    assert res.detail["reason"] == "no_nonvoice_estop"
    # Both criteria are reported so the operator knows WHICH to restore.
    assert res.detail["checked"] == {"teleop_estop_source": False,
                                     "estop_path_ok": False}


@pytest.mark.parametrize("source,path", [(True, False), (False, True)])
def test_either_estop_channel_alone_is_enough(source, path):
    """The check is a disjunction (S12A.3 criterion 7: 'or'). MUTATION: make it
    a conjunction and recording becomes impossible whenever the HMI is closed,
    which would get the check disabled in the field."""
    assert check_arming(_ok_arming(nonvoice_estop_source=source,
                                   estop_path_ok=path)).ok


def test_missing_teleop_driver_is_a_warning_not_a_refusal():
    """Check 6 relaxes the DRIVE channel only: start the session, then pick up
    the controller. MUTATION: refuse here and a legitimate workflow breaks;
    relax check 7 the same way and nobody can stop the robot."""
    res = check_arming(_ok_arming(teleop_driver_online=False))
    assert res.ok and "no_driver" in res.warn


# ------------------------------------------------------- session machine ----

def test_state_values_match_the_closed_set():
    reachable = {s for s, _a in TRANSITIONS} | set(TRANSITIONS.values())
    assert reachable <= set(TEACH_STATE), (
        "a transition names a state outside the 11 S12A.3 closed set")


def test_pause_keeps_the_buffer_but_stops_sampling():
    """S12A.3: paused keeps the buffer intact and takes no points. That
    difference IS pause. MUTATION: let is_sampling() be true while paused and a
    robot parked mid-recording adds points nobody asked for."""
    s = TeachSession(session_id="ts-1", kind="route", state="recording")
    assert s.is_sampling()
    s.apply("pause")
    assert s.state == "paused" and not s.is_sampling()
    s.apply("resume")
    assert s.state == "recording" and s.is_sampling()


def test_illegal_action_raises_rather_than_no_op():
    """A teach command that does nothing sounds to the operator exactly like one
    that worked. MUTATION: return the current state instead of raising and a
    'finish' sent to an idle session is silently swallowed."""
    s = TeachSession(session_id="ts-1", kind="route", state="idle")
    with pytest.raises(TeachStateError) as ei:
        s.apply("finish")
    assert ei.value.code == E_TEACH_STATE


def test_discard_is_reachable_from_recording_and_paused_only():
    """S12A.3: discard is the ONLY entry to closed(discarded), and it is not
    available from finalizing-after-save or from idle."""
    for state in ("recording", "paused", "finalizing"):
        s = TeachSession(session_id="ts-1", kind="route", state=state)
        assert s.can("discard")
    assert not TeachSession(session_id="ts-1", kind="route",
                            state="idle").can("discard")


def test_clamped_limits_are_reported():
    """S12A.4: an out-of-range sample setting is clamped AND reported. MUTATION:
    clamp silently and the operator believes they are recording at 50 Hz."""
    limits, applied = clamp_limits({"sample_hz": 50.0, "min_dist_m": 0.0},
                                   max_duration_s=99999)
    assert limits.sample_hz == 5.0 and applied["sample_hz"] == 5.0
    assert limits.dedup_min_dist_m == 0.05
    assert limits.max_duration_s == 7200        # capped at the S12A.4 ceiling


def test_min_interval_ms_is_honoured_not_ignored():
    """S12A.4 still documents the sample block with min_interval_ms. A sender
    using the documented name must not be silently overridden by the default."""
    limits, _applied = clamp_limits({"min_interval_ms": 500}, None)
    assert limits.sample_hz == 2.0


# --------------------------------------------------------------- sampling ---

def test_haversine_matches_metres_on_the_ground():
    """The dedup threshold is in METRES, so the distance function has to be.
    MUTATION: use hypot on raw degrees -- 1e-5 deg would read as 1e-5 'metres',
    every sample would be a duplicate, and a recorded route would hold two
    points."""
    north = (_LAT + _DEG_PER_M_LAT, _LON)
    assert 0.99 < haversine_m((_LAT, _LON), north) < 1.01


def test_fix_ranking_is_an_ordering_not_a_membership_test():
    assert fix_is_good_enough("rtk_fixed", "rtk_float")
    assert not fix_is_good_enough("rtk_float", "rtk_fixed")
    assert not fix_is_good_enough(None, "rtk_fixed")
    # An unknown fix name is refused rather than raising: the sample is dropped
    # and counted, and a firmware that invents a name cannot stop a recording.
    assert not fix_is_good_enough("galactic", "rtk_fixed")


def _rec(**over):
    base = dict(dedup_min_dist_m=0.5, sample_hz=1.0, require_fix="rtk_fixed",
                max_points=2000)
    base.update(over)
    return Recorder(**base)


def _sample(metres_north, t, **over):
    base = dict(lat=_LAT + metres_north * _DEG_PER_M_LAT, lon=_LON,
                mono_s=t, fix_type="rtk_fixed")
    base.update(over)
    return PoseSample(**base)


def test_sampling_keeps_one_point_per_second_beyond_half_a_metre():
    r = _rec()
    assert r.offer(_sample(0.0, 100.0)) == (True, "kept")
    # Same second -> interval gate.
    assert r.offer(_sample(3.0, 100.4)) == (False, "interval")
    # A second later but 0.2 m away -> distance gate (the parked-robot case).
    assert r.offer(_sample(0.2, 101.0)) == (False, "distance")
    assert r.offer(_sample(3.0, 101.0)) == (True, "kept")
    assert r.point_count == 2 and r.dropped_by_distance == 1


def test_low_quality_samples_are_dropped_and_counted():
    """S12A.6: below require_fix the sample is discarded and counted, never
    stored. MUTATION: store it anyway and a float-quality point lands in a
    route that a patrol will later drive metre-for-metre."""
    r = _rec()
    assert r.offer(_sample(0.0, 100.0, fix_type="rtk_float")) == (False,
                                                                  "quality")
    assert r.point_count == 0 and r.dropped_by_quality == 1


def test_mark_points_bypass_both_gates_but_not_quality():
    """F05 exists so the operator can say 'this corner matters'. MUTATION: let
    a mark point through the distance gate only -- two marks in the same second
    then silently become one."""
    r = _rec()
    r.offer(_sample(0.0, 100.0))
    assert r.offer(_sample(0.1, 100.1, manual=True)) == (True, "kept")
    assert r.offer(_sample(0.15, 100.2, manual=True)) == (True, "kept")
    assert r.manual_count == 2
    assert r.offer(_sample(0.2, 100.3, manual=True,
                           fix_type="single")) == (False, "quality")


def test_undo_removes_points_and_recomputes_length():
    r = _rec()
    for i in range(4):
        r.offer(_sample(i * 2.0, 100.0 + i))
    before = r.length_m
    assert r.undo(2) == 2
    assert r.point_count == 2
    # Recomputed, not decremented: MUTATION: subtract the removed segments and
    # the quoted length drifts across a long session of marks and undos.
    assert r.length_m < before
    assert abs(r.length_m - 2.0) < 0.05
    # And the interval gate is re-armed so the next point can be taken now.
    assert r.offer(_sample(10.0, 100.0))[0] is True


def test_max_points_stops_the_recorder():
    r = _rec(max_points=2)
    r.offer(_sample(0.0, 100.0))
    r.offer(_sample(2.0, 101.0))
    assert r.offer(_sample(4.0, 102.0)) == (False, "full")


# ------------------------------------------------------------- validation ---

def _square(side_m=20.0):
    """A ring that is square ON THE GROUND, not in degrees.

    A degree of longitude at this latitude is only cos(34.7) of a degree of
    latitude, so equal degree offsets would give a rectangle 18% narrower than
    it is tall -- and the area case below would then be asserting the wrong
    number against correct code.
    """
    import math
    dlat = side_m * _DEG_PER_M_LAT
    dlon = dlat / math.cos(math.radians(_LAT))
    return [(_LAT, _LON), (_LAT + dlat, _LON), (_LAT + dlat, _LON + dlon),
            (_LAT, _LON + dlon)]


def test_route_needs_two_points():
    v = validate_route([(_LAT, _LON)])
    assert not v.ok and "too_few_points" in v.codes()


def test_fence_needs_three_vertices_and_area():
    v = validate_fence(_square()[:2])
    assert not v.ok and "too_few_vertices" in v.codes()
    tiny = validate_fence(_square(side_m=3.0))
    assert not tiny.ok and "area_too_small" in tiny.codes()


def test_self_intersecting_ring_is_blocked_with_the_edge_pair():
    """CMD-18. MUTATION: skip the crossing test -- a bow-tie fence is stored,
    and 'inside' is then undefined for the region the ray-cast disagrees on."""
    d = 20.0 * _DEG_PER_M_LAT
    bowtie = [(_LAT, _LON), (_LAT + d, _LON + d), (_LAT + d, _LON),
              (_LAT, _LON + d)]
    assert ring_self_intersects(bowtie) is not None
    v = validate_fence(bowtie)
    assert not v.ok and "self_intersect" in v.codes()
    issue = [i for i in v.issues if i["code"] == "self_intersect"][0]
    assert len(issue["edges"]) == 2


def test_a_plain_square_does_not_self_intersect():
    """The counter-case. MUTATION: count shared endpoints as crossings and
    EVERY polygon becomes self-intersecting -- an always-red check, which
    CLAUDE.md 3.2 form 2 says gets loosened until it catches nothing."""
    assert ring_self_intersects(_square()) is None
    assert validate_fence(_square()).ok


def test_area_is_in_square_metres():
    assert 380.0 < ring_area_m2(_square(side_m=20.0)) < 420.0


def test_auto_closed_versus_gap_too_large():
    """A recording rarely ends where it began. Within tolerance that is info;
    beyond it the operator must be TOLD the gap was forced closed."""
    square = _square()
    assert "auto_closed" in validate_fence(square, close_tol_m=50.0).codes()
    v = validate_fence(square, close_tol_m=1.0)
    assert "close_gap_large" in v.codes()
    assert v.ok, "a large gap warns, it does not block"


def test_robot_outside_blocks_only_when_activating():
    """*** S12A.7 fence constraint 3. Saving a fence you are standing outside of
    is allowed with a warning; ACTIVATING it in the same breath is not, because
    fence_guard (priority 1000) takes control immediately from a position it
    already considers a violation.

    MUTATION: treat robot_outside as a plain warning regardless of activate --
    this case goes red, and on the robot the fence engages with the machine
    already outside it.
    """
    outside = (_LAT - 0.01, _LON - 0.01)
    warn_only = validate_fence(_square(), robot_at=outside, activate=False)
    assert warn_only.ok and "robot_outside" in warn_only.codes()
    blocked = validate_fence(_square(), robot_at=outside, activate=True)
    assert not blocked.ok


def test_robot_inside_is_not_flagged():
    inside = (_LAT + 10.0 * _DEG_PER_M_LAT, _LON + 10.0 * _DEG_PER_M_LAT)
    assert "robot_outside" not in validate_fence(_square(),
                                                 robot_at=inside).codes()


def test_low_quality_ratio_warns_without_blocking():
    v = validate_route([(_LAT, _LON), (_LAT + 0.001, _LON)],
                       dropped_by_quality=10)
    assert v.ok and "low_quality_ratio" in v.codes()


def test_degenerate_edges_are_merged_keeping_both_ends():
    pts = [(_LAT, _LON),
           (_LAT + 0.1 * _DEG_PER_M_LAT, _LON),      # 0.1 m -- degenerate
           (_LAT + 5.0 * _DEG_PER_M_LAT, _LON),
           (_LAT + 5.05 * _DEG_PER_M_LAT, _LON)]     # 0.05 m from the previous
    merged = merge_degenerate(pts)
    assert merged[0] == pts[0] and merged[-1] == pts[-1]
    assert len(merged) == 3


def test_blocking_and_advisory_sets_do_not_overlap():
    """Guards the guard: a code in both sets would make the save decision
    depend on which set was consulted first."""
    from xbrain.p3_task.teach.validate import ADVISORY_ISSUES
    assert not (BLOCKING_ISSUES & ADVISORY_ISSUES)


# ---------------------------------------------------------------- command ---

def _teach(**over):
    base = {"cmd_id": "c-1", "action": "start",
            "issuer": {"src": "p4_agent", "channel": "local_voice"},
            "start": {"kind": "route"}}
    base.update(over)
    return base


def test_parse_start():
    c = parse_teach_command(_teach())
    assert c.action == "start" and c.start["kind"] == "route"
    assert c.start["require_fix"] == "rtk_fixed"


def test_session_id_required_except_for_start_and_mark_once():
    """S12A.4: a stale 'finish' must not land in a session it was not meant
    for. MUTATION: make session_id optional and the second recording gets
    finished by a command aimed at the first."""
    for action in ("finish", "save", "pause", "undo", "mark"):
        with pytest.raises(TeachCommandError):
            parse_teach_command(_teach(action=action, start=None))
    # start and mark_once mint / need none.
    parse_teach_command(_teach())
    parse_teach_command(_teach(
        action="mark_once", start=None,
        mark_once={"kind": "waypoint", "name": "east gate"}))


@pytest.mark.parametrize("payload", [
    {"action": "rotate"},                                    # off-set action
    {"issuer": {"channel": "local_voice"}},                  # no issuer.src
    {"start": {"kind": "waypoint"}},                         # session kind
    {"start": {"kind": "route", "require_fix": "single"}},   # quality floor
    {"start": {"kind": "route", "name_hint": "x" * 40}},     # 32-char cap
])
def test_envelope_refusals(payload):
    with pytest.raises(TeachCommandError) as ei:
        parse_teach_command(_teach(**payload))
    assert ei.value.code == E_SCHEMA


def test_mark_once_dock_requires_a_captured_heading():
    """S12A.8: a dock's handover orientation IS the captured heading, so a dock
    without one is not a dock. MUTATION: default it to false and F10 stores a
    charging dock the robot cannot line up with."""
    with pytest.raises(TeachCommandError):
        parse_teach_command(_teach(
            action="mark_once", start=None,
            mark_once={"kind": "dock", "name": "dock 1",
                       "capture_heading": False}))


def test_save_defaults_to_not_activating():
    """S12A.7 constraint 1: saving a fence is not enabling it. MUTATION: default
    activate to true and one voice command changes the safety boundary."""
    c = parse_teach_command(_teach(action="save", session_id="ts-1",
                                   start=None, save={"name": "north fence"}))
    assert c.save["activate"] is False and c.save["overwrite"] is False


def test_every_action_in_the_closed_set_parses():
    """Guards the guard: an action added to TEACH_ACTION with no parse branch
    would be refused as malformed by whichever field it lacks -- which reads as
    a sender bug rather than as an unimplemented action."""
    for action in TEACH_ACTION:
        payload = _teach(action=action, session_id="ts-1", start=None)
        if action == "start":
            payload["start"] = {"kind": "route"}
        if action == "mark_once":
            payload["mark_once"] = {"kind": "waypoint", "name": "n"}
        if action == "save":
            payload["save"] = {"name": "n"}
        assert parse_teach_command(payload).action == action


def test_teach_ack_shape():
    a = teach_ack("c-1", "accepted", "OK", {"session_id": "ts-1"})
    assert a["schema"] == "teach_ack_v1" and a["result"] == "accepted"
    assert a["detail"]["session_id"] == "ts-1"
