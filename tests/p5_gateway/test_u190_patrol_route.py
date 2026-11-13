"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u190_patrol_route.py
Brief: 99 U190 -- the cloud PATROL_ROUTE task: patrol a whole recorded path, field checks, P3 round trip

Description:
User ruling 2026-11-13 ("按建议做"): a structured cloud command for patrolling a
whole recorded path, because GOTO_KEYPOINT only follows a path through its
waypoints to the last one (99 U164). Each case names the mutant it kills:
  * the internal shape: submit, type patrol, route_id from recorded_path_id,
    loops / round_trip in params.slots as sent, task_id kept (the cloud's
    correlation key), source cloud;
  * the field checks: recorded_path_id required and well formed, loops an
    integer >= 0 (0 = standing patrol), round_trip a bool, no other field;
  * the gateway's output survives P3's own parser and lap counter -- the
    two halves tested together, since each alone can be green while the
    seam is wrong (the GOTO_KEYPOINT route_id lesson of 2026-09-02).
"""
from __future__ import annotations  # postponed annotations

import pytest  # marker, parametrize

from xbrain.p3_task.ingest.task_command import parse_task_command  # P3 admission parser
from xbrain.p3_task.ingest.task_row import task_row_from_command  # P3 row builder
from xbrain.p3_task.route.laps import lap_spec_from_params  # P3 lap counter
from xbrain.p5_gateway.inbound.cloud_inbound import InboundReject  # the refusal
from xbrain.p5_gateway.inbound.task_router import KEY_TASK, route  # under test

pytestmark = pytest.mark.no_device  # pure in-process


def _data(payload, task_id="t-cloud-0042"):  # one v2.0 data object
    return {"msg_id": "m-77", "task_id": task_id, "task_type": "PATROL_ROUTE",  # envelope ids
            "payload": payload}  # the part under test


def _reject(payload):  # the refusal fields, or None when accepted
    try:
        route(_data(payload))  # the gateway's decision
    except InboundReject as exc:  # refused at the gateway
        return exc.fields  # error_code / reason / detail
    return None  # accepted


def test_patrol_route_becomes_an_internal_patrol_submit():
    """MUTATION: type "goto" / route_id dropped / task_id not passed /
    slots not filled -> red."""
    (key, body), = route(_data({"recorded_path_id": "r-004", "loops": 3,
                                "round_trip": True}))  # one command
    assert key == KEY_TASK and body["action"] == "submit" and body["source"] == "cloud"
    task = body["task"]  # the 11 S7.2 task
    assert task == {  # the whole shape, so an extra or missing key is red
        "task_id": "t-cloud-0042", "type": "patrol", "route_id": "r-004",
        "params": {"slots": {"loops": 3, "round_trip": True}}}
    assert body["cmd_id"] == "m-77"  # the ack correlates back to msg_id


def test_absent_options_are_left_to_p3():
    """Only what the cloud sent goes into slots; P3 owns the default (one
    pass). MUTATION: the gateway writes its own loops default -> red."""
    (_k, body), = route(_data({"recorded_path_id": "r-004"}))  # path only
    assert body["task"]["params"] == {"slots": {}}


def test_loops_zero_is_a_standing_patrol():
    """User approved loops = 0. MUTATION: loops >= 1 enforced -> red."""
    assert _reject({"recorded_path_id": "r-004", "loops": 0}) is None  # accepted: ends only by cancel


@pytest.mark.parametrize("payload, field", [
    ({"recorded_path_id": "r-004", "loops": -1}, "loops"),
    ({"recorded_path_id": "r-004", "loops": 1.5}, "loops"),
    ({"recorded_path_id": "r-004", "loops": True}, "loops"),
    ({"recorded_path_id": "r-004", "loops": "2"}, "loops"),
    ({"recorded_path_id": "r-004", "round_trip": "yes"}, "round_trip"),
    ({"recorded_path_id": "r-004", "round_trip": 1}, "round_trip"),
    ({"recorded_path_id": "r-004", "laps": 2}, "laps"),
    ({"recorded_path_id": "w-004"}, "recorded_path_id"),
])
def test_bad_fields_are_1003_naming_the_field(payload, field):
    """MUTATION: any of the three checks or the closed field set removed -> red."""
    f = _reject(payload)  # refused
    assert f is not None and f["error_code"] == 1003  # E_INVALID_FIELD
    assert f["detail"].get("field") == field  # which one


def test_missing_path_is_1002():
    """MUTATION: recorded_path_id optional -> a patrol of nothing -> red."""
    f = _reject({"loops": 1})  # no path
    assert f is not None and f["error_code"] == 1002  # E_REQUIRED_FIELD


def test_the_gateway_output_survives_p3_admission_and_lap_count():
    """End to end with P3's own parser and lap counter: a 2-loop round trip
    on an open path is 4 passes. MUTATION: slots under another key -> P3
    reads the default 1 pass -> red."""
    (_k, body), = route(_data({"recorded_path_id": "r-004", "loops": 2, "round_trip": True}))
    cmd = parse_task_command(body)  # raises on a shape P3 refuses
    assert cmd.task_id == "t-cloud-0042"  # the cloud's id is the task id
    row = task_row_from_command(cmd, submit_seq=1, now_mono_ms=1000,
                                created_at="2026-11-13T00:00:00Z")  # the stored row
    assert row.task_type == "patrol" and row.route_geo_id == "r-004"
    spec = lap_spec_from_params(cmd.task["params"])  # what dispatch will run
    assert spec.passes("open") == 4 and spec.passes("closed") == 2
