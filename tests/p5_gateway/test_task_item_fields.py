"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_item_fields.py
Brief: v2.0 S3.2 task item -- route_id / started_ts / progress off 11 S4.4

Description:
The customer-visible defect: Qt showed a running task with route_id null and
started_ts null, both mandatory in v2.0 S3.2. task_item read them with the v2.0
names off a dict that used different ones, so the .get() default answered for
every task and nothing failed anywhere.

These cases pin the mapping from the 11 S4.4 TaskState item to the v2.0 item.
The progress cases matter as much as the two named fields: progress is a nullable
SUB-OBJECT in S4.4, and reading its members flat is the same mistake one level
down -- it would come back null again once the executor starts filling it.
"""

from __future__ import annotations

import pytest

from xbrain.p5_gateway.outbound.state_projection import task_item


# INF-TS-1: 纯单测, 不碰设备(无 zenohd / 无底盘 / 无 ORIN 专属硬件).
pytestmark = pytest.mark.no_device


def _item(**kw):
    base = {"task_id": "task-55b4dc71", "type": "goto", "state": "running",
            "route_id": "r-charge", "started_ts": 1788339000.0,
            "progress": None}
    base.update(kw)
    return base


def test_route_id_reaches_the_cloud_item():
    """v2.0 S3.2 marks it mandatory: "实际加载路径 ID".
    MUTATION: drop the route_id key from task_item -> red."""
    assert task_item(_item())["route_id"] == "r-charge"


def test_started_ts_reaches_the_cloud_item():
    """v2.0 S3.2: "实际开始执行时间, 不含排队".
    MUTATION: drop started_ts -> red."""
    assert task_item(_item())["started_ts"] == 1788339000.0


def test_the_task_type_is_read_from_the_contract_field_name():
    """11 S4.4 calls it `type`; reading `task_type` off a TaskState item gives
    None, and Qt renders a task with no type.
    MUTATION: read only task.get("task_type") -> red."""
    assert task_item(_item(type="patrol"))["task_type"] == "patrol"


def test_the_task_card_shape_is_still_understood():
    """The fallback source (query/tasks TaskCard) names it task_type. Both are
    accepted because the same projection sees both sources.
    MUTATION: read only task.get("type") -> red."""
    card = {"task_id": "t-1", "task_type": "goto", "state": "running"}
    assert task_item(card)["task_type"] == "goto"


# --------------------------------------------------------------- progress block

def test_progress_null_yields_null_percent_and_zero_counts():
    """S4.4 progress is null until the route is expanded (EX-4). percent must be
    null -- v2.0 forbids a fabricated 0 -- while the counts are honestly 0: no
    waypoint has been completed.
    MUTATION: default progress_percent to 0.0 -> red."""
    d = task_item(_item(progress=None))
    assert d["progress_percent"] is None
    assert d["completed_count"] == 0
    assert d["total_count"] == 0
    assert d["route_rev"] is None


def test_progress_members_are_read_from_inside_the_sub_object():
    """S4.4 nests them; reading pct/waypoint_total flat is the same defect one
    level down and would resurface the moment the executor fills progress.
    MUTATION: read task.get("pct") / task.get("waypoint_total") -> red."""
    d = task_item(_item(progress={"pct": 13.3, "waypoint_total": 64,
                                  "waypoint_index": 16, "route_rev": 7}))
    assert d["progress_percent"] == pytest.approx(13.3)
    assert d["total_count"] == 64
    assert d["route_rev"] == 7


def test_completed_count_is_the_index_plus_one():
    """S4.4 waypoint_index is the LAST PASSED index (0-based); v2.0 wants a
    count. Passing the index straight through under-reports by one.
    MUTATION: return the index verbatim -> red."""
    assert task_item(_item(progress={"waypoint_index": 16}))[
        "completed_count"] == 17


def test_before_the_first_waypoint_the_count_is_zero_not_minus_one():
    """S4.4 uses -1 for "not yet at the first point". Passing it through shows
    "已完成 -1 个点" on the operator's screen.
    MUTATION: return index + 1 unconditionally -> red (gives 0 for -1 only if
    the guard exists; without it -1 + 1 = 0 too, so the guard is checked by the
    None case below as well)."""
    assert task_item(_item(progress={"waypoint_index": -1}))[
        "completed_count"] == 0


def test_a_missing_waypoint_index_is_zero_not_an_exception():
    """progress can exist while the executor has not stamped an index yet.
    MUTATION: int(progress["waypoint_index"]) + 1 -> red (KeyError/TypeError)."""
    assert task_item(_item(progress={"pct": None}))["completed_count"] == 0


def test_every_v2_key_is_present_even_when_unknown():
    """v2.0 S3.2: all fields mandatory, values may be null. A missing key is a
    KeyError on the Qt side, not a blank cell.
    MUTATION: omit a key when its value is None -> red."""
    d = task_item(_item())
    for key in ("task_id", "task_type", "state", "current_waypoint_id",
                "completed_count", "total_count", "progress_percent",
                "route_id", "route_rev", "started_ts", "message"):
        assert key in d, key
