"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_field_validate.py
Brief: v2.0 入站字段级校验判据 (审计 B-2/B-3/C-2)

Description:
2026-08-24 审计发现: v2.0 S2.1/S2.4 的字段范围/闭集/ID 正则[两边都没校验]
(网关只拆分, p3 不认这些 v2.0 字段). 本文件守补上的那层.

*** 每条约束都要有一个[越界必红]的用例, 不能只测合法值通过.
一个"validate_goto 里什么都不查"的空实现能让所有合法值通过 -- 只有注入
非法值(arrival_radius_m=100)看它变红, 才证明校验真的在做(CLAUDE.md 3.3).

*** error_code 分级(v2.0 S10): 缺失 1002, 范围/枚举 1003.
一个把两者混成一个码的实现会让 Qt 分不清"漏填了字段"和"填了非法值",
而这两种要操作员做的事不同(补字段 vs 改值).
"""
from __future__ import annotations

import copy

import pytest

pytestmark = pytest.mark.no_device


def _reject_fields(fn, payload):
    from xbrain.p5_gateway.inbound.cloud_inbound import InboundReject

    try:
        fn(payload)
    except InboundReject as exc:
        return exc.fields
    return None


# --- 合法基线 ---------------------------------------------------------

_GOOD_GOTO = {
    "coordinate_system": "WGS84",
    "recorded_path_id": "r-route_north",
    "waypoints": [{
        "id": "w-north_gate", "name": "北门",
        "latitude": 31.2301971, "longitude": 121.4732683,
        "altitude": 8.4, "arrival_radius_m": 3.0,
    }],
}

_GOOD_ALARM = {
    "alarm_level": 1, "siren_level": 70, "duration_sec": 5,
    "cooldown_sec": 2.0,
    "alarm_window": {"start": "22:00", "end": "05:00"},
    "rules": [{"type": "person_in_region", "enabled": True,
               "alarm_role": "include", "applies_to": ["person"],
               "region_ids": ["f-x"]}],
    "regions": [{"id": "f-x", "op": "upsert", "base_rev": 0, "name": "设备区",
                 "type": "alarm_region", "enabled": True,
                 "applies_to": ["person"],
                 "vertices": [{"latitude": 31.0, "longitude": 121.0},
                              {"latitude": 31.1, "longitude": 121.0},
                              {"latitude": 31.1, "longitude": 121.1}]}],
}


def test_a_fully_legal_goto_passes():
    from xbrain.p5_gateway.inbound.field_validate import validate_goto

    validate_goto(copy.deepcopy(_GOOD_GOTO))       # 不抛即通过


def test_a_fully_legal_alarm_passes():
    from xbrain.p5_gateway.inbound.field_validate import validate_alarm

    validate_alarm(copy.deepcopy(_GOOD_ALARM))


# --- B-2 GOTO 字段 ----------------------------------------------------

def test_coordinate_system_must_be_wgs84_uppercase():
    """*** 只允许 "WGS84"(大写). 小写 wgs84 或 GCJ02 都拒.

    坐标系搞错会让机器人按错误的椭球去解经纬度, 到达点整体偏移.

    MUTATION: validate_goto 里删掉 coordinate_system 的 _enum -> 这里红.
    """
    from xbrain.p5_gateway.inbound.field_validate import validate_goto

    for bad in ("wgs84", "GCJ02", "WGS-84", ""):
        f = _reject_fields(validate_goto, dict(_GOOD_GOTO,
                                               coordinate_system=bad))
        assert f and f["error_code"] == 1003, bad


def test_arrival_radius_range_and_no_silent_default():
    """*** arrival_radius_m 必填且 0.5..10.0. 缺失不补默认(v2.0 S2.1 逐字).

    缺失 -> 1002(必填缺失); 越界 -> 1003(范围). 一个在缺失时补 1.0 的实现
    会让 Qt 以为自己设的半径生效了, 而实际用的是后端默认.

    MUTATION: 把 _range(ar, 0.5, 10.0) 改成 (0.0, 100.0) -> 越界那条红.
    """
    from xbrain.p5_gateway.inbound.field_validate import validate_goto

    # 缺失 -> 1002
    wp = dict(_GOOD_GOTO["waypoints"][0])
    del wp["arrival_radius_m"]
    f = _reject_fields(validate_goto, dict(_GOOD_GOTO, waypoints=[wp]))
    assert f and f["error_code"] == 1002, "缺 arrival_radius_m 应 1002"

    # 越界 -> 1003
    for bad in (0.4, 10.1, 100.0, -1.0):
        wp = dict(_GOOD_GOTO["waypoints"][0], arrival_radius_m=bad)
        f = _reject_fields(validate_goto, dict(_GOOD_GOTO, waypoints=[wp]))
        assert f and f["error_code"] == 1003, bad


def test_waypoint_id_must_carry_the_w_prefix():
    """waypoint id 必须 w-[a-z0-9_]{1,40}. 前缀错会让机器人按错的 ID 找路点."""
    from xbrain.p5_gateway.inbound.field_validate import validate_goto

    for bad in ("north_gate", "r-north", "W-north", "w-北门"):
        wp = dict(_GOOD_GOTO["waypoints"][0], id=bad)
        f = _reject_fields(validate_goto, dict(_GOOD_GOTO, waypoints=[wp]))
        assert f and f["error_code"] == 1003, bad


def test_recorded_path_id_must_carry_the_r_prefix():
    from xbrain.p5_gateway.inbound.field_validate import validate_goto

    for bad in ("route_north", "w-route", "p-9"):
        f = _reject_fields(validate_goto, dict(_GOOD_GOTO,
                                               recorded_path_id=bad))
        assert f and f["error_code"] == 1003, bad


def test_waypoint_level_recorded_path_id_is_rejected():
    """*** v2.0 S2.1 逐字: 不得接受 waypoint 级 recorded_path_id.

    路径 ID 是任务级的; 放到 waypoint 上意味着一条路径里每个点各自指一条
    录制路径 -- 那不是 v2.0 的模型, 静默接受会让语义悄悄变样.
    """
    from xbrain.p5_gateway.inbound.field_validate import validate_goto

    wp = dict(_GOOD_GOTO["waypoints"][0], recorded_path_id="r-x")
    f = _reject_fields(validate_goto, dict(_GOOD_GOTO, waypoints=[wp]))
    assert f and f["error_code"] == 1003


def test_empty_waypoints_is_rejected():
    from xbrain.p5_gateway.inbound.field_validate import validate_goto

    f = _reject_fields(validate_goto, dict(_GOOD_GOTO, waypoints=[]))
    assert f and f["error_code"] == 1003


def test_latitude_longitude_out_of_wgs84_range():
    """经纬度越界(v2.0 S1.3: lat[-90,90], lon[-180,180])."""
    from xbrain.p5_gateway.inbound.field_validate import validate_goto

    for field, bad in (("latitude", 91.0), ("latitude", -91.0),
                       ("longitude", 181.0), ("longitude", -181.0)):
        wp = dict(_GOOD_GOTO["waypoints"][0])
        wp[field] = bad
        f = _reject_fields(validate_goto, dict(_GOOD_GOTO, waypoints=[wp]))
        assert f and f["error_code"] == 1003, (field, bad)


def test_a_boolean_is_not_a_number():
    """*** True 不是合法经纬度. bool 是 int 的子类, 不显式排除会当 1.0 混过.

    MUTATION: _number 里去掉 isinstance(bool) 排除 -> 这里红.
    """
    from xbrain.p5_gateway.inbound.field_validate import validate_goto

    wp = dict(_GOOD_GOTO["waypoints"][0], latitude=True)
    f = _reject_fields(validate_goto, dict(_GOOD_GOTO, waypoints=[wp]))
    assert f and f["error_code"] == 1003


# --- B-3 ALARM 字段 ---------------------------------------------------

def test_alarm_scalar_ranges():
    """*** 报警标量范围逐个越界(v2.0 S2.4).

    alarm_level 1|2, siren_level 0..100, duration_sec 1..20, cooldown_sec
    0.5..600.0. 一个不查范围的实现会让 siren_level=200 或 duration_sec=0
    落进机内, 而那些值在硬件上要么无意义要么危险.
    """
    from xbrain.p5_gateway.inbound.field_validate import validate_alarm

    cases = [("alarm_level", 3), ("alarm_level", 0),
             ("siren_level", 101), ("siren_level", -1),
             ("duration_sec", 0), ("duration_sec", 21),
             ("cooldown_sec", 0.4), ("cooldown_sec", 600.1)]
    for field, bad in cases:
        f = _reject_fields(validate_alarm, dict(_GOOD_ALARM, **{field: bad}))
        assert f and f["error_code"] == 1003, (field, bad)


def test_alarm_window_must_be_hhmm():
    """alarm_window.start/end 必须 HH:mm(v2.0 S2.4). 允许跨午夜(start>end)."""
    from xbrain.p5_gateway.inbound.field_validate import validate_alarm

    for bad in ("2200", "22:60", "24:00", "22:00:00", "10 pm"):
        f = _reject_fields(validate_alarm,
                           dict(_GOOD_ALARM, alarm_window={"start": bad,
                                                           "end": "05:00"}))
        assert f and f["error_code"] == 1003, bad
    # 跨午夜合法(22:00 -> 05:00, start>end).
    validate_alarm(copy.deepcopy(_GOOD_ALARM))


def test_rule_type_and_role_closed_sets():
    """rules[].type / alarm_role 闭集(v2.0 S2.4)."""
    from xbrain.p5_gateway.inbound.field_validate import validate_alarm

    bad_type = copy.deepcopy(_GOOD_ALARM)
    bad_type["rules"][0]["type"] = "animal_in_region"
    assert _reject_fields(validate_alarm, bad_type)["error_code"] == 1003

    bad_role = copy.deepcopy(_GOOD_ALARM)
    bad_role["rules"][0]["alarm_role"] = "maybe"
    assert _reject_fields(validate_alarm, bad_role)["error_code"] == 1003


def test_enabled_rule_needs_nonempty_region_ids():
    """*** 启用的规则 region_ids 必须非空(v2.0 S2.4 逐字).

    一条启用却不指向任何区域的规则永远不会命中 -- 操作员以为设了防护,
    实际什么都没保护.
    """
    from xbrain.p5_gateway.inbound.field_validate import validate_alarm

    bad = copy.deepcopy(_GOOD_ALARM)
    bad["rules"][0]["region_ids"] = []
    assert _reject_fields(validate_alarm, bad)["error_code"] == 1003


def test_region_op_closed_set_and_id_prefix():
    """regions[].op 闭集 upsert|delete|set_state; id 必须 f- 前缀."""
    from xbrain.p5_gateway.inbound.field_validate import validate_alarm

    bad_op = copy.deepcopy(_GOOD_ALARM)
    bad_op["regions"][0]["op"] = "replace"
    assert _reject_fields(validate_alarm, bad_op)["error_code"] == 1003

    bad_id = copy.deepcopy(_GOOD_ALARM)
    bad_id["regions"][0]["id"] = "region-x"
    assert _reject_fields(validate_alarm, bad_id)["error_code"] == 1003


def test_upsert_region_needs_at_least_three_vertices():
    """*** 网关只查顶点数>=3(纯结构); 自交/面积由机器人端权威校验.

    v2.0 S2.4: 顶点至少 3 个. 一个 2 顶点的"多边形"根本不是面, 这条网关
    能查; 但顶点是否自交/面积是否非零要 geo 几何, 留给机器人端.
    """
    from xbrain.p5_gateway.inbound.field_validate import validate_alarm

    bad = copy.deepcopy(_GOOD_ALARM)
    bad["regions"][0]["vertices"] = [{"latitude": 31.0, "longitude": 121.0},
                                     {"latitude": 31.1, "longitude": 121.0}]
    assert _reject_fields(validate_alarm, bad)["error_code"] == 1003


def test_delete_region_needs_only_id_and_base_rev():
    """delete op 只要 id + base_rev, 不要 vertices/name(v2.0 S2.4 op 表)."""
    from xbrain.p5_gateway.inbound.field_validate import validate_alarm

    d = copy.deepcopy(_GOOD_ALARM)
    d["regions"] = [{"id": "f-old", "op": "delete", "base_rev": 3}]
    validate_alarm(d)                              # 不抛即通过


# --- C-2 ID 正则 ------------------------------------------------------

def test_msg_id_and_task_id_regex():
    """*** v2.0 S1.2: msg_id/task_id 匹配 [A-Za-z0-9][A-Za-z0-9._:-]{0,127}.

    带目录分隔符或空格的 id 会被机内当幂等键/关联键用 -- 格式不对关联对不上.

    MUTATION: check_ids 里去掉 _regex, 只留非空检查 -> 这里红.
    """
    from xbrain.p5_gateway.inbound.field_validate import check_ids

    check_ids({"msg_id": "msg-001", "task_id": "task-abc.1:2"})  # 合法
    for bad_data in (
            {"msg_id": "msg/001", "task_id": "t"},     # 分隔符
            {"msg_id": "-lead", "task_id": "t"},       # 首字符非字母数字
            {"msg_id": "m id", "task_id": "t"},        # 空格
            {"msg_id": "x" * 129, "task_id": "t"}):    # 超长
        from xbrain.p5_gateway.inbound.cloud_inbound import InboundReject
        try:
            check_ids(bad_data)
            assert False, "非法 id 未被拒: %s" % bad_data
        except InboundReject as exc:
            assert exc.fields["error_code"] in (1002, 1003)


def test_missing_id_is_1002_not_1003():
    """缺失 id -> 1002(必填缺失), 与格式错(1003)分开."""
    from xbrain.p5_gateway.inbound.cloud_inbound import InboundReject
    from xbrain.p5_gateway.inbound.field_validate import check_ids

    try:
        check_ids({"msg_id": "m-1"})               # 缺 task_id
        assert False
    except InboundReject as exc:
        assert exc.fields["error_code"] == 1002
