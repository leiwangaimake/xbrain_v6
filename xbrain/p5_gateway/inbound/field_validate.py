"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: field_validate.py
Brief: v2.0 入站字段级校验 -- 范围/闭集/ID 正则 (审计 B-2/B-3/C-2 修复)

Description:
2026-08-24 审计发现: task_router 只做 task_type -> 机内 key 的拆分, 对 v2.0
[字段级]约束(coordinate_system=WGS84, arrival_radius_m 0.5..10.0, 报警标量
范围, ID 正则)完全不校验, 理由是"由 p3 既有解析器做". 但 p3 的解析器是
[语音链路]的, 只认 11 S7.2 的机内形状, 根本不认这些 v2.0 特有字段 -- 于是
两边都没校验, 非法值(arrival_radius_m=100, coordinate_system=GCJ02, alarm_
level=9)会被静默接受, 违反 v2.0 S2.1/S2.4/S1.3.

*** 为什么校验必须在网关做, 不能推给 p3.
这些字段(coordinate_system / arrival_radius_m / alarm_level ...)是 v2.0 协议
的, 机内根本没有它们 -- 网关是 v2.0 与机内之间的[唯一翻译点](评审 R1.3).
翻译点不校验, 非法的 v2.0 值就直接落进了机内 payload, 而机内解析器不认识
它们, 只会忽略 -- 一个 arrival_radius_m=100 的到达半径就这样被静默丢掉,
而 v2.0 S2.1 逐字禁止"缺失时静默补默认值".

*** error_code 分两级(v2.0 S10).
  1002 E_REQUIRED_FIELD  必填字段缺失
  1003 E_INVALID_FIELD   类型/范围/枚举错误
缺失走 envelope_error(1002), 范围/枚举走 build_error_fields(E_SCHEMA)->1003.
detail.code 两者都是我方原生 E_SCHEMA(S10: detail.code 是后端原生码, 由整数
码区分具体种类).

Boundaries: 只校验[能在网关本地判定]的: 类型/范围/闭集/ID 正则/必填. NO 不
做需要机器人端知识的校验 -- waypoint 是否在围栏内(要 geo.db), 顶点是否自交/
面积是否非零(S2.4 逐字"机器人端执行权威几何校验"), 都留给下游. 网关只查
顶点数 >= 3 这种纯结构的.
"""

from __future__ import annotations

import re
from typing import Any, Dict, List

from ...common import errors
from ..outbound.error_map import (CODE_REQUIRED_FIELD, build_error_fields,
                                  envelope_error)
from .cloud_inbound import InboundReject

# --- v2.0 S1.2 ID 正则. 地理 ID 另有更严的正则(w-/r-/f-), 这是通用 ID. ------
_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")

# --- 地理 ID 正则(v2.0 S2.1/S2.4). 前缀区分 waypoint/path/region. ----------
_WAYPOINT_ID_RE = re.compile(r"^w-[a-z0-9_]{1,40}$")
_PATH_ID_RE = re.compile(r"^r-[a-z0-9_]{1,40}$")
_REGION_ID_RE = re.compile(r"^f-[a-z0-9_]{1,40}$")

# --- HH:mm(v2.0 S2.4 alarm_window). 00..23 : 00..59. -----------------------
_HHMM_RE = re.compile(r"^([01][0-9]|2[0-3]):[0-5][0-9]$")

#: v2.0 S1.3 WGS84 经纬度范围.
LAT_MIN, LAT_MAX = -90.0, 90.0
LON_MIN, LON_MAX = -180.0, 180.0

#: v2.0 S2.4 报警标量闭集/范围.
ALARM_LEVELS = (1, 2)
RULE_TYPES = ("person_in_region", "vehicle_in_region")
ALARM_ROLES = ("include", "exclude")
REGION_OPS = ("upsert", "delete", "set_state")


# --- 拒绝辅助 ---------------------------------------------------------

def _missing(field: str) -> InboundReject:
    """必填字段缺失 -> 1002(v2.0 S10 E_REQUIRED_FIELD)."""
    return InboundReject(envelope_error(
        CODE_REQUIRED_FIELD, "required field missing: %s" % field,
        {"field": field}))


def _invalid(field: str, reason: str, extra: Dict = None) -> InboundReject:
    """类型/范围/枚举错误 -> 1003(v2.0 S10 E_INVALID_FIELD)."""
    detail = {"field": field}
    detail.update(extra or {})
    return InboundReject(build_error_fields(errors.E_SCHEMA, reason, detail))


# --- 基础校验子 -------------------------------------------------------

def _req(payload: Dict, field: str) -> Any:
    """取一个必填字段. 缺失(键不存在或值为 None)即抛 1002.

    NO 空字符串/0/False 不算缺失 -- 它们是[给了值], 值对不对由后面的范围/
    枚举校验判. 只有键不存在或显式 null 才是"缺失".
    """
    if field not in payload or payload[field] is None:
        raise _missing(field)
    return payload[field]


def _number(value: Any, field: str) -> float:
    """必须是 number(int/float, 非 bool). bool 显式排除 -- True 是 int 会被
    当成 1.0 混过范围校验."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _invalid(field, "%s must be a number" % field,
                       {"got_type": type(value).__name__})
    return float(value)


def _range(value: float, lo: float, hi: float, field: str) -> None:
    """闭区间 [lo, hi]. v2.0 的范围都是含端点的(0.5..10.0 等)."""
    if not (lo <= value <= hi):
        raise _invalid(field, "%s out of range [%s, %s]: %s"
                       % (field, lo, hi, value), {"got": value})


def _enum(value: Any, closed, field: str) -> None:
    if value not in closed:
        raise _invalid(field, "%s not in %s: %r" % (field, list(closed), value),
                       {"got": value})


def _regex(value: Any, pat: "re.Pattern", field: str, hint: str) -> None:
    if not isinstance(value, str) or not pat.match(value):
        raise _invalid(field, "%s does not match %s: %r" % (field, hint, value),
                       {"got": value})


# --- C-2: 外层 ID 正则 ------------------------------------------------

def check_ids(data: Dict[str, Any]) -> None:
    """v2.0 S1.2: msg_id/task_id 必须匹配 [A-Za-z0-9][A-Za-z0-9._:-]{0,127}.

    task_router 此前只查非空, 不查格式. 一个带目录分隔符或超长的 id 会被
    原样带进机内, 而机内以它做幂等键与关联 -- 格式不对会让关联对不上.
    """
    for field in ("msg_id", "task_id"):
        value = data.get(field)
        if value is None or value == "":
            raise _missing(field)
        _regex(value, _ID_RE, field, "[A-Za-z0-9][A-Za-z0-9._:-]{0,127}")


# --- B-2: GOTO_KEYPOINT payload ---------------------------------------

def validate_goto(payload: Dict[str, Any]) -> None:
    """v2.0 S2.1 GOTO_KEYPOINT payload 的字段级校验.

    校验后 task_router 才把 payload 传给机内 -- 机内不认这些字段, 网关是
    唯一能挡住非法 v2.0 值的地方.
    """
    _enum(_req(payload, "coordinate_system"), ("WGS84",), "coordinate_system")
    _regex(_req(payload, "recorded_path_id"), _PATH_ID_RE, "recorded_path_id",
           "r-[a-z0-9_]{1,40}")

    waypoints = _req(payload, "waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise _invalid("waypoints", "waypoints must be a non-empty array")

    for i, wp in enumerate(waypoints):
        base = "waypoints[%d]" % i
        if not isinstance(wp, dict):
            raise _invalid(base, "%s must be an object" % base)
        # NO waypoint 级 recorded_path_id 一律拒(v2.0 S2.1 逐字).
        if "recorded_path_id" in wp:
            raise _invalid("%s.recorded_path_id" % base,
                           "waypoint-level recorded_path_id is not accepted "
                           "(v2.0 S2.1)")
        _regex(_req(wp, "id"), _WAYPOINT_ID_RE, "%s.id" % base,
               "w-[a-z0-9_]{1,40}")
        _req(wp, "name")                         # 显示名, 只查存在
        lat = _number(_req(wp, "latitude"), "%s.latitude" % base)
        lon = _number(_req(wp, "longitude"), "%s.longitude" % base)
        _range(lat, LAT_MIN, LAT_MAX, "%s.latitude" % base)
        _range(lon, LON_MIN, LON_MAX, "%s.longitude" % base)
        _number(_req(wp, "altitude"), "%s.altitude" % base)
        # *** arrival_radius_m 必填且 0.5..10.0, NO 缺失不补默认(v2.0 S2.1 逐字).
        ar = _number(_req(wp, "arrival_radius_m"), "%s.arrival_radius_m" % base)
        _range(ar, 0.5, 10.0, "%s.arrival_radius_m" % base)


# --- B-3: SET_ALARM_CONFIG payload ------------------------------------

def validate_alarm(payload: Dict[str, Any]) -> None:
    """v2.0 S2.4 SET_ALARM_CONFIG payload 的字段级校验.

    keep_in 的拒绝仍在 task_router._alarm(它要早于本函数, 因为 keep_in 是
    安全边界, 拒绝理由要逐字). 本函数补标量范围 + rules + regions 结构.
    """
    _enum(_req(payload, "alarm_level"), ALARM_LEVELS, "alarm_level")
    _range(_number(_req(payload, "siren_level"), "siren_level"),
           0, 100, "siren_level")
    _range(_number(_req(payload, "duration_sec"), "duration_sec"),
           1, 20, "duration_sec")
    _range(_number(_req(payload, "cooldown_sec"), "cooldown_sec"),
           0.5, 600.0, "cooldown_sec")

    window = _req(payload, "alarm_window")
    if not isinstance(window, dict):
        raise _invalid("alarm_window", "alarm_window must be an object")
    for edge in ("start", "end"):
        _regex(_req(window, edge), _HHMM_RE, "alarm_window.%s" % edge, "HH:mm")

    _validate_rules(payload)
    _validate_regions(payload)


def _validate_rules(payload: Dict[str, Any]) -> None:
    rules = _req(payload, "rules")
    if not isinstance(rules, list):
        raise _invalid("rules", "rules must be an array")
    for i, rule in enumerate(rules):
        base = "rules[%d]" % i
        if not isinstance(rule, dict):
            raise _invalid(base, "%s must be an object" % base)
        _enum(_req(rule, "type"), RULE_TYPES, "%s.type" % base)
        if not isinstance(_req(rule, "enabled"), bool):
            raise _invalid("%s.enabled" % base, "enabled must be a boolean")
        _enum(_req(rule, "alarm_role"), ALARM_ROLES, "%s.alarm_role" % base)
        applies = _req(rule, "applies_to")
        if not isinstance(applies, list):
            raise _invalid("%s.applies_to" % base, "applies_to must be an array")
        region_ids = _req(rule, "region_ids")
        if not isinstance(region_ids, list):
            raise _invalid("%s.region_ids" % base, "region_ids must be an array")
        # 启用规则时 region_ids 非空(v2.0 S2.4 逐字).
        if rule.get("enabled") and not region_ids:
            raise _invalid("%s.region_ids" % base,
                           "region_ids must be non-empty for an enabled rule")


def _validate_regions(payload: Dict[str, Any]) -> None:
    """区域增量操作(upsert/delete/set_state). keep_in 已在 _alarm 拒过."""
    regions = payload.get("regions") or []
    if not isinstance(regions, list):
        raise _invalid("regions", "regions must be an array")
    for i, region in enumerate(regions):
        base = "regions[%d]" % i
        if not isinstance(region, dict):
            raise _invalid(base, "%s must be an object" % base)
        op = _enum_ret(_req(region, "op"), REGION_OPS, "%s.op" % base)
        _regex(_req(region, "id"), _REGION_ID_RE, "%s.id" % base,
               "f-[a-z0-9_]{1,40}")
        # base_rev 三种 op 都必填(v2.0 S2.4 op 表).
        if not isinstance(_req(region, "base_rev"), int):
            raise _invalid("%s.base_rev" % base, "base_rev must be an integer")
        if op == "upsert":
            _enum(_req(region, "type"), ("alarm_region",), "%s.type" % base)
            if not isinstance(_req(region, "enabled"), bool):
                raise _invalid("%s.enabled" % base, "enabled must be a boolean")
            _req(region, "name")
            _req(region, "applies_to")
            vertices = _req(region, "vertices")
            # 网关只查顶点数 >= 3; 自交/面积由机器人端权威校验(S2.4 逐字).
            if not isinstance(vertices, list) or len(vertices) < 3:
                raise _invalid("%s.vertices" % base,
                               "a polygon needs at least 3 vertices")
        elif op == "set_state":
            if not isinstance(_req(region, "enabled"), bool):
                raise _invalid("%s.enabled" % base, "enabled must be a boolean")
        # delete: id + base_rev 已校验, 无其它必填.


def _enum_ret(value: Any, closed, field: str) -> Any:
    _enum(value, closed, field)
    return value


__all__ = ["check_ids", "validate_goto", "validate_alarm"]
