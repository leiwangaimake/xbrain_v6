"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cloud_inbound.py
Brief: 云端 Qt 报文入站: v2.0 六字段信封校验 + rid 一致性 (A-1)

Description:
这是[契约边界的外侧] -- Qt 发来的报文尚未转成我方信封, 所以本模块不施加
11 S3.0 的九字段校验, 而是按 v2.0 S1.1 的六字段判. 11 S2.2 的 cmd/task/ext
行逐字写着这一点: "本 key 上不施加 S3.0 九字段信封校验 ... 校验在网关重建
信封时施加".

*** 为什么每一种失败都要能回一个 ack, 而不是丢掉.
v2.0 S7.3 逐字: "非法版本, rid/key 不一致, 字段缺失, 类型错误, 枚举越界均
返回结构化拒绝, 不能静默丢弃". 静默丢弃在 Qt 那边的表现是"点了没反应" --
操作员会重试, 而重试同样被丢, 于是他会认为机器人死了.

*** rid 必须与 key 第二段[逐字]比, 大小写敏感.
v2.0 S1 第 3 条: "rid 必须匹配 [a-z0-9_-]{1,32}, 并与 key 第二段逐字一致,
大小写敏感". 这条守的是多机场景: 两台机器人各有 session, 一条 rid 写错的
报文会让 A 的指令落到 B 身上 -- 而 Zenoh 不会拦, key 是对的.

*** src 是区分[外部报文]与[机内报文]的唯一依据.
按用户 2026-08-23 裁决, 网关多订一个 cmd/task(与 p3_task 同一条 key).
于是这条 key 上会同时有两种形状:
  src="qt_hmi"      Qt 原始报文 -> 网关处理, 重建后转发
  src="p5_gateway"  机内已重建报文 -> 网关必须[忽略], 否则会重复处理自己
                    刚发出去的那条, 形成回环
这一条是本模块最要紧的判据, 见 is_cloud_frame().

Boundaries: 只做信封层. 不解析 payload 的业务内容(那按 task_type 分发到各
自的解析器), 不发布, 不判断能不能执行.
"""

from __future__ import annotations

import json
import re
from typing import Any, Dict, Optional, Tuple

from ..outbound.error_map import (CODE_JSON_PARSE, CODE_REQUIRED_FIELD,
                                  CODE_RID_MISMATCH, envelope_error)

#: v2.0 S1 第 3 条: rid 的值域.
RID_RE = re.compile(r"^[a-z0-9_-]{1,32}$")

#: v2.0 S1.1: Qt -> 后端固定 qt_hmi.
SRC_QT = "qt_hmi"

#: 我方网关自己发的. 见模块头注: 同一条 key 上必须忽略它.
SRC_GATEWAY = "p5_gateway"

#: 信封必填六字段.
REQUIRED_FIELDS = ("v", "rid", "ts", "seq", "src", "data")

#: v2.0 固定 v=1. 别的版本回 1005.
SUPPORTED_V = 1


class InboundReject(Exception):
    """入站被拒. fields 是可直接填进 ack 的三件(error_code/reason/detail).

    带着 ack 字段抛而不是只抛消息: 调用方拿到就能回 Qt, 不需要再判断
    "这是哪种错该回哪个码" -- 那个判断在这里做过一次了.
    """

    def __init__(self, fields: Dict[str, Any]):
        super().__init__(fields.get("reason", "inbound rejected"))
        self.fields = fields


def is_cloud_frame(body: Dict[str, Any]) -> bool:
    """这条报文是不是[云端发来的].

    *** 网关与 p3_task 订同一条 cmd/task, 所以必须能区分.
    网关重建后会往 cmd/task 发一条 src="p5_gateway" 的报文给 p3_task;
    如果网关也处理那一条, 它会再重建一次再发一次 -- 无限回环, 而每一圈
    都是合法报文, 没有任何东西会报错.

    NO 不靠"有没有 task_type"之类的形状差异来区分: 两种形状都可能带
    task_type, 而形状是会变的. src 是契约里唯一为此设的字段.
    """
    return isinstance(body, dict) and body.get("src") == SRC_QT


def parse_frame(raw: bytes, key_rid: str) -> Dict[str, Any]:
    """解析一条云端报文并做信封层校验. 失败抛 InboundReject.

    key_rid: 从 Zenoh key 第二段取出的 rid, 用于逐字比对.
    """
    # -- 1001 JSON 解析 ------------------------------------------------
    try:
        text = raw.decode("utf-8") if isinstance(raw, (bytes, bytearray)) else raw
        body = json.loads(text)
    except (UnicodeDecodeError, ValueError) as exc:
        raise InboundReject(envelope_error(
            CODE_JSON_PARSE, "frame is not valid JSON", {"parse_error": str(exc)}))
    if not isinstance(body, dict):
        raise InboundReject(envelope_error(
            CODE_JSON_PARSE, "frame top level is not an object",
            {"got_type": type(body).__name__}))

    # -- 1005 版本[先于]必填字段 ---------------------------------------
    #
    # *** 顺序不是随意的, 是断言逼出来的.
    # 第一版把版本放在必填字段之后, 于是一条 v=2 且少一个字段的报文被报成
    # 1002(字段缺失) -- 而真正的原因是"这是另一版协议". Qt 开发者照着 1002
    # 去补字段, 补完还是不通.
    # v=2 的报文其余字段可能是另一套语义, 按 v1 判它们本身就没有意义.
    if "v" not in body:
        raise InboundReject(envelope_error(
            CODE_REQUIRED_FIELD, "envelope missing required field: v", {"missing": ["v"]}))
    if body["v"] != SUPPORTED_V:
        from ...common import errors
        from ..outbound.error_map import CODE_VERSION_UNSUPPORTED
        raise InboundReject({
            "error_code": CODE_VERSION_UNSUPPORTED,
            "reason": "unsupported protocol version %r" % (body["v"],),
            "detail": {"code": errors.E_PROTO_VERSION,
                       "supported": [SUPPORTED_V], "got": body["v"]},
        })

    # -- 1002 必填字段 -------------------------------------------------
    missing = [f for f in REQUIRED_FIELDS if f not in body]
    if missing:
        raise InboundReject(envelope_error(
            CODE_REQUIRED_FIELD, "envelope missing required field: %s" % ", ".join(missing),
            {"missing": missing}))

    # -- 1004 rid 与 key 逐字一致 --------------------------------------
    rid = body["rid"]
    if not isinstance(rid, str) or not RID_RE.match(rid):
        raise InboundReject(envelope_error(
            CODE_RID_MISMATCH, "rid outside the value range [a-z0-9_-]{1,32}",
            {"got": rid}))
    if rid != key_rid:
        # 逐字比, 大小写敏感 -- 这条守的是多机场景: 一条 rid 写错的报文会让
        # A 的指令落到 B 身上, 而 Zenoh 不会拦(key 是对的).
        raise InboundReject(envelope_error(
            CODE_RID_MISMATCH, "rid does not match the second key segment",
            {"expected": key_rid, "got": rid}))

    # -- 1002 data 必须是对象 ------------------------------------------
    if not isinstance(body["data"], dict):
        raise InboundReject(envelope_error(
            CODE_REQUIRED_FIELD, "data is not an object",
            {"got_type": type(body["data"]).__name__}))
    return body


def frame_ids(body: Dict[str, Any]) -> Tuple[Optional[str], Optional[str],
                                             Optional[str]]:
    """取 (msg_id, task_id, task_type). 缺失返回 None, 不抛.

    信封层不判断它们必填 -- 那是按 key 分的: cmd/task 要三个都有,
    cmd/file/ack 只要 msg_id. 让各自的解析器去判, 报错才能指出是哪个 key
    的哪个字段.
    """
    data = body.get("data") or {}
    return (data.get("msg_id"), data.get("task_id"), data.get("task_type"))


def rid_from_key(key: str) -> Optional[str]:
    """从 xbrain/{rid}/... 取第二段.

    取不到返回 None -- 调用方据此拒绝, 而不是猜一个. 一条 key 结构不对的
    报文根本不该被处理.
    """
    parts = str(key).split("/")
    if len(parts) < 3 or parts[0] != "xbrain":
        return None
    return parts[1] or None
