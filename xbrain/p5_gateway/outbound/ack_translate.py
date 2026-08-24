"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ack_translate.py
Brief: 机内 cmd/task/ack 与 cmd/geo/ack -> v2.0 八字段 ack (审计 A-1)

Description:
2026-08-24 审计的头号发现(A-1): 网关对云端 cmd/task 立即回乐观 accepted, 不
承接 p3 的机内 ack, 于是 p3 的[业务拒绝](E_NOT_FOUND / E_OUT_OF_FENCE /
E_BUSY / duplicate)永远回不到云端. 而 v2.0 S1.4 逐字要求 ack "包括结构拒绝,
业务拒绝和 duplicate", 我方评审 R1.3 也承诺"task/ack 直接承接, 仅需字段映射".

本模块是那次[字段映射]: 把 p3 的机内 ack(11 S7.7 shape)翻译成 v2.0 S3.1 的
八字段 ack. 纯函数, 不碰 Zenoh -- 承接的接线在 runtime/cloud_wiring.py.

*** 机内 result 有四个值, v2.0 只有三个.
机内: accepted | rejected | duplicate | error(11 S7.7).
v2.0: accepted | rejected | duplicate(S3.1).
error(我方内部错误)映到 v2.0 的 rejected -- 对 Qt 来说"后端出错了"与"被拒"
都是"这条没成", 而 detail.code 保留了原码供排查.

*** accepted/duplicate 的 error_code 必须是 0; rejected 必须非 0 + 有 reason.
v2.0 S3.1/S10 逐字. 一条 {result:rejected, error_code:0} 会让 Qt 走成功分支.
build_ack(task_ack.py)本就强制这条不变式, 本模块复用它, 于是翻译出来的 ack
不可能自相矛盾.

Boundaries: 只做机内 ack -> v2.0 ack 的字段映射. ref_msg_id/task_id/task_type
从[网关登记的 pending]取(机内 ack 不带这些 v2.0 字段), 由调用方传入.
"""

from __future__ import annotations

from typing import Any, Dict

from ...common import errors
from .error_map import to_qt_code
from .task_ack import (RESULT_ACCEPTED, RESULT_DUPLICATE, RESULT_REJECTED,
                       build_ack)

#: 机内 result 四值 -> v2.0 三值. error 折成 rejected.
_RESULT_MAP = {
    "accepted": RESULT_ACCEPTED,
    "duplicate": RESULT_DUPLICATE,
    "rejected": RESULT_REJECTED,
    "error": RESULT_REJECTED,
}

#: rejected/error 但机内没给具体码时的兜底. 3001 = 后端内部/存储问题(S10),
#: 是"我方出了问题"最接近的桶; detail.code 里仍带原始的 code 供排查.
_FALLBACK_REJECT_CODE = errors.E_INTERNAL


def translate_ack(internal_ack: Dict[str, Any], *, ref_msg_id: str,
                  task_id: str, task_type: str,
                  new_msg_id: str) -> Dict[str, Any]:
    """机内 ack -> v2.0 S3.1 八字段 ack data.

    internal_ack: p3 的 {schema, cmd_id, result, code, message?, detail?}.
    ref_msg_id/task_id/task_type: 从网关 pending 取(机内 ack 不带这些).
    new_msg_id: 这条 ack 自己的 msg_id(v2.0 S1.2: 每条消息一个新 id).
    """
    raw_result = internal_ack.get("result")
    result = _RESULT_MAP.get(raw_result)
    if result is None:
        # 机内给了一个闭集外的 result -- 当拒绝处理(宁可让 Qt 知道没成),
        # 而不是静默丢(3.5 越界必抛的精神, 但这里在出站不能抛断回调, 折成
        # 一条可读的 rejected).
        result = RESULT_REJECTED
        raw_result = "error"

    accepted = result in (RESULT_ACCEPTED, RESULT_DUPLICATE)
    code = internal_ack.get("code") or "OK"
    detail = dict(internal_ack.get("detail") or {})
    reason = internal_ack.get("message") or ""

    if accepted:
        error_code = 0
    else:
        # rejected/error: 需要非零 error_code. code=="OK" 不该出现在拒绝上,
        # 兜底到 E_INTERNAL.
        e_code = code if code != "OK" else _FALLBACK_REJECT_CODE
        error_code = to_qt_code(e_code)
        detail["code"] = e_code
        if not reason:
            # v2.0 S3.1: 失败必须有可读 reason. 机内没给就用 code 兜底 --
            # 空 reason 在 Qt 上是"失败"两个字加一片空白.
            reason = "task rejected: %s" % e_code

    return build_ack(
        msg_id=new_msg_id, ref_msg_id=ref_msg_id, task_id=task_id,
        task_type=task_type, result=result, error_code=error_code,
        reason=reason, detail=detail)


def aggregate_child_acks(acks: list) -> Dict[str, Any]:
    """N 条机内子 ack 聚合成一条(SET_ALARM_CONFIG 是配置事务: 全成才成).

    一条 SET_ALARM_CONFIG fan-out 成 N 条机内命令(每个 alarm_region 一条
    cmd/geo fence upsert; 批B 起再加 cmd/config), 每条回一条机内 ack. 但 v2.0
    S4.1 只允许[一条] cmd/task/ack. 本函数把 N 条合成一条供 translate_ack 翻译:
      任一子命令未受理(rejected/error) -> 整体取[首个失败]那条(带它的 code/
        message/detail, Qt 据此定位是哪个 region 出的错);
      全部 accepted/duplicate -> 整体 accepted(报警区重复提交是幂等的, duplicate
        不算失败); 全 duplicate -> duplicate.

    *** 为什么一票否决而不是部分成功.
    配置是事务性的: 一半 region 写进去一半没写, 对操作员比"整体失败重发"更难
    处理(他不知道哪几个生效了). 各条 fence upsert 幂等, 整体重发是安全的, 所以
    宁可任一失败就整体判失败让操作员重发. 真正的[原子回滚]留给批D 的终态收尾
    (那里才有 state/fence.active.rev 的权威确认).

    返回机内 ack 形状 {result, code, message, detail}, 由 translate_ack 消费.
    """
    # NOTE 比较的是[机内] ack 的 result(accepted|duplicate|rejected|error,
    # 见本模块 _RESULT_MAP 的键), NO 不是 v2.0 的 RESULT_* -- 机内多一个 error
    # 值, 用 v2.0 常量比会漏掉它. 返回的也是机内形状, 交给 translate_ack 翻译.
    if not acks:
        # 收齐 N(>=1)条才调本函数, 不该为空. 防御: 当内部错处理.
        return {"result": "error", "code": errors.E_INTERNAL,
                "message": "empty alarm fan-out", "detail": {}}
    for ack in acks:
        if ack.get("result") not in ("accepted", "duplicate"):
            return ack                          # 首个失败(rejected/error), 原样返回
    if all(ack.get("result") == "duplicate" for ack in acks):
        return {"result": "duplicate", "code": "OK", "message": "", "detail": {}}
    return {"result": "accepted", "code": "OK", "message": "", "detail": {}}


__all__ = ["translate_ack", "aggregate_child_acks"]
