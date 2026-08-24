"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ack_translate.py
Brief: 机内 ack -> v2.0 八字段 ack 的翻译判据 (审计 A-1)

Description:
A-1 承接的[字段映射]那一半. 机内 result 四值(accepted/rejected/duplicate/
error) -> v2.0 三值; 机内 code -> v2.0 error_code + detail.code. 每条映射都
配一个变异体, 因为翻译错一个字段就会让 Qt 收到与真实结果相反的 ack.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


def _tr(internal, **over):
    from xbrain.p5_gateway.outbound.ack_translate import translate_ack

    kw = {"ref_msg_id": "m-1", "task_id": "t-1",
          "task_type": "GOTO_KEYPOINT", "new_msg_id": "ack-1"}
    kw.update(over)
    return translate_ack(internal, **kw)


def test_accepted_maps_clean():
    d = _tr({"schema": "task_ack_v1", "cmd_id": "c-m-1", "result": "accepted",
             "code": "OK"})
    assert d["result"] == "accepted" and d["accepted"] is True
    assert d["error_code"] == 0
    assert d["ref_msg_id"] == "m-1" and d["task_id"] == "t-1"
    assert d["task_type"] == "GOTO_KEYPOINT"
    assert d["msg_id"] == "ack-1"


def test_business_reject_carries_code_and_qt_error_code():
    """*** p3 的业务拒绝 -> v2.0 rejected + 对应整数码 + detail.code.

    MUTATION: _RESULT_MAP 把 rejected 映成 accepted -> 这里红.
    """
    d = _tr({"cmd_id": "c-m-1", "result": "rejected", "code": "E_OUT_OF_FENCE",
             "message": "围栏外"})
    assert d["result"] == "rejected" and d["accepted"] is False
    assert d["error_code"] == 2006             # E_OUT_OF_FENCE -> 2006
    assert d["detail"]["code"] == "E_OUT_OF_FENCE"
    assert d["reason"] == "围栏外"


def test_not_found_maps_to_1003():
    """STOP 不存在的任务: E_NOT_FOUND -> 1003(客户答复 4.3)."""
    d = _tr({"cmd_id": "c-m-1", "result": "rejected", "code": "E_NOT_FOUND",
             "message": "no such task"})
    assert d["error_code"] == 1003
    assert d["detail"]["code"] == "E_NOT_FOUND"


def test_duplicate_stays_duplicate_and_accepted_true():
    """duplicate -> accepted=true, error_code=0(v2.0 S3.1)."""
    d = _tr({"cmd_id": "c-m-1", "result": "duplicate", "code": "OK"})
    assert d["result"] == "duplicate" and d["accepted"] is True
    assert d["error_code"] == 0


def test_internal_error_folds_to_rejected():
    """*** 机内 error -> v2.0 rejected(v2.0 无 error 值).

    MUTATION: _RESULT_MAP 去掉 error 项 -> 这里红(未知 result 兜底也拒).
    """
    d = _tr({"cmd_id": "c-m-1", "result": "error", "code": "E_INTERNAL",
             "message": "boom"})
    assert d["result"] == "rejected" and d["accepted"] is False
    assert d["error_code"] == 3001             # E_INTERNAL -> 3001


def test_reject_without_message_gets_a_fallback_reason():
    """*** 失败必须有可读 reason(v2.0 S3.1). p3 没给就兜底.

    空 reason 在 Qt 上是"失败"加一片空白 -- 操作员不知道为什么.

    MUTATION: 去掉 reason 兜底 -> 这里红(reason 为空).
    """
    d = _tr({"cmd_id": "c-m-1", "result": "rejected", "code": "E_BUSY"})
    assert d["result"] == "rejected"
    assert d["reason"], "失败 ack 的 reason 为空"


def test_reject_with_ok_code_falls_back_to_internal():
    """rejected 却带 code=OK(不该出现)时兜底到非零码, NO 不发 error_code=0.

    一条 {result:rejected, error_code:0} 会让 Qt 走成功分支.
    """
    d = _tr({"cmd_id": "c-m-1", "result": "rejected", "code": "OK"})
    assert d["result"] == "rejected" and d["error_code"] != 0


def test_unknown_result_is_treated_as_reject():
    """机内给了闭集外的 result -> 当拒绝(宁可让 Qt 知道没成)."""
    d = _tr({"cmd_id": "c-m-1", "result": "weird", "code": "E_INTERNAL"})
    assert d["result"] == "rejected" and d["accepted"] is False
