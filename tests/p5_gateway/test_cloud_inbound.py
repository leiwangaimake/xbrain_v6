"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_inbound.py
Brief: A-1 云端入站信封校验 -- 每种失败都要能回 ack, 且不得回环

Description:
本文件守两件事, 第二件是本轮裁决带来的新风险:

*** 一 每一种失败都要回一个结构化拒绝, NO 不许静默丢弃.
v2.0 S7.3 逐字: "非法版本, rid/key 不一致, 字段缺失, 类型错误, 枚举越界
均返回结构化拒绝, 不能静默丢弃". 静默丢弃在 Qt 那边是"点了没反应" --
操作员会重试, 重试同样被丢, 于是他认为机器人死了.

*** 二 网关与 p3_task 订同一条 cmd/task, 必须靠 src 区分, 否则回环.
用户 2026-08-23 裁决 E-1: 网关多订 cmd/task. 于是这条 key 上有两种报文:
  src="qt_hmi"     Qt 原始报文 -> 网关重建后转发
  src="p5_gateway" 网关刚重建的那条 -> 网关必须忽略
不忽略的话, 网关会重建自己刚发的那条再发一次 -- 无限回环, 而每一圈都是
合法报文, 没有任何东西会报错. 这类缺陷在单条报文的测试里完全看不出来,
所以单列一组用例.

Boundaries: 只测信封层. 业务内容(waypoints 合不合法之类)由各 task_type 的
解析器负责.
"""
from __future__ import annotations

import json

import pytest

pytestmark = pytest.mark.no_device


def _frame(**over):
    body = {"v": 1, "rid": "gj-001", "ts": 1785732000.123456, "seq": 1,
            "src": "qt_hmi",
            "data": {"msg_id": "msg-1", "task_id": "task-1",
                     "task_type": "GOTO_KEYPOINT", "payload": {}}}
    body.update(over)
    return json.dumps(body, ensure_ascii=False).encode("utf-8")


def _reject_code(raw, key_rid="gj-001"):
    from xbrain.p5_gateway.inbound.cloud_inbound import (InboundReject,
                                                         parse_frame)
    try:
        parse_frame(raw, key_rid)
    except InboundReject as exc:
        return exc.fields
    return None


# --- 一 每种失败都有结构化拒绝 ---------------------------------------

def test_a_good_frame_parses():
    """基线. 没有这条, 一个"永远拒绝"的实现能让下面每条都通过."""
    from xbrain.p5_gateway.inbound.cloud_inbound import parse_frame

    body = parse_frame(_frame(), "gj-001")
    assert body["rid"] == "gj-001"
    assert body["data"]["task_type"] == "GOTO_KEYPOINT"


def test_broken_json_is_1001():
    """v2.0 S10: JSON 无法解析 -> 1001."""
    from xbrain.p5_gateway.outbound.error_map import CODE_JSON_PARSE

    fields = _reject_code(b'{"v": 1, "rid": ')
    assert fields and fields["error_code"] == CODE_JSON_PARSE
    # detail 里要带解析错误 -- 否则 Qt 那边只知道"报文坏了", 不知道坏在哪.
    assert "parse_error" in fields["detail"]


def test_a_json_array_at_top_level_is_1001():
    """顶层不是对象也算解析失败.

    这条防的是一个只 try json.loads 的实现: `[1,2,3]` 是合法 JSON,
    解析成功, 然后下一行 body.get("v") 抛 AttributeError -- 那是个未捕获
    异常, 会让整个回调死掉而不是回一条拒绝.
    """
    from xbrain.p5_gateway.outbound.error_map import CODE_JSON_PARSE

    fields = _reject_code(b'[1, 2, 3]')
    assert fields and fields["error_code"] == CODE_JSON_PARSE


def test_each_missing_field_is_1002_and_names_it():
    """*** 六个必填字段逐个缺一次.

    只测"缺 rid"一种的话, 一个只查 rid 的实现能通过 -- 而缺 seq 的报文会
    一路走到业务层才炸.
    报错必须[点名缺的是哪个]: 一条只说"字段缺失"的拒绝, 让 Qt 开发者要
    逐个字段试.
    """
    from xbrain.p5_gateway.outbound.error_map import CODE_REQUIRED_FIELD

    for field in ("v", "rid", "ts", "seq", "src", "data"):
        body = json.loads(_frame())
        del body[field]
        raw = json.dumps(body).encode("utf-8")
        fields = _reject_code(raw)
        assert fields, "缺 %s 却通过了" % field
        assert fields["error_code"] == CODE_REQUIRED_FIELD, field
        assert field in fields["detail"]["missing"], (
            "报错没点名缺的是 %s: %s" % (field, fields["detail"]))


def test_wrong_version_is_1005_and_stops_there():
    """*** 版本不对时不再往下校验.

    一个 v=2 的报文其余字段可能是另一套语义, 继续按 v1 判会报出一堆无意义
    的字段错, 把真正的原因(版本不支持)埋掉.
    """
    from xbrain.p5_gateway.outbound.error_map import CODE_VERSION_UNSUPPORTED

    body = json.loads(_frame())
    body["v"] = 2
    del body["seq"]                    # 同时制造一个字段错
    fields = _reject_code(json.dumps(body).encode("utf-8"))
    assert fields["error_code"] == CODE_VERSION_UNSUPPORTED, (
        "版本错被字段错盖过了: %s" % fields)
    assert fields["detail"]["got"] == 2


def test_rid_not_matching_the_key_is_1004():
    """*** 多机场景的要害.

    一条 rid 写错的报文会让 A 的指令落到 B 身上 -- 而 Zenoh 不会拦,
    key 是对的. 这是唯一能拦住它的地方.
    """
    from xbrain.p5_gateway.outbound.error_map import CODE_RID_MISMATCH

    fields = _reject_code(_frame(rid="gj-002"), key_rid="gj-001")
    assert fields["error_code"] == CODE_RID_MISMATCH
    # 两个值都要在 detail 里 -- 只说"不一致"没法定位是哪边错了.
    assert fields["detail"]["expected"] == "gj-001"
    assert fields["detail"]["got"] == "gj-002"


def test_rid_comparison_is_case_sensitive():
    """v2.0 S1 第 3 条逐字: 大小写敏感.

    评审 R1.5 提过甲方原稿用 `GJ-001`(含大写), v2.0 已收窄为小写值域.
    一个用 .lower() 比较的实现会放行 GJ-001 -- 而那台机器人的 key 是
    gj-001, 于是报文进了对的 session 却带着一个我方 geo 库里不存在的 rid.
    """
    from xbrain.p5_gateway.outbound.error_map import CODE_RID_MISMATCH

    fields = _reject_code(_frame(rid="GJ-001"), key_rid="gj-001")
    assert fields["error_code"] == CODE_RID_MISMATCH


def test_rid_outside_the_value_range_is_rejected():
    """值域 [a-z0-9_-]{1,32}. 空串与超长都要拒."""
    from xbrain.p5_gateway.outbound.error_map import CODE_RID_MISMATCH

    for bad in ("", "x" * 33, "gj 001", "gj/001"):
        fields = _reject_code(_frame(rid=bad), key_rid=bad)
        assert fields and fields["error_code"] == CODE_RID_MISMATCH, bad


def test_data_must_be_an_object():
    """data 是数组或字符串时要在信封层拦住.

    放过去的话, 业务解析器会对一个 list 调 .get() -- 未捕获异常.
    """
    from xbrain.p5_gateway.outbound.error_map import CODE_REQUIRED_FIELD

    fields = _reject_code(_frame(data=[1, 2]))
    assert fields["error_code"] == CODE_REQUIRED_FIELD


def test_every_reject_carries_the_three_ack_fields():
    """*** 每一种拒绝都要能直接填进 ack.

    v2.0 S10 逐字要求所有拒绝同时提供非零 error_code / 人类可读 reason /
    detail.code. 少任何一个, 调用方就得自己补 -- 而补的地方一多就会有一处
    忘了, 那条拒绝到 Qt 那边就是残缺的.
    """
    cases = [b'{bad', _frame(rid="gj-002"), _frame(data=[])]
    body = json.loads(_frame())
    del body["seq"]
    cases.append(json.dumps(body).encode("utf-8"))
    for raw in cases:
        fields = _reject_code(raw)
        assert fields, raw[:20]
        assert fields["error_code"] != 0
        assert fields["reason"], "reason 为空"
        assert fields["detail"].get("code", "").startswith("E_"), (
            "detail.code 不是我方 E_* 码: %s" % fields["detail"])


# --- 二 回环防护 ------------------------------------------------------

def test_only_qt_frames_are_treated_as_cloud():
    """*** 网关与 p3_task 订同一条 cmd/task, 靠 src 区分.

    不区分的话, 网关会重建自己刚发给 p3_task 的那条再发一次 -- 无限回环,
    而每一圈都是合法报文, 没有任何东西会报错.

    MUTATION: 让 is_cloud_frame 恒返回 True -> 这里红.
    """
    from xbrain.p5_gateway.inbound.cloud_inbound import is_cloud_frame

    assert is_cloud_frame({"src": "qt_hmi"}) is True
    assert is_cloud_frame({"src": "p5_gateway"}) is False, (
        "网关会处理自己发出的报文 -- 回环")
    # 机内其它发布者(HMI / p4_agent 经网关转发)同样不该被当成云端报文.
    for other in ("hmi", "p4_agent", "p3_task", None):
        assert is_cloud_frame({"src": other}) is False, other


def test_is_cloud_frame_does_not_guess_from_shape():
    """NO 不靠"有没有 task_type"之类的形状差异区分.

    两种形状都可能带 task_type, 而形状是会变的. src 是契约里唯一为此设的
    字段(v2.0 S1.1: "Qt -> 后端固定 qt_hmi").
    """
    from xbrain.p5_gateway.inbound.cloud_inbound import is_cloud_frame

    # 一条带齐业务字段但 src 是网关的报文 -- 必须仍被判为非云端.
    internal = {"src": "p5_gateway", "data": {"task_type": "GOTO_KEYPOINT"}}
    assert is_cloud_frame(internal) is False


def test_a_non_dict_is_not_a_cloud_frame():
    """回调可能拿到任何东西. None / list 不得让判定崩掉."""
    from xbrain.p5_gateway.inbound.cloud_inbound import is_cloud_frame

    for junk in (None, [], "qt_hmi", 42):
        assert is_cloud_frame(junk) is False


# --- rid_from_key -----------------------------------------------------

def test_rid_from_key_takes_the_second_segment():
    from xbrain.p5_gateway.inbound.cloud_inbound import rid_from_key

    assert rid_from_key("xbrain/gj-001/cmd/task") == "gj-001"
    assert rid_from_key("xbrain/gj-001/state/geo/manifest") == "gj-001"


def test_rid_from_key_refuses_a_malformed_key():
    """结构不对的 key 返回 None, 调用方据此拒绝 -- NO 不猜一个.

    一条 key 结构不对的报文根本不该被处理: 它可能来自一个用错前缀的
    发布者(评审 R1.2 说的 robots/ 就是那种).
    """
    from xbrain.p5_gateway.inbound.cloud_inbound import rid_from_key

    for bad in ("robots/gj-001/task/request", "xbrain", "xbrain//cmd/task",
                "", "cmd/task"):
        assert rid_from_key(bad) is None, bad
