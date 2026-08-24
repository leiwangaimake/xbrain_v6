"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_envelope.py
Brief: v2.0 六字段信封 / seq 语义 / link 三值 / state/task 双形态

Description:
四组判据, 每组对应一种[联调当天才会暴露]的错法:

  1 信封: 机内九字段与跨主机六字段混用 -> Qt 收到 mono/boot 而 v2.0 逐字
    禁止携带; 或者少了 src 让 Qt 分不清是谁发的.
  2 seq: 全局一个计数器 -> Qt 在 10 Hz 的 state/robot 与 1 Hz 的 state/link
    之间看到巨大号差, 按缺口诊断会误报丢包, 而一条都没丢. 单流测试看不出来.
  3 link 三值: L3 擅自映一个值 -> 要么 Qt 显示离线(机器人其实在返航),
    要么丢掉"已触发返航"这件事.
  4 双形态: progress 填 0 冒充未知 -> 0 与"卡在起点"在界面上一样, 而操作员
    据此做的判断相反(一个会等, 一个会去现场).

Boundaries: 只测形状与语义, 不测发布 -- 接线由 test_cloud_key_surface_wired
负责.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


# --- 1 信封 -----------------------------------------------------------

def test_envelope_has_exactly_the_six_fields():
    """*** v2.0 S1.1 六字段, 不多不少.

    多一个字段不会让 Qt 报错(S1.3 允许保留未知扩展), 但它会让下一个读契约
    的人以为那是约定的一部分 -- 而下一版客户升级时那个字段会突然消失.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import build_envelope

    env = build_envelope("gj-001", "state/link", {"state": "up"},
                         ts=1785732000.123456, seq=1)
    assert set(env) == {"v", "rid", "ts", "seq", "src", "data"}, (
        "信封字段集合不是 v2.0 的六个: %s" % sorted(env))


def test_envelope_never_carries_mono_or_boot():
    """*** v2.0 S1.1 逐字: 跨主机消息不得携带 mono, boot.

    机内信封(11 S3.0 九字段)是带 mono 的, 两套混用是最容易发生的错 --
    直接把机内消息透传出去就会带上. 11 S4.6 e 条也逐字要求"重新封装而非透传".
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import build_envelope

    env = build_envelope("gj-001", "state/link",
                         {"state": "up", "mono": 123, "boot": "b-1"},
                         ts=1.0, seq=1)
    # data 里的内容是业务方给的, 本函数不清洗; 但[信封层]不得有这两个.
    assert "mono" not in env and "boot" not in env


def test_ts_is_a_float():
    """v2.0 S1.1: ts 是 float64 Unix 秒, 禁 ISO 字符串与毫秒整数.

    毫秒整数尤其危险: 它看起来是个合法的 number, Qt 解析不会失败, 只是
    所有时间都变成 1970 年之后五万年.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import build_envelope

    env = build_envelope("gj-001", "state/link", {}, ts=1785732000, seq=1)
    assert isinstance(env["ts"], float), "ts 不是 float"


def test_src_defaults_to_the_gateway():
    """后端 -> Qt 固定 p5_gateway(S1.1). 真实来源放 data.source."""
    from xbrain.p5_gateway.outbound.cloud_envelope import (SRC_GATEWAY,
                                                           build_envelope)

    assert build_envelope("gj-001", "k", {}, ts=1.0, seq=1)["src"] == SRC_GATEWAY


# --- 2 seq 语义 -------------------------------------------------------

def test_seq_starts_at_one_per_key():
    """v2.0 S1.1: 发布进程启动时从 1 开始. 0 会被 Qt 当成未初始化."""
    from xbrain.p5_gateway.outbound.cloud_envelope import SeqCounter

    c = SeqCounter()
    assert c.next("gj-001", "state/link") == 1
    assert c.next("gj-001", "state/link") == 2


def test_seq_is_independent_per_key():
    """*** 按 key 分别递增, NO 不是一个全局计数器.

    全局计数器会让 Qt 在 10 Hz 的 state/robot 与 1 Hz 的 state/link 之间
    看到巨大号差 -- 它按缺口诊断会误报丢包, 而一条都没丢.
    这个错在单流测试里完全看不出来.

    MUTATION: 把 SeqCounter 改成一个全局计数器 -> 这里红.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import SeqCounter

    c = SeqCounter()
    for _ in range(5):
        c.next("gj-001", "state/robot")
    # 另一条 key 必须仍从 1 开始.
    assert c.next("gj-001", "state/link") == 1, (
        "两条 key 共用了一个计数器")


def test_seq_is_independent_per_rid():
    """*** 按 rid 分区: 多机器人时不得串号.

    v2.0 S9.3 逐字"状态缓存, 任务关联, 音频 stream_id, manifest, 媒体
    endpoint, 文件索引和事件去重都按 rid 分区".
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import SeqCounter

    c = SeqCounter()
    for _ in range(3):
        c.next("gj-001", "state/link")
    assert c.next("gj-002", "state/link") == 1, "两台机器人共用了一个计数器"


# --- 3 link 三值 ------------------------------------------------------

def test_levels_zero_to_two_map_to_the_three_words():
    """11 的 L0/L1/L2 -> v2.0 的 up/degraded/down."""
    from xbrain.p5_gateway.outbound.cloud_envelope import link_state_word

    assert link_state_word(0) == "up"
    assert link_state_word(1) == "degraded"
    assert link_state_word(2) == "down"


def test_level_three_maps_to_degraded():
    """*** L3(返航触发)映 degraded -- E-2 裁决(2026-08-24, 用户).

    L3 时机器人仍在线且在动(正在返航). degraded 表示"链路降级但在线",
    Qt 不会误判离线; 映 down 会让 Qt 显示离线并可能触发操作员应急流程.
    "已触发返航"不经 state/link 表达(v2.0 只有三值, 无返航字段), 而是通过
    state/task 的 return_home 任务体现 -- 各归各的 key, 不丢信息.

    MUTATION: 把 _LEVEL_TO_STATE 的 3 改成 "down" -> 这里红.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import link_state_word

    assert link_state_word(3) == "degraded"


def test_a_level_out_of_range_still_raises():
    """越界 level(<0 或 >3)仍抛 -- 那是上游缺陷, NO 不猜一个 state 发给 Qt.

    E-2 裁决只给了 L0..L3 落点; 一个 level=4 或 -1 是 link_state.py 的 bug,
    静默发一个猜的 state 会让 Qt 收到一个与真实链路无关的值.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import (UnmappedLinkLevel,
                                                           link_state_word)

    for bad in (4, -1, 99):
        with pytest.raises(UnmappedLinkLevel):
            link_state_word(bad)


def test_link_payload_has_exactly_four_fields():
    """v2.0 S4.1 只要四个字段.

    我方 snapshot 有九个. 多发不会让 Qt 报错, 但报文变大且会让人以为那些
    字段是约定的一部分.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import link_payload

    class _Snap:
        level = 1
        cloud_link = True
        disconnected_s = 6.0

    body = link_payload(_Snap(), "ok")
    assert set(body) == {"state", "cloud_link", "disconnected_s",
                         "estop_path"}, sorted(body)
    assert body["state"] == "degraded"
    assert body["estop_path"] == "ok"


# --- 4 state/task 双形态 ---------------------------------------------

def test_snapshot_and_result_are_the_same_key_different_type():
    """*** R12.4 的变通: 不另设 task/result.

    甲方把独立的 task/result key 合并进 state/task, 用 message_type 区分.
    我方 2026-08-08 答复接受了, 理由逐字: 终态仍由任务权威模块产生, 不由
    网关猜测; 有 duration_sec/distance_m/ended_ts 权威值; 少一条订阅.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import (MSG_RESULT,
                                                           MSG_SNAPSHOT,
                                                           task_result,
                                                           task_snapshot)

    snap = task_snapshot("m-1", None, [], [])
    assert snap["message_type"] == MSG_SNAPSHOT
    res = task_result("m-2", "task-1", "GOTO_KEYPOINT", "done", 0, "",
                      {"completed_count": 1})
    assert res["message_type"] == MSG_RESULT


def test_result_state_is_a_closed_set():
    """*** 三值闭集, 越界即抛.

    v2.0 S1.3 逐字禁止"把未知枚举降级解释为某个已知值" -- 所以 Qt 收到
    'completed' 时不会报错, 只是显示不出来. 我方这一侧必须拦住.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import task_result

    for good in ("done", "failed", "cancelled"):
        task_result("m", "t", "GOTO_KEYPOINT", good, 0, "", {})
    with pytest.raises(ValueError):
        task_result("m", "t", "GOTO_KEYPOINT", "completed", 0, "", {})


def test_no_current_task_is_null_not_empty_dict():
    """当前无任务是一个[确定的答案], 用 null 表达.

    {} 会让 Qt 去读 current.task_id 得到 undefined -- 而那与"字段还没填"
    无法区分.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import task_snapshot

    assert task_snapshot("m", None, [], [])["current"] is None


def test_unknown_progress_is_null_not_zero():
    """*** v2.0 S3.2 逐字: 禁止填 0 冒充未知.

    0 与"卡在起点"在界面上完全一样, 而操作员据此做的判断相反 --
    一个会等, 一个会去现场看.

    MUTATION: 让 normalise_progress(None) 返回 0.0 -> 这里红.
    """
    from xbrain.p5_gateway.outbound.cloud_envelope import normalise_progress

    assert normalise_progress(None) is None
    assert normalise_progress(0) == 0.0        # 真的是 0% 时仍要能表达
    assert normalise_progress(55.5) == 55.5


def test_progress_outside_range_raises():
    """越界即抛: 一个 150% 的进度条在界面上是坏的, 而它来自我方."""
    from xbrain.p5_gateway.outbound.cloud_envelope import normalise_progress

    with pytest.raises(ValueError):
        normalise_progress(150)
    with pytest.raises(ValueError):
        normalise_progress(-1)
