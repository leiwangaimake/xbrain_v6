"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_terminal_result_fields.py
Brief: v2.0 S3.3 result -- terminal facts ride the task event, unknown stays null

Description:
甲方 2026-09-03 停掉一条任务后收到的 result 是这样的: task_type 空串,
route_id / started_ts / ended_ts 全 null, duration_sec 0.0, reason 写着
"task ended without a recorded reason" -- 而库里 route_geo_id 是 r-night,
started_at 有值, 操作员发的 reason 是 operator_stop.

不是取错字段, 是[终态事实没有通路到 p5]: p5 不读 task.db(平面隔离), 而
11 S4.4 的 TaskState 只列非终态任务 -- 任务一终结就从广播里消失, 那些字段
跟着一起消失. 事件是终态那一刻唯一还带着任务的报文, 所以 p3 把这些事实放进
了事件 detail(11 S6.1 的 detail 本就是按类别自定义的 JSON).
"""

from __future__ import annotations

import pytest

from xbrain.p5_gateway.outbound.task_result import build_result

pytestmark = pytest.mark.no_device


def test_an_unknown_duration_stays_null_and_never_becomes_zero():
    """15 S9.5 / 11 S4.4 逐字: 跨重启时 duration_sec 写 NULL, 不得回退用墙钟
    差值. 0.0 在 Qt 上是"用时 0 秒" -- 一个假的已知值(CLAUDE.md 3.1).

    MUTATION: 把默认值改回 0.0, 或在 build_result 里 float(None or 0.0) -> 红.
    """
    r = build_result(task_id="t-1", task_type="GOTO_KEYPOINT", state="cancelled",
                     result_code=2001, reason="operator_stop")
    assert r["summary"]["duration_sec"] is None, r["summary"]


def test_a_known_duration_is_reported_as_a_number():
    """反向: 真有时长时必须带出去, 否则一个恒 null 的实现也能通过上面那条.

    MUTATION: 恒返回 None -> 红.
    """
    r = build_result(task_id="t-1", task_type="GOTO_KEYPOINT", state="done",
                     duration_sec=30.5)
    assert r["summary"]["duration_sec"] == pytest.approx(30.5)


def test_the_terminal_facts_all_reach_the_summary():
    """v2.0 S3.3: 这几个字段是甲方界面上"这条任务跑了什么"的全部依据.

    MUTATION: 任意一个不透传 -> 红.
    """
    r = build_result(task_id="t-1", task_type="GOTO_KEYPOINT", state="cancelled",
                     result_code=2001, reason="operator_stop",
                     route_id="r-night", started_ts=1788400000.0,
                     ended_ts=1788400030.0, duration_sec=30.0)
    assert r["task_type"] == "GOTO_KEYPOINT"
    assert r["reason"] == "operator_stop"
    s = r["summary"]
    assert s["route_id"] == "r-night"
    assert s["started_ts"] == 1788400000.0
    assert s["ended_ts"] == 1788400030.0
    assert s["duration_sec"] == pytest.approx(30.0)


def test_p5_forwards_the_whole_event_detail_to_the_result_tracker():
    """*** 接线: 只挑 task_id/state 的话, 上面那些字段在 result 里全是 null --
    甲方看到的就是那样. 事件 detail 必须整个交出去.

    读 p5 真实源码: observe_task 的入参是回调里现造的字典, 没有 fake 能看出
    它少带了字段.
    MUTATION: 改回 {"task_id":..., "state":...} -> 红.
    """
    import inspect

    from xbrain.p5_gateway.runtime.main_wiring import run_voice_loop_wiring

    src = inspect.getsource(run_voice_loop_wiring)
    body = src[src.index("def _on_event("):]
    body = body[:body.index("cloud_bridge.publish_event")]
    assert "observe_task(dict(_det))" in body, (
        "事件 detail 没有整个交给 result tracker")


def test_p3_puts_the_terminal_facts_on_the_event():
    """另一半接线: p5 拿得到, 前提是 p3 放得进去.

    MUTATION: 去掉 _fetch_terminal_facts 或不在终态迁移上调它 -> 红.
    """
    import inspect

    from xbrain.p3_task.runtime import main_wiring as p3_wiring

    src = inspect.getsource(p3_wiring._make_publish)
    # 守卫本身: 改成恒假(if False:)时这些标识符还都在源码里, 只查子串会漏.
    assert "if to_state in TERMINAL_STATES and fetch_terminal is not None:" in src, (
        "终态取事实的守卫没了或被改成恒假 -- result 里那几个字段会全空")
    assert "extra = dict(await fetch_terminal(task_id))" in src
    loop_src = inspect.getsource(p3_wiring._amain)
    # 取的字段逐个钉住: SELECT 里出现 duration_sec 不等于它被放进返回值.
    for field in ('"task_type": row[0]', '"route_id": row[1]',
                  '"started_ts": wall_iso_to_epoch(row[2])',
                  '"ended_ts": wall_iso_to_epoch(row[3])',
                  '"duration_sec": row[4]'):
        assert field in loop_src, "终态事实里缺 %s" % field


# --------------------------------- 走真实投影: _result_for 读的是事件 detail

class _FakeBridge:
    def __init__(self):
        self.published = []

    def publish_state(self, name, data):
        self.published.append((name, data))


def _projector():
    from xbrain.p5_gateway.runtime.cloud_state import CloudProjector

    b = _FakeBridge()
    return CloudProjector(b, now_mono=lambda: 0.0), b


def test_the_projector_builds_the_result_from_the_event_detail():
    """*** 端到端的那一段: p3 的事件 detail -> observe_task -> result.

    上面几条测的是 build_result 本身; 这条测[投影层有没有把 detail 里的字段
    喂给它]. A3 变异(在 _result_for 里写 `or 0.0`)只有走这条路才会被抓到.

    MUTATION: _result_for 里 duration_sec 用 `or 0.0` -> 红.
    MUTATION: task_type / route_id / started_ts 任一不透传 -> 红.
    """
    proj, _b = _projector()

    # 先看它非终态(跃迁规则要求先见到非终态那一半)
    proj.observe_task({"task_id": "t-x", "state": "running"})
    # 终态: p3 的事件 detail 长这样
    proj.observe_task({
        "kind": "cancelled", "task_id": "t-x", "state": "cancelled",
        "task_type": "goto", "route_id": "r-night",
        "started_ts": 1788400000.0, "ended_ts": 1788400030.0,
        "duration_sec": None, "reason": "operator_stop"})

    results = [d for d in proj._pending_results]
    assert results, "终态没有产出 result"
    r = results[-1]
    assert r["reason"] == "operator_stop", r
    s_ = r["summary"]
    assert s_["route_id"] == "r-night"
    assert s_["started_ts"] == 1788400000.0
    assert s_["ended_ts"] == 1788400030.0
    assert s_["duration_sec"] is None, (
        "未知时长被填成了 %r" % s_["duration_sec"])


def test_a_known_duration_survives_the_projector_too():
    """反向: 有值时要带出去, 否则恒 None 的实现也能过上面那条.

    MUTATION: _result_for 恒传 None -> 红.
    """
    proj, _b = _projector()
    proj.observe_task({"task_id": "t-y", "state": "running"})
    proj.observe_task({
        "kind": "completed", "task_id": "t-y", "state": "done",
        "task_type": "goto", "duration_sec": 42.5})
    r = proj._pending_results[-1]
    assert r["summary"]["duration_sec"] == pytest.approx(42.5)
