"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_result.py
Brief: state/task 权威终态的一次性与形状判据 (B-5)

Description:
*** 本文件的重心是"只发一条, 且一定发一条" -- 两个方向的代价都很大:
  重复发  Qt 的操作记录里同一个任务出现多条完成; 而 v2.0 S8 要求 task
          event 与 result 用相同 task_id 和结果值, 审计时对不上
  漏发    操作员看到任务永远停在 running, 他会去中止一个已经完成的任务
中间没有"发多了没关系"这种余地.

*** 判定按[跃迁]不按[当前值], 这是最容易写错的一处.
按当前值判的话, 网关重启后第一次收到 state/task 就会把一个早已完成的任务
当成"刚刚完成"再发一条 result. 用例里专门有一条模拟这件事.

Boundaries: 纯逻辑层. 与投影器的接线在 test_cloud_projector.py.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


def _tracker():
    from xbrain.p5_gateway.outbound.task_result import TaskResultTracker

    return TaskResultTracker()


# --- 一次性 -----------------------------------------------------------

def test_a_transition_into_a_terminal_state_fires_once():
    """*** 核心. running -> completed 产出一条, 之后再怎么重播都不产.

    MUTATION: 去掉 _done 集合的判断 -> 这里红.
    """
    t = _tracker()

    assert t.observe("t-1", "running") is None
    assert t.observe("t-1", "completed") == ("t-1", "done")
    # p3 的 state/task 是变化即发的广播, 同一个终态会被重播.
    for _ in range(5):
        assert t.observe("t-1", "completed") is None, "重播产出了第二条"


def test_a_task_already_terminal_on_first_sight_does_not_fire():
    """*** 网关重启的场景.

    重启后第一次收到 state/task 时, 里面可能有一个早已完成的任务. 按当前值
    判的话它会被当成"刚刚完成"再发一条 result -- 而 Qt 十分钟前就收到过了.

    MUTATION: 把 observe 改成只看当前值(不查 previous) -> 这里红.
    """
    t = _tracker()

    assert t.observe("t-old", "completed") is None, (
        "首次见到就是终态却产出了 result -- 那是重启后的重放, 不是跃迁")


def test_a_stale_running_broadcast_cannot_produce_a_second_result():
    """*** _done 这道闸守的是这个序列, 补于变异体实测之后.

    去掉 _done 的变异体[没有变红] -- 说明这道闸当时一条断言都没覆盖.
    原因是 observe 在产出后会把条目从 _last 里 pop 掉, 于是普通重播走的是
    "previous is None"那条分支, 与 _done 无关. 两道闸看起来在守同一件事,
    实际只有一道在起作用, 而另一道从没被验证过.

    _done 真正守的是这个: 终态之后又来了一条非终态的广播(重连时的乱序,
    或者 p3 补发了一条旧状态), 于是 _last 里重新有了 "running"; 再来一条
    completed 就会产出第二条 result -- 而任务只结束过一次.

    * 保留两道闸而不是删掉一道: "只发一条"这个保证不该依赖 _last 的 pop
    时机那种微妙交互. 但一道没被验证的闸等于没有(CLAUDE.md 3.3), 所以补
    这条用例.

    MUTATION: 去掉 _done 集合的判断 -> 这里红.
    """
    t = _tracker()

    t.observe("t-1", "running")
    assert t.observe("t-1", "completed") == ("t-1", "done")
    t.observe("t-1", "running")            # 乱序/补发的旧状态
    assert t.observe("t-1", "completed") is None, (
        "同一个任务产出了第二条 result -- 它只结束过一次")


def test_each_terminal_state_maps_to_its_v2_word():
    """机内词与 v2.0 词不同, 逐个对.

    * canceled / cancelled 两种拼写机内都出现过, 都要接 -- 一个只认其中
    一种的实现会让另一种拼写的任务永远发不出 result, 而它看起来只是
    "某些任务没有结果".
    """
    for internal, want in (("completed", "done"), ("done", "done"),
                           ("failed", "failed"), ("cancelled", "cancelled"),
                           ("canceled", "cancelled")):
        t = _tracker()
        t.observe("t", "running")
        assert t.observe("t", internal) == ("t", want), internal


def test_non_terminal_states_never_fire():
    """queued / running / paused 都不产 result.

    产了的话 Qt 会在任务刚开始时就收到一条"已完成".
    """
    t = _tracker()

    for state in ("queued", "running", "paused", "arming", "weird", None, 7):
        assert t.observe("t-1", state) is None, state


def test_two_tasks_are_tracked_independently():
    """一个任务的终态不得抑制另一个的.

    共用一个标志位的实现会让同一秒内结束的第二个任务永远没有 result.
    """
    t = _tracker()

    t.observe("t-1", "running")
    t.observe("t-2", "running")
    assert t.observe("t-1", "completed") == ("t-1", "done")
    assert t.observe("t-2", "failed") == ("t-2", "failed")


def test_an_in_flight_entry_is_dropped_once_it_terminates():
    """内存: 终结后条目从在途表里删掉.

    不删的话, 一台连续跑几个月的机器上这张表只增不减 -- 而它每条任务
    一个条目.
    """
    t = _tracker()

    t.observe("t-1", "running")
    assert t.pending() == 1
    t.observe("t-1", "completed")
    assert t.pending() == 0


# --- 形状 -------------------------------------------------------------

def _result(**over):
    from xbrain.p5_gateway.outbound.task_result import build_result

    kw = {"task_id": "task-goto-001", "task_type": "GOTO_KEYPOINT",
          "state": "done", "completed_count": 1, "total_count": 1,
          "distance_m": 86.4, "duration_sec": 85.35,
          "started_ts": 1785732040.2, "ended_ts": 1785732125.55,
          "route_id": "r-route_north", "route_rev": 3}
    kw.update(over)
    return build_result(**kw)


def test_a_done_result_carries_every_v2_field():
    """v2.0 S3.3 的九个顶层字段 + 八个 summary 字段, 一个不能少."""
    d = _result()

    assert set(d) == {"msg_id", "message_type", "task_id", "task_type",
                      "state", "result_code", "reason", "summary", "detail"}
    assert d["message_type"] == "result"
    assert set(d["summary"]) == {"completed_count", "total_count",
                                 "distance_m", "duration_sec", "started_ts",
                                 "ended_ts", "route_id", "route_rev"}
    assert d["result_code"] == 0 and d["reason"] == ""


def test_a_failure_must_carry_a_reason():
    """*** v2.0 S3.3 逐字: 失败/取消必填 reason.

    一条 state=failed 而 reason 为空的 result, 在 Qt 上是"任务失败"四个字
    加一片空白 -- 操作员唯一能做的是重试, 而如果失败原因是围栏外, 重试会
    再失败一次.
    """
    from xbrain.p5_gateway.outbound.state_projection import ProjectionError

    with pytest.raises(ProjectionError):
        _result(state="failed", result_code=2006, reason="")
    # 反向: 带了就该通过.
    d = _result(state="failed", result_code=2006, reason="目标点在围栏外")
    assert d["reason"] == "目标点在围栏外"


def test_result_code_and_state_cannot_disagree():
    """*** 与 ack 的 accepted 同一个理由.

    两个字段表达同一件事时它们迟早会不一致, 而 {state:"failed",
    result_code:0} 会让 Qt 走成功分支 -- 操作员以为任务完成了.
    """
    from xbrain.p5_gateway.outbound.state_projection import ProjectionError

    with pytest.raises(ProjectionError):
        _result(state="done", result_code=2001)
    with pytest.raises(ProjectionError):
        _result(state="cancelled", result_code=0, reason="用户取消")


def test_an_out_of_set_result_state_raises():
    """result 的 state 闭集是 done|failed|cancelled 三值.

    NO 与 snapshot 的七值任务态不是一回事: snapshot 描述"任务正处在哪",
    result 描述"它以什么方式结束". 把 completed 直接发过去就是越界.
    """
    from xbrain.p5_gateway.outbound.state_projection import ProjectionError

    for bad in ("completed", "running", "ok", ""):
        with pytest.raises(ProjectionError):
            _result(state=bad, result_code=1, reason="x")


def test_inapplicable_distance_stays_null_not_zero():
    """*** 不适用为 null(v2.0 逐字), 不是 0.

    一个 SET_ALARM_CONFIG 没有里程可言. 填 0 会让 Qt 显示"行进 0 米",
    而那是一个关于运动的陈述 -- 它根本没动过, 也根本不该动.
    """
    d = _result(task_type="SET_ALARM_CONFIG", distance_m=None,
                route_id=None, route_rev=None)

    assert d["summary"]["distance_m"] is None
    assert d["summary"]["route_id"] is None, (
        "route_id 被填成了空串或 0 -- Qt 会拿它去查一条不存在的路径")


def test_every_result_gets_a_fresh_msg_id():
    """v2.0 S1.2: 每条消息一个新 ID.

    复用的话 Qt 的幂等窗口会把第二条当成重复丢掉 -- 而两条 result 属于
    两个不同的任务.
    """
    assert _result()["msg_id"] != _result()["msg_id"]


# --- 与投影器的接线 ---------------------------------------------------

def test_the_projector_emits_a_result_on_termination():
    """*** 接线判据: tracker 写好了, 投影器要真的用它.

    MUTATION: 删掉 tick 里的 _drain_results 循环 -> 这里红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import CloudProjector

    class _Bridge:
        def __init__(self):
            self.published = []

        def publish_state(self, name, data):
            self.published.append((name, data))

    class _Clock:
        t = 1000.0

        def __call__(self):
            return self.t

    bridge, clock = _Bridge(), _Clock()
    proj = CloudProjector(bridge, now_mono=clock)

    running = {"task_id": "t-1", "task_type": "GOTO_KEYPOINT",
               "state": "running"}
    proj.tick({"tasks": [running]})
    clock.t += 1.0
    done = dict(running, state="completed", completed_count=1,
                total_count=1, distance_m=86.4, duration_sec=85.3)
    sent = proj.tick({"tasks": [done]})

    assert "state/task:result" in sent, "任务结束了却没有 result: %s" % sent
    results = [d for n, d in bridge.published
               if n == "state/task" and d.get("message_type") == "result"]
    assert len(results) == 1
    assert results[0]["state"] == "done" and results[0]["task_id"] == "t-1"


def test_the_result_goes_out_before_the_snapshot_that_drops_it():
    """*** 顺序有意义.

    反过来的话, 刚完成的任务会先从 snapshot 的 current 里消失(它不再
    running), 然后才收到 result -- Qt 中间那一瞬看到的是"没有任务在跑,
    也没有结果", 而操作员正盯着屏幕等结果.

    MUTATION: 把 tick 里的 _drain_results 循环挪到 for name, build 之后
    -> 这里红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import CloudProjector

    class _Bridge:
        def __init__(self):
            self.order = []

        def publish_state(self, name, data):
            self.order.append((name, data.get("message_type")))

    bridge = CloudProjector(_Bridge(), now_mono=lambda: 1000.0)
    running = {"task_id": "t-1", "task_type": "GOTO_KEYPOINT",
               "state": "running"}
    bridge.tick({"tasks": [running]})
    bridge._bridge.order.clear()
    bridge.tick({"tasks": [dict(running, state="completed")]})

    kinds = [m for n, m in bridge._bridge.order if n == "state/task"]
    assert kinds[0] == "result", "snapshot 抢在了 result 前面: %s" % kinds


def test_main_wiring_observes_every_broadcast_not_just_the_tick():
    """*** 守跃迁规则的[前提].

    规则是"非终态 -> 终态才发", 它成立的前提是观察点在每一次广播上.
    只在 10 Hz 的 tick 上采样的话, 一个 50 ms 内 running -> completed 的
    快任务只会被看到 completed 一次, 于是永远发不出 result -- Qt 一直等
    一个已经完成的任务.

    规则写在 task_result.py, 前提落在 main_wiring.py, 两个文件 -- 单看
    任何一边都看不出这层依赖, 所以要有一条断言把它们钉在一起.

    MUTATION: 删掉 _on_state_task 里的 observe_task 循环 -> 这里红.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p5_gateway" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    tree = ast.parse(src)
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and getattr(n.func, "attr", "") == "observe_task"]
    assert len(calls) == 1, (
        "main_wiring 里对 observe_task 的调用有 %d 处 -- 跃迁规则的前提"
        "不成立, 快任务的 result 会全部丢失" % len(calls))

    # 而且它必须在 state/task 的回调里, 不在别处.
    holder = [f for f in ast.walk(tree)
              if isinstance(f, ast.FunctionDef)
              and f.name == "_on_state_task"
              and any(getattr(n.func, "attr", "") == "observe_task"
                      for n in ast.walk(f) if isinstance(n, ast.Call))]
    assert holder, "observe_task 不在 _on_state_task 回调里"


def test_a_fast_task_still_gets_a_result_through_the_callback_path():
    """反向验证上一条描述的场景: running 与 completed 之间没有 tick.

    回调路径逐条观察, 所以跃迁不会丢; 而 tick 只负责把攒下的发出去.
    """
    from xbrain.p5_gateway.runtime.cloud_state import CloudProjector

    class _Bridge:
        def __init__(self):
            self.published = []

        def publish_state(self, name, data):
            self.published.append((name, data))

    bridge = _Bridge()
    proj = CloudProjector(bridge, now_mono=lambda: 1000.0)

    # 两条广播之间一次 tick 都没有 -- 这正是快任务的样子.
    proj.observe_task({"task_id": "t-9", "task_type": "STOP_TASK",
                       "state": "running"})
    proj.observe_task({"task_id": "t-9", "task_type": "STOP_TASK",
                       "state": "completed"})
    proj.tick({"tasks": []})

    results = [d for n, d in bridge.published
               if n == "state/task" and d.get("message_type") == "result"]
    assert len(results) == 1, "快任务的 result 丢了: %s" % bridge.published
    assert results[0]["task_id"] == "t-9"
