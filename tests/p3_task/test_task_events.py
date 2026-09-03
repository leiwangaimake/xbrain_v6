"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_events.py
Brief: 11 S6.2 task-event mapping -- (to_state, reason) -> kind/sev

Description:
Pins the 11 S6.2 task row (info/warn, accept/reject/start/complete/fail): a
validate-fail reason is a reject (warn); ready/running/succeeded are info;
failed/aborted/cancelled are warn; internal states (pending/suspended) produce no
event. Mutations paired per 3.3.
"""
from __future__ import annotations

import pytest

from xbrain.p3_task.state.machine import TRANSITIONS
from xbrain.p3_task.state.task_events import (
    _TRANSITION_EVENT, task_event_for_transition,
)

pytestmark = pytest.mark.no_device


# --------------------------------------------------- 判别依据是迁移, 不是 reason

def test_only_the_admission_failure_is_a_rejection():
    """*** 这条是本次重写的靶心.

    拒绝有唯一的迁移特征: 准入校验没过, pending -> failed. 其余六种带 reason
    的迁移都不是拒绝, 而旧实现按 "reason 非空" 判, 把它们全报成了 rejected --
    2026-09-03 甲方按一次暂停, 界面上就是一条 "task ... rejected".

    MUTATION: 把 ("pending","failed") 改成别的 kind -> 红.
    """
    assert task_event_for_transition("pending", "failed") == ("rejected", "warn")


def test_pause_emits_no_event_at_all():
    """S6.2 那一类逐字是"接受/拒绝/开始/完成/失败", 没有暂停. 操作员看暂停要
    从 state/task 快照看(state=paused, 1 Hz), 事件流不是状态镜像.

    MUTATION: 给 ("running","suspended") 填任何一个 kind -> 红.
    """
    assert task_event_for_transition("running", "suspended") is None


def test_resume_emits_no_event_either():
    """恢复同理. 顺带守住"让位结束自动恢复"那条(driver 的 phase 1b), 它走的是
    同一条边.

    MUTATION: 给 ("suspended","ready") 填 kind -> 红.
    """
    assert task_event_for_transition("suspended", "ready") is None


def test_preemption_is_not_a_rejection():
    """被高优先级抢占 = running -> suspended, 与操作员暂停同一条边. 旧实现因为
    reason="preempted" 非空而报 rejected.

    MUTATION: 恢复 `if reason:` 那条规则 -> 红(它会让本条返回 rejected).
    """
    assert task_event_for_transition("running", "suspended") is None


def test_cancel_says_cancelled_not_rejected():
    """甲方按停止 -> running -> cancelled. 旧实现报 rejected(reason 是操作员
    填的 operator_stop).

    MUTATION: 把 ("running","cancelled") 改成 rejected -> 红.
    """
    assert task_event_for_transition("running", "cancelled") == \
        ("cancelled", "warn")
    # 排队中被取消也一样是取消, 不是拒绝.
    assert task_event_for_transition("ready", "cancelled") == \
        ("cancelled", "warn")


def test_a_deferred_task_is_accepted_not_rejected():
    """定时任务与等依赖的任务都是[受理了], 只是还不能跑. 旧实现同样按 reason
    非空报成 rejected -- 甲方提交一条定时任务会收到"被拒绝".

    MUTATION: 把这两条改成 None 或 rejected -> 红.
    """
    assert task_event_for_transition("pending", "scheduled") == \
        ("accepted", "info")
    assert task_event_for_transition("pending", "blocked") == \
        ("accepted", "info")


# ------------------------------------------------------------- 正向的那几条不能丢

def test_the_five_s6_2_kinds_still_come_out():
    """重写不能把原本对的那半弄丢.

    MUTATION: 删掉 ("pending","ready") 或 ("ready","running") -> 红.
    """
    assert task_event_for_transition("pending", "ready") == ("accepted", "info")
    assert task_event_for_transition("ready", "running") == ("started", "info")
    assert task_event_for_transition("running", "done") == ("completed", "info")
    assert task_event_for_transition("running", "failed") == ("failed", "warn")
    assert task_event_for_transition("pending", "failed") == ("rejected", "warn")


def test_completion_finally_emits_something():
    """*** 旧表的键写的是 "succeeded", 而状态机里的终态叫 "done" -- 于是任务
    跑完[永远不发完成事件]. 今天没暴露只因为还没有执行器能让任务跑完.

    MUTATION: 把键改回 "succeeded" -> 红(("running","done") 查不到就抛).
    """
    assert task_event_for_transition("running", "done") == ("completed", "info")


def test_the_power_off_bookkeeping_does_not_double_report():
    """任务终结时已经发过一条事件, 关机前再发一条就是同一件事上报两次.

    MUTATION: 给 ("done","wait_for_power_off") 填 kind -> 红.
    """
    for frm in ("done", "failed", "cancelled", "needs_review"):
        assert task_event_for_transition(frm, "wait_for_power_off") is None


# ------------------------------------------------------------------ 完备性判据

def test_the_table_covers_every_edge_the_state_machine_can_take():
    """*** 表缺一条边, 那条迁移在运行期就抛.

    这条判据是那个 raise 的前提: 它保证 raise 在生产里不可达, 同时保证[新增
    一条边必须在这里表态]. 没有它的话, 加一条边就会静默不发事件 -- 正是
    running->done 那条的老毛病.

    MUTATION: 从 _TRANSITION_EVENT 删掉任意一条 -> 红.
    """
    machine_pairs = {(frm, to) for (frm, _ev), to in TRANSITIONS.items()}
    missing = machine_pairs - set(_TRANSITION_EVENT)
    assert not missing, "状态机有边而事件表没表态: %r" % sorted(missing)


def test_the_table_has_no_edge_the_state_machine_cannot_take():
    """反向差集: 表里的键必须都是真实存在的边.

    旧表的 "succeeded"/"aborted" 就是这么混进来的 -- 两个状态机里根本没有的
    状态, 写在表里没人发现, 而真正该映射的 "done" 反而漏了.

    MUTATION: 往表里加一条 ("running","succeeded") -> 红.
    """
    machine_pairs = {(frm, to) for (frm, _ev), to in TRANSITIONS.items()}
    extra = set(_TRANSITION_EVENT) - machine_pairs
    assert not extra, "事件表里有状态机走不到的迁移: %r" % sorted(extra)


def test_an_impossible_transition_raises_instead_of_returning_none():
    """静默返回 None 会让新增的边悄悄不发事件. 抛出来才有人看见.

    MUTATION: 把 raise 换成 return None -> 红.
    """
    with pytest.raises(KeyError) as exc:
        task_event_for_transition("running", "pending")   # 状态机里没有这条边
    assert "running" in str(exc.value) and "pending" in str(exc.value)


# --------------------------------------------- 接线: _make_publish 用哪一对判别

@pytest.mark.asyncio
async def test_the_publish_seam_decides_on_the_from_to_pair():
    """*** 纯函数对了不等于接线传对了.

    变异测试显示: 把 _make_publish 里的 task_event_for_transition(from, to) 改成
    (to, to), 上面所有单测照样全绿 -- 因为它们测的是函数本身. 判别既然按 (from,
    to) 做, 就要有一条断言盯着[接线实际喂进去的那一对].

    MUTATION: 改成 (to_state, to_state) -> 红(running->suspended 会变成
    suspended->suspended, 表里没有, 抛 KeyError).
    """
    import json

    from xbrain.p3_task.runtime.main_wiring import _make_publish

    class _Pub:
        def __init__(self):
            self.puts = []

        def put(self, payload):
            self.puts.append(json.loads(payload.decode("utf-8")))

    emitted = []

    def _emit(task_id, to_state, kind, sev):
        emitted.append((task_id, to_state, kind, sev))

    pub = _Pub()
    publish = _make_publish(pub, _emit)

    # 取消: running -> cancelled, 且 reason 非空(甲方填的). 旧实现在这里报
    # rejected, 新实现必须报 cancelled.
    await publish("t-1", "running", "cancelled", "operator_stop")
    assert emitted == [("t-1", "cancelled", "cancelled", "warn")], emitted

    # 暂停: running -> suspended, reason 同样非空 -> 一条事件都不发.
    emitted.clear()
    await publish("t-2", "running", "suspended", "operator_pause")
    assert emitted == [], emitted


@pytest.mark.asyncio
async def test_the_publish_seam_still_logs_the_reason_it_no_longer_judges_by():
    """reason 不再参与判别, 但它是给人看的, 不能顺手丢掉 -- 日志里没有它,
    "任务为什么停了"就查不出来了.

    MUTATION: 从 _publish 的日志里去掉 reason -> 红.
    """
    import inspect

    from xbrain.p3_task.runtime.main_wiring import _make_publish

    src = inspect.getsource(_make_publish)
    assert "reason" in src
    assert "from_state, to_state, reason" in src, (
        "日志没有同时带上 from/to/reason, 出问题时看不出迁移是从哪来的")
