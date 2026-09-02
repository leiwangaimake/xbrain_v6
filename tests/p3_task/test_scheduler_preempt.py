"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_scheduler_preempt.py
Brief: 15 S6.1 调度决策树 + S6.3 抢占语义 + 调度取数范围

Description:
守 2026-09-02 补的三处. 逐项核对甲方下发任务在系统内的行为时查出:

*** 一 调度取数不按状态过滤(最严重).
list_by_priority 原来是 "ORDER BY priority DESC LIMIT 32", 无 WHERE. 终态任务
(cancelled/done/failed)也按优先级占名额. 实测库里 145 行时, 那 32 行里 pending
数为 0 -- 107 条 pending [永远进不了调度视野]. 它们不是排队等, 是调度器看不见,
phase 1 遍历不到, 永远不会 pending -> ready. 而且持续恶化: 终态行不会消失,
最终把视野塞满, 任务系统在积累一定数量后静默停止工作.

*** 二 抢占未实现.
15 S6.1 的决策树第 2 步"ready 队列中有更高优先级任务 -> 抢占(S6.3)"在代码里
根本没有 -- 原实现只有 "if not any(running): dispatch". 后果: 甲方下发的
priority=80 任务被一条 priority=40 的本地语音任务无限期挡住.

*** 三 running 任务没有 started_at.
15 S9.5 有这一列, duration_sec 的 DDL CHECK 依赖终态时间戳, 而 running 没有
开始时间 => 任务时长永远算不出来.

Boundaries: 只测调度决策与状态写. NO 不测挂起动作的第 1/2 步(停止运动与采样
进度) -- 那两步依赖 P3->P1 的运动链路(CLD-2)与进度流(EX-3), 两者都未建,
driver 的注释里逐条登记了.
"""
from __future__ import annotations

import asyncio

import pytest

pytestmark = pytest.mark.no_device


class _FakeDao:
    """按 (task_id, priority, submit_seq, state) 建模, 与真 DAO 同形状."""

    def __init__(self, rows):
        # rows: list of dict(task_id, priority, submit_seq, state)
        self.rows = {r["task_id"]: dict(r) for r in rows}
        self.calls = []

    async def list_by_priority(self, limit: int = 512):
        active = [r for r in self.rows.values()
                  if r["state"] in ("pending", "scheduled", "blocked",
                                    "ready", "running", "suspended")]
        active.sort(key=lambda r: (-r["priority"], r["submit_seq"]))
        return [(r["task_id"], r["priority"], r["submit_seq"], r["state"])
                for r in active[:limit]]

    async def fetch_by_id(self, task_id):
        """返回带属性的对象 -- validate_pending 读 row.task_type / .priority /
        .mission_json, 传 dict 会 AttributeError. 用最小对象而不是真 TaskRow:
        真 TaskRow 有三十多个必填字段, 建一个要写满屏, 而本文件测的是调度
        决策不是行的完整性."""
        r = self.rows.get(task_id)
        if r is None:
            return None

        class _Row:
            task_type = "goto"
            priority = r["priority"]
            mission_json = '{"source":"local","params":{}}'
        return _Row()

    async def update_state(self, task_id, state, updated_ms):
        self.rows[task_id]["state"] = state
        self.calls.append(("update_state", task_id, state))
        return 1

    async def dispatch_task(self, task_id, updated_ms, started_at,
                            started_mono):
        self.rows[task_id]["state"] = "running"
        self.rows[task_id]["started_at"] = started_at
        self.rows[task_id]["started_mono"] = started_mono
        self.calls.append(("dispatch", task_id, started_at))
        return 1

    async def preempt_task(self, task_id, reason, updated_ms):
        self.rows[task_id].update(state="suspended", suspend_kind="yielding",
                                  suspend_reason=reason)
        self.calls.append(("preempt", task_id, reason))
        return 1


class _FakeConn:
    async def commit(self):
        return None


async def _noop(task_id, state, reason):
    return None


def _tick(rows, started_at="2026-09-02T08:00:00Z"):
    from xbrain.p3_task.schedule.driver import scheduler_tick
    dao = _FakeDao(rows)
    made = asyncio.run(scheduler_tick(_FakeConn(), dao, now_mono_ms=1000,
                                      on_transition=_noop,
                                      started_at=started_at))
    return dao, made


# --- 一 取数范围 -----------------------------------------------------

def test_terminal_tasks_do_not_occupy_the_scheduler_window():
    """*** 终态任务不得占调度名额.

    原实现无 WHERE, cancelled/done 按优先级排在前面就把 pending 挤出视野.
    这里放一批高优先级的终态行 + 一条低优先级 pending: pending 必须仍被看到.

    变异体: list_by_priority 去掉 WHERE => 本条红(真 DAO 侧由
    test_list_by_priority_filters_terminal_states 守).
    """
    rows = [{"task_id": "done-%d" % i, "priority": 95, "submit_seq": i,
             "state": "done"} for i in range(40)]
    rows.append({"task_id": "t-low", "priority": 10, "submit_seq": 99,
                 "state": "pending"})
    dao, made = _tick(rows)
    assert any(m[0] == "t-low" for m in made), (
        "低优先级 pending 被终态任务挤出了视野: %r" % made)


@pytest.mark.no_device
def test_list_by_priority_filters_terminal_states():
    """*** 真 DAO 的 SQL 必须带状态过滤.

    上一条用的是 fake; 这一条查真实现的 SQL 文本, 否则 fake 与真 DAO 会各自
    "对"而线上仍然坏 -- 那正是这个缺陷的形状.

    变异体: 去掉 WHERE 子句 => 本条红.
    """
    import inspect
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    src = inspect.getsource(TasksDAO.list_by_priority)
    assert "WHERE state IN" in src, "调度取数没有状态过滤"
    for st in ("pending", "ready", "running", "suspended"):
        assert "'%s'" % st in src, "活跃态 %s 不在取数范围" % st
    for st in ("done", "cancelled", "failed"):
        assert "'%s'" % st not in src, "终态 %s 被算进了调度视野" % st


# --- 二 抢占 ---------------------------------------------------------

def test_higher_priority_preempts_the_running_task():
    """*** 15 S6.1 决策树第 2 步 + S6.3 第一行.

    更高优先级到达 -> 当前任务 suspended(yielding/preempted), 新任务 running.

    变异体: 删掉 elif ready 那一支 => 本条红.
    """
    dao, made = _tick([
        {"task_id": "t-run", "priority": 40, "submit_seq": 1,
         "state": "running"},
        {"task_id": "t-hi", "priority": 80, "submit_seq": 2, "state": "ready"},
    ])
    assert dao.rows["t-run"]["state"] == "suspended", "高优先级没有抢占"
    assert dao.rows["t-run"]["suspend_kind"] == "yielding", (
        "抢占的 kind 必须是 yielding(CR-8 配对), 实得 %r"
        % dao.rows["t-run"].get("suspend_kind"))
    assert dao.rows["t-run"]["suspend_reason"] == "preempted"
    assert dao.rows["t-hi"]["state"] == "running", "新任务没有启动"


def test_equal_priority_does_not_preempt():
    """*** S6.3 表第二行逐字: "同优先级任务到达 -> 不抢占, 入队等待".

    用 >= 的话两条同优先级会互相抢占, 每拍换一次, 谁也跑不完 -- 而每次抢占
    都是一次合法的状态迁移, 没有任何东西会报错.

    变异体: 把 top_prio > cur_prio 改成 >= => 本条红.
    """
    dao, _made = _tick([
        {"task_id": "t-run", "priority": 50, "submit_seq": 1,
         "state": "running"},
        {"task_id": "t-eq", "priority": 50, "submit_seq": 2, "state": "ready"},
    ])
    assert dao.rows["t-run"]["state"] == "running", "同优先级发生了抢占"
    assert dao.rows["t-eq"]["state"] == "ready", "同优先级任务被启动了"


def test_lower_priority_does_not_preempt():
    """低优先级到达当然不抢占 -- 这条防的是把比较写反."""
    dao, _made = _tick([
        {"task_id": "t-run", "priority": 80, "submit_seq": 1,
         "state": "running"},
        {"task_id": "t-lo", "priority": 40, "submit_seq": 2, "state": "ready"},
    ])
    assert dao.rows["t-run"]["state"] == "running"
    assert dao.rows["t-lo"]["state"] == "ready"


def test_preempt_is_suspend_not_cancel():
    """*** S6.3 末行: "抢占必须是挂起不是取消" -- U07 断点续跑的前提.

    取消掉的话任务再也回不来, 而 yielding 的恢复条件是"让位对象一进终态就
    自动回 ready"(15 S3.2), 不需要人工干预.

    *** 查[真 DAO 的 SQL], NO 不能用 fake 的行状态断言.
    2026-09-02 第一版本条断言 dao.rows["t-run"]["state"] == "suspended",
    而那个 state 是 fake 自己写进去的 -- 把真 DAO 的 SQL 改成
    state='cancelled', 本条依然全绿. 一条测不到被测代码的判据, 与真判据
    在报告里长得一模一样(CLAUDE.md 3.2 形态1: 有没有一个"什么都不做"的
    实现能通过它).

    变异体: preempt_task 的 SQL 写 state='cancelled' => 本条红.
    """
    import inspect
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    src = inspect.getsource(TasksDAO.preempt_task)
    assert "state='suspended'" in src, (
        "preempt_task 写的不是 suspended -- 抢占变成了别的动作")
    for bad in ("state='cancelled'", "state='failed'", "state='done'"):
        assert bad not in src, "抢占把任务写成了终态 %s" % bad

    # 决策层面同时确认: 抢占后那条任务仍在活跃集里(15 S3.2), 不是终态.
    dao, _made = _tick([
        {"task_id": "t-run", "priority": 40, "submit_seq": 1,
         "state": "running"},
        {"task_id": "t-hi", "priority": 95, "submit_seq": 2, "state": "ready"},
    ])
    assert dao.rows["t-run"]["state"] in ("ready", "running", "suspended"), (
        "抢占后任务离开了活跃集: %r" % dao.rows["t-run"]["state"])


def test_preempt_kind_is_fixed_not_caller_supplied():
    """*** preempt_task 不接受调用方指定 kind.

    CR-8 配对(11 S4.4): kind == 'yielding' IFF reason in
    {preempted, mode_takeover}. 让调用方传 kind 就给了它写出
    yielding+low_battery 的机会, 那会被 DDL CHECK 拒掉. 在 DAO 里定死,
    违规不可能发生.

    变异体: 给 preempt_task 加一个 kind 形参 => 本条红.
    """
    import inspect
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    sig = inspect.signature(TasksDAO.preempt_task)
    assert "kind" not in sig.parameters, (
        "preempt_task 暴露了 kind 形参, CR-8 配对不再由 DAO 保证")
    src = inspect.getsource(TasksDAO.preempt_task)
    assert "suspend_kind='yielding'" in src


# --- 三 started_at ---------------------------------------------------

def test_dispatch_records_started_at():
    """*** ready -> running 必须落 started_at.

    15 S9.5 有这一列, duration_sec 的 DDL CHECK 依赖终态时间戳. running 却
    没有开始时间的话, 任务时长永远算不出来. 2026-09-02 实测库里那条 running
    的 started_at 是 NULL.

    变异体: dispatch 改回 update_state => 本条红.
    """
    dao, _made = _tick([
        {"task_id": "t-a", "priority": 50, "submit_seq": 1, "state": "ready"},
    ], started_at="2026-09-02T08:00:00Z")
    assert dao.rows["t-a"]["state"] == "running"
    assert dao.rows["t-a"].get("started_at") == "2026-09-02T08:00:00Z", (
        "started_at 没落: %r" % dao.rows["t-a"].get("started_at"))
    assert dao.rows["t-a"].get("started_mono") is not None, (
        "started_mono 没落 -- 时长要用单调钟算(CLK-C1)")


def test_started_at_comes_from_the_caller_not_a_second_clock():
    """*** driver 里不得有第二处时钟来源.

    started_at 与 created_at 必须同一口径(15 S9.5 都是墙钟 UTC ISO), 由
    main_wiring 的 _now_utc_iso() 生成后传入. driver 自己取时间的话, 两个
    时间戳会来自两个 time 调用, 而 CLK-C1 的理由(时间从外面传)同样适用.

    变异体: driver 里 import time 自己生成 => 本条红.
    """
    import inspect
    from xbrain.p3_task.schedule import driver
    src = inspect.getsource(driver)
    assert "started_at: str" in src, "scheduler_tick 没有接收 started_at"
    for bad in ("datetime.utcnow", "time.gmtime", "strftime"):
        assert bad not in src, "driver 里出现了第二处时钟来源: %s" % bad
