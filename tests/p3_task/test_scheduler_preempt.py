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
            # V-8 读它; 不在 rows 里给就当作"这条任务不引用路径".
            route_geo_id = r.get("route_geo_id")
        return _Row()

    async def update_state(self, task_id, state, updated_ms):
        self.rows[task_id]["state"] = state
        self.calls.append(("update_state", task_id, state))
        return 1

    async def dispatch_task(self, task_id, updated_ms, started_at,
                            started_mono, started_boot):
        self.rows[task_id].update(state="running", started_at=started_at,
                                  started_mono=started_mono,
                                  started_boot=started_boot)
        self.calls.append(("dispatch", task_id, started_at))
        return 1

    async def finish_task(self, task_id, state, updated_ms, finished_at,
                          duration_sec):
        self.rows[task_id].update(state=state, finished_at=finished_at,
                                  duration_sec=duration_sec)
        self.calls.append(("finish", task_id, state, duration_sec))
        return 1

    async def list_yielding(self, limit: int = 512):
        out = [r for r in self.rows.values()
               if r["state"] == "suspended"
               and r.get("suspend_kind") == "yielding"]
        out.sort(key=lambda r: (-r["priority"], r["submit_seq"]))
        return [(r["task_id"], r["priority"], r["submit_seq"],
                 r.get("suspend_reason")) for r in out[:limit]]

    async def resume_task(self, task_id, updated_ms):
        self.rows[task_id].update(state="ready", suspend_kind=None,
                                  suspend_reason=None)
        self.calls.append(("resume", task_id))
        return 1

    async def preempt_task(self, task_id, reason, updated_ms):
        self.rows[task_id].update(state="suspended", suspend_kind="yielding",
                                  suspend_reason=reason)
        self.calls.append(("preempt", task_id, reason))
        return 1


class _FakeConn:
    async def commit(self):
        return None


async def _noop(task_id, from_state, to_state, reason):
    return None


def _tick(rows, started_at="2026-09-02T08:00:00Z", boot_id="boot1"):
    from xbrain.p3_task.schedule.driver import scheduler_tick
    dao = _FakeDao(rows)
    made = asyncio.run(scheduler_tick(_FakeConn(), dao, now_mono_ms=1000,
                                      on_transition=_noop,
                                      started_at=started_at, boot_id=boot_id))
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


# --- yielding 自动恢复 (15 S3.2) -------------------------------------

def test_yielding_task_auto_resumes_when_nothing_is_running():
    """*** 让位对象进终态后, 被抢占的任务自动回 ready, 无需人工.

    15 S3.2 逐字: yielding 的恢复条件是"让位对象一进终态就自动回 ready --
    无需任何外部条件". 状态机同样写着 suspended --resume--> ready
    (yielding: yielded-to task done).

    这条扫描此前[根本不存在] -- machine.py 的注释引用了"the yielding
    auto-resume scan", 但全仓没有实现. 2026-09-02 补抢占之后, 被抢占的任务
    会永久停在 suspended, 抢占反而变成了单向的丢弃.

    变异体: 删掉 phase 1b => 本条红.
    """
    dao, _made = _tick([
        {"task_id": "t-yield", "priority": 40, "submit_seq": 1,
         "state": "suspended", "suspend_kind": "yielding",
         "suspend_reason": "preempted"},
    ])
    assert dao.rows["t-yield"]["state"] in ("ready", "running"), (
        "让位任务没有自动恢复: %r" % dao.rows["t-yield"]["state"])
    assert dao.rows["t-yield"].get("suspend_kind") is None, (
        "恢复后 suspend_kind 没清 -- DDL CHECK 要求非 suspended 时必须为 NULL")


def test_yielding_does_not_resume_while_something_is_still_running():
    """*** 还有任务在跑时不恢复 -- "让位对象一进终态"的那个前提没满足.

    提前恢复的话, 刚回 ready 的任务会立刻参与调度, 而它当初正是因为优先级
    低才被让位 -- 于是它要么再被抢一次, 要么在让位对象还没跑完时插进去.

    变异体: 去掉 busy 判断 => 本条红.
    """
    dao, _made = _tick([
        {"task_id": "t-run", "priority": 95, "submit_seq": 2,
         "state": "running"},
        {"task_id": "t-yield", "priority": 40, "submit_seq": 1,
         "state": "suspended", "suspend_kind": "yielding",
         "suspend_reason": "preempted"},
    ])
    assert dao.rows["t-yield"]["state"] == "suspended", (
        "让位对象还在跑, 却提前恢复了")


def test_passive_suspended_is_never_auto_resumed():
    """*** passive 的挂起绝不自动恢复 -- 那是 15 S3.2 区分两种 kind 的全部理由.

    文档逐字: "合成一种会导致低电暂停的任务在充电任务结束后被误判为可以
    自动恢复, 而实际电量条件可能还没满足". 急停(estop_soft)同理: 15 S11.3
    明令 p3 不自动恢复, 必须等人显式 submit/resume.

    *** 查[真 DAO 的 SQL], NO 不能只靠 fake 的行状态.
    2026-09-02 第一版只用 fake 断言, 而 fake 的 list_yielding 自己做了 kind
    过滤 -- 把真 DAO 的 WHERE 去掉 suspend_kind='yielding', 本条依然全绿.
    与本文件 test_preempt_is_suspend_not_cancel 第一版同一个坑: fake 替被测
    代码做了正确的事(CLAUDE.md 3.2 形态1).

    变异体: list_yielding 的 SQL 去掉 suspend_kind 条件 => 本条红.
    """
    import inspect
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    src = inspect.getsource(TasksDAO.list_yielding)
    assert "suspend_kind='yielding'" in src, (
        "list_yielding 没按 kind 过滤 -- passive 的挂起会被卷进自动恢复")

    dao, _made = _tick([
        {"task_id": "t-estop", "priority": 80, "submit_seq": 1,
         "state": "suspended", "suspend_kind": "passive",
         "suspend_reason": "estop_soft"},
        {"task_id": "t-low", "priority": 70, "submit_seq": 2,
         "state": "suspended", "suspend_kind": "passive",
         "suspend_reason": "low_battery"},
    ])
    for tid in ("t-estop", "t-low"):
        assert dao.rows[tid]["state"] == "suspended", (
            "%s 被自动恢复了 -- passive 必须等条件满足或人工" % tid)


def test_mode_takeover_is_not_resumed_by_this_scan():
    """*** yielding 的两种 reason 恢复条件不同, 本扫描只处理 preempted.

    mode_takeover 的条件是 mode.motion_behavior 回 normal(15 S4.1A), 要订
    state/mode 并由 P2 驱动, 不在调度器的知情范围内. 混进来会让"模式还没
    交回来"的任务被误判为可恢复 -- 而那时 P2 仍在驱动 P1, 两个源会对打.

    变异体: 去掉 reason != "preempted" 的 continue => 本条红.
    """
    dao, _made = _tick([
        {"task_id": "t-mode", "priority": 40, "submit_seq": 1,
         "state": "suspended", "suspend_kind": "yielding",
         "suspend_reason": "mode_takeover"},
    ])
    assert dao.rows["t-mode"]["state"] == "suspended", (
        "mode_takeover 被本扫描恢复了 -- 它的条件是模式交回, 不是让位对象终态")


def test_resume_scan_runs_before_preemption_not_after():
    """*** 顺序不能换: 恢复扫描必须在抢占决策之前.

    放在后面的话, 刚恢复到 ready 的任务会在同一拍里被决策树再抢一次(它优先级
    本来就低于当前 running), 于是 resume->preempt 每拍来回一次 -- 而每一步都
    是合法迁移, 日志里看起来像两条任务在正常调度.

    静态查源码顺序(与 test_wiring_feeds_streaming_not_is_alive 同一手法):
    行为上很难构造一个只在顺序错时才红的用例, 因为错的顺序在单拍内的[最终
    状态]可能相同, 差别要跑很多拍才显现.

    变异体: 把 phase 1b 挪到 phase 2 之后 => 本条红.
    """
    import inspect
    from xbrain.p3_task.schedule import driver
    src = inspect.getsource(driver.scheduler_tick)
    i_resume = src.index("phase 1b")
    i_decide = src.index("phase 2")
    assert i_resume < i_decide, (
        "yielding 恢复扫描排在了调度决策之后 -- 会 resume/preempt 来回抖")


# --- 终态时间戳 (15 S9.5) --------------------------------------------

def test_duration_is_none_across_a_reboot():
    """*** 跨重启 duration_sec 必须是 NULL, NO 不得回退用墙钟差值.

    15 S9.5 逐字: "若终态时的 boot != started_boot, duration_sec 写 NULL,
    不得回退用墙钟差值充数 -- 那正是本组列要消除的东西". 单调钟只在同一次
    开机内可比(11 CLK-C4): 跨重启的 now_mono - started_mono 是个没有意义的
    数, 而它[看起来完全像个正常时长], 没有任何迹象表明它是错的.

    变异体: 去掉 started_boot != boot_id 的分支 => 本条红.
    """
    from xbrain.p3_task.schedule.driver import compute_duration_sec

    # 同一次开机: 算得出
    assert compute_duration_sec(10.0, "bootA", 15_000, "bootA") == 5.0
    # 跨重启: 必须 None
    assert compute_duration_sec(10.0, "bootA", 15_000, "bootB") is None, (
        "跨重启却算出了一个时长 -- 那是个没有意义的数")
    # 没记开始: 也是 None(无从算起), NO 不是 0
    assert compute_duration_sec(None, "bootA", 15_000, "bootA") is None
    assert compute_duration_sec(10.0, "", 15_000, "bootA") is None


def test_duration_excludes_queue_wait():
    """*** 口径是[开始到终态], 不含排队等待(15 S9.5).

    基准必须是 started_mono(派发那一刻), 不是 created_ms(入库那一刻). 用后者
    的话, 一条排了两小时队的任务会报出两小时的执行时长, 而 v2.0 S3.3 的
    summary.duration_sec 逐字是"机上实际执行时长, 不含排队".

    变异体: compute_duration_sec 改用 created_ms => 本条红(签名就不对).
    """
    import inspect
    from xbrain.p3_task.schedule.driver import compute_duration_sec
    sig = inspect.signature(compute_duration_sec)
    assert "started_mono" in sig.parameters, "时长基准不是 started_mono"
    assert "created_ms" not in sig.parameters, (
        "时长用了入库时刻做基准 -- 那会把排队等待算进执行时长")


def test_dispatch_records_started_boot():
    """*** 派发时必须记 started_boot, 否则跨重启判据无据可依.

    compute_duration_sec 要比对 started_boot 与当前 boot; 派发时不记的话
    那个比对恒为"没记开始", duration_sec 永远 NULL -- 一条永远绿的
    "跨重启"判定(CLAUDE.md 3.2 形态1).

    *** 查[真 DAO 的 SQL], NO 不能只靠 fake.
    2026-09-02 本文件第三次踩这个坑: fake 的 dispatch_task 自己写了
    started_boot, 把真 DAO 的 UPDATE 去掉那一列, 判据依然全绿. fake 替被测
    代码做了正确的事(CLAUDE.md 3.2 形态1) -- 前两次分别是 preempt 的
    state='suspended' 与 list_yielding 的 kind 过滤.

    变异体: dispatch_task 的 SQL 去掉 started_boot => 本条红.
    """
    import inspect
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    src = inspect.getsource(TasksDAO.dispatch_task)
    assert "started_boot=?" in src, (
        "dispatch_task 的 SQL 没写 started_boot -- 跨重启判据将永远无据可依")

    dao, _made = _tick([
        {"task_id": "t-a", "priority": 50, "submit_seq": 1, "state": "ready"},
    ], boot_id="bootX")
    assert dao.rows["t-a"].get("started_boot") == "bootX", (
        "started_boot 没落: %r" % dao.rows["t-a"].get("started_boot"))


def test_terminal_writes_finished_at_and_duration():
    """*** apply_motion_result 必须走 finish_task 而不是 update_state.

    update_state 只写 state, 于是任务进终态既没有完成时间也没有时长.
    v2.0 S3.3 的 result.summary.duration_sec 因此无源; 而 15 S9.5 的
    idx_tasks_finished_at 是个部分索引(WHERE finished_at IS NOT NULL),
    列永远为空的话它命中不了任何行, 历史任务查询退化成全表扫.

    变异体: 改回 update_state => 本条红.
    """
    import inspect
    from xbrain.p3_task.schedule import driver
    src = inspect.getsource(driver.apply_motion_result)
    assert "dao.finish_task(" in src, (
        "终态仍走 update_state -- finished_at 与 duration_sec 都不会写")
    assert "compute_duration_sec(" in src, "终态没有算时长"


# --- 接线: 传给回调的 from_state 必须是真实的来源状态 -------------------

def _tick_capturing(rows, started_at="2026-09-02T08:00:00Z", boot_id="boot1"):
    """跑一次 tick, 把 (task_id, from_state, to_state) 全记下来.

    纯函数单测看不见接线传了什么: 变异测试显示, 把抢占那处的 from_state 写成
    "pending" 时全部单测照样绿(查表抛 KeyError, 被 tick 外层的 except 吞掉,
    只多一条日志). 判别既然按 (from, to) 做, 就必须有一条断言盯着真实传进去
    的那一对.
    """
    from xbrain.p3_task.schedule.driver import scheduler_tick
    seen = []

    async def _cap(task_id, from_state, to_state, reason):
        seen.append((task_id, from_state, to_state))

    dao = _FakeDao(rows)
    asyncio.run(scheduler_tick(_FakeConn(), dao, now_mono_ms=1000,
                               on_transition=_cap,
                               started_at=started_at, boot_id=boot_id))
    return seen


def test_dispatch_reports_the_ready_to_running_pair():
    """派发的来源状态是 ready. 传错的话事件表要么查不到(抛), 要么映射成别的
    kind -- 两种都不会被纯函数单测发现.

    MUTATION: driver 的派发处把 "ready" 写成别的 -> 红.
    """
    seen = _tick_capturing([
        {"task_id": "t-1", "priority": 50, "submit_seq": 1, "state": "ready"}])
    assert ("t-1", "ready", "running") in seen, seen


def test_preemption_reports_the_running_to_suspended_pair():
    """*** 抢占的来源状态是 running. 这一对决定了甲方看到的是"无事件"还是
    "被拒绝" -- 本次重写的靶心就在这里.

    MUTATION: driver 的抢占处把 "running" 写成 "pending" -> 红.
    """
    seen = _tick_capturing([
        {"task_id": "t-low", "priority": 40, "submit_seq": 1,
         "state": "running"},
        {"task_id": "t-high", "priority": 90, "submit_seq": 2,
         "state": "ready"}])
    assert ("t-low", "running", "suspended") in seen, seen
    assert ("t-high", "ready", "running") in seen, seen


def test_the_pairs_the_wiring_reports_are_all_known_to_the_event_table():
    """接线传出去的每一对都必须在事件表里有表态 -- 否则运行期抛.

    这条把[接线]与[事件表]钉在一起: 单独看任何一边都发现不了传错的 from_state.
    MUTATION: 任意一处 on_transition 的 from_state 写错 -> 红.
    """
    from xbrain.p3_task.state.task_events import _TRANSITION_EVENT

    seen = _tick_capturing([
        {"task_id": "t-low", "priority": 40, "submit_seq": 1,
         "state": "running"},
        {"task_id": "t-high", "priority": 90, "submit_seq": 2,
         "state": "ready"},
        {"task_id": "t-new", "priority": 30, "submit_seq": 3,
         "state": "pending"}])
    unknown = [(f, t) for _tid, f, t in seen if (f, t) not in _TRANSITION_EVENT]
    assert not unknown, "接线报出了事件表没表态的迁移: %r" % unknown


# --- V-8: 引用的路径必须存在于当前 manifest (v2.0 S2.1) -------------------

def _tick_routes(rows, route_ids, started_at="2026-09-02T08:00:00Z",
                 boot_id="boot1"):
    """跑一拍, 注入一个路径全集查询."""
    from xbrain.p3_task.schedule.driver import scheduler_tick

    async def _list():
        return list(route_ids)

    dao = _FakeDao(rows)
    made = asyncio.run(scheduler_tick(_FakeConn(), dao, now_mono_ms=1000,
                                      on_transition=_noop,
                                      started_at=started_at, boot_id=boot_id,
                                      list_route_ids=_list))
    return dao, made


def test_a_task_naming_a_missing_route_is_rejected_at_admission():
    """*** v2.0 S2.1 逐字: recorded_path_id "必须存在于当前 manifest".

    2026-09-03 实测: 发 r-does_not_exist, 任务被 accepted 并 started -- 机器人
    接了一条它无从执行的任务, 而甲方界面显示"已开始".

    MUTATION: 把 check_v8 从 validate_pending 拿掉 -> 红.
    """
    dao, made = _tick_routes(
        [{"task_id": "t-bad", "priority": 50, "submit_seq": 1,
          "state": "pending", "route_geo_id": "r-nope"}],
        route_ids=["r-perimeter"])
    assert dao.rows["t-bad"]["state"] == "failed", dao.rows["t-bad"]
    assert ("t-bad", "pending", "failed") in made, made


def test_a_task_naming_a_real_route_still_passes():
    """反向: 恒拒的实现同样能通过上面那条.

    MUTATION: 让 check_v8 无条件返回失败 -> 红.
    """
    dao, _made = _tick_routes(
        [{"task_id": "t-ok", "priority": 50, "submit_seq": 1,
          "state": "pending", "route_geo_id": "r-perimeter"}],
        route_ids=["r-perimeter"])
    # 同一拍里校验通过就会被派发, 所以终态是 running -- 本条要的是[没被拒].
    assert dao.rows["t-ok"]["state"] != "failed", dao.rows["t-ok"]


def test_a_task_without_a_route_is_not_rejected():
    """纯航点 goto 不引用路径, V-8 不该管它.

    MUTATION: 把"route_geo_id 为空则跳过"去掉 -> 红.
    """
    dao, _made = _tick_routes(
        [{"task_id": "t-nr", "priority": 50, "submit_seq": 1,
          "state": "pending", "route_geo_id": None}],
        route_ids=[])
    assert dao.rows["t-nr"]["state"] != "failed", dao.rows["t-nr"]


def test_without_an_injected_lookup_v8_is_skipped_not_failed():
    """没接线时跳过而不是拒: 否则任何没传 list_route_ids 的调用方(既有测试 .
    别的入口)会把全部待校验任务判成失败.

    MUTATION: 没有查询时返回失败 -> 红.
    """
    dao, _made = _tick([
        {"task_id": "t-skip", "priority": 50, "submit_seq": 1,
         "state": "pending", "route_geo_id": "r-whatever"}])
    assert dao.rows["t-skip"]["state"] != "failed", dao.rows["t-skip"]


def test_a_failing_route_snapshot_does_not_reject_everything():
    """geo.db 短暂不可用时, 批量误拒合法任务比放过一条坏路径严重得多.

    MUTATION: 去掉 except 分支(让异常冒上去) -> 红(整拍抛).
    """
    from xbrain.p3_task.schedule.driver import scheduler_tick

    async def _boom():
        raise RuntimeError("geo.db busy")

    dao = _FakeDao([{"task_id": "t-e", "priority": 50, "submit_seq": 1,
                     "state": "pending", "route_geo_id": "r-x"}])
    asyncio.run(scheduler_tick(_FakeConn(), dao, now_mono_ms=1000,
                               on_transition=_noop,
                               started_at="2026-09-02T08:00:00Z",
                               boot_id="b1", list_route_ids=_boom))
    assert dao.rows["t-e"]["state"] != "failed", dao.rows["t-e"]


def test_the_wiring_supplies_the_route_lookup():
    """接线断了的话上面几条全部退化成"跳过" -- 而跳过是绿的.

    MUTATION: p3 wiring 不传 list_route_ids -> 红.
    """
    import inspect

    from xbrain.p3_task.runtime import main_wiring

    src = inspect.getsource(main_wiring._amain)
    assert "list_route_ids=_list_route_ids" in src, "调度器没拿到路径全集查询"
    assert "tombstone=0" in src, "墓碑路径被当成存在的"
