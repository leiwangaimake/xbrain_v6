"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p3_estop_resume.py
Brief: ES-2 挂起 running task + ES-3 人显式 cmd/task 解冻 (CLD-1c)

Description:
批64 做了 ES-1(freeze 冻结派发); 本文件是 CLD-1c 的两块:
  ES-2  把[当前 running]任务挂起(kind=passive, reason=estop_soft)
  ES-3  人显式 cmd/task{submit|resume} 解冻(15 S11.1 F5), auto/charge 不解冻

*** ES-3 的解冻源必须是[人]的通道, 这是最容易放松的一处.
15 S11.1 F5 逐字: 解冻 = 一条人显式发起的任务指令. 一个"任何 cmd/task 都
解冻"的实现会让急停期间自动注入的 return_home(source=charge)把冻结解开 --
而 return_home 正是低电时系统自己塞的, 它一来就解冻等于急停白按了.

*** ES-2 只挂当前 running 那一条, 且挂起归因不能与 pause 混.
pause 的 reason 是 operator_pause(人按暂停), estop 是 estop_soft. 审计要能
区分, 所以两条写库路径分开(DAO.suspend_task vs task_apply._write_state).
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.lifecycle.estop import (EstopController, HUMAN_SOURCES,
                                            is_human_resume_command)
from xbrain.p3_task.lifecycle.estop_suspend import suspend_running_for_estop
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS

pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def dao_conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield TasksDAO(c), c


def _task(task_id, state, source="cloud", priority=50, seq=1):
    return TaskRow(
        task_id=task_id, task_type="goto", state=state,
        priority=priority, submit_seq=seq, mission_json='{"a":1}',
        total_steps=1, current_step=0, step_status_json="[]", created_ms=0,
        updated_ms=0, source=source, trace_id="tr", resume_policy="continue")


# --- ES-2: suspend running task ---------------------------------------

@pytest.mark.asyncio
async def test_es2_suspends_the_running_task(dao_conn):
    """*** ES-2 核心: running 任务被挂起, kind=passive, reason=estop_soft.

    MUTATION: suspend_running_for_estop 不调 dao.suspend_task -> 这里红.
    """
    dao, conn = dao_conn
    await dao.insert(_task("t1", "running"))
    await conn.commit()

    result = await suspend_running_for_estop(dao, conn, now_mono_ms=100)

    assert result == ("t1", "suspended", "estop_soft")
    row = await dao.fetch_by_id("t1")
    assert row.state == "suspended"
    assert row.suspend_kind == "passive"
    assert row.suspend_reason == "estop_soft"


@pytest.mark.asyncio
async def test_es2_commits_so_the_next_submit_still_works(dao_conn):
    """*** 挂起之后, 连接必须能再开一条事务.

    2026-09-01 联调预演实测出来的: ES-2 原先调完 dao.suspend_task 就 return,
    aiosqlite 那条隐式事务一直开着, 之后任何 BEGIN IMMEDIATE 都报
    "cannot start a transaction within a transaction". 后果是[急停之后每一条
    cmd/task submit 都失败], 直到 p3 重启 -- 而急停是甲方验收的头号项.

    *** 为什么既有 4 条 ES-2 判据全漏.
    它们都是挂起后在[同一连接]上 fetch_by_id 读回那一行做断言. 而未提交的写
    在同一连接上本来就读得到 -- 判据验的是"写可见", 不是"已提交". 一条永远
    绿的断言(CLAUDE.md 3.2 形态1): 一个不 commit 的实现同样通过它们.

    本条改验[副作用]: 挂起之后连接还能不能用. 这是调用方真正依赖的性质,
    也是唯一能把"没提交"和"提交了"分开的观察点 -- 因为 sqlite 不提供
    "当前是否在事务里"的公开查询.

    MUTATION: 去掉 suspend_running_for_estop 里的 commit -> 这里红.
    """
    dao, conn = dao_conn
    await dao.insert(_task("t1", "running"))
    await conn.commit()

    await suspend_running_for_estop(dao, conn, now_mono_ms=100)

    # 这一句就是 task_apply.submit 的第一步. 事务没闭合时它抛 OperationalError.
    await conn.execute("BEGIN IMMEDIATE")
    await conn.rollback()


@pytest.mark.asyncio
async def test_es2_write_survives_a_rollback_by_another_writer(dao_conn):
    """*** 挂起必须真的落盘, 不只是在本连接可见.

    上一条验的是"连接还能用", 本条验的是"数据还在". 两者都需要: 一个把
    suspend_task 换成 no-op 但保留 commit 的实现能过上一条, 一个不 commit
    但没有后续写的实现能过原有 4 条.

    做法是在挂起之后显式 rollback 一次. 已提交的写不受 rollback 影响; 留在
    开着的事务里的写会被抹掉.

    MUTATION: 去掉 commit -> rollback 抹掉挂起, state 退回 running -> 红.
    """
    dao, conn = dao_conn
    await dao.insert(_task("t1", "running"))
    await conn.commit()

    await suspend_running_for_estop(dao, conn, now_mono_ms=100)
    await conn.rollback()          # 已提交的写不该受影响

    row = await dao.fetch_by_id("t1")
    assert row.state == "suspended", (
        "rollback 之后挂起没了 -- 说明 ES-2 的写没有提交, 只是留在一条"
        "开着的事务里")
    assert row.suspend_reason == "estop_soft"


@pytest.mark.asyncio
async def test_es2_is_a_noop_with_nothing_running(dao_conn):
    """没有 running 任务时返回 None, 不碰 ready/queued.

    freeze 期间调度冻住, 挂掉那条后就没有 running 了; 后续每拍空操作.
    """
    dao, conn = dao_conn
    await dao.insert(_task("t1", "ready"))       # ready, 不是 running
    await conn.commit()

    assert await suspend_running_for_estop(dao, conn, now_mono_ms=100) is None
    assert (await dao.fetch_by_id("t1")).state == "ready", "ready 任务被动了"


@pytest.mark.asyncio
async def test_es2_leaves_a_valid_suspended_row(dao_conn):
    """*** 挂起后的行必须过 schema CHECK(suspended <=> kind/reason 非空).

    一个只写 state 不写 kind/reason 的实现会撞 DB CHECK -- 但那要到 sqlite
    才炸. 这条直接验证写出来的行是合法的(fetch 不报错 + 字段齐).

    MUTATION: DAO.suspend_task 里去掉 suspend_kind 的写 -> sqlite CHECK 报错.
    """
    dao, conn = dao_conn
    await dao.insert(_task("t1", "running"))
    await conn.commit()

    await suspend_running_for_estop(dao, conn, now_mono_ms=100)
    await conn.commit()
    # 重新查, 若 CHECK 被违反上面的 commit 就已经抛了.
    row = await dao.fetch_by_id("t1")
    assert row.suspend_kind and row.suspend_reason


@pytest.mark.asyncio
async def test_es2_reason_differs_from_pause(dao_conn):
    """*** estop 挂起归因 estop_soft, 不是 pause 的 operator_pause.

    审计要能区分"急停停的"和"人按了暂停". 归成同一个 reason 等于丢了这个
    区别.
    """
    dao, conn = dao_conn
    await dao.insert(_task("t1", "running"))
    await conn.commit()

    await suspend_running_for_estop(dao, conn, now_mono_ms=100)

    assert (await dao.fetch_by_id("t1")).suspend_reason == "estop_soft"


# --- ES-3: human command unfreezes ------------------------------------

def test_es3_human_submit_resume_is_recognised():
    """人显式 submit/resume 是解冻信号(15 S11.1 F5)."""
    for action in ("submit", "resume"):
        for source in HUMAN_SOURCES:
            assert is_human_resume_command(
                {"action": action, "source": source}) is True, (action, source)


def test_es3_auto_and_charge_do_not_unfreeze():
    """*** 系统自动任务(auto/charge)不解冻.

    return_home(source=charge)在低电时自动注入; 它一来就解冻 = 急停白按.
    这是 ES-3 最要害的一条.

    MUTATION: is_human_resume_command 不查 source(只看 action) -> 这里红.
    """
    for source in ("auto", "charge"):
        assert is_human_resume_command(
            {"action": "submit", "source": source}) is False, source
        assert is_human_resume_command(
            {"action": "resume", "source": source}) is False, source


def test_es3_non_submit_resume_actions_do_not_unfreeze():
    """cancel/pause/clear_queue 不是解冻信号 -- 它们不该让机器人重新动起来."""
    for action in ("cancel", "pause", "clear_queue"):
        assert is_human_resume_command(
            {"action": action, "source": "cloud"}) is False, action


def test_es3_unfreeze_accepts_human_sources_rejects_system():
    """*** EstopController.unfreeze: 人来源放行, auto/charge/未知拒绝.

    MUTATION: UNFREEZE_SOURCES 加进 'charge' -> 下面 charge 那条红.
    """
    for source in ("cloud", "wecom", "local", "p2_operator"):
        c = EstopController()
        c.freeze("estop_soft")
        c.unfreeze(source)                        # 不抛
        assert c.frozen is False, source

    for bad in ("charge", "auto", "some_other", ""):
        c = EstopController()
        c.freeze("estop_soft")
        with pytest.raises(PermissionError):
            c.unfreeze(bad)
        assert c.frozen is True, "%s 不该解冻却解了" % bad


# --- 接线 -------------------------------------------------------------

def test_main_wiring_wires_es2_and_es3():
    """*** 守 ES-2/ES-3 接线.

    ES-2: 调度门控的 frozen 分支里调 suspend_running_for_estop.
    ES-3: cmd/task accepted 后, frozen + 人指令 -> unfreeze.
    AST 查这两处调用真的在 main_wiring 里.

    MUTATION: 删掉门控里的 suspend_running_for_estop 调用 -> ES-2 那条红.
    MUTATION: 删掉 unfreeze 调用 -> ES-3 那条红.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p3_task" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    tree = ast.parse(src)
    calls = {getattr(n.func, "attr", "") or getattr(n.func, "id", "")
             for n in ast.walk(tree) if isinstance(n, ast.Call)}

    assert "suspend_running_for_estop" in calls, (
        "ES-2 没接: main_wiring 不调 suspend_running_for_estop")
    assert "unfreeze" in calls, (
        "ES-3 没接: main_wiring 不调 estop_ctrl.unfreeze")
    assert "is_human_resume_command" in calls, (
        "ES-3 没判人来源: 任何 cmd/task 都会解冻 -- auto return_home 也能解")
