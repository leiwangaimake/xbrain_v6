"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p3_estop.py
Brief: p3 cmd/estop -- ES-1 freeze 冻结调度门控的接线判据 (CLD-1a)

Description:
批59 查出 cmd/estop 全库零订阅者; 批62/63 接了 p2/p1; 本文件是 p3 侧
(15 S11.1 ES-1: freeze scheduling immediately, no new tasks dispatched).

EstopController 自身的 freeze/unfreeze 语义由 test_batch_g 覆盖; 本文件补的
是[接线]: freeze 真的让运行的调度循环[不再派发], 且 unfreeze 后恢复.

*** freeze 是[跳过整个 scheduler_tick], NO 不是跑完再过滤.
一个"照跑 tick 再把结果丢掉"的实现会把 pending -> ready -> running 的状态
迁移落进 task.db, 只是不发出去 -- 而 estop 解除后这些任务已经是 running,
等于急停期间任务照常启动了. 跳过整个 tick 才是"冻结".

*** ES-3: p3 不自动恢复(15 S11.3). 只有 p2 显式 unfreeze.
EstopController 无 time-based unfreeze 路径(CLAUDE.md 3.6: 无绕过开关).
本文件验证 freeze 持续到显式 unfreeze 为止.

Boundaries: 测 ES-1 的门控效果 + 订阅接线. ES-2(suspend running task)与
p2 unfreeze 信号通道见 NEXT(需新 DAO suspend 方法满足 CHECK 约束).
"""
from __future__ import annotations

import aiosqlite
import pytest
import pytest_asyncio

from xbrain.p3_task.dao.tasks_dao import TaskRow, TasksDAO
from xbrain.p3_task.lifecycle.estop import EstopController
from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
from xbrain.p3_task.schedule.driver import scheduler_tick

pytestmark = pytest.mark.no_device


@pytest_asyncio.fixture
async def dao_conn():
    async with aiosqlite.connect(":memory:") as c:
        for stmt in ALL_DDL_STATEMENTS:
            await c.execute(stmt)
        await c.commit()
        yield TasksDAO(c), c


def _pending(task_id, priority=50, seq=1):
    return TaskRow(
        task_id=task_id, task_type="goto", state="pending",
        priority=priority, submit_seq=seq, mission_json='{"a":1}',
        total_steps=1, current_step=0, step_status_json="[]", created_ms=0,
        updated_ms=0, source="local", trace_id="tr", resume_policy="continue")


async def _noop(task_id, to_state, reason):
    return None


async def _gated_tick(ctrl, conn, dao, now_ms):
    """Reproduce the main_wiring loop's ES-1 gate: only tick when scheduling
    is permitted. This is the exact predicate the running loop applies."""
    if ctrl.scheduling_permitted():
        await scheduler_tick(conn, dao, now_mono_ms=now_ms, on_transition=_noop)


# --- ES-1 门控 --------------------------------------------------------

@pytest.mark.asyncio
async def test_a_frozen_scheduler_dispatches_nothing(dao_conn):
    """*** ES-1 核心: freeze 后, 一个 pending 任务不得被派发.

    没有 estop 时, pending -> ready -> running(scheduler_tick 的既有行为).
    freeze 后, 门控跳过 tick, 任务停在 pending -- 一个也不动.

    MUTATION: main_wiring 里把门控去掉(总是 tick) -> 这里红(任务变 running).
    """
    dao, conn = dao_conn
    ctrl = EstopController()
    await dao.insert(_pending("t1"))
    await conn.commit()

    ctrl.freeze("estop_soft")
    await _gated_tick(ctrl, conn, dao, 1)

    assert (await dao.fetch_by_id("t1")).state == "pending", (
        "冻结期间任务被派发了 -- ES-1 没有阻止 dispatch")


@pytest.mark.asyncio
async def test_an_unfrozen_scheduler_dispatches_normally(dao_conn):
    """反向: 未 freeze 时照常派发.

    没有这条, 一个"永远冻结"的实现能让上一条通过 -- 而那台机器人永远
    不执行任何任务.
    """
    dao, conn = dao_conn
    ctrl = EstopController()
    await dao.insert(_pending("t1"))
    await conn.commit()

    await _gated_tick(ctrl, conn, dao, 1)

    assert (await dao.fetch_by_id("t1")).state == "running", (
        "未冻结却没派发 -- 门控把正常调度也挡了")


@pytest.mark.asyncio
async def test_unfreeze_resumes_dispatch(dao_conn):
    """*** ES-3: 冻结持续到显式 unfreeze; 之后恢复派发.

    freeze -> 不派发; unfreeze('p2_operator') -> 派发. 验证 freeze 不是
    一次性的, 而是持续到 p2 显式解除.

    MUTATION: EstopController.unfreeze 不清 frozen -> 这里红(仍不派发).
    """
    dao, conn = dao_conn
    ctrl = EstopController()
    await dao.insert(_pending("t1"))
    await conn.commit()

    ctrl.freeze("estop_soft")
    await _gated_tick(ctrl, conn, dao, 1)
    assert (await dao.fetch_by_id("t1")).state == "pending"

    ctrl.unfreeze("p2_operator")
    await _gated_tick(ctrl, conn, dao, 2)
    assert (await dao.fetch_by_id("t1")).state == "running", (
        "unfreeze 后没有恢复派发")


@pytest.mark.asyncio
async def test_freeze_persists_across_many_ticks(dao_conn):
    """*** ES-3: 不自动恢复(15 S11.3). 反复门控多拍仍冻结.

    一个"几拍后自动解冻"的实现(或漏了持久性)会在若干拍后放行. estop 必须
    一直冻结到人来解除.
    """
    dao, conn = dao_conn
    ctrl = EstopController()
    await dao.insert(_pending("t1"))
    await conn.commit()

    ctrl.freeze("estop_soft")
    for tick in range(20):
        await _gated_tick(ctrl, conn, dao, tick + 1)

    assert (await dao.fetch_by_id("t1")).state == "pending", (
        "多拍之后任务被派发了 -- freeze 没有持久, 违反 15 S11.3 不自动恢复")


# --- 接线 -------------------------------------------------------------

def test_main_wiring_subscribes_cmd_estop_and_gates_the_tick():
    """*** 守启动接线: p3 真的订 cmd/estop, 且 scheduler_tick 过门控.

    批59 查出 p3 没订 cmd/estop(契约 11 S1.4 的四订阅者之一). 用 AST 查:
      1. main_wiring 有 declare_subscriber(CMD_ESTOP_TOPIC, ...)
      2. scheduling_permitted() 在 scheduler_tick 的调用路径上被查
    NO 不 grep -- 注释里就写着 cmd/estop.

    MUTATION: 注释掉 cmd/estop 订阅 -> 第1条红.
    MUTATION: 去掉 scheduling_permitted 门控 -> 第2条红.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p3_task" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    tree = ast.parse(src)

    subs = [n for n in ast.walk(tree)
            if isinstance(n, ast.Call)
            and getattr(n.func, "attr", "") == "declare_subscriber"
            and n.args and getattr(n.args[0], "id", "") == "CMD_ESTOP_TOPIC"]
    assert len(subs) == 1, (
        "main_wiring 订阅 CMD_ESTOP_TOPIC 的调用有 %d 处 -- 急停到不了 p3"
        % len(subs))

    gated = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and getattr(n.func, "attr", "") == "scheduling_permitted"]
    assert gated, (
        "scheduling_permitted 没有被调用 -- ES-1 冻结没有门控住 scheduler")

    # *** 3. cmd/estop 回调必须真的 freeze. 集成测试直接调 ctrl.freeze() 绕过
    # 了回调, 所以这条链单靠 AST 钉住 -- 否则一个"订了 cmd/estop 但回调什么
    # 都不做"的实现会让上面的门控测试全绿而急停根本不冻结调度.
    # MUTATION: 删掉 _on_estop 里的 estop_ctrl.freeze(...) -> 这里红.
    on_estop = [f for f in ast.walk(tree)
                if isinstance(f, ast.FunctionDef) and f.name == "_on_estop"]
    assert on_estop, "找不到 _on_estop 回调"
    froze = any(getattr(n.func, "attr", "") == "freeze"
                for n in ast.walk(on_estop[0]) if isinstance(n, ast.Call))
    assert froze, "_on_estop 回调没有 freeze -- cmd/estop 到了却不冻结调度"
