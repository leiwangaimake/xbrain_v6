"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p1_estop.py
Brief: p1 cmd/estop -- ctrl_loop 本拍零速 + latch/re-arm + 订阅接线 (P1-21)

Description:
批59 查出 cmd/estop 全库零订阅者(CLD-1). 批62 接了 p2; 本文件是 p1 侧
(P1-21: 软急停本拍零速 + 落 stop_reason, NO 不转发).

*** 本文件分两层:
  ctrl_loop  20 Hz 每拍的[本拍零速 + stop_reason 归因] -- 纯函数, P1-21 本体
  latch      运行进程(main_wiring)里落锁 + re-arm 状态机 -- 订阅的真实产出
两层各有变异体. ctrl_loop 的 estop 分支是 P1-21 的核心, 即便 20 Hz 循环
当前是 skeleton(GATED-HW), 这段逻辑本身是真实且要正确的.

*** estop wins over everything(11 S9.12.2).
最要命的一条: 一个 ACTIVE 状态且 computed_vx 非零的拍, estop 必须[仍然]
零速. estop 判定排在 ACTIVE 分支[之前], 否则一个漏改的路径会让运动中的
机器人忽略急停.
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device


# --- ctrl_loop 本拍零速 + 归因 ----------------------------------------

def _loop():
    from xbrain.p1_motion.ctrl_loop import CtrlLoop, CtrlState

    published = []
    loop = CtrlLoop(lambda vx, wz: published.append((vx, wz)))
    return loop, published, CtrlState


def test_estop_zeroes_even_an_active_tick():
    """*** estop wins over everything. ACTIVE + 非零 computed -> 仍零速.

    这是 P1-21 的本拍零速. 一个正在 1.5 m/s 前进的 ACTIVE 拍, 收到 estop
    必须这一拍就零 -- 而不是等状态机慢慢切到 SAFE_STOP.

    MUTATION: 把 run_one_tick 的 estop 判定挪到 ACTIVE 分支之后 -> 这里红.
    """
    loop, published, CtrlState = _loop()
    loop.transition(CtrlState.ACTIVE)

    tick = loop.run_one_tick(computed_vx=1.5, computed_wz=0.3, estop=True)

    assert (tick.vx, tick.wz) == (0.0, 0.0), "ACTIVE 拍 estop 没有零速"
    assert published[-1] == (0.0, 0.0), "零速没有真的发出去"
    assert tick.stop_reason == "soft_estop"


def test_a_driving_tick_without_estop_keeps_its_velocity():
    """反向: 没有 estop 的 ACTIVE 拍照常用 computed 值.

    没有这条, 一个"estop 恒 True"的实现能让上一条通过 -- 而那台机器人
    永远不动.
    """
    loop, _pub, CtrlState = _loop()
    loop.transition(CtrlState.ACTIVE)

    tick = loop.run_one_tick(computed_vx=1.5, computed_wz=0.3, estop=False)

    assert (tick.vx, tick.wz) == (1.5, 0.3)
    assert tick.stop_reason == "none"


def test_stop_reason_distinguishes_estop_from_no_source():
    """*** 零速的两种原因归因不同(P1-21: 落 stop_reason).

    estop 零速 -> soft_estop; 尚未驾驶(INIT 等)零速 -> no_source. 都零速,
    但下游要区分"急停停的"和"还没开始". 归成同一个值等于丢了归因.

    MUTATION: else 分支的 no_source 写成 soft_estop -> 这里红.
    """
    loop, _pub, CtrlState = _loop()
    # INIT 状态, 未驾驶.
    tick = loop.run_one_tick(estop=False)
    assert (tick.vx, tick.wz) == (0.0, 0.0)
    assert tick.stop_reason == "no_source", "未驾驶被归成了 estop"


def test_stop_reason_values_are_in_the_closed_set():
    """归因必须在 stop_reason 闭集内(common/enums, CLAUDE.md 3.5).

    发一个闭集外的 stop_reason 就是让下游收到它字典里没有的值.
    """
    from xbrain.common.enums import STOP_REASON

    loop, _pub, CtrlState = _loop()
    loop.transition(CtrlState.ACTIVE)
    for estop in (True, False):
        tick = loop.run_one_tick(computed_vx=1.0, estop=estop)
        assert tick.stop_reason in STOP_REASON, (
            "stop_reason %r 不在闭集 %s" % (tick.stop_reason,
                                           sorted(STOP_REASON)))


def test_estop_defaults_off_backward_compatible():
    """estop 默认 False: 既有调用点(不传 estop)行为不变.

    run_one_tick 的既有调用者(test_batch_c 等)不传 estop, 必须照旧.
    """
    loop, _pub, CtrlState = _loop()
    loop.transition(CtrlState.ACTIVE)
    tick = loop.run_one_tick(computed_vx=2.0, computed_wz=0.0)   # 不传 estop
    assert tick.vx == 2.0 and tick.stop_reason == "none"


# --- latch: 落锁 + 归因 + re-arm --------------------------------------

def _latch():
    from xbrain.p1_motion.runtime.estop_latch import P1EstopLatch

    return P1EstopLatch()


def test_an_estop_latches_and_reports_soft_estop():
    """cmd/estop -> 落锁, is_active True, stop_reason soft_estop.

    ctrl_loop 每拍读 is_active(); 这条保证锁真的被落上.
    """
    import json

    latch = _latch()
    assert latch.is_active() is False
    assert latch.stop_reason() == "none"

    latch.on_estop(json.dumps({"action": "stop", "cmd_id": "e-1"}).encode(
        "utf-8"))

    assert latch.is_active() is True
    assert latch.stop_reason() == "soft_estop"


def test_a_new_intent_rearms_and_forwards():
    """*** 新运动指令 re-arm 并放行(14 S3.7 / U35).

    喊急停 -> 停 -> 喊前进两米 -> 立刻走. gate_intent 清锁并放行那个新
    intent. NO 不拒绝新运动.

    MUTATION: gate_intent 在 latched 时 return False -> 这里红.
    """
    import json

    latch = _latch()
    latch.on_estop(json.dumps({"cmd_id": "e-1"}).encode("utf-8"))
    assert latch.is_active() is True

    forwarded = latch.gate_intent()

    assert forwarded is True, "急停后的新运动指令被拒了 -- U35 现场行为被堵死"
    assert latch.is_active() is False, "新指令没有 re-arm"
    assert latch.rearms == 1


def test_gate_on_an_unlatched_latch_just_forwards():
    """未 latched 时 gate_intent 直接放行, 不算 re-arm.

    没有这条, 一个"每次 gate 都 rearm"的实现会在正常运行时不停加 rearm 计数.
    """
    latch = _latch()
    assert latch.gate_intent() is True
    assert latch.rearms == 0


def test_repeated_estop_same_cmd_id_is_idempotent():
    """同 cmd_id 10 Hz 重发(1 s 窗口, 11 S2.2.3)幂等, 不刷计数."""
    import json

    latch = _latch()
    raw = json.dumps({"cmd_id": "e-1"}).encode("utf-8")
    latch.on_estop(raw)
    latch.on_estop(raw)
    latch.on_estop(raw)

    assert latch.is_active() is True
    assert latch.latches == 1, "同一次急停的重发被当成多次"


def test_a_different_cmd_id_relatches():
    """不同 cmd_id 是新的一次急停, 重新落锁计数."""
    import json

    latch = _latch()
    latch.on_estop(json.dumps({"cmd_id": "e-1"}).encode("utf-8"))
    latch.gate_intent()                          # re-arm
    latch.on_estop(json.dumps({"cmd_id": "e-2"}).encode("utf-8"))

    assert latch.is_active() is True
    assert latch.latches == 2


def test_parse_is_fail_safe():
    """*** 坏帧照样落锁(11 S3.0.1). 急停丢不起.

    MUTATION: parse_estop_cmd_id 在 decode 失败时 raise -> 这里红.
    """
    from xbrain.p1_motion.runtime.estop_latch import parse_estop_cmd_id

    for bad in (b'{not json', b'', b'\xff', b'[1,2]'):
        cid = parse_estop_cmd_id(bad)
        assert cid, "坏帧 %r 没有回退出一个 cmd_id" % bad


def test_cmd_id_fallback_order():
    """cmd_id -> msg_id -> origin. 云端信封用 msg_id."""
    import json

    from xbrain.p1_motion.runtime.estop_latch import parse_estop_cmd_id

    assert parse_estop_cmd_id(json.dumps({"cmd_id": "c", "msg_id": "m"}).encode(
        "utf-8")) == "c"
    assert parse_estop_cmd_id(json.dumps({"msg_id": "m"}).encode(
        "utf-8")) == "m"
    assert parse_estop_cmd_id(json.dumps({"origin": "cloud"}).encode(
        "utf-8")) == "estop-cloud"


# --- 启动接线 ---------------------------------------------------------

def test_main_wiring_subscribes_cmd_estop_and_gates_intents():
    """*** 守启动接线: p1 真的订 cmd/estop, 且 _on_intent 过 gate.

    P1-21 的白名单登记了 cmd/estop, 而代码此前没订(批59). 这条用 AST 查:
      1. main_wiring 有 declare_subscriber(CMD_ESTOP_TOPIC, ...)
      2. gate_intent 在 _on_intent 里被调(急停后新 intent 走 re-arm)
    NO 不 grep -- 注释里就写着 cmd/estop.

    MUTATION: 注释掉 declare_subscriber(CMD_ESTOP_TOPIC,...) -> 第1条红.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p1_motion" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    tree = ast.parse(src)

    subs = [n for n in ast.walk(tree)
            if isinstance(n, ast.Call)
            and getattr(n.func, "attr", "") == "declare_subscriber"
            and n.args and getattr(n.args[0], "id", "") == "CMD_ESTOP_TOPIC"]
    assert len(subs) == 1, (
        "main_wiring 订阅 CMD_ESTOP_TOPIC 的调用有 %d 处 -- 急停到不了 p1"
        % len(subs))

    on_intent = [f for f in ast.walk(tree)
                 if isinstance(f, ast.FunctionDef) and f.name == "_on_intent"]
    assert on_intent, "找不到 _on_intent"
    gated = any(getattr(n.func, "attr", "") == "gate_intent"
                for n in ast.walk(on_intent[0]) if isinstance(n, ast.Call))
    assert gated, "_on_intent 没有过 estop gate -- 急停后 re-arm 语义缺失"
