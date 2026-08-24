"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_p2_estop_wiring.py
Brief: cmd/estop -> 域1缴械 + 广播 + 爆闪 + re-arm 的接线判据 (CLD-1)

Description:
three_stops 的逻辑由 test_three_stops 钉住; 本文件钉的是[接线]那段 --
2026-08-24 查出 cmd/estop 全库零订阅者(批59 CLD-1), 三个软件来源都发进空气.
本文件保证: 一条 cmd/estop 真的落到域1 缴械, 且 suspended 真的被广播出去
(p1 靠它零速; 只缴械不广播 = 状态锁在 p2 进程内, 软急停对运动无效).

*** 解析必须 fail-safe(11 S3.0.1).
cmd/estop 是唯一豁免 v 校验的 key. 一条解析失败的急停不能被丢弃 -- 丢了
就是没停. parse_estop_frame 永不抛, 坏帧照样当一次停.

*** 广播是接线的靶心, 单测构建器看不见它.
批14-16 的教训: 只测"构建了什么"的用例, 看不见"发到了哪条总线". 这里用
一个记录每次 publish 的假发布器, 断言 suspended 真的被 put 出去.

Boundaries: 不起真 Zenoh. arbiter 与 strobe 用真对象(逻辑真实), emit 与
publish 注入.
"""
from __future__ import annotations

import json

import pytest

pytestmark = pytest.mark.no_device


def _coord():
    from xbrain.common.arbiter.core import Arbiter
    from xbrain.p2_core.runtime.estop_wiring import EstopCoordinator
    from xbrain.p2_core.three_stops import ForceStrobeState

    arb = Arbiter("motion", wait_atomic_timeout_ms=3000)
    strobe = ForceStrobeState()
    events = []
    published = []       # [(suspended, gen)]

    def _emit(ev):
        events.append(ev)

    def _publish(now_ms):
        from xbrain.p2_core.runtime.estop_wiring import suspended_frame
        published.append(suspended_frame(arb, now_ms))

    coord = EstopCoordinator(arb, strobe, _emit, _publish)
    return coord, arb, strobe, events, published


# --- fail-safe 解析 ---------------------------------------------------

def test_parse_reads_cmd_id_and_action():
    from xbrain.p2_core.runtime.estop_wiring import parse_estop_frame

    f = parse_estop_frame(json.dumps(
        {"type": "estop", "action": "stop", "cmd_id": "e-7"}).encode("utf-8"))
    assert f.cmd_id == "e-7" and f.action == "stop"


def test_parse_falls_back_to_msg_id_then_origin():
    """云端信封用 msg_id, 不带 cmd_id. 回退顺序 cmd_id -> msg_id -> origin."""
    from xbrain.p2_core.runtime.estop_wiring import parse_estop_frame

    f = parse_estop_frame(json.dumps(
        {"type": "estop", "action": "stop", "msg_id": "m-3"}).encode("utf-8"))
    assert f.cmd_id == "m-3"
    g = parse_estop_frame(json.dumps(
        {"type": "estop", "action": "stop", "origin": "cloud"}).encode("utf-8"))
    assert g.cmd_id == "estop-cloud"


def test_malformed_bytes_still_parse_to_a_stop():
    """*** fail-safe: 坏帧不抛, 照样是一次停(11 S3.0.1).

    丢弃一条解析失败的急停 = 没停. 这条守的就是"不丢".

    MUTATION: 让 parse_estop_frame 在 decode 失败时 raise -> 这里红.
    """
    from xbrain.p2_core.runtime.estop_wiring import parse_estop_frame

    for bad in (b'{not json', b'', b'\xff\xff', b'[1,2,3]'):
        f = parse_estop_frame(bad)
        assert f.action == "stop", "坏帧 %r 没有回退成 stop" % bad
        assert f.cmd_id                       # 有一个稳定回退 id


# --- 缴械 + 广播 ------------------------------------------------------

def test_an_estop_disarms_domain1_and_broadcasts_suspended():
    """*** 接线靶心: 缴械[且]广播.

    只缴械不广播的话, suspended 锁在 p2 进程内, p1 读不到, 软急停对运动
    毫无效果 -- 而 p2 侧看起来一切正常.

    MUTATION: EstopCoordinator.on_estop 里删掉 self._publish(...) -> 这里红.
    """
    coord, arb, strobe, events, published = _coord()

    coord.on_estop(json.dumps(
        {"type": "estop", "action": "stop", "cmd_id": "e-1"}).encode("utf-8"),
        1000)

    assert arb.suspended() == "soft_estop"
    assert strobe.active is True
    assert len(published) == 1, "suspended 没有被广播 -- p1 读不到"
    assert published[0]["suspended"] == "soft_estop"
    assert published[0]["domain"] == "motion"


def test_the_broadcast_carries_null_when_not_suspended():
    """suspended_frame 在未缴械时是 JSON null, 不是省略字段.

    p1 靠这个字段判零速; 省略它 p1 会当成"未知"而不是"正常", 可能一直 hold.
    """
    from xbrain.common.arbiter.core import Arbiter
    from xbrain.p2_core.runtime.estop_wiring import suspended_frame

    arb = Arbiter("motion", wait_atomic_timeout_ms=3000)
    body = suspended_frame(arb, 1000)
    assert "suspended" in body and body["suspended"] is None


def test_a_repeated_estop_same_cmd_id_stays_suspended():
    """同 cmd_id 重发(10 Hz 重发, 1 s 窗口)仍是缴械态, 不翻转."""
    coord, arb, _strobe, _ev, published = _coord()

    raw = json.dumps({"action": "stop", "cmd_id": "e-1"}).encode("utf-8")
    coord.on_estop(raw, 1000)
    coord.on_estop(raw, 1100)

    assert arb.suspended() == "soft_estop"
    assert coord.stops == 2                    # 都处理了(广播每次都发, p1 幂等)


# --- re-arm -----------------------------------------------------------

def test_a_new_motion_command_rearms_only_soft_estop():
    """*** 新运动指令解除 soft_estop, 广播 suspended=null.

    MUTATION: maybe_rearm 里去掉 suspended=="soft_estop" 判别 -> 见下条.
    """
    coord, arb, strobe, _ev, published = _coord()

    coord.on_estop(json.dumps({"action": "stop", "cmd_id": "e-1"}).encode(
        "utf-8"), 1000)
    published.clear()

    did = coord.maybe_rearm(2000)

    assert did is True
    assert arb.suspended() is None
    assert strobe.active is False
    assert published[-1]["suspended"] is None, "解除后没广播 null, p1 不恢复"


def test_rearm_does_not_touch_a_hardware_lock():
    """*** 只解 soft_estop, NO 不解 hes.

    14 S3.7 表: soft_estop 新指令即可解除; hes 要 HES 归零 + 人工 enable.
    一条新语音指令不该把硬件锁解开 -- 那是"人必须到现场"的锁.

    MUTATION: maybe_rearm 里把 != "soft_estop" 改成恒 True 进入 -> 这里红.
    """
    from xbrain.p2_core.three_stops import (StopEvent, StopReason, apply_stop)

    coord, arb, strobe, _ev, _pub = _coord()
    # 直接用 hes 缴械(模拟硬件锁到达, 走的是别的通道不是 cmd/estop).
    apply_stop(StopEvent(StopReason.HES, "h-1", 1000), arb, strobe,
               lambda e: None)
    assert arb.suspended() == "hes"

    did = coord.maybe_rearm(2000)

    assert did is False, "re-arm 动了 hes 硬件锁"
    assert arb.suspended() == "hes", "硬件锁被一条新运动指令解开了"


def test_rearm_on_an_armed_domain_is_a_noop():
    """未缴械时 maybe_rearm 什么都不做, 不广播."""
    coord, arb, _strobe, _ev, published = _coord()

    assert coord.maybe_rearm(1000) is False
    assert not published


# --- 启动接线 ---------------------------------------------------------

def test_main_wiring_subscribes_cmd_estop_and_drains_it_first():
    """*** 守启动接线(与云端那三条同型的第三层断言).

    协调器写好了而 main_wiring 不订 cmd/estop, 上面每条都全绿而真机上
    急停到不了 p2. 用 AST 查:
      1. main_wiring 里有 declare_subscriber(CMD_ESTOP_TOPIC, ...)
      2. estop_queue 在 motion_queue [之前] drain(急停不排在运动帧后面)

    NO 不 grep 字符串 -- 注释里就写着 cmd/estop.

    MUTATION: 注释掉 declare_subscriber(CMD_ESTOP_TOPIC,...) -> 第1条红.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p2_core" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    tree = ast.parse(src)

    subs = [n for n in ast.walk(tree)
            if isinstance(n, ast.Call)
            and getattr(n.func, "attr", "") == "declare_subscriber"
            and n.args and getattr(n.args[0], "id", "") == "CMD_ESTOP_TOPIC"]
    assert len(subs) == 1, (
        "main_wiring 订阅 CMD_ESTOP_TOPIC 的调用有 %d 处 -- 急停到不了 p2"
        % len(subs))

    # estop_queue.get_nowait 必须先于 motion_queue.get_nowait 出现.
    src_no_ws = src
    ei = src_no_ws.find("estop_queue.get_nowait")
    mi = src_no_ws.find("motion_queue.get_nowait")
    assert ei != -1 and mi != -1 and ei < mi, (
        "cmd/estop 没有先于 cmd/motion/intent 处理 -- 急停排在运动帧后面")


def test_main_wiring_publishes_state_arb_motion():
    """缴械态必须真的广播到 state/arb/motion, 否则 p1 读不到."""
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain"
           / "p2_core" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    pubs = [n for n in ast.walk(ast.parse(src))
            if isinstance(n, ast.Call)
            and getattr(n.func, "attr", "") == "declare_publisher"
            and n.args
            and getattr(n.args[0], "id", "") == "STATE_ARB_MOTION_TOPIC"]
    assert len(pubs) == 1, "state/arb/motion 没有发布者 -- 缴械态传不出 p2"
