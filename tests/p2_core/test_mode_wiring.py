"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_mode_wiring.py
Brief: P2 cmd/mode 接收端 -- 换态 / 拒绝 / 幂等 / state/mode 发布 (11 S7.3)

Description:
守 P2 模式面的总线入口. 这个入口在 2026-08-21 之前不存在: 11 S2.2.3 把
p2_core 列为 cmd/mode 的订阅者, 而 main_wiring 从未订过, 于是 18 册 C 类
8 条指令全部落空且两侧都不报错.

每条断言都配了一个必然违反它的变异体(CLAUDE.md 3.3). 最值得点名的三个,
因为每一个都是看起来对的写法:

  * 把 dispatch() 的 accepted 当成"模式已切" -- dispatch 只验形状与闭集,
    真正换态在 ModeStateMachine 里, 会因仲裁域被占而失败;
  * 把 set_speed_profile 也放进换态表 -- 于是一句"慢一点"把机器人从喊话
    模式踢回 idle;
  * 未知 voice_mode 就近解释成最像的模式 -- S13.6 明禁, 而模式决定麦克风
    门控与喊话行为.

Boundaries: 不测 ModeStateMachine 的转换规则本身(那有自己的测试), 只测
"总线帧 -> 模式机 -> ack / state/mode" 这一跳.
"""
from __future__ import annotations

import json

import pytest

from xbrain.common.errors import E_BUSY, E_SCHEMA
from xbrain.p2_core.mode.state_machine import ModeState, ModeStateMachine
from xbrain.p2_core.runtime import mode_wiring
from xbrain.p2_core.runtime.mode_wiring import ModeFace

pytestmark = pytest.mark.no_device


def _frame(**kw) -> bytes:
    body = {"cmd_id": "c-1"}
    body.update(kw)
    return json.dumps(body).encode("utf-8")


class _Pub:
    """记录发布, 用来断言 state/mode 只在真换态时发. """

    def __init__(self) -> None:
        self.sent = []

    def __call__(self, key, data):
        self.sent.append((key, json.loads(data.decode("utf-8"))))


def _face(pub=None, arb=None, initial=ModeState.IDLE):
    return ModeFace(ModeStateMachine(initial), publish=pub, arb_snapshot=arb)


# -- 换态 ----------------------------------------------------------------

def test_set_voice_mode_broadcast_switches_and_publishes_state():
    """*** C03"开始喊话"的整条落点.

    变异体: 不调 ModeStateMachine 只回 ack -> face.state 仍是 idle, 且
    state/mode 不会发出, 本条即红.
    """
    pub = _Pub()
    face = _face(pub)
    ack = face.handle_frame(
        _frame(action="set_voice_mode", voice_mode="broadcast"),
        now_mono_ms=1000)
    assert ack["result"] == "accepted"
    assert ack["detail"]["applied"]["mode"] == "broadcast"
    assert ack["detail"]["applied"]["from_mode"] == "idle"
    assert face.state is ModeState.BROADCAST
    # P2 是 state/mode 的唯一发布者(11 S2.2.3), 换了态就要说出去 --
    # 否则 HMI 的模式显示与真实模式会长期不一致.
    assert pub.sent and pub.sent[0][0] == "state/mode"
    # *** 字段名是 voice_mode (11 S4.3 ModeState 的首字段), 不是 mode.
    # 变异体: 发 {"mode": ...} -- 那是 P5 reader 在没有发布者的年代与本模块
    # "对上"的一个不存在于契约的名字, 两侧都改对了才算数.
    assert pub.sent[0][1]["voice_mode"] == "broadcast"
    assert "mode" not in pub.sent[0][1]
    # S4.3 的其余字段(payload{} / ptz{})不发空壳: 少一个字段是"没有",
    # 发个假的会让 HMI 显示从没发生过的载荷状态.
    assert "payload" not in pub.sent[0][1]


def test_exit_broadcast_returns_to_idle():
    """C04, 也是 HMI 白名单 W3 要落到的那个动作. """
    face = _face(initial=ModeState.BROADCAST)
    ack = face.handle_frame(_frame(action="exit_broadcast"), now_mono_ms=1)
    assert ack["result"] == "accepted"
    assert face.state is ModeState.IDLE


def test_exit_alarm_returns_to_idle():
    face = _face(initial=ModeState.ALARM)
    assert face.handle_frame(
        _frame(action="exit_alarm"), now_mono_ms=1)["result"] == "accepted"
    assert face.state is ModeState.IDLE


# -- 不换态的两个合法动作 -------------------------------------------------

def test_set_speed_profile_is_accepted_without_changing_mode():
    """*** 速度档位动作留在 ModeCommand 里(S7.3.1 原 D-04 裁决), 但它不是模式.

    变异体: 把 set_speed_profile 写进换态表. 它没有对应的 ModeState, 最"自然"
    的实现是落到 IDLE -- 于是操作员在喊话模式里说一句"慢一点", 喊话就停了.
    本条断言模式纹丝不动.
    """
    pub = _Pub()
    face = _face(pub, initial=ModeState.BROADCAST)
    ack = face.handle_frame(
        _frame(action="set_speed_profile", profile="patrol"), now_mono_ms=1)
    assert ack["result"] == "accepted"
    assert face.state is ModeState.BROADCAST          # 没被踢回 idle
    assert pub.sent == []                             # 也没有假的 state/mode
    # SP-C3: 三字段必须齐, 否则操作员不知道到底切没切, 锁没锁.
    applied = ack["detail"]["applied"]
    assert set(applied) >= {"profile_to", "profile_locked", "max_profile"}


def test_reset_profile_lock_without_token_is_refused():
    """SP-C2: 解锁必须带执行方签发的 confirm_token. """
    ack = _face().handle_frame(
        _frame(action="reset_profile_lock"), now_mono_ms=1)
    assert ack["result"] == "rejected"


# -- 拒绝路径 ------------------------------------------------------------

def test_unknown_action_is_refused_with_schema():
    """S7.3 的六动作是闭集; 表外的一律拒. """
    ack = _face().handle_frame(_frame(action="fly"), now_mono_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_SCHEMA


def test_unknown_voice_mode_is_refused_not_snapped_to_nearest():
    """*** S13.6 禁止"未知值降级解释".

    模式决定本地麦是否被半双工门控关闭, 以及是否在喊话 -- 把一个不认识的
    voice_mode 解释成"最接近"的模式, 是让一条没人审过的字符串去拨这个开关.

    变异体: _VOICE_MODE_TO_STATE.get(wanted, ModeState.DIALOG_A) 这种带默认
    值的取法, 本条即绿转红.
    """
    ack = _face().handle_frame(
        _frame(action="set_voice_mode", voice_mode="stealth"), now_mono_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_SCHEMA
    assert "stealth" in ack["detail"]["reason"]


def test_blocked_domain_refuses_with_e_busy_and_names_every_domain():
    """*** 14 S12.1 场景 3 逐字: 整体拒绝 + blocked[] 一次报全不短路.

    码是 E_BUSY -- 14 S12.1 的样例就用它. 我一度写了 E_MODE_BLOCKED, 而 11
    S13 的 40 值闭集里没有这个码(CLAUDE.md 3.5 禁自造).

    变异体: 只报第一个被占的域 -> 操作员放开 ptz 后再试, 又被 speaker 拦住,
    要来回试. 本条断言两个域同时出现.
    """
    # 夹具照 14 S12.1 场景 3: 域5(ptz) 被 manual_cloud 抢走 -> blocked;
    # 域4(payload_light) 是 DIALOG 自己持有的 -> self_held. 两列必须都非空,
    # 否则"两者形状不同"这件事根本测不到.
    face = _face(arb=lambda: (frozenset({"ptz", "speaker"}),
                              frozenset({"payload_light"})))
    ack = face.handle_frame(
        _frame(action="set_voice_mode", voice_mode="alarm"), now_mono_ms=1)
    assert ack["result"] == "rejected"
    assert ack["code"] == E_BUSY
    # *** 元素是[对象]不是字符串: 14 S12.1 逐字 {"domain": "ptz", ...}.
    # 发字符串数组会让照契约写的消费方 b["domain"] 抛 TypeError -- 我第一版
    # 就是字符串, 是查这条错误码时顺带查出来的第二处形状分叉.
    assert ack["detail"]["blocked"] == [{"domain": "ptz"}, {"domain": "speaker"}]
    # holder 等字段[不填]而不是编: 仲裁器 MVP 不跑, 拿不到真的占用者.
    assert "holder" not in ack["detail"]["blocked"][0]
    # self_held 反之就是字符串数组(同一样例 ["payload_light"]) -- 两者形状
    # 不同是契约本来的样子.
    assert ack["detail"]["self_held"] == ["payload_light"]
    assert ack["detail"]["from_mode"] == "idle"
    assert ack["detail"]["to_mode"] == "alarm"


def test_a_malformed_frame_never_raises():
    """来自不鉴权 HMI(U23) 的垃圾帧不得掀掉 P2 的循环 -- 那个循环同时在跑
    健康度与设备域. """
    ack = _face().handle_frame(b"not json at all", now_mono_ms=1)
    assert ack["result"] in ("rejected", "error")


# -- 幂等 ----------------------------------------------------------------

def test_duplicate_cmd_id_replays_and_does_not_republish_state():
    """*** S2.3: 重发同一 cmd_id 回放首次结果, 不二次执行.

    变异体: 把回放也当成一次新的换态去发 state/mode -> HMI 会看到一次并未
    发生的模式变更, 事件流里也会多一条. 本条断言只发了一次.
    """
    pub = _Pub()
    face = _face(pub)
    raw = _frame(action="set_voice_mode", voice_mode="broadcast")
    first = face.handle_frame(raw, now_mono_ms=1)
    second = face.handle_frame(raw, now_mono_ms=2)
    assert first["result"] == "accepted"
    assert second["result"] == "duplicate"
    assert len(pub.sent) == 1


# -- 元测试: 换态表与闭集的关系 ------------------------------------------

def test_every_mode_changing_action_is_in_the_closed_set():
    """换态表不得含 S7.3 六动作以外的键 -- 否则就是在代码里偷加了一个动作. """
    from xbrain.p2_core.mode_actions.dispatch import ACTION_CLOSED_SET
    assert set(mode_wiring._ACTION_TO_STATE) <= set(ACTION_CLOSED_SET)


def test_voice_mode_table_covers_every_mode_state():
    """*** 反向差集: 每个 ModeState 都要有一条 voice_mode 能到达.

    否则会出现"切得进去切不出来"的态. 这条元测试是双向的另一半 --
    只查表内的值合法, 一个漏掉整个 ALARM 的表同样全绿.
    """
    reachable = set(mode_wiring._VOICE_MODE_TO_STATE.values())
    assert reachable == set(ModeState)
