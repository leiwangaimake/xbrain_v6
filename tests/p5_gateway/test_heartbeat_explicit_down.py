"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_heartbeat_explicit_down.py
Brief: HB-1 (11 S4.6.3 step 3) -- heartbeat state=down forces the link down

Description:
11 S4.6.3 原本只有一条 gap 驱动的入口("多久没听到"). heartbeat/qt 带来了第二个
输入: 对方明确告知它要下线. 再等 degraded_s(5 s)才认, 是把[已知]当[未知].

这里守三件容易写反的事:
  * down_since_mono 仍从[最后一次听到]起算, 不是从判定时刻 -- 否则
    disconnected_s 会比真实断开时长短一截, 而它是 TSK-21 返航判据的输入;
  * disconnected_s 不得跳变或归零 -- 操作员看到的断开时长必须连续;
  * _rx 包装先刷新 . down 后覆盖的[顺序] -- 反了的话这一条 down 自己会把自己
    刷成"在线".
"""

from __future__ import annotations

import pytest

from xbrain.p5_gateway.uplink.link_state import (
    LinkStateMachine, LinkThresholds,
)

pytestmark = pytest.mark.no_device

#: 与生产同值(11 S4.6.2): 5 s 降级 / 20 s 断开 / 1800 s 返航 / 10 s 稳定.
_TH = LinkThresholds(degraded_s=5.0, down_s=20.0, rtb_s=1800.0, stable_s=10.0)


def _machine(**kw):
    return LinkStateMachine(_TH, gw_start_mono=0.0, **kw)


def _up(m, t):
    """喂心跳直到链路稳定 up(要连续 stable_s)."""
    for i in range(int(t)):
        m.on_cloud_rx(float(i))
        m.evaluate(float(i))
    return m.evaluate(float(t))


def test_an_explicit_down_takes_effect_immediately():
    """*** HB-1 的靶心: 不等 degraded_s.

    MUTATION: 去掉 evaluate 里的 not self._forced_down -> 红(gap 还很小,
    仍会被判成 up).
    """
    m = _machine()
    snap = _up(m, 20)
    assert snap.cloud_link == "up", snap

    m.on_cloud_explicit_down()
    snap = m.evaluate(20.5)          # gap 只有 0.5 s, 远小于 degraded_s
    assert snap.cloud_link != "up", snap
    assert snap.reason != "ok" or snap.level > 0, snap


def test_an_explicit_down_does_not_skip_levels():
    """*** 立即离开 up, 但 NO 不得跳级.

    level 仍按 disconnected_s 的阈值梯度走 -- level 3 意味着返航(TSK-21, 30
    分钟). 显式下线只是说"我走了", 不是说"我已经走了半小时"; 跳到 L3 会让
    操作员一关 Qt 机器人就自己往回跑.

    MUTATION: 在 on_cloud_explicit_down 里直接写 level = 3 或 2 -> 红.
    """
    m = _machine()
    _up(m, 20)
    m.on_cloud_explicit_down()
    assert m.evaluate(20.5).level == 1, "刚下线就跳过了 degraded"
    assert m.evaluate(25.0).level == 1, "还没到 down_s 就升级了"
    assert m.evaluate(45.0).level == 2, "过了 down_s 应升到 2"
    assert m.evaluate(1000.0).level == 2, "远未到 rtb_s 却升到了 3"


def test_the_outage_is_timed_from_the_last_beat_not_from_the_notice():
    """down_since_mono 从[最后一次听到]起算(11 S4.6.3 步骤二同一口径).

    从判定时刻起算的话, disconnected_s 会短一截 -- 而它是 TSK-21 返航判据的
    输入, 短了就等于把返航往后推.
    MUTATION: down_since_mono = now -> 红.
    """
    m = _machine()
    _up(m, 20)                       # 最后一次听到 = t=19
    m.on_cloud_explicit_down()
    snap = m.evaluate(30.0)
    # 从 t=19 起算 -> 11 s; 从"通知时刻"起算 -> 只有 10 s 或更少
    assert snap.disconnected_s >= 10.9, snap.disconnected_s


def test_disconnected_seconds_never_jumps_backwards():
    """操作员看到的断开时长必须连续 -- 归零或跳变会让"断了多久"这个数不可信.

    MUTATION: 在 on_cloud_explicit_down 里把 disconnected_s 清零 -> 红.
    """
    m = _machine()
    _up(m, 20)
    m.on_cloud_explicit_down()
    prev = -1.0
    for t in (20.5, 22.0, 25.0, 30.0):
        cur = m.evaluate(t).disconnected_s
        assert cur >= prev, "disconnected_s 回退: %r -> %r" % (prev, cur)
        prev = cur


def test_a_later_beat_lifts_the_forced_down():
    """对方又出声了, 强制断开的理由消失. 但 NO 不立刻置 up -- LNK-3 的迟滞
    (连续 stable_s)仍然要走.

    MUTATION: on_cloud_rx 不清 _forced_down -> 红(永远回不来).
    """
    m = _machine()
    _up(m, 20)
    m.on_cloud_explicit_down()
    assert m.evaluate(21.0).cloud_link != "up"
    # 对方回来, 连续心跳
    for i in range(21, 45):
        m.on_cloud_rx(float(i))
        m.evaluate(float(i))
    assert m.evaluate(45.0).cloud_link == "up", "强制断开没有被后续心跳解除"


def test_link_epoch_advances_once_per_outage_not_per_notice():
    """link_epoch 是"这是同一次断开吗"的判据. 连发两条 down 不该让它 +2.

    MUTATION: 无条件 link_epoch += 1 -> 红.
    """
    m = _machine()
    _up(m, 20)
    m.on_cloud_explicit_down()
    e1 = m.evaluate(21.0).link_epoch
    m.on_cloud_explicit_down()
    e2 = m.evaluate(22.0).link_epoch
    assert e1 == e2, (e1, e2)


def test_the_bridge_reports_down_only_for_state_down():
    """up 的心跳 NO 不得触发强制断开 -- 否则每一拍心跳都把链路打断.

    MUTATION: 把 state == "down" 判断去掉 -> 红.
    """
    from tests.p5_gateway.test_cloud_rx_refreshes_link import (
        _Sample, _FakeSession)
    from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge

    downs = []
    sess = _FakeSession()
    b = CloudBridge(sess, "gj-001", on_cloud_rx=lambda: None,
                    on_cloud_down=lambda: downs.append(1))
    b.wire()
    hb = sess.subs["xbrain/gj-001/heartbeat/qt"]
    hb(_Sample("xbrain/gj-001/heartbeat/qt",
               b'{"v":1,"rid":"gj-001","data":{"session_id":"s","state":"up"}}'))
    assert not downs, "up 的心跳触发了强制断开"
    hb(_Sample("xbrain/gj-001/heartbeat/qt",
               b'{"v":1,"rid":"gj-001","data":{"session_id":"s","state":"down"}}'))
    assert downs == [1], downs


def test_the_runtime_hands_the_bridge_the_real_down_entry():
    """接线断了的话上面全部退化 -- 桥拿不到入口就什么也不会发生.

    MUTATION: main_wiring 不传 on_cloud_down -> 红.
    """
    import inspect
    import pathlib

    from xbrain.p5_gateway.runtime import cloud_wiring

    src = (pathlib.Path(cloud_wiring.__file__).parent
           / "main_wiring.py").read_text(encoding="utf-8")
    call = src[src.index("cloud_bridge = maybe_wire("):]
    call = call[:call.index("\n\n")]
    assert "on_cloud_down=link_state.on_cloud_explicit_down" in call, call
