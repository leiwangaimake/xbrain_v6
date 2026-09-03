"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_heartbeat_new_session.py
Brief: HB-2 (11 S2.2.2A) -- a new session_id forces the full-snapshot keys out

Description:
v2.0 S4.5 逐字要求"session 建立后 2 秒内必须发布一份 full=true 的全量清单",
而在心跳之前我方[拿不到 session 建立信号] -- Qt 的订阅是 Zenoh 内部行为, 不产生
任何回调. 只能用周期重发近似(cdbbd5c 把 manifest 的周期从 None 改成 2.0,
b11904b 给 file/index 补 5.0). session_id 让这条要求第一次能按字面实现.

这里守的第一件事是[同一 session 不得反复触发]: 心跳是 1 Hz, 每拍都重发全量
清单会把一条低频全量面刷成 1 Hz, 而它恰恰是最大的那几条报文之一.
"""

from __future__ import annotations

import pytest

from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge
from tests.p5_gateway.test_cloud_rx_refreshes_link import (
    _FakeSession, _Sample,
)

pytestmark = pytest.mark.no_device


def _beat(session_id, state="up"):
    return ('{"v":1,"rid":"gj-001","src":"qt_hmi","data":'
            '{"session_id":"%s","state":"%s"}}' % (session_id, state)).encode()


def _bridge_with_session():
    seen = []
    sess = _FakeSession()
    b = CloudBridge(sess, "gj-001", on_cloud_rx=lambda: None,
                    on_new_session=lambda: seen.append(1))
    b.wire()
    return sess.subs["xbrain/gj-001/heartbeat/qt"], seen


def test_the_first_heartbeat_counts_as_a_new_session():
    """首次出现也算新 session -- 否则 p5 重启后 Qt 那边 session 没变, 全量清单
    永远不会因 HB-2 发出去(只能等周期).

    MUTATION: 只在 session_id 变化时触发而首次不算 -> 红.
    """
    hb, seen = _bridge_with_session()
    hb(_Sample("xbrain/gj-001/heartbeat/qt", _beat("s-1")))
    assert seen == [1], seen


def test_the_same_session_does_not_retrigger():
    """*** 心跳是 1 Hz. 每拍都重发全量清单会把一条低频全量面刷成 1 Hz.

    MUTATION: 去掉 session_id != self._hb_session 判断 -> 红.
    """
    hb, seen = _bridge_with_session()
    for _ in range(10):
        hb(_Sample("xbrain/gj-001/heartbeat/qt", _beat("s-1")))
    assert seen == [1], "同一 session 触发了 %d 次" % len(seen)


def test_a_changed_session_triggers_again():
    """Qt 重启或重建 Zenoh session -> 新值 -> 必须再发一次.

    MUTATION: 记住第一个就再也不更新 -> 红.
    """
    hb, seen = _bridge_with_session()
    hb(_Sample("xbrain/gj-001/heartbeat/qt", _beat("s-1")))
    hb(_Sample("xbrain/gj-001/heartbeat/qt", _beat("s-2")))
    assert len(seen) == 2, seen


def test_a_down_beat_does_not_trigger_a_resend():
    """一条 down 同样带 session_id, 但那时重发全量清单没有意义 -- 对方正要走.

    MUTATION: 不看 state 只看 session_id -> 红.
    """
    hb, seen = _bridge_with_session()
    hb(_Sample("xbrain/gj-001/heartbeat/qt", _beat("s-new", state="down")))
    assert seen == [], seen


def test_a_beat_without_a_session_id_is_ignored():
    """缺字段的心跳不该被当成"新 session" -- 那会让每一条坏报文都触发重发.

    MUTATION: 去掉 session_id 非空判断 -> 红.
    """
    hb, seen = _bridge_with_session()
    # 初始态下"缺字段"与"还没见过 session"恰好都是 None, 两者相等所以不触发 --
    # 这一步单独测不出守卫的作用. 守卫真正起作用的是[已经见过一个 session
    # 之后]再来一条缺字段的: 没有守卫的话 None != "s-1" 为真, 会误判成新
    # session 并把记录的 session 抹成 None(于是下一拍真心跳又触发一次).
    hb(_Sample("xbrain/gj-001/heartbeat/qt",
               b'{"v":1,"rid":"gj-001","data":{"state":"up"}}'))
    assert seen == [], seen

    hb(_Sample("xbrain/gj-001/heartbeat/qt", _beat("s-1")))
    assert len(seen) == 1, seen
    hb(_Sample("xbrain/gj-001/heartbeat/qt",
               b'{"v":1,"rid":"gj-001","data":{"state":"up"}}'))
    assert len(seen) == 1, "缺 session_id 的心跳被当成了新 session"
    # 记录没有被抹掉: 同一个 session 的下一拍仍然不触发.
    hb(_Sample("xbrain/gj-001/heartbeat/qt", _beat("s-1")))
    assert len(seen) == 1, "session 记录被缺字段的那一条抹掉了"


# --- 投影层: force_resend 真的让那两条 key 重新发出去 --------------------

def test_force_resend_clears_the_cadence_bookkeeping():
    """做法是清节律记账而不是直接 put -- 直接 put 会绕过 tick 里的组装与闭集
    校验, 变成第二条发布路径, 两条路径迟早对同一条 key 发出两种形状.

    MUTATION: force_resend 改成空实现 -> 红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import CloudProjector

    class _B:
        def __init__(self):
            self.published = []

        def publish_state(self, name, data):
            self.published.append(name)

    proj = CloudProjector(_B(), now_mono=lambda: 0.0)
    proj._last_sent["state/geo/manifest"] = 0.0
    proj._last_body["state/geo/manifest"] = "whatever"
    proj.force_resend()
    assert "state/geo/manifest" not in proj._last_sent
    assert "state/geo/manifest" not in proj._last_body


def test_the_session_scoped_keys_are_the_two_full_snapshot_faces():
    """两条都是"连接/变化时发完整快照"型. 漏掉 file/index 的话, Qt 连上后
    永远拿不到文件索引(它恒为空数组, 也就恒不变化).

    MUTATION: 从 SESSION_FULL_KEYS 删掉任意一条 -> 红.
    """
    from xbrain.p5_gateway.runtime.cloud_state import CloudProjector

    assert set(CloudProjector.SESSION_FULL_KEYS) == {
        "state/geo/manifest", "data/file/index"}


def test_the_runtime_hands_the_bridge_the_resend_entry():
    """接线断了的话上面全部退化成"只有周期".

    MUTATION: main_wiring 不传 on_new_session -> 红.
    """
    import pathlib

    from xbrain.p5_gateway.runtime import cloud_wiring

    src = (pathlib.Path(cloud_wiring.__file__).parent
           / "main_wiring.py").read_text(encoding="utf-8")
    call = src[src.index("cloud_bridge = maybe_wire("):]
    call = call[:call.index("\n\n")]
    assert "on_new_session=" in call, call
    assert "force_resend" in call, call
