"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_rx_refreshes_link.py
Brief: 11 S4.6.3 step 1 -- every inbound cloud message refreshes the outage clock

Description:
The defect, and it reached the customer: on_cloud_rx was wired to event/ack and
recon/rsp only, while 11 S4.6.3 step 1 lists "心跳 pong . cmd/** . event/ack .
data/ack". A cloud that dispatches tasks but never acks an event therefore looked
silent: state/link stayed reason=never_connected, disconnected_s climbed past
rtb_s, and P3's link-loss trigger injected a return_home at priority 95 that
preempted the task the customer had just sent.

The wrapper is asserted at the DECLARATION site rather than per handler: five
handlers means five places to forget, and the sixth (the next subscriber someone
adds) is guaranteed to be forgotten -- which is exactly how this happened.
"""

from __future__ import annotations

import ast
import inspect
import pathlib

import pytest

from xbrain.p5_gateway.runtime import cloud_wiring
from xbrain.p5_gateway.runtime.cloud_wiring import CloudBridge, maybe_wire

# INF-TS-1: 纯单测, 不碰设备(无 zenohd / 无底盘 / 无 ORIN 专属硬件).
pytestmark = pytest.mark.no_device


class _FakeSession:
    """Records declared subscribers so a test can feed one a sample."""

    def __init__(self):
        self.subs = {}      # key -> handler
        self.pubs = {}

    def declare_subscriber(self, key, handler):
        self.subs[key] = handler
        return object()

    def declare_publisher(self, key):
        self.pubs[key] = _FakePub()
        return self.pubs[key]


class _FakePub:
    def __init__(self):
        self.puts = []

    def put(self, payload):
        self.puts.append(payload)


class _Sample:
    def __init__(self, key, payload=b"{}"):
        self.key_expr = key
        self.payload = payload


def _bridge(on_rx=None):
    s = _FakeSession()
    b = CloudBridge(s, "gj-001", on_cloud_rx=on_rx)
    b.wire()
    return b, s


def test_an_inbound_task_command_refreshes_the_outage_clock():
    """THE defect. 11 S4.6.3 step 1 names cmd/** explicitly; a customer that
    dispatches tasks is a customer we can hear.
    MUTATION: drop the _rx wrapper from the cmd/task subscriber -> red."""
    seen = []
    _b, s = _bridge(on_rx=lambda: seen.append(1))
    s.subs["xbrain/gj-001/cmd/task"](_Sample("xbrain/gj-001/cmd/task"))
    assert seen, "an inbound cmd/task did not refresh the outage clock"


def test_every_cloud_subscriber_refreshes_it_not_just_cmd_task():
    """The contract says ANY cloud message. Five cloud subscribers today; each
    one is evidence the cloud is alive.
    MUTATION: wrap only cmd/task -> red for the other four."""
    seen = []
    _b, s = _bridge(on_rx=lambda: seen.append(1))
    cloud = [k for k in s.subs if k.startswith("xbrain/")]
    assert len(cloud) >= 5, cloud
    for key in cloud:
        before = len(seen)
        s.subs[key](_Sample(key))
        assert len(seen) > before, "%s did not refresh the outage clock" % key


def test_internal_traffic_must_never_refresh_the_outage_clock():
    """*** The dangerous direction, and the reason the wrapper is not applied to
    every subscriber the bridge declares.

    The bridge also subscribes to机内 keys (cmd/task/ack, cmd/geo/ack,
    state/fence). Those are OUR OWN traffic and they never stop, so refreshing
    the clock on them would peg the link at "up" forever -- a real outage would
    never be detected at all. That is strictly worse than the bug this file
    exists for: a false outage preempts a task, a false "connected" silently
    disables every link-loss safeguard (TSK-21 return-to-base included).

    MUTATION: wrap the internal subscribers too -> red.
    """
    seen = []
    _b, s = _bridge(on_rx=lambda: seen.append(1))
    internal = [k for k in s.subs if not k.startswith("xbrain/")]
    assert internal, "expected the bridge to hold internal subscriptions too"
    for key in internal:
        s.subs[key](_Sample(key))
    assert not seen, (
        "internal key(s) %r refreshed the cloud outage clock" % internal)


def test_a_malformed_message_still_counts_as_contact():
    """"任何一条报文", not "任何一条合法报文". A garbled dispatch still proves the
    cloud is there, and that is precisely when a false outage is most harmful.
    MUTATION: notify only after a successful json parse -> red."""
    seen = []
    _b, s = _bridge(on_rx=lambda: seen.append(1))
    s.subs["xbrain/gj-001/cmd/task"](_Sample("xbrain/gj-001/cmd/task",
                                             b"not json at all"))
    assert seen


def test_a_failing_notifier_does_not_swallow_the_command():
    """The command matters more than the link timer. A raising callback must not
    stop the dispatch from being processed.
    MUTATION: let the exception propagate -> red."""
    def boom():
        raise RuntimeError("clock broke")
    _b, s = _bridge(on_rx=boom)
    # Must not raise out of the subscriber.
    s.subs["xbrain/gj-001/cmd/task"](_Sample("xbrain/gj-001/cmd/task"))


def test_the_bridge_works_without_a_notifier_and_stays_quiet(caplog):
    """maybe_wire's caller may not have a link state (the dev loop has none).

    Not just "does not crash": calling None() would be caught by the wrapper's
    own except and logged, so the absence of a crash proves nothing (the
    try/except masks the missing guard). What the guard actually buys is silence
    -- without it EVERY inbound message logs an exception traceback, which buries
    the real errors in the gateway log.
    MUTATION: replace the `is not None` guard with `if True:` -> red on the log.
    """
    _b, s = _bridge(on_rx=None)
    with caplog.at_level("ERROR"):
        s.subs["xbrain/gj-001/cmd/task"](_Sample("xbrain/gj-001/cmd/task"))
    noisy = [r for r in caplog.records if "rx notify" in r.getMessage()]
    assert not noisy, (
        "a bridge with no notifier logged an error per inbound message: %r"
        % [r.getMessage() for r in noisy])


def test_the_wrapper_is_applied_where_subscribers_are_declared():
    """Per-handler notification would be five places to forget and a sixth
    guaranteed when a subscriber is added. Asserted structurally so a new
    subscriber cannot quietly skip it.
    MUTATION: unwrap any one declare_subscriber -> red."""
    src = inspect.getsource(CloudBridge.wire)
    tree = ast.parse("class C:\n" + "\n".join(
        "    " + ln for ln in src.splitlines()))
    bare_cloud, wrapped_internal = [], []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "attr", "") != "declare_subscriber":
            continue
        if len(node.args) < 2:
            continue
        key, handler = node.args[0], node.args[1]
        # Cloud keys are built as CLOUD_X % rid; internal ones are plain
        # literals. Telling them apart structurally is what lets this case
        # assert BOTH directions instead of demanding a blanket wrap.
        is_cloud = isinstance(key, ast.BinOp) and isinstance(key.op, ast.Mod)
        wrapped = (isinstance(handler, ast.Call)
                   and getattr(handler.func, "attr", "") == "_rx")
        if is_cloud and not wrapped:
            bare_cloud.append(ast.dump(handler)[:60])
        if not is_cloud and wrapped:
            wrapped_internal.append(ast.dump(key)[:60])
    assert not bare_cloud, (
        "cloud subscriber(s) declared without the _rx wrapper: %r" % bare_cloud)
    assert not wrapped_internal, (
        "internal subscriber(s) wrapped -- our own traffic would peg the link "
        "at up and disable outage detection: %r" % wrapped_internal)


def test_maybe_wire_passes_the_notifier_through():
    """The wrapper is useless if the runtime never hands the bridge a notifier.
    MUTATION: drop on_cloud_rx from the CloudBridge(...) call -> red."""
    seen = []
    s = _FakeSession()
    b = maybe_wire(s, "gj-001", on_cloud_rx=lambda: seen.append(1))
    assert b is not None
    s.subs["xbrain/gj-001/cmd/task"](_Sample("xbrain/gj-001/cmd/task"))
    assert seen, "maybe_wire did not pass on_cloud_rx to the bridge"


def test_p5_runtime_hands_the_bridge_the_real_link_state():
    """The last link in the chain: the wrapper and the pass-through are both
    pointless if the runtime wires a notifier that is not the link state.
    Read from p5's real source -- a fake bridge would just do the right thing.
    MUTATION: call maybe_wire without on_cloud_rx -> red."""
    src = (pathlib.Path(cloud_wiring.__file__).parent
           / "main_wiring.py").read_text(encoding="utf-8")
    call = src[src.index("cloud_bridge = maybe_wire("):]
    call = call[:call.index("\n\n")]
    assert "on_cloud_rx=" in call, "p5 wires the bridge without a rx notifier"
    assert "link_state.on_cloud_rx" in call, (
        "the notifier does not reach the LinkStateMachine")


# --- 11 S2.2.2A: Qt 在线心跳 ------------------------------------------

def test_the_qt_heartbeat_refreshes_the_outage_clock():
    """*** 心跳存在的全部意义就是"它到了".

    在它之前 Qt 可以长时间只订阅不发布, on_cloud_rx 根本不会被调用 -- 链路恒
    为 never_connected, disconnected_s 一路累到 rtb_s(1800s)触发返航(TSK-21),
    而甲方明明在线. 2026-09-02/03 连续发生了三次.

    MUTATION: heartbeat 订阅不过 _rx 包装 -> 红.
    """
    seen = []
    _b, s = _bridge(on_rx=lambda: seen.append(1))
    assert "xbrain/gj-001/heartbeat/qt" in s.subs, sorted(s.subs)
    s.subs["xbrain/gj-001/heartbeat/qt"](
        _Sample("xbrain/gj-001/heartbeat/qt",
                b'{"v":1,"rid":"gj-001","src":"qt_hmi",'
                b'"data":{"session_id":"hmi-1","state":"up"}}'))
    assert seen, "心跳没有刷新断线计时"


def test_a_heartbeat_is_not_acked():
    """单向存活信号. 回 ack 会在两侧之间形成 1 Hz 的来回, 而 11 S2.2.2A 也
    没有给它定义应答 key.

    MUTATION: 在处理器里 put 一条 ack -> 红.
    """
    _b, s = _bridge(on_rx=lambda: None)
    s.subs["xbrain/gj-001/heartbeat/qt"](
        _Sample("xbrain/gj-001/heartbeat/qt",
                b'{"v":1,"rid":"gj-001","data":{"session_id":"h","state":"up"}}'))
    assert not s.pubs.get("xbrain/gj-001/heartbeat/qt/ack"), s.pubs.keys()


def test_an_out_of_set_state_is_reported_not_silently_taken_as_up(caplog):
    """闭集是 up|down(11 S2.2.2A). 收到别的值说明两侧对协议的理解已经不一致,
    这时按"大概是 up"处理等于拿一个猜测去喂链路判据.

    MUTATION: 去掉 state 闭集检查 -> 红.
    """
    _b, s = _bridge(on_rx=lambda: None)
    with caplog.at_level("ERROR"):
        s.subs["xbrain/gj-001/heartbeat/qt"](
            _Sample("xbrain/gj-001/heartbeat/qt",
                    b'{"v":1,"rid":"gj-001","data":{"state":"paused"}}'))
    assert any("closed set" in r.getMessage() for r in caplog.records), (
        [r.getMessage() for r in caplog.records])


def test_a_malformed_heartbeat_still_counted_as_contact():
    """坏报文同样证明云端在线 -- 刷新在 _rx 里, 先于本处理器.

    MUTATION: 把刷新挪进处理器的 try 里 -> 红.
    """
    seen = []
    _b, s = _bridge(on_rx=lambda: seen.append(1))
    s.subs["xbrain/gj-001/heartbeat/qt"](
        _Sample("xbrain/gj-001/heartbeat/qt", b"not json"))
    assert seen
