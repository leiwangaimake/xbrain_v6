"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_device_sync.py
Brief: DeviceModeSync -- POST /mode verdicts (200/409 ok, timeout/5xx fail), reconcile, lights

Description:
xbrain/p2_core/mode/device_sync.py is the first production caller of the 14
S5.7 two-layer mode mapping. Its absence is why every cloud broadcast on
2026-10-27 got HTTP 403 from payload-service's /play (device stuck in `idle`).

The transport is injected, so every HTTP outcome is driven without a
payload-service. Each case names its mutant (CLAUDE.md 3.3):
  * 409 treated as failure (ML-2)               -> test_409_is_ok
  * a timeout reported as success               -> test_no_response_is_a_failure
  * reconcile writes blind when /status is down -> test_reconcile_unknown_sends_nothing
  * reconcile skips a real drift                -> test_reconcile_repairs_drift
"""

from __future__ import annotations

import pytest  # raises + marks

from xbrain.p2_core.mode.device_sync import DeviceModeSync  # unit under test

pytestmark = pytest.mark.no_device  # no payload-service needed


class _Fake:  # scripted transport double
    """Scripted transport: records calls, answers from a queue."""

    def __init__(self, *answers):  # answers are consumed in order
        self.answers = list(answers)   # (status, body) or an exception
        self.calls = []                # (method, path, body, timeout)

    def __call__(self, method, url, body, timeout_s):  # same signature as the real transport
        self.calls.append((method, url.split("18080")[-1], body, timeout_s))  # record path only
        a = self.answers.pop(0)  # next scripted answer
        if isinstance(a, Exception):  # an exception scripts a transport failure
            raise a  # like a timeout or a refused socket
        return a  # (status, body)


def _sync(*answers):  # a DeviceModeSync on a fake transport
    fake = _Fake(*answers)  # the scripted answers
    return DeviceModeSync("http://127.0.0.1:18080/", 0.3, transport=fake), fake  # T-PAY-1 = 0.3 s


def test_200_switches_and_bounds_the_call_by_t_pay_1():
    s, fake = _sync((200, {"ok": True, "mode": "func2", "previous": "func1"}))  # a real switch
    r = s.apply("func2")  # enter B's device mode
    assert r.ok and r.status == 200  # 200 = switched
    assert fake.calls == [("POST", "/mode", {"mode": "func2"}, 0.3)]  # one bounded POST
    assert s.confirmed == "func2"  # the new mode is remembered


def test_409_is_ok():
    """ML-2: re-entering the current device mode is success to us.
    MUTATION: only 200 counts -> this fails."""
    s, _ = _sync((409, {"detail": "already in func1"}))  # service says: already there
    r = s.apply("func1")  # re-enter the current mode
    assert r.ok and r.status == 409  # ML-2: success to us
    assert s.confirmed == "func1"  # and the mode is known


def test_no_response_is_a_failure():
    """T-PAY-1 timeout -> E_UNHEALTHY reason service_timeout (14 S2.3).
    MUTATION: treat the exception as "probably switched" -> ok -> fails."""
    s, _ = _sync(TimeoutError("timed out"))  # no answer inside T-PAY-1
    r = s.apply("func2")  # enter B's device mode
    assert not r.ok and r.status == 0 and r.reason == "service_timeout"  # the E_UNHEALTHY reason
    assert s.confirmed is None  # unknown, not the old value


def test_server_error_is_a_failure():
    s, _ = _sync((503, {}))  # service up but failing
    r = s.apply("func2")  # enter B's device mode
    assert not r.ok and r.status == 503 and r.reason == "payload_mode"  # refused, with the status


def test_unknown_target_is_a_programming_error():
    s, fake = _sync()  # no answers needed
    with pytest.raises(ValueError):  # loud on a caller bug
        s.apply("broadcast")             # a SYSTEM mode, not a device mode
    assert fake.calls == []  # nothing went on the wire


def test_reconcile_in_place_sends_nothing():
    s, fake = _sync((200, {"mode": "func1"}))  # device already in func1
    assert s.reconcile("func1") is None  # nothing sent
    assert [c[0] for c in fake.calls] == ["GET"]  # only the status read


def test_reconcile_repairs_drift():
    """payload-service restarted and booted into idle: MM-1 wants func1.
    MUTATION: reconcile never POSTs -> device stays idle, /play refused."""
    s, fake = _sync((200, {"mode": "idle"}), (200, {"mode": "func1"}))  # drifted to idle
    r = s.reconcile("func1")  # MM-1 target
    assert r is not None and r.ok  # a corrective POST went out
    assert [(c[0], c[1]) for c in fake.calls] == [("GET", "/status"),  # read, then
                                                  ("POST", "/mode")]  # repair


def test_reconcile_unknown_sends_nothing():
    """/status unreachable = unknown -> retry next heartbeat, never write
    blind. MUTATION: treat None as drift and POST -> a second call appears."""
    s, fake = _sync(ConnectionRefusedError())  # service down
    assert s.reconcile("func1") is None  # nothing sent
    assert len(fake.calls) == 1  # only the failed GET


def test_lights_is_one_partial_post_and_never_raises():
    """GL-8a / ML-4: one POST with both fields; failure is reported only."""
    s, fake = _sync((200, {}), TimeoutError())  # first ok, then silent
    assert s.set_lights({"redblue": 1, "strobe": False}).ok  # entering B
    assert fake.calls[0][:3] == ("POST", "/lights", {"redblue": 1, "strobe": False})  # one request
    r = s.set_lights({"redblue": 0})  # leaving B
    assert not r.ok and r.reason == "service_timeout"  # reported, not raised
