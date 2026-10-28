"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_lapi_focus.py
Brief: LAPI focus client -- refocus order and end state, restore on failure, write guard, F1/F2 session

Description:
lapi_focus.LapiFocusClient is the only code that writes the PTZ camera's
LAPI focus settings (11 S7.4.7 F1 / F2 / F4, 99 U98). These cases pin:
  * refocus is GET, PUT FocusMode=0, dwell 300 ms, PUT FocusMode=auto -- and
    the camera ends in auto;
  * every failure after FocusMode left auto triggers one more auto write
    BEFORE returning, and focus_mode_restored tells the truth about it;
  * a failure before anything was written reports restored=True (it never
    left auto) -- the ack must not claim a fault that did not happen;
  * success is ResponseCode == 0 in the body, not HTTP 200;
  * the write guard (domains/lapi_guard.py) refuses a non-authorised key
    BEFORE any request is sent (no bitrate write can slip out, 11 B0);
  * session_start sets F1 + F2 and remembers MovePTZ; session_end restores
    MovePTZ and keeps FocusMode auto.
The HTTP transport is a scripted fake; no camera. Each case names its mutant.
"""
from __future__ import annotations  # postponed annotations

import json  # fake LAPI bodies are JSON

import pytest  # test runner

from xbrain.p2_core.domains.lapi_guard import LapiWriteViolation  # the guard's exception type
from xbrain.p2_core.ptz.lapi_focus import (  # the module under test
    MANUAL_FOCUS_MODE,  # the intermediate value
    REFOCUS_DWELL_MS,  # the contract dwell
    LapiFocusClient,  # the client
)  # end of imports

pytestmark = pytest.mark.no_device  # runs without any device


def _resp(code=0, data=None):  # build a LAPI answer
    """A LAPI response body."""
    return json.dumps({"Response": {"ResponseCode": code,  # Response wrapper, as the camera sends
                                    "Data": data or {}}}).encode("utf-8")  # Data defaults to an empty object


class FakeHttp:  # scripted transport
    """Records (method, body) and answers from a script; the camera's focus
    object starts in auto with MovePTZ 0 (the factory value)."""

    def __init__(self, fail_on=None, bad_code_on=None):  # which calls fail, which answer badly
        self.calls = []                  # (method, decoded body or None)
        self.fail_on = fail_on or set()  # call indexes that raise (transport)
        self.bad_code_on = bad_code_on or set()   # indexes answering code != 0
        self.state = {"FocusMode": 2, "ShieldTrigger": {"MovePTZ": 0}, "Other": 7}  # camera starts in auto, MovePTZ at the factory 0

    def __call__(self, method, url, body, timeout_s):  # same signature as HttpCall
        i = len(self.calls)  # index of this call
        self.calls.append((method, json.loads(body) if body else None))  # decoded body for the asserts
        assert timeout_s == 0.5                  # the configured per-call bound
        if i in self.fail_on:  # transport failure here
            raise OSError("timed out")  # like a socket timeout
        if i in self.bad_code_on:  # HTTP 200 but refused
            return _resp(code=5)  # non-zero ResponseCode
        if method == "PUT":  # a successful write
            self.state = json.loads(body)  # the camera adopts the whole object
        return _resp(data=json.loads(json.dumps(self.state)))  # copy so later edits do not alias


def make(http):  # client wired to a fake
    """A client with recorded sleeps."""
    slept = []  # dwell durations recorded
    c = LapiFocusClient("192.168.66.13", http=http, timeout_s=0.5,  # 0.5 s per call, as configured
                        focus_mode_auto=2, shield_move_ptz=1, sleep=slept.append)  # F2 auto 2, F1 shield 1
    c.slept = slept  # recorded dwell
    return c  # ready


def test_refocus_order_and_end_state():  # happy path
    """GET, PUT 0, dwell, PUT 2; the camera ends in auto and the other fields
    are written back untouched. MUTATION: skip the final auto write -> state
    stays 0 -> red; drop the dwell -> slept empty -> red."""
    http = FakeHttp()  # camera healthy
    c = make(http)  # client
    assert c.refocus() == (True, True)  # ok and auto restored
    assert [m for m, _b in http.calls] == ["GET", "PUT", "PUT"]  # read, manual write, auto write
    assert http.calls[1][1]["FocusMode"] == MANUAL_FOCUS_MODE  # the intermediate value really was manual
    assert http.state["FocusMode"] == 2 and http.state["Other"] == 7  # auto at the end, other fields untouched
    assert c.slept == [REFOCUS_DWELL_MS / 1000.0]  # exactly one 300 ms dwell


def test_read_failure_reports_restored_true():  # failure before any write
    """Nothing was written -> FocusMode never left auto -> restored True.
    MUTATION: report restored False -> red."""
    http = FakeHttp(fail_on={0})  # the GET fails
    assert make(http).refocus() == (False, True)  # not ok, but never left auto
    assert [m for m, _b in http.calls] == ["GET"]  # nothing was written


def test_auto_write_failure_retries_and_reports():  # second write fails once
    """The auto write fails once: one more auto write before returning, and
    restored reflects that it succeeded. MUTATION: no restore attempt ->
    camera left in manual and restored claimed -> red."""
    http = FakeHttp(fail_on={2})  # index 2 = the auto write
    assert make(http).refocus() == (False, True)  # not ok, but restored by the retry
    assert http.calls[-1] == ("PUT", http.state) and http.state["FocusMode"] == 2  # the retry put auto back


def test_restore_failure_is_reported_false():  # both auto writes fail
    """Both auto writes fail -> restored False, the ack will say so."""
    http = FakeHttp(fail_on={2, 3})  # auto write and its retry
    assert make(http).refocus() == (False, False)  # the ack will say restored=False


def test_response_code_not_http_status_decides():  # ResponseCode beats HTTP status
    """HTTP 200 with ResponseCode 5 on the manual write is a failure.
    MUTATION: ignore ResponseCode -> (True, True) -> red."""
    http = FakeHttp(bad_code_on={1})  # index 1 = the manual write answers code 5
    ok, restored = make(http).refocus()  # the client's verdict
    assert ok is False and restored is True  # failure, and auto never left


def test_guard_refuses_unauthorised_key_before_sending():  # 11 B0 guard
    """11 B0: a bitrate write raises before any request goes out.
    MUTATION: send first, check after -> a recorded PUT -> red."""
    http = FakeHttp()  # healthy camera
    c = make(http)  # client
    with pytest.raises(LapiWriteViolation):  # the guard must raise
        c._put({"VideoBitrate": 16384}, ["VideoBitrate"])  # a forbidden key
    assert http.calls == []  # and nothing was sent


def test_session_start_sets_f1_f2_and_end_restores():  # F1 / F2 round trip
    """F1 MovePTZ=1 + F2 auto at start; at end MovePTZ goes back to the
    remembered 0 and FocusMode stays auto. MUTATION: forget the original ->
    MovePTZ stays 1 after end -> red."""
    http = FakeHttp()  # camera starts healthy
    http.state["FocusMode"] = 0                  # left in manual by a crash
    c = make(http)  # client
    assert c.session_start() is True  # start succeeds
    assert http.state["FocusMode"] == 2 and http.state["ShieldTrigger"]["MovePTZ"] == 1  # F2 auto and F1 shield in force
    assert c.session_end() is True  # end succeeds
    assert http.state["FocusMode"] == 2 and http.state["ShieldTrigger"]["MovePTZ"] == 0  # MovePTZ back to the remembered 0, still auto


def test_session_start_failure_is_not_fatal():  # unreachable camera
    """An unreachable camera returns False (logged), never raises."""
    assert make(FakeHttp(fail_on={0})).session_start() is False  # returns False, never raises


def test_timeout_must_be_positive():  # construction guard
    """A zero timeout would mean no bound at all (fail-silent)."""
    with pytest.raises(ValueError):  # zero is refused
        LapiFocusClient("h", http=FakeHttp(), timeout_s=0, focus_mode_auto=2,  # timeout 0 would mean no bound
                        shield_move_ptz=1)  # the rest is valid
