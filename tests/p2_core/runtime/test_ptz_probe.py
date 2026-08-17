"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptz_probe.py
Brief: PtzLivenessProbe -- background 3-state reachability probe (SW-12)

Description:
The probe runs an injected check on a thread and exposes the latest verdict, so the
p2 heartbeat never blocks on the ONVIF round-trip. Tests pin: unknown before the
first poll; VERDICT_UP -> True, VERDICT_DOWN -> False; and the LOCKOUT-SAFETY path
(PTZ report S8) -- VERDICT_AUTH stops the loop, sets reachable back to None (a config
fault is NOT a device_offline), and never retries (which would lock the camera).
Mutations paired per 3.3.
"""

import time

import pytest

from xbrain.p2_core.runtime.ptz_wiring import (
    VERDICT_AUTH, VERDICT_DOWN, VERDICT_UP, PtzLivenessProbe,
)


pytestmark = pytest.mark.no_device


def test_reachable_none_before_first_poll():
    p = PtzLivenessProbe(check_reachable=lambda: VERDICT_UP, period_s=10.0)
    assert p.reachable is None   # unknown until a poll completes -> bridge silent


def test_probe_reports_reachable():
    p = PtzLivenessProbe(check_reachable=lambda: VERDICT_UP, period_s=0.02)
    p.start()
    try:
        time.sleep(0.12)
        assert p.reachable is True
    finally:
        p.stop()


def test_probe_reports_down():
    p = PtzLivenessProbe(check_reachable=lambda: VERDICT_DOWN, period_s=0.02)
    p.start()
    try:
        time.sleep(0.12)
        # MUTATION: if DOWN were mapped to True, a down camera would never emit
        # device_offline.
        assert p.reachable is False
        assert p.auth_blocked is False   # transport-down must NOT block the probe
    finally:
        p.stop()


def test_auth_reject_stops_probe_and_stays_unknown():
    calls = {"n": 0}

    def _check():
        calls["n"] += 1
        return VERDICT_AUTH

    p = PtzLivenessProbe(check_reachable=_check, period_s=0.02)
    p.start()
    try:
        time.sleep(0.15)
        # LOCKOUT SAFETY: the loop must have stopped after the FIRST auth reject.
        # MUTATION: if the loop kept polling on auth, calls['n'] would climb past 1
        # (every 5s in prod = account lockout per PTZ report S8).
        assert calls["n"] == 1
        assert p.auth_blocked is True
        # An auth reject is a config fault, not a down camera -> None, NOT False,
        # so the bridge feeds nothing and no false device_offline is emitted.
        assert p.reachable is None
    finally:
        p.stop()


def test_unexpected_exception_treated_as_down():
    def _check():
        raise RuntimeError("unexpected")
    p = PtzLivenessProbe(check_reachable=_check, period_s=0.02)
    p.start()
    try:
        time.sleep(0.12)
        assert p.reachable is False        # defensive: unexpected error = transport down
        assert p.auth_blocked is False
    finally:
        p.stop()


def test_stop_is_idempotent_and_joins():
    p = PtzLivenessProbe(check_reachable=lambda: VERDICT_UP, period_s=0.02)
    p.start()
    time.sleep(0.05)
    p.stop()
    p.stop()   # second stop must not raise
