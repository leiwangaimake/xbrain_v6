"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_link_state.py
Brief: LinkStateMachine -- 11 S4.6 cloud-link state (LNK-1..6)

Description:
Pins the intricate parts of the state machine on a synthetic monotonic timeline:
cold start is never_connected NOT up (LNK-5); the outage escalates L1->L2 by
disconnected_s; the reconnect edge fires ONE backfill; the LNK-3 hysteresis holds
'degraded' for stable_s and a flap does not reset the timer; link_epoch bumps once
per outage. Mutations paired per 3.3.
"""

import pytest

from xbrain.p5_gateway.uplink.link_state import (
    LinkStateMachine, LinkThresholds,
)


pytestmark = pytest.mark.no_device


def _mk(gw=0.0, degraded=5.0, down=20.0, rtb=None, stable=10.0):
    return LinkStateMachine(
        LinkThresholds(degraded, down, rtb, stable), gw_start_mono=gw)


def _settle_up(m, t=1.0, stable=10.0):
    """Connect and advance past the LNK-3 hysteresis window so the machine sits at
    L0/up. Returns the monotonic time of the last evaluate."""
    m.on_cloud_rx(t)
    m.evaluate(t + 0.1)                    # reconnect edge -> degraded (in window)
    m.on_cloud_rx(t + stable + 0.5)
    end = t + stable + 0.6
    s = m.evaluate(end)                    # window elapsed -> up
    assert s.cloud_link == "up" and s.level == 0
    return end


# --- cold start (LNK-5) ---

def test_cold_start_is_never_connected_not_up():
    m = _mk()
    s = m.evaluate(3.0)
    # MUTATION: a small boot gap must NOT read as 'up' -- that silently disables the
    # L2/L3 limits for a robot that never reached the cloud.
    assert s.cloud_link != "up"
    assert s.reason == "never_connected"
    assert s.reconnected is False
    assert s.link_epoch == 0


def test_never_connected_still_escalates_to_down():
    m = _mk()
    assert m.evaluate(4.0).level == 1            # degraded
    down = m.evaluate(25.0)
    assert down.level == 2 and down.cloud_link == "down"
    assert down.reason == "never_connected"      # reason stays, not heartbeat_timeout


# --- first connect ---

def test_first_contact_backfills_immediately_then_up_after_hysteresis():
    m = _mk(stable=10.0)
    m.evaluate(10.0)                     # degraded, never connected
    m.on_cloud_rx(11.0)                  # first cloud message
    s = m.evaluate(11.1)
    # Backfill fires on first contact, NOT delayed by the hysteresis.
    assert s.reconnected is True
    # ...but cloud_link stays degraded through the observation window.
    assert s.level == 1 and s.cloud_link == "degraded"
    m.on_cloud_rx(21.5)
    s2 = m.evaluate(21.6)               # 21.6 - reconnect(11.0) = 10.6 >= stable
    assert s2.cloud_link == "up" and s2.level == 0 and s2.reason == "ok"
    assert s2.disconnected_s == 0.0
    # MUTATION: a second reconnected=True would fire a backfill every tick.
    m.on_cloud_rx(22.0)
    assert m.evaluate(22.1).reconnected is False


# --- outage after being up ---

def test_outage_escalates_and_bumps_epoch_once():
    m = _mk()
    now = _settle_up(m)
    s1 = m.evaluate(now + 6.0)          # gap >= 5 -> degraded
    assert s1.level == 1 and s1.reason == "heartbeat_timeout"
    assert s1.link_epoch == 1           # a NEW outage
    s2 = m.evaluate(now + 25.0)         # disconnected >= 20 -> down
    assert s2.level == 2 and s2.cloud_link == "down"
    # MUTATION: bumping the epoch every evaluate breaks P3's "one RTB per outage".
    assert s2.link_epoch == 1


def test_reconnect_fires_backfill_once_per_outage():
    m = _mk()
    now = _settle_up(m)
    m.evaluate(now + 6.0)               # outage (degraded)
    m.on_cloud_rx(now + 7.0)            # cloud back
    assert m.evaluate(now + 7.1).reconnected is True
    assert m.evaluate(now + 7.2).reconnected is False


# --- hysteresis (LNK-3) ---

def test_flap_during_window_does_not_reset_timer():
    m = _mk(stable=10.0)
    now = _settle_up(m)
    m.evaluate(now + 6.0)              # outage, down_since = last_rx (~now-0.4)
    disc_at_outage = m.evaluate(now + 6.0).disconnected_s
    m.on_cloud_rx(now + 6.5)          # brief reconnect (window opens)
    m.evaluate(now + 7.0)            # in window, degraded
    # flap: silence again, evaluate well past degraded_s from the brief rx
    s = m.evaluate(now + 13.0)       # gap from rx(now+6.5) = 6.5 >= 5 -> unreachable
    # MUTATION: if the flap reset down_since, disconnected_s would drop back near 0.
    assert s.disconnected_s > disc_at_outage + 5.0   # kept growing across the flap


# --- L3 / thresholds ---

def test_level3_only_when_rtb_set():
    m = _mk(rtb=30.0)
    now = _settle_up(m)
    assert m.evaluate(now + 35.0).level == 3        # disconnected >= rtb


def test_l3_disabled_stays_l2_when_rtb_none():
    m = _mk(rtb=None)
    now = _settle_up(m)
    # far past any rtb -- with rtb None, L3 never engages (fail-safe: no auto-RTB).
    assert m.evaluate(now + 5000.0).level == 2


def test_to_next_level_countdown_at_l1():
    m = _mk()
    now = _settle_up(m)
    s = m.evaluate(now + 6.0)          # degraded; to L2 = down_s(20) - disconnected
    assert s.to_next_level_s is not None
    assert abs(s.to_next_level_s - (20.0 - s.disconnected_s)) < 0.01


def test_thresholds_validation():
    with pytest.raises(ValueError):
        LinkThresholds(0, 20, None, 10)          # degraded_s must be > 0
    with pytest.raises(ValueError):
        LinkThresholds(5, 5, None, 10)           # down_s must exceed degraded_s
    with pytest.raises(ValueError):
        LinkThresholds(5, 20, 20, 10)            # rtb_s must exceed down_s
