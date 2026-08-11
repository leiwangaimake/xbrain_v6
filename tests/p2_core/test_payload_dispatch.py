"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_payload_dispatch.py
Brief: p2 PayloadDomain routes D01/D02/D06/D07/D10/D17/D18 to the client

Description:
Tests that the payload subscriber turns an intent envelope (with its slot)
into the right payload-service call, and that the D17 level / D18 mode / D10
volume resolvers map to the correct device values. The HTTP client is
stubbed, so no device is needed. Each carries a mutation guard (CLAUDE.md
3.3).
"""
from __future__ import annotations

import pytest

import xbrain.p4_agent.ai_client.lights_client as lc
from xbrain.p2_core.runtime.payload_wiring import (
    PayloadDomain, PayloadWiringConfig,
)

pytestmark = pytest.mark.no_device


@pytest.fixture
def domain(monkeypatch):
    calls = []
    monkeypatch.setattr(lc, "set_redblue",
                        lambda **kw: calls.append(("redblue", kw)) or {"ok": True})
    monkeypatch.setattr(lc, "set_searchlight",
                        lambda **kw: calls.append(("searchlight", kw)) or {"ok": True})
    monkeypatch.setattr(lc, "set_volume",
                        lambda **kw: calls.append(("volume", kw)) or {"ok": True})
    d = PayloadDomain(PayloadWiringConfig(
        payload_base_url="http://127.0.0.1:18080", http_timeout_s=1.0))
    d._calls = calls          # expose for assertions
    return d


def _last(d):
    return d._calls[-1]


# -- D06/D07 red-blue on/off (regression) --------------------------------

def test_d06_redblue_on(domain):
    domain._dispatch({"intent_id": "D06"})
    kind, kw = _last(domain)
    assert kind == "redblue" and kw["on"] is True


def test_d07_redblue_off(domain):
    domain._dispatch({"intent_id": "D07"})
    assert _last(domain)[1]["on"] is False


# -- D18 strobe pattern (function 2) -------------------------------------

def test_d18_explicit_mode(domain):
    domain._dispatch({"intent_id": "D18", "mode": 3})
    kind, kw = _last(domain)
    assert kind == "redblue" and kw["pattern"] == 3 and kw["on"] is True


def test_d18_empty_mode_cycles(domain):
    """MUTATION guard: '换一种' (no mode) must cycle current+1, not drop.
    From last=0 the first cycle is 1, then 2."""
    domain._dispatch({"intent_id": "D18"})
    assert _last(domain)[1]["pattern"] == 1
    domain._dispatch({"intent_id": "D18"})
    assert _last(domain)[1]["pattern"] == 2


# -- D17 brightness level -> 0..30 (function 3b) -------------------------

def test_d17_absolute_levels(domain):
    domain._dispatch({"intent_id": "D17", "level": "max"})
    assert _last(domain)[1]["bright"] == 30
    domain._dispatch({"intent_id": "D17", "level": "mid"})
    assert _last(domain)[1]["bright"] == 15


def test_d17_relative_up_down_clamped(domain):
    """up/down step +/-7 from the last brightness, clamped [1,30]. Start at
    D01's on-value 30 -> down -> 23. MUTATION: ignoring the last value would
    give a fixed number."""
    domain._dispatch({"intent_id": "D01"})               # sets last=30
    domain._dispatch({"intent_id": "D17", "level": "down"})
    assert _last(domain)[1]["bright"] == 23
    domain._dispatch({"intent_id": "D17", "level": "up"})
    assert _last(domain)[1]["bright"] == 30              # 23+7=30, clamped


def test_d17_no_level_dropped(domain):
    before = domain.calls_made
    domain._dispatch({"intent_id": "D17"})               # no level slot
    assert domain.calls_made == before                   # nothing sent
    assert domain.calls_dropped >= 1


# -- D10 volume -> 0..100 (function 4) -----------------------------------

def test_d10_absolute(domain):
    domain._dispatch({"intent_id": "D10", "volume": {"abs": 100}})
    kind, kw = _last(domain)
    assert kind == "volume" and kw["volume"] == 100


def test_d10_relative_clamped(domain):
    """rel applies to last volume (init 50), clamped [0,100]. +20 -> 70."""
    domain._dispatch({"intent_id": "D10", "volume": {"rel": 20}})
    assert _last(domain)[1]["volume"] == 70
    domain._dispatch({"intent_id": "D10", "volume": {"rel": -100}})
    assert _last(domain)[1]["volume"] == 0               # clamped floor


def test_d10_no_slot_dropped(domain):
    before = domain.calls_made
    domain._dispatch({"intent_id": "D10"})
    assert domain.calls_made == before
