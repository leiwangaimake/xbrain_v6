"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptz_wiring.py
Brief: cmd/ptz consumer -- config load + envelope -> driver routing

Description:
Tests the p2 PTZ wiring: a missing/bad credential file disables PTZ (does
not raise), and a cmd/ptz envelope reaches the driver with its slots. Device
stubbed. Mutation guards per CLAUDE.md 3.3.
"""
from __future__ import annotations

import json

import pytest

import xbrain.p2_core.ptz.onvif_client as oc
from xbrain.p2_core.runtime.ptz_wiring import (
    OnvifConfig, PtzDomain, load_onvif_config,
)

pytestmark = pytest.mark.no_device


# -- config load: missing/bad file disables PTZ, never raises ------------

def test_load_missing_file_returns_none():
    assert load_onvif_config("/no/such/onvif.json") is None


def test_load_bad_json_returns_none(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{not json", encoding="utf-8")
    assert load_onvif_config(str(p)) is None


def test_load_missing_key_returns_none(tmp_path):
    p = tmp_path / "partial.json"
    p.write_text(json.dumps({"host": "h"}), encoding="utf-8")   # no user/pwd
    assert load_onvif_config(str(p)) is None


def test_load_good_file(tmp_path):
    p = tmp_path / "ok.json"
    p.write_text(json.dumps({"host": "h", "user": "u", "pwd": "p"}),
                 encoding="utf-8")
    cfg = load_onvif_config(str(p))
    assert cfg == OnvifConfig(host="h", user="u", pwd="p")


# -- envelope -> driver --------------------------------------------------

def test_envelope_reaches_driver_with_slots(monkeypatch):
    """A cmd/ptz envelope (intent_id + slots) must reach the driver with its
    slots intact. MUTATION: dropping the slots would move the head with no
    direction."""
    seen = []
    monkeypatch.setattr(oc, "get_profile_token", lambda s, *a, **k: "tok")
    monkeypatch.setattr(oc, "ptz_continuous",
                        lambda s, p, t, **kw: seen.append(("move", kw)))
    monkeypatch.setattr(oc, "ptz_stop", lambda s, p, t, **kw: None)
    monkeypatch.setattr("xbrain.p2_core.ptz.ptz_driver.time.sleep",
                        lambda s: None)

    dom = PtzDomain(OnvifConfig(host="h", user="u", pwd="p"))
    env = json.dumps({"intent_id": "E01", "direction": "left",
                      "amount": "small"}).encode("utf-8")
    dom.handle_envelope(env)
    assert seen and seen[-1][1]["pan"] < 0        # moved left
    assert dom.calls_made == 1


def test_bad_envelope_does_not_raise(monkeypatch):
    monkeypatch.setattr(oc, "get_profile_token", lambda s, *a, **k: "tok")
    dom = PtzDomain(OnvifConfig(host="h", user="u", pwd="p"))
    dom.handle_envelope(b"{not json")             # must not raise
