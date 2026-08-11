"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_onvif_client.py
Brief: ONVIF WS-Security digest + PTZ SOAP body correctness

Description:
Tests the ported ONVIF client without a device. The WS-Security digest is
the single form the camera accepts (report S4.3), so it carries a mutation
guard per CLAUDE.md 3.3: a fixed nonce+time yields an exact, verifiable
SHA1 digest.
"""
from __future__ import annotations

import base64
import hashlib

import pytest

import xbrain.p2_core.ptz.onvif_client as oc

pytestmark = pytest.mark.no_device


# -- WS-Security PasswordDigest (the auth the device is picky about) ------

def test_ws_header_digest_is_sha1_nonce_created_pwd(monkeypatch):
    """report S4.3: digest = base64(SHA1(nonce + created + pwd)), Created
    with NO milliseconds. Fix the nonce + clock and verify the exact digest.
    MUTATION: any change to the digest recipe (order, hash, ms in Created)
    breaks this equality -- and on-device would give ter:NotAuthorized."""
    import datetime as _dtmod
    fixed_nonce = b"0123456789abcdef"
    monkeypatch.setattr(oc.os, "urandom", lambda n: fixed_nonce[:n])

    # Pre-build the fixed time with the REAL datetime class before patching,
    # so _FixedDT.now does not recurse into the patched module.
    fixed = _dtmod.datetime(2026, 8, 11, 10, 20, 30,
                            tzinfo=_dtmod.timezone.utc)

    class _FixedDT:
        @staticmethod
        def now(tz=None):
            return fixed
    monkeypatch.setattr(oc.datetime, "datetime", _FixedDT)

    header = oc._ws_header("admin", "Admin123.")
    created = "2026-08-11T10:20:30Z"
    assert created in header
    assert "." not in created                      # NO milliseconds
    expected = base64.b64encode(
        hashlib.sha1(fixed_nonce + created.encode()
                     + b"Admin123.").digest()).decode()
    assert f">{expected}<" in header               # exact digest present
    assert "PasswordDigest" in header
    assert "mustUnderstand=\"1\"" in header
    assert base64.b64encode(fixed_nonce).decode() in header   # Nonce


def test_ws_header_nonce_is_fresh_each_call():
    # A replayable header would reuse the nonce; each call must differ.
    h1 = oc._ws_header("admin", "pw")
    h2 = oc._ws_header("admin", "pw")
    assert h1 != h2


# -- PTZ SOAP bodies -----------------------------------------------------

class _CapSession:
    """Session stub that records the last (path, body) instead of calling."""
    def __init__(self):
        self.calls = []
    def call(self, path, body):
        self.calls.append((path, body))
        return "<ok/>"


def test_ptz_continuous_body():
    s = _CapSession()
    oc.ptz_continuous(s, "/onvif/ptz", "media_profile1",
                      pan=-1.0, tilt=0.0, zoom=0.0)
    path, body = s.calls[-1]
    assert path == "/onvif/ptz"
    assert "<ProfileToken>media_profile1</ProfileToken>" in body
    assert 'x="-1.0" y="0.0"' in body              # pan left
    assert "ContinuousMove" in body


def test_ptz_stop_body():
    s = _CapSession()
    oc.ptz_stop(s, "/onvif/ptz", "media_profile1")
    _, body = s.calls[-1]
    assert "<Stop" in body
    assert "<PanTilt>true</PanTilt>" in body
    assert "<Zoom>true</Zoom>" in body


def test_soap_fault_detection():
    assert oc.soap_fault("<ok/>") is None
    assert oc.soap_fault(
        "<s:Fault><Text>ter:NotAuthorized</Text></s:Fault>") == "ter:NotAuthorized"
