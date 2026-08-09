"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_gate.py
Brief: audio tests -- gate

Description:
BIZ-P2-4 -- AsrGate reason chain + gate_seq + variants.
"""


import pytest

from xbrain.p2_core.audio.gate import (
    AsrGateMessage, GateInputs, GatePublisher, evaluate_reason,
)


pytestmark = pytest.mark.no_device


def _default(**over):
    d = dict(
        hes_asserted=False, mic_device_fault=False,
        mic_not_configured=False, b_mode_active=False,
        speaker_holder_present=False, in_tail_hold=False,
        reopen_eta_ms=None,
    )
    d.update(over)
    return GateInputs(**d)


# --- Reason priority chain ------------------------------------------

def test_no_input_hits_report_unknown_mic_open():
    assert evaluate_reason(_default()) == "unknown"


@pytest.mark.parametrize("field,expected", [
    ("hes_asserted",           "hes"),
    ("mic_device_fault",       "device_fault"),
    ("mic_not_configured",     "not_configured"),
    ("b_mode_active",          "b_mode"),
    ("speaker_holder_present", "speaker_active"),
    ("in_tail_hold",           "tail_hold"),
])
def test_single_input_maps_to_expected_reason(field, expected):
    inp = _default(**{field: True})
    assert evaluate_reason(inp) == expected


# --- Priority: device_fault beats speaker_active --------------------

def test_priority_device_fault_beats_speaker_active():
    """VARIANT (spec P2-4): 'mic=fail and speaker active MUST report
    device_fault'. If the priority were reversed the fault would be
    masked."""
    inp = _default(mic_device_fault=True, speaker_holder_present=True)
    assert evaluate_reason(inp) == "device_fault"


def test_priority_hes_beats_everything_else():
    inp = _default(
        hes_asserted=True,
        mic_device_fault=True,
        speaker_holder_present=True,
        in_tail_hold=True,
    )
    assert evaluate_reason(inp) == "hes"


# --- reopen_eta_ms is REQUIRED only for two reasons ----------------

def test_reopen_eta_present_for_speaker_active():
    pub = GatePublisher()
    m = pub.compose(_default(speaker_holder_present=True,
                              reopen_eta_ms=1500))
    assert m.reason == "speaker_active"
    assert m.reopen_eta_ms == 1500


def test_reopen_eta_present_for_tail_hold():
    pub = GatePublisher()
    m = pub.compose(_default(in_tail_hold=True, reopen_eta_ms=300))
    assert m.reason == "tail_hold"
    assert m.reopen_eta_ms == 300


def test_reopen_eta_forced_none_for_other_reasons():
    """VARIANT (spec): the other 5 reasons must OMIT reopen_eta_ms.
    If a caller provides a value it MUST be dropped -- filling it
    (as null or a stale number) is a schema violation."""
    pub = GatePublisher()
    for over in [
        {"mic_device_fault": True},
        {"hes_asserted": True},
        {"mic_not_configured": True},
        {"b_mode_active": True},
    ]:
        inp = _default(reopen_eta_ms=9999, **over)
        m = pub.compose(inp)
        assert m.reopen_eta_ms is None, \
            "reason=%s must not carry reopen_eta_ms; got %s" % (
                m.reason, m.reopen_eta_ms)


# --- gate_seq -------------------------------------------------------

def test_gate_seq_starts_at_zero_and_first_compose_bumps():
    """GS-3: seq starts at 0 per process. First state -> bump to 1."""
    pub = GatePublisher()
    assert pub.gate_seq == 0
    pub.compose(_default())
    assert pub.gate_seq == 1


def test_gate_seq_bumps_on_state_change():
    pub = GatePublisher()
    pub.compose(_default())                             # unknown
    assert pub.gate_seq == 1
    pub.compose(_default(speaker_holder_present=True,
                          reopen_eta_ms=100))
    assert pub.gate_seq == 2


def test_gate_seq_does_NOT_bump_on_repeated_same_state():
    """VARIANT (spec GS-1 variant): if heartbeat also bumped gate_seq,
    the seq would be useless as a change indicator."""
    pub = GatePublisher()
    pub.compose(_default(speaker_holder_present=True, reopen_eta_ms=100))
    before = pub.gate_seq
    pub.compose(_default(speaker_holder_present=True, reopen_eta_ms=200))
    # reason unchanged (still speaker_active) -> no bump.
    # (reopen_eta_ms updating is not a state change per spec.)
    assert pub.gate_seq == before


def test_heartbeat_does_not_bump_gate_seq():
    """GS-1: 1 Hz heartbeat re-publishes the same seq. If heartbeat
    bumped, the downstream 'seq changed = state changed' invariant
    breaks and every hb-tick looks like a real event."""
    pub = GatePublisher()
    pub.compose(_default())   # seq=1
    before = pub.gate_seq
    for _ in range(5):
        m = pub.heartbeat()
        assert m.gate_seq == before


def test_heartbeat_without_prior_compose_produces_safe_default():
    """Fresh publisher heartbeat: default to closed mic + not_configured.
    Prevents publishing an ambiguous state at process start."""
    pub = GatePublisher()
    m = pub.heartbeat()
    assert m.mic_open is False
    assert m.reason == "not_configured"
    assert m.gate_seq >= 1
