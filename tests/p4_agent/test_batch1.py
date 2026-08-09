"""GWY-P4-00 + P4-02 + P4-02b batch 1 tests."""

import pytest

from xbrain.p4_agent.audio_rx.gate_observer import (
    GateHeartbeatWatch, GateSample, SchemaError, SpeakRequest,
    default_est_duration_ms, is_mic_closed_by_speaker,
)
from xbrain.p4_agent.gateway.gpu_token import (
    AdmissionResult, CircuitState, GpuTokenState, release, try_admit,
)
from xbrain.p4_agent.threads import (
    InvocationGuard, P1Violation, P2Violation,
)


pytestmark = pytest.mark.no_device


# --- P4-00 InvocationGuard ---

def test_p1_violation_when_asr_post_before_bypass():
    g = InvocationGuard()
    with pytest.raises(P1Violation):
        g.note_asr_post()


def test_correct_order_ok():
    g = InvocationGuard()
    g.note_bypass_check()
    g.note_asr_post()
    g.note_llm_call(timeout_ms=5000)


def test_p2_violation_when_llm_called_without_timeout():
    g = InvocationGuard()
    g.note_bypass_check()
    g.note_asr_post()
    with pytest.raises(P2Violation):
        g.note_llm_call(timeout_ms=None)
    with pytest.raises(P2Violation):
        g.note_llm_call(timeout_ms=0)


# --- P4-02 GPU token + circuit breaker ---

def test_first_admission_takes_slot():
    st = GpuTokenState()
    r = try_admit(st, now_mono_ms=0)
    assert r.admitted
    assert st.slot_taken


def test_second_concurrent_denied_busy():
    st = GpuTokenState()
    try_admit(st, now_mono_ms=0)
    r = try_admit(st, now_mono_ms=1)
    assert not r.admitted
    assert r.reason == "slot_taken"


def test_release_success_frees_slot():
    st = GpuTokenState()
    try_admit(st, 0)
    release(st, success=True, now_mono_ms=100)
    assert not st.slot_taken
    r = try_admit(st, 200)
    assert r.admitted


def test_three_consecutive_timeouts_open_circuit():
    st = GpuTokenState(timeouts_before_open=3)
    for i in range(3):
        try_admit(st, now_mono_ms=i * 1000)
        release(st, success=False, now_mono_ms=i * 1000 + 100)
    # Circuit now open.
    r = try_admit(st, now_mono_ms=3500)
    assert not r.admitted
    assert r.reason == "circuit_open"
    assert r.must_tts is True
    assert r.tts_text                    # non-empty


def test_success_between_timeouts_resets_counter():
    st = GpuTokenState(timeouts_before_open=3)
    try_admit(st, 0)
    release(st, success=False, now_mono_ms=100)   # 1 timeout
    try_admit(st, 200)
    release(st, success=True, now_mono_ms=300)    # reset
    try_admit(st, 400)
    release(st, success=False, now_mono_ms=500)   # 1 timeout again
    # Only 1 in a row; not open.
    r = try_admit(st, 600)
    assert r.admitted


def test_circuit_transitions_to_half_open_after_duration():
    st = GpuTokenState(timeouts_before_open=1, open_duration_millis=1000)
    try_admit(st, 0)
    release(st, success=False, now_mono_ms=100)   # opens circuit
    assert st.circuit_state(500) == CircuitState.OPEN
    assert st.circuit_state(1200) == CircuitState.HALF_OPEN


# --- P4-02b gate observer ---

def test_is_mic_closed_by_speaker_active():
    s = GateSample(mic_open=False, reason="speaker_active")
    assert is_mic_closed_by_speaker(s)


def test_is_mic_closed_by_tail_hold():
    s = GateSample(mic_open=False, reason="tail_hold")
    assert is_mic_closed_by_speaker(s)


def test_is_mic_closed_by_speaker_returns_false_on_hes():
    """VARIANT (spec 判据 3): the correct predicate is
    mic_open==False AND reason in {speaker_active, tail_hold} --
    NOT reason == 'muted' (which is perma-false).

    A hes-caused close should NOT count as 'closed by speaker'."""
    s = GateSample(mic_open=False, reason="hes")
    assert not is_mic_closed_by_speaker(s)


def test_muted_string_would_never_match():
    """A regression that reintroduced reason == 'muted' would fire
    NEVER because 'muted' is not in the 7-reason set. This test
    documents that 'muted' is not a valid reason."""
    s = GateSample(mic_open=False, reason="muted")
    # is_mic_closed_by_speaker checks reason in the set; 'muted'
    # is not there -> returns False even though mic_open is False.
    assert not is_mic_closed_by_speaker(s)


def test_gate_heartbeat_assumes_closed_after_gap():
    w = GateHeartbeatWatch(max_gap_millis=1000)
    # note_publish at non-zero time so 'never seen' branch (== 0) does not fire.
    w.note_publish(now_mono_ms=100)
    assert not w.assume_closed(500)
    assert w.assume_closed(1500)


def test_gate_heartbeat_assumes_closed_when_never_seen():
    w = GateHeartbeatWatch(max_gap_millis=1000)
    assert w.assume_closed(now_mono_ms=100)


# --- SpeakRequest schema ---

def test_speak_request_requires_max_duration_ms():
    with pytest.raises(SchemaError):
        SpeakRequest(text="hi", max_duration_ms=0)
    with pytest.raises(SchemaError):
        SpeakRequest(text="hi", max_duration_ms=-1)


def test_speak_request_ok_when_max_provided():
    r = SpeakRequest(text="hi", max_duration_ms=5000)
    assert r.max_duration_ms == 5000


def test_default_est_duration_counts_sentences():
    assert default_est_duration_ms("hi") == 4000
    assert default_est_duration_ms("你好。再见。") == 8000
    assert default_est_duration_ms("Q？A！") == 8000
