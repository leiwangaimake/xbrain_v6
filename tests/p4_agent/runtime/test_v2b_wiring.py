"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_v2b_wiring.py
Brief: V-2B tests -- VAD segmentation + naive classifier + intent dispatch

Description:
Pure-function coverage for V-2B. Live ASR HTTP + Zenoh flow is
exercised by V-3 smoke test.
"""

from __future__ import annotations

import pytest

from xbrain.p4_agent.runtime.intent_dispatch import (
    CMD_AUDIO_SPEAK, CMD_MOTION_INTENT, CMD_PAYLOAD, CMD_PTZ, CMD_TASK,
    INTENT_TO_KEY, UnknownIntentDispatch,
    build_payload, choose_key, dispatch,
)
from xbrain.p4_agent.runtime.turn_loop import (
    MIC_TOPIC, naive_classify,
)
from xbrain.p4_agent.runtime.vad import (
    VadConfig, VadConfigError, VadState, VadState_,
    feed_frame, frame_energy,
)


pytestmark = pytest.mark.no_device


# -- VAD --

_VAD = VadConfig(
    energy_threshold=100,
    tail_silence_ms=60,
    min_utterance_ms=40,
    frame_ms=20,
)


def _silent_frame():
    return [0] * 320


def _voiced_frame(level=500):
    return [level] * 320


def test_vad_config_zero_field_refused():
    for bad in ("energy_threshold", "tail_silence_ms",
                  "min_utterance_ms", "frame_ms"):
        kwargs = dict(energy_threshold=100, tail_silence_ms=60,
                        min_utterance_ms=40, frame_ms=20)
        kwargs[bad] = 0
        with pytest.raises(VadConfigError, match=bad):
            VadConfig(**kwargs)


def test_vad_min_utterance_below_frame_refused():
    with pytest.raises(VadConfigError, match="min_utterance_ms"):
        VadConfig(energy_threshold=100, tail_silence_ms=60,
                    min_utterance_ms=10, frame_ms=20)


def test_frame_energy_zero_for_empty():
    assert frame_energy([]) == 0


def test_frame_energy_positive_for_voice():
    assert frame_energy([500, -500, 500, -500]) == 500


def test_vad_idle_stays_on_silence():
    state = VadState_()
    r = feed_frame(state, _silent_frame(), _VAD)
    assert r is None and state.state == VadState.IDLE


def test_vad_utterance_opens_on_voice():
    state = VadState_()
    feed_frame(state, _voiced_frame(), _VAD)
    assert state.state == VadState.SPEAKING


def test_vad_utterance_closes_after_tail_silence():
    """3 voice frames (60 ms) + 3 silence frames (60 ms, >= tail
    threshold) -> utterance closes."""
    state = VadState_()
    for _ in range(3):
        feed_frame(state, _voiced_frame(), _VAD)
    # Two silent frames (40 ms) -- not enough
    r = feed_frame(state, _silent_frame(), _VAD)
    assert r is None
    r = feed_frame(state, _silent_frame(), _VAD)
    assert r is None
    # Third silent frame (60 ms tail) -- closes
    r = feed_frame(state, _silent_frame(), _VAD)
    assert r is not None
    # utter should be 3 voiced + 3 tail frames = 6 * 320 = 1920 samples
    assert len(r) == 6 * 320


def test_vad_short_utterance_below_min_dropped():
    """1 frame voice (20 ms) + tail silence -> below 40ms min ->
    close but returns None (dropped)."""
    state = VadState_()
    feed_frame(state, _voiced_frame(), _VAD)
    for _ in range(3):
        r = feed_frame(state, _silent_frame(), _VAD)
    # closed but dropped
    assert r is None
    assert state.state == VadState.IDLE


def test_vad_resumes_speaking_from_tail():
    state = VadState_()
    feed_frame(state, _voiced_frame(), _VAD)
    feed_frame(state, _voiced_frame(), _VAD)
    feed_frame(state, _silent_frame(), _VAD)  # -> TAIL
    assert state.state == VadState.TAIL
    feed_frame(state, _voiced_frame(), _VAD)
    assert state.state == VadState.SPEAKING


# -- naive classifier --

def test_classifier_patrol():
    assert naive_classify("巡逻") == "B01"


def test_classifier_start_patrol_longer_match_wins():
    """'开始巡逻' beats '巡逻' via longest-first rule."""
    assert naive_classify("开始巡逻") == "B01"


def test_classifier_estop():
    assert naive_classify("急停") == "B09"
    assert naive_classify("停止") == "B09"


def test_classifier_unknown_utterance():
    assert naive_classify("今天天气怎么样") == "D_UNKNOWN"


def test_classifier_empty_text():
    assert naive_classify("") == "D_UNKNOWN"


def test_classifier_none_treated_as_empty():
    assert naive_classify(None) == "D_UNKNOWN"


# -- intent dispatch --

def test_dispatch_patrol_goes_to_task():
    r = dispatch("B01", "巡逻")
    assert r.key == CMD_TASK
    assert r.payload["intent_id"] == "B01"
    assert r.payload["text"] == "巡逻"


def test_dispatch_d07_strobe_off_goes_to_payload():
    # V-2B originally mislabeled D07 as 'greeting' because naive_classify's
    # demo map had 你好->D07 (a wrong mapping). D07 is really strobe_off
    # (18 S6 D-class: red-blue warning lamp OFF), which routes to
    # cmd/payload (14 S4 P2 payload domain), NOT cmd/audio/speak. Corrected
    # 2026-08-11 to reflect the real 18 command-set identity.
    r = dispatch("D07", "关爆闪")
    assert r.key == CMD_PAYLOAD


def test_dispatch_speak_stop_overrides_to_task():
    """D12 speak_stop is TASK-family (INTENT_TO_KEY override)."""
    assert INTENT_TO_KEY.get("D12") == CMD_TASK


def test_dispatch_ad_hoc_motion_a_series():
    r = dispatch("A05", "站起来")
    assert r.key == CMD_MOTION_INTENT


def test_dispatch_ptz_e_series():
    r = dispatch("E01", "转到停机坪")
    assert r.key == CMD_PTZ


def test_dispatch_payload_r_series():
    r = dispatch("R05", "打开警笛")
    assert r.key == CMD_PAYLOAD


def test_dispatch_unknown_prefix_raises():
    with pytest.raises(UnknownIntentDispatch, match="prefix"):
        dispatch("Z99", "??")


def test_dispatch_empty_intent_raises():
    with pytest.raises(UnknownIntentDispatch, match="empty"):
        dispatch("", "??")


def test_choose_key_uses_specific_override_before_prefix():
    """B09 estop specifically maps to CMD_TASK (INTENT_TO_KEY)."""
    assert choose_key("B09") == CMD_TASK
    # B01 uses prefix rule
    assert choose_key("B01") == CMD_TASK


def test_build_payload_shape():
    p = build_payload("B01", "巡逻")
    for k in ("schema", "intent_id", "text", "mono_ms"):
        assert k in p


def test_build_payload_extra_merged():
    p = build_payload("B01", "巡逻", extra={"priority": 30})
    assert p["priority"] == 30


def test_mic_topic_matches_spec():
    assert MIC_TOPIC == "rt/audio/mic"
