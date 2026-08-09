"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch6.py
Brief: p4_agent tests -- batch6

Description:
GWY-P4-19 + P4-21 batch 6 tests.
"""


import pytest

from xbrain.p4_agent.failure.handlers import (
    FailureCode, HandlerResult, TtsQuotaLimiter,
    handle_estop1_arbiter_down, handle_estop2_dispatch_failed,
    handle_gate1_heartbeat_lost, handle_gate2_hes,
    handle_speak1_tts_failed,
)
from xbrain.p4_agent.registry.evolution import (
    EvolutionError, HOT_UPDATABLE_FILES,
    check_cf1_no_shared_key, check_cf3_version_compat,
    is_hot_updatable,
)


pytestmark = pytest.mark.no_device


# --- P4-19 failure handlers ---

def test_gate1_drops_asr():
    r = handle_gate1_heartbeat_lost()
    assert r.action == "drop"


def test_gate2_drops_all_voice():
    r = handle_gate2_hes()
    assert r.action == "drop"


def test_estop1_falls_to_local_wav():
    r = handle_estop1_arbiter_down()
    assert r.action == "local_wav"


def test_estop2_retries_up_to_3():
    for n in range(3):
        r = handle_estop2_dispatch_failed(retry_count=n)
        assert r.action == "retry"
    r_fail = handle_estop2_dispatch_failed(retry_count=3)
    assert r_fail.action == "fault"


def test_speak1_falls_to_local():
    r = handle_speak1_tts_failed()
    assert r.action == "local_wav"


def test_limiter_defers_when_quota_exceeded():
    lim = TtsQuotaLimiter(per_minute_cap=2)
    assert lim.try_emit(0).action == "emit"
    assert lim.try_emit(100).action == "emit"
    r = lim.try_emit(200)
    assert r.action == "defer"
    assert "LIMITER-1" in r.detail


def test_limiter_slides_60s_window():
    lim = TtsQuotaLimiter(per_minute_cap=1)
    lim.try_emit(0)
    # 61s later, bucket empty again.
    r = lim.try_emit(61_000)
    assert r.action == "emit"


# --- P4-21 evolution CF-* ---

def test_is_hot_updatable_recognizes_yaml_names():
    assert is_hot_updatable("/opt/xbrain_v6/configs/suspicion_rules.yaml")
    assert is_hot_updatable("speech_presets.yaml")
    assert not is_hot_updatable("intents.yaml")
    assert not is_hot_updatable("/opt/xbrain_v6/configs/p4_agent.yaml")


def test_cf1_no_shared_key_ok():
    check_cf1_no_shared_key({
        "asr_dict.yaml":       ["东门", "西门"],
        "query_templates.yaml": ["battery_ok"],
    })


def test_cf1_shared_key_raises():
    with pytest.raises(EvolutionError):
        check_cf1_no_shared_key({
            "asr_dict.yaml":  ["东门"],
            "chitchat.yaml":  ["东门"],
        })


def test_cf3_version_major_mismatch_raises():
    with pytest.raises(EvolutionError):
        check_cf3_version_compat(current_major=2, file_version="1.5.0")


def test_cf3_version_major_ok():
    check_cf3_version_compat(current_major=2, file_version="2.3.1")


def test_cf3_bad_version_string_raises():
    with pytest.raises(EvolutionError):
        check_cf3_version_compat(current_major=1, file_version="not.a.version")
