"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_sequencer.py
Brief: BIZ-P2-16 D-mode sequencer -- mode/deter/lights three steps + DT-1..DT-7

Description:
*** Brief 由占位串改写(2026-08-23). 原值是按路径自动生成的
"d_mode tests -- sequencer" -- 既没说清本文件测什么, 也无法据以索引任务号, 于是 P2 是唯一
无法自动提取证据映射的子系统(CLAUDE.md 2.5 要求 Brief 一行说清).
BIZ-P2-16 -- D-mode sequencer tests.
"""


import pytest

from xbrain.p2_core.d_mode.sequencer import (
    DModeConfig, DModeState, DModeStep,
    check_cycle_boundary, next_action_after_response,
)


pytestmark = pytest.mark.no_device


def _cfg(**over):
    d = dict(
        voice="male", redblue_mode=1, siren_level=0.45,
        tts_reps=3, speech_max_repeats=10, loop_s=20.3,
        deter_texts_supported=False,
        speech_sequence=["warn_01", "warn_02"],
    )
    d.update(over)
    return DModeConfig(**d)


def test_sequence_mode_deter_lights_running_progression():
    """Happy path: /mode -> /deter -> /lights -> running."""
    st = DModeState(step=DModeStep.MODE_POST)
    cfg = _cfg()

    n = next_action_after_response(st, 200, now_mono_ms=0, cfg=cfg)
    assert n == "post_deter"
    assert st.step == DModeStep.DETER_POST

    n = next_action_after_response(st, 200, now_mono_ms=100, cfg=cfg)
    assert n == "post_lights"
    assert st.step == DModeStep.LIGHTS_POST

    n = next_action_after_response(st, 200, now_mono_ms=200, cfg=cfg)
    assert n == "enter_running"
    assert st.step == DModeStep.RUNNING
    assert st.started_mono_ms == 200


def test_non_2xx_aborts_to_idle():
    st = DModeState(step=DModeStep.DETER_POST)
    n = next_action_after_response(st, 500, now_mono_ms=0, cfg=_cfg())
    assert n == "abort_to_idle"


def test_dt7_fallback_on_422_when_texts_supported_and_not_yet_fallback():
    """DT-7c: /deter returns 422 with deter_texts_supported=true ->
    auto-retry once without texts[]."""
    st = DModeState(step=DModeStep.DETER_POST)
    cfg = _cfg(deter_texts_supported=True)
    n = next_action_after_response(st, 422, now_mono_ms=0, cfg=cfg)
    assert n == "retry_deter_no_texts"
    assert st.texts_fallback_fired is True


def test_dt7_no_fallback_when_texts_not_supported():
    """deter_texts_supported=false -> 422 is a normal abort."""
    st = DModeState(step=DModeStep.DETER_POST)
    n = next_action_after_response(st, 422, now_mono_ms=0, cfg=_cfg())
    assert n == "abort_to_idle"


def test_cycle_boundary_stops_speech_after_max_repeats():
    """DT-5: after speech_max_repeats cycles, keep siren+strobe but stop
    speech."""
    cfg = _cfg(speech_max_repeats=2, loop_s=10.0)
    st = DModeState(step=DModeStep.RUNNING, started_mono_ms=0)
    # After 1 cycle (10s): still running.
    assert check_cycle_boundary(st, 10_500, cfg) is False
    assert st.step == DModeStep.RUNNING
    # After 2 cycles (20s): hit max_repeats, SPEECH_STOPPED.
    assert check_cycle_boundary(st, 20_500, cfg) is True
    assert st.step == DModeStep.SPEECH_STOPPED
