"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: sequencer.py
Brief: BIZ-P2-16 -- D-mode 3-step orchestration (POST /mode /deter /lights)

Description:
D-mode entry sequence (14 S7.3.2 DT-1..DT-7):

  DT-1: POST /mode  {mode: deter}   -- device switches to deter
  DT-2: POST /deter {siren_level, voice, tts_reps, redblue_mode, ...}
                                     -- start siren + speech + red/blue
  DT-3: POST /lights {mode: 0x07}   -- confirm strobe pattern
  DT-4: monitor loop_s cycles; after speech_max_repeats, keep siren
        + strobe but stop speech (DT-5)
  DT-7: deter_texts_supported = false -> use device-side single lines,
        NOT texts[] field; on 422 auto-fallback (DT-7c)

* D-mode enter/exit is ATOMIC at the mode SM level (BIZ-P2-11). This
module owns the WITHIN-D-mode step-by-step orchestration.

* Implementation note: actual HTTP calls delegate to payload_client
(BIZ-P2-2). This module holds the state (which step we're in,
current cycle count) and returns SequenceCommands the payload_io
thread executes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class DModeStep(str, Enum):
    """Where we are in the D-mode entry sequence."""
    IDLE = "idle"                  # not in D mode
    MODE_POST = "mode_post"        # posting /mode
    DETER_POST = "deter_post"      # posting /deter
    LIGHTS_POST = "lights_post"    # posting /lights
    RUNNING = "running"            # cycles running
    SPEECH_STOPPED = "speech_stopped"   # DT-5: max repeats reached


@dataclass
class DModeConfig:
    """From p2_core.yaml.d_mode."""
    voice: str                    # 'male' / 'female'
    redblue_mode: int             # 1..16
    siren_level: float            # 0.0..1.0
    tts_reps: int
    speech_max_repeats: int
    loop_s: float                 # single cycle wall length
    deter_texts_supported: bool
    speech_sequence: list         # e.g. ['warn_01', 'warn_02']


@dataclass
class DModeState:
    """Runtime state of D-mode. Reset on entry / exit."""
    step: DModeStep = DModeStep.IDLE
    cycles_completed: int = 0
    started_mono_ms: int = 0
    # DT-7: whether we've seen a 422 fallback happen already.
    texts_fallback_fired: bool = False


def next_action_after_response(
    state: DModeState,
    prev_http_status: int,
    now_mono_ms: int,
    cfg: DModeConfig,
) -> Optional[str]:
    """Given the previous step and the HTTP status the payload
    service returned, decide what the next step should be.

    Returns the next step name, or None if the sequence is complete
    (running / speech_stopped)."""
    # Fault first: anything not 2xx (except 422 on /deter with
    # deter_texts_supported=true triggering DT-7 fallback).
    if prev_http_status == 422 and state.step == DModeStep.DETER_POST \
            and cfg.deter_texts_supported \
            and not state.texts_fallback_fired:
        # DT-7c: auto-retry without texts[] once.
        state.texts_fallback_fired = True
        return "retry_deter_no_texts"

    if not (200 <= prev_http_status < 300):
        return "abort_to_idle"

    # Step-by-step progression on success.
    if state.step == DModeStep.MODE_POST:
        state.step = DModeStep.DETER_POST
        return "post_deter"
    if state.step == DModeStep.DETER_POST:
        state.step = DModeStep.LIGHTS_POST
        return "post_lights"
    if state.step == DModeStep.LIGHTS_POST:
        state.step = DModeStep.RUNNING
        state.started_mono_ms = now_mono_ms
        return "enter_running"
    return None


def check_cycle_boundary(
    state: DModeState,
    now_mono_ms: int,
    cfg: DModeConfig,
) -> bool:
    """DT-5: has speech_max_repeats been reached? Called at each loop_s
    tick from the main thread. Returns True the tick we transition to
    SPEECH_STOPPED."""
    if state.step != DModeStep.RUNNING:
        return False
    elapsed_s = (now_mono_ms - state.started_mono_ms) / 1000.0
    cycles = int(elapsed_s / cfg.loop_s)
    if cycles > state.cycles_completed:
        state.cycles_completed = cycles
    if state.cycles_completed >= cfg.speech_max_repeats:
        state.step = DModeStep.SPEECH_STOPPED
        return True
    return False
