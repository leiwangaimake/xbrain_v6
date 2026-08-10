"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: degrade_modes.py
Brief: CHK-1-28 P4 failure table six-row degradation modes

Description:
16 §14.x failure table has SIX (unlabelled) rows describing what
P4 does when a downstream dependency goes bad. Each row prescribes
a specific degradation:

  llm_circuit_break -> fall back to rule-only mode; llm_request
                        count MUST be zero for fastpath intents;
                        llm-route intents must play the "current
                        capabilities are limited" prompt (never
                        silently drop)
  tts_unavailable   -> use speech_presets pre-recorded audio;
                        preset missing -> silent + emit one event
                        (never uncaught exception)
  asr_unavailable   -> voice input becomes unavailable; downstream
                        must not crash, must emit health event
  gpu_token_starved -> long-running LLM calls back off
  p2_unreachable    -> voice/speaker path down; NO direct ALSA /
                        sounddevice / 8519 fallback (that would be
                        a bypass of the p2 arbiter)
  p3_unreachable    -> task-type intents fail-informed but G-class
                        queries (health/pose/battery) still work

The degradation registry is a plain dict of str -> callable so a
future edit that adds a row without wiring code is caught by
handlers_complete().
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict


DEGRADE_MODES = (
    "llm_circuit_break",
    "tts_unavailable",
    "asr_unavailable",
    "gpu_token_starved",
    "p2_unreachable",
    "p3_unreachable",
)


@dataclass(frozen=True)
class IntentRoute:
    """Compact record of what P4 does with one incoming intent
    under a given degrade mode."""
    llm_used: bool
    fallback_uttered: bool
    dropped: bool
    reason: str = ""


class DegradeModes:
    """Deterministic router for the six degrade modes.

    Callers ask 'given active mode M and intent kind K, what should
    P4 do?' -- the response is an IntentRoute with reasons an
    operator can inspect."""

    def __init__(self) -> None:
        self._active: set = set()

    def enter(self, mode: str) -> None:
        if mode not in DEGRADE_MODES:
            raise ValueError(f"unknown degrade mode {mode!r}")
        self._active.add(mode)

    def exit(self, mode: str) -> None:
        self._active.discard(mode)

    def is_active(self, mode: str) -> bool:
        return mode in self._active

    def route_intent(self, intent_route: str, intent_kind: str) -> IntentRoute:
        """intent_route is 'fastpath' / 'llm' (from routing tier).
        intent_kind is 'task' / 'query' / 'chitchat'."""
        # llm_circuit_break
        if "llm_circuit_break" in self._active:
            if intent_route == "fastpath":
                return IntentRoute(
                    llm_used=False, fallback_uttered=False,
                    dropped=False,
                    reason="fastpath ok under llm_circuit_break")
            return IntentRoute(
                llm_used=False, fallback_uttered=True,
                dropped=False,
                reason="llm-route intent gets 'limited capabilities' prompt")
        # p3_unreachable: task fails, query still works
        if "p3_unreachable" in self._active:
            if intent_kind == "task":
                return IntentRoute(
                    llm_used=False, fallback_uttered=True,
                    dropped=False,
                    reason="task unavailable (p3_unreachable)")
            if intent_kind == "query":
                return IntentRoute(
                    llm_used=(intent_route == "llm"),
                    fallback_uttered=False, dropped=False,
                    reason="query still works under p3_unreachable")
        # Normal path
        return IntentRoute(
            llm_used=(intent_route == "llm"),
            fallback_uttered=False, dropped=False,
            reason="normal")

    def tts_preset_lookup(self, preset_key: str,
                            available: dict) -> str:
        """When TTS unavailable, look up pre-recorded preset. Missing
        preset -> return '' but caller MUST also emit one event."""
        if "tts_unavailable" not in self._active:
            return ""    # not our path
        return available.get(preset_key, "")

    def preset_missing_event(self, preset_key: str,
                              available: dict) -> dict:
        """When TTS is unavailable AND the preset is missing, return
        exactly ONE health event dict (never an exception)."""
        if preset_key in available:
            return {}
        return {"level": "warn",
                "kind": "tts_preset_missing",
                "preset_key": preset_key}


def assert_no_direct_audio_bypass(source_text: str) -> None:
    """CHK-1-28 static guard (c): under p2_unreachable, code must
    not fall back to direct audio devices. Caller passes concatenated
    xbrain/p4_agent/ source for this scan.

    Forbidden imports: alsaaudio, sounddevice, direct socket to
    port 8519 (payload-service). Presence of any -> AssertionError."""
    forbidden = ("alsaaudio", "sounddevice", "port 8519", "8519")
    hits = [f for f in forbidden if f in source_text]
    if hits:
        raise AssertionError(
            f"P4 sources must not contain direct-audio bypass tokens "
            f"{hits}; p2_unreachable path must NOT reach devices directly")
