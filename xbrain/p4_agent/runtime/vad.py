"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: vad.py
Brief: Simple energy-VAD utterance segmenter for the voice-loop MVP

Description:
Minimal VAD for smoke-test-quality voice-loop. Real VAD (webrtcvad
or silero) lands with GWY-P4-02b's production audio_rx; this
module is deliberately dependency-free so the smoke-test can run
on any dev machine without extra wheels.

Algorithm:
  * per-frame energy = mean(abs(samples)) as int
  * frame is 'speech' iff energy > energy_threshold
  * utterance opens on first speech frame
  * utterance closes after tail_silence_ms of continuous silence
  * min_utterance_ms floor so a single random click does not
    trigger a whole ASR round-trip

All parameters are injected at construction (CLAUDE.md 3.1) --
no defaults in code. Fixture / configs supply them per deploy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class VadState(str, Enum):
    IDLE = "idle"          # no speech seen recently
    SPEAKING = "speaking"  # inside an utterance
    TAIL = "tail"          # possible end; counting silence


class VadConfigError(Exception):
    pass


@dataclass(frozen=True)
class VadConfig:
    """All fields required at construction."""
    energy_threshold: int              # sample-abs mean threshold
    tail_silence_ms: int               # silence to close utterance
    min_utterance_ms: int              # ignore utterances shorter
    frame_ms: int                      # ms per input frame

    def __post_init__(self) -> None:
        for name in ("energy_threshold", "tail_silence_ms",
                       "min_utterance_ms", "frame_ms"):
            v = getattr(self, name)
            if v <= 0:
                raise VadConfigError(
                    f"{name} must be > 0, got {v} "
                    f"(fail-silent form of no VAD)")
        if self.min_utterance_ms < self.frame_ms:
            raise VadConfigError(
                f"min_utterance_ms ({self.min_utterance_ms}) must be "
                f">= frame_ms ({self.frame_ms}); would drop every "
                f"utterance")


@dataclass
class VadState_:
    """Mutable VAD state. Kept separate from config so the same
    config can drive multiple concurrent VADs (per-source).

    speech_frames counts only frames that were ABOVE the energy
    threshold; utterance_frames counts total (including tail).
    min_utterance_ms tests against SPEECH content so a short
    utterance followed by a long silence still drops correctly."""
    state: VadState = VadState.IDLE
    tail_silence_frames: int = 0
    utterance_frames: int = 0
    speech_frames: int = 0
    accumulated_samples: List[int] = field(default_factory=list)


def frame_energy(samples: List[int]) -> int:
    """Mean absolute value of one frame's int16 samples."""
    if not samples:
        return 0
    return sum(abs(s) for s in samples) // len(samples)


def feed_frame(state: VadState_, samples: List[int],
                cfg: VadConfig) -> Optional[List[int]]:
    """Feed one 20 ms frame of samples. Returns None when no
    utterance has closed this frame, OR a concatenated int16 list
    when an utterance just closed AND it exceeds min_utterance_ms."""
    energy = frame_energy(samples)
    is_speech = energy > cfg.energy_threshold

    if state.state == VadState.IDLE:
        if is_speech:
            state.state = VadState.SPEAKING
            state.tail_silence_frames = 0
            state.utterance_frames = 1
            state.speech_frames = 1
            state.accumulated_samples = list(samples)
        return None

    if state.state == VadState.SPEAKING:
        state.accumulated_samples.extend(samples)
        state.utterance_frames += 1
        if is_speech:
            state.speech_frames += 1
            state.tail_silence_frames = 0
        else:
            state.state = VadState.TAIL
            state.tail_silence_frames = 1
        return None

    # TAIL
    state.accumulated_samples.extend(samples)
    state.utterance_frames += 1
    if is_speech:
        state.speech_frames += 1
        state.state = VadState.SPEAKING
        state.tail_silence_frames = 0
        return None
    state.tail_silence_frames += 1
    tail_ms = state.tail_silence_frames * cfg.frame_ms
    if tail_ms < cfg.tail_silence_ms:
        return None
    # Close utterance.
    utter = state.accumulated_samples
    speech_ms = state.speech_frames * cfg.frame_ms
    # Reset
    state.state = VadState.IDLE
    state.tail_silence_frames = 0
    state.utterance_frames = 0
    state.speech_frames = 0
    state.accumulated_samples = []
    if speech_ms < cfg.min_utterance_ms:
        return None
    return utter
