"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: gate_observer.py
Brief: GWY-P4-02b -- P4-side half-duplex observation on rt/audio/gate

Description:
P4 subscribes rt/audio/gate to know when its mic is closed. Spec (GWY-P4-02b):

  * P4 MUST NOT open any audio device -- static grep guard elsewhere
  * P4 receives ALREADY-16k PCM via rt/audio/mic
  * Half-duplex EXECUTOR is P2; P4 is only an OBSERVER
  * *** mic_open == False must be tested with:
      mic_open is False and reason in {speaker_active, tail_hold}
    NOT with `== 'muted'`  (no such value; that spelling is a
    perma-false test that fires never)

* 1 s heartbeat gap fail-safe: if rt/audio/gate has not been
re-published for > 1 s, P4 assumes CLOSED (defensive; PA-3).

* SpeakRequest.max_duration_ms is REQUIRED (E_SCHEMA if missing);
est_duration_ms defaults to 4.0 s/sentence * sentences.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# 11 S8.9.2: reasons that indicate an active close (mic can reopen
# once the reason is cleared).
_ACTIVE_CLOSE_REASONS = frozenset({"speaker_active", "tail_hold"})


@dataclass(frozen=True)
class GateSample:
    """One snapshot of rt/audio/gate as P4 sees it."""
    mic_open: bool
    reason: str


def is_mic_closed_by_speaker(sample: GateSample) -> bool:
    """*** The correct check per GWY-P4-02b judgeria #3:
        mic_open is False AND reason in {speaker_active, tail_hold}

    The wrong (banned) form is `sample.reason == 'muted'` -- there
    is no 'muted' value in the 7-reason set, so that comparison is
    perma-false and never fires."""
    return (sample.mic_open is False
            and sample.reason in _ACTIVE_CLOSE_REASONS)


@dataclass
class GateHeartbeatWatch:
    """PA-3: if the gate publisher goes silent > 1 s the observer
    assumes mic is CLOSED (fail-safe direction: refuse to send
    audio to ASR while state is unknown)."""
    last_seen_millis: int = 0
    max_gap_millis: int = 1000

    def note_publish(self, now_mono_ms: int) -> None:
        self.last_seen_millis = now_mono_ms

    def assume_closed(self, now_mono_ms: int) -> bool:
        """True iff no publish for > max_gap_ms."""
        if self.last_seen_millis == 0:
            # never seen anything; assume closed (never seen != OK)
            return True
        return (now_mono_ms - self.last_seen_millis) > self.max_gap_millis


class SchemaError(RuntimeError):
    """SpeakRequest missing max_duration_ms."""


@dataclass(frozen=True)
class SpeakRequest:
    """A speak intent that P4 hands off to P2 (via cmd/audio/speak).

    max_duration_ms is REQUIRED (spec judgeria #4). Absence would let
    L1b get stuck forever if the gate message is lost -- so this
    field is required at construction, no default.
    """
    text: str
    max_duration_ms: int
    est_duration_millis: Optional[int] = None

    def __post_init__(self) -> None:
        if self.max_duration_ms <= 0:
            raise SchemaError(
                "SpeakRequest.max_duration_ms required and > 0; got %r"
                % self.max_duration_ms)


def default_est_duration_ms(text: str,
                            sentence_millis: float = 4000.0) -> int:
    """When caller does not supply est_duration_ms, derive it as
    sentence_count * sentence_ms. Sentence count is the number of
    CJK / ASCII sentence terminators; minimum 1."""
    import re
    n = len(re.findall("[\u3002\uff1f\uff01?!]|(?:\\.(?=\\s|$))", text))
    n = max(1, n)
    return int(n * sentence_millis)
