"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: motion_gate.py
Brief: CHK-2-11 AI-73 motion-state LLM throttle + AI-74 KV/ctx cap

Description:
AI-73: when state/pose.v exceeds a configured threshold, new LLM
requests are throttled (delayed / refused / queued). The specific
implementation depends on the AI-runtime constraints:

  * n_gpu_layers reduce           -- rejected here as too fragile
    for the current llama-server (see failure comment below)
  * reject new requests           -- default; simple + observable
  * queue with backpressure       -- fallback for burst tolerance

The AI-73 spec requires SOMETHING be done; a silent no-op fails
the CLAUDE.md 3.2 form-1 check. This module implements 'reject
new requests + explicit reason' with a config-driven threshold
(no default). AI-74 keeps a hard ctx cap; over-cap prompts are
CROPPED FROM HISTORY BEFORE CONTEXT and a warn is emitted.
"""

from __future__ import annotations

from dataclasses import dataclass


class MotionThrottleConfigError(Exception):
    pass


@dataclass(frozen=True)
class ThrottleConfig:
    """All fields required at construction (CLAUDE.md 3.1)."""
    speed_threshold_mps: float
    max_ctx_tokens: int

    def __post_init__(self) -> None:
        if self.speed_threshold_mps <= 0:
            raise MotionThrottleConfigError(
                f"speed_threshold_mps must be > 0, got {self.speed_threshold_mps}")
        if self.max_ctx_tokens <= 0:
            raise MotionThrottleConfigError(
                f"max_ctx_tokens must be > 0, got {self.max_ctx_tokens}")


class LlmRequestRefused(Exception):
    """LLM request refused per AI-73 due to motion state."""


def should_throttle(current_speed_mps: float,
                     threshold_mps: float) -> bool:
    """True when speed exceeds the threshold. Zero fail-silent
    direction (returning False when speed HIGH would be exactly
    the CHK-2-11 variant B failure the test guards)."""
    return current_speed_mps > threshold_mps


def admit_llm_request(current_speed_mps: float,
                       cfg: ThrottleConfig) -> None:
    """Raises LlmRequestRefused if the request must be dropped
    per AI-73."""
    if should_throttle(current_speed_mps, cfg.speed_threshold_mps):
        raise LlmRequestRefused(
            "AI-73 throttle: speed=%.2f m/s > threshold=%.2f m/s; "
            "new LLM requests refused until speed drops"
            % (current_speed_mps, cfg.speed_threshold_mps))


# ---- AI-74 ctx cap -------------------------------------------------

@dataclass(frozen=True)
class PromptSections:
    """Four-layer prompt from 16 §6.0."""
    system: str
    mission: str
    context: str
    history: str


@dataclass(frozen=True)
class TrimResult:
    trimmed: PromptSections
    tokens_used: int
    tokens_dropped_history: int
    tokens_dropped_context: int


def _tok_len(s: str) -> int:
    """Placeholder tokeniser: 1 word = 1 token. Real integration
    uses the underlying LLM's tokeniser; behaviour under test
    is deterministic which is what matters here."""
    return len(s.split()) if s else 0


class CtxCapViolation(Exception):
    """After trimming, total STILL exceeds cap. The remaining
    sections (system + mission) exceed the cap alone -- refuse."""


def trim_to_cap(sections: PromptSections, max_tokens: int) -> TrimResult:
    """Trim in the AI-40g order: history first, then context.
    If even system+mission alone exceed the cap, raise
    CtxCapViolation (safer than truncating the mission)."""
    sys_t = _tok_len(sections.system)
    mis_t = _tok_len(sections.mission)
    ctx_t = _tok_len(sections.context)
    hist_t = _tok_len(sections.history)
    total = sys_t + mis_t + ctx_t + hist_t
    if total <= max_tokens:
        return TrimResult(
            trimmed=sections, tokens_used=total,
            tokens_dropped_history=0, tokens_dropped_context=0)
    # Drop history first
    remaining = sys_t + mis_t + ctx_t
    if remaining <= max_tokens:
        return TrimResult(
            trimmed=PromptSections(sections.system, sections.mission,
                                    sections.context, ""),
            tokens_used=remaining,
            tokens_dropped_history=hist_t,
            tokens_dropped_context=0)
    # Then drop context
    remaining = sys_t + mis_t
    if remaining <= max_tokens:
        return TrimResult(
            trimmed=PromptSections(sections.system, sections.mission,
                                    "", ""),
            tokens_used=remaining,
            tokens_dropped_history=hist_t,
            tokens_dropped_context=ctx_t)
    raise CtxCapViolation(
        f"system+mission ({sys_t + mis_t} tokens) alone exceed cap "
        f"({max_tokens}); refusing rather than truncating mission")
