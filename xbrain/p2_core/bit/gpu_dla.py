"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: gpu_dla.py
Brief: CHK-2-24 deep BIT gpu + dla items (19 §9.4 ②③: age-fail, NEVER unknown)

Description:
19 §9.4 ruling (2):
  * age > threshold -> fail (NOT unknown)
19 §9.4 ruling (3):
  * never received a golden_match event -> fail (NOT unknown)

The 'unknown' state is a fail-silent form: an operator scanning
the health tree sees 'gpu = unknown' and might interpret 'not
verified yet, waiting'; the truth is 'we have no evidence, fail'.

DLA is a build-time choice:
  * dla.enabled == False AND no engine compiled to DLA -> item is
    ALWAYS 'ok' with detail == 'not_used' (nothing to check)
  * unknown for dla is also forbidden

Static guard: no PC-3 key anywhere in the codebase (19 §9.4 ①:
'no new PC-3, no new key').
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class BitItemState(str, Enum):
    OK = "ok"
    FAIL = "fail"
    # NOTE: no UNKNOWN. Callers must not silently degrade to
    # unknown; the two ways gpu/dla can be OK are (1) recent
    # golden_match event, (2) explicitly disabled DLA.


class BitAssertViolation(Exception):
    pass


DEFAULT_GPU_AGE_MAX_SEC = 3600      # 19 §9.4 (2)


@dataclass(frozen=True)
class GpuItemResult:
    state: str      # 'ok' / 'fail'
    reason: str
    age_s: Optional[float] = None


def evaluate_gpu(golden_event_age_s: Optional[float],
                  max_age_s: int = DEFAULT_GPU_AGE_MAX_SEC) -> GpuItemResult:
    """CHK-2-24 (i)/(ii)/(iii): three failure modes.
      * never received -> fail 'no_event'
      * age > max      -> fail 'stale'
      * age <= max     -> ok
    'unknown' is never returned; the docstring above lists why."""
    if golden_event_age_s is None:
        return GpuItemResult(state=BitItemState.FAIL.value,
                              reason="no_event")
    if golden_event_age_s > max_age_s:
        return GpuItemResult(state=BitItemState.FAIL.value,
                              reason="stale",
                              age_s=golden_event_age_s)
    return GpuItemResult(state=BitItemState.OK.value,
                          reason="fresh",
                          age_s=golden_event_age_s)


@dataclass(frozen=True)
class DlaItemResult:
    state: str
    detail: str


def evaluate_dla(dla_enabled: bool,
                  has_dla_engine: bool,
                  golden_event_age_s: Optional[float],
                  max_age_s: int = DEFAULT_GPU_AGE_MAX_SEC) -> DlaItemResult:
    """CHK-2-24 (iv): dla.enabled False AND no DLA engine ->
    always 'ok' with 'not_used'. Otherwise same rule as gpu."""
    if not dla_enabled and not has_dla_engine:
        return DlaItemResult(state=BitItemState.OK.value,
                              detail="not_used")
    # Real DLA path: same rules as gpu
    if golden_event_age_s is None:
        return DlaItemResult(state=BitItemState.FAIL.value,
                              detail="no_event")
    if golden_event_age_s > max_age_s:
        return DlaItemResult(state=BitItemState.FAIL.value,
                              detail="stale")
    return DlaItemResult(state=BitItemState.OK.value,
                          detail="fresh")


class Pc3Violation(Exception):
    """19 §9.4 (1) forbids PC-3."""


def assert_no_pc3_key(source_text: str) -> None:
    """Whole-tree scan: PC-3 must not appear anywhere."""
    if "PC-3" in source_text or "\"PC-3\"" in source_text:
        raise Pc3Violation("PC-3 key detected; 19 §9.4 (1) forbids it")


def assert_no_unknown_state_in_source(source_text: str) -> None:
    """CHK-2-24 (v) static guard: the strings 'unknown' /
    BitItemState.UNKNOWN must not appear in gpu/dla evaluation
    paths (the enum doesn't even define UNKNOWN)."""
    # Look for the two shapes an implementation would use to
    # 'quietly return unknown'.
    forbidden = ('"unknown"', "'unknown'", "BitItemState.UNKNOWN")
    hits = [f for f in forbidden if f in source_text]
    if hits:
        raise BitAssertViolation(
            f"gpu/dla source path contains forbidden UNKNOWN token(s): "
            f"{hits}; 19 §9.4 ② requires fail-not-unknown")
