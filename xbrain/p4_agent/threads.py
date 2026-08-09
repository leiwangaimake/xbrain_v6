"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: threads.py
Brief: GWY-P4-00 -- P4 process skeleton (4 threads + P-1/P-2 realtime)

Description:
16 S2 defines P4's 4 threads:
  main    intent routing / prompt assembly / session
  asr_rx  receive ASR results, run 3-layer post-processing
  gw      AI gateway: request queue, GPU token, timeout circuit-break
  tx      publish task/mode requests, TTS, events

Hard requirements (16 S2 P-1/P-2):
  P-1  emergency-bypass MUST fork off BEFORE ASR post-processing,
       must NOT wait for LLM
  P-2  LLM calls MUST have a timeout ceiling; never wait forever

This module owns the LOOP + ordering discipline for these threads.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional


class P1Violation(RuntimeError):
    """A code path in the 'main' thread reached the LLM invocation
    WITHOUT first checking safety-bypass. That is 16 P-1 verbatim
    banned: '急停旁路必须在 ASR 后处理之前分流, 不得等待 LLM'."""


class P2Violation(RuntimeError):
    """An LLM invocation was made without a timeout. 16 P-2: LLM
    call MUST have a timeout ceiling."""


@dataclass
class InvocationGuard:
    """Records the ordering of one utterance's processing to catch
    P-1 / P-2 violations at runtime. Not a substitute for the
    static CI grep; this is the belt to the CI's braces."""
    bypass_checked: bool = False
    asr_post_run: bool = False
    llm_called: bool = False
    llm_timeout_millis: Optional[int] = None

    def note_bypass_check(self) -> None:
        self.bypass_checked = True

    def note_asr_post(self) -> None:
        if not self.bypass_checked:
            raise P1Violation(
                "asr_post reached BEFORE bypass check -- 16 P-1 "
                "requires safety_bypass to fork off first")
        self.asr_post_run = True

    def note_llm_call(self, timeout_ms: Optional[int]) -> None:
        if timeout_ms is None or timeout_ms <= 0:
            raise P2Violation(
                "LLM invocation without a bounded timeout -- 16 P-2 "
                "requires timeout_ms > 0; got %r" % timeout_ms)
        self.llm_called = True
        self.llm_timeout_millis = timeout_ms
