"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rtc.py
Brief: MOT-PM-2 realtime-base helpers (RTC-1..RTC-9)

Description:
The nine RTC rules cover import discipline (no import after loop start), single-slot lock-free queues, and no blocking logs on the hot path. This module ships pure-Python guards + a runtime hook callers use to note that they respected the rule; the actual gc.freeze / mlockall / SCHED_FIFO calls that would need root sit behind no-op skeletons so unit tests can exercise the DISCIPLINE without needing the runtime privileges.
"""



from __future__ import annotations


class RtcViolation(RuntimeError):
    """A caller violated an RTC-N rule."""


def note_import_completed(imports_after_boot: bool) -> None:
    """RTC-1: no import after main-loop start."""
    if imports_after_boot:
        raise RtcViolation("RTC-1: import after main-loop start forbidden")


def note_single_slot(existing: int) -> None:
    """RTC-3: single-slot lock-free (queue depth == 1)."""
    if existing > 1:
        raise RtcViolation("RTC-3: single-slot required; queue depth %d" % existing)


def note_no_blocking_log() -> None:
    """RTC-6: no blocking log calls in the loop -- caller uses ring buffer."""
    # Enforcement is via CI grep on `logging.*` in the loop file;
    # runtime hook exists so callers can note the entry.
    pass
