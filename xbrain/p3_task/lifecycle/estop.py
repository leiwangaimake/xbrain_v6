"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: estop.py
Brief: BIZ-P3-20 P3 lifecycle S10 + estop ES-1..3 + S11.3 no auto-resume

Description:
15 S11.1 emergency stop handling:

  ES-1  freeze scheduling immediately (no new tasks dispatched)
  ES-2  suspend running task (kind='estop')
  ES-3  wait for explicit unfreeze signal from p2 (do NOT auto-resume
        after some timeout)

15 S11.3 is emphatic that p3 does NOT auto-resume from an estop
condition; that always waits for a human. This is CLAUDE.md 3.6
territory: no toggle exists to bypass it.

*** What "waits for a human" means, verbatim (15 S11.1 step 3, 终审 F5):
the unfreeze IS a human-initiated cmd/task{submit|resume}; that command is
itself the unfreeze signal and is then executed immediately. It is NOT a
separate p2 signal. An auto-injected task (return_home / charge, source
auto|charge) must NOT unfreeze -- that would defeat the "wait for a human"
intent, resuming patrol the moment the battery tops up mid-estop. So the
authorized unfreeze sources are the human channels only.

99 U111 (2026-11-04, live test): two changes.
  * An operator's PAUSE freezes scheduling the same way (reason
    "operator_pause", 15 S11.4). Measured: "暂缓任务" suspended the running
    task and the scheduler started the next queued one in the same second,
    so the robot never paused. The same human submit / resume lifts it.
  * The human test now reads the TASK source the frame's channel maps to
    (task_row.task_source_for_channel), not the raw channel. The raw test
    only knew cloud / wecom / local, so a resume from the microphone
    (source "voice") or the HMI ("hmi") could never lift a freeze.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

#: 15 S9.5 human task sources -- the operator channels. auto|charge are the
#: system's own (return_home / charge) and deliberately CANNOT unfreeze.
HUMAN_SOURCES = frozenset({"cloud", "wecom", "local"})

#: Sources allowed to unfreeze: the human channels plus the operator console.
#: NOT auto|charge; NOT an unknown source.
UNFREEZE_SOURCES = HUMAN_SOURCES | {"p2_operator"}


@dataclass
class EstopController:
    """Track the freeze state. All transitions are explicit;
    there is no time-based unfreeze path."""
    frozen: bool = False
    freeze_reason: str = ""

    def freeze(self, reason: str) -> None:
        """ES-1: idempotent; keep the first reason."""
        if self.frozen:
            return
        self.frozen = True
        self.freeze_reason = reason

    def unfreeze(self, source: str) -> None:
        """ES-3: only a human-authorized source may unfreeze (15 S11.1 F5).

        source must be a human channel (cloud|wecom|local) or the operator
        console (p2_operator); an auto|charge system source or an unknown one
        raises. This is the gate that keeps an auto-injected return_home from
        silently lifting an emergency stop.
        """
        if source not in UNFREEZE_SOURCES:
            raise PermissionError(
                f"unfreeze source {source!r} not authorized")
        self.frozen = False
        self.freeze_reason = ""

    def scheduling_permitted(self) -> bool:
        return not self.frozen


def is_human_resume_command(payload: Any) -> bool:
    """ES-3 predicate: is this a human-initiated submit/resume?

    15 S11.1 F5: a human cmd/task{submit|resume} is the unfreeze signal. NO an
    auto|charge task does not qualify -- an auto-injected return_home arriving
    during an estop must not lift the freeze. Checks the S7.2 action AND the
    15 S9.5 source; both must be human, or it is not an unfreeze.
    """
    if not isinstance(payload, dict):
        return False
    return (payload.get("action") in ("submit", "resume")
            and human_source(payload) is not None)


def human_source(payload: Any) -> Any:
    """The 15 S9.5 task source of a human cmd/task frame, or None.

    The frame carries the CHANNEL (voice / text / hmi / cloud / wecom); the
    unfreeze set is written in task sources. Mapped through the one table the
    task rows use, so the two can never disagree about who is a human.
    """
    from xbrain.p3_task.ingest.task_row import task_source_for_channel
    if not isinstance(payload, dict):
        return None
    source = task_source_for_channel(payload.get("source"))
    return source if source in HUMAN_SOURCES else None


def is_human_pause_command(payload: Any) -> bool:
    """99 U111: is this an operator's pause (15 S11.4 operator hold)?"""
    return (isinstance(payload, dict) and payload.get("action") == "pause"
            and human_source(payload) is not None)
