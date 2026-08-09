"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: dispatcher.py
Brief: BIZ-P3-25 seven-type task dispatch (patrol/goto/charge/return_home/standby/teach/follow)

Description:
Once the scheduler picks a task, dispatcher maps task_type to the
right executor entry point. It does NOT execute; it looks up the
handler and asserts it exists. This decouples the state machine
from executor code and, more importantly, makes it a static error
to add a new task_type without wiring an executor.

The registry is populated at process startup; a missing type at
dispatch time raises UnknownDispatchTarget rather than degrading
silently (per CLAUDE.md 3.5 closed-set rule).
"""

from __future__ import annotations

from typing import Callable


VALID_TYPES = frozenset({
    "patrol", "goto", "charge", "return_home", "standby", "teach", "follow",
})


class UnknownDispatchTarget(Exception):
    """dispatch called with an unregistered task_type."""


class DispatcherIncomplete(Exception):
    """Not all seven types have handlers -- refuse to start."""


class Dispatcher:
    """Maps task_type -> executor callable. Fully populated at
    process startup; a missing entry at process-start time is a
    fatal error, not a runtime lookup miss."""

    def __init__(self) -> None:
        self._handlers: dict = {}

    def register(self, task_type: str, handler: Callable) -> None:
        if task_type not in VALID_TYPES:
            raise UnknownDispatchTarget(
                f"can't register unknown type {task_type!r}")
        if task_type in self._handlers:
            raise UnknownDispatchTarget(
                f"handler for {task_type!r} already registered")
        self._handlers[task_type] = handler

    def assert_complete(self) -> None:
        missing = VALID_TYPES - set(self._handlers)
        if missing:
            raise DispatcherIncomplete(
                f"missing handlers: {sorted(missing)}")

    def dispatch(self, task_type: str):
        try:
            return self._handlers[task_type]
        except KeyError:
            raise UnknownDispatchTarget(
                f"no handler for {task_type!r}")
