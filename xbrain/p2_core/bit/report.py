"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: report.py
Brief: BIZ-P2-23 -- BIT report + BIT-G1 fatal-item guard

Description:
14 S9 quick BIT reports one BitReport containing per-item results.
The report SUBSCRIBER (BIZ-P2-22 lifecycle SM) decides GRANT vs
BLOCKED based on whether any fatal item failed.

BIT-G1 (14 S11 configuration): p2_core.yaml.bit.quick.non_blocking_items
and skip_items MUST NOT contain any item whose level is FATAL. A
fatal item in non_blocking_items would let a broken chassis pass BIT
silently -> Stage 4 releases -> robot ships on unverified chassis.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, FrozenSet, Iterable, List

from xbrain.p2_core.health.items import (
    HealthLevel, HealthState, ITEM_LEVELS, is_fatal,
)


class BitResult(str, Enum):
    """Final BIT result per 14 S9 BIT-03."""
    PASS = "pass"
    FAIL = "fail"                # at least one fatal item failed
    DEGRADED = "degraded"        # only non-fatal items failed


@dataclass
class BitItemReport:
    item: str
    state: HealthState


@dataclass
class BitReport:
    """One BIT run's outcome. items lists every checked item."""
    items: List[BitItemReport] = field(default_factory=list)
    skipped: List[str] = field(default_factory=list)   # skip_items
    non_blocking: List[str] = field(default_factory=list)   # non_blocking_items

    def result(self) -> BitResult:
        """Combine per-item states + BIT config policy into one result.

        Non-blocking items' fail does NOT push toward FAIL (CFG-42:
        that's the whole point of the list).
        """
        for r in self.items:
            if r.state != HealthState.FAIL:
                continue
            if r.item in self.non_blocking or r.item in self.skipped:
                continue
            if is_fatal(r.item):
                return BitResult.FAIL
        # No fatal fail; check degraded.
        for r in self.items:
            if r.state in (HealthState.FAIL, HealthState.DEGRADED):
                return BitResult.DEGRADED
        return BitResult.PASS


class BitConfigViolation(RuntimeError):
    """A fatal item was listed in non_blocking_items or skip_items."""


def check_bit_g1(non_blocking_items: Iterable[str],
                 skip_items: Iterable[str]) -> None:
    """BIT-G1 startup gate: neither list may contain a fatal item.
    Raises with the offending item + its level."""
    bad_nb = [x for x in non_blocking_items if is_fatal(x)]
    bad_skip = [x for x in skip_items if is_fatal(x)]
    if bad_nb or bad_skip:
        raise BitConfigViolation(
            "BIT-G1 violation: fatal items in exempt list(s). "
            "non_blocking_items with fatal level: %s; "
            "skip_items with fatal level: %s. "
            "A fatal item in either list makes its failure invisible; "
            "the robot could ship on unverified safety-critical hardware."
            % (bad_nb, bad_skip))
