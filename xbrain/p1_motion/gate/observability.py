"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: observability.py
Brief: MOT-PM-31 double-loop 'clipped' observability PX-1..PX-4

Description:
Two loops exist inside a single P1 tick: the OUTER loop is the
20 Hz arbiter+gate+publish cycle; the INNER loop is a per-source
computation (e.g., RNS candidate search, path_follow lookahead
step). Both can produce a 'clipped' result:

  PX-1  outer loop clipped: gate reduced source's requested vx
  PX-2  inner loop clipped: RNS candidate hit corridor L_min
  PX-3  outer 'no-op' (all clip factors == 1): safe path
  PX-4  outer + inner BOTH clipped: log both, do not merge

The variant these guard against: reporting only ONE side of the
clip (whichever fired first) hides the other. Consumers of the
audit stream would see 'RNS clipped' but miss that gate ALSO
clipped, or vice versa.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class ClipReport:
    """One tick's clipping observability record.

    Both `outer_clipped` and `inner_clipped` are boolean flags with
    a list of 'why' reasons. Merging them into a single flag is the
    exact regression PX-1..PX-4 exist to catch."""
    outer_clipped: bool
    outer_reasons: tuple            # limiter values that fired
    inner_clipped: bool
    inner_reasons: tuple            # RNS / path_follow inner reasons


def build_clip_report(
    outer_reasons: List[str],
    inner_reasons: List[str],
) -> ClipReport:
    """A source is 'clipped' iff its reasons list is non-empty. Both
    flags are computed INDEPENDENTLY -- there is deliberately no
    'merged' output so the caller cannot lose one side."""
    return ClipReport(
        outer_clipped=len(outer_reasons) > 0,
        outer_reasons=tuple(outer_reasons),
        inner_clipped=len(inner_reasons) > 0,
        inner_reasons=tuple(inner_reasons),
    )
