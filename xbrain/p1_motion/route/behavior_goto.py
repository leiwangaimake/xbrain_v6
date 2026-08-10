"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: behavior_goto.py
Brief: CHK-1-22 HMI goto route -- path_follow(300), NOT teleop(600)

Description:
12 §4.7 last row: `cmd/motion/behavior{behavior:'goto'}` is a
single-waypoint navigation intent from the HMI. It MUST route
through path_follow(300), NOT through the teleop four-source
arbiter.

v0.4 clarification (why the rule is precise this way):
  If HMI-click-navigate were mounted as a fifth input to the
  teleop arbiter, it would inherit teleop's priority 600 -- higher
  than path_follow's 300 AND relative_move's 500. The immediate
  bug: rns_avoid(900) still preempts (900 > 600 > 500 > 300), but
  relative_move(500) would suddenly LOSE to a click-navigate that
  should have been the same priority tier as ordinary path_follow.

Test discipline (CHK-1-22 spec):
  * happy path: goto in isolation -> winner is path_follow(300)
  * negative regression: goto + relative_move(500) -> winner MUST
    be relative_move (proves goto DID NOT inherit 600). If goto
    inherited 600 via teleop, it would beat relative_move -- test
    catches that specific mistake.
  * deadman must not bleed: goto is not a teleop input, so no
    teleop-deadman stale check applies to it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


BEHAVIOR_GOTO = "goto"
PATH_FOLLOW_PRIORITY = 300


@dataclass(frozen=True)
class GotoRouteDecision:
    """Where a goto behavior should land in the source arbiter."""
    source: str        # 'path_follow'
    priority: int      # 300
    goes_via_teleop: bool     # MUST be False


class GotoRouteError(Exception):
    """A malformed behavior payload or an implementation that tries
    to route goto through teleop."""


def route_behavior_goto(behavior_payload: dict) -> GotoRouteDecision:
    """Translate a cmd/motion/behavior{'behavior':'goto', ...}
    payload into the single-waypoint route that path_follow accepts.
    Raises on any shape defect or a caller trying to mount this
    through teleop."""
    if not isinstance(behavior_payload, dict):
        raise GotoRouteError("behavior payload not object")
    if behavior_payload.get("behavior") != BEHAVIOR_GOTO:
        raise GotoRouteError(
            f"expected behavior='goto', got {behavior_payload.get('behavior')!r}")
    # Waypoint must be present -- default not allowed (CLAUDE.md 3.1).
    wp = behavior_payload.get("waypoint")
    if not isinstance(wp, dict) or not all(k in wp for k in ("x", "y")):
        raise GotoRouteError(
            "waypoint dict with (x, y) required; no silent default")
    return GotoRouteDecision(
        source="path_follow",
        priority=PATH_FOLLOW_PRIORITY,
        goes_via_teleop=False)


def assert_not_registered_in_teleop(teleop_source_names) -> None:
    """Meta-check called at P1 startup: the teleop four-source
    arbiter must NOT list 'behavior_goto' as an input source.
    Guards against the specific CHK-1-22 defect where a future
    editor mounts goto into the teleop chain (inheriting 600)."""
    forbidden = {"behavior_goto", "hmi_click_navigate", "goto"}
    hits = set(teleop_source_names) & forbidden
    if hits:
        raise GotoRouteError(
            f"teleop input sources must not include {sorted(hits)}; "
            f"HMI goto goes via path_follow(300), not teleop(600)")


def deadman_applies_to_source(source: str) -> bool:
    """Teleop deadman applies to teleop sources only. path_follow
    is not a teleop source, so goto in flight cannot be marked
    stale by teleop-side deadman timers."""
    TELEOP_SOURCES = frozenset({
        "teleop_local", "teleop_cloud", "teleop_gamepad",
        "teleop_keyboard",
    })
    return source in TELEOP_SOURCES
