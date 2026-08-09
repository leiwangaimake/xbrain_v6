"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: path_follow.py
Brief: MOT-PM-17 path_follow pure pursuit + LP-1..LP-8 loop SM

Description:
Path_follow is one of the 8 P1 behavior sources. It runs a pure-
pursuit controller over a loaded path (list of waypoints) with
lookahead. The loop state machine LP-1..LP-8 handles multiple
traversals of the same path (loops parameter): loops=0 means
INFINITE circles and MUST NOT auto-arrive (LP-3a variant); a bad
implementation that treats loops=0 as 'done immediately' would
skip the whole traversal.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple


class LoopState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    ARRIVED = "arrived"


@dataclass
class PathFollowConfig:
    """One path_follow session's config."""
    waypoints: List[Tuple[float, float]]
    loops_total: int          # 0 = infinite; N > 0 = exactly N traversals
    lookahead_m: float


@dataclass
class PathFollowState:
    """LP state machine."""
    state: LoopState = LoopState.IDLE
    loop_index: int = 0       # completed traversal count
    wp_index: int = 0         # current waypoint within traversal


def is_arrived(state: PathFollowState, cfg: PathFollowConfig) -> bool:
    """LP-3a: loops_total == 0 means infinite -> NEVER arrived.
    Any impl that returns True for loops_total==0 is a bug that
    would skip the entire traversal without moving the robot."""
    if cfg.loops_total == 0:
        return False       # infinite; the RUNNING state persists
    return state.loop_index >= cfg.loops_total


def advance_waypoint(state: PathFollowState,
                     cfg: PathFollowConfig) -> None:
    """LP-4: increment wp_index; when past last waypoint, close the
    loop (increment loop_index, reset wp_index to 0). Caller MUST
    check is_arrived after this to decide whether to keep running."""
    state.wp_index += 1
    if state.wp_index >= len(cfg.waypoints):
        state.wp_index = 0
        state.loop_index += 1


def pure_pursuit_target(state: PathFollowState,
                         cfg: PathFollowConfig,
                         robot_x: float, robot_y: float
                         ) -> Optional[Tuple[float, float]]:
    """Return the current lookahead target waypoint, or None if
    arrived (LP-3a: with loops_total==0 never returns None)."""
    if is_arrived(state, cfg):
        return None
    if not cfg.waypoints:
        return None
    return cfg.waypoints[state.wp_index]
