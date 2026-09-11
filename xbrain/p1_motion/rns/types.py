"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: types.py
Brief: RNS module closed sets and DTOs (20 S9.0.1/S9.0.2/S4.1/S4.2c)

Description:
Single export point for every closed set the RNS module (20 v1.15) commits to.
CLAUDE.md 3.5 makes closed sets throw on out-of-range values rather than pass
them through -- this file is where the RNS module form of that rule lives.

Why one file, imported everywhere else in rns/:
  - a state or reason string hardcoded in two modules drifts (CLAUDE.md 3.5);
  - the arbiter/gateway need the SAME NavFailReason strings the module emits,
    and 12 S4.2c.6 maps them to abort_reason/path_progress -- one authority.

What is NOT here (and why):
  - E_* error codes: the module never invents them; 12 S4.2c.6 maps NavFailReason
    to the carrier (abort_reason / path_progress.fail_reason). Book 20 S9.0.2.
  - config values: those live in rns.yaml (12 S12.0A), consumed by config.py.
  - the follow_target machinery: reserved (20 #20-13); MissionKind carries the
    enum value so schema round-trips, but no consumer branch exists this phase.

Traps this file's shape guards:
  - MissionKind.FOLLOW_TARGET present as a value != a live feature. It is a
    reserved schema slot; classify/route/source have no branch for it.
  - NavState.ARRIVED/FAILED/CANCELLED are EVENT exits, not resident states
    (20 S9.0.1): they are reported once, then the module clears to IDLE. They
    live in the same enum only because they name a mission's outcome.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

from xbrain.common.types.units import Mps, Seconds


# ── Three-state space (20 S3.1.2 / RNS-N-5) ───────────────────────────────────
class Cell(str, Enum):
    """The only three occupancy states. 20 S4.1: everything is one of these;
    UNKNOWN is a first-class value, never silently promoted to FREE (RNS-I-1)."""
    FREE = "free"
    BLOCKED = "blocked"
    UNKNOWN = "unknown"


# 11 S3.1B.2 v2.1 / 20 S5.1.1 v1.35: the traversable-segmentation class is
# the T channel ITSELF (carried by profile bit0 / d_free), never an object.
# A tracked object bearing this class is a producer error; the consumer
# drops it (A-CLS-5) rather than fold the whole walkable area into a wall
# via the unmapped->block default. Closed set: one name today.
T_CLASS_NAMES = frozenset({"traversable_area"})


class SrcBit:
    """src bitmask bit positions (11 S3.1B.1). A cell records WHICH channel
    produced the BLOCKED/FREE decision; bit0 doubles as the FREE-completeness
    marker (bit0 unset in a d_free run = geometry-only, RNS limits speed,
    20 S3.1.11). Not an Enum: these OR together."""
    SEG = 1 << 0        # traversable segmentation (T channel)
    GEOM = 1 << 1       # geometric voxel (G channel)
    SEMANTIC = 1 << 2   # semantic object (S channel, 11 S3.4A)
    NEGATIVE = 1 << 3   # negative obstacle (pit / cliff)


# ── Run-state machine (20 S9.0.1) ─────────────────────────────────────────────
class NavState(str, Enum):
    """Resident run states plus the three event exits. Transition legality is
    enforced by source.py against TRANSITIONS below; any move not in the table
    throws (20 S9.0.1, asserted by A-ST-1)."""
    IDLE = "idle"                 # no mission; is_active() == false
    FOLLOW = "follow"            # polyline follow (incl. S2.5 align segment)
    THREAD = "thread"            # threading a gap (a candidate in execution)
    DETOUR = "detour"            # detouring (a candidate in execution)
    WALL_FOLLOW = "wall_follow"  # bug-style wall following (20 S7)
    WAIT_DYNAMIC = "wait_dynamic"  # stopped for a dynamic obstacle (20 S5.3/S5.3A)
    SUSPENDED = "suspended"      # estop / preemption held; NON-terminal
    # event exits (reported once -> clear -> IDLE), not resident:
    ARRIVED = "arrived"
    FAILED = "failed"
    CANCELLED = "cancelled"


# Legal transitions, verbatim from 20 S9.0.1. Event exits ARRIVED/FAILED/
# CANCELLED collapse to IDLE and so are not transition targets here; they are
# reported by source.py then it sets IDLE. WAIT_DYNAMIC returns to whatever
# state entered it, so its resume targets are the four motion states.
_MOTION = frozenset({NavState.FOLLOW, NavState.THREAD, NavState.DETOUR})
TRANSITIONS: dict = {
    NavState.IDLE: frozenset({NavState.FOLLOW}),
    NavState.FOLLOW: frozenset({NavState.THREAD, NavState.DETOUR,
                                NavState.WALL_FOLLOW, NavState.WAIT_DYNAMIC,
                                NavState.SUSPENDED, NavState.IDLE}),
    NavState.THREAD: frozenset({NavState.FOLLOW, NavState.WALL_FOLLOW,
                                NavState.WAIT_DYNAMIC, NavState.SUSPENDED,
                                NavState.IDLE}),
    NavState.DETOUR: frozenset({NavState.FOLLOW, NavState.WALL_FOLLOW,
                                NavState.WAIT_DYNAMIC, NavState.SUSPENDED,
                                NavState.IDLE}),
    NavState.WALL_FOLLOW: frozenset({NavState.FOLLOW, NavState.WAIT_DYNAMIC,
                                     NavState.SUSPENDED, NavState.IDLE}),
    # resume target of WAIT_DYNAMIC is the pre-wait state; source.py restores it
    # explicitly, so the table permits the four motion states plus exits.
    NavState.WAIT_DYNAMIC: frozenset(_MOTION | {NavState.WALL_FOLLOW,
                                                NavState.SUSPENDED,
                                                NavState.IDLE}),
    NavState.SUSPENDED: frozenset({NavState.FOLLOW, NavState.IDLE}),
}


def is_legal_transition(frm: NavState, to: NavState) -> bool:
    """A-ST-1: table-external transitions are illegal. IDLE is always a legal
    target (any-state -> IDLE on arrive/fail/cancel, 20 S9.0.1)."""
    return to in TRANSITIONS.get(frm, frozenset())


# ── Failure attribution (20 S9.0.2) ───────────────────────────────────────────
class NavFailReason(str, Enum):
    """The closed set of navigation failure reasons. 12 S4.2c.6 maps these to
    the outward carrier (abort_reason for relmove, path_progress.fail_reason for
    route) -- this enum is that mapping's single upstream. Out-of-set = throw."""
    MAX_DEVIATION = "max_deviation"            # 20 S2.7
    WALL_CLOSED_LOOP = "wall_closed_loop"      # 20 S7.6-(1): unreachable
    WALL_NO_PROGRESS = "wall_no_progress"      # 20 S7.6-(2)
    WALL_BUDGET = "wall_budget"                # 20 S7.6-(3) backstop
    WATCHDOG_NO_PROGRESS = "watchdog_no_progress"  # 20 S7.3A branch two
    BLOCKED_BY_DYNAMIC = "blocked_by_dynamic"  # 20 S5.3A wait budget
    RTK_UNRELIABLE = "rtk_unreliable"          # 20 S3.2.1 SINGLE/no-fix
    TARGET_LOST = "target_lost"                # 20 S2.10.4 (reserved #20-13)
    NO_PATH_IN_DOMAIN = "no_path_in_domain"    # 20 S4A.3 (G2): both search
    #   modes exhausted -- the remembered BLOCKED set separates robot from
    #   goal inside the planning domain. A PROVEN no-path, not a tired-of-
    #   trying heuristic; retry only after the world (or the goal) changes.


# ── Mission (20 S4.2c / S9.0.3) ───────────────────────────────────────────────
class MissionKind(str, Enum):
    GOTO = "goto"                    # single target (RNS-N-1 degenerate polyline)
    PATH = "path"                    # recorded polyline (0.5 m spacing)
    FOLLOW_TARGET = "follow_target"  # reserved, 20 #20-13; no consumer this phase


class Origin(str, Enum):
    """Which entry a mission came in on. 12 S4.2c.1: the report channel is
    chosen by ORIGIN, not by mission category -- a single-point route is a GOTO
    but origin=ROUTE, so it reports on path_progress, not status. This field is
    the reason source.py can pick the right carrier on failure."""
    ROUTE = "route"       # cmd/motion/route (P1-11); reports on path_progress
    RELMOVE = "relmove"   # cmd/motion/relative_move (P1-5); reports on status


@dataclass(frozen=True)
class NavFailure:
    """A terminal failure, reported exactly once (20 S9.0.3). detail carries the
    per-reason required fields (20 S9.0.2 detail column): e.g. track_id/class for
    BLOCKED_BY_DYNAMIC, argmax limiter source for WATCHDOG_NO_PROGRESS."""
    reason: NavFailReason
    detail: dict  # required keys are per-reason; validated at emit site, not here


@dataclass(frozen=True)
class VelocityCandidate:
    """RNS output for one tick (20 RNS-M-3): a candidate, never a command.
    None-return (not this type) means "no output this tick" and is the normal
    idle value, distinct from a zero-velocity candidate."""
    vx: Mps
    vy: Mps
    wz: float  # rad/s; wz has no strong unit type in units.py, kept float


# NavFailReason values that map to a fence-origin abort (12 S4.2c.6): only the
# watchdog reason can resolve to fence, and only when its argmax source is fence.
# Kept as a constant so the mapping table and the code cannot drift.
WATCHDOG_FENCE_SOURCE = "fence"
