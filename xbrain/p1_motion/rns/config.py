"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: config.py
Brief: RNS config consume + startup assertions (20 S12 / S7A.1 D2 / S5.1.1)

Description:
Consumes the resolved rns.yaml snapshot (definition point is 12 S12.0A, the
values live in /run/xbrain/resolved/rns.yaml -- 10 S5.4.1: runtime reads the
snapshot, never the source). Runs the RNS-module startup assertions that must
fail LOUD before the 20 Hz loop ever emits a candidate.

Two assertions the freeze line cannot express alone, so they run here too as a
second gate (same A/G split as 12 S-1):
  D2 (20 S7A.1): leave_progress_m < 2 * r_eff. Violated => a thin obstacle's
     far-side legal leave point cannot satisfy (2)', the robot circles once and
     FALSELY reports unreachable. A-CVG-5. The freeze line has S-7 for this too;
     both gates matter (S-7 catches it before boot, this catches a hand-edited
     resolved snapshot).
  person lock (20 S5.1.1): class_map mapping person to anything but person_stop
     is the one config typo that hits a human. Refuse to start. A-CLS-4.

Why null is refused, not defaulted (CLAUDE.md 3.1): an uncalibrated safety
param defaulted to a number runs the robot on a guess; null -> refuse-to-start
naming the key path is the only fail-loud form. This module NEVER supplies a
default for a null key -- it raises RnsConfigError with the key path.

What this is NOT: it does not read the yaml source (PSC-1 forbids), does not
provide skip switches (CLAUDE.md 3.6), does not define keys (that is 12 S12.0A).
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from .types import T_CLASS_NAMES


class RnsConfigError(RuntimeError):
    """An rns.yaml key is null/missing/invalid, or a startup assertion failed.
    Carries the offending key path so the operator can fill it (10 S5.4.4)."""


# person is locked to person_stop in CODE, not config (20 S5.1.1): this is the
# one mapping whose typo hits a human, so it is not a configurable row.
PERSON_BEHAVIOR = "person_stop"

# behavior classes a class_map value may take (20 S5.1.1). Unmapped classes are
# handled as block+slow at consume time, so they need no entry; person_stop is
# reserved for the code-locked person row.
BEHAVIOR_CLASSES = frozenset({
    "person_stop", "vehicle_dynamic", "block", "traverse", "hazard",
    # 20 S5.1.1 v1.35 (#20-25): sixth class, airborne targets ONLY (bird /
    # kite / ball ...). Explicit rows, never the default -- the default
    # stays block, so a typo cannot silently ignore a ground obstacle.
    "ignore",
})


def require(cfg: Dict[str, Any], path: str) -> Any:
    """Fetch cfg at a dotted path or raise naming the path. This is the ONLY
    reader; there is no `.get(k, default)` anywhere in RNS (CLAUDE.md 3.1)."""
    node: Any = cfg
    walked = []
    for seg in path.split("."):
        walked.append(seg)
        if not isinstance(node, dict) or seg not in node:
            raise RnsConfigError(
                "rns.yaml missing key %r (10 S5.4.4: fill it, do not default)"
                % ".".join(walked))
        node = node[seg]
    if node is None:
        raise RnsConfigError(
            "rns.yaml key %r is null -- uncalibrated (CLAUDE.md 3.1: refuse to "
            "start, never default)" % path)
    return node


def assert_leave_progress_below_2r_eff(cfg: Dict[str, Any], r_eff_m: float) -> None:
    """D2 (20 S7A.1, A-CVG-5): leave_progress_m must be < 2*r_eff, else the
    convergence proof's L4 breaks and thin obstacles false-report unreachable.

    r_eff is passed in (12 S6A owns its single definition, 20 S1.3) -- RNS does
    NOT compute it here, only reads leave_progress_m and compares."""
    delta_s = require(cfg, "rns.wall_follow.leave_progress_m")
    limit = 2.0 * r_eff_m
    if not (delta_s < limit):
        raise RnsConfigError(
            "D2 violated (20 S7A.1): rns.wall_follow.leave_progress_m=%r must be "
            "< 2*r_eff=%r. Larger delta_s makes a thin obstacle's far-side leave "
            "point unreachable -> robot circles once and false-reports "
            "unreachable (A-CVG-5)." % (delta_s, limit))


def assert_person_locked(cfg: Dict[str, Any]) -> None:
    """person lock (20 S5.1.1, A-CLS-4): class_map must not map person to any
    behavior but person_stop. Absent-from-class_map is fine (person is locked in
    code); a WRONG explicit mapping is the refuse-to-start case."""
    class_map = cfg.get("rns", {}).get("class_map", {})
    if not isinstance(class_map, dict):
        return
    mapped = class_map.get("person")
    if mapped is not None and mapped != PERSON_BEHAVIOR:
        raise RnsConfigError(
            "person lock violated (20 S5.1.1): class_map maps person -> %r, only "
            "%r is legal. This is the one mapping whose typo hits a human "
            "(A-CLS-4)." % (mapped, PERSON_BEHAVIOR))


def assert_class_map_values(cfg: Dict[str, Any]) -> None:
    """class_map hygiene (20 S5.1.1 v1.35, A-CLS-8): every value must be a
    behavior class of the closed set, and the T class (traversable_area)
    must not appear as a row at all -- it is the T channel, not an object
    (11 S3.1B.2 v2.1). An out-of-set value would be consumed as a string
    nobody dispatches on, i.e. silently behave like block for the wrong
    reason; a T-class row signals the contract was misread. Both refuse."""
    class_map = cfg.get("rns", {}).get("class_map", {})
    if not isinstance(class_map, dict):
        return
    for name, beh in class_map.items():
        if name in T_CLASS_NAMES:
            raise RnsConfigError(
                "class_map row %r is the traversable-segmentation class, "
                "not an object (11 S3.1B.2 v2.1); remove it (A-CLS-8)." % name)
        if beh not in BEHAVIOR_CLASSES:
            raise RnsConfigError(
                "class_map %r -> %r is outside the behavior closed set %s "
                "(20 S5.1.1, A-CLS-8)." % (name, beh, sorted(BEHAVIOR_CLASSES)))


def run_startup_assertions(cfg: Dict[str, Any], r_eff_m: float) -> None:
    """All RNS startup assertions, in one call for the boot selfcheck. Raises
    RnsConfigError (fail-loud) on the first violation; the caller lets it
    propagate to refuse-to-start (never catches-and-continues, CLAUDE.md 3.6)."""
    assert_leave_progress_below_2r_eff(cfg, r_eff_m)
    assert_person_locked(cfg)
    assert_class_map_values(cfg)
