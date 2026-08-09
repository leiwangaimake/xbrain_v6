"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: hot_update.py
Brief: MOT-PM-26 cmd/config + forwarded keys mirror to P1

Description:
P1 receives cmd/config messages for a narrow set of hot-updatable
keys (mostly RNS tuning). It also receives FORWARDED state keys
that P2 mirrors down: cmd/chassis/mode, cmd/chassis/light,
state/clock. These forwarded keys are READ-ONLY from P1's side;
P1 never PUBLISHES them, only observes.

Hot-updatable P1 keys are a whitelist; anything else is refused.
"""

from __future__ import annotations

from typing import FrozenSet


# P1's hot-updatable key surface. Non-hot: rns.rcg constants (need
# restart because they change the rotation permit math).
HOT_UPDATABLE_KEYS: FrozenSet[str] = frozenset({
    "rns.corridor.lambda_len",
    "rns.corridor.side_hold_ticks",
    "rns.candidate.psi_ref_deg",
})


# Keys P2 mirrors down for P1 to READ (never publish).
MIRROR_READ_ONLY_KEYS: FrozenSet[str] = frozenset({
    "cmd/chassis/mode",
    "cmd/chassis/light",
    "state/clock",
})


class HotUpdateError(RuntimeError):
    """Key outside the hot-updatable whitelist."""


def check_hot_updatable(key: str) -> None:
    if key not in HOT_UPDATABLE_KEYS:
        raise HotUpdateError(
            "%r is not hot-updatable; requires P1 restart" % key)


def is_mirror_read_only(key: str) -> bool:
    return key in MIRROR_READ_ONLY_KEYS
