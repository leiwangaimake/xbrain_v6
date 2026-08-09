"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: zenoh_planes.py
Brief: MOT-PM-32 dual-plane Zenoh sessions + cross-plane whitelist selfcheck

Description:
P1 is one of the three cross-plane processes (p1_motion / perception
/ chassis_relay). It holds TWO Zenoh sessions: RT plane (loopback
7449) for cmd_vel + estop + perception; GEN plane (7447) for
cmd/motion/factor + state/mode subscribes.

RT-C3: five keys are RT-plane-only and MUST NEVER be published on
the GEN plane. RT-C4: certain subscribes MUST come from GEN, not
RT. This module provides a startup-time selfcheck that verifies
the two session's declared key sets match the doc-mandated plane
assignment.
"""

from __future__ import annotations

from typing import FrozenSet


# 11 S1.1.6 RT-only publisher keys for P1. Publishing any of these
# on GEN would leak safety-critical traffic to the wider LAN.
RT_ONLY_PUB: FrozenSet[str] = frozenset({
    "rt/motion/cmd_vel",
    "rt/motion/twist",
    "rt/audio/lease",
    "rt/motion/status",
    "rt/motion/gate_seq",
})


class PlaneViolation(RuntimeError):
    """A key was published on the wrong plane."""


def check_rt_key_on_rt_only(key: str, plane: str) -> None:
    """RT-C3: RT-only keys refuse to publish on GEN plane."""
    if key in RT_ONLY_PUB and plane != "rt":
        raise PlaneViolation(
            "RT-C3: key %r is RT-only; refused on plane %r"
            % (key, plane))


def check_gen_pub_not_rt_only(gen_declared: FrozenSet[str]) -> None:
    """Startup selfcheck: GEN plane publishers must not include any
    RT_ONLY_PUB key."""
    leaks = gen_declared & RT_ONLY_PUB
    if leaks:
        raise PlaneViolation(
            "RT-C3 startup: GEN plane declares RT-only key(s) %s"
            % sorted(leaks))
