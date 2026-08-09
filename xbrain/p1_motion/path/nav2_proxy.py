"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: nav2_proxy.py
Brief: MOT-PM-19/20 nav2_proxy delegate + CD-1..CD-7 verify + PL-1..PL-3 crash policy

Description:
nav2_proxy is one of P1's 8 behavior sources that delegates to Nav2
(behavior_server + behavior_proxy). Two guards keep the delegation
sane:

  double-gate: cmd_vel from Nav2 must carry BOTH a matching cmd_id
    AND a fresh grant seq before being accepted; missing either drops
    the frame. PG-2: a cmd_vel without cmd_id is treated as unmatched
    (a permissive branch here would allow /spin action to drive the
    robot without a proper P1 grant).

  accuracy-verify (DELEG_VERIFY): after Nav2 reports 'succeeded',
    P1 waits 500 ms, reads L1 heading, computes heading error.
    If |err| > 3 deg, issues up to 2 corrective spins; if still
    off, terminates with detail.accuracy_notmet (CD-2).

  crash policy (PL-1..PL-3): if behavior_proxy dies mid-execution,
    P1 falls back to D-b (delegate-broken) and issues zero-vel until
    a new grant.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class DelegateStage(str, Enum):
    IDLE = "idle"
    ACTIVE = "active"
    DELEG_VERIFY = "deleg_verify"
    CORRECTING = "correcting"
    ARRIVED = "arrived"
    FAILED = "failed"


@dataclass
class DoubleGate:
    """Double-gate for accepting Nav2-produced cmd_vel."""
    expected_cmd_id: Optional[str] = None
    expected_gen: Optional[int] = None

    def accept(self, frame_cmd_id: Optional[str],
               frame_gen: Optional[int]) -> bool:
        """PG-2: BOTH must match. Missing cmd_id or gen -> reject."""
        if frame_cmd_id is None or frame_gen is None:
            return False
        if self.expected_cmd_id is None or self.expected_gen is None:
            return False
        return (frame_cmd_id == self.expected_cmd_id
                and frame_gen == self.expected_gen)


@dataclass
class VerifyState:
    """CD-2 accuracy-verify state after Nav2 'succeeded'."""
    started_mono_ms: int = 0
    corrections_left: int = 2      # up to 2 corrective spins
    tolerance_deg: float = 3.0


def needs_correction(err_deg: float, tolerance_deg: float) -> bool:
    return abs(err_deg) > tolerance_deg


def can_correct(vs: VerifyState) -> bool:
    return vs.corrections_left > 0


def consume_correction(vs: VerifyState) -> None:
    vs.corrections_left -= 1
