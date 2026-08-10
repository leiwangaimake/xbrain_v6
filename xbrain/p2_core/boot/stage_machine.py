"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: stage_machine.py
Brief: INF-BT-1 P2 boot Stage A~D + BLOCKED + BOOT-I1..I4 (with BOOT-I2 initial-value guard)

Description:
P2 boot progresses through five stages. BLOCKED is a sink; the
three exits from Stage C (any fatal fail / timeout_lock true /
common_digest mismatch) each drop into BLOCKED.

  Stage A  process up, threads registered
  Stage B  Zenoh sessions live, whitelists asserted
  Stage C  BIT quick + gate checks
  Stage D  publish cmd/motion/factor{allow_motion:true},
           speak the ready preset, enter normal operation
  BLOCKED  cannot leave without operator intervention

*** BOOT-I2 is the most-important-and-expensive rule in this
whole item: the INITIAL VALUE of cmd/motion/factor MUST BE
allow_motion=False, speed_factor=0.0, v_max=0 BEFORE ANY factor
message has been published or received. This defends against the
'never received cmd/motion/factor -> enjoy T-07 3s grace' pattern,
which at patrol 2.0 m/s covers 6 meters -- a 6-meter untested
run.

Every publish of cmd/motion/factor is gated by the SM: BLOCKED
cannot publish; Stage D publishes the first factor with real
values from BIT results.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional


class BootStage(str, Enum):
    STAGE_A = "stage_a"
    STAGE_B = "stage_b"
    STAGE_C = "stage_c"
    STAGE_D = "stage_d"
    BLOCKED = "blocked"


ALLOWED: dict = {
    BootStage.STAGE_A: {BootStage.STAGE_B, BootStage.BLOCKED},
    BootStage.STAGE_B: {BootStage.STAGE_C, BootStage.BLOCKED},
    BootStage.STAGE_C: {BootStage.STAGE_D, BootStage.BLOCKED},
    BootStage.STAGE_D: {BootStage.BLOCKED},        # BLOCKED from D on emergency
    BootStage.BLOCKED: set(),                        # sink; operator must reset
}


class InvalidBootTransition(Exception):
    pass


class BootI2Violation(Exception):
    """cmd/motion/factor published with unsafe initial values."""


@dataclass
class MotionFactor:
    """BOOT-I2: initial values BEFORE any factor was ever received
    or published. Zero everything; not True/1.0/reasonable."""
    allow_motion: bool = False
    speed_factor: float = 0.0
    v_max_mps: float = 0.0


def initial_motion_factor() -> MotionFactor:
    """Named constructor so the initial values are declared in
    ONE spot; any future edit that flips a default here is
    caught by the paired test."""
    return MotionFactor(allow_motion=False, speed_factor=0.0,
                          v_max_mps=0.0)


@dataclass
class BootFailure:
    """One entry on the BLOCKED reason list shown to HMI."""
    item: str
    reason: str


@dataclass
class BootStageMachine:
    """The SM itself. never_received_motion_factor is INDEPENDENT
    of the T-07 grace path: BOOT-I2 requires the initial state to
    be zero even after 2.9 s have elapsed."""
    stage: BootStage = BootStage.STAGE_A
    factor: MotionFactor = None
    never_received_factor: bool = True
    blocked_reasons: List[BootFailure] = None

    def __post_init__(self) -> None:
        if self.factor is None:
            self.factor = initial_motion_factor()
        if self.blocked_reasons is None:
            self.blocked_reasons = []

    def transition(self, to_stage: BootStage) -> None:
        if to_stage not in ALLOWED[self.stage]:
            raise InvalidBootTransition(
                f"{self.stage.value!r} -> {to_stage.value!r} not allowed")
        self.stage = to_stage

    def block(self, failures: List[BootFailure]) -> None:
        """Any transition into BLOCKED lists reasons for HMI."""
        self.blocked_reasons = list(failures)
        # BLOCKED reachable from any stage.
        self.stage = BootStage.BLOCKED

    def enter_stage_c_result(self, *,
                              any_fatal_fail: bool,
                              timeout_lock: bool,
                              common_digest_mismatch: bool,
                              failures: Optional[List[BootFailure]] = None,
                              ) -> None:
        """Three exits from Stage C: any triggers BLOCKED."""
        reasons: List[BootFailure] = list(failures or [])
        if any_fatal_fail:
            reasons.append(BootFailure(item="bit", reason="fatal_fail"))
        if timeout_lock:
            reasons.append(BootFailure(item="chassis", reason="timeout_lock"))
        if common_digest_mismatch:
            reasons.append(BootFailure(item="common_digest",
                                          reason="mismatch"))
        if reasons:
            self.block(reasons)
            return
        self.transition(BootStage.STAGE_D)

    def can_publish_motion_factor(self) -> bool:
        """Only Stage D may publish a real factor. Anywhere else,
        publishing MUST be refused."""
        return self.stage == BootStage.STAGE_D

    def factor_for_downstream(self, now_mono_ms: int,
                                grace_would_be_over_ms: int = 3000) -> MotionFactor:
        """BOOT-I2: if never received factor, initial zero is
        HARD -- no T-07 grace applies even after 2.9 s. The
        grace_would_be_over_ms parameter exists solely to make
        the '3 s later still zero' assertion legible."""
        _ = now_mono_ms
        _ = grace_would_be_over_ms
        if self.never_received_factor:
            return initial_motion_factor()
        return self.factor

    def note_factor_received(self, factor: MotionFactor) -> None:
        """Called when a real cmd/motion/factor arrives from P2."""
        self.factor = factor
        self.never_received_factor = False


def check_boot_i2_initial(mf: MotionFactor) -> None:
    """Anywhere in code, a MotionFactor produced before Stage D
    MUST match initial_motion_factor()."""
    good = initial_motion_factor()
    if (mf.allow_motion != good.allow_motion
            or mf.speed_factor != good.speed_factor
            or mf.v_max_mps != good.v_max_mps):
        raise BootI2Violation(
            f"BOOT-I2: initial motion factor must be {good!r}, "
            f"got {mf!r}")
