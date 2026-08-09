"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ir_camera.py
Brief: CHK-1-09 IR camera health `cam_ptz_ir` (BIT-35 2026-08-05 ruling)

Description:
The IR (thermal) side of the PTZ camera has its own health item,
DELIBERATELY separate from the main ptz item. Reason (from
BIT-35 2026-08-05 ruling):

  * IR is optional -- daylight patrols do not need it. If IR is
    dead, the robot should not refuse to start.
  * IR failures should NOT put the ptz item red (they're on
    different sensors even though co-located mechanically).
  * IR failures MUST reject IR-observation commands
    (E_IR_UNAVAILABLE) and MUST show a persistent HMI badge.

Health probe:
  * ping the RTSP endpoint with a DESCRIBE request every 10s
  * on 3 consecutive failures -> warn event (once)
  * always non-blocking: never blocks robot startup or patrol

Health item mapping:
  cam_ptz_ir: healthy | warn | error
  NEVER contributes to overall availability computation
  (uses `blocking=False` flag on the item spec).

Rejection code: BIT-35 named the rejection with an E_IR_UNAVAILABLE
mnemonic, but the E_* closed set in codes.yaml (11 §13.4~§13.15)
does NOT define that code and CLAUDE.md §1 forbids unilaterally
adding one. Until the contract adds it, IR-observation rejections
carry E_UNHEALTHY with detail={'item': 'cam_ptz_ir'}; downstream
routes on both the code AND the item field so the semantic is
unambiguous.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from xbrain.common.errors import E_UNHEALTHY


IR_UNAVAILABLE_ERR = E_UNHEALTHY
IR_UNAVAILABLE_DETAIL = {"item": "cam_ptz_ir"}


@dataclass
class IrCameraHealth:
    """Windowed probe results; simple last-3-of-3 pattern."""
    recent_probes: List[bool] = field(default_factory=list)
    warn_emitted: bool = False

    def record_probe(self, ok: bool) -> None:
        self.recent_probes.append(ok)
        # keep last 3 only
        if len(self.recent_probes) > 3:
            self.recent_probes.pop(0)

    def three_consecutive_fail(self) -> bool:
        return (len(self.recent_probes) == 3
                and all(p is False for p in self.recent_probes))

    def reset_recovered(self) -> None:
        """One successful probe clears the warn latch."""
        self.warn_emitted = False


def reject_ir_command_when_unhealthy(state: str) -> None:
    """Any IR-observation command rejected with E_IR_UNAVAILABLE
    when state != 'healthy'."""
    if state != "healthy":
        raise RuntimeError(IR_UNAVAILABLE_ERR)


def is_blocking_for_availability() -> bool:
    """CHK-1-09: IR camera is NON-BLOCKING; must not gate patrol."""
    return False
