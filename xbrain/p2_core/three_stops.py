"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: three_stops.py
Brief: BIZ-P2-21 -- soft_estop / hes / cmd_timeout unified handling

Description:
Three "stop" causes converge to ONE handler (spec: 'three-stop
processing branch count == 1'):

  soft_estop      cmd/estop{stop} from cloud / HMI / voice
  hes             hardware estop switch asserted
  cmd_timeout     20 Hz cmd_vel age > 200 ms threshold

For all three the P2 domain response is IDENTICAL:
  * domain 1 (motion): arb_suspend (14 BIZ-CM-3 disarm); every
    cmd/motion/intent thereafter is denied E_ARB_DISARMED
  * domain 2/3/4/5: suspended REMAINS null (verbatim rule 场景 6)
  * domain 2: holder unchanged (D siren keeps sounding)
  * domain 3: holder unchanged (voice keeps working)
  * domain 4: red/blue strobe FORCED ON via SE-1
  * domain 5: no change

The ONLY difference between the three stops: event/audit
detail.reason value.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional


class StopReason(str, Enum):
    """Closed set: 3 stop causes."""
    SOFT_ESTOP = "soft_estop"
    HES = "hes"
    CMD_TIMEOUT = "cmd_timeout"


@dataclass
class StopEvent:
    """One stop event. Fed into apply_stop for handling."""
    reason: StopReason
    cmd_id: str
    now_mono_ms: int


@dataclass
class ForceStrobeState:
    """SE-1: force red/blue strobe ON. During stop, this override
    stacks OVER the mode requirement (max(mode_required, forced)).
    On re-arm, this flag clears; strobe returns to mode requirement."""
    active: bool = False


def apply_stop(
    event: StopEvent,
    domain1_arbiter,           # arbiter for motion domain
    strobe_state: ForceStrobeState,
    emit_event: Callable[[dict], None],
) -> None:
    """Handle one stop event.

    Single branch: reason only tunes the event.detail.reason field.
    """
    # 1. Disarm domain 1 (motion). BIZ-CM-3 arb_suspend.
    #    Same call for all three reasons; the reason string differs.
    domain1_arbiter.arb_suspend(
        reason=event.reason.value,
        cmd_id=event.cmd_id,
        now_mono_ms=event.now_mono_ms,
    )
    # 2. Force red/blue strobe ON (SE-1).
    strobe_state.active = True
    # 3. Emit audit event. Detail.reason distinguishes the three.
    emit_event({
        "kind": "estop",
        "detail": {
            "reason": event.reason.value,
            "cmd_id": event.cmd_id,
        },
    })


def apply_rearm(
    cmd_id: str,
    now_mono_ms: int,
    domain1_arbiter,
    strobe_state: ForceStrobeState,
    emit_event: Callable[[dict], None],
) -> None:
    """Handle re-arm on a NEW motion command (BIZ-CM-3 arb_rearm).
    Clears the force-strobe override."""
    domain1_arbiter.arb_rearm(now_mono_ms=now_mono_ms)
    strobe_state.active = False
    emit_event({
        "kind": "estop_rearm",
        "detail": {"cmd_id": cmd_id},
    })
