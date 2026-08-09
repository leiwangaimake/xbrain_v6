"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: four_source.py
Brief: MOT-PM-21 teleop 4-source arbiter + TL-1/TL-2 estop-before-normalize

Description:
Teleop admits four sources with distinct priorities and
freshness deadlines. TL-1/TL-2: emergency-stop key parsing MUST
run BEFORE any velocity normalization. If the arriving frame is
corrupt AND estop is asserted, the pipeline STILL emits cmd/estop
-- moving estop parse into or after normalize would let a bad
frame swallow the stop, exactly the failure mode TL-1/TL-2 exist
to prevent.

TL-3: local sources time out at 200 ms (keyboard/joystick);
HMI at 500 ms; cloud at 1000 ms. state/teleop.mark_seq
monotonically increments each accepted mark so consumers can
detect frame loss.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import FrozenSet, Optional


class TeleopSource(str, Enum):
    KEYBOARD = "keyboard"
    JOYSTICK = "joystick"
    HMI = "hmi"
    CLOUD = "cloud"


# TL-3 per-source freshness deadlines.
_TIMEOUT_MS = {
    TeleopSource.KEYBOARD: 200,
    TeleopSource.JOYSTICK: 200,
    TeleopSource.HMI: 500,
    TeleopSource.CLOUD: 1000,
}


@dataclass(frozen=True)
class TeleopFrame:
    """One arrived teleop frame BEFORE parsing / normalization."""
    source: TeleopSource
    raw_bytes: bytes
    arrived_mono_ms: int


@dataclass(frozen=True)
class ParsedEstop:
    """Result of the estop-first parse."""
    estop_asserted: bool
    raw_ok: bool          # True if the rest of the frame parsed clean


def parse_estop_first(frame: TeleopFrame) -> ParsedEstop:
    """TL-1/TL-2: extract estop bit BEFORE trying to parse velocities.
    A corrupt frame whose estop bit is still readable MUST still fire
    the stop; a variant that parses velocities first would drop the
    frame on corruption AND lose the stop."""
    if not frame.raw_bytes:
        return ParsedEstop(estop_asserted=False, raw_ok=False)
    # Convention: first byte is the estop flag; nonzero = asserted.
    # Real wire format would use a proper header; the SEMANTIC that
    # matters here is 'estop parse is INDEPENDENT of the rest'.
    estop_bit = frame.raw_bytes[0] != 0
    # Rest of the frame may or may not parse; that's INDEPENDENT.
    rest_ok = len(frame.raw_bytes) >= 4   # nominal 4-byte payload
    return ParsedEstop(estop_asserted=estop_bit, raw_ok=rest_ok)


def is_fresh(frame: TeleopFrame, now_mono_ms: int) -> bool:
    """TL-3: per-source freshness."""
    return (now_mono_ms - frame.arrived_mono_ms) <= _TIMEOUT_MS[frame.source]
