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

TL-3 freshness, two deadlines split by LINK not by device:
  * local rt/teleop/input  (gamepad, keyboard_local)   200 ms
  * HMI   cmd/teleop       (keyboard_hmi, virtual_stick) 400 ms
11 S12A.9.6 states them as `now_mono - t_mono > 200 ms` and
`now_mono - t_rx_mono > 400 ms`; 11 S13 E_TELEOP_STALE repeats
the same pair verbatim, and so does 12 S4.7.1 TL-3.

*** 2026-09-30. Both halves of this module's source model were off
contract and its tests pinned them, so nothing was red.

  device names. 11 S12A.9.7 closes the set at gamepad /
  keyboard_local / keyboard_hmi / virtual_stick (plus the `none`
  sentinel, which is an active_source value, not a source). This
  module carried keyboard / joystick / hmi / cloud -- not one of
  the four is in the closed set, so a value from here reaching
  state/teleop.sources[].device would be an out-of-set value on
  the wire (CLAUDE.md 3.5), and P3's S12A.3 arming criterion 1,
  which looks for device in {gamepad, keyboard_local}, would find
  no local e-stop source and refuse every recording.

  deadlines. 500 ms for HMI and 1000 ms for cloud appear nowhere
  in 11 for teleop. 12 S4.7.1 TL-3 carries the 500 ms reading with
  a struck-through 已作废 next to it and the reason: 11's
  E_TELEOP_STALE row gives 200/400 and "both are stricter than
  500 ms". A deadline that is too long is the failure that matters
  here -- the source stays in the arbitration while its link is
  already gone, and the robot keeps driving on the last frame.

The authoritative definition point for the two numbers is the
config, 12 S12 verbatim: "TL-3 的两个超期门限 [不在本段定义] --
唯一定义处 = timeouts_ms.teleop_local(200) 与 timeouts_ms.
teleop_hmi(400)". Those keys have no consumer yet (p1_motion.yaml
S12 names teleop_* as landing with their consumer), so the pair
is spelled here, as teleop/state.py spells it. Wiring this module
means taking them from the resolved config, not keeping these.

state/teleop.mark_seq monotonically increments each accepted mark
so consumers can detect frame loss.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class TeleopSource(str, Enum):
    """11 S12A.9.7 device closed set. Values are the wire spellings.

    The member NAMES follow the values so a reader cannot pair the
    wrong two: TeleopSource.HMI would have to be read against the
    table to learn which of the two network devices it meant.
    """
    GAMEPAD = "gamepad"
    KEYBOARD_LOCAL = "keyboard_local"
    KEYBOARD_HMI = "keyboard_hmi"
    VIRTUAL_STICK = "virtual_stick"


#: TL-3 per-source freshness deadlines, keyed by the LINK the device
#: arrives on: the two local devices share rt/teleop/input's 200 ms,
#: the two network devices share cmd/teleop's 400 ms. Written per
#: device rather than per link because is_fresh() is handed a frame,
#: and a frame knows its device; deriving the link here would put a
#: second device-to-link table in the tree.
_TIMEOUT_MS = {
    TeleopSource.GAMEPAD: 200,
    TeleopSource.KEYBOARD_LOCAL: 200,
    TeleopSource.KEYBOARD_HMI: 400,
    TeleopSource.VIRTUAL_STICK: 400,
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
