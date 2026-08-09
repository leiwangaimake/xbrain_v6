"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: device_map.py
Brief: BIZ-P2-12 -- system-layer mode -> device-layer mode mapping (14 S5.7.1)

Description:
The two-layer mode system: system-layer mode names (what P2 reasons
about) are DIFFERENT from device-layer names (what payload-service
POSTs to /mode). 14 S5.7.1 pins the mapping:

  system idle       -> device func1   (MM-1: idle mounts func1 not idle,
                                        because func1 is idle's strict
                                        superset -- one less mode swap)
  system dialog_*   -> device func1   (A/C/E all func1 -> A-to-C etc
                                        does NOT call POST /mode; ML-2)
  system broadcast  -> device func2
  system alarm      -> device deter

ML-2: POST /mode returning 409 (already in the target device mode)
must be treated as OK, not an error. If two adjacent transitions
both land on func1 the second POST is 409 -> OK.

ML-5: switch_order = [device_mode, payload_light, ptz, motion, audio].
Reordering (e.g., payload_light before device_mode) causes the
device_mode change to reset the light AFTER we set it -- exact same
class of bug as V5 mode/light race.
"""

from __future__ import annotations

from typing import FrozenSet, List

from xbrain.p2_core.mode.state_machine import ModeState


# 14 S5.7.1 verbatim map. Adding a mode requires a doc change.
_SYS_TO_DEV = {
    ModeState.IDLE:      "func1",
    ModeState.DIALOG_A:  "func1",
    ModeState.DIALOG_C:  "func1",
    ModeState.DIALOG_E:  "func1",
    ModeState.BROADCAST: "func2",
    ModeState.ALARM:     "deter",
}

_DEVICE_MODES: FrozenSet[str] = frozenset({"func1", "func2", "deter"})


# 14 S5.7 ML-5 switch order (this is a COPY of the p2_core.yaml value;
# assertion in xbrain/p2_core/config/assertions.py enforces the two
# stay in step).
SWITCH_ORDER: List[str] = [
    "device_mode", "payload_light", "ptz", "motion", "audio",
]


def to_device_mode(sys_mode: ModeState) -> str:
    """Map a system mode to its device counterpart. Every ModeState
    has a mapping; adding a state requires updating _SYS_TO_DEV."""
    try:
        return _SYS_TO_DEV[sys_mode]
    except KeyError as exc:
        raise ValueError(
            "system mode %r has no device mapping" % sys_mode) from exc


def is_device_mode_change(from_sys: ModeState, to_sys: ModeState) -> bool:
    """True iff going from -> to actually requires a POST /mode call.
    If both sys modes map to the same device mode (e.g., dialog_a ->
    dialog_c both func1), no /mode call is issued (ML-2 recognition
    happens at 409 anyway, but skipping the call is cleaner)."""
    return to_device_mode(from_sys) != to_device_mode(to_sys)


def is_409_ok(http_status: int, in_switching_transition: bool) -> bool:
    """ML-2: POST /mode 409 during a switching transition IS OK
    (already-in-target-device-mode is a success)."""
    return http_status == 409 and in_switching_transition
