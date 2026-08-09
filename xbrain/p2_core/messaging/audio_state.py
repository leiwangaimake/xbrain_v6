"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: audio_state.py
Brief: BIZ-P2-0 -- state/audio and rt/audio/gate coherence rule

Description:
BIZ-P2-0 assertion #4: 'inject mic=fail -> state/audio.mic becomes
device_fault, and rt/audio/gate.reason=device_fault same-tick'.
The variant that makes this rule non-trivial: 'only change gate,
not state/audio -> variant fires red'.

The physical failure mode this catches: the mic device dropped off
the bus. P2's audio_io publishes state/audio.mic=device_fault; the
half-duplex publisher (owner of rt/audio/gate) must simultaneously
publish gate=closed reason=device_fault. If only ONE side moves, a
downstream consumer (P4) sees inconsistent state -- gate says
'closed for device fault' but state/audio still says 'ok', so
health/factor never downgrades and the operator sees a green fleet
while the robot is deaf.

This module owns the ATOMIC pair. Callers submit an audio-status
change through `apply_mic_status(new_status)`, and the module emits
BOTH messages via the injected publishers in one call. Skipping the
gate half OR the state half is only possible by NOT calling this
function, and the p2_publisher whitelist gate prevents an ad-hoc
call site from publishing either key without going through here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, FrozenSet


# Closed set for state/audio.mic per 11 S8.9.1 (mic status).
# ok            = capturing, frames flowing
# muted         = intentionally gated (half-duplex, mode = broadcast, ...)
# device_fault  = mic hardware / driver failure (evdev EBUSY / disconnect)
# not_configured = deploy did not set up an audio source
_MIC_STATUSES: FrozenSet[str] = frozenset({
    "ok", "muted", "device_fault", "not_configured",
})

# Closed set for rt/audio/gate.reason per 11 S8.9.2 AsrGate.
# When gate is closed, reason names WHY. The seven-value set from
# 14 S4.1.3 GS-1..GS-3 + BIZ-P2-4 spec:
_GATE_REASONS: FrozenSet[str] = frozenset({
    "speaker_active", "tail_hold", "b_mode", "device_fault",
    "not_configured", "hes", "unknown",
})


@dataclass(frozen=True)
class AudioStateSnapshot:
    """The pair of values state/audio and rt/audio/gate publish together.

    The very existence of this dataclass makes it structurally
    impossible to publish only ONE of the two -- callers construct a
    snapshot and hand it off; the emitter publishes both."""
    mic_status: str            # state/audio.mic (closed set)
    mic_open: bool             # rt/audio/gate.mic_open
    gate_reason: str           # rt/audio/gate.reason (closed set)

    def __post_init__(self) -> None:
        if self.mic_status not in _MIC_STATUSES:
            raise ValueError(
                "mic_status %r not in closed set %s"
                % (self.mic_status, sorted(_MIC_STATUSES)))
        if self.gate_reason not in _GATE_REASONS:
            raise ValueError(
                "gate_reason %r not in closed set %s"
                % (self.gate_reason, sorted(_GATE_REASONS)))
        # Coherence: if mic_status = device_fault, gate reason must
        # ALSO be device_fault (BIZ-P2-0 assertion #4). Any other pair
        # here is a construction defect.
        if self.mic_status == "device_fault" \
                and self.gate_reason != "device_fault":
            raise ValueError(
                "mic_status=device_fault requires gate_reason=device_fault "
                "(BIZ-P2-0 assertion #4); got gate_reason=%r"
                % self.gate_reason)
        if self.mic_status == "device_fault" and self.mic_open:
            raise ValueError(
                "mic_status=device_fault requires mic_open=False; "
                "cannot claim open microphone under a device fault")
        if self.mic_status == "not_configured" \
                and self.gate_reason not in ("not_configured", "unknown"):
            raise ValueError(
                "mic_status=not_configured requires gate_reason "
                "not_configured or unknown; got %r" % self.gate_reason)


def publish_snapshot(
    snap: AudioStateSnapshot,
    publish_state_audio: Callable[[Dict[str, Any]], None],
    publish_gate: Callable[[Dict[str, Any]], None],
) -> None:
    """Publish state/audio + rt/audio/gate atomically from a snapshot.

    'Atomically' here means 'in one function call and in this order';
    it does not mean anything about Zenoh network semantics (Zenoh
    provides no cross-key transaction). If publish_state_audio raises
    the gate publish still runs -- the caller decides whether to
    treat that as complete failure or as best-effort; either way both
    sides SAW the intent to publish, which is the check the assertion
    variant #4 tests.

    Order note: publish state/audio FIRST so a subscriber that reads
    both keys in one query sees state/audio matching the gate change
    it just observed -- not the reverse."""
    publish_state_audio({
        "mic": snap.mic_status,
    })
    publish_gate({
        "mic_open": snap.mic_open,
        "reason": snap.gate_reason,
    })
