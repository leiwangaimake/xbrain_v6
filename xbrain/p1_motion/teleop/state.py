"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: state.py
Brief: 11 S12A.9.7 input-source arbitration + the state/teleop broadcast

Description:
P1 owns the teleop behaviour source, so it is the only process that can say
which input is actually driving. This module holds the per-source freshness and
deadman, arbitrates T-1..T-5, and shapes the state/teleop message.

Its consumers are not cosmetic. P3 reads sources[] as criterion 1 of the S12A.3
recording arming gate: while recording, lateral avoidance and voice e-stop are
both suppressed, so a live gamepad or local keyboard -- with its dedicated
e-stop key -- is one of the two things that may still stop the robot. An
optimistic answer here becomes a recording admitted with no reachable e-stop.

Which is why an unheard-from source is ABSENT from sources[] rather than listed
as stale-but-known: P3's criterion asks whether such a source is alive, and a
list that mentions a gamepad nobody has ever seen invites the reader to treat
its absence of evidence as evidence.

*** Two contract mismatches this had to settle, both recorded in S12A.9.7 on
2026-08-20:

  * device NAMES. S12A.9.7 fixes the closed set as gamepad | keyboard_local |
    keyboard_hmi | virtual_stick | none, while S12A.3's criterion 1 says
    "device in {gamepad, keyboard}" -- a name that is in neither set. Read here
    as the two LOCAL physical inputs (gamepad, keyboard_local): keyboard_hmi
    travels over the network, and criterion 7 exists precisely because the
    operator needs a key they can physically reach. The stricter reading does
    not lock anyone out -- an HMI-only operator still passes on criterion 2
    (the cmd/estop path).
  * alive vs stale. S12A.3 tests sources[].alive; S12A.9.7 defines the field as
    `stale`. Both are emitted, with alive = not stale, so neither reader has to
    know about the other's spelling.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

#: S12A.9.7 device closed set (minus the `none` sentinel, which is an
#: active_source value rather than a source entry).
TELEOP_DEVICES = ("gamepad", "keyboard_local", "keyboard_hmi", "virtual_stick")

#: S12A.9.7 internal priority. Higher wins. The HMI paths sit lowest because
#: they cross the network and their jitter is not controllable.
_PRIORITY = {"gamepad": 30, "keyboard_local": 20,
             "keyboard_hmi": 10, "virtual_stick": 10}

#: S12A.9.6 per-source freshness. The two local links are held to 200 ms; the
#: two that cross the network get 500 ms, which is the same figure 11 S2.2 gives
#: the HMI teleop key.
_TIMEOUT_MS = {"gamepad": 200, "keyboard_local": 200,
               "keyboard_hmi": 500, "virtual_stick": 500}

#: T-2: a source change must persist this long before it takes over. Without it
#: a marginal link flaps the active source and, with T-4, restarts the ramp on
#: every flap.
SWITCH_HYSTERESIS_MS = 300

#: The two devices that satisfy S12A.3 arming criterion 1 -- see the module
#: docstring on why keyboard_hmi is not among them.
LOCAL_ESTOP_DEVICES = frozenset({"gamepad", "keyboard_local"})


@dataclass
class _Source:
    last_mono_ms: int
    deadman: bool
    axes: Dict[str, float] = field(default_factory=dict)


@dataclass
class TeleopTracker:
    """Per-device freshness + the S12A.9.7 arbitration.

    Holds no clock: every entry point takes now_mono_ms. That is what makes the
    hysteresis and the timeouts testable without sleeping, and it is required
    anyway -- CLK-C1 puts every age computation on the monotonic clock.
    """
    sources: Dict[str, _Source] = field(default_factory=dict)
    mark_seq: int = 0
    _active: Optional[str] = None
    _candidate: Optional[str] = None
    _candidate_since_ms: int = 0

    def observe(self, device: str, *, now_mono_ms: int, deadman: bool,
                axes: Optional[Dict[str, float]] = None,
                mark_edge: bool = False) -> None:
        """One arrived teleop frame. An off-set device name raises: the S12A.9.7
        set is closed, and accepting an unknown name would put a source into the
        arbitration that no priority or timeout is defined for."""
        if device not in TELEOP_DEVICES:
            raise ValueError(
                "unknown teleop device %r, expected one of %s"
                % (device, list(TELEOP_DEVICES)))
        self.sources[device] = _Source(last_mono_ms=now_mono_ms,
                                       deadman=bool(deadman),
                                       axes=dict(axes or {}))
        if mark_edge:
            # S12A.9.8: the mark button's RISING edge, counted monotonically so
            # P3 can add a point idempotently even if a frame is redelivered.
            self.mark_seq += 1

    def is_stale(self, device: str, now_mono_ms: int) -> bool:
        src = self.sources.get(device)
        if src is None:
            return True
        return (now_mono_ms - src.last_mono_ms) > _TIMEOUT_MS[device]

    def arbitrate(self, now_mono_ms: int) -> Optional[str]:
        """T-1 + T-2: the highest-priority source that is fresh AND holding its
        deadman, subject to the switch hysteresis.

        Losing the current source is immediate; GAINING a new one waits out the
        hysteresis. The asymmetry is deliberate -- delaying a release would keep
        driving on an input that has gone away.
        """
        eligible = [d for d in TELEOP_DEVICES
                    if not self.is_stale(d, now_mono_ms)
                    and self.sources[d].deadman]
        best = max(eligible, key=lambda d: _PRIORITY[d]) if eligible else None
        if best == self._active:
            self._candidate = None
            return self._active
        if best is None:
            # T-5: no admissible source at all -> teleop goes inactive at once.
            # Handled before the hysteresis because "nobody is driving" is not
            # a switch between sources, and because the candidate bookkeeping
            # below would otherwise compare None with None and fall through on
            # the initial candidate timestamp.
            self._active = None
            self._candidate = None
            return None
        if self._active is None:
            # Nobody was driving: taking control is not a SWITCH, and T-2's
            # hysteresis is about switching. Delaying here would mean the first
            # 300 ms after picking up the controller went nowhere, which reads
            # on site as a dead gamepad and gets the deadman held harder.
            self._active = best
            self._candidate = None
            return self._active
        if self._active not in eligible:
            # The active source went stale or released: drop it now.
            self._active = best
            self._candidate = None
            return self._active
        # A different source wants to take over while the current one is still
        # eligible -> hysteresis.
        if best != self._candidate:
            self._candidate = best
            self._candidate_since_ms = now_mono_ms
            return self._active
        if now_mono_ms - self._candidate_since_ms >= SWITCH_HYSTERESIS_MS:
            self._active = best
            self._candidate = None
        return self._active

    def build_state(self, now_mono_ms: int, *,
                    profile_req: Optional[str] = None) -> Dict[str, Any]:
        """The S12A.9.7 TeleopState payload."""
        active = self.arbitrate(now_mono_ms)
        entries: List[Dict[str, Any]] = []
        for device in TELEOP_DEVICES:
            src = self.sources.get(device)
            if src is None:
                # Never heard from -> not listed. See the module docstring.
                continue
            stale = self.is_stale(device, now_mono_ms)
            entries.append({"device": device,
                            "age_ms": now_mono_ms - src.last_mono_ms,
                            "deadman": src.deadman,
                            "stale": stale,
                            # S12A.3 spells this one `alive`; emitted alongside
                            # so neither reader depends on the other's wording.
                            "alive": not stale})
        axes = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        if active is not None:
            axes.update(self.sources[active].axes)
        return {"schema": "teleop_state_v1",
                "active_source": active or "none",
                "deadman": bool(active and self.sources[active].deadman),
                # S12A.9.7: normalised and BEFORE the speed gate / fence /
                # clamp, so "what the operator asked for" can be compared with
                # "what the system allowed".
                "axes_out": axes,
                "mark_seq": self.mark_seq,
                "profile_req": profile_req,
                "sources": entries}


def has_local_estop_source(state: Optional[Dict[str, Any]]) -> bool:
    """S12A.3 arming criterion 1, evaluated against a TeleopState body.

    Lives here rather than in P3 so the device-name reading (see the module
    docstring) has one home; P3 imports nothing from p1_motion, so it carries
    its own copy of the CHECK but this is the definition it mirrors.
    """
    if not state:
        return False
    for entry in state.get("sources") or []:
        if not isinstance(entry, dict):
            continue
        alive = entry.get("alive")
        if alive is None:
            alive = not entry.get("stale", True)
        if alive and entry.get("device") in LOCAL_ESTOP_DEVICES:
            return True
    return False
