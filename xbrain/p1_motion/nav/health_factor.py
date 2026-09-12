"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: health_factor.py
Brief: cmd/motion/factor latest-value slot with the 11 S3.6 timeout ladder

Description:
The speed gate's h() input (11 S3.6 HealthFactor, P1-4) is a 1 Hz general-plane
message from p2_core: speed_factor in [0, 1], allow_motion, max_profile. This
module holds the latest one and turns "how old is it" into what the gate may
use this tick, per the contract's timeout line, verbatim: 3 s without the
message -> treat as speed_factor = 0.3 and raise a warn event; 10 s without ->
allow_motion = false.

Never received is NOT the same as fresh. A p1 that started before p2 granted
motion (14 Stage D publishes the first factor) must sit at zero, not drive on an
imagined h = 1.0; 12 S3.3's degradation table puts the health factor's DEAD row
at allow_motion = false -> zero. So view() before any message reports state
"never" with allow_motion False. The failure direction is the one CLAUDE.md 3.1
asks for: nothing here fabricates a grant.

Degraded keeps the LOWER of 0.3 and the last received value (12 S3.3: every
degradation path's ceiling is monotone non-increasing). Thresholds are injected
from the resolved config (12 S12 timeouts_ms.health_degrade / health_dead); this
file carries no numeric default for them.

Threading: on_message runs in the Zenoh Rust callback thread (CLAUDE.md 4.2) and
only assigns; view() runs on the 20 Hz loop. The stored tuple is replaced
whole, never mutated in place, so the loop never reads a half-written frame.

What it does NOT do: it does not decide the gate (host_gate.py does), does not
publish the warn event (the wiring does, on the state edge view() exposes), and
does not validate max_profile beyond the two-value closed set the contract
gives (obstacle_avoid | patrol; U33 removed the others).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Tuple

#: 11 S3.6 verbatim: "3 s 未收到本消息, 视为 speed_factor = 0.3".
DEGRADED_SPEED_FACTOR = 0.3
#: 11 S3.6 max_profile closed set ("U33 后仅此两值").
MAX_PROFILES = ("obstacle_avoid", "patrol")
#: view().state closed set.
STATE_NEVER = "never"
STATE_OK = "ok"
STATE_DEGRADED = "degraded"
STATE_DEAD = "dead"


class HealthFactorError(ValueError):
    """A cmd/motion/factor body outside the 11 S3.6 shape; the frame is dropped."""


@dataclass(frozen=True)
class HealthView:
    """What the gate may use THIS tick."""
    speed_factor: float
    allow_motion: bool
    max_profile: Optional[str]
    state: str
    age_ms: Optional[int]


def parse_health_factor(body: Any) -> Tuple[float, bool, str]:
    """Validate an 11 S3.6 body -> (speed_factor, allow_motion, max_profile).
    Off-shape values raise rather than degrade (CLAUDE.md 3.5): a speed_factor
    of 1.7 or "true" is a producer defect, not a value to clamp quietly."""
    if not isinstance(body, dict):
        raise HealthFactorError("factor body is not an object")
    sf = body.get("speed_factor")
    if (isinstance(sf, bool) or not isinstance(sf, (int, float))
            or not 0.0 <= float(sf) <= 1.0):
        raise HealthFactorError(
            "speed_factor must be a number in [0, 1], got %r" % (sf,))
    am = body.get("allow_motion")
    if not isinstance(am, bool):
        raise HealthFactorError("allow_motion must be a bool, got %r" % (am,))
    mp = body.get("max_profile")
    if mp not in MAX_PROFILES:
        raise HealthFactorError(
            "max_profile %r not in %s" % (mp, list(MAX_PROFILES)))
    return float(sf), am, mp


class HealthFactorSlot:
    """Latest-wins slot + the timeout ladder. degrade_after_ms < dead_after_ms,
    both > 0, both injected (12 S12 timeouts_ms)."""

    def __init__(self, degrade_after_ms: int, dead_after_ms: int) -> None:
        for name, v in (("health_degrade", degrade_after_ms),
                        ("health_dead", dead_after_ms)):
            if isinstance(v, bool) or not isinstance(v, int) or v <= 0:
                raise HealthFactorError(
                    "timeouts_ms.%s must be a positive int, got %r" % (name, v))
        if degrade_after_ms >= dead_after_ms:
            raise HealthFactorError(
                "timeouts_ms.health_degrade must be < health_dead")
        self._degrade_ms = degrade_after_ms
        self._dead_ms = dead_after_ms
        # (rx_mono_ms, speed_factor, allow_motion, max_profile) or None; replaced
        # whole, so the loop thread never sees a half-written frame.
        self._last: Optional[Tuple[int, float, bool, str]] = None
        self.received = 0
        self.rejected = 0

    def on_message(self, body: Any, rx_mono_ms: int) -> None:
        """RUST THREAD: validate + assign. Raises HealthFactorError on a bad
        body after counting it; the caller logs, the old value stays."""
        try:
            sf, am, mp = parse_health_factor(body)
        except HealthFactorError:
            self.rejected += 1
            raise
        self._last = (rx_mono_ms, sf, am, mp)
        self.received += 1

    def view(self, now_mono_ms: int) -> HealthView:
        """The ladder. never -> no motion; age >= dead -> no motion; age >=
        degrade -> min(0.3, last) with the last allow_motion; else last as is.
        mutant: return allow_motion=True on 'never' -> a p1 that never heard
        p2 drives -> test_never_received_forbids_motion red."""
        last = self._last
        if last is None:
            return HealthView(0.0, False, None, STATE_NEVER, None)
        rx, sf, am, mp = last
        age = now_mono_ms - rx
        if age >= self._dead_ms:
            return HealthView(0.0, False, mp, STATE_DEAD, age)
        if age >= self._degrade_ms:
            return HealthView(min(DEGRADED_SPEED_FACTOR, sf), am, mp,
                              STATE_DEGRADED, age)
        return HealthView(sf, am, mp, STATE_OK, age)
