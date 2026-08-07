"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: breaker.py
Brief: The 16 S9.3 circuit breaker for an AI service -- N consecutive failures
       open it for a cooldown, measured on the monotonic clock

Description:
16 S9.3: an AI service that fails N times in a row (contract N=3) is circuit-
broken for a cooldown (contract 60 s), so the gateway stops hammering a dead
service and the degrade path (11 S8.13.5: ASR broken -> text channel still works;
TTS broken -> preset/tone only) takes over cleanly. This is that breaker.

Why the time is a parameter, not read here. Every deadline in this project is
measured on CLOCK_MONOTONIC (CLK-C1): a wall-clock step must never make a broken
service look recovered or a healthy one look broken. The breaker takes the
monotonic reading from the caller (the same discipline the arbiter and the
envelope writer use), so the cooldown maths is testable with injected values and
there is no CLOCK_REALTIME to read the wrong one from.

The three states (standard breaker, with the S9.3 numbers):
  CLOSED     -- requests pass. Each failure increments a run; SUCCESS resets it.
                At `threshold` consecutive failures the breaker OPENS.
  OPEN       -- requests are rejected outright until `cooldown_s` has passed on
                the monotonic clock, then it becomes HALF_OPEN.
  HALF_OPEN  -- ONE probe request is allowed. Its success CLOSES the breaker
                (service recovered); its failure OPENS it again for another
                cooldown (still down). This half-open probe is the standard
                reading of "熔断 60 s then retry"; without it a recovered service
                would stay locked out forever.

Traps -- things that look right and are not:
  1. Counting NON-consecutive failures. S9.3 says CONSECUTIVE: a success in the
     middle resets the run to zero. A breaker that counted total failures would
     trip on a service that is mostly fine, which is trap-worthy because it reads
     as "being careful".
  2. Injecting the threshold / cooldown as code defaults. They are contract
     values (3 / 60) but are passed in at construction, so a deployment that
     needs a different cooldown changes config, not code; and a 0 cooldown (which
     would make OPEN meaningless) is rejected at construction, not honoured.
"""

from enum import Enum

__all__ = ["BreakerState", "CircuitBreaker"]


class BreakerState(str, Enum):
    """The breaker's state. str-valued so .value is a stable token for a
    state/health readout."""

    CLOSED = "closed"          # normal: requests pass
    OPEN = "open"              # tripped: requests rejected during cooldown
    HALF_OPEN = "half_open"    # cooldown elapsed: one probe allowed


class CircuitBreaker:
    """One AI service's breaker (16 S9.3). Not thread-safe: the gateway serialises
    its calls onto one loop, like every other single-threaded object here.

    threshold and cooldown_s are injected -- no code defaults (trap 2). The
    contract values are 3 and 60; a bring-up caller passes those.
    """

    def __init__(self, threshold: int, cooldown_s: float) -> None:
        if threshold < 1:
            # A threshold below 1 would open on zero failures -- reject it rather
            # than build a breaker that is always open.
            raise ValueError("threshold must be >= 1, got %d" % threshold)
        if cooldown_s <= 0:
            # A non-positive cooldown makes OPEN last no time, so the breaker
            # never actually holds a dead service off. Reject it (trap 2).
            raise ValueError("cooldown_s must be > 0, got %r" % cooldown_s)
        self._threshold = threshold
        self._cooldown_s = cooldown_s
        self._state = BreakerState.CLOSED
        self._consecutive = 0          # consecutive failures in the CLOSED run
        self._opened_mono_s = 0.0      # when it last OPENed (monotonic)

    def state(self, now_mono_s: float) -> BreakerState:
        """The current state at `now_mono_s`, advancing OPEN -> HALF_OPEN when the
        cooldown has elapsed. Reads the clock only through the passed value."""
        if self._state is BreakerState.OPEN and \
                now_mono_s - self._opened_mono_s >= self._cooldown_s:
            # Cooldown elapsed: allow a single probe.
            self._state = BreakerState.HALF_OPEN
        return self._state

    def allow(self, now_mono_s: float) -> bool:
        """Whether a request may proceed now.

        CLOSED and HALF_OPEN allow it (HALF_OPEN allows exactly the one probe);
        OPEN rejects until the cooldown turns it HALF_OPEN. The gateway calls this
        before every request and raises E_TIMEOUT (from the degrade path) when it
        returns False.
        """
        return self.state(now_mono_s) is not BreakerState.OPEN

    def record_success(self, now_mono_s: float) -> None:
        """A request succeeded: reset the consecutive run and CLOSE.

        From HALF_OPEN this is the recovery: the probe worked, so the service is
        back and the breaker closes. From CLOSED it just clears the run (trap 1:
        a success resets consecutive failures to zero).
        """
        self._consecutive = 0
        self._state = BreakerState.CLOSED

    def record_failure(self, now_mono_s: float) -> None:
        """A request failed: advance the breaker.

        In HALF_OPEN a single failure re-OPENs for another full cooldown (the
        probe showed the service is still down). In CLOSED it increments the
        consecutive run and OPENs at the threshold.
        """
        if self._state is BreakerState.HALF_OPEN:
            # The probe failed: still down, open again from now.
            self._trip(now_mono_s)
            return
        self._consecutive += 1
        if self._consecutive >= self._threshold:
            self._trip(now_mono_s)

    def _trip(self, now_mono_s: float) -> None:
        """OPEN the breaker at `now_mono_s`, starting the cooldown."""
        self._state = BreakerState.OPEN
        self._opened_mono_s = now_mono_s
        # Keep _consecutive at/above threshold; it is reset only by a success, so
        # a re-trip from HALF_OPEN does not need to touch it.
