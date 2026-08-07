"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_gateway_core.py
Brief: VL2 gateway core -- the 11 S8.13.5 error map and the 16 S9.3 breaker, each
       with the mutant that would defeat it

Description:
Covers the pure half of the ai_client (16 S14): the status->code map and the
circuit breaker. Both are exercised without a live service. Each behaviour is
paired with a mutant per CLAUDE.md 3.3.
"""

import pytest

from xbrain.common import errors
from xbrain.p4_agent.ai_client import (
    BreakerState, CircuitBreaker, map_status, map_transport_error, AS7_TIMEOUT_S,
)


# --------------------------------------------------------------------------
# error map (11 S8.13.5)
# --------------------------------------------------------------------------

@pytest.mark.parametrize("status,code,retryable,max_retries", [
    (200, errors.OK, False, 0),
    (400, errors.E_SCHEMA, False, 0),
    (422, errors.E_SCHEMA, False, 0),
    (404, errors.E_NOT_FOUND, False, 0),
    (429, errors.E_BUSY, True, 1),
    (503, errors.E_BUSY, True, 1),
    (500, errors.E_INTERNAL, True, 1),
])
def test_status_map_matches_the_contract(status, code, retryable, max_retries):
    """Every row of the S8.13.5 table, verbatim."""
    m = map_status(status)
    assert m.code == code
    assert m.retryable is retryable
    assert m.max_retries == max_retries


def test_unmapped_status_is_schema_not_internal():
    """*** An unenumerated status must be E_SCHEMA (contract mismatch), not
    E_INTERNAL (trap 1).

    Mutation: default the unmapped branch to E_INTERNAL and this goes red -- a
    418 or a stray 3xx would then be silently swallowed as 'service internal
    error' instead of surfacing that the two ends disagree on the contract.
    """
    assert map_status(418).code == errors.E_SCHEMA
    assert map_status(301).code == errors.E_SCHEMA


def test_busy_and_internal_retry_at_most_once():
    """*** 429/503/500 retry ONCE, not forever (trap 2).

    Mutation: make max_retries unbounded for 500 and this goes red -- a
    persistently-500 service would be retried without limit, which is the
    breaker's job to bound, not this call's.
    """
    assert map_status(429).max_retries == 1
    assert map_status(503).max_retries == 1
    assert map_status(500).max_retries == 1


def test_transport_error_is_timeout_and_breaker_bounded():
    """A connection failure / timeout -> E_TIMEOUT, freely retryable here because
    the breaker (16 S9.3), not this map, bounds it."""
    m = map_transport_error()
    assert m.code == errors.E_TIMEOUT
    assert m.retryable is True
    assert m.max_retries == 0            # bounded by the breaker, not a counter


def test_as7_ceiling_is_five_seconds():
    """AS-7: the transport-timeout ceiling is 5 s, not the 30 s the 16 S14 sketch
    once carried."""
    assert AS7_TIMEOUT_S == 5.0


# --------------------------------------------------------------------------
# circuit breaker (16 S9.3)
# --------------------------------------------------------------------------

def _breaker():
    """The contract breaker: 3 consecutive failures, 60 s cooldown."""
    return CircuitBreaker(threshold=3, cooldown_s=60.0)


def test_three_consecutive_failures_open_it():
    """*** 16 S9.3: 3 consecutive failures open the breaker.

    Mutation: open at 4 (or never) and this goes red at the third failure.
    """
    b = _breaker()
    assert b.allow(0.0)
    b.record_failure(0.0)
    b.record_failure(0.1)
    assert b.allow(0.2)                  # two failures: still closed
    b.record_failure(0.2)
    assert not b.allow(0.3)              # third: open
    assert b.state(0.3) is BreakerState.OPEN


def test_a_success_resets_the_consecutive_run():
    """*** S9.3 counts CONSECUTIVE failures (trap 1).

    Two failures, then a success, then one failure must NOT be open -- the run
    reset. Mutation: count total failures and this trips, going red.
    """
    b = _breaker()
    b.record_failure(0.0)
    b.record_failure(0.1)
    b.record_success(0.2)                # resets the run
    b.record_failure(0.3)
    assert b.allow(0.4)                  # only one failure since the reset


def test_open_rejects_during_cooldown_then_half_opens():
    """OPEN rejects until the 60 s cooldown elapses (monotonic), then HALF_OPEN
    allows one probe."""
    b = _breaker()
    for t in (0.0, 0.1, 0.2):
        b.record_failure(t)             # opens at t=0.2
    assert not b.allow(30.0)            # 30 s in: still open
    assert not b.allow(59.9)            # just before cooldown: still open
    assert b.allow(60.2)               # >= 60 s after opening: half-open, probe allowed
    assert b.state(60.2) is BreakerState.HALF_OPEN


def test_half_open_success_closes_it():
    """The probe succeeds -> service recovered -> CLOSED."""
    b = _breaker()
    for t in (0.0, 0.1, 0.2):
        b.record_failure(t)
    assert b.allow(61.0)               # -> half-open
    b.record_success(61.0)             # probe worked
    assert b.state(61.0) is BreakerState.CLOSED
    assert b.allow(61.1)


def test_half_open_failure_reopens_for_another_cooldown():
    """*** The probe fails -> still down -> OPEN again for a full cooldown.

    Mutation: leave it HALF_OPEN (or CLOSED) after a failed probe and this goes
    red -- a still-dead service would keep being probed every call.
    """
    b = _breaker()
    for t in (0.0, 0.1, 0.2):
        b.record_failure(t)
    assert b.allow(61.0)               # half-open at t=61
    b.record_failure(61.0)             # probe failed
    assert not b.allow(61.1)           # open again
    assert not b.allow(120.0)          # still open until 61+60=121
    assert b.allow(121.5)              # next cooldown elapsed -> half-open again


def test_construction_rejects_degenerate_params():
    """threshold < 1 or cooldown <= 0 are rejected at construction (trap 2), not
    honoured into an always-open or never-holding breaker."""
    with pytest.raises(ValueError):
        CircuitBreaker(threshold=0, cooldown_s=60.0)
    with pytest.raises(ValueError):
        CircuitBreaker(threshold=3, cooldown_s=0.0)
