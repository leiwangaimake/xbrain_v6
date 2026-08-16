"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_local_time.py
Brief: Unit tests for the site-timezone display formatter (local_time.py)

Description:
Locks the DISPLAY-only local-time helpers used by G24 (voice) and the HMI
footer clock. The properties that matter, each paired with a mutant that would
turn the assertion red (CLAUDE.md 3.3):

  * the zone is actually applied -- Shanghai (+8) and Tokyo (+9) differ by one
    hour for the SAME utc epoch. A mutant that ignores tz_name and formats UTC
    (or the host zone) collapses the two, so the difference assertion catches it.
  * a bad zone name RAISES rather than silently falling back to UTC. The module
    exists to surface a misconfigured site timezone, not to mislead the operator
    with a wrong-but-plausible time; a mutant that swallows the error and returns
    a UTC string fails test_format_local_raises_on_bad_zone.
  * the spoken weekday maps Monday=0..Sunday=6 to 周一..周日. A fixed known date
    (2023-11-15 is a Wednesday) pins _WEEKDAY_CN indexing; an off-by-one mutant
    (e.g. Sunday=0) prints the wrong weekday and fails.

All inputs are fixed utc epochs -- no wall-clock read -- so the vectors are
deterministic across hosts and timezones.
"""

import pytest

from xbrain.common.time.local_time import (
    format_local, format_spoken, is_valid_tz,
)

# 2023-11-14T22:13:20Z. Chosen so +8 (Shanghai) and +9 (Tokyo) both roll into
# 2023-11-15 -- a Wednesday -- exercising the date carry and the weekday map.
_UTC = 1700000000.0


def test_format_local_applies_zone():
    """The offset is real: Shanghai +8 vs Tokyo +9 differ by exactly one hour.
    MUTATION: format UTC ignoring tz_name -> both equal -> this fails."""
    assert format_local(_UTC, "Asia/Shanghai") == "2023-11-15 06:13:20"
    assert format_local(_UTC, "Asia/Tokyo") == "2023-11-15 07:13:20"


def test_format_local_raises_on_bad_zone():
    """A misconfigured site tz must FAIL LOUD, not degrade to UTC.
    MUTATION: try/except -> return UTC string -> no raise -> this fails."""
    with pytest.raises(Exception):
        format_local(_UTC, "Mars/Olympus")


def test_format_spoken_weekday_and_fields():
    """Spoken form for G24: month/day/weekday/hour/minute in the site zone.
    MUTATION: wrong weekday index (Sunday=0) -> 周X mismatch -> this fails."""
    assert format_spoken(_UTC, "Asia/Shanghai") == "11月15日 周三 6点13分"
    # Tokyo is one hour later on the same day -> hour advances, weekday stable.
    assert format_spoken(_UTC, "Asia/Tokyo") == "11月15日 周三 7点13分"


def test_is_valid_tz():
    """Zone-name validation used by the p4/p5 config guards.
    MUTATION: always return True -> the bad-name assertion fails."""
    assert is_valid_tz("Asia/Shanghai") is True
    assert is_valid_tz("Asia/Tokyo") is True
    assert is_valid_tz("Mars/Olympus") is False
    assert is_valid_tz("not a zone") is False
