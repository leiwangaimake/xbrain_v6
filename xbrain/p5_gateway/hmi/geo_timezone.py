"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geo_timezone.py
Brief: Derive the HMI footer-clock IANA timezone from a GPS fix (17 S6.10.2 v1.3)

Description:
The footer clock used to tick in the STATIC common.timezone (17 S6.10.2 v1.2).
The user wants it to follow the robot's position: in Japan show Tokyo time, in
China Beijing time, in Malaysia Kuala Lumpur time (17 S6.10.2 v1.3). This module
turns a (lat, lon) fix into an IANA zone name via tzfpy (a Rust reverse-lookup
table), falling back to the site timezone when there is no usable fix.

This is a DISPLAY concern only: the internal wall clock is UTC and every
timeout/period uses the monotonic clock (CLK-C1), so a wrong zone can only
mislead the readout, never the machine (same non-authoritative status as
17 S6.10.2 v1.2). What this module is NOT: it does not decide the site
timezone for TTS G24 (that stays common.timezone via xbrain/common/time), and
it never blanks the clock -- there is ALWAYS a zone to return.

Two looks-right-but-wrong traps this file exists to close:
  * tzfpy's arg order is get_tz(LON, LAT) -- longitude FIRST (x, y order). Swap
    them and it returns a plausible-but-wrong zone instead of erroring, so the
    bug is silent. test_geo_timezone pins the order.
  * tzfpy maps (0,0) to 'Etc/GMT' -- a REAL-looking zone. A zeroed pose (no fix
    but coords defaulted to 0,0) would then silently pin the clock to GMT
    (3.2 fail-silent). So exact (0,0) is treated as no fix -> site fallback.
"""

from __future__ import annotations

from typing import Optional

# tzfpy is a Rust-backed wheel (installs on aarch64). Guard the import so a host
# without it degrades to the site fallback rather than failing to import the
# whole HMI snapshot path -- the clock still ticks, just in the site zone.
try:
    from tzfpy import get_tz as _get_tz          # get_tz(lon, lat) -> str
    _HAVE_TZFPY = True
except ImportError:                              # pragma: no cover - host-dependent
    _get_tz = None
    _HAVE_TZFPY = False


def timezone_for_fix(lat: Optional[float], lon: Optional[float],
                     fallback_tz: Optional[str]) -> Optional[str]:
    """IANA zone for a GPS fix, else `fallback_tz` (17 S6.10.2 v1.3).

    Returns the site fallback (never raises, never None-because-of-error) for
    every not-a-usable-fix case so the footer clock always has a zone to tick in
    (a display value must never blank the page). `fallback_tz` is common.timezone
    and may itself be None (last resort: the frontend then uses the browser
    zone), so this function can return None ONLY when the fallback is None.
    """
    # No fix, or a non-numeric coordinate (e.g. pose group's None) -> fallback.
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
        return fallback_tz
    # Null Island: exact (0,0) is the classic "coords defaulted, no real fix"
    # tell. tzfpy maps it to Etc/GMT (looks real), so guard it as no fix.
    if lat == 0.0 and lon == 0.0:
        return fallback_tz
    # Out of range / NaN / inf -> garbage. (NaN/inf fail every comparison, so
    # this catches them too.) tzfpy returns '' here rather than raising, but we
    # validate explicitly so the 2 Hz snapshot path cannot be taken down by a
    # future tzfpy that DID raise on bad input.
    if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
        return fallback_tz
    if not _HAVE_TZFPY:
        return fallback_tz
    # LON FIRST (see module docstring). '' (should not occur post-validation) or
    # None -> fall back rather than hand the frontend an unusable zone.
    return _get_tz(lon, lat) or fallback_tz
