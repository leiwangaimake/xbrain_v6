"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: lapi_guard.py
Brief: BIZ-P2-10 -- PTZ LAPI write-key guard (BR-1/BR-3/BR-4)

Description:
The 2026-08-05 change removed PTZ boost / bitrate control. P2's
LAPI WRITE surface to the PTZ ball is now EXACTLY two keys:

  FocusMode              (14 S11: focus_mode = 2)
  ShieldTrigger.MovePTZ  (14 S11: shield_move_ptz = 1)

Any other WRITE call is a defect. The most dangerous class of
regression: a stray bitrate=16384 or bitrate=6144 write reintroduces
the removed boost mechanism silently. This guard makes such a call
an unrecoverable startup refusal.

Scan surface is EXPLICIT (BIZ-P2-10 spec verbatim): WRITE / SET / PUT
call sites only. Read (GET) sites are NOT scanned because BIT thread
per 11 S7.4A.1 legitimately reads bitrate for comparison; scanning
reads would make the guard permanently red.
"""

from __future__ import annotations

from typing import FrozenSet, Iterable, List


# The 14 S11 authorised LAPI WRITE set. Adding a third key requires a
# doc change; this constant IS the guard.
ALLOWED_WRITE_KEYS: FrozenSet[str] = frozenset({
    "FocusMode",
    "ShieldTrigger.MovePTZ",
})

# Forbidden keys that if written would reintroduce a removed feature.
# Not exhaustive (that's ALLOWED_WRITE_KEYS' job) -- this list carries
# the ones whose accidental re-introduction has a specific failure
# story worth naming in the error message.
_KNOWN_FORBIDDEN = {
    "VideoBitrate":       "reintroduces removed PTZ boost bitrate control",
    "bitrate":            "reintroduces removed PTZ boost bitrate control",
    "I_interval":         "reintroduces removed PTZ boost keyframe control",
    "sources_allowed":    "reintroduces removed PTZ boost admission",
}


class LapiWriteViolation(RuntimeError):
    """A LAPI write was attempted to a key outside ALLOWED_WRITE_KEYS.
    Startup-level defect: raise; do not swallow."""


def check_write_key(key: str) -> None:
    """Raise LapiWriteViolation if key is not in the allowed set.
    Named forbidden keys get an extra 'why this is dangerous' reason."""
    if key in ALLOWED_WRITE_KEYS:
        return
    reason = _KNOWN_FORBIDDEN.get(key, "not in the authorised 2-key set")
    raise LapiWriteViolation(
        "LAPI WRITE to %r is forbidden -- %s. Authorised keys: %s."
        % (key, reason, sorted(ALLOWED_WRITE_KEYS)))


def check_batch(keys: Iterable[str]) -> List[str]:
    """Batch check; returns list of violating keys (empty on pass).
    Convenience wrapper for a startup selfcheck that collects all
    violations before raising."""
    bad: List[str] = []
    for k in keys:
        try:
            check_write_key(k)
        except LapiWriteViolation:
            bad.append(k)
    return bad
