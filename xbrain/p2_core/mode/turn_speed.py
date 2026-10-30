"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: turn_speed.py
Brief: The robot's in-place turn speed -- default max, voice-adjustable in steps, an explicit value kept across restarts (99 U104)
Description:
What problem this solves
------------------------
Field test 2026-10-29 (R019): "左转90度 / 180度" turned short, left turns
especially. The p1 log shows every finished turn within 1.5 degrees; the bad
ones were stopped by the stall guard, many after ~5 degrees, because the
fixed 0.3 rad/s sits on the chassis' "twists but does not step" threshold
(0.2 only twists, measured 2026-09-15). User ruling 2026-10-30 (99 U104): turn
at the maximum by default; "机器人转速快一点 / 慢一点 / 最快 / 最慢" adjusts it,
never below the twist threshold; an explicitly set value stays, across
restarts, exactly like the device volume (99 U100 continued).

The numbers (all from p2_core.yaml turn_speed, none in code)
------------------------------------------------------------
  max_radps   0.6  INTERIM, user ruling 2026-10-30: the three vendor manuals
                   in docs/QUADRUPED give no maximum yaw rate (nor vx / vy);
                   replace with the measured chassis limit after a field run.
  min_radps   0.4  above the twist threshold (0.2 twists, 0.3 still stalls).
  step_radps  0.1  one "快一点 / 慢一点".

Rules
-----
  * nothing set -> max (the default); an explicit value is persisted and
    restored at the next start;
  * a stored value outside [min, max] (the config moved since) is CLAMPED
    into range with a warning, never trusted above max;
  * arithmetic runs on integer milli-rad/s, so ten "快一点" never drift to
    0.6000000001 and miss the max;
  * the file is written new + fsync + rename + directory fsync (this robot
    loses power uncleanly); an unreadable file falls back to the default.

Where it is used: ModeFace applies the set_turn_speed action and publishes
snapshot() in state/mode; the relmove builder fills MO-2 max_yaw_rate_radps
with effective().
"""
from __future__ import annotations  # postponed annotations

import json  # the state file is one small JSON object
import logging  # load warnings
import os  # atomic write
import threading  # ModeFace and the relmove builder run on different threads
from typing import Any, Dict, Optional  # type hints

_logger = logging.getLogger("xbrain.p2.turn_speed")  # module logger

_KEY = "turn_speed_radps"  # the one field in the state file
LEVELS = ("faster", "slower", "max", "min")  # 11 S7.3 set_turn_speed.turn_speed closed set


def _milli(radps: float) -> int:
    """rad/s -> integer milli-rad/s (the internal unit)."""
    return int(round(float(radps) * 1000.0))  # 0.6 -> 600


class TurnSpeedSetting:
    """Default-max or explicit turn speed, persisted across P2 restarts."""

    def __init__(self, path: str, min_radps: float, max_radps: float, step_radps: float) -> None:
        """All four from the resolved p2_core.turn_speed block; a bad range is a config error."""
        lo, hi, st = _milli(min_radps), _milli(max_radps), _milli(step_radps)  # internal unit
        if not (0 < lo <= hi and st > 0):  # config error: loud at start
            raise ValueError("turn_speed min %r / max %r / step %r out of range"
                             % (min_radps, max_radps, step_radps))  # names the values
        self._path = str(path)  # absolute path under data/run
        self._lo, self._hi, self._step = lo, hi, st  # bounds and step
        self._lock = threading.Lock()  # guards _explicit and the file write
        self._explicit: Optional[int] = self._load()  # None = never set

    def _load(self) -> Optional[int]:
        """The persisted explicit value (clamped into range), or None."""
        try:  # the file is optional
            with open(self._path, "r", encoding="utf-8") as f:  # small file
                v = json.load(f).get(_KEY)  # the one field
        except FileNotFoundError:  # never set on this robot
            return None  # default applies
        except (OSError, ValueError, AttributeError) as exc:  # truncated / not an object
            _logger.warning("turn speed state %s unreadable (%s); using max", self._path, exc)  # visible
            return None  # default applies
        if isinstance(v, bool) or not isinstance(v, (int, float)):  # foreign content
            _logger.warning("turn speed state %s holds %r; using max", self._path, v)  # visible
            return None  # default applies
        m = _milli(v)  # internal unit
        if not self._lo <= m <= self._hi:  # the config range moved since it was set
            clamped = min(max(m, self._lo), self._hi)  # never above max
            _logger.warning("turn speed state %.3f outside %.3f..%.3f; clamped to %.3f",
                            m / 1000.0, self._lo / 1000.0, self._hi / 1000.0, clamped / 1000.0)  # visible
            return clamped  # in range
        return m  # the operator's value

    def _current(self) -> int:
        """Explicit when set, else the max (caller holds the lock)."""
        return self._hi if self._explicit is None else self._explicit  # explicit wins

    def effective(self) -> float:
        """The turn speed every relmove should use, rad/s."""
        with self._lock:  # vs a concurrent apply()
            return self._current() / 1000.0  # rad/s

    def origin(self) -> str:
        """'explicit' or 'default'."""
        with self._lock:  # vs a concurrent apply()
            return "default" if self._explicit is None else "explicit"  # log / state wording

    def snapshot(self) -> Dict[str, Any]:
        """The 11 S4.3 state/mode.turn_speed object."""
        with self._lock:  # one consistent view
            return {"radps": self._current() / 1000.0, "min_radps": self._lo / 1000.0,  # value + bounds
                    "max_radps": self._hi / 1000.0,  # upper bound
                    "origin": "default" if self._explicit is None else "explicit"}  # where it came from

    def apply(self, level: str) -> Dict[str, Any]:
        """One set_turn_speed: the new value, whether it changed, and which bound it sits on.

        Raises ValueError for a level outside LEVELS (the dispatcher refuses
        those first) and OSError when the file cannot be written; memory
        only follows a durable file."""
        if level not in LEVELS:  # caller bug
            raise ValueError("turn_speed level %r not in %s" % (level, LEVELS))  # loud
        with self._lock:  # one writer at a time
            cur = self._current()  # before
            new = {"faster": min(cur + self._step, self._hi), "slower": max(cur - self._step, self._lo),
                   "max": self._hi, "min": self._lo}[level]  # clamped to [min, max]
            self._write(new)  # durable first
            self._explicit = new  # memory follows the disk
            limit = "max" if new == self._hi else ("min" if new == self._lo else None)  # bound reached
            return {"turn_speed_radps": new / 1000.0, "changed": new != cur, "at_limit": limit}  # ack.applied

    def _write(self, m: int) -> None:
        """Persist m (milli-rad/s) atomically (caller holds the lock)."""
        tmp = self._path + ".new"  # same directory: the rename stays atomic
        with open(tmp, "w", encoding="utf-8") as f:  # fresh temp file
            json.dump({_KEY: m / 1000.0}, f)  # one field, rad/s
            f.flush()  # into the kernel
            os.fsync(f.fileno())  # onto the disk before the rename
        os.replace(tmp, self._path)  # old or new, never half
        dfd = os.open(os.path.dirname(self._path) or ".", os.O_RDONLY)  # the directory entry
        try:  # always close the fd
            os.fsync(dfd)  # the rename survives a power cut
        finally:  # no fd leak
            os.close(dfd)  # released


__all__ = ["TurnSpeedSetting", "LEVELS"]
