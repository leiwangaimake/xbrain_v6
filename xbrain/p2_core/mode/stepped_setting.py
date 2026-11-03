"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: stepped_setting.py
Brief: One operator-adjustable motion speed -- a default, voice steps inside bounds, an explicit value kept across restarts
Description:
What problem this solves
------------------------
Two motion speeds follow the same discipline as the device volume (99 U100
continued): the in-place turn speed (99 U104) and the move speed for forward /
back / strafe (99 U106). Each has a default until somebody sets it; "快一点 /
慢一点 / 最快 / 最慢" moves it by one step inside [min, max]; an explicitly set
value is persisted, restored at the next start, and can be set again any time.
This class is that discipline once; turn_speed.py and move_speed.py only name
the numbers and the wording.

Rules
-----
  * nothing set -> the default; an explicit value is persisted and restored
    at the next start;
  * a stored value outside [min, max] (the config moved since) is CLAMPED
    into range with a warning, never trusted above max;
  * deadzone (user ruling 2026-11-03, "死区抑制"): a speed at or below the
    body's control deadzone does not step, so min must be STRICTLY above it
    -- a config that allows a non-moving speed is refused at construction;
  * arithmetic runs on integer milli-units, so ten "快一点" never drift to
    0.6000000001 and miss the max;
  * the file is written new + fsync + rename + directory fsync (this robot
    loses power uncleanly); an unreadable file falls back to the default.

All numbers come from p2_core.yaml (turn_speed / move_speed); none live in code.
"""
from __future__ import annotations  # postponed annotations

import json  # the state file is one small JSON object
import logging  # load warnings
import os  # atomic write
import threading  # ModeFace and the relmove builder run on different threads
from typing import Any, Dict, Optional  # type hints

_logger = logging.getLogger("xbrain.p2.stepped_setting")  # module logger

LEVELS = ("faster", "slower", "max", "min")  # the 11 S7.3 level closed set (both actions)


def _milli(v: float) -> int:
    """Value -> integer milli-units (the internal unit)."""
    return int(round(float(v) * 1000.0))  # 0.6 -> 600


class SteppedSetting:
    """A default-or-explicit speed, persisted across P2 restarts.

    key names the state-file field and the ack field (e.g. "turn_speed_radps");
    unit is the suffix of the snapshot fields (e.g. "radps" -> radps /
    min_radps / max_radps); label is the English name used in the log.
    """

    def __init__(self, path: str, *, key: str, unit: str, label: str,
                 min_v: float, max_v: float, step: float, default: float,
                 deadzone: float) -> None:
        """All numbers from the resolved p2_core block; a bad range is a config error."""
        lo, hi, st, df, dz = (_milli(min_v), _milli(max_v), _milli(step),
                              _milli(default), _milli(deadzone))  # internal unit
        if not (0 < lo <= df <= hi and st > 0):  # config error: loud at start
            raise ValueError("%s min %r / default %r / max %r / step %r out of range"
                             % (label, min_v, default, max_v, step))  # names the values
        if not (0 <= dz < lo):  # 2026-11-03: a speed at or below the deadzone does not step
            raise ValueError("%s min %r must be above the control deadzone %r"
                             % (label, min_v, deadzone))  # names both values
        self._path = str(path)  # absolute path under data/run
        self._key, self._unit, self._label = key, unit, label  # naming only
        self._lo, self._hi, self._step, self._default = lo, hi, st, df  # bounds, step, default
        self._lock = threading.Lock()  # guards _explicit and the file write
        self._explicit: Optional[int] = self._load()  # None = never set

    def _load(self) -> Optional[int]:
        """The persisted explicit value (clamped into range), or None."""
        try:  # the file is optional
            with open(self._path, "r", encoding="utf-8") as f:  # small file
                v = json.load(f).get(self._key)  # the one field
        except FileNotFoundError:  # never set on this robot
            return None  # default applies
        except (OSError, ValueError, AttributeError) as exc:  # truncated / not an object
            _logger.warning("%s state %s unreadable (%s); using the default",
                            self._label, self._path, exc)  # visible
            return None  # default applies
        if isinstance(v, bool) or not isinstance(v, (int, float)):  # foreign content
            _logger.warning("%s state %s holds %r; using the default",
                            self._label, self._path, v)  # visible
            return None  # default applies
        m = _milli(v)  # internal unit
        if not self._lo <= m <= self._hi:  # the config range moved since it was set
            clamped = min(max(m, self._lo), self._hi)  # never above max
            _logger.warning("%s state %.3f outside %.3f..%.3f; clamped to %.3f", self._label,
                            m / 1000.0, self._lo / 1000.0, self._hi / 1000.0, clamped / 1000.0)  # visible
            return clamped  # in range
        return m  # the operator's value

    def _current(self) -> int:
        """Explicit when set, else the default (caller holds the lock)."""
        return self._default if self._explicit is None else self._explicit  # explicit wins

    def effective(self) -> float:
        """The value every relmove should use."""
        with self._lock:  # vs a concurrent apply()
            return self._current() / 1000.0  # back to the config unit

    def origin(self) -> str:
        """'explicit' or 'default'."""
        with self._lock:  # vs a concurrent apply()
            return "default" if self._explicit is None else "explicit"  # log / state wording

    def snapshot(self) -> Dict[str, Any]:
        """The 11 S4.3 state/mode object: value + bounds + origin."""
        with self._lock:  # one consistent view
            u = self._unit  # field suffix
            return {u: self._current() / 1000.0, "min_" + u: self._lo / 1000.0,  # value + lower bound
                    "max_" + u: self._hi / 1000.0,  # upper bound
                    "origin": "default" if self._explicit is None else "explicit"}  # where it came from

    def apply(self, level: str) -> Dict[str, Any]:
        """One set action: the new value, whether it changed, and which bound it sits on.

        Raises ValueError for a level outside LEVELS (the dispatcher refuses
        those first) and OSError when the file cannot be written; memory
        only follows a durable file."""
        if level not in LEVELS:  # caller bug
            raise ValueError("%s level %r not in %s" % (self._label, level, LEVELS))  # loud
        with self._lock:  # one writer at a time
            cur = self._current()  # before
            new = {"faster": min(cur + self._step, self._hi), "slower": max(cur - self._step, self._lo),
                   "max": self._hi, "min": self._lo}[level]  # clamped to [min, max]
            self._write(new)  # durable first
            self._explicit = new  # memory follows the disk
            limit = "max" if new == self._hi else ("min" if new == self._lo else None)  # bound reached
            return {self._key: new / 1000.0, "changed": new != cur, "at_limit": limit}  # ack.applied

    def _write(self, m: int) -> None:
        """Persist m (milli-units) atomically (caller holds the lock)."""
        tmp = self._path + ".new"  # same directory: the rename stays atomic
        with open(tmp, "w", encoding="utf-8") as f:  # fresh temp file
            json.dump({self._key: m / 1000.0}, f)  # one field, config unit
            f.flush()  # into the kernel
            os.fsync(f.fileno())  # onto the disk before the rename
        os.replace(tmp, self._path)  # old or new, never half
        dfd = os.open(os.path.dirname(self._path) or ".", os.O_RDONLY)  # the directory entry
        try:  # always close the fd
            os.fsync(dfd)  # the rename survives a power cut
        finally:  # no fd leak
            os.close(dfd)  # released


__all__ = ["SteppedSetting", "LEVELS"]
