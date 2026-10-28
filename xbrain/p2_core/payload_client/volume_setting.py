"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: volume_setting.py
Brief: The device playback volume P2 restores at start -- max by default, the operator's explicit D10 value once set, kept across restarts

Description:
What problem this solves
------------------------
The GZH-2 speaker has ONE global volume register ([14], 11 S7.5 volume_pct,
18 S6.4) shared by broadcast (B), alarm and D-mode siren / TTS. The unit has
no [99] readback, so P2 never knows the device's value -- it only knows what
it last WROTE. Before this file:
  * 99 U100 (first form) wrote boot_volume_pct (100) on every P2 start, so an
    operator who said "volume 60" got 100 back after any restart;
  * payload_wiring tracked the last-set volume from a hard 50, so the first
    "louder" after a start computed 50 + step and turned a 100 device DOWN.
User ruling 2026-10-28 (99 U100 continued): broadcast, alarm and D mode are at
max volume by default; once a voice or text command sets the volume
explicitly, that value stays, even across a system restart.

What it does
------------
  * effective(): the explicit value when one was recorded, else the default
    (payload_svc.boot_volume_pct). P2 writes it to the device once after
    start, and D10 relative steps ("louder") start from it.
  * record(pct): called after a D10 POST /volume succeeded. Persists pct to
    one small JSON file (payload_svc.volume_state_path) with write-new +
    fsync + rename + directory fsync: this robot loses power uncleanly
    often, and a plain overwrite leaves a 0-byte file after a cut.

What it does NOT do
-------------------
It sends nothing to the device (main_wiring / payload_wiring do) and it does
not decide WHAT counts as explicit (only a successful D10 does; the boot write
of the default is not a setting and is never recorded).

Looks right but is wrong
------------------------
  * Treating an unreadable file as "fail start": a volume file is not a
    safety parameter (CLAUDE.md 3.1 covers common.safety / spec); refusing to
    start P2 over it would silence alarms entirely. An unreadable or
    out-of-range file is logged at WARNING and the DEFAULT (max) is used --
    the failure direction is "too loud", never "silent".
  * Recording the default at boot: then the default could never change by
    config again, every robot would look "explicitly set" from day one.
  * 0 is a legal explicit value ("mute", 18-A D10): it IS persisted and
    restored. 18 S6.4 lesson 3 warns that a lowered volume does not recover
    by itself; per the 2026-10-28 ruling that is now intended, and P2 logs
    the restored value and its origin at every start so it is visible.
"""

from __future__ import annotations  # postponed annotations

import json  # the state file is one small JSON object
import logging  # restore / failure lines
import os  # atomic write: fsync + replace
import threading  # record() and effective() run on different threads
from typing import Optional  # type hints only

_logger = logging.getLogger("xbrain.p2.volume")  # module logger

VOL_MIN, VOL_MAX = 0, 100  # 18 S6.4 / 11 S7.5 volume_pct range
_KEY = "volume_pct"  # the one field in the state file


def _valid(pct) -> bool:  # range + type check shared by load and record
    """True for an int (not bool) in [0, 100]."""
    return isinstance(pct, int) and not isinstance(pct, bool) and VOL_MIN <= pct <= VOL_MAX  # bool is an int subclass


class VolumeSetting:
    """Default-or-explicit device volume, persisted across P2 restarts."""

    def __init__(self, path: str, default_pct: int) -> None:
        """path: payload_svc.volume_state_path; default_pct:
        payload_svc.boot_volume_pct. Both from the resolved snapshot, no
        code default (a bad default is a config error and raises here)."""
        if not _valid(default_pct):  # config error: loud at start
            raise ValueError("boot_volume_pct %r not an int in 0..100" % (default_pct,))  # names the bad value
        self._path = str(path)  # absolute path under data/run
        self._default = default_pct  # used while nothing was set explicitly
        self._lock = threading.Lock()  # guards _explicit and the file write
        self._explicit: Optional[int] = self._load()  # None = never set

    def _load(self) -> Optional[int]:  # read once at construction
        """The persisted explicit value, or None (no file / unusable file)."""
        try:
            with open(self._path, "r", encoding="utf-8") as f:  # small file
                pct = json.load(f).get(_KEY)  # the one field
        except FileNotFoundError:  # never set on this robot: the normal case
            return None  # default applies
        except (OSError, ValueError, AttributeError) as exc:  # truncated / not an object / unreadable
            _logger.warning("volume state %s unreadable (%s); using default %d",
                            self._path, exc, self._default)  # visible, not fatal
            return None  # fail loud-side: default (max)
        if not _valid(pct):  # hand-edited or foreign content
            _logger.warning("volume state %s holds %r, not 0..100; using default %d",
                            self._path, pct, self._default)  # visible, not fatal
            return None  # default applies
        return pct  # the operator's value

    def effective(self) -> int:  # what the device should be at
        """Explicit value when one was recorded, else the default."""
        with self._lock:  # vs a concurrent record()
            return self._default if self._explicit is None else self._explicit  # explicit wins

    def origin(self) -> str:  # for the start log line
        """'explicit' or 'default' -- which of the two effective() returned."""
        with self._lock:  # vs a concurrent record()
            return "default" if self._explicit is None else "explicit"  # log wording

    def record(self, pct: int) -> None:  # after a successful D10 POST /volume
        """Persist pct as the explicit volume. Raises ValueError on a bad
        pct (caller bug) and OSError when the file cannot be written; the
        in-memory value is only updated after the file is durable."""
        if not _valid(pct):  # caller bug: never persist garbage
            raise ValueError("volume %r not an int in 0..100" % (pct,))  # loud
        with self._lock:  # one writer at a time
            tmp = self._path + ".new"  # same directory: rename stays atomic
            with open(tmp, "w", encoding="utf-8") as f:  # fresh temp file
                json.dump({_KEY: pct}, f)  # one field
                f.flush()  # into the kernel
                os.fsync(f.fileno())  # onto the disk before the rename
            os.replace(tmp, self._path)  # POSIX rename(2): old or new, never half
            dfd = os.open(os.path.dirname(self._path) or ".", os.O_RDONLY)  # the directory entry
            try:
                os.fsync(dfd)  # make the rename itself survive a power cut
            finally:
                os.close(dfd)  # no fd leak
            self._explicit = pct  # memory follows the disk


__all__ = ["VolumeSetting", "VOL_MIN", "VOL_MAX"]  # public surface
