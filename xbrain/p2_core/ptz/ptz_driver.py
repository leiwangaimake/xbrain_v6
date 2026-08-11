"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptz_driver.py
Brief: E-intent -> ONVIF PTZ device control (move pulse / zoom / stop / scan)

Description:
Turns a classified PTZ intent (E01/E05/E06/E07/E08/E09) into ONVIF
ContinuousMove/Stop on the 可见光 camera (onvif_client). A voice move is a
TIMED PULSE (18-B E01): ContinuousMove in the requested direction, sleep the
pulse length picked by the amount archive (350/1000/2800 ms), then Stop --
because the head is a PELCO-D with no absolute positioning (report S4:
absolute is a no-op, the T-PTZ-1 wall), so jogging for a fixed time is how
'向左一点' becomes a bounded motion.

  E01 ptz_move      -> ContinuousMove(pan/tilt = dir * speed) pulse, Stop
  E06 ptz_zoom      -> ContinuousMove(zoom = +/- speed) pulse, Stop
  E07 ptz_scan      -> a bounded left-right sweep, then Stop
  E05/E08 stop      -> Stop immediately
  E09 set_ptz_speed -> adjust the velocity magnitude used by E01/E06

18-B blocked intents (E02 home / E03 preset / E04 track / E10 move_deg) are
NOT driven here -- they sit behind T-PTZ-1 / T-PTZ-3 (absolute positioning /
degree move are no-ops on this head). The orchestrator rejects them with
E_CAPABILITY before dispatch; if one still reaches here it is logged and
dropped, never sent as a fake success.

Threading: called from a p2 worker thread (the cmd/ptz callback trampoline),
serialised under the caller's lock -- the ONVIF Session is one keep-alive
connection and must not be used concurrently. A move holds the driver for
its pulse (<= 2800 ms); voice commands are seconds apart so this is fine.

Speed/pulse values are initial tuning (18-B E01 pulse archives are fixed;
E09 speed magnitudes wait on T-PTZ-3) -- motion UX values, not common.safety
params, so a code constant with a 'field-tune' note is acceptable here.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Mapping, Optional

from xbrain.p2_core.ptz import onvif_client as oc


_logger = logging.getLogger("xbrain.p2.ptz")


# 18-B E01: three pulse archives (ms) picked by the amount slot.
_PULSE_MS = {"small": 350, "normal": 1000, "large": 2800}
# E09 speed magnitude per level (initial; T-PTZ-3). ptzkey2 ran a fixed 1.0.
_SPEED = {"slow": 0.3, "normal": 0.6, "fast": 1.0}
_SPEED_ORDER = ("slow", "normal", "fast")
# E01 direction -> (pan, tilt) unit sign.
_DIR_VEC = {"left": (-1.0, 0.0), "right": (1.0, 0.0),
            "up": (0.0, 1.0), "down": (0.0, -1.0)}
# E07 sweep: right, left (2x), right (2x), stop -- a bounded look-around,
# not the device's built-in cruise (18-B E07: 上装发脉冲).
_SCAN_LEG_MS = 1500
# E07 orbit ('环视一周'): one long single-direction pan approximating a full
# circle. The exact 360-degree duration depends on the pan rate (uncalibrated,
# T-PTZ-4-like) and the head may hit a pan limit before a full turn -- initial
# value, field-tune. 2026-08-11 ORIN ask: '环视' should be a full turn, not a
# small sweep.
_ORBIT_MS = 12000


@dataclass
class PtzDriverConfig:
    """All fields required at construction (no defaults for the endpoint)."""
    host: str                  # '192.168.66.13' or host:port
    user: str
    pwd: str
    ptz_path: str = "/onvif/ptz"
    media_path: str = "/onvif/media"


class PtzDriver:
    """Drives the camera PTZ over ONVIF. Built once; holds one Session."""

    def __init__(self, cfg: PtzDriverConfig) -> None:
        self._cfg = cfg
        self._sess = oc.OnvifSession(cfg.host, cfg.user, cfg.pwd)
        self._token: Optional[str] = None
        self._speed_level = "normal"       # E09 default (open-boot value)
        # Observability.
        self.calls_made = 0
        self.calls_dropped = 0
        self.errors: list = []

    def _ensure_token(self) -> Optional[str]:
        """Fetch the media profile token once (ContinuousMove needs it)."""
        if self._token is None:
            self._token = oc.get_profile_token(self._sess, self._cfg.media_path)
        return self._token

    def _speed(self) -> float:
        return _SPEED[self._speed_level]

    def handle(self, intent_id: str, slots: Mapping[str, object]) -> str:
        """Route a PTZ intent. Returns a short outcome string for the log."""
        try:
            if intent_id == "E01":
                return self._move(slots)
            if intent_id == "E06":
                return self._zoom(slots)
            if intent_id in ("E05", "E08"):
                return self._stop()
            if intent_id == "E07":
                return self._scan(slots)
            if intent_id == "E09":
                return self._set_speed(slots)
            # E02/E03/E04/E10 are capability-blocked (rejected upstream); if
            # one reaches here, drop it -- never fake a success on the head.
            self.calls_dropped += 1
            _logger.warning("ptz intent %r not driven (blocked or unknown); "
                            "dropped", intent_id)
            return "dropped"
        except oc.OnvifError as exc:
            self.errors.append(str(exc))
            _logger.error("ptz onvif call failed: %s", exc)
            return "error"

    # -- motions ---------------------------------------------------------

    def _pulse(self, *, pan: float, tilt: float, zoom: float,
               pulse_ms: int) -> None:
        """One timed jog: ContinuousMove, hold pulse_ms, Stop."""
        token = self._ensure_token()
        if not token:
            raise oc.OnvifError("no ONVIF media profile token")
        oc.ptz_continuous(self._sess, self._cfg.ptz_path, token,
                          pan=pan, tilt=tilt, zoom=zoom)
        # A fixed sleep duration (not a clock read); Stop bounds the motion.
        time.sleep(pulse_ms / 1000.0)
        oc.ptz_stop(self._sess, self._cfg.ptz_path, token)

    def _move(self, slots: Mapping[str, object]) -> str:
        direction = slots.get("direction")
        if direction not in _DIR_VEC:
            self.calls_dropped += 1
            _logger.warning("ptz E01 no/bad direction %r; dropped", direction)
            return "no_direction"
        px, ty = _DIR_VEC[direction]
        spd = self._speed()
        pulse = _PULSE_MS[slots.get("amount", "normal")]
        self._pulse(pan=px * spd, tilt=ty * spd, zoom=0.0, pulse_ms=pulse)
        self.calls_made += 1
        _logger.info("ptz E01 move %s amount=%s speed=%s pulse=%dms",
                     direction, slots.get("amount", "normal"),
                     self._speed_level, pulse)
        return "move"

    def _zoom(self, slots: Mapping[str, object]) -> str:
        zdir = slots.get("zoom_dir")
        if zdir not in ("in", "out"):
            self.calls_dropped += 1
            _logger.warning("ptz E06 no/bad zoom_dir %r; dropped", zdir)
            return "no_zoom_dir"
        z = self._speed() if zdir == "in" else -self._speed()
        pulse = _PULSE_MS[slots.get("amount", "normal")]
        self._pulse(pan=0.0, tilt=0.0, zoom=z, pulse_ms=pulse)
        self.calls_made += 1
        _logger.info("ptz E06 zoom %s amount=%s pulse=%dms",
                     zdir, slots.get("amount", "normal"), pulse)
        return "zoom"

    def _stop(self) -> str:
        token = self._ensure_token()
        if token:
            oc.ptz_stop(self._sess, self._cfg.ptz_path, token)
        self.calls_made += 1
        _logger.info("ptz stop")
        return "stop"

    def _scan(self, slots: Mapping[str, object]) -> str:
        """E07: a bounded left-right sweep, or a full-circle orbit in one
        direction ('环视一周'). Not interruptible mid-motion in this simple
        form (a following E08 stop runs after)."""
        spd = self._speed()
        mode = slots.get("scan_mode", "sweep")
        if mode == "orbit":
            # One long single-direction pan approximating a full turn.
            direction = slots.get("direction", "left")
            pan = spd if direction == "right" else -spd
            self._pulse(pan=pan, tilt=0.0, zoom=0.0, pulse_ms=_ORBIT_MS)
            self.calls_made += 1
            _logger.info("ptz E07 orbit %s (~full circle) done", direction)
            return "orbit:" + str(direction)
        # sweep: right, left(long), right(back toward center)
        self._pulse(pan=spd, tilt=0.0, zoom=0.0, pulse_ms=_SCAN_LEG_MS)
        self._pulse(pan=-spd, tilt=0.0, zoom=0.0, pulse_ms=_SCAN_LEG_MS * 2)
        self._pulse(pan=spd, tilt=0.0, zoom=0.0, pulse_ms=_SCAN_LEG_MS)
        self.calls_made += 1
        _logger.info("ptz E07 scan sweep done")
        return "scan"

    def _set_speed(self, slots: Mapping[str, object]) -> str:
        level = slots.get("level")
        if level in _SPEED:
            self._speed_level = level
        elif level == "up":
            i = _SPEED_ORDER.index(self._speed_level)
            self._speed_level = _SPEED_ORDER[min(i + 1, len(_SPEED_ORDER) - 1)]
        elif level == "down":
            i = _SPEED_ORDER.index(self._speed_level)
            self._speed_level = _SPEED_ORDER[max(i - 1, 0)]
        else:
            self.calls_dropped += 1
            _logger.warning("ptz E09 no/bad level %r; dropped", level)
            return "no_level"
        _logger.info("ptz E09 speed -> %s", self._speed_level)
        return "speed:" + self._speed_level

    def close(self) -> None:
        try:
            self._stop()
        except Exception:            # noqa: BLE001
            pass
        self._sess.close()
