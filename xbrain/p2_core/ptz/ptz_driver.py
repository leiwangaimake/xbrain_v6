"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptz_driver.py
Brief: ONVIF PTZ primitives (move / stop) + voice E-intent -> pulse-leg plan

Description:
Two jobs, both about the 可见光 camera head behind ONVIF (onvif_client):

1. Camera primitives (PtzOps for ptz_controller): move(pan, tilt, zoom) is ONE
   ContinuousMove carrying all three axes (11 PTZ-C5), stop(pantilt, zoom) is
   one Stop. The media profile token is fetched once and cached.

2. The voice plan: a classified PTZ intent (E01 / E05 / E06 / E07 / E08 /
   E09) becomes a Plan -- a list of timed legs (velocity, ms), a stop, or a
   speed change. The legs are EXECUTED by ptz_controller's tick, not here.

   E01 ptz_move      -> one leg: pan / tilt = dir * speed for the amount archive
   E06 ptz_zoom      -> one leg: zoom = +/- speed for the amount archive
   E07 ptz_scan      -> sweep: three legs right / left (2x) / right; or one long
                        orbit leg ('环视一周')
   E05 / E08 stop    -> stop all axes immediately
   E09 set_ptz_speed -> adjust the velocity magnitude used by E01 / E06 / E07

   A voice move is a TIMED PULSE (18-B E01) because the head is a PELCO-D with
   no absolute positioning (11 PTZ-C1): jogging for a fixed time is how
   '向左一点' becomes a bounded motion.

*** Why the legs are no longer slept here (2026-10-28, Qt contract v0.2 S9):
the old driver ran each pulse as ContinuousMove, time.sleep(pulse), Stop
UNDER the domain lock. A Stop arriving meanwhile waited behind it -- 2.8 s for
a large nudge, 12 s for the orbit -- and pan has no mechanical limit
(11 PTZ-C6). Planning here and running the legs in the controller's tick lets
a cloud or voice stop interrupt any pulse at once.

18-B blocked intents (E02 home / E03 preset / E04 track / E10 move_deg) are
NOT driven -- they sit behind T-PTZ-1 / T-PTZ-3 (absolute positioning and
degree moves are no-ops on this head). The orchestrator rejects them with
E_CAPABILITY before dispatch; if one still arrives, plan() returns a 'drop'
and nothing is sent -- never a fake success.

Threading: move / stop are called by ptz_controller under its lock -- the
ONVIF Session is one keep-alive connection and must not be used concurrently.

Speed / pulse values are initial tuning (18-B E01 pulse archives are fixed;
E09 speed magnitudes wait on T-PTZ-3) -- motion UX values, not common.safety
params, so a code constant with a 'field-tune' note is acceptable here.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import List, Mapping, Optional, Tuple

from xbrain.p2_core.ptz import onvif_client as oc
from xbrain.p2_core.ptz.ptz_controller import PtzOps

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
# T-PTZ-4-like) -- initial value, field-tune. 2026-08-11 ORIN ask: '环视'
# should be a full turn, not a small sweep.
_ORBIT_MS = 12000

Vel = Tuple[float, float, float]


@dataclass(frozen=True)
class Plan:
    """What a voice intent asks the head to do.

    kind: 'legs' (run `legs` in order), 'stop', 'speed' (nothing to send;
    the speed level already changed) or 'drop' (unknown / blocked / missing
    slot -- nothing is sent). note: a short outcome for the log."""
    kind: str
    legs: List[Tuple[Vel, int]] = field(default_factory=list)
    note: str = ""


@dataclass
class PtzDriverConfig:
    """All fields required at construction (no defaults for the endpoint)."""
    host: str                  # '192.168.66.13' or host:port
    user: str
    pwd: str
    ptz_path: str = "/onvif/ptz"
    media_path: str = "/onvif/media"


class PtzDriver(PtzOps):
    """ONVIF primitives + voice planning. Built once; holds one Session."""

    def __init__(self, cfg: PtzDriverConfig) -> None:
        self._cfg = cfg
        self._sess = oc.OnvifSession(cfg.host, cfg.user, cfg.pwd)
        self._token: Optional[str] = None
        self._speed_level = "normal"       # E09 default (open-boot value)
        self.calls_dropped = 0             # observability: intents not driven

    def _ensure_token(self) -> str:
        """Fetch the media profile token once (ContinuousMove needs it)."""
        if self._token is None:
            self._token = oc.get_profile_token(self._sess, self._cfg.media_path)
        if not self._token:
            raise oc.OnvifError("no ONVIF media profile token")
        return self._token

    # -- PtzOps (called by ptz_controller under its lock) ----------------

    def move(self, pan: float, tilt: float, zoom: float) -> None:
        """One ContinuousMove with all three axes (11 PTZ-C5)."""
        oc.ptz_continuous(self._sess, self._cfg.ptz_path, self._ensure_token(),
                          pan=pan, tilt=tilt, zoom=zoom)

    def stop(self, pantilt: bool, zoom: bool) -> None:
        """One Stop for the named axis groups (ONVIF groups pan + tilt)."""
        oc.ptz_stop(self._sess, self._cfg.ptz_path, self._ensure_token(),
                    pantilt=pantilt, zoom=zoom)

    # -- voice planning --------------------------------------------------

    def plan(self, intent_id: str, slots: Mapping[str, object]) -> Plan:
        """Route a PTZ voice intent to a Plan (nothing is sent here)."""
        if intent_id == "E01":
            return self._move(slots)
        if intent_id == "E06":
            return self._zoom(slots)
        if intent_id in ("E05", "E08"):
            return Plan("stop", note="stop")
        if intent_id == "E07":
            return self._scan(slots)
        if intent_id == "E09":
            return self._set_speed(slots)
        # E02/E03/E04/E10 are capability-blocked (rejected upstream); if one
        # reaches here, drop it -- never fake a success on the head.
        return self._drop("ptz intent %r not driven (blocked or unknown)"
                          % intent_id)

    def _drop(self, why: str) -> Plan:
        """Count + log an intent that sends nothing."""
        self.calls_dropped += 1
        _logger.warning("%s; dropped", why)
        return Plan("drop", note=why)

    def _speed(self) -> float:
        """Velocity magnitude of the current E09 level."""
        return _SPEED[self._speed_level]

    def _move(self, slots: Mapping[str, object]) -> Plan:
        """E01: one pan / tilt leg for the amount archive."""
        direction = slots.get("direction")
        if direction not in _DIR_VEC:
            return self._drop("ptz E01 no/bad direction %r" % direction)
        amount = slots.get("amount", "normal")
        if amount not in _PULSE_MS:
            return self._drop("ptz E01 bad amount %r" % amount)
        px, ty = _DIR_VEC[direction]
        spd = self._speed()
        return Plan("legs", [((px * spd, ty * spd, 0.0), _PULSE_MS[amount])],
                    note="move %s %s" % (direction, amount))

    def _zoom(self, slots: Mapping[str, object]) -> Plan:
        """E06: one zoom leg for the amount archive."""
        zdir = slots.get("zoom_dir")
        if zdir not in ("in", "out"):
            return self._drop("ptz E06 no/bad zoom_dir %r" % zdir)
        amount = slots.get("amount", "normal")
        if amount not in _PULSE_MS:
            return self._drop("ptz E06 bad amount %r" % amount)
        z = self._speed() if zdir == "in" else -self._speed()
        return Plan("legs", [((0.0, 0.0, z), _PULSE_MS[amount])],
                    note="zoom %s %s" % (zdir, amount))

    def _scan(self, slots: Mapping[str, object]) -> Plan:
        """E07: a bounded left-right sweep, or a full-circle orbit."""
        spd = self._speed()
        if slots.get("scan_mode", "sweep") == "orbit":
            pan = spd if slots.get("direction", "left") == "right" else -spd
            return Plan("legs", [((pan, 0.0, 0.0), _ORBIT_MS)], note="orbit")
        # sweep: right, left (long), right (back toward centre)
        return Plan("legs", [((spd, 0.0, 0.0), _SCAN_LEG_MS),
                             ((-spd, 0.0, 0.0), _SCAN_LEG_MS * 2),
                             ((spd, 0.0, 0.0), _SCAN_LEG_MS)], note="sweep")

    def _set_speed(self, slots: Mapping[str, object]) -> Plan:
        """E09: absolute level or one step up / down."""
        level = slots.get("level")
        if level in _SPEED:
            self._speed_level = level
        elif level in ("up", "down"):
            i = _SPEED_ORDER.index(self._speed_level)
            i = min(i + 1, len(_SPEED_ORDER) - 1) if level == "up" else max(i - 1, 0)
            self._speed_level = _SPEED_ORDER[i]
        else:
            return self._drop("ptz E09 no/bad level %r" % level)
        _logger.info("ptz E09 speed -> %s", self._speed_level)
        return Plan("speed", note="speed:" + self._speed_level)

    def close(self) -> None:
        """Best-effort Stop, then drop the connection."""
        try:
            self.stop(True, True)
        except Exception:            # noqa: BLE001 -- shutting down anyway
            pass
        self._sess.close()
