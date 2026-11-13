"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: alarm_wiring.py
Brief: 99 U188 -- feeds the warning-zone autonomous alarm and carries out what it decides

Description:
What problem this solves
------------------------
suspicion/auto_alarm.py decides; this file gives it its inputs and acts on
its answers, so the judgment stays free of sessions and clocks (testable).

Inputs (all latest-value slots; the Rust-thread callbacks only store, CLAUDE.md 4.2)
  * cmd/fence (GEN, P3): the enabled warning polygons and the "alarm" block
    (window, watched targets, siren scalars; 11 S9A.2, 99 U187). The only
    geometry source P2 may use (11 S9A.13 SR-1).
  * state/task (GEN, P3): current.type -- is a patrol / navigation running.
    New consumer, 11 S2.2 row (99 U188).
  * xbrain/{rid}/rt/perception/objects (RT, perception): the tracked objects
    in base_link (11 S3.1B.2). New consumer, 11 S2.2.1 row (99 U188). An
    object that does not parse is skipped on its own -- here a dropped object
    is a missed alarm, not a safety hazard (p1's strict whole-message reject
    exists for the RNS consume face, not for this one).
  * state/pose and state/clock: the cache main_wiring already keeps.

Outputs
  * event/alarm/intrusion on a fire (11 S9A.13 SR-8: category intrusion,
    severity alarm), detail = the judgment's detail plus fence_set_id / rev
    and the deterrent outcome.
  * event/warn/intrusion "zone_rules_suspended" and event/info/intrusion
    "zone_rules_resumed" (heading or position lost while in a warning zone
    during a patrol / navigation), and "alarm_clock_unsynced" (window judged
    as all day; once per unsynced episode). The plan's event table.
  * d_mode/auto_deter.py: siren + red/blue for duration_sec, then the spoken
    warning, the system mode untouched (the robot keeps moving, user ruling
    "不停车, 也响警笛").

Local time for the window: the zone the robot is in has a GPS fix, so the
time zone comes from it (tzfpy, lon first, the (0, 0) guard -- same rules as
p5_gateway/hmi/geo_timezone.py; that module belongs to the P5 process, so the
dozen lines are repeated here rather than imported across processes). No
tzfpy or no zone -> local minute unknown -> all day, the same answer as an
unsynced clock.

What it does NOT do
-------------------
No judgment, no device sequencing, no system-mode change, no announcement by
P4 (the deterrent speaks itself). Without XBRAIN_ROBOT_ID the RT key cannot
be formed: no objects subscription, logged once, every patrol then judges an
empty scene (and says nothing) -- the same visible-absence rule as the
broadcast receiver in main_wiring.
"""

from __future__ import annotations  # postponed annotations

import json  # wire bodies
import logging  # English-only log text (CLAUDE.md 2.1)
import os  # eid boot token
import threading  # slot lock
import time  # monotonic ages; the one wall reading below
from typing import Any, Callable, Dict, List, Optional, Tuple  # hints

from xbrain.p2_core.suspicion.auto_alarm import (  # judgment
    AlarmInputs,
    AutoAlarm,
    Target,
    Zone,
)

_logger = logging.getLogger("xbrain.p2.auto_alarm")  # P2 logger tree

#: GEN keys (relative, bus convention) and the RT key template
FENCE_TOPIC = "cmd/fence"  # P3 FenceSet (11 S9A.2)
TASK_TOPIC = "state/task"  # P3 TaskState (11 S4.4)
OBJECTS_KEY = "xbrain/%s/rt/perception/objects"  # perception (11 S3.1B.0), absolute like p1

try:  # Rust wheel; a host without it judges all day (see the module docstring)
    from tzfpy import get_tz as _get_tz  # get_tz(lon, lat) -> IANA name
except ImportError:  # pragma: no cover - depends on the host
    _get_tz = None  # no zone lookup available


def _unwrap(raw: bytes) -> Optional[Dict[str, Any]]:
    """JSON body, the 11 S3.0 envelope's data when present; None if unreadable."""
    try:
        body = json.loads(bytes(raw).decode("utf-8"))  # one sample
    except (ValueError, UnicodeDecodeError):  # garbage: ignored
        return None  # the previous value stays
    if isinstance(body, dict) and isinstance(body.get("data"), dict):  # enveloped
        return body["data"]  # the S3.0 body
    return body if isinstance(body, dict) else None  # bare (P3 puts FenceSet bare)


def parse_zones(fs: Optional[Dict[str, Any]]) -> Tuple[List[Zone], Dict[str, Any]]:
    """cmd/fence FenceSet -> (enabled warning zones, alarm block). The set holds
    the ACTIVE polygons only (P3 builds it from list_active)."""
    if not fs:  # nothing received yet
        return [], {}  # no zones, no settings
    zones = []  # warning polygons
    for p in fs.get("polygons") or []:  # every polygon
        if not isinstance(p, dict) or p.get("role") != "warning":  # other roles: P1's business
            continue  # skip it
        try:
            verts = tuple((float(v["lat"]), float(v["lon"])) for v in p["vertices"])  # WGS84
        except (KeyError, TypeError, ValueError):  # a malformed polygon is skipped
            continue  # skip it, the others still count
        if len(verts) >= 3:  # a polygon
            zones.append(Zone(str(p.get("poly_id")), str(p.get("name") or ""), verts))
    alarm = fs.get("alarm") if isinstance(fs.get("alarm"), dict) else {}  # 99 U187 block
    return zones, alarm


def parse_targets(body: Optional[Dict[str, Any]]) -> List[Target]:
    """rt/perception/objects body -> targets; footprint centroid in base_link."""
    out: List[Target] = []  # parsed objects
    for o in (body or {}).get("objects") or []:  # each object
        try:
            fp = o.get("footprint_xy")  # ground hull or None (not localised)
            xy = None if fp is None else (sum(float(p[0]) for p in fp) / len(fp),
                                         sum(float(p[1]) for p in fp) / len(fp))  # centroid
            out.append(Target(int(o["track_id"]), str(o["class_name"]),
                              float(o["confidence"]), xy))
        except (AttributeError, KeyError, TypeError, ValueError, ZeroDivisionError):
            continue  # this object only
    return out  # possibly empty


def local_minute(lat: Optional[float], lon: Optional[float], utc_s: float) -> Optional[int]:
    """Minutes since local midnight at (lat, lon), or None when the zone is unknown."""
    if _get_tz is None or lat is None or lon is None or (lat == 0.0 and lon == 0.0):
        return None  # no lookup / no fix / a zeroed pose (tzfpy maps it to Etc/GMT)
    name = _get_tz(lon, lat)  # LON first
    if not name:  # open sea / no answer
        return None  # no zone name: unknown
    from datetime import datetime  # noqa: PLC0415 only here
    from zoneinfo import ZoneInfo  # noqa: PLC0415 only here
    t = datetime.fromtimestamp(utc_s, ZoneInfo(name))  # the site's local time
    return t.hour * 60 + t.minute  # minutes


class AlarmWiring:
    """Owned by the P2 main loop; subscribe() once, tick() every pass."""

    def __init__(self, engine: AutoAlarm, deter, gen, rt, rid: Optional[str],
                 state_cache: Dict[str, Any], rx_mono: Dict[str, float]) -> None:
        self._engine = engine  # suspicion/auto_alarm.AutoAlarm
        self._deter = deter  # d_mode/auto_deter.AutoDeter
        self._gen, self._rt, self._rid = gen, rt, rid  # sessions and robot id
        self._cache, self._rx = state_cache, rx_mono  # pose / clock from main_wiring
        self._subs: List[Any] = []  # long-lived handles (CLAUDE.md 4.3)
        self._lock = threading.Lock()  # the three slots below
        self._slots: Dict[str, Tuple[float, Optional[Dict[str, Any]]]] = {}  # name -> (rx mono, body)
        self._boot = os.urandom(3).hex()  # eid token: seq restarts, record.db does not
        self._seq = 0  # eid sequence

    def _store(self, name: str) -> Callable[[Any], None]:
        """A Rust-thread callback that only stores the latest body."""
        def _cb(sample) -> None:
            body = _unwrap(bytes(sample.payload))  # decode
            if body is not None:  # unreadable samples keep the previous value
                with self._lock:
                    self._slots[name] = (time.monotonic(), body)  # P2-domain arrival time
        return _cb  # one per slot name

    def subscribe(self) -> None:
        """Declare the three subscribers; the handles live as long as this object."""
        self._subs.append(self._gen.declare_subscriber(FENCE_TOPIC, self._store("fence")))
        self._subs.append(self._gen.declare_subscriber(TASK_TOPIC, self._store("task")))
        if self._rid:  # the RT key needs the robot id
            self._subs.append(self._rt.declare_subscriber(OBJECTS_KEY % self._rid,
                                                          self._store("objects")))
        else:  # visible absence, see the module docstring
            _logger.error("p2 auto alarm: XBRAIN_ROBOT_ID unset, rt/perception/objects not "
                          "subscribed -- no target will ever be judged")
        _logger.info("p2 wiring: auto alarm subscribed %s, %s, objects=%s",
                     FENCE_TOPIC, TASK_TOPIC, bool(self._rid))

    def _slot(self, name: str, now: float) -> Tuple[Optional[float], Optional[Dict[str, Any]]]:
        """(age s, body) of one slot; (None, None) when never received."""
        with self._lock:
            got = self._slots.get(name)  # latest
        return (None, None) if got is None else (now - got[0], got[1])  # age from arrival

    def _publish(self, sev: str, title: str, detail: Dict[str, Any], wall_s: float) -> None:
        """One event on the relative event/{sev}/intrusion key (p5 persists it)."""
        self._seq += 1  # unique per boot
        self._gen.put("event/%s/intrusion" % sev, json.dumps({
            "eid": "alarm-%s-%d" % (self._boot, self._seq), "title": title,
            "dedup_window_s": 0,  # each alarm is its own row; the cooldown already limits them
            "detail": detail, "src": "p2_core", "ts": wall_s},
            ensure_ascii=False).encode("utf-8"))

    def tick(self, now_ms: int, system_idle: bool, system_is_alarm: bool, mapped: str) -> None:
        """One main-loop pass: judge, then act. mapped = the system mode's device mode."""
        now = now_ms / 1000.0  # monotonic seconds
        self._deter.tick(now_ms, system_idle, system_is_alarm, mapped)  # stop when due
        _, fs = self._slot("fence", now)  # FenceSet; its age is not judged (P3 re-sends it)
        _, ts = self._slot("task", now)  # TaskState; same
        obj_age, ob = self._slot("objects", now)  # ObjectsMsg
        zones, alarm = parse_zones(fs)  # enabled warning zones + settings
        cur = (ts or {}).get("current") if isinstance((ts or {}).get("current"), dict) else None
        pose = self._cache.get("pose")  # state/pose data
        pose_rx = self._rx.get("pose")  # its arrival (P2 domain)
        clock = self._cache.get("clock") or {}  # state/clock
        # WALL-CLOCK-OK(record): 11 S6.1 Event.ts of what this tick emits; the 11 SR-5 calendar window reads the same value
        wall_s = time.time()
        synced = clock.get("sync") is True  # CLK-A3: absent = unsynced
        p = pose or {}  # for the zone lookup
        acts = self._engine.step(AlarmInputs(
            now_s=now, task_type=(cur or {}).get("type"), pose=pose,
            pose_age_s=None if pose_rx is None else now - pose_rx,
            targets=parse_targets(ob), objects_age_s=obj_age, zones=zones, alarm=alarm,
            clock_synced=synced,
            local_minute=local_minute(p.get("lat"), p.get("lon"), wall_s) if synced else None))
        for a in acts:  # usually none
            d = dict(a.detail, fence_set_id=(fs or {}).get("fence_set_id"), rev=(fs or {}).get("rev"))
            if a.kind == "fire":  # alarm: deter first so the event says what happened
                d["deterrent"] = self._deter.start(now_ms, a.detail.get("siren_level"),
                                                   a.detail["duration_sec"], system_idle)
                _logger.warning("p2 auto alarm in %s: %s (%s)", d["zone_id"],
                                [t["cls"] for t in d["targets"]], d["deterrent"])
                self._publish("alarm", "zone_intrusion", d, wall_s)
            elif a.kind == "suspend":  # heading / position lost in a warning zone
                _logger.warning("p2 auto alarm suspended: %s", d.get("reason"))
                self._publish("warn", "zone_rules_suspended", d, wall_s)
            elif a.kind == "resume":  # usable again (or the patrol ended)
                _logger.info("p2 auto alarm resumed (was %s)", d.get("reason"))
                self._publish("info", "zone_rules_resumed", d, wall_s)
            elif a.kind == "clock_unsynced":  # window judged as all day
                _logger.warning("p2 auto alarm: clock not synchronised, window treated as all day")
                self._publish("warn", "alarm_clock_unsynced", d, wall_s)


__all__ = ["AlarmWiring", "local_minute", "parse_targets", "parse_zones"]  # public surface
