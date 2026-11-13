"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: auto_alarm.py
Brief: 99 U188 -- autonomous alarm in the warning zones: who is inside the zone the robot is in

Description:
What problem this solves
------------------------
The warning zones (fence role "warning", 报警区域) had geometry, settings and
announcements, but nothing ever raised an alarm on what the camera saw in
them: the old suspicion slice (hysteresis.py / target_track.py /
rules_loader.py) was never wired, and its input state/targets has no
producer any more (perception publishes rt/perception/objects in base_link,
11 S3.1B). This module is the judgment; runtime/alarm_wiring.py feeds it and
carries out what it returns.

The user's rules (99 U188, 2026-11-13; the plan's section 7 as amended)
------------------------------------------------------------------------
All of these at once, or nothing is judged:
  * a path patrol or a point-to-point navigation is running ("对路径巡逻和
    PTP导航只要进入到报警区域内都要支持自主报警"): the task types come from
    config (auto_alarm.task_types), not from this file;
  * the robot itself is inside an enabled warning zone ("机器人不在报警区内时,
    对报警区里的情况不做任何判断");
  * inside the time window (the cloud's alarm_window; none = all day; a clock
    that is not synchronised = all day, reported once);
  * position and heading usable -- otherwise the zone rules are suspended and
    that is reported (start and recovery, one event each).
Then: a target of a watched class (the cloud's rules, default people) inside
THAT zone -- not anywhere in view -- for enter_persist_s raises the alarm,
at most once per cooldown per zone ("按冷却时间限频").

Target classes (user ruling): 人 = person / bicycle / motorcycle (a rider is a
person), 车 = the vehicles, 动物 = animals but not birds; whether an animal is
reported depends on what the perception model recognises. The class names
per group are p2_core.yaml auto_alarm.class_groups -- the model's own names,
so a model change is a config change. A group that lists "bird" is refused at
construction (the user ruled birds out).

Geometry
--------
Local metres around the robot (equirectangular; the zones are tens of metres,
the error is far below the band width). A target's ground position is the
centroid of its base_link footprint rotated by heading_rad (11 S3.1B / the
11 wpos formula: ENU, east = 0, counter-clockwise positive):
    E = x cos(th) - y sin(th),  N = x sin(th) + y cos(th)
The pose is the latest state/pose, not the pose at the capture instant (11
W-1..W-3 asked perception for an interpolated wpos, which it does not
publish). The error is speed x (pose age + perception latency), about 0.3 m
at 1 m/s -- inside the hysteresis band; this is an alarm, not a stop.

Hysteresis (hysteresis.py, 11 S9A.15 CAL-12): signed distance d (positive
inside), h = k_h * cov_h_m + h_min_m. A track is "inside" after d > -h has
held enter_persist_s; it leaves after d < -h (or not seen) has held
exit_persist_s. Enter fast, exit slow, so a person on the line does not
re-arm the alarm every second.

What it does NOT do
-------------------
No I/O, no clock, no publishing: step() takes the caller's monotonic seconds
and returns actions. It does not decide the siren / speech (alarm_wiring.py
and d_mode/auto_deter.py), and it does not stop the robot: the user's plan
says a warning zone does not restrict motion.

Looks right but is wrong
------------------------
  * Judging every warning zone in view: the user said the target must be in
    the zone the ROBOT is in.
  * Treating an unknown position as "outside every zone": that silently stops
    the alarm with no trace. Unknown position or heading = suspended, said so.
  * Dropping a track's state when it is missing for one frame: detections
    flicker; a missing frame counts toward exit, it does not reset it.
"""

from __future__ import annotations  # postponed annotations

import math  # projection, rotation
from dataclasses import dataclass, field  # config, track state
from typing import (  # hints
    Any,
    Callable,
    Dict,
    FrozenSet,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
)

from xbrain.p2_core.suspicion.hysteresis import (  # CAL-12 band
    HysteresisConfig,
    band_width_m,
)

#: metres per degree of latitude (spherical earth; the zones are small)
_M_PER_DEG = 111320.0  # 111.32 km
#: fix types good enough to say which zone the robot is in (11 S3.2.1)
_USABLE_FIX = frozenset({"rtk_fixed", "rtk_float"})  # dgps / single are not
#: the one class the user excluded by name (2026-11-13)
_BANNED_CLASS = "bird"  # never reported


class AutoAlarmConfigError(ValueError):  # refuse to start
    """auto_alarm config missing or out of range (refuse to start, CLAUDE.md 3.6)."""


@dataclass(frozen=True)
class AutoAlarmConfig:
    """p2_core.yaml auto_alarm, every field required (no defaults, CLAUDE.md 3.1)."""
    task_types: FrozenSet[str]                 # running task types that are judged
    class_groups: Mapping[str, FrozenSet[str]]  # person / vehicle / animal -> model names
    min_confidence: float                      # detections below are ignored
    pose_max_age_s: float                      # older state/pose = position unknown
    objects_max_age_s: float                   # older objects = nothing seen this tick
    hysteresis: HysteresisConfig               # k_h, h_min_m, enter / exit persist
    default_cooldown_s: float                  # cloud cooldown_sec not set
    default_duration_s: float                  # cloud duration_sec not set

    @classmethod
    def from_config(cls, get: Callable[[str], Any]) -> "AutoAlarmConfig":
        """Build from the resolved p2_core snapshot; get(key) -> value or None."""
        def num(key: str) -> float:  # a required positive number
            v = get("auto_alarm." + key)  # dotted path
            if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0:
                raise AutoAlarmConfigError("p2_core config: auto_alarm.%s missing or not > 0" % key)
            return float(v)  # as float
        types = get("auto_alarm.task_types")  # e.g. [patrol, goto]
        if not isinstance(types, list) or not types or not all(isinstance(t, str) for t in types):
            raise AutoAlarmConfigError("p2_core config: auto_alarm.task_types must be a non-empty list")
        groups = get("auto_alarm.class_groups")  # person / vehicle / animal
        if not isinstance(groups, dict) or set(groups) != {"person", "vehicle", "animal"}:
            raise AutoAlarmConfigError(
                "p2_core config: auto_alarm.class_groups must have exactly person / vehicle / animal")
        out: Dict[str, FrozenSet[str]] = {}  # validated groups
        for name, members in groups.items():  # each group
            if not isinstance(members, list) or not all(isinstance(m, str) and m for m in members):
                raise AutoAlarmConfigError("p2_core config: auto_alarm.class_groups.%s must be a list" % name)
            if _BANNED_CLASS in members:  # user ruling: birds are never reported
                raise AutoAlarmConfigError(
                    "p2_core config: auto_alarm.class_groups.%s lists bird (99 U188: not reported)" % name)
            out[name] = frozenset(members)  # frozen
        conf = num("min_confidence")  # (0, 1]
        if conf > 1.0:  # perception confidence is 0..1
            raise AutoAlarmConfigError("p2_core config: auto_alarm.min_confidence must be in (0, 1]")
        return cls(task_types=frozenset(types), class_groups=out, min_confidence=conf,
                   pose_max_age_s=num("pose_max_age_s"), objects_max_age_s=num("objects_max_age_s"),
                   hysteresis=HysteresisConfig(k_h=num("k_h"), h_min_m=num("h_min_m"),
                                               enter_persist_s=num("enter_persist_s"),
                                               exit_persist_s=num("exit_persist_s")),
                   default_cooldown_s=num("default_cooldown_s"),
                   default_duration_s=num("default_duration_s"))


@dataclass(frozen=True)
class Target:
    """One perception object as the judgment needs it (base_link metres)."""
    track_id: int            # MOT track id (11 S3.1B.2)
    class_name: str          # the model's class name
    confidence: float        # 0..1
    xy: Optional[Tuple[float, float]]  # footprint centroid; None = not localised


@dataclass(frozen=True)
class Zone:
    """One enabled warning polygon from cmd/fence (11 S9A.2)."""
    poly_id: str                              # the fence geo_id
    name: str                                 # for the event and the HMI
    vertices: Tuple[Tuple[float, float], ...]  # (lat, lon), not closed


@dataclass(frozen=True)
class AlarmInputs:
    """Everything one step judges; the wiring builds it from the latest samples."""
    now_s: float                       # caller's monotonic seconds
    task_type: Optional[str]           # the running task's type, None = none
    pose: Optional[Mapping[str, Any]]  # state/pose data
    pose_age_s: Optional[float]        # None = never received
    targets: Sequence[Target]          # latest objects
    objects_age_s: Optional[float]     # None = never received
    zones: Sequence[Zone]              # enabled warning zones
    alarm: Mapping[str, Any]           # the cmd/fence "alarm" block (11 S9A.2)
    clock_synced: bool                 # state/clock sync
    local_minute: Optional[int]        # minutes since local midnight, None = unknown


@dataclass(frozen=True)
class AlarmAction:
    """What the wiring must do: kind fire | suspend | resume | clock_unsynced."""
    kind: str                     # see above
    detail: Dict[str, Any]        # the event detail


@dataclass
class _Track:
    """Hysteresis state of one target in one zone."""
    enter_since: Optional[float] = None  # d > -h held since
    exit_since: Optional[float] = None   # d < -h or unseen held since
    inside: bool = False                 # confirmed inside
    last: Dict[str, Any] = field(default_factory=dict)  # last sighting, for the event


def _local_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:  # equirectangular
    """(east, north) metres of (lat, lon) from (lat0, lon0)."""
    return ((lon - lon0) * _M_PER_DEG * math.cos(math.radians(lat0)),  # east
            (lat - lat0) * _M_PER_DEG)  # north


def _seg_dist(p: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:  # helper of signed_distance
    """Distance from p to segment ab."""
    ax, ay = b[0] - a[0], b[1] - a[1]  # segment vector
    den = ax * ax + ay * ay  # squared length
    t = 0.0 if den == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / den))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)  # to the closest point


def signed_distance(p: Tuple[float, float], ring: Sequence[Tuple[float, float]]) -> float:  # even-odd + nearest edge
    """Signed distance from p to the polygon (positive inside), local metres."""
    inside = False  # even-odd ray cast
    n = len(ring)  # vertices
    for i in range(n):  # each edge
        (x1, y1), (x2, y2) = ring[i], ring[(i + 1) % n]  # edge endpoints
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < (x2 - x1) * (p[1] - y1) / (y2 - y1) + x1:
            inside = not inside  # crossed
    d = min(_seg_dist(p, ring[i], ring[(i + 1) % n]) for i in range(n))  # to the boundary
    return d if inside else -d  # signed


def in_window(window: Optional[Mapping[str, str]], minute: Optional[int]) -> bool:  # the 99 U187 window
    """The cloud's daily window: None or an unknown minute = all day; start ==
    end = all day; start > end crosses midnight (user ruling)."""
    if not window or minute is None:  # not set / clock unusable
        return True  # all day
    def hm(s: str) -> int:  # "HH:mm" -> minutes
        h, m = s.split(":")  # validated by P3 / P5
        return int(h) * 60 + int(m)  # minutes
    a, b = hm(window["start"]), hm(window["end"])  # bounds
    if a == b:  # same time = all day
        return True  # all day
    return a <= minute < b if a < b else (minute >= a or minute < b)  # plain or overnight


class AutoAlarm:  # one per P2
    """The per-P2 judgment. step() once per main-loop tick (10 Hz), main thread only."""

    def __init__(self, cfg: AutoAlarmConfig) -> None:  # built once at startup
        self._cfg = cfg  # validated config
        self._tracks: Dict[Tuple[str, int], _Track] = {}  # (poly_id, track_id) -> state
        self._next_fire: Dict[str, float] = {}  # poly_id -> earliest next alarm
        self._suspended: Optional[str] = None  # reason while suspended
        self._clock_told = False  # unsynced clock reported in this episode

    def _group(self, class_name: str) -> Optional[str]:  # class -> group
        """person / vehicle / animal for a model class name, None = not watched."""
        for g, members in self._cfg.class_groups.items():  # three groups
            if class_name in members:  # listed
                return g
        return None  # bird, chair, ...

    def _suspend(self, out: List[AlarmAction], reason: Optional[str], base: Dict[str, Any]) -> None:  # edge-triggered
        """Edge-trigger the suspend / resume events; reason None = usable."""
        if reason == self._suspended:  # no change
            return  # edge trigger: nothing to say
        if reason is None:  # recovered
            out.append(AlarmAction("resume", dict(base, reason=self._suspended)))
        elif self._suspended is None:  # newly suspended (a reason change is not a new start)
            out.append(AlarmAction("suspend", dict(base, reason=reason)))
        self._suspended = reason  # remember
        self._tracks.clear()  # nothing judged while blind; start again afterwards

    def step(self, inp: AlarmInputs) -> List[AlarmAction]:  # once per tick
        """This tick's actions (usually none)."""
        out: List[AlarmAction] = []  # to do
        cfg = self._cfg  # short
        if inp.task_type not in cfg.task_types or not inp.zones:  # out of scope
            self._suspend(out, None, {"scope": "ended"})  # a pending suspension ends with it
            self._tracks.clear()  # no judgment outside a patrol / navigation
            return out
        pose = inp.pose or {}  # latest state/pose
        lat, lon = pose.get("lat"), pose.get("lon")  # robot position
        if (lat is None or lon is None or pose.get("fix_type") not in _USABLE_FIX
                or inp.pose_age_s is None or inp.pose_age_s > cfg.pose_max_age_s):
            self._suspend(out, "position_invalid", {"fix_type": pose.get("fix_type")})
            return out
        here = []  # zones the robot is in
        for z in inp.zones:  # each enabled warning zone
            ring = [_local_xy(a, b, lat, lon) for a, b in z.vertices]  # robot at the origin
            if signed_distance((0.0, 0.0), ring) > 0:  # robot inside
                here.append((z, ring))
        here_ids = {z.poly_id for z, _ in here}  # for pruning
        for key in [k for k in self._tracks if k[0] not in here_ids]:  # left a zone
            del self._tracks[key]  # its tracks are no longer judged
        if not here:  # not in any warning zone: no judgment at all
            self._suspend(out, None, {"scope": "outside"})
            return out
        heading = pose.get("heading_rad")  # ENU, east = 0, ccw
        if not pose.get("heading_valid") or heading is None:  # cannot place a target
            self._suspend(out, "heading_invalid", {"heading_level": pose.get("heading_level"),
                                                   "zones": sorted(here_ids)})
            return out
        self._suspend(out, None, {"zones": sorted(here_ids)})  # usable again (no-op if it was)
        if not inp.clock_synced:  # window judged as all day, said once per episode
            if not self._clock_told:
                out.append(AlarmAction("clock_unsynced", {"window": inp.alarm.get("window")}))
                self._clock_told = True
        else:
            self._clock_told = False  # a later loss of sync is told again
        if not in_window(inp.alarm.get("window"), inp.local_minute if inp.clock_synced else None):
            self._tracks.clear()  # outside the window: start fresh when it opens
            return out
        fresh = inp.objects_age_s is not None and inp.objects_age_s <= cfg.objects_max_age_s
        cos_t, sin_t = math.cos(heading), math.sin(heading)  # rotation base_link -> ENU
        h = band_width_m(float(pose.get("cov_h_m") or 0.0), cfg.hysteresis)  # CAL-12 band
        for z, ring in here:  # each zone the robot is in
            watched = set()  # classes the rules ask for in this zone
            for r in inp.alarm.get("rules") or []:  # cloud rules or the default
                if r.get("region_ids") is None or z.poly_id in r["region_ids"]:
                    watched.update(r.get("targets") or [])
            seen = set()  # track ids seen inside the band this tick
            if fresh:  # a stale objects message moves nothing
                for t in inp.targets:  # this frame's objects
                    g = self._group(t.class_name)  # its class group
                    if (t.xy is None or g not in watched
                            or t.confidence < cfg.min_confidence):  # not ours
                        continue
                    e = t.xy[0] * cos_t - t.xy[1] * sin_t  # east (11 wpos formula)
                    n = t.xy[0] * sin_t + t.xy[1] * cos_t  # north
                    d = signed_distance((e, n), ring)  # positive inside
                    st = self._tracks.setdefault((z.poly_id, t.track_id), _Track())  # its state
                    st.last = {"track_id": t.track_id, "cls": t.class_name, "group": g,
                               "conf": round(t.confidence, 2), "d_inside_m": round(d, 2),
                               "lat": lat + n / _M_PER_DEG,  # back to WGS84 for the cloud wpos
                               "lon": lon + e / (_M_PER_DEG * math.cos(math.radians(lat)))}
                    if d > -h:  # in (or within the band): the enter side
                        seen.add(t.track_id)  # counts as present
                        st.exit_since = None  # the exit clock restarts
                        if not st.inside:  # confirm after enter_persist_s
                            st.enter_since = inp.now_s if st.enter_since is None else st.enter_since
                            if inp.now_s - st.enter_since >= cfg.hysteresis.enter_persist_s:
                                st.inside = True  # confirmed
            for (pid, tid), st in list(self._tracks.items()):  # tracks of this zone not seen in
                if pid != z.poly_id or tid in seen or not fresh:  # seen, or no new frame
                    continue
                st.enter_since = None  # an unconfirmed entry starts over
                if not st.inside:  # never confirmed: forget it
                    del self._tracks[(pid, tid)]
                    continue
                st.exit_since = inp.now_s if st.exit_since is None else st.exit_since  # leaving
                if inp.now_s - st.exit_since >= cfg.hysteresis.exit_persist_s:  # gone long enough
                    del self._tracks[(pid, tid)]
            hits = [st.last for (pid, _t), st in self._tracks.items()
                    if pid == z.poly_id and st.inside]  # confirmed intruders in this zone
            if hits and inp.now_s >= self._next_fire.get(z.poly_id, float("-inf")):  # cooldown
                cooldown = inp.alarm.get("cooldown_sec") or cfg.default_cooldown_s  # cloud or config
                self._next_fire[z.poly_id] = inp.now_s + float(cooldown)  # rate limit
                lead = min(hits, key=lambda x: x["track_id"])  # the target the v2.0 fields name
                rule = "%s_in_region" % lead["group"]  # the cloud's rule type for that group
                out.append(AlarmAction("fire", {
                    # v2.0 S5.2 alarm detail (p5 _alarm_detail fills only what is missing)
                    "alarm_type": rule, "matched_rule": rule, "zone_hits": [z.poly_id],
                    "track_id": lead["track_id"], "dwell_s": None,  # dwell is not tracked
                    "wpos": {"latitude": lead["lat"], "longitude": lead["lon"], "altitude_m": None,
                             "valid": True, "sigma_m": pose.get("cov_h_m")},
                    "zone_id": z.poly_id, "zone_name": z.name,
                    "targets": sorted(hits, key=lambda x: x["track_id"]),
                    "rules_source": inp.alarm.get("rules_source"),
                    "window": inp.alarm.get("window"),
                    "alarm_level": inp.alarm.get("alarm_level"),
                    "cooldown_sec": float(cooldown),
                    "duration_sec": float(inp.alarm.get("duration_sec") or cfg.default_duration_s),
                    "siren_level": inp.alarm.get("siren_level"),
                    "heading_level": pose.get("heading_level"),
                    "fix_type": pose.get("fix_type")}))
        return out


__all__ = ["AlarmAction", "AlarmInputs", "AutoAlarm", "AutoAlarmConfig", "AutoAlarmConfigError",
           "Target", "Zone", "in_window", "signed_distance"]
