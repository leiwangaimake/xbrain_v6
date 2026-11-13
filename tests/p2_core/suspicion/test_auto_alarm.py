"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_auto_alarm.py
Brief: 99 U188 -- the warning-zone autonomous alarm: judgment, device borrowing, wiring parsers

Description:
The user's rules (99 U188, plan section 7 as amended 2026-11-13), each pinned
by a case naming the mutant it kills:
  * only during a path patrol / point-to-point navigation (config task_types);
  * only while the robot itself is inside an enabled warning zone, and only
    for a target inside THAT zone;
  * people by default (person / bicycle / motorcycle), the cloud's rules
    otherwise; birds never (a config listing bird is refused);
  * the daily window (overnight allowed; unsynced clock = all day, told once);
  * heading or position lost in a zone = suspended, start and recovery told;
  * hysteresis (enter after enter_persist_s, leave after exit_persist_s) and a
    per-zone cooldown;
  * the device: siren + red/blue for duration_sec in the device's deter mode
    while the SYSTEM mode stays IDLE, then the spoken warning; the 5 s
    reconcile aims at deter meanwhile; an operator mode takes the device back.
The production p2_core.yaml is read, so a config edit that breaks a rule
turns these red.
"""
from __future__ import annotations  # postponed annotations

import json  # wire bodies
import math  # metres -> degrees
from pathlib import Path  # repo root

import pytest  # marker, raises
import yaml  # the production config

from xbrain.p2_core.d_mode.auto_deter import AutoDeter  # under test
from xbrain.p2_core.mode.device_sync import SyncResult  # the device verdict shape
from xbrain.p2_core.runtime.alarm_wiring import (  # under test
    local_minute,
    parse_targets,
    parse_zones,
)
from xbrain.p2_core.suspicion.auto_alarm import (  # under test
    AlarmInputs,
    AutoAlarm,
    AutoAlarmConfig,
    AutoAlarmConfigError,
    Target,
    Zone,
    in_window,
)

pytestmark = pytest.mark.no_device  # pure in-process

_ROOT = Path(__file__).resolve().parents[3]  # repo root
_CFG = yaml.safe_load((_ROOT / "configs" / "p2_core.yaml").read_text(encoding="utf-8"))  # production


def _get(cfg):  # dotted reader like the resolved snapshot's
    def get(key):  # one key
        cur = cfg  # from the root
        for part in key.split("."):  # each segment
            if not isinstance(cur, dict) or part not in cur:  # absent
                return None  # the wiring maps _MISSING to None
            cur = cur[part]  # descend
        return cur  # leaf
    return get  # reader


_C = AutoAlarmConfig.from_config(_get(_CFG))  # production values
_LAT, _LON = 34.6970, 135.5050  # a site
_DLAT = 1.0 / 111320.0  # one metre north
_DLON = _DLAT / math.cos(math.radians(_LAT))  # one metre east


def _sq(x0, y0, side, pid="f-yard", name="yard"):  # a square zone, metres from the site
    return Zone(pid, name, tuple((_LAT + y * _DLAT, _LON + x * _DLON)
                                 for x, y in ((x0, y0), (x0 + side, y0), (x0 + side, y0 + side),
                                              (x0, y0 + side))))


_PEOPLE = {"window": None, "rules": [{"targets": ["person"], "region_ids": None}],
           "rules_source": "default"}  # the default alarm block (99 U187)
_POSE = {"lat": _LAT, "lon": _LON, "fix_type": "rtk_fixed", "heading_valid": True,
         "heading_rad": 0.0, "heading_level": 1, "cov_h_m": 0.0}  # facing east, at the site


def _inp(now, targets=(), task="patrol", pose=_POSE, zones=None, alarm=_PEOPLE,
         synced=True, minute=600, obj_age=0.05, pose_age=0.05):  # one tick's inputs
    return AlarmInputs(now_s=now, task_type=task, pose=pose, pose_age_s=pose_age,
                       targets=list(targets), objects_age_s=obj_age,
                       zones=[_sq(-20, -20, 40)] if zones is None else zones,
                       alarm=alarm, clock_synced=synced, local_minute=minute)


def _person(x=5.0, y=0.0, tid=1, cls="person", conf=0.9):  # a target x m ahead, y m left
    return Target(tid, cls, conf, (x, y))


def _run(eng, seconds, **kw):  # tick at 10 Hz from t=0, all actions
    out = []  # every action
    for i in range(int(seconds * 10) + 1):  # inclusive end
        out += eng.step(_inp(i / 10.0, **kw))  # one tick
    return out  # actions


def _fires(acts):  # just the alarms
    return [a for a in acts if a.kind == "fire"]


# -- the judgment -------------------------------------------------------------
def test_a_person_in_the_robots_zone_fires_after_enter_persist():
    """MUTATION: enter_persist ignored -> fires at t=0 -> red; group lookup
    broken -> never fires -> red."""
    eng = AutoAlarm(_C)  # fresh
    assert _fires(eng.step(_inp(0.0, [_person()]))) == []  # not yet: persistence
    acts = _run(eng, _C.hysteresis.enter_persist_s + 0.2, targets=[_person()])  # held
    fired = _fires(acts)  # alarms
    assert len(fired) == 1 and fired[0].detail["zone_id"] == "f-yard"  # once, this zone
    assert fired[0].detail["targets"][0]["cls"] == "person"  # who
    assert fired[0].detail["duration_sec"] == _C.default_duration_s  # config default
    d = fired[0].detail  # the v2.0 S5.2 fields the cloud reads
    assert d["alarm_type"] == d["matched_rule"] == "person_in_region" and d["zone_hits"] == ["f-yard"]
    assert d["track_id"] == 1 and d["wpos"]["valid"] is True  # the target named
    assert abs(d["wpos"]["latitude"] - _LAT) < 1e-7  # facing east: same latitude
    assert abs((d["wpos"]["longitude"] - _LON) / _DLON - 5.0) < 0.01  # 5 m east of the robot


def test_two_people_one_alarm_naming_the_lowest_track():
    """One alarm per zone lists both; the v2.0 single-target fields name the
    lowest track id (stable across ticks). MUTATION: max instead of min -> red."""
    fired = _fires(_run(AutoAlarm(_C), 1.5, targets=[_person(tid=5), _person(x=3.0, tid=2)]))
    assert len(fired) == 1 and [t["track_id"] for t in fired[0].detail["targets"]] == [2, 5]
    assert fired[0].detail["track_id"] == 2  # the lead target


@pytest.mark.parametrize("cls", ["bicycle", "motorcycle"])
def test_riders_count_as_people(cls):
    """User ruling: 行人 / 自行车 / 摩托车 = 人. MUTATION: drop them from
    class_groups.person -> no alarm -> red."""
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(cls=cls)]))  # alarm


def test_birds_never_and_a_config_listing_bird_is_refused():
    """User ruling: 鸟类不报. MUTATION: drop the bird check -> loads -> red."""
    rules = dict(_PEOPLE, rules=[{"targets": ["animal"], "region_ids": None}])  # animals asked
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(cls="bird")], alarm=rules)) == []
    dog = _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(cls="dog")], alarm=rules))  # a dog is
    assert dog and dog[0].detail["alarm_type"] == "animal_in_region"  # named for the cloud
    bad = json.loads(json.dumps(_CFG))  # deep copy
    bad["auto_alarm"]["class_groups"]["animal"].append("bird")  # someone adds it
    with pytest.raises(AutoAlarmConfigError):
        AutoAlarmConfig.from_config(_get(bad))  # refused


def test_default_rules_watch_people_only():
    """A car is not watched by the default. MUTATION: watched ignored -> red."""
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(cls="car")])) == []
    cars = dict(_PEOPLE, rules=[{"targets": ["vehicle"], "region_ids": None}])  # the cloud asks
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(cls="car")], alarm=cars))


def test_a_rule_limited_to_another_zone_does_not_apply():
    """region_ids name the zones. MUTATION: region_ids ignored -> red."""
    other = dict(_PEOPLE, rules=[{"targets": ["person"], "region_ids": ["f-gate"]}])  # elsewhere
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person()], alarm=other)) == []


def test_only_patrol_and_goto_are_judged():
    """User: path patrol + PTP navigation. MUTATION: task gate removed -> red."""
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person()], task="goto"))  # PTP counts
    for task in ("return_home", "charge", None):  # anything else
        assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person()], task=task)) == []


def test_robot_outside_every_zone_judges_nothing():
    """"机器人不在报警区内时... 不做任何判断", even with a person in a zone
    ahead. MUTATION: robot-inside check removed -> fires -> red."""
    ahead = [_sq(3, -5, 10)]  # a zone 3..13 m ahead, the robot outside it
    acts = _run(AutoAlarm(_C), 2.0, targets=[_person(x=8.0)], zones=ahead)  # person in it
    assert _fires(acts) == [] and not [a for a in acts if a.kind == "suspend"]  # silent


def test_target_must_be_inside_the_robots_zone():
    """In view but outside the zone is not an alarm. MUTATION: the target
    distance test removed -> red."""
    small = [_sq(-3, -3, 6)]  # 6 m square around the robot
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(x=15.0)], zones=small)) == []
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(x=2.0)], zones=small))


def test_heading_rotates_the_target_into_the_world():
    """A zone 2 m wide (east) and 10 m long (north) around the robot: a
    person 5 m ahead is inside when facing north, outside when facing east.
    MUTATION: rotation dropped (body x taken as east) -> red; sign of the
    sine flipped -> "ahead" lands south when facing north -> red."""
    narrow = [Zone("f-ns", "ns", ((_LAT - _DLAT, _LON - _DLON), (_LAT - _DLAT, _LON + _DLON),
                                  (_LAT + 9 * _DLAT, _LON + _DLON), (_LAT + 9 * _DLAT, _LON - _DLON)))]
    facing_n = dict(_POSE, heading_rad=math.pi / 2)  # ENU: north = +90 deg, ccw
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(x=5.0)], pose=facing_n, zones=narrow))
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(x=5.0)], zones=narrow)) == []  # east
    facing_s = dict(_POSE, heading_rad=-math.pi / 2)  # south: "ahead" is outside
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person(x=5.0)], pose=facing_s, zones=narrow)) == []


def test_heading_lost_in_a_zone_suspends_and_recovery_is_told():
    """MUTATION: heading not checked -> no suspend event -> red; the edge
    trigger removed -> a suspend every tick -> red."""
    eng = AutoAlarm(_C)  # fresh
    blind = dict(_POSE, heading_valid=False)  # L3
    acts = _run(eng, 1.0, targets=[_person()], pose=blind)  # 11 ticks blind
    assert [a.kind for a in acts] == ["suspend"] and acts[0].detail["reason"] == "heading_invalid"
    back = eng.step(_inp(2.0, [_person()]))  # heading back
    assert [a.kind for a in back] == ["resume"] and back[0].detail["reason"] == "heading_invalid"


def test_a_blind_spell_restarts_the_hysteresis():
    """Tracks confirmed before heading was lost are not trusted after it
    returns: the target must hold enter_persist_s again. MUTATION: tracks
    kept across the suspension -> fires on the first tick back -> red."""
    eng = AutoAlarm(_C)  # fresh
    alarm = dict(_PEOPLE, cooldown_sec=0.5)  # cooldown out of the way
    _run(eng, 1.5, targets=[_person()], alarm=alarm)  # confirmed and fired
    eng.step(_inp(1.6, [_person()], alarm=alarm, pose=dict(_POSE, heading_valid=False)))  # blind
    back = eng.step(_inp(2.0, [_person()], alarm=alarm))  # heading back, cooldown long over
    assert [a.kind for a in back] == ["resume"]  # resumed, but no alarm on this tick


def test_position_lost_suspends():
    """single fix / stale pose = position unknown, said so. MUTATION: fix
    type not checked -> judged on a bad fix -> red."""
    single = dict(_POSE, fix_type="single")  # not good enough
    acts = AutoAlarm(_C).step(_inp(0.0, [_person()], pose=single))  # one tick
    assert [a.kind for a in acts] == ["suspend"] and acts[0].detail["reason"] == "position_invalid"
    stale = AutoAlarm(_C).step(_inp(0.0, [_person()], pose_age=_C.pose_max_age_s + 0.1))
    assert [a.kind for a in stale] == ["suspend"]  # stale pose too


def test_window_overnight_and_unsynced_clock_all_day_told_once():
    """22:00-05:00 crosses midnight; an unsynced clock judges all day and
    says so once. MUTATION: overnight handled as start <= m < end -> red;
    clock_told never set -> a clock event every tick -> red."""
    night = dict(_PEOPLE, window={"start": "22:00", "end": "05:00"})  # overnight
    assert in_window(night["window"], 23 * 60) and in_window(night["window"], 4 * 60)
    assert not in_window(night["window"], 12 * 60)  # noon is outside
    assert in_window({"start": "08:00", "end": "08:00"}, 3)  # same = all day
    assert _fires(_run(AutoAlarm(_C), 2.0, targets=[_person()], alarm=night, minute=12 * 60)) == []
    acts = _run(AutoAlarm(_C), 2.0, targets=[_person()], alarm=night, synced=False, minute=None)
    assert _fires(acts)  # all day
    assert [a.kind for a in acts].count("clock_unsynced") == 1  # told once


def test_cooldown_limits_repeats_and_a_lingering_person_fires_again():
    """MUTATION: no cooldown -> fires every tick -> red; cooldown applied
    forever -> no second alarm -> red."""
    short = dict(_PEOPLE, cooldown_sec=3.0)  # cloud 3 s
    fired = _fires(_run(AutoAlarm(_C), 6.5, targets=[_person()], alarm=short))  # 6.5 s present
    assert len(fired) == 2  # at 1.0 s and 4.0 s; the next would be at 7.0 s
    assert fired[0].detail["cooldown_sec"] == 3.0  # the cloud's value used


def test_exit_needs_exit_persist_and_a_flicker_does_not_reset():
    """One missing frame does not end a confirmed track. MUTATION: delete
    the track on the first unseen frame -> the second alarm (after the
    cooldown) needs a new enter_persist -> fires later -> red."""
    eng = AutoAlarm(_C)  # fresh
    alarm = dict(_PEOPLE, cooldown_sec=1.0)  # short cooldown
    _run(eng, 1.5, targets=[_person()], alarm=alarm)  # confirmed, fired at ~1.0
    assert eng.step(_inp(1.6, [], alarm=alarm)) == []  # one empty frame
    again = []  # alarms right after the cooldown
    for i in range(17, 25):  # 1.7 .. 2.4 s
        again += _fires(eng.step(_inp(i / 10.0, [_person()], alarm=alarm)))
    assert again and again[0].detail["targets"][0]["track_id"] == 1  # still confirmed: fires at once


def test_stale_objects_freeze_instead_of_counting_as_gone():
    """MUTATION: stale frames treated as empty -> the exit clock runs."""
    eng = AutoAlarm(_C)  # fresh
    _run(eng, 1.5, targets=[_person()])  # confirmed
    for i in range(16, 80):  # 6.4 s of stale frames
        eng.step(_inp(i / 10.0, [], obj_age=_C.objects_max_age_s + 1))  # nothing new
    assert eng._tracks  # still tracked (frozen)


# -- the device ---------------------------------------------------------------
class _Dev:  # DeviceModeSync double
    def __init__(self, deter_ok=True):
        self.calls = []  # (what, arg)
        self.bodies = []  # every /deter body
        self.deter_ok = deter_ok  # scripted /deter answer

    def apply(self, mode):  # POST /mode
        self.calls.append(("mode", mode))  # record
        return SyncResult(ok=True, status=200)  # ok

    def deter(self, body):  # POST /deter
        self.calls.append(("deter", body.get("on")))  # record
        self.bodies.append(body)  # whole body
        ok = self.deter_ok or body.get("on") is False  # off always accepted
        return SyncResult(ok=ok, status=200 if ok else 409)  # scripted


def test_deter_sounds_for_duration_then_speaks_and_restores():
    """MUTATION: tts_reps not 0 (warnings cut off) / no /mode back / no speak -> red."""
    dev, said = _Dev(), []  # device, speech
    ad = AutoDeter(dev, 1, 1.0, "leave", said.append)  # redblue 1, default level 1.0
    assert ad.start(0, 70, 5.0, system_idle=True) == "started"  # siren on
    assert dev.calls == [("mode", "deter"), ("deter", True)]  # device borrowed
    assert dev.bodies[0] == {"on": True, "mode": 1, "siren_level": 0.7, "tts_reps": 0}  # siren only
    assert ad.reconcile_target("func1") == "deter"  # the 5 s reconcile holds deter
    ad.tick(4999, True, False, "func1")  # not yet
    assert said == [] and ad.running  # still sounding
    ad.tick(5000, True, False, "func1")  # due
    assert dev.calls[-2:] == [("deter", False), ("mode", "func1")] and said == ["leave"]
    assert ad.reconcile_target("func1") == "func1" and not ad.running  # handed back


def test_deter_only_from_idle_and_a_refusal_restores_the_device():
    """MUTATION: start in a non-IDLE mode -> fights the operator -> red;
    refused /deter leaves the device in deter -> red."""
    dev = _Dev()  # device
    ad = AutoDeter(dev, 1, 1.0, "leave", lambda t: None)  # speech ignored
    assert ad.start(0, None, 5.0, system_idle=False) == "not_idle" and dev.calls == []
    bad = _Dev(deter_ok=False)  # device refuses /deter
    ad2 = AutoDeter(bad, 1, 1.0, "leave", lambda t: None)  # second instance
    assert ad2.start(0, None, 5.0, system_idle=True) == "refused"  # failed
    assert bad.calls[-1] == ("mode", "func1") and not ad2.running  # put back


def test_an_operator_mode_takes_the_device_back():
    """Into DIALOG (func1 -> func1, no /mode from the transition) the siren
    must be switched off here; into ALARM D owns it. MUTATION: no _off on
    takeover -> the siren keeps going in a dialog -> red."""
    dev = _Dev()  # device
    ad = AutoDeter(dev, 1, 1.0, "leave", lambda t: None)  # speech ignored
    ad.start(0, None, 5.0, system_idle=True)  # sounding
    ad.tick(1000, False, False, "func1")  # a dialog started
    assert dev.calls[-2:] == [("deter", False), ("mode", "func1")] and not ad.running
    dev2 = _Dev()  # second device
    ad2 = AutoDeter(dev2, 1, 1.0, "leave", lambda t: None)  # second instance
    ad2.start(0, None, 5.0, system_idle=True)  # sounding
    n = len(dev2.calls)  # before the takeover
    ad2.tick(1000, False, True, "deter")  # D took over
    assert len(dev2.calls) == n and not ad2.running  # nothing sent, ownership dropped


# -- the wiring parsers -------------------------------------------------------
def test_parse_zones_keeps_only_warning_polygons_and_the_alarm_block():
    """MUTATION: role filter removed -> the allow polygon becomes a zone -> red."""
    fs = {"fence_set_id": "fs", "rev": 3, "alarm": {"window": None},
          "polygons": [{"poly_id": "f-a", "role": "allow", "vertices": [{"lat": 1, "lon": 2}] * 3},
                       {"poly_id": "f-w", "role": "warning", "name": "w",
                        "vertices": [{"lat": 1, "lon": 2}, {"lat": 1, "lon": 3}, {"lat": 2, "lon": 3}]}]}
    zones, alarm = parse_zones(fs)  # parsed
    assert [z.poly_id for z in zones] == ["f-w"] and alarm == {"window": None}


def test_parse_targets_centroid_and_skips_bad_objects():
    """MUTATION: a bad object drops the whole message -> red."""
    body = {"objects": [{"track_id": 7, "class_name": "person", "confidence": 0.8,
                         "footprint_xy": [[1, 0], [3, 0], [3, 2], [1, 2]]},
                        {"track_id": "x"},  # malformed
                        {"track_id": 9, "class_name": "dog", "confidence": 0.7, "footprint_xy": None}]}
    ts = parse_targets(body)  # parsed
    assert [(t.track_id, t.xy) for t in ts] == [(7, (2.0, 1.0)), (9, None)]


def test_local_minute_uses_the_fix_time_zone():
    """Osaka is UTC+9. MUTATION: lat / lon swapped into tzfpy -> red."""
    pytest.importorskip("tzfpy")  # the wheel is on the robot
    assert local_minute(_LAT, _LON, 0.0) == 9 * 60  # 1970-01-01 09:00 JST
    assert local_minute(0.0, 0.0, 0.0) is None  # a zeroed pose is no fix


# -- the wiring end to end (fake sessions) -------------------------------------
class _Sample:  # a zenoh sample double
    def __init__(self, body):
        self.payload = json.dumps(body).encode("utf-8")  # wire bytes


class _Sess:  # a zenoh session double: keeps callbacks, records puts
    def __init__(self):
        self.cbs, self.puts = {}, []  # key -> callback, (key, body)

    def declare_subscriber(self, key, cb):  # the handle is the key
        self.cbs[key] = cb  # remember
        return key  # held by the wiring

    def put(self, key, raw):  # an event
        self.puts.append((key, json.loads(raw)))  # decoded


def test_wiring_tick_publishes_the_alarm_and_borrows_the_device(monkeypatch):
    """cmd/fence + state/task + objects in, event/alarm/intrusion out with
    the deterrent outcome; the device borrowed. MUTATION: the fire branch
    not starting the deterrent / not publishing -> red; the task current
    type not read -> never fires -> red."""
    import xbrain.p2_core.runtime.alarm_wiring as aw  # the module under test
    from xbrain.p2_core.runtime.alarm_wiring import AlarmWiring  # under test
    clock = [100.0]  # fake monotonic seconds
    monkeypatch.setattr(aw.time, "monotonic", lambda: clock[0])  # the slots' arrival time
    gen, rt, dev = _Sess(), _Sess(), _Dev()  # sessions and device
    cache = {"pose": _POSE, "clock": {"sync": False}}  # unsynced: all day
    rx = {"pose": 100.0}  # pose just arrived
    w = AlarmWiring(AutoAlarm(_C), AutoDeter(dev, 1, 1.0, "leave", lambda t: None),
                    gen, rt, "gj-001", cache, rx)
    w.subscribe()  # three subscribers
    assert set(gen.cbs) == {"cmd/fence", "state/task"}  # GEN keys
    assert set(rt.cbs) == {"xbrain/gj-001/rt/perception/objects"}  # absolute RT key
    z = _sq(-20, -20, 40)  # the zone around the robot
    gen.cbs["cmd/fence"](_Sample({"fence_set_id": "fs", "rev": 4, "alarm": _PEOPLE, "polygons": [
        {"poly_id": z.poly_id, "role": "warning", "name": z.name,
         "vertices": [{"lat": a, "lon": b} for a, b in z.vertices]}]}))
    gen.cbs["state/task"](_Sample({"data": {"current": {"type": "patrol"}}}))  # enveloped
    for i in range(20):  # 2 s at 10 Hz
        clock[0] = 100.0 + i / 10.0  # time moves
        rx["pose"] = clock[0]  # fresh pose each tick
        rt.cbs["xbrain/gj-001/rt/perception/objects"](_Sample({"data": {"objects": [
            {"track_id": 3, "class_name": "person", "confidence": 0.9,
             "footprint_xy": [[4, -1], [6, -1], [6, 1], [4, 1]]}]}}))
        w.tick(int(clock[0] * 1000), True, False, "func1")  # one pass
    alarms = [b for k, b in gen.puts if k == "event/alarm/intrusion"]  # the alarm events
    assert len(alarms) == 1 and alarms[0]["title"] == "zone_intrusion"  # once (cooldown)
    d = alarms[0]["detail"]  # its detail
    assert d["deterrent"] == "started" and d["rev"] == 4 and d["fence_set_id"] == "fs"
    assert ("mode", "deter") in dev.calls  # the device borrowed
    assert [k for k, _ in gen.puts].count("event/warn/intrusion") == 1  # the clock note, once
