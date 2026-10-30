"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: status_answers.py
Brief: G-class query answers from live GEN-plane state -- health, faults, mode, task, motion, geo, uptime -- and an honest reply for every query with no data path

Description:
What problem this solves
------------------------
Field test 2026-10-29 (R001-R006, P131, P235): "有故障吗", "系统状态怎么样",
"载荷都开着吗", "音量多大" ... answered only "收到", often after the robot had
read the user's own words back half-way. Cause: only G02 / G24 / G43-G47 had a
query_fn; every other G id fell through to a normal dispatch, and the G class
dispatches to cmd/audio/speak -- so the UTTERANCE was spoken, then the "收到"
ack cut it off. sources_g01_g24.py had declared a data source for each of the
24 queries, but nothing read them.

16 S8.2 CMD-40: take the real state -> pick the branch -> fill the template
(configs/query_templates.yaml, hot-tunable wording) -> TTS. This module does
the first two steps and calls templates.query_engine.render_reply for the
third; no reply wording lives in code. It reads only fields that exist on the
wire today (checked against the running robot on 2026-10-30):
  health/summary     items{name: {state, level, detail}}, allow_motion, speed_factor
  state/mode         voice_mode, motion_behavior                    (last value)
  state/task         current{type, state, progress}, queue[]        (flat, last value)
  state/pose         fix_type, lat, lon, speed_mps, heading_valid, heading_rad
  state/power        remain_mile_km
  state/geo/objects  waypoints[], routes[], docks[]                 (flat, last value)
  state/arb/ptz      holder{source_id, note}
  uptime             this host's CLOCK_MONOTONIC (Linux: seconds since boot)

Rules
-----
  * a stale or missing source gives the query_unreadable reply, never a
    last-known value and never a guess (16 S8.2.1; CLAUDE.md 3.2);
  * health/summary `level` is the item's CRITICALITY class (fatal / degraded /
    warn), NOT its status -- `state` (ok / degraded / fail / unknown) is the
    status. Judging a fault by level would report every healthy fatal-class
    item (chassis, camera) as faulty;
  * G32 answers query_ptz_pose.unknown (QT-5: the head has no position
    readback); G37 answers query_ptz_owner held / stale (QT-7);
  * a G query whose producer field is not published (11 S7.16 state/*.payload
    and .ptz are designed but not on the wire; events live in p5's record.db)
    answers query_no_data[id]: what is missing. It never echoes the utterance.

The id -> spoken-word tables below are vocabulary (closed sets from the
producers), not reply wording, so they stay with the code that reads the ids.

Boundary: no classification, no speaking, no publishing. Pure
(cache, templates, now) -> QueryAnswer; orchestrator_turn wires it in.
"""
from __future__ import annotations  # postponed annotations

import math  # heading in degrees
import time  # uptime = CLOCK_MONOTONIC since boot
from typing import Callable, Dict, Mapping, Optional

from xbrain.p4_agent.state.cache import StateCache  # freshness-aware cache
from xbrain.p4_agent.state.query_data import QueryAnswer  # the shared answer type
from xbrain.p4_agent.templates.query_engine import render_reply  # 16 S8.2 step 3

#: health/summary item id -> spoken name (p2 health aggregate ids).
ITEM_NAMES: Dict[str, str] = {
    "battery": "电池", "cam_rgbd": "深度相机", "chassis": "底盘", "clock": "时钟",  # id -> spoken label
    "compute": "计算单元", "dla": "DLA", "gpu": "GPU", "heading": "航向",  # id -> spoken label
    "lidar": "激光雷达", "mic": "麦克风", "network": "云端网络", "payload_light": "照明灯",  # id -> spoken label
    "payload_siren": "警笛", "payload_speaker": "喇叭", "payload_strobe": "警示灯",  # id -> spoken label
    "ptz": "云台", "rtk": "RTK 定位", "speech_preset": "预设语句", "storage": "存储",  # id -> spoken label
}
_STATE_WORDS = {"ok": "正常", "degraded": "降级", "fail": "故障", "unknown": "状态未知"}
_SENSORS = ("cam_rgbd", "lidar", "rtk", "heading", "clock", "ptz", "mic")  # G17 scope
_MODE_WORDS = {"idle": "空闲待命", "dialog_a": "现场对话", "dialog_c": "云端对话",
               "dialog_e": "企业微信对话", "broadcast": "远程喊话", "alarm": "报警驱离"}  # id -> spoken label
_TASK_WORDS = {"patrol": "巡逻", "goto": "前往目标点", "return_home": "返航",
               "charge": "回桩充电", "follow": "跟随", "standby": "待命", "teach": "路径录制"}  # id -> spoken label
_TASK_STATE_WORDS = {"running": "执行中", "ready": "排队中", "suspended": "已挂起",
                     "pending": "待执行", "paused": "已暂停"}  # id -> spoken label
_FIX_WORDS = {"single": "单点定位", "float": "RTK 浮点解", "fixed": "RTK 固定解"}
_PTZ_HOLDER_WORDS = {"manual_cloud": "人工操控", "auto_track": "报警或喊话模式的自动跟踪",
                     "preset_patrol": "预置位巡视"}  # id -> spoken label

Templates = Mapping[str, Mapping[str, str]]


def _say(t: Templates, name: str, branch: str, **values) -> str:  # one query answer: say
    """render_reply with keyword values; a missing template / branch raises."""
    return render_reply(t, name, branch, values)  # result to the caller


def _unreadable(t: Templates, what: str) -> QueryAnswer:  # one query answer: unreadable
    """The honest reply for a stale or missing source."""
    return QueryAnswer(False, _say(t, "query_unreadable", "stale", what=what))  # honest reply: source missing or stale


def _data(cache: StateCache, key: str, now_mono_ms: int, max_age_ms: int) -> Optional[Mapping]:  # one query answer: data
    """Fresh value's data object (envelope unwrapped), or None."""
    v = cache.get_fresh(key, now_mono_ms, max_age_ms)  # stale -> None
    if not isinstance(v, Mapping):  # guard: nothing usable
        return None  # not ours: let another query group answer
    inner = v.get("data")  # enveloped producers (11 S3.0)
    return inner if isinstance(inner, Mapping) else v  # result to the caller


def _latest(cache: StateCache, key: str) -> Optional[Mapping]:  # one query answer: latest
    """Last value of an on-change key: the last change IS the current state."""
    v = cache.latest(key)  # local read of the state field
    if not isinstance(v, Mapping):  # guard: nothing usable
        return None  # not ours: let another query group answer
    inner = v.get("data")  # value used below
    return inner if isinstance(inner, Mapping) else v  # result to the caller


def _not_ok(items: Mapping) -> Dict[str, Mapping]:  # one query answer: not ok
    """Health items whose STATE is not ok (level is the class, not the status).

    Filtering on `level` here instead is an EQUIVALENT mutant: every caller
    re-classifies by `state`, so the output does not change. The mutant that
    matters -- judging a fault by `level == "fatal"` -- is killed by
    test_faults_are_judged_by_state_not_level."""
    return {k: v for k, v in items.items()  # result to the caller
            if isinstance(v, Mapping) and v.get("state") != "ok"}  # branch on the live value


def _item_phrase(name: str, item: Mapping) -> str:  # one query answer: item phrase
    """'航向故障' / '激光雷达状态未知'."""
    st = str(item.get("state"))  # local read of the state field
    return ITEM_NAMES.get(name, name) + _STATE_WORDS.get(st, st)  # result to the caller


def _health(cache, now_mono_ms, max_age_ms) -> Optional[Mapping]:  # one query answer: health
    h = _data(cache, "health/summary", now_mono_ms, max_age_ms)  # local read of the state field
    return h if h is not None and isinstance(h.get("items"), Mapping) else None  # result to the caller


def faults_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: faults answer
    """G15 有故障吗: faults and degraded items named, unknown ones counted."""
    h = _health(cache, now_mono_ms, max_age_ms)  # local read of the state field
    if h is None:  # no usable value: take the honest branch
        return _unreadable(t, "健康状态")  # result to the caller
    bad = _not_ok(h["items"])  # local read of the state field
    fails = [k for k, v in bad.items() if v.get("state") == "fail"]  # local read of the state field
    degr = [k for k, v in bad.items() if v.get("state") == "degraded"]  # local read of the state field
    unk = [k for k, v in bad.items() if v.get("state") == "unknown"]  # local read of the state field
    if not fails and not degr:  # guard: nothing usable
        if unk:  # branch on the live value
            return QueryAnswer(True, _say(t, "query_faults_extra", "none_unknown", unknown=len(unk)))  # answer built from the live field
        return QueryAnswer(True, _say(t, "query_faults", "none"))  # answer built from the live field
    named = [_item_phrase(k, bad[k]) for k in fails + degr]  # value used below
    if unk:  # branch on the live value
        named.append("另有%d项状态未知" % len(unk))
    return QueryAnswer(True, _say(t, "query_faults", "some", count=len(fails) + len(degr),  # answer built from the live field
                                  items=",".join(named)))


def health_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: health answer
    """G16 系统状态怎么样: may it move, how fast, what is wrong."""
    h = _health(cache, now_mono_ms, max_age_ms)  # local read of the state field
    if h is None:  # no usable value: take the honest branch
        return _unreadable(t, "系统状态")  # result to the caller
    motion = "允许运动" if h.get("allow_motion") else "禁止运动"  # local read of the state field
    sf = h.get("speed_factor")  # local read of the state field
    limited = isinstance(sf, (int, float)) and not isinstance(sf, bool) and sf < 1  # value used below
    speed = ("限速%d%%" % round(float(sf) * 100)) if limited else "不限速"  # local read of the state field
    bad = _not_ok(h["items"])  # local read of the state field
    real = [k for k, v in bad.items() if v.get("state") in ("fail", "degraded")]  # local read of the state field
    if not real:  # guard: nothing usable
        return QueryAnswer(True, _say(t, "query_health", "ok", motion=motion, speed=speed))  # answer built from the live field
    return QueryAnswer(True, _say(t, "query_health", "some", motion=motion, speed=speed,  # answer built from the live field
                                  items=",".join(_item_phrase(k, bad[k]) for k in real)))


def sensor_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: sensor answer
    """G17 雷达 / 相机 / 定位正常吗: each sensor item with its status."""
    h = _health(cache, now_mono_ms, max_age_ms)  # local read of the state field
    if h is None:  # no usable value: take the honest branch
        return _unreadable(t, "传感器状态")  # result to the caller
    out = [_item_phrase(k, h["items"][k]) for k in _SENSORS  # local read of the state field
           if isinstance(h["items"].get(k), Mapping)]  # branch on the live value
    if not out:  # guard: nothing usable
        return _unreadable(t, "传感器状态")  # result to the caller
    return QueryAnswer(True, _say(t, "query_sensor", "ok", items=",".join(out)))  # answer built from the live field


def bit_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: bit answer
    """G18 自检结果: quick BIT is not wired (00 BIT-30) -- say so, then the live health."""
    live = health_answer(cache, t, now_mono_ms, max_age_ms=max_age_ms)  # local read of the state field
    return QueryAnswer(live.known, _say(t, "query_bit_result", "not_wired", health=live.text))  # answer (known flag says whether data was live)


def mode_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: mode answer
    """G05 当前什么模式 from state/mode."""
    m = _latest(cache, "state/mode")  # local read of the state field
    if m is None:  # no usable value: take the honest branch
        return _unreadable(t, "模式")  # result to the caller
    vm = str(m.get("voice_mode"))  # value used below
    hold = ",运动已暂停" if m.get("motion_behavior") == "hold" else ""  # value used below
    return QueryAnswer(True, _say(t, "query_mode", "ok", mode=_MODE_WORDS.get(vm, vm), hold=hold))  # answer built from the live field


def task_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: task answer
    """G06 当前任务 / 在干什么 from state/task."""
    s = _latest(cache, "state/task")  # local read of the state field
    if s is None:  # no usable value: take the honest branch
        return _unreadable(t, "任务状态")  # result to the caller
    queued = len(s.get("queue") or [])  # local read of the state field
    queue = (",另有%d个任务排队" % queued) if queued else ""  # value used below
    cur = s.get("current")  # local read of the state field
    if not isinstance(cur, Mapping):  # guard: nothing usable
        return QueryAnswer(True, _say(t, "query_task", "none", queue=queue))  # answer built from the live field
    kind = str(cur.get("type"))  # local read of the state field
    st = str(cur.get("state"))  # local read of the state field
    return QueryAnswer(True, _say(t, "query_task", "ok", kind=_TASK_WORDS.get(kind, kind),  # answer built from the live field
                                  state=_TASK_STATE_WORDS.get(st, st), queue=queue))


def progress_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: progress answer
    """G07 还有多久完成 / 巡逻到哪了: only what state/task.current.progress carries."""
    s = _latest(cache, "state/task")  # local read of the state field
    if s is None:  # no usable value: take the honest branch
        return _unreadable(t, "任务进度")  # result to the caller
    cur = s.get("current")  # local read of the state field
    if not isinstance(cur, Mapping):  # guard: nothing usable
        return QueryAnswer(True, _say(t, "query_progress", "none"))  # answer built from the live field
    prog = cur.get("progress")  # local read of the state field
    if isinstance(prog, Mapping):  # branch on the live value
        pct = prog.get("pct", prog.get("percent"))  # local read of the state field
        if isinstance(pct, (int, float)) and not isinstance(pct, bool):  # branch on the live value
            return QueryAnswer(True, _say(t, "query_progress", "pct", pct=round(pct)))  # answer built from the live field
        done = prog.get("dist_done_m")  # local read of the state field
        if isinstance(done, (int, float)) and not isinstance(done, bool):  # branch on the live value
            return QueryAnswer(True, _say(t, "query_progress", "dist", dist="%.0f" % done))  # answer built from the live field
    return QueryAnswer(True, _say(t, "query_progress", "no_data"))  # answer built from the live field


def position_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: position answer
    """G01 在哪: latitude / longitude only with a fix."""
    p = _data(cache, "state/pose", now_mono_ms, max_age_ms)  # local read of the state field
    if p is None:  # no usable value: take the honest branch
        return _unreadable(t, "位置")  # result to the caller
    fix = str(p.get("fix_type"))  # value used below
    if p.get("lat") is None or p.get("lon") is None or fix == "no_fix":  # no usable value: take the honest branch
        return QueryAnswer(True, _say(t, "query_position", "no_fix"))  # answer built from the live field
    return QueryAnswer(True, _say(t, "query_position", "ok", fix=_FIX_WORDS.get(fix, fix),  # answer built from the live field
                                  lat="%.6f" % p["lat"], lon="%.6f" % p["lon"]))


def speed_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: speed answer
    """G04 现在多快 from state/pose.speed_mps."""
    p = _data(cache, "state/pose", now_mono_ms, max_age_ms)  # local read of the state field
    v = p.get("speed_mps") if p is not None else None  # local read of the state field
    if not isinstance(v, (int, float)) or isinstance(v, bool):  # guard: nothing usable
        return _unreadable(t, "速度")  # result to the caller
    if float(v) < 0.05:  # branch on the live value
        return QueryAnswer(True, _say(t, "query_speed", "still"))  # answer built from the live field
    return QueryAnswer(True, _say(t, "query_speed", "ok", v="%.1f" % float(v)))  # answer built from the live field


def heading_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: heading answer
    """G08 朝哪个方向: only a VALID heading is spoken (H-1)."""
    p = _data(cache, "state/pose", now_mono_ms, max_age_ms)  # local read of the state field
    if p is None:  # no usable value: take the honest branch
        return _unreadable(t, "航向")  # result to the caller
    rad = p.get("heading_rad")  # value used below
    if not p.get("heading_valid") or not isinstance(rad, (int, float)):  # guard: nothing usable
        return QueryAnswer(True, _say(t, "query_heading", "invalid"))  # answer built from the live field
    return QueryAnswer(True, _say(t, "query_heading", "ok",  # answer built from the live field
                                  deg="%.0f" % ((math.degrees(float(rad)) + 360.0) % 360.0)))


def range_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: range answer
    """G03 还能跑多久 from state/power.remain_mile_km (the chassis' own estimate)."""
    p = _data(cache, "state/power", now_mono_ms, max_age_ms)  # local read of the state field
    km = p.get("remain_mile_km") if p is not None else None  # value used below
    if not isinstance(km, (int, float)) or isinstance(km, bool):  # guard: nothing usable
        return _unreadable(t, "续航")  # result to the caller
    return QueryAnswer(True, _say(t, "query_range", "ok", km="%.1f" % float(km)))  # answer built from the live field


def uptime_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: uptime answer
    """G09 开机多久了: Linux CLOCK_MONOTONIC counts from boot."""
    h, m = divmod(int(time.monotonic()) // 60, 60)
    if h:  # branch on the live value
        return QueryAnswer(True, _say(t, "query_uptime", "hm", h=h, m=m))  # answer built from the live field
    return QueryAnswer(True, _say(t, "query_uptime", "m", m=m))  # answer built from the live field


def _geo_answer(field: str, word: str) -> Callable[..., QueryAnswer]:  # one query answer: geo answer
    """G21 / G22 / G23 from state/geo/objects[field]."""
    def _ans(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: ans
        g = _latest(cache, "state/geo/objects")  # local read of the state field
        if g is None or not isinstance(g.get(field), list):  # no usable value: take the honest branch
            return _unreadable(t, word)  # result to the caller
        objs = [o for o in g[field] if isinstance(o, Mapping)]  # local read of the state field
        if not objs:  # guard: nothing usable
            return QueryAnswer(True, _say(t, "query_geo", "none", word=word))  # answer built from the live field
        names = [str(o.get("name") or o.get("id")) for o in objs]  # local read of the state field
        shown = ",".join(names[:5]) + ("等" if len(names) > 5 else "")  # value used below
        return QueryAnswer(True, _say(t, "query_geo", "some", count=len(objs), word=word,  # answer built from the live field
                                      names=shown))
    return _ans  # result to the caller


def ptz_pose_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: ptz pose answer
    """G32 云台朝哪: QT-5, there is no position readback -- always unknown."""
    return QueryAnswer(False, _say(t, "query_ptz_pose", "unknown"))  # honest reply: source missing or stale


def ptz_owner_answer(cache, t, now_mono_ms, *, max_age_ms) -> QueryAnswer:  # one query answer: ptz owner answer
    """G37 云台谁在控 from state/arb/ptz; stale -> the QT-7 stale branch."""
    a = _data(cache, "state/arb/ptz", now_mono_ms, max_age_ms)  # local read of the state field
    if a is None:  # no usable value: take the honest branch
        return QueryAnswer(False, _say(t, "query_ptz_owner", "stale"))  # honest reply: source missing or stale
    h = a.get("holder")  # local read of the state field
    if not isinstance(h, Mapping):  # guard: nothing usable
        return QueryAnswer(True, _say(t, "query_ptz_owner", "held", holder="没有人",  # answer built from the live field
                                      note="空闲"))
    sid = str(h.get("source_id"))  # value used below
    return QueryAnswer(True, _say(t, "query_ptz_owner", "held",  # answer built from the live field
                                  holder=_PTZ_HOLDER_WORDS.get(sid, sid),
                                  note=str(h.get("note") or sid)))


#: intent id -> answer function (live data).
STATUS_QUERY_ANSWERS: Dict[str, Callable[..., QueryAnswer]] = {
    "G01": position_answer, "G03": range_answer, "G04": speed_answer,  # intent id -> spoken text
    "G05": mode_answer, "G06": task_answer, "G07": progress_answer,  # intent id -> spoken text
    "G08": heading_answer, "G09": uptime_answer, "G15": faults_answer,  # intent id -> spoken text
    "G16": health_answer, "G17": sensor_answer, "G18": bit_answer,  # intent id -> spoken text
    "G21": _geo_answer("routes", "巡逻路径"), "G22": _geo_answer("waypoints", "位置点"),  # intent id -> spoken text
    "G23": _geo_answer("docks", "充电桩"), "G32": ptz_pose_answer, "G37": ptz_owner_answer,  # intent id -> spoken text
}


def status_answer(entry_id: str, cache: StateCache, templates: Templates,  # one query answer: status answer
                  now_mono_ms: int, *, max_age_ms: int) -> Optional[QueryAnswer]:
    """Answer for a G id this module owns (live or query_no_data), else None."""
    fn = STATUS_QUERY_ANSWERS.get(entry_id)  # local read of the state field
    if fn is not None:  # branch on the live value
        return fn(cache, templates, now_mono_ms, max_age_ms=max_age_ms)  # result to the caller
    if entry_id in (templates.get("query_no_data") or {}):  # branch on the live value
        return QueryAnswer(False, _say(templates, "query_no_data", entry_id))  # honest reply: source missing or stale
    return None  # not ours: let another query group answer


def generic_no_data(templates: Templates) -> str:  # one query answer: generic no data
    """The reply for any G id nobody answers: honest, and never the utterance."""
    return _say(templates, "query_no_data", "generic")  # result to the caller


__all__ = ["STATUS_QUERY_ANSWERS", "ITEM_NAMES", "status_answer", "generic_no_data"]  # value used below
