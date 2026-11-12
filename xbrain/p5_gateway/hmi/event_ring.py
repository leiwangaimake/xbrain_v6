"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: event_ring.py
Brief: HMI event list ring -- 200 entries, exact repeats merged, robot position stamped (17 S6.6)

Description:
What problem this solves
------------------------
The HMI event list (user design 2026-11-12: a bottom-centre drawer that shows
every reported event, fault and state) was fed by a 50-entry ring of
{eid, title, sev, cat, ts, pos}. Measured in record.db the same day: of 31.5 k
rows, 20 k are the low-battery alarm that p3 repeats every 2 s and 2.6 k the
speaker grant / release pair -- a 50-entry ring holds under two minutes of the
low-battery alarm and nothing else. And with no detail in the ring the drawer
could not say which battery, which device, which task.

Rules (user ruling 2026-11-12, recommendations 5 / 7 / 8)
---------------------------------------------------------
  * RING_MAX entries, newest last;
  * an exact repeat -- same cat, sev, title and detail.type (or detail.kind) --
    within MERGE_WINDOW_S of that entry's last occurrence is folded into it:
    count + 1, ts / eid / detail become the newest, first_ts stays, and the
    entry moves to the end. A continuous alarm is ONE entry "since first_ts";
  * a task event carries the task id in its title, so two tasks never merge;
  * detail rides along, capped at DETAIL_MAX_BYTES serialised (the whole group
    is resent on every WS delta in which it changed);
  * pos: the event's own pos when it has lat / lon; otherwise the robot's fresh
    position at arrival (pos_src "robot"), because no producer stamps one today
    (record.db location_lat is empty in every row) and the drawer's click-to-
    flash needs a place. A stale or missing pose gives pos None, never 0,0.

Boundary: pure functions, no I/O, no clock -- main_wiring calls push() from the
event callbacks and seeds the ring at start-up from event/recent.py. ts is the
producer's display stamp (11 S6.2); comparing two of them to decide a merge is
grouping for the screen, never an age or a timeout.
"""
from __future__ import annotations  # postponed annotations

import json  # detail size cap
from typing import Any, Dict, List, Mapping, Optional, Sequence  # signatures

RING_MAX = 200  # recommendation 7: the list shows the newest 200 entries
MERGE_WINDOW_S = 10.0  # repeats closer than this fold into one entry (p3 repeats every 2 s)
DETAIL_MAX_BYTES = 512  # serialised detail cap per entry (low_battery is ~260 B)


def _detail(raw: Any) -> Optional[Dict[str, Any]]:
    """detail as a dict, or a type/kind stub when it is too large to carry."""
    if not isinstance(raw, Mapping):  # absent or not an object
        return None  # nothing to show
    d = dict(raw)  # a private copy: the ring must not alias producer data
    try:  # a non-JSON value must not break the event callback
        size = len(json.dumps(d, ensure_ascii=False).encode("utf-8"))  # wire size
    except (TypeError, ValueError):  # unserialisable content
        size = DETAIL_MAX_BYTES + 1  # treat it as oversized
    if size <= DETAIL_MAX_BYTES:  # small enough to carry whole
        return d  # as sent
    # Oversized: keep the two keys the drawer words its text on, say it was cut.
    return {"type": d.get("type"), "kind": d.get("kind"), "truncated": True}  # stub


def _latlon(p: Any) -> Optional[Dict[str, float]]:
    """{lat, lon} when p carries two numbers, else None (never a fake 0,0)."""
    if not isinstance(p, Mapping):  # absent
        return None  # no place
    lat, lon = p.get("lat"), p.get("lon")  # WGS84 degrees
    ok = all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in (lat, lon))  # numbers
    return {"lat": float(lat), "lon": float(lon)} if ok else None  # both or nothing


def ring_item(body: Mapping[str, Any], sev: Optional[str], cat: Optional[str],
              robot_pose: Optional[Mapping[str, Any]], ts: Any = None) -> Dict[str, Any]:
    """One event body -> one ring entry (count 1)."""
    pos = _latlon(body.get("pos"))  # the event's own place first
    pos_src = "event" if pos is not None else None  # where pos came from
    if pos is None:  # the producer gave none (every producer today)
        pos = _latlon(robot_pose)  # robot position at arrival, if fresh
        pos_src = "robot" if pos is not None else None  # or nowhere at all
    stamp = body.get("ts") if ts is None else ts  # producer display stamp
    return {"eid": body.get("eid") or body.get("event_id"),  # identity
            "title": body.get("title") or body.get("message"),  # English title
            "sev": sev, "cat": cat,  # from the key (main_wiring._event_sev_cat)
            "code": body.get("code"), "src": body.get("src"),  # producer extras
            "detail": _detail(body.get("detail")),  # what the drawer words
            "ts": stamp, "first_ts": stamp, "count": 1,  # one occurrence so far
            "pos": pos, "pos_src": pos_src}  # click-to-flash place


def merge_key(item: Mapping[str, Any]) -> tuple:
    """What makes two entries the same thing happening again."""
    d = item.get("detail") or {}  # may be None
    sub = d.get("type") or d.get("kind")  # producers use one or the other
    return (item.get("cat"), item.get("sev"), item.get("title"), sub)  # exact repeat


def push(ring: Sequence[Mapping[str, Any]], item: Mapping[str, Any],
         window_s: float = MERGE_WINDOW_S, cap: int = RING_MAX) -> List[Dict[str, Any]]:
    """A NEW list with item folded in (callers replace, never mutate: the web
    thread reads the old list under the GIL while this runs)."""
    out = [dict(e) for e in ring]  # copy: the old list stays readable
    ts = item.get("ts")  # newest occurrence stamp
    key = merge_key(item)  # what to look for
    if isinstance(ts, (int, float)) and not isinstance(ts, bool):  # no stamp -> never merge
        for i in range(len(out) - 1, -1, -1):  # newest first: the latest same-key entry
            prev = out[i]  # candidate
            if merge_key(prev) != key:  # something else
                continue  # keep looking
            last = prev.get("ts")  # its newest occurrence
            if isinstance(last, (int, float)) and 0 <= ts - last <= window_s:  # a repeat
                merged = dict(item)  # newest eid / ts / detail / pos
                merged["count"] = int(prev.get("count") or 1) + int(item.get("count") or 1)  # tally
                merged["first_ts"] = prev.get("first_ts", last)  # since when
                del out[i]  # it moves to the end
                out.append(merged)  # newest last
                return out[-cap:]  # trim
            break  # the latest same-key entry is too old: a new episode
    out.append(dict(item))  # a new entry
    return out[-cap:]  # oldest fall off


def seed(rows: Sequence[Mapping[str, Any]], cap: int = RING_MAX,
         window_s: float = MERGE_WINDOW_S) -> List[Dict[str, Any]]:
    """record.db rows (oldest first, event/recent.py) -> the start-up ring."""
    ring: List[Dict[str, Any]] = []  # built oldest to newest
    for r in rows:  # replay in order so merges match the live path
        item = ring_item(r, r.get("sev"), r.get("cat"), None)  # stored pos only
        ring = push(ring, item, window_s=window_s, cap=cap)  # same fold as live
    return ring  # newest last
