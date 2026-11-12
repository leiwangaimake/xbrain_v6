"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_event_ring.py
Brief: HMI event list ring -- 200 entries, exact repeats merged, robot position, record.db seed

Description:
Pins hmi/event_ring.py and event/recent.py (user design 2026-11-12, the bottom
event drawer; recommendations 5 / 7 / 8). Each test names the mutant that turns
it red. What a lazy ring would get wrong:
  * the 2 s low-battery alarm filling the whole list (no merge);
  * two different tasks folded into one entry (merge key too loose);
  * a merged alarm forgetting when it started (first_ts lost);
  * a fake 0,0 position, or a stale pose stamped as the event place;
  * a p5 restart emptying the list (no seed) or a broken record.db killing p5.
"""

from __future__ import annotations  # postponed annotations

import json  # detail column is JSON text
import sqlite3  # BUSINESS-IMPORT-OK(test-fixture): builds a throwaway record.db in tmp_path

from xbrain.p5_gateway.event.recent import read_recent  # record.db seed reader
from xbrain.p5_gateway.hmi import event_ring as er  # the ring under test


def _lb(ts, eid="e"):
    """A low-battery body as p3 sends it every 2 s (the one that flooded the old ring)."""
    return {"eid": eid, "title": "low battery, returning to charge", "ts": ts,  # one repeat
            "src": "p3_task", "detail": {"type": "low_battery", "soc_pct": 9}}  # detail.type keys the merge


def _push(ring, body, sev="alarm", cat="charging", pose=None):
    """The live path in one call: build the entry, fold it in (main_wiring._ring_push)."""
    return er.push(ring, er.ring_item(body, sev, cat, pose))  # a NEW list


def test_a_continuous_alarm_is_one_entry_with_count_and_start():
    """MUTATION: no merge -> 30 entries -> red; first_ts overwritten -> red;
    count not summed -> red."""
    ring = []  # empty at start
    for i in range(30):  # one minute of the 2 s alarm
        ring = _push(ring, _lb(100.0 + 2 * i, eid="lb-%d" % i))  # each a new eid
    assert len(ring) == 1  # one entry, not thirty
    e = ring[0]  # the merged entry
    # count summed, start kept, newest ts / eid carried
    assert (e["count"], e["first_ts"], e["ts"], e["eid"]) == (30, 100.0, 158.0, "lb-29")  # tally


def test_a_gap_longer_than_the_window_starts_a_new_entry():
    """MUTATION: window ignored -> one entry -> red."""
    ring = _push([], _lb(100.0))  # first episode
    ring = _push(ring, _lb(100.0 + er.MERGE_WINDOW_S + 0.1))  # just past the window
    assert [e["count"] for e in ring] == [1, 1]  # two separate entries


def test_only_exact_repeats_merge():
    """Different title (another task), sev or detail.type -> separate entries.
    MUTATION: key on cat only -> red."""
    ring = []  # empty at start
    cases = [  # each differs from the one before in exactly one key field
        ({"title": "task t-1 started", "ts": 1.0, "detail": {"kind": "started"}}, "info"),  # base
        ({"title": "task t-2 started", "ts": 2.0, "detail": {"kind": "started"}}, "info"),  # title
        ({"title": "task t-2 started", "ts": 3.0, "detail": {"kind": "started"}}, "warn"),  # sev
        ({"title": "task t-2 started", "ts": 4.0, "detail": {"kind": "done"}}, "warn"),  # kind
    ]
    for body, sev in cases:  # all within the 10 s window
        ring = _push(ring, body, sev=sev, cat="task")  # same cat throughout
    assert len(ring) == 4  # nothing merged


def test_a_merged_entry_moves_to_the_end():
    """Grant / release alternating: each folds into its own entry, newest last.
    MUTATION: merge in place -> order wrong -> red."""
    ring = []  # empty at start
    for i, t in enumerate(["grant", "release", "grant"]):  # the speaker arbitration pair
        ring = _push(ring, {"title": t, "ts": float(i)}, sev="info", cat="arbitration")  # 1 s apart
    assert [(e["title"], e["count"]) for e in ring] == [("release", 1), ("grant", 2)]  # newest last


def test_the_ring_is_capped_and_never_mutates_its_input():
    """MUTATION: cap ignored -> 250 -> red; in-place append -> old list grows -> red."""
    ring = []  # empty at start
    for i in range(er.RING_MAX + 50):  # 50 more than fit
        ring = _push(ring, {"title": "t%d" % i, "ts": float(i)}, sev="info", cat="task")  # distinct
    assert len(ring) == er.RING_MAX == 200 and ring[-1]["title"] == "t249"  # oldest fell off
    before = list(ring)  # what the web thread may be reading
    _push(ring, {"title": "x", "ts": 999.0}, sev="info", cat="task")  # result discarded
    assert ring == before  # the input list is untouched


def test_position_event_first_then_fresh_robot_never_zero():
    """MUTATION: robot pose overriding the event's own pos -> red; a pose
    without lat/lon stamped as 0,0 -> red."""
    pose = {"lat": 34.7, "lon": 135.5, "yaw_deg": 3}  # a fresh robot fix
    own = er.ring_item({"pos": {"lat": 1.0, "lon": 2.0}}, "warn", "x", pose)  # event has a place
    assert (own["pos"], own["pos_src"]) == ({"lat": 1.0, "lon": 2.0}, "event")  # it wins
    rob = er.ring_item({}, "warn", "x", pose)  # event has none (every producer today)
    assert (rob["pos"], rob["pos_src"]) == ({"lat": 34.7, "lon": 135.5}, "robot")  # robot place
    none = er.ring_item({}, "warn", "x", {"lat": None, "lon": None})  # no-fix shell
    assert (none["pos"], none["pos_src"]) == (None, None)  # nothing, not 0,0
    assert er.ring_item({}, "warn", "x", None)["pos"] is None  # stale pose -> None


def test_detail_rides_along_but_is_capped():
    """MUTATION: detail dropped -> the drawer cannot word it -> red; no cap ->
    a 10 kB detail rides every WS delta -> red."""
    small = er.ring_item({"detail": {"type": "rtk_lost"}}, "alarm", "rtk", None)  # small
    assert small["detail"] == {"type": "rtk_lost"}  # carried whole
    big = er.ring_item({"detail": {"kind": "k", "blob": "x" * 5000}}, "info", "x", None)  # 5 kB
    assert big["detail"] == {"type": None, "kind": "k", "truncated": True}  # stub only


def _db(path, rows):
    """A record.db with just the columns event/recent.py reads."""
    c = sqlite3.connect(path)  # throwaway file in tmp_path
    c.execute("CREATE TABLE events (id INTEGER PRIMARY KEY, eid TEXT, sev TEXT, cat TEXT,"  # id order
              " title TEXT, detail TEXT, src TEXT, ts REAL, location_lat REAL,"  # payload
              " location_lon REAL)")  # stamped place
    c.executemany("INSERT INTO events (eid,sev,cat,title,detail,src,ts,location_lat,"  # rows
                  "location_lon) VALUES (?,?,?,?,?,?,?,?,?)", rows)  # in insert order
    c.commit()  # visible to the read-only reader
    c.close()  # release the file


def test_seed_reads_newest_rows_oldest_first_and_folds_them(tmp_path):
    """MUTATION: ORDER BY id ASC (oldest rows) -> red; rows not reversed ->
    the fold sees time backwards and never merges -> red; bad detail JSON
    raising -> p5 start-up dies -> red."""
    p = str(tmp_path / "record.db")  # fixture path
    rows = [("old", "info", "task", "very old", None, "p3", 1.0, None, None)]  # outside limit
    rows += [("lb%d" % i, "alarm", "charging", "low battery, returning to charge",  # 5 repeats
              json.dumps({"type": "low_battery"}), "p3", 100.0 + 2 * i, None, None)  # 2 s apart
             for i in range(5)]  # lb0..lb4
    rows += [("bad", "warn", "x", "bad detail", "{not json", "p2", 200.0, 34.7, 135.5)]  # broken detail
    _db(p, rows)  # write them
    got = read_recent(p, limit=6)  # the newest 6
    assert [r["eid"] for r in got] == ["lb0", "lb1", "lb2", "lb3", "lb4", "bad"]  # oldest first
    assert got[-1]["detail"] is None and got[-1]["pos"] == {"lat": 34.7, "lon": 135.5}  # row survives
    ring = er.seed(got)  # the start-up fold
    assert [(e["title"], e["count"]) for e in ring] == [  # the 5 repeats merged
        ("low battery, returning to charge", 5), ("bad detail", 1)]  # in order
    assert ring[0]["first_ts"] == 100.0 and ring[1]["pos_src"] == "event"  # start + stored place


def test_an_unreadable_record_db_gives_none(tmp_path):
    """MUTATION: let the sqlite error escape -> p5 does not start -> red."""
    assert read_recent(str(tmp_path / "missing.db")) is None  # mode=ro on a missing file


def test_snapshot_carries_server_ts_and_the_new_entry_fields():
    """The drawer reads count / first_ts / detail / pos_src through events_group,
    and judges "still repeating" against server_ts (P5's clock, not the
    browser's). MUTATION: drop server_ts from build_snapshot -> red; _event
    not passing detail / count -> the drawer words nothing -> red."""
    from xbrain.p5_gateway.hmi.data_readers import build_snapshot  # the projection

    item = er.ring_item(_lb(100.0), "alarm", "charging", {"lat": 1.0, "lon": 2.0})  # robot place
    snap = build_snapshot(events=[item], server_ts=123.5)  # as the web server builds it
    assert snap["server_ts"] == 123.5  # top-level, P5's clock
    got = snap["events"]["items"][0]  # the projected entry
    assert got["detail"]["type"] == "low_battery" and got["count"] == 1  # wording inputs
    assert (got["first_ts"], got["pos"], got["pos_src"]) == (  # merge + place fields
        100.0, {"lat": 1.0, "lon": 2.0}, "robot")  # passed through
