"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: event_e2e_check.py
Brief: ORIN end-to-end check for the event subsystem (real record.db + real Zenoh)

Description:
Proves the SW-12 event subsystem end to end on the real GEN router, using the
parts the :memory: unit tests cannot exercise: a real record.db file (3-connection
WAL model) and the backfill replay over real Zenoh. It stands up a live subsystem,
a Zenoh subscriber that feeds it (the main_wiring path), a loopback cloud stub that
subscribes event/replay/**, injects one alarm event, checks it persisted, triggers
a backfill, and checks the stub received begin/item/end over the wire.

Run on the ORIN (a GEN router must be up):
  XBRAIN_ROBOT_ID=dev PYTHONPATH=/opt/xbrain_v6 python3 scripts/dev/event_e2e_check.py

Not a pytest: it needs the live router (marked needs_orin territory) and prints a
human PASS/FAIL. It does NOT disturb a running p5 -- it opens its own session, its
injected event is a test event, and R-2 keeps replay off the live event path.
"""

from __future__ import annotations

import asyncio
import json
import os
import shutil
import sys
import tempfile
import time


def _query(db_path, sql, args=()):
    async def q():
        import aiosqlite
        async with aiosqlite.connect(db_path, isolation_level=None) as c:
            cur = await c.execute(sql, args)
            return await cur.fetchall()
    return asyncio.run(q())


def main() -> int:
    import zenoh

    from xbrain.common.zenoh.session_factory import build_session_config
    from xbrain.p5_gateway.runtime.event_subsystem import EventSubsystem

    rid = os.environ.get("XBRAIN_ROBOT_ID", "dev")
    workdir = tempfile.mkdtemp(prefix="xbrain_evt_e2e_")
    db_path = os.path.join(workdir, "record.db")
    ok = True

    subs = EventSubsystem(
        rid, db_path, db_path + ".degrade.jsonl",
        now_iso=lambda: "2026-08-17T02:00:00Z", now_mono=time.monotonic)
    if not subs.start():
        print("FAIL: subsystem did not start (record.db not opened)")
        return 1
    print("ok: subsystem started, record.db at", db_path)

    sess = zenoh.open(build_session_config("gen"))
    stub_got: list = []
    try:
        # main_wiring path: feed the subsystem from event/** (skip replay/ack, R-2).
        def _feed(sample):
            k = str(sample.key_expr).split("/")
            if len(k) > 1 and k[1] in ("replay", "ack"):
                return
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            ev = {
                "eid": d.get("eid"), "rid": rid, "sev": k[1] if len(k) > 1 else None,
                "cat": k[2] if len(k) > 2 else None, "title": d.get("title", ""),
                "detail": d.get("detail", {}), "src": d.get("src", "test"),
                "ts": 100.0, "ts_sync": 0, "detected_at": "2026-08-17 10:00:00",
                "created_at": "2026-08-17T02:00:00Z",
            }
            if ev["eid"] and ev["sev"] and ev["cat"]:
                subs.submit_event(ev, link_connected=False)   # queue for backfill
        feed_sub = sess.declare_subscriber("event/**", _feed)

        # Loopback cloud stub: collect event/replay/** (what the cloud would get).
        stub_sub = sess.declare_subscriber(
            "event/replay/**",
            lambda s: stub_got.append(json.loads(bytes(s.payload).decode("utf-8"))))

        # Wire the subsystem's replay publisher (relative keys, per channel).
        rp_alarm = sess.declare_publisher("event/replay/alarm")
        rp_normal = sess.declare_publisher("event/replay/normal")
        subs.set_replay_publisher(
            lambda _k, data: (rp_alarm if data.get("channel") == "alarm"
                              else rp_normal).put(
                json.dumps(data).encode("utf-8")))

        # Inject one alarm event (need_ack=1 -> stays delivered=0, queued).
        ev_pub = sess.declare_publisher("event/alarm/intrusion")
        ev_pub.put(json.dumps({
            "eid": "e2e-1", "title": "intruder", "detail": {"track_id": 42},
            "src": "test"}).encode("utf-8"))
        time.sleep(1.0)   # let the subscriber + persist coroutine run

        rows = _query(db_path, "SELECT eid, channel, delivered FROM events")
        if rows == [("e2e-1", "alarm", 0)]:
            print("ok: event persisted (channel=alarm, delivered=0, queued)")
        else:
            print("FAIL: expected [('e2e-1','alarm',0)], got", rows)
            ok = False

        # Trigger a backfill: the runner replays over real Zenoh to the stub.
        subs.trigger_backfill()
        time.sleep(1.0)
        kinds = [m.get("kind") for m in stub_got]
        if kinds.count("begin") >= 1 and kinds.count("item") >= 1 and \
                kinds.count("end") >= 1:
            print("ok: cloud stub received backfill over Zenoh:", kinds)
        else:
            print("FAIL: stub did not get begin/item/end, got", kinds)
            ok = False

        # The replayed item carried our event verbatim (R-1).
        items = [m for m in stub_got if m.get("kind") == "item"]
        if items and items[0]["event"].get("eid") == "e2e-1":
            print("ok: replay item carried eid verbatim (R-1)")
        else:
            print("FAIL: replay item missing/rewrote the event:", items)
            ok = False

        for e in (ev_pub, rp_alarm, rp_normal, feed_sub, stub_sub):
            try:
                e.undeclare()
            except Exception:      # noqa: BLE001
                pass
    finally:
        sess.close()
        subs.stop()
        shutil.rmtree(workdir, ignore_errors=True)

    print("PASS" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
