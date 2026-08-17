"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: main_wiring.py
Brief: p3_task voice-loop wiring -- subscribe cmd/task + RECORD into task.db

Description:
The live P3 half of voice/text -> task.db (15 S3.3). Earlier this file only
logged cmd/task; now it opens task.db and records each task-create request:

  * open one aiosqlite connection to task.db (WAL + the 15 S9.1 REQUIRED
    PRAGMAs) and apply the DDL (CREATE ... IF NOT EXISTS is idempotent);
  * subscribe cmd/task. The Zenoh callback runs on a RUST thread, so it does
    NO async and NO db work (CLAUDE.md 4.2): it decodes the frame and hands it
    to the asyncio loop via loop.call_soon_threadsafe -> an asyncio.Queue;
  * a consumer coroutine drains the queue and calls record_task_from_payload
    (dedup + BEGIN IMMEDIATE transaction), then reflects the recorded task into
    state/task so p5/HMI see it;
  * a heartbeat line every few seconds so p5 sees life.

Why an asyncio loop here: aiosqlite is async (15 S9 forbids the sync sqlite3
driver on the event loop). The whole wiring therefore runs under asyncio.run;
the sync Zenoh session lives inside it and bridges into the loop by the
threadsafe hand-off above.

The task.db PATH is injected (the voice-loop default is data/run/task.db). The
scheduler that MOVES a recorded task out of 'pending' is PB6 -- until then a
recorded task correctly sits at 'pending' (recorded, not yet validated).
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import time

from xbrain.p3_task.state.task_events import task_event_for_transition


_logger = logging.getLogger("xbrain.p3.wiring")


CMD_TASK_TOPIC = "cmd/task"
STATE_TASK_TOPIC = "state/task"
STATE_LINK_TOPIC = "state/link"          # 11 S4.6 cloud-link level -> F-5 return_home

# Voice-loop default. All four DBs live under data/run/ (operator 2026-08-12).
DEFAULT_TASK_DB = "/opt/xbrain_v6/data/run/task.db"


def _now_mono_ms() -> int:
    return int(time.monotonic() * 1000)


def _today_yyyymmdd() -> str:
    # DISPLAY value for the task_id (t-YYYYMMDD-NNN): operators read/say it.
    # strftime is a wall-clock READ but not a timing decision, so it is outside
    # the CLK-C1 ban (which is about timeouts/age via time.time/datetime.now).
    return time.strftime("%Y%m%d")


def run_voice_loop_wiring(stop_flag: dict,
                          heartbeat_period_s: float = 5.0,
                          task_db_path: str = DEFAULT_TASK_DB) -> int:
    """Block until stop_flag['stop'] is truthy. Returns 0 on clean shutdown."""
    return asyncio.run(
        _amain(stop_flag, heartbeat_period_s, task_db_path))


async def _amain(stop_flag: dict, heartbeat_period_s: float,
                 task_db_path: str) -> int:
    from xbrain.common.runtime.session_ctx import open_planes
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    from xbrain.p3_task.persistence.base import open_configured
    from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
    from xbrain.p3_task.schedule.driver import scheduler_tick

    # Open + configure + create-schema THROUGH the persistence layer -- this
    # file imports no sqlite driver of its own (CLAUDE.md 4.1); it holds the
    # connection only to hand it to the DAO / recorder.
    _logger.info("p3 wiring: opening task.db at %s", task_db_path)
    conn = await open_configured(task_db_path, ALL_DDL_STATEMENTS)
    try:
        dao = TasksDAO(conn)

        loop = asyncio.get_running_loop()
        queue: asyncio.Queue = asyncio.Queue()
        recorded = 0

        _logger.info("p3 wiring: opening GEN session")
        with open_planes(("gen",)) as gen:
            state_pub = gen.declare_publisher(STATE_TASK_TOPIC)
            state_pub.put(json.dumps({
                "schema": "state_task_v1", "active_task": None,
                "mono_ms": _now_mono_ms()}).encode("utf-8"))

            # F-5 (11 S4.6.4): watch P5's cloud-link level and inject one
            # return_home at L3 (cloud down past rtb_s). The subscriber only STORES
            # the latest snapshot (RUST thread -> no db, no await, CLAUDE.md 4.2);
            # the loop below reads it and does the insert.
            from xbrain.p3_task.lifecycle.link_loss import (
                RETURN_HOME_PRIORITY, LinkLossReturnTrigger,
                maybe_inject_return_home,
            )
            link_holder: dict = {}
            return_trigger = LinkLossReturnTrigger()

            # 11 S6.2 task events: emit event/{sev}/task on scheduler transitions.
            # Published like any producer (RUST thread never touches this -- it runs
            # on the loop from _publish); p5's event/** subscriber persists it. eid is
            # boot-unique (a raw seq would collide across a P3 restart).
            _task_evt_seq = [0]
            _task_evt_boot = os.urandom(3).hex()

            def _emit_task_event(task_id: str, to_state: str,
                                 kind: str, sev: str) -> None:
                _task_evt_seq[0] += 1
                gen.put("event/%s/task" % sev, json.dumps({
                    "eid": "task-%s-%d" % (_task_evt_boot, _task_evt_seq[0]),
                    "title": "task %s %s" % (task_id, kind),
                    "detail": {"kind": kind, "task_id": task_id, "state": to_state},
                    "src": "p3_task", "ts": 0.0,
                }).encode("utf-8"))

            def _on_link(sample) -> None:
                try:
                    p = json.loads(bytes(sample.payload).decode("utf-8"))
                except Exception:      # noqa: BLE001
                    return
                # Hand the fields to the loop thread; never touch the db here.
                loop.call_soon_threadsafe(link_holder.update, {
                    "level": p.get("level"),
                    "gw_start_mono": p.get("gw_start_mono"),
                    "link_epoch": p.get("link_epoch"),
                    "disconnected_s": p.get("disconnected_s"),
                })

            def _on_task(sample) -> None:
                # RUST THREAD: decode only, then hand to the loop. No await, no
                # db, no state_pub here (CLAUDE.md 4.2 -- the callback must
                # return fast and touch nothing the loop owns).
                try:
                    payload = json.loads(bytes(sample.payload).decode("utf-8"))
                except Exception:      # noqa: BLE001
                    _logger.warning("p3 malformed cmd/task payload")
                    return
                loop.call_soon_threadsafe(queue.put_nowait, payload)

            # Sub handles held in a list (strong ref, CLAUDE.md 4.3).
            _subs = [gen.declare_subscriber(CMD_TASK_TOPIC, _on_task),
                     gen.declare_subscriber(STATE_LINK_TOPIC, _on_link)]
            _logger.info("p3 wiring: subscribed %s + %s (task.db + F-5 return_home)",
                         CMD_TASK_TOPIC, STATE_LINK_TOPIC)

            last_hb = time.monotonic()
            try:
                while not stop_flag.get("stop"):
                    # Wait for a task or wake up to heartbeat / re-check stop.
                    try:
                        payload = await asyncio.wait_for(queue.get(),
                                                         timeout=0.5)
                    except asyncio.TimeoutError:
                        payload = None
                    if payload is not None:
                        recorded += await _record_one(
                            conn, dao, payload, state_pub)
                    # F-5 (11 S4.6.4): if the cloud link is L3, inject one
                    # return_home BEFORE the tick so it is dispatched this pass.
                    # Idempotent per outage (LinkLossReturnTrigger); a bad insert
                    # is logged, never crashes the loop.
                    try:
                        rtb_id = await maybe_inject_return_home(
                            conn, dao, return_trigger, link_holder,
                            priority=RETURN_HOME_PRIORITY,
                            now_mono_ms=_now_mono_ms())
                        if rtb_id:
                            _logger.warning(
                                "p3 F-5: cloud link L3 (epoch %s, %.0fs) -> "
                                "injected return_home %s",
                                link_holder.get("link_epoch"),
                                link_holder.get("disconnected_s") or 0.0, rtb_id)
                    except Exception as exc:      # noqa: BLE001
                        _logger.error("p3 F-5 return_home inject failed: %s", exc)
                    # Drive the machine every pass: validate pending -> ready,
                    # dispatch the top ready -> running (PB6). Cheap on a small
                    # table; a bad tick is logged, never crashes the loop.
                    try:
                        await scheduler_tick(
                            conn, dao, now_mono_ms=_now_mono_ms(),
                            on_transition=_make_publish(state_pub,
                                                        _emit_task_event))
                    except Exception as exc:      # noqa: BLE001
                        _logger.error("p3 scheduler tick failed: %s", exc)
                    now = time.monotonic()
                    if now - last_hb >= heartbeat_period_s:
                        _logger.info("p3 alive; recorded=%d qdepth=%d",
                                     recorded, queue.qsize())
                        last_hb = now
            finally:
                for s in _subs:
                    try:
                        s.undeclare()
                    except Exception:      # noqa: BLE001
                        pass
                try:
                    state_pub.undeclare()
                except Exception:          # noqa: BLE001
                    pass
    finally:
        await conn.close()
    return 0


def _make_publish(state_pub, emit_task_event=None):
    """Build the scheduler on_transition callback: publish each task state change on
    state/task (event + 1 Hz, 11 S2.2.2) so p5/HMI/cloud can rebuild the machine, log
    it, AND emit the 11 S6.2 task event (accept/reject/start/complete/fail) via
    emit_task_event when the transition warrants one. reason is non-empty only on a
    validate_fail."""
    async def _publish(task_id: str, to_state: str, reason: str) -> None:
        if reason:
            _logger.info("p3 task %s -> %s (%s)", task_id, to_state, reason)
        else:
            _logger.info("p3 task %s -> %s", task_id, to_state)
        state_pub.put(json.dumps({
            "schema": "state_task_v1",
            "active_task": {"task_id": task_id, "state": to_state,
                            "mono_ms": _now_mono_ms()},
        }).encode("utf-8"))
        # 11 S6.2 task event -- a separate stream from the state/task heartbeat.
        if emit_task_event is not None:
            ev = task_event_for_transition(to_state, reason)
            if ev is not None:
                emit_task_event(task_id, to_state, ev[0], ev[1])
    return _publish


async def _record_one(conn, dao, payload, state_pub) -> int:
    """Record one cmd/task payload; reflect the outcome into state/task.
    Returns 1 if a NEW task was recorded, else 0 (duplicate/skipped/error)."""
    try:
        out = await record_task_from_payload(
            conn, dao, payload,
            date_str=_today_yyyymmdd(), now_mono_ms=_now_mono_ms())
    except Exception as exc:               # noqa: BLE001
        # A bad row must not kill the loop -- log and drop this one task.
        _logger.error("p3 record failed: %s", exc)
        return 0
    if out.kind == "recorded":
        _logger.info("p3 recorded task %s type=%s state=%s",
                     out.task_id, payload.get("task_request", {}).get(
                         "task_type"), out.state)
        state_pub.put(json.dumps({
            "schema": "state_task_v1",
            "active_task": {"task_id": out.task_id, "state": out.state,
                            "admitted_mono_ms": _now_mono_ms()},
        }).encode("utf-8"))
        return 1
    if out.kind == "duplicate":
        _logger.info("p3 duplicate task %s ignored (TSK-12)", out.task_id)
    else:
        # A control/device frame reached cmd/task (e.g. a non-create); log the
        # intent for visibility but record nothing.
        _logger.info("p3 cmd/task not a task-create (intent=%s); no row",
                     payload.get("intent_id"))
    return 0


# Import at module scope so the recorder is a stable reference (not re-imported
# per call). Kept after the functions to avoid a circular import at load.
from xbrain.p3_task.ingest.task_recorder import record_task_from_payload  # noqa: E402
