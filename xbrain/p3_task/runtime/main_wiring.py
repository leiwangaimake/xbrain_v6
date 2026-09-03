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
  * a consumer coroutine drains the queue and calls handle_task_payload
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
from typing import Dict, Optional

from xbrain.p3_task.state.task_events import task_event_for_transition


_logger = logging.getLogger("xbrain.p3.wiring")


CMD_TASK_TOPIC = "cmd/task"
CMD_ESTOP_TOPIC = "cmd/estop"            # 15 S11.1 ES-1: freeze scheduling
CMD_TASK_ACK_TOPIC = "cmd/task/ack"      # 11 S7.7 Ack; W2/W7 require ack <= 2s
STATE_TASK_TOPIC = "state/task"
STATE_LINK_TOPIC = "state/link"          # 11 S4.6 cloud-link level -> F-5 return_home
QUERY_TASKS_TOPIC = "query/tasks"        # 11 S12.4 HMI task-panel queryable (P5 pulls)
QUERY_TASKS_TIMEOUT_S = 2.0              # cap the zenoh-thread block on a db read
STATE_GEO_OBJECTS_TOPIC = "state/geo/objects"  # 11 S7.10A geo geometry broadcast (P5 -> HMI)
STATE_GEO_MANIFEST_TOPIC = "state/geo/manifest"  # 11 S7.10 sync baseline (P1/P4/P5)
GEO_PUBLISH_PERIOD_S = 5.0               # geo re-publish cadence (>= 0.1 Hz keepalive)
CMD_TEACH_TOPIC = "cmd/teach"            # 11 S12A.4: P3 owns the recording session
CMD_TEACH_ACK_TOPIC = "cmd/teach/ack"
STATE_TEACH_TOPIC = "state/teach"        # 11 S12A.5, event + 1 Hz
STATE_POSE_TOPIC = "state/pose"          # 11 S3.3, the sampling source
# The three state sources the S12A.3 arming checks read. None of them has a
# publisher in this build; the runtime refuses to arm and NAMES what is missing
# rather than defaulting the gate to pass (see teach/runtime.py design point 2).
HEALTH_SUMMARY_TOPIC = "health/summary"
STATE_ROBOT_TOPIC = "state/robot"
STATE_POWER_TOPIC = "state/power"
STATE_TELEOP_TOPIC = "state/teleop"
TEACH_PUBLISH_PERIOD_S = 1.0             # S12A.5 floor
TASK_STATE_PERIOD_S = 1.0                # 11 S2.2.2 state/task = event + 1 Hz
CMD_GEO_TOPIC = "cmd/geo"                # 11 S7.9: P3 is the sole cmd/geo subscriber
CMD_GEO_ACK_TOPIC = "cmd/geo/ack"        # 11 S7.9.4: the answer goes back to the sender
CMD_FENCE_TOPIC = "cmd/fence"            # 11 S9A.3: P3 is the sole cmd/fence publisher
FENCE_PUBLISH_PERIOD_S = 5.0             # fence re-publish cadence (>= 0.1 Hz keepalive)
FENCE_SET_ID = "fs-active"               # the single active FenceSet's id (11 S9A.2)

# Voice-loop default. All four DBs live under data/run/ (operator 2026-08-12).
DEFAULT_TASK_DB = "/opt/xbrain_v6/data/run/task.db"
DEFAULT_GEO_DB = "/opt/xbrain_v6/data/run/geo.db"
DEFAULT_FENCE_DB = "/opt/xbrain_v6/data/run/fence.db"


def _now_mono_ms() -> int:
    return int(time.monotonic() * 1000)


def _now_wall_ms() -> int:
    # AUDIT value for the geo tables' updated_ms column (15 S9.3): it is the
    # "when was this object last edited" the HMI shows and the token
    # state/geo/objects derives catalog_rev from. Monotonic ms cannot serve
    # either -- it restarts at every boot, so an object edited before the last
    # reboot would sort as NEWER than one edited after it. Same wall-vs-mono
    # split as _now_utc_iso above; ages and timeouts still use _now_mono_ms.
    # WALL-CLOCK-OK(record): geo edit timeline, 15 S9.3 updated_ms
    return int(time.time() * 1000)


def _today_yyyymmdd() -> str:
    # DISPLAY value for the task_id (t-YYYYMMDD-NNN): operators read/say it.
    # strftime is a wall-clock READ but not a timing decision, so it is outside
    # the CLK-C1 ban (which is about timeouts/age via time.time/datetime.now).
    return time.strftime("%Y%m%d")


#: 本次开机的标识, 进程级唯一. 15 S9.5 的 duration_sec 判据依赖它:
#: "若终态时的 boot != started_boot, duration_sec 写 NULL, 不得回退用墙钟
#: 差值充数". 单调钟只在同一次开机内可比(11 CLK-C4), 跨重启的 started_mono
#: 与当前 now_mono 之差是个没有意义的数, 而它看起来完全像个正常时长.
#:
#: NO 不在各处各自 os.urandom() -- 那样一个进程里会有多个 boot id, 比对就
#: 恒不相等, 于是 duration_sec 永远为 NULL(一条永远绿的"跨重启"判定).
#: 现状: teach 与 task_event 各自生成过一个, 本常量是唯一来源.
_BOOT_ID = os.urandom(3).hex()


def _now_utc_iso() -> str:
    # DISPLAY/AUDIT value for tasks.created_at (15 S9.5): the wall-clock dispatch
    # time the HMI task panel shows as 下发时间 (17 S6.8.4 field 2), formatted in
    # the viewer's GPS-derived zone. Stored as UTC ISO 'Z' so the HMI can render
    # it in ANY zone (same wall-vs-monotonic split as _today_yyyymmdd: this is a
    # display read, NOT a timing decision, so it is outside the CLK-C1 ban -- age
    # and timeouts still use _now_mono_ms). gmtime() => UTC.
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def run_voice_loop_wiring(stop_flag: dict,
                          heartbeat_period_s: float = 5.0,
                          task_db_path: str = DEFAULT_TASK_DB,
                          geo_db_path: str = DEFAULT_GEO_DB,
                          fence_db_path: str = DEFAULT_FENCE_DB,
                          enu_origin: Optional[Dict[str, float]] = None) -> int:
    """Block until stop_flag['stop'] is truthy. Returns 0 on clean shutdown.

    enu_origin 来自 common.geo.enu_origin(L4 sites/{site_id}.yaml), 由
    __main__ 从解析产物取出后传入. 默认 None 保留是为了既有调用方(测试)不必
    全部改签名 -- 但生产路径必须传, 否则 FenceSet 的必填字段为空(11 S9A.2).
    """
    return asyncio.run(
        _amain(stop_flag, heartbeat_period_s, task_db_path, geo_db_path,
               fence_db_path, enu_origin))


async def _amain(stop_flag: dict, heartbeat_period_s: float,
                 task_db_path: str, geo_db_path: str = DEFAULT_GEO_DB,
                 fence_db_path: str = DEFAULT_FENCE_DB,
                 enu_origin: Optional[Dict[str, float]] = None) -> int:
    from xbrain.common.runtime.session_ctx import open_planes
    from xbrain.p3_task.dao.simple_daos import FencesDAO
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    from xbrain.p3_task.fence.fence_set import build_fence_set
    from xbrain.p3_task.fence.geom import InvalidFenceSet
    from xbrain.p3_task.geo.objects import read_geo_objects
    from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload
    from xbrain.p3_task.state.geo_events import render_geo_event
    from xbrain.p3_task.state.task_state import (
        read_task_state, wall_iso_to_epoch,
    )
    from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload
    from xbrain.p3_task.ingest.geo_read import build_manifest
    from xbrain.p3_task.teach.runtime import TeachRuntime
    from xbrain.p3_task.persistence.base import open_configured
    from xbrain.p3_task.persistence.schema_geo import (
        FENCE_DB_STATEMENTS, GEO_DB_STATEMENTS,
    )
    from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
    from xbrain.p3_task.schedule.driver import scheduler_tick

    # Open + configure + create-schema THROUGH the persistence layer -- this
    # file imports no sqlite driver of its own (CLAUDE.md 4.1); it holds the
    # connection only to hand it to the DAO / recorder.
    _logger.info("p3 wiring: opening task.db at %s", task_db_path)
    conn = await open_configured(task_db_path, ALL_DDL_STATEMENTS)
    # Second connection: geo.db, read-only use here -- P3 broadcasts its geometry
    # on state/geo/objects (11 S7.10A) so the HMI can render routes/keypoints/docks
    # without P5 ever reading geo.db (11 S7843). A separate handle keeps geo reads
    # off the task.db write path.
    _logger.info("p3 wiring: opening geo.db at %s", geo_db_path)
    geo_conn = await open_configured(geo_db_path, GEO_DB_STATEMENTS)
    # Third connection: fence.db. P3 is the sole cmd/fence publisher (11 S9A.3);
    # it reads the active fences here and broadcasts the FenceSet on cmd/fence so
    # P1/P2/P4/P5 render/consume the real stored geometry (the demo injector is
    # retired). Only the GEOMETRY broadcast half is wired now -- the two-stage
    # stage/ack/commit handshake is deferred until P1 clipping lands (see fence_set).
    _logger.info("p3 wiring: opening fence.db at %s", fence_db_path)
    fence_conn = await open_configured(fence_db_path, FENCE_DB_STATEMENTS)
    try:
        dao = TasksDAO(conn)
        fences_dao = FencesDAO(fence_conn)

        loop = asyncio.get_running_loop()
        queue: asyncio.Queue = asyncio.Queue()
        # 11 S7.9: cmd/geo rides its OWN queue, not the cmd/task one. A geo CRUD
        # frame and a task-create frame are answered on different keys and by
        # different code; sharing a queue would force a type tag on every item
        # and make one loop's back-pressure the other's problem.
        geo_queue: asyncio.Queue = asyncio.Queue()
        # 11 S12A: cmd/teach gets its own queue for the same reason cmd/geo does
        # -- a different key, a different answer topic, different code.
        teach_queue: asyncio.Queue = asyncio.Queue()
        recorded = 0

        _logger.info("p3 wiring: opening GEN session")
        with open_planes(("gen",)) as gen:
            state_pub = gen.declare_publisher(STATE_TASK_TOPIC)
            # 11 S4.4 TaskState, empty shape: the db is not open on this line yet,
            # and a subscriber that arrives before the first loop pass must still
            # see the contract's three lists rather than a different schema it has
            # to special-case (that is how the old active_task placeholder spread).
            state_pub.put(json.dumps({
                "schema": "task_state_v1", "current": None,
                "queue": [], "suspended": []}).encode("utf-8"))
            # 11 S7.10A: broadcast geo geometry (routes/keypoints/docks) so the
            # HMI renders them live without P5 reading geo.db (S7843). Published
            # from the loop below every GEO_PUBLISH_PERIOD_S (>= 0.1 Hz keepalive).
            geo_pub = gen.declare_publisher(STATE_GEO_OBJECTS_TOPIC)
            # 11 S7.10: the manifest is the SYNC baseline -- summaries only, no
            # geometry, so it stays small enough for the 0.1 Hz floor. Its three
            # subscribers each key off a different field: P1 on active_fence,
            # P4 on catalog_rev (GBNF regeneration) and on items (resolving a
            # spoken name to a geo_id), P5 as the cloud diff baseline.
            geo_manifest_pub = gen.declare_publisher(STATE_GEO_MANIFEST_TOPIC)
            # 11 S9A.3: broadcast the active FenceSet from fence.db on cmd/fence so
            # the HMI (and later P1/P2) render/consume the real stored geometry.
            fence_pub = gen.declare_publisher(CMD_FENCE_TOPIC)
            # 11 S7.9.4: every cmd/geo gets an answer here, including refusals.
            geo_ack_pub = gen.declare_publisher(CMD_GEO_ACK_TOPIC)
            # 11 S7.7: every cmd/task frame in the contract shape gets an ack.
            task_ack_pub = gen.declare_publisher(CMD_TASK_ACK_TOPIC)
            # 11 S12A.4 / S12A.5: the recording session answers on cmd/teach/ack
            # and publishes its state on state/teach at 1 Hz plus on change.
            teach_ack_pub = gen.declare_publisher(CMD_TEACH_ACK_TOPIC)
            teach_state_pub = gen.declare_publisher(STATE_TEACH_TOPIC)

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
            _task_evt_boot = _BOOT_ID

            def _emit_task_event(task_id: str, to_state: str,
                                 kind: str, sev: str,
                                 extra: dict = None) -> None:
                _task_evt_seq[0] += 1
                detail = {"kind": kind, "task_id": task_id, "state": to_state}
                # 终态事实随事件带出. 11 S6.1 的 detail 是[按类别自定义]的 JSON,
                # 所以这不是扩协议. 必须走这条路的原因: v2.0 S3.3 的 result 要
                # duration_sec / started_ts / route_id 等, 而 p5 不能读 task.db
                # (平面隔离), 且 11 S4.4 的 TaskState [只列非终态任务] -- 任务
                # 一终结就从广播里消失, 那些字段跟着一起消失. 事件是终态那一刻
                # 唯一还带着任务的报文.
                if extra:
                    detail.update(extra)
                gen.put("event/%s/task" % sev, json.dumps({
                    "eid": "task-%s-%d" % (_task_evt_boot, _task_evt_seq[0]),
                    "title": "task %s %s" % (task_id, kind),
                    "detail": detail,
                    "src": "p3_task", "ts": 0.0,
                }, ensure_ascii=False).encode("utf-8"))

            async def _list_route_ids():
                """V-8 用的路径全集(当前 manifest).

                墓碑不算存在 -- 一条被删的路径与从未存在过, 对"能不能执行这条
                任务"是同一个答案.
                调度器每拍取一次, 不是每条任务查一次: 活跃路径是几十条量级,
                一次读进集合比 N 次点查便宜, 而且校验函数得以保持纯同步.
                """
                cur = await geo_conn.execute(
                    "SELECT geo_id FROM routes WHERE tombstone=0")
                return [r[0] for r in await cur.fetchall()]

            async def _fetch_terminal_facts(task_id: str) -> dict:
                """终态那一刻的任务事实, 供事件带给云端.

                只在终态迁移上读一次库 -- 不是每条事件都读. 字段与 v2.0 S3.3 的
                result.summary 对齐, 时间戳转 epoch(v2.0 要数字, 库里存 ISO).
                """
                cur = await conn.execute(
                    "SELECT task_type, route_geo_id, started_at, finished_at, "
                    "       duration_sec FROM tasks WHERE task_id=?", (task_id,))
                row = await cur.fetchone()
                if row is None:
                    return {}
                return {"task_type": row[0], "route_id": row[1],
                        "started_ts": wall_iso_to_epoch(row[2]),
                        "ended_ts": wall_iso_to_epoch(row[3]),
                        "duration_sec": row[4]}

            # 11 S6.2 geo events: 地理要素 CRUD 审计. applier 已经把
            # (sev, detail.type, detail) 三元组放进 ApplyResult.events; 少的一直
            # 是这个回调 -- handle_geo_payload 的 on_event 默认 None, 于是每一次
            # 改图都不留审计. 与 task 事件同一条发布路径(p5 的 event/** 订阅入库
            # 并转云), eid 同样带 boot token(裸 seq 跨 P3 重启会撞 UNIQUE(eid)).
            _geo_evt_seq = [0]
            _geo_evt_boot = _BOOT_ID

            def _emit_geo_event(sev: str, etype: str, detail: dict) -> None:
                # render_geo_event owns the S6.2 closed set + the info/warn
                # pairing and raises on either violation. handle_geo_payload
                # guards this callback per event, so a raise is logged and the
                # already-committed write still acks accepted.
                _geo_evt_seq[0] += 1
                key, body = render_geo_event(
                    sev, etype, detail,
                    "geo-%s-%d" % (_geo_evt_boot, _geo_evt_seq[0]))
                gen.put(key, json.dumps(
                    body, ensure_ascii=False).encode("utf-8"))

            # 11 S2.2.2 state/task = "event + 1 Hz". Both halves publish through
            # this one builder: the transition callback awaits it (event) and the
            # loop calls it every TASK_STATE_PERIOD_S (floor). Reading the db each
            # time is deliberate -- a cached copy is how `queue` goes stale while
            # `current` looks right, and the read is one indexed SELECT over the
            # non-terminal rows only.
            async def _publish_task_state() -> None:
                state_pub.put(json.dumps(
                    await read_task_state(conn),
                    ensure_ascii=False).encode("utf-8"))

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

            # 15 S11.1 ES-1..3 estop handling. freeze() is idempotent and a
            # scalar assignment, safe from the Rust callback (CLAUDE.md 4.2);
            # the scheduling loop below reads scheduling_permitted() each pass.
            # ES-3 / 15 S11.3: p3 does NOT auto-resume -- there is no time-based
            # unfreeze, only an explicit p2 signal (CLAUDE.md 3.6: no bypass).
            from xbrain.p3_task.lifecycle.estop import (EstopController,
                                                        is_human_resume_command)
            from xbrain.p3_task.lifecycle.estop_suspend import (
                suspend_running_for_estop)
            estop_ctrl = EstopController()

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

            def _on_estop(sample) -> None:
                # RUST THREAD (CLAUDE.md 4.2): ES-1 freeze only. freeze() is
                # idempotent (keeps the first reason) and touches two scalar
                # fields, so it is safe here; no db, no publish. The scheduling
                # loop reads the freeze state. Parse is fail-safe (11 S3.0.1):
                # a malformed estop must still freeze -- an unfrozen scheduler
                # would keep dispatching patrol tasks through an emergency stop.
                estop_ctrl.freeze("estop_soft")
                _logger.warning("p3 ES-1: scheduling FROZEN by cmd/estop "
                                "(no new dispatch; awaits p2 unfreeze, 15 S11.3)")

            def _on_geo(sample) -> None:
                # RUST THREAD: same discipline as _on_task -- decode, hand off,
                # return. Everything about a GeoCommand (permission matrix, db
                # write, ack) happens on the loop thread below (CLAUDE.md 4.2).
                try:
                    payload = json.loads(bytes(sample.payload).decode("utf-8"))
                except Exception:      # noqa: BLE001
                    # Undecodable bytes carry no cmd_id, so there is nobody to
                    # ack: log and drop. Every DECODABLE frame gets an answer.
                    _logger.warning("p3 malformed cmd/geo payload")
                    return
                loop.call_soon_threadsafe(geo_queue.put_nowait, payload)

            def _on_teach(sample) -> None:
                # RUST THREAD: decode and hand off only (CLAUDE.md 4.2).
                try:
                    payload = json.loads(bytes(sample.payload).decode("utf-8"))
                except Exception:      # noqa: BLE001
                    _logger.warning("p3 malformed cmd/teach payload")
                    return
                loop.call_soon_threadsafe(teach_queue.put_nowait, payload)

            # The state caches the S12A.3 arming checks read. Each callback only
            # stores the decoded body; the loop thread hands it to the runtime,
            # so nothing touches the session from a Zenoh thread.
            state_cache: dict = {}

            def _make_state_sink(name: str):
                def _sink(sample) -> None:
                    try:
                        body = json.loads(bytes(sample.payload).decode("utf-8"))
                    except Exception:      # noqa: BLE001
                        return
                    # p1 publishes state/pose enveloped as {..., data:{...}};
                    # the bare form is accepted too so a stub publisher works.
                    data = body.get("data") if isinstance(body, dict) else None
                    loop.call_soon_threadsafe(
                        state_cache.__setitem__, name,
                        data if isinstance(data, dict) else body)
                return _sink

            def _on_query(query) -> None:
                # RUST THREAD: the HMI task-panel queryable (11 S12.4). Run the
                # async db read on the loop and reply synchronously here --
                # run_coroutine_threadsafe blocks THIS zenoh thread (never the
                # loop), so the Query lifetime stays trivial (reply sent inside
                # the callback). A slow/failed query -> log + no reply (the
                # querier times out) rather than a wrong or partial answer.
                try:
                    payload = asyncio.run_coroutine_threadsafe(
                        answer_task_query(conn, query.selector.parameters), loop
                    ).result(timeout=QUERY_TASKS_TIMEOUT_S)
                except ValueError as exc:          # unknown scope -> bad selector
                    _logger.warning("p3 query/tasks bad selector: %s", exc)
                    return
                except Exception as exc:           # noqa: BLE001
                    _logger.error("p3 query/tasks failed: %s", exc)
                    return
                query.reply(query.key_expr, payload)

            # Sub/queryable handles held in a list (strong ref, CLAUDE.md 4.3 --
            # a dropped queryable is silently unregistered, same GC trap as subs).
            _subs = [gen.declare_subscriber(CMD_TASK_TOPIC, _on_task),
                     gen.declare_subscriber(CMD_ESTOP_TOPIC, _on_estop),
                     gen.declare_subscriber(STATE_LINK_TOPIC, _on_link),
                     gen.declare_subscriber(CMD_GEO_TOPIC, _on_geo),
                     gen.declare_subscriber(CMD_TEACH_TOPIC, _on_teach),
                     gen.declare_queryable(QUERY_TASKS_TOPIC, _on_query)]
            # Held in the same strong-ref list (CLAUDE.md 4.3).
            for _topic, _name in ((STATE_POSE_TOPIC, "pose"),
                                  (HEALTH_SUMMARY_TOPIC, "health"),
                                  (STATE_ROBOT_TOPIC, "robot"),
                                  (STATE_POWER_TOPIC, "power"),
                                  (STATE_TELEOP_TOPIC, "teleop")):
                _subs.append(gen.declare_subscriber(_topic,
                                                    _make_state_sink(_name)))
            # boot_id makes session ids unique across a restart; a plain counter
            # would re-mint ts-0001 after a reboot and collide with the id an
            # HMI tab is still holding.
            teach = TeachRuntime(conn, geo_conn, fence_conn,
                                 boot_id=_BOOT_ID)
            _logger.info(
                "p3 wiring: subscribed %s + %s + %s + %s (+ %d state sources), "
                "queryable %s (task.db + geo single writer + teach session "
                "+ F-5 return_home)",
                CMD_TASK_TOPIC, STATE_LINK_TOPIC, CMD_GEO_TOPIC,
                CMD_TEACH_TOPIC, 5, QUERY_TASKS_TOPIC)
            # 11 S7.9: the single-writer context. task_conn is the SAME handle
            # the scheduler uses -- P3 has one db thread (15 S2.1), so a geo
            # applier reaching into task.db (GC-1..7 linkage, refs) serialises
            # with the scheduler instead of racing it.
            geo_ctx = GeoContext(geo_conn=geo_conn, fence_conn=fence_conn,
                                 task_conn=conn)
            # 11 S7.2: the contract-shaped cmd/task path. Same connection and
            # same DAO the scheduler uses -- P3 has one db thread (15 S2.1), so
            # an HMI cancel serialises with the tick rather than racing it.
            task_ctx = TaskContext(task_conn=conn, dao=dao)

            last_hb = time.monotonic()
            last_geo = 0.0            # 0 -> publish geo on the very first pass
            last_fence = 0.0          # 0 -> publish fence on the very first pass
            last_teach = 0.0          # 0 -> publish teach state immediately
            last_task_state = 0.0     # 0 -> publish TaskState on first pass
            fence_rev = [1]           # 11 S9A.2 rev; +1 on any geometry change
            last_fence_sig = [None]   # rev-0 crc32 of the last broadcast set
            fence_invalid_logged = [False]
            try:
                while not stop_flag.get("stop"):
                    # Wait for a task or wake up to heartbeat / re-check stop.
                    try:
                        payload = await asyncio.wait_for(queue.get(),
                                                         timeout=0.5)
                    except asyncio.TimeoutError:
                        payload = None
                    if payload is not None:
                        # 11 S7.2: a frame in the CONTRACT shape (an `action`
                        # member) goes to the TaskCommand path. Since
                        # 2026-08-20 every task-CREATE arrives that way --
                        # p4_agent, the HMI and the cloud all send S7.2, and
                        # p4's private task_request shape is gone.
                        if isinstance(payload, dict) and "action" in payload:
                            ack = await handle_task_payload(
                                payload, task_ctx, now_mono_ms=_now_mono_ms(),
                                created_at=_now_utc_iso(),
                                date_str=_today_yyyymmdd(),
                                # 与 created_at 同一范式: 墙钟与 boot 由调用方
                                # 注入, 被调用方里不该有第二处时钟来源(CLK-C1).
                                finished_at=_now_utc_iso(),
                                boot_id=_BOOT_ID,
                                on_transition=_make_publish(
                                    state_pub, _emit_task_event,
                                    fetch_terminal=_fetch_terminal_facts))
                            task_ack_pub.put(json.dumps(
                                ack, ensure_ascii=False).encode("utf-8"))
                            if ack.get("result") == "accepted":
                                recorded += 1
                                # ES-3 (15 S11.1 F5): a human submit/resume is
                                # itself the unfreeze signal, and is executed
                                # (already recorded above). NO an auto/charge
                                # task must not unfreeze -- a return_home
                                # arriving mid-estop must not lift the freeze.
                                if (estop_ctrl.frozen
                                        and is_human_resume_command(payload)):
                                    estop_ctrl.unfreeze(payload.get("source"))
                                    _logger.warning(
                                        "p3 ES-3: unfrozen by human %s %s -> "
                                        "scheduling resumes",
                                        payload.get("source"),
                                        payload.get("action"))
                        else:
                            # What is left on this key with no `action`: p4's
                            # p4_intent_v1 frames for the CONTROL intents
                            # (voice pause / cancel / stop_follow). S7.2 cannot
                            # express those -- it requires task_id and forbids
                            # "omit = the current task" -- so they are not
                            # contract frames and the recorder skips them.
                            #
                            # *** That means voice pause/cancel still does
                            # NOTHING. It is a known gap, recorded in NEXT.md,
                            # not something this branch quietly handles: the
                            # missing piece is resolving "the task I mean" to a
                            # task_id, which needs the operator to be told which
                            # task that is.
                            recorded += _log_non_contract_frame(payload)
                    # 11 S7.9: drain every cmd/geo that arrived, answer each on
                    # cmd/geo/ack. Drained fully (not one per pass) so a burst of
                    # chunked upserts is not spread over seconds of loop passes.
                    # An accepted write forces the geo/fence broadcasts to fire on
                    # this pass instead of waiting out their period -- S7.10
                    # specifies "on change plus a 0.1 Hz floor", and the change
                    # is what the HMI is waiting to redraw.
                    while not geo_queue.empty():
                        ack = await handle_geo_payload(
                            geo_queue.get_nowait(), geo_ctx,
                            now_ms=_now_wall_ms(), on_event=_emit_geo_event)
                        geo_ack_pub.put(json.dumps(
                            ack, ensure_ascii=False).encode("utf-8"))
                        if ack.get("result") == "accepted":
                            last_geo = 0.0
                            last_fence = 0.0
                    # 11 S12A: feed the caches, sample the pose, answer every
                    # cmd/teach. Sampling runs BEFORE the commands so a finish
                    # arriving in the same pass includes the point just taken.
                    now_mono_s = time.monotonic()
                    for _key, _setter in (("pose", None), ("health", None),
                                          ("robot", None), ("power", None),
                                          ("teleop", None)):
                        _body = state_cache.get(_key)
                        if _body is None:
                            continue
                        if _key == "pose":
                            teach.update_pose(_body, now_mono_s)
                        elif _key == "health":
                            teach.update_health(_body)
                        elif _key == "robot":
                            teach.update_robot(_body)
                        elif _key == "power":
                            teach.update_power(_body)
                        else:
                            teach.update_teleop(_body)
                    try:
                        await teach.offer_pose(now_mono_s, _now_wall_ms())
                        expired = teach.expire(now_mono_s)
                        if expired:
                            _logger.warning("p3 teach session auto-finished: %s",
                                            expired)
                    except Exception as exc:      # noqa: BLE001
                        _logger.error("p3 teach sampling failed: %s", exc)
                    while not teach_queue.empty():
                        t_ack = await teach.handle(
                            teach_queue.get_nowait(), now_mono_s=now_mono_s,
                            now_ms=_now_wall_ms())
                        teach_ack_pub.put(json.dumps(
                            t_ack, ensure_ascii=False).encode("utf-8"))
                        # A save writes geometry, so refresh the map broadcasts.
                        if t_ack.get("result") == "accepted":
                            last_geo = 0.0
                            last_fence = 0.0
                        last_teach = 0.0          # state change -> publish now
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
                    # ES-1: while frozen by an estop, dispatch NO new task
                    # (15 S11.1). The tick is skipped entirely rather than run
                    # and filtered, so a frozen scheduler cannot move pending ->
                    # ready -> running. ES-3 keeps it frozen until an explicit
                    # p2 unfreeze; there is no timeout path (15 S11.3).
                    if not estop_ctrl.scheduling_permitted():
                        # ES-2 (15 S11.1): suspend the currently-running task
                        # (kind=passive, reason=estop_soft). Runs at most once
                        # per freeze -- after it, freeze keeps anything new from
                        # dispatching, so no task is running and this no-ops.
                        try:
                            # conn 显式传入: ES-2 自己开 BEGIN IMMEDIATE 并
                            # 提交. 不传的话那次写会留在一条开着的隐式事务里,
                            # 之后每一条 submit 都撞 "cannot start a
                            # transaction within a transaction".
                            _susp = await suspend_running_for_estop(
                                dao, conn, _now_mono_ms())
                            if _susp is not None:
                                _tid, _to, _reason = _susp
                                _logger.warning(
                                    "p3 ES-2: suspended running task %s "
                                    "(reason=%s)", _tid, _reason)
                                # ES-2 挂起的是[正在跑的]那条, 所以来源
                                # 状态是 running -- 事件判别按 (from, to) 做,
                                # 传错会查不到表而抛.
                                await _make_publish(
                                    state_pub, _emit_task_event,
                                    fetch_terminal=_fetch_terminal_facts)(
                                        _tid, "running", _to, _reason)
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 ES-2 suspend failed: %s", exc)
                    else:
                        try:
                            await scheduler_tick(
                                conn, dao, now_mono_ms=_now_mono_ms(),
                                on_transition=_make_publish(
                                    state_pub, _emit_task_event,
                                    _publish_task_state,
                                    fetch_terminal=_fetch_terminal_facts),
                                # 与 created_at 同一口径的墙钟(15 S9.5):
                                # started_at 是给人看的下发时间, 由调用方生成
                                # 而不是 driver 自己取 -- driver 里不应有第二
                                # 处时钟来源(CLK-C1 的同一理由: 时间从外面传).
                                started_at=_now_utc_iso(),
                                boot_id=_BOOT_ID,
                                list_route_ids=_list_route_ids)
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 scheduler tick failed: %s", exc)
                    now = time.monotonic()
                    if now - last_hb >= heartbeat_period_s:
                        _logger.info("p3 alive; recorded=%d qdepth=%d",
                                     recorded, queue.qsize())
                        last_hb = now
                    # 11 S7.10A: re-broadcast geo geometry every
                    # GEO_PUBLISH_PERIOD_S (>= 0.1 Hz keepalive). Full payload each
                    # time (幂等自愈, 同 manifest 语义); a read error is logged, never
                    # crashes the loop.
                    if now - last_geo >= GEO_PUBLISH_PERIOD_S:
                        try:
                            geo_pub.put(json.dumps(
                                await read_geo_objects(geo_conn),
                                ensure_ascii=False).encode("utf-8"))
                            manifest = await build_manifest(geo_ctx)
                            manifest["schema"] = "geo_manifest_v1"
                            geo_manifest_pub.put(json.dumps(
                                manifest, ensure_ascii=False).encode("utf-8"))
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 geo broadcast failed: %s", exc)
                        last_geo = now
                    # 11 S9A.3: re-broadcast the active FenceSet from fence.db every
                    # FENCE_PUBLISH_PERIOD_S (>= 0.1 Hz keepalive); rev +1 on any
                    # geometry change (detected via the rev-independent crc32). An
                    # invalid set (no allow / > 5, S9A.1A FS-5A) is NOT broadcast --
                    # the old fence stays in effect; logged once until it recovers.
                    # 11 S2.2.2: state/task at 1 Hz (the "event" half rides the
                    # transition callback). The floor is what lets a consumer tell
                    # "nothing changed" from "P3 is gone" -- with event-only
                    # publishing an idle system and a dead one look identical.
                    if now - last_task_state >= TASK_STATE_PERIOD_S:
                        try:
                            await _publish_task_state()
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 state/task broadcast failed: %s", exc)
                        last_task_state = now
                    # 11 S12A.5: state/teach at 1 Hz plus on every change.
                    if now - last_teach >= TEACH_PUBLISH_PERIOD_S:
                        try:
                            teach_state_pub.put(json.dumps(
                                teach.teach_state_payload(now),
                                ensure_ascii=False).encode("utf-8"))
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 teach state publish failed: %s",
                                          exc)
                        last_teach = now
                    if now - last_fence >= FENCE_PUBLISH_PERIOD_S:
                        try:
                            rows = await fences_dao.list_active()
                            # rev=0 gives a rev-independent signature to detect change.
                            # crc32 只用于[变更检测], 不上总线: 传 rev=0 让
                            # 签名与 rev 无关. enu_origin 同样不参与 -- 它不
                            # 进 crc32 配方(11 S9A.2 的配方只含 polygons), 所以
                            # 这里传不传都不影响比对结果; 不传是为了让"签名"
                            # 这一步与"发布"那一步的意图区分得更清楚.
                            sig = build_fence_set(
                                rows, fence_set_id=FENCE_SET_ID, rev=0)["crc32"]
                            if sig != last_fence_sig[0]:
                                if last_fence_sig[0] is not None:
                                    fence_rev[0] += 1
                                last_fence_sig[0] = sig
                            # enu_origin 必须随发布带上(11 S9A.2 必填字段).
                            fs = build_fence_set(
                                rows, fence_set_id=FENCE_SET_ID,
                                rev=fence_rev[0], enu_origin=enu_origin)
                            fence_pub.put(json.dumps(
                                fs, ensure_ascii=False).encode("utf-8"))
                            fence_invalid_logged[0] = False
                        except InvalidFenceSet as exc:
                            if not fence_invalid_logged[0]:
                                _logger.warning(
                                    "p3 fence broadcast skipped, invalid set: %s",
                                    exc)
                                fence_invalid_logged[0] = True
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 fence broadcast failed: %s", exc)
                        last_fence = now
            finally:
                for s in _subs:
                    try:
                        s.undeclare()
                    except Exception:      # noqa: BLE001
                        pass
                for p in (state_pub, geo_pub, geo_manifest_pub, fence_pub,
                          geo_ack_pub, teach_ack_pub, teach_state_pub):
                    try:
                        p.undeclare()
                    except Exception:      # noqa: BLE001
                        pass
    finally:
        await conn.close()
        for c in (geo_conn, fence_conn):
            try:
                await c.close()
            except Exception:              # noqa: BLE001
                pass
    return 0


def _make_publish(state_pub, emit_task_event=None, publish_state=None,
                  fetch_terminal=None):
    """Build the scheduler on_transition callback: publish each task state change on
    state/task (event + 1 Hz, 11 S2.2.2) so p5/HMI/cloud can rebuild the machine, log
    it, AND emit the 11 S6.2 task event (accept/reject/start/complete/fail) via
    emit_task_event when the transition warrants one. reason is non-empty only on a
    validate_fail.

    publish_state is the "event" half of S2.2.2: it re-reads the live tasks and puts
    a full 11 S4.4 TaskState. It is a callback rather than a db read done here so
    this factory keeps needing no connection, and so the loop's 1 Hz half and this
    one publish through the same code (two builders of the same broadcast is how the
    old placeholder shape survived -- nothing compared them).

    Publishing the whole state on every transition, rather than just the task that
    moved, is what makes `queue` and `suspended` correct: one task starting changes
    the queue for all the others, and a delta of one task cannot express that."""
    async def _publish(task_id: str, from_state: str, to_state: str,
                       reason: str) -> None:
        if reason:
            _logger.info("p3 task %s %s -> %s (%s)",
                         task_id, from_state, to_state, reason)
        else:
            _logger.info("p3 task %s %s -> %s", task_id, from_state, to_state)
        if publish_state is not None:
            try:
                await publish_state()
            except Exception as exc:      # noqa: BLE001
                # A broadcast failure must not abort the transition: the state
                # change is already committed, and the 1 Hz pass re-sends.
                _logger.error("p3 state/task publish failed: %s", exc)
        # 11 S6.2 task event -- a separate stream from the state/task heartbeat.
        if emit_task_event is not None:
            # 判别只看迁移. reason 仍然进日志(给人看), 但不再参与决定发什么
            # 事件 -- 那正是"暂停被报成 rejected"的来源.
            # 局部导入: 本模块顶部只放 stdlib, 全部 xbrain 依赖都在函数内取
            # (文件既有惯例, 避免加载期的循环).
            from xbrain.p3_task.state.machine import TERMINAL_STATES

            ev = task_event_for_transition(from_state, to_state)
            if ev is not None:
                extra = None
                if to_state in TERMINAL_STATES and fetch_terminal is not None:
                    try:
                        extra = dict(await fetch_terminal(task_id))
                    except Exception as exc:      # noqa: BLE001
                        # 读不到就少几个字段, NO 不能因此丢掉整条终态事件 --
                        # 那是云端唯一能知道任务结束了的报文.
                        _logger.error("p3 terminal facts read failed: %s", exc)
                    if extra is not None and reason:
                        # 操作员填的原因: 它在参数里现成, 不必再查库.
                        extra["reason"] = reason
                emit_task_event(task_id, to_state, ev[0], ev[1], extra)
    return _publish


def _log_non_contract_frame(payload) -> int:
    """A cmd/task frame with no top-level `action`. Logged, recorded nowhere.

    *** This REPLACED _record_one + record_task_from_payload on 2026-08-23.

    That path existed to turn p4_agent's private `task_request` shape into a
    task row. Since batch 15 no sender emits that shape -- p4_agent, the HMI and
    the cloud all send the 11 S7.2 contract shape, which goes to
    handle_task_payload instead. So the create half of the legacy path had
    become unreachable code that still owned a db write.

    What still arrives here is the handful of voice intents that route to
    cmd/task but have no S7.2 action to express them:
      * B10 skip_waypoint -- S7.2's five actions have no `skip`
      * H04 reload_config -- 18 says verbatim it belongs on cmd/config (S7.6)
    Both are logged by intent id so they are VISIBLE as gaps. They were being
    silently dropped before; a log line is the difference between "we know this
    does nothing" and "nobody noticed".

    Returns 0 always -- nothing is recorded, and the caller's counter should
    not move.
    """
    _logger.info(
        "p3 cmd/task frame has no contract action (intent=%s); not a task, "
        "nothing recorded -- see NEXT SW-16 for the two known gaps",
        payload.get("intent_id") if isinstance(payload, dict) else None)
    return 0



# Import at module scope so the recorder is a stable reference (not re-imported
# per call). Kept after the functions to avoid a circular import at load.
from xbrain.p3_task.query.queryable import answer_task_query  # noqa: E402
