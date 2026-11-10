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
STATE_MODE_TOPIC = "state/mode"          # 15 S4.1A: B / D takeover -> yield (99 U99)
QUERY_TASKS_TOPIC = "query/tasks"        # 11 S12.4 HMI task-panel queryable (P5 pulls)
QUERY_TASKS_TIMEOUT_S = 2.0              # cap the zenoh-thread block on a db read
STATE_GEO_OBJECTS_TOPIC = "state/geo/objects"  # 11 S7.10A geo geometry broadcast (P5 -> HMI)
STATE_GEO_MANIFEST_TOPIC = "state/geo/manifest"  # 11 S7.10 sync baseline (P1/P4/P5)
GEO_PUBLISH_PERIOD_S = 5.0               # geo re-publish cadence (>= 0.1 Hz keepalive)
CMD_TEACH_TOPIC = "cmd/teach"            # 11 S12A.4: P3 owns the recording session
CMD_TEACH_ACK_TOPIC = "cmd/teach/ack"
STATE_TEACH_TOPIC = "state/teach"        # 11 S12A.5, event + 1 Hz
STATE_POSE_TOPIC = "state/pose"          # 11 S3.3, the sampling source
# P1-12 (11 S3.5B): 2 Hz + one extra frame at every waypoint. 15 S2.1 calls it
# "断点的唯一来源" and it is also the confirmation channel for a route push
# (15 S2.4.4 RA-1 -- cmd/motion/route has no ack key). Subscribed since
# 2026-09-28; before that p3_task matched it in one comment and nowhere else,
# so patrol_progress had no writer and a finished patrol stayed `running`.
STATE_PATH_PROGRESS_TOPIC = "state/motion/path_progress"
#: Bound on the progress backlog. 2 Hz * 16 = 8 s of history, which is longer
#: than the 3 s staleness window (11 S3.5B) -- past that the frames are stale
#: anyway and keeping them would only delay the fresh one behind them.
PROGRESS_QUEUE_MAX = 16
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
                          enu_origin: Optional[Dict[str, float]] = None,
                          teach_snap_max_age_ms: Optional[int] = None,
                          recorded_arrive_radius_m: Optional[float] = None,
                          route_membership_m: Optional[float] = None,
                          teach_sample_hz: Optional[float] = None,
                          teach_min_dist_m: Optional[float] = None,
                          rtk_lost_sustain_s: Optional[float] = None,
                          rtk_recover_sustain_s: Optional[float] = None,
                          low_battery: Optional[Dict[str, float]] = None) -> int:
    """Block until stop_flag['stop'] is truthy. Returns 0 on clean shutdown.

    enu_origin 来自 common.geo.enu_origin(L4 sites/{site_id}.yaml), 由
    __main__ 从解析产物取出后传入. 默认 None 保留是为了既有调用方(测试)不必
    全部改签名 -- 但生产路径必须传, 否则 FenceSet 的必填字段为空(11 S9A.2).

    teach_snap_max_age_ms is p3_task.yaml teach.snap_max_age_ms (18 PS-6,
    99 U109). None (tests) leaves every snapshot-carrying capture refused.

    recorded_arrive_radius_m is p3_task.yaml route.recorded_arrive_radius_m
    (= common.recording.arrive_radius_m, 99 U126): the arrival radius of a
    teach-recorded route (mode B, 15 S2.4.3). None (tests) keeps every mode B
    push refused by build_snapshot, which names the key; the production
    __main__ refuses to start without it (EX_CONFIG).

    route_membership_m is p3_task.yaml route.membership_m (99 U127): a
    waypoint within this distance of a route polyline belongs to it, and a
    goto to it follows that route. None (tests) keeps the single-point goto;
    the production __main__ refuses to start without it.

    teach_sample_hz / teach_min_dist_m are p3_task.yaml teach.sample_hz /
    teach.min_dist_m (= common.recording.*, 99 U132): the S12A.6 sampling
    pair. None (tests) refuses every recording start by key name; the
    production __main__ refuses to start without them.

    rtk_lost_sustain_s / rtk_recover_sustain_s are p3_task.yaml rtk_loss.*
    (11 S3.3.3, 99 U142): how long single / no_fix must last before the task
    is suspended and the alarm raised, and how long a usable fix must last
    before recovery is reported. None (tests) leaves the judgement off; the
    production __main__ refuses to start without them.

    low_battery is p3_task.yaml low_battery.{trigger_soc_pct, stop_soc_pct,
    cloud_alarm_period_s, charge_full_soc_pct, swap_resume_soc_pct} (99 U146,
    U157). None (tests) leaves the low-battery return off; the production
    __main__ refuses to start without it.
    """
    return asyncio.run(
        _amain(stop_flag, heartbeat_period_s, task_db_path, geo_db_path,
               fence_db_path, enu_origin, teach_snap_max_age_ms,
               recorded_arrive_radius_m, route_membership_m,
               teach_sample_hz=teach_sample_hz,
               teach_min_dist_m=teach_min_dist_m,
               rtk_lost_sustain_s=rtk_lost_sustain_s,
               rtk_recover_sustain_s=rtk_recover_sustain_s,
               low_battery=low_battery))


async def _amain(stop_flag: dict, heartbeat_period_s: float,
                 task_db_path: str, geo_db_path: str = DEFAULT_GEO_DB,
                 fence_db_path: str = DEFAULT_FENCE_DB,
                 enu_origin: Optional[Dict[str, float]] = None,
                 teach_snap_max_age_ms: Optional[int] = None,
                 recorded_arrive_radius_m: Optional[float] = None,
                 route_membership_m: Optional[float] = None,
                 teach_sample_hz: Optional[float] = None,
                 teach_min_dist_m: Optional[float] = None,
                 rtk_lost_sustain_s: Optional[float] = None,
                 rtk_recover_sustain_s: Optional[float] = None,
                 low_battery: Optional[Dict[str, float]] = None) -> int:
    from xbrain.common.errors import (E_DEGRADED, E_GEO_INCOMPLETE, E_LOW_BATTERY,
                                      E_NOT_FOUND, E_SCHEMA)
    from xbrain.p3_task.ingest.task_command import task_ack
    from xbrain.p3_task.lifecycle.rtk_loss import RtkLossMonitor, suspend_running_for_rtk
    from xbrain.p3_task.lifecycle import low_battery as lb
    from xbrain.common.runtime.session_ctx import open_planes
    from xbrain.p3_task.dao.simple_daos import FencesDAO, PatrolProgressDAO, SnapshotDAO
    from xbrain.p3_task.dao.tasks_dao import TasksDAO
    from xbrain.p3_task.fence.fence_set import build_fence_set
    from xbrain.p3_task.fence.geom import InvalidFenceSet
    from xbrain.p3_task.geo.objects import read_geo_objects
    from xbrain.p3_task.geo.waypoint_kind import nearest_dock, resolve_return_target  # 99 U119 / U128
    from xbrain.p3_task.ingest.geo_apply import GeoContext, handle_geo_payload
    from xbrain.p3_task.ingest.geo_read import build_manifest
    from xbrain.p3_task.ingest.task_apply import TaskContext, handle_task_payload
    from xbrain.p3_task.persistence.base import open_configured
    from xbrain.p3_task.persistence.schema_geo import (
        FENCE_DB_STATEMENTS,
        GEO_DB_STATEMENTS,
    )
    from xbrain.p3_task.persistence.schema_task import ALL_DDL_STATEMENTS
    from xbrain.p3_task.route.laps import lap_spec_from_params
    from xbrain.p3_task.route.push import RoutePushTrigger, build_route_clear
    from xbrain.p3_task.runtime.progress_sink import apply_path_progress, predates_push
    from xbrain.p3_task.runtime.route_push_runtime import (
        CMD_ROUTE_TOPIC,
        ack_outcome_is_fatal,
        make_ack_window,
        load_route_geoms,
        push_goto_for_task,
        push_route_for_task,
        start_from_pose,
    )
    from xbrain.p3_task.schedule.driver import scheduler_tick
    from xbrain.p3_task.state.geo_events import render_geo_event
    from xbrain.p3_task.state.path_progress import ProgressTracker, parse_path_progress
    from xbrain.p3_task.state.task_state import (
        _waypoint_total,
        read_task_state,
        wall_iso_to_epoch,
    )
    from xbrain.p3_task.teach.runtime import TeachRuntime

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
        # P1-12 progress rides its own queue too, and it is the one that must
        # never block behind a command: it arrives at 2 Hz forever, whereas the
        # other three are event-driven. Bounded, and dropping the OLDEST on
        # overflow, because the newest position is the only one that matters --
        # an unbounded queue here turns a stalled loop into a memory leak, and
        # dropping the newest would freeze the reported position at the moment
        # of the stall (which reads as a stopped robot).
        progress_queue: asyncio.Queue = asyncio.Queue(maxsize=PROGRESS_QUEUE_MAX)
        progress_tracker = ProgressTracker()
        # 99 U131: tasks whose next pass has been pushed but not yet seen
        # running (progress_sink's pass latch). In memory on purpose: a P3
        # restart suspends every running task (15 S10 INIT).
        pass_latch: set = set()
        progress_dao = PatrolProgressDAO(conn)
        snapshot_dao = SnapshotDAO(conn)
        # EX-2: one ack window for the process -- 15 S6.1 allows one running
        # task at a time, so there is never a second push in flight.
        ack_window = make_ack_window()
        progress_dropped = [0]
        recorded = 0

        _logger.info("p3 wiring: opening GEN session")
        with open_planes(("gen",)) as gen:
            # --- 执行器闸门(A3) -------------------------------------------------
            # Dispatcher 的设计意图逐字是"缺执行器就拒绝启动"(assert_complete ->
            # DispatcherIncomplete). 2026-09-04 终测发现它[零实例化] -- 生产代码
            # 从没建过 Dispatcher, 也从没调过 assert_complete; 8 处调用全在测试里,
            # 包括一条逐字写着 "must catch a missing type BEFORE runtime" 的用例.
            # 于是本该是启动期硬失败的情况, 退化成"安静接受一个执行不了的任务,
            # 并持续向甲方报 running"(终测实测: 一条 goto 报 running 四分钟, 而
            # cmd/motion/intent 等 8 条运动 key 全是 0 帧).
            #
            # *** 这里[不]退出进程, NO 不把它做成 fatal.
            # EX-1(路径展开) 与 EX-2..6 在 NEXT.md 里是登记在案的 [GATED-HW] 欠账,
            # 卡在云深处底盘/RTK 硬件上. 现在拒绝启动会让 geo / 围栏 / 喊话 /
            # 急停这些不依赖执行器的能力一起停摆 -- 代价远大于收益.
            # 能做的是让它[每次开机都喊出来], 而不是像现在这样一声不响.
            try:
                from xbrain.p3_task.schedule.dispatcher import (
                    Dispatcher,
                    DispatcherIncomplete,
                )
                _dispatcher = Dispatcher()
                # 还没有任何执行器可注册 -- EX-1/EX-4 未落地. 有了就在这里 register.
                _dispatcher.assert_complete()
            except DispatcherIncomplete as _exc:
                _logger.error(
                    "p3 NO TASK EXECUTOR IS WIRED: %s. Tasks will be admitted and "
                    "reported running, but nothing will drive the motion plane "
                    "(EX-1/EX-4, NEXT.md [GATED-HW]).", _exc)
            except Exception as _exc:      # noqa: BLE001
                _logger.error("p3 dispatcher self-check failed: %s", _exc)

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
            # EX-2 (11 S3.5A / P1-11). P3 is the only publisher of the route
            # geometry: P1 never opens geo.db, so without this key patrol has
            # no geometry at all and path_follow has nothing to follow
            # (15 S2.4 opening paragraph).
            # 99 U149: wrapped so the first push after an operator lifts a
            # soft estop tells P1 to re-arm (estop_ctrl is bound below; the
            # lambda reads it only when a frame is sent).
            from xbrain.p3_task.lifecycle.estop import RearmingRoutePublisher
            route_pub = RearmingRoutePublisher(
                gen.declare_publisher(CMD_ROUTE_TOPIC),
                lambda: estop_ctrl.scheduling_permitted())

            # F-5 (11 S4.6.4): watch P5's cloud-link level and inject one
            # return_home at L3 (cloud down past rtb_s). The subscriber only STORES
            # the latest snapshot (RUST thread -> no db, no await, CLAUDE.md 4.2);
            # the loop below reads it and does the insert.
            from xbrain.p3_task.lifecycle.link_loss import (
                RETURN_HOME_PRIORITY,
                LinkLossReturnTrigger,
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
                # WALL-CLOCK-OK(record): 11 S6.1 Event.ts, 任务跃迁发生的墙钟
                # 时刻. 原写死 0.0 -- 被 p5 的 `or` 兜底遮住(见上面 geo 那处的
                # 同一条论证), 入库成 p5 收包时刻. 这一处代价更大: 上面注释已
                # 写明"任务一终结就从 11 S4.4 的 TaskState 广播里消失, 事件是
                # 终态那一刻唯一还带着任务的报文" -- 那份报文的时间戳本来就是
                # 唯一一份, 写 0.0 等于把它交给 p5 的调度时机.
                # NO 不带 dedup_key: 11 S6.2 的 task 行只规定 sev 与 channel,
                # 既没有 key 也没有窗口; 而 p5 只在 ev.get("dedup_key") 为真时
                # 才走 _try_merge, 带 key 不带窗口才是会静默并掉事件的那一半.
                # 不做年龄/超时判定(那些一律 time.monotonic(), CLK-C1).
                gen.put("event/%s/task" % sev, json.dumps({
                    "eid": "task-%s-%d" % (_task_evt_boot, _task_evt_seq[0]),
                    "title": "task %s %s" % (task_id, kind),
                    "detail": detail,
                    # WALL-CLOCK-OK(record): 11 S6.1 Event.ts, 这条任务终态事件
                    # 发生的墙钟时刻. 标记必须落在本行而不是调用上方的注释块 --
                    # marker_for 的上行查找在第一个非注释行停住, 发布调用的首行
                    # 就是代码, 所以写在上面的标记谁也挂不上.
                    "src": "p3_task", "ts": time.time(),
                }, ensure_ascii=False).encode("utf-8"))

            async def _list_route_ids():
                """PC-8 用的路径全集(当前 manifest).

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
                    "       duration_sec, mission_json, error_context_json "
                    "FROM tasks "
                    "WHERE task_id=?", (task_id,))
                row = await cur.fetchone()
                if row is None:
                    return {}
                out = {"task_type": row[0], "route_id": row[1],
                       "started_ts": wall_iso_to_epoch(row[2]),
                       "ended_ts": wall_iso_to_epoch(row[3]),
                       "duration_sec": row[4]}
                # v2.0 S3.3 逐字: summary.completed_count/total_count 是
                # [权威计数]. 快照面(state/task)已经从 mission_json 报出真值,
                # 终态面是[另一条路](p5 的 observe_task 读的是本 detail, 不是
                # 快照) -- 只修一条的话, 运行中显示 2 个点而终态又变回 0.
                # 2026-09-04 终测实测到这个不一致.
                _total = _waypoint_total({"mission_json": row[5]})
                if _total is not None:
                    out["total_count"] = _total
                # 11 S6.2 (99 U140): a failed task's error_context_json rides
                # out as detail.error -- {code, detail, cause} for a geometry
                # failure, {fail_reason} for a navigation failure. P4 voices
                # the failure from it; until then the event said "failed" and
                # nobody could tell the operator why.
                try:
                    _err = json.loads(row[6]) if row[6] else None
                except ValueError:
                    _err = None  # unreadable context: the event still goes out
                if isinstance(_err, dict):
                    out["error"] = _err
                return out

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
                # WALL-CLOCK-OK(record): 11 S6.1 Event.ts, 这次改图发生的墙钟
                # 时刻. 原为 render_geo_event 内写死的 0.0 -- p5 的
                # _normalise_event 是 `d.get("ts") or ... or now.timestamp()`,
                # 0.0 为假值, 于是入库的是 p5 收包时刻, 审计里"谁什么时候改了
                # 图"退化成"网关什么时候看到的". 这里读钟而不在 renderer 里读:
                # 那个模块是纯映射, 同 12 S6A.8 PUB-1 的分工.
                # 不做年龄/超时判定(那些一律 time.monotonic(), CLK-C1).
                key, body = render_geo_event(
                    sev, etype, detail,
                    "geo-%s-%d" % (_geo_evt_boot, _geo_evt_seq[0]),
                    # WALL-CLOCK-OK(record): 上面那段注释说的就是本行 --
                    # 11 S6.1 Event.ts. 标记重复落在这里是因为 marker_for 只认
                    # 本行或[紧贴本行]的连续注释块, 而 renderer 调用的首行是
                    # 代码, 把上行查找截断了.
                    # 注意本行注释里[不能]出现未配对的左括号: tests/p3_task/
                    # test_geo_events.py 的 _call_text 按括号配平截取本调用,
                    # 注释里多一个左括号就让它读到调用之外 -- 2026-10-01 实测,
                    # 本注释的初稿写了 renderer 函数名带括号, 当场红了两条.
                    time.time())
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
                # p5 wraps state/link in the 11 S3.0 envelope (since 2026-09-27
                # -- S3.0 verbatim: every Zenoh JSON payload shares that outer
                # structure). Read the body out of data, keeping the bare form
                # as the fallback so a stub publisher still works and so this
                # does not have to land in the same deploy as the producer.
                #
                # Reading the top level unconditionally is what would break:
                # every field below would come back None and F-5's return_home
                # judgement would silently stop seeing the cloud link -- no
                # error, no log, just a robot that never returns home.
                if isinstance(p, dict) and isinstance(p.get("data"), dict):
                    p = p["data"]
                # Hand the fields to the loop thread; never touch the db here.
                loop.call_soon_threadsafe(link_holder.update, {
                    "level": p.get("level"),
                    "gw_start_mono": p.get("gw_start_mono"),
                    "link_epoch": p.get("link_epoch"),
                    "disconnected_s": p.get("disconnected_s"),
                    # 11 S4.6 LinkState.estop_path (P5's S8.5 probe verdict):
                    # the recording arming gate's criterion 7 (2) reads it
                    # (2026-11-05 runtime test: it used to look in state/robot,
                    # which has no such field, and could never pass)
                    "estop_path": p.get("estop_path"),
                })

            # 15 S11.1 ES-1..3 estop handling. freeze() is idempotent and a
            # scalar assignment, safe from the Rust callback (CLAUDE.md 4.2);
            # the scheduling loop below reads scheduling_permitted() each pass.
            # ES-3 / 15 S11.3: p3 does NOT auto-resume -- there is no time-based
            # unfreeze, only an explicit p2 signal (CLAUDE.md 3.6: no bypass).
            from xbrain.p3_task.lifecycle.estop import (
                EstopController,
                human_source,
                is_human_pause_command,
                is_human_resume_command,
            )
            from xbrain.p3_task.lifecycle.estop_suspend import suspend_running_for_estop
            from xbrain.p3_task.lifecycle.mode_takeover import (
                resume_after_takeover,
                suspend_running_for_takeover,
            )
            # 15 S4.1A: last state/mode.motion_behavior (None = none seen yet).
            mode_view: dict = {"motion_behavior": None}
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

            def _on_state_mode(sample) -> None:
                # RUST THREAD (CLAUDE.md 4.2): one dict write, nothing else.
                # 15 S4.1A reads motion_behavior ("hold" during B / D since
                # 99 U99, "normal" otherwise). Enveloped or bare body; a body
                # without the field leaves the last value (P2 re-sends 1 Hz).
                try:
                    _d = json.loads(bytes(sample.payload).decode("utf-8"))
                    _b = _d.get("data") if isinstance(_d.get("data"), dict) else _d
                    _mb = _b.get("motion_behavior")
                except Exception:                 # noqa: BLE001
                    return
                if isinstance(_mb, str):
                    mode_view["motion_behavior"] = _mb

            def _on_estop(sample) -> None:
                # RUST THREAD (CLAUDE.md 4.2): ES-1 freeze only. freeze() is
                # idempotent (keeps the first reason) and touches two scalar
                # fields, so it is safe here; no db, no publish. The scheduling
                # loop reads the freeze state. Parse is fail-safe (11 S3.0.1):
                # a malformed estop must still freeze -- an unfrozen scheduler
                # would keep dispatching patrol tasks through an emergency stop.
                estop_ctrl.freeze("estop_soft")
                route_pub.arm()  # 99 U149: P1 latched; an operator's next push lifts it
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

            def _offer_progress(body) -> None:
                # LOOP THREAD (reached via call_soon_threadsafe below). Bounded
                # put with an oldest-first drop; put_nowait alone would raise
                # QueueFull inside a call_soon callback, where the exception
                # goes to the loop's exception handler and the frame is lost
                # with a traceback instead of a counter.
                if progress_queue.full():
                    try:
                        progress_queue.get_nowait()
                        progress_dropped[0] += 1
                    except asyncio.QueueEmpty:      # drained between the two
                        pass
                progress_queue.put_nowait(body)

            def _on_path_progress(sample) -> None:
                # RUST THREAD: decode and hand off only (CLAUDE.md 4.2). This
                # callback runs at 2 Hz forever, so it must stay the cheapest
                # of the five -- no db, no publish, no await.
                try:
                    payload = json.loads(bytes(sample.payload).decode("utf-8"))
                except Exception:      # noqa: BLE001
                    _logger.warning("p3 malformed path_progress payload")
                    return
                # p1 enveloped (11 S3.0) or bare -- accept both, the same way
                # the state sinks above do, so a stub publisher works.
                data = payload.get("data") if isinstance(payload, dict) else None
                loop.call_soon_threadsafe(
                    _offer_progress,
                    data if isinstance(data, dict) else payload)

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
                     gen.declare_subscriber(STATE_MODE_TOPIC, _on_state_mode),
                     gen.declare_subscriber(CMD_GEO_TOPIC, _on_geo),
                     gen.declare_subscriber(CMD_TEACH_TOPIC, _on_teach),
                     gen.declare_subscriber(STATE_PATH_PROGRESS_TOPIC,
                                            _on_path_progress),
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
                                 boot_id=_BOOT_ID,
                                 # 99 U132: the configured sampling pair
                                 sample_hz=teach_sample_hz,
                                 min_dist_m=teach_min_dist_m,
                                 snap_max_age_ms=teach_snap_max_age_ms,
                                 route_membership_m=route_membership_m)  # 99 U146
            if teach_snap_max_age_ms is None:
                # Said once at startup, not discovered at the first capture.
                _logger.warning("p3 teach.snap_max_age_ms unset: voice "
                                "keypoint captures will be refused")
            _logger.info(
                "p3 wiring: subscribed %s + %s + %s + %s + %s "
                "(+ %d state sources), queryable %s (task.db + geo single "
                "writer + teach session + F-5 return_home + path progress)",
                CMD_TASK_TOPIC, STATE_LINK_TOPIC, CMD_GEO_TOPIC,
                CMD_TEACH_TOPIC, STATE_PATH_PROGRESS_TOPIC, 5,
                QUERY_TASKS_TOPIC)
            # 11 S7.9: the single-writer context. task_conn is the SAME handle
            # the scheduler uses -- P3 has one db thread (15 S2.1), so a geo
            # applier reaching into task.db (GC-1..7 linkage, refs) serialises
            # with the scheduler instead of racing it.
            geo_ctx = GeoContext(geo_conn=geo_conn, fence_conn=fence_conn,
                                 task_conn=conn,
                                 route_membership_m=route_membership_m)  # 99 U146
            # 11 S7.2: the contract-shaped cmd/task path. Same connection and
            # same DAO the scheduler uses -- P3 has one db thread (15 S2.1), so
            # an HMI cancel serialises with the tick rather than racing it.
            # 99 U133: submits are refused while a recording runs (CMD-16)
            from xbrain.p3_task.ingest.goto_precheck import hmi_goto_refusal  # noqa: PLC0415

            async def _keypoint_latlon(wid: str):
                cur = await geo_conn.execute(
                    "SELECT rtk_lat, rtk_lon FROM waypoints WHERE geo_id=? "
                    "AND tombstone=0 AND state<>'deleted'", (wid,))
                row = await cur.fetchone()
                return None if row is None else (float(row[0]), float(row[1]))

            async def _active_fence_set():
                # Built from fence.db on demand, the same way the cmd/fence
                # broadcast builds it, so the check sees the fence P1 is being
                # given. An invalid set is not broadcast (the old one stays in
                # force, P1 side) -- its geometry is unknown here, so no fence
                # refusal is made from it; positioning is still checked.
                rows = await fences_dao.list_active()
                if not rows:
                    return None
                try:
                    return build_fence_set(rows, fence_set_id=FENCE_SET_ID, rev=0,
                                           enu_origin=enu_origin,
                                           on_unusable=lambda _pid, _r: None)
                except InvalidFenceSet:
                    return None

            async def _goto_precheck(task):
                # 11 S12.1.2 W2, P3's half (ingest/goto_precheck): positioning
                # failed / keypoint missing / target outside the fence.
                return await hmi_goto_refusal(
                    task, pose=state_cache.get("pose"),
                    fence_set=await _active_fence_set(),
                    keypoint=_keypoint_latlon)

            task_ctx = TaskContext(task_conn=conn, dao=dao,
                                   teach_busy=lambda: teach.active,
                                   goto_precheck=_goto_precheck)

            #: task_id of the push the ack window is currently waiting on, so
            #: a retry knows what to re-push and a failure knows what to fail.
            #: goto_params: a goto with no route pushes ONE point from these
            #: (its mission params), so a retry re-resolves the same target.
            pushing = {"task_id": None, "route_geo_id": None,
                       "goto_params": None}

            def _goto_params(full) -> dict:
                """The admitted goto's params (mission_json is immutable after
                admission, 15 S5.10); {} when unreadable."""
                try:
                    mission = json.loads(full.mission_json or "{}")
                except ValueError:
                    return {}
                params = mission.get("params") if isinstance(mission, dict) else None
                return params if isinstance(params, dict) else {}

            async def _push_goto(task_id: str, params: dict, trigger) -> None:
                """A goto with no route: ONE point (11 S3.5A v2.0, 12
                S4.2c.3), from the robot's position as P3 last saw it."""
                res = await push_goto_for_task(
                    task_id=task_id, params=params,
                    start=start_from_pose(state_cache.get("pose")),
                    task_conn=conn, geo_conn=geo_conn,
                    snapshot_dao=snapshot_dao, progress_dao=progress_dao,
                    route_pub=route_pub, ack_window=ack_window,
                    trigger=trigger, now_mono_ms=_now_mono_ms(),
                    now_wall_ms=_now_wall_ms(),
                    # 99 U127: follow a route the target belongs to (B01 /
                    # B08 / B09 all come through here)
                    route_membership_m=route_membership_m)
                if not res.pushed:
                    pushing["task_id"] = None
                    # A retry / remap follows an earlier push of this task, so
                    # P1 may hold it; a first dispatch never sent anything.
                    await _fail_task_for_geometry(
                        task_id, res.reason, E_GEO_INCOMPLETE, res.cause,
                        withdraw=trigger is not RoutePushTrigger.RP1_DISPATCH)

            async def _push_route_on_dispatch(task_id: str) -> None:
                """EX-2 RP-1. patrol pushes its route; goto pushes its route
                if it names one, else its single target point (2026-11-03:
                until then a waypoint goto was dispatched and never moved).
                standby / teach / follow have no geometry and push nothing."""
                full = await dao.fetch_by_id(task_id)
                if full is None:
                    return
                # 15 S7.3 step 5: a task RESUMED from suspended already has its
                # progress run open (RP-1 opened it at the first dispatch), so
                # it is re-pushed as RP-2, which keeps that run. Until
                # 2026-11-06 every dispatch was RP-1 and re-opened the run:
                # UNIQUE patrol_progress.task_id, the tick raised, nothing was
                # pushed and the resumed task sat `running` with P1 idle. It
                # never showed while a pause left P1 driving the old route;
                # the route clear on suspend (route_clear_needed) exposed it.
                trigger = (RoutePushTrigger.RP2_REMAP
                           if await progress_dao.fetch_active(task_id) is not None
                           else RoutePushTrigger.RP1_DISPATCH)
                route_geo_id = getattr(full, "route_geo_id", None)
                if not route_geo_id and full.task_type in ("return_home", "charge"):
                    # 99 U119 / U128: return_home and charge are gotos to a
                    # role waypoint. Until 2026-11-05 neither had a branch: the
                    # task fell through to "not a goto -> push nothing" and sat
                    # in `running` (rh-7899-0 held it for hours; a voice
                    # "回充电桩" was accepted and never moved).
                    pose = start_from_pose(state_cache.get("pose"))
                    routes = await load_route_geoms(geo_conn)
                    params = _goto_params(full)
                    if full.task_type == "return_home":
                        # home first; the dock fallback is for the link-loss
                        # return only (source 'charge', U119 rule 4) -- a
                        # spoken "返航" with no home point fails (U128).
                        target = await resolve_return_target(
                            geo_conn, pose,
                            allow_dock=(getattr(full, "source", None) == "charge"),
                            routes=routes, membership_m=route_membership_m)
                        gid = target[0] if target is not None else None
                        missing = ("return_home: no active home point "
                                   "(返航终点站)")
                    elif isinstance(params.get("waypoint_id"), str) and params["waypoint_id"]:
                        # B09 with a name ("去1号充电站"): that dock, however
                        # far (U128) -- P4 resolved the name to the id.
                        gid, missing = params["waypoint_id"], ""
                    else:
                        # B09 with no name: the dock reached by the shortest
                        # travel distance (U128, route-aware U127 plan).
                        gid = await nearest_dock(geo_conn, pose, routes,
                                                 route_membership_m)
                        missing = "charge: no dock point (充电桩/充电站)"
                    if gid is None:
                        # 15 S9.5.1: no target row -> E_NOT_FOUND. Failed,
                        # never left running with nothing moving.
                        await _fail_task_for_geometry(task_id, missing, E_NOT_FOUND,
                                                      "target_missing", withdraw=False)
                        return
                    params = {"waypoint_id": gid}
                    _logger.info("p3 %s %s -> point %s", full.task_type,
                                 task_id, gid)
                    pushing.update(task_id=task_id, route_geo_id=None,
                                   goto_params=params)
                    await _push_goto(task_id, params, trigger)
                    return
                if not route_geo_id:
                    if full.task_type != "goto":
                        return
                    params = _goto_params(full)
                    pushing.update(task_id=task_id, route_geo_id=None,
                                   goto_params=params)
                    await _push_goto(task_id, params, trigger)
                    return
                try:
                    # 99 U131: laps as admitted (V-8a checked them already;
                    # a task stored before that check is refused, not run
                    # with a guessed count)
                    lap_spec = lap_spec_from_params(_goto_params(full))
                except ValueError as exc:
                    pushing["task_id"] = None
                    await _fail_task_for_geometry(task_id, str(exc), E_SCHEMA,
                                                  "plan_refused", withdraw=False)
                    return
                pushing["task_id"] = task_id
                pushing["route_geo_id"] = route_geo_id
                pushing["goto_params"] = None
                res = await push_route_for_task(
                    task_id=task_id, route_geo_id=route_geo_id,
                    task_conn=conn, geo_conn=geo_conn,
                    snapshot_dao=snapshot_dao, progress_dao=progress_dao,
                    route_pub=route_pub, ack_window=ack_window,
                    trigger=trigger,  # RP-1, or RP-2 on a resume (above)
                    now_mono_ms=_now_mono_ms(), now_wall_ms=_now_wall_ms(),
                    # 15 S2.4.3: a recorded route (mode B) takes its arrive
                    # radius from common.recording.arrive_radius_m (99 U126,
                    # user ruling 2026-11-05: 1.0 m). Until then the key did
                    # not exist and None was passed on purpose, so EVERY
                    # teach-recorded patrol was refused by build_snapshot.
                    # Still None in tests that do not pass it -> refused, with
                    # the key named (CLAUDE.md 3.1: no value invented here).
                    recorded_arrive_radius_m=recorded_arrive_radius_m,
                    # 99 U127: the entry leg and the pass start from here
                    start=start_from_pose(state_cache.get("pose")),
                    lap_spec=lap_spec)  # opens the run with the pass count
                if not res.pushed:
                    pushing["task_id"] = None
                    await _fail_task_for_geometry(task_id, res.reason,
                                                  E_GEO_INCOMPLETE, res.cause,
                                                  withdraw=False)  # nothing in P1: first push, or a resume after the suspend clear

            async def _push_next_pass(task_id: str) -> None:
                """99 U131 RP-5: the next pass of a multi-pass patrol, from
                where the robot is now (the U127 plan alternates the ends of
                an open route by itself). Called by the progress sink after it
                has counted the pass; a refused push fails the task exactly as
                a refused dispatch does -- never left running with nothing
                moving."""
                full = await dao.fetch_by_id(task_id)
                route_geo_id = getattr(full, "route_geo_id", None) if full else None
                if not route_geo_id:
                    return
                pushing.update(task_id=task_id, route_geo_id=route_geo_id,
                               goto_params=None)
                res = await push_route_for_task(
                    task_id=task_id, route_geo_id=route_geo_id,
                    task_conn=conn, geo_conn=geo_conn,
                    snapshot_dao=snapshot_dao, progress_dao=progress_dao,
                    route_pub=route_pub, ack_window=ack_window,
                    trigger=RoutePushTrigger.RP5_NEXT_PASS,
                    now_mono_ms=_now_mono_ms(), now_wall_ms=_now_wall_ms(),
                    recorded_arrive_radius_m=recorded_arrive_radius_m,
                    start=start_from_pose(state_cache.get("pose")),
                    lap_spec=None)  # the run stays open; only loop_index moved
                if not res.pushed:
                    pushing["task_id"] = None
                    # The previous pass ARRIVED, so P1 is idle: nothing to withdraw.
                    await _fail_task_for_geometry(task_id, res.reason,
                                                  E_GEO_INCOMPLETE, res.cause,
                                                  withdraw=False)

            def _stop_motion(task_id: str) -> None:
                """Withdraw P1's navigation mission for a task that just left
                `running` (15 S7.2 step 1, 12 S4.2c.4): one cmd/motion/route
                op=clear on the SAME publisher the pushes use, so it cannot
                overtake a push sent before it nor be overtaken by the next
                task's push sent after it (one publisher, reliable, ordered).

                The ack window goes too when it belongs to this task. Without
                that, a cancel inside the 3 s confirmation window reaches the
                RA-3 retry branch of _on_ack_outcome, which re-pushes the whole
                route -- the cancelled task's geometry sent again after the
                clear (found reading the code on 2026-11-06)."""
                if pushing["task_id"] == task_id:
                    # Nothing is in flight for this task any more.
                    pushing["task_id"] = None
                    ack_window.reset()
                route_pub.put(json.dumps(build_route_clear(),
                                         ensure_ascii=False).encode("utf-8"))
                _logger.info("p3 route clear sent for task %s", task_id)

            async def _fail_task_for_geometry(task_id: str, why: str,
                                              code: str, cause: str, *,
                                              withdraw: bool) -> None:
                """15 S2.4.2 / S11.2: a task whose geometry could not be
                delivered goes to `failed` with E_GEO_INCOMPLETE -- NOT
                suspended (11 S4.4's suspend_reason closed set has no entry
                for it and 15 does not invent enum values, Q-P3-24).

                The alternative, leaving it `running`, is the failure this
                whole batch exists to remove: a task reported running with
                nothing moving."""
                _logger.error("p3 route push failed for task %s: %s",
                              task_id, why)
                try:
                    full = await dao.fetch_by_id(task_id)
                    if full is None or full.state != "running":
                        return
                    await dao.finish_task(
                        task_id, "failed", _now_mono_ms(),
                        finished_at=_now_utc_iso(), duration_sec=None)
                    await conn.execute(
                        "UPDATE tasks SET error_context_json=? "
                        "WHERE task_id=?",
                        # cause: the 11 S6.2 closed value P4 voices the
                        # failure by (99 U140); detail stays the log text.
                        (json.dumps({"code": code, "detail": why,
                                     "cause": cause}, ensure_ascii=False),
                         task_id))
                    await conn.commit()
                    await _make_publish(
                        state_pub, _emit_task_event, _publish_task_state,
                        fetch_terminal=_fetch_terminal_facts,
                        # withdraw: an earlier push of THIS task may be in P1
                        # (RA-3 is a timeout, not a refusal). Required, no
                        # default: a refusal at first dispatch sent nothing,
                        # and a stray clear there would preempt a running
                        # relative_move (CLAUDE.md 9.1A: no silent default).
                        stop_motion=_stop_motion if withdraw else None)(
                            task_id, "running", "failed", "geo_push_failed")
                except Exception as exc:      # noqa: BLE001
                    _logger.error("p3 geo-push failure handling failed: %s",
                                  exc)

            async def _on_ack_outcome(outcome) -> None:
                """RA-1 / RA-2 / RA-3 (15 S2.4.4)."""
                task_id = pushing["task_id"]
                route_geo_id = pushing["route_geo_id"]
                goto_params = pushing["goto_params"]
                if outcome.ack.value == "RA-1":
                    _logger.info("p3 route push confirmed (RA-1) for task %s "
                                 "after %d attempt(s)", task_id,
                                 outcome.attempts)
                    pushing["task_id"] = None
                    return
                if not task_id or (not route_geo_id and goto_params is None):
                    return
                if ack_outcome_is_fatal(outcome):
                    pushing["task_id"] = None
                    ack_window.reset()
                    await _fail_task_for_geometry(
                        task_id, "%s after %d attempt(s)"
                        % (outcome.ack.value, outcome.attempts),
                        E_GEO_INCOMPLETE, "route_unconfirmed",
                        withdraw=True)  # RA-3 is a timeout: P1 may still load it
                    return
                # Retry: the WHOLE route again, not the missing chunks --
                # 11 S3.5A discards the whole set on a gap, so P1 holds no
                # partial copy to complete (15 S2.4.2 retry row, Q-P3-26).
                _logger.warning("p3 route push %s for task %s, re-pushing "
                                "(attempt %d)", outcome.ack.value, task_id,
                                outcome.attempts + 1)
                if goto_params is not None:
                    await _push_goto(task_id, goto_params,
                                     RoutePushTrigger.RP2_REMAP)
                    return
                res = await push_route_for_task(
                    task_id=task_id, route_geo_id=route_geo_id,
                    task_conn=conn, geo_conn=geo_conn,
                    snapshot_dao=snapshot_dao, progress_dao=progress_dao,
                    route_pub=route_pub, ack_window=ack_window,
                    trigger=RoutePushTrigger.RP2_REMAP,
                    now_mono_ms=_now_mono_ms(), now_wall_ms=_now_wall_ms(),
                    recorded_arrive_radius_m=recorded_arrive_radius_m,  # same key as RP-1 (U126)
                    start=start_from_pose(state_cache.get("pose")),  # re-planned from where the robot is now (U127)
                    lap_spec=None)  # the run is already open
                if not res.pushed:
                    pushing["task_id"] = None
                    await _fail_task_for_geometry(task_id, res.reason,
                                                  E_GEO_INCOMPLETE, res.cause,
                                                  withdraw=True)  # an earlier push is out

            # 11 S3.3.4 positioning loss (99 U142): judged from state/pose each
            # pass. None in tests that pass no sustain times (the judgement is
            # off there); the production __main__ refuses to start without them.
            rtk_monitor = (RtkLossMonitor(rtk_lost_sustain_s, rtk_recover_sustain_s)
                           if rtk_lost_sustain_s is not None else None)
            _rtk_seq = [0]  # eid counter, boot-unique with _BOOT_ID
            _rtk_task = {"id": None}  # the task this loss suspended (named on recovery)

            def _emit_rtk(sev: str, title: str, detail: dict, *, dedup: bool) -> None:
                """event/{sev}/rtk (11 S3.3.4; key segment == payload sev, V-2).
                channel alarm by category (U18); the loss carries the 60 s dedup
                window of the contract example, the recovery none."""
                _rtk_seq[0] += 1
                pose = state_cache.get("pose") or {}
                body = {"eid": "rtk-%s-%d" % (_BOOT_ID, _rtk_seq[0]), "sev": sev,
                        "cat": "rtk", "channel": "alarm", "title": title,
                        "detail": detail,
                        "pos": {"lat": pose.get("lat"), "lon": pose.get("lon"),
                                "heading_rad": pose.get("heading_rad")},
                        # WALL-CLOCK-OK(record): 11 S6.1 Event.ts of this transition.
                        "src": "p3_task", "ts": time.time()}
                if dedup:
                    body.update(dedup_key="rtk:lost", dedup_window_s=60)
                gen.put("event/%s/rtk" % sev, json.dumps(
                    body, ensure_ascii=False).encode("utf-8"))

            async def _on_rtk_lost(now_s: float) -> None:
                """RL-2 then RL-4: suspend the running task (its mission is
                withdrawn by the route clear, so recovery cannot restart it),
                then raise the alarm with the task facts (11 S3.3.4)."""
                pose = state_cache.get("pose") or {}
                detail = rtk_monitor.lost_detail(pose, now_s)
                susp = await suspend_running_for_rtk(dao, conn, _now_mono_ms())
                detail["task_id"] = susp[0] if susp else None
                _rtk_task["id"] = detail["task_id"]  # P4 says "请说继续" only if one waits
                detail["action_taken"] = "stop_and_suspend" if susp else "stop_and_hold"
                detail["task_progress"] = None
                if susp:
                    row = await progress_dao.fetch_active(susp[0])
                    if row is not None:  # U07a breakpoint, 11 S4.4 field names
                        detail["task_progress"] = {"waypoint_index": row[4],
                                                   "seg_done_m": row[6],
                                                   "dist_done_m": row[7]}
                    await _make_publish(
                        state_pub, _emit_task_event, _publish_task_state,
                        fetch_terminal=_fetch_terminal_facts,
                        stop_motion=_stop_motion)(susp[0], "running", susp[1], susp[2])
                _logger.warning("p3 RL-2: positioning lost (fix %s), task %s -> %s",
                                pose.get("fix_type"), detail["task_id"],
                                detail["action_taken"])
                _emit_rtk("alarm", "positioning lost, robot stopped", detail, dedup=True)

            def _on_rtk_recovered(now_s: float) -> None:
                """The recovery report (11 S3.3.3 L3 -> L2/L1). Nothing resumes:
                RL-7, the suspended task waits for a human resume."""
                pose = state_cache.get("pose") or {}
                lost_s = (round(now_s - rtk_monitor.lost_since, 1)
                          if rtk_monitor.lost_since is not None else None)
                _logger.info("p3 positioning recovered (fix %s) after %ss",
                             pose.get("fix_type"), lost_s)
                _emit_rtk("info", "positioning recovered",
                          {"type": "heading_recovered", "fix_type": pose.get("fix_type"),
                           "heading_source": pose.get("heading_source"),
                           "lost_s": lost_s, "task_id": _rtk_task["id"]}, dedup=False)
                _rtk_task["id"] = None

            # 99 U146 low-battery return (the user's rule; replaces 15 S8.2's
            # CHG-10/11 and S8.2A). None in tests that pass no thresholds.
            lb_monitor = (lb.LowBatteryMonitor(low_battery["trigger_soc_pct"],
                                               low_battery["stop_soc_pct"],
                                               low_battery["cloud_alarm_period_s"],
                                               low_battery["charge_full_soc_pct"],
                                               low_battery["swap_resume_soc_pct"])
                          if low_battery is not None else None)
            # The episode's facts: the target picked at "low", the task
            # injected, whether the next alarm is the first (P4 speaks once),
            # and the held set (99 U157): every task that was live when the
            # episode began -- the scheduler neither resumes nor dispatches
            # them until the episode ends.
            lb_ctx: dict = {"target": None, "task_id": None, "first": False, "n": 0,
                            "held": set()}
            from xbrain.p3_task.dao.simple_daos import MemoryDAO
            lb_memory = MemoryDAO(conn)

            async def _lb_persist() -> None:
                """Write the episode to task.db memory (99 U157) so a restart
                mid-charge keeps holding the interrupted tasks. Idle writes an
                idle record (decode_episode reads it as no episode)."""
                phase = lb_monitor.phase if lb_monitor is not None else lb.PHASE_IDLE
                await lb_memory.put(lb.EPISODE_MEMORY_KEY,
                                    lb.encode_episode(phase, lb_ctx["held"], lb_ctx["task_id"]),
                                    _now_mono_ms())
                await conn.commit()
            chassis_ctrl_pub = gen.declare_publisher("cmd/chassis/ctrl")
            _ctrl_seq = [0]  # S3.0 seq per key for this process

            def _emit_charging(sev: str, title: str, detail: dict) -> None:
                """event/{sev}/charging (11 S6.2 category charging; channel
                alarm so the cloud cursor replays it). No dedup: the 2 s
                cadence IS the "continuous" report the ruling asks for."""
                lb_ctx["n"] += 1
                gen.put("event/%s/charging" % sev, json.dumps({
                    "eid": "lowbatt-%s-%d" % (_BOOT_ID, lb_ctx["n"]), "sev": sev,
                    "cat": "charging", "channel": "alarm", "title": title,
                    "detail": detail,
                    # WALL-CLOCK-OK(record): 11 S6.1 Event.ts of this report.
                    "src": "p3_task", "ts": time.time(),
                }, ensure_ascii=False).encode("utf-8"))

            def _publish_prone() -> None:
                """cmd/chassis/ctrl {action: prone} (11 S9.3.4) in a full S3.0
                envelope -- quadruped refuses a bare body (S9.3.4 E-3, the P4
                stamper's lesson). origin "system": P3's own safety action
                (99 U146 adds it to the S9.3.4 origin set)."""
                from xbrain.common.envelope.envelope import Envelope, encode
                from xbrain.common.envelope import read_local_boot_id
                rid = os.environ.get("XBRAIN_ROBOT_ID", "")
                if not rid:
                    _logger.error("p3 low-battery prone NOT sent: XBRAIN_ROBOT_ID unset")
                    return
                body = {"cmd_id": "cc-lb-%s-%d" % (_BOOT_ID, _ctrl_seq[0]),
                        "action": "prone", "origin": "system"}
                env = Envelope(v=1, rid=rid,
                               # WALL-CLOCK-OK(align): the S3.0 ts field only.
                               ts=time.time(), mono=time.monotonic(),
                               boot=read_local_boot_id(), seq=_ctrl_seq[0],
                               src="p3_task", ts_sync=False, data=body)
                _ctrl_seq[0] += 1
                chassis_ctrl_pub.put(json.dumps(encode(env), ensure_ascii=False,
                                                separators=(",", ":")).encode("utf-8"))

            def _lb_detail(kind: str, power: dict) -> dict:
                """The facts every low-battery report carries."""
                tgt = lb_ctx["target"]
                return {"type": kind, "soc_pct": power.get("soc_pct"),
                        "remain_mile_km": power.get("remain_mile_km"),
                        "target_kind": tgt.kind if tgt else None,
                        "target_name": tgt.name if tgt else None,
                        "task_id": lb_ctx["task_id"], "first": lb_ctx["first"]}

            async def _on_low_battery(ev: str) -> None:
                """One LowBatteryMonitor event (99 U146)."""
                power = state_cache.get("power") or {}
                pose = start_from_pose(state_cache.get("pose"))
                routes = await load_route_geoms(geo_conn)
                member = route_membership_m if route_membership_m is not None else 0.0
                if ev == "low":
                    tgt = await lb.pick_target(geo_conn, pose, routes, member)
                    lb_ctx.update(target=tgt, task_id=None, first=True)
                    if tgt is not None:
                        from xbrain.p3_task.ingest.id_alloc import next_submit_seq
                        tid = "lb-%s-%d" % (_BOOT_ID, lb_ctx["n"] + 1)
                        await dao.insert(lb.build_low_battery_row(
                            tid, await next_submit_seq(conn), tgt,
                            soc_pct=power.get("soc_pct"), now_mono_ms=_now_mono_ms(),
                            created_at=_now_utc_iso()))
                        await conn.commit()
                        lb_ctx["task_id"] = tid
                    # 99 U157: hold every task live right now except the one
                    # just injected -- the running one is preempted by it next
                    # tick and must not resume when it arrives. A union: back
                    # from charging, the earlier holds stay.
                    lb_ctx["held"] |= (await lb.live_task_ids(dao)) - {lb_ctx["task_id"]}
                    await _lb_persist()
                    _logger.warning("p3 low battery (soc %s): target %s -> task %s, "
                                    "holding %s", power.get("soc_pct"),
                                    tgt.name if tgt else None, lb_ctx["task_id"],
                                    sorted(lb_ctx["held"]))
                elif ev == "alarm" and lb_monitor.phase == lb.PHASE_CHARGING:
                    # 99 U157: on the charger the alarm goes on, worded
                    # "charging", until charge_full_soc_pct. Never voiced.
                    d = _lb_detail("battery_charging", power)
                    d["first"] = False
                    _emit_charging("alarm", "charging after low battery", d)
                elif ev == "alarm" and lb_monitor.phase == lb.PHASE_INTERRUPTED:
                    # 99 U157 ruling 2: unplugged before full, still above the
                    # trigger -- the interrupted tasks keep waiting.
                    d = _lb_detail("charge_interrupted", power)
                    d["first"] = False
                    _emit_charging("alarm", "charging interrupted before full", d)
                elif ev == "alarm":
                    unr = lb.unreachable(power, pose, lb_ctx["target"], routes, member)
                    kind = ("low_battery_no_target" if lb_ctx["target"] is None
                            else "low_battery_unreachable" if unr else "low_battery")
                    d = _lb_detail(kind, power)
                    d["unreachable"] = unr
                    _emit_charging("alarm", "low battery, returning to charge", d)
                    lb_ctx["first"] = False  # P4 speaks the first one only
                elif ev == "stop":
                    susp = await lb.suspend_running_for_low_battery(dao, conn, _now_mono_ms())
                    if susp is not None:
                        await _make_publish(
                            state_pub, _emit_task_event, _publish_task_state,
                            fetch_terminal=_fetch_terminal_facts,
                            stop_motion=_stop_motion)(susp[0], "running", susp[1], susp[2])
                    _publish_prone()
                    _logger.warning("p3 low battery stop (soc %s): stopped and prone",
                                    power.get("soc_pct"))
                    d = _lb_detail("low_battery_stop", power)
                    d["first"] = True  # its own one-time voice
                    _emit_charging("alarm", "battery critical, stopped and lying down", d)
                elif ev == "charging":
                    # 99 U157: charging does NOT end the episode any more -- it
                    # turns the alarm into "charging" (above) and keeps the
                    # held tasks held. The injected task has done its job:
                    # cancelled, as under U146 (left suspended, a later
                    # "继续" would drive the charging robot back to the dock).
                    # The 5 % dispatch pause lifts (the monitor's `stopped`).
                    _logger.info("p3 low battery: charging (soc %s), holding %s "
                                 "until %s %%", power.get("soc_pct"),
                                 sorted(lb_ctx["held"]), low_battery["charge_full_soc_pct"])
                    closed = await lb.close_low_battery_task(
                        dao, conn, lb_ctx["task_id"], _now_mono_ms())
                    if closed is not None:
                        await _make_publish(
                            state_pub, _emit_task_event, _publish_task_state,
                            fetch_terminal=_fetch_terminal_facts,
                            stop_motion=_stop_motion)(closed[0], closed[1], closed[2],
                                                      "low_battery_charging")
                    lb_ctx.update(task_id=None)
                    await _lb_persist()
                elif ev == "interrupted":
                    _logger.warning("p3 low battery: charging stopped at soc %s before "
                                    "full, still holding %s", power.get("soc_pct"),
                                    sorted(lb_ctx["held"]))
                    await _lb_persist()
                elif ev in ("charge_full", "swapped", "swapped_partial"):
                    # The ends of an episode (99 U157): full charge, or a swap
                    # at / above swap_resume_soc_pct -> the held tasks resume by
                    # themselves on the next tick; a swap below it ends the
                    # alarm but parks the interrupted tasks for a person.
                    auto = ev != "swapped_partial"
                    cause = "charge_full" if ev == "charge_full" else "battery_swap"
                    _logger.info("p3 low battery closed by %s (soc %s), auto resume %s",
                                 cause, power.get("soc_pct"), auto)
                    closed = await lb.close_low_battery_task(
                        dao, conn, lb_ctx["task_id"], _now_mono_ms())
                    if closed is not None:
                        await _make_publish(
                            state_pub, _emit_task_event, _publish_task_state,
                            fetch_terminal=_fetch_terminal_facts,
                            stop_motion=_stop_motion)(
                                closed[0], closed[1], closed[2],
                                "low_battery_charging" if cause == "charge_full"
                                else "low_battery_swapped")
                    parked: list = []
                    if not auto:
                        parked = await lb.park_held_for_manual(
                            dao, conn, lb_ctx["held"], _now_mono_ms())
                        if parked:
                            await _publish_task_state()
                    d = _lb_detail("low_battery_cleared", power)
                    d["cause"] = cause  # 11 S9.11.2: charge_full | battery_swap
                    d["auto_resume"] = auto
                    d["held"] = sorted(lb_ctx["held"])
                    d["parked"] = parked
                    _emit_charging("info", "low battery alarm closed by %s" % cause, d)
                    lb_ctx.update(target=None, task_id=None, first=False, held=set())
                    await _lb_persist()

            # 99 U157: a P3 restart mid-episode -- restore the phase, the held
            # tasks and the injected task from task.db memory, so the restart
            # neither resumes the interrupted tasks on the charger nor injects
            # a second return task. Read before the loop's first tick.
            if lb_monitor is not None:
                try:
                    _rec = lb.decode_episode(await lb_memory.get(lb.EPISODE_MEMORY_KEY))
                    if _rec is not None:
                        lb_monitor.restore(_rec[0])
                        lb_ctx.update(held=set(_rec[1]), task_id=_rec[2])
                        if _rec[0] == lb.PHASE_LOW:
                            # the alarm names the target; no pose yet at boot,
                            # so the same deterministic pick as a pose-less low
                            lb_ctx["target"] = await lb.pick_target(
                                geo_conn, None, await load_route_geoms(geo_conn),
                                route_membership_m if route_membership_m is not None else 0.0)
                        _logger.warning("p3 boot: low-battery episode restored (phase %s, "
                                        "holding %s, task %s)", _rec[0],
                                        sorted(_rec[1]), _rec[2])
                except Exception as exc:      # noqa: BLE001
                    _logger.error("p3 boot: low-battery episode restore failed: %s", exc)

            # 15 S10 INIT / 11 S9.14: a `running` row at start-up is left over
            # from an unclean shutdown -- suspend it (passive, unclean_shutdown)
            # before the first scheduler tick, or it blocks every dispatch for
            # good (measured on the robot 2026-11-06). Published here, after
            # the publishers exist: the task transition (state/task + 11 S6.2
            # event) and the 11 S9.14 system event "补发给云端".
            from xbrain.p3_task.lifecycle.boot_recover import (
                BOOT_SUSPEND_REASON, suspend_unclosed_running)
            try:
                _dead = await suspend_unclosed_running(dao, conn, _now_mono_ms())
            except Exception as exc:      # noqa: BLE001
                # Loud, not fatal: P3 still serves geo / teach / queries, and the
                # operator can cancel the dead row by voice; the log says why
                # nothing dispatches.
                _dead = []
                _logger.error("p3 INIT unclean-shutdown recovery failed, a "
                              "leftover running task will block dispatch: %s", exc)
            for _tid in _dead:
                _logger.warning("p3 INIT: task %s was running at start-up "
                                "(unclean shutdown) -> suspended", _tid)
                await _make_publish(
                    state_pub, _emit_task_event, _publish_task_state,
                    fetch_terminal=_fetch_terminal_facts)(
                        _tid, "running", "suspended", BOOT_SUSPEND_REASON)
                gen.put("event/warn/system", json.dumps({
                    "eid": "boot-%s-%s" % (_BOOT_ID, _tid),
                    "title": "task %s suspended after unclean shutdown" % _tid,
                    "detail": {"type": BOOT_SUSPEND_REASON, "task_id": _tid},
                    # WALL-CLOCK-OK(record): 11 S6.1 Event.ts of this recovery.
                    "src": "p3_task", "ts": time.time(),
                }, ensure_ascii=False).encode("utf-8"))

            # 99 U146: at boot, no 返航终点站 (it must exist) or a 充电桩 / 充电站
            # point off every route -> one cloud report naming what is wrong.
            try:
                _homes = await lb.list_role_points(geo_conn, "home")
                _docks = await lb.list_role_points(geo_conn, "dock")
                _routes0 = await load_route_geoms(geo_conn)
                _member0 = route_membership_m if route_membership_m is not None else 0.0
                _off = [d[1] for d in _docks
                        if not lb.on_route((d[2], d[3]), _routes0, _member0)]
                _on = [d for d in _docks if d[1] not in _off]
                if not _homes or _off:
                    _missing = (["返航终点站"] if not _homes else []) + (
                        ["充电桩/充电站"] if not _on else [])
                    _logger.warning("p3 boot: return targets missing %s, docks off "
                                    "route %s", _missing, _off)
                    _emit_charging("alarm", "return targets not defined", {
                        "type": "return_target_missing", "missing": _missing,
                        "docks_off_route": _off, "first": True})
            except Exception as exc:      # noqa: BLE001
                _logger.error("p3 boot return-target check failed: %s", exc)

            last_hb = time.monotonic()
            last_geo = 0.0            # 0 -> publish geo on the very first pass
            last_fence = 0.0          # 0 -> publish fence on the very first pass
            last_teach = 0.0          # 0 -> publish teach state immediately
            last_task_state = 0.0     # 0 -> publish TaskState on first pass
            fence_rev = [1]           # 11 S9A.2 rev; +1 on any geometry change
            last_fence_sig = [None]   # rev-0 crc32 of the last broadcast set
            fence_invalid_logged = [False]
            # poly_id -> reason, for the rows build_fence_set could not put on
            # the wire. A latch per ID, not a boolean: two different unusable
            # fences must both be named, and neither may be re-logged every
            # second by the 1 Hz broadcast. Cleared for an id that stops being
            # unusable, so a fence the operator then fixes logs again if it
            # ever breaks a second time.
            fence_unusable_logged: dict = {}
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
                        if (isinstance(payload, dict) and payload.get("action") == "submit"
                                and lb_monitor is not None and lb_monitor.stopped):
                            # 99 U146: stopped and lying down at the last 5 %:
                            # no task is accepted (it could not run).
                            task_ack_pub.put(json.dumps(task_ack(
                                str(payload.get("cmd_id") or ""), "rejected", E_LOW_BATTERY,
                                {"reason": "battery_critical",
                                 "soc_pct": (state_cache.get("power") or {}).get("soc_pct")}),
                                ensure_ascii=False).encode("utf-8"))
                        elif (isinstance(payload, dict) and rtk_monitor is not None
                                and rtk_monitor.lost and payload.get("action") == "submit"):
                            # 11 S3.2.1 single / no_fix: no task is accepted
                            # (99 U142). Refused, not queued: a queued task would
                            # start by itself on recovery (RL-7).
                            task_ack_pub.put(json.dumps(task_ack(
                                str(payload.get("cmd_id") or ""), "rejected", E_DEGRADED,
                                {"reason": "rtk_lost",
                                 "fix_type": (state_cache.get("pose") or {}).get("fix_type")}),
                                ensure_ascii=False).encode("utf-8"))
                        elif isinstance(payload, dict) and "action" in payload:
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
                                    fetch_terminal=_fetch_terminal_facts,
                                    # operator cancel / pause of the running
                                    # task: the robot stops (2026-11-06).
                                    stop_motion=_stop_motion))
                            task_ack_pub.put(json.dumps(
                                ack, ensure_ascii=False).encode("utf-8"))
                            if ack.get("result") == "accepted":
                                recorded += 1
                                # 99 U157: a person resumed (or cancelled) a held
                                # task -- their call; it leaves the held set so the
                                # scheduler dispatches it like any ready task.
                                _acked = (ack.get("detail") or {}).get("task_id")
                                if (payload.get("action") in ("resume", "cancel")
                                        and _acked in lb_ctx["held"]):
                                    lb_ctx["held"].discard(_acked)
                                    await _lb_persist()
                                    _logger.info("p3 low battery: %s released from the "
                                                 "hold by %s", _acked, payload.get("action"))
                                # ES-3 (15 S11.1 F5): a human submit/resume is
                                # itself the unfreeze signal, and is executed
                                # (already recorded above). NO an auto/charge
                                # task must not unfreeze -- a return_home
                                # arriving mid-estop must not lift the freeze.
                                if (estop_ctrl.frozen
                                        and is_human_resume_command(payload)):
                                    estop_ctrl.unfreeze(human_source(payload))
                                    _logger.warning(
                                        "p3 ES-3: unfrozen by human %s %s -> "
                                        "scheduling resumes",
                                        payload.get("source"),
                                        payload.get("action"))
                                # 15 S11.4 (99 U111): an operator's pause holds
                                # the queue too -- without it the next queued
                                # task started the same second and the robot
                                # never paused. Lifted like an estop freeze.
                                if is_human_pause_command(payload):
                                    estop_ctrl.freeze("operator_pause")
                                    _logger.warning(
                                        "p3 operator pause: scheduling HELD "
                                        "until a human submit / resume")
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
                            # the S8.5 probe verdict rides state/link (11 S4.6)
                            teach.update_estop_path(link_holder.get("estop_path"))
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
                            # 99 U133: a named session is saved at once, as
                            # an operator's finish would be
                            _auto = await teach.save_named(_now_wall_ms())
                            if _auto.get("reason") != "no_named_session":
                                last_teach = 0.0
                                last_geo = last_fence = 0.0
                    except Exception as exc:      # noqa: BLE001
                        _logger.error("p3 teach sampling failed: %s", exc)
                    if rtk_monitor is not None:
                        # 11 S3.3.4 (99 U142): before the scheduler block, so a
                        # loss declared this pass already holds this pass.
                        try:
                            _rtk_ev = rtk_monitor.update(state_cache.get("pose"), now_mono_s)
                            if _rtk_ev == "lost":
                                await _on_rtk_lost(now_mono_s)
                            elif _rtk_ev == "recovered":
                                _on_rtk_recovered(now_mono_s)
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 positioning-loss step failed: %s", exc)
                    if lb_monitor is not None:
                        # 99 U146 low battery, every pass (the 2 s alarm cadence
                        # needs the 0.5 s loop, not the 1 Hz power frame).
                        try:
                            for _lev in lb_monitor.update(state_cache.get("power"),
                                                          now_mono_s):
                                await _on_low_battery(_lev)
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 low-battery step failed: %s", exc)
                    while not teach_queue.empty():
                        # S12A.3 arming check 2 needs the running task types;
                        # until 99 U133 none were passed, so the check never
                        # fired in production.
                        _rcur = await conn.execute(
                            "SELECT task_type FROM tasks WHERE state='running'")
                        _running = tuple(r[0] for r in await _rcur.fetchall())
                        t_ack = await teach.handle(
                            teach_queue.get_nowait(), now_mono_s=now_mono_s,
                            now_ms=_now_wall_ms(), running_task_types=_running)
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
                            now_mono_ms=_now_mono_ms(),
                            created_at=_now_utc_iso())
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
                    # 15 S4.1A (99 U99): B / D took over motion. None until the
                    # first state/mode arrives -- treated as "no takeover", the
                    # pre-2026-10-28 behaviour, so a P2 that is down does not
                    # freeze the task layer.
                    _mb = mode_view.get("motion_behavior")
                    _takeover = _mb is not None and _mb != "normal"
                    try:
                        if _takeover:
                            _ys = await suspend_running_for_takeover(
                                dao, conn, _now_mono_ms())
                            if _ys is not None:
                                _tid, _to, _reason = _ys
                                _logger.warning("p3 mode takeover: yielded "
                                                "task %s (%s)", _tid, _reason)
                                await _make_publish(
                                    state_pub, _emit_task_event,
                                    _publish_task_state,
                                    fetch_terminal=_fetch_terminal_facts)(
                                        _tid, "running", _to, _reason)
                        elif _mb is not None:
                            for _tid, _to in await resume_after_takeover(
                                    dao, conn, _now_mono_ms()):
                                _logger.info("p3 mode takeover over: task %s "
                                             "-> %s", _tid, _to)
                                await _make_publish(
                                    state_pub, _emit_task_event,
                                    _publish_task_state,
                                    fetch_terminal=_fetch_terminal_facts)(
                                        _tid, "suspended", _to, "resume")
                    except Exception as exc:      # noqa: BLE001
                        _logger.error("p3 mode takeover step failed: %s", exc)
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
                                    fetch_terminal=_fetch_terminal_facts,
                                    # 15 S7.2 trigger 3: the estop holds P1
                                    # now, but its release would let RNS
                                    # resume a suspended task's mission.
                                    stop_motion=_stop_motion)(
                                        _tid, "running", _to, _reason)
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 ES-2 suspend failed: %s", exc)
                    elif _takeover:
                        # 15 S4.1A: no new task is dispatched during B / D.
                        pass
                    elif lb_monitor is not None and lb_monitor.stopped:
                        # 99 U146: lying down at the last 5 %: nothing dispatches.
                        pass
                    elif rtk_monitor is not None and rtk_monitor.lost:
                        # 11 S3.2.1 single / no_fix (99 U142): nothing is
                        # dispatched -- queued and timed tasks wait for a fix.
                        pass
                    elif teach.active:
                        # 11 S12A.3 CMD-16 / 99 U133: nothing is dispatched
                        # while a recording runs; queued and timed tasks wait
                        # until it ends (a timed patrol firing mid-recording
                        # would drive off under the operator's keyboard).
                        pass
                    else:
                        try:
                            _made = await scheduler_tick(
                                conn, dao, now_mono_ms=_now_mono_ms(),
                                on_transition=_make_publish(
                                    state_pub, _emit_task_event,
                                    _publish_task_state,
                                    fetch_terminal=_fetch_terminal_facts,
                                    # preempt (15 S7.2 trigger 2): the clear
                                    # goes out in-tick, ahead of the winner's
                                    # push that follows the tick below.
                                    stop_motion=_stop_motion),
                                # 与 created_at 同一口径的墙钟(15 S9.5):
                                # started_at 是给人看的下发时间, 由调用方生成
                                # 而不是 driver 自己取 -- driver 里不应有第二
                                # 处时钟来源(CLK-C1 的同一理由: 时间从外面传).
                                started_at=_now_utc_iso(),
                                boot_id=_BOOT_ID,
                                list_route_ids=_list_route_ids,
                                # 99 U110: timed tasks (not_before_ts) come
                                # due on the system clock P4 scheduled them on.
                                now_wall_s=_now_wall_ms() / 1000.0,
                                # 99 U157: tasks a low-battery episode holds.
                                held=frozenset(lb_ctx["held"]))
                            # EX-2 RP-1 (15 S2.4.1): every ready -> running
                            # this tick made gets its geometry pushed. Driven
                            # off the tick's RETURN VALUE rather than from
                            # inside on_transition, because the push must not
                            # run inside the tick's single transaction -- SN-1
                            # requires the snapshot COMMITTED before the first
                            # frame leaves, and committing mid-tick would
                            # publish half a scheduling pass.
                            for _tid, _from, _to in _made:
                                if (_from, _to) != ("ready", "running"):
                                    continue
                                await _push_route_on_dispatch(_tid)
                        except Exception as exc:      # noqa: BLE001
                            _logger.error("p3 scheduler tick failed: %s", exc)
                    # EX-2 RA-2/RA-3 (15 S2.4.4): the ack window is a timeout,
                    # so it has to be ticked by something. Done every pass
                    # (0.5 s) against a 3 s window.
                    _ack = ack_window.tick(now_mono_ms=_now_mono_ms())
                    if _ack is not None:
                        await _on_ack_outcome(_ack)
                    # EX-3 / EX-4: drain the P1-12 backlog. AFTER the
                    # scheduler tick, so a task the tick has just dispatched
                    # already exists as `running` when the first frame for it
                    # is applied; before it, the first frame or two of every
                    # run would be dropped as "no running task".
                    #
                    # Drained to EMPTY each pass rather than one per pass: at
                    # 2 Hz against this loop's 0.5 s cadence the two rates are
                    # equal, so a one-per-pass drain has no margin at all and
                    # any hiccup becomes a permanent backlog.
                    while True:
                        try:
                            _pbody = progress_queue.get_nowait()
                        except asyncio.QueueEmpty:
                            break
                        # EX-2 RA-1 BEFORE the persistence half: the very
                        # frame that confirms the push is also the first one
                        # that describes the new route, so feeding the window
                        # after would leave the window open for one extra
                        # pass on every dispatch. Parse failures are the
                        # persistence half's to report -- this one stays
                        # silent so a bad frame is not logged twice.
                        # 99 U147 nav audit: a frame P1 built before the
                        # latest push is the previous run's (same route id
                        # and rev on a repeat / next pass / resume), so it
                        # may neither confirm the push nor close the task.
                        if predates_push(_pbody, ack_window.pushed_ms):
                            continue
                        if ack_window.is_open:
                            try:
                                _pp = parse_path_progress(_pbody)
                            except Exception:      # noqa: BLE001
                                _pp = None
                            if _pp is not None:
                                _ack = ack_window.observe(
                                    loading=_pp.loading,
                                    route_id=_pp.route_id,
                                    route_rev=_pp.route_rev)
                                if _ack is not None:
                                    await _on_ack_outcome(_ack)
                        try:
                            await apply_path_progress(
                                _pbody, conn=conn, dao=dao,
                                progress_dao=progress_dao,
                                tracker=progress_tracker,
                                now_mono_ms=_now_mono_ms(),
                                now_wall_ms=_now_wall_ms(),
                                on_transition=_make_publish(
                                    state_pub, _emit_task_event,
                                    _publish_task_state,
                                    fetch_terminal=_fetch_terminal_facts),
                                boot_id=_BOOT_ID,
                                next_pass=_push_next_pass,  # U131 multi-pass
                                pass_latch=pass_latch)
                        except Exception as exc:      # noqa: BLE001
                            # A progress frame must never take down the loop
                            # that also runs task scheduling and the geo
                            # single-writer.
                            _logger.error("p3 path_progress apply failed: %s",
                                          exc)
                    now = time.monotonic()
                    if now - last_hb >= heartbeat_period_s:
                        _logger.info(
                            "p3 alive; recorded=%d qdepth=%d progress_q=%d "
                            "progress_dropped=%d",
                            recorded, queue.qsize(), progress_queue.qsize(),
                            progress_dropped[0])
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
                            def _on_unusable(poly_id: str, reason: str) -> None:
                                # 11 FV-13 等校验挡不住的[已存]坏行: 该多边形
                                # 被丢出广播, 其余照发(见 build_fence_set).
                                # NO 不拒整集 -- 那会让刚启动的 p1 一个围栏都
                                # 收不到, 为保一条本就无从施加的区内限速, 把
                                # 真正安全关键的 allow/forbid 边界弄成 fail-open.
                                if fence_unusable_logged.get(poly_id) != reason:
                                    _logger.warning(
                                        "p3 fence %s dropped from the broadcast: "
                                        "%s", poly_id, reason)
                                    fence_unusable_logged[poly_id] = reason

                            sig = build_fence_set(
                                rows, fence_set_id=FENCE_SET_ID, rev=0,
                                on_unusable=_on_unusable)["crc32"]
                            if sig != last_fence_sig[0]:
                                if last_fence_sig[0] is not None:
                                    fence_rev[0] += 1
                                last_fence_sig[0] = sig
                            # enu_origin 必须随发布带上(11 S9A.2 必填字段).
                            # 第二次调用不再传 on_unusable: 同一批 rows 已在
                            # 上面的签名计算里报过一遍, 再传会让每拍记两条.
                            # 传 None 会落到模块 logger -- 所以给一个显式的
                            # no-op, 并写明为什么它是 no-op 而不是忘了传.
                            fs = build_fence_set(
                                rows, fence_set_id=FENCE_SET_ID,
                                rev=fence_rev[0], enu_origin=enu_origin,
                                on_unusable=lambda _pid, _r: None)
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
                  fetch_terminal=None, stop_motion=None):
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
    the queue for all the others, and a delta of one task cannot express that.

    stop_motion(task_id) withdraws P1's navigation mission (cmd/motion/route
    op=clear) when route_clear_needed says the task left `running` for a state in
    which the robot must not keep driving -- 15 S7.2 step 1, and the 2026-11-06
    finding that a cancelled / paused task kept moving. It is passed ONLY by the
    callers whose transitions P3 or an operator causes (task commands, scheduler
    preempt, ES-2, P3's own geometry failures). The P1-terminal relay
    (apply_path_progress) must not pass it: P1 is already idle there and a clear
    would preempt the relative_move or next push that follows. The B / D
    takeover caller must not pass it either: 15 S7.2 trigger 5 skips step 1
    because P2 is driving P1 already."""
    # Local import, same convention as TERMINAL_STATES below: this module keeps
    # only stdlib at the top.
    from xbrain.p3_task.route.push import route_clear_needed

    async def _publish(task_id: str, from_state: str, to_state: str,
                       reason: str) -> None:
        if reason:
            _logger.info("p3 task %s %s -> %s (%s)",
                         task_id, from_state, to_state, reason)
        else:
            _logger.info("p3 task %s %s -> %s", task_id, from_state, to_state)
        if stop_motion is not None and route_clear_needed(from_state, to_state):
            # FIRST, before any broadcast: 15 S7.2 orders "stop motion" ahead of
            # everything else, and a broadcast failure below must not be able to
            # leave the robot driving.
            try:
                stop_motion(task_id)
            except Exception as exc:      # noqa: BLE001
                # Logged loudly, never swallowed silently: a lost clear is a
                # robot still driving a task that is no longer running.
                _logger.error("p3 route clear for task %s failed: %s",
                              task_id, exc)
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
      * H04 reload_config -- 18 says verbatim it belongs on cmd/config (S7.6)
      (B10 skip_waypoint was the other one until 2026-11-05, when the intent
      was deleted, 99 U129.)
    It is logged by intent id so it is VISIBLE as a gap. They were being
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
