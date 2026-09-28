"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: main_wiring.py
Brief: p5_gateway voice-loop MVP wiring -- state/link publisher + event drain

Description:
Minimum-viable p5 for the voice-loop smoke test:

  * open GEN session
  * publish state/link (P5 is the UNIQUE publisher, 11 §7.1A)
    every 1 s -- lets HMI + Qt see 'gateway alive'
  * subscribe cmd/audio/speak/ack + state/task and log
  * subscribe event/{severity}/{category} and log

Full event pipeline (schema check + dedupe + record.db + cloud
uplink) lives in xbrain/p5_gateway/event/ and stays untouched by
this MVP. The purpose here is: 'gateway is alive AND observes the
downstream ACKs'.
"""

from __future__ import annotations

import json
import logging
import os
import time
from datetime import datetime, timezone
from typing import Optional

#: 11 S3.0 信封的共享编解码器. 探活 ping 走它而不是手拼一个 dict --
#: 八字段手写一次就是一次拼错的机会, 而拼错的表现是对端 read_envelope
#: 在某个字段上静默退出, 不报错也不回内容(13 DDS-9 的形状).
from xbrain.common.envelope import Envelope, encode, read_local_boot_id

_logger = logging.getLogger("xbrain.p5.wiring")


STATE_LINK_TOPIC = "state/link"
CMD_AUDIO_SPEAK_ACK_TOPIC = "cmd/audio/speak/ack"
STATE_TASK_TOPIC = "state/task"


def _now_mono_ms() -> int:
    return int(time.monotonic() * 1000)


CMD_ESTOP_TOPIC = "cmd/estop"
# 11 S12.1.1 W4: HMI geo edits out, P3's answers back. The ack key is subscribed
# so the browser learns the outcome; without it a delete would look like it
# vanished, and 12.3's reconnect rule (resend with the same req_id) would have
# nothing to resolve against.
CMD_GEO_TOPIC = "cmd/geo"
CMD_GEO_ACK_TOPIC = "cmd/geo/ack"
#: 11 S12.1.1 W2 (goto) and W7 (task) both land here. S2.2.3 (2) already listed
#: the HMI as a cmd/task publisher, so opening those two classes adds no key --
#: it uses one that was reserved for the HMI and never declared.
CMD_TASK_TOPIC = "cmd/task"
CMD_TASK_ACK_TOPIC = "cmd/task/ack"
#: 11 S12.1.1 W3 exit_broadcast lands on cmd/mode as a S7.3 ModeCommand.
#: S2.2.3 already lists the HMI among that key's publishers, so W3 -- like
#: W2/W7 before it -- opens a class without adding a key.
CMD_MODE_TOPIC = "cmd/mode"
CMD_MODE_ACK_TOPIC = "cmd/mode/ack"
# 11 S12A.5: the recording session, READ ONLY on this side. P5 shows it and
# never writes to cmd/teach -- teach is not one of the five upstream types of
# S12.1.1, and W6 teleop being removed means a browser could not drive the
# robot along a route it started recording anyway.
STATE_TEACH_TOPIC = "state/teach"
CMD_FENCE_TOPIC = "cmd/fence"            # W1: fence geometry (17 S6.9, P5 consumes)
STATE_GEO_OBJECTS_TOPIC = "state/geo/objects"  # 11 S7.10A: routes/keypoints/docks geometry
STATE_MODE_TOPIC = "state/mode"          # W3: P2 usage mode (10 Hz)
STATE_AUDIO_TOPIC = "state/audio"        # 11 S8.10: P2 audio state (1 Hz + on change)
STATE_POSE_TOPIC = "state/pose"          # P1-1: pose + RTK heading (p1 bridge)
STATE_CLOCK_TOPIC = "state/clock"        # P1-13: clock sync mirror (18-C G47)
EVENT_WILDCARD_TOPIC = "event/**"        # W2: event/{severity}/{category} stream
EVENT_ACK_TOPIC = "event/ack"            # 17 S3.5.1: cloud ack -> mark delivered
EVENT_RECON_RSP_TOPIC = "event/recon/rsp"  # 17 S3Y.3: cloud recon answer
RECON_PERIOD_S = 300.0                    # 17 S3Y.3 recon.period_s (interim const)
#: 11 S2.2.2 的机内裸 key. 两条都由 chassis_relay 转发上来(CR-4 / CR-5),
#: 2026-09-26 relay 上机后才第一次真的有内容 -- 在那之前 p5 订了也只会
#: 收到 0 条, 所以直到本轮才接.
#:
#: state/robot: S2.2.2 该行的消费者列逐字含 p5_gateway.
#: state/power: 同表该行的消费者列写的是 "p3_task . p2_core . HMI . 云端" --
#:   而 HMI 后端与云端面[都在 p5 进程内](CLAUDE.md S0.1: 系统无独立 HMI
#:   进程; 11:1671 逐字: 云端面 p5_gateway 是唯一发布/订阅方). 换句话说
#:   那两个消费者除了在这里订, 没有别的地方可以存在.
STATE_ROBOT_TOPIC = "state/robot"        # CR-4: RobotState 10 Hz (11 S4.1)
STATE_POWER_TOPIC = "state/power"        # CR-5: PowerState 1 Hz (11 S4.2)
PROBE_ESTOP_PING_TOPIC = "probe/estop/ping"  # W5: P5 ping (11 CR-2, 17 S6.3)
PROBE_ESTOP_PONG_TOPIC = "probe/estop/pong"
# W5 的 pong 由 chassis_relay 发 -- 11 S2.2 那一行逐字: 发布者
# chassis_relay(CR-3, 转发自 rt/safety/probe/pong), 消费者 p5_gateway, 1 Hz.
# *** 本行原写 "quadruped pong", 是错的.
# 两个进程都在急停链路上且都还没编出来, 所以现象一样(0 个 pong,
# estop_path 恒 down), 错的注释不会被现象戳穿 -- 只会在有人去接这条线时
# 让他找错进程. 2026-09-03 终测查 estop_path=down 的成因时发现.
#: P2 的 1 Hz 健康度广播(11 S2.2 登记 p2_core 为唯一发布者, S5.1 定 schema).
#:
#: *** 本行原写 "health/factor" -- 那不是一个 key.
#: 11 S2.2 逐字: "health/factor 不是 key -- 该写法是 v0.1 rt/health/factor 的
#: 残留(S2.2.13 已登记为已迁移), 正确名为 cmd/motion/factor". 而
#: cmd/motion/factor 是发给 P1 的速度门约束(给机器执行), 与健康度广播(给人看)
#: 是两件事. 订一个不存在的 key 不会报任何错: Zenoh 照样建订阅, 只是永远收不
#: 到东西. 表现为 /api/health 恒 available:false 与云端 devices 恒空数组, 而
#: 后者在 v2.0 S4.2 下看起来完全合规("后端只发布实际发现的设备").
#: 2026-09-01 联调预演: 总线上实测只有 health/summary 在流, 1 Hz, items 是满的.
HEALTH_SUMMARY_TOPIC = "health/summary"  # -> /api/health + 云端 state/robot.devices
#: 开机自检报告(11 S5.2 BitReport). p2 的 bit/report.py 已实现但未接线, 所以
#: 这条今天没有发布者, /api/bit 恒 available:false.
#: NO 本行原注释写 "W8" -- W8 是 17 S6.7 预留给 PTZ 直控的上行编号(本轮不开放),
#: 与 BIT 无关. /api/bit 的出处是 17 S6.5 的只读 REST 表.
HEALTH_BIT_TOPIC = "health/bit"
FENCE_STALE_AFTER_MS = 10000             # P5F-2: cache silent this long -> degraded
# state/pose is 10 Hz (P1-1). If it goes silent this long the source is gone (RTK
# unplugged, P1 stopped) -> the snapshot must report the pose UNAVAILABLE, never
# keep showing the last fix as if it were live (3.1/3.2 fail-silent). Tight vs the
# geo window because pose is a real-time safety readout, not semi-static geometry.
POSE_STALE_AFTER_MS = 1500
EVENT_RING = 50                          # HMI keeps the most recent N events
DEFAULT_RTT_DEGRADE_MS = 200             # hmi.link_rtt_degrade_ms fallback (probe, not safety)
DEFAULT_DOWN_MISSES = 3                   # hmi.link_down_misses fallback (17 S6.3)


def _extract_active_tasks(payload: dict) -> list:
    """W7: flatten a state/task envelope into the flat task dicts the HMI plan
    panel reads (data_readers._plan needs task_id/state/current_step/total_steps
    at the top level, not nested).

    P3 publishes the 11 S4.4 TaskState: {schema, current, queue, suspended}. It
    used to publish {schema, active_task:{task_id, state, mono_ms}} instead -- a
    placeholder that carried neither route_id nor started_ts, which is how the
    cloud came to show a running task with no route and no start time.

    Flattening order is current, then queue, then suspended, so a consumer that
    takes the first entry gets the running task. The older shapes are still
    accepted: a list payload (HeartbeatState) has a live producer in progress.py,
    and returning [] for an unrecognised payload (not a fabricated card) keeps the
    panel at "no plan" -- the trap here is wrapping the whole envelope as one
    'plan', which is what the MVP did and made _plan read state/targets off the
    envelope and get None.

    Contract field names pass through UNCHANGED (current.type stays `type`): the
    v2.0 rename belongs in the projection that owns the v2.0 shape, not here,
    where the HMI reads the same dicts.
    """
    if not isinstance(payload, dict):
        return []
    # 11 S4.4 TaskState: three lists, current first.
    if ("current" in payload or "queue" in payload
            or "suspended" in payload):
        out = []
        cur = payload.get("current")
        if isinstance(cur, dict) and cur.get("task_id"):
            out.append(cur)
        for key in ("queue", "suspended"):
            for t in (payload.get(key) or ()):
                if isinstance(t, dict) and t.get("task_id"):
                    out.append(t)
        return out
    # Legacy single-object shape.
    at = payload.get("active_task")
    if isinstance(at, dict) and at.get("task_id"):
        return [at]
    # Forward-compat: a list of per-task heartbeat states.
    for key in ("active_tasks", "tasks"):
        lst = payload.get(key)
        if isinstance(lst, list):
            return [t for t in lst if isinstance(t, dict) and t.get("task_id")]
    return []


def _fence_snapshot(hmi_state: dict):
    """(fences_list_or_None, is_degraded) from the P5F-2 FenceCache.

    is_degraded is True when the cmd/fence stream has been silent past
    FENCE_STALE_AFTER_MS OR nothing has ever been staged -- in both cases the
    HMI must not present the (possibly empty) cache as authoritative: the map
    greys the layer and /api/fences returns 503 E_DEGRADED, never 200 [] (17
    S6.9 P5F-2). A fresh non-empty cache returns the fence list."""
    cache = hmi_state.get("fence_cache")
    if cache is None:
        return None, True
    fences, is_stale = cache.snapshot(_now_mono_ms(), FENCE_STALE_AFTER_MS)
    if is_stale or not fences:
        return None, True
    return list(fences), False


def _pose_if_fresh(pose, updated_ms: int, now_ms: int,
                   stale_after_ms: int = POSE_STALE_AFTER_MS):
    """The pose ONLY while state/pose is still fresh; None once it has been silent
    past stale_after_ms (RTK unplugged / P1 stopped). Returning None makes
    pose_group emit the no-fix shell, so the HMI greys the coord/ENU/RTK readouts
    and hides the robot arrow instead of freezing on the last fix as if it were
    live (3.1/3.2 fail-silent). enu_origin is NOT gated this way -- once adopted it
    is a fixed local anchor, so the geo layers keep rendering while pose greys."""
    if pose is None:
        return None
    return pose if (now_ms - updated_ms) <= stale_after_ms else None


def hmi_estop_cmd_id(boot: str, seq: int) -> str:
    """The cmd_id the HMI button puts on cmd/estop (11 S7.1 EstopCommand).

    boot + seq, not a bare counter: record-keeping aside, a counter restarts at
    0 on every p5 restart, so two boots would hand the SAME cmd_id to two
    different presses and quadruped's echo could not tell them apart.

    The h- prefix is the HMI namespace 11 S12.1.1 W1 already uses for req_id
    ("h-91c1"), the way the cloud path uses c- (cloud_wiring CLOUD_CMD_PREFIX).
    All four initiators in 11 S7.1 share ONE ack key, so the prefix is what lets
    a reader of cmd/estop/ack say which one an ack answers.
    """
    return "h-estop-%s-%d" % (boot, seq)


def hmi_estop_frame(cmd_id: str) -> dict:
    """The W1 cmd/estop frame for the HMI button (11 S7.1 + S12.1.1 W1).

    A module-level builder rather than a dict literal inside the closure, so the
    three audit fields can be asserted without standing up the web server.

    Why each field is here -- none of them is decoration:
      * cmd_id   11 S7.1's first field, the idempotency key. quadruped echoes it
                 verbatim and falls back to "anonymous" when the request carried
                 none (rt_bridge handle_estop). With four initiators on one ack
                 key, no cmd_id means every real ack is called "anonymous" and
                 nobody can say whose it is. 11 S7.1.1's 2026-09-27 note
                 registered the HMI button as the half still missing one.
      * reason   free text, lands in 11 S4.1 last_soft_estop.reason.
                 operator_hmi is the spelling S4.1 and S6.2 use in their own
                 examples, so the HMI and the cloud agree on one vocabulary.
      * src_role from 11 S7.1's five-value set; S12.1.1 W1 names this one
                 verbatim ("P5 补填 src_role: hmi"). HW-5 forbids P5 relabelling
                 an HMI action as cloud, which is why each publishing point
                 states its OWN role instead of one helper guessing it.
    last_soft_estop is {epoch, reason, src_role, age_ms} and quadruped stores
    what arrives, publishing null for what does not -- so an unfilled pair is
    not cosmetic: the object the HMI reads to say "3.2 s ago, by the HMI" stays
    permanently half empty, and an HMI stop is indistinguishable from a voice one.

    action is "stop": 11 S7.1 makes it the only legal value since v0.3.
    """
    return {"type": "estop", "action": "stop", "cmd_id": cmd_id,
            "reason": "operator_hmi", "src_role": "hmi"}


def _start_hmi(gen, hmi_cfg: dict, hmi_state: dict,
               site_timezone: Optional[str] = None):
    """Wire + start the HMI web server against what P5 can serve TODAY.

    Returns (server, thread) or (None, None) when HMI is not configured / cannot
    start -- an HMI failure must NEVER take down the voice loop, so every error
    here is logged and swallowed (the gateway must stay up so the operator can
    still see the voice side, 10 S3.3.7 W-1). What is wired now: state/task ->
    plan panel, state/link -> status/ESTOP arming, and the ESTOP button ->
    cmd/estop. What is NOT (fences/events/pose/mode) is recorded in NEXT.md and
    surfaces as available:false so the frontend greys those layers, never fakes.
    """
    from xbrain.p5_gateway.hmi.web_server import (
        HmiBindError,
        build_app,
        make_bound_sockets,
        start_in_thread,
    )

    bind = hmi_cfg.get("bind") if isinstance(hmi_cfg, dict) else None
    web = hmi_cfg.get("web") if isinstance(hmi_cfg, dict) else None
    if not bind or not web:
        _logger.warning("p5 HMI: no hmi.bind/hmi.web config; HMI not started")
        return None, None

    # ESTOP button -> W1 (17 S6.2). MVP sends the frame on cmd/estop; the
    # dedicated <=10 ms fast path (17 S6.4 / P-1) is a follow-up (NEXT.md).
    estop_pub = gen.declare_publisher(CMD_ESTOP_TOPIC)
    # Boot token + seq for the cmd_id, same construction as _comm_boot below:
    # the id must be unique across restarts, and a bare counter restarts at 0
    # so two boots would hand the same cmd_id to two different presses.
    _hmi_estop_boot = os.urandom(3).hex()
    _hmi_estop_seq = [0]

    def _estop_sender() -> None:
        # Three lines on purpose: the frame's content is hmi_estop_frame's job
        # (see there for why each audit field is required), so this stays a
        # publish and nothing a reader has to check for correctness sits on the
        # <=10 ms W1 path.
        _hmi_estop_seq[0] += 1
        cmd_id = hmi_estop_cmd_id(_hmi_estop_boot, _hmi_estop_seq[0])
        estop_pub.put(json.dumps(hmi_estop_frame(cmd_id)).encode("utf-8"))
        _logger.warning("p5 HMI ESTOP pressed -> cmd/estop published "
                        "(cmd_id=%s, src_role=hmi)", cmd_id)

    # 11 S12.1.1 W4: the browser's geo edits go out on cmd/geo with
    # origin="hmi" (stamped in hmi/uplink.py, never here -- CH-2 makes that one
    # field the whole permission boundary, so it has exactly one writer).
    # Publishers are declared once, not per frame: a per-frame declare leaks a
    # Zenoh resource for every click.
    uplink_pubs = {"cmd/geo": gen.declare_publisher(CMD_GEO_TOPIC),
                   # W2 goto + W7 task: both build a TaskCommand (S7.2) and
                   # both go out on this one key, which is why the map is keyed
                   # by KEY and not by uplink class.
                   "cmd/task": gen.declare_publisher(CMD_TASK_TOPIC),
                   # W3 exit_broadcast.
                   "cmd/mode": gen.declare_publisher(CMD_MODE_TOPIC)}

    def _send_uplink(key: str, payload: dict) -> None:
        pub = uplink_pubs.get(key)
        if pub is None:
            # An uplink class whose key was never declared. Refused loudly
            # rather than dropped: the browser is waiting on an ack, and a
            # silent drop is the one outcome it cannot distinguish from a slow
            # robot.
            raise KeyError("no publisher declared for uplink key %r" % (key,))
        pub.put(json.dumps(payload, ensure_ascii=False).encode("utf-8"))
        _logger.info("p5 HMI uplink -> %s (%s %s)", key,
                     payload.get("action"),
                     payload.get("geo_id") or payload.get("task_id") or "")

    class _Provider:
        """Reads P5's live shared state (updated by the sync callbacks) into the
        snapshot kwargs. Only the wired sources are non-None; the rest default to
        None so data_readers reports them unavailable (17 S6.10.4)."""

        def snapshot_inputs(self):
            # Copy references under the GIL; the callbacks REPLACE whole values,
            # never mutate in place, so a torn read is not possible here.
            fences, _degraded = _fence_snapshot(hmi_state)
            # routes/keypoints from the state/geo/objects cache (11 S7.10A) -- P5
            # never reads geo.db (S7843). None (never received / stale) -> the map
            # greys those layers, never a fabricated set.
            from xbrain.p5_gateway.geo.cache import geo_layers  # noqa: PLC0415
            _geo = hmi_state["geo_cache"].snapshot(_now_mono_ms())
            routes, waypoints = geo_layers(_geo)
            return {
                "tasks": hmi_state.get("tasks"),   # state/task (W-wired)
                "link": hmi_state.get("link"),     # state/link (W-wired)
                "fences": fences,                  # cmd/fence cache (W1)
                "mode": hmi_state.get("mode"),     # state/mode (W3)
                "events": hmi_state.get("events"),  # event/** ring (W2)
                # routes/keypoints now flow via state/geo/objects (11 S7.10A).
                # v1.5 PLAN A: geo geometry is WGS84 {lat,lon}, so it REQUIRES an
                # enu_origin to project -- until SITE calibration (W4 GATED-HW) that
                # origin is the first-fix demo fallback adopted below.
                "routes": routes, "waypoints": waypoints,
                # enu_origin PERSISTS once adopted (a fixed local anchor), so it is
                # NOT staled here -- only the live pose is.
                "enu_origin": hmi_state.get("enu_origin"),
                # pose flows: p1 assembles rt/gnss/heading -> state/pose. STALENESS
                # GATE: if state/pose has been silent past POSE_STALE_AFTER_MS the
                # source is gone (RTK unplugged, P1 stopped) -> pass None so
                # pose_group returns the no-fix shell and the HMI greys the readout,
                # never shows the last fix as if it were live (3.1/3.2 fail-silent).
                "pose": _pose_if_fresh(hmi_state.get("pose"),
                                       hmi_state.get("pose_updated_ms", 0),
                                       _now_mono_ms()),
                "clock": hmi_state.get("clock"),   # RTK time-sync (18-C G47)
                "health": hmi_state.get("health"),  # health/factor (W8)
                "teach": hmi_state.get("teach"),   # state/teach (S12A.5, read only)
            }

        def send_uplink(self, key, payload):
            _send_uplink(key, payload)

        def take_uplink_ack(self, req_id):
            # Keyed by the cmd_id P5 stamped (S12.1.1: "h-" + req_id). pop, so
            # the WS poll delivers each ack exactly once.
            return hmi_state.get("uplink_acks", {}).pop("h-" + req_id, None)

        def fence_degraded(self):
            # 503 E_DEGRADED (P5F-2) until a fresh cmd/fence has been staged;
            # never a 200 empty set.
            _fences, degraded = _fence_snapshot(hmi_state)
            return degraded

        def rest_inputs(self):
            # W8: sources for the 17 S6.5 REST endpoints NOT in the A..F snapshot.
            # health/bit are wired (P2 health/factor / health/bit); routes/docks/
            # metrics/approval have no P5 source yet -> None -> available:false.
            return {
                "health": hmi_state.get("health"),  # /api/health (W8-wired)
                "bit": hmi_state.get("bit"),        # /api/bit (W8-wired)
                "routes": None,      # /api/routes: geo.db gated (W8/geo src)
                "docks": None,       # /api/docks: geo.db gated
                "metrics": None,     # /api/metrics: telemetry aggregator gated
                "approval_pending": None,  # /api/approval/pending: L3 queue gated
            }

        def query_tasks(self, scope, limit, before):
            # GET /api/tasks -> P3's query/tasks queryable (11 S12.2A). P5 does
            # not read P3's task.db (plane isolation); it get()s over the gen
            # session P3 answers on. BLOCKING (iterates the reply channel), so
            # the route calls this via asyncio.to_thread -- never inline on the
            # FastAPI loop. No reply -> empty page (task_query_client), never a 500.
            from xbrain.p5_gateway.hmi.task_query_client import (  # noqa: PLC0415
                query_tasks as _query_tasks,
            )
            return _query_tasks(gen, scope=scope, limit=limit, before=before)

    try:
        socks = make_bound_sockets(bind)
        # `os` is the MODULE-level import (line 27). The redundant local
        # `import os` that used to sit here made `os` a local name for the
        # WHOLE function, so the boot token above -- textually earlier, but
        # executed before this line -- raised UnboundLocalError and p5 could
        # not start. Caught on the robot, not by a test: nothing exercised
        # _start_hmi, which is why one now does.
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        static_root = os.path.join(here, web.get("static_dir", "hmi/static"))
        app = build_app(web, _Provider(), _estop_sender, static_root,
                        site_timezone=site_timezone)
        server, thread = start_in_thread(app, socks)
        _logger.info("p5 HMI: serving on %s (static %s)",
                     [e for e in bind if e], static_root)
        return server, thread
    except HmiBindError as exc:
        # A bind failure (e.g. all-null or a wildcard) must not crash the voice
        # loop; the HMI just does not come up and the reason is logged.
        _logger.error("p5 HMI: bind refused (%s); HMI not started", exc)
        return None, None
    except Exception as exc:      # noqa: BLE001
        _logger.error("p5 HMI: failed to start (%s: %s); voice loop continues",
                     type(exc).__name__, exc)
        return None, None


def stamp_internal(data: dict, *, rid: str, boot: str, seq: int,
                   ts_sync: bool) -> bytes:
    """一段机内载荷 -> 11 S3.0 信封的 JSON 字节.

    S3.0 逐字"所有 Zenoh JSON 载荷共用此外层结构". p5 的 state/link 与
    event/{sev}/comm 在 2026-09-27 之前发的是裸 dict, 与 estop ping 那条是
    同一个缺陷: 消费方按 S3.0 解码会在必填字段那一步退出, 而 state/link 正
    是[云端判在线]的那条 key.

    *** 走共享编码器 xbrain.common.envelope.encode, NO 不手写八个键.
    手写过的地方(p1 的 stamp_envelope)把 ts/mono 戳成了毫秒整数, 而 S3.0 是
    秒 float64 -- 按 S3.0 计龄的消费方会把 5 s 前的消息读成 5000 s 前的.
    一个编码器意味着这类单位错只可能犯一次.

    *** 模块级函数而不是闭包内联: 闭包只能靠读源码断言, 而信封是逐字段的
    契约, 要能逐字段断言(八个键 / ts 与 mono 都是秒 / 无 boot 则 mono 一并
    省略). 接线那一层再把 rid/boot/seq/ts_sync 喂进来.

    boot 为空时 mono 一并省略(CLK-C4: 没有 boot 就没有 mono 的定义域),
    NO 不写一个裸 mono -- 那会让对端拿自己的 boot 域去解释别人的读数.
    """
    return json.dumps(encode(Envelope(
        v=1, rid=rid,
        # WALL-CLOCK-OK(align): S3.0 的信封 ts 只做跨机对齐 / 录包 / 延迟
        # 统计. 超时与年龄判定一律走 mono(CLK-C1).
        ts=time.time(),
        mono=time.monotonic() if boot else None,
        boot=boot or None,
        seq=seq, src="p5_gateway", ts_sync=ts_sync, data=data,
    )), ensure_ascii=False).encode("utf-8")


def _event_seg_index(segs: list) -> int:
    """Index of the 'event' segment, or -1. Works for BOTH the absolute contract
    key (xbrain/{rid}/event/{sev}/{cat}) and the relative dev-bus key
    (event/{sev}/{cat}) -- the two schemes differ by the xbrain/{rid} prefix, so
    locating 'event' rather than a fixed offset is the only robust parse."""
    try:
        return segs.index("event")
    except ValueError:
        return -1


def _event_sev_cat(key: str, d: dict):
    """(sev, cat) for one event message. The KEY is authoritative: every on-board
    producer publishes on event/{sev}/{cat} and puts NEITHER field in the body
    (cloud_wiring.publish_event says so in as many words). Reading them off the
    body instead yields (None, None) for every event ever produced -- which is
    exactly how the cloud relay came to be silently disabled, so the body is kept
    only as the fallback for a message that carries them and no usable key."""
    segs = key.split("/")
    ei = _event_seg_index(segs)
    sev = (segs[ei + 1] if 0 <= ei and len(segs) > ei + 1
           else (d.get("sev") or d.get("severity")))
    cat = (segs[ei + 2] if 0 <= ei and len(segs) > ei + 2
           else (d.get("cat") or d.get("category")))
    return sev, cat


def _normalise_event(key: str, d: dict) -> Optional[dict]:
    """Best-effort normalise an incoming event/{sev}/{cat} message to the
    record.db ev shape (the EventSubsystem persists it). sev/cat are the two
    segments after 'event' in the KEY (authoritative); rid is the segment before
    'event' when absolute, else XBRAIN_ROBOT_ID; eid/title/detail from the payload.
    Returns None if the essentials are missing -- the pipeline would drop it
    anyway, and a None here just skips the persist without touching the HMI ring.
    created_at/detected_at are wall-clock record fields (p5 is not in the
    monotonic-clock scan face; display/audit only, never used to order)."""
    segs = key.split("/")
    ei = _event_seg_index(segs)
    sev, cat = _event_sev_cat(key, d)
    rid = segs[ei - 1] if ei >= 1 else None            # absolute: .../{rid}/event
    rid = rid or os.environ.get("XBRAIN_ROBOT_ID") or d.get("rid")
    data = d.get("data") if isinstance(d.get("data"), dict) else d
    eid = data.get("eid") or data.get("event_id") or d.get("eid")
    if not (eid and rid and sev and cat):
        return None
    now = datetime.now(timezone.utc)  # WALL-CLOCK-OK(record): the event record ts, 11 S6.2; ages/timeouts elsewhere use the monotonic clock
    detail = data.get("detail")
    return {
        "eid": eid, "rid": rid, "sev": sev, "cat": cat,
        "title": data.get("title") or data.get("message") or "",
        "detail": detail if isinstance(detail, dict) else {},
        "src": d.get("src") or data.get("src") or "unknown",
        "ts": d.get("ts") or data.get("ts") or now.timestamp(),
        "ts_sync": d.get("ts_sync") or data.get("ts_sync") or 0,
        "detected_at": now.strftime("%Y-%m-%d %H:%M:%S"),
        "created_at": now.isoformat(),
        "dedup_key": data.get("dedup_key"),
        "dedup_window_s": data.get("dedup_window_s"),
        "task_id": data.get("task_id"), "trace_id": data.get("trace_id"),
    }


def run_voice_loop_wiring(stop_flag: dict,
                            heartbeat_period_s: float = 1.0,
                            hmi_cfg: Optional[dict] = None,
                            site_timezone: Optional[str] = None,
                            record_db_path: Optional[str] = None) -> int:
    """Block until stop_flag['stop'] truthy. Returns 0 on clean shutdown.

    hmi_cfg is the resolved `hmi` config subtree (bind + web) or None. When
    present the HMI web server starts in a background thread (17 S6.10); when
    absent or malformed the voice loop runs exactly as before -- the HMI is
    strictly additive and never a precondition for the voice side.

    site_timezone (common.timezone) is forwarded to the HMI so its footer clock
    shows site-local time; None leaves the frontend on the browser zone.
    """
    from xbrain.common.runtime.session_ctx import open_planes
    from xbrain.p5_gateway.fence.cache import FenceCache
    from xbrain.p5_gateway.geo.cache import GeoCache
    from xbrain.p5_gateway.hmi.estop_probe import EstopProbe, build_ping_data, pong_seq

    # W5: estop-path probe (17 S6.3). Thresholds ride on the hmi subtree
    # (link_rtt_degrade_ms / link_down_misses); the fallbacks are probe tuning,
    # NOT safety params, so a default is allowed here (3.1 governs common.spec/
    # safety, not HMI liveness thresholds). Starts "down": until the quadruped
    # estop channel actually replies the button greys honestly, never armed on
    # faith (the endpoint is GATED-HW today, so "down" is the truthful state).
    _probe_cfg = hmi_cfg if isinstance(hmi_cfg, dict) else {}
    estop_probe = EstopProbe(
        _probe_cfg.get("link_rtt_degrade_ms", DEFAULT_RTT_DEGRADE_MS),
        _probe_cfg.get("link_down_misses", DEFAULT_DOWN_MISSES),
    )

    # Pre-bound so the event callback below can read it safely. _on_event is
    # registered (line ~750) BEFORE the bridge is built (line ~775), so a
    # cloud event arriving in that window would hit an unbound closure cell
    # and raise NameError inside a Zenoh Rust-thread callback -- where the
    # exception is swallowed and the event silently vanishes. Binding None up
    # front closes the window; the callback's `is not None` guard does the rest.
    cloud_bridge = None
    cloud_projector = None

    # Shared state the HMI provider reads. The sync callbacks below REPLACE whole
    # values (never mutate in place) so the web thread's reads stay consistent
    # under the GIL without a lock. fence_cache follows the same rule: on_update
    # swaps the tuple wholesale (P5F-1), so a concurrent snapshot is consistent.
    hmi_state: dict = {
        "tasks": None,               # state/task  -> plan panel
        "link": None,                # state/link  -> status + ESTOP arming
        "mode": None,                # state/mode  -> footer mode (W3)
        "pose": None,                # state/pose  -> coord panel + heading dial + RTK
        "stream_id": None,           # B 模式会话 ID (p2 分配, 经 cmd/mode/ack)
        "audio": None,               # 11 S8.10 AudioState (p2 发)
        "audio_updated_ms": 0,       # last state/audio arrival (mono)
        "pose_updated_ms": 0,        # last state/pose arrival (mono) -> staleness gate
        "clock": None,               # state/clock -> RTK time-sync indicator
        "robot": None,               # state/robot -> hes / faults[] / conn (CR-4)
        "power": None,               # state/power -> soc_pct / batteries (CR-5)
        "events": [],                # event/**    -> event stream ring (W2)
        "health": None,              # health/factor -> /api/health (W8)
        "bit": None,                 # health/bit  -> /api/bit (W8)
        "enu_origin": None,          # localisation origin (gated, W4)
        # True 表示当前 enu_origin 来自 common.geo.enu_origin(经 cmd/fence 送达),
        # False 表示来自首次定位兜底. 两者优先级不同, 见 _on_cmd_fence.
        "enu_origin_authoritative": False,
        #: query/tasks 拉来的任务全量(v2.0 snapshot 的 queue/suspended 用它).
        #: 与 hmi_state["tasks"](只有 active_task, 供 HMI plan 面板)是两份,
        #: 因为两者的形状与刷新节律都不同.
        "cloud_tasks": [],
        "fence_cache": FenceCache(),  # cmd/fence   -> map fences (W1)
        "geo_cache": GeoCache(),      # state/geo/objects -> routes/keypoints (11 S7.10A)
    }

    _logger.info("p5 wiring: opening GEN session")
    with open_planes(("gen",)) as gen:
        link_pub = gen.declare_publisher(STATE_LINK_TOPIC)
        # W5: probe ping out, pong in (11 CR-2/CR-3). The ping rides the estop
        # channel path (via chassis_relay) so a dead estop LINK (not merely a
        # dead app) surfaces as estop_path "down", not a false "ok".
        estop_ping_pub = gen.declare_publisher(PROBE_ESTOP_PING_TOPIC)

        # Event subsystem (17 S3: record.db persist + delivery mark + backfill).
        # ADDITIVE: record_db_path None or store-open failure -> disabled, every
        # submit a no-op, the HMI ring below still works, voice loop untouched.
        event_subsystem = None
        replay_pub_normal = replay_pub_alarm = None
        recon_req_pub = None
        if record_db_path:
            from xbrain.p5_gateway.runtime.event_subsystem import EventSubsystem
            event_subsystem = EventSubsystem(
                os.environ.get("XBRAIN_ROBOT_ID", "unknown"),
                record_db_path, record_db_path + ".degrade.jsonl",
                now_iso=lambda: datetime.now(timezone.utc).isoformat(),  # WALL-CLOCK-OK(record): record.db event timestamp; the subsystem takes now_mono separately for ages
                now_mono=time.monotonic)
            if event_subsystem.start():
                _logger.info("p5 wiring: event subsystem ON (record.db=%s)",
                             record_db_path)
                # Backfill replay publisher. The bus uses RELATIVE keys (no rid
                # prefix), so publish to event/replay/{channel}; route by the
                # message's channel field (the runner hands an absolute key we
                # ignore). R-2: p5/HMI never SUBSCRIBE event/replay/** (self-loop).
                replay_pub_normal = gen.declare_publisher("event/replay/normal")
                replay_pub_alarm = gen.declare_publisher("event/replay/alarm")

                def _put_replay(_key, data):
                    pub = (replay_pub_alarm if data.get("channel") == "alarm"
                           else replay_pub_normal)
                    pub.put(json.dumps(data).encode("utf-8"))

                event_subsystem.set_replay_publisher(_put_replay)
                # recon/req publisher (17 S3Y.3): P5 periodically asks the cloud
                # which ch_seqs it is missing. Relative key (dev bus); the cloud
                # answers on event/recon/rsp (handled below). R-2 covers 'recon'.
                recon_req_pub = gen.declare_publisher("event/recon/req")
                event_subsystem.set_recon_req_publisher(
                    lambda _k, d: recon_req_pub.put(json.dumps(d).encode("utf-8")))

        # Cloud-link state machine (11 S4.6). P5 is the sole authority for cloud_link
        # / level / disconnected_s / link_epoch (LNK-6) -- the one judge for return-
        # to-base and the reconnect signal for the event backfill. Thresholds are the
        # S4.6.2 values, injected as interim constants (config keys land later; rtb_s
        # = None keeps L3/auto-RTB disabled while TSK-21 is undefined -- the fail-safe
        # per 3.1). It also subsumes the old LinkReconnectDetector: its snapshot's
        # .reconnected edge drives trigger_backfill.
        from xbrain.p5_gateway.uplink.link_state import (
            LinkStateMachine,
            LinkThresholds,
        )
        # rtb_s = 1800 s (30 min): the L2->L3 return-to-base threshold. User decision
        # 2026-08-17 taking the 11 S4.6.2 / 15 S11.2 suggested value; still an INTERIM
        # value pending the operator's final sign-off (U-05 / TSK-21), and all four
        # thresholds should migrate to a config key together (a follow-up), so this
        # is not a frozen safety constant -- it is a recorded, changeable decision.
        link_thresholds = LinkThresholds(
            degraded_s=5.0, down_s=20.0, rtb_s=1800.0, stable_s=10.0)
        link_state = LinkStateMachine(
            link_thresholds, gw_start_mono=time.monotonic())
        # 11 S4.6.8 comm events: P5 owns the link state, so it produces one
        # event/{sev}/comm per level transition. Track the previous level +
        # disconnected_s to diff each heartbeat. eid is boot-unique (same F8
        # rationale: link_epoch resets on restart, so a raw seq would collide).
        from xbrain.p5_gateway.event.comm_events import comm_event_for_level
        _prev_link_level: Optional[int] = None
        _prev_disc_s = 0.0
        _comm_seq = [0]
        _comm_boot = os.urandom(3).hex()

        # 11 S9.8.4 ChassisFault -> 11 S6.1 Event (user ruling 2026-09-27; the
        # UM-4 precedent is verbatim "事件由 P1 依 state/robot 派生 ... 不由
        # quadruped 发" -- the CONSUMER derives). Until this existed the
        # event/fault/chassis payload reached _normalise_event with no eid and
        # was dropped as missing_field:eid, i.e. with a real chassis attached
        # every fault report died here. Edge-triggered inside the deriver, which
        # is what keeps a 2 Hz level snapshot from becoming an event flood and
        # what makes the intentional P1-20 double delivery cost nothing.
        from xbrain.p5_gateway.event.chassis_events import ChassisFaultDeriver
        _chs_seq = [0]
        # Boot-unique token, same reason as _comm_boot / p2's _dev_eid_boot:
        # record.db outlives the process, a bare seq restarts at 0, and a
        # repeated eid makes the DAO degrade the row to JSONL instead of
        # persisting it (found in the SW-12 audit).
        _chs_boot = os.urandom(3).hex()

        def _chs_eid(code: str, cleared: bool) -> str:
            _chs_seq[0] += 1
            # The code goes in the eid so a human reading record.db can see
            # which fault a row is about without opening detail.
            return "chs-%s-%s-%s-%d" % (
                code.replace(":", "_"), "clr" if cleared else "set",
                _chs_boot, _chs_seq[0])

        chassis_deriver = ChassisFaultDeriver(
            rid=os.environ.get("XBRAIN_ROBOT_ID", "unknown"),
            eid_gen=_chs_eid)

        speak_acks_seen = 0
        state_task_updates = 0
        #: 11 S8.5 的端到端关联号, 放在信封的 data 里(见下方发布处的长注).
        probe_seq = 0
        #: 探活 ping 自己的信封 seq. 与上面那个是两个数, 不许合并 --
        #: 合并就等于把一个 RT-C3.e 允许转发者改写的字段当成关联号用.
        probe_env_seq = 0
        #: 信封的 rid / boot 读一次就够: read_local_boot_id 是文件 IO,
        #: 放进 1 Hz 心跳里是白白的系统调用.
        probe_rid = os.environ.get("XBRAIN_ROBOT_ID", "")
        probe_boot = read_local_boot_id()
        if not probe_rid:
            # 11 S3.0 的 rid 必填, 而 quadruped 对 rid 不符的 ping 只回一条
            # seq=0 的 pong(不静默丢, 13 F-15). 现象是 estop_path 恒 down 而
            # 总线上 pong 照流 -- 不说一声的话, 这与"链路真断"分不清.
            _logger.warning(
                "p5 estop probe: XBRAIN_ROBOT_ID unset, ping envelope carries "
                "an empty rid; quadruped will refuse it and estop_path stays down")

        # 11 S3.0 信封的 seq: 按 key 各自递增(进程重启从 0 起). 一个全局计数
        # 器会让每条 key 在消费方看来一直在跳号, 而 seq 正是 U18 补发游标与
        # 缺口判定的依据. state/link 只有一条 key 所以是标量; comm 事件的 key
        # 带 {sev} 段, 所以按实际 key 分桶.
        _link_env_seq = [0]
        _comm_env_seq: dict = {}

        def _stamp(data: dict, seq_slot) -> bytes:
            """把一段机内载荷包进 11 S3.0 的信封并序列化.

            seq_slot 是一个单元素 list(按 key 各自递增, 见上); 其余四项从本
            闭包取: rid/boot 开机读一次, ts_sync 抄 P1-13 镜像来的
            ClockStatus.sync(CLK-A2, NO 本进程不自行判定授时状态).
            """
            seq_slot[0] += 1
            return stamp_internal(
                data, rid=probe_rid, boot=probe_boot, seq=seq_slot[0],
                ts_sync=bool(
                    (hmi_state.get("clock") or {}).get("sync") is True))


        def _on_estop_pong(sample) -> None:
            # W5: 一条 pong. 按 data.seq 匹配, 迟到的旧 pong 对不上号会被
            # EstopProbe 忽略(不得让晚到的应答掩盖当前的中断).
            #
            # *** 关联号取 data.seq, NO 不取信封 seq(11 S8.5, 2026-09-27 裁决).
            # chassis_relay 在两条腿上都转发本探活(CR-2/CR-3), 而 RT-C3.e
            # [要求]它用自己的计数改写信封 seq. 拿信封 seq 匹配 = 拿 relay 的
            # 计数去对我们自己的计数, 永远对不上, RTT(T-23/T-24)就永远测不出来
            # -- 而 pong 以 1 Hz 照流, 两侧进程都健康, 只有按钮一直是灰的.
            #
            # *** 拿不到 data.seq 时 NO 不回落到顶层 seq.
            # 顶层 seq 是 relay 的计数器, 1 Hz 自增, 与 probe_seq 同频同量级 --
            # 它迟早会[偶然相等], 那一拍就是一次假匹配, 把一条实际对不上的
            # 应答记成一次成功 RTT. 一个偶尔为真的匹配比永远不匹配更坏:
            # 后者是 down(fail-safe), 前者是间歇性 ok(fail-silent).
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = {}
            seq = pong_seq(d)
            if seq is not None:
                estop_probe.on_pong(seq, _now_mono_ms())

        def _on_speak_ack(sample) -> None:
            nonlocal speak_acks_seen
            speak_acks_seen += 1
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = {}
            _logger.info("p5 obs speak/ack #%d: %s",
                         speak_acks_seen,
                         json.dumps(d, ensure_ascii=False))

        def _on_state_task(sample) -> None:
            nonlocal state_task_updates
            state_task_updates += 1
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = {}
            # W7: feed the HMI plan panel. Flatten the state/task envelope into
            # the flat task dicts _plan reads (extract active_task, NOT the whole
            # {schema, active_task} envelope). None when nothing usable so the
            # panel stays "no plan" rather than showing an empty card.
            tasks = _extract_active_tasks(d)
            hmi_state["tasks"] = tasks or None
            # Cloud result face (v2.0 R12.4) is NOT fed from here any more --
            # see _on_event. 11 S4.4 TaskState lists only non-terminal tasks, so
            # a finishing task leaves the broadcast rather than appearing in it
            # as done/failed/cancelled, and the transition rule needs to SEE the
            # terminal state. Feeding the observer from here would silently stop
            # producing results the moment the broadcast became contract-shaped.
            # Live tasks are still observed so the tracker knows they were
            # non-terminal before they go (the rule is a transition, not a level).
            if cloud_projector is not None:
                for _t in tasks or ():
                    cloud_projector.observe_task(_t)
            _logger.info("p5 obs state/task update #%d: %s",
                         state_task_updates,
                         json.dumps(d, ensure_ascii=False))

        def _on_cmd_fence(sample) -> None:
            # W1: cache the staged FenceSet geometry (P5F-1 overwrite). Each
            # polygon keeps name + vertices (WGS84 lat/lon) + role for the map;
            # role 'warning' (old name 'zone', 11 S9A.1A) renders as an alarm
            # region, else a keep-in boundary. The role passes through verbatim.
            # (The map can place them once an enu_origin exists -- gated, W4.)
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = {}
            polys = d.get("polygons")
            if polys:
                # poly_id 也留下: v2.0 S2 的 manifest objects[] 要 geo_id,
                # 而围栏只在这条 key 上出现(state/geo/objects 不含 fences).
                # 不留的话 manifest 里永远没有 alarm_region, 而 v2.0 S2.4 的
                # 报警规则要用 region_ids 引用它们.
                fences = [{"geo_id": p.get("poly_id"),
                           "name": p.get("name"),
                           "vertices": p.get("vertices"),
                           "role": p.get("role")} for p in polys]
                hmi_state["fence_cache"].on_update(fences, _now_mono_ms())
            # *** 权威 enu_origin 走这条路进来.
            # 11 S9A.2 把 enu_origin 定为 FenceSet 的必填字段, 值来自
            # common.geo.enu_origin(L4 sites/{site_id}.yaml, 11 第 7815 行逐字
            # "各进程不得各自选原点"). p3 从解析产物读出后随 FenceSet 发出,
            # 这里是它到达 HMI 的入口.
            #
            # 它[覆盖]下面 _on_state_pose 的首次定位兜底, 而不是被后者挡住:
            # 兜底原点跟着"机器人第一次定位在哪"跑, 换个地方开机就换一个原点,
            # 只是 W4 场地标定落地前的权宜(见 _on_state_pose 的注释). 权威值
            # 一旦到达就该顶掉它 -- 否则先开机后配围栏的顺序下, 系统会一直用
            # 那个漂移的兜底原点, 而且没有任何迹象表明用错了.
            eo = d.get("enu_origin")
            if isinstance(eo, dict) and eo.get("lat") is not None \
                    and eo.get("lon") is not None:
                if not hmi_state.get("enu_origin_authoritative"):
                    _logger.info(
                        "p5 HMI: adopted authoritative enu_origin from "
                        "cmd/fence (lat=%s lon=%s)", eo.get("lat"), eo.get("lon"))
                hmi_state["enu_origin"] = {"lat": eo.get("lat"),
                                           "lon": eo.get("lon"),
                                           "alt": eo.get("alt")}
                hmi_state["enu_origin_authoritative"] = True

        def _on_geo_objects(sample) -> None:
            # 11 S7.10A: P3 broadcasts routes/keypoints/docks geometry; cache the
            # whole payload (RUST thread -> decode + store only). The snapshot
            # reshapes it into the routes/waypoints map layers (geo_layers). P5
            # never reads geo.db (S7843) -- this broadcast IS the data.
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            hmi_state["geo_cache"].on_update(d, _now_mono_ms())

        def _on_state_mode(sample) -> None:
            # W3: last usage mode for the footer (P2 publishes state/mode 10 Hz).
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = {}
            # 11 S4.3 ModeState 的字段是 voice_mode. 原来读的是 d["mode"] --
            # 那个拼法在 2026-08-21 之前无所谓, 因为 state/mode 根本没有发布者
            # (p2_core 的模式面当天才接线); 两边就着一个不存在于契约的名字对上
            # 了, 谁也不会发现. 现按 S4.3 对齐.
            if d.get("voice_mode"):
                hmi_state["mode"] = d["voice_mode"]
            # 11 S4.3(v1.1, 99 U84): broadcast 时必填, 其余模式为 null.
            # v2.0 S4.3 要求云端的 state/mode 带它 -- 没有它, mode_payload
            # 会抛 ProjectionError 并让整条 key 发不出去(实测 10 Hz 刷错误).
            # *** 读[状态面]而不是 cmd/mode/ack.
            # ack 是事件, 掉一条就再也拿不到; state/mode 是可重复读的,
            # 网关中途重启也能立刻补上. 且 p2 那边两面同源(见 mode_wiring
            # 的 _stream_id), 不存在两个来源打架的问题.
            if "stream_id" in d:
                hmi_state["stream_id"] = d["stream_id"]

        def _on_state_audio(sample) -> None:
            # 11 S8.10 AudioState. p2_core 是契约指定的发布者.
            # *** 整包存下, NO 不在这里摊平.
            # 云端要的是 v2.0 S4.4 的扁平形状(speaker_state/microphone_state/
            # playing/recording), 那个映射在 cloud_state._audio 里做 -- 回调跑
            # 在 Rust 线程池上(CLAUDE.md 4.2), 只做一次 dict 赋值(原子), 不做
            # 闭集校验: 校验会抛, 而在 Rust 线程上抛出去没人接得住.
            #
            # *** 存的是信封里的 data, 不是整条报文(2026-09-28, p2 给
            # state/audio 补上 11 S3.0 信封的同一批). 裸形态照样接受, 理由与
            # _on_health 同一条: 桩发布者与旧版本 p2 都不带信封.
            # ! 这一层 NO 不能省, 而且这条 key 漏解包的后果比 health 更坏:
            #   下游 cloud_state._audio 读 a.get("speaker") 取不到 -> holder
            #   为 None -> speaking = (holder != "none") 判成 True ->
            #   Qt 上"喇叭一直在响"而机器人其实静默. 一个按信封发的发布者
            #   会让这一格变成恒真, 与"没解包"在报文上不可区分.
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            if isinstance(d, dict):
                inner = d.get("data")
                if isinstance(inner, dict):
                    d = inner
                hmi_state["audio"] = d
                # 陈旧度判据的时基. p2 的下限是 1 Hz, 所以"很久没来"是可判的.
                # 没有这个戳的话, p2 挂掉之后 hmi_state["audio"] 会一直停在
                # 最后一帧, 云端看到的是一个[冻结但看起来正常]的音频状态.
                hmi_state["audio_updated_ms"] = _now_mono_ms()

        def _on_state_pose(sample) -> None:
            # P1-1: p1 publishes state/pose (3.0 envelope) 10 Hz; the HMI reads the
            # data part for the coord panel + heading dial + RTK status. Callback
            # stores data only (dict assign is atomic; no work on the Rust thread).
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            hmi_state["pose"] = d.get("data")
            hmi_state["pose_updated_ms"] = _now_mono_ms()   # for the staleness gate
            # DEMO / W4-pending (user 2026-08-18): the ENU E/N readout and the map
            # robot need an enu_origin to project lat/lon into local metres. The
            # authoritative origin is common.geo.enu_origin, set by SITE calibration
            # in configs/sites/ (7.8.4) -- it is null until then, so ENU shows "--".
            # Until that calibration exists, adopt the FIRST valid GPS fix as the
            # local origin so the readout works. NOT a substitute for real
            # calibration: this origin follows wherever the robot first fixed.
            # NOTE: the RAW state/pose data has no "available" key (pose_group adds
            # it downstream); a real fix is just lat/lon present, so key off those.
            # NO 权威原点已到达时不再兜底 -- 否则一个跟着首次定位跑的原点会
            # 把配置里那个测绘出来的锚点顶掉(优先级正好反了).
            if hmi_state.get("enu_origin") is None \
                    and not hmi_state.get("enu_origin_authoritative"):
                p = hmi_state["pose"]
                if isinstance(p, dict) and p.get("lat") is not None and p.get("lon") is not None:
                    hmi_state["enu_origin"] = {
                        "lat": p["lat"], "lon": p["lon"], "alt": p.get("alt")}

        def _on_state_clock(sample) -> None:
            # P1-13: clock sync mirror -> RTK time-sync indicator (18-C G47).
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            hmi_state["clock"] = d.get("data")

        def _on_state_robot(sample) -> None:
            # CR-4: chassis_relay 把 rt/chassis/state 转上来. 取 data 那一层
            # (relay 按 RT-C3.e 重建过信封, 内容原样在 data 里).
            #
            # *** 解析失败时 NO 不清缓存.
            # 一条坏报文不是"底盘没了"的证据; 清掉会让 robot_state 在坏报文
            # 那一拍跳回 idle, 而 idle 是[比真相更乐观]的那个方向 --
            # 急停接合着却报 idle, 正是 3.2 要挡的 fail-silent.
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            body = d.get("data")
            if isinstance(body, dict):
                hmi_state["robot"] = body

        def _on_state_power(sample) -> None:
            # CR-5. 同上: 坏报文不清缓存.
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            body = d.get("data")
            if isinstance(body, dict):
                hmi_state["power"] = body

        def _relay_to_cloud(sev, cat, body: dict) -> None:
            """The ONE cloud_bridge.publish_event call site in this module.

            Both event producers here route through it -- the generic event/**
            body and the Events derived from a ChassisFault. A second call site
            is what would double every event on the cloud face, which is why
            test_cloud_bridge counts them in the AST and requires exactly one.

            Best-effort by design: a relay failure must never touch the HMI ring
            or the record.db write, both of which are the caller's own steps.
            """
            if cloud_bridge is None or not sev or not cat:
                return
            try:
                cloud_bridge.publish_event(sev, cat, body)
            except Exception:      # noqa: BLE001
                _logger.exception("p5 cloud event relay failed")

        def _ingest_derived_event(ev: dict) -> None:
            """One already-assembled 11 S6.1 Event -> HMI ring + cloud + record.

            The same three destinations _on_event gives a normal event, in the
            same order. Factored out because the ChassisFault path arrives as a
            SNAPSHOT and produces zero or more Events, so it cannot reuse the
            one-message-one-event body above it.
            """
            hmi_state["events"] = (hmi_state["events"] + [{
                "eid": ev["eid"], "title": ev["title"], "sev": ev["sev"],
                "cat": ev["cat"], "ts": ev["ts"], "pos": None,
            }])[-EVENT_RING:]
            _relay_to_cloud(ev["sev"], ev["cat"], ev)
            if event_subsystem is not None and event_subsystem.enabled:
                link = hmi_state.get("link") or {}
                # Same S3.5.1 delivery signal as the generic path: only an
                # authoritative cloud_link == up marks a need_ack=0 event
                # delivered; anything else queues it for backfill.
                event_subsystem.submit_event(
                    ev, link.get("cloud_link") == "up")

        def _on_chassis_fault(key: str, d: dict) -> None:
            """event/fault/chassis carries a ChassisFault, NOT an Event.

            11 S9.8.4's payload has no eid/cat/sev/title, so the generic path
            would drop it at the pipeline's first step. Derive here instead
            (user ruling; UM-4 is the same shape one key over) and feed each
            derived Event through the normal three destinations.

            *** Handled INSIDE _on_event rather than by a second subscriber.
            The wiring already subscribes event/** and that expression matches
            event/fault/chassis; declaring a dedicated subscriber for the same
            key would make BOTH callbacks fire and double every derived event.
            """
            try:
                derived = chassis_deriver.observe(d, now_wall=time.time())  # WALL-CLOCK-OK(record): the event record stamp, 11 S6.2; no age or timeout is computed from it
            except Exception:      # noqa: BLE001
                # A malformed snapshot must not kill the subscriber thread; the
                # deriver already swallows per-entry defects, so reaching here
                # means something structural. Counted by the logger, not lost.
                _logger.exception("p5 chassis fault derive failed on %s", key)
                return
            for ev in derived:
                _ingest_derived_event(ev)

        def _on_event(sample) -> None:
            # R-2: "event/**" also matches our OWN event/replay/** (backfill),
            # event/ack, and event/recon/{req,rsp} -- all handled by dedicated
            # subscribers, none are live events. Processing them here would
            # re-persist / self-loop; skip when the segment right after 'event' is
            # one of these (robust for absolute + relative keys).
            try:
                key = str(sample.key_expr)
            except Exception:      # noqa: BLE001
                key = ""
            _segs = key.split("/")
            _ei = _event_seg_index(_segs)
            if (0 <= _ei < len(_segs) - 1
                    and _segs[_ei + 1] in ("replay", "ack", "recon")):
                return
            # W2: keep the most recent EVENT_RING events for the stream + map
            # dots. REPLACE the whole list (never append in place) so the web
            # thread's read is consistent under the GIL.
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = {}
            if not d:
                return
            # sev/cat off the KEY (see _event_sev_cat): reading them from the
            # body left both None for every event, which blanked them in the HMI
            # stream and, because the relay below gates on them, meant not one
            # event ever reached the cloud.
            _sev, _cat = _event_sev_cat(key, d)
            # The ONE key on this stream whose payload is not an Event: CR-9
            # forwards the raw ChassisFault onto event/fault/chassis (11 S9.8.4).
            # Routed before the generic body, which would otherwise put an
            # eid-less row in the HMI ring and relay a payload the cloud's S5.1
            # projector rejects for the same missing eid.
            if _cat == "chassis" and _sev == "fault":
                _on_chassis_fault(key, d)
                return
            # *** The Event body, which is NOT always the message.
            # Producers differ: p2 / p1 / p3 publish the bare Event, while p5's
            # own event/{sev}/comm rides inside the 11 S3.0 envelope (and every
            # producer eventually will -- S3.0 verbatim: all Zenoh JSON payloads
            # share that outer structure). Reading the top level unconditionally
            # takes eid / title / detail off the ENVELOPE, where they are not:
            # the HMI ring gets a nameless row and, worse, the cloud relay's
            # event_payload raises "event missing eid (v2.0 S5.1)" and the event
            # never reaches the cloud. Measured on the ORIN 2026-09-27, one
            # traceback per comm event, the same day the envelope was added.
            # _normalise_event below has always unwrapped the same way.
            _body = d["data"] if isinstance(d.get("data"), dict) else d
            ev = {
                "eid": _body.get("eid") or _body.get("event_id"),
                "title": _body.get("title") or _body.get("message"),
                "sev": _sev,
                "cat": _cat,
                # Body ts first, envelope ts as the fallback: for a bare
                # producer they are the same field, and for an enveloped one the
                # body's is the event's own stamp (11 S6.2) while the envelope's
                # is the publish instant.
                "ts": _body.get("ts") or d.get("ts"),
                "pos": _body.get("pos"),   # None until pose stamps it (W4)
            }
            hmi_state["events"] = (hmi_state["events"] + [ev])[-EVENT_RING:]
            # Cloud result face (v2.0 R12.4): the TERMINAL half of the task
            # transition arrives here, not on state/task. 11 S4.4 TaskState lists
            # only non-terminal tasks, so a finishing task drops OUT of that
            # broadcast; 11 S6.2 makes the task event stream the thing that
            # carries "完成 / 失败 / 取消", and p3 emits one per scheduler
            # transition with detail {kind, task_id, state}. Observed on the
            # event, never on the 10 Hz tick: a task that runs and completes
            # inside one tick would only ever be sampled terminal, and the
            # transition rule would never fire.
            if cloud_projector is not None and _cat == "task":
                _det = _body.get("detail")
                if isinstance(_det, dict) and _det.get("task_id"):
                    # 整个 detail 交出去, NO 不只挑 task_id/state.
                    # p3 在终态事件里带上了 task_type / route_id / started_ts /
                    # ended_ts / duration_sec / reason -- v2.0 S3.3 的 result
                    # 要这些, 而它们没有第二条通路: p5 不读 task.db(平面隔离),
                    # 且 11 S4.4 的 TaskState 只列非终态任务, 任务一终结就从
                    # 广播里消失. 只挑两个字段的话, result 里其余全是 null.
                    cloud_projector.observe_task(dict(_det))
            # Cloud relay (v2.0 S2: the cloud event key is
            # xbrain/{rid}/event/{sev}/{cat}, NOT the bare key producers use).
            # Two reasons this must be a relay and not a producer-side key
            # change -- see CLOUD_EVENT in cloud_wiring.py. The call itself sits
            # in _relay_to_cloud so this module keeps exactly one publish_event
            # call site with the chassis path sharing it.
            _relay_to_cloud(ev["sev"], ev["cat"], _body)
            # Persist + deliver via the event subsystem (fire-and-forget, no-op
            # when disabled). A malformed event normalises to None and is skipped;
            # the HMI ring above is unaffected either way.
            if event_subsystem is not None and event_subsystem.enabled:
                full = _normalise_event(key, d)
                if full is not None:
                    link = hmi_state.get("link") or {}
                    # Cloud-link signal for the S3.5.1 delivery judgment -- now the
                    # authoritative S4.6 cloud_link (up only after the LNK-3
                    # hysteresis). In the dev loop the cloud is never heard from, so
                    # this stays False and events queue for backfill rather than
                    # being falsely marked delivered (DEGRADED counts as not-sent).
                    connected = link.get("cloud_link") == "up"
                    event_subsystem.submit_event(full, connected)

        def _on_event_ack(sample) -> None:
            # 17 S3.5.1: a cloud ack marks that eid delivered (out of backfill).
            if event_subsystem is None or not event_subsystem.enabled:
                return
            try:
                a = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            # A parsed ack IS cloud contact -> feed the reconnect detector.
            link_state.on_cloud_rx(time.monotonic())
            eid = a.get("eid") or a.get("event_id")
            # 11 S8.4 result closed set is {ok, duplicate}; a missing result is a
            # malformed ack -> "" (not in the set) leaves the event delivered=0 for
            # re-send. Do NOT default to a fake "accepted" (audit F9: that was the
            # command-Ack model, and it is not an EventAck value).
            result = a.get("result") or ""
            if eid:
                event_subsystem.submit_ack(eid, result)

        def _on_recon_rsp(sample) -> None:
            # 17 S3Y.3: the cloud's answer to our recon/req -> compute + resend the
            # gap. A rsp is also cloud contact, so it feeds the reconnect detector.
            if event_subsystem is None or not event_subsystem.enabled:
                return
            try:
                r = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            link_state.on_cloud_rx(time.monotonic())
            event_subsystem.submit_recon_rsp(r)

        def _on_health(sample) -> None:
            # Relay P2's latest health/summary to /api/health. P5 forwards the
            # authoritative payload unchanged (G-2 same-source), REPLACING the
            # whole value so the web thread's read stays consistent under the GIL.
            #
            # *** 存的是信封里的 data, 不是整条报文(2026-09-28, p2 给
            # health/summary 补上 11 S3.0 信封的同一批). 裸形态照样接受, 理由
            # 与 p2/p3 的 _make_state_sink 同一条: 桩发布者与旧版本 p2 都不带
            # 信封, 两种形态各写一条代码路径必然分叉.
            # ! 这一层 NO 不能省: 下游三处(REST /api/health 透传 /
            #   cloud_state 的 _devices_from_health / state_projection)全部按
            #   HealthSummary 的顶层字段(schema/items/overall)取值. 只改发布侧
            #   而消费侧仍读顶层, 正是 2026-09-27 修 event/*/comm 时引入过的
            #   那次回归(5d6981a) -- 现象是 /api/health 里 items 整块消失,
            #   而两侧进程都健康.
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = None
            if isinstance(d, dict):
                inner = d.get("data")
                if isinstance(inner, dict):
                    d = inner
            if d:
                hmi_state["health"] = d

        def _on_bit(sample) -> None:
            # W8: relay P2's latest health/bit self-test report to /api/bit.
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = None
            if d:
                hmi_state["bit"] = d

        def _on_state_teach(sample) -> None:
            # RUST THREAD: decode + store. The snapshot builder reads it.
            try:
                body = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            hmi_state["teach"] = body

        teach_sub = gen.declare_subscriber(STATE_TEACH_TOPIC, _on_state_teach)

        def _on_uplink_ack(sample) -> None:
            # RUST THREAD: decode and stash only (CLAUDE.md 4.2). The WS loop
            # polls take_uplink_ack; nothing here touches a WebSocket.
            #
            # Serves cmd/geo/ack (W4) AND cmd/task/ack (W2 goto, W7 task): the
            # ack shapes are the same S7.7 Ack and the routing key is the cmd_id,
            # so one handler is not a shortcut here -- two would be two places to
            # forget the "h-" check below.
            try:
                body = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                return
            cmd_id = body.get("cmd_id")
            if not isinstance(cmd_id, str) or not cmd_id.startswith("h-"):
                # Not ours: these keys also carry answers to cloud- and
                # voice-originated commands. Keyed on the "h-" prefix P5 itself
                # stamped, so one bus key serves every origin without P5
                # claiming acks that belong to another sender.
                return
            hmi_state.setdefault("uplink_acks", {})[cmd_id] = body

        geo_ack_sub = gen.declare_subscriber(CMD_GEO_ACK_TOPIC, _on_uplink_ack)
        # W2/W7 answers. The SAME handler: an ack is routed by the "h-" cmd_id
        # P5 stamped, not by which key it arrived on, so cmd/task/ack answers to
        # voice- and cloud-originated commands are ignored here exactly as
        # cmd/geo/ack's already are.
        task_ack_sub = gen.declare_subscriber(CMD_TASK_ACK_TOPIC, _on_uplink_ack)
        mode_ack_sub = gen.declare_subscriber(CMD_MODE_ACK_TOPIC, _on_uplink_ack)
        ack_sub = gen.declare_subscriber(
            CMD_AUDIO_SPEAK_ACK_TOPIC, _on_speak_ack)
        task_sub = gen.declare_subscriber(
            STATE_TASK_TOPIC, _on_state_task)
        fence_sub = gen.declare_subscriber(CMD_FENCE_TOPIC, _on_cmd_fence)
        geo_sub = gen.declare_subscriber(STATE_GEO_OBJECTS_TOPIC, _on_geo_objects)
        mode_sub = gen.declare_subscriber(STATE_MODE_TOPIC, _on_state_mode)
        audio_sub = gen.declare_subscriber(STATE_AUDIO_TOPIC, _on_state_audio)
        pose_sub = gen.declare_subscriber(STATE_POSE_TOPIC, _on_state_pose)
        clock_sub = gen.declare_subscriber(STATE_CLOCK_TOPIC, _on_state_clock)
        robot_sub = gen.declare_subscriber(STATE_ROBOT_TOPIC, _on_state_robot)
        power_sub = gen.declare_subscriber(STATE_POWER_TOPIC, _on_state_power)
        event_sub = gen.declare_subscriber(EVENT_WILDCARD_TOPIC, _on_event)
        event_ack_sub = gen.declare_subscriber(EVENT_ACK_TOPIC, _on_event_ack)
        recon_rsp_sub = gen.declare_subscriber(
            EVENT_RECON_RSP_TOPIC, _on_recon_rsp)
        estop_pong_sub = gen.declare_subscriber(
            PROBE_ESTOP_PONG_TOPIC, _on_estop_pong)
        health_sub = gen.declare_subscriber(HEALTH_SUMMARY_TOPIC, _on_health)
        bit_sub = gen.declare_subscriber(HEALTH_BIT_TOPIC, _on_bit)

        # --- Cloud Qt face (v2.0). See runtime/cloud_wiring.py -------------
        # ADDITIVE by construction: the cloud keys are xbrain/{rid}/... and
        # every key declared above is relative, so the two sets cannot
        # intersect. The bridge REPUBLISHES onto the relative keys p3_task
        # already subscribes, in the same 11 S7.2 shape voice uses -- which is
        # why turning the cloud face on changes nothing for voice/text/HMI.
        #
        # rid absent -> no bridge (same trade-off as p1 GNSS bridge): a
        # "xbrain//cmd/task" key would subscribe to something nobody publishes
        # and look exactly like a client that never connected.
        from xbrain.p5_gateway.hmi.task_query_client import query_tasks
        #: query/tasks 的节流时刻. list 而不是标量, 因为它在下面的闭包与循环
        #: 之间共享; nonlocal 在这个函数里已经用了好几个, 再加一个会更难读.
        _last_task_query = [0.0]

        from xbrain.p5_gateway.runtime.cloud_wiring import maybe_wire
        # 11 S4.6.3 步骤 1: 云端来的每一条报文都刷新断线起算点. 不接这条线
        # 的话, 甲方持续下发任务而我们仍判 never_connected -- 断线时长累到
        # rtb_s(L3)就自动注入返航, 优先级 95, 把客户的任务抢占掉.
        cloud_bridge = maybe_wire(gen, os.environ.get("XBRAIN_ROBOT_ID", ""),
                                  on_cloud_rx=lambda: link_state.on_cloud_rx(
                                      time.monotonic()),
                                  # HB-1: 心跳带 state="down" -> 立即断开.
                                  on_cloud_down=link_state.on_cloud_explicit_down,
                                  # HB-2: session_id 变化 -> 全量面立即重发.
                                  # 用 lambda 而不是直接传方法: cloud_projector
                                  # 在本行之后才建, 名字要到调用时才解析.
                                  on_new_session=lambda: (
                                      cloud_projector.force_resend()
                                      if cloud_projector is not None else None))
        if cloud_bridge is not None:
            from xbrain.p5_gateway.runtime.cloud_state import CloudProjector
            # The outbound face needs DRIVING, not just publishers: a declared
            # publisher nobody puts to looks exactly like a healthy key that
            # happens to be quiet. The projector rides the 10 Hz loop below,
            # which is also state/robot's required rate (v2.0 S4.2).
            cloud_projector = CloudProjector(cloud_bridge)
            _logger.info("p5 wiring: cloud Qt face on, %d inbound keys",
                         cloud_bridge.alive())
        _logger.info("p5 wiring: subscribed speak/ack + state/task + "
                     "cmd/fence + state/mode + event/** + estop/pong + health "
                     "+ cmd/geo/ack + cmd/task/ack + cmd/mode/ack "
                     "(HMI W2/W3/W4/W7 uplink) "
                     "+ state/teach (read only)")

        # Start the HMI web server (best-effort; never blocks the voice loop).
        hmi_server, _hmi_thread = (None, None)
        if hmi_cfg:
            hmi_server, _hmi_thread = _start_hmi(gen, hmi_cfg, hmi_state,
                                                 site_timezone)

        try:
            last_hb = time.monotonic()
            last_recon = time.monotonic()
            while not stop_flag.get("stop"):
                now = time.monotonic()
                # Periodic recon (17 S3Y.3): ask the cloud what it is missing. Runs
                # regardless of link state -- it is how P5 discovers holes AND that
                # the cloud is back. No-op until the event subsystem + cloud exist.
                if (event_subsystem is not None and event_subsystem.enabled
                        and now - last_recon >= RECON_PERIOD_S):
                    event_subsystem.send_recon_reqs()
                    last_recon = now
                if now - last_hb >= heartbeat_period_s:
                    # W5: send one estop probe per heartbeat and read back the
                    # ok/degraded/down verdict. on_ping_sent must run BEFORE the
                    # verdict so a missing pong for the prior ping is counted this
                    # tick; without a chassis no pong ever arrives -> "down", and
                    # the HMI greys the button honestly (17 S6.3).
                    probe_seq += 1
                    probe_env_seq += 1
                    ping_mono = _now_mono_ms()
                    estop_probe.on_ping_sent(probe_seq, ping_mono)
                    # 11 S8.5 的 ping 体 {type, seq, t_mono_ms}, 装进 S3.0 信封.
                    #
                    # *** 信封是必须的, 不是装饰(11 S3.0 逐字: "所有 Zenoh JSON
                    # 载荷共用此外层结构"). 本行在 2026-09-27 之前发的是裸对象,
                    # 于是 chassis_relay 走 wrap-if-bare 兜底转发, 而那条兜底
                    # [写不出 rid](它没有原信封可抄) => quadruped 的 read_envelope
                    # 在 rid 那一步就退出, data 根本没被填, 回显恒 0.
                    #
                    # *** 关联号 seq 放在 data 里, 信封 seq 是另一个数.
                    # RT-C3.e 要求转发者重建信封并换上自己的 seq, relay 就在这
                    # 两条腿中间 => 信封 seq 到不了对端. data 是唯一被逐字节
                    # 搬运的部分, 端到端关联字段只能放那里(见 _on_estop_pong).
                    ts_sync = bool((hmi_state.get("clock") or {}).get("sync") is True)
                    estop_ping_pub.put(json.dumps(encode(Envelope(
                        v=1, rid=probe_rid,
                        # WALL-CLOCK-OK(align): 11 S3.0 envelope ts, cross-host
                        # alignment and recording only; the RTT that drives
                        # estop_path is computed from t_mono_ms, never from this
                        ts=time.time(),
                        # CLK-C1 单调秒; CLK-C4: 同机才带 mono, 且 mono 与 boot
                        # 成对出现 -- 没有 boot 就没有 mono 的定义域.
                        mono=time.monotonic() if probe_boot else None,
                        boot=probe_boot or None,
                        seq=probe_env_seq, src="p5_gateway",
                        # CLK-A2: ts_sync 抄 ClockStatus.sync(P1-13 镜像过来的),
                        # NO 本进程不自行判定授时状态; 无来源一律 false(CLK-A3).
                        ts_sync=ts_sync,
                        data=build_ping_data(probe_seq, ping_mono),
                    ))).encode("utf-8"))
                    # 11 S4.6 cloud-link state (P5 is the sole authority, LNK-6).
                    st = link_state.evaluate(now)
                    link_payload = {
                        "schema": "state_link_v1",
                        "gateway_up": True,
                        # -- cloud link (11 S4.6.2): the RTB judge (NFR-12/TSK-20..22)
                        "cloud_link": st.cloud_link,
                        "level": st.level,
                        "disconnected_s": st.disconnected_s,
                        "to_next_level_s": st.to_next_level_s,
                        "reason": st.reason,
                        "last_rx_mono": st.last_rx_mono,
                        "link_epoch": st.link_epoch,
                        "gw_start_mono": st.gw_start_mono,
                        "thresholds": {
                            "degraded_s": link_thresholds.degraded_s,
                            "down_s": link_thresholds.down_s,
                            "rtb_s": link_thresholds.rtb_s,
                            "stable_s": link_thresholds.stable_s,
                        },
                        # estop_path lets the HMI arm/grey its ESTOP button
                        # (NAV-64): ok only on a fresh pong under the RTT
                        # threshold, degraded when slow, down after
                        # link_down_misses missing pongs (17 S6.3). EP-3: the
                        # cloud link and the estop path are judged separately.
                        "estop_path": estop_probe.estop_path(),
                        # latency_ms IS the estop probe's last RTT (11 S4.6.5 /
                        # 17 S6.2 link.data.latency_ms; 17 line "latency_ms = S6.3
                        # link_probe 最近一次 RTT"). status_group reads latency_ms.
                        "latency_ms": estop_probe.rtt_ms,
                        "mono_ms": _now_mono_ms(),
                        "speak_acks": speak_acks_seen,
                        "task_updates": state_task_updates,
                    }
                    hmi_state["link"] = link_payload   # feed HMI status/ESTOP
                    # 11 S3.0 逐字"所有 Zenoh JSON 载荷共用此外层结构".
                    # 本条在 2026-09-27 之前发的是裸 dict -- 与 ping 那条同一个
                    # 缺陷(见上方 estop_ping_pub 处的长注): 消费方按 S3.0 解码
                    # 会在必填字段那一步退出, 而它是[云端判在线]的那条 key.
                    # * 云端形态另发(cloud_wiring publish_state -> v2.0 S1.1 的
                    #   六字段信封, 无 mono/boot), 两层各自对各自的契约, NO 不
                    #   共用一个信封 -- CLK-C4 逐字禁止跨主机消息带 mono/boot.
                    # * 机内消费方(p2 _make_state_sink / p3 _on_link)都已能读
                    #   data 嵌套形, 且两者都保留了裸形兜底, 所以本改动不需要
                    #   两侧同时上线.
                    link_pub.put(_stamp(link_payload, _link_env_seq))
                    # Reconnect -> backfill (17 S3.5.2): the state machine flags the
                    # once-per-outage down->up edge. No-op while the cloud has never
                    # been heard from (dev has no cloud) -> dormant until real uplink.
                    if (st.reconnected and event_subsystem is not None
                            and event_subsystem.enabled):
                        _logger.info(
                            "cloud link reconnect (epoch %d) -> trigger backfill",
                            st.link_epoch)
                        event_subsystem.trigger_backfill()
                    # 11 S4.6.8: a comm event on each cloud-link level transition.
                    # P5 publishes it like any producer; its own event/** subscriber
                    # persists it (not a self-loop -- it is a real event, not a
                    # replay, so the R-2 filter lets 'comm' through).
                    _ce = comm_event_for_level(
                        _prev_link_level, st.level, _prev_disc_s, st.link_epoch)
                    if _ce is not None:
                        _ckind, _csev, _cdetail = _ce
                        _comm_seq[0] += 1
                        _ckey = "event/%s/comm" % _csev
                        # 同 state/link: 本条此前也是裸 dict, 违 11 S3.0.
                        # 信封的 seq 按 key 分桶 -- {sev} 段会变, 共用一个计数
                        # 器会让每条 key 在消费方看来一直跳号.
                        #
                        # * 内层的 ts 原写死 0.0. 信封补上以后 p5 自己的
                        #   _normalise_event 会优先取信封 ts, 但 HMI 事件环
                        #   与别的消费方仍可能直读内层 -- p2 踩过同一个坑
                        #   (急停事件在本机界面上显示 1970 年), 所以一并填真.
                        _cwall = time.time()  # WALL-CLOCK-OK(record): 11 S6.2 的事件墙钟戳, 只用于显示与审计
                        gen.put(_ckey, _stamp({
                            "eid": "comm-%s-%d" % (_comm_boot, _comm_seq[0]),
                            "title": "cloud link %s" % _ckind,
                            "detail": _cdetail,
                            "src": "p5_gateway", "ts": _cwall,
                        }, _comm_env_seq.setdefault(_ckey, [0])))
                        _logger.info("p5 comm event: %s (sev=%s level=%d)",
                                     _ckind, _csev, st.level)
                    _prev_link_level = st.level
                    _prev_disc_s = st.disconnected_s
                    last_hb = now
                # Cloud outbound projection. OUTSIDE the last_hb gate on
                # purpose: that gate is the 1 Hz link heartbeat, and
                # state/robot must go out at 10 Hz (v2.0 S4.2 verbatim).
                # The projector does its own per-key cadence, so running it
                # every loop iteration costs one dict build per key and no
                # extra publishes.
                # *** 云端快照的任务全量: 走 P3 的 query/tasks queryable.
                # v2.0 S3.2 的 snapshot 要 current + queue + suspended 三个列表,
                # 而 state/task 广播只带 active_task 一条(15 S12A 的形状) --
                # 靠它永远填不出 queue/suspended, 实测两个列表恒空.
                # 11 S12.2A 的 query/tasks 正是为这件事存在的(HMI 的
                # /api/tasks 已经在用), 平面隔离下 p5 不能直接读 p3 的 task.db.
                #
                # *** 1 Hz, NO 不放进 0.1 s 主循环.
                # query_tasks 是[阻塞]调用(迭代 reply channel), 每拍跑一次会把
                # 10 Hz 的 state/robot 一起拖慢 -- 而后者是 Qt 判"机器人还活着"
                # 的依据(v2.0 S4.2 逐字要求 10 Hz). 任务队列的变化是人操作的
                # 节奏, 1 Hz 足够; 真正要求快的是终态, 而终态走 observe_task
                # 那条即时路径(见 _on_state_task), 不受本节流影响.
                if cloud_projector is not None:
                    _now_q = time.monotonic()
                    if _now_q - _last_task_query[0] >= 1.0:
                        _last_task_query[0] = _now_q
                        try:
                            _page = query_tasks(gen, scope="current", limit=50)
                            hmi_state["cloud_tasks"] = _page.get("tasks") or []
                        except Exception:      # noqa: BLE001
                            # 查不到就保持上一份, NO 不清空: 清空会让 Qt 看到
                            # "队列突然空了", 与"任务真的都跑完了"不可区分.
                            _logger.debug("p5 cloud task query failed")
                    cloud_projector.tick(hmi_state)
                # A-1: 清理超时的 cmd/task pending, 回 timeout ack(v2.0 S1.4).
                if cloud_bridge is not None:
                    cloud_bridge.tick()
                time.sleep(0.1)
        finally:
            # Stop the HMI first so it stops reading shared state, then the event
            # subsystem (flush + close record.db), then the zenoh entities.
            if hmi_server is not None:
                hmi_server.should_exit = True
            if event_subsystem is not None:
                event_subsystem.stop()
            # *** 这个元组原本漏了九个订阅: teach / geo_ack / task_ack /
            # mode_ack / audio / pose / clock / robot / power. ruff F841 把
            # 它们报成 "赋值了从不使用" -- 那不是死代码, 是[声明了却忘了拆]:
            # 局部名字只出现一次, 恰恰说明它没有进过这条拆除路径.
            # 后果是这九个订阅活过 hmi_server.should_exit 与
            # event_subsystem.stop(), 它们的回调继续往 hmi_state 里写, 直到
            # with 退出关会话为止.
            # NO 这里仍然是一张手写名单, 所以它会再漂 -- 守它的是
            # tests/p_processes/test_wiring_teardown_covers_subs.py:
            # 那条判据用 AST 数[本函数里 X = ....declare_subscriber(...) 的
            # X] 与[本元组里的名字], 要求前者被后者全覆盖.
            for entity in (teach_sub, geo_ack_sub, task_ack_sub, mode_ack_sub,
                           ack_sub, task_sub, fence_sub, geo_sub, mode_sub,
                           audio_sub, pose_sub, clock_sub, robot_sub, power_sub,
                           event_sub, event_ack_sub, recon_rsp_sub, estop_pong_sub,
                           health_sub, bit_sub, estop_ping_pub, link_pub,
                           replay_pub_normal, replay_pub_alarm, recon_req_pub):
                if entity is None:
                    continue
                try:
                    entity.undeclare()
                except Exception:      # noqa: BLE001
                    pass
    return 0
