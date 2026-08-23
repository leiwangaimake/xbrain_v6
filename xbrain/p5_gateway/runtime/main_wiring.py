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
STATE_POSE_TOPIC = "state/pose"          # P1-1: pose + RTK heading (p1 bridge)
STATE_CLOCK_TOPIC = "state/clock"        # P1-13: clock sync mirror (18-C G47)
EVENT_WILDCARD_TOPIC = "event/**"        # W2: event/{severity}/{category} stream
EVENT_ACK_TOPIC = "event/ack"            # 17 S3.5.1: cloud ack -> mark delivered
EVENT_RECON_RSP_TOPIC = "event/recon/rsp"  # 17 S3Y.3: cloud recon answer
RECON_PERIOD_S = 300.0                    # 17 S3Y.3 recon.period_s (interim const)
PROBE_ESTOP_PING_TOPIC = "probe/estop/ping"  # W5: P5 ping (11 CR-2, 17 S6.3)
PROBE_ESTOP_PONG_TOPIC = "probe/estop/pong"  # W5: quadruped pong (11 CR-3, authoritative source)
HEALTH_FACTOR_TOPIC = "health/factor"    # W8: P2 health snapshot -> /api/health
HEALTH_BIT_TOPIC = "health/bit"          # W8: P2 self-test report -> /api/bit
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

    P3 publishes {schema, active_task:{task_id, state, mono_ms}} today (11 S2.2.2,
    p3 main_wiring _make_publish). A future 1 Hz heartbeat may instead carry a
    LIST of HeartbeatState (progress.py: task_id/state/current_step/total_steps).
    Accept BOTH shapes so the panel keeps working when P3 upgrades the payload,
    and return [] (not a fabricated card) for anything without a task_id -- the
    trap here is wrapping the whole envelope as one 'plan', which is what the MVP
    did and made _plan read state/targets off {schema, active_task} and get None.
    """
    if not isinstance(payload, dict):
        return []
    # Current P3 shape: a single active_task object.
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
        HmiBindError, build_app, make_bound_sockets, start_in_thread,
    )

    bind = hmi_cfg.get("bind") if isinstance(hmi_cfg, dict) else None
    web = hmi_cfg.get("web") if isinstance(hmi_cfg, dict) else None
    if not bind or not web:
        _logger.warning("p5 HMI: no hmi.bind/hmi.web config; HMI not started")
        return None, None

    # ESTOP button -> W1 (17 S6.2). MVP sends the frame on cmd/estop; the
    # dedicated <=10 ms fast path (17 S6.4 / P-1) is a follow-up (NEXT.md).
    estop_pub = gen.declare_publisher(CMD_ESTOP_TOPIC)

    def _estop_sender() -> None:
        estop_pub.put(json.dumps({"type": "estop", "action": "stop"})
                      .encode("utf-8"))
        _logger.warning("p5 HMI ESTOP pressed -> cmd/estop published")

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
        import os                                    # noqa: PLC0415
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


def _event_seg_index(segs: list) -> int:
    """Index of the 'event' segment, or -1. Works for BOTH the absolute contract
    key (xbrain/{rid}/event/{sev}/{cat}) and the relative dev-bus key
    (event/{sev}/{cat}) -- the two schemes differ by the xbrain/{rid} prefix, so
    locating 'event' rather than a fixed offset is the only robust parse."""
    try:
        return segs.index("event")
    except ValueError:
        return -1


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
    sev = (segs[ei + 1] if 0 <= ei and len(segs) > ei + 1
           else (d.get("sev") or d.get("severity")))
    cat = (segs[ei + 2] if 0 <= ei and len(segs) > ei + 2
           else (d.get("cat") or d.get("category")))
    rid = segs[ei - 1] if ei >= 1 else None            # absolute: .../{rid}/event
    rid = rid or os.environ.get("XBRAIN_ROBOT_ID") or d.get("rid")
    data = d.get("data") if isinstance(d.get("data"), dict) else d
    eid = data.get("eid") or data.get("event_id") or d.get("eid")
    if not (eid and rid and sev and cat):
        return None
    now = datetime.now(timezone.utc)
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
    from xbrain.p5_gateway.hmi.estop_probe import EstopProbe

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

    # Shared state the HMI provider reads. The sync callbacks below REPLACE whole
    # values (never mutate in place) so the web thread's reads stay consistent
    # under the GIL without a lock. fence_cache follows the same rule: on_update
    # swaps the tuple wholesale (P5F-1), so a concurrent snapshot is consistent.
    hmi_state: dict = {
        "tasks": None,               # state/task  -> plan panel
        "link": None,                # state/link  -> status + ESTOP arming
        "mode": None,                # state/mode  -> footer mode (W3)
        "pose": None,                # state/pose  -> coord panel + heading dial + RTK
        "pose_updated_ms": 0,        # last state/pose arrival (mono) -> staleness gate
        "clock": None,               # state/clock -> RTK time-sync indicator
        "events": [],                # event/**    -> event stream ring (W2)
        "health": None,              # health/factor -> /api/health (W8)
        "bit": None,                 # health/bit  -> /api/bit (W8)
        "enu_origin": None,          # localisation origin (gated, W4)
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
                now_iso=lambda: datetime.now(timezone.utc).isoformat(),
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
            LinkStateMachine, LinkThresholds,
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

        speak_acks_seen = 0
        state_task_updates = 0
        probe_seq = 0

        def _on_estop_pong(sample) -> None:
            # W5: a quadruped reply. Match by seq so a late pong for an older
            # ping cannot mask a current outage (EstopProbe ignores the mismatch).
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = {}
            seq = d.get("seq")
            if isinstance(seq, int):
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
                fences = [{"name": p.get("name"),
                           "vertices": p.get("vertices"),
                           "role": p.get("role")} for p in polys]
                hmi_state["fence_cache"].on_update(fences, _now_mono_ms())

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
            if hmi_state.get("enu_origin") is None:
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
            ev = {
                "eid": d.get("eid") or d.get("event_id"),
                "title": d.get("title") or d.get("message"),
                "sev": d.get("severity") or d.get("sev"),
                "cat": d.get("category") or d.get("cat"),
                "ts": d.get("ts"),
                "pos": d.get("pos"),   # None until pose stamps it (W4)
            }
            hmi_state["events"] = (hmi_state["events"] + [ev])[-EVENT_RING:]
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
            # W8: relay P2's latest health/factor to /api/health. P5 forwards the
            # authoritative payload unchanged (G-2 same-source), REPLACING the
            # whole value so the web thread's read stays consistent under the GIL.
            try:
                d = json.loads(bytes(sample.payload).decode("utf-8"))
            except Exception:      # noqa: BLE001
                d = None
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
        pose_sub = gen.declare_subscriber(STATE_POSE_TOPIC, _on_state_pose)
        clock_sub = gen.declare_subscriber(STATE_CLOCK_TOPIC, _on_state_clock)
        event_sub = gen.declare_subscriber(EVENT_WILDCARD_TOPIC, _on_event)
        event_ack_sub = gen.declare_subscriber(EVENT_ACK_TOPIC, _on_event_ack)
        recon_rsp_sub = gen.declare_subscriber(
            EVENT_RECON_RSP_TOPIC, _on_recon_rsp)
        estop_pong_sub = gen.declare_subscriber(
            PROBE_ESTOP_PONG_TOPIC, _on_estop_pong)
        health_sub = gen.declare_subscriber(HEALTH_FACTOR_TOPIC, _on_health)
        bit_sub = gen.declare_subscriber(HEALTH_BIT_TOPIC, _on_bit)
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
                    ping_mono = _now_mono_ms()
                    estop_probe.on_ping_sent(probe_seq, ping_mono)
                    # ping payload per 11 S8.5: {type:"ping", seq, t_mono_ms}.
                    estop_ping_pub.put(json.dumps(
                        {"type": "ping", "seq": probe_seq,
                         "t_mono_ms": ping_mono}).encode("utf-8"))
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
                    link_pub.put(json.dumps(link_payload).encode("utf-8"))
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
                        gen.put("event/%s/comm" % _csev, json.dumps({
                            "eid": "comm-%s-%d" % (_comm_boot, _comm_seq[0]),
                            "title": "cloud link %s" % _ckind,
                            "detail": _cdetail,
                            "src": "p5_gateway", "ts": 0.0,
                        }).encode("utf-8"))
                        _logger.info("p5 comm event: %s (sev=%s level=%d)",
                                     _ckind, _csev, st.level)
                    _prev_link_level = st.level
                    _prev_disc_s = st.disconnected_s
                    last_hb = now
                time.sleep(0.1)
        finally:
            # Stop the HMI first so it stops reading shared state, then the event
            # subsystem (flush + close record.db), then the zenoh entities.
            if hmi_server is not None:
                hmi_server.should_exit = True
            if event_subsystem is not None:
                event_subsystem.stop()
            for entity in (ack_sub, task_sub, fence_sub, geo_sub, mode_sub,
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
