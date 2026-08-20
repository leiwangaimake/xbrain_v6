"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: web_server.py
Brief: FastAPI HMI web server skeleton -- per-interface bind, static, snapshot, ESTOP

Description:
The problem this solves. 17 S6.10 asks for a runnable HMI web server in the
p5_gateway process (no separate HMI process, 10 S3.1) that serves the customer-
styled frontend plus its data. Before this, endpoints.py / ws_protocol.py /
data_readers.py were library components with tests but nothing assembled them
into a listening server (p5_gateway __main__ said "FastAPI HMI backend NOT done
yet"). This module is that assembly -- routes + bind + wiring seams -- and
NOTHING else: no subscription logic, no DB, no estop routing of its own.

Which section this follows: 17 S6.10 (web server), S6.10.3 (bind + port),
S6.5 (read-only REST), S6.2 (WS kinds), S6.3/S6.4/NAV-64 (ESTOP + link).

Two hard rules this file exists to honour:
  * BIND (S6.10.3 / NET-C9): NEVER 0.0.0.0. The web server binds one socket per
    interface in `hmi.bind` (LAN2 IP, wifi IP, 127.0.0.1) and refuses to start
    if none is configured -- an ESTOP-capable control surface on 0.0.0.0 exposes
    unauthenticated estop/mode/ptz to the whole network (10 S4.5 threat model).
    make_bound_sockets() is the one place that enforces this.
  * DELEGATION: the server owns no state and no estop path. The current P5
    runtime state reaches it through an injected `providers` object, and ESTOP
    is delegated to an injected `estop_sender` (which, in main_wiring, sends the
    W1 frame on 17 S6.4's dedicated <=10 ms path). So this file can be unit-
    tested with fakes, and the real fast path is never re-implemented here.

What it does NOT do, and the boundary. It does not subscribe to Zenoh, open a
DB, or fabricate pose -- data_readers.build_snapshot() returns availability
flags and the frontend greys absent layers (17 S6.10.4). It does not add auth:
per-interface binding is the security boundary today (S6.10.3); an auth layer,
if added later, lands here without changing the readers.

Trap already noted: /api/fences must return 503 E_DEGRADED (never 200 []) when
the fence cache is degraded (17 S6.9 P5F-2); the route below asks the provider
for degraded-ness and uses rest.endpoints.fences_endpoint to pick the code.
"""

from __future__ import annotations

import socket
from typing import Any, Callable, Dict, List, Optional, Protocol, Tuple

from starlette.websockets import WebSocket, WebSocketDisconnect

from xbrain.common.errors import E_SCHEMA
from xbrain.p5_gateway.hmi import uplink
from xbrain.p5_gateway.hmi.ws_protocol import RateLimitBucket

from xbrain.p5_gateway.hmi import data_readers
from xbrain.p5_gateway.hmi.ui_config import build_ui_config

# WebSocket / WebSocketDisconnect are imported at MODULE level, not lazily inside
# build_app, on purpose: `from __future__ import annotations` (top of this file)
# turns the endpoint annotation `websocket: WebSocket` into the STRING
# "WebSocket", which FastAPI resolves against THIS module's globals. A local
# import inside build_app is invisible there, so FastAPI mis-reads `websocket` as
# a required query param and rejects every handshake 403. Resolving the name here
# fixes it (starlette's WebSocket IS fastapi's -- same class). Starlette is a
# FastAPI dependency and this module is itself imported lazily by the wiring, so
# there is no new package-import side effect.


# W6: resend a full keyframe every N WS ticks so a missed/misapplied delta cannot
# drift the client past one period (at push_hz=2 this is ~15 s). Not a safety
# value -- purely a self-heal cadence, so a literal is fine here (not 3.1).
WS_KEYFRAME_EVERY = 30


class HmiBindError(RuntimeError):
    """No usable bind interface, or a 0.0.0.0 bind was requested.

    Raised at startup so the process refuses to serve rather than silently
    binding wide (NET-C9) or binding nothing at all. The message names the
    offending entry so the deploy operator can fix the interface list.
    """


class RuntimeStateProvider(Protocol):
    """The seam between the web server and P5's live runtime state.

    main_wiring supplies the real implementation (reading the fence cache, the
    state/task cache, state/link, the record.db events, and -- once built -- the
    pose stream). A test supplies a fake. Every method returns exactly what
    data_readers.build_snapshot expects, or None when that source is absent.
    Keeping this a Protocol (not a concrete class) is what lets the server carry
    no state of its own.
    """

    def snapshot_inputs(self) -> Dict[str, Any]:
        """Return the kwargs for data_readers.build_snapshot (fences/routes/
        waypoints/pose/tasks/mode/link/health/events), each None when absent."""
        ...

    def fence_degraded(self) -> bool:
        """True when the fence cache cannot answer authoritatively -> 503
        E_DEGRADED on /api/fences (17 S6.9 P5F-2), never a 200 empty set."""
        ...

    def rest_inputs(self) -> Dict[str, Any]:
        """Sources for the 17 S6.5 REST endpoints NOT in the build_snapshot A..F
        set (health/bit/routes/docks/metrics/approval_pending), each None when
        absent. OPTIONAL: build_app reads it via getattr, so a legacy provider
        without it simply serves those endpoints as available:false."""
        ...

    def send_uplink(self, key: str, payload: Dict[str, Any]) -> None:
        """Publish one HMI-originated command on the general plane (11 S12.1.1
        W2/W3/W4/W7). OPTIONAL: read via getattr, so a provider without it makes
        the upstream classes answer E_NOT_IMPLEMENTED instead of failing the
        handshake -- the browser then shows a refusal rather than a dead link."""
        ...

    def take_uplink_ack(self, req_id: str) -> Optional[Dict[str, Any]]:
        """The downstream ack for a forwarded req_id, once, or None.

        Once, because the WS loop polls: leaving it in place would resend the
        same ack every tick. The wiring holds acks keyed by the cmd_id it
        stamped (S12.1.1: "h-" + req_id), so the round trip needs no session
        state in this module."""
        ...


def parse_bind_entry(entry: str) -> Tuple[str, int]:
    """Split a "IP:port" bind entry into (host, port).

    Rejects 0.0.0.0 loudly (NET-C9): a wide bind on an ESTOP surface is the one
    mistake this whole module is built to prevent, so it fails here rather than
    being silently accepted. IPv6-bracket forms are out of scope for the LAN2/
    wifi deployment and rejected with a clear message rather than mis-split.
    """
    if entry.count(":") != 1:
        raise HmiBindError("bad bind entry %r (want IP:port)" % entry)
    host, port_s = entry.rsplit(":", 1)
    if host in ("0.0.0.0", "::", ""):
        raise HmiBindError(
            "bind %r is a wildcard address; NET-C9 forbids 0.0.0.0 on the "
            "ESTOP-capable HMI (17 S6.10.3). Bind LAN2/wifi IPs explicitly."
            % entry)
    try:
        port = int(port_s)
    except ValueError:
        raise HmiBindError("bad port in bind entry %r" % entry) from None
    return host, port


def make_bound_sockets(bind: List[Optional[str]]) -> List[socket.socket]:
    """Create one bound TCP socket per non-null interface in `hmi.bind`.

    Null entries are skipped (17 S6.10.3: LAN2/wifi IPs are null until the
    deployment fills them). If NOTHING is bindable the HMI refuses to start --
    an HMI reachable from nowhere is a config bug the operator must see, and a
    silent fallback to a wildcard would be the exact NET-C9 violation this
    guards. Each socket is passed to uvicorn's Server.serve(sockets=...), which
    binds to these specific addresses and never widens them.
    """
    entries = [e for e in bind if e]
    if not entries:
        raise HmiBindError(
            "hmi.bind has no interface (all null). Set at least the LAN2 or "
            "wifi IP:port before starting the HMI (17 S6.10.3).")
    socks: List[socket.socket] = []
    for entry in entries:
        host, port = parse_bind_entry(entry)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # SO_REUSEADDR so a restart does not wait out TIME_WAIT; NOT
        # SO_REUSEPORT (that would let a second process silently share the port).
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))       # explicit host -> never wildcard
        sock.listen(128)
        sock.set_inheritable(False)
        socks.append(sock)
    return socks


#: 11 S12.1.1 W4 rate limit: 10 msg/s. The burst is one second's worth -- a
#: dialog that fires a refs query and then the delete must not be throttled,
#: while a held button still cannot build a queue (SS-4 refuses, never queues).
UPLINK_RATE_PER_S = 10.0
UPLINK_BURST = 10


async def _uplink_reader(websocket, provider, bucket, pending) -> None:
    """Read HMI upstream frames for one connection until it closes.

    Every outcome answers the browser -- forwarded, refused, or rate-limited.
    A frame that produced no answer would leave the operator's dialog spinning,
    and 12.3's reconnect rule (resend with the same req_id) depends on them
    being able to tell "no answer yet" from "answered".
    """
    import time as _time                        # noqa: PLC0415

    while True:
        try:
            msg = await websocket.receive_json()
        except Exception:      # noqa: BLE001
            # Disconnect, or a frame that is not JSON. Either way this
            # connection is done; the send loop's finally cancels this task.
            return
        try:
            env = uplink.parse_envelope(msg)
        except ValueError as exc:
            # No req_id may exist here, so the ack carries whatever was sent.
            await websocket.send_json(uplink.ack_frame(
                str(msg.get("req_id") or "") if isinstance(msg, dict) else "",
                str(msg.get("type") or "") if isinstance(msg, dict) else "",
                "rejected", E_SCHEMA, {"reason": str(exc)}))
            continue
        req_id, req_type = env["req_id"], env["type"]
        # W1 estop bypasses the bucket (S12.1.1: it bypasses G1-G6, the rate
        # limit and the restricted downgrade). It is not served here at all --
        # the dedicated <=10 ms path is POST /api/estop -- so it is refused
        # explicitly rather than silently rate-limited into a queue.
        if req_type != "estop" and not bucket.try_take(
                int(_time.monotonic() * 1000)):
            ref = uplink.rate_limited(req_type)
            await websocket.send_json(uplink.ack_frame(
                req_id, req_type, "rejected", ref.code, ref.detail))
            continue
        if req_type != "geo":
            ref = uplink.not_implemented(req_type)
            await websocket.send_json(uplink.ack_frame(
                req_id, req_type, "rejected", ref.code, ref.detail))
            continue
        built = uplink.build_geo_command(msg)
        if isinstance(built, uplink.UplinkRefusal):
            await websocket.send_json(uplink.ack_frame(
                req_id, req_type, "rejected", built.code,
                {**(built.detail or {}), "reason_text": built.reason}))
            continue
        sender = getattr(provider, "send_uplink", None)
        if sender is None:
            ref = uplink.not_implemented(req_type)
            await websocket.send_json(uplink.ack_frame(
                req_id, req_type, "rejected", ref.code, ref.detail))
            continue
        sender(built.key, built.payload)
        # Answered later, when P3's cmd/geo/ack comes back through the poll in
        # the send loop. Recorded as pending so a lost ack is visible as a
        # request still waiting, rather than as one that was never sent.
        pending[req_id] = req_type


def build_app(
    hmi_web: Dict[str, Any],
    provider: RuntimeStateProvider,
    estop_sender: Callable[[], None],
    static_root: str,
    *,
    site_timezone: Optional[str] = None,
):
    """Build (do not run) the FastAPI app.

    Split from serve() so the routes can be tested with FastAPI's TestClient and
    a fake provider -- no socket, no uvicorn. `estop_sender` is invoked by
    POST /api/estop and MUST perform the 17 S6.4 dedicated-path W1 send; this
    module never routes estop itself. `hmi_web` is the resolved hmi.web subtree;
    ui_config is built once at construction so a malformed config refuses here.
    site_timezone (common.timezone) rides in ui_config for the footer clock.
    """
    # Imported lazily (like services/payload) so importing this module has no
    # FastAPI side effect and the W-1 startup window can still report why P5 did
    # not come up. FastAPI is a p5_gateway dependency (17 S6.10.0).
    from fastapi import FastAPI                     # noqa: PLC0415
    from fastapi.responses import JSONResponse      # noqa: PLC0415
    from fastapi.staticfiles import StaticFiles     # noqa: PLC0415

    from xbrain.p5_gateway.rest.endpoints import fences_endpoint  # noqa: PLC0415

    ui_config = build_ui_config(hmi_web, site_timezone=site_timezone)
    app = FastAPI(title="XBRAIN_V6 HMI", docs_url=None, redoc_url=None)

    @app.middleware("http")
    async def _no_stale_cache(request, call_next):
        # HMI assets + data must never be served stale from the browser cache.
        # StaticFiles sends etag/last-modified but NO Cache-Control, so browsers
        # apply heuristic caching and can keep an OLD hmi.css/hmi.js after a
        # redeploy (observed: a cached hmi.css kept the coord panel bottom-RIGHT
        # after the source moved it bottom-LEFT). "no-cache" forces revalidation
        # every load -- with the etag that is a cheap 304 when nothing changed,
        # a 200 with the new bytes when it did. Applies to /api + /ws too, which
        # is correct: the operator must always see current UI and current state.
        resp = await call_next(request)
        resp.headers["Cache-Control"] = "no-cache"
        return resp

    @app.get("/api/hmi/ui_config")
    def get_ui_config() -> Dict[str, Any]:
        """U1..U6 presentation config for the frontend (17 S6.10.2). Static per
        process -- the frontend reads it once on load to size grid/fonts/panels."""
        return ui_config

    @app.get("/api/hmi/snapshot")
    def get_snapshot() -> Dict[str, Any]:
        """The 17 S6.8 A..F snapshot. Absent sources report available=false so
        the map greys those layers rather than drawing fabricated data."""
        # site_timezone is the footer-clock FALLBACK; the live zone is derived
        # from the pose fix inside build_snapshot (17 S6.10.2 v1.3).
        return data_readers.build_snapshot(**provider.snapshot_inputs(),
                                           site_timezone=site_timezone)

    # response_model=None: the return annotation is JSONResponse (a Response
    # subclass), and `from __future__ import annotations` makes it the STRING
    # "JSONResponse"; without this, FastAPI 0.140 tries to build a response
    # schema from that forward ref and /openapi.json 500s (PydanticUserError
    # "not fully defined"). None tells it there is no response model. Same on
    # every route below that returns a JSONResponse directly.
    @app.get("/api/fences", response_model=None)
    def get_fences() -> JSONResponse:
        """17 S6.5/S6.9: fence polygons, but 503 E_DEGRADED (never 200 []) when
        the cache is degraded (P5F-2). fences_endpoint picks the status/body;
        JSONResponse carries the status code directly (a `Response` PARAMETER
        makes FastAPI try to validate it as a request field -> 422)."""
        result = fences_endpoint(provider.fence_degraded())
        return JSONResponse(status_code=result.status, content=result.body)

    @app.get("/api/tasks")
    async def get_tasks(scope: str = "current", limit: int = 50,
                        before: Optional[int] = None):
        """17 S6.5 / 11 S12.2A: current OR history task cards for the panel,
        relayed from P3's query/tasks queryable (P5 does not read P3's task.db).
        scope must be current|history; limit is clamped [1,500]; before is the
        keyset paging cursor for lazy history. The Zenoh get() is BLOCKING, so it
        runs in a worker thread -- never inline on the FastAPI event loop. A
        provider without query_tasks (a legacy/test seam) -> available:false empty
        so the panel greys rather than 500s."""
        import asyncio                              # noqa: PLC0415
        if scope not in ("current", "history"):
            return JSONResponse(
                status_code=400,
                content={"error": "scope must be current|history"})
        limit = max(1, min(int(limit), 500))
        qt = getattr(provider, "query_tasks", None)
        if qt is None:
            return {"tasks": [], "has_more": False, "next_before": None,
                    "available": False}
        return await asyncio.to_thread(qt, scope, limit, before)

    from xbrain.p5_gateway.hmi.data_readers import (  # noqa: PLC0415
        events_group, geo_group,
    )

    @app.get("/api/fences/active", response_model=None)
    def get_fences_active() -> JSONResponse:
        """17 S6.5 / 11 S9A.11: the active fence set's full geometry. Same P5F-2
        degraded rule as /api/fences (503, never a 200 empty set); on a fresh
        cache it returns the geo group's fence layer (W8 endpoint alignment)."""
        result = fences_endpoint(provider.fence_degraded())
        if result.status != 200:
            return JSONResponse(status_code=result.status, content=result.body)
        inputs = provider.snapshot_inputs()
        layer = geo_group(inputs.get("fences"), None, None,
                          inputs.get("enu_origin"))["fences"]
        return JSONResponse(status_code=200, content=layer)

    @app.get("/api/events")
    def get_events() -> Dict[str, Any]:
        """17 S6.5: recent events for the HMI stream + map dots. Sourced from the
        wiring's event ring (W2); available:false until the ring is fed. Map dots
        appear only for events with a stamped pos (W4)."""
        return events_group(provider.snapshot_inputs().get("events"))

    # W8: the remaining 17 S6.5 read-only endpoints, so the endpoint SET matches
    # the frozen contract (11 S12.2 == 17 S6.5). Each reads provider.rest_inputs()
    # (separate from snapshot_inputs, which is the keyword-locked build_snapshot
    # A..F set -- adding keys there would break build_snapshot(**inputs)). Sources
    # not yet subscribed report available:false, never a fabricated body.
    from xbrain.p5_gateway.hmi.data_readers import (  # noqa: PLC0415
        rest_list_endpoint, rest_object_endpoint,
    )

    def _rest() -> Dict[str, Any]:
        # getattr so a minimal/legacy provider (or a test fake) without
        # rest_inputs still serves these endpoints as uniformly unavailable
        # rather than 500-ing -- the HMI degrades, it does not crash.
        fn = getattr(provider, "rest_inputs", None)
        return fn() if callable(fn) else {}

    @app.get("/api/routes")
    def get_routes() -> Dict[str, Any]:
        """17 S6.5: route + waypoint list. Gated on geo.db (P5 does not read P3's
        dbs; needs the geo query key, 17 S6.9 note) -> available:false today."""
        return rest_list_endpoint(_rest().get("routes"), "routes")

    @app.get("/api/docks")
    def get_docks() -> Dict[str, Any]:
        """17 S6.5: charging-dock list. Same geo.db gate as /api/routes."""
        return rest_list_endpoint(_rest().get("docks"), "docks")

    @app.get("/api/health")
    def get_health() -> Dict[str, Any]:
        """17 S6.5: health snapshot -- passthrough of P2 health/factor (W8 wired).
        available:false until the first health/factor arrives."""
        return rest_object_endpoint(_rest().get("health"), "health")

    @app.get("/api/bit")
    def get_bit() -> Dict[str, Any]:
        """17 S6.5: last self-test report -- passthrough of P2 health/bit (W8
        wired). available:false until the first health/bit arrives."""
        return rest_object_endpoint(_rest().get("bit"), "bit")

    @app.get("/api/metrics")
    def get_metrics() -> Dict[str, Any]:
        """17 S6.5: telemetry snapshot. Gated on a telemetry aggregator not yet
        instantiated in the voice-loop MVP -> available:false (NEXT.md)."""
        return rest_object_endpoint(_rest().get("metrics"), "metrics")

    @app.get("/api/approval/pending")
    def get_approval_pending() -> Dict[str, Any]:
        """17 S6.5 / S3.8: L3 pending-approval queue snapshot. Same in-memory
        queue as state/approval (G-2), which has no feed in the MVP -> empty +
        available:false, never a fabricated pending item."""
        return rest_list_endpoint(_rest().get("approval_pending"), "pending")

    @app.post("/api/estop")
    def post_estop() -> Dict[str, str]:
        """17 S6.2 W1 / S6.4: the ESTOP button. Delegates to estop_sender, which
        runs the dedicated <=10 ms path; this handler adds no logic on that path
        so nothing here can slow it. The frontend still greys the button on link
        loss (NAV-64) so this is reached only when estop_path was ok."""
        estop_sender()
        return {"status": "sent"}

    # W6: WS push. Replaces the frontend's 1 Hz REST poll with a server push
    # (17 S6.2). On connect the client gets a full state_snapshot (keyframe), then
    # a state_delta each tick carrying ONLY the changed top-level groups -- most
    # quiet ticks send an empty delta (the bandwidth win over a full snapshot per
    # tick, HMI-06 "front-end must stay light"). A periodic keyframe self-heals
    # any drift, and reconnect always starts with a fresh keyframe.
    push_hz = float(hmi_web.get("push_hz", 2) or 2)
    push_interval_s = 1.0 / max(0.2, min(20.0, push_hz))

    @app.websocket("/ws")
    async def ws_snapshot(websocket: WebSocket) -> None:
        # The `WebSocket` type annotation is REQUIRED -- FastAPI injects the
        # connection by type, and without it the handshake is rejected 403.
        import asyncio                          # noqa: PLC0415
        import time                             # noqa: PLC0415
        await websocket.accept()
        # Per-connection delta state: the last-sent snapshot for THIS client (a
        # late joiner must diff against what it has actually received, so this is
        # local to the coroutine, never shared between connections).
        last: Dict[str, Any] = {}
        ticks = 0
        # 11 S12.1.1: the upstream half. One bucket and one pending-set PER
        # CONNECTION -- a shared bucket would let one browser tab rate-limit
        # another, and a shared pending set would deliver tab A's ack to tab B.
        uplink_bucket = RateLimitBucket(
            capacity=UPLINK_BURST, tokens=UPLINK_BURST,
            fill_rate_per_ms=UPLINK_RATE_PER_S / 1000.0,
            last_refill_ms=int(time.monotonic() * 1000))
        pending: Dict[str, str] = {}          # req_id -> req_type
        # The receive side runs as its own task: the send loop below sleeps
        # between pushes, and awaiting a frame inside it would stall the
        # snapshot stream for as long as the operator is not clicking.
        recv_task = asyncio.ensure_future(
            _uplink_reader(websocket, provider, uplink_bucket, pending))
        try:
            while True:
                # site_timezone is the footer-clock fallback; the live zone is
                # derived from the pose fix in build_snapshot (17 S6.10.2 v1.3).
                snap = data_readers.build_snapshot(**provider.snapshot_inputs(),
                                                   site_timezone=site_timezone)
                if not last or ticks % WS_KEYFRAME_EVERY == 0:
                    # Keyframe: full snapshot on connect + every N ticks to self-
                    # heal (a missed/misapplied delta cannot drift past one period).
                    await websocket.send_json(
                        {"kind": "state_snapshot", "data": snap})
                    last = snap
                else:
                    # Delta: only the changed groups. An empty delta is still sent
                    # as a keepalive so the frontend's no-message watchdog (17 S6.3
                    # "2 s no link -> grey") never trips on a live-but-quiet link.
                    changed = data_readers.snapshot_delta(last, snap)
                    await websocket.send_json(
                        {"kind": "state_delta", "data": changed})
                    if changed:
                        last = {**last, **changed}
                # 11 S12.1.1: drain whatever acks P3 answered since the last
                # tick and push them to the tab that asked. Polling rather than
                # a callback keeps the wiring free of any WebSocket knowledge.
                for req_id in list(pending):
                    ack = None
                    taker = getattr(provider, "take_uplink_ack", None)
                    if taker is not None:
                        ack = taker(req_id)
                    if ack is None:
                        continue
                    await websocket.send_json(uplink.ack_frame(
                        req_id, pending.pop(req_id),
                        ack.get("result", "rejected"), ack.get("code", "OK"),
                        ack.get("detail")))
                ticks += 1
                await asyncio.sleep(push_interval_s)
        except WebSocketDisconnect:
            # Normal client close; nothing to clean up (the server owns no
            # per-connection state beyond this coroutine).
            return
        finally:
            recv_task.cancel()

    # Static frontend LAST so /api/* is matched first. html=True serves
    # index.html at "/". A missing static_root is a deploy error, surfaced by
    # StaticFiles at mount time rather than a blank 404 later.
    app.mount("/", StaticFiles(directory=static_root, html=True), name="hmi")
    return app


def start_in_thread(app, sockets: List[socket.socket]):
    """Serve `app` on the pre-bound sockets in a daemon thread; return the
    uvicorn Server so the caller can stop it (server.should_exit = True).

    Why a thread. The p5_gateway voice-loop wiring (main_wiring) is a synchronous
    stop-flag loop, not an asyncio loop, so the async HMI server runs beside it
    on its own loop rather than trying to own the process loop. daemon=True means
    a hard process exit never hangs on it; the caller sets should_exit for a
    clean stop. uvicorn's Server.serve(sockets=...) attaches to exactly the
    sockets make_bound_sockets created -- the NET-C9 per-interface guarantee made
    at bind time is the one that actually serves, never widened here.
    """
    import asyncio                                   # noqa: PLC0415
    import threading                                 # noqa: PLC0415

    import uvicorn                                    # noqa: PLC0415

    # ws="wsproto": uvicorn's legacy websockets_impl handshake is incompatible
    # with the websockets 16.x API present here (rejects the upgrade 403);
    # wsproto is the stable backend across versions. Falls back to "auto" if
    # wsproto is not installed so a REST-poll-only deploy still starts.
    ws_impl = "wsproto"
    try:
        import wsproto  # noqa: F401,PLC0415
    except ImportError:
        ws_impl = "auto"
    # access_log ON: the HMI is a LAN dev/ops surface, and the request log is the
    # only way to see which asset version + which /api/tasks a browser actually
    # fetched (a cached old hmi.js is otherwise invisible from the server side).
    config = uvicorn.Config(app, log_level="info", access_log=True, ws=ws_impl)
    server = uvicorn.Server(config)
    # install_signal_handlers=False: the process's own signal handlers own
    # shutdown (the voice-loop stop flag); uvicorn must not steal SIGINT/SIGTERM
    # from a background thread (it cannot install handlers off the main thread
    # anyway, and trying warns).
    server.config.install_signal_handlers = False

    def _run() -> None:
        asyncio.run(server.serve(sockets=sockets))

    thread = threading.Thread(target=_run, name="hmi-web", daemon=True)
    thread.start()
    return server, thread
