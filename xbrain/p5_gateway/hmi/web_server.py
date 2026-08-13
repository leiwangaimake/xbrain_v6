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


def build_app(
    hmi_web: Dict[str, Any],
    provider: RuntimeStateProvider,
    estop_sender: Callable[[], None],
    static_root: str,
):
    """Build (do not run) the FastAPI app.

    Split from serve() so the routes can be tested with FastAPI's TestClient and
    a fake provider -- no socket, no uvicorn. `estop_sender` is invoked by
    POST /api/estop and MUST perform the 17 S6.4 dedicated-path W1 send; this
    module never routes estop itself. `hmi_web` is the resolved hmi.web subtree;
    ui_config is built once at construction so a malformed config refuses here.
    """
    # Imported lazily (like services/payload) so importing this module has no
    # FastAPI side effect and the W-1 startup window can still report why P5 did
    # not come up. FastAPI is a p5_gateway dependency (17 S6.10.0).
    from fastapi import FastAPI                     # noqa: PLC0415
    from fastapi.responses import JSONResponse      # noqa: PLC0415
    from fastapi.staticfiles import StaticFiles     # noqa: PLC0415

    from xbrain.p5_gateway.rest.endpoints import fences_endpoint  # noqa: PLC0415

    ui_config = build_ui_config(hmi_web)            # raises on malformed config
    app = FastAPI(title="XBRAIN_V6 HMI", docs_url=None, redoc_url=None)

    @app.get("/api/hmi/ui_config")
    def get_ui_config() -> Dict[str, Any]:
        """U1..U6 presentation config for the frontend (17 S6.10.2). Static per
        process -- the frontend reads it once on load to size grid/fonts/panels."""
        return ui_config

    @app.get("/api/hmi/snapshot")
    def get_snapshot() -> Dict[str, Any]:
        """The 17 S6.8 A..F snapshot. Absent sources report available=false so
        the map greys those layers rather than drawing fabricated data."""
        return data_readers.build_snapshot(**provider.snapshot_inputs())

    @app.get("/api/fences")
    def get_fences() -> JSONResponse:
        """17 S6.5/S6.9: fence polygons, but 503 E_DEGRADED (never 200 []) when
        the cache is degraded (P5F-2). fences_endpoint picks the status/body;
        JSONResponse carries the status code directly (a `Response` PARAMETER
        makes FastAPI try to validate it as a request field -> 422)."""
        result = fences_endpoint(provider.fence_degraded())
        return JSONResponse(status_code=result.status, content=result.body)

    from xbrain.p5_gateway.hmi.data_readers import (  # noqa: PLC0415
        events_group, geo_group,
    )

    @app.get("/api/fences/active")
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
    # (17 S6.2 state_snapshot). On connect the client gets a full snapshot, then
    # one snapshot per push interval until it disconnects. state_delta is a later
    # optimisation (NEXT HMI-W6); a full snapshot per tick is correct + simplest,
    # and the frontend renders it exactly like the /api/hmi/snapshot poll body.
    push_hz = float(hmi_web.get("push_hz", 2) or 2)
    push_interval_s = 1.0 / max(0.2, min(20.0, push_hz))

    @app.websocket("/ws")
    async def ws_snapshot(websocket: WebSocket) -> None:
        # The `WebSocket` type annotation is REQUIRED -- FastAPI injects the
        # connection by type, and without it the handshake is rejected 403.
        import asyncio                          # noqa: PLC0415
        await websocket.accept()
        try:
            while True:
                snap = data_readers.build_snapshot(**provider.snapshot_inputs())
                await websocket.send_json({"kind": "state_snapshot", "data": snap})
                await asyncio.sleep(push_interval_s)
        except WebSocketDisconnect:
            # Normal client close; nothing to clean up (the server owns no
            # per-connection state beyond this coroutine).
            return

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
    config = uvicorn.Config(app, log_level="info", access_log=False, ws=ws_impl)
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
