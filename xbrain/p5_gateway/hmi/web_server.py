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

from xbrain.p5_gateway.hmi import data_readers
from xbrain.p5_gateway.hmi.ui_config import build_ui_config


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
    from fastapi import FastAPI, Response          # noqa: PLC0415
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
    def get_fences(response: Response) -> Dict[str, Any]:
        """17 S6.5/S6.9: fence polygons, but 503 E_DEGRADED (never 200 []) when
        the cache is degraded (P5F-2). fences_endpoint picks the status/body."""
        result = fences_endpoint(provider.fence_degraded())
        response.status_code = result.status
        return result.body

    @app.post("/api/estop")
    def post_estop() -> Dict[str, str]:
        """17 S6.2 W1 / S6.4: the ESTOP button. Delegates to estop_sender, which
        runs the dedicated <=10 ms path; this handler adds no logic on that path
        so nothing here can slow it. The frontend still greys the button on link
        loss (NAV-64) so this is reached only when estop_path was ok."""
        estop_sender()
        return {"status": "sent"}

    # Static frontend LAST so /api/* is matched first. html=True serves
    # index.html at "/". A missing static_root is a deploy error, surfaced by
    # StaticFiles at mount time rather than a blank 404 later.
    app.mount("/", StaticFiles(directory=static_root, html=True), name="hmi")
    return app


async def serve(
    app,
    sockets: List[socket.socket],
) -> None:
    """Run the app on the pre-bound sockets (never re-binding, never widening).

    uvicorn's Server.serve(sockets=...) attaches to exactly the sockets
    make_bound_sockets created, so the NET-C9 per-interface guarantee made at
    bind time is the one that actually serves. Kept async so it composes into
    the p5_gateway event loop next to the voice-loop / link tasks.
    """
    import uvicorn                                   # noqa: PLC0415

    config = uvicorn.Config(app, log_level="info", access_log=False)
    server = uvicorn.Server(config)
    await server.serve(sockets=sockets)
