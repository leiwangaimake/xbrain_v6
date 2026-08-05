#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: app.py
Brief: payload-service entry point -- assembles config, device link and REST + WS API.

Description:
  The wiring layer for payload-service. It performs the small amount of glue that
  turns the individual pieces into a running service:
    - read configuration from the environment (config.from_env),
    - own the device sockets for exactly the lifetime of the ASGI app, via a FastAPI
      lifespan, so the links connect on startup and flush-close on shutdown,
    - own the SessionManager (the mode state machine and R2/R3 gate) for that same
      lifetime, tearing it down before the sockets close so no deter loop or audio
      session outlives the app,
    - mount the REST control router and the WebSocket audio router,
    - run uvicorn bound to loopback.

  This module is intentionally thin: it contains no protocol code, no threading, and
  no request handling of its own. Those live in protocol/, core/device_link, and
  api/rest respectively. Keeping app.py to pure composition means the interesting
  logic is each unit-testable on its own, and the only thing that lives here is the
  decision of how the units are connected and for how long each one lives.

  Lifetime coupling (why a lifespan rather than a module-level DeviceLink): the device
  links must be up for exactly as long as the server is accepting requests -- not a
  moment before (importing the app must have no device side effect) and not a moment
  after (shutdown must flush-close). A FastAPI lifespan brackets precisely that
  window, so binding the DeviceLink's start()/stop() to it makes "sockets are open"
  and "server is serving" the same interval by construction, with no way for one to
  outlive the other.

  Two invariants are enforced right here at the edge:
    - R1 (single socket owner): only the DeviceLink created below opens device
      sockets; the app just holds a reference to it and every request handler reads
      through that reference, so there is exactly one socket owner in the process.
    - R2 (single audio client): the only intended client of this service is the
      local AI_runtime on the same box. To make an accidental off-box exposure
      impossible, the listen host is HARD-PINNED to 127.0.0.1 here rather than taken
      from config, so no stray environment value can widen the bind. Making this a
      code constant instead of a setting means the safe bind is not something an
      operator can misconfigure -- it is not configurable at all.

  Run (from the /opt/xbrain_v6 root on the Orin, using its python3.10):
      python3 -m services.payload.app
  See development plan sections 5, 8, 10 and 16.
"""
from __future__ import annotations

import logging
from contextlib import asynccontextmanager
from typing import AsyncIterator

import uvicorn
from fastapi import FastAPI

# Two routers: the REST control plane (status/lights/tts) and the WebSocket data plane
# (the /mic and /play audio streams). Both are mounted below; keeping them as separate
# routers lets each plane own its own file and traffic shape (see api/rest, api/ws).
from .api.rest import router as status_router
from .api.ws import router as ws_router
from .config import PayloadConfig
from .core.device_link import DeviceLink
from .core.session import SessionManager

# Module logger. House rule: log and exception text is English even though the
# system's user-facing dialogue is Chinese.
logger = logging.getLogger("payload.app")

# Fixed loopback bind address. This is a module constant, NOT a config field, on
# purpose: the device-facing service must never be reachable off-box (invariant R2),
# so the host is not something an operator env override should be able to change.
_LISTEN_HOST = "127.0.0.1"


def create_app(config: PayloadConfig) -> FastAPI:
    """Build the FastAPI application with a DeviceLink bound to its lifespan.

    Args:
        config: the immutable service configuration. It is captured by the lifespan
            closure below and used to build the DeviceLink at startup, so the same
            snapshot governs both the socket setup and the reported listen port.

    Returns:
        A configured FastAPI app. The DeviceLink is created and started when the app
        starts serving and stopped when it shuts down, so importing/building the app
        has no device side effect until it actually runs.

    Splitting this out from main() lets a test build the app with a chosen config
    without starting a uvicorn server -- the test can drive the lifespan and the
    routes against an injected config (for instance one pointed at a fake device) and
    never bind a real port. That testability is the whole reason construction and
    running are separate functions rather than one main().
    """

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        # Startup half: create and own the device link for the whole serving
        # lifetime, and publish it on app.state so request handlers can read its
        # cached state (they never open sockets themselves -- invariant R1).
        # app.state is FastAPI's per-application scratch space; stashing the single
        # DeviceLink there is what gives every request handler a shared handle to the
        # one socket owner without resorting to a module-level global. start() returns
        # immediately (it only spawns the supervisor threads), so a momentarily
        # unreachable device does not stall app startup -- the links come up in the
        # background and the connectivity flags report progress.
        link = DeviceLink(config)
        app.state.device_link = link
        # POST /tts reads the TTS timing knobs to compute est_ms, so the same config
        # snapshot is published alongside the link. Handlers read both from app.state and
        # never re-read the environment, so one authoritative snapshot governs the process.
        app.state.config = config
        # The mode/gate authority (invariants R2/R3): it holds the current Mode, gates the
        # /mic, /play and /tts activities, and owns the deter loop's lifecycle. Published on
        # app.state so REST /mode + /deter and the WS audio routes all consult the one
        # instance. It borrows the link (R1: it never opens a socket itself) and does no
        # device I/O at construction, so it is safe to build here before the links are up.
        session = SessionManager(link, config)
        app.state.session = session
        link.start()
        logger.info(
            "payload-service up: device=%s audio=%d lights=%d listen=%s:%d",
            config.device_host,
            config.port_audio,
            config.port_lights,
            _LISTEN_HOST,
            config.port_payload,
        )
        try:
            # Hand control to the server for the duration of its run. Everything
            # before the yield is startup; everything after is shutdown. uvicorn keeps
            # the coroutine suspended here for the entire serving lifetime.
            yield
        finally:
            # Shutdown half, in order: first quiesce the mode machine, then stop the links.
            # aclose() runs the same teardown as a mode switch -- it stops any running deter
            # loop (which resets the lights and silences the hail) and cooperatively closes
            # every open /mic or /play session -- and it MUST run while the link is still up,
            # so those final device writes actually reach the hardware. Only AFTER that are
            # the reader threads stopped and the sockets flush-closed, so the device's final
            # frame is not lost to an RST (development plan section 9). This finally wraps the
            # yield, so it runs on any exit from serving -- clean shutdown or an exception out
            # of the server -- leaving no deter loop, audio session, open socket, or daemon
            # thread behind in the exiting process.
            await session.aclose()
            link.stop()
            logger.info("payload-service down")

    app = FastAPI(title="payload-service", lifespan=lifespan)
    # Register the control-plane routes (M1 GET /status, M2 POST /lights, M3 POST /tts).
    app.include_router(status_router)
    # Register the data-plane routes (M3 WS /mic record, WS /play hail). They are
    # WebSocket rather than REST because they carry continuous audio streams, not
    # one-shot commands (see api/ws for the traffic-shape rationale).
    app.include_router(ws_router)
    return app


def main() -> None:
    """Process entry point: configure logging, build the app, run uvicorn.

    This is the only function with real-world side effects at import-vs-run boundary:
    it reads the environment, sets up logging, and binds a port, so it runs solely
    under the __main__ guard and never on import. Configuration is read exactly once
    here and threaded through create_app, so the whole process shares a single
    immutable config snapshot rather than re-reading the environment in scattered
    places where values could drift apart.
    """
    # English, machine-greppable log format (timestamp, level, logger, message) to
    # match the house logging convention. basicConfig is called once here, at the true
    # process entry point, so every module logger inherits this handler and format;
    # the library modules themselves never configure logging, which keeps a single
    # consistent format across the whole service and avoids duplicate handlers.
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    # One config snapshot for the whole process lifetime.
    config = PayloadConfig.from_env()
    app = create_app(config)
    # Pass the app OBJECT directly (not an "module:app" import string), which keeps
    # uvicorn's hot-reload off. Reload would fork a second process, and two owners of
    # the device sockets would violate invariant R1 -- so reload must stay disabled.
    # The host is the module-pinned loopback constant, never config.device_host: the
    # bind address of THIS service and the address of the DEVICE are unrelated, and
    # conflating them is exactly the mistake the hard-pin exists to prevent. The port,
    # by contrast, is fine to take from config since it does not affect reachability
    # off-box once the host is fixed to loopback.
    uvicorn.run(app, host=_LISTEN_HOST, port=config.port_payload, log_level="info")


if __name__ == "__main__":
    # Only run the server when executed as a script/module, not on import.
    main()
