#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: app.py
Brief: asr-service entry point -- loads the model once and serves the OpenAI ASR endpoint.

Description:
  The wiring layer for asr-service. Like services/payload/app.py it is intentionally thin:
  no inference code, no audio parsing, no request handling of its own. It does exactly
  three things -- read configuration from the environment, build the single Recognizer for
  the lifetime of the ASGI app, and mount the REST router -- so that every interesting
  piece stays unit-testable on its own and the only decision living here is how the pieces
  are connected and for how long each one lives.

  Lifetime coupling (why a lifespan rather than a module-level Recognizer): loading the
  zipformer takes seconds and hundreds of megabytes, and it must happen once per process,
  before the first request, but NOT as an import side effect -- importing this module to
  inspect the app, or to run a unit test, must not load a 300 MB model. A FastAPI lifespan
  brackets precisely the serving window, so "the model is loaded" and "the server accepts
  requests" become the same interval by construction.

  Fail-fast startup: the model load happens inside the lifespan BEFORE the yield, so a
  missing model file, an unreadable hotwords file or an absent sherpa-onnx aborts startup
  with the specific cause. The alternative -- lazy-loading on first request -- would let
  the process bind its port and look healthy while being incapable of transcribing
  anything, and AI_runtime would discover that only mid-dialogue.

  Loopback bind: like payload-service, the listen host is HARD-PINNED to 127.0.0.1 here
  rather than taken from config. asr-service has no authentication and the only intended
  client is the local AI_runtime on the same Orin, so the safe bind is deliberately not
  something an operator can misconfigure -- it is not configurable at all.

  Run (from the /opt/xbrain_v6 root on the Orin, using its python3.10):
      python3 -m services.asr.app
  See development plan sections 5.3 and 16.
"""
from __future__ import annotations

import logging
from contextlib import asynccontextmanager
from typing import AsyncIterator

import uvicorn
from fastapi import FastAPI

from .api.rest import router as asr_router
from .config import AsrConfig
from .core.recognizer import Recognizer

# Module logger. House rule: log text is English even though the recognized content is
# Chinese.
logger = logging.getLogger("asr.app")

# Fixed loopback bind address. A module constant, NOT a config field, on purpose: see the
# loopback-bind note in the module docstring.
_LISTEN_HOST = "127.0.0.1"


def create_app(config: AsrConfig) -> FastAPI:
    """Build the FastAPI application with a Recognizer bound to its lifespan.

    Args:
        config: the immutable service configuration. It is captured by the lifespan
            closure below and used to build the Recognizer at startup, so one snapshot
            governs both the model that gets loaded and the reported listen port.

    Returns:
        A configured FastAPI app. The model is loaded when the app starts serving and
        released when it shuts down, so building the app has no inference side effect
        until it actually runs.

    Splitting this out from main() lets a test build the app against a chosen config --
    for instance one pointing at a small fixture model -- and drive the routes without
    binding a real port. That testability is the whole reason construction and running are
    separate functions rather than one main().
    """

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        # Startup half: load the model once and publish it on app.state, FastAPI's
        # per-application scratch space, so every handler shares the one Recognizer
        # without a module-level global. This call blocks the startup coroutine for
        # several seconds by design -- nothing should be served until it succeeds.
        recognizer = Recognizer(config)
        app.state.recognizer = recognizer
        # The config snapshot is published alongside it because the request path needs one
        # value from it -- the model's minimum decodable duration -- and reading it from
        # app.state keeps the handler free of a module-level global while guaranteeing it
        # is the SAME snapshot that built the recognizer it is guarding.
        app.state.config = config
        logger.info(
            "asr-service up: model=%s hotwords=%d conf=%s listen=%s:%d",
            recognizer.model_name,
            recognizer.hotword_count,
            "yes" if recognizer.reports_confidence else "no",
            _LISTEN_HOST,
            config.port_asr,
        )
        try:
            # Hand control to the server for the duration of its run. Everything before
            # the yield is startup, everything after is shutdown; uvicorn keeps this
            # coroutine suspended here for the entire serving lifetime.
            yield
        finally:
            # Shutdown half. There is nothing to close: the recognizer owns no socket, no
            # thread of its own and no temporary file (the hotwords copy is removed during
            # construction), so dropping the reference is the whole teardown and the
            # process exit reclaims the model memory. The finally block still exists so
            # the "down" line is logged on any exit path, clean or exceptional.
            app.state.recognizer = None
            app.state.config = None
            logger.info("asr-service down")

    app = FastAPI(title="asr-service", lifespan=lifespan)
    # One router: GET /status and the OpenAI-compatible POST /v1/audio/transcriptions.
    app.include_router(asr_router)
    return app


def main() -> None:
    """Process entry point: configure logging, build the app, run uvicorn.

    This is the only function with real side effects at the import-vs-run boundary -- it
    reads the environment, sets up logging and binds a port -- so it runs solely under the
    __main__ guard and never on import. Configuration is read exactly once here and
    threaded through create_app, so the process shares a single immutable snapshot rather
    than re-reading the environment in scattered places where values could drift apart.
    """
    # English, machine-greppable log format, matching payload-service so a combined
    # capture from both services reads uniformly. basicConfig is called once, here at the
    # true process entry point, so every module logger inherits this handler and format;
    # library modules never configure logging, which avoids duplicate handlers.
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    # One config snapshot for the whole process lifetime.
    config = AsrConfig.from_env()
    app = create_app(config)
    # Pass the app OBJECT (not an "module:app" import string), which keeps uvicorn's
    # hot-reload off. Reload would fork a second process, and a second process would load
    # a second copy of the model -- gigabytes of duplicated weights on a box that is also
    # running llama-server. The host is the module-pinned loopback constant; only the port
    # comes from config, which is safe once the bind address is fixed.
    uvicorn.run(app, host=_LISTEN_HOST, port=config.port_asr, log_level="info")


if __name__ == "__main__":
    # Only run the server when executed as a script/module, not on import.
    main()
