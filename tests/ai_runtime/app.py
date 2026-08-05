"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: app.py
Brief: AI_runtime process entry point -- runs either 功能1 dialogue or 功能2 intercom.

Description:
  The launcher for the two modes AI_runtime drives (plan section 6): 功能1 near-field AI
  dialogue and 功能2 far-field half-duplex intercom. It configures logging, takes one
  config snapshot from the environment, and runs the selected one until it ends. All the
  behaviour lives in turn_loop / intercom and the clients around them; this file exists
  only to be the process boundary -- the place the environment is read, logging is set up
  and an exit status is chosen.

  One entry point with a selector, rather than two nearly identical launchers, because the
  only thing that differs between them is which object gets run: the logging setup, the
  config snapshot and the operational-fault table are identical, and duplicating them
  would mean two places to update every time a client grows a new error type. It is also
  honest about invariant R3 -- the selector makes it a choice of ONE mode, which is what
  the invariant says, whereas two launchers would invite running both at once.

  功能3 驱离 is deliberately NOT a choice here: its loop runs inside payload-service and is
  armed with POST /deter, so AI_runtime has no part to play in it.

  Run it on the Orin, with payload-service, asr-service and llama-server already up:
      cd /opt/xbrain_v6 && python3 -m tests.ai_runtime.app            # 功能1
      cd /opt/xbrain_v6 && python3 -m tests.ai_runtime.app --function 2   # 功能2
  The -m form is required rather than a path, because this package uses relative imports;
  running the file directly would leave it outside its package and break them.

  Exit status is chosen so a supervisor or a shell can act on it: 0 for a session that
  ended cleanly (including Ctrl-C, which is the normal way an operator stops a bring-up
  run), and 1 for a session that could not start or was cut short by a fault. A stopped
  session is not an error just because a human stopped it.

  SIGTERM is routed onto the Ctrl-C path deliberately, and 功能2 is why. Its intercom
  serves forever, so a signal is the ONLY way it ever ends -- and a run started with nohup
  can only be signalled, never interrupted from a keyboard. Left on the default handler,
  SIGTERM kills the process outright: nothing unwinds, the mode restore in IntercomServer
  never runs, and the payload is left parked in func2 blocking whatever mode comes next
  (R3). Measured on the Orin before this was added -- the mode did stay func2 after every
  stop. Raising KeyboardInterrupt instead gives the two signals one shutdown path, which is
  also the one that already gets exercised every time an operator presses Ctrl-C.

  Why the operational faults are printed as one line rather than a traceback: every one
  of them names its own cause -- which service, which URL, which status -- and an
  operator reading a bring-up log is better served by that line than by a stack through
  requests. A fault that is NOT one of those types is a bug, and it keeps its traceback.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
from types import FrameType
from typing import Optional

# The client error types are imported to be CAUGHT, never raised here: they are exactly
# the faults that deserve a one-line operator message instead of a traceback (module
# docstring). AiRuntimeConfigError is separate because it is a startup fault -- the
# environment is wrong, the process never got as far as talking to anything.
from .asr_client import AsrClientError
from .config import AiRuntimeConfig, AiRuntimeConfigError
# IntercomError is caught for the same reason as the client errors: a malformed control
# message from office-client names its own fix and does not need a stack through websockets.
from .intercom import IntercomError, IntercomServer
from .llm_client import LlmClientError
from .local_mic import LocalMicError
from .payload_client import PayloadClientError

# The two behaviours this file can launch. Everything else here is process setup.
from .turn_loop import TurnLoop

# Caught for the same reason as the client errors, though it is a startup fault rather
# than a service one: a silero model that is not where the config says it is names its own
# fix, and an operator does not need a stack through onnxruntime to apply it.
from .vad import VadError

# Namespaced under "ai_runtime." like the clients and the loop, so an operator can raise
# or lower verbosity for one part of the pipeline without silencing the rest.
logger = logging.getLogger("ai_runtime.app")


def _stop_on_sigterm(signum: int, frame: Optional[FrameType]) -> None:
    """Turn SIGTERM into the interruption the shutdown path is already built around.

    Args:
        signum: the signal received, unused -- only SIGTERM is routed here.
        frame: the interrupted stack frame, unused.

    Raises:
        KeyboardInterrupt: always, which asyncio.run unwinds by cancelling the session's
            task and letting its finally blocks run to completion.

    See the module docstring for why this exists: without it a signalled 功能2 run leaves
    the payload in func2.
    """
    raise KeyboardInterrupt


async def _run(config: AiRuntimeConfig, function: int) -> None:
    """Run the selected function to completion.

    Args:
        config: the process configuration.
        function: 1 for the 功能1 dialogue loop, 2 for the 功能2 intercom server.

    Both objects are constructed INSIDE this coroutine rather than passed in, because each
    holds asyncio state (tasks, sockets) bound to the running loop and so must not be
    created before asyncio.run starts one.
    """
    if function == 1:
        await TurnLoop(config).run()
    else:
        await IntercomServer(config).run()


def main() -> int:
    """Configure logging, build the config, and run the selected function.

    Returns:
        0 if the session ended cleanly or the operator interrupted it, 1 if the
        configuration was rejected or a service fault ended the session.

    Logging is configured HERE, at the true process entry point, so every module logger
    inherits one handler and one format; the library modules never call basicConfig
    themselves, which is what keeps the format consistent and avoids duplicate handlers.
    The format matches payload-service and asr-service so a bring-up run can be read
    across all three logs at once.
    """
    parser = argparse.ArgumentParser(description="AI_runtime: 功能1 dialogue or 功能2 intercom")
    # choices, not a free integer: an unrecognised function must fail at the command line
    # rather than fall through to a default and run the wrong mode on real hardware.
    parser.add_argument(
        "--function", type=int, choices=(1, 2), default=1,
        help="1 = near-field AI dialogue (default), 2 = far-field PTT intercom",
    )
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    # Installed before anything is opened, so a signal arriving during startup is handled
    # the same way as one arriving mid-session.
    signal.signal(signal.SIGTERM, _stop_on_sigterm)
    try:
        # One immutable config snapshot for the whole process, rather than re-reading the
        # environment in scattered places where values could drift apart.
        config = AiRuntimeConfig.from_env()
    except AiRuntimeConfigError as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 1
    # The three URLs are echoed at startup because pointing a client at the wrong port is
    # the most common bring-up fault by a wide margin, and it otherwise surfaces much
    # later as a connection error naming a port the operator has to go look up. Printed
    # here, the line can be compared directly against the three services' own banners.
    # The mic source is echoed alongside them because it decides where the audio comes
    # from, and a run listening to the wrong microphone looks exactly like a run nobody is
    # talking to.
    logger.info(
        "ai_runtime starting: function=%d payload=%s asr=%s llm=%s mic=%s",
        args.function,
        config.payload_url,
        config.asr_url,
        config.llm_url,
        config.mic_source,
    )
    try:
        # asyncio.run owns the event loop for the whole process: one session per run, so
        # there is no loop to reuse and nothing to keep alive after the session ends.
        asyncio.run(_run(config, args.function))
    except KeyboardInterrupt:
        # The normal way an operator ends a bring-up run, so it is a clean exit. The loop's
        # own finally has already closed the mic socket and restored the mode by the time
        # this is caught.
        logger.info("interrupted by operator")
    except (
        AsrClientError,
        IntercomError,
        LlmClientError,
        LocalMicError,
        PayloadClientError,
        VadError,
    ) as exc:
        # A service or device fault that ended the whole session (as opposed to a single
        # turn, which turn_loop absorbs). One line, because the message already names the
        # service and the reason.
        print(f"session failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    # Only run when executed as a module, never on import.
    sys.exit(main())
