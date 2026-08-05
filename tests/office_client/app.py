"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: app.py
Brief: office-client process entry point -- the operator's end of the 功能2 intercom.

Description:
  The process boundary for office-client (plan sections 5.5 and 6): it parses the command
  line, configures logging, and runs one intercom session until the operator quits. All the
  behaviour lives in client.py and audio_io.py; this file exists only to be the place the
  settings are read, logging is set up and an exit status is chosen.

  Why the command line rather than an environment config module like AI_runtime's: the two
  processes are used differently. AI_runtime is started once on the robot and left running,
  so its settings belong in the environment where a supervisor can hold them. office-client
  is an interactive tool an operator starts, points at a robot, and stops -- and the two
  things they change most, which robot and which sound device, are exactly what a flag is
  for. A config module here would mean exporting variables before every run.

  Run it on the office PC with AI_runtime already serving 功能2 on the robot:
      cd /opt/xbrain_v6 && python3 -m tests.office_client.app --server ws://<robot>:18082
  and, on a PC with no microphone, with a file standing in for the operator's voice:
      ... --talk-wav /opt/xbrain_v6/assets/hello_16k.wav
  The -m form is required rather than a path, because this package uses relative imports;
  running the file directly would leave it outside its package and break them.

  ALSA device names: pass a plug-layer name ("default", "plughw:1,0"), never a bare "hw:"
  name -- see audio_io's docstring for what a bare hw: device does silently wrong.

  Exit status is 0 for a session the operator ended (including Ctrl-C, which is a normal
  way to stop) and 1 for one that could not start or was cut short by a fault. Being
  stopped by a human is not an error.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import sys

# Both error types are imported to be CAUGHT, never raised here: each already names its own
# cause -- which URL, which ALSA device, which format -- and an operator reading a bring-up
# log is better served by that one line than by a stack through websockets or asyncio.
from .audio_io import AudioIoError
from .client import IntercomClient, IntercomClientError

# Namespaced under "office_client." like the modules it launches, so verbosity can be
# raised for the audio path without also raising it for the socket.
logger = logging.getLogger("office_client.app")

# Loopback, because that is the one address that is right on any machine: it lets the
# client be smoke-tested against an AI_runtime on the same host. Pointing it at a real
# robot is what --server is for, and is the normal case.
_DEFAULT_SERVER = "ws://127.0.0.1:18082"
# "default" follows whatever the desktop has selected, which is what an operator expects
# from a tool they did not configure.
_DEFAULT_DEVICE = "default"
_DEFAULT_OPEN_TIMEOUT_S = 5.0


def _parse_args() -> argparse.Namespace:
    """Read the command line.

    Returns:
        The parsed arguments.
    """
    parser = argparse.ArgumentParser(
        description="office-client: the office end of the 功能2 PTT intercom",
    )
    parser.add_argument(
        "--server", default=_DEFAULT_SERVER,
        help=f"intercom websocket URL on the robot (default: {_DEFAULT_SERVER})",
    )
    parser.add_argument(
        "--speaker-device", default=_DEFAULT_DEVICE,
        help=f"ALSA playback device for the listen direction (default: {_DEFAULT_DEVICE})",
    )
    parser.add_argument(
        "--mic-device", default=_DEFAULT_DEVICE,
        help=f"ALSA capture device for the talk direction (default: {_DEFAULT_DEVICE})",
    )
    parser.add_argument(
        "--talk-wav", default=None,
        help="send this 16 kHz mono 16-bit wav instead of the microphone, replayed from "
             "the start on every press; required on a PC with no microphone",
    )
    parser.add_argument(
        "--open-timeout", type=float, default=_DEFAULT_OPEN_TIMEOUT_S,
        help=f"seconds allowed to connect and to open the audio devices "
             f"(default: {_DEFAULT_OPEN_TIMEOUT_S})",
    )
    return parser.parse_args()


def main() -> int:
    """Configure logging and run one intercom session.

    Returns:
        0 if the operator ended the session, 1 if it could not start or a fault ended it.

    Logging is configured HERE, at the true process entry point, so both modules inherit one
    handler and one format; they never call basicConfig themselves, which is what keeps the
    format consistent and avoids duplicate handlers. The format matches AI_runtime and the
    services so a bring-up run can be read across all the logs at once.
    """
    args = _parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    # The server URL and the two devices are echoed at startup because pointing the client
    # at the wrong robot or the wrong soundcard are the two most common faults on this end,
    # and both otherwise present identically to a working client in a silent room.
    logger.info(
        "office-client starting: server=%s speaker=%s talk=%s",
        args.server,
        args.speaker_device,
        args.talk_wav if args.talk_wav is not None else f"mic {args.mic_device}",
    )
    client = IntercomClient(
        url=args.server,
        speaker_device=args.speaker_device,
        mic_device=args.mic_device,
        talk_wav=args.talk_wav,
        open_timeout_s=args.open_timeout,
    )
    try:
        # asyncio.run owns the event loop for the whole process: one session per run, so
        # there is no loop to reuse and nothing to keep alive after the session ends.
        asyncio.run(client.run())
    except KeyboardInterrupt:
        # A normal way to stop an interactive tool, so it is a clean exit. The session's own
        # teardown has already stopped any transmission and reaped the audio processes.
        logger.info("interrupted by operator")
    except (AudioIoError, IntercomClientError) as exc:
        # One line, because the message already names the device or the URL and the reason.
        print(f"intercom failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    # Only run when executed as a module, never on import.
    sys.exit(main())
