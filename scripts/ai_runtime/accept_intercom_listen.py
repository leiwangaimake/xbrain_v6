"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: accept_intercom_listen.py
Brief: Accept the 功能2 listen direction on real hardware, without making any sound.

Description:
  功能2 has two directions and only one of them can be checked in an empty office. The talk
  direction ends at the robot's loudspeaker, so exercising it means the 三合一 says something
  out loud; the listen direction ends at a socket, so it can be measured with nobody there.
  This script does the second one and says so plainly, because a PASS here must not be read
  as 功能2 having been accepted.

  It is a CLIENT of the intercom, not a second copy of one. It speaks the published socket
  -- header, {"ptt": ...}, PCM -- and nothing else, so what it proves is what an office PC
  would experience. It deliberately does not use tests/office_client, because that module
  opens the office soundcard, and playing the robot's microphone through a speaker in the
  same room is how a feedback loop starts.

  Three things are checked, and the third is the one that is easy to leave out:

    1. audio arrives at all      -- the socket, the mode, the microphone, the gate
    2. it arrives at realtime    -- the number issue A1 is about: the payload's own [40]
                                    uplink runs at 0.235x, and the whole reason the test
                                    microphone was moved onto the dog is that this path
                                    does not
    3. it STOPS when released    -- a gate that never closes would satisfy checks 1 and 2
                                    perfectly and leave the operator transmitting to a
                                    channel they believe is shut

  The level is reported but never asserted on. Issue E8 has the USB microphone's input
  floating, so its content is garbage while every software metric stays green -- exactly the
  fault a level threshold would fail to catch and a person listening would notice at once.
  Printing it and refusing to judge it is the honest option.

  Run it on the Orin with payload-service up and AI_runtime already serving 功能2:
      cd /opt/xbrain_v6 && python3 -m tests.ai_runtime.app --function 2 &
      cd /opt/xbrain_v6 && python3 -m scripts.ai_runtime.accept_intercom_listen
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
from typing import Any, Dict, List, Tuple

import numpy as np
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed, WebSocketException

# Where AI_runtime's intercom listens by default (config.intercom_port). Loopback, because
# this script runs on the Orin next to it.
_DEFAULT_URL = "ws://127.0.0.1:18082"

# The format the intercom must announce (plan section 7). Checked rather than adapted to:
# audio at the wrong rate is the one audio fault where every byte count still adds up.
_ENCODING = "s16le"
_SAMPLE_RATE = 16000
_CHANNELS = 1
_BYTES_PER_SAMPLE = 2

# How far from realtime the uplink may run and still pass. Wide enough not to fail on a
# scheduling hiccup, far narrower than the 0.235x that issue A1 is about -- the point is to
# catch a path that is fundamentally not keeping up, not to grade jitter.
_RT_MIN = 0.90
_RT_MAX = 1.10

# Frames already on the wire when the release is sent are not a gate failure, so the first
# moment after it is ignored. Everything after that must be silence.
_RELEASE_GRACE_S = 0.3
_GATE_CHECK_S = 1.0

# A bring-up check, not a soak test: an upper bound so a mistyped argument cannot leave the
# payload held in func2 for an afternoon.
_MAX_RUN_S = 60.0

_OPEN_TIMEOUT_S = 5.0
# Long enough that a genuinely slow path is measured rather than timed out -- a stall is
# what check 2 exists to find, so it must be reported as a bad rate and not as an error.
_RECV_TIMEOUT_S = 5.0


class AcceptError(RuntimeError):
    """Raised when the acceptance cannot be carried out.

    House rule bans bare Exception. This is distinct from a FAILED check: it means the run
    could not reach a verdict -- the intercom was not up, or spoke a format this script does
    not -- which is a different thing from the hardware being wrong.
    """


async def _receive_header(connection: Any) -> Dict[str, Any]:
    """Read the intercom's format header and check it against what this script assumes.

    Args:
        connection: the open websocket.

    Returns:
        The header, with frame_bytes usable.

    Raises:
        AcceptError: if the channel is busy, closed early, or announces another format.
    """
    try:
        raw = await asyncio.wait_for(connection.recv(), _RECV_TIMEOUT_S)
    except asyncio.TimeoutError as exc:
        raise AcceptError("intercom accepted the connection but sent no format header") from exc
    except ConnectionClosed as exc:
        # 1013 is the intercom's "already in use": another client holds the channel, which
        # is a reason to wait rather than a reason to investigate.
        code = exc.rcvd.code if exc.rcvd is not None else 1006
        if code == 1013:
            raise AcceptError("the intercom is already in use by another client") from exc
        raise AcceptError(f"intercom closed before sending its header (code {code})") from exc
    if not isinstance(raw, str):
        raise AcceptError("intercom sent audio before its format header")
    header = json.loads(raw)
    for key, want in (
        ("encoding", _ENCODING),
        ("sample_rate", _SAMPLE_RATE),
        ("channels", _CHANNELS),
    ):
        if header.get(key) != want:
            raise AcceptError(f"intercom announces {key}={header.get(key)!r}, expected {want!r}")
    return header


async def _set_ptt(connection: Any, state: str) -> None:
    """Ask the intercom for a PTT state."""
    await connection.send(json.dumps({"ptt": state}))


async def _listen_window(connection: Any, seconds: float) -> Tuple[int, float, List[float]]:
    """Collect audio for a while and report how much arrived, how fast, and how loud.

    Args:
        connection: the open websocket, already in listen.
        seconds: how long to collect for.

    Returns:
        A (frames, elapsed, levels) triple, where levels holds the RMS of each frame.

    Raises:
        AcceptError: if the stream stalls or the intercom drops the connection.

    The clock starts at the FIRST frame, not when listen was requested. The setup in
    between -- the control message crossing, the server applying it -- is real latency but
    it is not part of the stream's rate, and including it would report a path that keeps up
    perfectly as running slow.
    """
    frames = 0
    levels: List[float] = []
    started = None
    while True:
        if started is not None and time.monotonic() - started >= seconds:
            break
        try:
            message = await asyncio.wait_for(connection.recv(), _RECV_TIMEOUT_S)
        except asyncio.TimeoutError as exc:
            raise AcceptError(
                f"no audio for {_RECV_TIMEOUT_S:.0f}s while listening; the robot's "
                f"microphone has stopped"
            ) from exc
        except ConnectionClosed as exc:
            raise AcceptError("intercom closed the connection while listening") from exc
        if not isinstance(message, bytes):
            # The server sends one text frame and it has already been read; anything else
            # is a protocol change this script has not been told about.
            raise AcceptError(f"unexpected control message while listening: {message[:200]}")
        if started is None:
            started = time.monotonic()
        frames += 1
        samples = np.frombuffer(message, dtype=np.int16).astype(np.float64)
        levels.append(float(np.sqrt(np.mean(samples * samples))) if samples.size else 0.0)
    return frames, time.monotonic() - started, levels


async def _drain_after_release(connection: Any) -> int:
    """Count frames that arrive after the release, past the in-flight grace period.

    Args:
        connection: the open websocket, already released to idle.

    Returns:
        How many frames arrived that should not have.

    Frames already on the wire when the release was sent are not a gate failure -- the
    server cannot recall them -- so the first moment is discarded and only what comes after
    is counted.
    """
    deadline = time.monotonic() + _RELEASE_GRACE_S
    while time.monotonic() < deadline:
        try:
            await asyncio.wait_for(connection.recv(), deadline - time.monotonic())
        except (asyncio.TimeoutError, ConnectionClosed):
            break
    stray = 0
    deadline = time.monotonic() + _GATE_CHECK_S
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return stray
        try:
            message = await asyncio.wait_for(connection.recv(), remaining)
        except asyncio.TimeoutError:
            # Nothing arrived for the rest of the window, which is the passing outcome.
            return stray
        except ConnectionClosed:
            return stray
        if isinstance(message, bytes):
            stray += 1


async def _run(url: str, seconds: float) -> bool:
    """Carry out the acceptance and print what happened.

    Args:
        url: the intercom websocket URL.
        seconds: how long to listen for.

    Returns:
        True if every check passed.

    Raises:
        AcceptError: if a verdict could not be reached.
    """
    try:
        connection = await connect(url, compression=None, open_timeout=_OPEN_TIMEOUT_S)
    except (OSError, WebSocketException) as exc:
        raise AcceptError(
            f"cannot reach the intercom at {url}: {exc}; is "
            f"'python3 -m tests.ai_runtime.app --function 2' running?"
        ) from exc
    async with connection:
        header = await _receive_header(connection)
        print(f"header: {header}")
        await _set_ptt(connection, "listen")
        frames, elapsed, levels = await _listen_window(connection, seconds)
        # Always release, even on the way to a failure: leaving the intercom transmitting
        # would hold the robot's microphone open after this script has gone.
        await _set_ptt(connection, "idle")
        stray = await _drain_after_release(connection)

    frame_bytes = header["frame_bytes"]
    audio_s = frames * frame_bytes / (_BYTES_PER_SAMPLE * _CHANNELS * _SAMPLE_RATE)
    realtime = audio_s / elapsed if elapsed > 0 else 0.0
    peak = max(levels) if levels else 0.0
    median = float(np.median(levels)) if levels else 0.0
    print(
        f"listened {elapsed:.2f}s: {frames} frames = {audio_s:.2f}s of audio "
        f"({realtime:.3f}x realtime)"
    )
    # Reported in dBFS as well because a bare RMS count means nothing without the scale it
    # sits on, and dBFS is what a person comparing this against a recording will expect.
    print(
        f"level: median rms {median:.0f} ({_dbfs(median):.1f} dBFS), "
        f"peak rms {peak:.0f} ({_dbfs(peak):.1f} dBFS)  [not asserted on -- see issue E8]"
    )
    print(f"after release: {stray} stray frames in {_GATE_CHECK_S:.1f}s")

    ok = True
    if frames == 0:
        print("FAIL: no audio arrived on the listen direction")
        ok = False
    if not _RT_MIN <= realtime <= _RT_MAX:
        print(f"FAIL: uplink ran at {realtime:.3f}x realtime, expected {_RT_MIN}-{_RT_MAX}")
        ok = False
    if stray:
        print("FAIL: the gate did not close -- audio kept arriving after the release")
        ok = False
    if ok:
        print("PASS: the listen direction carries realtime audio and stops when released")
        print("NOTE: the talk direction was NOT exercised; it needs the robot's loudspeaker")
    return ok


def _dbfs(rms: float) -> float:
    """Convert an int16 RMS to dBFS, with silence reported as a floor rather than -inf."""
    if rms <= 0:
        return -120.0
    return 20.0 * math.log10(rms / 32768.0)


def main() -> int:
    """Parse arguments and run the acceptance.

    Returns:
        0 if every check passed, 1 otherwise.
    """
    parser = argparse.ArgumentParser(
        description="Accept the 功能2 listen direction (no sound is produced)",
    )
    parser.add_argument("--url", default=_DEFAULT_URL, help=f"intercom URL (default: {_DEFAULT_URL})")
    parser.add_argument("--seconds", type=float, default=10.0, help="how long to listen")
    args = parser.parse_args()
    # Clamped rather than rejected: the argument is a duration, and the safe reading of a
    # too-large one is "as long as this tool is willing to hold the channel".
    seconds = min(args.seconds, _MAX_RUN_S)
    try:
        return 0 if asyncio.run(_run(args.url, seconds)) else 1
    except AcceptError as exc:
        print(f"cannot run: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
