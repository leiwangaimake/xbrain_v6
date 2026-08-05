"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: accept_deter.py
Brief: On-device acceptance for 功能3 驱离 -- arm the deterrent, then prove it went away.

Description:
  Drives the real GZH-2 through payload-service's published HTTP boundary (POST /mode,
  POST /deter, GET /status) rather than through DeviceLink directly, because the thing
  section 13 accepts is the SERVICE's behaviour: that a deter run raised by /deter is
  fully cleared when it is disarmed. Going through HTTP exercises the mode gate, the
  SessionManager teardown and the controller's reset in the same arrangement an operator
  would use; driving DeviceLink directly would skip exactly the layers most likely to
  leave the device lit.

  The acceptance has TWO halves and the first one is the one that is easy to forget:

    1. The deterrent must actually be RAISED. A /deter that silently did nothing would
       satisfy "everything is off afterwards" perfectly. So the run polls GET /status
       while the loop is armed and requires the searchlight to have been seen ON and the
       red/blue mode to have been seen non-zero. Without this half the test is a
       tautology -- the same "green for the wrong reason" trap recorded as C3 in
       docs/ISSUE.md.
    2. The deterrent must then be fully CLEARED (section 13: 灯全灭收尾). After disarming
       and returning to idle, the status readback must show searchlight off, red/blue 0,
       and mode idle. The check reads the device's own 0x25 report through /status, not
       our request history, so it is the hardware answering rather than our own optimism.

  Teardown runs in a finally block and is the whole point of the script, so it also runs
  when the arming half fails or the operator interrupts. An interrupted run that left the
  device screaming and strobing would be a worse outcome than the failure it reported.

  Strobe is deliberately not asserted: GET /status does not surface it (rest.py's
  LightsView carries searchlight/bright/redblue only), and section 13 names the
  searchlight b7 and the red/blue reset. Asserting a field the service does not publish
  would mean guessing, so the script checks what the device actually reports.

  IT MAKES NOISE AND LIGHT. The siren level defaults far below the deter default because
  this is normally run in an office; --siren-level scales the synthesised waveform, which
  is the correct knob (changing the DEVICE volume would also change TTS, see ISSUE A6).

  Run it on the Orin, with payload-service already up:

      cd /opt/xbrain_v6 && python3 -m scripts.payload.accept_deter

  Exit status is 0 only if the deterrent was both observed raised and observed cleared.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Any, Dict, List, Optional

import requests

# payload-service binds the loopback on purpose (its own address is a module constant, not
# config, so it cannot be confused with the DEVICE address -- see ISSUE E6).
_DEFAULT_BASE = "http://127.0.0.1:18080"
# Short per-request timeout: every route here either touches memory or does one socket
# write, so anything slower means the service is wedged and we want to know immediately
# rather than hang with the siren running.
_HTTP_TIMEOUT_S = 10.0
# The device pushes a 0x25 status frame every ~500 ms, so this is the fastest poll that
# can yield new information; polling faster would only re-read the same cached snapshot.
_POLL_INTERVAL_S = 0.5
# After disarming, wait for at least a couple of fresh 0x25 reports before believing the
# "lights are off" readback -- a snapshot taken immediately would still be the armed one.
_SETTLE_S = 2.0
# Hard ceiling on how long the deterrent may run, whatever --seconds asks for. This script
# is normally started by hand and left, so a typo of 600 must not leave a siren going.
_MAX_RUN_S = 60.0


class AcceptError(RuntimeError):
    """Raised when the service or the device did not do what acceptance requires.

    A dedicated type (house rule bans bare Exception) so main() can print one clean
    failure line for an acceptance fault while letting a genuine bug traceback normally.
    """


def _post(base: str, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
    """POST one control request and return the decoded body, or raise AcceptError.

    Args:
        base: service base URL.
        path: route path, e.g. "/mode".
        body: JSON body to send.

    Returns:
        The decoded JSON response.

    Raises:
        AcceptError: on a transport failure or any non-2xx status. The service's own
            detail string is included because its 409/503 taxonomy (rest.py) already says
            precisely what went wrong -- re-wording it here would lose information.
    """
    try:
        response = requests.post(f"{base}{path}", json=body, timeout=_HTTP_TIMEOUT_S)
    except requests.RequestException as exc:
        raise AcceptError(f"POST {path} failed to reach the service: {exc}") from exc
    if response.status_code >= 300:
        raise AcceptError(f"POST {path} -> HTTP {response.status_code}: {response.text}")
    return response.json()


def _status(base: str) -> Dict[str, Any]:
    """Read GET /status, or raise AcceptError.

    Kept separate from _post rather than generalised into one _request helper: the two
    have different bodies, different failure meanings, and are called in different phases,
    and a shared helper would only be a parameter-shuffling indirection.
    """
    try:
        response = requests.get(f"{base}/status", timeout=_HTTP_TIMEOUT_S)
    except requests.RequestException as exc:
        raise AcceptError(f"GET /status failed to reach the service: {exc}") from exc
    if response.status_code >= 300:
        raise AcceptError(f"GET /status -> HTTP {response.status_code}: {response.text}")
    return response.json()


def _lights(status: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Pull the lights block out of a status body, which may legitimately be null.

    /status reports null when no valid 0x25 frame has arrived yet (rest.py), so callers
    must handle absence rather than assume a dict -- treating null as "all off" would let
    a device that never reported anything pass the teardown check.
    """
    return status.get("lights")


def _watch_armed(base: str, seconds: float) -> List[Dict[str, Any]]:
    """Poll /status while the deterrent runs and return every lights snapshot seen.

    Args:
        base: service base URL.
        seconds: how long to watch.

    Returns:
        The list of non-null lights blocks observed, in order.

    Collecting them all (rather than testing one late sample) means a deterrent that came
    up and then dropped out is still visible in the record, and it gives the operator a
    printed trace to compare against what they could see and hear in the room.
    """
    snapshots: List[Dict[str, Any]] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        lights = _lights(_status(base))
        if lights is not None:
            snapshots.append(lights)
            print(
                f"  armed t+{seconds - (deadline - time.monotonic()):4.1f}s  "
                f"searchlight={lights['searchlight']} "
                f"bright={lights['bright']} redblue={lights['redblue']}",
                flush=True,
            )
        time.sleep(_POLL_INTERVAL_S)
    return snapshots


def _check_raised(snapshots: List[Dict[str, Any]]) -> None:
    """Require that the deterrent was actually observed on the hardware.

    Raises:
        AcceptError: if no status arrived at all, or if the searchlight was never seen lit
            or the red/blue pattern never seen running.

    This is the half of the acceptance that stops the teardown check from being vacuous
    (see the module docstring). "Never observed on" is reported separately from "no status
    at all" because they point at different faults: a deter loop that did not arm versus a
    lights link that is not reporting.
    """
    if not snapshots:
        raise AcceptError("no lights status arrived while armed -- 8529 link not reporting")
    # any(), not the last sample: the deterrent only has to have been genuinely raised at
    # some point during the window for the arming half to be satisfied.
    if not any(snap["searchlight"] for snap in snapshots):
        raise AcceptError("searchlight was never reported ON while the deterrent was armed")
    if not any(snap["redblue"] for snap in snapshots):
        raise AcceptError("red/blue was never reported running while the deterrent was armed")


def _check_cleared(base: str) -> None:
    """Require the section 13 teardown state: lights off, red/blue reset, mode idle.

    Raises:
        AcceptError: if the device still reports a lit searchlight or a running red/blue
            pattern, or the service is not back in idle.

    Waits _SETTLE_S first so the snapshot being judged is a 0x25 report the device sent
    AFTER the reset frames, not the cached armed one. Brightness is deliberately not
    asserted to be 0: the device retains the brightness register while the lamp is off
    (ISSUE B5), so bright=30 with searchlight=false is the correct cleared state and
    demanding 0 would fail a device that behaved perfectly.
    """
    time.sleep(_SETTLE_S)
    status = _status(base)
    lights = _lights(status)
    if lights is None:
        raise AcceptError("no lights status after teardown -- cannot confirm the lamp is off")
    print(
        f"  cleared        searchlight={lights['searchlight']} "
        f"bright={lights['bright']} redblue={lights['redblue']} mode={status['mode']}",
        flush=True,
    )
    if lights["searchlight"]:
        raise AcceptError("searchlight still ON after teardown (section 13: 灯全灭收尾)")
    if lights["redblue"]:
        raise AcceptError(f"red/blue still running (mode {lights['redblue']}) after teardown")
    if status["mode"] != "idle":
        raise AcceptError(f"service left in mode {status['mode']!r}, expected 'idle'")


def _teardown(base: str) -> None:
    """Disarm the deterrent and return the service to idle, best effort.

    Each step is attempted independently and its failure only logged, because this runs in
    a finally block: the second step must still be tried if the first fails, and an
    exception raised here would replace the real reason the run is ending. A 409 is
    expected and harmless when the run never got as far as entering deter mode.
    """
    for path, body in (("/deter", {"on": False}), ("/mode", {"mode": "idle"})):
        try:
            _post(base, path, body)
        except AcceptError as exc:
            # Logged, not raised: teardown is best-effort by design (see docstring).
            print(f"  teardown: POST {path} did not apply: {exc}", file=sys.stderr)


def main() -> int:
    """Arm the deterrent for a bounded window, then prove the device went quiet and dark.

    Returns:
        0 only if the deterrent was observed raised AND observed cleared; 1 on any
        acceptance failure or an unreachable service.
    """
    parser = argparse.ArgumentParser(description="on-device acceptance for 功能3 deter")
    parser.add_argument("--base", default=_DEFAULT_BASE, help="payload-service base URL")
    # One deter cycle is a 6 s siren plus the spoken warnings, so 12 s covers a full cycle
    # including the first warning -- enough to accept the behaviour, short enough to be
    # tolerable in a shared office.
    parser.add_argument("--seconds", type=float, default=12.0, help="how long to stay armed")
    # Far below the 0.45 deter default: this is normally run indoors. It scales the
    # synthesised waveform, which is the right knob (device volume would also move TTS).
    parser.add_argument("--siren-level", type=float, default=0.12, help="siren amplitude 0..1")
    parser.add_argument("--redblue", type=int, default=1, help="red/blue pattern 1..16")
    parser.add_argument("--tts-reps", type=int, default=1, help="warnings spoken per cycle")
    args = parser.parse_args()

    # Clamp rather than reject: an over-long request is almost always a typo, and the safe
    # response to a typo that would leave a siren running is to run the shorter time.
    seconds = min(args.seconds, _MAX_RUN_S)
    if seconds != args.seconds:
        print(f"capping run at {_MAX_RUN_S:.0f}s (asked for {args.seconds:.0f}s)")

    base = args.base.rstrip("/")
    try:
        start = _status(base)
    except AcceptError as exc:
        print(f"cannot reach payload-service: {exc}", file=sys.stderr)
        return 1
    # Refuse to start from a non-idle mode instead of stomping it: another mode being
    # active means something else owns the device right now (invariant R3), and taking it
    # away would corrupt whatever that was.
    if start["mode"] != "idle":
        print(f"service is in mode {start['mode']!r}, expected 'idle' -- not taking over",
              file=sys.stderr)
        return 1

    print(f"THE DEVICE WILL SIREN AND STROBE FOR {seconds:.0f}s "
          f"(siren_level={args.siren_level})", flush=True)
    try:
        _post(base, "/mode", {"mode": "deter"})
        _post(base, "/deter", {
            "on": True,
            "mode": args.redblue,
            "siren_level": args.siren_level,
            "tts_reps": args.tts_reps,
        })
        snapshots = _watch_armed(base, seconds)
        _check_raised(snapshots)
    except AcceptError as exc:
        print(f"FAIL while armed: {exc}", file=sys.stderr)
        return 1
    finally:
        # Runs on success, on failure, and on Ctrl-C: never leave the device deterring.
        _teardown(base)
    try:
        _check_cleared(base)
    except AcceptError as exc:
        print(f"FAIL at teardown: {exc}", file=sys.stderr)
        return 1
    print("PASS: deterrent was raised on the device and fully cleared afterwards")
    return 0


if __name__ == "__main__":
    sys.exit(main())
