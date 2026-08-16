#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: zenoh_echo.py
Brief: Dev-only Zenoh inspector -- the `ros2 topic echo` / `ros2 topic list`
       analog for XBRAIN_V6's two Zenoh planes

Description:
Why this exists. Zenoh is NOT ROS2/DDS: a key expression is just a string, and
there is no discovered-topic registry, so there is no built-in `zenoh topic
list`. The two ways to inspect a live bus are (1) subscribe to a wildcard and
watch what flows, and (2) query the router admin space for declared sub/pub.
This tool wraps (1) with the plane topology baked in so an operator does not
have to remember which router port a message rides on.

The two planes (CLAUDE.md S0.1, deploy/zenoh/*.json5):
  * rt  -> tcp/127.0.0.1:7449  (RT router, loopback only; FULL keys like
           xbrain/{rid}/rt/gnss/heading -- rtk_driver / p1_motion publish here)
  * gen -> tcp/127.0.0.1:7447  (GEN router; BARE keys like state/pose)

Modes:
  echo (default) -- stream every matching sample: key + payload (pretty JSON if
                    the payload parses as JSON, raw bytes otherwise).
  --list         -- subscribe for --seconds, then print the DISTINCT keys seen.
                    This is the honest "what is flowing" answer, NOT a registry
                    dump: a key with no current publisher will not appear (Zenoh
                    has nothing to enumerate it from). For declared-but-idle
                    entities you would query the router admin space instead
                    (@/router/**), which needs the zenohd REST/admin plugin --
                    out of scope for this passive client.

Connects as mode=client so it never becomes a router peer and never gossips; it
just asks the running router to forward matching publications. If the router is
down the connect fails loudly (there is nothing to echo) rather than hanging.

NOT for production: this is a scripts/dev tool. It reads no config source, holds
no state, and its run-duration deadline uses the monotonic clock (CLK-C1) so a
wall-clock step during a long echo never skews the timeout.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

# Plane -> the loopback endpoint its router listens on. Kept here (not read from
# deploy/zenoh/*.json5) because this is a throwaway inspector: hard-coding the
# two well-known ports keeps it dependency-free, and if a port ever moves the
# failure is an obvious "connect refused" the operator fixes in one line.
_PLANE_ENDPOINT = {
    "rt": "tcp/127.0.0.1:7449",    # RT router, loopback only (SEC-2/SEC-7)
    "gen": "tcp/127.0.0.1:7447",   # GEN router
}

# Per-plane default key pattern. RT keys are fully qualified under xbrain/{rid};
# GEN keys are bare (state/*, cmd/*). '**' matches everything on the plane, which
# is what an operator usually wants when first looking.
_PLANE_DEFAULT_KEY = {
    "rt": "xbrain/**",
    "gen": "**",
}


def _fmt_payload(raw: bytes) -> str:
    """Render a payload for the terminal: pretty one-line JSON when it parses,
    otherwise a bounded raw repr. Bounded so a stray large blob does not flood
    the screen -- an inspector should stay readable."""
    try:
        obj = json.loads(raw.decode("utf-8"))
        # Compact but key-sorted so repeated frames line up column-wise and a
        # changed field is easy to spot between two lines.
        return json.dumps(obj, ensure_ascii=False, sort_keys=True,
                          separators=(",", ":"))
    except (ValueError, UnicodeDecodeError):
        text = raw[:200].decode("utf-8", "replace")
        return text + ("..." if len(raw) > 200 else "")


def _open_session(plane: str):
    """Open a client-mode Zenoh session connected to the plane's router. Import
    zenoh lazily so `--help` works even where zenoh-python is absent."""
    import zenoh  # noqa: PLC0415 -- lazy so --help needs no zenoh install

    endpoint = _PLANE_ENDPOINT[plane]
    conf = zenoh.Config()
    # mode=client: attach to the running router as a leaf, never a peer. A peer
    # would try to gossip/scout and, on the RT plane, is exactly what the
    # deployment locks down (see the RT-plane gossip notes). A client just
    # subscribes and receives forwarded pubs.
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["%s"]' % endpoint)
    return zenoh.open(conf), endpoint


def _run_echo(session, key: str, seconds: float) -> int:
    """Stream matching samples until Ctrl-C (or --seconds elapses). The callback
    runs on Zenoh's Rust thread pool; it only formats + prints (no await, no
    cross-thread queue), which is allowed off that thread."""
    def _cb(sample) -> None:
        # str(key_expr) so a wildcard subscription still shows the CONCRETE key
        # each sample arrived on (that is the whole point of an echo).
        print("%s | %s" % (str(sample.key_expr), _fmt_payload(bytes(sample.payload))),
              flush=True)

    sub = session.declare_subscriber(key, _cb)
    # Hold a strong ref so the subscription is not GC'd (same hazard as the
    # runtime code, CLAUDE.md 4.3) -- _sub is read once at teardown.
    _sub = sub
    # Monotonic deadline (CLK-C1): 0 or negative means run until interrupted.
    deadline = None if seconds <= 0 else time.monotonic() + seconds
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        del _sub
    return 0


def _run_list(session, key: str, seconds: float) -> int:
    """Sample for `seconds`, then print the distinct keys that carried traffic.
    This is 'what is flowing now', not a topic registry -- an idle key never
    appears because Zenoh has nothing to enumerate it from."""
    seen: dict = {}

    def _cb(sample) -> None:
        seen[str(sample.key_expr)] = None

    sub = session.declare_subscriber(key, _cb)
    _sub = sub
    time.sleep(max(0.5, seconds))
    del _sub
    if not seen:
        print("(no keys carried traffic in %.0fs -- is a publisher running?)"
              % seconds)
        return 0
    print("distinct keys seen (%d):" % len(seen))
    for k in sorted(seen):
        print("  " + k)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="zenoh_echo.py",
        description="Echo / list Zenoh keys on the RT or GEN plane "
                    "(the ros2 topic echo / list analog).")
    ap.add_argument("--plane", choices=("rt", "gen"), default="rt",
                    help="which router to attach to (default: rt = 7449)")
    ap.add_argument("--key", default=None,
                    help="key expression to subscribe (default: xbrain/** on "
                         "rt, ** on gen)")
    ap.add_argument("--list", action="store_true",
                    help="print distinct keys seen over --seconds instead of "
                         "streaming every sample")
    ap.add_argument("--seconds", type=float, default=5.0,
                    help="--list sample window, or echo auto-stop; <=0 in echo "
                         "mode runs until Ctrl-C (default: 5)")
    args = ap.parse_args(argv)

    key = args.key or _PLANE_DEFAULT_KEY[args.plane]
    try:
        session, endpoint = _open_session(args.plane)
    except ImportError:
        print("zenoh-python not installed (pip install eclipse-zenoh)",
              file=sys.stderr)
        return 2
    except Exception as exc:      # noqa: BLE001 -- surface the connect failure
        # The common case: the plane's router is not running, so there is
        # nothing to echo. Say so plainly rather than dumping a Rust backtrace.
        print("cannot attach to %s plane (%s): %s\n"
              "is the %s router up? (systemctl status xbrain-zenohd-%s / "
              "scripts/start_all.sh)"
              % (args.plane, _PLANE_ENDPOINT[args.plane], exc,
                 args.plane, args.plane),
              file=sys.stderr)
        return 1

    print("attached to %s plane at %s; key=%r; mode=%s"
          % (args.plane, endpoint, key, "list" if args.list else "echo"),
          file=sys.stderr)
    try:
        if args.list:
            return _run_list(session, key, args.seconds)
        return _run_echo(session, key, args.seconds)
    finally:
        session.close()


if __name__ == "__main__":
    sys.exit(main())
