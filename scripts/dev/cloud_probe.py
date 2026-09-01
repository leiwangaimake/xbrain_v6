#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cloud_probe.py
Brief: Dev-only stand-in for the customer Qt client -- publishes one v2.0 cloud
       command and prints every uplink key that answers

Description:
What this solves. run_stack_cloud.sh can prove the GEN router listens on the LAN
and that p5 logged a cloud bridge, but neither fact says a command actually
traverses the face. The only honest check is to BE the client: connect as a
Zenoh client over the LAN endpoint exactly as Qt does, publish a real v2.0
envelope, and read what comes back. Everything else is inference.

Why it matters that this connects to the LAN address and not loopback: the
on-board processes attach to tcp/127.0.0.1:7447 (constants in
session_factory.py, per 11 S1.1.4). If the probe also used loopback it would
pass on a router that never accepted an off-board client, which is the one
failure the cloud test cares about.

What this is NOT. It does not validate the customer's field values, does not
assert protocol conformance, and does not replace tests/p5_gateway. It answers
one question -- does a frame published from a client session reach the gateway
and produce the uplink v2.0 requires -- and prints the raw answers so a human
reads them rather than trusting a green line.

The frame shape is 任务枚举_qt端v2.0 S1.4 (six-field envelope) plus the
json格式文件_qt端v2.0 S2.1 GOTO_KEYPOINT example. ts is float64 Unix seconds
(S1.5, verbatim: no ISO strings, no millisecond integers). mono/boot are NOT
sent -- S1.6 forbids them on cross-host messages.

Usage:
  python3 scripts/dev/cloud_probe.py --rid dev --endpoint tcp/192.168.1.19:7447
  python3 scripts/dev/cloud_probe.py --rid dev --command ESTOP
  python3 scripts/dev/cloud_probe.py --rid dev --listen-only --seconds 15
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import uuid


# The uplink keys v2.0 S2 declares. Subscribed as one wildcard per family so an
# unexpected key still shows up: a probe that only listens for what it expects
# cannot report "the gateway answered on a key we did not plan for".
UPLINK_PATTERNS = (
    "xbrain/{rid}/cmd/task/ack",
    "xbrain/{rid}/cmd/estop/ack",
    "xbrain/{rid}/state/**",
    "xbrain/{rid}/event/**",
    "xbrain/{rid}/data/**",
)


def _envelope(rid: str, seq: int, data: dict) -> bytes:
    """Six-field cross-host envelope (S1.4). v is fixed at 1.

    ts is a JSON number of Unix seconds. This is the ONE place a wall clock is
    correct in this repo: S1.5 freezes the wire format as UTC epoch seconds, and
    CLK-C3 scopes the monotonic-clock rule to timeout / age decisions, which a
    probe makes none of.
    """
    return json.dumps({
        "v": 1,
        "rid": rid,
        "ts": time.time(),
        "seq": seq,
        "src": "qt_hmi",
        "data": data,
    }, ensure_ascii=False).encode("utf-8")


def _goto_payload() -> dict:
    """json格式文件_qt端v2.0 S2.1 verbatim shape.

    The waypoint id is deliberately one the manifest will not contain: on a
    machine with an empty geo.db the interesting answer is the STRUCTURED
    rejection (S3.1 requires ID-not-found to be refused with locating fields),
    not a fabricated acceptance. A probe that only ever tests the happy path
    would report the same green line against a gateway that accepts anything.
    """
    return {
        "msg_id": "msg-probe-%s" % uuid.uuid4().hex[:8],
        "task_id": "task-probe-%s" % uuid.uuid4().hex[:8],
        "task_type": "GOTO_KEYPOINT",
        "payload": {
            "coordinate_system": "WGS84",
            "recorded_path_id": "r-probe_route",
            "waypoints": [{
                "id": "w-probe_point",
                "name": "probe",
                "latitude": 31.2301971,
                "longitude": 121.4732683,
                "altitude": 8.4,
                "arrival_radius_m": 3.0,
            }],
        },
    }


def _estop_payload() -> dict:
    """S3.3: action fixed to stop, reason free text. Rides cmd/estop, NOT
    cmd/task -- the whole point of T03 is that it bypasses the task queue."""
    return {
        "msg_id": "msg-probe-%s" % uuid.uuid4().hex[:8],
        "task_type": "ESTOP",
        "payload": {"action": "stop", "reason": "cloud_probe"},
    }


def _unsupported_payload() -> dict:
    """S3.6 / S7.10: a retired task_type MUST be refused, never silently
    mapped. Sending one is the only way to see the refusal path work."""
    return {
        "msg_id": "msg-probe-%s" % uuid.uuid4().hex[:8],
        "task_id": "task-probe-%s" % uuid.uuid4().hex[:8],
        "task_type": "MANUAL_VELOCITY",
        "payload": {},
    }


def _stop_payload() -> dict:
    """S3.2: target_task_id is mandatory -- "omit means current task" is
    forbidden verbatim. The id below does not exist, so the expected answer is
    rejected/E_NOT_FOUND: that IS the behaviour S3.2 specifies, and probing it
    is how we learn the not-found path is wired rather than swallowed."""
    return {
        "msg_id": "msg-probe-%s" % uuid.uuid4().hex[:8],
        "task_id": "task-probe-%s" % uuid.uuid4().hex[:8],
        "task_type": "STOP_TASK",
        "payload": {
            "target_task_id": "task-probe-absent",
            "action": "cancel",
            "reason": "cloud_probe",
        },
    }


def _alarm_payload() -> dict:
    """json格式文件_qt端v2.0 S2.4 shape, scalars inside their stated ranges
    (alarm_level 1|2, siren_level 0..100, duration_sec 1..20,
    cooldown_sec 0.5..600.0). base_rev 0 is the "new object" value per S3.4."""
    return {
        "msg_id": "msg-probe-%s" % uuid.uuid4().hex[:8],
        "task_id": "task-probe-%s" % uuid.uuid4().hex[:8],
        "task_type": "SET_ALARM_CONFIG",
        "payload": {
            "alarm_level": 1,
            "siren_level": 70,
            "duration_sec": 5,
            "cooldown_sec": 2.0,
            "alarm_window": {"start": "22:00", "end": "05:00"},
            "rules": [{
                "type": "person_in_region",
                "enabled": True,
                "alarm_role": "include",
                "applies_to": ["person"],
                "region_ids": ["f-probe_zone"],
            }],
            "regions": [{
                "id": "f-probe_zone",
                "op": "upsert",
                "base_rev": 0,
                "name": "probe zone",
                "type": "alarm_region",
                "enabled": True,
                "applies_to": ["person"],
                "vertices": [
                    {"latitude": 31.2301971, "longitude": 121.4732683},
                    {"latitude": 31.2301971, "longitude": 121.4738640},
                    {"latitude": 31.2305962, "longitude": 121.4738640},
                    {"latitude": 31.2305962, "longitude": 121.4732683},
                ],
            }],
        },
    }


def _audio_payload() -> dict:
    """S3.5: only mode=pc_to_dog is open, and a start request must NOT carry a
    stream_id -- the backend allocates it and returns it in the ack. A probe
    that sent one would mask the allocation path entirely."""
    return {
        "msg_id": "msg-probe-%s" % uuid.uuid4().hex[:8],
        "task_id": "task-probe-%s" % uuid.uuid4().hex[:8],
        "task_type": "AUDIO_CONTROL",
        "payload": {"mode": "pc_to_dog", "action": "start"},
    }


COMMANDS = {
    "GOTO_KEYPOINT": ("cmd/task", _goto_payload),
    "STOP_TASK": ("cmd/task", _stop_payload),
    "SET_ALARM_CONFIG": ("cmd/task", _alarm_payload),
    "AUDIO_CONTROL": ("cmd/task", _audio_payload),
    "ESTOP": ("cmd/estop", _estop_payload),
    "MANUAL_VELOCITY": ("cmd/task", _unsupported_payload),
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rid", required=True,
                    help="must match the rid the stack was started with")
    ap.add_argument("--endpoint", default="tcp/127.0.0.1:7447",
                    help="GEN router endpoint; use the LAN address to prove "
                         "an off-board client can reach it")
    ap.add_argument("--command", default="GOTO_KEYPOINT",
                    choices=sorted(COMMANDS))
    ap.add_argument("--seconds", type=float, default=8.0,
                    help="how long to collect uplink after publishing")
    ap.add_argument("--listen-only", action="store_true",
                    help="subscribe and report without publishing anything")
    args = ap.parse_args(argv)

    import zenoh

    cfg = zenoh.Config()
    # mode=client: the probe must never become a router peer. A peer would
    # gossip and could carry traffic on its own, which would make a broken
    # router look healthy.
    cfg.insert_json5("mode", '"client"')
    cfg.insert_json5("connect/endpoints", json.dumps([args.endpoint]))
    cfg.insert_json5("scouting/multicast/enabled", "false")

    print("connecting mode=client -> %s" % args.endpoint)
    try:
        session = zenoh.open(cfg)
    except Exception as exc:                      # noqa: BLE001
        print("FAIL cannot open session: %s" % exc, file=sys.stderr)
        return 1

    received = []

    def _on(sample):
        try:
            body = bytes(sample.payload).decode("utf-8")
        except Exception:                          # noqa: BLE001
            body = repr(bytes(sample.payload))
        received.append((str(sample.key_expr), body))

    # Hold the handles in a list: a dropped subscriber is silently deregistered
    # on the Rust side and the probe would report "no answer" (CLAUDE.md 4.3).
    subs = []
    for pat in UPLINK_PATTERNS:
        key = pat.format(rid=args.rid)
        subs.append(session.declare_subscriber(key, _on))
    print("subscribed %d uplink patterns under xbrain/%s/" % (len(subs), args.rid))

    if not args.listen_only:
        key_tail, builder = COMMANDS[args.command]
        key = "xbrain/%s/%s" % (args.rid, key_tail)
        frame = _envelope(args.rid, 1, builder())
        time.sleep(1.0)          # let the subscriptions land before publishing
        session.put(key, frame)
        print("published %s -> %s (%d bytes)" % (args.command, key, len(frame)))

    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        time.sleep(0.2)

    print("\n--- uplink received in %.0fs: %d sample(s) ---"
          % (args.seconds, len(received)))
    seen_keys = {}
    for key, body in received:
        seen_keys[key] = seen_keys.get(key, 0) + 1
    for key in sorted(seen_keys):
        print("  %-44s x%d" % (key, seen_keys[key]))

    # Print the ack bodies in full: the ack is the one answer whose CONTENT the
    # operator has to read (accepted vs rejected vs duplicate, and the reason).
    print("\n--- ack / result bodies ---")
    shown = 0
    for key, body in received:
        if "/ack" in key or "state/task" in key:
            print("  %s\n    %s" % (key, body[:600]))
            shown += 1
            if shown >= 6:
                break
    if not shown:
        print("  (none -- the gateway did not answer this command)")

    for s in subs:
        try:
            s.undeclare()
        except Exception:                          # noqa: BLE001
            pass
    session.close()
    return 0 if received else 2


if __name__ == "__main__":
    sys.exit(main())
