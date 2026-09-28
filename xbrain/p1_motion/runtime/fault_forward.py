"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: fault_forward.py
Brief: P1-20 -- rt/chassis/fault forwarded to event/fault/chassis with the RT-C3.e envelope rebuild

Description:
What this solves. 11 S2.2.1 registers TWO subscribers of rt/chassis/fault and
says so in as many words: chassis_relay is the normal path (CR-9), p1_motion is
the path for when the relay is DEAD -- and the message this key carries includes
E_SAFETY_LINK_LOST, whose only trigger scenario is precisely that the relay has
died. Hanging one subscriber on it means the message is published into nothing at
the exact moment it matters. P1-20 is that second path, and until now p1 did not
implement it: it subscribed rt/chassis/state (P1-24) and nothing else on the RT
plane, so the second path existed only in the contract.

Downstream of the gap: p2_core's degraded hold criterion IS E_SAFETY_LINK_LOST
(11 S4.x verbatim: 不得改从 estop_path 取). With the report lost, p2 never enters
hold and keeps accepting and running tasks with a severed soft-estop link, and no
process in the system knows.

Steady-state DOUBLE delivery is the design, not a defect (user ruling 2026-09-27,
option (a)). Both paths run all the time; p5 de-duplicates by CF-4's fully
prefixed code, and its ChassisFault derivation is edge triggered, so the second
copy of a report carries codes that are already active and produces no second
Event. An "only forward when the relay looks dead" design would need a liveness
judgement about another process on the safety path -- and would be exercised for
the first time on the day it is needed.

The envelope rebuild is RT-C3.e, and it is the same rule chassis_relay follows
(envelope_rebuild.cc): seq / ts / src become the FORWARDER's, the originals are
preserved as orig_ts / orig_src, everything else is copied, and data is carried
across untouched. Two consequences worth stating:
  * the two copies of one report that reach p5 (relay's and ours) differ in
    seq/ts/src but agree byte for byte on data AND on (orig_src, orig_ts) -- so
    that pair is a message-level identity for anyone who wants one.
  * rebuilding rather than forwarding verbatim is what keeps gap detection
    meaningful on the destination plane: a consumer counting seq is counting the
    forwarder's stream, not two interleaved producers'.

What this module does NOT do:
  * it does not parse the ChassisFault. Deriving Events from it is p5's job
    (11 UM-4 shape: the consumer derives), and a forwarder that understood the
    payload would be a second place where the fault schema lives.
  * it does not decide whether to forward. There is no liveness gate and no
    filter -- see the double-delivery note above.
  * it does not own the monotonic clock. mono/boot are COPIED from the producer
    (quadruped), because a mono reading is only meaningful inside the boot domain
    it was taken in (CLK-C4); stamping our own would silently re-base the age a
    consumer computes.
"""

from __future__ import annotations

import time
from typing import Any, Dict

# The envelope fields carried across unchanged. v and rid identify the schema and
# the robot; mono/boot are the producer's monotonic reading and its boot domain
# (CLK-C4 -- they travel as a pair and are NOT re-stamped here); ts_sync is the
# producer's copied ClockStatus.sync (CLK-A2, rtk_driver is the sole authority, so
# a forwarder never upgrades it).
_COPIED = ("v", "rid", "mono", "boot", "ts_sync")


def rebuild_forward(src_env: Dict[str, Any], *, seq: int, src: str,
                    ts: float) -> Dict[str, Any]:
    """One RT-plane envelope -> the general-plane envelope to publish.

    RT-C3.e: seq / ts / src are the forwarder's own; the producer's ts and src
    are preserved as orig_ts / orig_src; data is carried verbatim.

    ts is injected rather than read here so a test can assert exactly which
    value landed in which field -- and so this stays a pure function. It is a
    WALL clock value (11 S3.0: the envelope ts is for cross-host alignment,
    recording and latency statistics only); no age or timeout is computed from
    it anywhere in this path.

    A src_env that is not an object, or carries no data, is returned as None by
    the caller's guard rather than wrapped here: wrapping a bare payload is the
    relay's defensive branch and it cannot write a rid it never received, which
    produced a message quadruped's read_envelope rejected at the rid step. This
    module refuses instead of producing that shape.
    """
    out: Dict[str, Any] = {}
    for field in _COPIED:
        if field in src_env:
            out[field] = src_env[field]
    # The forwarder's own trio. Written AFTER the copies so a producer that
    # (wrongly) already carried an orig_* pair cannot shadow them, and so the
    # order in the JSON mirrors the S3.0 field listing for a human diffing a
    # capture against the contract.
    out["ts"] = ts
    out["seq"] = seq
    out["src"] = src
    # The originals, kept as RT-C3.e requires. Absent on the wire means absent
    # here: writing a null orig_src would claim we know the producer was
    # anonymous, when what we know is that the field was not sent.
    if "ts" in src_env:
        out["orig_ts"] = src_env["ts"]
    if "src" in src_env:
        out["orig_src"] = src_env["src"]
    # Verbatim. The forwarder does not read, reshape or validate the payload --
    # that is the consumer's job (p5 derives the Events), and a forwarder that
    # parsed it would be a second home for the ChassisFault schema.
    out["data"] = src_env["data"]
    return out


def now_wall() -> float:
    """The envelope ts for a forwarded frame, in seconds.

    Separated so the caller can inject a clock in tests and so the
    WALL-CLOCK-OK marker sits in exactly one place on this path.
    """
    # WALL-CLOCK-OK(align): 11 S3.0 envelope ts -- cross-host alignment,
    # recording and latency statistics only. Every timeout / period / age in
    # p1 uses time.monotonic (CLK-C1).
    return time.time()
