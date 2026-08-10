"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: observation_window.py
Brief: INF-DP-8 观察窗 -- p5_gateway 最小模式 + boot_fail 落盘 + 三态 BIT 区分

Description:
W-1 observation window: when freeze fails, p5_gateway starts in
MINIMAL MODE so the HMI still comes up and shows the failing
assertions + key paths. Minimal mode differs from full-mode in
one hard-invariant way:

  * MINIMAL_MODE_PUBLISHERS MUST NOT include any 'cmd/motion/*'
    key. A minimal-mode instance publishing motion commands
    would defeat the whole point of the safe-fallback (the very
    reason freeze failed was we could not trust our config).

boot_fail persistence (W-2):
  * /opt/xbrain_v6/data/run/boot_fail.json    -- volatile snapshot
                                                for current-boot state
  * /opt/xbrain_v6/data/boot_fail.jsonl       -- durable, APPEND-only
                                                (never overwrite --
                                                otherwise a fresh
                                                failure erases the
                                                previous record)
  Both paths are the V6-standard 'runtime files under /opt/xbrain_v6/'
  location. The functions here take paths as arguments; the constants
  above are what callers (p5_gateway minimal mode + diag bundler) use.
  * on next successful boot, each JSONL row is transposed to
    event/fault/system with the recorded (stage, code, boot_id,
    message) fields.

W-3 three-state BIT surface:
  * NEVER_RAN          -- BIT was not scheduled yet
  * RAN_NO_RESULT      -- BIT scheduled but no result frame
  * RAN_FAILED         -- BIT completed and reported failure
Merging the three into two would hide the difference between
'BIT never ran' (probe failure) and 'BIT ran but produced no
result' (BIT process crashed) -- the operator's next action
is different for each.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


class MinimalModeSurfaceViolation(Exception):
    pass


# W-1: minimal mode may publish these; NOTHING else.
MINIMAL_MODE_PUBLISHERS = frozenset({
    "state/link",
    "state/boot_fail",      # exposes the failure to HMI
    "event/warn/boot",      # boot-time diagnostic events
})


FORBIDDEN_IN_MINIMAL = frozenset({
    # Anything under cmd/motion/* is a hard no.
    "cmd/motion/cmd_vel",
    "cmd/motion/factor",
    "cmd/motion/behavior",
    "cmd/motion/route",
    # Also block ptz + payload from minimal (only diagnostics).
    "cmd/ptz",
    "cmd/payload",
    # State keys that MUST come from real p5_gateway.
    "state/task",
    "state/mode",
})


def assert_minimal_publisher_set(pubs) -> None:
    """The declared publishers of a minimal-mode instance MUST be
    a subset of MINIMAL_MODE_PUBLISHERS AND disjoint from
    FORBIDDEN_IN_MINIMAL."""
    pubs = set(pubs)
    forbidden_hits = pubs & FORBIDDEN_IN_MINIMAL
    if forbidden_hits:
        raise MinimalModeSurfaceViolation(
            f"minimal mode publishers include forbidden keys "
            f"{sorted(forbidden_hits)}; motion/ptz/payload must "
            f"never fire from minimal (INF-DP-8 W-1)")
    extras = pubs - MINIMAL_MODE_PUBLISHERS
    if extras:
        raise MinimalModeSurfaceViolation(
            f"minimal mode publishers include unknown keys "
            f"{sorted(extras)}; only {sorted(MINIMAL_MODE_PUBLISHERS)} "
            f"are sanctioned")


# ---- W-2 boot_fail persistence ------------------------------------

@dataclass(frozen=True)
class BootFailRecord:
    stage: str        # 'stage_a' / 'stage_b' / 'stage_c' etc
    code: str         # 'E_CONFIG_INVALID' / 'E_STORAGE_CORRUPT' / etc
    boot_id: str
    message: str


def append_boot_fail_jsonl(path: str, rec: BootFailRecord) -> None:
    """APPEND ONLY. Overwriting the file would lose earlier failure
    records that a next-boot handler still needs to replay."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps({
            "stage": rec.stage,
            "code": rec.code,
            "boot_id": rec.boot_id,
            "message": rec.message,
        }, ensure_ascii=False, sort_keys=True) + "\n")


def read_boot_fail_jsonl(path: str) -> List[BootFailRecord]:
    """Read all queued failure records. Missing file = empty list."""
    if not os.path.exists(path):
        return []
    out: List[BootFailRecord] = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            out.append(BootFailRecord(
                stage=d["stage"], code=d["code"],
                boot_id=d["boot_id"], message=d["message"]))
    return out


def transpose_to_event(rec: BootFailRecord) -> dict:
    """Next successful boot: each queued record becomes an
    event/fault/system with detail carrying the four fields."""
    return {
        "kind": "event/fault/system",
        "detail": {
            "stage": rec.stage,
            "code": rec.code,
            "boot_id": rec.boot_id,
            "message": rec.message,
        },
    }


# ---- W-3 three-state BIT surface ----------------------------------

class BitObservationState(str, Enum):
    NEVER_RAN = "never_ran"
    RAN_NO_RESULT = "ran_no_result"
    RAN_FAILED = "ran_failed"
    RAN_PASSED = "ran_passed"


def classify_bit_observation(
        bit_scheduled: bool,
        bit_result: Optional[str],
        result_indicates_pass: bool) -> BitObservationState:
    """Distinguish the FOUR observation states. The three failure/
    unknown states MUST be reported distinctly (W-3 spec) so the
    HMI operator's next action is unambiguous:
      never_ran      -> re-run probe
      ran_no_result  -> BIT process crashed; check logs
      ran_failed     -> a specific BIT item is red; consult health"""
    if not bit_scheduled:
        return BitObservationState.NEVER_RAN
    if bit_result is None:
        return BitObservationState.RAN_NO_RESULT
    if result_indicates_pass:
        return BitObservationState.RAN_PASSED
    return BitObservationState.RAN_FAILED
