"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: pipeline.py
Brief: GWY-P5-01 seven-step event pipeline (schema -> dedup -> level -> persist -> cloud -> HMI -> delivered)

Description:
17 S3 event pipeline is SEVEN stages, in a strict order:

  1  schema-validate    reject if unknown category / bad shape
  2  dedupe             (source, event_id) already-seen -> drop
  3  level              apply severity classifier (info/warn/error)
  4  persist            append to record.db events (writer=T-2)
  5  cloud              enqueue for uplink
  6  hmi                enqueue for HMI broadcast
  7  delivered          mark delivery lifecycle 'submitted'

Each stage can DROP or CONTINUE; drops carry a reason.
Order MUST be as listed: e.g. persist before cloud, so a crash
between the two loses the cloud copy (recoverable via replay)
but never loses the local record.
"""

from __future__ import annotations

from dataclasses import dataclass


PIPELINE_STAGES = (
    "schema", "dedupe", "level", "persist", "cloud", "hmi", "delivered",
)


@dataclass(frozen=True)
class StageResult:
    stage: str
    dropped: bool
    reason: str = ""


class PipelineOrderViolation(Exception):
    pass


VALID_CATEGORIES = frozenset({
    "safety", "task", "sensor", "network", "audit", "diagnostic",
})


VALID_LEVELS = frozenset({"info", "warn", "error"})


def stage_schema(event: dict) -> StageResult:
    """1. Reject unknown categories or events missing required fields."""
    if not isinstance(event, dict):
        return StageResult("schema", True, "not_a_dict")
    cat = event.get("category")
    if cat not in VALID_CATEGORIES:
        return StageResult("schema", True, f"unknown_category:{cat!r}")
    if "event_id" not in event or "source" not in event:
        return StageResult("schema", True, "missing_id_or_source")
    return StageResult("schema", False)


def stage_dedupe(event: dict, seen_keys: set) -> StageResult:
    """2. (source, event_id) idempotency."""
    key = (event["source"], event["event_id"])
    if key in seen_keys:
        return StageResult("dedupe", True, "already_seen")
    seen_keys.add(key)
    return StageResult("dedupe", False)


def stage_level(event: dict) -> StageResult:
    """3. Ensure level is in the 3-value closed set; assign 'info'
    if the field is absent (default classifier for unclassified
    events)."""
    lvl = event.get("level", "info")
    if lvl not in VALID_LEVELS:
        return StageResult("level", True, f"bad_level:{lvl!r}")
    event["level"] = lvl
    return StageResult("level", False)


def assert_stage_order(actual, expected=PIPELINE_STAGES) -> None:
    """Order-preserving check used by the orchestrator's self-test."""
    if tuple(actual) != tuple(expected):
        raise PipelineOrderViolation(
            f"pipeline stage order: got {tuple(actual)}, "
            f"want {tuple(expected)}")
