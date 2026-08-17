"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: pipeline.py
Brief: GWY-P5-01 seven-step event pipeline (schema->dedupe->level->persist->cloud->hmi->delivered)

Description:
17 S3.1 pipeline, rewritten onto the real contract (the earlier version validated
against a placeholder category set {safety,task,sensor,...} and level set
{info,warn,error} that never existed in 11 -- this one uses EVENT_CATEGORY's 23
values and the 4-value SEVERITY, and persists through the record.db DAO).

The seven steps, in the ONE order the contract fixes (17 S3.1):
  1 schema    reject unknown category / bad sev / missing field (E_SCHEMA)
  2 dedupe    dedup_key hit in window -> merge, no new record, push nothing again
  3 level     derive the 11 S6.2 channel (the pipeline is the authority, S3.3)
  4 persist   record.db FIRST (crash between 4 and 5 keeps the local record)
  5 cloud     enqueue for uplink -- alarm/fault + any alarm-channel event need ack
  6 hmi       enqueue for HMI broadcast
  7 delivered mark on ack / best-effort success (done by the uplink, not here)

Step 4 MUST precede step 5 (S3.1): send-then-persist would lose an event that is
acknowledged by the cloud but crashes before it hits the DB. Dedupe (step 2) is
realized INSIDE persist -- the DAO merges on dedup_key and returns 'merged', so
this pipeline pushes nothing further for a merged event (S3.2). The pipeline does
NOT itself talk to Zenoh: it returns an Outcome saying what to enqueue, and the
wiring (batch 5) + uplink (batch 4) do the I/O. So the whole ordering + drop logic
is testable against an in-memory DAO with no network.

Self-loop ban (S3.4): when persist DEGRADES (JSONL, DB write failed), this
pipeline must NOT synthesize a fault event about the failure and feed it back in
-- that is the "write fails -> fault -> write -> fails" loop (V5 bug 9). The
degrade is recorded in the Outcome and left there; the caller may log/telemeter
it but must never re-enter process() with it.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from xbrain.common.enums import EVENT_CATEGORY, SEVERITY

from .channel_map import derive_channel
from ..persistence.schema_record import need_ack


# The seven stages in contract order. Kept as data so assert_stage_order can gate
# a reordering (persist-before-cloud is the one that matters, S3.1).
PIPELINE_STAGES = (
    "schema", "dedupe", "level", "persist", "cloud", "hmi", "delivered",
)

# Fields every event must carry to be persistable (17 S3.4 NOT NULL columns that
# the producer, not the DAO, supplies). channel is NOT here: it is DERIVED at
# step 3, never trusted from the producer.
_REQUIRED_FIELDS = (
    "eid", "rid", "cat", "sev", "title", "detail", "src",
    "ts", "detected_at", "created_at",
)


class PipelineOrderViolation(Exception):
    """The stage list is not in contract order (17 S3.1). The load-bearing case is
    persist appearing after cloud, which would let a crash lose an acked event."""


def assert_stage_order(stages) -> None:
    if tuple(stages) != PIPELINE_STAGES:
        raise PipelineOrderViolation(
            f"stages must be {PIPELINE_STAGES}, got {tuple(stages)}")


@dataclass(frozen=True)
class Outcome:
    """What the pipeline did with one event -- the single instruction the wiring
    acts on. dropped is set (with a reason) iff schema rejected it; otherwise
    persisted/merged/degraded describe the DB result and to_cloud/to_hmi say what
    to enqueue. A merged event enqueues nothing (S3.2); a degraded event skips the
    cloud (it is in JSONL, replays later) but still shows live on HMI."""

    dropped: bool = False
    reason: str = ""
    persisted: bool = False
    merged: bool = False
    degraded: bool = False
    ch_seq: Optional[int] = None
    channel: Optional[str] = None
    need_ack: bool = False
    to_cloud: bool = False
    to_hmi: bool = False


class EventPipeline:
    """Runs the seven steps for each event against a RecordDao. Stateless beyond
    the DAO -- dedup + ch_seq state live in the DB, not here, so a p5 restart
    resumes from the DB (SEQ-3), not from lost in-memory state."""

    def __init__(self, dao) -> None:
        self._dao = dao

    def _validate(self, ev: dict) -> Optional[str]:
        """Step 1. Returns a drop reason string, or None if the event is well
        formed. Missing field / unknown category / bad sev are all E_SCHEMA."""
        for f in _REQUIRED_FIELDS:
            if ev.get(f) is None:
                return f"missing_field:{f}"
        if ev["cat"] not in EVENT_CATEGORY:
            return f"unknown_category:{ev['cat']}"
        if ev["sev"] not in SEVERITY:
            return f"bad_sev:{ev['sev']}"
        return None

    async def process(self, ev: dict) -> Outcome:
        # 1 schema
        reason = self._validate(ev)
        if reason is not None:
            return Outcome(dropped=True, reason=reason)

        # 3 level: derive the S6.2 channel and OVERWRITE any producer value, so
        # the channel is the contract's, not the producer's (S3.3). (Step 2
        # dedupe is performed inside persist by the DAO's merge.)
        channel = derive_channel(ev["cat"], ev.get("detail"))
        ev["channel"] = channel

        # 4 persist (record.db first). The DAO returns 'inserted' | 'merged' |
        # 'degraded'; dedupe (step 2) already happened here on a merge.
        res = await self._dao.insert_event(ev)

        if res.status == "merged":
            # S3.2: a merge bumped an existing row's count -- push nothing again.
            return Outcome(persisted=True, merged=True, channel=channel)

        if res.status == "degraded":
            # In JSONL, not the DB. Skip the live cloud enqueue (nothing durable
            # to reference) but still show it live on HMI (best-effort). Do NOT
            # loop a fault about the failure back in (S3.4).
            return Outcome(persisted=False, degraded=True, channel=channel,
                           to_hmi=True)

        # 5+6 inserted: every event goes to cloud and HMI. need_ack (the S3.3
        # union) tells the uplink whether this one must be acked or is best-effort.
        na = need_ack(channel, ev["sev"])
        return Outcome(persisted=True, ch_seq=res.ch_seq, channel=channel,
                       need_ack=na, to_cloud=True, to_hmi=True)
