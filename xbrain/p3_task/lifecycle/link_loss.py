"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: link_loss.py
Brief: F-5 l3_lost -> inject return_home on cloud-link L3 (11 S4.6.4 / NFR-12)

Description:
lifecycle/failure.py already DECIDES the F-5 response for a lost cloud link
(kind='l3_lost' -> 'inject_return_home'), but nothing wired the actual trigger or
built the task. This module is that realization: P3 reads state/link.level (11 S4.6,
published by P5 which is the sole authority, LNK-6) and, when the link has been down
past rtb_s (level == 3), injects ONE return_home task so the robot comes home instead
of patrolling unsupervised out of contact (TSK-20..22).

Two things this must get right:
  * Idempotency by (gw_start_mono, link_epoch), 11 S4.6.4: a sustained L3 stays L3
    for the whole outage (disconnected_s only grows), so without the guard we would
    enqueue a return_home every scheduler tick. link_epoch bumps once per outage and
    gw_start_mono changes on a P5 restart, so the pair identifies "this outage" --
    fire once per pair. (In-memory: a P3 restart mid-outage re-arms and could inject
    a second one; a persistent check against task.db is a later refinement.)
  * The return_home is a NORMAL task on the one queue: same id_alloc + TaskRow +
    TasksDAO as any task, so it shares the scheduler / suspend-resume / retention.
    source='charge' + priority=95 per 15 S4.2.1 (the 11 S4.6.4 v0.7 correction: 90
    has no slot in common.priority.task.*). state='pending' -- it waits its turn like
    everything else, it does not pre-empt by construction.

This does NOT drive motion or pick a dock -- it only records the intent; the
scheduler + charge/return executor own the rest. L2 admission (reject unsupervised
new tasks by source) is a separate consumer of the same level, not built here.
"""

from __future__ import annotations

import json
from typing import Optional

from xbrain.p3_task.dao.tasks_dao import TaskRow


# 15 S4.2.1 / 11 S4.6.4 v0.7: the link-loss return_home priority. A contract value
# (one of the locked common.priority.task.* five), inlined as an interim constant
# until that config axis is wired -- NOT a made-up default.
RETURN_HOME_PRIORITY = 95


class LinkLossReturnTrigger:
    """Fire at most once per outage (11 S4.6.4 idempotency by gw_start_mono +
    link_epoch). Feed it the latest link snapshot each tick."""

    def __init__(self) -> None:
        self._fired = None      # the (gw_start_mono, link_epoch) already injected

    def should_inject(self, level, gw_start_mono, link_epoch) -> bool:
        """True iff level is L3 AND we have not already injected for this
        (gw_start_mono, link_epoch). A missing field (no link state yet) is not L3."""
        if level is None or level < 3:
            return False
        if gw_start_mono is None or link_epoch is None:
            return False
        key = (gw_start_mono, link_epoch)
        if key == self._fired:
            return False
        self._fired = key
        return True


def link_loss_trace_id(gw_start_mono, link_epoch) -> str:
    """A stable trace_id for the injected task (link loss has no cmd envelope to
    thread). Deterministic from the outage identity so an audit can tie the task
    back to which outage caused it."""
    return "rtb-linkloss-le%s-gw%d" % (link_epoch, int(gw_start_mono))


def build_return_home_row(task_id: str, submit_seq: int, *, priority: int,
                          level: int, disconnected_s, link_epoch,
                          gw_start_mono, now_mono_ms: int,
                          trace_id: str) -> TaskRow:
    """The F-5 return_home TaskRow. mission_json records WHY (cloud_link_lost + the
    outage identity) so an operator/audit sees it was link loss, not a low battery
    or an operator command. created/updated are MONOTONIC ms (CLK-C1), supplied by
    the caller."""
    mission = {
        "source": "auto",                 # self-generated, not an operator channel
        "reason": "cloud_link_lost",
        "level": level,
        "disconnected_s": disconnected_s,
        "link_epoch": link_epoch,
        "gw_start_mono": gw_start_mono,
    }
    return TaskRow(
        task_id=task_id,
        task_type="return_home",
        state="pending",                  # waits its turn, does not pre-empt
        priority=priority,
        submit_seq=submit_seq,
        mission_json=json.dumps(mission, ensure_ascii=False,
                                separators=(",", ":")),
        total_steps=1,                    # 15 S4.2.1: return_home is one step (go home)
        current_step=0,
        step_status_json="[]",
        created_ms=now_mono_ms,
        updated_ms=now_mono_ms,
        source="charge",                  # 15 S4.2.1: return_home rides the charge lane
        trace_id=trace_id,
        resume_policy="continue",
    )


async def maybe_inject_return_home(conn, dao, trigger: LinkLossReturnTrigger,
                                   link: dict, *, priority: int,
                                   now_mono_ms: int) -> Optional[str]:
    """If `link` is L3 (a new outage), INSERT one return_home through the same DAO as
    any task, keyed by the deterministic 15 S4.2.1 task_id. Returns the task_id
    inserted, or None (below L3, or already injected for this outage). `link` = the
    latest {level, gw_start_mono, link_epoch, disconnected_s}."""
    gw = link.get("gw_start_mono")
    epoch = link.get("link_epoch")
    if not trigger.should_inject(link.get("level"), gw, epoch):
        return None
    # 15 S4.2.1: the task_id ITSELF is the idempotency key -- rh-{gw_start_mono}-
    # {link_epoch}. Because tasks.task_id is PRIMARY KEY, a duplicate is impossible,
    # so idempotency survives a P3 restart (the in-memory trigger only re-arms; the
    # persisted row is the real guard, T-3). Check-then-insert is race-free in P3's
    # single-threaded loop; a hit means this outage already has a return_home.
    task_id = "rh-%d-%d" % (int(gw), int(epoch))
    if await dao.fetch_by_id(task_id) is not None:
        return None
    from xbrain.p3_task.ingest.id_alloc import next_submit_seq

    submit_seq = await next_submit_seq(conn)
    trace_id = link_loss_trace_id(gw, epoch)
    row = build_return_home_row(
        task_id, submit_seq, priority=priority,
        level=link.get("level"), disconnected_s=link.get("disconnected_s"),
        link_epoch=epoch, gw_start_mono=gw,
        now_mono_ms=now_mono_ms, trace_id=trace_id)
    await dao.insert(row)
    return task_id
