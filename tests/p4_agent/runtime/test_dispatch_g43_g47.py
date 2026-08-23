"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_dispatch_g43_g47.py
Brief: 18-C G43-G47 dispatch per V6 architecture (16 S8.2: queries not scheduled)

Description:
The user asked that all voice/text commands (except ESTOP) participate in
scheduling, then deferred to the V6 architecture. The architecture (16, G-class
row: "G 类查询 | 不下发 | 直接走 S8.2 模板答复") routes QUERIES to a direct spoken
reply, NOT to the task scheduler; only ACTION commands go to cmd/task -> P3. So
"per the architecture" means: actions are scheduled, queries are answered
directly. This test pins that the new G43-G47 queries follow the query path
(cmd/audio/speak) via the "G" prefix, and never leak onto the scheduler
(cmd/task). If a future edit mis-routed a query to cmd/task it would be silently
queued instead of answered -- the RED case here.
"""

from __future__ import annotations

from xbrain.p4_agent.runtime.intent_dispatch import (
    CMD_AUDIO_SPEAK, CMD_TASK, choose_key,
)


def test_g43_g47_route_to_direct_answer_not_scheduler():
    for gid in ("G43", "G44", "G45", "G46", "G47"):
        # 16 S8.2 / VD-3: a query is spoken directly, not queued.
        assert choose_key(gid) == CMD_AUDIO_SPEAK
        assert choose_key(gid) != CMD_TASK


def test_action_prefixes_still_route_to_scheduler():
    # Sanity anchor: action families still go to cmd/task -> P3 scheduler, so
    # "queries are not scheduled" is a real distinction, not a broken map.
    #
    # *** C01 used to be the C-class anchor here and it was the WRONG one:
    # 18 C01 enter_alarm is a MODE command ("P2 -> D 模式"), and this assertion
    # was quietly holding the mis-routing in place -- the whole C class went to
    # cmd/task where P3 skipped it. C06 standby is the honest C-class anchor:
    # 18's effect column for it reads "P3 挂起任务 + P1 hold", so it really is
    # a task-family command (see intent_dispatch's C entry).
    for aid in ("B01", "C06", "T01"):
        assert choose_key(aid) == CMD_TASK
