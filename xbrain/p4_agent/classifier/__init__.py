"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p4_agent.classifier -- 16 §5.2 五层意图分类

Description:
Two-level routing per 16 §5:
  Level 1 (this package) -- 128 intents -> route decision
                            (bypass | fastpath | fastpath_then_llm | llm)
  Level 2 (§6/§7)         -- slot filling via GBNF + LLM

* IMPORTANT SCOPE: this package does NOT include the safety-bypass
matcher (急停/趴下/站立). That matcher lives at xbrain/p4_agent/
safety_bypass/ and runs BEFORE this classifier (16 §4). The reason
they are separate:

  * Safety bypass matches on BOTH raw ASR text AND post-normalized
    text (double-match to defeat post-processing that mangles "急停")
  * Safety bypass has a NON-SYMMETRIC cost: 漏 > 误 (16 §4.1). It
    tunes toward "宽松" match with wider phonetic tolerance.
  * Safety bypass has a UNIQUE dispatch path for `estop`: direct to
    cmd/estop -> quadruped Tier 1, bypassing arbitration entirely
    (16 §4 table). No other classifier row does this.

Putting safety_bypass inside classifier/ would risk it running AFTER
the 5-layer chain, which is 16 §4 P-1 "急停旁路必须在 ASR 后处理之
前分流" -- a doc-mandated hard constraint.

Public API (added incrementally as GWY-P4-* items land):
  * routes.py -- the 4-value ROUTE closed set (16 §5.3)
  * (future) chain.py -- the 5-layer priority chain (§5.2)
  * (future) directional.py -- §5.2.1 overheard filter
  * (future) session_state.py -- §5.2 rule ③ (recording/awaiting state)

* This package currently holds ONLY routes.py + closed-set enforcement.
The 5-layer chain body is GWY-P4-06 / GWY-P4-08 territory and depends
on the intents.yaml registry being loaded (GWY-P4-07). Landing the
chain before those would be building on unresolved values.
"""
