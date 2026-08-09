"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: p4_agent.safety_bypass -- 16 §4 语音安全旁路 (急停/趴下/站立)

Description:
Runs BEFORE ASR post-processing and BEFORE the intent classifier.
16 §4 P-1 verbatim: "★ **急停旁路必须在 ASR 后处理之前分流**,不得
等待 LLM". This package is that pre-classifier gate.

Three bypass actions, three different dispatch paths (16 §4 table):

  | 指令  | 旁路 LLM | 旁路仲裁 | 路径                             |
  |-------|----------|----------|----------------------------------|
  | 急停  |    ✅    |    ✅    | 直发 cmd/estop → quadruped Tier 1 |
  | 趴下  |    ✅    |    ❌    | rt/chassis/ctrl → 经 P1 仲裁      |
  | 站立  |    ✅    |    ❌    | 同上                             |

★ Non-symmetric cost (16 §4.1) -- ★★★ this is why we do NOT reuse
the general intent classifier for these three verbs:
  * 误触发 (heard stop when none said) -> robot stops once, no lasting
    lockout (U35: 软急停不锁定). Operator says next command, done.
  * 漏触发 (heard nothing, actual stop said) -> ★ 可能压到人, ★ 不可
    恢复.
  * ⇒ Match threshold tuned toward "宽松". Fuzzy + pinyin matching
    included. General classifier's non-fuzzy threshold would drop
    real stops.

★ Two match points (16 §4 约束表): matches on BOTH raw ASR text AND
post-normalized text. Reason: post-processing can corrupt "急停" via
L2 音近纠错 or L3 闭集吸附. If we only matched normalized text, an
L1 edit that turned "急停" into "紧张" would silently defeat safety.

★★ One documented exception -- 16 §4.2 U45: while in
`geometry_recording` state, VOICE estop is SUPPRESSED (keyboard/
handle estop still works). This module MUST honor that exception
FIRST (before the estop match runs). Operators are next to the robot
during recording and speaking constantly; a voice estop false-trigger
would destroy the recording session.
"""
