"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: mission_select.py
Brief: tier-2 mission selection -- coarse cue -> one M-mission (16 S5.2 layer 4)

Description:
The candidate-selection half of tier-2 (16 S5.2 step 6 / S6.7). When the six
deterministic layers all miss (the text is 'unknown' but directed), the LLM is
asked to CLASSIFY it -- but a 3B model degrades past ~5 tools (AI-36), so it is
never shown the whole registry. Instead one MISSION (a <=5-intent family, the
16 S6.7 group table) is picked from a coarse cue and ONLY that mission's
intents form the GBNF the model may emit.

This module is that picker: text -> mission key, or None. It is the same idea
as classifier.large_class (the deterministic device-family router), one layer
out: large_class RESOLVES a device intent outright; here we only choose the
FAMILY and let the LLM pick the specific intent + fill the free-text slot the
fastpath could not (e.g. speak_custom's message, a route/fence NAME).

Missions and their cues (families from missions.EXPECTED_EMISSIONS):
  M2_turn      转/掉头/转身/朝向        -> A09-A12 turns
  M1_translate 平移/挪/横move + 前进后退 -> A05-A08 straight moves
  M3_nav       去/前往/巡逻/绕          -> B01-B04 goto/patrol
  M4_follow    跟/跟随/盯着            -> follow + track
  M5_speak     说/喊/播报/广播/念       -> speak_preset/custom (needs the text)
  M6_naming    命名/保存为/改名         -> save/rename a route/fence
  M6b_mark     记为/标记为/把这里记      -> mark a waypoint/dock (needs the name)
  M7_objref    删除/删掉/启用/换成       -> delete/activate a geo object
  M8_events    事件/报告/最近..发生      -> event queries + report

Only families whose intents are LLM-route (llm / fastpath_then_llm) are worth a
mission: a fastpath intent bypasses the LLM (GB-1c forbids it in a mission
grammar). Returning None means 'no confident family' -- the caller declines
(the polite 'did not catch that'), never burning a GPU slot on out-of-scope.

Order matters where cues overlap: 转 (turn, M2) is checked before the bare
直行 words; 记为 (mark, M6b) before 命名/保存 (naming, M6). A cue table, not
an LLM call -- selection must be cheap and deterministic (it runs on every
unknown turn, before the single GPU slot is even considered).
"""
from __future__ import annotations

from typing import List, Optional, Tuple


# Ordered (mission, cue-substrings). FIRST mission with any cue present wins,
# so more specific families are listed before the ones they could shadow.
_MISSION_CUES: List[Tuple[str, Tuple[str, ...]]] = [
    # speak: a spoken message. High-value -- the message is free text only the
    # LLM can lift out. Checked early: '喊话'/'播报' are unambiguous.
    ("M5_speak", ("喊话", "播报", "广播", "喊一", "说一", "念", "朗读", "通知",
                  "警告", "对他说", "对外说")),
    # mark a point: '记为' / '标记...为' / '把这里' -- BEFORE naming, since '记为'
    # is more specific than the bare '命名'.
    ("M6b_mark", ("记为", "标记", "把这里", "把此处", "存为点", "记一个点")),
    # naming/saving a route or fence, or renaming.
    ("M6_naming", ("命名", "保存为", "存为", "改名", "重命名", "叫做")),
    # object ops: delete / activate / switch a geo object.
    ("M7_objref", ("删除", "删掉", "去掉", "启用", "停用", "换成", "切到")),
    # event queries + report.
    ("M8_events", ("事件", "报告", "最近", "发生了", "都干了", "记录一下发生")),
    # follow / track a target.
    ("M4_follow", ("跟随", "跟着", "跟上", "盯住", "盯着", "别跟", "停止跟")),
    # navigation: go somewhere / patrol.
    ("M3_nav", ("巡逻", "前往", "去到", "去往", "绕一圈", "绕场", "到岗", "去")),
    # turns: rotate / face. BEFORE translate so '左转' is a turn, not a move.
    ("M2_turn", ("转身", "掉头", "转向", "朝向", "面向", "转个", "左转", "右转",
                 "转到")),
    # straight moves (translate).
    ("M1_translate", ("前进", "后退", "倒退", "往前", "往后", "平移", "挪",
                      "横向", "左移", "右移")),
]


def select_mission(text: str) -> Optional[str]:
    """Return the M-mission key whose family the text most likely belongs to,
    or None when no family cue is present (the caller then declines without an
    LLM call). First match in the ordered cue table wins."""
    text = text or ""
    for mission, cues in _MISSION_CUES:
        if any(cue in text for cue in cues):
            return mission
    return None
