"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: turn_orchestrator.py
Brief: GWY-P4-38 (32.F) -- one P4 turn: six-step chain + confirm + envelope

Description:
16 S5.2: the authoritative order for turning one ASR utterance into an
action. This orchestrator strings the already-built P4 modules into that
order, REPLACING the V-2B naive_classify MVP (turn_loop.py). The steps:

  1 safety-bypass (16 S4)   estop / prone / stand match BEFORE any
    classification. A bypass hit goes straight to Tier1 and NEVER reaches
    the classifier -- the whole point of a bypass is that it cannot be
    reasoned about. Recording suppresses VOICE estop (U45).
  2-6 priority chain (16 S5.2) via classifier.classify_text:
    2 long-phrase exact, 3 session-state (confirm response), 4 large-class,
    5 overheard (silent, 16 S5.2.1), 6 unknown -> LLM.
  route (registry): fastpath -> dispatch DIRECTLY (no LLM, no prompt
    assemble); llm / unknown -> tier-2 (GPU-gated grammar-constrained
    classify, GWY-P4-37).
  auth gate: L0/L1a/L1b dispatch now; L2 opens a confirm and WAITS for
    I01/I02 (the confirm response arrives as the NEXT turn's session-state
    match) -- it is NOT dispatched until confirmed; L3 opens a cloud
    approval. CL-2: estop_path==down upgrades an L1b to L2 (one more gate
    when both non-voice estops are unhealthy).
  envelope + validation: a dispatched intent is wrapped in an
    IntentEnvelope (EV-1..7) before it leaves P4.
  reply: chitchat/out_of_scope go through the preset responder (never the
    LLM, never an echo of the user's words).

What this does NOT own (kept in their sub-tasks, injected here): the LLM
call + grammar + prompt assembly (tier2_fn, GWY-P4-37/12/10); live state
for G-query data (GWY-P4-39); DB record/schedule (GWY-P4-40). The
orchestrator decides WHAT happens; those provide the moving parts.

Traps this guards (each has a mutation test):
  * a bypass that went through the classify chain -- estop would then be
    subject to overheard/LLM latency, which is the one thing it must not be
  * an L2 intent dispatched on the same turn it was heard -- the confirm
    would be cosmetic
  * a fastpath intent that touched the LLM -- burns the GPU slot and adds
    latency to a path defined as not needing it
"""
from __future__ import annotations

import re
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

from xbrain.p4_agent.classifier.keyword_matcher import (
    KeywordMatcher,
    classify_text,
)
from xbrain.p4_agent.classifier.large_class import (
    has_ptz_subject,
    resolve_large_class,
    resolve_ptz,
)
from xbrain.p4_agent.envelope.intent_envelope import IntentEnvelope
from xbrain.p4_agent.registry.intents import IntentEntry, IntentRegistry
from xbrain.p4_agent.runtime.geo_request import (
    CATALOGUE_UNAVAILABLE,
    GeoRequestError,
    is_geo_intent,
    manifest_from_state,
    resolve_geo_id,
    to_geo_command,
)
from xbrain.p4_agent.runtime.intent_dispatch import (
    CMD_MOTION_INTENT,
    DispatchResult,
    dispatch,
)
from xbrain.p4_agent.runtime.mode_request import (
    ModeRequestError,
    is_mode_intent,
    to_mode_command,
)
from xbrain.p4_agent.slots.motion_slots import (
    ground_llm_slots,
    missing_motion_slot,
    motion_slots,
)
from xbrain.p4_agent.slots.name_slots import (
    canonical_name,
    ground_name,
    name_length_ok,
)
from xbrain.common.geo import HOME_WAYPOINT_NAME  # 99 U128 B08 pre-check
from xbrain.p4_agent.slots.ptp_slots import PtpParse, parse_ptp
from xbrain.p4_agent.registry.time_expr import (
    TimeExpr,
    TimeParseError,
    resolve_epoch,
)
from xbrain.p4_agent.runtime.turn_labels import (
    check_turn,
    mode_source,
    origin,
    task_source,
    teach_issuer_channel,
)
from xbrain.p4_agent.runtime.motion_intent_request import (
    MotionIntentError,
    is_motion_intent,
    to_motion_intent,
)
from xbrain.p4_agent.runtime.system_request import (
    is_system_intent,
    to_system_command,
)
from xbrain.p4_agent.runtime.task_control_request import (
    TaskControlError,
    current_task_from_state,
    is_task_control_intent,
    to_task_control_command,
)
from xbrain.p4_agent.slots.record_slots import RecordParse, parse_record  # 99 U133
from xbrain.p4_agent.runtime.task_request import (
    DESTINATION_UNRESOLVED,
    GEO_REF_SLOT,
    GOTO_NEEDS_WAYPOINT,
    PATROL_NEEDS_ROUTE,
    TaskRequestError,
    is_task_create_intent,
    missing_destination,
    to_task_command,
)
from xbrain.p4_agent.runtime.teach_request import (
    TeachRequestError,
    is_teach_intent,
    needs_pose_snap,
    session_id_from_state,
    to_teach_command,
)
from xbrain.p4_agent.safety_bypass import matcher as bypass_matcher
from xbrain.p4_agent.safety_bypass import recording_gate
from xbrain.p4_agent.session.chitchat import ChitchatResponder, ChitchatState
from xbrain.p4_agent.session.state_machines import L2ConfirmState, L2Slot

# Spoken wording for the F-class build failures. Hot-tunable phrasing, ASCII
# punctuation per CLAUDE.md 2.2; the keys are the English reasons the builders
# raise. An unmapped reason falls back to a generic line rather than speaking
# the English text at the operator (the 2026-08-11 ORIN test heard an intent
# name read aloud, which is the same defect).
_F_REASON_CN = {
    "no recording session is open": "现在没有在录制",  # 99 U133 wording
    # 99 U133: a recording start without a full route / fence name is
    # refused, never followed by a question.
    "a recording needs a full name": "录制指令要带上完整名称, 比如: 现在开始录制1号路径, 或: 现在开始录制东门围栏",
    "save needs a name": "要保存的话请说个名字",
    # 99 U110 (user ruling 2026-11-04): a capture without a name is REFUSED,
    # not asked about -- the operator repeats the whole command with a name.
    "a keypoint needs a name": "这个点没有记录, 因为没有说名字. 请说: 把这里记为某某点",
    # 16 S8.0.4: a name is 2-12 characters, refused rather than cut (99 U109).
    "name length": "名字要两到十二个字, 请换个名字",
    # 18 PS-6 / 16 EV-6: no pose near the first word -> nothing is stored.
    "no pose snapshot": "现在拿不到定位, 这个点没有记录",
    CATALOGUE_UNAVAILABLE: "对象列表还没加载好, 稍后再说一次",
    # 16 S6.6.4: B01's miss branch ends in a clarify. Reached when neither the
    # fast leg nor the tier-2 LLM leg could name the destination.
    GOTO_NEEDS_WAYPOINT: "要去哪个点? 请说出点位名称",
    # 11 S7.2 patrol needs route_id; the operator named none -> ask.
    PATROL_NEEDS_ROUTE: "巡逻哪条路线? 请说出路线名称",
    # A named destination the turn did not resolve (should not be reachable:
    # _build_task_command resolves first). Spoken as "not found" either way.
    DESTINATION_UNRESOLVED: "没找到你说的地点, 换个说法试试",
}

#: 99 U110 deterministic PTP replies (ptp_slots / time_expr). ASCII
#: punctuation (charset rule); {name} / {when} are filled at the turn.
_PTP_CN = {
    "timed_geo": "删除和改名不能定时, 请去掉时间再说一遍",
    "no_wall_clock": "现在不能定时, 请直接说去哪个点",
    "past": "这个时间已经过了, 请换个时间",
    "too_far": "定时只能在一天以内, 或一周内某天的某个时刻",
    "no_target": "没有说要删除哪个点, 没有删除",
    "same_name": "新名字和原来一样, 没有改名",
    "not_found": "没找到{name}, 可以问我有哪些位置点",
    "not_found_route": "没找到{name}, 可以问我有哪些路径",  # U121 patrol
    "ack_goto": "好的,{when}去{name}",
    "ack_delete": "好的,删除{name}",
    "ack_rename": "好的,把{name}改名为{new}",
    # 99 U121 named patrol; {how} is the lap phrase of _laps_cn (U131: P3
    # counts passes and re-pushes, so a spoken count is honoured).
    "ack_patrol": "好的,{when}沿{name}{how}",
    # 99 U131 B04: the latest completed patrol again; refused while one runs.
    "ack_repeat": "好的,{when}再{how}",
    "patrol_running": "当前巡逻结束后再说",
    # 99 U128: B09 with a named charging point, and B08 with no home point.
    "ack_dock": "好的,{when}去{name}",
    "no_home": "还没有定义返航终点站, 请先把一个位置保存为返航终点站",
    # 99 U130: G48 / G49 route-membership answers.
    "geo_unloaded": "路径数据还没有加载好, 请稍后再问",
    "route_points": "{route}上有{count}个航点: {names}",
    "route_no_points": "{route}上还没有航点",
    "point_routes": "{name}在{routes}上",
    "point_no_routes": "{name}不在任何路径上",
    "route_not_found": "没找到{name}, 可以问我有哪些路径",
    # 99 U136: G50 / G51 / G52 answers.
    "nearest_point": "离你最近的航点是{name}, 直线约{dist}米",
    "no_points": "还没有记录任何航点",
    "distance_route": "去{name}约{dist}米, 沿{route}走{time}",
    "distance_straight": "去{name}直线约{dist}米{time}",
    "route_length": "{route}全长约{length}米",
    "no_pose": "现在拿不到定位, 算不出距离",
}

#: 99 U133 recording replies (ASCII punctuation, Chinese words). {name} is the
#: recording's name, {what} 路径 / 围栏.
_REC_CN = {
    "noname": "录制指令要带上完整名称, 比如: 现在开始录制1号路径, 或: 现在开始录制东门围栏",
    "name_length": "名字要两到十二个字, 请换个名字再录制",
    "taken": "已经有叫{name}的{what}了, 请换个名字再录制",
    "ack_start": "好的,开始录制{name}. 请用键盘控制机器人移动, 说退出录制结束并保存",
    "busy_start": "正在录制{name}, 不能开始新的录制. 要结束请说退出录制",
    "busy": "正在录制{name}, 录制期间只能用键盘控制机器人. 要结束请说退出录制",
    "ack_exit": "好的,退出录制, 正在保存{name}",
    "ack_cancel": "好的,取消录制{name}",  # 99 U137: no confirmation
    "not_recording": "现在没有在录制",
    "status": "正在录制{name}, 已经录了{count}个点, 长{length}米",
}
#: The F intents still accepted while recording (99 U133): exit, cancel, mark.
_REC_ALLOWED = frozenset({"record_route_stop", "record_fence_stop",
                          "record_route_cancel", "record_route_mark"})
#: The recording starts; said while recording they get busy_start.
_REC_STARTS = frozenset({"record_route_start", "record_fence_start"})

#: 99 U136: rule-parsed geometry questions -> their G ids.
_GEO_QUERY_IDS = {"nearest_point": "G50", "distance_to": "G51",
                  "route_length": "G52"}

#: F06 / F10: the captures whose name must follow their keyword.
_CAPTURE_INTENTS = frozenset({"record_waypoint", "record_dock"})

#: Linking words between a capture keyword and the name ("记录当前位置为 X").
_NAME_LINKS = ("为", "成", "作", "叫", "是", "做")


def _said_after_keyword(entry: IntentEntry, text: str) -> bool:
    """True when anything but linking words and punctuation follows the
    EARLIEST-ENDING keyword of entry in text -- i.e. the operator said a name.

    Not the longest: 18's own example "把这里记为东门岗亭" is a keyword, and
    measured live (2026-11-04) the longest match swallowed the name and the
    capture was refused as nameless. The verb phrase that ends first ("这里
    记为") is where the name starts."""
    ends = [text.index(kw) + len(kw) for kw in entry.keywords
            if kw and kw in text]  # where each matched keyword ends
    if not ends:
        return True  # reached without a keyword (tier-2 classified): unknown
    tail = text[min(ends):]  # what came after the first verb phrase
    tail = "".join(ch for ch in tail if ch.isalnum())  # no punctuation / spaces
    while tail and tail[0] in _NAME_LINKS:
        tail = tail[1:]  # "为 X" -> "X"
    return bool(tail)


def _motion_held(state: Dict[str, Any]) -> bool:
    """True when P2 reports P1 held (state/mode.motion_behavior == "hold").

    99 U111: held by the operator's "原地待命" or by a B / D mode; P2 tells
    the two apart when the release arrives. Enveloped or bare body.
    """
    body = state.get("state/mode") if isinstance(state, dict) else None
    if isinstance(body, dict) and isinstance(body.get("data"), dict):
        body = body["data"]
    return isinstance(body, dict) and body.get("motion_behavior") == "hold"


#: Type words that, said alone, name no keypoint ("删除航点").
_BARE_TYPE_WORDS = frozenset({"航点", "点位", "关键点", "位置点", "这个点"})


def _spoken_when(expr: Optional[TimeExpr]) -> str:
    """A TimeExpr read back as the operator would say it ("" for now)."""
    if expr is None:
        return ""
    if expr.delay_seconds is not None:
        sec = expr.delay_seconds
        if sec % 3600 == 0:
            return "%d小时后" % (sec // 3600)
        if sec % 60 == 0:
            return "%d分钟后" % (sec // 60)
        return "%d秒后" % sec
    day = {0: "今天", 1: "明天", 2: "后天"}.get(expr.day_offset,
                                           "%d天后" % expr.day_offset)
    hh, mm = (int(x) for x in expr.at_local.split(":"))
    return day + ("%d点" % hh) + ("%d分" % mm if mm else "")


def _lap_slots(loops: Optional[int], round_trip: bool) -> Dict[str, Any]:
    """The B02 / B04 lap slots P3 reads (99 U131, route/laps.py): loops only
    when said (P3 takes 1 otherwise, 16 S6.7.3 [3]), round_trip only when
    true -- an absent key and a default are the same answer to P3."""
    out: Dict[str, Any] = {}
    if loops is not None:
        out["loops"] = loops
    if round_trip:
        out["round_trip"] = True
    return out


def _laps_cn(loops: Optional[int], round_trip: bool) -> str:
    """The lap phrase of the ack, restating what was understood (11 AP-1):
    "巡逻" / "巡逻3圈" / "来回巡逻3次". A round trip with no count is one."""
    if round_trip:
        return "来回巡逻%d次" % (loops or 1)
    if loops is None or loops == 1:
        return "巡逻" if loops is None else "巡逻1圈"
    return "巡逻%d圈" % loops


# Reply-family intent NAMES routed to the preset responder (never LLM,
# never echo). J01/J02/I05 plus the out_of_scope sentinel (16 S11.5).
# Spoken reply when B-class control has nothing to act on. Said plainly rather
# than sending a task_id-less frame for P3 to guess at (S7.2).
# Spoken when a named route / waypoint is not in the catalogue (16 S8.1).
_NOT_FOUND_CN = {
    "route": "没找到你说的那条路径, 换个说法试试",
    "waypoint": "没找到你说的那个点位, 换个说法试试",
}

_TASK_CONTROL_CN = {
    "no_active_task": "现在没有正在执行的任务",
    # 99 U111: resume targets the latest paused (passive) task, not current.
    "no_suspended_task": "现在没有暂停中的任务可以继续",
}

# Spoken follow-ups when an A-class slot is missing (MI-2: ask, never send).
_MOTION_REASON_CN = {
    "missing_slot:distance_m": "要走多远? 请说一个米数",
    "missing_slot:angle_deg": "要转多少度?",
    "missing_slot:heading": "要朝哪个方向? 比如朝北",
}

_CHITCHAT_NAMES = frozenset({"greeting", "identity", "help", "out_of_scope",
                             "thanks", "farewell"})  # J04 / J05: 99 U101 (2026-10-30)
#: The time of day the operator SAID, for the J01 greeting variant (longest first).
_SAID_TIME = (("早上", "morning"), ("早安", "morning"), ("上午", "forenoon"),
              ("中午", "noon"), ("下午", "afternoon"), ("晚上", "evening"), ("晚安", "evening"))

# Short Chinese labels for spoken confirm/ack feedback (hot-tunable
# wording; ASCII punct). Keyed by 18 id. Only the common action/payload
# intents need one; anything absent falls back to a generic ack. This is
# feedback wording, NOT a data source -- the authoritative restate with
# slot values is GWY-P4-35 (render_restate), wired separately.
# 2026-10-26: A05 / A06 read "站起来" / "趴下" -- labels from an older id
# plan, where A05/A06 were stand/prone. In the registry they are
# move_forward / move_backward (stand and prone are A03 / A02, the bypass),
# so a robot moving forward said "好的,站起来". It went unnoticed while no
# motion frame was ever sent for A05-A10; it mattered the day they moved.
# A07-A10 added at the same time. No distance or angle in the label: 11 CMD-30
# lets TTS restate only measured values (P2's ack), never the request.
_CN_LABELS = {
    "A04": "原地待命", "A05": "前进", "A06": "后退", "A07": "左移",
    "A08": "右移", "A09": "左转", "A10": "右转", "A11": "转身",
    "A12": "调整朝向",
    "B02": "开始巡逻", "B07": "取消任务", "B08": "返航", "B09": "回充电桩",
    "D01": "打开照明灯", "D02": "关闭照明灯", "D04": "打开警笛",
    "D06": "打开爆闪灯", "D07": "关闭爆闪灯",
    "E01": "转动云台", "E05": "停止跟踪", "E06": "变焦", "E07": "云台扫描",
    "E08": "停止扫描", "E09": "调整云台转速", "H07": "重启系统",
    # F06 / F10 (99 U109): the turn only says what was asked; the result
    # ("已记录 X" or why not) follows from cmd/teach/ack (ack_speech).
    "F06": "记录这个点", "F10": "记录充电桩",
    # B05 / B06 (99 U111): say what is being done; the result follows from
    # cmd/task/ack (ack_speech) when P3 refuses it.
    "B05": "暂停任务", "B06": "继续任务",
}

#: A15 set_turn_speed ack wording per level (99 U104).
_TURN_SPEED_LABELS = {
    "faster": "机器人转速调快一点", "slower": "机器人转速调慢一点",
    "max": "机器人转速调到最快", "min": "机器人转速调到最慢",
}

#: A16 set_move_speed ack wording per level (99 U106).
_MOVE_SPEED_LABELS = {
    "faster": "移动速度调快一点", "slower": "移动速度调慢一点",
    "max": "移动速度调到最快", "min": "移动速度调到最慢",
}

# Intents whose complete stop phrases the safety bypass leaves to them
# (2026-10-25, decided as option (b): keep the keywords and CF-1 as they are,
# make the bypass yield). 18's E08 boundary table: a bare 停 / 停止 / 停下 is
# always A01, E08 is the stop carrying a PTZ qualifier; E05 "停止跟踪" is the
# same PTZ stop family. Matching estop tokens as substrings (16 S4.1) gave
# "停止扫描" / "停止环视" / "停止跟踪" to A01: the chassis stopped, the PTZ
# kept turning. Only the phrases are removed before matching; any other stop
# word in the utterance still fires.
#
# Deliberately NOT here (yet):
#   * D12 / C04 "停止喊话" "停止播报": p2 drops D12 today (payload_wiring
#     handles D01/D02/D06/D07/D10/D17/D18 only), so yielding would turn
#     "misread as estop, speaker keeps playing" into "answered OK, speaker
#     keeps playing" -- a new false success (CLAUDE.md 3.2). They join once
#     D12's downstream (18 S6.3A.1 route 2) exists.
#   * B05 / B07 / B12 / C07 / F02 (先停一下 / 停止任务 / 停止跟随 / 停下面对
#     / 停止记录): motion-related, where an estop still serves the operator
#     and errs to the safe side (16 S4.1).
_BYPASS_YIELD_IDS = ("E05", "E08")

# PTZ intents that are capability-BLOCKED on this head (18-B): E02 home /
# E03 preset / E04 track sit behind T-PTZ-1 (absolute positioning is a
# no-op on the PELCO-D head), E10 degree-move behind T-PTZ-3. They are
# rejected with an E_CAPABILITY spoken reply BEFORE dispatch -- never sent
# to cmd/ptz (where they would be a no-op that looks like success).
_PTZ_BLOCKED = {
    "E02": "云台归位暂时用不了,可以说向左或向右转一点",
    "E03": "云台预置位暂时用不了,可以说向左或向右转一点",
    "E04": "云台自动跟踪暂时用不了",
    "E10": "云台按角度转暂不支持,可以说向左转一点或向右转一点",
}

# A degree/angle specifier (30 du / 九十度 / 一百八十度). A PTZ move carrying
# an explicit angle is E10 ptz_move_deg -- NOT a bounded jog (E01). The head
# has no absolute positioning (T-PTZ-1) and the pulse->degree map is
# uncalibrated (T-PTZ-4), so E10 is capability-rejected. Without this guard a
# degree move keyword-matches E01 ("云台左转") and silently jogs a fixed pulse,
# so 30/90/180 du all turn the same small amount and the operator is misled
# that the angle took effect (18-B R-6: never execute a clamped/ignored angle
# silently). The number may be Arabic or Chinese numerals.
_DEGREE_RE = re.compile(r"[0-9零一二三四五六七八九十百千两]+\s*度")

# Scan cues. A ptz_move (E01) whose text ALSO says 环视/一圈/一周/扫 is really
# a scan (E07). Layer-2 keyword match cannot decide this: for "平台向左环视一周"
# the E01 keyword "平台向左" (4) TIES the E07 "环视一周" (4), and E01 (earlier in
# the registry) wins the tie -> a full-circle command executes as a single
# small move. Overriding E01->E07 when a scan cue is present fixes every
# prefix (云台/平台/镜头) uniformly, without piling more keyword strings.
_SCAN_CUES = ("环视", "一圈", "一周", "扫")


def _resolve_task_geo(items, gtype: str, spoken: str) -> str:
    """A task-create slot -> one live geo_id of the right type, or raise.

    The slot holds either a spoken NAME (fast leg, or the LLM copying the
    operator) or a geo_id: 16 S6.7.3 M3_nav tells the tier-2 LLM to answer
    with the catalogue id ("route":"r-east"). An exact live id of the right
    type is accepted as is -- it IS the existence check (16 S8.1). Anything
    else goes through the name / alias / number resolver, which raises when
    nothing (or more than one object) matches. A fabricated value such as
    "候选" or "到这边=w" (2026-10-25) is neither, and is refused.
    """
    for i in items:
        if (i.get("geo_id") == spoken and i.get("type") == gtype
                and i.get("state") != "deleted"):
            return spoken
    return resolve_geo_id(items, gtype, spoken)


def _has_ptz_degree(text: str) -> bool:
    return bool(_DEGREE_RE.search(text or ""))


def _has_scan_cue(text: str) -> bool:
    return any(c in (text or "") for c in _SCAN_CUES)


def refine_ptz_intent(intent_id: str, text: str) -> str:
    """Post-classify PTZ correction (18-B). Three fixes a flat keyword match
    cannot make:

      * PTZ-prefix reclaim: a 云台-prefixed command that a chassis keyword
        stole (A13 '慢一点' <- '云台旋转速度慢一点', matched at layer 2 before
        the large-class router could run) is reclaimed to the PTZ family, per
        the operator's prefix rule (云台 prefix -> PTZ). Only when resolve_ptz
        finds a concrete E-intent; otherwise the original stands (so a
        non-action like '云台坏了' is not forced into a move).
      * degree move -> E10 (rejected): an explicit angle is unsupported.
      * scan phrased as a move -> E07.
    """
    # Reclaim first: a non-PTZ classification with a PTZ subject present is the
    # prefix-rule violation; resolve_ptz gives the right E-intent (possibly
    # E01, which the degree/scan step below then refines further).
    if intent_id[:1] != "E" and has_ptz_subject(text):
        reclaimed = resolve_ptz(text)
        if reclaimed is not None:
            intent_id = reclaimed
    if intent_id == "E01":
        # Degree move first: an angle makes it E10 (rejected), even if it also
        # said a scan word (a degree'd scan is not a thing we support).
        if _has_ptz_degree(text):
            return "E10"
        if _has_scan_cue(text):
            return "E07"
    return intent_id

# latency_class per route (EV-7 consistency, mirrored from the envelope).
_LATENCY_BY_ROUTE = {
    "fastpath": "fastpath",
    "fastpath_then_llm": "fastpath",
    "llm": "llm",
    "bypass": "bypass",
}


def _payload_slots(intent_id: str, text: str) -> Dict[str, Any]:
    """Fill the closed-set slot for a device intent from the ASR text
    (16 S8.0.4). Returns an extra dict merged into the dispatch payload so
    the consumer (p2) gets the requested value. Empty when a slot is absent
    (e.g. D18 '换一种' -> no mode -> p2 cycles). Covers the payload
    level/mode/volume intents (D10/D17/D18) and the PTZ move/zoom/speed
    intents (E01/E06/E09)."""
    from xbrain.p4_agent.slots.payload_slots import (
        parse_light_level,
        parse_strobe_mode,
        parse_volume,
    )
    from xbrain.p4_agent.slots.ptz_slots import (
        parse_ptz_amount,
        parse_ptz_direction,
        parse_ptz_speed_level,
        parse_scan,
        parse_zoom_direction,
    )
    if intent_id == "D17":                       # set_light_bright
        level = parse_light_level(text)
        return {"level": level} if level else {}
    if intent_id == "D18":                       # set_strobe_mode
        mode = parse_strobe_mode(text)
        return {"mode": mode} if mode is not None else {}
    if intent_id == "D10":                        # set_volume
        vol = parse_volume(text)
        return {"volume": vol} if vol else {}
    if intent_id == "E01":                        # ptz_move
        d = parse_ptz_direction(text)
        out: Dict[str, Any] = {"amount": parse_ptz_amount(text)}
        if d:
            out["direction"] = d
        return out
    if intent_id == "E06":                        # ptz_zoom
        z = parse_zoom_direction(text)
        out = {"amount": parse_ptz_amount(text)}
        if z:
            out["zoom_dir"] = z
        return out
    if intent_id == "E07":                        # ptz_scan (sweep/orbit)
        return parse_scan(text)
    if intent_id == "E09":                        # set_ptz_speed
        level = parse_ptz_speed_level(text)
        return {"level": level} if level else {}
    return {}


@dataclass
class PendingConfirm:
    """An L2 intent awaiting I01/I02. Holds enough to dispatch on confirm.

    *** llm_slots is held for a reason. The turn that OPENED the confirm may
    have run tier-2 and extracted the slots (F14 rename's target, a route name);
    the confirm turn itself says only 'yes'. Dropping them here meant the
    confirmed dispatch had no slots at all -- and since the F-class and task
    builders run off the slots, a confirmed 'delete the east route' dispatched
    an EMPTY envelope: the operator was asked to confirm a deletion that was
    then never commanded. Holding them also avoids a second LLM call on an
    utterance already classified.
    """
    entry: IntentEntry
    text: str
    slot: L2Slot
    llm_slots: Optional[dict] = None


@dataclass
class OrchestratorSession:
    """Per-operator turn state. One instance lives across a dialog."""
    recording: recording_gate.RecordingState = field(
        default_factory=recording_gate.RecordingState)
    chitchat: ChitchatState = field(default_factory=ChitchatState)
    pending_confirm: Optional[PendingConfirm] = None
    # CL-2 (16): when both non-voice estop paths are unhealthy the link
    # publisher sets estop_path=down; an L1b then upgrades to L2.
    estop_path: str = "up"
    # 2026-10-30 (field test P101-P103): what the robot last SAID, for I03
    # "再说一遍 / 什么 / 没听清". Before, I03 was dispatched to cmd/audio/speak
    # and spoke the user's own words back, then "收到".
    last_reply: Optional[str] = None
    # 2026-10-30 (field test P143 / P147 / P151): the last payload level intent
    # actually dispatched (D10 volume / D17 brightness), so an elliptical "中等",
    # "开到最大", "调到最低" continues THAT adjustment instead of "没太听清".
    last_level_intent: Optional[str] = None


@dataclass
class TurnDecision:
    """The outcome of one turn. `kind` names the branch taken."""
    kind: str
    intent_id: Optional[str] = None
    intent_name: Optional[str] = None
    route: Optional[str] = None
    auth: Optional[str] = None
    level: Optional[str] = None
    layer: Optional[str] = None
    bypass_action: Optional[str] = None
    dispatch_result: Optional[DispatchResult] = None
    envelope: Optional[IntentEnvelope] = None
    reply_text: Optional[str] = None
    tts_text: Optional[str] = None
    # Observability for the mutation tests: did this turn touch the LLM /
    # assemble a prompt? A fastpath turn must show both false.
    llm_used: bool = False
    prompt_assembled: bool = False
    # The entry this turn came in on (mic_local | cloud | wecom | hmi), set by
    # handle_turn. decision_to_publishes needs it for frames it builds itself
    # (the bypass posture's S9.3.4 origin), which never pass the builders
    # that read self._turn_channel.
    channel: str = "mic_local"


@dataclass(frozen=True)
class Tier2Classification:
    """What the tier-2 LLM classify returns: the picked intent NAME plus the
    slots the model extracted (e.g. speak_custom's `text`, a route `name`).
    The slots are what the fastpath could not fill -- carrying them is why
    tier-2 returns this instead of a bare name (the free-text slot would be
    lost otherwise).

    name is "unknown" when the model declined (99 U109: the grammar can say so
    now); reason is then its 16 S8.0.3 first-level code, else None."""
    name: str
    slots: dict
    reason: Optional[str] = None


# tier2_fn(text, session, now_mono_ms) -> Tier2Classification or None. None
# means no confident mission / the GPU gate denied the call / the LLM failed
# (the caller declines the turn). Injected so the orchestrator stays testable
# without a live LLM; main_wiring binds it to llm_tier2_fn.build_tier2_fn.
# Calling it is what marks a turn as LLM-using.
Tier2Fn = Callable[[str, "OrchestratorSession", int],
                   Optional[Tier2Classification]]


#: H-class system intents with NO executor today (no cmd/system subscriber in
#: the repo; H04's cmd/task fallback is dropped by P3). Remove an id here in the
#: same change that wires its executor. 2026-10-30, field test R007-R016.
#: H03 left the set the same day (99 U102): "校时" is answered by system_fn.
_UNWIRED_SYSTEM = frozenset({"H01", "H02", "H04", "H05", "H06", "H07", "H08"})

#: Action intents that are classified and accepted by P2 but change nothing:
#: A13 set_speed_profile -- P2 only records the requested profile and P1 runs
#: a fixed PROFILE_REQ, so "慢一点 / 快一点 / 全速" never changed a speed (and
#: answered "这个模式指令没听清", the profile slot was never parsed). 2026-11-02,
#: 99 U105: say so instead, until the profile reaches P1.
#: B10 skip_waypoint was here until 2026-11-05: the user ruled it out of
#: scope ("不支持, 不需要实现, 删除", 99 U129) and the intent itself was removed
#: from the registry, so there is nothing left to answer as "not connected".
_UNWIRED_ACTIONS = frozenset({"A13"})

#: Level intents whose adjustment an elliptical phrase may continue (D10
#: volume, D17 brightness), and the phrases that name no device. 2026-10-30.
_LEVEL_INTENTS = frozenset({"D10", "D17"})
_ELLIPTICAL_LEVELS = frozenset({
    "中等", "一半", "开到最大", "调到最大", "调到最高", "调到最低", "调到最小",
    "最大", "最小", "最高", "最低", "开到最小", "大一点", "小一点"})

#: Reply for a G query in a build with no query_fn (smoke / tests). Production
#: answers every G id from query_templates.yaml (state/status_answers.py).
_G_NO_QUERY_FN = "这项查询还没有接通数据,暂时答不了"


class TurnOrchestrator:
    """Runs 16 S5.2 for one utterance. Stateless across turns except for
    what the caller carries in OrchestratorSession."""

    def __init__(
        self,
        registry: IntentRegistry,
        *,
        chitchat: ChitchatResponder,
        tier2_fn: Tier2Fn,
        l2_timeout_ms: int,
        matcher: Optional[KeywordMatcher] = None,
        query_fn: Optional[Callable[[IntentEntry], Optional[str]]] = None,
        # H-class intents answered from live state instead of a cmd/system
        # dispatch (2026-10-30, 99 U102: H03 "校时"). Takes the TEXT too: the
        # forced variant ("强制校时") is told apart by its words.
        system_fn: Optional[Callable[[IntentEntry, str], Optional[str]]] = None,
        # A snapshot of the state keys P4 subscribes, keyed by zenoh key.
        # Named state_fn and not geo_state_fn (its 2026-08-20 name): it started
        # out serving only state/geo/manifest, but the B-class task control
        # added 2026-08-23 reads state/task from the same snapshot, and two
        # accessors over one snapshot is how they drift.
        state_fn: Optional[Callable[[], Dict[str, Any]]] = None,
        # The bypass word groups (16 S4) from the resolved config, unioned
        # with the built-in tables (matcher.tokens_from_config). None = the
        # built-in tables alone -- tests and tools without a config.
        bypass_tokens: Optional[bypass_matcher.BypassTokens] = None,
        # 18 S0.4.1 PS-6 (99 U109): (t_first_word, now_mono_ms) -> the
        # pose_snap block of the robot's position when the operator started
        # speaking, or None when there is none. A keypoint capture (F06 / F10)
        # is refused without one (16 EV-6): reading the pose when the command
        # is built would store where the robot is after the LLM, not "here".
        pose_snap_fn: Optional[Callable[[int, int],
                                        Optional[Dict[str, Any]]]] = None,
        # 99 U110: the wall clock a timed goto ("下午3点去 X") is scheduled
        # against -- the robot's system time by the 2026-10-26 user ruling,
        # the same clock G24 speaks -- and the site timezone a clock time is
        # read in (common.timezone; None = the system zone). No clock = timed
        # commands are refused, never scheduled against a guess.
        wall_now_fn: Optional[Callable[[], float]] = None,
        site_timezone: Optional[str] = None,
        membership_fn: Optional[Callable[[str, str, Any], Any]] = None,
        geo_query_fn: Optional[Callable[..., Any]] = None,
    ) -> None:
        self._registry = registry
        self._matcher = matcher or KeywordMatcher(registry)
        self._chitchat = chitchat
        self._tier2 = tier2_fn
        self._l2_timeout_ms = l2_timeout_ms
        # 2026-10-26: the constructor `source` ("voice" by default, the one
        # value every command was labelled with) is gone. One orchestrator
        # serves the microphone, cloud text, cloud voice and the HMI, so
        # provenance is a property of the TURN: handle_turn takes channel +
        # modality, and each command asks runtime/turn_labels for the value
        # its own contract spells (task / mode / system / geo / teach).
        # The entry channel of the turn being handled (11 MO-2 closed set),
        # set by handle_turn for exactly one turn. One orchestrator serves the
        # mic, cloud text, cloud voice and HMI alike, so the channel cannot be
        # a constructor constant (2026-10-26: it was -- the removed `source`
        # -- and every motion frame said mic_local). Turns run on one worker thread,
        # so a per-turn attribute is safe; handle_turn sets it every turn.
        self._turn_channel = "mic_local"
        self._turn_modality = "voice"  # voice | text, set with the channel
        # The turn's first-word moment (18 PS-1), set with the channel; the
        # turn's own clock when the entry did not know it.
        self._turn_t_first = 0
        self._pose_snap_fn = pose_snap_fn
        self._wall_now_fn = wall_now_fn
        self._site_timezone = site_timezone
        # 99 U130: G48 / G49 answers (state/route_membership.make_membership_fn,
        # built with common.geo.route_membership_m); None answers "not loaded".
        self._membership_fn = membership_fn
        # 99 U136: G50 / G51 / G52 answers (state/geo_queries.make_geo_query_fn,
        # built with common.geo.route_membership_m); None answers "not loaded".
        self._geo_query_fn = geo_query_fn
        # 99 U110: the restatement of a rule-parsed PTP command, set by
        # _route_ptp and read by _dispatch_ack in the same turn.
        self._turn_ptp_ack: Optional[str] = None
        # GWY-P4-39/41: for a G-class query, query_fn returns the answer
        # rendered from LIVE state (or an 'unknown' reply when stale). When
        # it is None or returns None, the query falls through to a normal
        # dispatch (the data source for that G id is not wired yet).
        self._query_fn = query_fn
        self._system_fn = system_fn  # H03 answer (None: the unwired reply)
        # 11 S12A.1: the F class needs two pieces of LIVE state that only exist
        # outside this class -- the open recording session (state/teach) and the
        # object catalogue (state/geo/manifest). Injected as one getter for the
        # same reason query_fn is: the orchestrator stays a pure decision
        # function and the wiring owns the caches. None means neither is wired,
        # and every F intent then answers with the reason rather than sending a
        # command built on a guess.
        self._state_fn = state_fn
        # The qualified stop phrases the safety bypass must leave to their own
        # intent (see _BYPASS_YIELD_IDS): taken from the registry, so the
        # keyword table stays the single source, and filtered to the ones
        # the bypass would actually fire on.
        self._bypass_tokens = bypass_tokens
        self._bypass_yield = tuple(
            kw for iid in _BYPASS_YIELD_IDS
            for kw in registry.by_id(iid).keywords
            if bypass_matcher.is_estop_phrase(kw, bypass_tokens))

    # -- public entry ----------------------------------------------------

    def handle_turn(self, text: str, session: OrchestratorSession,
                    now_mono_ms: int,
                    raw_text: Optional[str] = None,
                    addressed: bool = False,
                    channel: str = "mic_local",
                    modality: str = "voice",
                    t_first_word: Optional[int] = None) -> TurnDecision:
        """One turn (see _handle_turn), then remember what was SAID for I03.

        Recorded here, on the way out, so every branch below -- reply,
        dispatch ack, confirm prompt, denial -- feeds "再说一遍" from one
        place. A repeat does not overwrite it (repeating twice repeats the
        same answer, not "the repeat")."""
        decision = self._turn_with_channel(text, session, now_mono_ms, raw_text=raw_text,
                                     addressed=addressed, channel=channel,
                                     modality=modality,
                                     t_first_word=t_first_word)
        if decision.tts_text and decision.intent_id != "I03":
            session.last_reply = decision.tts_text
        if decision.kind == "dispatch" and decision.intent_id in _LEVEL_INTENTS:
            session.last_level_intent = decision.intent_id
        return decision

    def _turn_with_channel(self, text: str, session: OrchestratorSession,
                     now_mono_ms: int,
                     raw_text: Optional[str] = None,
                     addressed: bool = False,
                     channel: str = "mic_local",
                     modality: str = "voice",
                     t_first_word: Optional[int] = None) -> TurnDecision:
        """Process one ASR utterance end-to-end. See module docstring for
        the ordering (16 S5.2).

        text is what the classifier sees (the ASR post-processed text on the
        voice path). raw_text, when given, is the ASR output BEFORE 16 S3
        post-processing; the safety bypass matches it first (16 S4 constraint
        1). None means there was no post-processing (text channel).

        addressed=True waives the layer-5 directional cue for a channel that
        is directed by construction (cmd/voice_text); every microphone path,
        including cloud audio, leaves it False. It changes layer 5 only: the
        bypass and the confirm gate run exactly as for voice.

        channel is the entry the turn came in on (mic_local | cloud | wecom |
        hmi) and modality how (voice | text). Together they label every
        command the turn builds: the motion frame's channel (MO-2), cmd/task
        source, cmd/mode source, cmd/system / cmd/geo origin and cmd/teach
        issuer.channel (turn_labels). The defaults are the microphone path;
        the text and cloud entries pass theirs explicitly. An out-of-set or
        impossible pair (mic_local + text, hmi + voice) raises.

        t_first_word is when the operator started speaking, in monotonic ms
        (18 PS-1: the first frame of the utterance; the arrival of a typed
        command) -- a timestamp, hence no unit suffix for the safety-default
        scan to read as a limit. A keypoint capture takes its position from
        it. None = now_mono_ms.
        """
        check_turn(channel, modality)
        # Set, not saved and restored: every turn sets it here before anything
        # reads it, and nothing reads it outside a turn, so a previous turn's
        # value (even one that raised) can never be seen. A save/restore was
        # tried and is an EQUIVALENT mutant -- no test can tell it apart --
        # so it is not kept (CLAUDE.md 7.2.1).
        self._turn_channel = channel
        self._turn_modality = modality
        self._turn_t_first = (now_mono_ms if t_first_word is None
                              else t_first_word)
        self._turn_ptp_ack = None
        decision = self._handle_turn(text, session, now_mono_ms, raw_text,
                                     addressed)
        decision.channel = channel  # every branch, one place
        return decision

    def _handle_turn(self, text: str, session: OrchestratorSession,
                     now_mono_ms: int, raw_text: Optional[str],
                     addressed: bool) -> TurnDecision:
        """handle_turn's body, with the turn's channel already set."""
        text = (text or "").strip()
        raw = (raw_text or "").strip() if raw_text is not None else text
        # 99 U133: the U45 recording flag is the live state/teach session.
        # Until 2026-11-05 nothing ever set it, so the U45 suppression below
        # never fired.
        session.recording.in_recording = self._recording() is not None

        # STEP 1: safety-bypass BEFORE classification (16 S4). Match the raw
        # ASR text, then the corrected text: correction may ADD a stop word
        # (a mangled "停止" restored) but must never hide one the raw had.
        # Qualified PTZ stops ("停止扫描") yield to their intent first; any
        # stop word left after removing them still fires (_BYPASS_YIELD_IDS).
        hit = (bypass_matcher.match_raw(raw, self._bypass_yield,
                                        self._bypass_tokens)
               or bypass_matcher.match_normalized(text, self._bypass_yield,
                                                  self._bypass_tokens))
        if hit is not None:
            supp = recording_gate.evaluate(session.recording, hit.action)
            if supp is not None:
                # U45: voice estop suppressed while recording; still logged
                # + advise the handle. NOT dispatched as a stop.
                return TurnDecision(
                    kind="bypass_suppressed",
                    bypass_action=hit.action,
                    tts_text=supp.tts_advice)
            # Straight to Tier1 (bypass route). No classify, no envelope
            # gating -- that is the safety guarantee.
            return TurnDecision(
                kind="bypass", bypass_action=hit.action, route="bypass")

        # STEP 3 (early): if a confirm is open, THIS turn is its response.
        if session.pending_confirm is not None:
            return self._resolve_pending_confirm(text, session, now_mono_ms)

        if not text:
            return TurnDecision(kind="overheard", layer="overheard")

        # 99 U133: recording sentences (start with a full name / exit / status)
        # by rule, before every other layer -- a bare F01 keyword would start
        # a nameless recording, which the ruling forbids.
        rp = parse_record(text)
        if rp is not None:
            return self._route_record(rp, text, session, now_mono_ms)

        # 2026-10-30 (field test P236-P247): light requests the hardware cannot
        # do (red/blue dimming, strobe rate, searchlight strobe, pattern outside
        # 1..16) are answered with the reason BEFORE classification -- they used
        # to classify as D17 and change the searchlight, or get "没太听清".
        # After the safety bypass, so a stop word is never shadowed.
        from xbrain.p4_agent.classifier.payload_limits import limit_hit
        limit = limit_hit(text)
        if limit is not None:
            reply = self._chitchat.limit_reply(limit)
            return TurnDecision(kind="denied", layer="payload_limits",
                                reply_text=reply, tts_text=reply)

        # Elliptical level phrases name no device ("中等", "开到最大", "调到最低"):
        # continue the volume / brightness adjustment made earlier in this
        # session, or ask which one -- never "没太听清" (field test P143-P151).
        if text.rstrip("吧啊了呀.!\u3002\uff01") in _ELLIPTICAL_LEVELS:  # trailing particles / CJK stop and bang
            if session.last_level_intent is not None:
                entry = self._registry.by_id(session.last_level_intent)
                return self._route_entry(entry, text, session, now_mono_ms,
                                         layer="session_context")
            reply = self._chitchat.session_reply("which_device")
            return TurnDecision(kind="reply", layer="session_context",
                                reply_text=reply, tts_text=reply)

        # 99 U110: the three PTP commands (go to X / delete X / rename X to Y)
        # are parsed by rule before any keyword or LLM layer -- the LLM legs
        # for them invented catalogue ids (ptp_slots module docstring). After
        # the safety bypass and an open confirm, so neither can be shadowed.
        ptp = parse_ptp(text)
        if ptp is not None:
            return self._route_ptp(ptp, text, session, now_mono_ms)

        # STEPS 2-6: the priority chain. large_class_fn supplies layer 4
        # (16 S5.2): the deterministic device-family router that rescues
        # keyword MISSES for PTZ (E) and payload (D) so a slightly-reworded
        # command ("云台朝左", "把照明灯搞亮点") still resolves. It runs only
        # when layer 2 missed, so exact keywords are never overridden.
        result = classify_text(text, self._registry, self._matcher,
                               large_class_fn=resolve_large_class,
                               addressed=addressed)
        if result.layer == "overheard":
            # 16 S5.2.1: not addressed to the robot -> completely silent.
            return TurnDecision(kind="overheard", layer="overheard")
        if result.fires_llm:
            # Layer 6: nothing matched but it was directed -> tier-2 LLM.
            return self._run_tier2(text, session, now_mono_ms)

        # Layers 2/3/4 matched an intent id. Apply the PTZ post-classify
        # correction (18-B): a degree move -> E10 (rejected), a scan phrased
        # as a move -> E07. A flat keyword match cannot make either call.
        intent_id = refine_ptz_intent(result.intent, text)
        entry = self._registry.by_id(intent_id)
        return self._route_entry(entry, text, session, now_mono_ms,
                                 layer=result.layer)

    # -- recording mode (99 U133) ---------------------------------------

    def _recording(self) -> Optional[Dict[str, Any]]:
        """The open recording session from state/teach when it is in
        recording mode (arming / recording / paused), else None. Read from
        the same state P4 already uses to name the session (S12A.4)."""
        state = self._state_fn() if self._state_fn else {}
        body = state.get("state/teach")
        sess = body.get("session") if isinstance(body, dict) else None
        if not isinstance(sess, dict) or sess.get("state") not in (
                "arming", "recording", "paused"):
            return None
        stats = body.get("stats") if isinstance(body.get("stats"), dict) else {}
        return {"name": sess.get("name") or sess.get("name_hint") or "",
                "kind": sess.get("kind"), "stats": stats}

    def _rec_reply(self, intent_id: str, key: str, layer: str,
                   **fmt: Any) -> TurnDecision:
        """A spoken recording reply; nothing is sent."""
        entry = self._registry.by_id(intent_id)
        spoken = _REC_CN[key].format(**fmt)
        return TurnDecision(kind="reply", intent_id=entry.id,
                            intent_name=entry.name, route=entry.route,
                            auth=entry.auth, layer=layer,
                            reply_text=spoken, tts_text=spoken)

    def _recording_block(self, entry: IntentEntry,
                         layer: str) -> Optional[TurnDecision]:
        """99 U133: while recording, everything but queries (G), session
        answers (I), chit-chat (J) and the recording's own exit / cancel /
        mark is answered and not sent -- the robot is the keyboard's."""
        rec = self._recording()
        if rec is None:
            return None
        if (entry.id[:1] in ("G", "I", "J") or entry.name in _CHITCHAT_NAMES
                or entry.name in _REC_ALLOWED):
            return None
        key = "busy_start" if entry.name in _REC_STARTS else "busy"
        return self._rec_reply(entry.id, key, layer, name=rec["name"])

    def _route_record(self, rp: RecordParse, text: str,
                      session: OrchestratorSession,
                      now_mono_ms: int) -> TurnDecision:
        """A rule-parsed recording sentence (record_slots): start with a full
        name, exit-and-save, or a status question. Refusals are spoken and
        nothing is sent; no follow-up question is ever asked (99 U133)."""
        rec = self._recording()
        if rp.kind == "status":
            if rec is None:
                return self._rec_reply("F02", "not_recording", "record_rule")
            stats = rec["stats"]
            return self._rec_reply(
                "F02", "status", "record_rule", name=rec["name"],
                count=int(stats.get("point_count") or 0),
                length=int(round(float(stats.get("length_m") or 0.0))))
        if rp.kind == "cancel":
            # 99 U137: discard at once, no "确认吗" (F04 is L1a now)
            if rec is None:
                return self._rec_reply("F04", "not_recording", "record_rule")
            self._turn_ptp_ack = _REC_CN["ack_cancel"].format(name=rec["name"])
            return self._route_entry(self._registry.by_id("F04"), text,
                                     session, now_mono_ms, layer="record_rule")
        if rp.kind == "exit":
            if rec is None:
                return self._rec_reply("F02", "not_recording", "record_rule")
            intent_id = "F08" if rec["kind"] == "fence" else "F02"
            self._turn_ptp_ack = _REC_CN["ack_exit"].format(name=rec["name"])
            return self._route_entry(self._registry.by_id(intent_id), text,
                                     session, now_mono_ms, layer="record_rule")
        if rec is not None:
            return self._rec_reply("F01", "busy_start", "record_rule",
                                   name=rec["name"])
        if rp.kind == "start_noname":
            return self._rec_reply("F01", "noname", "record_rule")
        intent_id = "F07" if rp.rtype == "fence" else "F01"
        self._turn_ptp_ack = _REC_CN["ack_start"].format(name=rp.name)
        return self._route_entry(self._registry.by_id(intent_id), text,
                                 session, now_mono_ms, layer="record_rule",
                                 rule_slots={"name": rp.name})

    # -- routing ---------------------------------------------------------

    def _route_entry(self, entry: IntentEntry, text: str,
                     session: OrchestratorSession, now_mono_ms: int,
                     layer: str,
                     llm_slots: Optional[dict] = None,
                     rule_slots: Optional[dict] = None) -> TurnDecision:
        """Apply route (fastpath vs LLM) and auth (confirm gate). llm_slots
        (tier-2 only) are the slots the LLM extracted; they are merged OVER the
        fastpath slots so a free-text slot (speak_custom's text) survives."""
        # 99 U133: recording mode answers every control command first.
        blocked_rec = self._recording_block(entry, layer)
        if blocked_rec is not None:
            return blocked_rec
        # Reply-family intents (chitchat / help) answer from a preset and
        # never dispatch an action. Handled before the auth gate: they are
        # all L0 and produce speech, not motion.
        if entry.name in _CHITCHAT_NAMES:
            return self._reply_chitchat(entry, session, layer, text)

        # *** H01-H08 system intents (2026-10-30, field test R007-R016): nothing in
        # the repo consumes cmd/system (intent_dispatch: "ROUTED but not yet
        # CONSUMED"), and H04 falls back to cmd/task, which P3 drops. They used
        # to answer "收到" -- or ask "确认吗" for reboot and then do nothing --
        # i.e. pretend success. Until an executor exists, say so and send
        # nothing (CLAUDE.md 3.2). Before the auth gate: no confirm for a no-op.
        # *** H03 "校时" (2026-10-30, 99 U102 user ruling): ANSWERED, not
        # dispatched -- the sync status from state/clock, whether chronyd keeps
        # the clock, and the time to the second (state/time_sync_answer.py);
        # "强制校时" (a wall-clock step) answers "not wired" from the same
        # function. Nothing goes on cmd/system: its only active step (chronyc
        # burst) needs root, which the ruling excludes. Before the auth gate:
        # a reading needs no confirm (L1b / L2 were for the command).
        if entry.id == "H03":
            answer = self._system_fn(entry, text) if self._system_fn is not None else None
            if answer is None:
                answer = self._chitchat.unwired_reply(entry.id)
            return TurnDecision(
                kind="reply", intent_id=entry.id, intent_name=entry.name,
                route=entry.route, auth=entry.auth, layer=layer,
                reply_text=answer, tts_text=answer)

        if entry.id in _UNWIRED_SYSTEM or entry.id in _UNWIRED_ACTIONS:
            reply = self._chitchat.unwired_reply(entry.id)
            return TurnDecision(
                kind="denied", intent_id=entry.id, intent_name=entry.name,
                route=entry.route, auth=entry.auth, layer=layer,
                reply_text=reply, tts_text=reply)

        # *** I01-I04 session intents (2026-10-30, field test P101-P106): the I
        # class dispatches to cmd/audio/speak, so these used to SPEAK THE USER'S
        # OWN WORDS and then "收到". A pending confirm consumes I01 / I02 before
        # this point; reaching here means there is nothing to confirm.
        if entry.id in ("I01", "I02", "I03", "I04"):
            if entry.id == "I03":
                reply = session.last_reply or self._chitchat.session_reply("repeat_none")
            elif entry.id == "I04":
                # No pending_confirm to drop here: a pending confirm consumes
                # the next turn before routing (and clears itself), so this
                # branch never sees one -- clearing it here was dead code (an
                # equivalent mutant, 2026-10-30).
                reply = self._chitchat.session_reply("cancel_session")
            else:
                reply = self._chitchat.session_reply("no_pending_confirm")
            return TurnDecision(
                kind="reply", intent_id=entry.id, intent_name=entry.name,
                route=entry.route, auth=entry.auth, layer=layer,
                reply_text=reply, tts_text=reply)

        # Capability-blocked PTZ intents (18-B T-PTZ-1/3): reject with a
        # spoken reason and do NOT dispatch -- they are a no-op on the head,
        # and a silent dispatch would look like success.
        blocked = _PTZ_BLOCKED.get(entry.id)
        if blocked is not None:
            return TurnDecision(
                kind="denied", intent_id=entry.id, intent_name=entry.name,
                route=entry.route, auth=entry.auth, layer=layer,
                tts_text=blocked)

        # G-class query: answer from LIVE state (GWY-P4-39). query_fn
        # returns the rendered answer (or an 'unknown' reply when the state
        # is stale); None means the data source for this G id is not wired,
        # so fall through to a normal dispatch.
        # *** 2026-10-30 (field test R001-R006): a G query is NEVER dispatched.
        # The G class dispatches to cmd/audio/speak, so a G id with no answer
        # used to SPEAK THE USER'S OWN WORDS, cut off by the "收到" ack -- the
        # "复读半句后改说收到" the testers heard. No answer -> an honest no-data
        # reply instead (query_templates.yaml query_no_data, via status_answers).
        # In production make_status_query_fn (composed last) answers EVERY G id
        # from query_templates.yaml; this constant is only the last resort of a
        # build with no query_fn at all, and it still never echoes.
        if entry.id[:1] == "G":
            answer = self._query_fn(entry) if self._query_fn is not None else None
            if answer is None:
                answer = _G_NO_QUERY_FN
            return TurnDecision(
                kind="reply", intent_id=entry.id, intent_name=entry.name,
                route=entry.route, auth=entry.auth, layer=layer,
                reply_text=answer, tts_text=answer)

        # Effective auth with CL-2 upgrade (L1b -> L2 when estop down).
        eff_auth = self._effective_auth(entry.auth, session)

        if eff_auth == "L2":
            # Open a confirm and WAIT. Do NOT dispatch this turn.
            slot = L2Slot(timeout_millis=self._l2_timeout_ms)
            slot.request(now_mono_ms=now_mono_ms)
            session.pending_confirm = PendingConfirm(
                entry=entry, text=text, slot=slot,
                llm_slots=llm_slots if llm_slots is not None else rule_slots)
            return TurnDecision(
                kind="await_confirm", intent_id=entry.id,
                intent_name=entry.name, route=entry.route, auth=eff_auth,
                layer=layer, tts_text=self._confirm_prompt(entry))
        if eff_auth == "L3":
            # Cloud approval flow (GWY-P4-38 hand-off to P5). Not dispatched
            # until the cloud confirm_token arrives.
            return TurnDecision(
                kind="await_approval", intent_id=entry.id,
                intent_name=entry.name, route=entry.route, auth=eff_auth,
                layer=layer, tts_text=self._approval_prompt(entry))

        # L0 / L1a / L1b -> dispatch now. Pass through any tier-2 LLM slots.
        return self._dispatch_entry(entry, text, session, now_mono_ms,
                                    layer, eff_auth, llm_slots=llm_slots,
                                    rule_slots=rule_slots)

    def _route_ptp(self, ptp: PtpParse, text: str,
                   session: OrchestratorSession,
                   now_mono_ms: int) -> TurnDecision:
        """Route a rule-parsed PTP command (99 U110 / U121) to B01 / B02 /
        F12 / F14.

        Everything the sentence said is already a slot; the catalogue decides
        whether the name exists (B01 in _build_task_command, F12 / F14 in
        to_geo_command). Refusals are spoken and nothing is sent: a timed
        delete or rename, a time that has passed, no clock to schedule on, a
        delete that names no keypoint, a bad new name.
        """
        def _reply(intent_id: str, key: str, **fmt) -> TurnDecision:
            entry = self._registry.by_id(intent_id)
            spoken = _PTP_CN[key].format(**fmt)
            return TurnDecision(kind="reply", intent_id=entry.id,
                                intent_name=entry.name, route=entry.route,
                                auth=entry.auth, layer="ptp_rule",
                                reply_text=spoken, tts_text=spoken)

        rec = self._recording()
        if rec is not None and ptp.kind not in ("route_points", "point_routes",
                                                "nearest_point", "distance_to",
                                                "route_length"):
            # 99 U133: go / patrol / delete / rename are control commands
            return self._rec_reply("B01", "busy", "ptp_rule", name=rec["name"])
        if ptp.kind == "goto":
            slots: Dict[str, Any] = {"waypoint": ptp.name}
            if ptp.when is not None:
                if self._wall_now_fn is None:
                    return _reply("B01", "no_wall_clock")
                try:
                    slots["not_before_ts"] = resolve_epoch(
                        ptp.when, self._wall_now_fn(), self._site_timezone)
                except TimeParseError as exc:
                    return _reply("B01", "past" if "passed" in str(exc)
                                  else "too_far")
            self._turn_ptp_ack = _PTP_CN["ack_goto"].format(
                when=_spoken_when(ptp.when), name=ptp.name)
            return self._route_entry(self._registry.by_id("B01"), text,
                                     session, now_mono_ms, layer="ptp_rule",
                                     rule_slots=slots)
        if ptp.kind in ("route_points", "point_routes"):
            return self._answer_membership(ptp, _reply)  # 99 U130: G48 / G49
        if ptp.kind in _GEO_QUERY_IDS:
            return self._answer_geo_query(ptp, _reply)  # 99 U136: G50 / G51 / G52
        if ptp.kind == "dock":
            # 99 U128: a NAMED charging point -- that dock, however far. The
            # name is resolved like a goto waypoint (GEO_REF_SLOT goto_dock),
            # and P3 drives the route-aware goto to it.
            dslots: Dict[str, Any] = {"dock": ptp.name}
            if ptp.when is not None:
                if self._wall_now_fn is None:
                    return _reply("B09", "no_wall_clock")
                try:
                    dslots["not_before_ts"] = resolve_epoch(
                        ptp.when, self._wall_now_fn(), self._site_timezone)
                except TimeParseError as exc:
                    return _reply("B09", "past" if "passed" in str(exc)
                                  else "too_far")
            self._turn_ptp_ack = _PTP_CN["ack_dock"].format(
                when=_spoken_when(ptp.when), name=ptp.name)
            return self._route_entry(self._registry.by_id("B09"), text,
                                     session, now_mono_ms, layer="ptp_rule",
                                     rule_slots=dslots)
        if ptp.kind == "patrol":
            # 99 U121: the route name is resolved against the catalogue in
            # _build_task_command (GEO_REF_SLOT patrol_route), exactly as a
            # goto's waypoint is. 99 U131: the lap count and the "来回" flag
            # ride in the slots; P3 turns them into passes (route/laps.py).
            pslots: Dict[str, Any] = {"route": ptp.name,
                                      **_lap_slots(ptp.loops, ptp.round_trip)}
            if ptp.when is not None:
                if self._wall_now_fn is None:
                    return _reply("B02", "no_wall_clock")
                try:
                    pslots["not_before_ts"] = resolve_epoch(
                        ptp.when, self._wall_now_fn(), self._site_timezone)
                except TimeParseError as exc:
                    return _reply("B02", "past" if "passed" in str(exc)
                                  else "too_far")
            self._turn_ptp_ack = _PTP_CN["ack_patrol"].format(
                when=_spoken_when(ptp.when), name=ptp.name,
                how=_laps_cn(ptp.loops, ptp.round_trip))
            return self._route_entry(self._registry.by_id("B02"), text,
                                     session, now_mono_ms, layer="ptp_rule",
                                     rule_slots=pslots)
        if ptp.kind == "repeat":
            # 99 U131 B04: no route on purpose -- P3 fills the latest
            # completed patrol's. The "patrol running" refusal is in
            # _build_task_command, so the keyword leg ("接着巡") gets it too.
            rslots: Dict[str, Any] = _lap_slots(ptp.loops, ptp.round_trip)
            if ptp.when is not None:
                if self._wall_now_fn is None:
                    return _reply("B04", "no_wall_clock")
                try:
                    rslots["not_before_ts"] = resolve_epoch(
                        ptp.when, self._wall_now_fn(), self._site_timezone)
                except TimeParseError as exc:
                    return _reply("B04", "past" if "passed" in str(exc)
                                  else "too_far")
            self._turn_ptp_ack = _PTP_CN["ack_repeat"].format(
                when=_spoken_when(ptp.when),
                how=_laps_cn(ptp.loops, ptp.round_trip))
            return self._route_entry(self._registry.by_id("B04"), text,
                                     session, now_mono_ms, layer="ptp_rule",
                                     rule_slots=rslots)
        if ptp.kind == "activate_fence":
            # 99 U138: F15 by name; stays L2 (the confirm turn opens below)
            if ptp.when is not None:
                return _reply("F15", "timed_geo")
            return self._route_entry(self._registry.by_id("F15"), text, session,
                                     now_mono_ms, layer="ptp_rule",
                                     rule_slots={"fence": ptp.name})
        if ptp.kind in ("delete_route", "delete_fence"):
            # 99 U133: route / fence deletes by name; F11 / F13 stay L2, so the
            # turn asks for confirmation (safety-relevant, 18 S8.1).
            fid = "F11" if ptp.kind == "delete_route" else "F13"
            if ptp.when is not None:
                return _reply(fid, "timed_geo")
            self._turn_ptp_ack = _PTP_CN["ack_delete"].format(name=ptp.name)
            return self._route_entry(
                self._registry.by_id(fid), text, session, now_mono_ms,
                layer="ptp_rule",
                rule_slots={"route" if fid == "F11" else "fence": ptp.name})
        intent_id = "F12" if ptp.kind == "delete" else "F14"
        if ptp.when is not None:
            return _reply(intent_id, "timed_geo")
        if ptp.kind == "delete":
            if not ptp.name or ptp.name in _BARE_TYPE_WORDS:
                return _reply("F12", "no_target")
            self._turn_ptp_ack = _PTP_CN["ack_delete"].format(name=ptp.name)
            return self._route_entry(self._registry.by_id("F12"), text,
                                     session, now_mono_ms, layer="ptp_rule",
                                     rule_slots={"waypoint": ptp.name})
        new = canonical_name(ptp.new or "")
        if not name_length_ok(new):
            spoken = _F_REASON_CN["name length"]
            entry = self._registry.by_id("F14")
            return TurnDecision(kind="reply", intent_id=entry.id,
                                intent_name=entry.name, route=entry.route,
                                auth=entry.auth, layer="ptp_rule",
                                reply_text=spoken, tts_text=spoken)
        if new == canonical_name(ptp.name):
            return _reply("F14", "same_name")
        self._turn_ptp_ack = _PTP_CN["ack_rename"].format(name=ptp.name, new=new)
        return self._route_entry(self._registry.by_id("F14"), text, session,
                                 now_mono_ms, layer="ptp_rule",
                                 rule_slots={"old": ptp.name, "new": new})

    def _answer_membership(self, ptp: PtpParse, reply) -> TurnDecision:
        """G48 "X路径上有哪些航点" / G49 "X在哪条路径上" (99 U130): answered
        at once by the injected membership_fn (state/route_membership.py,
        built with common.geo.route_membership_m) from state/geo/objects.
        No membership_fn (tests / tools) answers "not loaded", never a guess."""
        intent = "G48" if ptp.kind == "route_points" else "G49"
        if self._membership_fn is None:  # not configured
            return reply(intent, "geo_unloaded")
        state = self._state_fn() if self._state_fn else {}
        key, fields = self._membership_fn(ptp.kind, ptp.name, state.get("state/geo/objects"))
        return reply(intent, key, **fields)

    def _answer_geo_query(self, ptp: PtpParse, reply) -> TurnDecision:
        """G50 nearest waypoint / G51 how far to X / G52 how long is route X
        (99 U136): answered at once by the injected geo_query_fn from
        state/geo/objects, state/pose and state/mode. No geo_query_fn (tests /
        tools) answers "not loaded", never a guess."""
        intent = _GEO_QUERY_IDS[ptp.kind]
        if self._geo_query_fn is None:  # not configured
            return reply(intent, "geo_unloaded")
        state = self._state_fn() if self._state_fn else {}
        key, fields = self._geo_query_fn(
            ptp.kind, ptp.name, state.get("state/geo/objects"),
            state.get("state/pose"), state.get("state/mode"))
        return reply(intent, key, **fields)

    def _dispatch_entry(self, entry: IntentEntry, text: str,
                        session: OrchestratorSession, now_mono_ms: int,
                        layer: str, eff_auth: str,
                        llm_slots: Optional[dict] = None,
                        rule_slots: Optional[dict] = None) -> TurnDecision:
        """Route the FAST leg. fastpath / fastpath_then_llm dispatch
        directly (no LLM, no prompt assemble). A pure-llm matched intent
        still needs slot fill via tier-2. llm_slots (from a layer-6 tier-2
        classification) are merged over the fastpath slots below."""
        llm_used = llm_slots is not None       # layer-6 already ran the LLM
        # A route=='llm' matched intent needs the LLM to fill its slots -- BUT
        # only when a layer-6 classification did not already supply them (else
        # this would be a wasteful second LLM call on the same utterance).
        # 99 U110 (user ruling): a keypoint capture that names nothing after
        # its keyword ("记录当前位置", "把这里记为") is refused here, without
        # the LLM and without a follow-up question.
        if (entry.name in _CAPTURE_INTENTS and llm_slots is None
                and not _said_after_keyword(entry, text)):
            spoken = _F_REASON_CN["a keypoint needs a name"]
            return TurnDecision(kind="reply", intent_id=entry.id,
                                intent_name=entry.name, route=entry.route,
                                auth=eff_auth, level=eff_auth, layer=layer,
                                reply_text=spoken, tts_text=spoken)
        # A rule-parsed command (rule_slots, 99 U110) has its slots already.
        if entry.route == "llm" and llm_slots is None and not rule_slots:
            classified = self._tier2(text, session, now_mono_ms)
            llm_used = True
            # An "unknown / missing_slot" on a keyword-matched intent is the
            # operator naming the action without its argument ("记录当前位置"):
            # carry on with no LLM slots so the builder asks for the slot.
            # Any other decline ends the turn as before.
            asks = (classified is not None and classified.name == "unknown"
                    and classified.reason == "missing_slot")
            if classified is None or (classified.name == "unknown" and not asks):
                return TurnDecision(kind="denied", intent_id=entry.id,
                                    intent_name=entry.name, route=entry.route,
                                    auth=eff_auth, layer=layer, llm_used=True,
                                    prompt_assembled=True)
            llm_slots = classified.slots
        # Fill fastpath closed-set slots from the text (16 S8.0.4) for the
        # payload level/mode/volume intents, so p2 gets the requested value
        # (D17 level / D18 mode / D10 volume), not just the intent id.
        extra = _payload_slots(entry.id, text)
        # A05-A10 numeric slot from the text (2026-10-26): "前进三米" ->
        # distance_m 3, "左转九十度" -> angle_deg 90. Before this the fast leg
        # had no number reader and every such command ended in "要走多远?".
        fast_motion = motion_slots(entry.name, text)
        if fast_motion:
            extra = {**(extra or {}), **fast_motion}
        # 99 U110: slots a rule already read off the sentence (ptp_slots):
        # complete, so the fast leg below never enters the LLM for them.
        if rule_slots:
            extra = {**(extra or {}), **rule_slots}
        # Merge the tier-2 LLM slots OVER the fastpath ones: the LLM fills the
        # slots the fastpath regex cannot (free text, names), and the fastpath
        # fills the closed-set enums. LLM values win a key collision.
        # A numeric motion slot from the LLM must be GROUNDED in what was said
        # (motion_slots.ground_llm_slots): the 3B model invents distances for
        # vague commands, and an invented number here becomes real motion. An
        # ungrounded one is dropped and the turn asks instead (16 EV-2).
        if llm_slots:
            extra = {**(extra or {}),
                     **ground_llm_slots(entry.name, llm_slots, text)}
            # 99 U109: a made-up `name` (F03 F06 F09 F10) must also have been
            # said, and travels in its canonical spelling ("一号" -> "1号") so
            # the later lookup finds it; an invented one is dropped and the
            # builder asks for it (name_slots).
            if "name" in entry.slots and "name" in extra:
                said = ground_name(extra.pop("name"), text)
                if said is not None:
                    extra["name"] = said
        # 16 S6.6.4 fastpath_then_llm: a keyword hit that did not fill the
        # destination is NOT a finished fast leg -- it goes to the tier-2 LLM
        # leg, which may name the waypoint (or classify the utterance as
        # something else entirely). Only once, though: the LLM leg re-enters
        # here with llm_used set, and a still-missing destination then falls to
        # to_task_command, which refuses and the turn ends in the clarify
        # question (GOTO_NEEDS_WAYPOINT). Before 2026-10-24 this case went
        # straight to cmd/task as a goto with no destination -- see
        # task_request.missing_destination for how it was triggered.
        if (entry.route == "fastpath_then_llm" and not llm_used
                and missing_destination(entry.name, extra)):
            return self._run_tier2(text, session, now_mono_ms)
        # The same LLM leg for A05-A10 when the fast leg found no number
        # (2026-10-26; 16 S6.6.4 lists them with the B01 bimodal set). One
        # difference from B01: when the LLM is unavailable or declines, the
        # turn does not end as "denied" -- it falls through to the MI-2
        # clarify below ("要走多远?"), which is what the fast leg alone gave
        # before and what an operator can act on.
        if (entry.route == "fastpath_then_llm" and not llm_used
                and missing_motion_slot(entry.name, extra)):
            slow = self._run_tier2(text, session, now_mono_ms)
            if not (slow.kind == "denied" and slow.intent_name is None):
                return slow  # the LLM leg produced a real outcome
            llm_used = True  # tried once; never re-enter, ask instead
        # PB4: a task-CREATE intent (goto/patrol/charge/teach/follow) carries a
        # cmd/task REQUEST P3's ingest records into task.db -- {task_type,
        # intent, id, slots, source}. Without it the cmd/task frame is just the
        # p4_intent_v1 {intent_id, text} P3 cannot turn into a task row. Control
        # intents (pause/cancel/stop_follow) are NOT creates: to_task_request
        # returns None and the frame stays as-is (they act on an existing task).
        if is_task_create_intent(entry.name):
            # 11 S7.2 TaskCommand (migrated 2026-08-20 from the private
            # task_request shape). The turn's text lands in task.params and
            # from there in tasks.command_text (15 S9.5A.4 / 17 S6.8.4 field 3):
            # party-A requires the raw command stored for incident
            # traceability, and `text` here is what the turn acted on.
            merged, failure = self._build_task_command(entry, extra, text)
            if failure is not None:
                return TurnDecision(kind="reply", intent_id=entry.id,
                                    intent_name=entry.name, route=entry.route,
                                    auth=eff_auth, level=eff_auth, layer=layer,
                                    reply_text=failure, tts_text=failure,
                                    llm_used=llm_used,
                                    prompt_assembled=llm_used)
            extra = merged
        # 18 H class -> 11 S7.15 SystemCommand on cmd/system (2026-08-23).
        if is_system_intent(entry.name):
            sys_cmd = to_system_command(
                entry.name, slots=dict(extra or {}),
                cmd_id="c-" + uuid.uuid4().hex[:12],
                source=origin(self._turn_channel))
            if sys_cmd is not None:
                extra = {**(extra or {}), "system_command": sys_cmd}
        # 18 B class control (B05/B06/B07) -> 11 S7.2 TaskCommand, with the
        # task_id resolved HERE (2026-08-23). S7.2 forbids "omit = the current
        # task" -- it forbids the RECEIVER guessing, and resolving on the
        # sender side satisfies all four of its stated reasons. See
        # task_control_request.py.
        release_hold = False
        if is_task_control_intent(entry.name):
            _state = self._state_fn() if self._state_fn else {}
            if entry.name == "resume_task" and _motion_held(_state):
                # 99 U111: while P1 is held (the operator's "原地待命", or a
                # B / D mode), "继续" means "move again": a MotionIntent
                # resume on cmd/motion/intent. P2 releases only the operator's
                # own hold and refuses a mode's (E_BUSY mode_hold).
                release_hold = True
                extra = {**(extra or {}), "motion_intent": to_motion_intent(
                    "resume", slots={}, cmd_id="mi-" + uuid.uuid4().hex[:12],
                    turn_id="vt-" + uuid.uuid4().hex[:12],
                    channel=self._turn_channel, asr_text=text)}
        if is_task_control_intent(entry.name) and not release_hold:
            try:
                tc = to_task_control_command(
                    entry.name, state=_state,
                    cmd_id=uuid.uuid4().hex,
                    source=task_source(self._turn_channel))
            except TaskControlError as exc:
                spoken = _TASK_CONTROL_CN.get(
                    str(exc), "现在没有可以操作的任务")
                return TurnDecision(kind="reply", intent_id=entry.id,
                                    intent_name=entry.name, route=entry.route,
                                    auth=eff_auth, level=eff_auth, layer=layer,
                                    reply_text=spoken, tts_text=spoken,
                                    llm_used=llm_used,
                                    prompt_assembled=llm_used)
            if tc is not None:
                extra = {**(extra or {}), "task_command": tc}
        # 18 A class -> 11 S9.3.2A.3 MotionIntent for cmd/motion/intent
        # (2026-08-21). The routing already pointed at this key, but P4 sent
        # its own p4_intent_v1 envelope -- no data wrapper, no intent/slots/
        # auth_level/turn_id -- and P2 had no subscriber at all.
        if is_motion_intent(entry.name):
            try:
                mi = to_motion_intent(
                    entry.name, slots=dict(extra or {}),
                    cmd_id="mi-" + uuid.uuid4().hex[:12],
                    turn_id="vt-" + uuid.uuid4().hex[:12],
                    channel=self._turn_channel, asr_text=text)
            except MotionIntentError as exc:
                # MI-2: a missing slot NEVER goes on the wire -- ask again.
                spoken = _MOTION_REASON_CN.get(
                    str(exc), "这个动作没说清楚, 再说一遍")
                return TurnDecision(kind="reply", intent_id=entry.id,
                                    intent_name=entry.name, route=entry.route,
                                    auth=eff_auth, level=eff_auth, layer=layer,
                                    reply_text=spoken, tts_text=spoken,
                                    llm_used=llm_used,
                                    prompt_assembled=llm_used)
            if mi is not None:
                extra = {**(extra or {}), "motion_intent": mi}
        # 18 C class -> 11 S7.3 ModeCommand for cmd/mode (2026-08-21). Before
        # this the whole C class went to cmd/task and P3 skipped it: eight voice
        # commands ("enter broadcast", "exit alarm", ...) that reached nobody.
        if is_mode_intent(entry.name):
            try:
                mode_cmd = to_mode_command(
                    entry.name, slots=dict(extra or {}),
                    cmd_id=uuid.uuid4().hex,
                    source=mode_source(self._turn_channel))
            except ModeRequestError as exc:
                spoken = _F_REASON_CN.get(str(exc), "这个模式指令没听清, 再说一遍")
                return TurnDecision(kind="reply", intent_id=entry.id,
                                    intent_name=entry.name, route=entry.route,
                                    auth=eff_auth, level=eff_auth, layer=layer,
                                    reply_text=spoken, tts_text=spoken,
                                    llm_used=llm_used,
                                    prompt_assembled=llm_used)
            if mode_cmd is not None:
                extra = {**(extra or {}), "mode_command": mode_cmd}
        # 11 S12A.1: the F class. F01-F10 build a TeachCommand for cmd/teach,
        # F11-F15 a GeoCommand for cmd/geo. A build failure is spoken back
        # ("there is no recording in progress", "no route named X") rather than
        # dispatched -- sending a command that cannot succeed costs the operator
        # a round trip and tells them nothing they can act on.
        if is_teach_intent(entry.name) or is_geo_intent(entry.name):
            extra, failure = self._build_f_class(entry, extra, now_mono_ms)
            if failure is not None:
                return TurnDecision(kind="reply", intent_id=entry.id,
                                    intent_name=entry.name, route=entry.route,
                                    auth=eff_auth, level=eff_auth, layer=layer,
                                    reply_text=failure, tts_text=failure,
                                    llm_used=llm_used,
                                    prompt_assembled=llm_used)
        # Build the envelope (EV-1..7) and dispatch.
        env = self._build_envelope(entry, eff_auth, slots=dict(extra))
        result = dispatch(entry.id, text, extra or None)
        if release_hold:
            # B06 is a B-class id (cmd/task); its hold release is a motion frame.
            result = DispatchResult(intent_id=result.intent_id,
                                    key=CMD_MOTION_INTENT, payload=result.payload)
        return TurnDecision(
            kind="dispatch", intent_id=entry.id, intent_name=entry.name,
            route=entry.route, auth=eff_auth, level=eff_auth, layer=layer,
            dispatch_result=result, envelope=env,
            tts_text=self._dispatch_ack(entry, extra),
            llm_used=llm_used, prompt_assembled=llm_used)

    def _run_tier2(self, text: str, session: OrchestratorSession,
                   now_mono_ms: int) -> TurnDecision:
        """Layer-6 unknown: tier-2 LLM classify. Marks LLM used + prompt
        assembled (both happen inside tier2_fn)."""
        result = self._tier2(text, session, now_mono_ms)
        if result is None or result.name == "unknown":
            # No mission / gate denied / LLM failed / the model declined: no
            # intent to route, so decline (caller may TTS).
            return TurnDecision(kind="denied", layer="unknown",
                                llm_used=True, prompt_assembled=True)
        classified = result.name
        if classified in _CHITCHAT_NAMES:
            # The LLM classified it as out_of_scope / chitchat.
            reply = self._chitchat.respond(classified, session.chitchat)
            return TurnDecision(
                kind="reply", intent_name=classified, layer="unknown",
                reply_text=reply, tts_text=reply, route="llm",
                llm_used=True, prompt_assembled=True)
        # A real intent the LLM picked: route it like a matched intent, passing
        # the LLM-extracted slots (the free-text ones the fastpath cannot fill).
        entry = self._registry.by_name(classified)
        decision = self._route_entry(entry, text, session, now_mono_ms,
                                     layer="unknown", llm_slots=result.slots)
        # Preserve the fact the LLM was used to reach this classification.
        decision.llm_used = True
        decision.prompt_assembled = True
        return decision

    # -- confirm resolution ---------------------------------------------

    def _resolve_pending_confirm(self, text: str,
                                 session: OrchestratorSession,
                                 now_mono_ms: int) -> TurnDecision:
        """A confirm is open; interpret this turn as the I01/I02 response
        (16 S5.2 layer 3 session-state)."""
        pc = session.pending_confirm
        # Timeout first: a stale confirm must not accept a late 'yes'.
        pc.slot.tick(now_mono_ms=now_mono_ms)
        if pc.slot.state == L2ConfirmState.TIMED_OUT:
            session.pending_confirm = None
            return TurnDecision(kind="confirm_timeout", intent_id=pc.entry.id,
                                intent_name=pc.entry.name,
                                tts_text=self._timeout_prompt(pc.entry))
        # Classify the response; I01 confirm / I02 deny are session-state
        # intents supplied here (they are excluded from the layer-2 index).
        response = self._classify_confirm_response(text, pc.entry)
        if response == "I02":       # deny
            session.pending_confirm = None
            return TurnDecision(kind="confirm_denied", intent_id=pc.entry.id,
                                intent_name=pc.entry.name,
                                tts_text=self._denied_prompt(pc.entry))
        if response != "I01":       # neither yes nor no -> keep waiting
            return TurnDecision(kind="await_confirm", intent_id=pc.entry.id,
                                intent_name=pc.entry.name,
                                tts_text=self._confirm_prompt(pc.entry))
        # Confirmed: dispatch the held intent NOW, through the SAME path an
        # unconfirmed dispatch takes.
        #
        # *** It used to call dispatch(entry.id, held_text) directly, which
        # skipped slot fill, the F-class GeoCommand/TeachCommand builder and the
        # TaskCommand builder in one step. Every L2 intent is destructive by
        # definition -- F11 delete_route, F13 delete_fence, F15 set_active_fence
        # are all L2 -- so the intents that were BUILT most carefully were
        # exactly the ones dispatched as a bare p4_intent_v1 envelope carrying
        # no target and no action. P3 could not have acted on any of them.
        #
        # Reusing _dispatch_entry cannot re-open a confirm: the auth gate lives
        # in the CALLER (_dispatch_or_confirm), not in _dispatch_entry.
        pc.slot.confirm()
        entry, held_text = pc.entry, pc.text
        session.pending_confirm = None
        return self._dispatch_entry(entry, held_text, session, now_mono_ms,
                                    "session_state", "L2",
                                    llm_slots=pc.llm_slots)

    def _classify_confirm_response(self, text: str,
                                   pending: Optional[IntentEntry] = None
                                   ) -> Optional[str]:
        """Map a confirm-window utterance to I01 (confirm) / I02 (deny) /
        None. Uses the registry keywords for I01/I02 (which are excluded
        from the layer-2 index, so they are matched here, at layer 3).

        The pending command's own keywords are removed first (99 U111): with
        "取消" a deny word (18 I02), repeating "取消任务" to "确认取消任务吗"
        would otherwise read as "no". Repeating the command is neither."""
        for kw in sorted((pending.keywords if pending else ()), key=len,
                         reverse=True):
            if kw:
                text = text.replace(kw, "")
        i01 = self._registry.by_id("I01")
        i02 = self._registry.by_id("I02")
        # Deny checked with the same longest-first spirit; here a simple
        # substring is enough because the window is a yes/no context.
        for kw in i02.keywords:
            if kw and kw in text:
                return "I02"
        for kw in i01.keywords:
            if kw and kw in text:
                return "I01"
        return None

    # -- replies ---------------------------------------------------------

    def _reply_chitchat(self, entry: IntentEntry,
                        session: OrchestratorSession,
                        layer: str, text: str = "") -> TurnDecision:
        # A greeting answers in the time of day the operator named ("下午好"),
        # never one picked from the clock (field test 4.9, 99 U101).
        tod = next((t for w, t in _SAID_TIME if w in text), None) if entry.name == "greeting" else None
        reply = self._chitchat.respond(entry.name, session.chitchat, time_of_day=tod)
        return TurnDecision(
            kind="reply", intent_id=entry.id, intent_name=entry.name,
            route=entry.route, auth=entry.auth, layer=layer,
            reply_text=reply, tts_text=reply)

    # -- envelope + helpers ---------------------------------------------

    def _build_task_command(self, entry: IntentEntry,
                            extra: Optional[Dict[str, Any]], text: str):
        """(slots with the TaskCommand attached, None) or (slots, spoken reason).

        The route / waypoint name is resolved to a geo_id HERE, because this
        is where the GeoManifest and the operator both are. A name that is NOT
        in the catalogue, or that names two objects, is refused at
        the turn -- the old behaviour queued the task with route_geo_id NULL and
        it failed later, further from the person who could have fixed it.

        *** An absent catalogue REFUSES the turn (changed 2026-10-25).

        Until then an absent catalogue "degraded": no route_id, the spoken name
        rode in task.params.slots, on the belief that P3 would match the name.
        It does not -- geo_refs only computes the delete-impact set (11
        S7.9.4), and P3's dispatch pushes a route only from route_geo_id
        (runtime/main_wiring _push_route_on_dispatch). So the degraded task was
        stored, reported "running", and moved nothing. Measured 2026-10-25 on
        .1.26: with the B01 tier-2 leg the LLM, which has no catalogue in its
        prompt, filled route="候选" and waypoint="到这边=w"; six such tasks
        were queued (one "running") before anyone noticed.

        16 S8.1 check (1) is "the object exists, before any side effect". With
        no catalogue that cannot be checked, so the turn is refused with the
        same reply the F11-F15 geo commands already give
        (CATALOGUE_UNAVAILABLE). Capability lost: none -- a name-only task
        never executed. Until state/geo/manifest is subscribed in production
        (state_fn has no production call site) voice goto/patrol answer
        "catalogue not loaded"; that is the honest description of what works.

        Which slot is resolved, per intent, is task_request.GEO_REF_SLOT.
        A missing slot is not decided here: to_task_command raises the
        clarify reason (GOTO_NEEDS_WAYPOINT / PATROL_NEEDS_ROUTE).
        """
        merged = dict(extra or {})
        if entry.name == "patrol_repeat" and "not_before_ts" not in merged:
            # 99 U131: "再巡一圈" while a patrol runs is answered, not queued
            # behind it -- the user's ruling is to say so and stop. A timed
            # repeat is not refused: by its time the patrol is long over.
            state = self._state_fn() if self._state_fn else {}
            current = current_task_from_state(state)
            if current is not None and current.get("type") == "patrol":
                return merged, _PTP_CN["patrol_running"]
        if entry.name == "return_home":
            # 99 U128: a spoken "返航" goes to 返航终点站 or nowhere -- never
            # silently to a dock (that fallback is the link-loss return's
            # alone). Checked here, before anything is sent, so the operator
            # hears why at once instead of a task that fails later.
            state = self._state_fn() if self._state_fn else {}
            manifest = manifest_from_state(state.get("state/geo/manifest"))
            if manifest is None:
                return merged, _F_REASON_CN[CATALOGUE_UNAVAILABLE]
            if not any(it.get("type") == "waypoint"
                       and it.get("name") == HOME_WAYPOINT_NAME
                       and it.get("state", "active") == "active"
                       for it in manifest["items"]):
                return merged, _PTP_CN["no_home"]
        # 11 S7.2 task.not_before_ts (99 U110), set by a timed PTP goto. Taken
        # out of the slots: it is a task field, not something the operator's
        # slots should carry into params.
        not_before_ts = merged.pop("not_before_ts", None)
        geo_ids: Dict[str, Optional[str]] = {"route": None, "waypoint": None}
        ref = GEO_REF_SLOT.get(entry.name)
        if ref is not None:
            slot, gtype = ref
            spoken = merged.get(slot)
            if isinstance(spoken, str) and spoken.strip():
                state = self._state_fn() if self._state_fn else {}
                manifest = manifest_from_state(state.get("state/geo/manifest"))
                if manifest is None:
                    return merged, _F_REASON_CN[CATALOGUE_UNAVAILABLE]
                try:
                    geo_ids[gtype] = _resolve_task_geo(
                        list(manifest["items"]), gtype, spoken.strip())
                except GeoRequestError as exc:
                    if self._turn_ptp_ack is not None:
                        # A rule-parsed goto / patrol names what it looked for.
                        return merged, _PTP_CN[
                            "not_found" if gtype == "waypoint"
                            else "not_found_route"].format(name=spoken.strip())
                    return merged, _F_REASON_CN.get(
                        str(exc), _NOT_FOUND_CN[gtype])
        try:
            cmd = to_task_command(
                entry.name, self._registry, slots=merged,
                source=task_source(self._turn_channel),
                cmd_id=uuid.uuid4().hex, text=text, route_id=geo_ids["route"],
                waypoint_id=geo_ids["waypoint"], not_before_ts=not_before_ts)
        except TaskRequestError as exc:
            return merged, _F_REASON_CN.get(str(exc), "这条任务现在没法下发")
        if cmd is not None:
            merged["task_command"] = cmd
        return merged, None

    def _build_f_class(self, entry: IntentEntry,
                       extra: Optional[Dict[str, Any]], now_mono_ms: int):
        """(slots with the command attached, None) or (slots, spoken reason).

        Returned as a pair rather than stashed on self: an orchestrator that
        carried per-turn state on the instance would leak one turn into the
        next, and these turns are exactly the ones where that matters (a failed
        save followed by a successful one).

        The reason is Chinese because it is spoken straight back to the
        operator; the codes behind it (E_TEACH_STATE, E_NOT_FOUND) would only
        reach them as "internal error".
        """
        state = self._state_fn() if self._state_fn else {}
        merged = dict(extra or {})
        cmd_id = uuid.uuid4().hex
        if entry.name in _REC_STARTS:
            # 99 U133: a start must carry a full name, of the right type, not
            # yet used by a live object of that type. Answered here so the
            # keyword leg ("开始录制路径", no slot) is refused the same way.
            name = merged.get("name")
            want = "fence" if entry.name == "record_fence_start" else "route"
            if not isinstance(name, str) or not name.strip():
                return merged, _REC_CN["noname"]
            name = name.strip()
            if not name_length_ok(name):
                return merged, _REC_CN["name_length"]
            manifest = manifest_from_state(state.get("state/geo/manifest"))
            if manifest is not None and any(
                    it.get("type") == want and it.get("state", "active") != "deleted"
                    and canonical_name(str(it.get("name") or "")) == canonical_name(name)
                    for it in manifest["items"]):
                return merged, _REC_CN["taken"].format(
                    name=name, what="围栏" if want == "fence" else "路径")
        try:
            if is_teach_intent(entry.name):
                # 18 PS-6: a keypoint capture carries where the robot was at
                # the first word; the builder refuses it without one.
                snap = (self._pose_snap_fn(self._turn_t_first, now_mono_ms)
                        if self._pose_snap_fn is not None
                        and needs_pose_snap(entry.name) else None)
                cmd = to_teach_command(
                    entry.name, slots=merged, cmd_id=cmd_id,
                    issuer_channel=teach_issuer_channel(
                        self._turn_channel, self._turn_modality),
                    session_id=session_id_from_state(state.get("state/teach")),
                    pose_snap=snap)
                if cmd is not None:
                    merged["teach_command"] = cmd
            else:
                cmd = to_geo_command(
                    entry.name, slots=merged, cmd_id=cmd_id,
                    manifest=manifest_from_state(
                        state.get("state/geo/manifest")),
                    origin=origin(self._turn_channel))
                if cmd is not None:
                    merged["geo_command"] = cmd
        except TeachRequestError as exc:
            return dict(extra or {}), _F_REASON_CN.get(
                str(exc), "这个录制指令现在没法执行")
        except GeoRequestError as exc:
            target = merged.get("waypoint") or merged.get("old")
            if (str(exc) not in _F_REASON_CN and self._turn_ptp_ack is not None
                    and isinstance(target, str)):
                # A rule-parsed delete / rename names what it looked for.
                return dict(extra or {}), _PTP_CN["not_found"].format(
                    name=target)
            return dict(extra or {}), _F_REASON_CN.get(
                str(exc), "没找到你说的那个对象")
        return merged, None

    def _build_envelope(self, entry: IntentEntry, level: str,
                        slots: Dict[str, Any]) -> IntentEnvelope:
        """Wrap a dispatched intent (EV-1..7 checked in __post_init__)."""
        return IntentEnvelope(
            id=entry.id,
            intent=entry.name,
            route=entry.route,
            auth=entry.auth,
            level=level,
            slots=slots,
            cmd_id=uuid.uuid4().hex,
            latency_class=_LATENCY_BY_ROUTE[entry.route],
        )

    def _effective_auth(self, auth: str, session: OrchestratorSession) -> str:
        """CL-2: estop_path==down upgrades L1b -> L2 (one extra gate)."""
        if auth == "L1b" and session.estop_path == "down":
            return "L2"
        return auth

    # Prompt wording (hot phrasing; ASCII punct, Chinese words). A Chinese
    # label per intent gives the operator something meaningful to confirm;
    # falls back to a generic prompt. NEVER speaks the English intent name
    # (the 2026-08-11 ORIN test heard '确认要执行cancel_task吗').
    def _confirm_prompt(self, entry: IntentEntry) -> str:
        label = _CN_LABELS.get(entry.id)
        if label:
            return "确认" + label + "吗,请说是或否"
        return "确认执行这个操作吗,请说是或否"

    def _approval_prompt(self, entry: IntentEntry) -> str:
        return "该操作需要云端审批,已上报等待批准"

    def _denied_prompt(self, entry: IntentEntry) -> str:
        return "好的,已取消"

    def _timeout_prompt(self, entry: IntentEntry) -> str:
        return "没有收到确认,已取消"

    def _dispatch_ack(self, entry: IntentEntry,
                      extra: Optional[Dict[str, Any]] = None) -> str:
        """Brief spoken acknowledgment for a dispatched action, so a motion
        command is not silent. A Chinese label when known, else 'received'.
        (Full L1a/L1b restate with slot values is GWY-P4-35 wiring; this is
        the minimal feedback the 2026-08-11 ORIN test showed was missing.)
        A15 / A16 name the direction they were asked for (99 U104 / U106);
        the resulting value is P2's (ack.applied), never predicted here."""
        if self._turn_ptp_ack is not None:
            # 99 U110: a rule-parsed PTP command restates what it will do.
            return self._turn_ptp_ack
        if entry.id == "A15":
            level = ((extra or {}).get("mode_command") or {}).get("turn_speed")
            label = _TURN_SPEED_LABELS.get(level)
            if label:
                return "好的," + label
        if entry.id == "A16":
            # 99 U106: the direction asked for; the value is P2's ack.applied.
            level = ((extra or {}).get("mode_command") or {}).get("move_speed")
            label = _MOVE_SPEED_LABELS.get(level)
            if label:
                return "好的," + label
        label = _CN_LABELS.get(entry.id)
        return ("好的," + label) if label else "收到"
