"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ack_speech.py
Brief: the executor's answer to a voice / text command, spoken (11 CMD-30 / AP-1)

Description:
A turn says what it is about to do ("好的,暂停任务") and sends one command
frame; the process that executes it answers on the matching ack key. Until
99 U109 nothing in P4 listened to any of those answers, so the operator heard
"收到" also when the executor REFUSED the command. The 2026-11-04 live tests
showed it twice: a capture P3 refused indoors, and "继续" refused by P3
(E_TASK_STATE) while the robot said "收到". 11 S7.7 AP-1 / CMD-30 is the rule:
an operator-facing command is restated from what the executor did.

What is said, per answer key (the ack key = command key + "/ack"):
  cmd/teach/ack          mark_once (F06 / F10): stored -> "已记录 X" (with
                         "没有记下朝向" on warn no_heading), refused -> why.
                         start (F01 / F07, 99 U133): refused -> why (the
                         turn already said "开始录制 X"). finish of a named
                         recording: saved -> "已保存 X, 共 N 个点, 长 M 米",
                         not saved -> "录制已退出, X 没有保存: why". The
                         other session actions answer through state/teach.
                         discard (F04, 99 U137): "已取消录制 X, 录到的 N
                         个点已丢弃" -- there is no confirmation turn now.
  cmd/task/ack           submit / pause / resume / cancel: refused -> why.
  cmd/geo/ack            delete / rename / set_state: refused -> why.
  cmd/motion/intent/ack  A05-A17, A04 hold, B06 release: refused -> why.
An accepted command other than a capture says nothing more: the turn already
said what it does, and saying it twice is noise on a speaker that also does
broadcasts. (99 U109 for the capture, U111 for the other three keys.)

What happens AFTER an accepted submit (16 S8.3C, 99 U140): the task can still
fail when P3 runs it (no position, target or route gone, P1 never confirmed
the route, navigation failed on the way). The operator had heard "好的,去X"
and then nothing. An accepted submit's task_id is remembered with the task's
spoken name; on_task_event() reads the 11 S6.2 task events (event/*/task) and
says "X没有完成: why" for a failed / rejected one, forgets it on completed /
cancelled, and says nothing about tasks it did not submit. The why comes from
detail.error.cause (closed set) or detail.error.fail_reason -- never from the
free-text reason. A timed task is covered the same way; a P4 restart empties
the table and a task submitted before it fails silently (said, not hidden).

How it is matched to the turn: the wiring hands every turn's published
frames to note_turn(). A command frame is remembered under its cmd_id with
the TTS source of that turn's own reply -- only when the turn spoke at all,
so a typed command sent with require_tts_reply=false (11 S8.7.5) stays
silent here too. on_ack() turns the matching ack into one sentence on that
source and forgets the cmd_id; a duplicate ack (replay) says nothing.

Bounded: at most _PENDING_MAX outstanding commands, oldest dropped. An ack
that never comes costs one slot, never memory, and needs no timer.
"""
from __future__ import annotations  # postponed annotations

import json  # frames in, frames out
import time  # mono_ms on the outgoing speak, like every p4_speak_v1
from collections import OrderedDict  # insertion order = age, for eviction
from typing import Any, Dict, Iterable, Optional, Tuple  # plain values

from xbrain.common.errors import (  # the 11 S13 closed set, never literals
    E_BUSY,  # G-8 / hold refusals
    E_LOW_BATTERY,  # S12A.3 arming check 5
    E_TEACH_BUSY,  # S12A.3 arming check 1
    E_TEACH_GEOMETRY,  # S12A.7 blocking issue
    E_TEACH_STATE,  # session state / config
    E_CAPABILITY,  # G-7
    E_CHANNEL_DENIED,  # S7.9.5 matrix
    E_DEGRADED,  # S3.2.1
    E_GEO_CONFLICT,  # base_rev
    E_GEO_INVALID,  # 99 U146 dock off every route
    E_LOCKED,  # G-9
    E_NAME_CONFLICT,  # name taken
    E_NO_HEADING,  # G-6
    E_NOT_FOUND,  # gone
    E_SCHEMA,  # G-2 / G-3
    E_TASK_STATE,  # S7.2 state
    E_TEACH_QUALITY,  # capture quality
    E_UNHEALTHY,  # G-4 / G-5
)
from xbrain.p4_agent.runtime.intent_dispatch import (  # the command keys
    CMD_AUDIO_SPEAK,
    CMD_GEO,
    CMD_MOTION_INTENT,
    CMD_TASK,
    CMD_TEACH,
)
from xbrain.p4_agent.runtime.voice_turn import encode as encode_turn  # 11 S8.7.6
from xbrain.p4_agent.runtime.voice_turn import new_turn_id  # result record id
from xbrain.p4_agent.runtime.voice_turn import SESSION_OF_CHANNEL  # channel -> session (11 S8.7.6)

#: 99 U162: the recording actions whose RESULT also goes out as a second
#: state/voice_turn record -- the turn's own record only says "正在保存 X"
#: (it is built before P3 answers), and a cloud client reads voice_turn, not
#: the speaker. Published even for a silent (require_tts_reply=false) turn:
#: then voice_turn is the ONLY place the result goes.
#: 99 U165: + start. The start turn says "好的,开始录制X" before P3 has run its
#: arming checks; measured 2026-11-11 indoors, P3 then refused (no rtk_fixed)
#: and nothing but cmd/teach/ack carried it, so a cloud client opened its
#: keyboard on a recording that never started.
#: 2026-11-12: + mark_once (F06 / F10). Measured 2026-11-11 22:29: "保存当前位置
#: 为2号充电站" read "好的,记录这个点" / accepted in voice_turn and in commx,
#: P3 refused it (E_GEO_INVALID dock_off_route), the TTS device was down, and
#: the refusal reached nobody. The record says "已记录X" (accepted) or why not
#: (rejected + reject_code); the new geo_id is on manifest, not in the record
#: (11 S8.7.6 has no field for it).
_RESULT_TO_TURN = frozenset({"start", "finish", "discard", "mark_once"})  # started / saved / dropped / stored
#: The layer value of such a record (the user-ruled extra field of 11 S8.7.6):
#: tells "the result of an earlier turn" apart from a turn of its own.
TEACH_RESULT_LAYER = "teach_result"  # voice_turn.layer of the result record
#: 99 U169 (user 2026-11-12): every OTHER command result P4 words -- an
#: executor refusal on cmd/task / cmd/geo / cmd/motion/intent / a teach action,
#: a clear_queue count, an accepted task that later failed -- also goes out as a
#: second voice_turn record. Until then those were spoken only (update_cloud.md
#: S8.7): the cloud and commx, which read voice_turn, saw the turn's "好的, ..."
#: and never the refusal; a silent text turn (no TTS) got no answer at all.
COMMAND_RESULT_LAYER = "command_result"  # voice_turn.layer of those records

#: The answer keys this speaker subscribes, one per command key (11 S2.2.4).
ACK_KEYS = tuple(k + "/ack" for k in (CMD_TEACH, CMD_TASK, CMD_GEO,
                                      CMD_MOTION_INTENT))  # wiring subscribes these

#: 11 S6.2 task events, every severity (info carries completed, warn failed).
TASK_EVENT_KEY = "event/*/task"  # wiring subscribes this (16 S8.3C)

#: 11 S3.3.4 rtk events: event/alarm/rtk (loss) and event/info/rtk (recovery),
#: published by P3 (99 U142). Spoken on the robot whoever started the task:
#: RL-3's voice is a local alarm, not an answer to a turn.
RTK_EVENT_KEY = "event/*/rtk"  # wiring subscribes this

#: The RL-3 wording ("定位失效, 已停车", 11 S3.3.4; ASCII comma, charset rule)
#: and the recovery line. "请说继续" only when a task waits: RL-7, nothing
#: restarts by itself, and telling the operator to resume nothing misleads.
_RTK_LOST = "定位失效, 已停车"
_RTK_LOST_TASK = "定位失效, 已停车, 任务已暂停"
_RTK_BACK = "定位已恢复"
_RTK_BACK_TASK = "定位已恢复, 要接着执行任务请说继续"
#: The speaker source of a local alarm voice (14 S4.1 registry, P2 SPEAK_SOURCES).
_RTK_SOURCE = "tts_local"

#: 99 U146 low-battery reports: event/alarm/charging from P3 every 2 s while a
#: low-battery episode lasts. Only the one marked first is spoken (the ruling:
#: a SINGLE voice alarm; the cloud gets them all).
CHARGING_EVENT_KEY = "event/*/charging"  # wiring subscribes this
#: 99 U186 fence events from P1: entering / leaving a 报警区域 or 限速区域
#: (zone_enter / zone_exit, once each) and boundary contact on the activity
#: area / a forbid zone (contact_start, then contact_hold every few seconds
#: while it lasts, contact_end silent). P3's objects_unreachable is NOT
#: spoken here: a voice-triggered one is already said with its ack.
FENCE_EVENT_KEY = "event/*/fence"  # wiring subscribes this
#: 99 U186: the cloud keyboard refused outside a recording (P1, once per episode)
TELEOP_EVENT_KEY = "event/*/teleop"  # wiring subscribes this
#: 99 U163: the recording reports P3 raises with no command behind them (the
#: time-left warnings, the auto-finish at the time limit).
TEACH_EVENT_KEY = "event/*/teach"  # wiring subscribes this
#: S12A.4 issuer.channel -> the 11 S8.7.6 voice_turn channel the report is
#: recorded under: the session that started the recording sees its end.
_TEACH_CHANNEL = (("local", "mic_local"), ("cloud", "cloud"), ("hmi", "hmi"),
                  ("wecom", "wecom"))
_KIND_CN = {"route": "路径", "fence": "围栏"}
_LB_SPEECH = {  # by detail.type
    "low_battery": "电量剩余{soc}%, 正在返回{target}",  # going to a dock / home
    "low_battery_unreachable": "电量剩余{soc}%, 正在返回{target}, 剩余电量可能不够走到",  # range may not cover it
    "low_battery_no_target": "电量剩余{soc}%, 没有定义充电桩和返航终点站, 无法自动返回",  # nothing defined
    "low_battery_stop": "电量剩余{soc}%, 已停车趴下",  # the last 5 %
}

#: Outstanding commands remembered at once (one per turn).
_PENDING_MAX = 32  # slots, not a time

#: cmd/teach/ack wording; {name} is the name the operator gave. ASCII
#: punctuation (charset rule), Chinese words.
_TEACH_REFUSED = {
    E_NAME_CONFLICT: "{name}这个名字已经有了, 请换个名字",  # 11 S12A.8 conflict row
    E_TEACH_QUALITY: "{name}没有记录, 定位精度不够, 需要RTK固定解",  # rtk_fixed rule
    E_SCHEMA: "{name}没有记录, 指令格式不对",  # a builder bug, said plainly
    # 99 U146: a 充电桩 / 充电站 point must be on a route (within 5 m)
    E_GEO_INVALID: "{name}没有记录, 充电桩和充电站必须在某条路径5米以内",
}
_SNAP_AGE = "{name}没有记录, 位置快照已过期, 请再说一遍"  # 18 PS-6 snap_age
#: 99 U169 (user 2026-11-12, "语音回复失败的语言你来组织"): a 充电站 / 充电桩
#: capture off every route (P3 E_GEO_INVALID dock_off_route, 99 U146) says how
#: far the nearest route is, the limit, and what to do. limit / distance come
#: from P3's detail (limit_m = its route_membership_m), never a typed-in 5.
_DOCK_OFF_ROUTE = "{name}没有保存: 这里离最近的路径有{d}米, 充电站和充电桩必须{rule}. 请走到路径旁边再说一次"
_DOCK_NO_ROUTE = "{name}没有保存: 还没有任何路径, 充电站和充电桩必须建在路径旁边. 请先录制路径"
_TEACH_OTHER = "{name}没有记录成功"  # any other code
_STORED = "已记录{name}"  # accepted
_STORED_NO_HEADING = "已记录{name}, 没有记下朝向"  # accepted, warn no_heading

#: 99 U133 recording start refusals (cmd/teach/ack of action start), by code;
#: E_TEACH_QUALITY and E_UNHEALTHY are told apart by detail.reason below.
_REC_START_REFUSED = {
    E_NAME_CONFLICT: "{name}这个名字已经有了, 请换个名字再录制",  # taken meanwhile
    E_TEACH_BUSY: "已经在录制了, 不能开始新的录制",  # check 1
    E_BUSY: "有任务正在执行, 现在不能开始录制",  # check 2
    E_LOCKED: "现在不能开始录制: 硬件急停按下了",  # check 4 (hes)
    E_LOW_BATTERY: "现在不能开始录制: 电量不足",  # check 5
}
_REC_START_BY_REASON = {
    "state_unavailable": "现在不能开始录制: 拿不到健康, 电量或遥控状态",  # caches missing
    "no_nonvoice_estop": "现在不能开始录制: 没有可用的非语音急停通道",  # check 7
    "recording_config_missing": "现在不能开始录制: 录制参数没有配置",  # U132 keys
    # 99 U176: a fence's type comes from its name, one activity area only.
    # ~~activity_area_missing~~ (U184: fences are recorded in any order)
    "fence_type_missing": "围栏名字里要说明类型: 活动区域, 报警区域, 限速区域或禁止区域",  # no keyword
    "activity_area_exists": "已经有活动区域围栏{existing}了, 只能有一个活动区域, 没有开始录制",  # FS-5A
}
#: 99 U176 / U185: the speed default unconfigured -- only 限速区域 is refused
#: (production always has it: p3 refuses to start without fence.speed_limit_*).
_REC_START_NO_SPEED_LIMIT = "现在不能录制限速区域围栏: 限速值没有配置"  # the other kinds still record
_REC_START_QUALITY = "现在不能开始录制: 定位不是RTK固定解"  # check 3
_REC_START_UNHEALTHY = "现在不能开始录制: 设备状态不允许移动"  # check 4
_REC_START_OTHER = "{name}没有开始录制"  # any other code
#: finish of a named recording (99 U133).
_REC_SAVED_ROUTE = "已保存{name}, 共{count}个点, 长{length}米"
_REC_SAVED_FENCE = "已保存{name}, 共{count}个顶点. 围栏已保存但未启用, 需要时请说启用{name}"
_REC_NOT_SAVED = "录制已退出, {name}没有保存: {why}"
_REC_ISSUE = {  # S12A.7 blocking issue codes, in words
    "too_few_points": "录到的点不足两个",
    "too_few_vertices": "围栏顶点不足三个",
    "self_intersect": "围栏边线自相交",
    "area_too_small": "围栏面积太小",
    # 99 U176
    "fence_type_missing": "围栏名字里没有说明类型",  # the save name has no keyword
    # ~~activity_area_missing / outside_activity_area / activity_area_excludes /
    # activity_area_inactive~~ -- 99 U184: saved and told, see _LAYOUT_NOTE
    # 99 U184: {fences} filled by _issue_text
    "forbid_overlap": "和{fences}重叠了, 禁止区域不能和报警区域, 限速区域或其它禁止区域重叠",
}
#: 99 U184 (user 2026-11-13): what a fence's place leaves out of reach, said
#: after the result (never a refusal). {area} / {fences} / {objs} / {fence}.
_LAYOUT_NOTE = {
    "outside_activity_area": "注意: 它不在活动区域{area}内, 机器人到不了, 不会生效",  # this fence
    "activity_area_excludes": "注意: {fences}不在这个活动区域里, 机器人到不了",  # older fences
    "inside_forbid": "注意: {objs}在禁止区域{fence}里面, 机器人到不了",  # covered points / routes
    "objects_outside": "注意: {objs}在活动区域外面, 机器人到不了",  # points / routes left out
}
#: 99 U184: where the robot stands when a fence is switched on
_ROBOT_NOTE = {
    "outside_activity_area": "机器人现在在活动区域外, 只能往区域里面开",
    "inside_forbid": "机器人现在在禁止区域{fence}里面, 只能往外开",
}
_NAMES_SAID = 5  # 99 U184: at most this many names are read out, then "等N个"
_REC_FINISH_OTHER = "录制没有正确结束"  # any other refusal
#: 99 U165: a finish whose write failed with the data kept (U163 retained):
#: the session waits in finalizing, so "录制已退出" would be false.
_REC_RETAINED = "{name}没有保存: {why}. 录制数据还在, 请说保存为新的名字, 或者放弃录制"
#: 99 U165: the start result record of an accepted start (not spoken: the turn
#: already said "好的,开始录制X"; this is what a client waits for).
_REC_STARTED = "已开始录制{name}"
_REC_DISCARDED = "已取消录制{name}, 录到的{count}个点已丢弃"  # 99 U137 discard
#: 99 U147 race fix: "取消所有定时任务" -- the count P3 actually cleared, never
#: the one P4's (possibly stale) state/task snapshot showed at the turn.
_SCHED_CLEARED = "已取消{n}个定时任务"
_SCHED_NONE_LEFT = "现在没有定时任务"
#: 99 U174: "取消所有任务" -- clear_queue with no filter, the count P3 cleared.
_QUEUE_CLEARED = "已取消{n}个排队中的任务"
_QUEUE_NONE_LEFT = "现在没有排队中的任务"

#: cmd/task/ack: what the refused action was, in words.
_TASK_VERB = {"pause": "暂停", "resume": "继续", "cancel": "取消",
              "submit": "执行"}  # S7.2 actions
_TASK_REFUSED = {
    E_TASK_STATE: "这个任务现在的状态不能{verb}",  # S7.2: pause needs running etc.
    E_NOT_FOUND: "没找到这个任务, 没有{verb}",  # gone meanwhile
    E_DEGRADED: "定位失效, 现在不接受新任务",  # 11 S3.2.1: single / no_fix only since 99 U142
    E_LOW_BATTERY: "电量过低, 现在不接受新任务",  # 99 U146: stopped at the last 5 %
}
_TASK_OTHER = "任务没有{verb}成功"  # any other code
#: cmd/task/ack refusals told apart by detail.reason before the code.
_TASK_BY_REASON = {
    # 99 U131 B04: P3 found no completed patrol to repeat
    "no_completed_patrol": "还没有巡逻完成过的路径, 没有执行. 请带上路径名说, 例如: 沿1号路径巡逻",  # U147
    # 99 U144: P3 refused a second timed task in the same minute (P4's own
    # pre-check normally answers first; this covers a broadcast not yet seen)
    "schedule_conflict": "这个时间已经有一个定时任务了, 同一时间只能排一个任务, 请换个时间",
}

#: cmd/geo/ack.
_GEO_REFUSED = {
    E_CHANNEL_DENIED: "这个操作不允许从这里发起",  # 11 S7.9.5 matrix
    E_GEO_CONFLICT: "这个对象刚被别处修改过, 请再说一遍",  # base_rev conflict
    E_NAME_CONFLICT: "这个名字已经被占用, 请换个名字",  # rename target taken
    E_NOT_FOUND: "没找到这个对象",  # deleted meanwhile
}
_GEO_OTHER = "这个对象没有改成功"  # any other code
# ~~99 U176 _GEO_BY_REASON: enabling outside / without the activity area
# refused~~ -- 99 U184: enabling is never refused for that; _layout_notes says it

#: cmd/motion/intent/ack: E_BUSY is told apart by detail.reason.
_MOTION_BUSY = {
    "voice_hold": "现在处于原地待命, 请先说继续",  # 99 U111 operator hold
    "mode_hold": "喊话或报警进行中, 现在不能动",  # B / D mode hold
    "teach_session_active": "正在录制路线, 现在不能执行这个动作",  # G-8
}
_MOTION_REFUSED = {
    E_LOCKED: "底盘锁定中, 现在不能动",  # G-9
    E_UNHEALTHY: "设备状态不允许移动",  # G-4 / G-5
    E_NO_HEADING: "现在没有可靠的航向, 不能转向",  # G-6
    E_CAPABILITY: "这个动作机器人做不到",  # G-7
    E_SCHEMA: "这个动作超出了允许的范围",  # G-2 / G-3
}
_MOTION_OTHER = "这个动作没有执行"  # any other code

#: 16 S8.3C task failure speech (99 U140). FS-3 sentence and names.
_TASK_FAILED = "{label}没有完成: {why}"  # with a known cause
_TASK_FAILED_BARE = "{label}没有完成"  # cause unknown
_TASK_LABEL = {"return_home": "返航", "follow": "跟随"}  # no slot needed
#: FS-4: 11 S6.2 error.cause in words.
_FAIL_CAUSE = {
    "no_position": "现在拿不到定位",
    "target_missing": "找不到目标点或路径",
    "plan_refused": "路线生成失败",
    "route_unconfirmed": "运动模块没有确认路线",
}
_FAIL_EN_ROUTE = "途中导航失败"  # error.fail_reason (11 S3.5B)
_FAIL_REJECTED = "任务检查没有通过"  # kind rejected (admission checks)


def _json(data: bytes) -> Any:  # safe decode
    """Decode one frame; None for anything that is not JSON."""
    try:  # parse
        return json.loads(data)  # bytes are accepted by json.loads
    except (ValueError, TypeError):  # garbage in
        return None  # not ours to fix


def _body(frame: Any) -> Any:  # unwrap
    """The bare body of a frame that may be 11 S3.0-enveloped."""
    if isinstance(frame, dict) and isinstance(frame.get("data"), dict):  # S3.0 envelope
        return frame["data"]  # enveloped (cmd/motion/intent is)
    return frame  # bare


class AckSpeaker:  # one per process
    """Remembers this process's command frames and voices their answers."""

    def __init__(self) -> None:  # empty
        # cmd_id -> (command key, what to say it about, speak source). source
        # None = the turn was silent (99 U169): answered as a record, not aloud.
        self._pending: "OrderedDict[str, Tuple[str, Dict[str, Any], Optional[str]]]" = OrderedDict()
        # task_id -> (spoken task name, speak source, turn record) of accepted
        # submits (FS-1); the record is (voice_turn key, record) or None.
        self._tasks: "OrderedDict[str, Tuple[str, Optional[str], Any]]" = OrderedDict()
        # 99 U169: cmd_id -> (voice_turn key, that turn's record) of every
        # command a turn sent, so a worded result can go out as a record.
        self._turns: "OrderedDict[str, Tuple[str, Dict[str, Any]]]" = OrderedDict()
        # 99 U162: cmd_id -> (command, voice_turn key, that turn's record) for
        # a recording finish / discard, kept whether or not the turn spoke.
        self._results: "OrderedDict[str, Tuple[Dict[str, Any], str, Dict[str, Any]]]" = OrderedDict()

    def note_turn(self, pairs: Iterable[Tuple[str, bytes]]) -> None:  # a turn's frames
        """Remember the command frames among one turn's published frames.

        Only a turn that also spoke is remembered: the answer is said on the
        same TTS source the turn's own reply used.
        """
        pairs = list(pairs)  # walked twice
        self._note_results(pairs)  # 99 U162: silent turns included
        source = None  # the turn's reply source, if it spoke
        for key, data in pairs:  # find the reply
            if key == CMD_AUDIO_SPEAK:
                speak = _json(data)  # p4_speak_v1
                if isinstance(speak, dict) and isinstance(speak.get("source"), str):
                    source = speak["source"]  # tts_local / tts_cloud / ...
                    break  # the first reply is the turn's
        # 99 U169: a silent turn (no TTS) is remembered too, with source None:
        # its result is not spoken, but it still goes out as a record.
        turn = next(((k, _json(d)) for k, d in pairs
                     if k.endswith("/state/voice_turn")), None)  # the turn's record
        for key, data in pairs:  # find the commands
            if key not in (CMD_TEACH, CMD_TASK, CMD_GEO, CMD_MOTION_INTENT):
                continue  # not a command this speaker answers for
            cmd = _body(_json(data))  # the command body
            if not isinstance(cmd, dict) or not isinstance(cmd.get("cmd_id"), str):
                continue  # nothing to match an answer to
            if key == CMD_TEACH and cmd.get("action") not in ("mark_once", "start",
                                                                  "finish", "discard"):
                continue  # the other session actions answer through state/teach
            self._pending[cmd["cmd_id"]] = (key, cmd, source)  # newest last
            while len(self._pending) > _PENDING_MAX:  # bounded
                self._pending.popitem(last=False)  # drop the oldest
            if turn is not None and isinstance(turn[1], dict):  # 99 U169
                self._turns[cmd["cmd_id"]] = turn  # the record to answer under
                while len(self._turns) > _PENDING_MAX:  # bounded
                    self._turns.popitem(last=False)  # drop the oldest

    def _note_results(self, pairs: Iterable[Tuple[str, bytes]]) -> None:
        """99 U162 / U165: remember a recording start / finish / discard, or a
        point capture (mark_once, 2026-11-12), with the turn's own voice_turn
        record, so its result can be published as a second record."""
        pairs = list(pairs)  # walked twice
        turn = next(((k, _json(d)) for k, d in pairs
                     if k.endswith("/state/voice_turn")), None)  # the record
        if turn is None or not isinstance(turn[1], dict):
            return  # no record to answer under (no robot id)
        for key, data in pairs:  # the teach commands of the turn
            if key != CMD_TEACH:  # only recording commands have a result
                continue  # task / geo acks are spoken, not recorded
            cmd = _body(_json(data))  # the command body
            if (isinstance(cmd, dict) and isinstance(cmd.get("cmd_id"), str)
                    and cmd.get("action") in _RESULT_TO_TURN):
                self._results[cmd["cmd_id"]] = (cmd, turn[0], turn[1])  # keyed by cmd_id, the ack's only link
                while len(self._results) > _PENDING_MAX:  # bounded
                    self._results.popitem(last=False)  # drop the oldest

    def on_ack_frames(self, data: bytes) -> "list":
        """Every (key, frame) to publish for one ack: the spoken answer (as
        on_ack), and for a recording start / finish / discard or a capture the
        result as a second state/voice_turn record (99 U162 / U165). [] when
        there is nothing new."""
        ack = _body(_json(data))  # S7.7 Ack shape
        if not isinstance(ack, dict) or ack.get("result") == "duplicate":
            return []  # nothing new
        out = []  # in publish order: speech, then the record
        cmd_id = ack.get("cmd_id")  # the only link back to the turn
        text, source = self._answer(ack)  # the worded result, where to say it
        spoken = _speak(text, source) if (text is not None and source) else None
        if spoken is not None:  # nothing spoken when the turn was silent
            out.append(spoken)  # speech first, as before U162
        turn_ref = self._turns.pop(cmd_id, None)  # 99 U169: the turn's record
        hit = self._results.pop(cmd_id, None)  # a recording result?
        if hit is not None:
            cmd, vt_key, turn = hit  # the command, its record key, the record
            text = _result_text(cmd, ack)  # the spoken sentence, or the started one
            if text is not None:  # None: nothing to put in a record
                out.append((vt_key, encode_turn(
                    teach_result_turn(turn, ack, text, spoken is not None))))
        elif text is not None and turn_ref is not None:  # 99 U169: any other worded result
            out.append((turn_ref[0], encode_turn(result_turn(
                turn_ref[1], ack.get("result") == "accepted", ack.get("code"),
                text, spoken is not None, COMMAND_RESULT_LAYER))))
        return out  # [] when the ack is neither spoken nor a result

    def on_ack(self, data: bytes) -> Optional[Tuple[str, bytes]]:
        """The spoken (key, frame) for one ack, or None (no record: on_ack_frames)."""
        ack = _body(_json(data))  # S7.7 Ack shape
        if not isinstance(ack, dict) or ack.get("result") == "duplicate":
            return None  # nothing new to say
        text, source = self._answer(ack)  # the worded result, where to say it
        self._turns.pop(ack.get("cmd_id"), None)  # answered: forget the record link
        if text is None or not source:
            return None  # accepted (the turn said it), or a silent turn
        return _speak(text, source)  # on the turn's source

    def _answer(self, ack: Dict[str, Any]) -> Tuple[Optional[str], Optional[str]]:
        """(sentence or None, speak source or None) for one of our acks; pops it."""
        hit = self._pending.pop(ack.get("cmd_id"), None)  # ours?
        if hit is None:
            return None, None  # another initiator's, or already answered
        key, cmd, source = hit  # what was sent, where to say it
        if key == CMD_TASK and cmd.get("action") == "submit" and ack.get("result") == "accepted":
            self._remember_task(cmd, ack, source)  # FS-1: its outcome may follow
        return _sentence(key, cmd, ack), source  # None when nothing more to say

    def _remember_task(self, cmd: Dict[str, Any], ack: Dict[str, Any],
                       source: Optional[str]) -> None:
        """FS-1: remember an accepted submit under the task_id P3 gave it."""
        detail = ack.get("detail") if isinstance(ack.get("detail"), dict) else {}
        task_id = detail.get("task_id")  # P3 allocates (11 S7.2)
        if not isinstance(task_id, str) or not task_id:
            return  # nothing to match an event to
        turn_ref = self._turns.get(ack.get("cmd_id"))  # 99 U169: its turn's record
        self._tasks[task_id] = (_task_label(cmd.get("task")), source, turn_ref)  # newest last
        while len(self._tasks) > _PENDING_MAX:  # bounded, like the commands
            self._tasks.popitem(last=False)  # drop the oldest

    def on_task_event_frames(self, data: bytes) -> "list":
        """FS-2 + 99 U169: speech (when the submitting turn spoke) and a
        command_result record for an accepted task that later failed."""
        ev = _json(data)  # Event shape (S6.1)
        detail = ev.get("detail") if isinstance(ev, dict) else None
        if not isinstance(detail, dict):
            return []  # not an event
        kind = detail.get("kind")  # accepted / started / completed / failed / ...
        if kind not in ("failed", "rejected", "completed", "cancelled"):
            return []  # not an outcome
        hit = self._tasks.pop(detail.get("task_id"), None)  # ours? (forgotten either way)
        if hit is None or kind in ("completed", "cancelled"):
            return []  # another initiator's, or nothing to say (FS-2)
        label, source, turn_ref = hit  # the name, where to say it, its record
        why = _fail_why(kind, detail.get("error"))  # FS-4
        text = (_TASK_FAILED.format(label=label, why=why) if why
                else _TASK_FAILED_BARE.format(label=label))  # FS-3
        out = [_speak(text, source)] if source else []  # silent turn: no speech
        if turn_ref is not None:  # the cloud reads this one
            err = detail.get("error") if isinstance(detail.get("error"), dict) else {}
            out.append((turn_ref[0], encode_turn(result_turn(
                turn_ref[1], False, err.get("code"), text, bool(source),
                COMMAND_RESULT_LAYER))))
        return out  # speech, then the record

    def on_task_event(self, data: bytes) -> Optional[Tuple[str, bytes]]:
        """FS-2: the spoken (key, frame) for one 11 S6.2 task event, or None
        (the record goes through on_task_event_frames)."""
        ev = _json(data)  # Event shape (S6.1)
        detail = ev.get("detail") if isinstance(ev, dict) else None
        if not isinstance(detail, dict):
            return None  # not an event
        kind = detail.get("kind")  # accepted / started / completed / failed / ...
        if kind not in ("failed", "rejected", "completed", "cancelled"):
            return None  # not an outcome
        hit = self._tasks.pop(detail.get("task_id"), None)  # ours? (forgotten either way)
        if hit is None or kind in ("completed", "cancelled"):
            return None  # another initiator's, or nothing to say (FS-2)
        label, source, _turn_ref = hit  # the name and where to say it
        if not source:
            return None  # a silent turn: the record only (on_task_event_frames)
        why = _fail_why(kind, detail.get("error"))  # FS-4
        text = (_TASK_FAILED.format(label=label, why=why) if why
                else _TASK_FAILED_BARE.format(label=label))  # FS-3
        return _speak(text, source)  # on the submitting turn's source

    def on_rtk_event(self, data: bytes) -> Optional[Tuple[str, bytes]]:
        """RL-3 voice (99 U142): the (key, frame) for one 11 S3.3.4 rtk
        event, or None. Stateless: P3 says in the event whether a task was
        suspended (action_taken on the loss, task_id on the recovery)."""
        ev = _json(data)  # Event shape (S6.1)
        detail = ev.get("detail") if isinstance(ev, dict) else None
        if not isinstance(detail, dict):
            return None  # not an event
        kind = detail.get("type")  # rtk_lost | heading_degraded | heading_recovered
        if kind == "rtk_lost":
            text = (_RTK_LOST_TASK if detail.get("action_taken") == "stop_and_suspend"
                    else _RTK_LOST)  # with or without a suspended task
        elif kind == "heading_recovered":
            text = _RTK_BACK_TASK if detail.get("task_id") else _RTK_BACK  # RL-7 hint
        else:
            return None  # heading_degraded is rtk_driver's, not spoken here
        return _speak(text, _RTK_SOURCE)  # a local alarm voice


def low_battery_speech(data: bytes) -> Optional[Tuple[str, bytes]]:  # P3 marks the first report
    """99 U146: the one-time voice of a low-battery episode, or None.

    Spoken only for a report marked first (the start of the episode, and the
    5 % stop, each once); the repeats every 2 s are for the cloud."""
    ev = _json(data)  # Event shape (S6.1)
    detail = ev.get("detail") if isinstance(ev, dict) else None
    if not isinstance(detail, dict) or detail.get("first") is not True:
        return None  # a repeat, or not an event
    template = _LB_SPEECH.get(detail.get("type"))  # which sentence
    if template is None:  # not a voiced type
        return None  # cleared / boot report / other: not voiced
    soc = detail.get("soc_pct")
    soc_txt = "%d" % int(round(soc)) if isinstance(soc, (int, float)) and not isinstance(soc, bool) else "不足"  # whole percent
    text = template.format(soc=soc_txt, target=detail.get("target_name") or "充电点")  # the sentence
    return _speak(text, _RTK_SOURCE)  # a local alarm voice


#: 99 U186 wording (user 2026-11-13: "所有涉及到的播报话术, 你来组织语言").
_ZONE_SPEECH = {
    ("zone_enter", "warning"): "已进入报警区域{name}",
    ("zone_exit", "warning"): "已离开报警区域{name}",
    ("zone_enter", "speed_limit"): "已进入限速区域{name}, 限速每秒{speed}米",
    ("zone_exit", "speed_limit"): "已离开限速区域{name}, 恢复正常速度",
}
_CONTACT_SPEECH = {  # (recording, role) -> sentence; None recording = autonomous
    (None, "allow"): "已到活动区域边界, 不能继续前进",
    (None, "forbid"): "已到禁止区域{name}边界, 不能进入",
    ("route", "allow"): "路径到了活动区域边界, 已停止前进, 请往回开",
    ("route", "forbid"): "路径碰到禁止区域{name}, 已停止前进, 请往回开",
    ("fence", "allow"): "注意, 现在已在活动区域外",  # once, never repeated
    ("fence", "forbid"): "注意, 现在进入了禁止区域{name}",
}
_KEYBOARD_REFUSED = "现在没有在录制, 键盘不能控制机器人"  # P1 refused the cloud keyboard


def _speed_said(v: Any) -> str:
    """A limit read out: 0.6 -> "0.6", 1.0 -> "1" (no trailing .0)."""
    if not isinstance(v, (int, float)) or isinstance(v, bool):  # absent / wrong type
        return "未知"  # never a made-up number
    return ("%.1f" % v).rstrip("0").rstrip(".")  # one decimal, trailing zeros dropped


def fence_event_speech(data: bytes) -> Optional[Tuple[str, bytes]]:
    """99 U186: the voice of one P1 fence event, or None (not voiced)."""
    ev = _json(data)  # Event shape (S6.1)
    detail = ev.get("detail") if isinstance(ev, dict) else None  # the S9A.9 detail
    if not isinstance(detail, dict):  # not an event body
        return None  # not an event
    kind, role, name = detail.get("kind"), detail.get("role"), detail.get("poly_name") or ""  # what and where
    if kind in ("zone_enter", "zone_exit"):  # once on the way in, once on the way out
        tpl = _ZONE_SPEECH.get((kind, role))  # warning or speed_limit
        text = None if tpl is None else tpl.format(name=name, speed=_speed_said(detail.get("speed_limit_mps")))  # filled
    elif kind in ("contact_start", "contact_hold"):  # held at the line: repeated
        tpl = _CONTACT_SPEECH.get((detail.get("recording"), role))  # autonomous / route / fence
        text = None if tpl is None else tpl.format(name=name)  # filled
    else:  # contact_end, the S9A.9 episode rows, objects_unreachable: not voiced
        text = None
    return None if text is None else _speak(text, _RTK_SOURCE)  # a local voice, like rtk / low battery


def teleop_event_speech(data: bytes) -> Optional[Tuple[str, bytes]]:
    """99 U186: "现在没有在录制, 键盘不能控制机器人" for P1's keyboard refusal."""
    ev = _json(data)  # Event shape (S6.1)
    detail = ev.get("detail") if isinstance(ev, dict) else None  # its detail
    if isinstance(detail, dict) and detail.get("kind") == "keyboard_refused":  # P1's one report per episode
        return _speak(_KEYBOARD_REFUSED, _RTK_SOURCE)  # said once
    return None  # the other teleop events (11 S12A.12) are not voiced here


def _teach_event_text(detail: Dict[str, Any]) -> Optional[str]:
    """99 U163: the sentence for one recording report (11 S12A.12
    teach.time_warning, or teach.expired by detail.result), None for any
    other teach event -- those are not spoken. ASCII punctuation (CLAUDE.md 2.2)."""
    kind = _KIND_CN.get(detail.get("session_kind"), "录制")  # 路径 / 围栏, as the operator calls it
    name = detail.get("name") or ""  # the name bound at start
    mins = int(round((detail.get("duration_s") or 0) / 60.0))  # the configured duration, in minutes
    ev = detail.get("kind")  # time_warning or expired
    if ev == "teach.time_warning":  # the 10 / 1 min warnings (U163)
        left = max(1, int(-(-(detail.get("remaining_s") or 0) // 60)))  # whole minutes, rounded up
        return "%s%s的录制还剩%d分钟, 到时会自动结束并保存" % (kind, name, left)
    if ev != "teach.expired":  # other teach events are not spoken
        return None
    result = detail.get("result")  # saved / save_failed / discarded / awaiting_name
    if result == "saved":  # auto-saved under the start name
        n, length = detail.get("point_count"), detail.get("length_m")
        tail = "共%s个点, 长%s米" % (n, int(round(length))) if isinstance(length, (int, float)) else ""
        if detail.get("session_kind") == "fence":  # a fence is saved as a draft
            return ("录制已到%d分钟上限, 已自动保存为围栏%s, %s. 围栏没有启用, 需要时请说启用围栏%s"
                    % (mins, name, tail, name))
        return "录制已到%d分钟上限, 已自动保存为路径%s, %s" % (mins, name, tail)
    if result == "save_failed":  # write failed, data kept (retained)
        why = "名字已被占用" if detail.get("code") == E_NAME_CONFLICT else "写入失败"
        return ("录制已到%d分钟上限, %s%s自动保存失败, %s. 录制数据还在, 请说保存为新的名字, 或者放弃录制"
                % (mins, kind, name, why))
    if result == "discarded":  # the track did not validate
        return "录制已到%d分钟上限, %s%s的轨迹无效, 没有保存" % (mins, kind, name)
    if result == "awaiting_name":  # an unnamed session: it cannot save itself
        return "录制已到%d分钟上限, 已停止录制. 请说保存为某个名字, 或者放弃录制" % mins
    return None  # an unknown result: said nothing, not guessed


def teach_event_frames(data: bytes, voice_turn_key: Optional[str]) -> list:
    """99 U163: what to publish for one event/*/teach report -- the spoken
    sentence, and a state/voice_turn record so the HMI and the cloud see it in
    the conversation log (layer teach_result, like U162's finish result). []
    for a report that is not ours to speak or not marked first.

    The record has no ASR (asr_raw / asr_text empty: nobody spoke) and no
    intent; it is filed under the channel that started the recording, so the
    operator who started it from the cloud reads its end in the cloud
    session. verdict is accepted for a warning and a save, rejected when the
    recording was not saved (failed / discarded / waiting for a name)."""
    ev = _json(data)  # the event envelope
    detail = ev.get("detail") if isinstance(ev, dict) else None
    if not isinstance(detail, dict) or detail.get("first") is not True:  # only the first report of an episode
        return []
    text = _teach_event_text(detail)  # None for a report not ours
    if text is None:
        return []
    out = [_speak(text, _RTK_SOURCE)]  # a local announcement
    if voice_turn_key:  # no robot id, no record
        issuer = str(detail.get("channel") or "")  # the channel that started the recording
        channel = next((c for p, c in _TEACH_CHANNEL if issuer.startswith(p)), "mic_local")  # mapped onto the voice_turn closed set
        ok = detail.get("kind") == "teach.time_warning" or detail.get("result") == "saved"  # accepted = a warning or a save
        out.append((voice_turn_key, encode_turn({
            "turn_id": new_turn_id(), "session": SESSION_OF_CHANNEL[channel],
            "channel": channel, "asr_raw": "", "asr_text": "", "corrections": [],
            "bypass": None, "intent": None, "slots": {}, "auth_level": None,
            "verdict": "accepted" if ok else "rejected",
            "reject_code": None if ok else (detail.get("code") or E_TEACH_STATE),
            "reply_text": text, "reply_kind": "stream",
            "layer": TEACH_RESULT_LAYER, "ts_mono": time.monotonic()})))
    return out


def _result_text(cmd: Dict[str, Any], ack: Dict[str, Any]) -> Optional[str]:
    """The reply_text of a recording result record (99 U165): the spoken
    sentence, except an accepted start, which is silent (the turn already said
    "开始录制 X") and gets "已开始录制 X" so the record always has a text."""
    if cmd.get("action") == "start" and ack.get("result") == "accepted":  # an accepted start says nothing aloud
        return _REC_STARTED.format(name=(cmd.get("start") or {}).get("name") or "")  # but its record needs a text
    return _sentence(CMD_TEACH, cmd, ack)  # the same sentence as spoken


def teach_result_turn(turn: Dict[str, Any], ack: Dict[str, Any], text: str,
                      spoken: bool) -> Dict[str, Any]:
    """The second state/voice_turn record of a recording start / finish /
    discard (99 U162 / U165): the turn's own record with a new turn_id and the RESULT in it.

    Kept from the turn: channel, session, asr_raw / asr_text (what the
    operator sent -- the only way a cloud client can tie it to its request,
    11 S8.7.6 has no request id), intent, auth_level. Replaced: verdict
    (accepted = saved / discarded, rejected = not saved), reject_code,
    reply_text (the result sentence, e.g. "已保存1号路径, 共64个点, 长320米"),
    reply_kind, layer = teach_result, ts_mono. Measured latencies are the
    turn's and are not copied.
    """
    return result_turn(turn, ack.get("result") == "accepted", ack.get("code"),
                       text, spoken, TEACH_RESULT_LAYER)  # saved / discarded = accepted


def result_turn(turn: Dict[str, Any], accepted: bool, code: Any, text: str,
                spoken: bool, layer: str) -> Dict[str, Any]:
    """A second voice_turn record carrying a RESULT (teach_result: 99 U162 /
    U165 / U167; command_result: 99 U169). Kept from the turn: channel,
    session, asr_raw / asr_text (the only tie to the request -- 11 S8.7.6 has
    no request id), intent, auth_level. Replaced: turn_id, verdict, reject_code,
    reply_text (the result sentence), reply_kind, layer, ts_mono. The turn's
    measured latencies are not copied."""
    rec = {k: v for k, v in turn.items() if k != "latency_ms"}  # the turn's fields
    rec.update({"turn_id": new_turn_id(), "bypass": None, "corrections": [],  # a record of its own
                "verdict": "accepted" if accepted else "rejected",
                "reject_code": None if accepted else code,
                "reply_text": text, "reply_kind": "stream" if spoken else "none",
                "layer": layer, "ts_mono": time.monotonic()})
    return rec  # the result record


def _num(v: Any) -> Optional[str]:
    """A spoken number: 13.5 -> "13.5", 5.0 -> "5"; None when not a number."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None  # absent / not a number
    return "%g" % round(float(v), 1)  # one decimal at most, no trailing .0


def _dock_off_route(name: str, detail: Dict[str, Any]) -> str:
    """99 U169: the refusal of a dock capture off every route, in words."""
    d = _num(detail.get("distance_m"))  # None: there is no route at all
    if d is None:
        return _DOCK_NO_ROUTE.format(name=name)  # record a route first
    limit = _num(detail.get("limit_m"))  # the configured membership distance
    rule = ("在路径%s米以内" % limit) if limit else "建在路径旁边"  # never an invented number
    return _DOCK_OFF_ROUTE.format(name=name, d=d, rule=rule)  # how far, and the rule


def _speak(text: str, source: str) -> Tuple[str, bytes]:
    """One p4_speak_v1 frame, the same shape as a turn reply."""
    payload = {"schema": "p4_speak_v1", "text": text, "source": source,
               "mono_ms": int(time.monotonic() * 1000)}  # same shape as a turn reply
    return (CMD_AUDIO_SPEAK,
            json.dumps(payload, ensure_ascii=False,
                       separators=(",", ":")).encode("utf-8"))  # wire bytes


def _task_label(task: Any) -> str:
    """FS-3: the spoken name of a submitted task, from its type and slots."""
    task = task if isinstance(task, dict) else {}
    params = task.get("params") if isinstance(task.get("params"), dict) else {}
    slots = params.get("slots") if isinstance(params.get("slots"), dict) else {}
    ttype = task.get("type")  # 15 S12 task_type
    if ttype == "goto" and slots.get("waypoint"):
        return "去%s" % slots["waypoint"]  # B01
    if ttype == "patrol":
        return "沿%s巡逻" % slots["route"] if slots.get("route") else "再巡逻"  # B02 / B04
    if ttype == "charge":
        return "去%s" % slots["dock"] if slots.get("dock") else "去充电站"  # B09
    return _TASK_LABEL.get(ttype, "任务")  # B08 / B11 / other


def _fail_why(kind: Any, error: Any) -> str:
    """FS-4: the reason in words; '' when it is not known."""
    error = error if isinstance(error, dict) else {}
    if error.get("cause") in _FAIL_CAUSE:
        return _FAIL_CAUSE[error["cause"]]  # geometry delivery (11 S6.2)
    if error.get("fail_reason"):
        return _FAIL_EN_ROUTE  # P1 failed on the way (11 S3.5B)
    if kind == "rejected":
        return _FAIL_REJECTED  # admission checks
    return ""  # said without a reason, never guessed


def _names(names: Any) -> str:
    """99 U184: a list of names read out: "A和B", "A, B, C", or the first
    _NAMES_SAID then "等N个"."""
    names = [str(n) for n in (names or ()) if n]  # empty names dropped
    if len(names) <= 2:  # one or two
        return "和".join(names)  # "A和B"
    if len(names) <= _NAMES_SAID:  # a short list
        return ", ".join(names)  # read out whole
    return ", ".join(names[:_NAMES_SAID]) + "等%d个" % len(names)  # capped, with the count


def _layout_notes(detail: Dict[str, Any]) -> str:
    """99 U184: the "out of reach" sentences an accepted fence / waypoint /
    route / enable ack carries, joined, or "" -- from the save's advisory
    issues, the enable's warnings, the unreachable report and the robot note
    (P3 fills them; see p3_task/fence/reach.py)."""
    out = []  # sentences, in this order
    issues = ((detail.get("validation") or {}).get("issues") or []) + (detail.get("warnings") or [])  # save / enable
    for i in issues:  # advisory issues of a save, warnings of an enable
        if not isinstance(i, dict):  # malformed entry
            continue  # skipped
        if i.get("code") == "outside_activity_area":  # this fence, outside the area
            out.append(_LAYOUT_NOTE["outside_activity_area"].format(area=i.get("activity_area") or ""))
        elif i.get("code") == "activity_area_excludes":  # a new area leaving fences out
            out.append(_LAYOUT_NOTE["activity_area_excludes"].format(fences=_names(i.get("fences"))))
    for u in detail.get("unreachable") or ():
        if isinstance(u, dict) and u.get("objects"):  # one entry per blocking fence
            which = "inside_forbid" if u.get("reason") == "inside_forbid" else "objects_outside"
            out.append(_LAYOUT_NOTE[which].format(objs=_names(u["objects"]),
                                                  fence=u.get("fence") or ""))
    robot = detail.get("robot") if isinstance(detail.get("robot"), dict) else {}  # enable only
    if robot.get("note") in _ROBOT_NOTE:  # the robot itself outside / inside
        out.append(_ROBOT_NOTE[robot["note"]].format(fence=robot.get("fence") or ""))
    return ". ".join(out)  # one string, "" when nothing to say


def _with_notes(text: str, detail: Dict[str, Any]) -> str:
    """text, then the layout notes if there are any (99 U184)."""
    notes = _layout_notes(detail)  # what is out of reach
    return text + ". " + notes if notes else text  # appended after the result


def _issue_text(issue: Dict[str, Any]) -> str:
    """One blocking issue in words; forbid_overlap names the fences (99 U184)."""
    return _REC_ISSUE[issue["code"]].format(fences=_names(issue.get("fences")))


def _sentence(key: str, cmd: Dict[str, Any], ack: Dict[str, Any]) -> Optional[str]:
    """What to say for one answer; None when nothing more needs saying."""
    detail = ack.get("detail") if isinstance(ack.get("detail"), dict) else {}
    accepted = ack.get("result") == "accepted"  # the executor did it
    code = ack.get("code")  # the refusal code, if any
    if key == CMD_TEACH and cmd.get("action") == "start":
        return _start_sentence(cmd, ack, detail, accepted, code)  # 99 U133
    if key == CMD_TEACH and cmd.get("action") == "finish":
        return _finish_sentence(detail, accepted, code)  # 99 U133
    if key == CMD_TEACH and cmd.get("action") == "discard":
        if accepted:  # 99 U137: what was thrown away
            return _REC_DISCARDED.format(name=detail.get("name") or "",
                                         count=int(detail.get("discarded_points") or 0))
        return "现在没有在录制"  # no session to discard
    if key == CMD_TEACH:
        name = (cmd.get("mark_once") or {}).get("name") or ""  # what was said
        if accepted:
            applied = detail.get("applied") or {}  # AP-1: the applied values
            said = applied.get("name") if isinstance(applied.get("name"), str) else name
            warn = detail.get("warn") or []  # e.g. no_heading
            return _with_notes((_STORED_NO_HEADING if "no_heading" in warn  # 99 U184 notes
                                else _STORED).format(name=said), detail)
        if detail.get("item") == "snap_age":
            return _SNAP_AGE.format(name=name)  # the snapshot, not the fix
        if code == E_GEO_INVALID and detail.get("reason") == "dock_off_route":
            return _dock_off_route(name, detail)  # 99 U169: distance, limit, what to do
        return _TEACH_REFUSED.get(code, _TEACH_OTHER).format(name=name)  # by code
    if (key == CMD_TASK and accepted and cmd.get("action") == "clear_queue"
            and cmd.get("filter") == {"state": "scheduled"}):
        n = len(detail.get("cleared_ids") or [])  # S7.2: cleared_ids is mandatory
        return _SCHED_CLEARED.format(n=n) if n else _SCHED_NONE_LEFT
    if (key == CMD_TASK and accepted and cmd.get("action") == "clear_queue"
            and not cmd.get("filter")):  # 99 U174: the whole queue
        n = len(detail.get("cleared_ids") or [])
        return _QUEUE_CLEARED.format(n=n) if n else _QUEUE_NONE_LEFT
    if key == CMD_GEO and accepted:
        # 99 U184: the turn said "好的,启用X"; what it leaves out of reach,
        # and where the robot stands, is said after it
        return _layout_notes(detail) or None
    if accepted:
        return None  # the turn already said what it does
    if key == CMD_TASK:
        if detail.get("reason") in _TASK_BY_REASON:  # a refusal with its own sentence
            return _TASK_BY_REASON[detail["reason"]]  # U131
        verb = _TASK_VERB.get(cmd.get("action"), "执行")  # the refused action
        return _TASK_REFUSED.get(code, _TASK_OTHER).format(verb=verb)  # by code
    if key == CMD_GEO:
        return _GEO_REFUSED.get(code, _GEO_OTHER)  # by code
    if code == E_BUSY:
        return _MOTION_BUSY.get(detail.get("reason"), _MOTION_OTHER)  # by reason
    return _MOTION_REFUSED.get(code, _MOTION_OTHER)  # motion, by code


def _start_sentence(cmd: Dict[str, Any], ack: Dict[str, Any], detail: Dict[str, Any],
                    accepted: bool, code: Any) -> Optional[str]:
    """A recording start's answer (99 U133): nothing when accepted (the turn
    said "开始录制 X"), the reason when P3 refused it."""
    if accepted:
        return None  # the turn already said it
    name = (cmd.get("start") or {}).get("name") or ""  # the name said
    reason = detail.get("reason")  # told apart below
    if (reason == "recording_config_missing"  # 99 U176: told apart by the key P3 names
            and "fence.speed_limit_default_mps" in (detail.get("missing") or [])):  # the U185 key
        return _REC_START_NO_SPEED_LIMIT  # 99 U176: only this kind is refused
    if reason in _REC_START_BY_REASON:  # a reason with its own sentence
        return _REC_START_BY_REASON[reason].format(  # by reason
            existing=detail.get("existing") or "")  # U176 the activity area there
    if code == E_TEACH_QUALITY:  # check 3: not rtk_fixed
        return _REC_START_QUALITY  # the fix
    if code == E_UNHEALTHY:  # check 4: health forbids motion
        return _REC_START_UNHEALTHY  # health
    return _REC_START_REFUSED.get(code, _REC_START_OTHER).format(name=name)  # by code


def _finish_sentence(detail: Dict[str, Any], accepted: bool, code: Any) -> Optional[str]:
    """A named recording's finish (99 U133): saved -> what was saved; not
    saved -> why. An unnamed finish (no `saved` key) says nothing here."""
    if accepted and detail.get("saved") is True:  # saved under the start name
        applied = detail.get("applied") or {}  # AP-1 values
        tpl = _REC_SAVED_FENCE if detail.get("kind") == "fence" else _REC_SAVED_ROUTE  # by kind
        return _with_notes(tpl.format(name=applied.get("name") or "",  # 99 U184 notes
                                      count=int(applied.get("point_count") or 0),
                                      length=int(round(float(applied.get("total_len_m") or 0.0)))),
                           detail)
    if accepted:  # an unnamed session waiting for save (HMI path)
        return None  # nothing to say here
    if detail.get("retained"):  # 99 U165: write failed, data kept in finalizing
        why = "名字已被占用" if code == E_NAME_CONFLICT else "写入失败"  # as U163 says it
        return _REC_RETAINED.format(name=detail.get("name") or "", why=why)  # data kept: save under a new name or discard
    if code == E_TEACH_GEOMETRY and detail.get("close_reason") == "invalid_geometry":
        issues = detail.get("issues") or []  # blocking issues
        why = next((_issue_text(i) for i in issues
                    if isinstance(i, dict) and i.get("code") in _REC_ISSUE),
                   "几何检查没有通过")  # first known issue
        return _REC_NOT_SAVED.format(name=detail.get("name") or "", why=why)
    if code == E_TEACH_STATE:  # no session / wrong id
        return "现在没有在录制"  # nothing to end
    return _REC_FINISH_OTHER  # any other refusal

