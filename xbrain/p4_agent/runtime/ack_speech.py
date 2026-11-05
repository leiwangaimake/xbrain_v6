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
                         Session actions answer through state/teach.
  cmd/task/ack           submit / pause / resume / cancel: refused -> why.
  cmd/geo/ack            delete / rename / set_state: refused -> why.
  cmd/motion/intent/ack  A05-A17, A04 hold, B06 release: refused -> why.
An accepted command other than a capture says nothing more: the turn already
said what it does, and saying it twice is noise on a speaker that also does
broadcasts. (99 U109 for the capture, U111 for the other three keys.)

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
    E_CAPABILITY,  # G-7
    E_CHANNEL_DENIED,  # S7.9.5 matrix
    E_DEGRADED,  # S3.2.1
    E_GEO_CONFLICT,  # base_rev
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

#: The answer keys this speaker subscribes, one per command key (11 S2.2.4).
ACK_KEYS = tuple(k + "/ack" for k in (CMD_TEACH, CMD_TASK, CMD_GEO,
                                      CMD_MOTION_INTENT))  # wiring subscribes these

#: Outstanding commands remembered at once (one per turn).
_PENDING_MAX = 32  # slots, not a time

#: cmd/teach/ack wording; {name} is the name the operator gave. ASCII
#: punctuation (charset rule), Chinese words.
_TEACH_REFUSED = {
    E_NAME_CONFLICT: "{name}这个名字已经有了, 请换个名字",  # 11 S12A.8 conflict row
    E_TEACH_QUALITY: "{name}没有记录, 定位精度不够, 需要RTK固定解",  # rtk_fixed rule
    E_SCHEMA: "{name}没有记录, 指令格式不对",  # a builder bug, said plainly
}
_SNAP_AGE = "{name}没有记录, 位置快照已过期, 请再说一遍"  # 18 PS-6 snap_age
_TEACH_OTHER = "{name}没有记录成功"  # any other code
_STORED = "已记录{name}"  # accepted
_STORED_NO_HEADING = "已记录{name}, 没有记下朝向"  # accepted, warn no_heading

#: cmd/task/ack: what the refused action was, in words.
_TASK_VERB = {"pause": "暂停", "resume": "继续", "cancel": "取消",
              "submit": "执行"}  # S7.2 actions
_TASK_REFUSED = {
    E_TASK_STATE: "这个任务现在的状态不能{verb}",  # S7.2: pause needs running etc.
    E_NOT_FOUND: "没找到这个任务, 没有{verb}",  # gone meanwhile
    E_DEGRADED: "定位降级, 现在不接受新任务",  # 11 S3.2.1
}
_TASK_OTHER = "任务没有{verb}成功"  # any other code
#: cmd/task/ack refusals told apart by detail.reason before the code.
_TASK_BY_REASON = {
    # 99 U131 B04: P3 found no completed patrol to repeat
    "no_completed_patrol": "还没有巡逻完成过的路径, 请说沿哪条路径巡逻",
}

#: cmd/geo/ack.
_GEO_REFUSED = {
    E_CHANNEL_DENIED: "这个操作不允许从这里发起",  # 11 S7.9.5 matrix
    E_GEO_CONFLICT: "这个对象刚被别处修改过, 请再说一遍",  # base_rev conflict
    E_NAME_CONFLICT: "这个名字已经被占用, 请换个名字",  # rename target taken
    E_NOT_FOUND: "没找到这个对象",  # deleted meanwhile
}
_GEO_OTHER = "这个对象没有改成功"  # any other code

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
        # cmd_id -> (command key, what to say it about, speak source).
        self._pending: "OrderedDict[str, Tuple[str, Dict[str, Any], str]]" = OrderedDict()

    def note_turn(self, pairs: Iterable[Tuple[str, bytes]]) -> None:  # a turn's frames
        """Remember the command frames among one turn's published frames.

        Only a turn that also spoke is remembered: the answer is said on the
        same TTS source the turn's own reply used.
        """
        pairs = list(pairs)  # walked twice
        source = None  # the turn's reply source, if it spoke
        for key, data in pairs:  # find the reply
            if key == CMD_AUDIO_SPEAK:
                speak = _json(data)  # p4_speak_v1
                if isinstance(speak, dict) and isinstance(speak.get("source"), str):
                    source = speak["source"]  # tts_local / tts_cloud / ...
                    break  # the first reply is the turn's
        if source is None:
            return  # a silent turn stays silent
        for key, data in pairs:  # find the commands
            if key not in (CMD_TEACH, CMD_TASK, CMD_GEO, CMD_MOTION_INTENT):
                continue  # not a command this speaker answers for
            cmd = _body(_json(data))  # the command body
            if not isinstance(cmd, dict) or not isinstance(cmd.get("cmd_id"), str):
                continue  # nothing to match an answer to
            if key == CMD_TEACH and cmd.get("action") != "mark_once":
                continue  # sessions answer through state/teach
            self._pending[cmd["cmd_id"]] = (key, cmd, source)  # newest last
            while len(self._pending) > _PENDING_MAX:  # bounded
                self._pending.popitem(last=False)  # drop the oldest

    def on_ack(self, data: bytes) -> Optional[Tuple[str, bytes]]:
        """The (key, frame) to publish for one ack, or None."""
        ack = _body(_json(data))  # S7.7 Ack shape
        if not isinstance(ack, dict) or ack.get("result") == "duplicate":
            return None  # nothing new to say
        hit = self._pending.pop(ack.get("cmd_id"), None)  # ours?
        if hit is None:
            return None  # another initiator's, or already answered
        key, cmd, source = hit  # what was sent, where to say it
        text = _sentence(key, cmd, ack)  # one sentence, or None
        if text is None:
            return None  # accepted: the turn already said it
        payload = {"schema": "p4_speak_v1", "text": text, "source": source,
                   "mono_ms": int(time.monotonic() * 1000)}  # same shape as a turn reply
        return (CMD_AUDIO_SPEAK,
                json.dumps(payload, ensure_ascii=False,
                           separators=(",", ":")).encode("utf-8"))  # wire bytes


def _sentence(key: str, cmd: Dict[str, Any], ack: Dict[str, Any]) -> Optional[str]:
    """What to say for one answer; None when nothing more needs saying."""
    detail = ack.get("detail") if isinstance(ack.get("detail"), dict) else {}
    accepted = ack.get("result") == "accepted"  # the executor did it
    code = ack.get("code")  # the refusal code, if any
    if key == CMD_TEACH:
        name = (cmd.get("mark_once") or {}).get("name") or ""  # what was said
        if accepted:
            applied = detail.get("applied") or {}  # AP-1: the applied values
            said = applied.get("name") if isinstance(applied.get("name"), str) else name
            warn = detail.get("warn") or []  # e.g. no_heading
            return (_STORED_NO_HEADING if "no_heading" in warn else _STORED).format(name=said)
        if detail.get("item") == "snap_age":
            return _SNAP_AGE.format(name=name)  # the snapshot, not the fix
        return _TEACH_REFUSED.get(code, _TEACH_OTHER).format(name=name)  # by code
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
