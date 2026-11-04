"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: teach_ack_speech.py
Brief: cmd/teach/ack -> one spoken result for a voice keypoint capture (11 CMD-30 / AP-1)

Description:
F06 record_waypoint and F10 record_dock are L1a: P4 says "好的" and sends a
mark_once, and P3 decides. Until 99 U109 nothing in P4 listened to
cmd/teach/ack, so the operator heard "收到" whether the point was stored or
P3 refused it (no rtk_fixed indoors, a name already in use, a stale
snapshot) -- the 2026-11-04 audit found an operator could "record" a point
that never existed. 11 S7.7 AP-1 / CMD-30 is the rule this closes: an L1
command is restated from what the executor ACTUALLY applied (`detail.applied`),
and 11 S2.2.4 lists the initiator as a subscriber of cmd/teach/ack.

How it is matched to the turn: the wiring hands every turn's published frames
to note_turn(). A mark_once frame is remembered under its cmd_id together
with the TTS source of that turn's own reply -- and only when the turn spoke
at all, so a typed command sent with require_tts_reply=false (11 S8.7.5)
stays silent here too. on_ack() turns the matching ack into one sentence on
the same source, then forgets the cmd_id; a duplicate ack (S12A.4 replay)
says nothing a second time.

Scope: mark_once only. Session actions (start / finish / save ...) already
answer through state/teach and the turn's own reply; widening this to them is
a separate change with its own wording.

Bounded: at most _PENDING_MAX outstanding captures, oldest dropped. An ack
that never comes (P3 down) costs one slot, never memory growth, and needs no
timer -- so no time constant lives here.
"""
from __future__ import annotations

import json  # frames in, frames out
import time  # mono_ms on the outgoing speak, like every p4_speak_v1
from collections import OrderedDict  # insertion order = age, for eviction
from typing import Any, Iterable, Optional, Tuple  # plain values

from xbrain.common.errors import (  # the 11 S13.11 closed set, never literals
    E_NAME_CONFLICT,
    E_SCHEMA,
    E_TEACH_QUALITY,
)
from xbrain.p4_agent.runtime.intent_dispatch import CMD_AUDIO_SPEAK, CMD_TEACH  # the two keys

#: 11 S2.2.4: the teach answer key, next to the command key it answers.
CMD_TEACH_ACK = CMD_TEACH + "/ack"  # "cmd/teach/ack"

#: Outstanding captures remembered at once (one per turn; operators do not
#: record dozens of points inside one ack round trip).
_PENDING_MAX = 32  # slots, not a time

#: Spoken per refusal code; {name} is the name the operator gave. ASCII
#: punctuation (charset rule), Chinese words.
_REFUSED = {
    E_NAME_CONFLICT: "{name}这个名字已经有了, 请换个名字",  # 11 S12A.8 conflict row
    E_TEACH_QUALITY: "{name}没有记录, 定位精度不够, 需要RTK固定解",  # rtk_fixed rule
    E_SCHEMA: "{name}没有记录, 指令格式不对",  # a builder bug, said plainly
}
_SNAP_AGE = "{name}没有记录, 位置快照已过期, 请再说一遍"  # 18 PS-6 snap_age
_REFUSED_OTHER = "{name}没有记录成功"  # any other code
_STORED = "已记录{name}"  # accepted
_STORED_NO_HEADING = "已记录{name}, 没有记下朝向"  # accepted, warn no_heading


def _json(data: bytes) -> Any:
    """Decode one frame; None for anything that is not JSON."""
    try:
        return json.loads(data)  # bytes are accepted by json.loads
    except (ValueError, TypeError):
        return None  # not ours to fix


def _body(frame: Any) -> Any:
    """The bare body of a frame that may be 11 S3.0-enveloped."""
    if isinstance(frame, dict) and isinstance(frame.get("data"), dict):
        return frame["data"]  # enveloped
    return frame  # bare (p3 publishes teach acks bare today)


class TeachAckSpeaker:
    """Remembers this process's mark_once commands and voices their acks."""

    def __init__(self) -> None:
        # cmd_id -> (name, speak source). Ordered so the oldest is evicted.
        self._pending: "OrderedDict[str, Tuple[str, str]]" = OrderedDict()

    def note_turn(self, pairs: Iterable[Tuple[str, bytes]]) -> None:
        """Remember the mark_once among one turn's published frames, if any.

        Only a turn that also spoke is remembered: the result is said on the
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
        for key, data in pairs:  # find the capture
            if key != CMD_TEACH:
                continue  # not a teach frame
            cmd = _body(_json(data))  # TeachCommand
            if not isinstance(cmd, dict) or cmd.get("action") != "mark_once":
                continue  # sessions answer elsewhere
            mark = cmd.get("mark_once") or {}  # the S12A.4 block
            cmd_id, name = cmd.get("cmd_id"), mark.get("name")  # match key + words
            if isinstance(cmd_id, str) and isinstance(name, str):
                self._pending[cmd_id] = (name, source)  # newest last
                while len(self._pending) > _PENDING_MAX:  # bounded
                    self._pending.popitem(last=False)  # drop the oldest

    def on_ack(self, data: bytes) -> Optional[Tuple[str, bytes]]:
        """The (key, frame) to publish for one cmd/teach/ack, or None."""
        ack = _body(_json(data))  # S7.7 Ack shape
        if not isinstance(ack, dict) or ack.get("result") == "duplicate":
            return None  # nothing new to say
        hit = self._pending.pop(ack.get("cmd_id"), None)  # ours?
        if hit is None:
            return None  # another initiator's, or already answered
        name, source = hit  # what the operator said, where to say it
        text = self._sentence(ack, name)  # one sentence
        payload = {"schema": "p4_speak_v1", "text": text, "source": source,
                   "mono_ms": int(time.monotonic() * 1000)}  # same shape as a turn reply
        return (CMD_AUDIO_SPEAK,
                json.dumps(payload, ensure_ascii=False,
                           separators=(",", ":")).encode("utf-8"))  # wire bytes

    @staticmethod
    def _sentence(ack: dict, name: str) -> str:
        """The spoken result: what was applied, or why not."""
        detail = ack.get("detail") if isinstance(ack.get("detail"), dict) else {}
        if ack.get("result") == "accepted":
            applied = detail.get("applied") or {}  # AP-1: the applied values
            said = applied.get("name") if isinstance(applied.get("name"), str) else name
            warn = detail.get("warn") or []  # e.g. no_heading
            template = _STORED_NO_HEADING if "no_heading" in warn else _STORED
            return template.format(name=said)  # restated from the ack
        if detail.get("item") == "snap_age":
            return _SNAP_AGE.format(name=name)  # the snapshot, not the fix
        return _REFUSED.get(ack.get("code"), _REFUSED_OTHER).format(name=name)  # by code
