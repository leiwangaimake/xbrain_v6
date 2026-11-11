"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: record_slots.py
Brief: deterministic parse of the recording commands -- start with a full name, exit, status (99 U133)

Description:
What problem this solves. Until 2026-11-05 a recording started from a bare
keyword ("开始录制路径", F01 / F07, no slot), ended with "结束录制" and was named
in a THIRD turn (F03 / F09), with an overwrite question on a clash. The user
ruled (99 U133): the start sentence carries the full name ("现在开始录制1号
路径"); any recording sentence without a full route or fence name is refused
with a spoken reason and never followed by a question; "退出路径录制 / 退出录制
/ 录制完成" ends the recording AND saves it. Fence recording works exactly the
same way. This module turns those sentences into one of four results:

  start          "[前缀] (开始) 录制 X" / "开始记录 X", X a full name
                 -> {kind: start, name: X, rtype: route | fence}
  start_noname   "开始录制" / "开始录制路径" / "录制1号" -- a 录制 sentence
                 whose object is not a full name -> refused by the caller
  exit           "退出(路径|围栏)录制" / "(路径|围栏)录制完成" / "记录完成"
                 / "结束(路径|围栏)录制" / "录完了" / "围栏录完了" / "结束围栏"
  cancel         "取消录制" / "放弃录制" / "取消路径(围栏)录制" -- discard at once,
                 no confirmation (99 U137)
  named exit     "1号路径录制完成" / "退出东门围栏录制" / "结束1号路径录制" /
                 "1号路径录完了" -> {kind: exit, name, rtype} (99 U162)
  named cancel   "取消1号路径录制" / "放弃东门围栏录制" -> {kind: cancel, name,
                 rtype} (99 U162). A named exit / cancel only counts when the
                 name is a FULL name; the caller checks it is the one recording.
  status         "录制状态" / "录了多少" / "录了几个点" / "正在录制吗"

A FULL name is the whole object and ends with its type word: 路径 / 路线 /
线路 / 环线 make a route recording, 围栏 a fence recording. "叫 / 名为 / 命名
为" introduce the name ("录制一条路径叫1号路径" -> 1号路径). The 2-12 length rule
and the catalogue clash are the caller's (it has the catalogue).

Boundary: this parses only. Whether a recording is running, whether the name
is taken and what is said are turn_orchestrator's; the session itself is P3's
(11 S12A).

Looks right but is wrong:
  * claiming every sentence that starts with 记录 -- "记录当前位置为1号点" is a
    keypoint capture (F06). 记录 starts a recording only when its object is
    a full route / fence name; 录制 always belongs here.
  * accepting "开始录制路径" with name "路径" -- the bare type word names
    nothing, and saving under it would make every later 路径 sentence
    ambiguous.
  * any exit phrase with 停 (停止录制): the safety bypass matches 停 as a
    substring first (16 S4.1, 99 U96), so such a phrase would only ever stop
    the robot. None is listed here.
"""
from __future__ import annotations  # postponed annotations

import re  # sentence patterns
from dataclasses import dataclass  # a frozen result
from typing import Optional  # plain values

#: Spaces and sentence punctuation (ASCII and full-width) dropped before
#: matching; the full-width marks are built from code points so this file
#: stays ASCII-punctuated (charset rule).
_FULL = "".join(chr(c) for c in (0x3001, 0x3002, 0xFF0C, 0xFF01, 0xFF1F,
                                 0xFF1B))  # CJK comma / stop, full-width , ! ? ;
_PUNCT = re.compile("[\\s,.!?;" + _FULL + "]")  # dropped everywhere
#: Courtesy / urgency words in front of a command, longest first.
_LEAD = ("现在请你", "现在请", "麻烦你", "请你", "请您", "麻烦", "帮我", "给我",
         "现在", "立刻", "立即", "马上", "那就", "然后", "好的", "你", "请")  # stripped
#: Type words that end a full name, and what they make.
_ROUTE_WORDS = ("路径", "路线", "线路", "环线")  # a route recording
_FENCE_WORD = "围栏"  # a fence recording
#: Exit sentences (whole sentence, after the lead words are stripped).
_EXIT = re.compile(
    r"^(?:(?:退出|结束)(?:路径|路线|围栏)?(?:录制|记录)(?:模式)?"
    r"|(?:路径|路线|围栏)?(?:录制|记录)(?:完成|完毕|结束|好了)"
    r"|(?:围栏)?录完了|结束围栏)(?:了|吧|啦)*$")  # no 停 anywhere (bypass)
#: 99 U137: cancel = discard at once (no confirmation) -- explicit phrases only.
_CANCEL = re.compile(r"^(?:取消|放弃)(?:路径|路线|围栏)?(?:录制|记录)(?:吧|了|啦)*$")  # discard
#: 99 U162 (user ruling 2026-11-11): exit / cancel sentences that NAME the
#: recording ("1号路径录制完成", "取消东门围栏录制"). The named part must be a
#: full name (_rtype), checked after the match. Tried after the unnamed forms,
#: so "路径录制完成" (the bare type word) stays the unnamed exit.
_EXIT_NAMED = re.compile(
    r"^(?:(?:退出|结束)(?P<a>.+?)(?:录制|记录)(?:模式)?"
    r"|(?P<b>.+?)(?:录制|记录)(?:完成|完毕|结束|好了)"
    r"|(?P<c>.+?)录完了)(?:了|吧|啦)*$")  # no 停 anywhere (bypass)
_CANCEL_NAMED = re.compile(r"^(?:取消|放弃)(?P<a>.+?)(?:录制|记录)(?:吧|了|啦)*$")
#: Status questions about the running recording.
_STATUS = re.compile(r"^(?:录制状态|录制进度|录了多少(?:米|个点)?|录了几个点|录到哪了"
                     r"|正在录制吗|还在录制吗|在录制吗)(?:了|呢|啊)*$")  # questions
#: Start: 录制 always ours; 记录 only with a full name (F06 shares the verb).
_START = re.compile(r"^(?:开始)?(?P<verb>录制|记录)(?P<x>.*)$")  # verb first
#: Words between the verb and the name that are not part of it.
_FILLER_HEAD = ("一条新的", "一个新的", "一条", "一个", "新的", "新")  # dropped in front
#: Words that introduce the name ("录制一条路径叫1号路径").
_NAMED_AS = ("命名为", "名字叫", "名为", "叫做", "叫")  # take what follows


@dataclass(frozen=True)  # immutable result
class RecordParse:  # one parsed recording sentence
    """kind is start / start_noname / exit / cancel / status; name and rtype are set
    for start, and for a NAMED exit / cancel (99 U162; "" / None = unnamed)."""
    kind: str  # start / start_noname / exit / cancel / status
    name: str = ""  # the full name as said (start, named exit / cancel)
    rtype: Optional[str] = None  # route / fence (start, named exit / cancel)


def _strip_lead(t: str) -> str:
    """t without leading courtesy / urgency words (repeatedly)."""
    changed = True  # loop until nothing more comes off
    while changed and t:  # e.g. "现在请马上..."
        changed = False  # assume done
        for w in _LEAD:  # longest first
            if t.startswith(w) and len(t) > len(w):  # keep a non-empty rest
                t, changed = t[len(w):], True  # strip one word
                break  # restart from the longest
    return t  # what is left


def _name_of(x: str) -> str:
    """The name inside a start's object: after 叫 / 名为 when present, else the
    object without filler in front and particles behind."""
    for w in _NAMED_AS:  # "...叫1号路径"
        if w in x:  # an explicit name follows
            x = x.split(w, 1)[1]  # what follows it
            break  # the first introducer wins
    for w in _FILLER_HEAD:  # "一条新的..."
        if x.startswith(w):  # filler in front
            x = x[len(w):]  # dropped
            break  # one filler at most
    return x.rstrip("吧了啊呀")  # trailing particles


def _rtype(name: str) -> Optional[str]:
    """route / fence for a full name, None when it does not end with a type
    word or IS only the type word ("路径")."""
    if name.endswith(_FENCE_WORD) and len(name) > len(_FENCE_WORD):  # "东门围栏"
        return "fence"  # a fence recording
    for w in _ROUTE_WORDS:  # "1号路径" / "东环线"
        if name.endswith(w) and len(name) > len(w):  # more than the bare word
            return "route"  # a route recording
    return None  # not a full name


def parse_record(text: str) -> Optional[RecordParse]:
    """The recording command text is, or None when it is not one.

    Exit and status are tried first (their sentences contain 录制 too), then
    start. None means "not a recording sentence" -- the keyword / LLM layers
    then see it, as before.
    """
    t = _strip_lead(_PUNCT.sub("", text or ""))  # normalised
    if not t:  # nothing left
        return None  # not ours
    if _EXIT.match(t):  # "退出录制" / "录制完成"
        return RecordParse("exit")  # end and save
    if _CANCEL.match(t):  # "取消录制" (99 U137)
        return RecordParse("cancel")  # discard, no confirmation
    if _STATUS.match(t):  # "录了几个点"
        return RecordParse("status")  # answered from state/teach
    for rx, kind in ((_EXIT_NAMED, "exit"), (_CANCEL_NAMED, "cancel")):  # 99 U162
        mn = rx.match(t)  # "1号路径录制完成" / "取消东门围栏录制"
        if mn:  # the shape fits; is the named part a full name?
            nm = next(g for g in mn.groups() if g)  # whichever branch matched
            rt = _rtype(nm)  # route / fence / not a full name
            if rt is not None:  # a full name: a named exit / cancel
                return RecordParse(kind, name=nm, rtype=rt)
    m = _START.match(t)  # "开始录制X" / "记录X"
    if not m:  # no recording verb at the front
        return None  # not ours
    name = _name_of(m.group("x"))  # the object as a name
    rtype = _rtype(name)  # route / fence / not a full name
    if rtype is None:  # "开始录制" / "开始录制路径" / "录制1号"
        if m.group("verb") == "记录":  # "记录当前位置为X" is F06's
            return None  # leave it to the keyword layer
        return RecordParse("start_noname")  # refused with the reason
    return RecordParse("start", name=name, rtype=rtype)  # a named start
