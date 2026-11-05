"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ptp_slots.py
Brief: deterministic parse of the PTP commands -- go to X, delete X, rename X to Y (99 U110)

Description:
The three keypoint commands an operator gives most often, parsed by rule
before any keyword or LLM layer sees them:

  goto    "[前缀] 去 / 前往 / 导航到 / 走到 ... X"         -> B01 {waypoint: X}
  delete  "[前缀] 删除 / 删掉 X"  or  "把 X 删掉"            -> F12 {waypoint: X}
  rename  "[前缀] 把 / 将 X 改名为 / 重命名为 Y"            -> F14 {old: X, new: Y}
  patrol  "[前缀] 沿 / 按 / 顺着 X 巡逻 [N圈]", "巡逻 X", "去 X 巡逻"
                                                       -> B02 {route: X, loops: N}
          (99 U121, 2026-11-05: the keyword layer only matched 巡逻 and left
          the route empty, and the tier-2 LLM has no route catalogue, so a
          named patrol always ended in "which route?". Same cure as U110.)

Why by rule (2026-11-04, measured on the real 3B model): the M3 / M7 / M6
prompts tell the model to copy an id from a 候选 (candidate) list that the
runtime never supplies, so it invented them -- "前往1号测试点" became
waypoint "1-test_point", "去东门岗亭" copied the prompt's own example id
"w-gate_post", "去1号测试点" came back as a patrol tomorrow at 00:00,
"删除1号测试点" as "r-test_point_1", and a rename lost its old name. The
structure of these sentences is fixed and small; the name is whatever
follows the verb, and the catalogue -- not a model -- decides whether it
exists (geo_request.resolve_geo_id, canonical spelling).

Prefixes (the operator's words, not ours to predict): courtesy and urgency
words (现在 / 请 / 立刻 / 马上 / 麻烦 / 帮我 ...) are stripped, and a goto may
start with a time (registry.time_expr.split_time_prefix: "下午3点", "5分钟
后", "明天14点"). Each strip is tried only if what remains then matches, and
the unstripped text is tried first -- so a name that begins with such a
word ("快递站改名为...") is not cut.

What is deliberately NOT taken: a goto whose object mentions patrol,
charging, a route or a fence (B02 / B09 / F-class own those), a question
("到底怎么回事", "去哪了"), "去掉" (a delete verb), and a delete / rename of
a route or fence (F11 / F13 / F14 by their own keywords). A rename needs the
word 名 (改名为 / 名字改成 ...): a bare "改成" also changes lamps and speeds.
"""
from __future__ import annotations  # postponed annotations

import re  # sentence patterns
from dataclasses import dataclass  # a frozen result
from typing import Optional  # plain values

from xbrain.p4_agent.registry.time_expr import TimeExpr, split_time_prefix  # time prefixes

#: Spaces and sentence punctuation (ASCII and full-width) removed before
#: matching. The full-width marks are built from code points so this file
#: stays ASCII-punctuated (charset rule); ':' is kept for "15:30".
_FULL = "".join(chr(c) for c in (0x3001, 0x3002, 0xFF0C, 0xFF01, 0xFF1F,
                                 0xFF1B))  # CJK comma / stop, full-width , ! ? ;
_PUNCT = re.compile("[\\s,.!?;" + _FULL + "]")  # dropped everywhere

#: Leading words that carry no command content, longest first.
_LEAD = ("现在请你", "现在请", "麻烦你", "请你", "请您", "麻烦", "帮我", "给我",
         "现在", "立刻", "立即", "马上", "赶紧", "赶快", "尽快", "快点", "快",  # urgency words
         "海卫", "机器人", "好的", "然后", "接着", "那就", "再", "你", "请")  # wake / filler words

#: Goto verbs, longest first. 去 is not a goto in 去掉 / 去除 (delete) or
#: 去巡 (patrol).
_GOTO = re.compile(r"^(?:导航到|导航去|导航至|去往|前往|走到|开到|去到|移动到"
                   r"|跑到|到|去(?![掉除巡]))(?P<x>.+)$")  # 去 but not 去掉 / 去除 / 去巡

#: Delete, verb first: "删除(航点)X(这个点)".
_DEL_FRONT = re.compile(r"^(?:删除|删掉|删去|移除)(?:航点|点位|关键点|位置点)?"
                        r"(?P<x>.+?)(?:这个点|这个航点|这个位置)?(?:吧|了|啊)*$")  # trailing 这个点 / 吧 dropped
#: Delete, object first: "把(航点)X(这个点)删掉".
_DEL_BACK = re.compile(r"^(?:把|将)(?:航点|点位|关键点)?(?P<x>.+?)"
                       r"(?:这个点|这个航点|航点|点位)?给?"  # optional 这个点 / 航点 before the verb
                       r"(?:删除|删掉|删了|删去|移除|去掉)(?:吧|了|啊)*$")  # the delete verb last

#: Rename: the verb must say 名 -- "改名为 / 更名为 / 重命名为 / 名字改成 ...".
_RENAME = re.compile(r"^(?:把|将)?(?:航点|点位|关键点)?(?P<old>.+?)(?:这个点)?"
                     r"(?:的名字改为|的名字改成|的名字改叫|名字改为|名字改成"  # only verbs that say 名
                     r"|改名为|改名成|改名叫|更名为|重命名为)(?P<new>.+?)(?:吧|了)*$")  # the new name last

#: Trailing filler after a goto object ("去1号测试点一趟吧").
_GOTO_TAIL = ("那里", "那边", "那儿", "一趟", "一下", "看看", "去", "吧", "呀",
              "啊", "了")  # particles
#: Type words in front of a name ("去航点1号测试点").
_TYPE_HEAD = ("航点", "点位", "关键点", "位置点")  # said before a name

#: An object with these is not a keypoint goto / delete (other intents).
_NOT_KEYPOINT = ("巡", "围栏", "路径", "路线", "充电", "桩")  # other intents' objects
#: Words that point at a place without naming it ("到这里啊", "去那边"): not
#: a keypoint. Left to B01's own leg, which asks for the destination.
_DEICTIC = frozenset({"这", "那", "这里", "那里", "这边", "那边", "这儿", "那儿",
                      "这个点", "那个点", "这个地方", "那个地方"})  # not names

#: A goto object with these is a question, not a place.
_QUESTION = ("吗", "呢", "什么", "哪", "怎么", "多少", "几", "为啥")  # question words


@dataclass(frozen=True)  # immutable result
class PtpParse:  # one parsed command
    """One parsed PTP command. kind is goto / delete / rename; name is the
    keypoint as said (goto / delete) or the OLD name (rename); new is the new
    name (rename only); when is a leading time (goto may carry one; for
    delete / rename it is reported so the caller can refuse it)."""
    kind: str  # goto / delete / rename
    name: str  # the keypoint (old name for rename)
    new: Optional[str] = None  # rename only
    when: Optional[TimeExpr] = None  # a leading time, if any
    loops: Optional[int] = None  # patrol only: the spoken lap count, None = not said


#: A lap count at the end of a patrol: "3圈", "两遍", "一趟".
_LOOPS = r"(?P<n>[0-9]+|[一二两三四五六七八九十]+)(?:圈|遍|趟)"  # digits or CJK numerals
#: Patrol, route first: "沿(着)X(这条路径)巡逻(3圈)". The optional "这条..."
#: group must NOT be a bare 路径: a route is usually NAMED "1号路径", and a
#: lazy name followed by an optional 路径 would cut it to "1号".
_PATROL_FRONT = re.compile(r"^(?:沿着|沿|按照|按|顺着|走)(?P<x>.+?)"
                           r"(?:这条路径|这条路线|这条线)?(?:去)?(?:巡逻|巡查|巡检)"
                           + _LOOPS.replace("(?P<n>", "(?:(?P<n>", 1) + r")?(?:吧|了)*$")  # laps optional
#: Patrol, verb first: "(开始)巡逻X(3圈)" and "去X巡逻(3圈)".
_PATROL_VERB = re.compile(r"^(?:开始)?(?:巡逻|巡查|巡检)(?P<x>.+?)"
                          + _LOOPS.replace("(?P<n>", "(?:(?P<n>", 1) + r")?(?:吧|了)*$")  # laps optional
_PATROL_GO = re.compile(r"^(?:去|到)(?P<x>.+?)(?:巡逻|巡查|巡检)"
                        + _LOOPS.replace("(?P<n>", "(?:(?P<n>", 1) + r")?(?:吧|了)*$")  # laps optional
#: A route object that is really just a lap count or a place word.
_LAPS_ONLY = re.compile("^" + _LOOPS + "$")  # "巡逻一圈": no route named
_CN_DIGIT = {"一": 1, "二": 2, "两": 2, "三": 3, "四": 4, "五": 5, "六": 6,
             "七": 7, "八": 8, "九": 9}  # CJK numerals for lap counts


def _laps(n: Optional[str]) -> Optional[int]:
    """The lap count as an int: "3" / "三" / "十二" -> 3 / 3 / 12; None when
    nothing was said. Only 1..99 is a count; anything else is None."""
    if not n:  # no lap count in the sentence
        return None  # the caller treats "not said" as one lap
    if n.isdigit():  # Arabic digits
        v = int(n)  # as said
    elif n == "十":  # ten alone
        v = 10  # 十
    elif n.startswith("十"):  # 十X = 10 + X
        v = 10 + _CN_DIGIT.get(n[1:], 0)  # 十二 -> 12
    elif n.endswith("十"):  # X十 = 10X
        v = 10 * _CN_DIGIT.get(n[:-1], 0)  # 二十 -> 20
    elif "十" in n:  # X十Y
        a, b = n.split("十", 1)  # tens and units
        v = 10 * _CN_DIGIT.get(a, 0) + _CN_DIGIT.get(b, 0)  # 二十三 -> 23
    else:  # a single CJK digit
        v = _CN_DIGIT.get(n, 0)  # 三 -> 3
    return v if 1 <= v <= 99 else None  # out of range: not a count


def _patrol(t: str, when: Optional[TimeExpr]) -> Optional[PtpParse]:
    """A named patrol in t, or None. The route name is whatever sits between
    the verb and 巡逻 / the lap count; the catalogue decides if it exists."""
    m = (_PATROL_FRONT.match(t) or _PATROL_GO.match(t)
         or _PATROL_VERB.match(t))  # any of the three word orders
    if not m:  # not a patrol sentence
        return None  # let the other rules look
    name = _trim(m.group("x"), ("吧", "了"))  # route without particles
    if (not name or _LAPS_ONLY.match(name) or name in _DEICTIC  # "巡逻一圈" / "去那边巡逻"
            or name.startswith(("这条", "那条"))  # "按这条路径巡逻": points, does not name
            or any(q in name for q in _QUESTION)):  # "巡逻什么"
        return None  # no route named: B02 asks which one
    return PtpParse("patrol", name, when=when, loops=_laps(m.group("n")))  # a named patrol


def _stages(text: str):
    """(time, rest) candidates: as said first, then with each leading
    courtesy word / time phrase removed in turn."""
    t = _PUNCT.sub("", text or "")  # no spaces or sentence marks
    when: Optional[TimeExpr] = None  # a time seen so far
    yield when, t  # the sentence as said
    for _ in range(6):  # a few stacked prefixes at most
        cut = next((w for w in _LEAD if t.startswith(w) and len(t) > len(w)),
                   None)  # a courtesy / urgency word
        if cut is not None:  # a lead word was found
            t = t[len(cut):]  # drop it
        else:  # else try a time phrase
            found, rest = split_time_prefix(t)  # a time phrase
            if found is None or when is not None or not rest:  # no time, a second time, or nothing left
                return  # nothing more to strip
            when, t = found, rest  # one time per command
        yield when, t  # try again with less in front


def _trim(name: str, tails) -> str:
    """name without trailing filler words, repeatedly."""
    changed = True  # loop until nothing changes
    while changed and name:  # stop on an empty name
        changed = False  # assume done
        for tail in tails:  # longest-first is not needed: each is a word
            if name.endswith(tail) and len(name) > len(tail):  # never strip the whole name
                name, changed = name[:-len(tail)], True  # drop one
    return name  # what remains


def _head(name: str) -> str:
    """name without a leading type word ("航点1号测试点" -> "1号测试点")."""
    for head in _TYPE_HEAD:  # each type word
        if name.startswith(head) and len(name) > len(head):  # keep a bare type word
            return name[len(head):]  # the bare name
    return name  # no type word


def _swallowed_lead(old: str) -> bool:
    """True when a rename's old name still holds the sentence's own lead
    ("现在把东门岗亭" from "现在把东门岗亭重命名为..."): 把 is optional in
    the pattern, so the unstripped stage can take the prefix as the name.
    The next stage, with the lead removed, then matches cleanly."""
    if "把" in old:  # a split sentence, not a name
        return True  # 把 never belongs to a keypoint name
    return any(old.startswith(w + "将") for w in _LEAD)  # "现在将..."


def _keypoint(name: str) -> bool:
    """True when name can be a keypoint object (not a route / fence / dock)."""
    return bool(name) and not any(w in name for w in _NOT_KEYPOINT)


def parse_ptp(text: str) -> Optional[PtpParse]:
    """The PTP command text is, or None when it is not one of the four.

    Rename is tried first (its sentence also contains an object), then
    delete, then a named patrol (U121), then goto. A time is kept for any kind; the caller decides that
    only a goto may carry one.
    """
    for when, t in _stages(text):  # least stripped first
        m = _RENAME.match(t)  # rename first: it also holds an object
        if m and not _swallowed_lead(m.group("old")):  # the lead was stripped
            old = _head(m.group("old"))  # old name without a type word
            new = _trim(m.group("new"), ("吧", "了"))  # new name without particles
            if old and new:  # equal names are refused by the caller, by reason
                return PtpParse("rename", old, new=new, when=when)
        m = _DEL_FRONT.match(t) or _DEL_BACK.match(t)  # either delete word order
        if m:  # a delete sentence
            name = _head(m.group("x"))  # the target
            if _keypoint(name):  # a keypoint, not a route / fence
                return PtpParse("delete", name, when=when)
            return None  # a route / fence delete: not ours
        pat = _patrol(t, when)  # a named patrol (U121), before goto: "去X巡逻"
        if pat is not None:  # the route is named
            return pat  # B02
        m = _GOTO.match(t)  # a goto sentence?
        if m:  # verb at the start
            name = _head(_trim(m.group("x"), _GOTO_TAIL))  # destination without filler or type word
            if (_keypoint(name) and name not in _DEICTIC  # named, not 这里 / 那边
                    and not any(q in name for q in _QUESTION)):
                return PtpParse("goto", name, when=when)  # a named place
            return None  # patrol / charge / "here" / a question: not ours
    return None  # not a PTP command
