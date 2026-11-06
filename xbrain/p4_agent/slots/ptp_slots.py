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
          "沿 X 来回巡逻 N次", "在 X 上来回巡 N趟", "X 来回巡逻 N次"
                                     -> B02 {route: X, loops: N, round_trip}
  repeat  "再巡一圈 / 再巡逻两圈 / 再来一圈 / 再来回巡一次", "来回巡三次"
                                     -> B04 {loops: N, round_trip}
          (99 U131: laps are counted by P3, one push per pass; on an open
          route a "来回" is two passes, A -> B -> A. B04 names no route:
          P3 repeats the latest completed patrol.)
  query   "X路径上有哪些航点 / 关键点 / 位置"               -> G48 {route: X}
          "X在哪条路径上 / X属于哪条路径"                    -> G49 {waypoint: X}
          (99 U130: answered from state/geo/objects with the route-membership
          rule P3 navigates by; a named object, so the keyword layer -- which
          carries no slot -- cannot answer them.)
  query   "离我最近的航点" / "去X多远" / "X路径多长"     -> G50 / G51 {waypoint} / G52 {route}
          (99 U136: answered from state/geo/objects + state/pose; G51 is the
          length of the path a goto would take, the 99 U127 plan)
  delete  "删除 1号路径" / "删除 东门围栏"          -> F11 {route: X} / F13 {fence: X}
  enable  "启用 东门围栏" / "换成 2号围栏"           -> F15 {fence: X}
          (99 U138: by name; still L2-confirmed -- enabling changes where the
          robot may go)
          (99 U133: by name, like U110's keypoints; still L2-confirmed)
  dock    "[前缀] 去 / 前往 / 导航到 1号充电站 / 东门充电桩"   -> B09 {dock: X}
          (99 U128: a NAMED charging point -- that point, however far. A bare
          "去充电站 / 回充电桩" names nothing and is left to B09's keyword leg,
          where P3 picks the nearest dock.)

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

from xbrain.common.geo import DOCK_NAME_KEYWORDS, is_dock_name  # 99 U119 / U128 role names
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


#: 99 U143 (user ruling 2026-11-06 "支持定时"): what is left of a TIMED
#: sentence after its time phrase that means "return home" (B08) or "go and
#: charge at the nearest dock" (B09 with no name). Whole-rest matches only:
#: a named dock ("去1号充电站") is the dock rule's, and an untimed "返航" stays
#: on the B08 / B09 keyword path exactly as before.
TIMED_RETURN = ("返航", "回来", "回到起点", "返回起点", "回去")
TIMED_CHARGE = ("充电", "去充电", "回去充电", "回充电", "回桩", "回充电桩",
                "去充电桩", "去充电站", "回充电站")


@dataclass(frozen=True)  # immutable result
class PtpParse:  # one parsed command
    """One parsed PTP command. kind is goto / delete / rename; name is the
    keypoint as said (goto / delete) or the OLD name (rename); new is the new
    name (rename only); when is a leading time (goto may carry one; for
    delete / rename it is reported so the caller can refuse it)."""
    kind: str  # goto / delete / delete_route / delete_fence / activate_fence / rename / patrol / repeat / dock / route_points / point_routes / nearest_point / distance_to / route_length / return_home / charge
    name: str  # the keypoint (old name for rename)
    new: Optional[str] = None  # rename only
    when: Optional[TimeExpr] = None  # a leading time, if any
    loops: Optional[int] = None  # patrol / repeat: the spoken lap count, None = not said
    round_trip: bool = False  # patrol / repeat: a "来回" sentence (U131)


#: G48: "X(路径)上有哪些航点" -- X must end like a route name (it is what the
#: question is about); 上 is optional ("1号路径有哪些航点").
_Q_ROUTE_POINTS = re.compile(r"^(?:查询|查一下|请问|告诉我)?(?P<x>.+?(?:路径|路线|线路|环线|线))"
                             r"上?都?有(?:哪些|什么|几个)(?:航点|关键点|位置点|位置|点)(?:吗|呢|啊)?$")  # route -> points
#: G49: "X在哪条路径上" / "X属于哪条路径".
_Q_POINT_ROUTES = re.compile(r"^(?:查询|查一下|请问|告诉我)?(?P<x>.+?)(?:在|属于)(?:哪条|哪个|什么)"
                             r"(?:路径|路线|线路|线)上?(?:吗|呢|啊)?$")  # point -> routes

#: 99 U138 F15: "启用东门围栏" / "换成2号围栏" -- by name. Until 2026-11-05 only
#: the literal registry example ("启用营区外围围栏") was understood, while the
#: U133 fence-saved answer told the operator to say "启用X".
_ACTIVATE_FENCE = re.compile(r"^(?:启用|激活|换成|切换到|改用|使用)(?P<x>.+?围栏)(?:吧|了)*$")  # F15 by name

#: 99 U136 G50: "离我最近的航点是哪个" -- no name, the robot's position.
_Q_NEAREST = re.compile(r"^(?:查询|查一下|请问|告诉我)?(?:离我|离你|离机器人|距离我)?最近的"
                        r"(?:航点|点位|关键点|位置点|位置|点)(?:是哪个|是哪里|是什么|在哪里|在哪)?"
                        r"(?:呢|啊)?$")  # nearest keypoint
#: 99 U136 G51: "去X多远" / "到X有多远" / "离X还有多远" / "去X要多久".
_Q_DISTANCE = re.compile(r"^(?:查询|查一下|请问|告诉我)?(?:去|到|离|距离)(?P<x>.+?)(?:还)?(?:有)?"
                         r"(?:多远|多少米|要多久|需要多久|要多长时间|多长时间)(?:呢|啊)?$")  # how far to X
#: 99 U136 G52: "X路径多长" -- X must end like a route name.
_Q_LENGTH = re.compile(r"^(?:查询|查一下|请问|告诉我)?(?P<x>.+?(?:路径|路线|线路|环线|线))"
                       r"(?:有)?(?:多长|多少米|长度是多少|全长多少)(?:呢|啊)?$")  # route length

#: A lap count at the end of a patrol: "3圈", "两遍", "一趟".
_LOOPS = r"(?P<n>[0-9]+|[一二两三四五六七八九十]+)(?:圈|遍|趟|次)"  # digits or CJK numerals
#: The patrol verb, plain or as a round trip (U131): "巡逻" / "来回巡逻" /
#: "来回巡" / "来回走". The bare 巡 and 走 only count after 来回 -- alone they
#: are other words (巡航, 走路).
_PATROL_WORD = r"(?:(?P<rt>来回)(?:巡逻|巡查|巡检|巡|走)|巡逻|巡查|巡检)"  # rt set on 来回
#: Optional lap count after the verb, and trailing particles.
_LAPS_OPT = _LOOPS.replace("(?P<n>", "(?:(?P<n>", 1) + r")?(?:吧|了)*$"  # laps optional
#: Patrol, route first with no verb in front: "X(上)来回巡逻3次". WEAK (X must
#: end like a route name) and round trip only -- "X巡逻" alone is too open.
_PATROL_RT_BARE = re.compile(r"^(?P<x>.+?)上?(?P<rt>来回)(?:巡逻|巡查|巡检|巡|走)"
                             + _LAPS_OPT)  # laps optional
#: B04 repeat (U131), no route named: "再巡(逻)(一圈)", "再来回巡一次". The
#: 再 / 又 is required -- it is what says "the same again".
_REPEAT_XUN = re.compile(r"^(?:再|又)(?P<rt>来回)?(?:巡逻|巡查|巡检|巡)" + _LAPS_OPT)  # laps optional
#: "再来一圈" / "再走两圈": 来 and 走 are too common to stand alone, so only
#: with a lap count in 圈 (再来一次 repeats anything, 再走一趟 goes somewhere).
_REPEAT_LAI = re.compile(r"^(?:再|又)(?P<rt>来回)?(?:来|走)"
                         r"(?P<n>[0-9]+|[一二两三四五六七八九十]+)圈(?:吧|了)*$")  # count required
#: "来回巡三次" with no route and no 再: a round trip on the route just
#: patrolled. The count is required (bare "来回巡逻" asks which route).
_REPEAT_RT = re.compile(r"^(?P<rt>来回)(?:巡逻|巡查|巡检|巡)" + _LOOPS + r"(?:吧|了)*$")  # count required
#: Patrol, route first: "沿(着)X(这条路径)巡逻(3圈)". The optional "这条..."
#: group must NOT be a bare 路径: a route is usually NAMED "1号路径", and a
#: lazy name followed by an optional 路径 would cut it to "1号".
_PATROL_FRONT = re.compile(r"^(?:沿着|沿|顺着)(?P<x>.+?)"
                           r"(?:这条路径|这条路线|这条线)?上?(?:去)?" + _PATROL_WORD
                           + _LOOPS.replace("(?P<n>", "(?:(?P<n>", 1) + r")?(?:吧|了)*$")  # laps optional
#: Patrol, "按 / 按照 / 走 X 巡逻": WEAK -- 按时巡逻 / 按照计划巡逻 / 走路巡逻
#: are not routes, so these forms only count when X names a route (_ROUTE_TAIL).
_PATROL_FRONT_WEAK = re.compile(r"^(?:按照|按|走|在)(?P<x>.+?)"
                                r"(?:这条路径|这条路线|这条线)?上?(?:去)?" + _PATROL_WORD
                                + _LOOPS.replace("(?P<n>", "(?:(?P<n>", 1) + r")?(?:吧|了)*$")  # laps optional
#: Patrol, verb first: "(开始)巡逻X(3圈)" -- WEAK, see _ROUTE_TAIL -- and the
#: strong "去X巡逻(3圈)".
_PATROL_VERB = re.compile(r"^(?:开始)?(?P<rt>来回)?(?:巡逻|巡查|巡检)(?P<x>.+?)"
                          + _LOOPS.replace("(?P<n>", "(?:(?P<n>", 1) + r")?(?:吧|了)*$")  # laps optional
_PATROL_GO = re.compile(r"^(?:去|到)(?P<x>.+?)" + _PATROL_WORD
                        + _LOOPS.replace("(?P<n>", "(?:(?P<n>", 1) + r")?(?:吧|了)*$")  # laps optional
#: The weak forms ("巡逻X", "按X巡逻", "走X巡逻") claim a sentence only when X
#: ends like a route name. Measured 2026-11-05 (audit 1): without it the rule
#: took 巡逻模式 / 巡逻状态 / 巡逻一下 / 巡逻吧 / 巡逻慢一点 / 按时巡逻 /
#: 走路巡逻 / 巡检一下设备 and answered "没找到模式" instead of letting the
#: keyword layer (B02 asks which route, A-class speed, G-class status) run.
_ROUTE_TAIL = ("路径", "路线", "线路", "环线", "线")  # how route names end
#: 99 U133 delete by name: the full-name tails (a bare 线 is too loose for a
#: delete) and the fence tail. "删除1号路径" -> F11, "删除东门围栏" -> F13.
_ROUTE_TAIL_FULL = ("路径", "路线", "线路", "环线")  # full route names
_FENCE_TAIL = "围栏"  # a fence name
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
    m = _PATROL_FRONT.match(t) or _PATROL_GO.match(t)  # the strong forms
    weak = m is None  # only a weak form left to try
    if weak:  # 巡逻X / 按X巡逻 / 走X巡逻 / X来回巡逻
        m = (_PATROL_FRONT_WEAK.match(t) or _PATROL_VERB.match(t)
             or _PATROL_RT_BARE.match(t))  # weak forms
    if not m:  # not a patrol sentence
        return None  # let the other rules look
    name = _trim(m.group("x"), ("吧", "了"))  # route without particles
    if weak and not name.endswith(_ROUTE_TAIL):  # "巡逻模式", "按时巡逻": not a route
        return None  # leave it to the keyword layer
    if (not name or _LAPS_ONLY.match(name) or name in _DEICTIC  # "巡逻一圈" / "去那边巡逻"
            or name.startswith(("这条", "那条"))  # "按这条路径巡逻": points, does not name
            or any(q in name for q in _QUESTION)):  # "巡逻什么"
        return None  # no route named: B02 asks which one
    return PtpParse("patrol", name, when=when, loops=_laps(m.group("n")),  # a named patrol
                    round_trip=bool(m.group("rt")))  # "来回": two passes per lap on a line (U131)


def _repeat(t: str, when: Optional[TimeExpr]) -> Optional[PtpParse]:
    """B04 "再巡一圈" / "来回巡三次" (U131): no route named, the latest
    completed patrol again. None when t is not one of those sentences or
    its count is not a count (out of 1..99)."""
    m = _REPEAT_XUN.match(t) or _REPEAT_LAI.match(t) or _REPEAT_RT.match(t)  # 再巡 / 再来N圈 / 来回巡N次
    if not m:  # not a repeat
        return None  # the other rules look
    n = m.group("n")  # the spoken count, if any
    loops = _laps(n)  # as an int
    if n and loops is None:  # "再巡一百圈": said but not a count
        return None  # not ours; the keyword layer answers
    return PtpParse("repeat", "", when=when, loops=loops,  # B04
                    round_trip=bool(m.group("rt")))  # "来回"


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
    """The PTP command text is, or None when it is not one of the seven.

    Rename is tried first (its sentence also contains an object), then
    delete, then a named patrol (U121), then goto. A time is kept for any kind; the caller decides that
    only a goto may carry one.
    """
    for when, t in _stages(text):  # least stripped first
        if when is not None and t in TIMED_RETURN:  # 99 U143: "下午3点返航"
            return PtpParse("return_home", "", when=when)  # B08, scheduled
        if when is not None and t in TIMED_CHARGE:  # 99 U143: "10分钟后去充电"
            return PtpParse("charge", "", when=when)  # B09 nearest dock, scheduled
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
            if name.endswith(_FENCE_TAIL) and len(name) > len(_FENCE_TAIL):  # "东门围栏"
                return PtpParse("delete_fence", name, when=when)  # 99 U133: F13 by name
            if (name.endswith(_ROUTE_TAIL_FULL)
                    and name not in _ROUTE_TAIL_FULL):  # "1号路径", not bare "路径"
                return PtpParse("delete_route", name, when=when)  # 99 U133: F11 by name
            return None  # anything else: not ours
        m = _Q_ROUTE_POINTS.match(t)  # G48 (U130)
        if m and when is None:  # a question carries no time
            return PtpParse("route_points", m.group("x"))  # which points are on route X
        m = _Q_POINT_ROUTES.match(t)  # G49 (U130)
        if m and when is None and m.group("x") not in _DEICTIC:  # "这里在哪条路径上" is a different question
            return PtpParse("point_routes", _head(m.group("x")))  # which routes is point X on
        rpt = _repeat(t, when)  # B04 (U131): "再巡一圈", before the named forms
        if rpt is not None:  # no route named on purpose
            return rpt  # B04
        m = _ACTIVATE_FENCE.match(t)  # F15 (U138)
        if m:  # "启用东门围栏"; a bare "启用围栏" cannot match (.+? needs a name first)
            return PtpParse("activate_fence", m.group("x"), when=when)  # F15 by name
        if when is None and _Q_NEAREST.match(t):  # G50 (U136)
            return PtpParse("nearest_point", "")  # the robot's nearest keypoint
        m = _Q_DISTANCE.match(t)  # G51 (U136), before goto: "去X多远" is not a goto
        if m and when is None and m.group("x") not in _DEICTIC:  # a named place
            return PtpParse("distance_to", _head(m.group("x")))  # how far to X
        m = _Q_LENGTH.match(t)  # G52 (U136)
        if m and when is None:  # a question carries no time
            return PtpParse("route_length", m.group("x"))  # how long is route X
        pat = _patrol(t, when)  # a named patrol (U121), before goto: "去X巡逻"
        if pat is not None:  # the route is named
            return pat  # B02
        m = _GOTO.match(t)  # a goto sentence?
        if m:  # verb at the start
            name = _head(_trim(m.group("x"), _GOTO_TAIL))  # destination without filler or type word
            if is_dock_name(name) and name not in DOCK_NAME_KEYWORDS:  # "1号充电站", not bare "充电站"
                return PtpParse("dock", name, when=when)  # U128: B09 with a name
            if (_keypoint(name) and name not in _DEICTIC  # named, not 这里 / 那边
                    and not any(q in name for q in _QUESTION)):
                return PtpParse("goto", name, when=when)  # a named place
            return None  # patrol / charge / "here" / a question: not ours
    return None  # not a PTP command
