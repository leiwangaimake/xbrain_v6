"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: name_slots.py
Brief: the operator-given `name` slot -- one spelling for numbered names, grounding, 16 S8.0.4 length

Description:
F03 / F06 / F09 / F10 carry a `name` the operator makes up ("1号测试点"), and
B01 / F11-F15 later look an object up by what is said. Three defects met here
(99 U109, 2026-11-04 audit):

  * Two spellings of one name. The recogniser writes "一号测试点", a typed
    command writes "1号测试点"; stored verbatim they were two different
    keypoints, and "去1号测试点" did not find the one recorded by voice.
    canonical_name() rewrites a Chinese numeral directly before 号 into
    Arabic digits, and both the recording side and the lookup side
    (geo_request.resolve_geo_id) apply it. It uses motion_slots.parse_number,
    the one numeral reader in P4, so there is still a single answer for "三".
  * Invented names. The mission prompts say "逐字照抄" (copy verbatim), but
    nothing checked it; measured on the 3B model, "标记一下这里" came back
    with the name "这里". ground_name() keeps a name only if it occurs in what
    was said, the same rule motion_slots.ground_llm_slots applies to numbers
    (16 EV-2: a missing slot is asked for, never filled in). Occurring is not
    enough on its own: for "记录当前位置" the model answered "当前位置",
    which WAS said -- as the place, not as a name. A name made only of the
    words that point at "here" is therefore no name.
  * Length. 16 S8.0.4 bounds `name` to 2-12 characters and says out-of-range
    is REFUSED, not truncated; name_length_ok() is that check, used by the
    teach builder so the refusal is spoken.

Boundary: only the numeral before 号 is rewritten. "一楼大厅" or "第一哨位"
keep their characters -- they are words, not numbers in the 11 S7.8.2 "3号"
sense, and a wider rewrite would change names the operator chose.
"""
from __future__ import annotations

import re  # the numeral-before-号 scanner
from typing import Optional  # plain strings in and out

from xbrain.p4_agent.slots.motion_slots import parse_number  # the one numeral reader

#: 16 S8.0.4 `name` / `new`: 2-12 characters, refused (not cut) outside it.
NAME_MIN_CHARS = 2  # 16 S8.0.4 lower bound
NAME_MAX_CHARS = 12  # 16 S8.0.4 upper bound

#: A Chinese integer numeral immediately followed by 号 ("十二号" -> "12号").
_CN_NUMBERED = re.compile(r"([零一二两三四五六七八九十百千]+)(?=号)")  # lookahead keeps 号

#: Words that point at "here" rather than name it ("把这里记为 X" names X).
#: A model answer that is one of these copied the command, not a name.
_PLACE_WORDS = frozenset({"这里", "这儿", "此处", "此地", "当前位置", "当前点",
                          "这个点", "这个位置", "位置", "点"})  # deictic, not names

#: Characters that never belong to a name: whitespace and the ASCII / CJK
#: punctuation an ASR or a typed command puts around it.
_STRIP = re.compile("[\\s,.;:!?\u3000-\u303f\uff01-\uff0f\uff1a-\uff20]")  # spaces + punctuation


def canonical_name(name: str) -> str:
    """The spelling a name is stored and looked up under.

    Punctuation and spaces are dropped, and a Chinese integer before 号 becomes
    digits ("一号测试点" -> "1号测试点"). A numeral parse_number refuses is
    left as written rather than guessed.
    """
    text = _STRIP.sub("", name or "")  # no spaces or punctuation inside a name

    def _digits(m: "re.Match[str]") -> str:  # one numeral run
        value = parse_number(m.group(1))  # the shared reader
        if value is None or value != int(value):  # malformed: keep it
            return m.group(1)  # left as written
        return str(int(value))  # "十二" -> "12"

    return _CN_NUMBERED.sub(_digits, text)  # every run before 号


#: Every Chinese integer numeral run, wherever it stands (name_key only).
_CN_ANY = re.compile(r"([零一二两三四五六七八九十百千]+)")


def name_key(name: str) -> str:
    """The key two names are COMPARED under (99 U177, user ruling 2026-11-12:
    "全字段匹配, 中文数字和阿拉伯数字等效").

    canonical_name, then every remaining Chinese numeral run as digits too, so
    "四号路径一点" and "4号路径1点" compare equal. Used for matching only, on
    BOTH sides, and never stored: canonical_name stays the stored spelling and
    keeps its narrower rule (only before 号) so a name such as "一楼大厅" is
    saved as the operator said it. Symmetric conversion is why the wider rule
    is safe here: "统一" becomes "统1" on both sides and still matches itself.
    """
    def _digits(m: "re.Match[str]") -> str:  # one numeral run
        value = parse_number(m.group(1))  # the shared reader
        if value is None or value != int(value):  # malformed: keep it
            return m.group(1)  # left as written
        return str(int(value))  # "十二" -> "12"

    return _CN_ANY.sub(_digits, canonical_name(name))  # all runs, after canonical


def ground_name(name: object, text: str) -> Optional[str]:
    """The canonical name when it was actually said, else None.

    Compared in canonical form on both sides, so "一号测试点" in the speech
    grounds a model answer of "1号测试点" and the reverse.
    """
    if not isinstance(name, str):  # a number or null from the model
        return None  # not a name
    canon = canonical_name(name)  # the stored spelling
    if not canon or canon in _PLACE_WORDS:  # empty, or "here" itself
        return None  # nothing was named
    return canon if canon in canonical_name(text) else None  # said, or invented


def name_length_ok(name: str) -> bool:
    """16 S8.0.4: 2-12 characters inclusive."""
    return NAME_MIN_CHARS <= len(name) <= NAME_MAX_CHARS  # inclusive bounds
