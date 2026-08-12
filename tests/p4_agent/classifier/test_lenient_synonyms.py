"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_lenient_synonyms.py
Brief: Guards the 2026-08-12 lenient-synonym expansion of thin intents

Description:
The 圆润 batch (18 S2.2) widened five keyword-thin intents that had NO
tier-2 mission fallback -- C08 query_mode_switch_ok, G09 query_uptime,
H07 reboot, H08 shutdown, F13 delete_fence -- from a single trigger word
to a natural synonym set. Without a mission to catch a keyword miss, a
dropped synonym silently makes those utterances fall to layer-6 decline;
the failure is invisible unless a test speaks the synonym.

Each positive criterion here is mutation-guarded per CLAUDE.md 3.3: the
asserted synonym is a substring of exactly ONE intent's keywords, so
removing that keyword from intents.yaml makes longest_match return None
(or a shorter neighbour) and the assertion turns red. The negative
criteria guard PRECISION -- the new short keywords (关机 / 重启 / 能切到)
must not substring-steal a neighbour: an imperative mode command must
stay a command (C03), 关灯 must stay light-off (D02), 重新加载 must stay
reload (H04). These are the collisions the expansion could have caused
and did not (verified offline before commit).

Why this file and not test_layer_match: that file guards the matcher
MECHANISM (longest-first, bypass exclusion); this file guards a DATA
expansion. Keeping them apart means a future keyword edit breaks the
data guard here, not the mechanism guard there -- the red test names the
real cause.
"""
from __future__ import annotations

import yaml

from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher
from xbrain.p4_agent.registry.intents import load_intent_registry

_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"


def _matcher() -> KeywordMatcher:
    return KeywordMatcher(
        load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8"))))


# -- positive: every widened synonym resolves to its intent -----------
# Each phrase's decisive substring is a keyword of exactly ONE intent;
# delete that keyword -> longest_match misses -> this table turns red.
_SYNONYMS = [
    # C08 query_mode_switch_ok -- modal prefix (能/可以/能不能) marks the
    # feasibility QUERY apart from the imperative mode command.
    ("现在能切换到巡逻吗", "C08"),
    ("可以切到驱离模式吗", "C08"),
    ("能不能切到充电", "C08"),
    # G09 query_uptime.
    ("运行多久了", "G09"),
    ("开机多长时间了", "G09"),
    ("上电多久了", "G09"),
    ("启动多久了", "G09"),
    # H07 reboot (still L2 -- widening说法 does not touch the auth gate).
    ("重启", "H07"),
    ("重新启动", "H07"),
    ("重启机器", "H07"),
    # H08 shutdown (still L3).
    ("关闭系统", "H08"),
    ("系统关机", "H08"),
    ("把系统关了", "H08"),
    # F13 delete_fence.
    ("删掉围栏", "F13"),
    ("移除围栏", "F13"),
]


def test_widened_synonyms_resolve_to_their_intent():
    """Every 2026-08-12 synonym resolves to its intent (18 S2.2 A/B).
    MUTATION: remove any one keyword from intents.yaml and the matching
    row here returns None/neighbour -> red (CLAUDE.md 3.3)."""
    m = _matcher()
    for phrase, expected in _SYNONYMS:
        got = m.longest_match(phrase)
        assert got == expected, (
            "lenient synonym %r resolved to %r, expected %r"
            % (phrase, got, expected))


# -- negative: the short new keywords must not steal a neighbour ------

def test_imperative_mode_command_not_stolen_by_query():
    """A bare imperative '切换到喊话模式' is the set_mode COMMAND (C-class),
    NOT the C08 feasibility query. The query keywords all carry a modal
    prefix (能/可以/能不能), so the prefix-less command misses them.
    MUTATION: had C08 been given a bare '切到' keyword, this command would
    resolve to C08 and this assertion would turn red."""
    m = _matcher()
    got = m.longest_match("切换到喊话模式")
    assert got != "C08", "imperative mode command mis-stolen by C08 query"


def test_shutdown_reboot_do_not_steal_light_or_reload():
    """The new short H07/H08 keywords (重启 / 关闭系统 / 系统关机) must not
    substring-steal payload light-off (D02) or reload_config (H04).
    MUTATION: a bare '关' or '重' keyword on H07/H08 would flip these and
    turn the assertions red."""
    m = _matcher()
    # 关灯 stays light-off, not shutdown.
    assert m.longest_match("关灯") == "D02"
    assert m.longest_match("关闭照明灯") == "D02"
    # 重新加载配置 stays reload, not reboot (shares only the 重 / 重新 stem).
    assert m.longest_match("重新加载配置") == "H04"
