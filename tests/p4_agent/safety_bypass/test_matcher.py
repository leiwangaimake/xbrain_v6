"""16 §4 safety-bypass matcher tests + non-symmetric-cost variants."""

import pytest

from xbrain.p4_agent.safety_bypass import matcher


pytestmark = pytest.mark.no_device


# --- Action closed set -------------------------------------------

def test_action_closed_set_has_exactly_three_values():
    """16 §4 table: three bypass actions. Adding a fourth requires
    a doc change (and a new dispatch path -- estop goes to Tier 1,
    prone/stand go via P1)."""
    assert matcher.BYPASS_ACTIONS == frozenset({"estop", "prone", "stand"})


def test_bypass_hit_rejects_out_of_set_action():
    with pytest.raises(ValueError):
        matcher.BypassHit(action="cancel", matched_token="x", source="raw")


def test_bypass_hit_rejects_out_of_set_source():
    with pytest.raises(ValueError):
        matcher.BypassHit(action="estop", matched_token="停", source="cooked")


# --- No match on empty / non-stop text --------------------------

def test_empty_text_returns_none():
    assert matcher.match_bypass("") is None


def test_non_stop_text_returns_none():
    assert matcher.match_bypass("你好机器人") is None
    assert matcher.match_bypass("今天天气不错") is None


# --- POSITIVE: estop verbs (V5 field-test canonical set) --------

@pytest.mark.parametrize("phrase", [
    "急停",
    "紧急停止",
    "立刻停下",
    "停下",
    "stop",
    "现在立刻停下来",     # V5 R8 field bug (was regex-anchored missing)
    "快停下我说了快",     # V5 R79 field bug
    "急停急停",           # V5 R68/69 phrase-fold case
])
def test_estop_variants_match(phrase):
    """* 16 §4.1: 漏 > 误. These are V5 field-test panic phrases;
    the anchored regex missed them all in V5 R8/R79. V6 CONTAINS
    matching MUST catch every one -- regression here would silently
    revive the missed-stop bug.

    NOTE: we do NOT pin the exact matched_token because several tokens
    may substring-match the same text; the safety guarantee is "any
    estop token matched", not "this specific token wins"."""
    hit = matcher.match_bypass(phrase)
    assert hit is not None, "phrase %r must match" % phrase
    assert hit.action == matcher.BYPASS_ESTOP
    assert hit.matched_token   # non-empty


# --- POSITIVE: prone + stand -----------------------------------

@pytest.mark.parametrize("phrase,expected", [
    ("趴下",        matcher.BYPASS_PRONE),
    ("赶紧趴下",    matcher.BYPASS_PRONE),
    ("卧倒",        matcher.BYPASS_PRONE),
    ("站立",        matcher.BYPASS_STAND),
    ("请站起来",    matcher.BYPASS_STAND),
    ("起立",        matcher.BYPASS_STAND),
])
def test_prone_stand_variants_match(phrase, expected):
    hit = matcher.match_bypass(phrase)
    assert hit is not None
    assert hit.action == expected


# --- Priority: estop wins over prone/stand on ambiguous text ----

def test_estop_wins_over_stand_when_both_present():
    """* 16 §4.1: safety-first. On the (unlikely) mixed phrase where
    an estop token AND a stand token both occur, estop must win --
    the failure direction is that a missed stop is unrecoverable."""
    hit = matcher.match_bypass("急停站立")
    assert hit.action == matcher.BYPASS_ESTOP


# --- Source-labeled wrappers -----------------------------------

def test_match_raw_labels_source_raw():
    hit = matcher.match_raw("急停")
    assert hit is not None
    assert hit.source == "raw"


def test_match_normalized_labels_source_normalized():
    hit = matcher.match_normalized("急停")
    assert hit is not None
    assert hit.source == "normalized"


def test_match_raw_none_on_no_match():
    assert matcher.match_raw("你好") is None


def test_match_normalized_none_on_no_match():
    assert matcher.match_normalized("你好") is None


# --- * 16 §4 变异体: 若 L1 词典把"急停"替换成"紧张"------------
# 只有 raw match 能救回来 -- normalized 也失败了.
# 这条测试证明 "双次匹配" 的价值: raw 单独存在就是护栏.

def test_double_match_catches_post_processing_corruption():
    """VARIANT: simulate the failure 16 §4 约束表 P.1 warns about --
    a bad L1 substitution that turned raw 急停 into a non-stop word.

    The pipeline MUST have called match_raw BEFORE post-processing;
    that raw call catches the estop. If only match_normalized was
    called, the estop would be lost silently.

    This test proves match_raw's independent utility -- without it,
    a single corrupted L1 dict entry would silently defeat safety."""
    raw = "急停"
    normalized_after_bad_l1 = "紧张"

    raw_hit = matcher.match_raw(raw)
    norm_hit = matcher.match_normalized(normalized_after_bad_l1)

    assert raw_hit is not None, "raw match MUST catch the estop"
    assert raw_hit.action == matcher.BYPASS_ESTOP
    assert norm_hit is None, "normalized would miss (this is the bug)"

    # In production the pipeline uses raw OR normalized -- either
    # match fires. Simulate that logic:
    winner = raw_hit or norm_hit
    assert winner is not None
    assert winner.action == matcher.BYPASS_ESTOP


# --- * Purity: same input -> same output --------------------------

def test_matcher_is_pure_function():
    """Called twice, must return equal results. Any state that
    accumulated between calls would be caught here."""
    a = matcher.match_bypass("急停")
    b = matcher.match_bypass("急停")
    assert a == b


# --- Regression: wake-prefix tolerance ---------------------------

def test_wake_prefix_tolerated_around_estop():
    """V5 field showed operators prefix estop with fillers:
    '嗯,先停下' / '哎急停' / '机器人,停下来'.

    Substring matching handles these naturally without needing the
    complex V5 _WAKE_PREFIX regex."""
    for text in ["嗯,先停下", "哎急停", "机器人,停下来", "泰莎,立刻停"]:
        hit = matcher.match_bypass(text)
        assert hit is not None, "wake-prefix form %r must still match" % text
        assert hit.action == matcher.BYPASS_ESTOP
