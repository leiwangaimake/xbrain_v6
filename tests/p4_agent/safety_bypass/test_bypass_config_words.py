"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_bypass_config_words.py
Brief: the safety bypass matches the configured word list (bare 停, 趴着), not only its built-in table

Description:
What went wrong (found 2026-10-25 on .1.26). 16 S4 and 18 A01 put the bare
"停" in the estop list, and "趴着" in the prone list; the user's 2026-09-21
ruling took the union of the two books' lists and placed the ONE copy in
configs/common.yaml common.audio.bypass_keywords (10 S5.4.5: "词表只此一份").
Two other copies existed and neither matched it: configs/p4_agent.yaml still
held the pre-ruling five words, and the matcher used neither -- its own
hard-coded table has no bare 停 and no 趴着. Measured: match_bypass("停") ->
None. An operator shouting "停!" was answered by the classifier, not by the
estop -- the miss 16 S4.1 ranks above every other error.

The fix. p4_agent.yaml's bypass.* now REFERENCE the common list (whole-node
${common.audio.bypass_keywords.*}, expanded by the freeze line); p4 startup
reads the resolved bypass.* and builds the matcher's groups with
tokens_from_config, which UNIONS them with the built-in V5 field harvest (so a
config edit can add a stop word but never remove 急刹车 or 停车); a missing or
empty group refuses startup.

What these cases pin:
  * with the configured words, "停" / "停!" / "停停停" fire the estop and
    "趴着" fires prone -- read from the real common.yaml, not a copy;
  * the built-in harvest survives the union (急刹车 / 紧急 / 停车 are not in
    the configured list);
  * an empty, missing or non-string group is refused;
  * the orchestrator uses the tokens it is given: "停" through a real turn is
    a bypass; and the PTZ stop phrases still yield (E05 / E08, 88eb9a0) even
    though the bare 停 is now a token inside them;
  * the token order is deterministic (the fired token is audited).

Mutants this file kills (run by hand, see the batch commit):
  * config groups ignored (built-in only)        -> bare-stop cases
  * config only, no union with the harvest      -> harvest case
  * empty-group check removed                   -> refusal case
  * orchestrator does not pass its tokens on    -> orchestrator case
  * tokens passed to only one match point        -> both-match-points case
  * ties broken by set order                    -> determinism case

Boundaries: the freeze-line expansion of the reference is exercised by
running scripts/dev/materialize_resolved.py (the resolved p4_agent.yaml then
holds 停 and 趴着), not here; this file reads the SOURCE list directly so it
does not depend on whichever snapshot happens to be on disk. The p4 __main__
exit-7 refusal path has no test of its own (it is two lines around
tokens_from_config, whose refusal is tested here).
"""
from __future__ import annotations

import pytest
import yaml

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,
    decision_to_publishes,
)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession
from xbrain.p4_agent.safety_bypass.matcher import (
    BYPASS_ESTOP,
    BYPASS_PRONE,
    match_bypass,
    tokens_from_config,
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# Pure unit cases: no zenoh, no ASR, no LLM, no hardware.
pytestmark = pytest.mark.no_device

# The single source of the list (10 S5.4.5), read as the freeze line reads it.
_COMMON = "/opt/xbrain_v6/configs/common.yaml"
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"


def _configured():
    """The matcher groups p4 builds at startup, from the real common.yaml.

    common.yaml nests the key as common -> audio -> bypass_keywords; the
    resolved p4_agent.yaml carries the same three lists under bypass.* once
    the freeze line has expanded the references.
    """
    words = yaml.safe_load(open(_COMMON, encoding="utf-8"))[
        "common"]["audio"]["bypass_keywords"]
    return tokens_from_config(words["estop"], words["prone"], words["stand"])


# --- the words the configured list adds ------------------------------------
# Each case passes the configured groups explicitly; the built-in-only answer
# for the same text is None, which is the defect being fixed.

@pytest.mark.parametrize("text", [
    "停",          # the bare word 16 S4 / 18 A01 list
    "停!",         # with the shout punctuation ASR sometimes keeps
    "停停停",      # repeated, as people shout it
])
def test_a_bare_stop_fires_the_estop_with_the_configured_words(text):
    """The case that was missed: a lone 停 must be an estop.

    The built-in table alone answers None here (measured 2026-10-25); the
    configured list is what makes it fire. A matcher that ignored the config
    would fail every row.
    """
    # No yield phrases: the bare-word behaviour alone.
    hit = match_bypass(text, (), _configured())
    # An estop, not prone / stand, and not None.
    assert hit is not None and hit.action == BYPASS_ESTOP, (text, hit)


def test_the_configured_prone_word_fires_prone():
    """趴着 is in the configured prone list (18 A02), not in the built-in.

    Prone is not the estop: it goes through P1 arbitration (16 S4 table), so
    a missed 趴着 is recoverable. It is still the configured list, and a
    matcher that read only the estop group would fail here.
    """
    hit = match_bypass("趴着", (), _configured())
    # The prone action specifically: estop would also be a wrong answer.
    assert hit is not None and hit.action == BYPASS_PRONE, hit


# --- the union keeps the built-in harvest ------------------------------------

# Three words the configured estop list does not have; each is in the
# matcher's built-in V5 field harvest.
@pytest.mark.parametrize("text", ["急刹车", "紧急", "停车"])
def test_the_built_in_harvest_survives_the_union(text):
    """Words only the built-in table has still fire with the configured groups.

    The configured estop list has six words; the V5 harvest has more. Using
    the configured list ALONE would have fixed 停 and silently dropped these
    -- trading one missed stop for others. 停车 also contains the bare 停, so
    it is asserted on the token that fires: the longest one wins, so the
    audit log names 停车, not 停.
    """
    hit = match_bypass(text, (), _configured())
    # Still an estop with the configured groups in place.
    assert hit is not None and hit.action == BYPASS_ESTOP, (text, hit)
    # Longest-first: the harvest word itself is what fired.
    assert hit.matched_token == text, hit


# --- refusal ------------------------------------------------------------------

# Empty list, missing (None), a blank word among real ones, a non-string word.
@pytest.mark.parametrize("estop", [[], None, ["急停", ""], ["急停", 3]])
def test_an_empty_or_malformed_group_is_refused(estop):
    """A bypass with no stop words is the fail-silent form of no bypass.

    p4 exits 7 on this (CLAUDE.md 3.1: no default for a safety value); the
    case pins the check that drives that exit. A blank or non-string word is
    refused too: a blank would match every utterance once stripped, a number
    would raise deep inside the substring test instead of naming the key.
    """
    # prone / stand are valid, so the only possible reason is the estop group,
    # and the message must name it (the operator fixes that key, nothing else).
    # The error text is "bypass.estop must be ...", which p4 logs before exit 7.
    with pytest.raises(ValueError, match="bypass.estop"):
        tokens_from_config(estop, ["趴下"], ["站立"])


# --- through the orchestrator ---------------------------------------------------

def _orch(tokens):
    """A real orchestrator built the way main_wiring builds it, with tokens.

    No LLM: these turns are decided by the bypass or the keyword layer.
    """
    reg = load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))
    cc = ChitchatResponder(yaml.safe_load(open(_CHITCHAT, encoding="utf-8")))
    return build_orchestrator(reg, cc, l2_timeout_ms=8000,
                              tier2_fn=lambda *_a: None, bypass_tokens=tokens)


def test_the_orchestrator_stops_on_a_bare_stop():
    """End to end: "停" is a bypass turn that publishes cmd/estop.

    This is the wiring half: the matcher can be right and the orchestrator
    still call it without the tokens, which is exactly how the configured
    list went unused before.
    """
    # A fresh session: not recording, so U45 does not suppress the estop.
    d = _orch(_configured()).handle_turn("停", OrchestratorSession(), 1_000)
    # Decided by the bypass, before any classification.
    assert d.kind == "bypass" and d.bypass_action == BYPASS_ESTOP, d.kind
    # And the Tier 1 frame is actually among the publishes.
    assert "cmd/estop" in [k for k, _ in decision_to_publishes(d)]


@pytest.mark.parametrize("text, raw", [
    ("调到最暗", "停"),   # the stop is only in the RAW transcript
    ("停", "听"),         # the stop is only in the CORRECTED text
])
def test_both_match_points_use_the_configured_words(text, raw):
    """16 S4 constraint 1 matches twice -- raw, then corrected -- and BOTH
    must use the configured groups.

    With raw == text (the case above) either match point alone catches the
    bare 停, so a wiring that passed the tokens to only one of them would
    pass it. Splitting the stop onto one side at a time makes each match
    point carry the case on its own.
    """
    # text is what correction produced, raw what ASR returned; the two
    # differ on purpose (see the docstring), and only one of them holds 停.
    # "听" / "调到最暗" hold no stop word under either token table.
    d = _orch(_configured()).handle_turn(text, OrchestratorSession(), 1_000,
                                         raw_text=raw)
    # Whichever side carries the stop, the turn is the estop bypass.
    assert d.kind == "bypass" and d.bypass_action == BYPASS_ESTOP, (text, raw, d.kind)


@pytest.mark.parametrize("text, intent_id", [
    ("停止扫描", "E08"), ("停止环视", "E08"), ("停止跟踪", "E05")])
def test_ptz_stops_still_yield_with_the_bare_stop_token(text, intent_id):
    """The bare 停 is inside every PTZ stop phrase; the yield must still win.

    The yield removes the complete phrase before matching, so the 停 inside
    it is gone with it. Without that ordering the new token would undo
    88eb9a0 and send every PTZ stop to the chassis again.
    """
    # Same orchestrator construction as production, configured tokens.
    d = _orch(_configured()).handle_turn(text, OrchestratorSession(), 1_000)
    # The PTZ intent, not the bypass.
    assert d.kind == "dispatch" and d.intent_id == intent_id, (d.kind, d.intent_id)


def test_the_token_order_is_deterministic():
    """Equal-length tokens are ordered by the word, not by set iteration.

    str hashing is randomised per process, so a set-ordered tuple could fire
    a different token for the same utterance in two runs, and the audit log
    would disagree with itself. Building twice must give the same order, and
    within one length the words must be sorted.
    """
    # Two independent builds from the same list.
    a, b = _configured(), _configured()
    # Same order both times.
    assert a.estop == b.estop
    # Within one length, alphabetical by code point. Two builds in ONE
    # process share a hash seed, so the equality above cannot catch a
    # set-ordered tuple on its own; this line is the one that does (unless
    # the set order happens to be sorted already -- with the current list's
    # seven two-character words that is a 1-in-5040 chance per run).
    two = [w for w in a.estop if len(w) == 2]
    assert two == sorted(two), two


# --- stand: exactly the configured words (2026-10-25 user ruling) ---------
# The built-in "起来" harvest word was removed: a false stand is a motion
# nobody asked for, unlike a false stop. Measured on .1.26: a misheard
# "云台向右看" came back "起来向右看" and fired the stand bypass.

@pytest.mark.parametrize("text", [
    "起来向右看",   # the .1.26 misread that stood the robot up
    "看起来不错",   # ordinary speech containing the same two characters
    "说起来",       # and another
])
def test_bare_qilai_no_longer_fires_the_stand_bypass(text):
    """Without the harvest word nothing in these sentences is a stand word.

    Asserted on the configured groups -- the production matcher -- so the
    case fails if "起来" ever comes back through either the built-in table
    or the union with the config.
    """
    # No bypass action at all: none of the sentences holds a stop / prone /
    # stand word once "起来" is gone.
    assert match_bypass(text, (), _configured()) is None, text


def test_the_stand_group_is_exactly_the_configured_stand_words():
    """The built-in stand table adds nothing to the configured stand list.

    The union in tokens_from_config can only ADD words; for stand the user
    ruled that the configured list is the whole list, so the effective group
    must equal it. A built-in word sneaking back in fails here.
    """
    words = yaml.safe_load(open(_COMMON, encoding="utf-8"))[
        "common"]["audio"]["bypass_keywords"]
    # Same set, order aside (the matcher sorts longest-first).
    assert set(_configured().stand) == set(words["stand"])


@pytest.mark.parametrize("text", ["站起来", "站立", "起立"])
def test_every_configured_stand_word_still_stands(text):
    """The removal did not take any configured word with it.

    The positive half of the two cases above: a matcher that dropped the
    whole stand group would pass them and fail here.
    """
    hit = match_bypass(text, (), _configured())
    # Each configured word alone is a stand bypass.
    assert hit is not None and hit.action == "stand", (text, hit)
