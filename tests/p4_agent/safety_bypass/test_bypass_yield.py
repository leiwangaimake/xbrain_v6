"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_bypass_yield.py
Brief: the safety bypass leaves qualified PTZ stops to E05/E08, every bare stop still fires

Description:
What went wrong (2026-10-24 / 10-25, .1.26 voice tests). The estop bypass
matches its tokens as SUBSTRINGS anywhere in the utterance (16 S4.1:
deliberately loose, a missed stop can hurt someone, a false one costs one
halt). "停止" is one of those tokens, so every qualified stop phrase that
starts with it went to the bypass: "停止扫描" / "停止环视" (E08) and "停止跟踪"
(E05) published cmd/estop and the PTZ kept scanning. 18's E08 boundary table
is explicit about the split -- a bare 停 / 停止 / 停下 is ALWAYS A01, E08 is the
stop that carries a PTZ qualifier.

The fix (option (b), chosen 2026-10-25): keep the keyword tables and CF-1
untouched, and have the bypass REMOVE the complete qualified phrases before it
matches. Removing, not skipping, is the safety property: whatever else the
operator said is still matched, so a stop word next to a PTZ stop still stops
the robot.

What these cases pin:
  * the three PTZ stop phrases reach E05 / E08 (cmd/ptz), never cmd/estop;
  * with a PTZ prefix too ("云台停止扫描");
  * a stop word NEXT TO a PTZ stop still fires the estop, and so does a
    stand word (the removal leaves the rest of the sentence alone);
  * every bare stop still fires (停止 / 停下 / 急停 / 立刻停下);
  * the phrases are removed whole, longest first, and the gap cannot join
    two halves into a token;
  * a misheard raw text still fires even when the CORRECTED text is the PTZ
    phrase -- the raw match runs on the raw words;
  * the stop phrases NOT yielded on purpose ("停止喊话", "停止任务") still
    fire, so changing that is a visible decision, not a side effect.

Mutants this file kills (run by hand, see the batch commit):
  * yield phrases ignored (old behaviour)          -> PTZ-stop cases
  * any yield phrase present -> no bypass at all   -> stop-next-to-PTZ case
  * removed phrase replaced by "" (halves join)    -> join case
  * yield applied only to the corrected text       -> PTZ-stop cases
  * D12 added to the yield set                     -> deliberate-non-yield case

Boundaries: p4's decision only. The cmd/ptz Stop itself is the p2 PTZ driver
(tests/p2_core); that the head physically stops was checked live on .1.26.
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
    is_estop_phrase,
    match_bypass,
    strip_yield_phrases,
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# Pure unit cases: no zenoh, no ASR, no LLM (the tier-2 slot answers None),
# no PTZ hardware.
pytestmark = pytest.mark.no_device

# The production tables. The yield set is DERIVED from the registry's E05 /
# E08 keywords, so a hand-made registry would test a different set than the
# robot runs.
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"


def _turn(text, raw_text=None):
    """One utterance through a real orchestrator: (decision, published keys).

    No LLM: every case here is decided by the bypass or the keyword layer,
    and a tier-2 call would itself be a sign the routing went wrong. A new
    orchestrator per call keeps the cases independent.
    """
    # Fresh tables per call; the yield set is computed from reg at build time.
    reg = load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))
    cc = ChitchatResponder(yaml.safe_load(open(_CHITCHAT, encoding="utf-8")))
    orch = build_orchestrator(reg, cc, l2_timeout_ms=8000,
                              tier2_fn=lambda *_a: None)
    # A fresh session: not recording (U45 suppression would hide the bypass).
    d = orch.handle_turn(text, OrchestratorSession(), 1_000_000,
                         raw_text=raw_text)
    return d, [k for k, _ in decision_to_publishes(d)]


# --- the PTZ stops now reach the PTZ --------------------------------------
# The four phrases are the yield set as the registry produces it today, plus
# the 云台-prefixed form (the E-class keywords are written both ways).

@pytest.mark.parametrize("text, intent_id", [
    ("停止扫描", "E08"),        # the 2026-10-24 batch-2 phrase
    ("停止环视", "E08"),        # the orbit form of the same stop
    ("停止跟踪", "E05"),        # the track-stop of the same family
    ("云台停止扫描", "E08"),    # with the PTZ prefix the operator also uses
])
def test_a_qualified_ptz_stop_reaches_the_ptz_not_the_estop(text, intent_id):
    """The phrase dispatches its PTZ intent; cmd/estop is not published.

    Both halves are asserted. Without the cmd/ptz check, an implementation
    that dropped the bypass AND the dispatch would pass; without the
    cmd/estop check, one that sent both would pass -- and the chassis would
    still halt every time the operator stopped a scan.
    """
    # No raw_text: the text channel shape, raw and corrected are the same.
    # The raw-vs-corrected split has its own case below.
    d, keys = _turn(text)
    # The PTZ stop goes out ...
    assert d.kind == "dispatch" and d.intent_id == intent_id, (d.kind, d.intent_id)
    assert "cmd/ptz" in keys, keys
    # ... and the chassis is left alone.
    assert "cmd/estop" not in keys, keys


# --- every other stop still stops -----------------------------------------

@pytest.mark.parametrize("text", [
    "停止",          # the bare token that ate the PTZ phrases
    "停下",
    "急停",
    "立刻停下",
    "停止扫描然后停下",   # a stop word NEXT TO the PTZ stop
    "停止扫描停止",       # the bare token repeated after the phrase
])
def test_a_stop_word_outside_the_qualified_phrase_still_fires(text):
    """16 S4.1: missing a stop is the unrecoverable error. Nothing here may
    route anywhere but the estop.

    The last two are the reason the phrases are REMOVED rather than used to
    switch the bypass off: an implementation that skipped the bypass when a
    PTZ phrase was present would send "停止扫描然后停下" to E08 and the robot
    would keep moving.
    """
    d, keys = _turn(text)
    # Decided by the bypass (no classification ran) and it is the estop,
    # not prone / stand.
    assert d.kind == "bypass" and d.bypass_action == BYPASS_ESTOP, (text, d.kind)
    # And the frame that reaches Tier 1 was actually produced.
    assert "cmd/estop" in keys


def test_a_stand_word_next_to_a_ptz_stop_still_fires_the_stand_bypass():
    """Removal touches only the phrase: the stand bypass still sees 站起来.

    Stand is the other bypass action that is not the estop; checking it shows
    the removal did not swallow the rest of the sentence for every action.
    """
    # 停止扫描 is removed; 站起来 is left and is a stand token.
    d, _ = _turn("停止扫描站起来")
    # Stand, not estop: the 停止 inside the PTZ phrase is gone, as intended,
    # and nothing else in the sentence is an estop token.
    assert d.kind == "bypass" and d.bypass_action == "stand"


def test_a_misheard_raw_stop_still_fires_when_correction_made_it_a_ptz_stop():
    """Raw "停止扫面" corrected to "停止扫描": the raw words still estop.

    The raw text is not a complete PTZ phrase, so its 停止 fires. The
    correction layer can repair a word; it must not be able to talk the
    bypass out of a stop the raw transcript contained (16 S4 constraint 1).
    """
    # text is what the classifier would see (corrected), raw_text what ASR
    # returned. Only the corrected one is the complete PTZ phrase.
    d, keys = _turn("停止扫描", raw_text="停止扫面")
    # The raw match fires first (match_raw before match_normalized).
    assert d.kind == "bypass" and "cmd/estop" in keys


# --- deliberately not yielded (yet) ---------------------------------------

@pytest.mark.parametrize("text", [
    "停止喊话",   # D12 / C04: p2 drops D12 today -> yielding = false success
    "停止播报",   # D12
    "停止任务",   # B07: motion-related, the estop errs to the safe side
    "停止跟随",   # B12: same
])
def test_the_stops_not_yielded_on_purpose_still_fire(text):
    """These keep the estop, and the reason is in _BYPASS_YIELD_IDS.

    Pinned so that adding D12 (or a B-class stop) to the yield set is a
    deliberate change with its own test update, not a side effect of a
    keyword edit. For D12 the precondition is its downstream (18 S6.3A.1
    route 2); until then an estop that leaves the speaker playing is the
    lesser failure than an "OK" that leaves it playing.
    """
    d, keys = _turn(text)
    # Unchanged behaviour: the estop fires, as before 2026-10-25.
    assert d.kind == "bypass" and "cmd/estop" in keys, (text, d.kind)


# --- the matcher primitives -------------------------------------------------

def test_removal_leaves_a_gap_so_the_two_sides_cannot_join():
    """Removing a phrase must not create an estop token across the gap.

    "紧停止扫描急" minus "停止扫描" would be "紧急" with an empty replacement
    -- an estop token the operator never said, and a chassis halt for a PTZ
    command. Replaced by a space it is "紧 急", which matches nothing.
    (Contrived on purpose: it is the smallest input where the join matters.)
    """
    out = strip_yield_phrases("紧停止扫描急", ["停止扫描"])
    # The two halves are separated, so no token can form across the gap.
    assert "紧急" not in out, out
    # And end to end through the matcher: no bypass action at all.
    assert match_bypass("紧停止扫描急", ["停止扫描"]) is None


def test_the_longest_phrase_is_removed_whole():
    """With nested phrases the longer one goes first.

    "停止扫描一遍" contains "停止扫描"; removing the short one first would
    leave "一遍" -- harmless here, but with a phrase that contained a stop
    token after its qualifier the leftover would fire on words that were
    part of a yielded phrase. Longest-first keeps the removal exact.
    """
    # The short phrase is listed FIRST on purpose: the function must sort,
    # not trust the caller's order.
    out = strip_yield_phrases("停止扫描一遍", ["停止扫描", "停止扫描一遍"])
    # Nothing of the longer phrase survives.
    assert out.strip() == ""


def test_is_estop_phrase_picks_only_what_the_bypass_would_eat():
    """The yield set is filtered through the same token table as the bypass.

    "别扫了" (E08) has no stop token and needs no yield; listing it would be
    harmless but would hide which phrases the bypass actually contests.
    """
    # Contains 停止: the bypass would eat it, so it must yield.
    assert is_estop_phrase("停止扫描") is True
    # No stop token: the bypass never sees it, nothing to yield.
    assert is_estop_phrase("别扫了") is False
