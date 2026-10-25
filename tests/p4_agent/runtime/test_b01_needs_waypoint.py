"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_b01_needs_waypoint.py
Brief: B01 goto_waypoint never reaches cmd/task without a destination -- LLM leg, then clarify

Description:
What went wrong (2026-10-24, .1.26 voice test). B01's keywords are the single
characters "去" / "到" / "前往" (16 S5.3 intents.yaml block). The fast leg filled
no slot -- p4 has no fastpath extractor for a waypoint name -- and dispatched a
TaskCommand {type:"goto", params.slots:{}} anyway. P3 accepted it, the speaker
said "收到", and nothing the operator asked for happened. Every sentence that
merely contained "到" did this: "灯开到最大", "声音调到最大", "调到最小", and any
ASR misread that kept the character ("照明灯调到最岸", "到这里啊").

What the design says. 16 S6.6.4 lists B01 as fastpath_then_llm with the miss
branch ending in CLARIFY ("未命中支收尾 = 澄清"), 16 S8.1 check (1) requires the
waypoint to exist before any side effect, and 11 S7.2's task table makes a
destination mandatory for goto ("waypoint_id 或 target"). The same rule already
holds for the A class (11 MI-2, "a missing slot NEVER goes on the wire -- ask
again"); B01 simply never got it.

What these cases pin:
  * to_task_command refuses a goto with no waypoint (the builder is the last
    point in p4 where a TaskCommand can still be stopped);
  * a B01 keyword hit with no waypoint goes to the tier-2 LLM leg exactly ONCE
    (16 S6.6.4 "then_llm"), never straight to cmd/task;
  * if the LLM fills the waypoint, the TaskCommand carries it;
  * if the LLM also cannot, the turn ends in a spoken clarify question and
    nothing is published on cmd/task;
  * if there is no LLM at all, the turn is declined -- still no cmd/task.
A do-nothing implementation fails the "no cmd/task" assertions; an
implementation that calls the LLM in a loop fails the call-count assertion.

Why the LLM leg and not just "refuse when the keyword fired without a slot".
Refusing outright would answer "去东门" with "去哪个点?" forever: p4 has no fast
extractor for a waypoint name, so on the fast leg the slot is ALWAYS empty. The
design's answer is the LLM leg -- the tier-2 classifier sees the waypoint
closed set (16 S6.7 M3_nav) and can name "东门". It can also decide the
sentence was never a goto ("灯开到最大" is a light command); then the turn routes
to whatever it picked, which is the point of handing it over.

Why the call count is asserted everywhere. The LLM leg re-enters the same
dispatch function with the LLM's slots. If the "already used the LLM" guard
were missing, a goto the LLM also could not fill would call the LLM again, and
again -- a recursion that ends in a stack overflow on the voice thread. One call
per turn is the property, so every orchestrator case checks it.

Mutants this file kills (run by hand, see the batch commit):
  * LLM leg removed (fast leg goes straight to the builder)
  * builder check removed (a slot-less goto is built and sent)
  * "already used the LLM" guard removed (the leg re-enters forever)

Boundaries: the orchestrator and the TaskCommand builder only. P3's own refusal
of a destination-less goto is tested on the P3 side (test_task_command.py).
"""
from __future__ import annotations

import pytest
import yaml

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,
    decision_to_publishes,
)
from xbrain.p4_agent.runtime.task_request import (
    TaskRequestError,
    to_task_command,
)
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession,
    Tier2Classification,
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# Pure unit cases: no zenoh, no ASR, no LLM server, no robot hardware.
# The LLM is the _Tier2 stub below; nothing here opens a socket.
pytestmark = pytest.mark.no_device

# The registry and preset tables are the real ones: the bug lived in the
# interaction between the real B01 keyword list and the dispatch path, and a
# hand-made registry would not have had the single-character keywords.
# If B01's keywords are ever changed (16 S5.3 / 18 S4), these cases still hold:
# they assert what happens AFTER a B01 hit, not which sentences hit it.
# The absolute paths match the rest of tests/p4_agent/runtime (the suite runs
# on the robot from /opt/xbrain_v6; there is no relocatable checkout).
# chitchat.yaml is needed only for the spoken lines of declined turns.
# Neither file is written to; both are read once per case.
# The intents table is the 18 S13.1 projection the loader asserts at startup.
# Changing a keyword there is a design change (16 S0.5 CS-1), not a test edit.
# These cases deliberately do not depend on any keyword other than B01's.
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"


def _reg():
    """The production intent registry, loaded from the source table.

    Loaded fresh per call so no case can mutate a shared registry and leak the
    change into the next one. The source table (not the resolved snapshot) is
    read because intents.yaml has no ${common.*} references -- the two are
    byte-identical, and the source is what a fresh checkout has.
    """
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


class _Tier2:
    """A tier-2 stub that records every call and returns a fixed answer.

    The call count is the point: the LLM leg must run once per turn, and an
    implementation that re-entered it on the second (still slot-less) pass
    would loop until the stack overflowed.
    """

    def __init__(self, answer):
        # The fixed classification this stub hands back on every call: None
        # (no LLM / gate denied), or a Tier2Classification(name, slots).
        self.answer = answer
        # Every utterance the orchestrator sent to the LLM, in order.
        self.calls = []

    def __call__(self, text, session, now_mono_ms):
        # Same signature as llm_tier2_fn.build_tier2_fn's product (Tier2Fn).
        self.calls.append(text)
        return self.answer


#: A catalogue holding the waypoint the positive case names. build_orchestrator
#: takes no state_fn (production has no state/geo/manifest subscription), so
#: _turn sets it on the orchestrator the way the integration tests do.
_GATE = {"geo_id": "w-east_gate", "type": "waypoint", "name": "东门",
         "state": "active", "num": 1}


def _turn(text, tier2, items=None):
    """Run one utterance through a real orchestrator.

    Returns (decision, published keys, published (key, body) pairs). The keys
    answer "did anything reach P3"; the pairs let the positive case read the
    TaskCommand body. A new orchestrator per turn keeps the cases independent:
    the orchestrator holds per-session state (pending confirms, chitchat
    counters) that one case must not hand to the next.
    """
    # Preset replies are needed because a declined turn still speaks.
    cc = ChitchatResponder(yaml.safe_load(open(_CHITCHAT, encoding="utf-8")))
    # Built the way main_wiring builds it, with the stub in the tier-2 slot.
    orch = build_orchestrator(_reg(), cc, l2_timeout_ms=8000, tier2_fn=tier2)
    if items is not None:
        # The catalogue the orchestrator resolves destinations against.
        orch._state_fn = lambda: {"state/geo/manifest": {"items": items}}
    # A fresh session: no pending confirm, nothing recording.
    d = orch.handle_turn(text, OrchestratorSession(), 1_000_000)
    # The list of (key, body) the voice loop would publish for this decision.
    pubs = decision_to_publishes(d)
    # Keys only for the "was anything sent" checks; bodies for the one case
    # that reads the TaskCommand content.
    return d, [k for k, _ in pubs], pubs


# --- the builder ---------------------------------------------------------
# to_task_command is called by the orchestrator for every task-create intent;
# these two cases fix its goto behaviour independently of the orchestrator.

def test_to_task_command_refuses_a_goto_with_no_waypoint():
    """11 S7.2: goto needs a destination. Empty or blank both count as none.

    The blank case matters because ASR output is stripped inconsistently: a
    waypoint slot holding only spaces is as empty as a missing one, and P3
    would have nothing to resolve.
    """
    # Missing key, empty string, whitespace only.
    for slots in ({}, {"waypoint": ""}, {"waypoint": "  "}):
        with pytest.raises(TaskRequestError, match="waypoint"):
            to_task_command("goto_waypoint", _reg(), slots=slots,
                            source="voice", cmd_id="c-1", text="到这里啊")


def test_to_task_command_builds_a_goto_that_names_its_waypoint():
    """A resolved waypoint builds the goto: id in params.waypoint_id.

    This is the positive half of the case above: a check that refused every
    goto would pass the negative case and break the one path that works. The
    spoken name still rides in params.slots (provenance); the geo_id is what
    11 S7.2 requires and what P3 can act on.
    """
    cmd = to_task_command("goto_waypoint", _reg(), slots={"waypoint": "东门"},
                          source="voice", cmd_id="c-1", text="去东门",
                          waypoint_id="w-east_gate")
    assert cmd["task"]["type"] == "goto"
    assert cmd["task"]["params"]["waypoint_id"] == "w-east_gate"
    assert cmd["task"]["params"]["slots"]["waypoint"] == "东门"


# --- the orchestrator ----------------------------------------------------

@pytest.mark.parametrize("text", [
    "照明灯调到最岸",      # the 2026-10-24 ASR misread of "照明灯调到最暗"
    "到这里啊",            # the misread of "调到最暗" with the leading 调 cut
    "灯开到最大",          # an ordinary light sentence that happens to hold 到
    "声音调到最大",        # an ordinary volume sentence that holds 到
])
def test_without_an_llm_a_b01_hit_is_declined_not_dispatched(text):
    """No tier-2 available (it returns None): the turn is declined.

    The cmd/task key must not appear at all -- that frame is what P3 accepted
    on the robot and answered with a goto task that had nowhere to go.

    The four sentences are the ones that actually produced a goto on
    2026-10-24 (the first two, from the live log) or would have (the last two,
    from running the orchestrator offline). Each holds the character 到 and
    nothing that names a place. "Declined" here is the existing tier-2
    failure behaviour (a spoken "did not catch that"), which is the honest
    answer when nothing understood the sentence.
    """
    # The LLM is unavailable: tier2_fn answers None for every call.
    t2 = _Tier2(None)
    d, keys, _ = _turn(text, t2)
    # Nothing on cmd/task, whatever else the declined turn does (it speaks).
    assert "cmd/task" not in keys, (text, d.kind, keys)
    # The LLM leg was tried, exactly once, with the utterance itself.
    assert t2.calls == [text]


def test_an_llm_that_also_finds_no_waypoint_ends_in_a_clarify_question():
    """16 S6.6.4: the miss branch of B01 ends in a clarify, not an execution.

    This is the path a real goto request takes today: the operator says
    "到那边去" (a goto, but no place the robot knows), the fast leg hits B01,
    the LLM agrees it is a goto and finds no waypoint in the catalogue. The
    operator must be asked where, not told "收到".
    """
    # The LLM agrees it is a goto but cannot name a place either -- what it
    # does today with an empty geo catalogue.
    t2 = _Tier2(Tier2Classification(name="goto_waypoint", slots={}))
    d, keys, _ = _turn("到这里啊", t2)
    # Still nothing for P3.
    assert "cmd/task" not in keys, keys
    # A spoken reply instead of a dispatch...
    assert d.kind == "reply"
    # ...and the reply is the clarify question, not a generic failure line.
    assert "哪个点" in (d.tts_text or ""), d.tts_text
    # One LLM call, not a loop: the second pass sees llm_used and stops.
    assert len(t2.calls) == 1


def test_an_llm_that_fills_the_waypoint_produces_the_task():
    """The then_llm leg working as designed: the LLM supplies the name.

    Without this case an implementation that refused every B01 (never sending
    any goto) would pass the whole file -- and voice navigation would be dead
    with no test noticing. Since 2026-10-25 the name is resolved HERE against
    the catalogue (P3 never resolves names at dispatch); the no-catalogue and
    invented-name refusals are in test_task_geometry.py.
    """
    # The LLM names the waypoint the fast leg could not extract.
    t2 = _Tier2(Tier2Classification(name="goto_waypoint",
                                    slots={"waypoint": "东门"}))
    d, keys, pubs = _turn("去东门", t2, items=[_GATE])
    # Now the TaskCommand goes out...
    assert "cmd/task" in keys, (d.kind, keys)
    body = dict(pubs)["cmd/task"]
    # ...as a goto on the catalogue's geo_id (16 S8.1 resolved it here).
    assert body["task"]["type"] == "goto"
    assert body["task"]["params"]["waypoint_id"] == "w-east_gate"
    assert len(t2.calls) == 1
