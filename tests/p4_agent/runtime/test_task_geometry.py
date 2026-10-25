"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_geometry.py
Brief: voice goto / patrol reach cmd/task only with a catalogue-resolved geo_id

Description:
What went wrong (2026-10-25, .1.26 voice test). "到那边去" and "到这边去" hit
B01 on the single character 到 with no waypoint, so the turn took the B01
tier-2 LLM leg (16 S6.6.4, added 2026-10-24). The LLM has no geo catalogue in
its prompt and the slots grammar is a free string, so it INVENTED the values:
B03 patrol_schedule with route "候选" (the prompt's own section label) at
08:00 tomorrow, and B01 with waypoint "到这边=w" (mimicking the prompt's
"name=w-id" candidate format). p4 sent them -- its route check ran only when a
catalogue was present, and in production it never is -- and P3 queued six
tasks, one of them "running". P3 pushes geometry only from route_geo_id, so
none of them could ever move; they only waited for the chassis to come up.

What the design says. 16 S8.1 check (1): the object EXISTS before any side
effect. 11 S7.2: patrol needs route_id, goto needs waypoint_id (or a target
position). 16 S6.7.3 M3_nav: the LLM answers with the catalogue's geo_id.

What these cases pin:
  * no catalogue -> the turn is refused with the catalogue-not-loaded reply
    (the reply F11-F15 already give), and nothing reaches cmd/task -- replayed
    with the LLM's actual 2026-10-25 outputs;
  * with a catalogue, an invented value is refused as not found;
  * a geo_id the LLM copies from the catalogue is accepted as is, but only a
    live one of the right type;
  * a patrol with no route asks which route; B04 (route from the current
    task, 16 S6.7.3 [5], not filled by P3 yet) says so instead of queueing;
  * the builder itself refuses a goto / patrol whose name was not resolved,
    so a caller that skips the orchestrator's step still cannot send one.

Why both p4 AND p3 refuse. p4 is where the operator is standing: a refusal
there is spoken back in the same turn ("which route?", "catalogue not
loaded"), while a refusal at P3 reaches nobody -- p4 does not read task acks,
so the operator would already have heard "收到". P3 is where every producer
converges (voice, HMI, cloud): its check is the one no future sender can
skip. Either alone leaves a hole; together they close it.

Why "refuse" and not "send and let P3 resolve the name". The pre-2026-10-25
code chose the latter, believing P3's geo_refs matched names. geo_refs only
computes the delete-impact set (11 S7.9.4); dispatch reads route_geo_id and
nothing else (p3 runtime/main_wiring _push_route_on_dispatch). So nothing was
lost by refusing -- no name-only voice task had ever executed.

How it was found, so the next reader can reproduce it. task.db on .1.26 was
read back after the test round (read-only sqlite, tasks t-20261025-*): the
mission_json of each held the slots quoted above, and p3's log showed
pending -> ready -> running for the first one. The six were cancelled with
scripts/dev/cloud_probe.py --rid gj-001 --command STOP_TASK. p1 stayed idle
throughout (no route was ever pushed) and quadruped was not running.

Mutants this file kills (run by hand, see the batch commit):
  * absent catalogue degrades to "send without geo_id"   -> no-catalogue cases
  * builder goto waypoint_id check removed               -> builder case
  * builder patrol route_id check removed                -> builder / bare-patrol
  * geo_id accepted whatever its type                    -> wrong-type case
  * geo_id accepted when tombstoned                      -> deleted case
  * geo_id branch removed (names only)                   -> geo_id case

Boundaries: p4 only. P3's own refusal of a patrol without route_id and of a
name-only goto is tests/p3_task/test_task_command.py. Wiring a production
state/geo/manifest subscription is NOT done here; until it is, voice goto /
patrol answer "catalogue not loaded", which is what actually works today.
"""
from __future__ import annotations

import pytest
import yaml

# The real orchestrator and the real builder: the defect was in how the two
# fit together (an LLM leg feeding a builder that did not demand a geo_id),
# so neither is stubbed. Only the LLM answer and the catalogue are fixtures.
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,
    decision_to_publishes,
)
from xbrain.p4_agent.runtime.task_request import (
    DESTINATION_UNRESOLVED,
    PATROL_NEEDS_ROUTE,
    TaskRequestError,
    to_task_command,
)
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession,
    Tier2Classification,
)
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# Pure unit cases: the LLM is a stub returning a fixed classification, and the
# catalogue is a literal list. Nothing opens a socket.
pytestmark = pytest.mark.no_device

# The production tables: the bug lived in the real B01 keyword list (到) and
# the real dispatch path; a hand-made registry would not have reproduced it.
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"

# A small catalogue in the 11 S7.10 GeoManifest item shape. The deleted route
# and the waypoint exist to make the type and tombstone checks bite.
#   _EAST  a live route: the positive patrol cases resolve to it
#   _OLD   a tombstoned route: its id must NOT resolve
#   _GATE  a live waypoint: the positive goto case, and the "right type"
#          half of the wrong-type case (r-east must not stand in for it)
_EAST = {"geo_id": "r-east", "type": "route", "name": "东门路线",
         "state": "active", "num": 1}
_OLD = {"geo_id": "r-old", "type": "route", "name": "旧路线",
        "state": "deleted", "num": 2}
_GATE = {"geo_id": "w-east_gate", "type": "waypoint", "name": "东门",
         "state": "active", "num": 1}
_ITEMS = [_EAST, _OLD, _GATE]

# The two LLM answers measured on .1.26 at 11:50 on 2026-10-25, slot for slot
# as they landed in task.db mission_json.
#   _LLM_B03  "到那边去" -> B03 patrol_schedule: route is the M3_nav prompt's
#             own section label, and the 08:00 / day_offset 1 schedule was
#             never said. Four tasks (one running).
#   _LLM_B01  "到这边去" -> B01 goto_waypoint: the waypoint imitates the
#             prompt's "name=w-id" candidate format. Two tasks.
_LLM_B03 = Tier2Classification(
    name="patrol_schedule",
    slots={"route": "候选", "loops": 1, "at_local": "08:00", "day_offset": 1})
_LLM_B01 = Tier2Classification(name="goto_waypoint",
                               slots={"waypoint": "到这边=w"})


def _reg():
    """The production intent registry, loaded fresh per call.

    Fresh per call so no case can mutate a shared registry and leak the change
    into another. The source table is read (not the resolved snapshot):
    intents.yaml has no ${common.*} references, so the two are identical, and
    the source is what a fresh checkout has.
    """
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


def _turn(text, answer, items=None):
    """One utterance through a real orchestrator with a fixed tier-2 answer.

    items=None is production today: no state/geo/manifest at all. Returns
    (decision, published keys). A new orchestrator per call keeps the cases
    independent (pending confirms and chitchat counters live in it).

    The tier-2 stub ignores its arguments and returns the fixed answer: these
    cases are about what happens AFTER the LLM answered, so the answer is the
    input. answer=None means "no LLM", which only matters for the cases whose
    utterance never reaches tier-2.
    """
    # Preset replies are needed because a refused turn still speaks.
    cc = ChitchatResponder(yaml.safe_load(open(_CHITCHAT, encoding="utf-8")))
    orch = build_orchestrator(_reg(), cc, l2_timeout_ms=8000,
                              tier2_fn=lambda *_a: answer)
    if items is not None:
        # build_orchestrator takes no state_fn (production has no manifest
        # subscription); set it the way the integration tests do.
        orch._state_fn = lambda: {"state/geo/manifest": {"items": items}}
    # One turn; the keys answer "did anything reach P3".
    d = orch.handle_turn(text, OrchestratorSession(), 1_000_000)
    return d, [k for k, _ in decision_to_publishes(d)]


def _task(text, answer, items):
    """The cmd/task body for a turn that is expected to dispatch one.

    Separate from _turn because the positive cases read the body, and a
    helper that returned it for every turn would invite a negative case to
    read a body that should not exist. The catalogue is always present here:
    without one nothing dispatches, by design.
    """
    # Same construction as _turn; the stub is the tier-2 slot.
    cc = ChitchatResponder(yaml.safe_load(open(_CHITCHAT, encoding="utf-8")))
    orch = build_orchestrator(_reg(), cc, l2_timeout_ms=8000,
                              tier2_fn=lambda *_a: answer)
    # The catalogue, keyed by zenoh key as state_fn returns it (11 S7.10).
    orch._state_fn = lambda: {"state/geo/manifest": {"items": items}}
    # A fresh session: nothing pending, nothing recording.
    d = orch.handle_turn(text, OrchestratorSession(), 1_000_000)
    # {key: body} exactly as the voice loop would publish it.
    pubs = dict(decision_to_publishes(d))
    # A failure here prints what happened instead of a bare KeyError.
    assert "cmd/task" in pubs, (d.kind, d.tts_text)
    return pubs["cmd/task"]


# --- the 2026-10-25 replay ----------------------------------------------
# The two utterances and the two LLM answers are the measured ones. The fast
# leg hits B01 on "到" with no waypoint, so the B01 tier-2 leg runs once and
# the stub answers what the real model answered; from there on the path is the
# production one.

@pytest.mark.parametrize("text, answer", [
    ("到那边去", _LLM_B03),     # became t-20261025-001/004/005/006
    ("到这边去", _LLM_B01),     # became t-20261025-002/003
])
def test_without_a_catalogue_the_llm_s_invention_is_not_sent(text, answer):
    """Production today: no catalogue. The turn is refused, nothing is queued.

    The reply is asserted, not just the absence of cmd/task: the builder also
    refuses an unresolved name, so a mutant that restored the old "send
    without a geo_id" degrade would still publish nothing -- and would answer
    "not found" instead of the honest "catalogue not loaded". Only the reply
    tells the two apart.
    """
    # No items: the state map has no state/geo/manifest key at all.
    d, keys = _turn(text, answer)
    # Nothing for P3 -- the six tasks of 2026-10-25 were this frame.
    assert "cmd/task" not in keys, (text, d.kind, keys)
    # A spoken reply, not a dispatch and not a silent drop.
    assert d.kind == "reply"
    # Same words the F11-F15 geo commands speak (CATALOGUE_UNAVAILABLE).
    assert "对象列表" in (d.tts_text or ""), d.tts_text


@pytest.mark.parametrize("text, answer", [
    # Same two utterances and answers as the replay above; only the
    # catalogue differs.
    ("到那边去", _LLM_B03),
    ("到这边去", _LLM_B01),
])
def test_with_a_catalogue_an_invented_name_is_refused(text, answer):
    """Once the catalogue is wired, "候选" / "到这边=w" match nothing: refused.

    This is the half that keeps holding after the subscription lands: the
    catalogue is what turns 16 S8.1's existence check from "cannot be made"
    into "made, and failed".

    Why it needs its own case. The no-catalogue case passes for any
    implementation that refuses when the manifest is missing -- including one
    that, once a manifest exists, sends whatever the LLM said. This case is
    the one that fails such an implementation. It also pins the reply: "not
    found" tells the operator to rephrase, "catalogue not loaded" tells them
    to wait; mixing them up sends them the wrong way.
    """
    # The full catalogue: neither invented value is a name, alias, number or
    # geo_id of anything in it.
    d, keys = _turn(text, answer, items=_ITEMS)
    # Still nothing for P3.
    assert "cmd/task" not in keys, (text, d.kind, keys)
    assert d.kind == "reply"
    # A "not found" reply, not the catalogue-missing one.
    assert "没找到" in (d.tts_text or ""), d.tts_text


# --- geo_id answers (16 S6.7.3 M3_nav) ----------------------------------
# The M3_nav prompt lists candidates as "name=geo_id" and its few-shot answers
# with the id ("route":"r-east"). "到这边=w" is the model imitating that
# format without a real candidate. A real id must therefore pass, and only a
# live one of the requested type.

def test_a_live_geo_id_of_the_right_type_is_accepted():
    """M3_nav tells the LLM to answer with the catalogue id: that resolves.

    Both geometry kinds: a route id becomes task.route_id, a waypoint id
    becomes params.waypoint_id (the two fields 11 S7.2 names). Without the
    geo_id branch the name resolver would look for a route NAMED "r-east" and
    refuse the one answer the prompt asks for.
    """
    # Route: the LLM copied the catalogue's route id.
    body = _task("到那边去", Tier2Classification(
        name="patrol_route", slots={"route": "r-east"}), _ITEMS)
    # task.route_id is the field P3 maps to tasks.route_geo_id and dispatches.
    assert body["task"]["route_id"] == "r-east"
    # Waypoint: the LLM copied the catalogue's waypoint id.
    body = _task("到这边去", Tier2Classification(
        name="goto_waypoint", slots={"waypoint": "w-east_gate"}), _ITEMS)
    # params.waypoint_id is where 11 S7.2 / the W2 row carry a goto target.
    assert body["task"]["params"]["waypoint_id"] == "w-east_gate"


def test_a_geo_id_of_the_wrong_type_is_refused():
    """A route id in the waypoint slot is not a waypoint.

    The id exists, so a check that only asked "is this id in the catalogue"
    would pass it, and P3 would get a goto whose waypoint_id names a route.
    11's W2 table assigns the waypoint existence check (E_NOT_FOUND) to P3,
    but any P3 refusal arrives after the operator heard "收到" (p4 does not
    read task acks); the type mismatch is cheaper to catch here, where the
    reply still reaches the person who spoke. Whether P3 implements that
    W2 check is not asserted anywhere in this file and is not relied on. The check is
    the geo type in GEO_REF_SLOT ("waypoint" for B01, "route" for B02/B03),
    compared with the catalogue item's own type field.
    """
    # r-east is a live ROUTE; the slot wants a waypoint.
    d, keys = _turn("到这边去", Tier2Classification(
        name="goto_waypoint", slots={"waypoint": "r-east"}), items=_ITEMS)
    # Refused at the turn: no frame, a spoken reason.
    assert "cmd/task" not in keys and d.kind == "reply"


def test_a_tombstoned_geo_id_is_refused():
    """A deleted route's id does not exist any more (11 S7.9 tombstone).

    The LLM's few-shot or a stale prompt could still carry it; patrolling a
    deleted route is the "object exists" failure 16 S8.1 is about.
    """
    # r-old is in the catalogue, state "deleted".
    d, keys = _turn("到那边去", Tier2Classification(
        name="patrol_route", slots={"route": "r-old"}), items=_ITEMS)
    # Refused at the turn, the same as a name that never existed.
    assert "cmd/task" not in keys and d.kind == "reply"


# --- patrol with no route ------------------------------------------------
# 11 S7.2: patrol needs route_id. These two are the intents that reached P3
# with none before 2026-10-25 -- one because the operator named no route,
# one because its route is meant to come from P3 itself.

def test_a_bare_patrol_asks_which_route():
    """ "开始巡逻" names no route: ask, do not queue an empty patrol.

    16 S6.6.4 lists B02's miss branch as "执行", but 11 S7.2 requires
    route_id and nothing provides a default route; an executed B02 without a
    route was a patrol that reported running and moved nothing. The conflict
    is recorded in 16 S6.6.4 for a ruling; until then the honest answer is
    the question.
    """
    # Keyword hit on B02 (fastpath_then_llm, but B02 has no LLM leg: only
    # B01 is in DESTINATION_SLOT). The catalogue is present, so the refusal
    # is about the missing name, not the missing catalogue.
    d, keys = _turn("开始巡逻", None, items=_ITEMS)
    # No empty patrol queued.
    assert "cmd/task" not in keys
    # The clarify question (PATROL_NEEDS_ROUTE), not a generic failure line.
    assert "哪条路线" in (d.tts_text or ""), d.tts_text


def test_patrol_repeat_says_the_step_is_not_wired():
    """B04 "再巡一圈": its route is the current task's (16 S6.7.3 [5]).

    P3 is to fill it and does not yet (nothing in p3_task reads
    patrol_repeat), so the operator is told so rather than hearing "收到"
    for a patrol P3 now refuses.
    """
    # Keyword hit on B04 ("再巡"). There is no route slot to resolve.
    d, keys = _turn("再巡一圈", None, items=_ITEMS)
    # Nothing queued for P3 to refuse behind the operator's back.
    assert "cmd/task" not in keys
    # PATROL_REPEAT_NEEDS_CURRENT's reply: the gap, said plainly.
    assert "还没接通" in (d.tts_text or ""), d.tts_text


# --- the builder alone -----------------------------------------------------
# to_task_command is the last point in p4 where a TaskCommand can be stopped.
# The orchestrator resolves first, so in production these raises are not
# reached; they exist so that a caller which skips the orchestrator (a test
# helper, a future text path) cannot build a frame P3 would have to refuse.

def test_the_builder_refuses_an_unresolved_name():
    """to_task_command refuses a name the caller did not resolve.

    This is the second wall: if a future caller forgot the orchestrator's
    resolution step, the frame would still not be built. Both geometry kinds;
    and the "no name at all" case keeps its own clarify reason.
    """
    # One registry for all three calls: the builder reads it only to resolve
    # the intent name (CS-A1), never to change it.
    reg = _reg()
    # goto with a name but no waypoint_id: the shape the orchestrator sent
    # before 2026-10-25 whenever the catalogue was absent.
    with pytest.raises(TaskRequestError, match=DESTINATION_UNRESOLVED):
        to_task_command("goto_waypoint", reg, slots={"waypoint": "东门"},
                        source="voice", cmd_id="c-1")
    # patrol with a name but no route_id: t-20261025-001 was this frame.
    with pytest.raises(TaskRequestError, match=DESTINATION_UNRESOLVED):
        to_task_command("patrol_route", reg, slots={"route": "东门路线"},
                        source="voice", cmd_id="c-2")
    # patrol with no name: the clarify reason, not "unresolved" -- the two
    # get different spoken replies, so the reason itself is the contract.
    with pytest.raises(TaskRequestError, match=PATROL_NEEDS_ROUTE):
        to_task_command("patrol_route", reg, slots={}, source="voice",
                        cmd_id="c-3")
