"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_f_class_requests.py
Brief: F01-F15 voice intents -> cmd/teach and cmd/geo (11 S12A.1, batch 6)

Description:
The P4 side of recording and geo CRUD. Three things are pinned:

  * the ROUTING. Every F id goes to cmd/teach or cmd/geo, never to cmd/task.
    Recording used to be modelled as a task, which left F02-F06 and F08-F10
    with no outlet at all -- an operator could start a recording and had no way
    to stop, name or discard it.
  * name RESOLUTION refuses ambiguity instead of guessing. The operator is
    about to delete something; a wrong match is not recoverable by voice.
  * a command that cannot be built is SPOKEN back, not sent. Sending a command
    that must fail costs a round trip and tells the operator nothing.
"""
from __future__ import annotations

import pytest

from xbrain.p4_agent.runtime.geo_request import (
    GeoRequestError, is_geo_intent, resolve_geo_id, to_geo_command,
)
from xbrain.p4_agent.runtime.intent_dispatch import (
    CMD_GEO, CMD_TASK, CMD_TEACH, choose_key,
)
from xbrain.p4_agent.runtime.task_request import is_task_create_intent
from xbrain.p4_agent.runtime.teach_request import (
    TeachRequestError, is_teach_intent, session_id_from_state,
    to_teach_command,
)

pytestmark = pytest.mark.no_device

_MANIFEST = {"items": [
    {"geo_id": "r-east", "type": "route", "name": "东门路线", "num": 3,
     "alias": ["东大门路线"], "rev": 7, "state": "active"},
    {"geo_id": "r-old", "type": "route", "name": "旧路线", "num": None,
     "alias": [], "rev": 2, "state": "deleted"},
    {"geo_id": "f-camp", "type": "fence", "name": "营区外围", "num": 1,
     "alias": [], "rev": 4, "state": "draft"},
    {"geo_id": "w-gate", "type": "waypoint", "name": "东门岗亭", "num": None,
     "alias": [], "rev": 1, "state": "active"},
]}

_RECORDING = {"session": {"session_id": "ts-b0-0001", "state": "recording"}}


# ---------------------------------------------------------------- routing ---

@pytest.mark.parametrize("intent_id,key", [
    ("F01", CMD_TEACH), ("F02", CMD_TEACH), ("F03", CMD_TEACH),
    ("F04", CMD_TEACH), ("F05", CMD_TEACH), ("F06", CMD_TEACH),
    ("F07", CMD_TEACH), ("F08", CMD_TEACH), ("F09", CMD_TEACH),
    ("F10", CMD_TEACH),
    ("F11", CMD_GEO), ("F12", CMD_GEO), ("F13", CMD_GEO), ("F14", CMD_GEO),
    ("F15", CMD_GEO),
])
def test_every_f_id_routes_off_cmd_task(intent_id, key):
    """11 S12A.1, one case per id. MUTATION: leave the F prefix pointing at
    cmd/task -- P3's task ingest receives a recording command it has no
    handler for, and the operator hears nothing back."""
    assert choose_key(intent_id) == key
    assert choose_key(intent_id) != CMD_TASK


def test_recording_is_no_longer_a_task_create():
    """F01/F07 were task-create intents until 2026-08-20. MUTATION: put them
    back and a teach TASK is minted alongside the session, so P3 holds a
    running task that blocks its own arming check 2."""
    assert not is_task_create_intent("record_route_start")
    assert not is_task_create_intent("record_fence_start")
    assert is_task_create_intent("patrol_route")      # unchanged


# ------------------------------------------------------------ cmd/teach -----

def test_start_carries_kind_and_no_session():
    cmd = to_teach_command("record_route_start", slots={}, cmd_id="c-1")
    assert cmd["action"] == "start" and cmd["start"]["kind"] == "route"
    assert "session_id" not in cmd
    assert cmd["issuer"] == {"src": "p4_agent", "channel": "local_voice"}
    fence = to_teach_command("record_fence_start", slots={}, cmd_id="c-2")
    assert fence["start"]["kind"] == "fence"


def test_session_actions_carry_the_open_session_id():
    for name, action in (("record_route_stop", "finish"),
                         ("record_route_mark", "mark"),
                         ("record_route_cancel", "discard")):
        cmd = to_teach_command(name, slots={}, cmd_id="c-1",
                               session_id="ts-b0-0001")
        assert cmd["action"] == action
        assert cmd["session_id"] == "ts-b0-0001"


def test_session_action_without_a_session_is_refused_not_sent():
    """S12A.4 requires the id; P4 does not invent one. MUTATION: omit the id and
    send anyway -- P3 answers E_TEACH_STATE, which reaches the operator as an
    error code instead of "there is no recording in progress"."""
    with pytest.raises(TeachRequestError, match="no recording session"):
        to_teach_command("record_route_stop", slots={}, cmd_id="c-1")


def test_save_needs_a_name_and_never_activates():
    """*** A spoken save must not change where the robot may go.

    S12A.7 constraint 1 keeps enabling a fence separate from saving it (F15,
    L2). MUTATION: set activate=true here and "save the fence" enables it in
    one breath, with no second confirmation anywhere in the path.
    """
    cmd = to_teach_command("record_fence_save", slots={"name": "营区外围"},
                           cmd_id="c-1", session_id="ts-1")
    assert cmd["save"] == {"name": "营区外围", "overwrite": False,
                           "activate": False}
    with pytest.raises(TeachRequestError, match="needs a name"):
        to_teach_command("record_route_save", slots={}, cmd_id="c-1",
                         session_id="ts-1")


def test_mark_once_shapes():
    wp = to_teach_command("record_waypoint", slots={"name": "东门岗亭"},
                          cmd_id="c-1")
    assert wp["mark_once"]["kind"] == "waypoint"
    assert wp["mark_once"]["capture_heading"] is True
    dock = to_teach_command("record_dock", slots={"name": "一号桩"},
                            cmd_id="c-2")
    assert dock["mark_once"]["kind"] == "dock"
    # S12A.8: a dock's handover orientation IS the captured heading.
    assert dock["mark_once"]["capture_heading"] is True
    with pytest.raises(TeachRequestError):
        to_teach_command("record_waypoint", slots={}, cmd_id="c-3")


def test_session_id_only_counts_a_live_session():
    """S12A.3 keeps a closed session for 60 s for idempotent queries. Treating
    that as current would aim finish/save at a recording that already ended.
    MUTATION: accept any session_id present and a 'save' lands on a closed
    session, answering E_TEACH_STATE for a reason the operator cannot see."""
    assert session_id_from_state(_RECORDING) == "ts-b0-0001"
    closed = {"session": {"session_id": "ts-b0-0001", "state": "closed"}}
    assert session_id_from_state(closed) is None
    assert session_id_from_state({"session": {"state": "idle"}}) is None
    assert session_id_from_state(None) is None


def test_non_f_intent_returns_none():
    """None means 'not my family' and is different from raising, which means
    'mine, but incomplete'. Collapsing the two would make a patrol command
    look like a broken recording command."""
    assert to_teach_command("patrol_route", slots={}, cmd_id="c-1") is None
    assert not is_teach_intent("patrol_route")


# -------------------------------------------------------------- cmd/geo -----

def test_delete_resolves_the_name_and_carries_base_rev():
    cmd = to_geo_command("delete_route", slots={"route": "东门路线"},
                         cmd_id="c-1", manifest=_MANIFEST)
    assert cmd["action"] == "delete" and cmd["geo_id"] == "r-east"
    assert cmd["type"] == "route" and cmd["origin"] == "voice"
    # base_rev comes from the manifest. MUTATION: send 0 and every object that
    # has ever been edited answers E_GEO_CONFLICT.
    assert cmd["base_rev"] == 7


def test_alias_and_number_both_resolve():
    by_alias = to_geo_command("delete_route", slots={"route": "东大门路线"},
                              cmd_id="c-1", manifest=_MANIFEST)
    assert by_alias["geo_id"] == "r-east"
    by_num = to_geo_command("delete_route", slots={"route": "3"},
                            cmd_id="c-2", manifest=_MANIFEST)
    assert by_num["geo_id"] == "r-east"


def test_unknown_name_refuses_rather_than_guessing():
    """*** The operator is about to delete something. MUTATION: fall back to a
    prefix or fuzzy match -- 'delete the east route' then removes whichever
    route happened to sort first."""
    with pytest.raises(GeoRequestError, match="no route named"):
        to_geo_command("delete_route", slots={"route": "西门路线"},
                       cmd_id="c-1", manifest=_MANIFEST)


def test_tombstones_are_not_resolution_targets():
    """A deleted object must not answer to its old name: the operator would be
    told 'deleted' for something that already was, and an ambiguous name would
    resolve to the dead one."""
    with pytest.raises(GeoRequestError):
        resolve_geo_id(_MANIFEST["items"], "route", "旧路线")


def test_ambiguous_number_refuses():
    items = [{"geo_id": "r-a", "type": "route", "name": "A", "num": 2,
              "alias": [], "rev": 1, "state": "active"},
             {"geo_id": "r-b", "type": "route", "name": "B", "num": 2,
              "alias": [], "rev": 1, "state": "active"}]
    with pytest.raises(GeoRequestError, match="more than one"):
        resolve_geo_id(items, "route", "2")


def test_missing_manifest_says_so():
    """MUTATION: pass the spoken text through as the geo_id -- P3 refuses it for
    the wrong reason (bad id prefix) and the operator is taught the NAME was
    wrong when the catalogue was simply not loaded yet."""
    with pytest.raises(GeoRequestError, match="catalogue"):
        to_geo_command("delete_route", slots={"route": "东门路线"},
                       cmd_id="c-1", manifest=None)


def test_rename_needs_both_names():
    cmd = to_geo_command("rename_object",
                         slots={"type": "route", "old": "东门路线",
                                "new": "东门主线"},
                         cmd_id="c-1", manifest=_MANIFEST)
    assert cmd["action"] == "rename" and cmd["obj"] == {"name": "东门主线"}
    assert cmd["geo_id"] == "r-east"
    with pytest.raises(GeoRequestError, match="new name"):
        to_geo_command("rename_object",
                       slots={"type": "route", "old": "东门路线"},
                       cmd_id="c-2", manifest=_MANIFEST)


def test_set_active_fence_is_an_activation():
    """F15: the L2 action that S12A.7 keeps separate from saving."""
    cmd = to_geo_command("set_active_fence", slots={"fence": "营区外围"},
                         cmd_id="c-1", manifest=_MANIFEST)
    assert cmd["action"] == "set_state" and cmd["obj"] == {"state": "active"}
    assert cmd["geo_id"] == "f-camp" and cmd["base_rev"] == 4


def test_delete_fence_is_built_and_sent_for_p3_to_refuse():
    """F13 is cloud-only (11 S7.9.5), and the refusal belongs to P3.

    Building it here and letting P3 answer E_CHANNEL_DENIED keeps ONE copy of
    the permission matrix. MUTATION: refuse locally instead -- the policy now
    lives in two places, and a drifted copy silently grants or removes a
    channel's permission with nothing comparing them.
    """
    cmd = to_geo_command("delete_fence", slots={"fence": "营区外围"},
                         cmd_id="c-1", manifest=_MANIFEST)
    assert cmd["action"] == "delete" and cmd["type"] == "fence"
    assert cmd["origin"] == "voice"      # P3 will deny on exactly this


def test_geo_and_teach_families_do_not_overlap():
    """Guards the split: an intent in both tables would build two commands and
    dispatch would send whichever key the routing table happened to name."""
    from xbrain.p4_agent.runtime.geo_request import _GEO_INTENTS
    from xbrain.p4_agent.runtime.teach_request import _TEACH_INTENTS
    assert not (set(_GEO_INTENTS) & set(_TEACH_INTENTS))
    assert len(set(_GEO_INTENTS) | set(_TEACH_INTENTS)) == 15


def test_every_f_intent_in_the_registry_has_a_builder():
    """The 18 F class is fifteen intents; each must reach a builder or it
    reaches dispatch with no payload and P3 receives an empty frame."""
    import os

    import yaml

    from xbrain.p4_agent.registry.intents import load_intent_registry
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    with open(os.path.join(root, "configs", "intents.yaml"),
              encoding="utf-8") as fh:
        registry = load_intent_registry(yaml.safe_load(fh))
    f_names = [e.name for e in registry.entries if e.id.startswith("F")]
    assert len(f_names) == 15
    for name in f_names:
        assert is_teach_intent(name) or is_geo_intent(name), name
