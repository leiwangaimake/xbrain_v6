"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_command.py
Brief: cmd/geo envelope parse + S7.9.5 channel matrix + dispatch ack (11 S7.9)

Description:
Three layers, each with the mutation that reddens it (CLAUDE.md 3.3):

  * the envelope parser -- the four closed sets refuse an off-value (E_SCHEMA),
    object actions need a geo_id, upsert needs an obj body;
  * the S7.9.5 channel matrix, asserted CELL BY CELL rather than by a summary
    ("cloud can do everything") that would stay green while a specific cell
    flipped. The permission table is the system's only authorisation boundary
    (U23: the HMI is not authenticated), so every allow AND every deny is a
    case here, including the three cloud-only cells;
  * the dispatch: a refused or unwired command still produces an ack, and an
    unwired action must NOT come back accepted.

Why the deny cases matter more than the allow cases: an implementation that
allowed everything passes every allow case. Only the denies distinguish it.
"""
from __future__ import annotations

import pytest

from xbrain.common.enums import GEO_ACTION, GEO_ORIGIN, GEO_TYPE
from xbrain.common.errors import (
    E_CHANNEL_DENIED, E_GEO_CONFLICT, E_INTERNAL, E_NOT_IMPLEMENTED, E_SCHEMA,
)
from xbrain.p3_task.ingest.geo_apply import (
    APPLIERS, GeoContext, handle_geo_payload,
)
from xbrain.p3_task.ingest.geo_command import (
    GeoCommandError, allowed_origins, check_channel, geo_ack,
    parse_geo_command,
)

pytestmark = pytest.mark.no_device


def _cmd(**over):
    base = {"cmd_id": "c-1", "action": "upsert", "type": "waypoint",
            "geo_id": "w-gate", "origin": "voice", "base_rev": 0,
            "obj": {"name": "东门"}}
    base.update(over)
    return base


# --------------------------------------------------------------- envelope ---

def test_parse_valid_upsert():
    c = parse_geo_command(_cmd())
    assert c.cmd_id == "c-1" and c.action == "upsert" and c.type == "waypoint"
    assert c.geo_id == "w-gate" and c.origin == "voice" and c.base_rev == 0
    assert c.obj == {"name": "东门"} and c.force is False


def test_parse_set_wide_actions_have_no_geo_id():
    # list / resync are set-wide: no type/geo_id required, and they are nulled so a
    # downstream applier never keys off a stray value. MUTATION: requiring geo_id
    # for every action rejects a legitimate list.
    c = parse_geo_command({"cmd_id": "c-2", "action": "list", "origin": "hmi"})
    assert c.type is None and c.geo_id is None


@pytest.mark.parametrize("over,frag", [
    ({"action": "schedule_patrol"}, "action"),   # off-set action
    ({"origin": "robot"}, "origin"),             # off-set origin
    ({"type": "zone"}, "type"),                  # off-set type (object action)
    ({"cmd_id": ""}, "cmd_id"),                  # empty cmd_id
    ({"geo_id": None}, "geo_id"),                # object action w/o geo_id
    ({"action": "upsert", "obj": None}, "obj"),  # upsert w/o body
    ({"base_rev": "6"}, "base_rev"),             # non-int base_rev
])
def test_parse_rejects_malformed(over, frag):
    """Every closed-set / required-field violation is E_SCHEMA, never a silent
    pass to the single writer. MUTATION: dropping any one guard lets that shape
    through and the applier acts on a malformed command."""
    with pytest.raises(GeoCommandError, match=frag) as ei:
        parse_geo_command(_cmd(**over))
    assert ei.value.code == E_SCHEMA


def test_parse_rejects_teach_as_an_origin():
    """teach is created_by (S7.8.2), NOT a channel: S7.9.5 has no teach column,
    so honouring it would grant a channel a permission row nobody reviewed. The
    2026-08-20 correction settled this; teach recordings travel as origin=voice.
    MUTATION: adding teach to GEO_ORIGIN makes this pass silently."""
    with pytest.raises(GeoCommandError, match="origin") as ei:
        parse_geo_command(_cmd(origin="teach"))
    assert ei.value.code == E_SCHEMA


def test_parse_payload_not_a_dict():
    with pytest.raises(GeoCommandError, match="object"):
        parse_geo_command(["not", "a", "dict"])


# ------------------------------------------------- S7.9.5 channel matrix ---

#: One row per S7.9.5 cell: (action, type, force, allowed origins).
#: Transcribed from the contract table, deliberately as DATA -- the code builds
#: the same answer through branches, and a test that re-ran those branches would
#: agree with any mistake in them.
MATRIX = [
    ("upsert",    "route",    False, {"cloud", "hmi", "voice"}),
    ("upsert",    "waypoint", False, {"cloud", "hmi", "voice"}),
    ("upsert",    "dock",     False, {"cloud", "hmi", "voice"}),
    ("upsert",    "fence",    False, {"cloud", "hmi", "voice"}),
    ("rename",    "route",    False, {"cloud", "hmi", "voice"}),
    ("set_state", "fence",    False, {"cloud", "hmi", "voice"}),
    ("delete",    "route",    False, {"cloud", "hmi", "voice"}),
    ("delete",    "waypoint", False, {"cloud", "hmi", "voice"}),
    ("delete",    "dock",     False, {"cloud", "hmi", "voice"}),
    # The three cloud-only cells (CMD-34). These are the ones worth having.
    ("delete",    "fence",    False, {"cloud"}),
    ("upsert",    "route",    True,  {"cloud"}),      # force spans every action
    ("resync",    None,       False, {"cloud"}),
    # Reads are open to every channel including wecom (no side effect).
    ("get",       "route",    False, {"cloud", "wecom", "hmi", "voice"}),
    ("list",      None,       False, {"cloud", "wecom", "hmi", "voice"}),
    ("refs",      "fence",    False, {"cloud", "wecom", "hmi", "voice"}),
]


@pytest.mark.parametrize("action,gtype,force,allowed", MATRIX)
@pytest.mark.parametrize("origin", sorted(GEO_ORIGIN))
def test_channel_matrix_cell(action, gtype, force, allowed, origin):
    """Every (cell x origin) pair, allow and deny alike.

    MUTATION A: widen the delete-fence cell to the three write channels -- the
    (delete, fence, hmi) and (delete, fence, voice) cases go red, which is the
    CMD-34 requirement that an operator cannot talk the camp fence away.
    MUTATION B: move the force check below the upsert branch -- the
    (upsert, force=True, hmi/voice) cases go red.
    MUTATION C: give wecom write access -- every wecom write case goes red.
    """
    payload = {"cmd_id": "c-x", "action": action, "origin": origin,
               "force": force}
    if gtype is not None:
        payload.update({"type": gtype, "geo_id": "x-1"})
    if action == "upsert":
        payload["obj"] = {"name": "n"}
    cmd = parse_geo_command(payload)
    if origin in allowed:
        check_channel(cmd)          # must not raise
    else:
        with pytest.raises(GeoCommandError) as ei:
            check_channel(cmd)
        assert ei.value.code == E_CHANNEL_DENIED
        # The refusal names the cell, not just "denied": an operator who is told
        # "denied" files a bug, one told "delete fence is cloud-only" does not.
        assert action in str(ei.value)


def test_every_action_has_a_matrix_cell():
    """Guards the guard: a ninth action added to GEO_ACTION without a cell in
    allowed_origins would otherwise reach the fall-through raise only when
    somebody sent one. Runs the whole closed set x the four types."""
    for action in GEO_ACTION:
        for gtype in list(GEO_TYPE) + [None]:
            origins, cell = allowed_origins(action, gtype, force=False)
            assert origins and cell, f"{action}/{gtype} has no cell"
            assert origins <= set(GEO_ORIGIN), (
                f"{cell} allows an origin outside the closed set")


def test_matrix_table_covers_every_action():
    """The transcribed MATRIX above must mention every action, or a new action
    would be exercised only by the completeness test (which cannot know its
    permissions) and never by a per-cell assertion."""
    assert {row[0] for row in MATRIX} == set(GEO_ACTION)


# ------------------------------------------------------------- dispatch ----

class _Ctx(GeoContext):
    def __init__(self):
        super().__init__(geo_conn=None, fence_conn=None, task_conn=None)


@pytest.mark.asyncio
async def test_unwired_action_is_refused_not_accepted():
    """An action with no applier answers rejected + E_NOT_IMPLEMENTED.

    *** This is the batch's load-bearing assertion. MUTATION: return an
    accepted ack when APPLIERS has no entry -- a caller then hears "saved" for a
    route that was never written, which is the fail-silent shape 3.2 catalogues.
    """
    ack = await handle_geo_payload(_cmd(action="rename"), _Ctx(), now_ms=1)
    assert ack["result"] == "rejected"
    assert ack["code"] == E_NOT_IMPLEMENTED
    assert ack["cmd_id"] == "c-1"


@pytest.mark.asyncio
async def test_refusals_still_produce_an_ack():
    """A malformed or denied command is answered, never dropped: the sender is
    an operator waiting on a spoken confirmation. MUTATION: return None / skip
    the publish on the refusal paths and the sender waits forever."""
    denied = await handle_geo_payload(
        _cmd(action="delete", type="fence", origin="hmi"), _Ctx(), now_ms=1)
    assert denied["result"] == "rejected" and denied["code"] == E_CHANNEL_DENIED
    bad = await handle_geo_payload({"cmd_id": "c-9", "action": "nope",
                                    "origin": "hmi"}, _Ctx(), now_ms=1)
    assert bad["result"] == "rejected" and bad["code"] == E_SCHEMA
    assert bad["cmd_id"] == "c-9"


@pytest.mark.asyncio
async def test_unattributable_frame_still_answers():
    # No usable cmd_id: the ack goes out with an empty one rather than raising
    # inside the loop. MUTATION: index payload['cmd_id'] directly -> KeyError
    # escapes into the P3 loop, which also runs task scheduling.
    ack = await handle_geo_payload({"action": "list"}, _Ctx(), now_ms=1)
    assert ack["result"] == "rejected" and ack["cmd_id"] == ""


@pytest.mark.asyncio
async def test_applier_exception_becomes_internal_not_success(monkeypatch):
    """An applier that blows up is reported as failed. MUTATION: let the
    exception propagate -- one bad geo frame stops task dispatch for good."""
    async def _boom(cmd, ctx, now_ms):
        raise RuntimeError("db is on fire")

    monkeypatch.setitem(APPLIERS, "list", _boom)
    ack = await handle_geo_payload({"cmd_id": "c-3", "action": "list",
                                    "origin": "cloud"}, _Ctx(), now_ms=1)
    assert ack["result"] == "rejected" and ack["code"] == E_INTERNAL


@pytest.mark.asyncio
async def test_applier_refusal_keeps_its_own_code_and_detail(monkeypatch):
    """A GeoCommandError from inside an applier keeps its code AND its detail --
    S7.9.2 step 3 requires the local rev to come back so the sender can retry.
    MUTATION: replace exc.detail with the plain reason text and the caller loses
    the rev it needs."""
    async def _conflict(cmd, ctx, now_ms):
        raise GeoCommandError(E_GEO_CONFLICT, "stale", {"rev": 8})

    monkeypatch.setitem(APPLIERS, "list", _conflict)
    ack = await handle_geo_payload({"cmd_id": "c-4", "action": "list",
                                    "origin": "cloud"}, _Ctx(), now_ms=1)
    assert ack["code"] == E_GEO_CONFLICT and ack["detail"] == {"rev": 8}


def test_geo_ack_shape():
    a = geo_ack("c-1", "accepted")
    assert a == {"schema": "geo_ack_v1", "cmd_id": "c-1",
                 "result": "accepted", "code": "OK"}
    # rejected carries a closed-set code + detail (E_GEO_CONFLICT etc.). MUTATION:
    # dropping the code lets the cloud/HMI see a free-text failure it cannot map.
    r = geo_ack("c-2", "rejected", E_GEO_CONFLICT, {"rev": 7})
    assert r["result"] == "rejected" and r["code"] == E_GEO_CONFLICT
    assert r["detail"] == {"rev": 7}
