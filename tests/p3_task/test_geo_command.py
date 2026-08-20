"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_geo_command.py
Brief: cmd/geo GeoCommand envelope parse + validate + ack (11 S7.9.1)

Description:
Pins the GeoCommand envelope parser: the eight actions / four types / four origins
are closed sets an off-value is REFUSED at (E_SCHEMA), object actions need a
geo_id, upsert needs an obj body, and the ack carries a closed-set code. Each
check names the mutation it reddens (CLAUDE.md 3.3). The per-action appliers land
on top of this in later batches; here we only guard the envelope so a malformed
command never reaches the single-writer applier.
"""
from __future__ import annotations

import pytest

from xbrain.p3_task.ingest.geo_command import (
    GeoCommandError, geo_ack, parse_geo_command,
)

pytestmark = pytest.mark.no_device


def _cmd(**over):
    base = {"cmd_id": "c-1", "action": "upsert", "type": "waypoint",
            "geo_id": "w-gate", "origin": "voice", "base_rev": 0,
            "obj": {"name": "东门"}}
    base.update(over)
    return base


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
    assert ei.value.code == "E_SCHEMA"


def test_parse_payload_not_a_dict():
    with pytest.raises(GeoCommandError, match="object"):
        parse_geo_command(["not", "a", "dict"])


def test_geo_ack_shape():
    a = geo_ack("c-1", "accepted")
    assert a == {"schema": "geo_ack_v1", "cmd_id": "c-1",
                 "result": "accepted", "code": "OK"}
    # rejected carries a closed-set code + detail (E_GEO_CONFLICT etc.). MUTATION:
    # dropping the code lets the cloud/HMI see a free-text failure it cannot map.
    r = geo_ack("c-2", "rejected", "E_GEO_CONFLICT", {"rev": 7})
    assert r["result"] == "rejected" and r["code"] == "E_GEO_CONFLICT"
    assert r["detail"] == {"rev": 7}
