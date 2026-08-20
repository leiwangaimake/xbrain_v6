"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_uplink_w4.py
Brief: HMI upstream whitelist + W4 geo + W4-F fence lock (11 S12.1.1)

Description:
The browser's writable surface, at both levels: the pure frame builder, and the
WebSocket round trip through a fake provider.

The case that carries this batch is the W4-F family. 00 HMI-03a requires that
the electronic fence never enters the HMI's writable surface, and S12.1.1 spells
out why refusing only `delete` is not enough -- disabling an allow fence is
equivalent to deleting it, and changing its geometry is worse. So there is one
case per write op, not one for delete.

The second is that the whitelist is a CLOSED SET (frozen item F-8): a type
nobody reviewed must be refused, and refused differently from a whitelisted
class this build has not wired -- the first is a frontend sending something
unapproved, the second is a backlog item.
"""
from __future__ import annotations

import os

import pytest

from xbrain.common.errors import (
    E_BUSY, E_CHANNEL_DENIED, E_CONFIRM_REQUIRED, E_NOT_IMPLEMENTED, E_SCHEMA,
)
from xbrain.p5_gateway.hmi import uplink

pytestmark = pytest.mark.no_device

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
_STATIC = os.path.join(_REPO, "xbrain", "p5_gateway", "hmi", "static")
_MIN_WEB = {"push_hz": 20, "map": {}, "font": {}, "layout": {},
            "fence": {}, "route": {}, "waypoint": {}}


def _geo_frame(op="rename", gtype="route", **over):
    frame = {"type": "geo", "req_id": "r-1", "op": op,
             "geo": {"type": gtype, "geo_id": "r-east", "base_rev": 7,
                     "name": "east perimeter"}}
    for key, value in over.items():
        if key == "geo":
            frame["geo"].update(value)
        else:
            frame[key] = value
    return frame


# ------------------------------------------------------- the whitelist -----

def test_upstream_types_are_exactly_the_five_of_the_table():
    """S12.1.1 is frozen item F-8 and says an implementer may not wire one more
    type in code. MUTATION: add a sixth -- this reddens, which is the point of
    pinning the set rather than a sample of it."""
    assert uplink.UPSTREAM_TYPES == {"estop", "goto", "exit_broadcast", "geo",
                                     "task"}


def test_unknown_type_is_refused():
    with pytest.raises(ValueError, match="whitelist"):
        uplink.parse_envelope({"type": "teleop", "req_id": "r-1"})
    with pytest.raises(ValueError, match="req_id"):
        uplink.parse_envelope({"type": "geo"})


def test_unwired_class_and_unknown_type_answer_differently():
    """"W7 task is not wired here" is a backlog item; "type flyaway" is a
    frontend sending something nobody reviewed. MUTATION: collapse them into one
    code and the second stops being visible."""
    assert uplink.not_implemented("task").code == E_NOT_IMPLEMENTED


# ----------------------------------------------------------- W4 build ------

def test_rename_builds_a_geo_command_with_hmi_origin():
    built = uplink.build_geo_command(_geo_frame())
    assert isinstance(built, uplink.UplinkCommand)
    assert built.key == "cmd/geo"
    p = built.payload
    # S12.1.1: cmd_id is "h-" + req_id, which is what routes the ack back.
    assert p["cmd_id"] == "h-r-1"
    assert p["action"] == "rename" and p["geo_id"] == "r-east"
    assert p["origin"] == "hmi"
    assert p["base_rev"] == 7
    assert p["obj"] == {"name": "east perimeter"}


def test_a_frame_claiming_to_be_cloud_is_still_stamped_hmi():
    """*** CH-2, and the single most load-bearing line in this file.

    origin is the ENTIRE permission boundary under U23 (the HMI is not
    authenticated), and S7.9.5 grants the cloud-only cells -- delete fence,
    force overwrite, resync -- on the strength of that one field. So the frame
    is tested with the browser ASKING to be cloud.

    A case built from a frame with no origin at all cannot see this: reading
    msg.get("origin", "hmi") and hard-coding "hmi" give the same answer there.
    That is how the first version of this test passed while the mutation was
    live.

    MUTATION: pass the frame's origin through -- this reddens, and on the robot
    an unauthenticated browser tab gains every cloud-only operation.
    """
    built = uplink.build_geo_command(_geo_frame(origin="cloud"))
    assert built.payload["origin"] == "hmi"
    # Same for the other spellings a frontend might try.
    for claimed in ("CLOUD", "voice", "wecom", ""):
        again = uplink.build_geo_command(_geo_frame(origin=claimed))
        assert again.payload["origin"] == "hmi", claimed


def test_upsert_and_set_state_shapes():
    up = uplink.build_geo_command(_geo_frame(
        op="upsert", geo={"obj": {"name": "n", "geom": {"points": []}}}))
    assert up.payload["obj"]["name"] == "n"
    ss = uplink.build_geo_command(_geo_frame(
        op="set_state", gtype="route", geo={"state": "disabled"}))
    assert ss.payload["obj"] == {"state": "disabled"}


def test_ops_outside_the_w4_row_are_refused():
    """force / resync / get / list are not on the HMI surface: S7.9.5 makes the
    first two cloud-only, and the reads are already in the snapshot the browser
    receives. MUTATION: accept any GEO_ACTION here and the HMI gains resync --
    a cloud-only operation -- through a path nobody reviewed."""
    for op in ("resync", "get", "list", "force"):
        refusal = uplink.build_geo_command(_geo_frame(op=op))
        assert isinstance(refusal, uplink.UplinkRefusal)
        assert refusal.code == E_SCHEMA


def test_delete_requires_the_l2_confirm():
    """12A.9's W4 row: a missing confirm on an L2 op is E_CONFIRM_REQUIRED.

    It is an AUDIT credential, not an authorisation one (U23 leaves the HMI
    unauthenticated, so the browser can fill it in itself) -- what actually
    protects the dangerous operations is their absence from the whitelist.
    Required anyway, because the audit trail is worth having.
    """
    refusal = uplink.build_geo_command(_geo_frame(op="delete"))
    assert refusal.code == E_CONFIRM_REQUIRED
    ok = uplink.build_geo_command(
        _geo_frame(op="delete", confirm={"level": "L2"}))
    assert isinstance(ok, uplink.UplinkCommand)
    # A wrong level is not a confirm.
    assert uplink.build_geo_command(
        _geo_frame(op="delete", confirm={"level": "L1"})).code == \
        E_CONFIRM_REQUIRED


# ------------------------------------------------------------- W4-F --------

@pytest.mark.parametrize("op", ["upsert", "delete", "rename", "set_state"])
def test_no_fence_write_reaches_the_bus(op):
    """*** W4-F, one case per write op.

    00 HMI-03a: the electronic fence never enters the HMI writable surface. The
    rule is by geo.type, NOT by op -- S12.1.1 records that a delete-only rule
    leaves set_state and upsert open, and that disabling an allow fence is
    equivalent to deleting it (S9A.1 admits exactly one).

    MUTATION: check only `op == "delete"` (the narrower rule that reads
    obviously sufficient) -- the other three cases redden, and on the robot an
    operator can switch off the camp boundary from a browser tab.
    """
    refusal = uplink.build_geo_command(
        _geo_frame(op=op, gtype="fence",
                   confirm={"level": "L2"},
                   geo={"geo_id": "f-camp", "state": "disabled",
                        "obj": {"name": "x"}}))
    assert isinstance(refusal, uplink.UplinkRefusal)
    assert refusal.code == E_CHANNEL_DENIED
    assert refusal.detail["reason"] == "fence_not_writable_from_hmi"


def test_fence_refs_is_still_allowed():
    """W4-F's second row: refs on a fence is a read and stays open -- it is what
    fills the impact text of a confirmation dialog."""
    built = uplink.build_geo_command(
        _geo_frame(op="refs", gtype="fence", geo={"geo_id": "f-camp"}))
    assert isinstance(built, uplink.UplinkCommand)
    assert built.payload["action"] == "refs"


# --------------------------------------------------- the WS round trip -----

class _Provider:
    """Records what was sent and hands back a canned ack."""

    def __init__(self, ack=None):
        self.sent = []
        self._ack = ack

    def snapshot_inputs(self):
        return {"fences": None, "routes": None, "waypoints": None,
                "enu_origin": None, "pose": None, "tasks": None, "mode": None,
                "link": None, "health": None, "events": None}

    def fence_degraded(self):
        return True

    def rest_inputs(self):
        return {"health": None, "bit": None, "routes": None, "docks": None,
                "metrics": None, "approval_pending": None}

    def send_uplink(self, key, payload):
        self.sent.append((key, payload))

    def take_uplink_ack(self, req_id):
        ack, self._ack = self._ack, None
        return ack


class _NoUplinkProvider(_Provider):
    """A provider from before the uplink seam existed."""
    send_uplink = None


def _client(provider):
    from fastapi.testclient import TestClient

    from xbrain.p5_gateway.hmi.web_server import build_app
    return TestClient(build_app(_MIN_WEB, provider, lambda: None, _STATIC))


def _recv_ack(ws, tries=40):
    """Read frames until an ack arrives (the socket also carries snapshots)."""
    for _ in range(tries):
        frame = ws.receive_json()
        if frame.get("kind") == "ack":
            return frame
    raise AssertionError("no ack in %d frames" % tries)


def test_geo_frame_is_forwarded_and_acked_end_to_end():
    provider = _Provider(ack={"result": "accepted", "code": "OK",
                              "detail": {"rev": 8}})
    with _client(provider).websocket_connect("/ws") as ws:
        ws.send_json(_geo_frame())
        ack = _recv_ack(ws)
    assert ack["req_type"] == "geo" and ack["req_id"] == "r-1"
    assert ack["result"] == "accepted" and ack["detail"] == {"rev": 8}
    assert provider.sent and provider.sent[0][0] == "cmd/geo"
    assert provider.sent[0][1]["origin"] == "hmi"


def test_a_refused_frame_is_answered_and_never_forwarded():
    """*** Every frame gets an answer. MUTATION: drop refused frames silently --
    the operator's dialog spins forever, and 12.3's reconnect rule (resend with
    the same req_id) cannot tell "no answer yet" from "answered"."""
    provider = _Provider()
    with _client(provider).websocket_connect("/ws") as ws:
        ws.send_json(_geo_frame(op="delete", gtype="fence",
                                confirm={"level": "L2"}))
        ack = _recv_ack(ws)
    assert ack["result"] == "rejected" and ack["code"] == E_CHANNEL_DENIED
    assert provider.sent == [], "a W4-F frame must not reach the bus"


def test_unwhitelisted_type_is_answered_with_schema_error():
    provider = _Provider()
    with _client(provider).websocket_connect("/ws") as ws:
        ws.send_json({"type": "teleop", "req_id": "r-9", "axes": {"vx": 1.0}})
        ack = _recv_ack(ws)
    assert ack["result"] == "rejected" and ack["code"] == E_SCHEMA
    assert provider.sent == []


def test_provider_without_the_uplink_seam_refuses_instead_of_failing():
    """An older provider must not break the socket. MUTATION: call send_uplink
    unconditionally -- the WS handler raises, the connection drops, and the HMI
    loses its snapshot stream because of an edit it could simply have refused."""
    provider = _NoUplinkProvider()
    with _client(provider).websocket_connect("/ws") as ws:
        ws.send_json(_geo_frame())
        ack = _recv_ack(ws)
    assert ack["code"] == E_NOT_IMPLEMENTED


def test_rate_limit_refuses_rather_than_queues():
    """SS-4: over the bucket the answer is a refusal, never a queued frame --
    queueing would let an operator build back-pressure by holding a button.

    The code is E_BUSY: 17 S11 names E_RATE_LIMIT, which 11 S13 (the closed set)
    does not define, and CLAUDE.md 3.5 forbids inventing one. detail.reason
    carries the real cause.
    """
    provider = _Provider()
    over = uplink_burst_plus_two()
    with _client(provider).websocket_connect("/ws") as ws:
        for i in range(over):
            ws.send_json(_geo_frame(req_id="r-%d" % i))
        # The accepted ones produce no ack here (this provider hands back none),
        # so the only acks on the wire are the refusals -- which is exactly what
        # is being asserted. Two are expected: burst + 2.
        first = _recv_ack(ws)
        second = _recv_ack(ws)
    for ack in (first, second):
        assert ack["result"] == "rejected" and ack["code"] == E_BUSY
        assert ack["detail"]["reason"] == "rate_limited"
    # And the refused ones were never forwarded.
    assert len(provider.sent) == over - 2


def uplink_burst_plus_two() -> int:
    from xbrain.p5_gateway.hmi.web_server import UPLINK_BURST
    return UPLINK_BURST + 2
