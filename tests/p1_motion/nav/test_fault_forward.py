"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fault_forward.py
Brief: P1-20 -- rt/chassis/fault forwarded to event/fault/chassis, envelope rebuilt per RT-C3.e

Description:
Why this path exists at all: 11 S2.2.1 registers TWO subscribers of
rt/chassis/fault and says the second one is deliberate -- chassis_relay is the
normal path (CR-9), p1_motion is the path for when the relay is dead, and
E_SAFETY_LINK_LOST (which rides this key) has exactly that as its trigger
scenario. p2_core's degraded-hold criterion IS that error, so a lost report means
p2 never enters hold and keeps dispatching tasks over a severed soft-estop link.

What each case pins:
  * the rebuild is RT-C3.e and not a verbatim forward. seq / ts / src become
    ours, the producer's ts / src are preserved as orig_ts / orig_src, and data
    crosses byte for byte. A verbatim forward passes any "did a message come
    out" test, which is why the fields are asserted one by one.
  * mono / boot are COPIED, not re-stamped. A monotonic reading only means
    something inside the boot domain it was taken in (CLK-C4); re-stamping would
    silently re-base every age computed downstream.
  * (orig_src, orig_ts) survives, so the two copies p5 receives (ours and the
    relay's) are recognisably one message.
  * a frame with no data object is refused rather than wrapped -- wrapping is
    the relay's defensive branch and it cannot write a rid it never received.

The runtime needs two zenohd routers, so the handler is exercised directly on a
stub carrying only the attributes it touches. Mutations verified red 2026-09-27.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from xbrain.p1_motion.runtime.fault_forward import rebuild_forward
from xbrain.p1_motion.runtime.nav_wiring import EVENT_CHASSIS_FAULT_TOPIC, NavRuntime

pytestmark = pytest.mark.no_device

_NAV = (Path(__file__).resolve().parents[3] / "xbrain" / "p1_motion" /
        "runtime" / "nav_wiring.py").read_text(encoding="utf-8")


#: One rt/chassis/fault frame as quadruped publishes it: the 11 S3.0 envelope
#: (its own ts / mono / boot / seq / src) around the S9.8.4 ChassisFault.
_SRC = {
    "v": 1, "rid": "m20s", "ts": 1753660812.5, "mono": 4321.75,
    "boot": "abc12345", "seq": 77, "src": "quadruped", "ts_sync": True,
    "data": {"faults": [{"code": "chs:0x8001", "level": "fatal",
                         "desc": "joint over limit",
                         "since_ts": 1753660812.0}],
             "cleared": []},
}


class _Pub:
    def __init__(self) -> None:
        self.sent = []

    def put(self, payload) -> None:
        self.sent.append(json.loads(payload.decode("utf-8")))


class _Sample:
    def __init__(self, raw: bytes) -> None:
        self.payload = raw


class _Stub:
    """Only what _on_chassis_fault touches."""

    def __init__(self) -> None:
        self._fault_pub = _Pub()
        self._seq = {"fault_fwd": 0}
        self._counts = {"fault_fwd_bad": 0}


def _feed(stub, body) -> None:
    raw = body if isinstance(body, bytes) else json.dumps(body).encode("utf-8")
    NavRuntime._on_chassis_fault(stub, _Sample(raw))


# -- the rebuild --------------------------------------------------------------


def test_seq_ts_and_src_become_the_forwarders_own():
    # RT-C3.e. MUTATION: return src_env unchanged -> all three red, and the p5
    # side would be counting two interleaved producers on one key.
    out = rebuild_forward(_SRC, seq=3, src="p1_motion", ts=9999.5)
    assert out["seq"] == 3 and out["src"] == "p1_motion" and out["ts"] == 9999.5


def test_the_producers_ts_and_src_are_preserved_as_orig():
    # This pair is what makes the relay's copy and ours recognisably ONE
    # message on the p5 side. MUTATION: drop the two orig_* writes -> red.
    out = rebuild_forward(_SRC, seq=3, src="p1_motion", ts=9999.5)
    assert out["orig_src"] == "quadruped"
    assert out["orig_ts"] == 1753660812.5


def test_mono_and_boot_are_copied_not_restamped():
    # CLK-C4: a mono reading means something only inside its boot domain, so
    # the forwarder carries the producer's pair across untouched. MUTATION:
    # stamp time.monotonic() here -> red (and every downstream age silently
    # re-bases onto our boot).
    out = rebuild_forward(_SRC, seq=1, src="p1_motion", ts=1.0)
    assert out["mono"] == 4321.75 and out["boot"] == "abc12345"
    # And it is NOT our reading: 4321.75 is far from this process' uptime on
    # any machine that has been up more than a couple of hours, but the
    # equality above is the load-bearing half -- this one only names what the
    # mutation would produce.
    assert out["mono"] != pytest.approx(time.monotonic(), abs=1.0)


def test_v_rid_and_ts_sync_cross_unchanged():
    # ts_sync in particular: CLK-A2 makes rtk_driver the sole authority, so a
    # forwarder never upgrades it. MUTATION: hardcode ts_sync False -> red.
    out = rebuild_forward(_SRC, seq=1, src="p1_motion", ts=1.0)
    assert out["v"] == 1 and out["rid"] == "m20s" and out["ts_sync"] is True


def test_the_payload_crosses_verbatim():
    # The forwarder does not read, reshape or validate the ChassisFault --
    # deriving Events from it is p5's job (11 UM-4 shape).
    out = rebuild_forward(_SRC, seq=1, src="p1_motion", ts=1.0)
    assert out["data"] == _SRC["data"]
    assert out["data"] is _SRC["data"]      # same object, no copy, no reshape


def test_absent_optional_fields_stay_absent():
    # A cloud-shaped producer omits mono/boot (CLK-C4). Writing nulls for them
    # would claim we know they were sent as null.
    bare = {"v": 1, "rid": "m20s", "ts": 5.0, "seq": 1, "src": "quadruped",
            "data": {"faults": [], "cleared": []}}
    out = rebuild_forward(bare, seq=2, src="p1_motion", ts=6.0)
    assert "mono" not in out and "boot" not in out and "ts_sync" not in out


# -- the handler --------------------------------------------------------------


def test_the_handler_publishes_one_rebuilt_frame_per_report():
    stub = _Stub()
    _feed(stub, _SRC)
    assert len(stub._fault_pub.sent) == 1
    out = stub._fault_pub.sent[0]
    assert out["src"] == "p1_motion" and out["seq"] == 1
    assert out["orig_src"] == "quadruped"
    assert out["data"] == _SRC["data"]
    assert stub._counts["fault_fwd_bad"] == 0


def test_the_forward_seq_is_its_own_counter():
    # RT-C3.e wants the FORWARDER's counter, per key. MUTATION: share the
    # cmd_vel counter -> the p5 side sees a stream that jumps by hundreds
    # between two 2 Hz frames and reads it as loss.
    stub = _Stub()
    _feed(stub, _SRC)
    _feed(stub, _SRC)
    assert [f["seq"] for f in stub._fault_pub.sent] == [1, 2]


@pytest.mark.parametrize("bad", [
    b"not json",
    json.dumps({"v": 1, "rid": "m20s", "src": "quadruped"}).encode("utf-8"),
    json.dumps({"v": 1, "data": "not an object"}).encode("utf-8"),
    json.dumps([1, 2, 3]).encode("utf-8"),
])
def test_a_frame_with_no_data_object_is_refused_and_counted(bad):
    # Refused, NOT wrapped. Wrapping a bare payload is chassis_relay's
    # defensive branch and it cannot write a rid it never received -- the
    # result is a frame the far side rejects at the rid step, which on the wire
    # is indistinguishable from a dead link.
    # MUTATION: wrap it instead -> a frame appears, red.
    stub = _Stub()
    _feed(stub, bad)
    assert stub._fault_pub.sent == []
    assert stub._counts["fault_fwd_bad"] == 1


def test_one_bad_frame_does_not_stop_the_next_good_one():
    # This is the path that exists for the case where the OTHER one is already
    # dead, so a single malformed report must not take it down.
    stub = _Stub()
    _feed(stub, b"not json")
    _feed(stub, _SRC)
    assert len(stub._fault_pub.sent) == 1
    assert stub._counts["fault_fwd_bad"] == 1


# -- the wiring ---------------------------------------------------------------


def test_the_rt_key_is_subscribed_and_the_gen_key_declared():
    # Source-text assertions, the discipline the neighbouring wiring tests use:
    # the runtime needs two routers, and these are the facts whose absence
    # produces silence rather than an error.
    # MUTATION: delete either declaration -> red.
    assert '"xbrain/%s/rt/chassis/fault" % self._rid' in _NAV
    assert "self._on_chassis_fault" in _NAV
    assert EVENT_CHASSIS_FAULT_TOPIC == "event/fault/chassis"
    assert "self._fault_pub = self._gen.declare_publisher(" in _NAV


def test_there_is_no_liveness_gate_on_the_relay():
    """Steady-state double delivery is the ruling (2026-09-27 option (a)).

    A "forward only when the relay looks dead" design would put a judgement
    about another process onto the safety path, and would execute for the first
    time on the day it is needed. p5 absorbs the duplicate by CF-4's prefixed
    code plus edge-triggered derivation.
    MUTATION: add an `if relay_seen_recently: return` -> red.
    """
    body = _NAV[_NAV.index("def _on_chassis_fault("):]
    body = body[:body.index("def _on_factor(")]
    for word in ("relay_alive", "relay_seen", "last_relay"):
        assert word not in body, "a relay liveness gate appeared: %s" % word
