"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ptz_admission.py
Brief: Domain-5 admission on the live arbiter (99 U99 C1) -- manual hold, 30 s idle release, auto_track yields, refocus takes nothing

Description:
Until 2026-10-28 the PTZ controller admitted commands against its current
motion only, so state/arb/ptz said "idle" while an operator steered and B / D
mode had no domain-5 hold. PtzArbAdmission puts that admission on ArbHub's
ptz arbiter. These cases drive it over the PRODUCTION p2_core.yaml arbiter
block (manual_cloud 700 with a 30 s lease, auto_track 500, preset_patrol 300,
manual_idle_release_s), alone and behind a real PtzController:
  * a manual command takes the domain and renews it, never a second attempt;
  * the hold outlives the motion and is released after T_idle (14 S4.4.2);
  * a manual command takes the head from B / D's auto_track at once;
  * auto_track is refused while a manual hold idles out (E_BUSY, holder named);
  * refocus is admission only and takes nothing (11 S7.4.4);
  * a hold lost to the arbiter is forgotten, the next command asks again.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations  # postponed annotations

from pathlib import Path  # repo-relative config

import json  # estop frame
import pytest  # marks
import yaml  # production config

from xbrain.common.errors import E_BUSY  # refusal code
from xbrain.p2_core.domains.hub import ArbHub  # the live arbiters
from xbrain.p2_core.domains.ptz_admission import PtzArbAdmission  # under test
from xbrain.p2_core.ptz.ptz_controller import PtzController, PtzLimits  # the consumer

pytestmark = pytest.mark.no_device  # pure in-process

_ROOT = Path(__file__).resolve().parents[3]  # repo root
_ARB = yaml.safe_load((_ROOT / "configs" / "p2_core.yaml").read_text(
    encoding="utf-8"))["arbiter"]  # production arbiter block
_IDLE_S = float(_ARB["domains"]["ptz"]["manual_idle_release_s"])  # 14 S4.4.2 T_idle
_IDLE_MS = int(_IDLE_S * 1000)  # in ms


class _Clock:  # fake monotonic ms
    """Fake monotonic milliseconds shared by hub calls and the adapter."""

    def __init__(self):  # arbitrary start
        self.t = 100_000  # ms

    def __call__(self):  # now_ms()
        return self.t  # current fake time


def _rig():  # hub + adapter on one clock
    """A fresh hub and adapter."""
    clock = _Clock()  # one time base
    hub = ArbHub(_ARB)  # production arbiters
    return hub, PtzArbAdmission(hub, clock, _IDLE_S), clock  # ready


def _holder(hub):  # who holds domain 5
    """source_id of the ptz holder, None when idle."""
    h = hub.holder("ptz")  # snapshot
    return h.source_id if h else None  # name only


def test_a_manual_command_takes_and_renews_the_domain():
    """The second command of the same operator renews (one req_id); a second
    attempt per command would churn gen and the audit stream. MUTATION:
    always request -> req_id changes -> this fails."""
    hub, adm, _c = _rig()  # fresh
    assert adm.admit("manual_cloud", acquire=True) is None  # admitted
    rid = hub.holder("ptz").req_id  # first attempt
    assert adm.admit("manual_cloud", acquire=True) is None  # again
    assert hub.holder("ptz").req_id == rid  # renewed, not re-requested


def test_the_manual_hold_idles_out_after_t_idle():
    """14 S4.4.2: released T_idle after the LAST command, not when the motion
    ends and not before. MUTATION: compare against 0 (release on the first
    tick) or never release -> one of the two asserts fails."""
    hub, adm, c = _rig()  # fresh
    adm.admit("manual_cloud", acquire=True)  # operator steers
    for step in range(0, _IDLE_MS - 100, 100):  # ticks across the idle window
        c.t += 100  # 10 Hz main loop
        hub.tick(c.t)  # the arbiter's own lease check runs too
        assert adm.tick() is False  # still held
    assert _holder(hub) == "manual_cloud"  # outlived the motion and the lease
    c.t += 200  # past T_idle
    assert adm.tick() is True and _holder(hub) is None  # released


def test_a_manual_command_takes_the_head_from_auto_track_at_once():
    """B / D hold domain 5 as auto_track; an operator takes it immediately
    (700 > 500, 14 S4.4.2). MUTATION: drop the ack on auto_track's behalf ->
    the manual request stays queued -> refused -> this fails."""
    hub, adm, c = _rig()  # fresh
    hub.request("ptz", "auto_track", c.t)  # the mode's hold
    assert adm.admit("manual_cloud", acquire=True) is None  # admitted at once
    assert _holder(hub) == "manual_cloud"  # the operator has it
    assert hub.states(c.t)["ptz"]["waiting"] == []  # no ghost waiter


def test_auto_track_is_refused_while_a_manual_hold_idles_out():
    """MUTATION: drop the higher-holder check -> auto_track admitted -> fails."""
    hub, adm, c = _rig()  # fresh
    hub.request("ptz", "manual_cloud", c.t)  # an operator holds it
    gen = hub.states(c.t)["ptz"]["gen"]  # the domain gen the refusal must carry
    busy = adm.admit("auto_track", acquire=True)  # tracker asks
    assert busy == {"domain": "ptz", "holder": "manual_cloud",
                    "holder_priority": 700, "gen": gen}  # 11 S7A.0 E_BUSY detail
    assert _holder(hub) == "manual_cloud"  # unchanged


def test_a_lower_refocus_is_refused_by_the_gate_itself():
    """Refocus takes nothing, so the arbiter never gets to deny it: the
    higher-holder check in admit is its ONLY gate. MUTATION: drop that check
    -> admitted under the operator's hold -> this fails."""
    hub, adm, c = _rig()  # fresh
    hub.request("ptz", "manual_cloud", c.t)  # an operator holds it
    busy = adm.admit("auto_track", acquire=False)  # a lower source's refocus
    assert busy is not None and busy["holder"] == "manual_cloud"  # refused, holder named


def test_refocus_is_admission_only():
    """11 S7.4.4: refocus is not a holder. MUTATION: ignore acquire=False ->
    the domain is taken -> this fails."""
    hub, adm, _c = _rig()  # fresh
    assert adm.admit("manual_cloud", acquire=False) is None  # admitted
    assert _holder(hub) is None  # nothing taken


def test_a_hold_lost_to_the_arbiter_is_forgotten():
    """The 30 s lease is the arbiter's own backstop (e.g. a stalled main loop
    ran no adapter tick). MUTATION: judge "mine" from the remembered attempt
    alone, not from the live holder -> the next command 'renews' a hold that
    is gone -> this fails. (Not clearing the stale attempt in tick is an
    EQUIVALENT mutant: admit re-requests whenever the holder is gone.)"""
    hub, adm, c = _rig()  # fresh
    adm.admit("manual_cloud", acquire=True)  # held
    c.t += 31_000  # past the 30 s lease without any adapter tick
    hub.tick(c.t)  # the arbiter revokes it
    assert _holder(hub) is None  # gone
    assert adm.admit("manual_cloud", acquire=True) is None  # next command, no adapter tick between
    assert _holder(hub) == "manual_cloud"  # held again
    assert adm.tick() is False  # the new hold is in use, not released


# -- behind the real controller ----------------------------------------------

class _Ops:  # recording camera
    """Records camera calls."""

    def __init__(self):  # empty
        self.calls = []  # (kind, args)

    def move(self, pan, tilt, zoom):  # PtzOps.move
        self.calls.append(("move", (pan, tilt, zoom)))  # recorded

    def stop(self, pantilt, zoom):  # PtzOps.stop
        self.calls.append(("stop", (pantilt, zoom)))  # recorded


class _Focus:  # refocus succeeds
    """refocus() always succeeds."""

    def refocus(self):  # FocusOps
        return True, True  # ok, restored


def _controller(hub, adm, clock):  # PtzController on the adapter
    """A controller whose admission is the adapter (production wiring)."""
    acks = []  # published acks
    c = PtzController(_Ops(), _Focus(),
                      PtzLimits(hold_max_ms=2000, min_pulse_ms=100,
                                max_pulse_ms=2800, max_cmd_hz=17),
                      {s: v["priority"] for s, v in _ARB["domains"]["ptz"]["sources"].items()},
                      now_ms=clock, publish_ack=acks.append,
                      publish_event=lambda code, d: None,
                      spawn=lambda fn: fn(), admission=adm)  # refocus inline
    c.acks = acks  # handle for asserts
    return c  # ready


def test_the_controller_puts_a_cloud_jog_on_the_arbiter():
    """state/arb/ptz must name the operator while they steer. MUTATION: the
    controller ignores the injected admission -> holder None -> fails."""
    hub, adm, clock = _rig()  # fresh
    c = _controller(hub, adm, clock)  # production shape
    a = c.handle_command({"cmd_id": "c-1", "action": "jog", "source": "cloud",
                          "vel": {"pan": 0.5, "tilt": 0.0, "zoom": 0.0}, "hold_ms": 800})
    assert a["result"] == "accepted" and _holder(hub) == "manual_cloud"  # held


def test_the_controller_refuses_when_the_arbiter_says_busy():
    """A holder the motion-only check cannot see (it is not moving) must still
    refuse a lower source. MUTATION: the controller drops the gate's verdict
    -> accepted -> this fails."""
    hub, adm, clock = _rig()  # fresh
    hub.request("ptz", "manual_cloud", clock.t)  # an idle manual hold, no motion
    c = _controller(hub, adm, clock)  # production shape
    a = c.handle_command({"cmd_id": "t-1", "action": "jog", "source": "auto_track",
                          "vel": {"pan": 0.5, "tilt": 0.0, "zoom": 0.0}, "hold_ms": 800})
    assert a["result"] == "rejected" and a["code"] == E_BUSY  # refused
    assert a["detail"]["holder"] == "manual_cloud"  # named
    assert a["detail"]["gen"] == hub.states(clock.t)["ptz"]["gen"] == adm.gen()  # 11 S7A.0 gen, live


def test_the_controller_refocus_takes_nothing():
    """MUTATION: refocus admitted with acquire=True -> held -> fails."""
    hub, adm, clock = _rig()  # fresh
    c = _controller(hub, adm, clock)  # production shape
    c.handle_command({"cmd_id": "c-2", "action": "refocus", "source": "cloud"})  # runs inline
    assert c.acks[-1]["result"] == "accepted" and _holder(hub) is None  # done, nothing held


def test_an_arbiter_denial_carries_holder_and_gen():
    """The second refusal path: the arbiter itself denies the request (here a
    disarmed domain, 11 S7A.6) -- the detail still has the 11 S7A.0 shape,
    gen included. MUTATION: drop gen from that path -> this fails."""
    hub, adm, clock = _rig()  # fresh
    hub._arbs["ptz"].arb_suspend("soft_estop", "e-1", clock.t)  # disarm domain 5 (test-only reach-in)
    gen = hub.states(clock.t)["ptz"]["gen"]  # gen after the disarm bump
    busy = adm.admit("manual_cloud", acquire=True)  # an operator asks
    assert busy == {"domain": "ptz", "holder": None,  # disarmed: nobody holds
                    "holder_priority": None, "gen": gen}  # but the shape is complete
    assert hub.states(clock.t)["ptz"]["waiting"] == []  # no ghost waiter left behind


# -- U-2 / 11 PTZ-D6: an estop does not stop the PTZ --------------------------

def _estop_coordinator(arb):  # the p2 estop path, publishers stubbed
    """EstopCoordinator on `arb` -- main_wiring builds it on its own
    Arbiter("motion"), never on an ArbHub domain."""
    from xbrain.p2_core.runtime.estop_wiring import EstopCoordinator  # p2 estop path
    from xbrain.p2_core.three_stops import ForceStrobeState  # strobe latch
    return EstopCoordinator(arb, ForceStrobeState(), lambda ev: None,
                            lambda now_ms: None)  # events / broadcast not under test


def test_an_estop_leaves_the_ptz_usable():
    """User ruling 2026-10-28 (U-2, 11 PTZ-D6 / U-PTZ-4): an estop stops
    the chassis, NOT the camera head -- that is when the operator needs the
    picture most. After a real estop through the p2 coordinator, domain 5
    is not disarmed and a cloud jog still reaches the camera.
    MUTATION: build the coordinator on the ptz arbiter (an "estop disarms
    every domain" wiring) -> domain 5 suspended, jog refused -> fails."""
    from xbrain.common.arbiter.core import Arbiter  # main_wiring's motion arbiter
    hub, adm, clock = _rig()  # fresh
    motion = Arbiter("motion", wait_atomic_timeout_ms=3000)  # as main_wiring builds it
    coord = _estop_coordinator(motion)  # the estop path
    coord.on_estop(json.dumps({"type": "estop", "action": "stop",
                               "cmd_id": "e-1"}).encode("utf-8"), clock.t)  # operator hits estop
    assert motion.suspended() == "soft_estop"  # the chassis IS stopped
    assert hub.states(clock.t)["ptz"]["suspended"] is None  # the head is not
    c = _controller(hub, adm, clock)  # production shape
    a = c.handle_command({"cmd_id": "c-1", "action": "jog", "source": "cloud",
                          "vel": {"pan": 0.5, "tilt": 0.0, "zoom": 0.0}, "hold_ms": 800})
    assert a["result"] == "accepted"  # still steerable
    assert c._ops.calls[-1][0] == "move"  # and the camera got the move
