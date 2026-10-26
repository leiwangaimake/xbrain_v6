"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_perception_mirror_wiring.py
Brief: state/perception is really published by the live wiring, not just mirrorable

Description:
mirror_status_to_gen has its own unit tests, and they prove the mapping is
right. They prove nothing about whether anybody CALLS it -- which is the
failure this repository has paid for repeatedly: a capability that is present,
unit-tested green, and wired to nothing (NEXT S5 SW-28 collects three such
cases). So these two cases drive the real entry point,
run_voice_loop_wiring, against a fake session pair and assert on what lands
on the wire.

How the loop is driven. run_voice_loop_wiring blocks on
`while not stop_flag.get("stop")` with a 0.1 s sleep per tick, so stop_flag
doubles as a per-tick hook: the flag object counts its own reads, feeds one
rt/perception/status sample into the captured subscriber callback on an early
tick, and answers True once enough ticks have passed for the 1 Hz mirror to
fire. That is why these cases take a couple of seconds rather than
milliseconds, and why the tick budget is generous -- a tight budget would make
them flaky on a loaded machine, and a flaky criterion gets deleted.

Two things are faked, both for a stated reason:
  * open_planes -> a pair of recording sessions. The real one opens Zenoh.
  * ChassisClient -> a no-op. The real one dials the chassis, and CLAUDE.md
    CA-1 says the chassis must never see a second live socket; a test that
    opened one could make the axis commands on a bench answer 0xE006.

What these cases do NOT establish, each worth stating because a green run
here is easy to over-read:

  * That p2_core acts on the key. That half is p2's wiring and belongs to its
    own criterion. These cases stop at "the body is on the wire".
  * ~~That the throttle period is right ... it produced nineteen bodies -- one
    per second, which is the throttle working.~~ Superseded 2026-10-26: those
    nineteen bodies from ONE heartbeat were the defect, not the throttle -- the
    same cached status re-sent every second kept p2's heartbeat age fresh after
    perception died (measured on .1.26: cam_rgbd OK 12 s after perception
    stopped). Each status is now mirrored once (three_keys.StatusMirror), and
    test_one_heartbeat_is_mirrored_exactly_once below pins it.
  * That a real Zenoh session behaves like _Session. The fake records instead
    of transmitting; QoS, key-expression matching and the strong-reference
    rule of CLAUDE.md 4.3 are all outside it.

On the one thing the test changes about its subject. stop_flag is read by the
loop once per tick, and this file uses that read as an injection hook. That is
a liberty: the production flag is a plain dict whose get() has no side effect.
It is taken because the alternative -- a second thread racing the loop to feed
a sample -- would make the case non-deterministic, and because the hook only
ADDS an inbound sample; it does not touch the publish path under test. If the
loop ever stops polling stop_flag every tick, these cases stop feeding and go
red rather than silently passing, which is the failure direction one wants.
"""

from __future__ import annotations

import json
import os
import sys
from types import SimpleNamespace

import pytest

#: no_device, even though this drives the production wiring entry point.
#: INF-TS-1's three markers are about HARDWARE, not about how much of the
#: stack a case exercises: both external dependencies here are replaced, so
#: the case runs on any Linux box with nothing attached. Marking it
#: needs_chassis "to be safe" would be the costlier mistake -- the CI filter
#: would drop it on every developer machine, and a wiring criterion nobody
#: runs is the shape this file exists to prevent.
pytestmark = pytest.mark.no_device

# Why this file is separate from perception_src/test_three_keys.py, where the
# fixtures live. That file tests parsing: given bytes, is the DTO right. This
# one tests wiring: given a running loop, does anything come out. Mixing them
# would put a five-second case among sub-second ones and, worse, would let a
# reader scanning for "is the mirror wired" find a file named after the
# parser. The import below is the price of the split -- the fixtures stay
# where they belong and this file reaches for them.
_PERC_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "perception_src")
if _PERC_SRC not in sys.path:
    sys.path.insert(0, _PERC_SRC)

#: A rid that cannot collide with a real deployment's. The value is arbitrary
#: but it must flow through unchanged: the RT key below is composed from it,
#: and if the wiring ever built its keys from something else (a config value,
#: a hostname) the injection would land on a key nobody subscribes to and the
#: first case would go red -- correctly, because that would be a real change
#: in how keys are addressed (11 S2.1).
RID = "wiring-probe"
#: Composed here by hand rather than via three_keys.key_expr(), on purpose: if
#: both the wiring and the test derived the key from the same helper, a change
#: that broke the key shape would move both sides together and the test could
#: not see it. The literal is the second opinion.
STATUS_KEY = "xbrain/%s/rt/perception/status" % RID


class _Session:
    """A recording stand-in for a Zenoh session.

    Only the four methods the wiring actually calls. declare_publisher hands
    back an object whose put() appends into `pubs[key]`, so a test can ask
    "what went out on this key" rather than "was this function called" --
    the distinction that makes these cases about behaviour instead of about
    call bookkeeping.

    declare_subscriber keeps the callback instead of discarding it, which is
    what lets a test inject an inbound sample later. The real session returns
    a handle the caller must hold (CLAUDE.md 4.3: a dropped handle silently
    unsubscribes on the Rust side), so object() stands in for that handle.
    """

    def __init__(self, pubs, subs):
        self._pubs, self._subs = pubs, subs

    def declare_subscriber(self, key, cb):
        self._subs[key] = cb
        return object()

    def declare_queryable(self, key, cb):
        self._subs[key] = cb
        return object()

    def declare_publisher(self, key):
        self._pubs.setdefault(key, [])
        # SimpleNamespace rather than a second class: the only thing a
        # publisher must do here is append, and a named class for one method
        # is more lines for a reader to hold than the lambda it wraps.
        return SimpleNamespace(put=lambda payload: self._pubs[key].append(payload))

    def put(self, key, payload):
        self._pubs.setdefault(key, []).append(payload)


def _run_wiring(monkeypatch, *, feed_status, ticks):
    """Drive run_voice_loop_wiring against fakes; return the published map.

    feed_status=False is not a degenerate case -- it is the one that proves
    absence stays absence (see the second test).

    On the tick budget. The mirror is throttled to 1 Hz and the loop sleeps
    0.1 s per tick, so roughly ten ticks buy one publish. The budget passed
    in is deliberately several times that: a budget tuned to the minimum
    would turn a loaded machine into a red run, and a criterion that goes red
    for reasons unrelated to its subject is one somebody eventually deletes
    (CLAUDE.md 3.2 form 2). The cost is a couple of wall-clock seconds.

    These two cases were written only after a throwaway probe confirmed the
    wiring published at all -- feeding one heartbeat produced nineteen bodies
    with the fixture's own fps_depth in them. Writing the assertion first and
    discovering it never fires would have left the question "is the wiring
    broken, or is my fake wrong" unanswerable.
    """
    monkeypatch.setenv("XBRAIN_ROBOT_ID", RID)
    pubs: dict = {}
    subs: dict = {}

    class _Planes:
        """Context manager standing in for open_planes(("rt", "gen")).

        Hands out two INDEPENDENT sessions sharing one recording dict, which
        matches the real shape (two physically isolated routers, CLAUDE.md
        0.1) while still letting a test read everything from one place.
        __exit__ returns False so an exception inside the loop propagates --
        swallowing it would turn a crash into a quiet pass.
        """

        def __enter__(self):
            return (_Session(pubs, subs), _Session(pubs, subs))

        def __exit__(self, *_a):
            return False

    # Patched on the MODULE, not on the importing site: main_wiring does
    # `from ... import open_planes` inside the function body, so the name is
    # looked up at call time and the module attribute is what it finds. A
    # patch aimed at main_wiring.open_planes would miss (the attribute does
    # not exist until that line runs) and the test would open real sessions.
    import xbrain.common.runtime.session_ctx as sctx
    monkeypatch.setattr(sctx, "open_planes", lambda planes: _Planes())

    # ChassisClient is replaced rather than given a harmless config, because
    # there is no harmless config: its constructor is where the socket is
    # dialled. cfg arrives as None below for the same reason -- nothing reads
    # it once the class is a stub, and passing a half-real config would
    # invite someone to make the stub honour it.
    import xbrain.p1_motion.runtime.main_wiring as mw
    monkeypatch.setattr(mw, "ChassisClient", lambda cfg: type("_C", (), {
        "connect": lambda s: None,
        "close": lambda s: None,
        "send_heartbeat": lambda s: None,
        "is_connected": lambda s: True,
    })())

    from test_three_keys import status_body, wire

    class _Flag(dict):
        """stop_flag that counts its reads and feeds one sample mid-run.

        Subclasses dict because the production flag IS a dict and the wiring
        passes it around (the nav thread reads the same object); a bare object
        with a get() would work here and break the moment someone indexes it.
        """

        def __init__(self):
            super().__init__()
            self.n = 0
            self.fed = False

        def get(self, _key, _default=None):
            self.n += 1
            if feed_status and not self.fed and self.n >= 3 and STATUS_KEY in subs:
                # bytes(sample.payload) is how the intake reads it, so the
                # payload is the raw bytes themselves -- an object with a
                # to_bytes() method does NOT work here, which cost one
                # probe run to find out.
                subs[STATUS_KEY](SimpleNamespace(payload=wire(status_body())))
                self.fed = True
            return self.n > ticks

    # heartbeat_period_s is pushed out of the way so the chassis heartbeat
    # never fires during the window; it is not what is under test here, and a
    # firing heartbeat would exercise the stubbed client for no reason.
    rc = mw.run_voice_loop_wiring(None, _Flag(), heartbeat_period_s=9999.0)
    # A non-zero rc means the loop raised on its way out. Asserted here, in
    # the shared helper, so neither case can pass on a wiring that published
    # correctly and then died -- the publish assertions alone would not
    # notice.
    assert rc == 0, "wiring did not shut down cleanly"
    return pubs


def test_state_perception_is_declared_and_carries_the_heartbeat(monkeypatch):
    """*** The criterion: the live wiring publishes P1-25, with real field values.

    fps_depth is asserted against the fixture's own value rather than merely
    checked for presence: a publisher that emitted a constant body would pass a
    presence check, and that is precisely the empty-shell shape CLAUDE.md 3.2
    form 1 names.

    mutant: delete the 1 Hz mirror block from main_wiring -> red (zero
    publishes). mutant: stop passing _snap.status (mirror a None) -> red.
    """
    pubs = _run_wiring(monkeypatch, feed_status=True, ticks=25)
    # Declared-but-never-published and never-declared are different defects
    # with different fixes, so they get different assertions rather than one
    # combined truthiness check on pubs.get(...).
    assert "state/perception" in pubs, (
        "state/perception publisher was never declared -- the key exists in "
        "11 S1.1.6 P1-25 and in the generated whitelist, but nothing opens it")
    bodies = pubs["state/perception"]
    assert bodies, (
        "a rt/perception/status heartbeat was delivered and nothing was "
        "mirrored to the general plane")
    # Decoded from the bytes that were actually put on the wire, not from the
    # dict the mirror returned: the encode step is part of what is under test
    # (an envelope that cannot round-trip through JSON is a defect a consumer
    # would hit, not p1).
    env = json.loads(bodies[0].decode("utf-8"))
    # 11 S3.0 envelope, same shape state/clock and state/pose already use.
    # Checked field by field rather than against a frozen key set: the
    # envelope is allowed to GAIN fields (S3.0 has grown twice), and a
    # set-equality assertion would turn every such addition into a failure
    # here, which trains people to edit this line instead of reading it.
    for field in ("rid", "seq", "src", "ts", "mono", "data"):
        assert field in env, "envelope missing %s" % field
    # src names the mirroring process, not the originating one. A consumer
    # that wanted to know where the measurement came from reads the key, not
    # this field -- P1-25 is a mirror, and 11 RT-C3.e says a rebuilt envelope
    # carries the rebuilder here.
    assert env["src"] == "p1_motion"
    # Against the fixture's own value, recomputed rather than hardcoded: a
    # literal here would have to be edited whenever the fixture changes, and
    # the edit would silently decouple the two.
    from test_three_keys import status_body
    assert env["data"]["fps_depth"] == status_body()["fps_depth"]


def test_state_perception_stays_silent_without_a_heartbeat(monkeypatch):
    """No rt/perception/status -> nothing published, by design.

    This is the wiring-level half of mirror_status_to_gen's None branch. A
    zero-filled body would read as "measured, camera delivered no frames" and
    would keep 14 S8.1's heartbeat-age criterion permanently fresh, so a dead
    perception would look alive. The publisher is still DECLARED -- declaring
    it costs nothing and keeps the wiring symmetric with state/clock.

    mutant: make the mirror emit a fail-safe body on None -> red.
    """
    pubs = _run_wiring(monkeypatch, feed_status=False, ticks=25)
    # The publisher IS expected -- this case is about what it does not send,
    # not about whether it exists. Asserting its presence first means a
    # regression that removes the declaration fails the previous case (which
    # names that defect) rather than this one (which would then report
    # "stayed silent" and be technically satisfied by a missing publisher).
    assert "state/perception" in pubs, "publisher should still be declared"
    assert pubs["state/perception"] == [], (
        "published a body with no heartbeat behind it -- p2_core would see a "
        "fresh heartbeat forever and 14 S8.1's age criterion would be dead")


def test_one_heartbeat_is_mirrored_exactly_once(monkeypatch):
    """*** 2026-10-26 regression: one rt/perception/status -> ONE state/perception.

    Before the fix the loop re-sent the cached status every second, so a single
    heartbeat produced a body per second for as long as the process ran, and a
    dead perception looked alive to p2 (14 S8.1 heartbeat age never grew).
    25 ticks is 2.5 s of loop time: the old code put 2-3 bodies here.

    mutant: drop the "already sent" check in StatusMirror.take -> red.
    """
    pubs = _run_wiring(monkeypatch, feed_status=True, ticks=25)  # one heartbeat fed
    assert len(pubs["state/perception"]) == 1, (
        "one heartbeat was mirrored %d times -- a stale status is being re-sent"
        % len(pubs["state/perception"]))
