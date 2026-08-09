"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_messaging.py
Brief: messaging tests -- messaging

Description:
BIZ-P2-0 -- p2_core messaging layer tests + spec variants.
"""


from pathlib import Path
from typing import List

import pytest

from xbrain.p2_core.messaging import (
    audio_state,
    p2_publisher,
    p2_subscriber,
    whitelist_gate,
)


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent.parent


# ============================================================
# whitelist_gate
# ============================================================

def test_pub_check_accepts_registered_literal():
    """A literal key in P2_CORE_PUB is accepted."""
    whitelist_gate.check_pub_keys(["state/mode"])
    whitelist_gate.check_pub_keys(["cmd/motion/factor"])
    whitelist_gate.check_pub_keys(["health/summary"])


def test_pub_check_accepts_template_match():
    """A concrete key matching a template in P2_CORE_PUB is accepted.
    P2_CORE_PUB contains `state/arb/{domain}` -- a live key like
    `state/arb/motion` must pass."""
    whitelist_gate.check_pub_keys(["state/arb/motion"])
    whitelist_gate.check_pub_keys(["state/arb/speaker"])
    whitelist_gate.check_pub_keys(["state/arb/ptz"])


def test_pub_check_rejects_unregistered_key():
    """VARIANT (spec #3): declaring a publisher for a key P2 is not
    supposed to publish (e.g. cmd/task -- that's P3's turf per DB1-1)
    must refuse startup."""
    with pytest.raises(whitelist_gate.WhitelistViolation) as ei:
        whitelist_gate.check_pub_keys(["cmd/task/create"])
    msg = str(ei.value)
    assert "cmd/task/create" in msg
    assert "P2_CORE_PUB" in msg


def test_pub_check_rejects_typo():
    """A one-char typo in a registered key must be caught. Prevents
    the silent failure where a subscription pointed at a wrong-cased
    or misspelled key never fires and no one notices."""
    with pytest.raises(whitelist_gate.WhitelistViolation):
        whitelist_gate.check_pub_keys(["state/mods"])   # typo of state/mode


def test_pub_check_reports_all_bad_keys_in_one_raise():
    """Given multiple bad keys, the raise names all of them; operator
    should not have to fix + rerun to find the second violation."""
    with pytest.raises(whitelist_gate.WhitelistViolation) as ei:
        whitelist_gate.check_pub_keys(["cmd/task/create", "cmd/task/cancel",
                                        "state/mode"])
    msg = str(ei.value)
    # state/mode is fine; only the two cmd/task must appear in the error.
    assert "cmd/task/create" in msg
    assert "cmd/task/cancel" in msg


def test_sub_check_accepts_registered_subscriber():
    whitelist_gate.check_sub_keys(["cmd/motion/intent"])
    whitelist_gate.check_sub_keys(["cmd/estop"])
    whitelist_gate.check_sub_keys(["state/robot"])


def test_sub_check_rejects_unregistered_subscriber():
    with pytest.raises(whitelist_gate.WhitelistViolation):
        # P2 is NOT supposed to subscribe cmd/task -- that's P3 too.
        whitelist_gate.check_sub_keys(["cmd/task/create"])


# ============================================================
# P2Publisher
# ============================================================

class _FakePublisher:
    """Minimal Zenoh publisher shape: has .put(payload)."""

    def __init__(self):
        self.puts: List[bytes] = []

    def put(self, payload):
        self.puts.append(payload)


class _FakeSession:
    """Minimal Zenoh session shape: declare_publisher returns a
    _FakePublisher."""

    def __init__(self):
        self.declared: List[str] = []
        self.publishers: dict = {}

    def declare_publisher(self, key_expr):
        self.declared.append(key_expr)
        p = _FakePublisher()
        self.publishers[key_expr] = p
        return p


def test_publisher_declares_registered_key():
    sess = _FakeSession()
    pub = p2_publisher.P2Publisher(session=sess)
    pub.declare("state/mode")
    assert "state/mode" in sess.declared
    assert "state/mode" in pub.declared_keys


def test_publisher_refuses_unregistered_key():
    """VARIANT: declare a key not in P2_CORE_PUB. Session must NOT
    see the declaration -- refusal is before the underlying call."""
    sess = _FakeSession()
    pub = p2_publisher.P2Publisher(session=sess)
    with pytest.raises(whitelist_gate.WhitelistViolation):
        pub.declare("cmd/task/create")
    assert "cmd/task/create" not in sess.declared


def test_publisher_put_reaches_underlying_publisher():
    sess = _FakeSession()
    pub = p2_publisher.P2Publisher(session=sess)
    pub.declare("state/mode")
    pub.put("state/mode", b'{"mode":"idle"}')
    assert sess.publishers["state/mode"].puts == [b'{"mode":"idle"}']


def test_publisher_put_undeclared_key_raises():
    sess = _FakeSession()
    pub = p2_publisher.P2Publisher(session=sess)
    with pytest.raises(KeyError):
        pub.put("state/mode", b"x")


def test_publisher_thread_affinity_rejects_wrong_thread():
    """CLAUDE.md 4.2: publishing from a Zenoh callback thread is
    forbidden. VARIANT: give an allowed_threads set that does NOT
    include the current thread; put() must raise."""
    import threading
    sess = _FakeSession()
    # allowed = a bogus tid so the current thread is NOT allowed.
    pub = p2_publisher.P2Publisher(session=sess,
                                    allowed_threads={9999999})
    pub.declare("state/mode")
    with pytest.raises(p2_publisher.ThreadAffinityError):
        pub.put("state/mode", b"x")


def test_publisher_thread_affinity_accepts_allowed_thread():
    """POSITIVE: put() from a thread whose id IS in allowed_threads
    proceeds normally."""
    import threading
    sess = _FakeSession()
    pub = p2_publisher.P2Publisher(session=sess,
                                    allowed_threads={threading.get_ident()})
    pub.declare("state/mode")
    pub.put("state/mode", b"x")   # must not raise


def test_publisher_publish_threadsafe_posts_via_call_soon():
    """publish_threadsafe hands the (put, key, payload) call to the
    loop's call_soon_threadsafe; verify the exact args."""
    sess = _FakeSession()
    pub = p2_publisher.P2Publisher(session=sess)
    pub.declare("state/mode")
    posted = []

    def fake_call_soon(fn, *args, **kwargs):
        posted.append((fn, args, kwargs))

    pub.publish_threadsafe(fake_call_soon, "state/mode", b"P")
    assert len(posted) == 1
    fn, args, kwargs = posted[0]
    assert fn == pub.put
    assert args == ("state/mode", b"P")


# ============================================================
# P2Subscriber
# ============================================================

class _FakeSubscriberRegistry:
    def __init__(self):
        self.declared: List[tuple] = []

    def declare(self, session, key_expr, handler):
        self.declared.append((key_expr, handler))

    def close(self):
        pass


def test_subscriber_declares_registered_key():
    reg = _FakeSubscriberRegistry()
    sub = p2_subscriber.P2Subscriber(registry=reg)
    handler = lambda sample: None
    sub.declare(None, "cmd/motion/intent", handler)
    assert reg.declared == [("cmd/motion/intent", handler)]
    assert "cmd/motion/intent" in sub.declared_keys


def test_subscriber_refuses_unregistered_key():
    """VARIANT: subscribe to a key not in P2_CORE_SUB. Registry does
    NOT see the declaration."""
    reg = _FakeSubscriberRegistry()
    sub = p2_subscriber.P2Subscriber(registry=reg)
    with pytest.raises(whitelist_gate.WhitelistViolation):
        sub.declare(None, "cmd/task/create", lambda s: None)
    assert reg.declared == []


# ============================================================
# audio_state coherence
# ============================================================

def test_audio_snapshot_accepts_ok():
    snap = audio_state.AudioStateSnapshot(
        mic_status="ok", mic_open=True, gate_reason="unknown")
    # Actually: mic_open=True is fine when reason is unknown / not
    # explicitly a closed state; the coherence rule only fires for
    # device_fault + not_configured.
    assert snap.mic_status == "ok"


def test_audio_snapshot_device_fault_requires_matching_reason():
    """VARIANT (assertion #4): mic_status=device_fault WITHOUT
    gate_reason=device_fault is a construction defect -- exactly the
    'only change gate not state/audio' failure mode from the spec."""
    # Correct pair: allowed.
    audio_state.AudioStateSnapshot(
        mic_status="device_fault", mic_open=False,
        gate_reason="device_fault")

    # Mismatched pair: refused at construction time.
    with pytest.raises(ValueError) as ei:
        audio_state.AudioStateSnapshot(
            mic_status="device_fault", mic_open=False,
            gate_reason="speaker_active")
    assert "device_fault" in str(ei.value)


def test_audio_snapshot_device_fault_forbids_mic_open():
    """VARIANT: device_fault + mic_open=True is a lie -- if the mic
    is faulted it cannot be open."""
    with pytest.raises(ValueError):
        audio_state.AudioStateSnapshot(
            mic_status="device_fault", mic_open=True,
            gate_reason="device_fault")


def test_audio_snapshot_rejects_out_of_set_mic_status():
    with pytest.raises(ValueError):
        audio_state.AudioStateSnapshot(
            mic_status="dying", mic_open=False,
            gate_reason="unknown")


def test_audio_snapshot_rejects_out_of_set_gate_reason():
    with pytest.raises(ValueError):
        audio_state.AudioStateSnapshot(
            mic_status="ok", mic_open=True,
            gate_reason="dying")


def test_publish_snapshot_calls_both_publishers_in_order():
    """POSITIVE: given a snapshot, publish_snapshot calls state/audio
    publisher then gate publisher, both with the right shape."""
    calls = []

    def pub_state(payload):
        calls.append(("state/audio", payload))

    def pub_gate(payload):
        calls.append(("rt/audio/gate", payload))

    snap = audio_state.AudioStateSnapshot(
        mic_status="device_fault", mic_open=False,
        gate_reason="device_fault")
    audio_state.publish_snapshot(snap, pub_state, pub_gate)

    assert len(calls) == 2
    # state/audio first (subscriber that reads both sees state matching gate).
    assert calls[0][0] == "state/audio"
    assert calls[0][1] == {"mic": "device_fault"}
    assert calls[1][0] == "rt/audio/gate"
    assert calls[1][1] == {"mic_open": False, "reason": "device_fault"}


# ============================================================
# Meta: static grep for BIZ-P2-0 rule #1 (no bare declare_subscriber)
# ============================================================

def test_no_bare_declare_subscriber_in_p2_core():
    """BIZ-P2-0 spec #1 static assertion: NO `declare_subscriber(` in
    xbrain/p2_core/ that lands as a bare local variable. Only allowed
    landing sites are self.x = / list.append / SubscriberRegistry.declare.

    This is a runtime canary that catches accidental sibling writers;
    the CI grep in scripts/lint/no_dangling_subscriber.py does the
    full-tree job."""
    import re
    p2_root = REPO / "xbrain" / "p2_core"
    bad: List[str] = []
    for py in p2_root.rglob("*.py"):
        for lineno, raw in enumerate(py.read_text(encoding="utf-8").splitlines(),
                                      start=1):
            line = raw.strip()
            if line.startswith("#"):
                continue
            # look for declare_subscriber( calls not landing on the
            # allowed patterns
            if "declare_subscriber(" not in line:
                continue
            # Allowed: self.x = ..., list.append(...), registry.declare(...)
            if re.search(r"self\.\w+\s*=", line):
                continue
            if ".append(" in line:
                continue
            # A local var like `sub = session.declare_subscriber(...)`
            # is the anti-pattern.
            bad.append("%s:%d %s" % (py.name, lineno, line))
    assert not bad, \
        "bare declare_subscriber() call in p2_core: %s" % bad
