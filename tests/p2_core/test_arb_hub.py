"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_arb_hub.py
Brief: P2 live arbitration (99 U99 C1) -- ArbHub P-1' preflight, speaker TTS arbitration and preemption, manual lights

Description:
Until 2026-10-28 P2's four domain arbiters were built by nobody: the speaker
was a first-come lock, the lights had no arbitration and ModeFace's P-1'
preflight got two empty sets. These cases drive the production pieces with the
production p2_core.yaml `arbiter` block (configs/p2_core.yaml, read directly
-- the hub takes the dict, the snapshot is not needed):
  * ArbHub.preflight applies 14 S5.5.3 rules 1-4 in order: free, self-held,
    strictly lower, otherwise blocked -- and reports every blocked domain;
  * SpeakerDomain refuses a speak without a domain-2 TTS source (11 S8.8.1),
    is refused E_BUSY while a broadcast holds the speaker, and when a broadcast
    arrives mid-playback it yields at once (ack_preempt) WITHOUT reopening the
    mic gate -- the old code reopened it under the broadcast;
  * PayloadDomain refuses a light intent while mode_driver holds the lights
    and releases its manual hold after a granted one.
Mutants named per case; run with __pycache__ cleared (CLAUDE.md 7.2.1).
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import pytest  # marks
import yaml  # read the config block

from xbrain.common.errors import E_BUSY, E_SCHEMA  # expected codes
from xbrain.p2_core.domains.hub import ArbHub  # under test
from xbrain.p2_core.runtime.payload_wiring import PayloadDomain, PayloadWiringConfig  # domain 4
from xbrain.p2_core.runtime.speaker_wiring import SpeakerDomain, SpeakerWiringConfig  # domain 2

pytestmark = pytest.mark.no_device  # pure in-process

_ROOT = Path(__file__).resolve().parents[2]  # repo root
_ARB = yaml.safe_load((_ROOT / "configs" / "p2_core.yaml").read_text(
    encoding="utf-8"))["arbiter"]  # the production arbiter block


def _hub():  # fresh per case
    """A fresh hub over the production config."""
    return ArbHub(_ARB)  # all four domains


# -- ArbHub.preflight (14 S5.5.3 P-1') --------------------------------------

def test_preflight_free_domains_pass():
    """Rule 1: nothing held -> nothing blocked."""
    hub = _hub()  # production arbiter config
    assert hub.preflight({"speaker": "broadcast_b"}, set()) == (
        frozenset(), frozenset())


def test_preflight_lower_holder_is_preemptible():
    """Rule 3: a TTS (400) under a broadcast (800) does not block."""
    hub = _hub()  # production arbiter config
    hub.request("speaker", "tts_local", 1000, atomic=True)  # a TTS plays
    assert hub.preflight({"speaker": "broadcast_b"}, set()) == (
        frozenset(), frozenset())


def test_preflight_self_held_before_priority():
    """Rule 2 before rule 3: D (alarm_d 900) -> B (broadcast_b 800) is legal
    because alarm_d is the current mode's own source. MUTATION: check rule 3
    first -> 900 > 800 -> blocked -> this fails (the 14 S5.5.3 gap)."""
    hub = _hub()  # production arbiter config
    hub.request("speaker", "alarm_d", 1000)  # D holds the speaker
    blocked, self_held = hub.preflight({"speaker": "broadcast_b"}, {"alarm_d"})  # D -> B
    assert blocked == frozenset() and self_held == frozenset({"speaker"})  # rule 2


def test_preflight_reports_every_blocked_domain():
    """Rule 4, no short-circuit: two higher / equal holders -> both named.
    MUTATION: return on the first blocked domain -> one missing -> fails."""
    hub = _hub()  # production arbiter config
    hub.request("speaker", "alarm_d", 1000)  # 900 over broadcast_b
    hub.request("payload_light", "mode_driver", 1000)  # equal: not lower
    blocked, _self = hub.preflight({"speaker": "broadcast_b",
                                    "payload_light": "mode_driver"}, set())
    assert blocked == frozenset({"speaker", "payload_light"})  # both named


# -- SpeakerDomain on the arbiter --------------------------------------------

class _Pub:  # gate publisher double
    """Recording publisher (rt/audio/gate)."""

    def __init__(self):  # empty
        self.puts = []  # raw gate frames

    def put(self, data):  # record
        self.puts.append(data)  # gate frame

    def undeclare(self):  # shutdown no-op
        pass  # nothing to free


class _Rt:  # RT session double
    """RT session stand-in: one publisher for the gate."""

    def __init__(self):  # empty
        self.pub = _Pub()  # one recorder

    def declare_publisher(self, _key):  # one key only
        return self.pub  # the gate publisher


class _Mic:  # mic publisher double
    """MicPublisher stand-in: counts mute / unmute."""

    def __init__(self):  # empty
        self.muted = False  # current state
        self.unmutes = 0  # how often it was reopened

    def mute(self):  # gate closed
        self.muted = True  # closed

    def unmute(self):  # gate reopened
        self.muted = False  # open
        self.unmutes += 1  # count reopenings


def _speaker(hub, est_ms):  # speaker on the hub
    """A SpeakerDomain whose TTS 'plays' for est_ms without any device."""
    cfg = SpeakerWiringConfig(payload_base_url="http://127.0.0.1:1",
                              tts_http_timeout_s=1.0, est_ms_per_sentence=0.0)
    mic = _Mic()  # observe mute
    spk = SpeakerDomain(cfg, _Rt(), lambda: int(time.monotonic() * 1000),
                        mic_publisher=mic, hub=hub)
    spk._invoke_tts = lambda text: float(est_ms)  # no HTTP
    return spk, mic  # both observed


def test_speak_without_a_source_is_refused():
    """11 S8.8.1 source is required. MUTATION: default a missing source to
    tts_local -> ok True -> this fails."""
    spk, _mic = _speaker(_hub(), 10)  # idle speaker
    ack = spk.handle_speak("hello", None)  # no source
    assert (ack["ok"], ack["code"]) == (False, E_SCHEMA)  # 11 S8.8.1


def test_speak_is_refused_while_a_broadcast_holds_the_speaker():
    """MUTATION: bypass the arbiter (the old lock-only path) -> the TTS plays
    over the broadcast -> ok True -> this fails."""
    hub = _hub()  # production arbiter config
    hub.request("speaker", "broadcast_b", 1000)  # B holds the speaker
    spk, _mic = _speaker(hub, 10)  # speaker under B
    ack = spk.handle_speak("hello", "tts_local")  # a local reply
    assert (ack["ok"], ack["code"]) == (False, E_BUSY)  # refused
    assert "broadcast_b" in ack["reason"]  # says who holds it


def test_a_broadcast_takes_the_speaker_mid_tts_and_the_gate_stays_closed():
    """A TTS of 2 s is playing; a broadcast requests the speaker. The TTS must
    yield within a poll period and must NOT reopen the mic gate or unmute.
    MUTATION: drop the being_preempted check -> the broadcast waits for the 3 s
    forced deadline and the TTS reopens the gate -> this fails."""
    hub = _hub()  # production arbiter config
    spk, mic = _speaker(hub, 2000)  # a 2 s TTS
    out = {}  # thread result
    t = threading.Thread(target=lambda: out.update(
        ack=spk.handle_speak("long reply", "tts_cloud")))
    t.start()  # TTS starts
    for _ in range(100):  # wait until the TTS holds the speaker
        h = hub.holder("speaker")  # poll
        if h is not None and h.source_id == "tts_cloud":  # TTS holds it
            break  # the TTS is playing
        time.sleep(0.01)  # 10 ms poll
    t0 = time.monotonic()  # B arrives now
    g = hub.request("speaker", "broadcast_b", int(t0 * 1000))  # B arrives
    assert getattr(g.result, "value", g.result) == "queued"  # TTS must yield
    t.join(2.0)  # TTS thread ends
    assert time.monotonic() - t0 < 0.5  # yielded, not forced at 3 s
    assert hub.holder("speaker").source_id == "broadcast_b"  # B holds it
    assert out["ack"]["ok"] is False  # the TTS was cut
    assert mic.muted is True and mic.unmutes == 0  # gate left to B


def test_a_finished_tts_releases_the_speaker():
    """Normal end: gate reopened, mic unmuted, domain idle again.
    MUTATION: forget the release -> holder stays tts_local -> fails."""
    hub = _hub()  # production arbiter config
    spk, mic = _speaker(hub, 20)  # short TTS
    ack = spk.handle_speak("short", "tts_local")  # plays 20 ms
    assert ack["ok"] is True  # it played
    assert hub.holder("speaker") is None  # and let go
    assert mic.muted is False and mic.unmutes == 1  # mic reopened


def _ticker(hub, stop):  # the P2 main loop's tick, 10 Hz
    """Tick the hub every 100 ms on the real monotonic clock until stop."""
    while not stop.is_set():  # until the case ends
        hub.tick(int(time.monotonic() * 1000))  # lease reclaim happens here
        time.sleep(0.1)  # main loop period


def test_a_tts_longer_than_the_lease_plays_to_the_end_under_ticks():
    """The 2026-10-28 on-robot defect: every speaker source has a 1 s lease
    (14 S11 default) and the C1 first cut never renewed, so a 3.4 s TTS was
    revoked at 1 s and left the mic muted for good. The earlier cases never
    ticked the hub, so they could not see it. MUTATION: drop the renew in the
    playback loop -> revoked -> ok False -> this fails."""
    hub = _hub()  # production arbiter config
    stop = threading.Event()  # ends the ticker
    th = threading.Thread(target=_ticker, args=(hub, stop))  # main loop stand-in
    th.start()  # ticking from now on
    try:  # always stop the ticker
        spk, mic = _speaker(hub, 1600)  # 1.6 s, over the 1 s lease
        ack = spk.handle_speak("a long reply", "tts_cloud")  # plays under ticks
    finally:
        stop.set()  # ticker ends
        th.join(1.0)  # joined
    assert ack["ok"] is True  # played to the end
    assert mic.muted is False and mic.unmutes == 1  # mic given back
    assert hub.holder("speaker") is None  # and the speaker released


def test_a_tts_revoked_into_an_idle_domain_gives_the_mic_back():
    """Revoked with nobody taking over (the holder slot went idle): no one
    else will ever reopen the gate, so the TTS must. MUTATION: drop the
    idle-domain reopen -> mic stays muted -> this fails."""
    hub = _hub()  # production arbiter config
    spk, mic = _speaker(hub, 2000)  # a 2 s TTS
    out = {}  # thread result
    t = threading.Thread(target=lambda: out.update(
        ack=spk.handle_speak("long reply", "tts_local")))
    t.start()  # TTS starts
    for _ in range(100):  # wait until the TTS holds the speaker
        h = hub.holder("speaker")  # poll
        if h is not None:  # TTS holds it
            break  # the TTS is playing
        time.sleep(0.01)  # 10 ms poll
    hub.release("speaker", "tts_local", h.req_id, "error", int(time.monotonic() * 1000))  # slot goes idle
    t.join(2.0)  # TTS thread ends
    assert out["ack"]["ok"] is False  # it was cut
    assert mic.muted is False and mic.unmutes == 1  # but the mic came back


def test_mode_holds_survive_ticks_only_when_renewed():
    """B holds the speaker as broadcast_b (1 s lease) for as long as B lasts:
    the main loop's renew_all before every tick keeps it. MUTATION: make
    renew_all a no-op -> the holder is reclaimed at 1 s -> this fails."""
    hub = _hub()  # production arbiter config
    rid = hub.new_req_id("speaker", "broadcast_b")  # B's attempt id
    hub.request("speaker", "broadcast_b", 0, req_id=rid)  # B enters at t=0
    holds = {"speaker": ("broadcast_b", rid)}  # what main_wiring keeps
    for now in range(100, 5001, 100):  # 5 s of 10 Hz passes, simulated clock
        hub.renew_all(holds, now)  # renewed first
        hub.tick(now)  # then the lease check
    assert hub.holder("speaker").source_id == "broadcast_b"  # still B's
    hub.tick(6200)  # no renew for 1.2 s
    assert hub.holder("speaker") is None  # the lease is real: reclaimed


# -- PayloadDomain manual lights -------------------------------------------

def _payload(hub):  # lights domain on the hub
    """A PayloadDomain whose light calls are recorded, not sent."""
    pd = PayloadDomain(PayloadWiringConfig(payload_base_url="http://127.0.0.1:1",
                                           http_timeout_s=0.3),
                       hub=hub, now_mono_ms_fn=lambda: int(time.monotonic() * 1000))
    return pd  # ready


def test_a_light_intent_is_refused_while_the_mode_holds_the_lights(monkeypatch):
    """14 S4.3: manual never fights the mode. MUTATION: skip the request ->
    the red/blue is switched off under a broadcast -> calls_made 1 -> fails."""
    import xbrain.p4_agent.ai_client.lights_client as lc  # patched below
    sent = []  # light calls that would have gone out
    monkeypatch.setattr(lc, "set_redblue", lambda **kw: sent.append(kw) or {"ok": True})
    hub = _hub()  # production arbiter config
    hub.request("payload_light", "mode_driver", 1000)  # B / D holds the lights
    pd = _payload(hub)  # the domain
    pd._dispatch({"intent_id": "D07"})  # "turn the red/blue off"
    assert sent == [] and pd.calls_dropped == 1  # refused
    hub.release("payload_light", "mode_driver",
                hub.holder("payload_light").req_id, "mode_exit", 2000)
    pd._dispatch({"intent_id": "D07"})  # mode over: allowed
    assert len(sent) == 1  # it went out
    assert hub.holder("payload_light") is None  # and manual let go
