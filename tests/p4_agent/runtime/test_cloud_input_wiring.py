"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_input_wiring.py
Brief: p4 wiring subscribes the two cloud keys only with a robot id, and reads B mode from state/mode

Description:
main_wiring._wire_cloud_inputs is the glue between the GEN session and
runtime/cloud_input. It is small, but each of its three decisions has a
failure that looks like success from the outside:

  * no XBRAIN_ROBOT_ID -> subscribe NOTHING (and say so). Subscribing
    xbrain/None/... or xbrain/unknown/... would be a key nobody publishes:
    the stack starts, the cloud sends, and nothing ever arrives -- the dead
    end p2 already refuses for audio/broadcast;
  * with an id -> exactly the two robot-prefixed keys, and the handles are
    kept (CLAUDE.md 4.3: a dropped zenoh-python handle undeclares itself);
  * the B-mode rule reads p2's LAST state/mode, whatever its age. state/mode
    is published on change only, so a freshness window would read a quiet,
    unchanged broadcast as "unknown" and let cloud audio into ASR.

Mutants this file kills (run by hand, see the commit):
  * rid check removed (subscribes xbrain/None/...)   -> no-id case
  * handles not appended to `subs`                   -> subscribe case
  * voice mode read with get_fresh (age-limited)     -> stale-mode case

Boundaries: a fake GEN session records declare_subscriber calls; whether a
real router delivers on those keys is the running-stack check.
"""
from __future__ import annotations

import base64  # voice_in payloads are base64 PCM
import json  # samples carry JSON bytes, like Zenoh delivers them
import queue  # the worker job queue the wiring hands to the callbacks

import pytest  # monkeypatch fixture and the no_device marker
import yaml  # the real content files, as p4 loads them

from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.cloud_input import VoiceInLimits
from xbrain.p4_agent.runtime.main_wiring import _wire_cloud_inputs  # under test
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator
from xbrain.p4_agent.runtime.turn_loop import TurnLoopConfig
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession
from xbrain.p4_agent.runtime.vad import VadConfig  # worker config only
from xbrain.p4_agent.session.chitchat import ChitchatResponder
from xbrain.p4_agent.state.cache import StateCache  # the state/* cache

# No robot, no Zenoh router: the session is faked below.
pytestmark = pytest.mark.no_device  # no robot needed

#: The content files p4's __main__ loads (absolute: see test_cloud_input).
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"  # the 18 registry file
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"  # chitchat presets


class _FakeGen:
    """Records declare_subscriber calls the way a GEN session would see them.

    Keeps (key, callback) pairs so a case can both check WHICH keys were
    asked for and drive a callback by hand.
    """

    def __init__(self):
        self.subs = []  # (key, callback), in declaration order

    def declare_subscriber(self, key, cb):
        """Record the call; the returned handle is the pair itself."""
        self.subs.append((key, cb))  # keep the pair
        return (key, cb)  # stands in for a zenoh Subscriber handle


def _wire(monkeypatch, rid, cache=None):
    """Run _wire_cloud_inputs over a fake session.

    Returns (gen, subs, ret, jobs, published, asr_calls). The ASR client is
    replaced by a recorder that answers "原地待命", so a sentence that
    reaches ASR both shows up in asr_calls and publishes an A04 frame.

    rid None removes XBRAIN_ROBOT_ID from the environment entirely -- the
    state a hand-restarted process is left in when the variable is not
    exported, which is exactly what the no-id rule is for.
    """
    # monkeypatch restores the environment after the case either way.
    if rid is None:
        monkeypatch.delenv("XBRAIN_ROBOT_ID", raising=False)  # truly unset
    else:
        monkeypatch.setenv("XBRAIN_ROBOT_ID", rid)  # as the stack exports
    asr_calls = []  # every PCM buffer the wiring sent to "ASR"

    def _fake_transcribe(base_url, pcm_16k_mono, *, timeout_s, **_kw):
        """Stands in for asr_client.transcribe (same call shape)."""
        asr_calls.append(pcm_16k_mono)  # record, do not transcribe
        return "原地待命"  # a command that would publish if it ran

    # The wiring imports transcribe at call time, so patching the module
    # attribute here is what it picks up.
    monkeypatch.setattr("xbrain.p4_agent.ai_client.asr_client.transcribe",
                        _fake_transcribe)
    # The production registry and chitchat: the control case needs a real
    # A04 classification to prove a job ran end to end.
    with open(_INTENTS, encoding="utf-8") as fh:
        reg = load_intent_registry(yaml.safe_load(fh))  # 18 registry
    with open(_CHITCHAT, encoding="utf-8") as fh:
        cc = ChitchatResponder(yaml.safe_load(fh))  # chitchat presets
    # The worker config is required but never exercised: no frame is fed.
    vad = VadConfig(energy_threshold=300, tail_silence_ms=500,
                    min_utterance_ms=200, frame_ms=20, pre_roll_ms=0)
    cfg = TurnLoopConfig(asr_base_url="http://127.0.0.1:1",  # never dialled
                         asr_http_timeout_s=1.0, vad_cfg=vad)
    # Production-like bounds; the 300 ms sentence is well inside them.
    limits = VoiceInLimits(max_utterance_ms=30000, min_utterance_ms=200,
                           stream_idle_ms=3000, max_streams=4)
    # What the wiring writes into: session calls, kept handles, jobs, frames.
    gen, subs, jobs, published = _FakeGen(), [], queue.Queue(), []
    orch = build_orchestrator(reg, cc, l2_timeout_ms=8000)  # as p4 builds it
    # asr_post None: the fake transcript is already the command text.
    ret = _wire_cloud_inputs(gen, cfg, orch, reg, None, cache or StateCache(),
                             lambda k, d: published.append((k, d)), jobs,
                             limits, subs, OrchestratorSession())  # same order as the wiring
    return gen, subs, ret, jobs, published, asr_calls  # everything a case may check


def _sentence():
    """A whole 300 ms cloud sentence as one eos chunk, in wire form."""
    # 300 ms is above the 200 ms floor the limits impose, so the assembler
    # completes it; the byte value is irrelevant (the ASR is faked).
    body = {"stream_id": "s", "chunk_seq": 0, "codec": "pcm_s16le",
            "sample_rate": 16000, "channels": 1, "channel": "cloud",
            "eos": True,  # one chunk is the whole sentence
            "payload_b64": base64.b64encode(b"\x01" * 32 * 300).decode()}

    class _S:
        """A zenoh-like sample: only .payload is read."""
        payload = json.dumps(body).encode("utf-8")  # the wire form
    return _S()  # one sample, one sentence


def test_without_a_robot_id_nothing_is_subscribed(monkeypatch):
    """No id: no subscription at all, and no assembler handed back.

    "Nothing" rather than "a key with a placeholder id": a placeholder key
    is indistinguishable from a working subscription until somebody notices
    that no cloud command ever arrives.
    """
    gen, subs, ret, _, _, _ = _wire(monkeypatch, None)
    # Neither key was declared, and nothing is held.
    assert gen.subs == [] and subs == []
    assert ret is None  # the heartbeat then has no streams to evict


def test_with_a_robot_id_both_prefixed_keys_are_subscribed(monkeypatch):
    """Exactly the two 11 S2.2 keys, handles kept in `subs` (4.3)."""
    gen, subs, ret, _, _, _ = _wire(monkeypatch, "gj-001")
    # Text first, then audio: the order the wiring declares them in.
    assert [k for k, _ in gen.subs] == ["xbrain/gj-001/cmd/voice_text",
                                        "xbrain/gj-001/audio/voice_in"]
    # The handles the session returned are the ones kept alive.
    assert subs == gen.subs  # strong refs
    assert ret is not None  # the assembler, for heartbeat eviction


def test_a_cloud_sentence_reaches_asr_and_publishes(monkeypatch):
    """The control for the case below: no B mode, the sentence runs.

    Without it, a wiring that never ran voice jobs at all would pass the
    broadcast case for the wrong reason.
    """
    gen, _, _, jobs, published, asr_calls = _wire(monkeypatch, "gj-001")
    # Drive the wiring's own voice_in callback, then run the job it queued
    # by hand -- the TurnLoopWorker would do the same on its thread.
    dict(gen.subs)["xbrain/gj-001/audio/voice_in"](_sentence())
    jobs.get_nowait()()  # what the TurnLoopWorker would do next
    assert len(asr_calls) == 1  # transcribed once
    # A04 went out; a spoken voice turn also answers aloud (cmd/audio/speak),
    # so only the motion frame's presence is asserted, not the full list.
    assert "cmd/motion/intent" in [k for k, _ in published]


def test_broadcast_is_read_from_the_last_state_mode_however_old(monkeypatch):
    """A broadcast reported long ago still keeps cloud audio out of ASR.

    state/mode is sent on change only. The cache entry below is stamped at
    t = 0 ms, i.e. it is as old as the host's monotonic clock; a reader with
    any freshness window would treat it as unknown and transcribe -- on the
    robot, the speaker commanding itself (11 S8.7.4 VOI-05).
    """
    # A real StateCache, written the way the state/* subscriber writes it.
    cache = StateCache()
    cache.update("state/mode", {"schema": "state_mode_v1",
                                "voice_mode": "broadcast"}, 0)  # ancient
    gen, _, _, jobs, published, asr_calls = _wire(monkeypatch, "gj-001",
                                                  cache)  # stamped t = 0
    # Same drive as the control case: callback, then the queued job.
    dict(gen.subs)["xbrain/gj-001/audio/voice_in"](_sentence())
    jobs.get_nowait()()  # the job runs; the runner must refuse it
    assert asr_calls == [] and published == []  # never reached ASR
