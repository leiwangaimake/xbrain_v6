"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_input.py
Brief: cmd/voice_text + audio/voice_in reach the orchestrator, in order, on the worker thread

Description:
runtime/cloud_input gives p4 its two cloud inputs (11 S8.7.3 / S8.7.5), which
had no subscriber until 2026-10-25. The cases are grouped by the piece they
pin; none needs Zenoh, the ASR service or the robot.

VoiceInAssembler (pure state):
  * chunks 0..n with eos -> exactly their bytes, in order; a repeated chunk
    is ignored; the stream is closed afterwards;
  * a hole (gap), a stream that starts after chunk 0, a channel switch, an
    over-long or too-short sentence each drop the stream with a named reason
    -- a sentence with a piece missing is never transcribed;
  * every format field outside the U92 format (pcm_s16le / 16000 / mono),
    a non-binary channel, a bool chunk_seq, bad base64 and odd PCM length
    are refused before any stream is touched;
  * idle streams are evicted; the open-stream ceiling holds;
  * the 11 S3.0 envelope form is accepted like the bare body.

CloudTurnRunner (real orchestrator over configs/, fake ASR):
  * typed text runs the six-step chain and its frames are published;
  * cloud audio: the transcript (post-processed) is classified, the RAW
    transcript reaches the bypass, and it is NOT addressed (overheard gate);
  * B mode drops channel=cloud audio before ASR, but not channel=wecom
    (11 S8.7.4 VOI-05 / VOI-24);
  * one session per channel: a confirm opened on cloud is not answered by
    wecom (16 S11.1).

Worker + callbacks:
  * TurnLoopWorker runs queued jobs, and a failing job does not stop it;
  * the voice_in callback queues a turn only at eos; a full job queue
    refuses the NEW turn and keeps the queued one.

Mutants this file kills (21, run by hand, see the commit): every rule above
has one, and the commit lists each with the case that turned red.

Boundaries: the Zenoh subscription itself (key strings, strong references)
is checked on the running stack; domain-3 ASR arbitration and the C > E > A
session priority are not implemented and not asserted here.
"""
from __future__ import annotations

import base64  # voice_in payloads are base64 PCM
import json  # the callback cases feed JSON bytes, like Zenoh does
import queue  # the worker's job queue and the bounded-queue case
import threading  # stop / done events for the worker cases

import pytest  # the runner, raises and parametrize
import yaml  # the real content files, loaded as p4 loads them

from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.cloud_input import (
    CloudTurnRunner,  # one turn: text or a completed sentence
    CompletedUtterance,  # the assembler's output value
    VoiceInAssembler,  # chunk -> sentence state machine
    VoiceInLimits,  # its resource bounds
    VoiceInRejected,  # every refusal, with a reason token
    _enqueue,  # the full-queue policy (private, pinned on purpose)
    cloud_input_keys,  # the robot-prefixed key strings
    make_voice_in_callback,  # the Zenoh callback for audio
)
from xbrain.p4_agent.runtime.orchestrator_turn import build_orchestrator
from xbrain.p4_agent.runtime.turn_loop import TurnLoopConfig, TurnLoopWorker
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession
from xbrain.p4_agent.runtime.vad import VadConfig
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# No robot, no Zenoh, no ASR / LLM service: a red case is always the code,
# never a service that happened to be down when the suite ran.
pytestmark = pytest.mark.no_device  # runs on any host with the repo

#: Content files p4's __main__ loads; absolute so a home-dir copy is never
#: picked up by accident when pytest runs from the wrong directory.
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"  # the 18 intent registry
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"  # chitchat presets

#: Small bounds so the limit cases need only a few bytes. 32 bytes = 1 ms.
#: max 10 ms (320 bytes), min 2 ms (64 bytes), idle 100 ms, 2 streams.
_LIMITS = VoiceInLimits(max_utterance_ms=10, min_utterance_ms=2,
                        stream_idle_ms=100, max_streams=2)

#: 1 ms of audio (16 samples x 2 bytes) at 16 kHz mono s16le.
_MS = 32  # 16 samples x 2 bytes


def _pcm(tag: int, ms: int = 1) -> bytes:
    """`ms` milliseconds of PCM whose every byte is `tag`.

    The tag is an order marker: each chunk carries a different byte value,
    so an assembled sentence shows at a glance whether its pieces arrived
    in sequence, twice, or not at all.
    """
    return bytes([tag]) * (_MS * ms)  # even length by construction


def _chunk(seq, *, sid="s-1", eos=False, pcm=None, **override):
    """One voice_in body (11 S8.7.3) in the U92 format; fields overridable.

    The default payload is 1 ms tagged seq+1 (tag 0 would be silence and
    indistinguishable from a zero-filled bug). `override` replaces any field
    after the body is built, which is how the refusal cases break one field
    at a time while every other field stays valid.
    """
    body = {"stream_id": sid, "chunk_seq": seq, "codec": "pcm_s16le",
            "sample_rate": 16000, "channels": 1, "channel": "cloud",
            "session_hint": "cloud", "eos": eos,
            "payload_b64": base64.b64encode(
                _pcm(seq + 1) if pcm is None else pcm).decode("ascii")}
    body.update(override)  # the one field under test, if any
    return body  # a fresh dict per call: cases never share


def _cfg():
    """A TurnLoopConfig for the worker cases.

    The values are never exercised -- no frame is fed, so neither VAD nor
    ASR runs -- but the worker requires a complete config, and an
    unreachable ASR address guarantees nothing is contacted by accident.
    """
    vad = VadConfig(energy_threshold=300, tail_silence_ms=500,
                    min_utterance_ms=200, frame_ms=20, pre_roll_ms=0)
    return TurnLoopConfig(asr_base_url="http://127.0.0.1:1",
                          asr_http_timeout_s=1.0, vad_cfg=vad)


# -- VoiceInAssembler: the happy path -----------------------------------

def test_chunks_assemble_in_order_and_close_at_eos():
    """Three chunks, eos on the last: one utterance, bytes in seq order.

    Also pins that eos CLOSES the stream: a stream left open after eos
    would hold a slot until idle eviction and count against max_streams.
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state: no case leaks into another
    # The first two only add audio; nothing is complete yet.
    assert asm.accept(_chunk(0), 0) is None  # chunk 0 opens the stream
    assert asm.accept(_chunk(1), 10) is None  # chunk 1 appends
    utt = asm.accept(_chunk(2, eos=True), 20)  # the closing chunk
    # The sentence is exactly the three payloads, concatenated in order.
    assert utt == CompletedUtterance("s-1", "cloud",
                                     _pcm(1) + _pcm(2) + _pcm(3))
    # eos closed the stream: nothing is left buffered.
    assert asm.open_streams() == 0  # eos closed it


def test_a_repeated_chunk_is_ignored():
    """A transport retry must not duplicate audio in the sentence.

    Duplicated audio would transcribe as a stutter ("原地原地待命") and
    could miss the keyword; refusing the repeat as an error would instead
    kill a perfectly good sentence. Ignoring it is the only right answer.
    """
    asm = VoiceInAssembler(_LIMITS)
    asm.accept(_chunk(0), 0)  # opens
    asm.accept(_chunk(1), 1)  # appends
    # Chunk 1 again (a retry) -- no error and no effect.
    assert asm.accept(_chunk(1), 2) is None
    utt = asm.accept(_chunk(2, eos=True), 3)  # closes
    # Chunk 1's bytes appear once.
    assert utt.pcm == _pcm(1) + _pcm(2) + _pcm(3)


def test_the_envelope_form_is_accepted():
    """{..., "data": body} works like the bare body (p2's cmd/mode rule).

    A cloud client built against 11 S3.0 wraps every message; refusing the
    wrapped form would make the same client work for one key and not the
    other, with nothing in the log but "bad_field".
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state
    # A single-chunk sentence (2 ms, at the floor), wrapped.
    wrapped = {"schema": "x", "data": _chunk(0, pcm=_pcm(7, 2), eos=True)}
    utt = asm.accept(wrapped, 0)  # the clock value is irrelevant here
    assert utt.pcm == _pcm(7, 2)  # the body inside, not the wrapper


# -- VoiceInAssembler: a broken sentence is never transcribed -----------

def test_a_gap_drops_the_whole_stream():
    """Chunk 2 without chunk 1: refused, and the stream is gone.

    The second half pins that the drop is final: the rest of the sentence
    arriving afterwards must not be assembled into a shorter, different
    sentence -- it is refused as a stream that starts mid-sentence.
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state
    asm.accept(_chunk(0), 0)  # stream open, next expected = 1
    with pytest.raises(VoiceInRejected) as ei:  # must refuse
        asm.accept(_chunk(2), 1)  # chunk 1 never arrived
    assert ei.value.reason == "chunk_gap"
    # Dropped, not paused: the rest of the sentence cannot revive it.
    assert asm.open_streams() == 0  # dropped, not paused
    with pytest.raises(VoiceInRejected) as ei:
        asm.accept(_chunk(3, eos=True), 2)  # the tail of the old sentence
    assert ei.value.reason == "stream_starts_mid_sentence"


def test_a_stream_must_start_at_chunk_zero():
    """The first chunk is the beginning of the sentence.

    A client restarted mid-sentence, or a subscriber that joined late,
    would otherwise transcribe the tail of a command as a whole command.
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state
    with pytest.raises(VoiceInRejected) as ei:
        asm.accept(_chunk(1), 0)  # no chunk 0 was ever seen
    assert ei.value.reason == "stream_starts_mid_sentence"


def test_a_channel_switch_mid_stream_drops_it():
    """One stream is one speaker on one channel.

    The channel picks the session and the B-mode rule, so a stream whose
    channel changes cannot be attributed; both halves are discarded.
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state
    asm.accept(_chunk(0), 0)  # opened on channel cloud
    with pytest.raises(VoiceInRejected) as ei:
        asm.accept(_chunk(1, channel="wecom"), 1)  # was cloud
    assert ei.value.reason == "channel_changed"
    assert asm.open_streams() == 0  # neither half survives


def test_an_over_long_sentence_is_dropped():
    """Past max_utterance_ms (10 ms here) the stream is discarded.

    Checked per chunk, not at eos: a client that never sends eos must not
    grow the buffer without bound.
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state
    # 6 ms + 6 ms = 12 ms > 10 ms on the second chunk.
    asm.accept(_chunk(0, pcm=_pcm(1, 6)), 0)
    with pytest.raises(VoiceInRejected) as ei:
        asm.accept(_chunk(1, pcm=_pcm(2, 6)), 1)  # crosses the ceiling
    assert ei.value.reason == "utterance_too_long"  # the ceiling, not eos
    assert asm.open_streams() == 0  # memory released at once


def test_a_too_short_sentence_is_not_sent_to_asr():
    """Under min_utterance_ms (2 ms here) eos yields a refusal, not audio.

    The local VAD applies the same floor; a click or a key-press sent as a
    sentence transcribes as a random syllable that could match a keyword.
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state
    with pytest.raises(VoiceInRejected) as ei:
        asm.accept(_chunk(0, pcm=_pcm(1, 1), eos=True), 0)  # 1 ms only
    assert ei.value.reason == "utterance_too_short"
    # Closed either way: a short sentence does not linger.
    assert asm.open_streams() == 0


# -- VoiceInAssembler: format and field refusals -------------------------

@pytest.mark.parametrize("override,reason", [
    ({"codec": "opus"}, "bad_codec"),              # pre-U92 example codec
    ({"sample_rate": 48000}, "bad_sample_rate"),   # HK-MIC native rate
    ({"channels": 2}, "bad_channels"),             # stereo
    ({"channel": "hmi"}, "bad_channel"),           # hmi is text-only
    ({"chunk_seq": True}, "bad_field"),            # bool is not a seq
    ({"eos": "yes"}, "bad_field"),                 # eos must be a bool
    ({"stream_id": ""}, "bad_field"),              # empty id
    ({"payload_b64": "!!not-b64!!"}, "bad_payload"),  # outside the alphabet
    ({"payload_b64": base64.b64encode(b"abc").decode()},  # 3 bytes:
     "odd_pcm_bytes"),                                     # half a sample
])
def test_out_of_format_chunks_are_refused(override, reason):
    """Each field outside the contract has its own reason token.

    Refused before any stream is created, so a malformed first chunk leaves
    nothing behind. The token is what the operator greps for: "bad_codec"
    says "your client sends Opus", which "bad_field" alone would not.
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state per parameter
    with pytest.raises(VoiceInRejected) as ei:
        asm.accept(_chunk(0, **override), 0)  # one field broken
    assert ei.value.reason == reason
    assert asm.open_streams() == 0  # nothing was opened for it


# -- VoiceInAssembler: resource bounds -----------------------------------

def test_an_idle_stream_is_evicted():
    """No chunk for longer than stream_idle_ms: the stream is reclaimed.

    The two probes straddle the bound, so both "evicts too early" and
    "never evicts" turn this red.
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state
    asm.accept(_chunk(0), 0)  # last chunk at t = 0
    # Exactly at the bound it survives; one ms past it, it is gone.
    assert asm.evict_idle(100) == []
    assert asm.evict_idle(101) == ["s-1"]
    assert asm.open_streams() == 0


def test_the_open_stream_ceiling_holds():
    """A third stream beyond max_streams=2 is refused; the two stay.

    The newcomer is refused rather than an open stream evicted: a sentence
    half spoken is worth more than one not yet begun.
    """
    asm = VoiceInAssembler(_LIMITS)  # fresh state
    asm.accept(_chunk(0, sid="a"), 0)  # stream a, open
    asm.accept(_chunk(0, sid="b"), 0)  # stream b, open: the ceiling is reached
    with pytest.raises(VoiceInRejected) as ei:
        asm.accept(_chunk(0, sid="c"), 0)  # one too many
    assert ei.value.reason == "too_many_streams"
    assert asm.open_streams() == 2  # a and b untouched


# -- CloudTurnRunner ------------------------------------------------------

class _FakePost:
    """A 16 S3 post-processor that rewrites by a fixed table.

    Lets a case decide exactly what the "corrected" text is, so the raw and
    corrected forms can be made to disagree on purpose.
    """

    def __init__(self, table):
        self._t = table  # raw transcript -> corrected text

    def apply(self, raw):
        """Same (text, notes) shape as the real AsrPostProcessor.apply."""
        return self._t.get(raw, raw), []  # unknown text passes unchanged


def _runner(transcript="", mode=None, post=None, local=None):
    """A runner over the production orchestrator, ASR faked.

    Returns (runner, published, asr_calls). `published` collects (key,
    bytes) exactly as main_wiring's _publish would receive them; asr_calls
    records every PCM buffer handed to the fake ASR, which answers with the
    scripted `transcript`. `mode` is what p2's state/mode would report.
    `local` is the microphone's session (main_wiring passes the real one);
    a case that inspects it passes its own, every other case gets a fresh one.
    """
    with open(_INTENTS, encoding="utf-8") as fh:  # the files p4 loads in production
        reg = load_intent_registry(yaml.safe_load(fh))
    with open(_CHITCHAT, encoding="utf-8") as fh:  # chitchat presets
        cc = ChitchatResponder(yaml.safe_load(fh))
    orch = build_orchestrator(reg, cc, l2_timeout_ms=8000)  # as p4 builds it
    published, asr_calls = [], []  # what the fakes record

    def _asr(pcm):
        """The fake ASR: record the audio, answer with the script."""
        asr_calls.append(pcm)
        return transcript

    runner = CloudTurnRunner(orch, KeywordMatcher(reg),
                             lambda k, d: published.append((k, d)), _asr,
                             lambda: 1000, lambda: mode,
                             local or OrchestratorSession(), asr_post=post)
    return runner, published, asr_calls


def _utt(channel="cloud"):
    """A completed 4 ms sentence on `channel` (above the 2 ms floor)."""
    return CompletedUtterance("s-9", channel, _pcm(5, 4))


def test_typed_text_runs_the_chain_and_publishes():
    """"原地待命" typed from the cloud -> A04 on cmd/motion/intent.

    The publish check is the point: a runner that classified correctly but
    never handed the frames to the publisher would look fine in the log
    and leave the robot standing.
    """
    runner, published, _ = _runner()  # no transcript needed: typed
    res = runner.run_text({"cmd_id": "c-1", "channel": "cloud",
                           "text": "原地待命", "require_tts_reply": False})
    assert res.decision.intent_id == "A04"  # keyword hit at layer 2
    # What the runner returned is exactly what reached the publisher.
    assert published == res.publishes
    assert [k for k, _ in published] == ["cmd/motion/intent"]


@pytest.mark.parametrize("bad", [
    {"channel": "cloud", "text": ""},            # empty text
    {"channel": "cloud", "text": 42},            # not a string
    {"channel": "pigeon", "text": "原地待命"},    # channel outside the set
    ["not", "an", "object"],                     # not even an object
])
def test_malformed_text_is_refused_without_a_turn(bad):
    """Refused and logged; nothing published, no turn counted.

    The int case is the one handle_text_command would not survive on its
    own (it strips the text without a type check).
    """
    runner, published, _ = _runner()  # each bad body gets its own runner
    assert runner.run_text(bad) is None
    assert published == [] and runner.text_turns == 0  # no frame, no count


def test_cloud_audio_is_transcribed_corrected_and_classified():
    """ASR -> 16 S3 post -> orchestrator; the ASR got the exact sentence.

    "原地代命" (代 misheard for 待) matches nothing; only the corrected
    text classifies as A04, so a runner skipping post-processing fails.
    """
    post = _FakePost({"原地代命": "原地待命"})  # 代 is a sound-alike of 待
    runner, published, asr_calls = _runner("原地代命", post=post)
    res = runner.run_voice(_utt())  # one completed sentence
    # The corrected text is what was classified.
    assert res.decision.intent_id == "A04"
    assert asr_calls == [_pcm(5, 4)]  # the whole sentence, once
    assert published == res.publishes


def test_the_raw_transcript_reaches_the_bypass():
    """Post-processing hides the stop word; the raw transcript still stops.

    16 S4 constraint 1: the bypass matches what ASR actually heard first,
    because a corrector must never be able to eat an emergency stop.
    """
    post = _FakePost({"急停": "今天天气怎么样"})  # a corrector gone wrong
    runner, _, _ = _runner("急停", post=post)  # the stop word only in the raw form
    assert runner.run_voice(_utt()).decision.kind == "bypass"


def test_cloud_audio_is_not_addressed():
    """A far-end microphone meets the overheard gate like the local one.

    The same sentence TYPED is answered (test_text_addressed); heard
    through the cloud microphone it may be a bystander, so it is silent.
    """
    runner, published, _ = _runner("今天天气怎么样")
    assert runner.run_voice(_utt()).decision.kind == "overheard"  # layer 5 silences it
    # Overheard is silent: nothing is published.
    assert published == []


def test_broadcast_drops_cloud_audio_before_asr():
    """11 S8.7.4 VOI-05: B mode, channel=cloud -> never into ASR.

    asr_calls == [] is the strong form: not "transcribed and ignored" but
    never transcribed, so the speaker can never command itself.
    """
    runner, published, asr_calls = _runner("原地待命", mode="broadcast")
    assert runner.run_voice(_utt("cloud")) is None  # refused before ASR
    assert asr_calls == [] and published == []
    assert runner.dropped_in_broadcast == 1  # counted for telemetry


def test_broadcast_still_hears_wecom_audio():
    """VOI-24: WeChat voice is the legal way out of B mode.

    Without this case a rule that dropped ALL audio in B mode would pass
    the case above -- and lock the robot in broadcast.
    """
    runner, _, asr_calls = _runner("原地待命", mode="broadcast")  # p2 reports broadcast
    assert runner.run_voice(_utt("wecom")).decision.intent_id == "A04"
    assert len(asr_calls) == 1  # it did reach ASR


def test_each_channel_has_its_own_session():
    """A confirm opened on cloud is not answered by a wecom "确认".

    B07 "取消任务" opens an L2 confirm. With one shared session the wecom
    turn would consume it and the cloud "确认" afterwards would find nothing
    to confirm -- one operator confirming another's command.
    """
    runner, _, _ = _runner()  # one runner: the sessions live in it

    def say(channel, text):
        """One typed turn on `channel`, reply muted; its decision."""
        return runner.run_text({"channel": channel, "text": text,
                                "require_tts_reply": False}).decision

    assert say("cloud", "取消任务").kind == "await_confirm"
    # wecom has no open confirm: its "确认" is not a confirmation of B07.
    assert say("wecom", "确认").intent_id != "B07"
    # The cloud confirm is still open and resolves to B07 now.
    assert say("cloud", "确认").intent_id == "B07"


def test_hmi_text_shares_the_local_microphone_session():
    """The HMI is an on-site input: its turns live in the local slot.

    16 S11.1 has three slots (local / cloud / wecom). A confirm the HMI
    opens is therefore held by the microphone's session -- so an operator
    can tap a command and say "确认" -- and NOT by the cloud slot.
    """
    local = OrchestratorSession()  # stands in for the microphone's session
    runner, _, _ = _runner(local=local)
    res = runner.run_text({"channel": "hmi", "text": "取消任务",
                           "require_tts_reply": False})
    assert res.decision.kind == "await_confirm"  # B07 asks for a confirm
    assert local.pending_confirm is not None  # held by the local slot
    # The cloud slot saw nothing: its own "确认" confirms no B07.
    assert runner.run_text({"channel": "cloud", "text": "确认",
                            "require_tts_reply": False}
                           ).decision.intent_id != "B07"


# -- worker and callbacks ------------------------------------------------

def test_the_worker_runs_jobs_and_survives_a_failing_one():
    """Two jobs: the first raises, the second still runs.

    A cloud turn that crashes (an LLM client bug, a malformed decision)
    must cost that one turn, not every later turn, local ones included.
    """
    jobs, done, stop = queue.Queue(), threading.Event(), threading.Event()

    def boom():
        """A turn that fails; the loop must outlive it."""
        raise RuntimeError("boom")

    jobs.put(boom)  # the first job: it raises
    jobs.put(done.set)  # the second job: proves the loop went on
    worker = TurnLoopWorker(_cfg(), queue.Queue(), lambda k, d: None, stop,
                            job_queue=jobs)
    worker.start()  # the job branch is live
    try:
        assert done.wait(2.0), "second job never ran"
    finally:
        stop.set()  # always stop the thread, even on a red assert
        worker.join(2.0)  # bounded: a hung worker fails the case
    # The failure was recorded, not swallowed silently.
    assert any("boom" in e for e in worker.errors)


def test_the_worker_without_jobs_is_unchanged():
    """No job queue (the V-2B smoke path): the worker still starts/stops.

    The job queue is optional; a worker built the old way must not fail on
    the job branch it does not have.
    """
    stop = threading.Event()
    worker = TurnLoopWorker(_cfg(), queue.Queue(), lambda k, d: None, stop)
    worker.start()
    stop.set()  # ask it to stop at once
    worker.join(2.0)  # bounded wait
    assert not worker.is_alive()  # exited cleanly, no exception


class _Sample:
    """Stands in for a zenoh Sample: only .payload is read."""

    def __init__(self, body):
        self.payload = json.dumps(body).encode("utf-8")  # wire form


class _SpyRunner:
    """Records which utterances were run, instead of running them."""

    def __init__(self):
        self.ran = []  # CompletedUtterance values, in run order

    def run_voice(self, utt):
        """The one method the voice_in callback's job calls."""
        self.ran.append(utt)


def test_the_voice_in_callback_queues_a_turn_only_at_eos():
    """Chunks before eos queue nothing; eos queues exactly one turn.

    One job per chunk would run ASR on every 100 ms piece; one per
    sentence is the contract (eos "触发整句 ASR").
    """
    jobs, spy = queue.Queue(), _SpyRunner()  # a spy, not a runner: no orchestrator
    cb = make_voice_in_callback(jobs, VoiceInAssembler(_LIMITS), spy,
                                lambda: 0)
    cb(_Sample(_chunk(0)))  # chunk 0: buffered
    assert jobs.empty()  # mid-sentence: nothing to run yet
    cb(_Sample(_chunk(1, eos=True)))  # chunk 1 with eos: the sentence closes
    # One job; running it hands the whole sentence to the runner.
    jobs.get_nowait()()
    assert jobs.empty()
    assert [u.pcm for u in spy.ran] == [_pcm(1) + _pcm(2)]


def test_a_full_job_queue_refuses_the_new_turn():
    """The queued (older) turn stays; the new one is dropped and logged.

    Drop-oldest would run commands out of the order they were given.
    """
    jobs = queue.Queue(maxsize=1)  # room for exactly one job
    older = object  # any distinct marker stands in for the queued job
    jobs.put(older)  # the queue is now full
    _enqueue(jobs, lambda: None, "voice_text")  # the queue is full
    # Still exactly the older job: order of commands is preserved.
    assert jobs.get_nowait() is older and jobs.empty()


def test_the_keys_are_robot_prefixed():
    """11 S2.2: both keys live under xbrain/{rid}/ (cloud-published).

    A bare key ("cmd/voice_text") would subscribe the in-robot namespace,
    where nothing from the cloud ever arrives.
    """
    assert cloud_input_keys("gj-001") == ("xbrain/gj-001/cmd/voice_text",
                                          "xbrain/gj-001/audio/voice_in")
