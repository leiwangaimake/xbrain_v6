"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cloud_input.py
Brief: p4 cloud inputs -- cmd/voice_text (11 S8.7.5) and audio/voice_in (11 S8.7.3) into the turn loop

Description:
Two contract keys bring commands into p4 from outside the robot's own
microphone. 11 S2.2 registers p4 as the only subscriber of both, and until
2026-10-25 neither had a subscriber at all:

  xbrain/{rid}/cmd/voice_text   typed text (cloud T / WeChat E / HMI), no ASR
  xbrain/{rid}/audio/voice_in   cloud (C) / WeChat (E) speech, one sentence
                                per stream, pcm_s16le 16 kHz mono (99 U92),
                                the last chunk carries eos=true

The trigger was practical: the robot's own microphone sits inside a closed
housing and hears almost nothing, so the operator speaks into a microphone on
a debugging PC (/opt/commx) and sends the sentence over the party-A 188
network. That client is only the first user; the cloud and the WeChat bot
use exactly the same two keys.

Three pieces, split so each can be tested without Zenoh:

  * VoiceInAssembler -- pure state. Collects the chunks of each stream in
    order and hands back one CompletedUtterance at eos. Every rule that can
    reject a chunk is here, each with a reason token that reaches the log.
  * CloudTurnRunner -- runs one turn: typed text straight into
    handle_text_command; a completed utterance through ASR + 16 S3
    post-processing first, then the same function. One OrchestratorSession
    per channel token (16 S11.1: the session slot follows the channel).
  * make_*_callback -- the Zenoh callbacks. They only decode, assemble and
    enqueue a job (CLAUDE.md 4.2: no HTTP on the Rust thread pool); the job
    runs on the TurnLoopWorker, the same thread that runs microphone turns,
    so a cloud turn and a local turn never touch the orchestrator at once.

Rules that are easy to get wrong and are deliberate here:
  * A chunk gap DROPS the stream. Transcribing a sentence with a hole in it
    yields a different sentence -- possibly a different command -- so the
    safe outcome of a lost chunk is "nothing happened", logged.
  * A stream must start at chunk_seq 0 for the same reason: the first chunk
    is the beginning of the sentence.
  * A repeated chunk (seq below the next expected) is ignored, not an error:
    a reliable transport may still deliver a retry twice.
  * voice_mode == broadcast drops channel=cloud audio before ASR (11 S8.7.4
    VOI-05): during B mode the cloud is supposed to send that microphone to
    audio/broadcast, and a stale client that keeps sending voice_in must not
    get "停止喊话" recognised as a command by the speaker it is driving.
    channel=wecom is still processed -- it is the legal way out of B mode.
  * Typed text is "addressed" (16 S5.2.1 implementation note): no wake word
    needed. Cloud AUDIO is not -- it is a microphone at the far end.

Boundaries: the domain-3 ASR arbitration between channels (11 S8.7.3 says
the channel decides the domain-3 claimant) and the C > E > A session
priority are not implemented; turns are simply serialised in arrival order
on the one worker. The state/voice_turn audit record is not published by p4
at all (a known gap, not introduced here); outcomes are logged instead.
"""
from __future__ import annotations

import base64  # payload_b64 of every voice_in chunk
import binascii  # base64's error type, caught to refuse a bad payload
import json  # both keys carry JSON bodies (11 S3.0)
import logging  # the log is the only record of a cloud turn today
import queue  # the worker's job queue (Full is the refusal signal)
import threading  # the assembler lock: callbacks run on many threads
from dataclasses import dataclass  # frozen value types crossing threads
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

# The keyword matcher is only a type here: the runner receives the instance
# the wiring built, the same one channel admission pre-classifies with.
from xbrain.p4_agent.classifier.keyword_matcher import KeywordMatcher
# The text entry is reused for audio on purpose: channel admission and the
# require_tts_reply rule must be one implementation, not one per input.
from xbrain.p4_agent.runtime.text_channel import (
    TextChannelError,  # a channel outside cloud/wecom/hmi
    TextTurnResult,  # what one turn produced: decision + frames
    handle_text_command,  # admission gate + orchestrator, no ASR
)
# Sessions are created here (one per channel); the orchestrator is passed in.
from xbrain.p4_agent.runtime.turn_orchestrator import (
    OrchestratorSession,  # per-dialog state: pending confirm, recording
    TurnOrchestrator,  # the six-step chain shared with the microphone
)

# One logger for both inputs: an operator debugging "the cloud said nothing
# happened" greps one name and sees text and audio turns interleaved in the
# order they ran, which is the order that matters.
_logger = logging.getLogger("xbrain.p4.cloud_input")  # one name, both keys

#: 11 S2.2 key templates (%s = robot id). These are the robot-PREFIXED keys:
#: the cloud, the WeChat bot and the debugging PC publish them directly on the
#: GEN router. p5 translates other cloud keys into bare in-robot keys, but not
#: these two -- 11 S2.2 names p4 as their subscriber, and an extra hop through
#: p5 would be one more place the two audio paths (RT-A3) could be joined.
VOICE_TEXT_KEY_FMT = "xbrain/%s/cmd/voice_text"  # 11 S2.2 Q3 event
VOICE_IN_KEY_FMT = "xbrain/%s/audio/voice_in"  # 11 S2.2 Q4 stream

#: 11 S8.7.3: voice_in.channel is binary. hmi is a TEXT channel only -- the
#: HMI has no audio uplink, so an hmi-labelled audio stream is a client bug.
VOICE_IN_CHANNELS = frozenset({"cloud", "wecom"})  # C mode, E mode
#: 99 U92 (2026-10-25): the one accepted audio format. Anything else is
#: refused rather than resampled -- a mislabelled stream would otherwise be
#: transcribed as noise and the operator would blame the ASR.
VOICE_IN_CODEC = "pcm_s16le"  # signed 16-bit little-endian, raw
VOICE_IN_SAMPLE_RATE = 16000  # Hz; what the ASR service is fed for the mic
VOICE_IN_CHANNEL_COUNT = 1  # mono: one speaker, one microphone
#: Bytes of pcm_s16le mono audio per millisecond at 16 kHz: 16 samples of 2
#: bytes. Derived from the two constants above so the limits below cannot
#: silently disagree with the format if the format is ever changed.
_BYTES_PER_MS = VOICE_IN_SAMPLE_RATE * 2 // 1000  # = 32

#: Channel tokens that own a session slot of their own (16 S11.1 has exactly
#: three slots: local / cloud / wecom). voice_in shares the slot of its
#: channel token: a cloud operator who types and then speaks is one dialog,
#: so a confirm asked by text can be answered by voice. The third text token,
#: hmi, is NOT here: the HMI is the on-site terminal, so it uses the LOCAL
#: slot -- the microphone's session -- which is also what 11 S8.7.6 implies
#: (state/voice_turn.session is local|cloud|wecom while channel may be hmi).
_SESSION_CHANNELS = ("cloud", "wecom")  # hmi -> the local slot


class VoiceInRejected(ValueError):
    """One voice_in chunk was refused; `reason` is a stable log token.

    The reason is a short fixed token so the log can be counted by it ("how
    many chunk_gap today?"); the detail carries the offending value for the
    person reading one line. Only the token is part of the tested contract.
    """

    def __init__(self, reason: str, detail: str = "") -> None:
        # str(exc) reads "reason: detail" -- what the callback logs verbatim.
        super().__init__("%s%s" % (reason, (": " + detail) if detail else ""))
        self.reason = reason  # the token alone, for tests and counters


@dataclass(frozen=True)
class VoiceInLimits:
    """Resource bounds for voice_in assembly. All required, from __main__.

    They come from the p4 command line, next to the VAD settings, because
    they are the cloud-side twin of those settings: how long a sentence may
    be and how short is too short to be speech. None is a safety value --
    they bound memory and decide what reaches ASR, never how the robot moves.

    max_utterance_ms -- a stream longer than this is dropped: the contract is
      one sentence per stream, and an unbounded buffer is a memory hazard.
    min_utterance_ms -- shorter than this is not sent to ASR (the local VAD
      uses the same floor; a click transcribes as a random syllable).
    stream_idle_ms   -- a stream with no chunk for this long is evicted, so a
      client that died mid-sentence does not hold a slot forever.
    max_streams      -- open streams at once; a new one beyond this is refused.
    """
    max_utterance_ms: int  # --voice-in-max-utterance-ms
    min_utterance_ms: int  # --vad-min-utterance-ms (shared floor)
    stream_idle_ms: int  # --voice-in-stream-idle-ms
    max_streams: int  # --voice-in-max-streams


@dataclass(frozen=True)
class CompletedUtterance:
    """One whole sentence from voice_in, ready for ASR.

    Frozen: it crosses from the Zenoh thread to the worker thread inside a
    job, and nothing on either side should be able to change it in flight.
    """
    stream_id: str   # the client's id, kept for the log line of the turn
    channel: str     # cloud | wecom -- picks the session and the B-mode rule
    pcm: bytes       # pcm_s16le 16 kHz mono, all chunks in order


@dataclass
class _Stream:
    """Assembly state of one open stream (internal, guarded by the lock)."""
    channel: str          # fixed by chunk 0; a later change drops the stream
    next_seq: int         # the only chunk_seq that may be appended next
    last_rx_ms: int       # monotonic ms of the newest chunk (idle eviction)
    chunks: List[bytes]   # payloads in order; joined once at eos
    size: int             # running byte total, checked against the ceiling


def _unwrap(msg: Any) -> Any:
    """Accept the bare body or an 11 S3.0 envelope {..., "data": {...}}.

    The same tolerance p2 applies to cmd/mode: a client that wraps its
    payload and one that does not both work, without a second code path.
    Anything that is not an object is passed through and refused by the
    caller, which knows which input it was and logs accordingly.
    """
    # Only a dict "data" counts as an envelope; a body that happens to carry
    # a scalar field named data is still treated as the bare body.
    if isinstance(msg, dict) and isinstance(msg.get("data"), dict):
        return msg["data"]  # the envelope's body
    return msg  # already bare (or not an object: the caller refuses it)


def _require(msg: Mapping[str, Any], name: str, typ: type) -> Any:
    """Field `name` of type `typ`, or VoiceInRejected("bad_field").

    bool is excluded where int is asked for: in Python True is an int, and a
    chunk_seq of True would otherwise be accepted as 1 -- a client bug that
    would surface as a confusing chunk_gap instead of a plain field error.
    """
    value = msg.get(name)
    # A missing field reads as None, which no asked-for type matches.
    if not isinstance(value, typ) or (typ is int and isinstance(value, bool)):
        raise VoiceInRejected("bad_field", "%s=%r" % (name, value))
    return value  # typed as asked; the caller checks the value range


class VoiceInAssembler:
    """Collects audio/voice_in chunks per stream_id; emits a sentence at eos.

    Thread-safe: Zenoh may run callbacks on several Rust threads, so every
    entry point takes the one lock. All times are monotonic ms (CLK-C1),
    passed in by the caller so the tests control the clock and no wall-clock
    jump (an NTP step on the robot) can evict a live stream.
    """

    def __init__(self, limits: VoiceInLimits) -> None:
        # Limits are fixed for the life of the process (from the CLI).
        self._limits = limits  # read-only after construction
        # stream_id -> assembly state of that open stream.
        self._streams: Dict[str, _Stream] = {}  # only mutated under _lock
        # One lock for the dict and every _Stream in it; the work done under
        # it is a few comparisons and an append, never I/O.
        self._lock = threading.Lock()  # not re-entrant: no nested entry
        # Byte bounds derived once from the duration bounds.
        self._max_bytes = limits.max_utterance_ms * _BYTES_PER_MS  # ceiling
        self._min_bytes = limits.min_utterance_ms * _BYTES_PER_MS  # floor

    def open_streams(self) -> int:
        """How many streams are mid-sentence (heartbeat log, tests).

        Taken under the lock like everything else: a count read while a
        callback is opening or closing a stream could be off by one, and a
        test asserting "nothing left buffered" must not race that.
        """
        with self._lock:
            return len(self._streams)  # a snapshot; may change right after

    def evict_idle(self, now_mono_ms: int) -> List[str]:
        """Drop every stream idle for longer than stream_idle_ms.

        Returns the evicted stream ids so the caller can log them. Called on
        every chunk and from the wiring heartbeat, so a dead client's stream
        is reclaimed even when no other audio arrives.
        """
        with self._lock:
            return self._evict_locked(now_mono_ms)  # same rule as per chunk

    def _evict_locked(self, now_mono_ms: int) -> List[str]:
        """evict_idle with the lock already held by the caller."""
        # Strictly greater: a stream exactly stream_idle_ms old survives, so
        # the bound reads as "may be silent this long", not "this long minus
        # one". Collect first, then delete: never mutate a dict mid-iteration.
        stale = [sid for sid, st in self._streams.items()
                 if now_mono_ms - st.last_rx_ms > self._limits.stream_idle_ms]
        for sid in stale:
            del self._streams[sid]  # its audio is discarded, never transcribed
        return stale  # ids only: the caller logs them, there is nothing else

    def accept(self, raw: Any,
               now_mono_ms: int) -> Optional[CompletedUtterance]:
        """Take one decoded voice_in message.

        Returns the CompletedUtterance when this chunk closes its stream
        (eos), None when more chunks are expected or the chunk was a repeat.
        Raises VoiceInRejected for every refusal; a refusal that concerns an
        open stream also discards that stream (see the module rules).
        """
        msg = _unwrap(raw)  # bare body or 11 S3.0 envelope
        if not isinstance(msg, Mapping):
            # A JSON array or scalar: there is no field to even look at.
            raise VoiceInRejected("not_an_object")  # no detail to give
        # Field checks first, all of them, so a malformed chunk never touches
        # a stream: a client sending garbage cannot damage a good sentence
        # that another client is in the middle of.
        stream_id = _require(msg, "stream_id", str)
        if not stream_id:  # "" would make every client share one stream
            raise VoiceInRejected("bad_field", "stream_id empty")
        seq = _require(msg, "chunk_seq", int)  # 0, 1, 2 ... per stream
        eos = _require(msg, "eos", bool)  # contract: required, a real bool
        channel = _require(msg, "channel", str)  # decides session + B rule
        if channel not in VOICE_IN_CHANNELS:  # hmi has no audio uplink
            raise VoiceInRejected("bad_channel", channel)
        # Format: exactly the one U92 format, field by field, each with its
        # own token so the log says WHICH field the client got wrong.
        if msg.get("codec") != VOICE_IN_CODEC:  # e.g. the old "opus" example
            raise VoiceInRejected("bad_codec", repr(msg.get("codec")))
        if msg.get("sample_rate") != VOICE_IN_SAMPLE_RATE:  # e.g. 48 kHz raw
            raise VoiceInRejected("bad_sample_rate",  # not resampled here
                                  repr(msg.get("sample_rate")))
        if msg.get("channels") != VOICE_IN_CHANNEL_COUNT:  # stereo capture
            raise VoiceInRejected("bad_channels", repr(msg.get("channels")))
        # Decoding is outside the lock: it is the only step whose cost grows
        # with the chunk, and it needs no shared state.
        pcm = self._decode_payload(msg)  # may raise bad_payload / odd bytes

        with self._lock:
            # Reclaim dead streams before counting the open ones, so a
            # crashed client cannot hold the ceiling against a live one.
            self._evict_locked(now_mono_ms)
            st = self._streams.get(stream_id)  # None: not open (or dropped)
            if st is None:
                # First chunk of a new stream (or of one already dropped).
                st = self._open_locked(stream_id, seq, channel, now_mono_ms)
            else:
                if channel != st.channel:
                    # One stream is one speaker on one channel; a switch
                    # mid-sentence means two clients reuse an id.
                    del self._streams[stream_id]  # neither half is trusted
                    raise VoiceInRejected("channel_changed", stream_id)
                if seq < st.next_seq:
                    # A retry of a chunk we already have: harmless, and
                    # deliberately not an error that would kill the stream.
                    return None
                if seq > st.next_seq:
                    # A hole: the sentence can no longer be transcribed
                    # faithfully, so none of it is.
                    del self._streams[stream_id]  # later chunks hit seq!=0
                    raise VoiceInRejected(
                        "chunk_gap", "%s want %d got %d"  # what was lost
                        % (stream_id, st.next_seq, seq))
            # In order: append and advance.
            st.chunks.append(pcm)  # kept as pieces; joined once at eos
            st.size += len(pcm)  # the running total the ceiling checks
            st.next_seq = seq + 1  # the only seq accepted after this one
            st.last_rx_ms = now_mono_ms  # a live stream is never idle-evicted
            if st.size > self._max_bytes:
                # Checked on every chunk, not only at eos: a client that never
                # sends eos must not be able to grow the buffer without end.
                del self._streams[stream_id]  # memory released at once
                raise VoiceInRejected("utterance_too_long", stream_id)
            if not eos:
                return None  # more of this sentence is coming
            # eos: the sentence is complete and the stream ends here, whether
            # or not it is long enough to be worth transcribing.
            del self._streams[stream_id]  # st stays usable: a local ref
            if st.size < self._min_bytes:  # same floor as the local VAD
                raise VoiceInRejected("utterance_too_short",  # not to ASR
                                      "%s %d bytes" % (stream_id, st.size))
            # Joined once, at the end: appending to bytes per chunk would
            # copy the whole sentence on every chunk.
            return CompletedUtterance(stream_id=stream_id, channel=channel,
                                      pcm=b"".join(st.chunks))  # in order

    def _open_locked(self, stream_id: str, seq: int, channel: str,
                     now_mono_ms: int) -> _Stream:
        """Start a new stream, or refuse it (lock held by the caller)."""
        if seq != 0:
            # The beginning of the sentence is missing (see module rules).
            # This is also what a chunk of an already-dropped stream hits, so
            # a gap or a channel switch cannot be "healed" by later chunks.
            raise VoiceInRejected("stream_starts_mid_sentence",
                                  "%s seq %d" % (stream_id, seq))
        if len(self._streams) >= self._limits.max_streams:
            # Refuse the newcomer rather than evict a stream in progress: a
            # sentence half spoken is worth more than one not yet begun.
            raise VoiceInRejected("too_many_streams", stream_id)
        # Empty on creation: the caller appends chunk 0 like any other chunk,
        # so the first chunk goes through the same size and eos checks.
        st = _Stream(channel=channel, next_seq=0, last_rx_ms=now_mono_ms,
                     chunks=[], size=0)
        self._streams[stream_id] = st  # registered before chunk 0 is added
        return st

    @staticmethod
    def _decode_payload(msg: Mapping[str, Any]) -> bytes:
        """payload_b64 -> PCM bytes; s16 samples are 2 bytes, so odd is bad.

        An empty payload is legal (an eos-only closing chunk carries none).
        validate=True makes non-alphabet characters an error instead of being
        skipped, so a corrupted payload is refused rather than half-decoded.
        """
        b64 = _require(msg, "payload_b64", str)  # "" is legal: no audio
        try:
            pcm = base64.b64decode(b64, validate=True)  # strict alphabet
        except (binascii.Error, ValueError) as exc:  # padding / alphabet
            raise VoiceInRejected("bad_payload", str(exc)) from exc
        if len(pcm) % 2:
            # Half a sample: every following sample would be misaligned.
            raise VoiceInRejected("odd_pcm_bytes", str(len(pcm)))
        return pcm  # raw little-endian s16 samples, ready to append


class CloudTurnRunner:
    """Runs one cloud turn on the worker thread and publishes its result.

    Holds the cloud and wecom OrchestratorSession slots and borrows the
    local one (the microphone's) for hmi. The orchestrator is the instance
    the microphone path uses; sharing it and the local session is safe
    because every turn, local or cloud, runs on the one TurnLoopWorker
    thread.
    """

    def __init__(self, orchestrator: TurnOrchestrator,
                 matcher: KeywordMatcher,
                 publish_fn: Callable[[str, bytes], None],
                 transcribe_fn: Callable[[bytes], str],
                 now_ms_fn: Callable[[], int],
                 voice_mode_fn: Callable[[], Optional[str]],
                 local_session: OrchestratorSession,
                 asr_post: Any = None) -> None:
        # Every dependency is injected so the tests run the real orchestrator
        # with a fake ASR, a fixed clock and a scripted voice mode.
        self._orch = orchestrator
        self._matcher = matcher            # channel-admission pre-classify
        self._publish = publish_fn         # (key, bytes) -> GEN publisher
        self._transcribe = transcribe_fn   # PCM bytes -> raw text; may raise
        self._now_ms = now_ms_fn           # monotonic ms (CLK-C1)
        self._voice_mode = voice_mode_fn   # p2's voice_mode, None if unknown
        self._asr_post = asr_post          # 16 S3: apply(raw)->(text, notes)
        # The session slots, created once: a slot must outlive a turn, or a
        # confirm asked in one turn could never be answered in the next.
        self._sessions = {ch: OrchestratorSession() for ch in _SESSION_CHANNELS}
        # hmi -> the microphone's slot: one on-site dialog, whichever of the
        # two on-site inputs (voice or the HMI keyboard) a turn came from.
        self._sessions["hmi"] = local_session
        # Counters for the heartbeat log and the tests.
        self.text_turns = 0  # cmd/voice_text turns that reached handle_text
        self.voice_turns = 0  # voice_in sentences that reached handle_text
        self.dropped_in_broadcast = 0  # 11 S8.11 telemetry wants this count

    def run_text(self, raw: Any) -> Optional[TextTurnResult]:
        """One cmd/voice_text message -> orchestrator -> publish.

        Returns the result, or None when the message was refused (logged).
        The checks here are the ones handle_text_command does not make with a
        clean error: it strips text without a type check, and an int text
        would surface as an AttributeError deep in the worker.
        """
        msg = _unwrap(raw)  # bare body or 11 S3.0 envelope
        if not isinstance(msg, Mapping):  # a JSON array or scalar
            _logger.warning("voice_text rejected: not an object")
            return None
        text = msg.get("text")  # 11 S8.7.5: required, a string
        if not isinstance(text, str) or not text.strip():  # blank = nothing
            _logger.warning("voice_text rejected: text=%r", text)
            return None
        channel = msg.get("channel")  # validated by handle_text_command
        try:
            # Typed text: addressed by construction, no raw ASR text.
            res = handle_text_command(msg, self._orch,
                                      self._session_for(channel),
                                      self._matcher, self._now_ms())
        except TextChannelError as exc:
            # Channel outside cloud/wecom/hmi: refused before any turn ran.
            _logger.warning("voice_text rejected: %s", exc)
            return None
        self.text_turns += 1
        # cmd_id is the sender's reference; it goes into the log line so a
        # cloud-side ledger entry can be matched to what the robot did.
        self._emit("voice_text", channel, msg.get("cmd_id"), text, res)
        return res

    def run_voice(self, utt: CompletedUtterance) -> Optional[TextTurnResult]:
        """One completed voice_in sentence -> ASR -> post -> orchestrator.

        Returns the result, or None when nothing was run (B-mode drop, ASR
        failure, empty transcript) -- each logged with its reason.
        """
        # The B-mode rule is checked here, at the start of the turn, rather
        # than when the first chunk arrived: the mode at the moment ASR would
        # run is the one that decides whether the speaker can hear itself.
        if utt.channel == "cloud" and self._voice_mode() == "broadcast":
            # 11 S8.7.4 VOI-05: never into ASR while broadcasting. The reason
            # token is the one 11 S8.11 registers for this event.
            self.dropped_in_broadcast += 1
            _logger.warning(
                "voice_in dropped: channel=cloud during broadcast "
                "(reason=voice_in_during_broadcast stream=%s total=%d)",
                utt.stream_id, self.dropped_in_broadcast)
            return None
        try:
            raw_text = (self._transcribe(utt.pcm) or "").strip()  # None -> ""
        except Exception as exc:      # noqa: BLE001 -- one turn must not kill the loop
            # ASR down or timed out: this sentence is lost, the next is not.
            _logger.warning("voice_in asr failed stream=%s: %s",
                            utt.stream_id, exc)
            return None
        if not raw_text:
            # Silence or noise: legal ASR output, nothing to act on.
            _logger.info("voice_in empty transcript stream=%s",
                         utt.stream_id)
            return None
        text = raw_text  # unchanged when no post-processor is configured
        if self._asr_post is not None:
            # 16 S3 three-layer correction applies to speech, not to text:
            # it repairs sound-alike errors, which typed text does not have.
            text, _notes = self._asr_post.apply(raw_text)  # notes: logged there
        # A spoken command expects a spoken answer: require_tts_reply true.
        msg = {"channel": utt.channel, "text": text,  # the corrected text
               "require_tts_reply": True}  # answer through the speaker
        # Far-end microphone: not addressed (it can carry bystanders), and
        # the raw transcript goes to the bypass (16 S4 constraint 1).
        res = handle_text_command(msg, self._orch,
                                  self._session_for(utt.channel),  # shared
                                  self._matcher, self._now_ms(),  # with text
                                  raw_text=raw_text, addressed=False)
        self.voice_turns += 1
        # Both forms logged, as 11 S8.7.6 wants them in the audit record.
        _logger.info("voice_in asr_raw=%r asr_text=%r", raw_text, text)
        self._emit("voice_in", utt.channel, utt.stream_id, text, res)
        return res

    def _session_for(self, channel: Any) -> OrchestratorSession:
        """The session slot of a channel token.

        An unknown token gets a throwaway session: handle_text_command
        refuses the channel before the session is used, so nothing leaks,
        and the refusal stays in one place instead of two.
        """
        return self._sessions.get(channel, OrchestratorSession())

    def _emit(self, source: str, channel: Any, ref: Any, text: str,
              res: TextTurnResult) -> None:
        """Publish the turn's frames and log one line describing it.

        The log line is the only record of a cloud turn today (p4 publishes
        no state/voice_turn), so it carries everything needed to answer
        "what did the robot make of what I sent": who, which text, refused
        or not and why, which branch, which intent, how many frames.
        """
        for key, data in res.publishes:  # in the order the turn built them
            self._publish(key, data)  # main_wiring's cached GEN publisher
        d = res.decision  # None when the channel gate refused the turn
        _logger.info(
            "%s turn channel=%s ref=%s text=%r denied=%s code=%s kind=%s "
            "intent=%s publishes=%d", source, channel, ref, text, res.denied,
            res.code or "-", d.kind if d else "-",  # "-": nothing to show
            d.intent_id if d else "-", len(res.publishes))  # 0 = silent


def _enqueue(jobs: "queue.Queue", job: Callable[[], Any], what: str) -> None:
    """Hand a job to the worker; on a full queue drop THIS job, logged.

    Not drop-oldest (the microphone queue's policy): a job is a whole turn,
    and silently discarding an older command to make room for a newer one
    would execute commands out of the order they were given.
    """
    try:
        jobs.put_nowait(job)  # never blocks the Zenoh callback thread
    except queue.Full:
        # Sixteen turns waiting means the worker is wedged (ASR hanging);
        # refusing more is the visible failure, queueing more the silent one.
        _logger.warning("%s dropped: worker job queue full", what)


def make_voice_text_callback(jobs: "queue.Queue", runner: CloudTurnRunner):
    """Zenoh callback for cmd/voice_text: decode, then enqueue the turn."""
    def _cb(sample) -> None:
        # Rust thread: JSON decode only; the turn (which may call the LLM)
        # runs on the worker.
        try:
            msg = json.loads(bytes(sample.payload))
        except Exception as exc:  # noqa: BLE001 -- a bad frame must not kill the sub
            _logger.warning("voice_text rejected: undecodable (%s)", exc)
            return
        # msg is bound per call, so each job runs its own message.
        _enqueue(jobs, lambda: runner.run_text(msg), "voice_text")
    return _cb  # the caller declares the subscriber and keeps the handle


def make_voice_in_callback(jobs: "queue.Queue", assembler: VoiceInAssembler,
                           runner: CloudTurnRunner,
                           now_ms_fn: Callable[[], int]):
    """Zenoh callback for audio/voice_in: assemble; at eos enqueue the turn."""
    def _cb(sample) -> None:
        # Rust thread: decode + in-memory assembly, both bounded and quick;
        # ASR (an HTTP call) happens in the job, on the worker.
        try:
            msg = json.loads(bytes(sample.payload))
        except Exception as exc:  # noqa: BLE001 -- a bad frame must not kill the sub
            _logger.warning("voice_in rejected: undecodable (%s)", exc)
            return
        try:
            utt = assembler.accept(msg, now_ms_fn())  # None until eos
        except VoiceInRejected as exc:
            # Every refusal is one line with its reason token.
            _logger.warning("voice_in rejected: %s", exc)
            return
        if utt is not None:
            # eos reached: one job per sentence, never one per chunk.
            _enqueue(jobs, lambda: runner.run_voice(utt), "voice_in")
    return _cb  # the caller declares the subscriber and keeps the handle


def cloud_input_keys(robot_id: str) -> Tuple[str, str]:
    """(voice_text key, voice_in key) for one robot id.

    One function for both so the wiring and the tests build the keys the
    same way; a typo in one copy would subscribe a key nobody publishes.
    """
    return VOICE_TEXT_KEY_FMT % robot_id, VOICE_IN_KEY_FMT % robot_id
