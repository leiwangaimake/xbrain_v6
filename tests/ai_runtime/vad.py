"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: vad.py
Brief: Voice activity detection that cuts the mic stream into utterances.

Description:
  The 功能1 turn begins with a question nobody else can answer: when did the person stop
  talking? The device sends an unbroken 16 kHz stream and offers no end-of-speech signal,
  while asr-service is batch -- it wants one complete utterance per request (plan section
  5.3). This module is the piece in between.

  The module is split at the one seam that matters. The SEGMENTATION -- when a turn opens,
  when it closes, what audio it carries -- is written from scratch here, because that is
  the behaviour the test system exists to exercise. The per-frame question "is this frame
  speech" is delegated to an interchangeable detector, because that question has a right
  answer that a trained model gives better than a threshold does.

  Two detectors exist and both are used:
    - SileroSpeechDetector, the default. silero_vad.onnx run through sherpa-onnx, which
      the plan's whitelist already admits as prebuilt inference infrastructure (section
      11); the model file itself was approved and supplied for this purpose.
    - EnergySpeechDetector, the original. Mean absolute amplitude against a threshold.

  Why the switch to silero was worth making: energy measures loudness, and loudness is not
  speech. The near-field mic separates a talker from the room easily enough -- a floor near
  2 counts against speech near 950 -- but it cannot separate a talker from a chair scrape,
  a door, or a fan spinning up, all of which clear any threshold that a quiet talker also
  clears. Those false segments are not harmless: the zipformer behind asr-service is a pure
  Chinese model with no way to answer "that was not speech", so it returns a plausible
  Chinese word instead, and the turn loop dutifully answers a question nobody asked. Silero
  was trained on exactly this distinction and rejects the noise before ASR is ever called.
  It costs 0.25 ms per 32 ms window on the Orin, which is nothing against a turn.

  Why energy is kept rather than deleted: it needs no model file and no inference, which is
  what makes the state machine above testable exhaustively from synthetic PCM, and what
  keeps the harness runnable on a box where the onnx model has not been deployed.

  Design: a two-state machine (SILENCE, SPEECH) with hysteresis on both edges. Entering
  speech needs vad_start_ms of continuous speech frames so one stray verdict cannot open a
  turn; leaving it needs vad_stop_ms of continuous quiet so a pause mid-sentence cannot end
  one.
  The two thresholds are independent because the costs are not symmetric -- a false start
  wastes a decode, a false end truncates the user mid-word.

  Two details that make the output usable rather than merely correct:
    - PREROLL. The frames that prove speech started are, by definition, already speech. A
      detector that emits only what follows the trigger clips its own first syllable --
      exactly the consonant that separates 前进 from 前近. A ring buffer of vad_preroll_ms
      is therefore carried through silence and prepended to every utterance.
    - LENGTH BOUNDS. A segment shorter than vad_min_utterance_ms is dropped as a transient
      rather than sent to ASR, where it would cost a decode and risk a hallucinated
      command. A segment reaching vad_max_utterance_ms is force-closed, so continuous noise
      cannot grow one utterance without bound and starve the turn loop.

  The class is deliberately pure: bytes in, utterances out, no clock and no I/O. Time is
  counted in frames, which is what makes the whole state machine testable at full speed
  from synthetic PCM, with no device and no waiting.
"""
from __future__ import annotations

import os
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Protocol

import numpy as np

from .config import AiRuntimeConfig

# The mic plane's fixed geometry, matching payload-service's WS /mic header: 16 kHz mono
# s16le in 20 ms frames, so 320 samples and 640 bytes per frame. Named here because the
# frame duration is what converts every millisecond knob in the config into a frame count.
_FRAME_MS = 20
_SAMPLE_RATE = 16000
_BYTES_PER_SAMPLE = 2
FRAME_BYTES = (_SAMPLE_RATE * _FRAME_MS // 1000) * _BYTES_PER_SAMPLE

# int16 little-endian: the encoding declared by the /mic header and the only one accepted.
_PCM_DTYPE = np.dtype("<i2")

# Divisor that maps int16 counts onto the [-1.0, 1.0) range the onnx model was trained on.
# 32768 rather than 32767 so the scaling is an exact binary shift and the most negative
# sample maps to exactly -1.0; using 32767 would put it fractionally past the range.
_INT16_FULL_SCALE = 32768.0

# The two accepted values of config.vad_backend. Named constants rather than bare strings
# so the factory below and any caller comparing against a backend cannot drift apart.
BACKEND_SILERO = "silero"
BACKEND_ENERGY = "energy"

# Samples silero consumes per inference at 16 kHz. Not a tunable: the network's input
# dimension is fixed, and 512 is the value it was exported with. It is 32 ms, which does
# NOT divide the 20 ms mic frame -- SileroSpeechDetector.is_speech is where that is handled.
_SILERO_WINDOW_SAMPLES = 512


class VadError(ValueError):
    """Raised when the detector cannot be built or is handed a frame of the wrong size.

    House rule bans bare Exception. Two situations share this type because they share a
    remedy -- the operator has to go fix something outside this module before the process
    can work at all:
      - A wrong frame size, meaning the caller and the mic stream disagree about geometry.
        That is a wiring fault distinct from any service or device failure, and one that
        would otherwise show up as a silently wrong reading rather than as an error,
        because a misaligned buffer still decodes to plausible numbers.
      - A silero backend that cannot load, meaning the model file is missing or sherpa-onnx
        is not installed on this box.
    ValueError is the base because in both cases an input -- an argument or a config field
    -- is what is wrong, as opposed to an operation that failed.
    """


@dataclass(frozen=True)
class Utterance:
    """One complete segment of speech, ready to be sent to asr-service.

    Frozen because it is a result value: once the detector has closed a segment, nothing
    downstream should be able to alter what was heard.
    """

    # Raw 16 kHz mono s16le PCM, preroll included, in capture order.
    pcm: bytes
    # How many 20 ms frames the segment spans, preroll included.
    frames: int

    @property
    def duration_s(self) -> float:
        """Length of the utterance in seconds.

        Returns:
            The duration implied by the frame count, which is exact because every frame is
            the same fixed 20 ms.
        """
        return self.frames * _FRAME_MS / 1000.0


def frame_energy(frame: bytes) -> float:
    """Measure one frame's loudness as its mean absolute sample amplitude.

    Args:
        frame: exactly FRAME_BYTES of 16 kHz mono s16le PCM.

    Returns:
        The mean absolute amplitude in int16 counts, 0.0 to 32768.0.

    Raises:
        VadError: if the frame is not exactly FRAME_BYTES long.

    Mean absolute amplitude is used rather than RMS because the two rank frames almost
    identically for this purpose while this one costs no square root, and -- the reason
    that actually matters -- its unit is int16 counts. A threshold expressed in counts can
    be compared directly against a level printed from a captured wav, so recalibrating in
    a new room is a measurement rather than a guess.

    The mean is taken in float64 (numpy's default for abs().mean()) so a loud frame cannot
    overflow the accumulator, which int16 arithmetic would do after only two samples.
    """
    if len(frame) != FRAME_BYTES:
        raise VadError(f"expected a {FRAME_BYTES}-byte frame, got {len(frame)}")
    samples = np.frombuffer(frame, dtype=_PCM_DTYPE)
    # abs() before the mean: without it a symmetric waveform averages to ~0 regardless of
    # how loud it is, which would report every frame as silence.
    return float(np.abs(samples.astype(np.float32)).mean())


class SpeechDetector(Protocol):
    """The single question the segmenter asks about each frame: was that speech.

    A Protocol rather than a base class because neither implementation wants anything from
    a parent -- one is four lines of arithmetic, the other wraps an onnx session -- and
    structural typing keeps them independent of each other. It is also what lets a test
    supply a two-line stand-in and drive the state machine directly.
    """

    def is_speech(self, frame: bytes) -> bool:
        """Classify one FRAME_BYTES frame of 16 kHz mono s16le PCM."""

    def reset(self) -> None:
        """Forget any state carried between frames."""


class EnergySpeechDetector:
    """Calls a frame speech when its mean absolute amplitude clears a threshold.

    Kept as the model-free backend. Its unit is int16 counts, so the threshold can be
    compared directly against a level printed from a captured wav, which makes retuning in
    a new room a measurement rather than a guess. What it cannot do is tell a voice from
    any other loud thing, which is why it is no longer the default.
    """

    def __init__(self, config: AiRuntimeConfig) -> None:
        """Prepare the detector from the configured energy threshold.

        Args:
            config: supplies vad_threshold, in int16 counts.
        """
        self._threshold = config.vad_threshold

    def is_speech(self, frame: bytes) -> bool:
        """Classify one frame by comparing its energy against the threshold.

        Args:
            frame: exactly FRAME_BYTES of 16 kHz mono s16le PCM.

        Returns:
            True when the frame's mean absolute amplitude reaches the threshold.

        Raises:
            VadError: if the frame is not exactly FRAME_BYTES long.
        """
        return frame_energy(frame) >= self._threshold

    def reset(self) -> None:
        """Do nothing, because this detector carries nothing between frames.

        Each frame's verdict depends only on that frame's samples, so there is no state a
        gap in the stream could corrupt. The method exists to satisfy SpeechDetector, and
        it is a genuine no-op rather than an oversight.
        """


class SileroSpeechDetector:
    """Calls a frame speech when silero_vad.onnx says the audio around it is a voice.

    The model is run through sherpa-onnx, which the plan admits as prebuilt inference
    infrastructure. Only the model's per-window verdict is taken; the segmentation around
    it stays this module's own, so sherpa-onnx's own VAD wrapper -- which would decide when
    turns start and end -- is deliberately not used.
    """

    def __init__(self, config: AiRuntimeConfig) -> None:
        """Load the silero model and prepare the window buffer.

        Args:
            config: supplies the model path and the speech probability threshold.

        Raises:
            VadError: if sherpa-onnx is not installed or the model file is missing.

        Blocking: this reads a model off disk and builds an onnx session, so it belongs in
        process startup, not in the frame path.

        sherpa_onnx is imported HERE rather than at module scope so that importing this
        module costs nothing on a box that has no onnx runtime. The state machine above is
        the part under test, and it must stay testable where the model is not deployed.
        """
        try:
            import sherpa_onnx
        except ImportError as exc:
            # Chained so the original import failure -- which names the missing shared
            # library when that is the real cause -- survives into the message.
            raise VadError(
                "vad_backend=silero needs sherpa-onnx, which is not importable"
            ) from exc
        # Checked before the model is handed over because sherpa-onnx answers a missing
        # file by writing to stderr and returning a detector that classifies nothing,
        # which would look like a microphone that never hears anyone.
        if not os.path.isfile(config.vad_silero_model):
            raise VadError(f"silero model not found: {config.vad_silero_model}")
        model_config = sherpa_onnx.VadModelConfig()
        model_config.silero_vad.model = config.vad_silero_model
        model_config.silero_vad.threshold = config.vad_silero_threshold
        model_config.silero_vad.window_size = _SILERO_WINDOW_SAMPLES
        model_config.sample_rate = _SAMPLE_RATE
        # One thread and CPU: a window costs 0.25 ms on this box, so there is nothing for a
        # second thread to do, and keeping it off the GPU leaves that entirely to the ASR
        # and LLM models, which are the only two that need it.
        model_config.num_threads = 1
        model_config.provider = "cpu"
        # The duration fields silero also accepts -- min_speech_duration,
        # min_silence_duration, max_speech_duration -- are left at their defaults and
        # deliberately unused: those are segmentation decisions, and they are made by the
        # state machine below from the vad_*_ms knobs, in one place.
        self._model = sherpa_onnx.VadModel.create(model_config)
        # Read back rather than assumed: this is the count is_speech will actually consume,
        # and buffering to a different number would feed the model short windows.
        self._window = self._model.window_size()
        # Samples received but not yet forming a whole window.
        self._pending = np.zeros(0, dtype=np.float32)
        # The most recent window's verdict, held for frames that complete no window.
        # Starts False so a stream is assumed silent until the model says otherwise.
        self._speech = False

    def is_speech(self, frame: bytes) -> bool:
        """Classify one frame using the most recent complete silero window.

        Args:
            frame: exactly FRAME_BYTES of 16 kHz mono s16le PCM.

        Returns:
            True when the last window silero evaluated was speech.

        The two clocks do not line up: a mic frame is 320 samples and a silero window is
        512, so 3 frames out of every 8 complete no new window and reuse the previous
        verdict. Resampling the audio to fit would be the wrong fix -- it would alter what
        the model sees to satisfy a bookkeeping detail. The cost of reusing instead is that
        a verdict can be up to one frame stale, which is 20 ms against a start edge of 120
        and a stop edge of 500. It cannot change a decision.
        """
        samples = np.frombuffer(frame, dtype=_PCM_DTYPE).astype(np.float32)
        self._pending = np.concatenate((self._pending, samples / _INT16_FULL_SCALE))
        # while rather than if: a frame can complete a window that a previous frame left
        # partly filled, and on a longer buffer it could complete more than one.
        while len(self._pending) >= self._window:
            self._speech = bool(self._model.is_speech(self._pending[: self._window]))
            self._pending = self._pending[self._window :]
        return self._speech

    def reset(self) -> None:
        """Clear the model's memory and drop the partial window.

        Silero is recurrent: each window is judged in the context of the ones before it.
        The turn loop resets after the Speak phase, and the frames captured while the robot
        was talking were never offered here -- so the stream this detector sees has a hole
        in it. Without clearing the state, the first windows of the next Listen phase would
        be judged against audio from before that hole. The buffered samples are dropped for
        the same reason: they are from the wrong side of the gap.
        """
        self._model.reset()
        self._pending = np.zeros(0, dtype=np.float32)
        self._speech = False


def build_speech_detector(config: AiRuntimeConfig) -> SpeechDetector:
    """Construct the per-frame speech detector named by the configuration.

    Args:
        config: supplies vad_backend and whichever backend's settings it names.

    Returns:
        The detector to hand to VoiceActivityDetector.

    Raises:
        VadError: if vad_backend is not a known name, or the named backend cannot load.

    An unknown name is refused rather than silently falling back to energy. Falling back
    would produce a process that runs, answers, and is quietly worse than the operator
    believes it is -- the exact failure that is hardest to notice on a device.
    """
    if config.vad_backend == BACKEND_SILERO:
        return SileroSpeechDetector(config)
    if config.vad_backend == BACKEND_ENERGY:
        return EnergySpeechDetector(config)
    raise VadError(
        f"unknown vad_backend {config.vad_backend!r}, "
        f"expected {BACKEND_SILERO!r} or {BACKEND_ENERGY!r}"
    )


class VoiceActivityDetector:
    """Cuts a continuous 16 kHz frame stream into utterances, one frame at a time.

    The detector holds all the state a turn needs -- which side of the hysteresis it is on,
    how long the current run has lasted, the preroll ring, and the audio collected so far.
    It never blocks and never sleeps: a caller feeds it whatever the mic produced and gets
    back either None or a finished Utterance, which is what lets the turn loop stay a
    simple read-and-dispatch loop.
    """

    def __init__(self, config: AiRuntimeConfig, detector: SpeechDetector) -> None:
        """Prepare a segmenter from the configured durations and a speech detector.

        Args:
            config: supplies the four duration knobs and the preroll length.
            detector: the per-frame speech test, normally from build_speech_detector.

        The detector is passed in rather than built from config here, so that this class
        depends on the QUESTION "was that speech" and not on any particular way of
        answering it. That is what keeps the segmentation tests -- which are the ones that
        cover the from-scratch logic -- free of a model file.

        Every millisecond knob is converted to a frame count once, here, rather than on
        each frame: the comparison in the hot path is then an integer against an integer,
        and -- more importantly -- the conversion is done in exactly one place, so the
        start and stop edges cannot end up rounding differently from each other.
        """
        self._detector = detector
        # max(1, ...) so a knob set below one frame still means "at least one frame" rather
        # than zero, which would make the edge trigger on nothing at all.
        self._start_frames = max(1, config.vad_start_ms // _FRAME_MS)
        self._stop_frames = max(1, config.vad_stop_ms // _FRAME_MS)
        self._min_frames = max(1, config.vad_min_utterance_ms // _FRAME_MS)
        self._max_frames = max(1, config.vad_max_utterance_ms // _FRAME_MS)

        # The preroll ring. A bounded deque drops its oldest entry automatically on append,
        # so the memory this holds during a long silence is capped by construction rather
        # than by a length check that could be forgotten.
        preroll_frames = max(0, config.vad_preroll_ms // _FRAME_MS)
        self._preroll: Deque[bytes] = deque(maxlen=preroll_frames)

        # True once the start edge has fired and until the stop edge does.
        self._in_speech = False
        # Length of the current same-side run, in frames. Read as "consecutive speech
        # frames" while silent and "consecutive silence frames" while in speech, which is
        # why the counter is reset on every state change.
        self._run = 0
        # Audio of the utterance being built, preroll first. Empty while silent.
        self._collected: List[bytes] = []

    @property
    def in_speech(self) -> bool:
        """Whether the detector currently considers the stream to be speech."""
        return self._in_speech

    def reset(self) -> None:
        """Discard all state, as if no audio had ever been seen.

        Called when the turn loop stops trusting its own buffer: after the Speak phase (the
        gated frames were never offered, so the run counters describe a stream with a hole
        in it) and when a mode change tears the mic session down. Without a reset there,
        the first frame of the next Listen phase would be judged against a run length
        accumulated before the gap, and a turn could open on audio that was never spoken.
        """
        self._in_speech = False
        self._run = 0
        self._collected = []
        self._preroll.clear()
        # The detector is reset too, because silero remembers: its verdict for a window
        # depends on the windows before it, and after a gap in the stream those are audio
        # the microphone never delivered.
        self._detector.reset()

    def push(self, frame: bytes) -> Optional[Utterance]:
        """Feed one 20 ms frame and return an utterance if this frame completed one.

        Args:
            frame: exactly FRAME_BYTES of 16 kHz mono s16le PCM.

        Returns:
            The finished Utterance when this frame ended a segment that cleared the minimum
            length, otherwise None. None is the overwhelmingly common answer -- one
            utterance spans dozens of frames.

        Raises:
            VadError: if the frame is not exactly FRAME_BYTES long.

        The two branches are asymmetric on purpose: while silent, a frame is a candidate
        that also has to be REMEMBERED (it may turn out to be preroll); while in speech,
        every frame is kept regardless of its verdict, because dropping the quiet frames
        inside a sentence would splice syllables together and change what ASR hears.
        """
        # Checked here rather than left to the detector so the geometry contract holds
        # whichever backend is loaded: silero would otherwise accept a short frame and
        # quietly misalign every window that followed it.
        if len(frame) != FRAME_BYTES:
            raise VadError(f"expected a {FRAME_BYTES}-byte frame, got {len(frame)}")
        speech = self._detector.is_speech(frame)
        if not self._in_speech:
            return self._advance_silence(frame, speech)
        return self._advance_speech(frame, speech)

    def flush(self) -> Optional[Utterance]:
        """Close any utterance in progress and return it if it is long enough.

        Returns:
            The pending Utterance when one was in progress and cleared the minimum length,
            otherwise None.

        This exists for the end of the stream -- the mic session closing, or the turn loop
        shutting down -- where the trailing silence that would normally end the segment is
        never going to arrive. Without it the last thing the user said before disconnecting
        would be silently dropped, which during a bring-up run looks exactly like the VAD
        having failed to hear it.
        """
        if not self._in_speech:
            return None
        return self._close()

    def _advance_silence(self, frame: bytes, speech: bool) -> Optional[Utterance]:
        """Handle one frame while the detector is in the SILENCE state.

        Args:
            frame: the frame's PCM, kept as potential preroll.
            speech: whether the detector called this frame speech.

        Returns:
            Always None: no utterance can END while the detector is silent.

        A non-speech frame resets the run rather than decrementing it, so the start edge needs
        vad_start_ms of CONTINUOUS speech. Requiring continuity is what makes a rhythmic
        noise -- a fan blade, a footstep -- unable to accumulate its way past the edge.
        """
        # Every frame seen while silent enters the preroll ring, including loud ones: the
        # frames that prove speech started are the very ones the utterance must begin with.
        self._preroll.append(frame)
        if not speech:
            self._run = 0
            return None
        self._run += 1
        if self._run < self._start_frames:
            return None
        # Start edge. The ring is drained into the segment, which both seeds the utterance
        # with its own opening syllable and leaves the ring empty for the next silence.
        self._in_speech = True
        self._collected = list(self._preroll)
        self._preroll.clear()
        # Reset for its other meaning: from here the run counts consecutive SILENCE frames.
        self._run = 0
        return None

    def _advance_speech(self, frame: bytes, speech: bool) -> Optional[Utterance]:
        """Handle one frame while the detector is in the SPEECH state.

        Args:
            frame: the frame's PCM, always appended to the segment.
            speech: whether the detector called this frame speech.

        Returns:
            The finished Utterance if this frame closed the segment, otherwise None.

        The frame is collected before the decision, so the trailing quiet that proves the
        utterance ended stays inside it. That silence is not waste: a decoder given audio
        that stops on the final consonant tends to clip it, and asr-service is batch, so
        there is no later chance to add it.
        """
        self._collected.append(frame)
        # A single speech frame resets the silence run, so an utterance survives the natural
        # pauses inside a sentence rather than being cut at every comma.
        self._run = 0 if speech else self._run + 1
        if self._run >= self._stop_frames:
            return self._close()
        # Runaway guard: continuous noise never produces the stop edge, so without this a
        # single segment would grow until memory ran out and the turn loop would never get
        # its chance to run. Force-closing hands over what was heard so far.
        if len(self._collected) >= self._max_frames:
            return self._close()
        return None

    def _close(self) -> Optional[Utterance]:
        """End the current segment, returning it only if it is long enough to be speech.

        Returns:
            The Utterance when it spans at least vad_min_utterance_ms, otherwise None.

        State is reset before the length test so that a rejected segment leaves the
        detector exactly as ready for the next utterance as an accepted one does -- the
        caller's None must not mean "still collecting" in one case and "discarded" in the
        other.
        """
        frames = self._collected
        self._in_speech = False
        self._run = 0
        self._collected = []
        # Too short to be an utterance: a burst that read as speech for a fraction
        # of a second is a transient, and decoding it would cost a request and risk
        # returning a plausible-looking command nobody spoke.
        if len(frames) < self._min_frames:
            return None
        return Utterance(pcm=b"".join(frames), frames=len(frames))
