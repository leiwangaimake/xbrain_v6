"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_vad.py
Brief: Unit tests for utterance segmentation and for the two speech detectors behind it.

Description:
  The tests come in two halves, matching the seam in vad.py.

  The SEGMENTATION half drives the state machine -- hysteresis, preroll, length bounds --
  from synthetic constant-amplitude PCM through the energy detector. Constant amplitude is
  chosen because a frame's mean absolute amplitude is then exactly that amplitude, so every
  decision the machine makes is deterministic rather than approximately loud. This half is
  where the from-scratch logic is covered, and it deliberately needs no model file.

  The DETECTOR half asks the question the energy threshold could never answer: is this
  actually a voice. Its central test feeds one signal to both backends and asserts they
  disagree -- energy opens a turn on a loud tone, silero does not. A test that only checked
  silero in isolation would pass just as well if the swap had never happened, so the
  disagreement is the assertion, not a detail of it.
"""
from __future__ import annotations

import dataclasses
import importlib.util
import os
import wave

import numpy as np
import pytest

from tests.ai_runtime.config import AiRuntimeConfig
from tests.ai_runtime.vad import (
    BACKEND_ENERGY,
    FRAME_BYTES,
    EnergySpeechDetector,
    SileroSpeechDetector,
    VadError,
    VoiceActivityDetector,
    build_speech_detector,
    frame_energy,
)

# Well under and well over the energy threshold, so no test depends on a borderline level.
_QUIET = 5
_LOUD = 900

# The mic plane's sample rate, needed here only to give the synthetic tone a real pitch.
_RATE = 16000

# The silero tests need the engine, which is installed on the Orin but not necessarily on a
# development box -- that absence is the ONE legitimate reason to skip them.
_HAVE_ENGINE = importlib.util.find_spec("sherpa_onnx") is not None

# The VAD model, from configuration, so pointing AI_VAD_SILERO_MODEL elsewhere moves the
# tests with it.
_SILERO_MODEL = AiRuntimeConfig().vad_silero_model

# A clip of real human speech, needed because silero judges by CONTENT: a synthetic tone
# would be correctly rejected as non-speech and could not exercise the positive path. It
# comes from the transducer export's test_wavs/, named explicitly rather than derived from
# the silero path -- the two files used to share a directory by accident, and deriving one
# from the other meant moving the VAD model silently relocated the speech fixture too.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SPEECH_WAV = os.path.join(
    _REPO_ROOT, "services", "asr", "model-zipformer-multi-zh-hans",
    "test_wavs", "0.wav",
)

# ★ Missing FIXTURES skip; missing DEPLOYED ASSETS fail.
#
# These two conditions used to be one skipif, and that hid a real regression: renaming the
# ASR model directory moved silero_vad.onnx out from under the configured default, the
# whole silero suite turned yellow instead of red, and the run still reported success --
# while function 1 could not have started a single utterance on the box. So the engine's
# absence still skips (a dev box legitimately has no sherpa-onnx), but once the engine IS
# present, a configured model file that does not exist is a failure, because on the box
# where the engine exists the model is supposed to exist too.
_requires_silero = pytest.mark.skipif(
    not _HAVE_ENGINE or not os.path.isfile(_SPEECH_WAV),
    reason="the silero backend needs sherpa-onnx and a speech fixture",
)


@pytest.mark.skipif(not _HAVE_ENGINE, reason="only meaningful where the engine is deployed")
def test_configured_silero_model_is_actually_deployed() -> None:
    # The guard the skipif above no longer performs. On any box carrying the engine, the
    # default VAD model path must resolve to a real file -- otherwise the default backend
    # cannot start, and every test that would have caught it is skipped for the same reason.
    assert os.path.isfile(_SILERO_MODEL), (
        f"vad_silero_model points at {_SILERO_MODEL}, which does not exist; the silero "
        f"backend is the default, so function 1 cannot start"
    )


def _frame(amplitude: int) -> bytes:
    """One 20 ms frame of constant amplitude, whose mean abs energy is that amplitude."""
    return np.full(FRAME_BYTES // 2, amplitude, dtype="<i2").tobytes()


def _split(raw: bytes) -> list:
    """Cut a PCM buffer into whole mic frames, discarding any short tail."""
    return [raw[i : i + FRAME_BYTES] for i in range(0, len(raw) - FRAME_BYTES + 1, FRAME_BYTES)]


def _tone_frames(count: int, hz: float = 220.0, amplitude: int = 9830) -> list:
    """A continuous sine, cut into count frames.

    Args:
        count: how many 20 ms frames to produce.
        hz: pitch, chosen inside the vocal range so the signal is not rejected merely for
            sitting somewhere no voice ever goes.
        amplitude: peak in int16 counts. 9830 gives a mean absolute amplitude near 6260,
            which is over 150x the energy threshold -- the point being that no threshold
            able to hear a quiet talker could possibly reject this.

    Returns:
        The frames, in order.

    Generated as ONE buffer and then cut, rather than per frame, so the phase runs
    continuously across frame boundaries. A per-frame sine would restart at zero every
    20 ms, adding a periodic click that is itself a broadband transient -- the test would
    then be measuring the clicks rather than the tone.
    """
    t = np.arange(count * (FRAME_BYTES // 2), dtype=np.float64) / _RATE
    return _split((amplitude * np.sin(2 * np.pi * hz * t)).astype("<i2").tobytes())


def _speech_frames() -> list:
    """The ASR model's own 16 kHz test utterance, cut into mic frames.

    Real recorded speech is used rather than anything synthetic because that is the one
    input silero must accept, and no generated waveform is a convincing stand-in for it.
    """
    with wave.open(_SPEECH_WAV, "rb") as handle:
        return _split(handle.readframes(handle.getnframes()))


def _config() -> AiRuntimeConfig:
    """Short knobs so a whole utterance is a handful of frames, not hundreds."""
    return dataclasses.replace(
        AiRuntimeConfig(),
        vad_threshold=40.0,
        vad_start_ms=40,            # 2 frames
        vad_stop_ms=100,            # 5 frames
        vad_preroll_ms=60,          # 3 frames
        vad_min_utterance_ms=200,   # 10 frames
        vad_max_utterance_ms=400,   # 20 frames
    )


def _energy_vad(config: AiRuntimeConfig = None) -> VoiceActivityDetector:
    """A segmenter backed by the energy detector, which needs no model file."""
    config = _config() if config is None else config
    return VoiceActivityDetector(config, EnergySpeechDetector(config))


def _silero_vad(config: AiRuntimeConfig = None) -> VoiceActivityDetector:
    """A segmenter backed by silero, at the production duration knobs.

    The defaults are used rather than the short test knobs because a 5-second recording of
    real speech under a 400 ms utterance ceiling would be chopped by the runaway guard, and
    the result would describe that guard rather than the detector.
    """
    config = AiRuntimeConfig() if config is None else config
    return VoiceActivityDetector(config, SileroSpeechDetector(config))


def _push(vad: VoiceActivityDetector, frames: list) -> list:
    """Push every frame in order and collect the utterances they produced."""
    return [u for u in (vad.push(frame) for frame in frames) if u is not None]


def _push_all(vad: VoiceActivityDetector, amplitude: int, count: int) -> list:
    """Push count frames of one amplitude and collect every utterance they produced."""
    return _push(vad, [_frame(amplitude)] * count)


class _ScriptedDetector:
    """A SpeechDetector stand-in with a fixed verdict that counts how often it is reset.

    Exists to prove the Protocol seam is real: the segmenter is driven here with no energy
    measurement and no model at all.
    """

    def __init__(self, verdict: bool = False) -> None:
        self.verdict = verdict
        self.resets = 0

    def is_speech(self, frame: bytes) -> bool:
        return self.verdict

    def reset(self) -> None:
        self.resets += 1


def test_frame_energy_is_the_mean_absolute_amplitude():
    assert frame_energy(_frame(1000)) == pytest.approx(1000.0)
    assert frame_energy(_frame(0)) == pytest.approx(0.0)


def test_frame_energy_ignores_the_sign_of_the_waveform():
    # A symmetric waveform averages to ~0 without the abs(), which would report every
    # loud frame as silence -- the single mistake this measurement must not make.
    samples = np.tile(np.array([1000, -1000], dtype="<i2"), FRAME_BYTES // 4)
    assert frame_energy(samples.tobytes()) == pytest.approx(1000.0)


def test_frame_energy_rejects_a_wrong_sized_frame():
    with pytest.raises(VadError):
        frame_energy(b"\x00" * (FRAME_BYTES - 2))


def test_silence_never_opens_an_utterance():
    detector = _energy_vad()
    assert _push_all(detector, _QUIET, 100) == []
    assert not detector.in_speech


def test_a_speech_burst_is_segmented_with_its_preroll():
    detector = _energy_vad()
    assert _push_all(detector, _QUIET, 10) == []
    assert _push_all(detector, _LOUD, 10) == []
    utterances = _push_all(detector, _QUIET, 5)

    assert len(utterances) == 1
    # 3 preroll frames + 8 speech frames after the start edge + 5 trailing quiet frames.
    assert utterances[0].frames == 16
    assert len(utterances[0].pcm) == 16 * FRAME_BYTES
    assert utterances[0].duration_s == pytest.approx(0.32)
    # The segment opens on preroll, i.e. on audio captured BEFORE the start edge fired.
    assert utterances[0].pcm[:FRAME_BYTES] == _frame(_QUIET)
    assert _frame(_LOUD) in utterances[0].pcm
    assert not detector.in_speech


def test_a_burst_shorter_than_the_minimum_is_dropped():
    detector = _energy_vad()
    _push_all(detector, _QUIET, 10)
    # Just enough to cross the start edge, then straight back to silence: 3 preroll plus
    # 5 trailing quiet frames is 8, under the 10-frame minimum.
    assert _push_all(detector, _LOUD, 2) == []
    assert _push_all(detector, _QUIET, 5) == []
    # A dropped segment must leave the detector as ready as an accepted one does.
    assert not detector.in_speech


def test_a_pause_inside_a_sentence_does_not_end_the_utterance():
    detector = _energy_vad()
    _push_all(detector, _QUIET, 10)
    assert _push_all(detector, _LOUD, 5) == []
    # 3 quiet frames is under the 5-frame stop edge, so the sentence continues.
    assert _push_all(detector, _QUIET, 3) == []
    assert _push_all(detector, _LOUD, 5) == []
    utterances = _push_all(detector, _QUIET, 5)

    assert len(utterances) == 1
    # One segment spanning the pause: 3 preroll + 3 + 3 + 5 + 5 frames.
    assert utterances[0].frames == 19


def test_continuous_noise_is_force_closed_at_the_maximum():
    detector = _energy_vad()
    _push_all(detector, _QUIET, 10)
    # Never any silence to close on, so only the runaway guard can end this.
    utterances = _push_all(detector, _LOUD, 19)

    assert len(utterances) == 1
    assert utterances[0].frames == 20


def test_flush_closes_an_utterance_still_in_progress():
    detector = _energy_vad()
    _push_all(detector, _QUIET, 10)
    _push_all(detector, _LOUD, 10)
    assert detector.in_speech

    utterance = detector.flush()

    assert utterance is not None
    assert utterance.frames == 11
    assert not detector.in_speech
    # Flushing twice must not re-emit the same audio.
    assert detector.flush() is None


def test_reset_discards_an_utterance_in_progress():
    detector = _energy_vad()
    _push_all(detector, _QUIET, 10)
    _push_all(detector, _LOUD, 10)

    detector.reset()

    assert not detector.in_speech
    assert detector.flush() is None
    # The preroll ring is cleared too, so the next utterance cannot start with audio from
    # before the reset -- which is the whole point of resetting after the Speak gate.
    assert _push_all(detector, _LOUD, 2) == []
    utterances = _push_all(detector, _QUIET, 5)
    assert utterances == []


def test_resetting_the_segmenter_also_resets_the_speech_detector():
    # Silero is recurrent: it judges each window in the context of the ones before it. A
    # reset that stopped at the state machine would leave the model still carrying audio
    # from before the Speak gate, so the first windows of the next Listen phase would be
    # judged against speech the microphone never delivered.
    detector = _ScriptedDetector()
    vad = VoiceActivityDetector(_config(), detector)

    vad.reset()

    assert detector.resets == 1


def test_a_detector_that_always_says_speech_drives_the_machine_with_no_audio_analysis():
    # Proves the seam is genuine rather than decorative: the segmenter reaches its runaway
    # guard on frames of pure digital silence, purely because the detector said so.
    vad = VoiceActivityDetector(_config(), _ScriptedDetector(verdict=True))

    utterances = _push(vad, [_frame(0)] * 21)

    assert len(utterances) == 1
    assert utterances[0].frames == 20


def test_push_rejects_a_wrong_sized_frame():
    detector = _energy_vad()
    with pytest.raises(VadError):
        detector.push(b"\x00" * (FRAME_BYTES + 2))


def test_build_speech_detector_honours_the_energy_backend():
    config = dataclasses.replace(_config(), vad_backend=BACKEND_ENERGY)
    assert isinstance(build_speech_detector(config), EnergySpeechDetector)


def test_build_speech_detector_refuses_an_unknown_backend():
    # Refused rather than quietly falling back, because a fallback produces a process that
    # runs, answers, and is worse than the operator believes it is.
    with pytest.raises(VadError):
        build_speech_detector(dataclasses.replace(_config(), vad_backend="webrtc"))


def test_a_missing_silero_model_is_an_error_rather_than_a_microphone_that_hears_nobody():
    # sherpa-onnx answers a missing model on stderr and hands back a detector that
    # classifies nothing, which on a device is indistinguishable from a dead microphone.
    # The check has to happen here, at startup, where the path is still in front of us.
    config = dataclasses.replace(_config(), vad_silero_model="/nonexistent/silero_vad.onnx")
    with pytest.raises(VadError):
        build_speech_detector(config)


@_requires_silero
def test_silero_is_the_default_backend():
    assert isinstance(build_speech_detector(AiRuntimeConfig()), SileroSpeechDetector)


@_requires_silero
def test_silero_rejects_the_loud_tone_that_the_energy_backend_calls_speech():
    # The reason the swap was made, stated as a disagreement. The tone's mean absolute
    # amplitude is around 6260 counts against a threshold of 40, so energy has no way to
    # refuse it: loudness is the only thing energy measures, and this is loud. Silero was
    # trained on the distinction that matters and hears no voice.
    #
    # Trailing silence is appended so the energy backend actually CLOSES its false segment
    # rather than sitting mid-utterance with nothing to show for it.
    frames = _tone_frames(100) + [_frame(0)] * 30

    assert _push(_energy_vad(AiRuntimeConfig()), frames), "energy must false-trigger here"
    assert _push(_silero_vad(), frames) == []


@_requires_silero
def test_silero_still_segments_real_speech():
    # The other half of the trade: a detector that rejected everything would pass the
    # disagreement test above perfectly. This is what stops that.
    vad = _silero_vad()

    # Nothing comes out of push(): this recording ends while the speaker is still talking,
    # so the trailing silence that normally closes a segment never arrives.
    assert _push(vad, _speech_frames()) == []
    # flush() is the path that exists for exactly that, and the one a mic session ending
    # mid-sentence takes.
    utterance = vad.flush()

    assert utterance is not None
    # Most of the speech has to survive; that is what this test is for. The lower bound is
    # the assertion, and the upper bound is only the arithmetic limit -- a segment cannot
    # be longer than the recording it came from.
    #
    # ★ The upper bound used to be 5.6 exclusive, to say "the detector did not simply pass
    # the whole file through". That stopped being a meaningful statement once vad_preroll_ms
    # was calibrated to 800 ms: this recording opens with less lead-in than that, so the
    # preroll ring legitimately holds the file from its first frame and the segment IS the
    # whole file. The property the old bound was reaching for -- that the detector
    # discriminates rather than passing everything -- is asserted directly, and far more
    # sharply, by test_silero_finds_no_utterance_in_silence below.
    assert 4.5 < utterance.duration_s <= 5.61


@_requires_silero
def test_silero_finds_no_utterance_in_silence():
    # ASR hallucinates on non-speech -- handed silence it returns a plausible Chinese word
    # rather than nothing -- so a segment that never opens is the only real defence.
    assert _push(_silero_vad(), [_frame(0)] * 200) == []


@_requires_silero
def test_silero_reset_returns_the_model_to_its_initial_state():
    config = AiRuntimeConfig()
    frames = _speech_frames()
    fresh = SileroSpeechDetector(config)
    reused = SileroSpeechDetector(config)
    # Drive unrelated audio through one of them, then reset it. Matching verdicts are what
    # "the model remembers nothing across the Speak gate" means in observable terms.
    for frame in frames[60:120]:
        reused.is_speech(frame)
    reused.reset()

    assert [fresh.is_speech(f) for f in frames[:60]] == [
        reused.is_speech(f) for f in frames[:60]
    ]
