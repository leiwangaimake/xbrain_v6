"""BIZ-P2-3 -- audio_io decimation math + AudioFrame invariants."""

import pytest

from xbrain.p2_core.audio.audio_io import (
    ASR_RATE_HZ, ASR_SAMPLES_PER_FRAME,
    AlsaCaptureUnavailable,
    AudioFrame, CAPTURE_RATE_HZ, CAPTURE_SAMPLES_PER_FRAME,
    DECIMATION_FACTOR, FRAME_MS,
    build_frame, decimate_3to1,
)


pytestmark = pytest.mark.no_device


# --- Constants match doc + arithmetic ---

def test_constants_match_doc_and_arithmetic():
    assert CAPTURE_RATE_HZ == 48_000
    assert ASR_RATE_HZ == 16_000
    assert DECIMATION_FACTOR == 3
    assert FRAME_MS == 20
    assert CAPTURE_SAMPLES_PER_FRAME == 960   # 48k * 20ms
    assert ASR_SAMPLES_PER_FRAME == 320       # 16k * 20ms


# --- decimation math ---

def test_decimate_length():
    inp = [0] * CAPTURE_SAMPLES_PER_FRAME
    out = decimate_3to1(inp)
    assert len(out) == ASR_SAMPLES_PER_FRAME


def test_decimate_silence_stays_silent():
    inp = [0] * CAPTURE_SAMPLES_PER_FRAME
    out = decimate_3to1(inp)
    assert all(s == 0 for s in out)


def test_decimate_dc_offset_preserved():
    """Constant input should stay approximately constant (3-tap MA
    of the same value is that value)."""
    inp = [1000] * CAPTURE_SAMPLES_PER_FRAME
    out = decimate_3to1(inp)
    for s in out:
        assert s == 1000


def test_decimate_clamps_to_int16_range():
    """3-tap MA can't overshoot input range, but explicit clamp is
    the last line of defense."""
    inp = [32_000] * CAPTURE_SAMPLES_PER_FRAME
    out = decimate_3to1(inp)
    for s in out:
        assert -32768 <= s <= 32767


def test_decimate_rejects_wrong_input_length():
    with pytest.raises(ValueError):
        decimate_3to1([0] * 100)   # not 960


# --- AudioFrame invariants ---

def test_audio_frame_rejects_wrong_rate():
    with pytest.raises(ValueError):
        AudioFrame(
            rate_hz=48000, channels=1, sample_width=2,
            frame_ms=20, samples=[0] * 320,
        )


def test_audio_frame_rejects_wrong_channels():
    with pytest.raises(ValueError):
        AudioFrame(
            rate_hz=16000, channels=2, sample_width=2,
            frame_ms=20, samples=[0] * 320,
        )


def test_audio_frame_rejects_wrong_sample_width():
    with pytest.raises(ValueError):
        AudioFrame(
            rate_hz=16000, channels=1, sample_width=4,
            frame_ms=20, samples=[0] * 320,
        )


def test_audio_frame_rejects_wrong_sample_count():
    with pytest.raises(ValueError):
        AudioFrame(
            rate_hz=16000, channels=1, sample_width=2,
            frame_ms=20, samples=[0] * 100,
        )


# --- build_frame end-to-end ---

def test_build_frame_from_valid_input():
    inp = [500] * CAPTURE_SAMPLES_PER_FRAME
    frame = build_frame(inp)
    assert isinstance(frame, AudioFrame)
    assert frame.rate_hz == 16000
    assert len(frame.samples) == 320


# --- ALSA capture: skeleton returns AlsaCaptureUnavailable
#     when arecord absent (on this CI host). Not a failure -- the
#     test proves the fail-safe path exists.

def test_open_capture_fails_loudly_when_arecord_missing(monkeypatch):
    """The fail-safe direction: when arecord isn't available, raise
    AlsaCaptureUnavailable (caller maps to mic=device_fault). Do
    NOT return a dead Popen or None."""
    import xbrain.p2_core.audio.audio_io as mod
    import shutil
    monkeypatch.setattr(shutil, "which", lambda _: None)
    with pytest.raises(AlsaCaptureUnavailable):
        mod.open_capture()
