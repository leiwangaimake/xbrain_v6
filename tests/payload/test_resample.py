"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_resample.py
Brief: Unit tests for the from-scratch linear PCM resampler (the /play 16k -> 8k step).

Description:
  Covers the development plan section 13 acceptance points that need no hardware for the
  resampler: it must keep the mono s16le format, size the output by the exact rate ratio,
  pass equal-rate audio through untouched, preserve a constant (DC) signal, produce the
  textbook linear-interpolation values, and reject malformed input with a typed
  ResampleError. Run from the /opt/xbrain_v6 root:
      python3 -m pytest tests/payload/test_resample.py
"""
from __future__ import annotations

import numpy as np
import pytest

from services.payload.codec.resample import ResampleError, resample_linear


def _tone(fs: int, n: int, hz: float = 440.0) -> bytes:
    # A short mono s16le sine used as generic PCM to resample; content is irrelevant to
    # the structural assertions, only its length (n samples) matters.
    t = np.arange(n)
    return (3000.0 * np.sin(2 * np.pi * hz * t / fs)).astype("<i2").tobytes()


def test_identity_returns_input_unchanged() -> None:
    # Equal rates must be a pass-through of the exact bytes -- no rounding drift on the
    # common case where the client already sent audio at the device rate.
    pcm = _tone(16000, 320)
    assert resample_linear(pcm, 16000, 16000) == pcm


def test_downsample_16k_to_8k_halves_sample_count() -> None:
    # The /play direction: 20 ms at 16k (320 samples) must become 160 samples at 8k, and
    # stay mono s16le (an even byte count that views cleanly as int16).
    out = resample_linear(_tone(16000, 320), 16000, 8000)
    assert len(out) % 2 == 0
    assert len(out) // 2 == 160


def test_upsample_8k_to_16k_doubles_sample_count() -> None:
    # The reverse ratio must double the sample count, confirming n_out scales with
    # sr_out/sr_in in both directions.
    out = resample_linear(_tone(8000, 160), 8000, 16000)
    assert len(out) // 2 == 320


def test_empty_input_returns_empty() -> None:
    # Zero samples in must give zero samples out, not an error or a stray frame.
    assert resample_linear(b"", 16000, 8000) == b""


def test_constant_signal_is_preserved() -> None:
    # A DC (constant) signal is the cleanest linearity check: every output sample is a
    # blend of two identical inputs, so the constant must survive resampling exactly.
    pcm = np.full(200, 1234, dtype="<i2").tobytes()
    out = np.frombuffer(resample_linear(pcm, 16000, 8000), dtype="<i2")
    assert np.all(out == 1234)


def test_linear_interpolation_values() -> None:
    # Pin the actual interpolation math: samples [0, 100] upsampled 2x map output indices
    # to source positions [0, 0.5, 1.0, 1.5]; the linear blend (with the past-end position
    # clamped to the last sample) is exactly [0, 50, 100, 100].
    pcm = np.array([0, 100], dtype="<i2").tobytes()
    out = np.frombuffer(resample_linear(pcm, 8000, 16000), dtype="<i2")
    assert list(out) == [0, 50, 100, 100]


@pytest.mark.parametrize(
    "sr_in,sr_out",
    [(0, 8000), (16000, 0), (-1, 8000), (16000, -1)],
)
def test_rejects_nonpositive_rate(sr_in: int, sr_out: int) -> None:
    # A non-positive rate would divide by zero or invert the mapping; it must fail as a
    # typed ResampleError at the boundary, not as an opaque numpy error deeper in.
    with pytest.raises(ResampleError):
        resample_linear(_tone(16000, 320), sr_in, sr_out)


def test_rejects_odd_byte_length() -> None:
    # int16 PCM is two bytes per sample; an odd byte count cannot be whole samples and
    # must be refused rather than fed to frombuffer as a malformed view.
    with pytest.raises(ResampleError):
        resample_linear(b"\x00\x01\x02", 16000, 8000)
