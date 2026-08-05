"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_siren.py
Brief: Unit tests for the from-scratch numpy 驱离 siren synthesiser.

Description:
  Covers the development plan section 13 acceptance points for the deter siren that
  need no hardware: the synthesiser must emit 8 kHz mono int16 PCM of the requested
  length, sweep only inside the verified 600..1500 Hz band, scale linearly with the
  master level, bake in the amplitude accent when asked, stay deterministic (a pure
  function of its SirenSpec), and reject illegal parameters with a typed SirenError.
  Run from the /opt/xbrain_v6 root:
      python3 -m pytest tests/payload/test_siren.py
"""
from __future__ import annotations

import numpy as np
import pytest

from services.payload.core.siren import (
    SirenError,
    SirenSpec,
    _frequency_track_hz,
    synthesize,
)

_FS = 8000
_F_LO = 600.0
_F_HI = 1500.0


def test_default_clip_is_8k_mono_int16() -> None:
    # The output must be the exact wire format the 8519 [42] hail path consumes: a
    # 1-D signed-16-bit little-endian mono array, so .tobytes() is s16le with no
    # reshape or resample. The default clip is 6 s at 8 kHz = 48000 samples.
    clip = synthesize()
    assert clip.dtype == np.dtype("<i2")
    assert clip.ndim == 1
    assert clip.size == int(6.0 * _FS)
    # int16 is two bytes per sample; the raw byte view must be exactly that long.
    assert len(clip.tobytes()) == clip.size * 2


@pytest.mark.parametrize("seconds", [0.5, 2.0, 6.0])
def test_length_tracks_requested_seconds(seconds: float) -> None:
    # Clip length must be round(seconds*fs) for any duration, since the deter loop
    # sizes its playback from this.
    clip = synthesize(SirenSpec(seconds=seconds))
    assert clip.size == int(round(seconds * _FS))


def test_frequency_track_stays_in_band() -> None:
    # The fundamental sweep must stay within the hardware-verified 600..1500 Hz band.
    # This is checked on the frequency TRACK, not on an FFT of the rendered audio,
    # because the timbre harmonics (2x, 3x the fundamental) intentionally extend the
    # output spectrum above 1500 Hz -- the band constraint applies to the swept
    # fundamental, which is exactly what _frequency_track_hz returns.
    track = _frequency_track_hz(SirenSpec())
    assert float(track.min()) >= _F_LO - 1e-6
    assert float(track.max()) <= _F_HI + 1e-6


def test_level_scales_peak_linearly() -> None:
    # The master level is a linear amplitude gain, so doubling it must double the peak
    # sample (within one LSB of int16 rounding). This pins level as a gain rather than
    # some perceptual curve.
    peak_lo = int(np.max(np.abs(synthesize(SirenSpec(level=0.45)))))
    peak_hi = int(np.max(np.abs(synthesize(SirenSpec(level=0.90)))))
    assert abs(peak_hi - 2 * peak_lo) <= 2


def test_level_zero_is_silent() -> None:
    # level 0 is a legal value (0..1 inclusive) and must render pure silence, not an
    # error and not a faint residual -- a guard that nothing adds a DC or floor after
    # the level scaling.
    clip = synthesize(SirenSpec(level=0.0))
    assert int(np.max(np.abs(clip))) == 0


def test_accent_changes_the_waveform() -> None:
    # The amplitude accent (声光同拍) must actually modulate the samples when enabled,
    # and must not change the clip length. Off-by-default vs a positive rate/depth
    # therefore produce different bytes of the same size.
    plain = synthesize(SirenSpec(accent_hz=0.0))
    accented = synthesize(SirenSpec(accent_hz=1.389, accent_depth=0.22))
    assert plain.size == accented.size
    assert not np.array_equal(plain, accented)


def test_synthesis_is_deterministic() -> None:
    # synthesize() is a pure function of its spec: identical specs must yield
    # byte-identical PCM so the deter loop can render once and replay, and so tests
    # can assert exact bytes. Also confirms the None default equals an explicit spec.
    assert np.array_equal(synthesize(), synthesize())
    assert np.array_equal(synthesize(), synthesize(SirenSpec()))


@pytest.mark.parametrize(
    "spec",
    [
        SirenSpec(seconds=0.0),          # non-positive duration
        SirenSpec(seconds=-1.0),         # negative duration
        SirenSpec(level=1.5),            # level above 1
        SirenSpec(level=-0.1),           # level below 0
        SirenSpec(accent_hz=-1.0),       # negative accent rate
        SirenSpec(accent_depth=1.5),     # depth above 1
        SirenSpec(f_lo=1500.0, f_hi=600.0),  # inverted band
        SirenSpec(f_lo=700.0, f_hi=700.0),   # collapsed band (f_lo == f_hi)
    ],
)
def test_invalid_spec_raises_siren_error(spec: SirenSpec) -> None:
    # Validation lives in synthesize (a frozen SirenSpec does not self-validate), so
    # each illegal parameter must surface as a typed SirenError when rendered, naming
    # the bad field rather than failing later as an opaque numpy error.
    with pytest.raises(SirenError):
        synthesize(spec)
