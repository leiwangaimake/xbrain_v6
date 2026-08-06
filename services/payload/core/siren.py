"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: siren.py
Brief: numpy synthesis of the 驱离 (deter) Wail+Yelp police siren as 8 kHz mono PCM.

Description:
  Renders the deterrent-mode siren the 功能3 驱离 loop plays through the robot
  loudspeaker. The output is 16-bit mono PCM at 8000 Hz -- exactly the format the 8519
  [42] hail path (Opus 8k/mono) consumes -- so a later milestone can frame these
  samples straight into Opus packets with no resampling. This module only SYNTHESISES
  the waveform; it opens no socket and does no Opus encoding. Streaming it to the
  device is deter.py's job on the M3 audio path.

  From scratch (test-system rule R0): the reference tool tools/siren_gen.py was read
  only to recover the hardware-verified tuning (sweep band, sweep periods, segment
  lengths, timbre), never imported and never copied. The DSP below -- raised-cosine
  sweep control, phase integration, harmonic sum, amplitude accent, edge fades -- is
  written here from the mathematical definitions.

  How the siren is built, and WHY each step is shaped the way it is:
    1. Frequency track. A police siren is a tone whose pitch sweeps up and down. The
       sweep is driven by a RAISED-COSINE control (0 -> 1 -> 0), not a linear ramp,
       because a raised cosine is continuous in both value AND slope at its endpoints,
       so tiling many sweeps back to back introduces no corner that would be heard as
       a tick. "Wail" is a slow sweep (multi-second period) and "Yelp" is a fast sweep
       (sub-second period); the deter siren alternates a Wail segment then a Yelp
       segment, tiled to fill the requested length. Wail and Yelp differ ONLY in the
       sweep period -- same band, same math -- which is why one segment builder serves
       both.
    2. Phase by integration. The oscillator phase is the running integral of the
       instantaneous frequency (phase = 2*pi * cumsum(f) / fs), computed ONCE over the
       whole concatenated track. Integrating the frequency -- rather than restarting a
       sine per segment -- is what makes the Wail/Yelp seams and the sweep turning
       points phase-continuous, so there is no click anywhere the frequency changes.
    3. Harmonics. Summing the fundamental with two quieter integer harmonics gives the
       bright, electronic-siren timbre a pure sine lacks. The sum is normalised by the
       total harmonic weight so adding harmonics cannot by itself drive the signal
       past full scale before the master level is applied.
    4. Amplitude accent (optional, 声光同拍). When enabled, the amplitude is pulsed at
       the red/blue flash GROUP rate so the sound swells in time with the light bursts.
       This must be baked in HERE, at synthesis time, because it modulates the waveform
       itself -- it cannot be added after the clip is rendered. It is off by default;
       deter.py (M3) computes the flash-group rate and passes it in. That is the sole
       reason accent_hz is a parameter rather than being hardcoded or omitted: the
       artifact this module exists to produce is the deter-ready siren, and the accent
       is part of that artifact.
    5. Level and fades. The whole clip is scaled by a master level in 0..1, then given
       a few-millisecond fade in and fade out. The fades matter because the deter loop
       plays this clip repeatedly: without them the wrap from the last sample back to
       the first would step discontinuously and click once per loop.

  Numerical note: all synthesis runs in float64 and is quantised to int16 only at the
  very end. The phase is a cumulative sum over the whole clip (48000 samples for the
  6 s default), and float64 keeps that running integral precise enough that the tone
  stays on pitch at the clip's end instead of drifting from accumulated rounding.
  Doing the sum in an integer or float32 type would let that error build up audibly
  over a long clip, which is why the working type stays float64 until the final cast.

  Statelessness: synthesize() is a pure function of its SirenSpec -- no globals, no
  clock, no I/O -- so identical specs yield byte-identical PCM. deter.py can therefore
  render the clip once and replay it every loop, or render it on a worker thread with
  no locking, and a test can assert exact bytes rather than fuzzy audio properties.

  The result is quantised to little-endian int16. Callers that need the raw 8519 wire
  bytes take .tobytes() on the returned array; callers that need to frame it for Opus
  (deter.py, M3) consume the sample array directly.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Sample rate of the produced PCM. Fixed at the 8519 hail rate so the samples feed the
# [42] Opus path with no resampling stage between synthesis and the wire.
_FS_HAIL_HZ = 8000


class SirenError(ValueError):
    """Raised when siren parameters cannot produce a valid waveform.

    House rule bans bare Exception; a dedicated type lets a caller (for example the
    future POST /deter handler validating a siren_level from a request body) tell an
    illegal-parameter fault apart from an unrelated error. It subclasses ValueError
    because every case it covers is a bad input value -- a non-positive duration, a
    level outside 0..1, a negative accent rate, or an inverted frequency band.
    """


@dataclass(frozen=True)
class SirenSpec:
    """Complete, immutable description of one siren clip to render.

    A single frozen DTO carries every knob so synthesize() takes one argument and a
    caller (deter.py, or a test) states exactly the clip it wants. Frozen because a
    spec is a value, not a mutable session object: to render a different clip you build
    a new SirenSpec rather than mutating a shared one, which removes any chance of one
    caller's edit perturbing another's render.

    The defaults are the values verified on real hardware for the deter mode (recovered
    from the reference tool, not copied): a 6 s clip at the office-quiet level, the
    600..1500 Hz band, a 4 s Wail period and 0.30 s Yelp period, 3.5 s Wail then 2.5 s
    Yelp per cycle, and a three-term harmonic timbre. Only the per-invocation fields
    (seconds, level, accent_hz) are normally overridden; the rest describe the fixed
    character of THIS device's siren and rarely change.

    Fields:
        seconds: clip length in seconds; must be > 0.
        level: master amplitude 0..1 (POST /deter's siren_level maps here). The default
            is deliberately quiet (0.45), matching the verified indoor deter default.
        accent_hz: amplitude-accent rate in Hz, 0 disables the accent. deter.py sets it
            to the red/blue flash-group rate so sound and light pulse together.
        accent_depth: accent strength 0..1, applied only when accent_hz > 0.
        fs: sample rate; defaults to the 8519 hail rate and rarely changes.
        f_lo, f_hi: sweep band bottom/top in Hz; f_lo must be < f_hi.
        wail_period_s, yelp_period_s: seconds per up-down sweep for each mode.
        wail_len_s, yelp_len_s: seconds of Wail then Yelp per alternation cycle.
        harmonics: per-harmonic amplitudes (index 1 = fundamental) shaping the timbre.
        fade_s: fade in/out length in seconds so a looped clip has no seam click.
    """

    seconds: float = 6.0
    level: float = 0.45
    # * Amplitude accent rate, 1.39 Hz = 1 / the 0.72 s red/blue flash group. This is the
    # value the deterrent prototype has always generated its siren with, and it is part of
    # what gives the clip its electronic-siren character rather than a plain tone sweep;
    # rendering with the accent off produces a noticeably flatter sound.
    #
    # ! The number is NOT a promise that the accent lands on the flashes. 14 DT-6:
    # "声光同拍在本架构下只能近似 ... accent 1.39 Hz 是警笛 WAV 的生成参数,红蓝节拍是固件
    # 动画 ---- 两者没有共同时基,无法对齐相位". The prototype could align them because it
    # drove both from one process; here the firmware animates the lights on its own clock.
    # 14 §7.3 accepts that explicitly: 声光不同拍只影响观感,不影响威慑功能与安全.
    accent_hz: float = 1.39
    accent_depth: float = 0.22
    fs: int = _FS_HAIL_HZ
    f_lo: float = 600.0
    f_hi: float = 1500.0
    wail_period_s: float = 4.0
    yelp_period_s: float = 0.30
    wail_len_s: float = 3.5
    yelp_len_s: float = 2.5
    harmonics: Tuple[float, ...] = (1.0, 0.35, 0.12)
    fade_s: float = 0.008


def _validate(spec: SirenSpec) -> None:
    """Reject a spec that cannot yield a sane waveform, with a clear typed error.

    Only the parameters a caller realistically gets wrong are checked -- the clip
    length, the master level, the accent settings, and the frequency band ordering --
    because those are the fields a POST /deter request drives. Catching a bad value
    here means the failure names the offending field instead of surfacing later as an
    opaque numpy error (an empty array, a NaN, or silent full-scale distortion).
    """
    # A non-positive duration would make an empty sample array downstream.
    if spec.seconds <= 0.0:
        raise SirenError(f"seconds must be > 0, got {spec.seconds}")
    # level scales the final waveform; outside 0..1 it would clip to noise or silence.
    if not 0.0 <= spec.level <= 1.0:
        raise SirenError(f"level must be within 0..1, got {spec.level}")
    # A negative accent rate has no meaning; 0 is the documented "accent off" value.
    if spec.accent_hz < 0.0:
        raise SirenError(f"accent_hz must be >= 0, got {spec.accent_hz}")
    # Depth is a 0..1 mix fraction; outside that it would over- or under-modulate.
    if not 0.0 <= spec.accent_depth <= 1.0:
        raise SirenError(f"accent_depth must be within 0..1, got {spec.accent_depth}")
    # A non-ascending band would invert the sweep or collapse it to a constant tone.
    if spec.f_lo >= spec.f_hi:
        raise SirenError(f"f_lo ({spec.f_lo}) must be < f_hi ({spec.f_hi})")


def _raised_cosine_lfo(n_samples: int, fs: int, period_s: float) -> np.ndarray:
    """Return an n-sample 0->1->0 control that is smooth in value and slope.

    Args:
        n_samples: length of the control array.
        fs: sample rate the samples are spaced at.
        period_s: seconds for one full 0->1->0 excursion.

    A raised cosine (0.5 - 0.5*cos) is used instead of a triangle or a linear ramp
    precisely because its derivative is also zero at the 0 and 1 turning points. That
    matters when many of these drive back-to-back sweeps: a shape with a slope
    discontinuity at the turn would put a corner in the frequency track, and a corner
    in frequency is audible as a periodic tick. The cosine has no such corner.
    """
    # Sample times in seconds; phase advances 2*pi per period_s.
    t = np.arange(n_samples) / fs
    return 0.5 - 0.5 * np.cos(2.0 * np.pi * t / period_s)


def _sweep_segment_hz(spec: SirenSpec, dur_s: float, period_s: float) -> np.ndarray:
    """Instantaneous-frequency track (Hz per sample) for one sweep segment.

    Args:
        spec: the siren spec supplying fs and the f_lo..f_hi band.
        dur_s: segment length in seconds.
        period_s: seconds per up-down sweep (small = Yelp, large = Wail).

    The raised-cosine LFO in 0..1 is mapped linearly onto the frequency band, so the
    tone sweeps f_lo -> f_hi -> f_lo once per period_s. Wail and Yelp are the SAME
    computation with only period_s differing, which is why this one builder produces
    both and there is no separate Yelp code path to keep in sync.
    """
    n = int(round(dur_s * spec.fs))
    lfo = _raised_cosine_lfo(n, spec.fs, period_s)
    return spec.f_lo + (spec.f_hi - spec.f_lo) * lfo


def _frequency_track_hz(spec: SirenSpec) -> np.ndarray:
    """Tile alternating Wail and Yelp segments into one per-sample frequency track.

    Args:
        spec: the siren spec supplying the segment lengths, periods and total seconds.

    Returns:
        A frequency-per-sample array exactly round(seconds*fs) samples long.

    Segments are appended Wail, Yelp, Wail, Yelp... until at least the requested length
    is reached, then the concatenation is truncated to the exact sample count. The last
    segment may be cut mid-sweep; that is harmless because phase is integrated over the
    final track downstream, so the truncation cannot create a discontinuity -- it just
    ends the clip partway through a sweep, which the loop fade then smooths.
    """
    total_n = int(round(spec.seconds * spec.fs))
    # (segment length, sweep period) for the two alternating modes, in play order.
    order = (
        (spec.wail_len_s, spec.wail_period_s),
        (spec.yelp_len_s, spec.yelp_period_s),
    )
    segments = []
    have = 0
    i = 0
    # Keep laying down whole segments until we have at least the samples we need; the
    # overshoot is trimmed once at the end rather than by shortening the last segment,
    # which keeps this loop simple and the trim is at most one segment of waste.
    while have < total_n:
        seg_len_s, period_s = order[i % 2]
        seg = _sweep_segment_hz(spec, seg_len_s, period_s)
        segments.append(seg)
        have += len(seg)
        i += 1
    return np.concatenate(segments)[:total_n]


def _apply_fades(sig: np.ndarray, fs: int, fade_s: float) -> np.ndarray:
    """Fade the first and last fade_s seconds in/out so a looped clip has no click.

    Args:
        sig: the float waveform to fade, modified in place and returned.
        fs: sample rate, to convert fade_s to a sample count.
        fade_s: fade length in seconds.

    The deter loop replays this clip end to end, so the wrap point (last sample back to
    first) is a real seam a listener hears every cycle. Ramping the edges to zero makes
    both ends meet silence, so the seam is inaudible. The guard skips the fade when the
    clip is too short to hold two non-overlapping ramps, since fading would otherwise
    eat the whole signal.
    """
    fade = int(fade_s * fs)
    # Need room for a distinct head and tail ramp; otherwise leave the signal as is.
    if fade > 0 and len(sig) > 2 * fade:
        ramp = np.linspace(0.0, 1.0, fade)
        # Head ramps 0->1; tail ramps 1->0 (the reversed ramp).
        sig[:fade] *= ramp
        sig[-fade:] *= ramp[::-1]
    return sig


def synthesize(spec: Optional[SirenSpec] = None) -> np.ndarray:
    """Render a deter siren clip as little-endian int16 mono PCM.

    Args:
        spec: the clip description; when None the verified deter defaults are used.

    Returns:
        A 1-D numpy array of dtype '<i2' (signed 16-bit little-endian), one mono
        channel at spec.fs. Take .tobytes() for the raw 8519 wire bytes, or consume the
        array directly to frame it for Opus.

    Raises:
        SirenError: if the spec parameters cannot produce a valid waveform.

    The pipeline is: build the Wail/Yelp frequency track, integrate it once to phase
    (so every seam is click-free), sum the harmonics for timbre and normalise by their
    total weight, optionally apply the flash-locked amplitude accent, scale by the
    master level, fade the edges for a clean loop seam, and quantise with a clip to
    guard the rare overshoot. Each stage is separated so the WHY of each (documented on
    the helpers) stays legible and independently testable.
    """
    # Default to the verified deter spec when the caller passes nothing.
    spec = spec or SirenSpec()
    _validate(spec)

    # 1) Per-sample frequency track for the whole clip (Wail/Yelp tiled).
    f_inst = _frequency_track_hz(spec)
    n = len(f_inst)

    # 2) Integrate frequency to phase ONCE over the full track. cumsum/fs is the
    #    running integral in cycles; times 2*pi gives radians. Doing it over the whole
    #    track (not per segment) is what makes every seam phase-continuous.
    phase = 2.0 * np.pi * np.cumsum(f_inst) / spec.fs

    # 3) Harmonic sum for the electronic-siren timbre: fundamental at k=1 plus quieter
    #    integer harmonics. Normalising by the summed absolute weights keeps the result
    #    within +-1 before the master level, so harmonics alone never force clipping.
    # enumerate from 1 so k is the harmonic number: k=1 is the fundamental at the swept
    # frequency, k=2/k=3 are the octave and octave-plus-fifth that add brightness.
    sig = np.zeros(n, dtype=np.float64)
    for k, amp in enumerate(spec.harmonics, start=1):
        sig += amp * np.sin(k * phase)
    sig /= sum(abs(a) for a in spec.harmonics)

    # 4) Optional amplitude accent locked to the flash group rate (声光同拍). Skipped
    #    unless a positive rate and depth are set, so a plain siren is unaffected. The
    #    ((t*hz) % 1.0) builds a 0..1 sawtooth phase at accent_hz that the raised cosine
    #    turns into a smooth per-group swell; depth sets how deep the swell cuts.
    if spec.accent_hz > 0.0 and spec.accent_depth > 0.0:
        t = np.arange(n) / spec.fs
        bump = 0.5 - 0.5 * np.cos(2.0 * np.pi * ((t * spec.accent_hz) % 1.0))
        sig *= (1.0 - spec.accent_depth) + spec.accent_depth * bump

    # 5) Master level, then edge fades for a click-free loop seam.
    sig *= spec.level
    sig = _apply_fades(sig, spec.fs, spec.fade_s)

    # 6) Quantise to int16. Clip first so a rare sample nudged past full scale by the
    #    harmonic sum saturates cleanly instead of wrapping to the opposite polarity.
    return (np.clip(sig, -1.0, 1.0) * 32767.0).astype("<i2")
