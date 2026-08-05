"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: resample.py
Brief: Linear sample-rate conversion for mono 16-bit PCM (the /play 16k -> 8k step).

Description:
  The audio data plane speaks 16 kHz PCM to browser clients but the device's realtime
  hail port ([42] on 8519) wants 8 kHz Opus, so /play must down-sample 16k -> 8k before
  encoding (development plan section 5.2). This module does only that sample-rate step
  on raw mono s16le bytes; the /mic path needs no conversion (it already streams the
  device's 16k up to the client unchanged) and so does not call in here.

  The conversion is naive linear interpolation, matching the real-machine-verified probe
  behaviour: for output sample i the source position is i*sr_in/sr_out, and the value is
  the linear blend of the two nearest input samples (edges clamped). This is written
  from the algorithm, not copied from the probe (test-system rule R0).

  Known tradeoff (deliberately accepted): linear down-sampling has no anti-alias
  low-pass, so frequencies above the 8k Nyquist fold back as aliasing. For speech hail
  at these rates it is inaudible in practice and is what the verified probe already did,
  so a heavier polyphase/FIR resampler is not built here (house V1/V2 discipline -- add
  it only if a real playback problem ever demands it).
"""
from __future__ import annotations

import numpy as np

# int16 little-endian: the one PCM wire format the whole audio plane uses. Named once so
# the dtype and the "2 bytes per sample" assumption below cannot drift apart.
_PCM_DTYPE = np.dtype("<i2")
_BYTES_PER_SAMPLE = 2


class ResampleError(ValueError):
    """Raised when PCM cannot be resampled from the given arguments.

    House rule bans bare Exception; a dedicated type lets the /play handler tell a
    malformed-input fault -- a non-positive rate, or a byte buffer whose length is not a
    whole number of int16 samples -- apart from an unrelated error. It subclasses
    ValueError because every case is a bad input value.
    """


def resample_linear(pcm_s16le: bytes, sr_in: int, sr_out: int) -> bytes:
    """Resample mono 16-bit little-endian PCM from sr_in to sr_out by linear interp.

    Args:
        pcm_s16le: raw mono PCM, signed 16-bit little-endian (2 bytes per sample).
        sr_in: the sample rate of pcm_s16le, in Hz (must be > 0).
        sr_out: the desired output sample rate, in Hz (must be > 0).

    Returns:
        The resampled PCM in the same mono s16le format. When sr_in == sr_out the input
        bytes are returned unchanged.

    Raises:
        ResampleError: if either rate is not positive, or len(pcm_s16le) is not a whole
            number of int16 samples (odd byte count). Failing here keeps a malformed
            buffer from reaching numpy as an opaque reshape/frombuffer error.
    """
    # Boundary validation: both rates must be positive (they divide below) and the byte
    # buffer must hold whole int16 samples, or the frombuffer view would be malformed.
    if sr_in <= 0 or sr_out <= 0:
        raise ResampleError(f"sample rates must be positive, got sr_in={sr_in} sr_out={sr_out}")
    if len(pcm_s16le) % _BYTES_PER_SAMPLE != 0:
        raise ResampleError(f"pcm length {len(pcm_s16le)} is not a whole number of int16 samples")

    # Equal rates need no work; return the exact bytes so no rounding drift is introduced
    # on the common /play-at-8k case where the client already sent 8 kHz.
    if sr_in == sr_out:
        return pcm_s16le

    # View the bytes as int16 samples without copying. An empty buffer resamples to empty.
    samples = np.frombuffer(pcm_s16le, dtype=_PCM_DTYPE)
    n_in = samples.size
    if n_in == 0:
        return b""

    # Output length scales by the rate ratio; keep at least one sample so a tiny non-empty
    # buffer never collapses to silence.
    n_out = max(1, int(n_in * sr_out / sr_in))
    # Map each output index back to a fractional source position, then let np.interp do
    # the linear blend of the two neighbouring samples. np.interp clamps positions past
    # the last input sample to that sample, which is exactly the edge behaviour the
    # per-sample formula (i1 = min(i0+1, n_in-1)) specifies.
    src_positions = np.arange(n_out, dtype=np.float64) * (sr_in / sr_out)
    interpolated = np.interp(src_positions, np.arange(n_in, dtype=np.float64), samples.astype(np.float64))
    # Round to nearest before narrowing to int16. Rounding (not truncation) avoids a small
    # bias toward zero; the blend of two in-range samples is itself in range, so no clip.
    return np.round(interpolated).astype(_PCM_DTYPE).tobytes()
