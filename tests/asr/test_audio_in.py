"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_audio_in.py
Brief: Unit tests for the ASR upload decoder -- wav parsing, raw PCM, format contract.

Description:
  Exercises core/audio_in against wav bytes built in memory with the stdlib, so the tests
  need no fixture files and can construct the awkward cases on demand: a wav carrying an
  extra chunk between "fmt " and "data" (which the model's own test_wavs do, and which
  breaks any parser that assumes audio starts at offset 44), a stereo file, a 24-bit file,
  and a truncated raw buffer.

  The assertions check both halves of the contract: accepted audio must round-trip to the
  right sample VALUES and the right rate -- a decoder that scaled or byte-swapped wrongly
  would still produce plausible-looking arrays -- and rejected audio must raise
  AudioInputError, because that is the exception the REST layer maps to HTTP 400.

  Import reach: this module imports numpy (via services.asr.core.audio_in) but nothing
  from sherpa-onnx or FastAPI, so it runs on the Orin's python3.10 as-is. Run from the
  /opt/xbrain_v6 root:
      python3 -m pytest tests/asr/test_audio_in.py
"""
from __future__ import annotations

import io
import struct
import wave

import numpy as np
import pytest

from services.asr.core.audio_in import AudioInputError, decode_upload, decode_wav

# int16 full scale, matching the decoder's normalization divisor. Declared here rather
# than imported so the test asserts against an independently stated constant.
_FULL_SCALE = 32768.0


def _wav_bytes(samples, *, rate: int = 16000, channels: int = 1, width: int = 2) -> bytes:
    """Build a wav file in memory from int16 sample values."""
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as writer:
        writer.setnchannels(channels)
        writer.setsampwidth(width)
        writer.setframerate(rate)
        writer.writeframes(b"".join(struct.pack("<h", value) for value in samples))
    return buffer.getvalue()


def _with_extra_chunk(wav: bytes) -> bytes:
    """Splice a LIST/INFO chunk between the fmt and data chunks of a wav.

    This reproduces the layout of the shipped model/test_wavs files, which were written by
    ffmpeg and carry metadata before the audio. The RIFF size field is patched so the file
    stays well-formed.
    """
    data_at = wav.index(b"data")
    extra = b"LIST" + struct.pack("<I", 4) + b"INFO"
    spliced = wav[:data_at] + extra + wav[data_at:]
    # Bytes 4..8 hold "everything after this field", so it grows by the inserted chunk.
    riff_size = struct.unpack_from("<I", spliced, 4)[0] + len(extra)
    return spliced[:4] + struct.pack("<I", riff_size) + spliced[8:]


def test_decode_wav_returns_samples_and_rate() -> None:
    # Values must survive as float32 in [-1, 1) at the file's own declared rate: a wrong
    # divisor or a byte swap would still yield an array, just a quietly wrong one.
    values = [0, 1000, -1000, 32767, -32768]
    audio = decode_wav(_wav_bytes(values, rate=16000))
    assert audio.sample_rate == 16000
    assert audio.samples.dtype == np.float32
    assert np.allclose(audio.samples, np.array(values, dtype=np.float32) / _FULL_SCALE)


def test_decode_wav_honours_its_own_rate() -> None:
    # An 8 kHz wav must report 8000, not the 16 kHz house default -- the engine resamples
    # from whatever rate it is told, so a wrong rate here becomes a pitch-shifted decode.
    audio = decode_wav(_wav_bytes([0, 1, 2, 3], rate=8000))
    assert audio.sample_rate == 8000
    assert len(audio.samples) == 4


def test_decode_wav_skips_intervening_chunks() -> None:
    # The shipped test_wavs carry a LIST/INFO chunk before "data". Reading that metadata
    # as audio is the classic fixed-offset parser bug, so it is asserted against directly.
    values = [100, 200, 300]
    audio = decode_wav(_with_extra_chunk(_wav_bytes(values)))
    assert np.allclose(audio.samples, np.array(values, dtype=np.float32) / _FULL_SCALE)


def test_duration_uses_rate_and_length() -> None:
    # duration_s is the denominator of the reported RTF, so it must track the real rate.
    audio = decode_wav(_wav_bytes([0] * 8000, rate=8000))
    assert audio.duration_s == pytest.approx(1.0)


def test_non_wav_bytes_rejected() -> None:
    # Anything RIFF-shaped but unreadable must surface as this module's own error type,
    # since that is what the REST layer maps to a 400.
    with pytest.raises(AudioInputError):
        decode_wav(b"RIFFnot-really-a-wav")


def test_stereo_and_wide_samples_rejected() -> None:
    # The whole audio plane is mono s16le. Silently dropping a channel or truncating
    # 24-bit samples would let a genuinely mis-wired pipeline produce plausible text.
    with pytest.raises(AudioInputError):
        decode_wav(_wav_bytes([0, 0, 0, 0], channels=2))
    with pytest.raises(AudioInputError):
        decode_wav(_wav_bytes([0, 0, 0, 0], width=3))


def test_empty_wav_rejected() -> None:
    # A header-only wav would hand the engine an empty waveform, whose empty transcript
    # looks like a recognition failure rather than the upload bug it is.
    with pytest.raises(AudioInputError):
        decode_wav(_wav_bytes([]))


def test_decode_upload_sniffs_wav_versus_raw() -> None:
    # One endpoint accepts both shapes on one field, decided by the RIFF magic -- not by a
    # filename, which a wrong extension could fake. A wav ignores the declared rate.
    wav = _wav_bytes([500, -500], rate=8000)
    assert decode_upload(wav, 16000).sample_rate == 8000
    # Raw bytes carry no header, so the declared rate is the only source of truth.
    raw = struct.pack("<hh", 500, -500)
    audio = decode_upload(raw, 16000)
    assert audio.sample_rate == 16000
    assert np.allclose(audio.samples, np.array([500, -500], dtype=np.float32) / _FULL_SCALE)


def test_decode_upload_rejects_empty_and_truncated_bodies() -> None:
    # Zero bytes is always a caller error.
    with pytest.raises(AudioInputError):
        decode_upload(b"", 16000)
    # An odd byte count means half a sample arrived: the transfer was cut short, and
    # analysing what did arrive would hide that from the caller.
    with pytest.raises(AudioInputError):
        decode_upload(b"\x01\x02\x03", 16000)


def test_raw_path_requires_a_positive_rate() -> None:
    # Guessing 16 kHz for an unstated rate would yield a confidently wrong transcription,
    # so an unusable rate is refused instead.
    with pytest.raises(AudioInputError):
        decode_upload(struct.pack("<hh", 1, 2), 0)


def test_min_duration_gate_rejects_clips_the_engine_cannot_decode() -> None:
    # Below roughly 100 ms the model frontend has fewer frames than its convolution needs
    # and the DECODE raises -- which the REST layer can only report as a 503, an
    # operational code telling the caller to retry bytes that can never succeed. Catching
    # it at the boundary is what turns that into an honest 400.
    short = _wav_bytes([100] * 800, rate=16000)  # 50 ms
    with pytest.raises(AudioInputError) as excinfo:
        decode_upload(short, 16000, min_duration_ms=100)
    # The message must carry both numbers: "too short" alone leaves the caller guessing
    # what would be long enough.
    assert "50" in str(excinfo.value) and "100" in str(excinfo.value)


def test_min_duration_gate_passes_a_real_one_syllable_command() -> None:
    # The floor exists for the convolution, NOT as a minimum-utterance policy: "停" is a
    # legitimate single-syllable estop and measures about 190 ms, so a gate that rejected
    # it would silently remove a safety command from the vocabulary.
    utterance = _wav_bytes([100] * 3040, rate=16000)  # 190 ms
    assert decode_upload(utterance, 16000, min_duration_ms=100).duration_s > 0.18


def test_min_duration_gate_is_off_by_default() -> None:
    # Zero disables the check, so a caller that has its own policy -- or a unit test
    # exercising the decode path alone -- is not forced through this one.
    assert decode_upload(_wav_bytes([100] * 160, rate=16000), 16000).duration_s > 0


def test_min_duration_gate_measures_time_not_bytes() -> None:
    # The same byte count is a long clip at 8 kHz and a short one at 48 kHz, so the gate
    # has to run after the rate is known. 1600 samples is 200 ms at 8 kHz (accepted) and
    # 33 ms at 48 kHz (rejected).
    samples = [100] * 1600
    assert decode_upload(_wav_bytes(samples, rate=8000), 16000, min_duration_ms=100)
    with pytest.raises(AudioInputError):
        decode_upload(_wav_bytes(samples, rate=48000), 16000, min_duration_ms=100)
