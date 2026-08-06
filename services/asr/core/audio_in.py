"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: audio_in.py
Brief: Decode an uploaded audio body (RIFF wav or raw s16le PCM) into float32 mono samples.

Description:
  POST /v1/audio/transcriptions accepts a multipart "file" that is either a wav or raw
  PCM (development plan section 5.3). The recognizer, however, wants exactly one thing:
  a contiguous float32 mono waveform plus its sample rate. This module is that single
  conversion step, kept out of the REST handler so the byte-level rules are unit-testable
  without standing up FastAPI, and out of the recognizer so the engine wrapper never has
  to care where its samples came from.

  Why the stdlib "wave" module instead of a hand-rolled RIFF parser: the dependency
  whitelist (plan section 11) names "wave" explicitly, and a real wav is not just a fixed
  44-byte header -- the model's own test_wavs carry a LIST/INFO chunk BETWEEN "fmt " and
  "data", so any parser that assumes data starts at offset 44 reads metadata as audio.
  The stdlib walks the chunk list correctly, which is exactly the kind of solved,
  standard-format plumbing R0 is not about. What IS written from scratch here is the part
  that matters to this service: the format contract, the raw-PCM path, and the int16 to
  float32 conversion.

  Format contract (deliberately strict): mono, 16-bit little-endian PCM only. That is the
  one format the whole audio plane already speaks -- the device's [40] uplink, the /mic
  stream and the /play downlink are all mono s16le -- so anything else arriving here is a
  caller mistake, not a case to paper over. Silently down-mixing a stereo file or
  truncating 24-bit samples would let a genuinely wrong pipeline produce plausible text,
  which is far more expensive to debug than an immediate, explicit rejection.

  Sample rate is passed THROUGH rather than resampled: sherpa-onnx resamples internally
  to the model's 16 kHz when the accepted waveform declares a different rate, so doing it
  again here would be a second, lossier conversion for no benefit. The rate still has to
  travel with the samples, which is why decode() returns them as one Audio pair.
"""
from __future__ import annotations

import io
import wave
from dataclasses import dataclass

import numpy as np

# The one PCM wire format this service accepts, named once so the dtype below and the
# sample-width check cannot drift apart.
_PCM_DTYPE = np.dtype("<i2")
_BYTES_PER_SAMPLE = 2
_MONO = 1

# int16 full scale. Dividing by 32768 (not 32767) maps the int16 range into [-1, 1) the
# same way every audio toolchain does, so the waveform the model sees matches what it was
# trained on; the half-LSB asymmetry is inaudible and universally accepted.
_INT16_FULL_SCALE = 32768.0

# A RIFF file always starts with these four bytes. The sniff below uses them to decide
# "wav or raw PCM", which is what lets one endpoint accept both without a format field.
_RIFF_MAGIC = b"RIFF"


class AudioInputError(ValueError):
    """Raised when an uploaded audio body cannot be turned into mono s16le samples.

    House rule bans bare Exception. A dedicated type is what lets the REST layer answer
    a malformed upload with 400 (the caller must fix the request) while an inference
    fault answers 503 (the caller should retry) -- two different client reactions that a
    shared exception type would collapse into one. ValueError is the base because every
    case here is a bad input value.
    """


@dataclass(frozen=True)
class Audio:
    """A decoded mono waveform and the rate it was sampled at.

    Frozen because it is handed straight to the decode worker thread; an immutable
    payload cannot be mutated underneath the running inference. Note that freezing the
    dataclass does not freeze the numpy buffer it points at -- nothing in this service
    writes to that buffer, and the array is built fresh per request, so there is no
    aliasing to defend against.

    Keeping samples and sample_rate in ONE object is the point: every consumer needs
    both (duration, and the rate the engine must be told), and a bare array would let a
    caller pair a waveform with the wrong rate -- which fails silently as a pitch-shifted
    transcription rather than loudly as an error.
    """

    # Contiguous float32 waveform in [-1, 1), one channel.
    samples: np.ndarray
    # Sample rate of that waveform in Hz, as declared by the wav header or the caller.
    sample_rate: int

    @property
    def duration_s(self) -> float:
        """Wall-clock length of this waveform in seconds.

        Returns:
            Sample count divided by sample rate; 0.0 for an empty waveform.

        This is the denominator of the RTF the service reports (decode seconds divided
        by audio seconds), so it lives on the audio object itself -- the one place that
        holds both numbers it needs and therefore cannot compute it from a stale rate.
        """
        # Guard the empty case explicitly: a zero-length upload is rejected upstream, but
        # returning 0.0 here keeps this property total rather than raising on a caller
        # that (for instance) logs the duration before validating.
        if self.sample_rate <= 0:
            return 0.0
        return len(self.samples) / float(self.sample_rate)


def _pcm_to_float32(pcm_s16le: bytes) -> np.ndarray:
    """Convert raw mono s16le bytes into a contiguous float32 waveform in [-1, 1).

    Args:
        pcm_s16le: raw mono PCM, signed 16-bit little-endian.

    Returns:
        A float32 numpy array of len(pcm_s16le) // 2 samples.

    Raises:
        AudioInputError: if the buffer is not a whole number of 16-bit samples.

    The odd-length check is what catches a truncated upload: half a sample at the end
    means the transfer was cut short, and continuing would silently analyse audio that
    the caller never finished sending.
    """
    if len(pcm_s16le) % _BYTES_PER_SAMPLE != 0:
        raise AudioInputError(
            f"pcm length {len(pcm_s16le)} is not a whole number of 16-bit samples"
        )
    # frombuffer is a zero-copy view over the (immutable) bytes; astype then produces the
    # contiguous float32 array the engine wants, so the division below never writes into
    # the caller's buffer.
    return (np.frombuffer(pcm_s16le, dtype=_PCM_DTYPE).astype(np.float32)) / _INT16_FULL_SCALE


def decode_wav(data: bytes) -> Audio:
    """Decode a RIFF wav body into mono float32 samples.

    Args:
        data: the complete wav file bytes, header included.

    Returns:
        The decoded Audio, at the rate declared by the file's own header.

    Raises:
        AudioInputError: if the bytes are not a readable wav, or the wav is not mono
            16-bit PCM, or it carries no frames.

    Everything the stdlib can reject (a bad magic, a truncated header, a compressed
    codec) surfaces as wave.Error and is re-raised as AudioInputError so the REST layer
    has ONE exception type to map to 400. The channel and sample-width checks are ours,
    because "readable wav" and "the format this service accepts" are different questions
    and only the second one has an actionable message for the caller.
    """
    try:
        # BytesIO because the body arrives as bytes in memory, never as a path -- there
        # is no temp file to clean up and no filesystem round trip per request. The
        # context manager closes the reader even when a check below raises.
        with wave.open(io.BytesIO(data), "rb") as reader:
            channels = reader.getnchannels()
            sample_width = reader.getsampwidth()
            sample_rate = reader.getframerate()
            frames = reader.readframes(reader.getnframes())
    except wave.Error as exc:
        # Chain the original so the traceback keeps the stdlib's specific complaint while
        # the top-level message stays in this service's vocabulary.
        raise AudioInputError(f"not a readable wav file: {exc}") from exc

    # Format gate: mono 16-bit only, for the reasons in the module docstring. Both
    # numbers are reported back so the caller can see exactly what it sent.
    if channels != _MONO:
        raise AudioInputError(f"wav must be mono, got {channels} channels")
    if sample_width != _BYTES_PER_SAMPLE:
        raise AudioInputError(f"wav must be 16-bit PCM, got {sample_width * 8}-bit")
    # A header-only wav decodes to zero samples and would hand the engine an empty
    # waveform, whose empty transcription looks like a recognition failure rather than
    # the upload bug it actually is.
    if not frames:
        raise AudioInputError("wav contains no audio frames")

    return Audio(samples=_pcm_to_float32(frames), sample_rate=sample_rate)


def decode_upload(data: bytes, raw_sample_rate: int, min_duration_ms: int = 0) -> Audio:
    """Decode an uploaded body that is either a wav file or raw mono s16le PCM.

    Args:
        data: the raw multipart file body as received.
        raw_sample_rate: the rate to assume when data is NOT a wav, in Hz. A wav
            declares its own rate in its header and this argument is then ignored.
        min_duration_ms: shortest clip the engine can decode. Zero disables the check.

    Returns:
        The decoded Audio.

    Raises:
        AudioInputError: if the body is empty, if a wav fails the format contract, if
            raw_sample_rate is not positive on the raw path, or if the clip is shorter
            than min_duration_ms.

    Sniffing the RIFF magic (rather than trusting a filename or a form field) is what
    lets the OpenAI-shaped endpoint take both shapes on one parameter: a caller that
    sends a wav gets the header's rate, and a caller that streams raw VAD-cut PCM -- the
    normal 功能1 path, where AI_runtime already knows the rate -- states the rate itself.
    A four-byte sniff cannot be fooled by a wrong file extension, which a filename check
    very much can.

    * The minimum-duration check belongs HERE, at the boundary, and not deeper: a clip too
    short for the model's frontend convolution makes the engine raise, which the REST layer
    can only report as a 503 -- an operational code telling the caller to retry bytes that
    can never succeed, and which 11 §8.13.5 additionally reads as evidence that the
    concurrency limit was bypassed. Refusing it as malformed input turns it into a 400,
    which says the true thing: send different audio.
    """
    # An empty body is always a caller error, and catching it here means neither the wav
    # path nor the raw path has to invent behaviour for zero bytes.
    if not data:
        raise AudioInputError("empty audio upload")

    if data.startswith(_RIFF_MAGIC):
        audio = decode_wav(data)
    else:
        # Raw path: the bytes carry no rate of their own, so an absent or nonsensical
        # declared rate is unrecoverable -- refuse rather than guess 16 kHz, since guessing
        # wrong yields a confidently wrong transcription instead of an error.
        if raw_sample_rate <= 0:
            raise AudioInputError(
                f"raw pcm needs a positive sample_rate, got {raw_sample_rate}")
        audio = Audio(samples=_pcm_to_float32(data), sample_rate=raw_sample_rate)

    # Checked after decoding, because the duration is only knowable once the rate is: the
    # same byte count is a long clip at 8 kHz and a short one at 48 kHz.
    duration_ms = audio.duration_s * 1000.0
    if min_duration_ms > 0 and duration_ms < min_duration_ms:
        raise AudioInputError(
            f"audio is {duration_ms:.0f} ms, shorter than the {min_duration_ms} ms the "
            f"model can decode"
        )
    return audio
