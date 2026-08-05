"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_opus_stream.py
Brief: Unit tests for the from-scratch streaming Opus encode/decode wrappers.

Description:
  Covers the development plan section 13 acceptance points that need no hardware for the
  streaming codec layer: the encoder must buffer arbitrary-length pushes and emit exactly
  one Opus packet per whole frame (never a short one), zero-pad only the final partial on
  flush(), and produce packets a matching decoder turns back into the right number of PCM
  samples; the decoder must skip a corrupt packet instead of raising, which is the
  tolerance the no-length 8519 uplink relies on. Opus is lossy, so assertions are on
  packet counts, sample counts and the no-raise contract, not on exact PCM equality. Run
  from the /opt/xbrain_v6 root:
      python3 -m pytest tests/payload/test_opus_stream.py
"""
from __future__ import annotations

import numpy as np

from services.payload.codec.opus_stream import OpusDecoderStream, OpusEncoderStream

_FS = 8000
_FRAME = 480
_FRAME_BYTES = _FRAME * 2


def _pcm(n_samples: int) -> bytes:
    # n_samples of mono s16le tone; a real waveform (not silence) so the encoder produces
    # ordinary voiced packets rather than any degenerate DTX output.
    t = np.arange(n_samples)
    return (2000.0 * np.sin(2 * np.pi * 300 * t / _FS)).astype("<i2").tobytes()


def test_encode_buffers_until_a_whole_frame() -> None:
    # A push shorter than one frame must emit nothing (buffered), and the push that
    # completes the frame must then emit exactly one packet -- proving no short frame is
    # ever encoded mid-stream.
    enc = OpusEncoderStream(_FS, _FRAME)
    assert enc.encode(_pcm(200)) == []
    out = enc.encode(_pcm(_FRAME - 200))
    assert len(out) == 1 and len(out[0]) > 0


def test_encode_emits_one_packet_per_whole_frame() -> None:
    # Two frames' worth in a single push must yield two packets, with nothing left that a
    # flush would add (the remainder is empty).
    enc = OpusEncoderStream(_FS, _FRAME)
    out = enc.encode(_pcm(2 * _FRAME))
    assert len(out) == 2
    assert enc.flush() == []


def test_encode_independent_of_chunk_boundaries() -> None:
    # The same total audio pushed in uneven chunks must produce the same whole-frame count
    # as one big push: 1000 samples = 2 whole 480-frames + a 40-sample remainder, so 2
    # packets stream out and the remainder waits for flush().
    enc = OpusEncoderStream(_FS, _FRAME)
    total = _pcm(1000)
    packets = []
    for chunk in (total[:333], total[333:1200], total[1200:]):
        packets += enc.encode(chunk)
    assert len(packets) == 2
    tail = enc.flush()
    assert len(tail) == 1


def test_flush_pads_final_partial_frame() -> None:
    # A trailing partial frame must be zero-padded and emitted by flush() as one packet,
    # and a second flush on the now-empty buffer must emit nothing.
    enc = OpusEncoderStream(_FS, _FRAME)
    assert enc.encode(_pcm(100)) == []
    assert len(enc.flush()) == 1
    assert enc.flush() == []


def test_roundtrip_sample_count() -> None:
    # Encoding N whole frames and decoding every packet must return N*frame_size samples,
    # closing the encode/decode loop on frame sizing (values differ -- Opus is lossy).
    enc = OpusEncoderStream(_FS, _FRAME)
    packets = enc.encode(_pcm(3 * _FRAME))
    assert len(packets) == 3
    dec = OpusDecoderStream(_FS)
    decoded = b"".join(dec.decode(p) for p in packets)
    assert len(decoded) // 2 == 3 * _FRAME


def test_decoder_skips_corrupt_packet() -> None:
    # A packet the decoder rejects (b"\xff\xff\xff\xff" reliably raises inside opuslib)
    # must come back as empty bytes, not propagate -- so a false marker split upstream
    # drops one packet and the stream resyncs on the next.
    dec = OpusDecoderStream(_FS)
    assert dec.decode(b"\xff\xff\xff\xff") == b""


def test_decoder_empty_packet_returns_empty() -> None:
    # An empty packet carries no audio and must decode to empty without touching opuslib.
    assert OpusDecoderStream(_FS).decode(b"") == b""
