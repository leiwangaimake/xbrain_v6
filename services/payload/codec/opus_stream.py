"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: opus_stream.py
Brief: Streaming Opus encode/decode over opuslib for the 8519 audio data plane.

Description:
  The audio plane moves Opus both ways over 8519: /play encodes 8 kHz PCM into [42]
  hail packets, and /mic decodes the 16 kHz Opus the device streams up (development
  plan section 5.2). opuslib is prebuilt infra we are allowed to use (test-system rule
  R0 exempts codecs); what is written from scratch here is the STREAMING layer around
  it -- the frame buffering, the flush padding, and the bad-packet tolerance -- because
  opuslib itself only encodes/decodes one fixed-size frame at a time.

  Why a streaming wrapper is needed, not a one-shot call:
    - Encode: opuslib.Encoder wants exactly one frame (frame_size samples) per call, but
      /play receives PCM in arbitrary-length WebSocket chunks. OpusEncoderStream buffers
      whatever does not fill a whole frame and carries it to the next call, emitting only
      complete frames -- so it never zero-pads mid-stream (which would inject clicks).
      The final ragged tail is padded to a frame only on an explicit flush() at end of
      stream.
    - Decode: the Opus decoder is stateful across frames, so one long-lived Decoder must
      persist for a session. OpusDecoderStream holds that decoder and, crucially, SKIPS a
      packet the decoder rejects instead of raising -- the 8519 uplink framer splits a
      no-length stream on a marker heuristic that can occasionally hand over a corrupt
      packet, and dropping one and resyncing on the next is the intended tolerance.
"""
from __future__ import annotations

from typing import List

import opuslib

# Encoder application profile. AUDIO (2049) tunes the encoder for general audio rather
# than the VOIP profile; it matches the profile the real-machine probe used, so the
# device decodes our hail exactly as it did in verification.
_APPLICATION = opuslib.APPLICATION_AUDIO

# Bytes per int16 sample: the PCM the encoder consumes and the decoder emits is mono
# s16le, so one audio sample is two bytes. Named so the frame-size math reads clearly.
_BYTES_PER_SAMPLE = 2

# A generous per-frame sample ceiling handed to Decoder.decode as its output buffer size.
# 5760 = 120 ms at 48 kHz, larger than any frame this system uses, so decode never
# truncates regardless of the packet's declared frame length.
_MAX_DECODE_SAMPLES = 5760

# Default encode frame: 480 samples = 60 ms at 8 kHz, the exact [42] hail frame the
# device expects. The /play encoder runs at 8 kHz, so this is its natural default.
_DEFAULT_FRAME_SIZE = 480


class OpusEncoderStream:
    """Buffers PCM and emits whole Opus frames, so callers can push arbitrary chunks.

    opuslib.Encoder encodes exactly frame_size samples per call. This wrapper holds a
    byte buffer, appends each push, and cuts off as many whole frames as have arrived,
    keeping the sub-frame remainder for next time. That lets /play feed it PCM in
    whatever sizes the socket delivers without ever encoding a short (click-inducing)
    frame; the last partial frame is padded only when flush() is called at end of stream.
    """

    def __init__(self, fs: int, frame_size: int = _DEFAULT_FRAME_SIZE) -> None:
        # One encoder instance for the whole stream: Opus carries inter-frame state, so a
        # single Encoder must persist rather than being recreated per chunk. The channel
        # count is fixed at 1 (mono) because the entire 8519 audio plane is single-channel;
        # fs is the input PCM rate (8000 for the /play hail path).
        self._enc = opuslib.Encoder(fs, 1, _APPLICATION)
        self._frame_size = frame_size
        # A full frame is frame_size samples of mono s16le; every emitted chunk is exactly
        # this many bytes, which is also what opuslib.encode requires per call.
        self._frame_bytes = frame_size * _BYTES_PER_SAMPLE
        # Rolling PCM buffer: bytes that have not yet filled a whole frame wait here for
        # the next push. A bytearray so consumed prefixes drop in place.
        self._buf = bytearray()

    def encode(self, pcm_s16le: bytes) -> List[bytes]:
        """Append PCM and return an Opus packet for every whole frame now buffered.

        Args:
            pcm_s16le: mono s16le PCM of any length (including zero or a partial frame).

        Returns:
            Zero or more Opus packets, one per complete frame; any leftover sub-frame PCM
            is retained for the next call.
        """
        self._buf.extend(pcm_s16le)
        packets: List[bytes] = []
        # Cut whole frames off the front while at least one full frame is buffered; stop
        # with the ragged remainder (if any) still buffered for the next push.
        while len(self._buf) >= self._frame_bytes:
            frame = bytes(self._buf[: self._frame_bytes])
            del self._buf[: self._frame_bytes]
            # opuslib.encode's second argument is the per-channel SAMPLE count, not a byte
            # count -- a subtle gotcha, since the first argument is a byte buffer. Passing
            # frame_size (samples) against a frame_bytes-long buffer is what it expects.
            packets.append(self._enc.encode(frame, self._frame_size))
        return packets

    def flush(self) -> List[bytes]:
        """Encode and return any final partial frame, zero-padded, then clear the buffer.

        Returns:
            A one-packet list if a sub-frame remainder was buffered, else an empty list.

        Called once at end of stream: the trailing PCM that never filled a frame is padded
        with silence up to a full frame so the device gets a valid final [42] packet. Only
        here is padding applied -- never mid-stream, where it would insert an audible gap.
        """
        if not self._buf:
            return []
        pad = self._frame_bytes - len(self._buf)
        frame = bytes(self._buf) + b"\x00" * pad
        self._buf.clear()
        # Same sample-count argument as in encode(): the padded buffer is now exactly one
        # frame, so it encodes into a single valid closing packet.
        return [self._enc.encode(frame, self._frame_size)]


class OpusDecoderStream:
    """Decodes uplink Opus packets to PCM, skipping any packet the decoder rejects.

    Holds one long-lived opuslib.Decoder because Opus decoding is stateful across a
    session. decode() returns the PCM for a good packet and an empty bytes for an empty
    or corrupt one -- the tolerance the no-length 8519 uplink needs, since its marker-
    split framer can occasionally emit a packet that is really two payloads glued by a
    false split. Dropping that one packet and continuing lets the stream resync on the
    next real frame instead of tearing down the session.
    """

    def __init__(self, fs: int) -> None:
        # One decoder for the session's lifetime, to preserve Opus inter-frame state. Mono
        # (1 channel) again, and fs is the uplink rate the device streams (16000 for /mic).
        self._dec = opuslib.Decoder(fs, 1)

    def decode(self, packet: bytes) -> bytes:
        """Decode one Opus packet to mono s16le PCM, or return b"" if it cannot be decoded.

        Args:
            packet: one Opus packet as handed out by the uplink framer.

        Returns:
            The decoded PCM bytes, or an empty bytes for an empty packet or one the
            decoder rejects (a bad packet is skipped, not raised, so the stream survives).
        """
        # An empty packet carries no audio; the framer already skips these, but guard here
        # too so a stray empty never reaches the decoder.
        if not packet:
            return b""
        try:
            return self._dec.decode(packet, _MAX_DECODE_SAMPLES)
        except opuslib.OpusError:
            # A corrupt packet (often the product of a false marker split upstream) is
            # dropped so decoding resynchronises on the next good frame.
            return b""
