"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_audio_8519.py
Brief: Unit tests for the from-scratch 8519 bracket-framed audio protocol.

Description:
  Covers the development plan section 13 acceptance points that need no hardware for
  the 8519 audio link: the outbound builders must emit the exact bracketed wire bytes
  (no length field, no checksum), build_tts must lay out its voice byte + UTF-8 text
  and reject illegal arguments, and the inbound AudioUplinkFramer must reassemble the
  no-length [40] record stream across every way TCP can fragment it -- including a
  marker split across a chunk boundary and the one-frame emit latency that a following
  marker is what bounds a packet. Run from the /opt/xbrain_v6 root:
      python3 -m pytest tests/payload/test_audio_8519.py
"""
from __future__ import annotations

import pytest

from services.payload.protocol.audio_8519 import (
    AudioProtocolError,
    AudioUplinkFramer,
    build_hail,
    build_hail_stop,
    build_record_start,
    build_record_stop,
    build_tts,
    encode_frame,
)

# A stand-in "Opus" payload. The framer is structural and never decodes, so any bytes
# that do NOT themselves contain the b"[40]" marker serve as an opaque packet.
_A = b"\x11\x22\x33\x44"
_B = b"\xaa\xbb\xcc"
_MARKER = b"[40]"


def test_encode_frame_is_bracket_plus_payload() -> None:
    # The wire format is exactly the ASCII "[cmd]" marker then the raw payload, with no
    # length prefix, terminator, or checksum -- this pins that whole contract.
    assert encode_frame(42, b"\x01\x02") == b"[42]\x01\x02"
    # A payload-less command is just the bracketed number.
    assert encode_frame(11) == b"[11]"


def test_builders_match_known_frames() -> None:
    # Each typed builder must select the right command number and emit the exact bytes,
    # so a call written as intent yields the same wire frame as a hand-written literal.
    assert build_hail(_A) == b"[42]" + _A
    assert build_hail_stop() == b"[11]"
    assert build_record_start() == b"[40]"
    assert build_record_stop() == b"[41]"


def test_build_tts_encodes_voice_then_utf8() -> None:
    # [31] payload layout is one gender byte then UTF-8 text; a Chinese prompt must
    # survive as its UTF-8 bytes so the device speaks it unaltered.
    assert build_tts(0, "hi") == b"[31]" + bytes([0]) + b"hi"
    assert build_tts(1, "你好") == b"[31]" + bytes([1]) + "你好".encode("utf-8")


@pytest.mark.parametrize("voice", [-1, 2, 255])
def test_build_tts_rejects_bad_voice(voice: int) -> None:
    # Only 0 (male) and 1 (female) are defined; any other selector is an undefined
    # payload the firmware might mishandle, so the builder refuses it rather than send.
    with pytest.raises(AudioProtocolError):
        build_tts(voice, "hi")


def test_build_tts_rejects_empty_text() -> None:
    # An empty utterance is almost certainly a caller mistake and must fail loud instead
    # of making the device speak nothing.
    with pytest.raises(AudioProtocolError):
        build_tts(0, "")


def test_record_start_bytes_equal_uplink_marker() -> None:
    # The same b"[40]" both starts recording (downlink) and prefixes every record frame
    # the device streams back (uplink); direction, not content, distinguishes them.
    assert build_record_start() == _MARKER


def test_framer_needs_following_marker_to_emit() -> None:
    # A packet's END is only known once the NEXT marker arrives, so a lone opening frame
    # yields nothing yet -- it stays buffered (the deliberate one-frame latency).
    assert AudioUplinkFramer().feed(_MARKER + _A) == []


def test_framer_emits_when_next_marker_arrives() -> None:
    # Once a following marker bounds the first packet, that packet is emitted; the second
    # (still-open) packet stays buffered behind the trailing marker.
    assert AudioUplinkFramer().feed(_MARKER + _A + _MARKER) == [_A]


def test_framer_back_to_back() -> None:
    # Several complete frames delivered in one chunk must all emit in order, with the
    # final open frame withheld until its own following marker.
    out = AudioUplinkFramer().feed(_MARKER + _A + _MARKER + _B + _MARKER)
    assert out == [_A, _B]


def test_framer_split_across_chunks() -> None:
    # TCP may split a frame anywhere; the framer buffers the partial and only emits once
    # the whole frame plus a bounding marker have arrived.
    framer = AudioUplinkFramer()
    assert framer.feed(_MARKER + _A[:2]) == []
    assert framer.feed(_A[2:] + _MARKER) == [_A]


def test_framer_resync_drops_leading_noise() -> None:
    # Bytes before the first marker (pre-stream junk or the tail of a false split) are
    # discarded, so the framer locks cleanly onto a stream joined mid-flight.
    out = AudioUplinkFramer().feed(b"\x00\xff" + _MARKER + _A + _MARKER)
    assert out == [_A]


def test_framer_reassembles_marker_split_byte_by_byte() -> None:
    # The four-byte marker can straddle recv() boundaries; fed one byte at a time it must
    # still be reassembled and never lost, so the first frame is recovered intact.
    framer = AudioUplinkFramer()
    for byte in _MARKER:
        assert framer.feed(bytes([byte])) == []
    assert framer.feed(_A + _MARKER) == [_A]


def test_framer_skips_zero_length_packet() -> None:
    # Two adjacent markers bound an empty packet that carries no audio; it is skipped
    # rather than handed downstream as a decode error.
    out = AudioUplinkFramer().feed(_MARKER + _MARKER + _A + _MARKER)
    assert out == [_A]


def test_framer_reset_discards_buffered_partial() -> None:
    # reset() (called on a recording restart) must drop a buffered trailing packet so it
    # is never prepended to the new session's first frame.
    framer = AudioUplinkFramer()
    assert framer.feed(_MARKER + _A) == []  # _A left open in the buffer
    framer.reset()
    assert framer.feed(_MARKER + _B + _MARKER) == [_B]  # _A is gone, not [_A, _B]
