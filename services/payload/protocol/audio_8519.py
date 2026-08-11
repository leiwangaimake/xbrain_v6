"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: audio_8519.py
Brief: 8519 audio protocol -- bracket-framed command encode and [40] uplink reframe.

Description:
  Encodes and decodes the GZH-2 audio link (TCP 8519). Unlike the 8529 lights link,
  this protocol has NO length field and NO checksum: a message on the wire is simply
  the command number written as literal ASCII inside square brackets, immediately
  followed by its raw payload -- for example b"[42]" then the Opus bytes, or b"[31]"
  then a gender byte and UTF-8 text (protocol doc section 4). The framing is therefore
  as thin as it gets; the whole subtlety of this module is on the INBOUND side, where
  the absence of a length field makes reassembling the record uplink genuinely tricky.

  Everything here is written from the specification and the real-machine behaviour the
  probe recorded; the probe was read only to learn the protocol (test-system rule R0),
  never imported and never copied.

  Milestone M3 needs exactly this set of messages, so only these are modelled (house
  V1/V2 discipline -- the many legacy commands in the protocol table that the test
  system does not drive are deliberately left out):
    - OUTBOUND (downlink), built by the typed helpers below:
        [42] realtime PC hail   -- one Opus packet per frame (8k/mono/480-sample)
        [11] stop realtime hail -- ends a [42] stream (the probe stops [42] with [11])
        [31] TTS V2             -- one gender byte (0 male / 1 female) then UTF-8 text
        [40] start recording    -- no payload; asks the device to open its mic
        [41] stop recording     -- no payload; asks the device to close its mic
    - INBOUND (uplink), reframed by AudioUplinkFramer:
        [40] <Opus>             -- one record frame: the marker b"[40]" then Opus
                                   (16k/mono); the device streams these while recording.

  Why the uplink is the hard part (the one thing to get right here): a downlink frame
  is self-contained, but the uplink is a continuous byte stream of b"[40]<opus>[40]
  <opus>..." with no length prefix and no delimiter other than the marker itself. The
  ONLY way to find frame boundaries is to split on the literal b"[40]" marker -- which
  is a heuristic, because an Opus payload can, by chance, contain those four bytes and
  cause a false split. That risk is inherent to a no-length protocol and cannot be
  fully removed here; it is made small in practice by Opus packets being short (tens of
  bytes at these bitrates) and is contained by the decoder tolerating an occasional bad
  packet (it skips one and resynchronises on the next marker). AudioUplinkFramer
  therefore stays purely structural -- it splits on the marker and hands out raw Opus
  packets -- and leaves decoding (and thus bad-packet tolerance) to the codec layer,
  exactly as the 8529 framer stays structural and leaves CRC validation to its caller.
"""
from __future__ import annotations

from typing import List

# Outbound command numbers (protocol doc section 4). Named so the typed builders read
# as intent rather than as bare magic numbers, and so the one place a number could be
# mistyped is this table. Only the M3-driven subset is defined; the legacy TTS/playback
# commands the test system never sends are intentionally absent.
CMD_HAIL_PC = 42        # realtime hail from a PC client; payload = one Opus packet
CMD_HAIL_STOP = 11      # stop the realtime hail stream (verified to end a [42] stream)
CMD_TTS_V2 = 31         # device-side TTS with gender; payload = voice byte + UTF-8 text
CMD_RECORD_START = 40   # ask the device to start streaming mic audio (no payload)
CMD_RECORD_STOP = 41    # ask the device to stop streaming mic audio (no payload)
CMD_VOLUME = 14         # set persistent playback volume; payload = one byte 0..100

# Volume payload range (protocol doc section 4, [14]${vol}): one raw byte 0..100.
# This is the PERSISTENT playback volume (11 S7.5 volume_pct), retained across
# hails -- distinct from a one-shot speak gain.
VOLUME_MIN = 0
VOLUME_MAX = 100

# TTS voice selector byte values (protocol doc section 4, [31] payload first byte).
VOICE_MALE = 0
VOICE_FEMALE = 1

# The literal marker that both STARTS recording (as a downlink command) and PREFIXES
# every uplink record frame. Splitting the inbound stream on this exact byte sequence
# is the only framing signal the protocol provides, so it is named once here and used
# by AudioUplinkFramer. It is four bytes long, which is why a partial marker can be
# split across two recv() chunks and must be handled (see the framer).
_RECORD_MARKER = b"[%d]" % CMD_RECORD_START
_MARKER_LEN = len(_RECORD_MARKER)


class AudioProtocolError(ValueError):
    """Raised when an 8519 audio command cannot be encoded from the given arguments.

    House rule bans bare Exception; a dedicated type lets a caller (for example the
    POST /tts handler) tell an illegal-argument fault -- a voice selector that is
    neither 0 nor 1, or empty TTS text -- apart from an unrelated error. It subclasses
    ValueError because every case it covers is a bad input value handed to a builder.
    """


def encode_frame(cmd: int, payload: bytes = b"") -> bytes:
    """Assemble one outbound 8519 frame: the ASCII "[cmd]" marker then the payload.

    Args:
        cmd: the command number (CMD_HAIL_PC / CMD_TTS_V2 / ...).
        payload: the raw payload bytes for that command (empty for commands like
            record start/stop that take none).

    Returns:
        The exact bytes to write to the 8519 socket. There is no length prefix, no
        terminator, and no checksum -- the wire format really is just the bracketed
        number followed by the payload (protocol doc section 4).

    This is the single encoder every typed builder funnels through, so the "[%d]"
    bracket convention lives in exactly one place. It performs no validation: it is an
    internal primitive fed known command constants, and the typed builders above it are
    where caller-supplied values (the voice byte, the TTS text) are range-checked --
    mirroring how lights_8529.build_control_frame stays validation-free and its typed
    builders guard their inputs.
    """
    # f-string keeps the bracket convention in one obvious expression; encode as ASCII
    # because the command number is always plain digits and the device parses the
    # literal characters '[', the digits, and ']'.
    return f"[{cmd}]".encode("ascii") + payload


def build_hail(opus_packet: bytes) -> bytes:
    """Build one [42] realtime-hail frame carrying a single Opus packet.

    Args:
        opus_packet: one Opus-encoded frame of 8k/mono audio (480 samples / 60 ms),
            as produced by the codec layer.

    Returns:
        The encoded [42] frame bytes.

    A hail stream is many of these sent back to back, one per 60 ms audio frame; this
    builder handles a single frame so the caller controls pacing. No validation: the
    payload is opaque Opus bytes the codec already produced, and this layer does not
    re-inspect them.
    """
    return encode_frame(CMD_HAIL_PC, opus_packet)


def build_hail_stop() -> bytes:
    """Build the [11] stop-hail frame that ends a [42] realtime-hail stream.

    Returns:
        The encoded [11] frame bytes (marker only, no payload).

    The protocol lists [11] as "stop mobile hail", but real-machine testing showed it
    also cleanly ends a [42] PC-hail stream, so the audio pipeline sends it when a
    /play session closes rather than merely ceasing to send [42] frames -- an explicit
    stop leaves the device in a known idle state instead of waiting on a silent stream.
    """
    return encode_frame(CMD_HAIL_STOP)


def build_tts(voice: int, text: str) -> bytes:
    """Build a [31] TTS-V2 frame: a gender byte followed by UTF-8 text.

    Args:
        voice: VOICE_MALE (0) or VOICE_FEMALE (1); the device's first payload byte.
        text: the sentence to speak, encoded to the wire as UTF-8.

    Returns:
        The encoded [31] frame bytes.

    Raises:
        AudioProtocolError: if voice is neither 0 nor 1, or text is empty. Both are
            refused here rather than sent, because a bad voice byte has undefined
            device behaviour and an empty utterance is almost certainly a caller
            mistake that should fail loud instead of making the device speak nothing.
    """
    # Reject an out-of-range gender selector: only 0 and 1 are defined, and anything
    # else is an undefined payload the firmware might mishandle.
    if voice not in (VOICE_MALE, VOICE_FEMALE):
        raise AudioProtocolError(f"voice must be {VOICE_MALE} or {VOICE_FEMALE}, got {voice}")
    # An empty utterance is treated as a caller error, not a silent no-op frame.
    if not text:
        raise AudioProtocolError("tts text must not be empty")
    # Payload layout: one voice byte, then the UTF-8 text. UTF-8 is the device's
    # documented text encoding, so Chinese prompts survive the round trip unaltered.
    payload = bytes([voice]) + text.encode("utf-8")
    return encode_frame(CMD_TTS_V2, payload)


def build_volume(volume: int) -> bytes:
    """Build a [14] set-volume frame: one raw byte 0..100 (protocol doc
    section 4, [14]${vol}). This sets the PERSISTENT playback volume
    (11 S7.5 volume_pct), retained after a hail ends.

    Raises AudioProtocolError if volume is outside 0..100 -- an out-of-range
    byte is refused here rather than sent, mirroring build_tts's bad-voice
    guard, because the device's behaviour on an illegal value is undefined.
    """
    if not VOLUME_MIN <= volume <= VOLUME_MAX:
        raise AudioProtocolError(
            f"volume must be {VOLUME_MIN}..{VOLUME_MAX}, got {volume}")
    return encode_frame(CMD_VOLUME, bytes([volume]))


def build_record_start() -> bytes:
    """Build the [40] start-recording command (marker only, no payload).

    Returns:
        The encoded [40] frame bytes.

    Note the bytes are identical to the uplink record MARKER: the same b"[40]" both
    asks the device to start recording (downlink) and prefixes every record frame the
    device sends back (uplink). Direction, not content, distinguishes the two uses.
    """
    return encode_frame(CMD_RECORD_START)


def build_record_stop() -> bytes:
    """Build the [41] stop-recording command (marker only, no payload).

    Returns:
        The encoded [41] frame bytes.
    """
    return encode_frame(CMD_RECORD_STOP)


class AudioUplinkFramer:
    """Reassembles inbound [40] record frames from the bytes recv() returns.

    The device streams b"[40]<opus>[40]<opus>..." while recording, with no length
    field and no delimiter other than the b"[40]" marker itself. feed() buffers
    whatever arrives and yields every Opus packet it can now bound, retaining any
    trailing partial for the next call.

    Boundary rule (the crux): a packet's END is only known once the NEXT marker
    arrives, because nothing else marks where the Opus bytes stop. So the framer can
    emit packet N only after it has seen the start of packet N+1's marker. The final
    packet of a stream therefore stays buffered until either another frame follows or
    the session is reset -- a deliberate one-frame (~20 ms) latency that is the price of
    a protocol with no length prefix.

    Resync mirrors the 8529 framer: any bytes before the first marker are discarded, so
    the framer locks cleanly onto a stream joined mid-flight and recovers after a false
    split. Because the marker is four bytes it can straddle two recv() chunks; when no
    whole marker is present yet, the framer keeps only a short tail that could be a
    marker prefix and drops the rest, so a split marker is never lost but unbounded
    noise never accumulates.

    Structural only, by design: like the 8529 framer it never decodes. It hands out raw
    Opus packets and lets the codec layer decode and tolerate the occasional bad packet
    a false split may produce, keeping the "find frames" and "understand frames" jobs in
    separate layers.
    """

    def __init__(self) -> None:
        # Rolling buffer of received-but-not-yet-framed bytes. A bytearray so consumed
        # prefixes are deleted in place rather than reallocating the remainder each time.
        self._buf = bytearray()

    def reset(self) -> None:
        """Discard any buffered bytes so a new recording session starts clean.

        Called when recording (re)starts: a partial trailing packet left over from a
        previous session must not be prepended to the new stream's first frame, so the
        buffer is emptied rather than trusting resync to sort out the stale bytes.
        """
        self._buf.clear()

    def feed(self, data: bytes) -> List[bytes]:
        """Add received bytes and return every whole Opus packet now available.

        Args:
            data: the bytes just returned by a socket recv() (any length, including a
                partial frame, several frames, or a marker split across the boundary).

        Returns:
            A list of raw Opus packets, one per complete record frame that could be
            bounded by a following marker. Empty when only a partial frame is buffered.
        """
        self._buf.extend(data)
        packets: List[bytes] = []
        # Resync: find the first marker. Everything before it is noise (pre-stream
        # bytes, or the tail of a false-split packet) and is dropped.
        first = self._buf.find(_RECORD_MARKER)
        if first == -1:
            # No whole marker yet. Keep only a short tail that could be the start of a
            # marker split across this and the next chunk; drop everything else so a
            # stream of non-marker noise cannot grow the buffer without bound.
            if len(self._buf) > _MARKER_LEN - 1:
                del self._buf[: len(self._buf) - (_MARKER_LEN - 1)]
            return packets
        if first > 0:
            # Drop leading noise so the buffer now begins exactly at a marker.
            del self._buf[:first]
        # The buffer starts with a marker. Emit each packet that a FOLLOWING marker
        # bounds; stop at the last marker, whose packet is still open (more may come).
        while True:
            nxt = self._buf.find(_RECORD_MARKER, _MARKER_LEN)
            if nxt == -1:
                # Only the trailing open packet remains; wait for the next marker.
                break
            packet = bytes(self._buf[_MARKER_LEN:nxt])
            # Skip a zero-length packet (two adjacent markers): it carries no audio and
            # decoding it would only produce a needless error downstream.
            if packet:
                packets.append(packet)
            # Consume this marker and its packet; the buffer again begins at a marker.
            del self._buf[:nxt]
        return packets
