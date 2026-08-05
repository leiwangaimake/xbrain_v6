"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: lights_8529.py
Brief: 8529 lights protocol -- frame reassembly and 0x25 status parsing.

Description:
  Encodes and decodes the GZH-2 lights link (TCP 8529). Every frame on this link has
  the layout 0x8D | len | MSG_ID | payload[len] | CRC8, where len counts payload bytes
  only (protocol doc section 7). This module carries the decode responsibilities that
  milestone M1 needs and the encode responsibilities that milestone M2 adds:

    1. Lights8529Framer converts the raw TCP byte stream into whole frames. TCP is a
       byte stream, not a message stream, so a single recv() may return part of a
       frame, several frames back to back, or a frame split across two reads; the
       framer hides all of that from the caller.
    2. parse_status_payload decodes the 0x25 status report -- which the device pushes
       unsolicited every 500 ms as soon as the socket connects -- into a typed,
       immutable LightsStatus.
    3. build_control_frame (and the typed builders on top of it) constructs the
       OUTBOUND control frames M2 sends: searchlight on/off, brightness, strobe, and
       red/blue mode. Each returns the exact bytes to hand device_link's send path.

  Everything here is written from the specification, never copied from the probe
  tools (test-system hard rule R0). The probes were read only to learn the protocol.

  CRC coverage asymmetry (verified on real hardware, protocol doc section 9) -- the
  single easiest thing to get wrong on this link, now that the module both reads and
  writes frames: the 0x25 status frame checksums the PAYLOAD ONLY, whereas control
  frames checksum len + MSG_ID + payload. The asymmetry is encapsulated on BOTH sides
  so no caller ever picks the range by hand: status_crc_ok() applies the payload-only
  rule when validating an inbound status, and build_control_frame() applies the
  len+MSG_ID+payload rule when constructing an outbound control frame.

  Worked framing example (a status frame is 28 bytes on the wire):
      byte  0        -> 0x8D            header
      byte  1        -> 0x18 (=24)      len (payload byte count)
      byte  2        -> 0x25            MSG_ID (status report)
      bytes 3..26    -> 24 payload      the status payload
      byte  27       -> 0xNN            CRC8 over the payload only
  If recv() returns, say, the first 5 of those bytes, feed() buffers them and
  returns []; when the remaining 23 arrive, the next feed() returns exactly one
  (msg_id, payload, crc) tuple. If recv() returns two whole frames at once, one
  feed() returns both, in order. The framer never assumes recv() boundaries align
  with frame boundaries, because on a TCP byte stream they never reliably do.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from .crc import crc8_maxim

# Message id and payload length of the periodic status report. len = 0x18 = 24
# bytes on the wire (protocol doc section 7.4). Both are named constants so the
# framer's length maths and device_link's validation refer to the same source.
MSG_ID_STATUS = 0x25
STATUS_PAYLOAD_LEN = 0x18

# Outbound control message ids (protocol doc section 7.2). Each names the one thing
# the device does with that frame, so the typed builders below read as intent rather
# than as raw hex. The 0x04 throw-hook command from the table is deliberately NOT
# modelled: this 三合一 unit has no hook, and house V1/V2 discipline bans building an
# interface for hardware we are not driving.
MSG_SEARCHLIGHT = 0x01  # searchlight on/off      payload [0|1]
MSG_BRIGHTNESS = 0x02   # searchlight brightness  payload [0..30]
MSG_STROBE = 0x03       # searchlight strobe on/off payload [0|1]  ★ UNUSED, see below
MSG_REDBLUE = 0x07      # red/blue warning mode   payload [0..16]  ★ this is 爆闪

# ★★ Which of these two is 爆闪, once and for all: MSG_REDBLUE. 00 PAY-02c settles the
# term for the whole requirement set, and MSG_STROBE -- the searchlight's own strobe -- is
# explicitly not used by this project. Both are implemented here because this module is the
# protocol's complete surface; only one of them is something a caller should reach for.
#
# The mistake this note exists to prevent has already been made once and is recorded in
# 00 VOI-43: D mode wants the searchlight STEADY at full brightness while the red/blue
# flashes, so wiring 爆闪 to MSG_STROBE flashes the wrong lamp and leaves the scene dark
# half the time, which is precisely what night perception needs it not to do (PER-42).
#
# ★ Note also what MSG_REDBLUE's payload is: a PATTERN SELECTOR (0 off, 1..16 firmware
# patterns), not a brightness and not a rate. The device offers no red/blue brightness and
# no flash-frequency parameter -- 14 GL-4d traces this to the protocol itself -- and 14
# GL-4a rules out synthesising a rate by toggling from the host, because the only path to
# this lamp is POST /lights and its 300 ms timeout dwarfs the 70-90 ms pulses involved.
# Verified against the device on 2026-08-03: commanding mode 0 turns the lamp off and the
# 0x25 report agrees, so mode 0 IS "off" and there is no separate on/off command (this
# settles 待确认 #7 in the GZH-2 protocol document).

# Inclusive value bounds the device accepts, from the same table. Brightness is
# 0..30 (0 means the lamp draws no current, i.e. dark) and the red/blue selector is
# 0..16 (0 = off, 1..16 = the sixteen built-in flash patterns). The builders reject
# anything outside these so an out-of-range command is caught here, in one place,
# rather than being silently sent for the firmware to clamp or ignore.
BRIGHT_MIN = 0
BRIGHT_MAX = 30
REDBLUE_MIN = 0
REDBLUE_MAX = 16

# First byte of every 8529 frame. The framer treats it as a resync anchor: it is the
# only byte value it trusts as a possible frame start when locking onto the stream.
_FRAME_HEADER = 0x8D

# Bytes that surround the payload on the wire: 0x8D header, the len byte, the MSG_ID
# byte, and the trailing CRC8. A complete frame is therefore payload_len +
# _OVERHEAD_BYTES long, which is the single arithmetic the framer relies on.
_OVERHEAD_BYTES = 4

# Bit masks inside the searchlight status byte. Bit layout is protocol doc section
# 7.4, with the precise byte OFFSET pinned on real hardware in section 9:
#   bit 7      -> searchlight on/off
#   bit 6      -> strobe (hardware flash) on/off
#   bits 0..5  -> brightness, 0..30
_SEARCHLIGHT_ON_BIT = 0x80
_SEARCHLIGHT_STROBE_BIT = 0x40
_SEARCHLIGHT_BRIGHT_MASK = 0x3F

# Byte offsets inside the 24-byte status payload, verified on real hardware. Named
# so parse_status_payload reads as intent and so a future offset correction happens
# in one place if the firmware layout is ever revised.
_OFF_SEARCHLIGHT = 3
_OFF_REDBLUE = 4


class LightsProtocolError(ValueError):
    """Raised when an 8529 frame cannot be decoded, or a control value cannot be encoded.

    Covers both directions: an inbound payload we cannot parse into a typed structure,
    and an outbound control value that falls outside the range the device accepts (for
    example a brightness above 30). A dedicated exception type (house rule bans bare
    Exception) lets a caller distinguish "the device sent something we cannot decode"
    or "we were asked to send an illegal command" from an ordinary programming
    ValueError. Subclassing ValueError keeps it semantically a bad-value error while
    remaining catchable as this specific protocol fault.
    """


@dataclass(frozen=True)
class LightsStatus:
    """Decoded 0x25 lights status; field meanings verified on real hardware.

    Frozen because it is a value snapshot handed from the reader thread to the REST
    handler: making it immutable removes any question of one side mutating what the
    other is reading.

    Fields:
        searchlight_on: True when the searchlight is illuminated (status bit 7).
        strobe: True when the searchlight hardware strobe is engaged (status bit 6).
        bright: brightness 0..30 from the low 6 bits. IMPORTANT: this value is
            RETAINED by the device even while the lamp is off (turning the lamp off
            only clears bit 7, not the brightness bits). Therefore "is the lamp off"
            must be judged from searchlight_on alone, never from bright == 0, or a
            dimmed-then-off lamp would be misread. See protocol doc section 9.
        redblue_mode: 0 when the red/blue warning light is off, or 1..16 to select
            one of the sixteen built-in flash patterns.
    """

    searchlight_on: bool
    strobe: bool
    bright: int
    redblue_mode: int


class Lights8529Framer:
    """Reassembles whole 8529 frames from the arbitrary chunks recv() returns.

    The problem this solves: TCP delivers a byte stream with no message boundaries,
    so the bytes handed back by a single recv() call bear no relation to frame edges.
    feed() buffers whatever arrives and yields every complete (msg_id, payload, crc)
    tuple it can now form, retaining any partial trailing frame for the next call.

    Resync behaviour: before measuring a frame, the framer discards any leading bytes
    that are not the 0x8D header. This does two useful things at once -- it lets the
    framer lock cleanly onto a stream that is already in flight when we connect, and
    it recovers the head of the buffer after a corrupt or misaligned frame.

    Why CRC validation is intentionally NOT done here: the checksum coverage differs
    by frame type (payload-only for 0x25 vs len+MSG_ID+payload for control replies),
    so validating inside the framer would force it to know every message type's rule.
    Instead the framer stays purely structural and hands the raw crc byte out, and
    the caller (which knows what message type it expects) validates with the right
    range. This keeps the one subtle rule at the site that documents it.
    """

    def __init__(self) -> None:
        # Rolling buffer of bytes received but not yet consumed into a whole frame.
        # A bytearray is used (not bytes) so we can delete consumed prefixes in place
        # without reallocating the whole remainder on every extraction.
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[Tuple[int, bytes, int]]:
        """Add received bytes and return every whole frame now available.

        Args:
            data: the bytes just returned by a socket recv() (may be any length,
                including a partial frame or several frames).

        Returns:
            A list of (msg_id, payload, crc) tuples, one per complete frame that
            could be formed. Empty when only a partial frame has arrived so far.
        """
        # Append first, then drain: a chunk may complete a frame started last call
        # and/or contain several whole frames of its own.
        self._buf.extend(data)
        frames: List[Tuple[int, bytes, int]] = []
        while True:
            frame = self._try_extract_one()
            # None means "no more whole frames right now" -- either the buffer is
            # empty, holds no header, or holds only a partial frame. Stop draining.
            if frame is None:
                break
            frames.append(frame)
        return frames

    def _try_extract_one(self) -> Optional[Tuple[int, bytes, int]]:
        """Pop a single whole frame from the head of the buffer, if one is present.

        Returns:
            The next (msg_id, payload, crc) tuple, or None if the buffer does not yet
            contain a complete frame (including the case where it holds no header).
        """
        # Resync step: locate the next frame header. Everything before it is noise
        # (leftover from a partial join or a corrupt frame) and is discarded so the
        # buffer head is always a candidate frame start.
        start = self._buf.find(_FRAME_HEADER)
        if start == -1:
            # No header byte anywhere in the buffer: nothing buffered can begin a
            # frame. A header is a single byte, so it cannot be "split" across the
            # boundary, which means it is safe to drop the whole buffer here.
            self._buf.clear()
            return None
        if start > 0:
            # Discard the leading noise up to the header in one slice deletion.
            del self._buf[:start]
        # Need at least the header and the len byte to know the total frame size.
        if len(self._buf) < 2:
            return None
        # len field (byte index 1) counts payload bytes only; the whole frame adds
        # the four overhead bytes (header, len, msg_id, crc).
        payload_len = self._buf[1]
        total = payload_len + _OVERHEAD_BYTES
        if len(self._buf) < total:
            # The full frame has not arrived yet; keep the partial and wait for more.
            return None
        # Slice out the fields by their fixed positions relative to the header.
        msg_id = self._buf[2]
        payload = bytes(self._buf[3:3 + payload_len])
        crc = self._buf[3 + payload_len]
        # Consume exactly this frame's bytes, leaving any following frame(s) in place.
        del self._buf[:total]
        return (msg_id, payload, crc)


def status_crc_ok(payload: bytes, crc: int) -> bool:
    """Validate a 0x25 status frame's checksum using the PAYLOAD-ONLY range.

    Args:
        payload: the 0x25 frame's payload bytes (no header/len/msg_id/crc).
        crc: the trailing CRC byte the framer extracted from the same frame.

    Returns:
        True when the payload-only CRC matches, False otherwise.

    This is the one place in the protocol where the checksum covers the payload
    alone; control frames instead cover len + MSG_ID + payload. Hiding that
    asymmetry behind this named helper stops a caller from reflexively applying the
    control-frame range to a status frame, which would reject every valid report.
    See protocol doc section 9.
    """
    return crc8_maxim(payload) == crc


def parse_status_payload(payload: bytes) -> LightsStatus:
    """Decode a validated 24-byte 0x25 payload into a LightsStatus.

    Args:
        payload: the status payload, expected to be STATUS_PAYLOAD_LEN bytes and to
            have already passed status_crc_ok().

    Returns:
        The decoded LightsStatus value.

    Raises:
        LightsProtocolError: if the payload is too short to contain the searchlight
            and red/blue bytes, so a truncated frame raises a typed error here rather
            than an IndexError deeper in the caller.

    Byte offsets were verified on real hardware (protocol doc section 9): payload[3]
    is the searchlight byte (bit7 on / bit6 strobe / bits0-5 brightness 0..30) and
    payload[4] is the red/blue mode (0 off, 1..16).
    """
    # Guard the two offsets we are about to read. We only require enough bytes for
    # the fields we actually decode, not the full 24, so a slightly short but usable
    # payload still parses; a genuinely truncated one raises a clear typed error.
    if len(payload) < _OFF_REDBLUE + 1:
        raise LightsProtocolError(f"status payload too short: {len(payload)} bytes")
    searchlight_byte = payload[_OFF_SEARCHLIGHT]
    return LightsStatus(
        # Bit 7: illuminated or not.
        searchlight_on=bool(searchlight_byte & _SEARCHLIGHT_ON_BIT),
        # Bit 6: hardware strobe engaged or not.
        strobe=bool(searchlight_byte & _SEARCHLIGHT_STROBE_BIT),
        # Low 6 bits: brightness, kept verbatim because it persists across off (see
        # the LightsStatus.bright note) -- we must not zero it just because bit7 is 0.
        bright=searchlight_byte & _SEARCHLIGHT_BRIGHT_MASK,
        # Whole byte: red/blue pattern selector, 0 off or 1..16.
        redblue_mode=payload[_OFF_REDBLUE],
    )


def build_control_frame(msg_id: int, payload: bytes) -> bytes:
    """Assemble one outbound 8529 control frame: 0x8D | len | MSG_ID | payload | CRC8.

    Args:
        msg_id: the control message id (MSG_SEARCHLIGHT / MSG_BRIGHTNESS / ...).
        payload: the already range-checked payload bytes for that command.

    Returns:
        The exact bytes to write to the 8529 socket, ready for device_link's send
        path -- the caller does no further wrapping or checksumming.

    This is the single encoder every typed builder funnels through, so the frame
    layout and the control-frame CRC range live in exactly one place. The checksum
    covers len + MSG_ID + payload -- everything after the 0x8D header -- which is the
    control-frame range and is DIFFERENT from the payload-only range status frames use
    (see status_crc_ok and the module-level CRC-asymmetry note). Computing it over
    `body`, which is precisely those bytes, makes the correct coverage structural
    rather than a rule each caller has to remember and could get wrong.
    """
    # len byte counts the payload only, msg_id follows it; body is exactly the CRC
    # range. bytes([...]) fixes the header/len/msg_id order in one expression.
    body = bytes([len(payload), msg_id]) + payload
    # Frame = header + body + CRC8(body). crc8_maxim is coverage-agnostic, so passing
    # it `body` (not the whole frame, not the payload alone) is what selects the
    # control-frame range here, mirroring how status_crc_ok passes the payload alone.
    return bytes([_FRAME_HEADER]) + body + bytes([crc8_maxim(body)])


def build_searchlight(on: bool) -> bytes:
    """Build the searchlight on/off control frame (MSG_SEARCHLIGHT).

    Args:
        on: True to illuminate the searchlight, False to turn it off.

    Returns:
        The encoded control frame bytes.

    The payload is a single 0/1 byte, so a bool is taken rather than an int: the call
    site reads as intent and no illegal value is representable, which is why this
    builder needs no range check where build_brightness/build_redblue do.
    """
    # bool -> 1/0 payload byte: int(True) is 1 (on) and int(False) is 0 (off), which
    # matches the on/off vectors in protocol doc section 7.5.
    return build_control_frame(MSG_SEARCHLIGHT, bytes([int(on)]))


def build_brightness(level: int) -> bytes:
    """Build the searchlight brightness control frame (MSG_BRIGHTNESS).

    Args:
        level: brightness BRIGHT_MIN..BRIGHT_MAX; 0 leaves the lamp drawing no
            current (dark) rather than turning it off (see LightsStatus.bright).

    Returns:
        The encoded control frame bytes.

    Raises:
        LightsProtocolError: if level is outside BRIGHT_MIN..BRIGHT_MAX. An out-of-
            range brightness is refused here rather than handed to the firmware to
            clamp, so an operator sees a clear cause instead of a silently altered
            command taking effect on the device.
    """
    # Reject out-of-range before encoding: firmware behaviour above BRIGHT_MAX is
    # unspecified, so refusing is safer than trusting an implicit clamp.
    if not BRIGHT_MIN <= level <= BRIGHT_MAX:
        raise LightsProtocolError(
            f"brightness {level} out of range {BRIGHT_MIN}..{BRIGHT_MAX}"
        )
    return build_control_frame(MSG_BRIGHTNESS, bytes([level]))


def build_strobe(on: bool) -> bytes:
    """Build the searchlight strobe on/off control frame (MSG_STROBE).

    Args:
        on: True to engage the hardware strobe (flash), False to stop it.

    Returns:
        The encoded control frame bytes.

    Like build_searchlight this takes a bool because the payload is a single 0/1 byte:
    a bool makes any other value unrepresentable, so no range check is needed.
    """
    # Same bool -> 1/0 mapping as the searchlight builder; matches the strobe on/off
    # vectors in protocol doc section 7.5.
    return build_control_frame(MSG_STROBE, bytes([int(on)]))


def build_redblue(mode: int) -> bytes:
    """Build the red/blue warning-mode control frame (MSG_REDBLUE).

    Args:
        mode: REDBLUE_MIN..REDBLUE_MAX -- 0 turns the red/blue warning light off, and
            1..16 select one of the sixteen built-in flash patterns.

    Returns:
        The encoded control frame bytes.

    Raises:
        LightsProtocolError: if mode is outside REDBLUE_MIN..REDBLUE_MAX, so an
            invalid pattern index fails loud here instead of reaching the device.
    """
    # Same up-front rejection rationale as build_brightness: the selector is a
    # documented 0..16, and firmware behaviour outside that is unspecified.
    if not REDBLUE_MIN <= mode <= REDBLUE_MAX:
        raise LightsProtocolError(
            f"redblue mode {mode} out of range {REDBLUE_MIN}..{REDBLUE_MAX}"
        )
    return build_control_frame(MSG_REDBLUE, bytes([mode]))
