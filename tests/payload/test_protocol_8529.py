"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_protocol_8529.py
Brief: Unit tests for the from-scratch 8529 CRC, framer and 0x25 status parser.

Description:
  Covers the development plan section 13 acceptance points that need no hardware:
  the CRC-8/MAXIM must reproduce the five verified control frames, the 0x25 status
  frame must validate under the payload-only CRC rule, and the status byte layout
  (searchlight bit7/bit6/bits0-5, red/blue byte) must decode correctly including
  the "brightness retained while off" quirk. Also exercises the stream framer over
  the ways TCP can fragment a frame. Run from the /opt/xbrain_v6 root:
      python3 -m pytest tests/payload/test_protocol_8529.py
"""
from __future__ import annotations

import pytest

from services.payload.protocol.crc import crc8_maxim
from services.payload.protocol.lights_8529 import (
    BRIGHT_MAX,
    BRIGHT_MIN,
    Lights8529Framer,
    LightsProtocolError,
    MSG_BRIGHTNESS,
    MSG_ID_STATUS,
    MSG_REDBLUE,
    MSG_SEARCHLIGHT,
    MSG_STROBE,
    REDBLUE_MAX,
    REDBLUE_MIN,
    STATUS_PAYLOAD_LEN,
    build_brightness,
    build_control_frame,
    build_redblue,
    build_searchlight,
    build_strobe,
    parse_status_payload,
    status_crc_ok,
)

# The five control frames the vendor spec lists with known-good checksums; the CRC
# here covers len+MSG_ID+payload (protocol doc section 7.3).
_CONTROL_VECTORS = [
    ((0x01, 0x01, 0x01), 0x31),  # light on
    ((0x01, 0x01, 0x00), 0x6F),  # light off
    ((0x01, 0x02, 0x07), 0xB9),  # brightness 7
    ((0x01, 0x03, 0x01), 0xA0),  # strobe on
    ((0x01, 0x07, 0x02), 0x79),  # red/blue mode 2
]


def _build_status_frame(searchlight_byte: int, redblue_byte: int) -> tuple[bytes, bytes, int]:
    # Assemble a wire-format 0x25 status frame around a 24-byte payload whose two
    # meaningful offsets are set; returns (whole_frame, payload, crc). The status
    # CRC covers the payload ONLY (protocol doc section 9).
    payload = bytearray(STATUS_PAYLOAD_LEN)
    payload[3] = searchlight_byte
    payload[4] = redblue_byte
    crc = crc8_maxim(bytes(payload))
    frame = bytes([0x8D, STATUS_PAYLOAD_LEN, MSG_ID_STATUS]) + bytes(payload) + bytes([crc])
    return frame, bytes(payload), crc


@pytest.mark.parametrize("body,expected", _CONTROL_VECTORS)
def test_crc_matches_spec_control_vectors(body: tuple[int, ...], expected: int) -> None:
    # The from-scratch CRC must agree with every vendor-verified control frame.
    assert crc8_maxim(bytes(body)) == expected


def test_status_crc_ok_uses_payload_only_range() -> None:
    # Positive check plus a guard that the wrong (control) range would NOT validate,
    # locking in the CRC-coverage asymmetry.
    _, payload, crc = _build_status_frame(0x9E, 0x02)
    assert status_crc_ok(payload, crc)
    control_range_crc = crc8_maxim(bytes([STATUS_PAYLOAD_LEN, MSG_ID_STATUS]) + payload)
    assert control_range_crc != crc


def test_parse_status_on_bright_max_and_redblue() -> None:
    # 0x9E searchlight byte = on, no strobe, brightness 30; red/blue mode 2.
    _, payload, _ = _build_status_frame(0x9E, 0x02)
    status = parse_status_payload(payload)
    assert status.searchlight_on is True
    assert status.strobe is False
    assert status.bright == 30
    assert status.redblue_mode == 2


def test_parse_status_off_retains_brightness() -> None:
    # Only bit7 clears when the lamp turns off; brightness stays in bits0-5, so
    # "off" must be judged from searchlight_on, never from bright == 0.
    _, payload, _ = _build_status_frame(0x1E, 0x00)
    status = parse_status_payload(payload)
    assert status.searchlight_on is False
    assert status.bright == 30
    assert status.redblue_mode == 0


def test_parse_status_rejects_short_payload() -> None:
    # A payload too short to hold the red/blue offset must raise, not index-error.
    with pytest.raises(LightsProtocolError):
        parse_status_payload(bytes([0x00, 0x00, 0x00]))


def test_framer_single_chunk() -> None:
    frame, payload, crc = _build_status_frame(0x9E, 0x02)
    frames = Lights8529Framer().feed(frame)
    assert frames == [(MSG_ID_STATUS, payload, crc)]


def test_framer_split_across_chunks() -> None:
    # TCP may split a frame anywhere; the framer must buffer the partial head and
    # only emit once the whole frame has arrived.
    frame, _, _ = _build_status_frame(0x9E, 0x02)
    framer = Lights8529Framer()
    assert framer.feed(frame[:5]) == []
    out = framer.feed(frame[5:])
    assert len(out) == 1 and out[0][0] == MSG_ID_STATUS


def test_framer_resync_and_back_to_back() -> None:
    # Leading noise before the 0x8D header must be discarded (resync), and two
    # frames delivered in one chunk must both be emitted.
    frame, _, _ = _build_status_frame(0x9E, 0x02)
    out = Lights8529Framer().feed(b"\x00\xff" + frame + frame)
    assert len(out) == 2
    assert all(f[0] == MSG_ID_STATUS for f in out)


# ---------------------------------------------------------------------------
# M2 outbound control-frame builders (protocol doc section 7.5).
# ---------------------------------------------------------------------------

# The full section 7.5 table as (label, msg_id, payload, whole-frame hex). Reproducing
# every vendor-listed frame byte-for-byte locks in that the encoder uses the CONTROL
# range for its CRC -- len + MSG_ID + payload -- which is the opposite of the
# payload-only 0x25 rule exercised above; a single mistaken range would flip every one
# of these. The three 0x04 throw-hook rows carry a 2-byte payload: the 三合一 has no
# hook so no typed builder exists for them, but driving them through the GENERIC
# encoder proves it handles a multi-byte payload and its len byte, not just the
# 1-byte commands.
_SECTION_7_5_VECTORS = [
    ("light_off",   MSG_SEARCHLIGHT, b"\x00",     "8D 01 01 00 6F"),
    ("light_on",    MSG_SEARCHLIGHT, b"\x01",     "8D 01 01 01 31"),
    ("bright_0",    MSG_BRIGHTNESS,  b"\x00",     "8D 01 02 00 3A"),
    ("bright_15",   MSG_BRIGHTNESS,  b"\x0F",     "8D 01 02 0F 7B"),
    ("bright_30",   MSG_BRIGHTNESS,  b"\x1E",     "8D 01 02 1E B8"),
    ("strobe_off",  MSG_STROBE,      b"\x00",     "8D 01 03 00 FE"),
    ("strobe_on",   MSG_STROBE,      b"\x01",     "8D 01 03 01 A0"),
    ("hook_all_on", 0x04,            b"\x00\x01", "8D 02 04 00 01 C7"),
    ("hook_1_on",   0x04,            b"\x01\x01", "8D 02 04 01 01 03"),
    ("hook_2_off",  0x04,            b"\x02\x00", "8D 02 04 02 00 08"),
    ("redblue_0",   MSG_REDBLUE,     b"\x00",     "8D 01 07 00 C5"),
    ("redblue_1",   MSG_REDBLUE,     b"\x01",     "8D 01 07 01 9B"),
    ("redblue_2",   MSG_REDBLUE,     b"\x02",     "8D 01 07 02 79"),
    ("redblue_16",  MSG_REDBLUE,     b"\x10",     "8D 01 07 10 58"),
]


@pytest.mark.parametrize(
    "label,msg_id,payload,frame_hex",
    _SECTION_7_5_VECTORS,
    ids=[v[0] for v in _SECTION_7_5_VECTORS],
)
def test_build_control_frame_matches_section_7_5(
    label: str, msg_id: int, payload: bytes, frame_hex: str
) -> None:
    # The generic encoder must emit the exact vendor frame for every row, including
    # the header, the len byte, and the control-range CRC.
    assert build_control_frame(msg_id, payload) == bytes.fromhex(frame_hex), label


def test_typed_builders_match_section_7_5() -> None:
    # Each typed builder must select the correct MSG_ID and emit the exact vendor
    # frame, so a call written as intent (build_searchlight(True)) yields the same
    # bytes as the raw table. Arguments cover both bool states and the range endpoints.
    assert build_searchlight(True) == bytes.fromhex("8D 01 01 01 31")
    assert build_searchlight(False) == bytes.fromhex("8D 01 01 00 6F")
    assert build_brightness(0) == bytes.fromhex("8D 01 02 00 3A")
    assert build_brightness(15) == bytes.fromhex("8D 01 02 0F 7B")
    assert build_brightness(30) == bytes.fromhex("8D 01 02 1E B8")
    assert build_strobe(True) == bytes.fromhex("8D 01 03 01 A0")
    assert build_strobe(False) == bytes.fromhex("8D 01 03 00 FE")
    assert build_redblue(0) == bytes.fromhex("8D 01 07 00 C5")
    assert build_redblue(16) == bytes.fromhex("8D 01 07 10 58")


def test_typed_builders_funnel_through_generic_encoder() -> None:
    # Locks the design invariant that every typed builder is just build_control_frame
    # with a fixed msg_id, so the frame layout and the CRC range live in exactly one
    # place and cannot drift between the typed and generic paths.
    assert build_searchlight(True) == build_control_frame(MSG_SEARCHLIGHT, b"\x01")
    assert build_brightness(15) == build_control_frame(MSG_BRIGHTNESS, b"\x0F")
    assert build_strobe(True) == build_control_frame(MSG_STROBE, b"\x01")
    assert build_redblue(2) == build_control_frame(MSG_REDBLUE, b"\x02")


@pytest.mark.parametrize("level", [BRIGHT_MIN - 1, BRIGHT_MAX + 1, -1, 255])
def test_build_brightness_rejects_out_of_range(level: int) -> None:
    # An out-of-range brightness must fail loud at the builder, not be sent for the
    # firmware to clamp; below the floor and above the ceiling both raise.
    with pytest.raises(LightsProtocolError):
        build_brightness(level)


@pytest.mark.parametrize("mode", [REDBLUE_MIN - 1, REDBLUE_MAX + 1, -1, 255])
def test_build_redblue_rejects_out_of_range(mode: int) -> None:
    # Same guard for the red/blue selector: an illegal pattern index raises here
    # rather than reaching the device.
    with pytest.raises(LightsProtocolError):
        build_redblue(mode)


def test_range_boundaries_are_accepted() -> None:
    # The inclusive endpoints must NOT raise -- guards against an off-by-one in the
    # bound check that would reject the legal maximum (brightness 30 / redblue 16).
    for level in (BRIGHT_MIN, BRIGHT_MAX):
        build_brightness(level)
    for mode in (REDBLUE_MIN, REDBLUE_MAX):
        build_redblue(mode)


def test_built_control_frame_roundtrips_through_framer() -> None:
    # A frame the builder emits must be exactly what the framer reads back: the same
    # msg_id and payload, and a CRC that validates over the CONTROL range
    # (len+MSG_ID+payload). This closes the encode/decode loop for the link -- the two
    # halves agree on layout and on the control-frame CRC coverage.
    frame = build_brightness(15)
    out = Lights8529Framer().feed(frame)
    assert len(out) == 1
    msg_id, payload, crc = out[0]
    assert msg_id == MSG_BRIGHTNESS
    assert payload == b"\x0F"
    assert crc8_maxim(bytes([len(payload), msg_id]) + payload) == crc
