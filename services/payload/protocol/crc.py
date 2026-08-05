"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: crc.py
Brief: CRC-8/MAXIM checksum for the 8529 lights protocol.

Description:
  The GZH-2 lights link (TCP 8529) protects every 8D frame with a one-byte
  CRC-8/MAXIM (Dallas / 1-Wire) checksum. The vendor specification only wrote the
  words "CRC8" with no polynomial or bit ordering, so the exact parameters below
  were reverse-derived and then verified against five known-good control frames
  (protocol doc section 7.3). This module re-implements the algorithm from those
  parameters alone; no vendor sample or probe-tool code is reused, per the
  test-system hard rule R0 (from-scratch mandate).

  Parameters (CRC-8/MAXIM):
    - polynomial  0x31  (reflected form 0x8C)
    - init        0x00
    - refin       true   (input bits reflected)
    - refout      true   (output bits reflected)
    - xorout      0x00

  Why the loop shifts RIGHT with 0x8C instead of LEFT with 0x31: when both input
  and output are reflected, the mathematically identical but cheaper way to compute
  the CRC is to process each byte least-significant-bit first and fold in the
  REFLECTED polynomial. That avoids having to bit-reverse every input byte on the
  way in and the accumulator on the way out, which a left-shifting 0x31 form would
  otherwise require. The result is bit-for-bit the same checksum the device expects.
"""
from __future__ import annotations

# Reflected form of polynomial 0x31. Working least-significant-bit first (refin and
# refout both true) turns the textbook "shift left, xor 0x31" reduction into this
# "shift right, xor 0x8C" reduction. Named as a constant so the single magic value
# that defines the checksum is documented in exactly one place.
_REFLECTED_POLY = 0x8C

# A CRC reduces its input one bit at a time; a byte is eight bits. Named so the
# inner loop reads as intent ("reduce all bits of this byte") rather than a bare 8.
_BITS_PER_BYTE = 8


def crc8_maxim(data: bytes) -> int:
    """Compute the CRC-8/MAXIM of a byte string.

    Args:
        data: the exact bytes to checksum. The caller decides the range (see below).

    Returns:
        The checksum as an int in the range 0..255, matching the trailing CRC byte
        the device appends to (or expects on) an 8529 frame.

    Why this function is coverage-agnostic: the 8529 protocol is NOT self-consistent
    about which bytes the checksum covers. A control frame checksums
    len + MSG_ID + payload, whereas the periodic 0x25 status report checksums the
    PAYLOAD ONLY (an asymmetry confirmed on real hardware, protocol doc section 9).
    Rather than bake either rule in here with a mode flag, this function simply
    checksums whatever bytes it is given, and each caller feeds the correct slice.
    That keeps the one subtle, easy-to-get-wrong rule (the coverage range) at the
    call sites where it is documented, instead of hidden inside the math.
    """
    # Init value is 0x00 (CRC-8/MAXIM). This accumulator carries the running CRC.
    crc = 0x00
    for byte in data:
        # Mix the next input byte into the accumulator before reduction. Because we
        # work LSB-first, this xor-then-reduce order is what makes refin=true hold.
        crc ^= byte
        for _ in range(_BITS_PER_BYTE):
            # LSB-first reduction step: if the low bit is set, the polynomial divides
            # in this position, so shift right and fold in the reflected polynomial;
            # otherwise the polynomial does not divide here, so just shift right.
            if crc & 0x01:
                crc = (crc >> 1) ^ _REFLECTED_POLY
            else:
                crc >>= 1
    # Mask to one byte. The accumulator never exceeds 8 bits given the operations
    # above, but the explicit mask documents the contract that the result is a byte.
    return crc & 0xFF
