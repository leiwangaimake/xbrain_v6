"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: fence.py
Brief: FenceSet crc32 per 11 S9A.2, the recipe every consumer recomputes itself

Description:
What this is for. A FenceSet travels from P3 to several consumers, and 11 S9A.2
FV-8 requires each of them to recompute the crc32 and reject a mismatch with
E_SCHEMA. Two further mechanisms are built on top of the same number: S-5 makes a
commit with the same fence_set_id + rev + crc32 idempotent, and op=ping
reconciles a running consumer against P3 by comparing exactly that triple.

The recipe, from 11 S9A.2 block B-3:

  per polygon, in array order:
    poly_id "|" role "|" winding "|" hard_enforce("1"/"0") "|"
    priority (decimal, no leading zeros, sign kept) "|"
    speed_limit_mps as "%.3f", empty string when role is not speed_limit "|"
    each vertex as "%.8f,%.8f;" of (lat, lon), in array order, never sorted
  prefix the whole with  fence_set_id "|" rev "|"
  CRC-32 of those UTF-8 bytes (IEEE 802.3, init 0xFFFFFFFF, reflected,
  final xor 0xFFFFFFFF)

*** Why priority and speed_limit_mps are in the recipe. Both change the outcome
of a judgement -- priority is the painter-algorithm stacking order of S9A.1, and
speed_limit_mps feeds the min() of S9.6.1 directly. The v0.2 recipe covered only
geometry and role, so lowering a slow zone's limit produced an IDENTICAL crc32.
S-5 then reports the commit as a duplicate and op=ping reconciles clean: the
change does not take effect and nothing anywhere reports an error. That is the
failure mode this project keeps finding, and it is why the two fields are worth
their own regression test rather than a comment.

Why name is NOT in the recipe. S9A.2 states it: renaming affects no machine
judgement, and folding it in would make a pure HMI rename trigger a full
stage/commit across the stack.

Why the field order is fixed and the vertices are never sorted. Concatenation is
not injective on its own -- without the separators, a poly_id ending in a digit
and a priority beginning with one would run together and two different fences
could produce identical bytes. And a sort would erase winding, which is exactly
what tells inside from outside.

A note on hard_enforce. It is spelled "1"/"0" and not "true"/"false" because the
section says so. That matters more than it looks: a C++ implementation streaming
a bool through operator<< also produces "1"/"0", while one that formats it as
text produces "true", and the two would disagree without either side being
obviously wrong.
"""

import binascii
from typing import Any, Dict, List

#: The one role for which speed_limit_mps enters the string. Every other role
#: contributes an empty field -- not a zero, and not an omitted separator. A
#: zero would be indistinguishable from a genuine speed_limit polygon whose limit
#: really is 0.000, and dropping the separator would shift every later field.
SPEED_LIMIT_ROLE = "speed_limit"

#: Field separator, from the recipe. Named so the two places that emit it cannot
#: drift apart.
SEP = "|"


class FenceRecipeError(ValueError):
    """A FenceSet that cannot be reduced to the canonical string.

    Distinct from a crc mismatch: this means the input is malformed, which is an
    E_SCHEMA situation, whereas a mismatch on a well-formed input points at
    transport. Conflating them would send the operator looking at the network for
    a data problem.
    """


def _format_priority(value: Any) -> str:
    """Decimal, no leading zeros, sign kept -- the recipe's own words.

    int() rather than a format string, because a float priority would otherwise
    render as "10.0" and disagree with a C++ int32 rendering of the same value.
    The field is declared int32 in S9A.2, so a non-integral value is malformed
    input rather than something to round.
    """
    # bool excluded explicitly, ahead of the numeric check. True is 1 in Python,
    # so a priority accidentally set to a boolean would render as "1" and stack
    # this polygon above one at priority 0 -- a real change in which constraint
    # wins, arrived at by a typo, and reported by nothing.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FenceRecipeError("priority must be an int32, got %r" % (value,))
    # A non-integral float is refused rather than rounded. Rounding would make
    # 10.5 and 10.4 the same priority while the source document plainly shows
    # them as different, and the operator would be left explaining why the
    # painter order does not match what they wrote.
    if isinstance(value, float) and value != int(value):
        raise FenceRecipeError("priority must be integral, got %r" % (value,))
    return str(int(value))


def _format_speed_limit(role: str, poly: Dict[str, Any]) -> str:
    """%.3f for a speed_limit polygon, the empty string for every other role.

    Reading the field even when the role is not speed_limit would be wrong in a
    way that only shows up later: S9A.2 makes it required ONLY for that role, so
    a stray value on a zone polygon is legal input, and letting it into the
    string would give two byte-identical fences different crc32 values.
    """
    # The role decides, never the presence of the field. Keying off "is the key
    # there" would let a stray value on a zone polygon change that fence's
    # checksum, and every consumer would then reject a valid FenceSet with
    # E_SCHEMA.
    if role != SPEED_LIMIT_ROLE:
        return ""
    # Required for this role, and absent is an error rather than a default. A
    # default here would be a safety parameter with a fallback, which CLAUDE.md
    # 3.1 forbids for exactly the reason it matters: whatever value were chosen
    # would silently become the speed limit of a real zone.
    if "speed_limit_mps" not in poly:
        raise FenceRecipeError(
            "role=speed_limit requires speed_limit_mps (11 S9A.2 field table)")
    value = poly["speed_limit_mps"]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise FenceRecipeError("speed_limit_mps must be a number, got %r" % (value,))
    # Three decimals, so 1 m/s and 1.0 m/s and 1.000 m/s are one string. Enough
    # to separate any two limits an operator would actually set, and short enough
    # that both languages print it identically without any rounding-mode
    # question arising.
    return "%.3f" % float(value)


def _format_vertices(vertices: Any) -> str:
    """Each vertex as "%.8f,%.8f;", in array order.

    Eight decimal places on a WGS84 degree is about 1.1 mm, well inside the
    positioning quality this system ever has, so the fixed width loses nothing
    real -- and unlike %.17g it renders identically in both languages without
    anyone reasoning about shortest round-trip representations.

    Accepts either {lat, lon} objects, which is the wire form S9A.2 illustrates,
    or two-element sequences, which is what a database row naturally produces.
    Both spell the same geometry, and forcing one caller to convert would put a
    second, unverified rendering of the same coordinates in the code base.
    """
    if not isinstance(vertices, (list, tuple)):
        raise FenceRecipeError("vertices must be an array, got %s"
                               % type(vertices).__name__)
    out: List[str] = []
    for i, vertex in enumerate(vertices):
        if isinstance(vertex, dict):
            # Both keys required. Defaulting a missing lon to zero would place
            # the vertex in the Gulf of Guinea and still produce a well-formed
            # polygon, so the fence would validate and enclose the wrong ground.
            if "lat" not in vertex or "lon" not in vertex:
                raise FenceRecipeError("vertex %d needs both lat and lon" % i)
            lat, lon = vertex["lat"], vertex["lon"]
        elif isinstance(vertex, (list, tuple)) and len(vertex) == 2:
            # Pair form, and the order is (lat, lon) to match the object form
            # above. Worth stating because GeoJSON uses the opposite order, and a
            # transposed pair is a valid coordinate somewhere else on Earth --
            # it fails no check, it just fences the wrong place.
            lat, lon = vertex[0], vertex[1]
        else:
            raise FenceRecipeError("vertex %d is neither {lat,lon} nor a pair: %r"
                                   % (i, vertex))
        # The index is in every message above. A fence can carry 512 vertices per
        # polygon and "a vertex is malformed" would leave someone counting.
        out.append("%.8f,%.8f;" % (float(lat), float(lon)))
    return "".join(out)


def canonical_fence_string(fence_set: Dict[str, Any]) -> str:
    """The exact string the crc32 is taken over.

    Exported, not private. When two implementations disagree on the crc32 the
    only useful first question is whether they built the same string, and a
    difference in the string is readable at a glance while a difference in the
    checksum tells you nothing about where it came from.
    """
    # Required at the top level. Every one of the three enters the string, so a
    # missing one is not a field to skip -- it would shift or shorten the bytes
    # and yield a checksum that is simply a different fence's.
    for field in ("fence_set_id", "rev", "polygons"):
        if field not in fence_set:
            raise FenceRecipeError("FenceSet is missing %s (11 S9A.2)" % field)

    polygons = fence_set["polygons"]
    if not isinstance(polygons, (list, tuple)):
        raise FenceRecipeError("polygons must be an array")

    parts: List[str] = []
    for i, poly in enumerate(polygons):
        if not isinstance(poly, dict):
            raise FenceRecipeError("polygon %d is not an object" % i)
        # The six fields the recipe names, checked before any of them is read.
        # Checking as we go would report only the first, and a document written
        # against the v0.2 field list is missing two -- priority and, on a
        # speed_limit polygon, its limit. Reporting them one build at a time is
        # two more round trips than it needs to be.
        #
        # name is absent from this list on purpose: S9A.2 keeps it out of the
        # recipe, so a FenceSet without one is perfectly valid here.
        for field in ("poly_id", "role", "winding", "hard_enforce",
                      "priority", "vertices"):
            if field not in poly:
                raise FenceRecipeError(
                    "polygon %d (%s) is missing %s (11 S9A.2)"
                    % (i, poly.get("poly_id", "?"), field))
        role = poly["role"]
        # hard_enforce as "1"/"0". Compared against True rather than coerced with
        # int(), so a string "false" -- which is truthy -- is rejected instead of
        # silently becoming "1" and hardening a fence that was meant to be soft.
        hard = poly["hard_enforce"]
        if not isinstance(hard, bool):
            raise FenceRecipeError("hard_enforce must be a bool, got %r" % (hard,))
        # Joined with SEP rather than concatenated, so the separator count is
        # structural: seven elements always produce six separators, including
        # when the speed field is empty. Hand concatenation is where a missing
        # separator creeps in, and a missing one does not fail -- it merges two
        # fields into a string that hashes cleanly and means something else.
        parts.append(SEP.join([
            str(poly["poly_id"]),
            str(role),
            str(poly["winding"]),
            "1" if hard else "0",
            _format_priority(poly["priority"]),
            _format_speed_limit(role, poly),
            _format_vertices(poly["vertices"]),
        ]))

    # rev is prefixed, not appended. Bumping rev must change the checksum, and it
    # is the one field an operator edits by hand; putting it where the string
    # starts makes it the first thing visible in a dumped canonical string.
    # The trailing empty element is what puts a separator after rev. Written as
    # part of the join rather than as a "|" glued on afterwards so the head has
    # the same shape as the polygon rows above it.
    head = SEP.join([str(fence_set["fence_set_id"]), str(fence_set["rev"]), ""])
    # Polygons are concatenated with nothing between them: each row already ends
    # in the ";" of its last vertex, and adding a separator here would change
    # every checksum this project has ever computed.
    return head + "".join(parts)


def fence_crc32(fence_set: Dict[str, Any]) -> str:
    """Eight lowercase hexadecimal characters, per the S9A.2 field table.

    binascii.crc32 is IEEE 802.3 with init 0xFFFFFFFF, reflected input and
    output, and a final xor of 0xFFFFFFFF -- the four properties the recipe
    names, and the same combination zlib, C++ boost and the usual table-driven
    implementations produce. It is worth naming them here because "CRC-32" alone
    identifies at least four mutually incompatible checksums.

    Lowercase is not cosmetic: FV-8 compares the recomputed value against a
    transmitted string, and a case difference would fail every frame while
    looking, in a log, exactly like a correct value.
    """
    data = canonical_fence_string(fence_set).encode("utf-8")
    # Masked with 0xFFFFFFFF before formatting. binascii.crc32 returns an
    # unsigned value on every Python 3, so the mask is a no-op today -- it is
    # kept because "%08x" on a negative int, which is what Python 2 and some
    # ports return, produces a nine-character string with a minus sign, and FV-8
    # would then reject every frame while the logged value looks almost right.
    return "%08x" % (binascii.crc32(data) & 0xFFFFFFFF)


def verify_fence_crc32(fence_set: Dict[str, Any]) -> bool:
    """FV-8: does the transmitted crc32 match a recomputed one.

    Returns a bool rather than raising, because the caller is the one that knows
    which error code applies -- E_SCHEMA on an inbound cmd/fence, but a stored
    FenceSet failing the same check is a storage-corruption situation
    (E_STORAGE_CORRUPT) and not a protocol violation.
    """
    # A FenceSet with no crc32 at all raises rather than returning False. The
    # two mean different things: False says "this fence is corrupt", which sends
    # someone looking at storage and transport, while a missing field says the
    # document was never complete and belongs to whoever produced it.
    if "crc32" not in fence_set:
        raise FenceRecipeError("FenceSet carries no crc32 to verify")
    # Lower-cased on the way in, not on the way out. We emit lower case per
    # S9A.2; rejecting a peer that transmits upper case would turn a purely
    # cosmetic difference into a rejected fence, and there is no reading of FV-8
    # under which that is the intent.
    return str(fence_set["crc32"]).lower() == fence_crc32(fence_set)
