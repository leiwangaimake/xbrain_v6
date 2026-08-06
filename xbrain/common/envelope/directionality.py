"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: directionality.py
Brief: The S3.0.1 tightening / loosening fail-safe around envelope decode

Description:
What this solves. 11 S3.0's rule "an unrecognised v must be rejected" and 11
S7.1's rule "a malformed estop is executed as stop" look contradictory. S3.0.1
resolves them by the DIRECTION of the command, and this module is that resolution
as code, sitting one layer above envelope.decode():

  * TIGHTENING (collapse-safe) -- a message whose EVERY malformed reading still
    lands on an action that makes the robot LESS able to move. A decode failure
    (unknown v, missing field, truncated JSON, bad encoding) does not reject: the
    caller applies the collapse-safe action. "宁可误停, 绝不误放行" -- a spurious
    stop costs nothing on the campus's authenticated link (U23); a spurious
    release could hurt.
  * everything else -- full validation, and a decode failure is a rejection with
    E_SCHEMA and no state change. This covers both S3.0.1's loosening row
    (enable, mode switch, any future release) and its "all other messages" row;
    the two share one fail action, so they share one Direction here.

*** The exemption criterion is collapse-safety, NOT a key name (99 U75,
2026-08-06). This is the trap the contract calls out by name. It is tempting to
write "if key == cmd/estop", and it is wrong twice over: it would MISS
rt/behavior/request with op == cancel, which is equally collapse-safe (S10.3.7
RQ-3: a malformed cancel becomes cancel_all), and it invites someone to later add
a key that has a loosening reading. So this module takes a Direction the caller
has already decided and never inspects a key. The decision "is THIS key, in THIS
op, collapse-safe" belongs to the consumer that owns the key, against the U75
test: does every malformed reading still tighten? Only cmd/estop and
rt/behavior/request(cancel) pass it today, and NEITHER may be widened to a key
that has any loosening interpretation.

What this module deliberately does NOT do:
  * It does not hold a key -> Direction table. See the paragraph above: the
    classification is op-sensitive (behavior/request is tightening only for
    cancel, loosening for start) and cannot be reduced to a key match, so
    building the table here would encode a wrong mapping. The caller passes the
    Direction it derived.
  * It does not decide the collapse-safe ACTION. Whether the tightening fallback
    means "stop" (cmd/estop) or "cancel_all" (behavior/request) is the consumer's
    -- this module only says "apply your collapse-safe action" versus "reject".
  * It does not exempt the v check for a LOOSENING message under any framing.
    S3.0.1 and E-3 are explicit: a malformed enable is rejected, never run.
  * It does not compute age or re-implement decode. It wraps decode() and adds
    exactly the direction-dependent handling of a decode failure.

Traps that look right and are not:
  1. Turning the tightening branch into a rejection "to be safe". That inverts
     the whole point: for a collapse-safe key, rejecting a malformed stop drops
     the stop, which is the DANGEROUS direction. INF-CM-2's fifth mutation is
     exactly this, and the test feeds a malformed estop and asserts a collapse-
     safe disposition rather than a raise.
  2. Catching Exception to be thorough. A tightening key swallows decode FAILURES
     (json / schema), not programming defects: an AttributeError from a caller
     bug must still reach the process fault path (CLAUDE.md 4.5), so the except
     list is exactly the decode-failure types and no wider.
"""

import enum
import json
from dataclasses import dataclass
from typing import Any, Optional, Union

# XbrainError is the base of EnvelopeSchemaError (decode's failure) and of
# ClosedSetViolation; catching it covers every deliberate contract failure decode
# can raise, while a genuine defect (TypeError, AttributeError) is NOT a subclass
# and so still propagates. E_SCHEMA is imported as a name to build the rejection
# for a raw-JSON failure (a truncated byte string never reached decode, so it
# needs its own E_SCHEMA wrapper). CLAUDE.md 3.5: the literal never appears.
from ..errors import E_SCHEMA
from ..errors.exceptions import XbrainError
from .envelope import Envelope, EnvelopeSchemaError, decode


# The two directions of S3.0.1's fail-safe. Only the tightening case is special;
# LOOSENING stands for both the loosening row and the "all other messages" row,
# because the contract gives them the same fail action (reject). Enum, not a
# bool, so a call site reads self-evidently -- guarded_decode(raw,
# Direction.TIGHTENING) -- and cannot be transposed with some other boolean.
class Direction(enum.Enum):
    """Whether a key's every malformed reading still tightens (S3.0.1 / U75)."""

    TIGHTENING = "tightening"
    LOOSENING = "loosening"


# The disposition of a guarded decode. Rejection is signalled by RAISING, not by
# a third enum value, because a rejection must carry the E_SCHEMA exception a
# consumer already handles -- returning a "rejected" sentinel would invite a
# caller to forget to check it and proceed as if accepted.
class Disposition(enum.Enum):
    """What the caller should do with a guarded-decode outcome."""

    ACCEPTED = "accepted"            # decode succeeded; envelope is present
    COLLAPSE_SAFE = "collapse_safe"  # decode failed on a tightening key; apply stop/cancel


@dataclass(frozen=True)
class GuardedResult:
    """A guarded-decode outcome. envelope is set iff disposition is ACCEPTED."""

    disposition: Disposition
    envelope: Optional[Envelope]


def is_collapse_safe(direction: Direction) -> bool:
    """True iff a decode failure on this direction should tighten, not reject.

    The one-line decision primitive S3.0.1 turns on. It is public and separately
    tested so the rule is pinned on its own, and it is the surface INF-CM-2's
    fifth mutation flips: making it return False for TIGHTENING turns a malformed
    stop into a rejection, which the test catches.
    """
    # Exactly TIGHTENING collapses safe. Written as an identity check against the
    # enum member rather than "direction != Direction.LOOSENING" so that adding a
    # future direction does not accidentally inherit the collapse-safe behaviour:
    # a new member would be non-collapse-safe until someone deliberately lists it.
    return direction is Direction.TIGHTENING


def guarded_decode(raw: Union[bytes, str, dict],
                   direction: Direction) -> GuardedResult:
    """Decode `raw` under S3.0.1's direction-dependent fail-safe.

    raw may be the bytes / str straight off the wire, or an already-parsed dict.
    Bytes and str are parsed HERE, inside the try, so that a truncated or wrongly
    encoded payload -- not just a schema violation -- takes the same fail-safe
    path: S3.0.1 lists "JSON 截断" and "编码错误" alongside "字段缺失" as things a
    tightening key must still execute as stop.

    On success: GuardedResult(ACCEPTED, envelope).
    On failure, tightening key: GuardedResult(COLLAPSE_SAFE, None) -- the caller
    applies its collapse-safe action (stop for cmd/estop, cancel_all for
    behavior/request).
    On failure, otherwise: raises EnvelopeSchemaError (E_SCHEMA); state unchanged.
    """
    try:
        # A dict has already been parsed by the caller; anything else is wire
        # bytes / text this function parses. json.loads accepts both bytes and
        # str (it autodetects the encoding for bytes), so a UnicodeDecodeError or
        # JSONDecodeError from a malformed byte string is raised right here and
        # caught below -- which is what routes an encoding error to the fail-safe.
        obj: Any = raw if isinstance(raw, dict) else json.loads(raw)
        env = decode(obj)
    except (json.JSONDecodeError, UnicodeDecodeError, XbrainError) as exc:
        # A decode FAILURE, of one of exactly three kinds: malformed JSON, bad
        # encoding, or a schema violation (EnvelopeSchemaError / any XbrainError
        # decode raised). A programming defect is deliberately NOT in this list
        # and still propagates -- CLAUDE.md 4.5.
        if is_collapse_safe(direction):
            # Trap 1 in the header, and INF-CM-2 mutation five: this branch must
            # NOT reject. The malformed stop is executed by returning the
            # collapse-safe disposition; the caller does the tightening action.
            return GuardedResult(Disposition.COLLAPSE_SAFE, None)
        # Loosening / general: reject with E_SCHEMA, state unchanged. An
        # EnvelopeSchemaError already carries E_SCHEMA, so it is re-raised
        # unchanged; a raw json / unicode failure is wrapped so the caller always
        # sees a contract code rather than a bare parser exception. `from exc`
        # keeps the original cause for a debugger without putting it in the code.
        if isinstance(exc, EnvelopeSchemaError):
            raise
        raise EnvelopeSchemaError(
            "envelope payload is not decodable JSON (11 S3.0.1, loosening path)"
        ) from exc
    return GuardedResult(Disposition.ACCEPTED, env)


__all__ = ["Direction", "Disposition", "GuardedResult",
           "is_collapse_safe", "guarded_decode"]
