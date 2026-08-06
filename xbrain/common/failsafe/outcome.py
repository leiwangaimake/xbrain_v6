"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: outcome.py
Brief: The one result type every fail-safe branch in this package returns

Description:
Why this file exists at all. INF-DB-3 lands three blocks of functionality that
are permanently unavailable today (V-33 side/rear LiDAR coverage, 11 T-PTZ-1 PTZ
homing, 18 T-PTZ-3 PTZ speed calibration). Each block answers a command the same
shape of way -- it REJECTS, fail-loud, with a closed-set error code and a
structured detail -- and every one of them was about to grow its own ad-hoc
"return a dict that looks like a reject" at the call site. This module gives them
one frozen result type so the three branches cannot drift into three different
spellings of "rejected", which is the exact failure 11 S13.8 forbids for the
error code and the same reasoning covers the ack status beside it.

Where the design comes from. The two ack statuses are the wire values 11 uses for
a command acknowledgement (grep 11 for "accepted" / "rejected"; e.g. 11 S7.4.8
"ack = accepted 只表示命令被接受"). They are defined here rather than imported
from a bigger ack-status closed set because no such set exists in the shared
library yet, and only these two are load-bearing for a fail-safe: a fail-safe
NEVER emits accepted, so having the constant is what lets the "never accepted"
assertion be written and mutation-tested (21 S1 T-PTZ-1 row, 逐字 "不得用
accepted 冒充'已到位'").

What this file does NOT do. It does not decide WHICH code a branch returns, does
not build the detail payload, and carries no PTZ or rotation knowledge -- those
live in rotation.py and ptz.py. It also does not map a reject onto TTS wording:
11 S8.13.5 makes error mapping and phrasing the gateway's job, so the Chinese the
operator finally hears is composed downstream from the reason token here, never
stored in this layer.

The looks-right-but-wrong trap this shape guards against. A tempting
"simplification" is a plain bool (allowed / not allowed) or a bare code string.
Both discard the status, and the moment the status is gone the accepted-vs-
rejected distinction the T-PTZ-1 fail-safe turns on can no longer be expressed --
an implementation that quietly answers "accepted (command received)" would then
be indistinguishable from a real rejection, which is precisely the假保证 (fake
guarantee) 21 S1 was written to stop. Keep status a first-class field.
"""

from dataclasses import dataclass
from typing import Mapping, Optional

# The two acknowledgement statuses a fail-safe branch can speak about. STATUS_
# REJECTED is the only one any function in this package ever RETURNS; STATUS_
# ACCEPTED exists so that "this branch must never return accepted" is a statement
# the tests can make and a mutant can violate. Spelled exactly as the wire value
# in 11 (see the module docstring) -- a different spelling here would validate
# against nothing and let a peer's real "rejected" slip past an equality check.
STATUS_REJECTED = "rejected"
STATUS_ACCEPTED = "accepted"

# Confirmation levels, 18 S0.3.1 (U53 split the old L1 into L1a / L1b). Only the
# two the lateral-move fail-safe needs are bound here; the full ladder is
# {L0, L1a, L1b, L2, L3} and belongs to the voice layer, not to common/.
#   L1a  并发播报 -- command and TTS fire together, no pre-announcement
#   L1b  前置播报 -- TTS plays to completion, THEN the command executes
# The distinction is load-bearing for A07/A08: 18 S3.1 says判 L1a 直接违反契约 for
# a blind-direction move, so the lateral fail-safe must be able to name L1b and be
# caught if it ever downgrades to L1a.
CONFIRM_L1A = "L1a"
CONFIRM_L1B = "L1b"


@dataclass(frozen=True)
class FailSafeResult:
    """One fail-loud rejection from a permanently-unavailable capability.

    frozen so a caller cannot turn a rejection into an acceptance after the fact
    by writing result.status -- the whole value of this type is that the verdict
    is fixed at construction. detail is still a plain mapping and therefore
    mutable in place; each factory below builds a fresh dict per call rather than
    sharing one, so mutation of a returned detail affects only that one result,
    the same trade-off XbrainError documents for its own detail field.
    """

    # status: always STATUS_REJECTED for every branch in this package. It is a
    # field and not a constant return because the T-PTZ-1 mutant (21 S1) is
    # literally "return accepted instead", and a function with no status to set
    # could not be mutated that way -- the assertion would have nothing to bite.
    status: str

    # code: a value from the E_* closed set, handed in by the branch. NOT checked
    # against ALL_CODES here -- errors.info() is the single membership gate (11
    # S13.6) and re-checking would either duplicate the table or raise a second
    # exception on top of the first. The branches pass errors.E_BUSY /
    # errors.E_CAPABILITY, never string literals (CLAUDE.md 3.5).
    code: str

    # detail: the structured payload EC-3 / RCE-2 require. Its required keys differ
    # per branch (rotation needs reason+occ_count+r_check_m, preset needs reason),
    # so this type does not prescribe them -- it only guarantees a detail is
    # present. A branch that shipped detail=None would be契约不合规 per RCE-2, so
    # the factories always pass a dict, never None.
    detail: Mapping[str, object]

    # guidance: an optional pointer to the intent the operator should use instead
    # (E10 -> E01). None when the block offers no alternative (a blocked rotation
    # has none -- there is no other way to turn in place). It is an intent id, not
    # a sentence: turning it into TTS is the gateway's job (11 S8.13.5).
    guidance: Optional[str] = None


__all__ = [
    "STATUS_REJECTED",
    "STATUS_ACCEPTED",
    "CONFIRM_L1A",
    "CONFIRM_L1B",
    "FailSafeResult",
]
