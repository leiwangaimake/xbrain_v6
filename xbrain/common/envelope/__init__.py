"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The 11 S3.0 envelope -- decode / encode, the S3.0.1 age, the fail-safe

Description:
INF-CM-2. This package is the single home for the outer Zenoh envelope every JSON
payload on either plane is wrapped in, and for the two rules the contract hangs on
it. Three concerns, three modules, kept apart because they fail and are tested
differently:

  envelope.py        the nine fields (v / rid / ts / mono / boot / seq / src /
                     ts_sync / data). decode() is the strict validator -- missing
                     required field -> E_SCHEMA, ts_sync missing -> false (never
                     true), mono absent -> the cloud case (CLK-C4), unknown fields
                     ignored (F-2). encode() is its inverse.
  age.py             message_age_s, the ONE age computation of S3.0.1, four
                     branches: produced age when mono is present and boot matches,
                     receive-time fallback when boot mismatches or mono is absent,
                     and the CLK-C5 negative-age case -- emit a warn / system event
                     with detail.kind == "negative_age", then clamp to 0.
  directionality.py  the S3.0.1 tightening / loosening fail-safe. A malformed
                     collapse-safe message (cmd/estop, behavior/request cancel)
                     is executed, not rejected; everything else rejects with
                     E_SCHEMA. Per 99 U75 the exemption is collapse-safety, NOT a
                     key name.

The C++ twin of the age computation lives at
common/include/xbrain/envelope/message_age.h, and
tests/common/envelope/test_age_cross_language.py compiles it against the same
golden vectors this side is tested on, so "Python and C++ produce the same age
for the same envelope" (INF-CM-2 criterion four) is measured, not asserted --
following the digest package's cross-language pattern exactly.

Why age and decode are separate packages from clock. xbrain.common.clock is the
positive half of CLK-C1: it hands out one monotonic reading and, deliberately,
computes no age, because age is a four-branch decision that needs envelope fields
(its own module docstring says so and points here). This package is where that
decision lives.

Naming note. CLAUDE.md and the documents write the shared library as common/;
on disk the Python source is xbrain/common/ (CLAUDE.md 0.2 reserves the top-level
common/ for deployed C++ artifacts, which is where message_age.h goes). INF-CM-2's
target-directory column says xbrain/common/envelope/, the same reading.
"""

# envelope.py -- the nine-field structure.
#   Envelope             a frozen decoded envelope; mono / boot are Optional
#                        because a cloud packet carries neither (CLK-C4)
#   decode / encode      the strict validator and its inverse
#   EnvelopeSchemaError  the E_SCHEMA failure, a named type so a caller catches
#                        exactly the envelope case (CLAUDE.md 4.5)
#   KNOWN_VERSIONS       the accepted schema versions, {1} today per S3.0
from .envelope import (
    Envelope,
    EnvelopeSchemaError,
    KNOWN_VERSIONS,
    decode,
    encode,
)

# age.py -- S3.0.1.
#   message_age_s        the age, with the CLK-C5 event wired through a required
#                        sink so the emission can never be silently dropped
#   compute_age          the pure branch / clamp logic, for cross-language tests
#   AgeResult            its result: clamped age, raw age, branch, was_negative
#   NegativeAgeEvent     the warn / system event; sev / cat / detail
#   NegativeAgeSink      the injected callback type
#   read_local_boot_id   this host's LOCAL_BOOT_ID (first 8 hex of boot_id)
#   BRANCH_* / NEGATIVE_AGE_*  the fixed labels and event fields
from .age import (
    AgeResult,
    BRANCH_PRODUCED,
    BRANCH_RX_FALLBACK,
    NEGATIVE_AGE_CAT,
    NEGATIVE_AGE_KIND,
    NEGATIVE_AGE_SEV,
    NegativeAgeEvent,
    NegativeAgeSink,
    compute_age,
    message_age_s,
    read_local_boot_id,
)

# directionality.py -- S3.0.1 fail-safe.
#   Direction            TIGHTENING (collapse-safe) vs LOOSENING (everything else)
#   Disposition          ACCEPTED vs COLLAPSE_SAFE; rejection RAISES instead
#   GuardedResult        the outcome of guarded_decode
#   is_collapse_safe     the one-line decision primitive
#   guarded_decode       decode wrapped in the direction-dependent fail-safe
from .directionality import (
    Direction,
    Disposition,
    GuardedResult,
    guarded_decode,
    is_collapse_safe,
)

# __all__ is spelled out in full rather than assembled from the sub-modules'
# own __all__ lists. Two reasons. It is the one place a reader can see the entire
# public surface of the package at a glance, without opening three files. And an
# explicit list is what makes "import *" carry exactly these names and no private
# helper -- the sub-modules export a few internals to each other (for instance
# EnvelopeSchemaError is imported by directionality.py) that have no business in
# the package's public surface.
#
# The grouping comments below are load-bearing for a maintainer, not decoration:
# a name added to a sub-module must also be added here, under its own group, or it
# is unreachable through the package and the omission is silent.
__all__ = [
    # envelope.py -- the nine-field structure, its E_SCHEMA failure type, and the
    # accepted schema versions.
    "Envelope", "EnvelopeSchemaError", "KNOWN_VERSIONS", "decode", "encode",
    # age.py -- the S3.0.1 age, its pure core, its result type, and the CLK-C5
    # event value / sink / fixed labels.
    "AgeResult", "BRANCH_PRODUCED", "BRANCH_RX_FALLBACK",
    "NEGATIVE_AGE_CAT", "NEGATIVE_AGE_KIND", "NEGATIVE_AGE_SEV",
    "NegativeAgeEvent", "NegativeAgeSink",
    "compute_age", "message_age_s", "read_local_boot_id",
    # directionality.py -- the tightening / loosening fail-safe: the two
    # directions, the disposition of a guarded decode, and the two entry points.
    "Direction", "Disposition", "GuardedResult",
    "guarded_decode", "is_collapse_safe",
]
