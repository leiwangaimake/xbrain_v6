"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The two cross-language fingerprints, and the one serialisation under both

Description:
CFG-CM-10. This package holds the two identifiers that have to come out
bit-identical from a Python implementation and a C++ one:

  common_digest   10 S5.4.4   sha256 of the resolved common.* subtree, 16 hex
  fence_crc32     11 S9A.2    CRC-32 of a FenceSet reduced to a canonical string

They are packaged together because 10 S5.4.4 says to: the two share one
normalisation implementation. The shared part is canonical.py -- key ordering,
whitespace, escaping and number rendering -- and the two fingerprints differ only
in what they cover and which hash they end in.

Why cross-language matters here and almost nowhere else in this code base. Both
numbers are used to decide whether two parties hold the same thing. A Python
producer and a C++ consumer that disagree do not produce a wrong answer; they
produce a permanent disagreement, and the visible symptom is a consumer that
rejects every frame with E_SCHEMA, or a commit that reports duplicate forever.
Neither symptom points at the serialiser.

The C++ side lives in common/include/xbrain/digest/canonical_digest.h. It is
header-only and depends on nothing outside the standard library -- in particular
no rclcpp, per CLAUDE.md 5.3, because its consumers include chassis_relay and
rtk_driver, neither of which may pull in ROS. It does use std::string and so
allocates; that is acceptable because both fingerprints are computed at
stage/commit time, never inside the 20 Hz loop or the relay hop CRL-5 budgets at
200 microseconds.

tests/common/test_digest_cross_language.py compiles that header and runs it
against the same golden vectors this side is tested on, so the claim that the two
agree is measured rather than asserted. If no compiler is present the test skips
loudly and names what went unverified, rather than passing.
"""

# Imports are grouped by the section each belongs to, not alphabetically. The
# two fingerprints are maintained against two different documents, and a reader
# arriving from one of them should not have to work out which names are theirs.
#
# The shared normalisation. Exported rather than kept private because both
# fingerprints are defined in terms of it, and a caller debugging a mismatch
# needs to see the string before the hash -- see the note on canonical_bytes and
# canonical_fence_string below.
from .canonical import canonical_json, format_number

# The config side, 10 S5.4.4.
#   common_digest        the value that goes into MANIFEST.json
#   canonical_bytes      the input to the hash, for diagnosing a disagreement
#   take_common_subtree  the "only common.*" rule, as a function rather than a
#                        convention callers are asked to remember
#   assert_resolved      refuses a tree that still holds ${...}
#   UnresolvedTree       its own type, so "not ready" and "not serialisable"
#                        can be told apart -- they need different fixes
from .digest import (
    DIGEST_HEX_LEN,
    UnresolvedTree,
    assert_resolved,
    canonical_bytes,
    common_digest,
    take_common_subtree,
)

# The fence side, 11 S9A.2.
#   fence_crc32            what FV-8 recomputes and S-5 keys idempotency on
#   verify_fence_crc32     the FV-8 comparison; returns a bool because the error
#                          code depends on where the FenceSet came from
#   canonical_fence_string the string under the checksum, for the same
#                          diagnostic reason as canonical_bytes
#   FenceRecipeError       malformed input, which is E_SCHEMA territory, as
#                          opposed to a mismatch, which points at transport
from .fence import (
    FenceRecipeError,
    canonical_fence_string,
    fence_crc32,
    verify_fence_crc32,
)

# Listed explicitly rather than left implicit. An import * that silently picked
# up hashlib and binascii from the submodules would let a caller reach a hashing
# primitive through this package, and the whole point of the package is that
# there is exactly one way to compute each of these two numbers.
__all__ = [
    # shared normalisation
    "canonical_json",
    "format_number",
    # 10 S5.4.4
    "common_digest",
    "canonical_bytes",
    "take_common_subtree",
    "assert_resolved",
    "UnresolvedTree",
    "DIGEST_HEX_LEN",
    # 11 S9A.2
    "fence_crc32",
    "verify_fence_crc32",
    "canonical_fence_string",
    "FenceRecipeError",
]
