"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: digest.py
Brief: common_digest per 10 S5.4.4 -- sha256 of the canonical common.* subtree

Description:
What this is for. The freeze line resolves configuration once and writes
/run/xbrain/resolved/{proc}.yaml plus MANIFEST.json; common_digest is the field
in that manifest by which every process can answer "am I running on the same
common.* as everyone else". Six processes read six different resolved files, and
without a shared fingerprint a partial rollout -- one process restarted against a
newer configuration than its peers -- is invisible until the mismatch produces
behaviour nobody can explain.

The rule, from 10 S5.4.4:
  digest = sha256( canonical_json(resolved common.* subtree) ), first 16 hex chars

Two boundaries the section states explicitly and this module enforces:

  * ONLY common.*. Process-private sections (L6) are excluded, because a change
    confined to one process must not block the whole stack from being released.
    take_common_subtree() below is what makes that true rather than merely
    intended -- a caller that passed the whole tree would produce a digest that
    changes whenever any process edits its own section.

  * The subtree must already be RESOLVED. Digesting a tree with ${common.x}
    still in it would fingerprint the text of the references rather than their
    values, so two configurations that differ only in what an alias points at
    would share a digest. assert_resolved() refuses that input rather than
    quietly hashing it, because the failure it prevents is silent.

Why 16 characters and not the full 64. The section says so. It is a human-facing
identifier -- it goes into the manifest, into the startup event, and gets read
aloud during commissioning -- and 64 bits is far beyond what an accidental
collision between two configuration files needs.

Why this is not simply hashlib over json.dumps. See canonical.py: the byte
sequence has to be reproducible from a C++ implementation as well, and neither
key ordering nor float rendering is stable across the two languages by default.
"""

import hashlib
import re
from typing import Any, Dict

from .canonical import canonical_json

#: Length of the digest in hexadecimal characters, from 10 S5.4.4. Named rather
#: than inlined so the one place it is defined is also the place the section
#: number is written down.
DIGEST_HEX_LEN = 16

#: A leaf that still looks like a reference. Deliberately loose -- it matches
#: anything with the ${...} shape, including the malformed spellings R-3 forbids,
#: because the point here is "this tree was not resolved", not "this reference is
#: well formed". Shape validation belongs to config/refs.py.
_UNRESOLVED = re.compile(r"\$\{[^}]*\}")


class UnresolvedTree(ValueError):
    """Raised when a tree still containing references is offered for digesting.

    Its own type rather than a bare ValueError so a caller can distinguish "your
    input was not ready" from "your input was not serialisable", which need
    different fixes: the first is a call-order bug at the freeze line, the second
    is bad data.
    """


def take_common_subtree(tree: Dict[str, Any]) -> Dict[str, Any]:
    """The common.* subtree, or an empty mapping when there is none.

    An absent common key yields {} rather than raising. A tree with no common
    section is a legitimate thing to digest -- it is what a process-private-only
    overlay looks like -- and it has a well defined digest, namely the digest of
    the empty object. Raising here would push the caller into writing its own
    special case, and special cases around a fingerprint are how two callers end
    up fingerprinting different things.
    """
    node = tree.get("common", {})
    if not isinstance(node, dict):
        raise TypeError("common must be a mapping, got %s" % type(node).__name__)
    return node


def assert_resolved(node: Any, path: str = "common") -> None:
    """Refuse a subtree that still carries ${...} references.

    Walks rather than searching the serialised text, so the offending key path
    can be named. "somewhere in your configuration there is an unresolved
    reference" is the kind of message that sends an operator through hundreds of
    keys by hand -- the same reason 10 S5.4.3 requires the cycle report to print
    the whole loop.
    """
    if isinstance(node, str):
        if _UNRESOLVED.search(node):
            raise UnresolvedTree(
                "cannot digest an unresolved tree: %s = %r still holds a "
                "reference; resolve the reference axis first (10 S5.4.2)"
                % (path, node)
            )
        return
    if isinstance(node, dict):
        for key, value in node.items():
            assert_resolved(value, path + "." + key)
        return
    if isinstance(node, (list, tuple)):
        for i, value in enumerate(node):
            assert_resolved(value, "%s[%d]" % (path, i))


def common_digest(tree: Dict[str, Any]) -> str:
    """The 16-character digest of a resolved configuration tree.

    Takes the WHOLE tree and extracts common.* itself, rather than asking the
    caller to hand in the subtree. That is deliberate: the "only common.*" rule
    is the one an incorrect caller breaks silently, and a signature that cannot
    express the mistake is worth more than a comment telling callers not to make
    it.
    """
    subtree = take_common_subtree(tree)
    assert_resolved(subtree)
    text = canonical_json(subtree)
    # Encoded here, once, on the whole string. Encoding piecewise inside the
    # serialiser would work but would put the choice of codec in several places,
    # and the digest is defined over UTF-8 bytes specifically.
    full = hashlib.sha256(text.encode("utf-8")).hexdigest()
    # The FIRST sixteen characters. "Take sixteen characters" also admits the
    # last sixteen and a hash of the hash, both of which would satisfy every
    # property a reader might check, so the slice is pinned by a test rather
    # than left to whoever ports this next.
    return full[:DIGEST_HEX_LEN]


def canonical_bytes(tree: Dict[str, Any]) -> bytes:
    """Exactly the bytes common_digest hashes.

    Exported so a cross-language test can compare the INPUT to the hash and not
    only its output. When Python and C++ disagree on a digest, the first question
    is whether they disagreed on the serialisation or on the sha256; without this
    the two are indistinguishable and the search starts from nothing.
    """
    subtree = take_common_subtree(tree)
    assert_resolved(subtree)
    return canonical_json(subtree).encode("utf-8")
