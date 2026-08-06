"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The E_* closed set, exported for every XBRAIN runtime process

Description:
CLAUDE.md 3.5 forbids E_* string literals outside this package. The reason is
concrete: a code spelled slightly differently in two processes compiles, runs,
and only surfaces during integration, when the cloud client cannot branch on it.

Every name here is derived from codes.yaml at import time, which is itself
generated from 11 S13.4~S13.15. Nothing is typed twice. The deployed C++ header
common/errors/errors.h comes from the same file (CFG-CM-3), so the two languages
cannot drift apart either.

*** Out-of-set values raise. 11 S13.6 requires it in so many words: no silent
pass-through, no "interpret the unknown value as something close". A parser that
falls back to E_INTERNAL on an unrecognised code turns a contract violation into
a plausible-looking log line, and the cloud can no longer tell them apart.

*** No third-party import may ever appear in this package. Every runtime process
imports it, so an "import yaml" here turns one missing wheel into a startup
failure for all of them at once, with a traceback pointing at the error-code
layer instead of at the environment. That constraint is the only reason _load
hand-parses a format a library already handles; its docstring bounds when that
trade is acceptable.

Importing this module parses and validates the table, so "import" itself is a
failure point: a malformed row raises ValueError before any caller runs. That is
the intent -- a bad table must stop startup, not the error path it describes.

What this module deliberately does NOT do, so nothing gets bolted onto it:
  * it does not map transport status onto codes. The 11 S8.13.5 heading (grep
    错误映射) names the gateway as the single implementation point, so an AI
    service answering with free-form detail is NOT in breach of anything; do not
    push E_* down into services/ to make some caller type-check.
  * it does not decide operator-facing wording. meaning is contract prose for a
    human reading a log. It is not an HMI string and not a translation table.
  * it does not own the other closed sets (domain, plane, Event.category,
    gate.limiter, stop_reason, TaskState). Those live in xbrain/common/enums/
    and raise ClosedSetViolation, which shares the XbrainError base so a single
    except clause still covers the whole family.
"""

import os
from typing import Dict, FrozenSet, NamedTuple

from .exceptions import UnknownErrorCode

# The table is resolved against THIS module's own directory -- not against the
# configs root, and above all not against /run/xbrain/resolved/. codes.yaml is a
# generated contract artifact shipped inside the package, so it is not
# configuration: every process must see byte-identical content, which is exactly
# the guarantee the resolved tree does not offer, since it holds one separately
# expanded file per process (10 S5.4.1). Putting the closed set there would let
# two processes disagree about what a code means, which is the one failure this
# package exists to prevent.
_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "codes.yaml")

# retryable, per 11 S13.2. That section states the column is a safety item, not a
# UX one -- treating E_LOCKED (HES not cleared) as retryable makes a cloud client
# resend motion commands at 10 Hz.
#
# Plain str rather than an Enum, and the spelling is load-bearing in two
# directions at once: these values are compared for equality against the raw
# column text of codes.yaml, and the same text is emitted into the generated C++
# header (CFG-CM-3). Renaming one here without regenerating leaves Python
# rejecting every row of the table it was meant to accept -- the failure appears
# as "the whole closed set is malformed", not as "one constant was renamed".
RETRY_YES = "yes"                   # transient, clears by itself within seconds
RETRY_CONDITIONAL = "conditional"   # an external condition must change first
RETRY_NO = "no"                     # invalid or not permitted -- STOP retrying
RETRY_NA = "n_a"                    # not a failure at all, e.g. OK, idempotent hit
# Spelled "n_a" because that is what the generator writes into the column;
# "n/a" here would match no row and every OK row would be rejected at import.
_RETRY_VALUES = frozenset({RETRY_YES, RETRY_CONDITIONAL, RETRY_NO, RETRY_NA})

# detail, per EC-3. Three states, not a bool, and kept three-way on purpose:
#   required     the contract literally says detail 必填
#   implied      the contract prescribes detail contents in different wording
#   unspecified  the contract says nothing about detail for this code
# Collapsing "implied" into either neighbour means guessing which side the
# contract meant, and that guess belongs in 11, not in this file. codes.yaml
# records the ambiguity as an open contract question rather than resolving it
# locally, so do not "simplify" this frozenset to two members.
_DETAIL_VALUES = frozenset({"required", "implied", "unspecified"})


class ErrorCode(NamedTuple):
    """One row of 11 S13.4~S13.15."""

    # Immutable on purpose (NamedTuple, or dataclass(frozen=True) -- what is NOT
    # allowed is a mutable row). info() hands out the very object stored in
    # _CODES, so a writable field would let one importer redefine a code for
    # every other importer sharing the interpreter, with no import failing.
    code: str        # the wire value, identical to the attribute bound below
    group: str       # the S13.4~S13.15 letter, A..L, the row was read from
    retryable: str   # one of the RETRY_* constants above, never free text
    detail: str      # one of _DETAIL_VALUES, never free text
    meaning: str     # contract prose. Nothing branches on it -- see _build.
    # group has no runtime consumer either; its one reader is the metatest
    # test_group_assignment_matches, which catches a code being moved between
    # S13.x sections -- a move the set-level difference test cannot see, because
    # the set is unchanged. Dropping the field would silently retire that check.


# Validation happens here, at load time, and NOT lazily inside retryable(). A row
# with a mistyped column that is only checked on use stays dormant until the
# error path it describes actually fires -- that is, in the field, during the
# incident it was written to classify. Checking at import moves the discovery to
# Stage 0 on the bench, where it costs nothing.
def _build(d: Dict[str, str], lineno: int) -> ErrorCode:
    """Validate one parsed block and turn it into a row.

    lineno is for the message only, and it is a bracket rather than a cursor:
    the caller flushes the previous block when it reaches the NEXT "- code:"
    line, so the number printed points at where the FOLLOWING block starts.
    -1 is the sentinel the post-loop flush passes, meaning "the last block in
    the file". Do not chase the printed line literally; look just above it.
    """
    # .get() with NO second argument. This is not the CLAUDE.md 3.1 fallback
    # pattern it resembles -- there is no default -- and the absence of one is
    # precisely what makes a MISSING column fail identically to a misspelt one:
    # .get returns None, None is not in the frozenset, the row is rejected with
    # its code name and file position attached. d["retryable"] would instead
    # raise a bare KeyError carrying neither.
    # NEVER add a default. d.get("retryable", RETRY_NO) would silently classify
    # every row whose column the generator happened to drop, which is the exact
    # fail-silent shape CLAUDE.md 3.1 exists to forbid.
    if d.get("retryable") not in _RETRY_VALUES:
        raise ValueError(f"{_YAML}:{lineno}: {d.get('code')} has retryable="
                         f"{d.get('retryable')!r}, not one of {sorted(_RETRY_VALUES)}")
    # Same shape for detail. Both messages sort the frozenset before printing so
    # the text is reproducible run to run; an unordered set would make two
    # identical failures print differently and read like two separate bugs.
    if d.get("detail") not in _DETAIL_VALUES:
        raise ValueError(f"{_YAML}:{lineno}: {d.get('code')} has detail="
                         f"{d.get('detail')!r}, not one of {sorted(_DETAIL_VALUES)}")
    # code and group are subscripted rather than fetched with .get: a block
    # missing either is structurally broken and must not become a row at all.
    # meaning is the one field allowed a default, and the split is not stylistic
    # -- nothing branches on meaning, so an empty one only degrades a log line,
    # whereas an empty retryable would change a decision.
    return ErrorCode(code=d["code"], group=d["group"], retryable=d["retryable"],
                     detail=d["detail"], meaning=d.get("meaning", ""))


# Why a hand-rolled reader is acceptable HERE and would not be anywhere else:
# the input is not written by a human. The generator emits one fixed shape --
# a "codes:" key at column 0, then blocks that each open with "- code:" and
# continue as plain "key: value" lines. Five keys, no nesting, no multi-line
# scalars, no anchors, no lists inside a block. This reader models exactly that
# shape and refuses everything else.
#
# Two format constraints the generator must keep, because this reader cannot
# express them and will not report them being broken:
#   * no "#" inside a value -- it is cut as a comment, tail and all, silently
#   * no embedded double quotes -- only the outer pair is stripped
# Chinese text is fine, and so is a ":" inside a value; see the split below.
def _load() -> Dict[str, ErrorCode]:
    """Parse codes.yaml without a yaml dependency.

    This package is imported by every runtime process, including ones that start
    before any virtualenv is guaranteed, so it deliberately has no third-party
    imports. The format is fixed and generated, so a five-key block reader is
    enough -- and it raises on any line it does not recognise rather than
    skipping it. A loader that skips what it cannot parse silently shrinks the
    closed set, which is the one failure this module exists to prevent.
    """
    out: Dict[str, ErrorCode] = {}
    cur: Dict[str, str] = {}
    in_codes = False                 # header prose above "codes:" is not data
    # encoding is stated, never left to the platform default. The meaning column
    # is Chinese, and the default is locale-derived: the same file that decodes
    # in a developer shell can raise UnicodeDecodeError under a service manager
    # that starts the process with a stripped environment. Failing that way
    # would look like a corrupt table rather than a missing locale.
    with open(_YAML, encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            # The two branches are equivalent, not complementary: split("#")[0]
            # already yields "" for a whole-line comment, so removing the first
            # branch would change nothing. It is spelled out to state the intent.
            # WARNING: the cut is at the first "#" anywhere on the line, quoted
            # or not -- see the generator constraints above.
            line = "" if raw.lstrip().startswith("#") else raw.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            if line.rstrip() == "codes:":
                in_codes = True      # exact and unindented: a nested key is indented
                continue
            # The ONLY place this reader is allowed to drop a line it does not
            # understand, and it is safe for one reason: nothing above "codes:"
            # is data. Everything up there is the file header. Widening this
            # branch to cover anything else reintroduces the silent-shrink bug.
            if not in_codes:
                continue
            if line.lstrip().startswith("- code:"):
                # A block is flushed when the NEXT one starts, which is why the
                # post-loop flush further down is not optional: dropping it loses
                # exactly the final code, and a closed set short by one entry
                # does not look broken -- the missing code simply starts raising
                # UnknownErrorCode, which reads like the caller's mistake.
                # Last block wins on a repeated code, silently: out is a dict, so
                # a second block naming an existing code discards the earlier
                # row's columns. Only the mistyped-name case is covered, by the
                # metatest -- the code the block was meant to define is then
                # absent and the symmetric difference goes non-empty. Two blocks
                # honestly naming the same code with conflicting columns is a
                # hole nothing catches; it belongs to the generator, upstream.
                if cur:
                    out[cur["code"]] = _build(cur, lineno)
                cur = {"code": line.split(":", 1)[1].strip()}
                continue
            # Refuse, do not skip. This is the enforcement point for the whole
            # docstring above: any line the generator emitted that this reader
            # does not model stops the import instead of quietly vanishing.
            if ":" not in line:
                raise ValueError(f"{_YAML}:{lineno}: unparsable line {line!r}")
            # maxsplit=1 so a ":" inside a value survives intact. strip('"')
            # then removes the generator's optional quoting -- note it strips
            # every quote character at both ends, not one matched pair, which is
            # the second generator constraint listed above.
            key, val = line.split(":", 1)
            cur[key.strip()] = val.strip().strip('"')
    if cur:
        out[cur["code"]] = _build(cur, -1)
    # An empty result is a hard failure, never an empty closed set. If "codes:"
    # were ever renamed or indented, in_codes would stay False, every data line
    # would be dropped by the header branch, and this function would return {}
    # without raising -- after which every info() call raises UnknownErrorCode
    # and the whole system reports "the caller used a bad code" while the truth
    # is "the table never loaded". Failing here keeps the two distinguishable.
    if not out:
        raise ValueError(f"{_YAML}: no codes parsed")
    return out


_CODES: Dict[str, ErrorCode] = _load()

#: The closed set itself. !! Never write its size into code or comments (3.7).
# frozenset and not set: the closed set has to be un-addable-to at run time. A
# caller able to do ALL_CODES.add(...) would have invented a code, which is what
# 11 S13.6 forbids, and it would do so without any import ever failing.
ALL_CODES: FrozenSet[str] = frozenset(_CODES)

# Bind every code as a module attribute so callers write E_TIMEOUT, not "E_TIMEOUT".
# The bound value IS the string, so the constant and the wire form cannot
# diverge. Generated rather than typed out because typing them out would be a
# second copy of the table, which is what CLAUDE.md 3.5 forbids.
# Accepted cost: mypy and the IDE cannot see these names. The compensation is
# that a typo now fails loudly and early -- "from xbrain.common.errors import
# E_TIMEOUTT" is an ImportError at import time, whereas the bare string literal
# it replaces would have travelled all the way to the cloud client and been
# ignored there.
globals().update({c: c for c in _CODES})

# The dynamic names must be listed explicitly or "import *" would not carry
# them: injecting into globals() is invisible to the default star-export rule.
__all__ = ["ALL_CODES", "ErrorCode", "UnknownErrorCode", "info", "retryable",
           "detail_requirement", "is_failure", "RETRY_YES", "RETRY_CONDITIONAL",
           "RETRY_NO", "RETRY_NA", *sorted(_CODES)]


# The single lookup point. retryable(), detail_requirement() and is_failure()
# all route through it, so there is exactly one place where an out-of-set value
# can enter the system, and that place raises. Adding a second lookup path that
# does not go through here is how 11 S13.6 gets violated by accident.
def info(code: str) -> ErrorCode:
    """The full row for a code. Raises UnknownErrorCode outside the set."""
    try:
        return _CODES[code]
    except KeyError:
        # "from None" suppresses the exception chain deliberately. The chained
        # "During handling of the above exception" block buries the message that
        # matters -- which value was not in the closed set, and where the set is
        # defined -- behind a KeyError that repeats the same string and adds
        # nothing a reader can act on.
        raise UnknownErrorCode(code, sorted(ALL_CODES)) from None


def retryable(code: str) -> str:
    """One of RETRY_YES / RETRY_CONDITIONAL / RETRY_NO / RETRY_NA."""
    # Returns the contract's classification, not a decision. 11 S13.2 says only
    # which failures may be resent at all; the interval, the ceiling and the
    # give-up rule are the caller's, and encoding them here would make one
    # policy binding on every process that imports this module.
    return info(code).retryable


def detail_requirement(code: str) -> str:
    """One of 'required' / 'implied' / 'unspecified' -- EC-3."""
    # Three-valued: a caller that branches two ways has already picked a side.
    # "implied" is not a weaker "required" -- folding it into required rejects
    # traffic the contract never forbade, folding it into unspecified ships a
    # code nobody can act on. Which way it folds is 11's call to make, not a
    # caller's, so treat an implied code as needing a decision, not a default.
    return info(code).detail


def is_failure(code: str) -> bool:
    """True for every code that is not a success.

    EC-1: a rejected result must carry code != OK. E_DUPLICATE is an idempotent
    hit -- the contract says to treat it as success, so it is not a failure here
    either, and its retryable is n_a for the same reason.
    """
    # Discriminated on the retryable column rather than on code != "OK". Where
    # the two tests diverge is the whole point: E_DUPLICATE is spelled like an
    # error but the contract calls it success, and any future code the contract
    # classes the same way is picked up here without editing this function.
    # Comparing against "OK" would also reintroduce a hardcoded code literal,
    # which CLAUDE.md 3.5 forbids even inside this package.
    return info(code).retryable != RETRY_NA
