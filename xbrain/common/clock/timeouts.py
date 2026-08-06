"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: timeouts.py
Brief: The 11 S1.6 global timeout registry, exported for every XBRAIN process

Description:
11 S1.6 is the single table where every timeout, period and age deadline in the
system is registered -- the volume states plainly that without one central table
"the worst case can never be worked out". This module makes that table importable
so nothing re-types a threshold, and so the deployed C++ header can be generated
from one source. It is the second half of INF-CM-3; mono_now_s / MonoClock next
door are the first half (the reading), and this is the table of deadlines those
readings are compared against.

What each field is, and the one that is easy to misread:
  * id        -- the T-xx label, e.g. T-01. The closed set of ids IS S1.6.
  * group     -- A safety loop / B link-session-health / C non-safety, from the
                 subsection the row sits under.
  * timebase  -- monotonic or wall. Derived from the document's own rule (A is
                 全部单调时钟 with no exception; T-46 is the single wall entry),
                 not from a column, because S1.6 has no clock column.
  * threshold -- *** CONTRACT TEXT, NOT A RUNTIME VALUE. It is the in-force token
                 from the 阈值 cell, kept verbatim so an uncalibrated entry reads
                 as the contract wrote it ("待实测", "100–200 ms 待定",
                 "timeout_s") and is NEVER a fabricated number. The number a
                 process actually enforces is loaded from that process's config
                 (p4_agent.yaml, p5_gateway.yaml, ...), where CLAUDE.md 3.1 lets
                 an uncalibrated value stay null and refuse startup. Treating
                 this string as a limit to compute with would reintroduce exactly
                 the hardcoded-safety-default that rule forbids.

*** No third-party import may appear in this package. clock is imported by every
runtime process, including the P1 20 Hz loop and ones that start before any
virtualenv is guaranteed, so an "import yaml" here would turn one missing wheel
into a startup failure for all of them at once. That is the only reason _load
hand-parses a format a library already handles; the reader models exactly the
fixed shape this file's own timeouts.yaml is written in and RAISES on anything
else rather than skipping it -- a loader that skips a line it does not understand
silently shrinks the registry, and a registry short one timeout is a protection
that quietly went missing.

Importing this module parses and validates the table, so "import" itself is a
failure point: a malformed row raises ValueError before any caller runs. That is
the intent -- a bad table must stop startup on the bench, not the deadline it
describes, in the field.

What this module deliberately does NOT do:
  * it does not compute age or decide whether a deadline has passed. It holds the
    thresholds; the comparison against a reading is the caller's, and the age
    arithmetic with its four branches belongs to the envelope layer (11 S3.0.1),
    not here.
  * it does not expose a threshold as a number. See the threshold note above --
    the runtime value lives in config, and parsing "200 ms" into 0.2 here would
    make this file a second, un-calibratable source for a safety limit.
  * it does not verify the thresholds are RIGHT for the hardware. That is a human
    reading S1.6 against the vendor. The metatest only proves this file and S1.6
    have not drifted apart (tests/common/clock/test_timeout_table.py).
"""

import os
import re
from collections import OrderedDict
from types import MappingProxyType
from typing import Dict, FrozenSet, Mapping, NamedTuple

# The three subsection groups of S1.6. Plain str, not an Enum, because the values
# are compared for equality against the raw column text of timeouts.yaml and are
# emitted verbatim into the generated C++ header -- an Enum member would have to
# be unwrapped in both places and the spelling is what has to match.
GROUP_SAFETY = "A"        # 11 S1.6.1 -- the safety loop
GROUP_LINK = "B"          # 11 S1.6.2 -- link, session and health
GROUP_NONSAFETY = "C"     # 11 S1.6.3 -- the non-safety loop
_GROUPS: FrozenSet[str] = frozenset({GROUP_SAFETY, GROUP_LINK, GROUP_NONSAFETY})

# The two timebases. monotonic is the rule for all but one row; wall is T-46
# alone, and S1.6 marks it "禁用于安全判定" for the reason CLK-C4 gives -- a
# monotonic reading cannot be compared across hosts, so a cloud freshness check
# has no other basis and must accept the wall clock's coarser guarantee.
TIMEBASE_MONOTONIC = "monotonic"
TIMEBASE_WALL = "wall"
_TIMEBASES: FrozenSet[str] = frozenset({TIMEBASE_MONOTONIC, TIMEBASE_WALL})

# An id is T- followed by digits or LETTERS (T-01 .. T-46, T-ESTOP-CLOUD,
# T-BCAST-MAX). Anchored and fullmatch-checked at load so a stray value in the id
# slot -- a header cell that slipped through, say -- stops the import instead of
# becoming a registry key that no lookup will ever match.
_ID_RE = re.compile(r"T-[0-9A-Za-z]+(?:-[0-9A-Za-z]+)*")


class Timeout(NamedTuple):
    """One row of 11 S1.6."""

    # Immutable on purpose (NamedTuple). timeout() and TIMEOUTS hand out the very
    # object stored, so a writable field would let one importer redefine a
    # timeout for every other importer in the interpreter, with no import failing.
    id: str          # the T-xx label, identical to its key in TIMEOUTS
    group: str       # one of the GROUP_* constants, never free text
    timebase: str    # one of the TIMEBASE_* constants, never free text
    threshold: str   # contract text from the 阈值 cell -- NOT a number (see header)


# Resolved against THIS module's own directory, exactly like errors/codes.yaml:
# timeouts.yaml is a contract artifact shipped inside the package, not
# configuration. Every process must see byte-identical content, which the
# resolved config tree cannot promise because it expands one file per process
# (10 S5.4.1). Reading it from here, never from configs/ or /run/xbrain/, is what
# keeps two processes from disagreeing about what a timeout is.
_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "timeouts.yaml")


# Validation happens here, at load time, not lazily on first lookup. A row with a
# mistyped group that is only checked on use stays dormant until something asks
# for that timeout -- which, for a safety row, is during the incident it was
# written to bound. Checking at import moves the discovery to Stage 0 on the
# bench, where it costs nothing.
def _build(d: Dict[str, str], lineno: int) -> Timeout:
    """Validate one parsed block and turn it into a Timeout.

    lineno is for the message only and points at where the FOLLOWING block
    starts (the caller flushes the previous block when it reaches the next
    "- id:" line); -1 means the last block in the file. Look just above the
    printed number, not at it.
    """
    # id is subscripted, not fetched with .get: a block with no id is
    # structurally broken and must not become a row at all. The regex is
    # fullmatch so a partial hit ("T-" with nothing after) is rejected too.
    tid = d["id"]
    if not _ID_RE.fullmatch(tid):
        raise ValueError(f"{_YAML}:{lineno}: id {tid!r} is not a T-xx timeout id")
    # .get() with NO second argument, on purpose. A default here would be the
    # CLAUDE.md 3.1 fallback pattern: d.get("group", GROUP_SAFETY) would silently
    # file every row whose group the file happened to drop under safety, and a
    # non-safety timeout mislabelled safety would then be asserted monotonic by
    # criterion (2) and read as fine. .get returns None, None is not in the
    # frozenset, the row is rejected with its id and file position attached.
    if d.get("group") not in _GROUPS:
        raise ValueError(f"{_YAML}:{lineno}: {tid} has group={d.get('group')!r}, "
                         f"not one of {sorted(_GROUPS)}")
    if d.get("timebase") not in _TIMEBASES:
        raise ValueError(f"{_YAML}:{lineno}: {tid} has timebase={d.get('timebase')!r}, "
                         f"not one of {sorted(_TIMEBASES)}")
    # threshold must be present and non-empty. Empty is rejected rather than
    # tolerated because two timeouts that both normalise to "" would compare
    # equal on their value in the metatest and hide a real difference; and a
    # blank threshold in the registry is a row someone half-filled. It is NOT
    # validated for content -- the contract writes "待实测" and "timeout_s" as
    # legitimate thresholds, and second-guessing those is how a fabricated
    # number gets in.
    thr = d.get("threshold")
    if not thr:
        raise ValueError(f"{_YAML}:{lineno}: {tid} has an empty threshold")
    return Timeout(id=tid, group=d["group"], timebase=d["timebase"], threshold=thr)


# Why a hand-rolled reader is acceptable HERE and would not be in ordinary code:
# the input shape is fixed and this file owns both ends of it. timeouts.yaml is a
# "timeouts:" key at column 0, then blocks that each open with "- id:" and
# continue as plain "key: value" lines. Four keys, no nesting, no multi-line
# scalars, no anchors. This reader models exactly that and refuses everything
# else. Two constraints it cannot express and will not report broken: no "#"
# inside a value (cut as a comment, tail and all) and no embedded double quote
# (only the outer pair is stripped). Chinese text and a full-width colon (U+FF1A) are fine; an
# ASCII ":" inside a value survives because the split is maxsplit=1.
def _load() -> "OrderedDict[str, Timeout]":
    """Parse timeouts.yaml without a yaml dependency, preserving file order.

    Raises on any line it does not recognise rather than skipping it: a loader
    that skips silently shrinks the registry, which is the one failure this
    module exists to prevent. Order is preserved so TIMEOUTS reads A, then B,
    then C, matching the table a human is looking at.
    """
    out: "OrderedDict[str, Timeout]" = OrderedDict()
    cur: Dict[str, str] = {}
    in_data = False                  # header prose above "timeouts:" is not data
    # encoding is stated, never left to the platform default: the threshold
    # column is Chinese, and a locale-derived default that decodes in a developer
    # shell can raise UnicodeDecodeError under a service manager with a stripped
    # environment -- which would look like a corrupt table, not a missing locale.
    with open(_YAML, encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            # A whole-line comment becomes empty; otherwise cut at the first "#"
            # so an inline "# 11 S1.6.1" note after a value is dropped. WARNING:
            # the cut is at the first "#" anywhere, so a value must not contain
            # one (none in S1.6 do).
            line = "" if raw.lstrip().startswith("#") else raw.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            if line.rstrip() == "timeouts:":
                in_data = True       # exact and unindented; a nested key is indented
                continue
            # The ONLY place a line may be dropped unrecognised, safe because
            # nothing above "timeouts:" is data -- it is all file header.
            if not in_data:
                continue
            if line.lstrip().startswith("- id:"):
                # Flush the previous block when the next one opens; the post-loop
                # flush below is therefore not optional, or the last row is lost
                # -- and a registry short by one does not look broken, the missing
                # id simply starts raising KeyError like a caller's typo.
                if cur:
                    out[cur["id"]] = _build(cur, lineno)
                cur = {"id": line.split(":", 1)[1].strip()}
                continue
            # Refuse, do not skip: any line the file carries that this reader does
            # not model stops the import rather than vanishing.
            if ":" not in line:
                raise ValueError(f"{_YAML}:{lineno}: unparsable line {line!r}")
            # maxsplit=1 so a ":" inside a value survives; strip('"') then removes
            # the optional quoting (every quote at both ends, not one matched pair).
            key, val = line.split(":", 1)
            cur[key.strip()] = val.strip().strip('"')
    if cur:
        out[cur["id"]] = _build(cur, -1)
    # An empty result is a hard failure, never an empty registry. If "timeouts:"
    # were renamed or indented, in_data would stay False, every row would be
    # dropped as header, and this would return {} without raising -- after which
    # every timeout() call raises KeyError and the system blames the caller while
    # the truth is the table never loaded. Failing here keeps the two apart.
    if not out:
        raise ValueError(f"{_YAML}: no timeouts parsed")
    return out


_TIMEOUTS: "OrderedDict[str, Timeout]" = _load()

#: The registry, read-only. MappingProxyType and not a plain dict: a caller able
#: to do TIMEOUTS["T-01"] = ... would have redefined a timeout for every importer
#: sharing the interpreter, with no import failing. Iterates in file order (A, B,
#: C). !! Never write its size into code or a comment (CLAUDE.md 3.7).
TIMEOUTS: Mapping[str, Timeout] = MappingProxyType(_TIMEOUTS)

#: The closed set of ids on its own, for membership tests that do not need the
#: rows. frozenset so it cannot be added to at run time -- inventing a timeout id
#: is exactly what the metatest against S1.6 forbids.
TIMEOUT_IDS: FrozenSet[str] = frozenset(_TIMEOUTS)


def timeout(tid: str) -> Timeout:
    """The row for one id. Raises KeyError, with the valid ids, outside the set.

    A miss is a caller bug, not a reason to hand back a plausible default: the
    registry is a closed set like the error codes, and a "nearest" timeout is the
    degrade-to-something-close path the contract forbids for closed sets. The
    message lists the set so the caller sees what it should have asked for.
    """
    try:
        return _TIMEOUTS[tid]
    except KeyError:
        # "from None" suppresses the chained KeyError, which would just repeat the
        # id and bury the part that helps -- the set the caller could have chosen
        # from.
        raise KeyError(f"{tid!r} is not a S1.6 timeout id; valid ids: "
                       f"{sorted(_TIMEOUTS)}") from None


__all__ = ["Timeout", "TIMEOUTS", "TIMEOUT_IDS", "timeout",
           "GROUP_SAFETY", "GROUP_LINK", "GROUP_NONSAFETY",
           "TIMEBASE_MONOTONIC", "TIMEBASE_WALL"]
