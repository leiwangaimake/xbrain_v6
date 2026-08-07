"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The non-error closed sets, exported for every XBRAIN runtime process

Description:
The non-error closed sets of the contract live here: plane, domain,
event_category, gate_limiter, stop_reason, task_state, cls, device,
release_reason, arb_suspended, gate_reason, suspend_kind, suspend_reason,
charge_stage. CLAUDE.md 3.5 forbids their literals outside this package for the
same reason as the error codes -- a value spelled differently in two processes
compiles, runs, and only surfaces during integration.

The list above is a roll call and not a tally, deliberately (CLAUDE.md 3.7). Ask
SET_NAMES at run time for the authoritative membership; a number written here
would have gone stale the day cls and device were added, and again the day the
arbitration and task-lifecycle sets arrived.

One thing here is NOT a closed set: LIMITER_CN, the gate.limiter -> Chinese
broadcast wording (CFG-CM-6). It lives with GATE_LIMITER on purpose -- 16 S8.3B.2
requires the map to sit beside the closed set it mirrors -- and it is guarded by
its own startup assertion, assert_limiter_cn_matches_gate_limiter (RS-LIM), which
refuses to start unless the map's keys and the gate.limiter set are equal both
ways. It is a mapping, not a membership set, so it is reached by key rather than
through get()/SET_NAMES; the long comment beside its definition explains why the
values are authored here rather than generated like sets.yaml.

Most of these come from 11, but not all of them: charge_stage is defined in 15
S8.5, because 11 states that its own charging section 定接口 while 15 S8.5 定判据
与阈值 and that 两处不得各写一份阈值 (CFG-40). Each set records the volume it came
from in .source, and the metatest reads that volume rather than assuming 11 --
which matters, because 11's charging table lists only the stages an event can
announce and does not carry to_handover at all.

*** Out-of-set values raise ClosedSetViolation. 11 S13.6 requires it in so many
words: no silent pass-through, and no "interpret the unknown value as something
close". The contract even names the tempting version it forbids -- degrading an
unrecognised PTZ action to jog -- because that turns a contract violation into
motion the operator did not ask for.

* Two of the sets are ORDERED, not just membership: gate_limiter's order is the
attribution priority (estop wins over everything) and stop_reason's order is the
decision order of 11 S9.12.2. Losing the order silently changes which cause gets
reported, so the metatest checks sequence for those two, not just membership.


Why the values are not typed into this file
-------------------------------------------
They live in sets.yaml, generated from 11 and compared back against 11 by
tests/common/test_closed_sets.py. Were the values literals in this module, the
metatest would be comparing the contract against one person's transcription of
the contract, and a shared typo would read as agreement. Keeping the data in a
separate file also keeps a contract change reviewable: one yaml block moves, no
Python is touched, and the diff shows the set changing rather than logic
changing around it.

Why the sets are built at import time
-------------------------------------
_load() runs on import, so a malformed sets.yaml kills the process during
startup instead of at the first boundary decode. That placement is deliberate.
The consumer it matters most for is p1_motion, where CLAUDE.md 4.4 requires a
raise inside the 20 Hz loop to be absorbed as a zero-speed tick plus a fault --
so a broken enum file first touched from there would present as a robot that
will not move, with a per-tick fault, rather than as a process that refused to
start. Failing at import puts the failure where a human is still watching it.

What this module deliberately does NOT do
-----------------------------------------
  * It does not hold the E_* error codes. Those are xbrain/common/errors/, kept
    apart because an error code carries four elements (name, meaning,
    retryability, required detail keys) that none of these sets have. Do not
    merge the two loaders to save code: the shapes are not the same.
  * It does not know who owns a value. That the ptz and payload_light domains
    are arbitrated by p2_core is arbitration policy and lives in p2_core; this
    table only says which domain names exist.
  * It does not validate combinations. A key is assembled as
    xbrain/{robot_id}/{plane}/{domain}/{name}; parse() accepts "rt", accepts
    "ptz", and has no opinion on whether that pair names a legal key. Whole-key
    legality belongs to the transport layer, and a caller expecting this module
    to reject a nonsensical pair gets no signal that it did not.
  * It does not alias, normalise or case-fold. There is no legacy-to-current
    mapping table here and none may be added: an alias table is "degrade the
    unknown value to something close" written with extra steps, which is what
    11 S13.6 forbids by name.
  * It does not decide anything about a value it validated. Which limiter wins,
    which stop_reason is reported this tick -- those are p1_motion's, and they
    only borrow the ORDER defined here.

A note on the docstrings below
------------------------------
They are one-liners, and the reasoning sits in comment blocks above each
definition instead. That is not a style preference: the reasoning is about why
the code is shaped this way, which is maintainer material rather than something
a caller reading help() needs.
"""

import os
from types import MappingProxyType
from typing import Dict, FrozenSet, Iterable, List, Mapping, Tuple

# ClosedSetViolation is defined with the error hierarchy rather than here, so a
# caller catches one exception type for the whole common/ family. It is
# re-exported in __all__ for the same reason: a module that validates values
# should not force its callers to import a second package to catch the failure.
from ..errors.exceptions import ClosedSetViolation

# E_CONFIG_INVALID and XbrainError come from the errors PACKAGE, not from
# .exceptions, because the code constants are bound in errors/__init__ (from
# codes.yaml) and not in the exceptions module. Importing them adds no cost the
# line above did not already pay: "from ..errors.exceptions import ..." already
# runs errors/__init__ to reach the submodule, so the code table is loaded by
# the time this line executes. The constant is imported rather than the literal
# "E_CONFIG_INVALID" typed here because CLAUDE.md 3.5 forbids E_* literals
# outside common/errors/ and scripts/lint/no_literal_ecode.py enforces it -- the
# same rule every module in common/config/ follows for this exact code.
from ..errors import E_CONFIG_INVALID, XbrainError

# Path derived from __file__, not from the configuration root. These values are
# the contract itself, not tunable configuration: they must not be reachable
# under /opt/xbrain_v6/configs/ where an operator could edit them, and they must
# not travel through the resolved-artifact path in /run/xbrain/resolved/ that
# CLAUDE.md 3.6 defines for configuration. A closed set an operator can widen is
# not a closed set.
#
# abspath wraps __file__ before dirname is taken, not the joined result. Python
# has made __file__ absolute since 3.9, so on the supported interpreters this is
# belt and braces; it stays because the failure it covers -- a relative __file__
# resolving against whatever cwd the unit was started in -- is a file-not-found
# at import in every process at once, and the guard costs one call.
_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sets.yaml")


# ClosedSet -- one set plus the contract location it came from.
#
# source, anchor and note travel with the values on purpose. When a value fails
# validation at run time the first question is what the contract actually says,
# and 11 runs to tens of thousands of lines; carrying the section and a grep-able
# anchor turns that question into a single search.
#
# * anchor is a literal string to grep for, never a line number. CLAUDE.md 1.1
# NUM-4 forbids line-number citations because documents shift underneath them: in
# one review round all three line references had drifted onto a blank line or an
# unrelated table, while the citing side believed it had checked them.
#
# * The instances built at the bottom of this module are process-wide singletons
# living for the life of the process. __slots__ is what makes a mistyped
# attribute assignment raise AttributeError instead of quietly creating a new
# attribute on a shared object that no reader ever consults.
class ClosedSet:
    """One closed set, with the contract location it was generated from."""

    __slots__ = ("name", "source", "anchor", "note", "ordered", "_values", "_frozen")

    # Every field is injected; nothing is defaulted in the signature. That is
    # CLAUDE.md 3.1 applied beyond the safety parameters it names: a default here
    # would let a set construct itself with an empty value list and an ordered
    # flag nobody set, and the resulting object looks valid from the outside.
    def __init__(self, name: str, source: str, anchor: str, note: str,
                 ordered: bool, values: List[str]):
        self.name = name
        self.source = source
        self.anchor = anchor
        self.note = note
        # Whether an order exists is a property of the SET, decided by the shape
        # of the contract table, so it is recorded once here and consulted by
        # index(). The alternative -- each call site deciding for itself whether
        # the order it is reading means anything -- is how an unordered set
        # acquires an accidental priority.
        self.ordered = ordered
        # Two representations, both immutable, both needed. The tuple preserves
        # contract order, which IS the priority for the ordered sets; the
        # frozenset gives parse() an O(1) membership test, and parse() sits on
        # every boundary decode including the 20 Hz path. Deriving one from the
        # other at call time would trade that constant-time check away for
        # nothing.
        self._values = tuple(values)
        self._frozen = frozenset(values)

    # A tuple, and a property with no setter: every importer inside the process
    # shares this one object, so returning something mutable would let any single
    # module reorder the attribution priority for all the others.
    #
    # For an unordered set the sequence is merely the order the contract table
    # happened to print in, and reading meaning into it is a mistake. That is
    # precisely what index() refuses to assist with.
    @property
    def values(self) -> Tuple[str, ...]:
        """In contract order. Meaningful when .ordered is True."""
        return self._values

    def __contains__(self, v: object) -> bool:
        # Membership as a question, for code entitled to ask it: metatests, diffs
        # against the contract, tooling. NOT for boundary decoding. The shape to
        # avoid is "if v in SET: use(v)" with no else branch, which is silent
        # pass-through of the out-of-set case under a different spelling. At a
        # boundary use parse(), so the violation raises instead of vanishing.
        return v in self._frozen

    def __iter__(self):
        # Contract order, never sorted(). Sorting gate_limiter would place brake
        # ahead of estop, and a priority table built from the sorted sequence
        # holds exactly the same strings as the correct one -- so the defect is
        # invisible to any comparison that checks membership alone. That is why
        # the metatest asserts sequence for the two ordered sets and not just
        # set equality.
        return iter(self._values)

    def __len__(self) -> int:
        # Provided so a caller can size a table at run time. The result must
        # never be copied into a comment or a document (CLAUDE.md 3.7): counts of
        # exactly this kind have already gone stale inside the contract itself.
        return len(self._values)

    # parse -- the boundary validator. Use it wherever a value arrives from
    # outside this process.
    #
    # !! Never add a fallback here. 11 S13.6 forbids both silent pass-through and
    # degrading to a nearby value; either one hides a contract violation behind
    # something that looks like normal operation.
    #
    # This is a validator, not a normaliser. It does not strip whitespace, fold
    # case or repair separators, and it must not learn to: the wire spellings in
    # 11 are exact, and a tolerant parser lets two spellings of one value coexist
    # across processes right up until the day they have to match each other.
    #
    # It returns the input unchanged so it can be used inline at the decode site
    # -- plane = PLANE.parse(msg["plane"]) -- which welds the check to the
    # assignment instead of leaving a separate check that a later edit can drop
    # without the assignment noticing.
    def parse(self, v: str) -> str:
        """Return v if it is in the set, else raise ClosedSetViolation."""
        if v not in self._frozen:
            raise ClosedSetViolation(self.name, v)
        return v

    # index -- position in contract order, which for gate_limiter IS the
    # attribution priority.
    #
    # It raises on an unordered set rather than returning a meaningless number:
    # a caller asking for the priority of a plane has misunderstood something,
    # and a silent answer would let that misunderstanding propagate into whatever
    # comparison the caller was about to make.
    #
    # Lower is stronger. gate_limiter holds estop at position zero and the none
    # sentinel last, so "nothing limited this tick" can never outrank a real
    # limiter in attribution. stop_reason is the mirror image: none leads because
    # it means not stopped, and the rest follow the decision order of 11 S9.12.2.
    #
    # Indices compare only within one set. A gate_limiter index measured against
    # a stop_reason index is meaningless, and nothing here can catch it, because
    # both are plain ints.
    #
    # parse() runs first so an unknown value raises ClosedSetViolation instead of
    # the bare ValueError tuple.index would produce. An off-contract value has to
    # report as the same type here as it does at every other boundary; a
    # ValueError would slip past the handler written for the closed-set case.
    #
    # ValueError, not ClosedSetViolation, for the unordered case: the value is
    # fine, the question is wrong. Reporting it as a closed-set violation would
    # send whoever reads the log looking for a bad value that does not exist.
    def index(self, v: str) -> int:
        """Position in contract order. Raises unless the set is ordered."""
        if not self.ordered:
            raise ValueError(f"closed set {self.name!r} carries no meaningful order")
        return self._values.index(self.parse(v))


# _load -- parse sets.yaml without a yaml dependency.
#
# This package is imported by every runtime process, including ones that start
# before any virtualenv is guaranteed, so it deliberately has no third-party
# imports. It raises on any line it does not recognise rather than skipping it: a
# loader that skips silently shrinks the closed set, which is the exact failure
# this module exists to prevent. That is not hypothetical -- the generator for
# this file first dropped nine of task_state's twelve values because its row
# filter treated an empty leading cell as a separator row.
#
# Strictness is the second reason for hand-parsing, and it outlives the first: a
# real yaml loader accepts input this file must not tolerate. A duplicated set
# name is the clearest case -- YAML resolves it by keeping the last one, so one
# whole set would be replaced without a word.
#
# The grammar accepted is exactly the shape sets.yaml is written in and no more:
# two-space indent for a set name, four for that set's scalar fields, six for a
# list item under values:. Indentation carries the entire parser state, which is
# why anything at an unexpected indent is an error rather than something to guess
# about. Widening the grammar to be helpful re-opens the skip-silently hole from
# the other side.
def _load() -> Dict[str, ClosedSet]:
    """Parse sets.yaml into ClosedSet objects. Raises on anything unrecognised."""
    out: Dict[str, ClosedSet] = {}
    # cur accumulates the set currently being read. It is flushed when the NEXT
    # set name appears, and once more after the loop ends. A streaming parser has
    # to be told twice, and omitting the second call is how the last set in a
    # file goes missing -- silently, because every other set still parses.
    cur: Dict[str, object] = {}
    # Everything above the top-level sets: key is file header and metadata.
    in_sets = False
    # Raised by a values: line and cleared by the next scalar field, so a list
    # item can never be attached to whichever set was read last if the file is
    # rearranged.
    reading_values = False

    def flush():
        # Guarded on cur so the first set name in the file does not flush an
        # empty accumulator. Absent scalars fall back to empty strings instead of
        # raising, and that is safe here only because source/anchor/note are
        # documentation: a set that loses its citation should still load. The
        # VALUES are the part that must never be defaulted, and structurally they
        # cannot be -- an empty list makes every parse() of that set raise, which
        # is loud rather than fail-silent.
        if cur:
            out[cur["name"]] = ClosedSet(
                str(cur["name"]), str(cur.get("source", "")), str(cur.get("anchor", "")),
                str(cur.get("note", "")), bool(cur.get("ordered")),
                list(cur.get("values", [])),
            )

    with open(_YAML, encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            # A whole-line comment is replaced by an empty string rather than
            # stripped in place, because a comment may contain anything at all --
            # including text that would read as a set name at two-space indent.
            line = "" if raw.lstrip().startswith("#") else raw.rstrip()
            if not line.strip():
                continue
            if line.rstrip() == "sets:":
                in_sets = True
                continue
            if not in_sets:
                # Header lines ahead of sets: are skipped, not validated. They are
                # the five-field file header required by CLAUDE.md 2.5, i.e. prose.
                continue
            indent = len(line) - len(line.lstrip())
            body = line.strip()
            if indent == 2 and body.endswith(":"):
                # A new set name terminates the previous one.
                flush()
                cur = {"name": body[:-1], "values": []}
                reading_values = False
                continue
            if indent == 4:
                if body == "values:":
                    reading_values = True
                    continue
                reading_values = False
                if ":" not in body:
                    raise ValueError(f"{_YAML}:{lineno}: unparsable line {body!r}")
                k, v = body.split(":", 1)
                # split(":", 1), not split(":"): anchor and note carry contract
                # prose, where a colon is legal content and only the first one is
                # the separator. The trailing-comment strip happens here rather
                # than at the top of the loop because only a scalar line can carry
                # one, and doing it globally would corrupt any value containing a
                # hash.
                v = v.split("#", 1)[0].strip().strip('"')
                # ordered is compared against the literal "true" and NOT passed to
                # bool(). bool("false") is True, so a bool() conversion would mark
                # every set ordered -- including the ones whose sequence is only a
                # table's typography -- and index() would then hand out priorities
                # that mean nothing while looking perfectly plausible. flush()
                # does call bool() on this, which is safe only because the string
                # was already turned into a real bool right here; do not drop this
                # ternary on the grounds that flush() converts anyway.
                cur[k.strip()] = (v == "true") if k.strip() == "ordered" else v
                continue
            if indent == 6 and reading_values and body.startswith("- "):
                # A list item. reading_values is part of the condition so that an
                # item appearing outside a values: block falls through to the
                # raise below instead of being appended to the wrong field.
                cur["values"].append(body[2:].strip())  # type: ignore[union-attr]
                continue
            # Everything else is an error, deliberately. This is the branch the
            # first generator lacked: it fell through and continued, which is how
            # nine task_state values disappeared while the file still loaded and
            # every test still passed.
            raise ValueError(f"{_YAML}:{lineno}: unexpected line {line!r}")
    # The second flush, for the last set in the file. See the note on cur above.
    flush()
    # An empty parse is a failure, not an empty library. Without this the module
    # would import cleanly and every lookup below would raise KeyError at first
    # use, a long way from the cause. It also blocks the degenerate case an empty
    # sets.yaml would create: a package that exports nothing and therefore never
    # rejects anything, because nothing ever calls it.
    if not out:
        raise ValueError(f"{_YAML}: no sets parsed")
    return out


# Built at import. See "Why the sets are built at import time" in the module
# docstring: a broken sets.yaml has to stop the process at startup rather than
# surface later as a fault inside a control tick.
_SETS: Dict[str, ClosedSet] = _load()

#: Names of every closed set defined here. !! Never write the count (3.7).
# The metatest asserts this equals its own extractor table, so a set added to
# sets.yaml without an extractor cannot slip in uncompared against 11 -- an
# assertion that covers less than it appears to is CLAUDE.md 3.2 form 1.
SET_NAMES: FrozenSet[str] = frozenset(_SETS)

# Subscript, not .get(): a renamed or missing set must break the import of every
# process immediately, with the offending key in the traceback. .get() would hand
# back None and defer the failure to the first decode, by which time the stack no
# longer mentions this file.
#
# plane -- membership only. NOTE that plane is not transport: the two physically
#   separate Zenoh routers are a different axis, and nothing in this set says
#   which router a given plane travels on.
PLANE = _SETS["plane"]
# domain -- the arbitration domains. Membership only; which process
#   arbitrates which domain is p2_core's concern, not this table's.
DOMAIN = _SETS["domain"]
# event_category -- grew on 2026-08-05 when ptz and voice were added. That growth
#   is the change a containment-only check stayed green through, which is why the
#   metatest now compares both directions of the difference.
EVENT_CATEGORY = _SETS["event_category"]
# severity -- the {severity} segment of event/{severity}/{category} (11 S2.2.11
#   W-5) and Event.sev (S6.1). The arbiter audit stream stamps info/warn/fault
#   from it (S7A.7); alarm is used by other categories. Kept a closed set so an
#   off-contract sev is E_SCHEMA, not a key the cloud alarm subscription silently
#   never matches (V-2).
SEVERITY = _SETS["severity"]
# reason / control_mode -- CFG-CM-16 (11 S R2.6-e / R2.6-f). reason is the
#   task/progress motion/teleop block reason (rotation_blocked,
#   lateral_clearance_unavailable, teleop_stale, deadman_released) -- our set, not
#   the chassis's and not suspend_reason. control_mode is the direct-control mode,
#   currently the single value jog (point-control); an off-set value must raise,
#   never be degraded to jog.
REASON = _SETS["reason"]
CONTROL_MODE = _SETS["control_mode"]
# gate_limiter -- ORDERED. The sequence is the speed-gate attribution priority:
#   estop first, the none sentinel last. Read it with .index(); never rebuild it
#   by iterating anything that has been sorted.
GATE_LIMITER = _SETS["gate_limiter"]

# ---------------------------------------------------------------------------
# limiter_cn -- the gate.limiter -> Chinese broadcast wording, plus its startup
# assertion RS-LIM. CFG-CM-6. It sits HERE, hard against GATE_LIMITER, by
# instruction: 16 S8.3B.2 (grep anchor "加值时中文就在旁边") requires the map to
# live in the shared library right next to the closed set, so a maintainer
# adding a limiter value sees the empty Chinese slot in the same glance. RS-LIM
# below is what turns "should not leave it empty" into "will not start if empty".
#
# Why this map is hand-written here and NOT generated from a doc table the way
# sets.yaml is. The closed set (the LEFT operand of RS-LIM) is contract-defined
# in 11 S3.4; the Chinese (the RIGHT operand) is authored broadcast wording that
# 11 refuses to define at all -- 11 S3.4 states in so many words that it fixes
# the value domain of gate.limiter and not how each value reads aloud, leaving
# that to 16 (the P4 broadcast pipeline). The two are therefore written by
# different people at different times, which is exactly what makes a
# symmetric-difference check between them worth running: 16 S8.3B.3 (grep anchor
# "为什么这条判据非自证") calls it non-self-certifying for that reason -- neither
# side can drift without the other going out of step.
#
# Sources, one authority per value, greppable and with no line numbers (NUM-4):
#   * the twelve non-heading/clock values: 16 S8.3, the inline map on the row
#     whose cell reads "现有 12 条" (free_space through none).
#   * heading and clock: 99 U76(6) (grep anchor "heading = 航向未就绪"), the
#     ruling that resolved the gap 16 S8.3B.3 flagged as still red (grep anchor
#     "本断言今天就是红的"). Before U76(6) these two values had no authored
#     wording and RS-LIM was designed to stay red on purpose until they did.
# tests/common/test_limiter_cn.py binds every value back to those anchors, the
# same way test_closed_sets.py binds sets.yaml back to 11: a hand transcription
# that nothing checks against its source is the one defect this package exists
# to stop, and the RS-LIM key check alone does not cover the VALUES.
#
# NOT hot-updatable (16 S8.3B.2, grep anchor "改它要发版"): it ships inside the
# shared library and never travels through configs/ or /run/xbrain/resolved/.
# The restate templates keep their {limiter_cn} placeholder and fill it FROM
# here; a second copy of this map anywhere else is forbidden by the same section.
#
# What this map deliberately does NOT do: it does not carry the out-of-set
# fail-safe. 16 S13 LIMITER-1 requires an off-contract raw limiter to broadcast a
# generic "current speed cap" line and log event/warn/motion carrying the raw
# value, WITHOUT reading a reason and WITHOUT blocking -- and that branch is the
# consumer's (P4), taken BEFORE any lookup here, exactly as arb_suspended's null
# is branched before parse(). After RS-LIM passes, every GATE_LIMITER member is a
# key, so LIMITER_CN[v] on an already-validated limiter cannot miss; a miss means
# the caller skipped the LIMITER-1 branch -- a caller bug, never a value to
# invent on this side.
#
# The dict is ordered to mirror the GATE_LIMITER attribution priority. The order
# carries no meaning for a by-key lookup, but keeping it in lockstep with the
# closed set is what makes "the Chinese is right beside it" literally true for a
# human reading the two together. Inline comments cite only the section, never
# the Chinese word: repeating the value in an English comment would both mix
# languages (CLAUDE.md 2.1) and create a second copy for the value-binding test
# to disagree with.
_LIMITER_CN_RAW = {
    "estop": "急停",           # 11 S3.4 priority 1 ; 16 S8.3
    "mode": "模式限制",         # 11 NAV-111 B/D-mode cap ; 16 S8.3
    "health": "传感器降级",      # 16 S8.3
    "rtk": "定位质量",          # 16 S8.3
    "heading": "航向未就绪",     # 99 U76(6) ; heading-quality degrade, 0.5 m/s cap
    "clock": "时钟未同步",       # 99 U76(6) ; clock-unsync, 0.5 m/s cap
    "fence": "电子围栏",         # 16 S8.3
    "free_space": "前方障碍",    # 16 S8.3
    "target": "附近有人",        # 16 S8.3
    "brake": "加减速限制",       # 16 S8.3
    "gait": "当前步态",          # 16 S8.3
    "profile": "档位上限",       # 16 S8.3
    "spec": "底盘上限",          # 16 S8.3
    "none": "无限制",           # 16 S8.3 ; the not-limited sentinel
}

# MappingProxyType, not the bare dict: every importer in the process shares this
# one object, so a writable map would let one module rewrite a broadcast reason
# for all the others -- the same guarantee ClosedSet.values gives with a tuple.
LIMITER_CN: Mapping[str, str] = MappingProxyType(_LIMITER_CN_RAW)


# assert_limiter_cn_matches_gate_limiter -- the RS-LIM startup assertion.
#
# 16 S8.3B.3, verbatim: the limiter_cn key set and the 11 S3.4 gate.limiter
# closed set must have an EMPTY symmetric difference; unequal means refuse to
# start, with E_CONFIG_INVALID.
#
# !! Both directions, never one-way containment. 16 S8.3B.3 (grep anchor "绝不可
# 改成") forbids "the map's keys are all in the closed set", because that stays
# green while a closed-set value has no Chinese -- which was the live state for
# heading and clock before U76(6). The two operands below are subtracted each
# way and either non-empty difference raises.
#
# Operands are passed in, not read from the module globals, and nothing is
# defaulted -- CLAUDE.md 3.1 applied past the safety params it names, the same
# stance ClosedSet.__init__ takes. Injected operands are also what lets the
# mutation tests feed a doctored closed set or a doctored map without reloading
# the module or touching sets.yaml; the import-time call below passes the real
# pair.
#
# E_CONFIG_INVALID is the code the whole config-refusal family carries (group L,
# retryable no); RS-LIM is one more member of it, so it raises with the same code
# a bad reference or a failed schema check does, and a caller branching on the
# code treats them alike.
def assert_limiter_cn_matches_gate_limiter(cn_keys: Iterable[str],
                                           limiter_values: Iterable[str]) -> None:
    """RS-LIM: raise E_CONFIG_INVALID unless the two sets are equal both ways."""
    cn = frozenset(cn_keys)
    limiter = frozenset(limiter_values)
    # missing_cn: a gate.limiter value the map has no Chinese for. This is the
    # direction a one-way "keys subset of closed set" check cannot see, and the
    # one 16 S8.3B.3 was written to catch.
    missing_cn = sorted(limiter - cn)
    # extra_cn: a Chinese key for something that is not a gate.limiter value --
    # a typo, or wording left behind after a value was removed from the set.
    extra_cn = sorted(cn - limiter)
    if missing_cn or extra_cn:
        # Both differences are named so the failure says which side is short and
        # by what -- a bare "RS-LIM failed" would send the reader to diff two
        # lists by hand. No count is printed (CLAUDE.md 3.7); the elements are.
        raise XbrainError(
            E_CONFIG_INVALID,
            "RS-LIM: limiter_cn keys and the gate.limiter closed set differ. "
            f"closed-set values with no Chinese: {missing_cn}; "
            f"Chinese keys not in the closed set: {extra_cn}. "
            "See 16 S8.3B.3; add the wording or amend 11 S3.4, do not relax this",
        )


# Enforced at IMPORT, so a mismatch stops the process at Stage 0 while a human is
# watching, exactly like a malformed sets.yaml does above (see the module
# docstring on why the sets are built at import). The alternative -- a lazy check
# on first broadcast -- would surface the failure the first time the robot tries
# to say why it slowed down, which is both later and further from the cause.
assert_limiter_cn_matches_gate_limiter(LIMITER_CN.keys(), GATE_LIMITER.values)

# stop_reason -- ORDERED. The sequence is the decision order of 11 S9.12.2, with
#   none leading as the not-stopped sentinel. Exactly one reason is reported per
#   tick, so this order is what decides which one that is.
STOP_REASON = _SETS["stop_reason"]
# task_state -- membership only, and deliberately so. Task state is a graph, not
#   a line, and the contract's grouping is presentation. index() refuses it,
#   which is the correct answer to "which state is further along".
TASK_STATE = _SETS["task_state"]
# cls -- the target class names carried by targets[].cls on state/targets.
#   Membership only. The contract prints the values in one line separated by
#   middle dots and says nothing about order, so there is nothing here to rank.
#
#   NOTE that unknown is a member, and it means "detected but not classifiable".
#   It is a value perception is entitled to send, not a hole to fill in on the
#   consumer side: a decoder that maps an unrecognised class onto unknown has
#   written the degrade-to-something-close path 11 S13.6 forbids, and it would
#   make a contract violation indistinguishable from an honest low-confidence
#   detection. Off-contract classes must raise.
#
#   It is also NOT the customer-facing rule vocabulary. Whether an intrusion
#   rule written as "vehicle" covers bicycle and motorcycle too is an open
#   question the contract logs against S3.1.1 rather than answers, so no
#   grouping of these values may be built here.
CLS = _SETS["cls"]
# device -- the self-test items whose kind column reads device, i.e. the health
#   items backed by a physical box rather than by a capability.
#
#   !! Two neighbouring sets this one is easy to confuse with, and neither may
#   be served from here:
#     * the S0.3 global device ID table, which is what CS-1 constrains the
#       frames.* keys to. It registers IDs that are not self-test rows at all
#       (cam_ptz, imu_chassis, gnss_chassis, payload_svc among them), so using
#       this set for CS-1 would reject valid extrinsics.
#     * the full S5.1A item set, device plus cap. Membership here says how an
#       item fails, not whether it exists: heading and clock are items, they are
#       kind cap, and they are correctly absent below.
#
#   Membership only, and it carries no level. Which items are fatal is the level
#   column of the same table and it is deliberately not mirrored here -- the BIT
#   exemption guard reads level from the contract table, and a second copy in
#   this file would be the thing that silently disagrees with it.
DEVICE = _SETS["device"]

# ---------------------------------------------------------------------------
# The arbitration and task-lifecycle sets. All membership only: none of the six
# contract sites states an order, so index() refuses every one of them.
# ---------------------------------------------------------------------------

# release_reason -- why a holder gave a domain back voluntarily.
#
#   !! mode_switch is NOT a member today, and that is the contract's current
#   state rather than an omission here. 14 S13 CR-1 asks 11 to add it (11's own
#   rt/audio/lease.reason already lists it, which is the inconsistency CR-1
#   names), and 14 S3.2.1 gives the transitional instruction to follow until it
#   lands: emit mode_exit and mark detail.note self_held_handover. Do that
#   rather than adding the value here -- a value present in the library and
#   absent from the contract is the direction the metatest calls "closed sets
#   are not extended in code".
RELEASE_REASON = _SETS["release_reason"]
# arb_suspended -- why an arbitration domain is disarmed. ArbDomainState's field
#   is spelled suspended; it is qualified here because task_state exports a
#   VALUE spelled suspended and the two are unrelated.
#
#   !! null is not a member. The field is string|null and null carries the
#   normal case, so a decoder must branch on the null BEFORE reaching parse().
#   Feeding it here raises, which is correct: absence is not a value. The
#   distinction is load-bearing at 11 G-10, where a motion domain disarmed with
#   soft_estop still forwards commands rather than answering E_ARB_DISARMED.
ARB_SUSPENDED = _SETS["arb_suspended"]
# gate_reason -- why the microphone is in its current state, on rt/audio/gate.
#   P2 is the sole authority: the contract forbids p4_agent from deriving it, on
#   the grounds that P4 cannot know how much audio the speaker still holds.
#   NOTE that open is a member, so this is not a shut-only reason code.
GATE_REASON = _SETS["gate_reason"]
# suspend_kind -- passive or yielding, required exactly when a task is
#   suspended. The two may not be collapsed: yielding resumes by itself when the
#   preemptor finishes, passive waits for a condition that may still not hold.
SUSPEND_KIND = _SETS["suspend_kind"]
# suspend_reason -- required alongside suspend_kind. The contract table has
#   fewer rows than this set has values, because two rows carry a pair of
#   reasons sharing one kind; the mapping from reason to kind is the task
#   machine's, not this table's, so it is deliberately not mirrored here.
SUSPEND_REASON = _SETS["suspend_reason"]
# charge_stage -- from 15 S8.5, not from 11. Membership only despite reading
#   like a sequence: an unplug confirmation sends charging BACK to waiting_plug,
#   so a rank derived from position would run backwards.
CHARGE_STAGE = _SETS["charge_stage"]

# ClosedSetViolation is re-exported so a caller catches the failure without also
# importing common.errors. get and parse_enum are exported for the by-name path
# used by generic decoders that read the set name out of a schema instead of
# knowing it when they are written. _SETS and _load stay unexported: the sets are
# reached through the constants or through get(), so there is one place that
# turns an unknown name into a raise.
#
# LIMITER_CN and assert_limiter_cn_matches_gate_limiter are exported alongside
# GATE_LIMITER: the map is the P4 broadcast source (CFG-CM-6) and the assertion
# is the RS-LIM startup check, callable by the boot sequence and the tests as
# well as running once at import here. _LIMITER_CN_RAW stays unexported so the
# only reachable handle is the read-only MappingProxyType.
__all__ = ["ClosedSet", "ClosedSetViolation", "SET_NAMES", "get", "parse_enum",
           "PLANE", "DOMAIN", "EVENT_CATEGORY", "SEVERITY", "GATE_LIMITER",
           "REASON", "CONTROL_MODE",
           "STOP_REASON",
           "LIMITER_CN", "assert_limiter_cn_matches_gate_limiter",
           "TASK_STATE", "CLS", "DEVICE",
           "RELEASE_REASON", "ARB_SUSPENDED", "GATE_REASON", "SUSPEND_KIND",
           "SUSPEND_REASON", "CHARGE_STAGE"]


# get -- look a set up by name, raising on an unknown NAME and not only on an
# unknown value.
#
# The sentinel set name "__set_name__" in the raised violation is how a reader
# tells the two failures apart in a log. A mistyped SET name and an off-contract
# VALUE would otherwise read identically, and their fixes differ: one is a bug in
# our decoder, the other is a peer sending something the contract does not define.
#
# KeyError is converted rather than propagated so a bad set NAME leaves this
# package as the same type a bad VALUE does; index()'s ValueError is the one
# deliberate exception, and it means something else. `from None` suppresses the
# KeyError context because the chained traceback adds nothing here and pushes the
# message that does explain the problem off the top of the report.
def get(name: str) -> ClosedSet:
    """The set by name. Raises on an unknown set name, not just an unknown value."""
    try:
        return _SETS[name]
    except KeyError:
        raise ClosedSetViolation("__set_name__", name) from None


# parse_enum -- the by-name entry point, for decoders that learn which set applies
# from a schema rather than from the call site.
#
# Prefer the module-level constants wherever the set is known at write time:
# PLANE.parse(v) is checked by the reader and by mypy, parse_enum("plane", v) only
# at run time, so a typo in the set name surfaces from whichever code path happens
# to run -- possibly a rare one, possibly on the robot.
def parse_enum(set_name: str, value: str) -> str:
    """Validate value against the named set. Raises ClosedSetViolation otherwise."""
    return get(set_name).parse(value)
