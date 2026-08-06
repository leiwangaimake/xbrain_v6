"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: refs.py
Brief: The ${common.*} reference axis, R-1 through R-7 of 10 S5.4.2

Description:
CFG-CM-8. Runs AFTER the whole L0~L5 overlay finishes -- 10 S5.4.3 requires that
order, and the reason is that expanding while merging reads a value the site
layer has not overridden yet. That failure is invisible whenever the lab site and
the field site happen to agree, so it passes every test written in the lab and
goes wrong on exactly one robot.

The seven rules, each with the contract's own reason:

  R-1  a reference occupies the WHOLE scalar node; no string interpolation.
       Interpolation silently degrades an object or int into a string, and the
       schema check downstream then sees the wrong type.
  R-2  paths start with common. only. Cross-process references would turn YAML
       into an implicit IPC channel that breaks when startup order changes.
  R-3  ** NO default syntax -- no ${a:-b}, no ?. Unresolvable is
       E_CONFIG_INVALID and the stack refuses to start. "Fall back to a default
       when the reference misses" is precisely the silent drift CFG-40 exists to
       kill; making it a syntax feature would institutionalise it.
  R-4  aliases may nest inside common.*, chain length <= 3, and * NO expressions
       or arithmetic of any kind. A calculator in the loader hides safety logic
       inside a configuration file.
  R-5  list overrides replace the whole table (implemented in merge.py; this
       module asserts the two stay consistent).
  R-6  ** L6 must not carry a `common` top-level key, nor a private key listed in
       the S5.4.5 alias table. The contract calls this the only rule of the seven
       with any teeth -- without it the first five are advice.
  R-7  * YAML anchors & / * may not be used to express sharing. Anchors do not
       cross files while looking like they do, and having two sharing mechanisms
       makes expansion order undefinable.

*** R-7 is checked against RAW FILE TEXT, not the parsed tree. Anchors are a YAML
syntax feature: by the time a parser hands back a dict they have already been
resolved and are invisible. A check written against the tree would pass on every
file that uses them.

----------------------------------------------------------------------------
WHAT THIS MODULE DOES NOT DO
----------------------------------------------------------------------------
Written down because each of these has a natural-looking home here and belongs
somewhere else. Implementing one of them here would give a single rule two
implementations that are free to disagree, and the disagreement surfaces on a
robot rather than in review.

  * It does not read files. It is handed an already-parsed tree (resolve,
    check_l6) or an already-read string (check_no_anchors). Nothing here knows
    where configs/ is; that is layers.resolve_config_root, ENV-1 and ENV-4.
  * It does not merge layers. deep_merge owns the overlay axis, and this module
    must run after it finishes, never inside it -- see the first paragraph.
  * It does not ENFORCE R-5. Whole-table list replacement is merge.py's rule.
    What this module owns is the promise not to quietly re-open it: resolve
    walks flattened leaves, so a list arrives as one opaque value and is never
    entered. test_r5_lists_pass_through_reference_axis_untouched pins that.
  * It does not type-check or range-check anything. A reference that expands to
    the wrong type, or to null, belongs to the schema check and to startup
    assertion A. Keeping expansion and validation as separate passes is not
    tidiness: a resolver that also judged values would need the schema, and the
    schema needs the already-resolved artifact.
  * It does not report cycles in full. CFG-CM-9 owns that. lookup() only refuses
    to loop forever, and says so at the point where it does it.

----------------------------------------------------------------------------
SCAN SURFACES, DECLARED
----------------------------------------------------------------------------
CLAUDE.md 3.2 lists "scan surface not declared" as one of the seven shapes in
which a check appears to give a guarantee it does not actually give. Both
text-level checks below are narrower than the rule they serve, so the narrowing
is recorded here instead of being discovered during commissioning:

  validate_shape / classify
      Surface is one scalar STRING at a time. A reference hidden in a key NAME,
      or assembled from two layers each supplying half of it, is invisible here
      and is not claimed to be caught.

  check_no_anchors
      Surface is VALUE POSITION only: a colon followed by &name or *name, the
      space between them optional. Anchors and aliases on LIST ITEMS ("- &name",
      "- *name") are NOT matched -- measured 2026-08-06. R-7 is broader than
      what this function checks, so a pass here does not license the claim
      "this file contains no anchors".
"""

import re
from typing import Any, Dict, Iterable, List, Optional, Tuple

from .layers import ConfigLayerError
from .merge import flatten

#: A whole-node reference: the entire scalar is ${common....} and nothing else.
#
# Anchored ^...$ on purpose: an unanchored pattern would accept
# "robot-${common.robot_id}", which is exactly the R-1 interpolation this rule
# exists to reject. The path charset is deliberately [A-Za-z0-9_.] with no
# hyphen, so no legal path can ever contain a character from the R-4 operator
# set -- that is what keeps the two rules from overlapping on real input.
# At least one dot is required by (?:\.[...]+)+, so a bare "${common}" is not a
# whole-node match; it is reported under R-2, see validate_shape.
_WHOLE_NODE = re.compile(r"^\$\{(common(?:\.[A-Za-z0-9_]+)+)\}$")

#: Any ${...} occurrence, used to tell "interpolated" apart from "not a reference".
#
# [^}]* cannot cross a closing brace, so nested forms such as ${a${b}} are not
# treated as one reference. That is acceptable because the outcome is still a
# rejection: whatever it matches will fail R-2 or R-1. The job of this pattern
# is only to answer "does this string want to be a reference at all" -- deciding
# whether it is a LEGAL one is _WHOLE_NODE's job, and conflating the two is how
# an illegal reference ends up classified as an ordinary string.
_ANY_REF = re.compile(r"\$\{([^}]*)\}")

#: R-4: chain length limit. A alias pointing at an alias pointing at an alias is
#: already hard to follow; four is a graph nobody reads.
#
# Counted including the ORIGIN key, because lookup() seeds `seen` with the key
# being resolved. Measured behaviour on 2026-08-06: with a=5, b=${a}, c=${b} the
# tree resolves; adding d=${c} raises R-4. So the effective allowance is two
# dereferences past the origin, not three. Read the limit off this note rather
# than off the constant name if you are changing it.
MAX_CHAIN = 3

#: R-7: anchors and aliases in raw YAML. Matched at the value position so a `&`
#: inside a quoted string is not mistaken for an anchor.
#
# The value-position requirement carries ONE of the two lines in
# test_r7_does_not_fire_on_ordinary_text: `b: "star * inside a string"` has a
# quote, not a `*`, directly after the colon. The other line, `# & in a comment`,
# is carried by the comment strip inside check_no_anchors instead. Neither
# mechanism keeps that test green on its own, so do not read either comment as
# saying the other one is redundant.
# A looser pattern would flag ordinary prose, and a checker that flags
# every file gets switched off -- which is the "permanently red assertion decays
# into permanently green" shape in CLAUDE.md 3.2.
# See the SCAN SURFACES block above for what this pattern does NOT reach.
_ANCHOR = re.compile(r":\s*&[A-Za-z0-9_-]+(\s|$)")
_ALIAS = re.compile(r":\s*\*[A-Za-z0-9_-]+(\s|$)")


class ReferenceError_(ConfigLayerError):
    """A reference violated one of R-1 ~ R-7. Carries code E_CONFIG_INVALID."""

    # The trailing underscore is not a typo and not a style tic: ReferenceError
    # is a Python BUILTIN. Taking that name would shadow it module-wide, and an
    # `except ReferenceError` written anywhere downstream would then silently
    # change meaning depending on the import order. Renaming this class is a
    # breaking change to that reasoning, not a cosmetic cleanup.


def _rule(n: str, msg: str) -> ReferenceError_:
    # Every message starts with the rule id so the reader can jump straight to
    # the right paragraph of 10 S5.4.2. Volume plus section only, never a line
    # number -- NUM-4: line numbers drift and the reader still believes them.
    #
    # NOTE for anyone parsing these messages: the XbrainError base prefixes the
    # closed-set code, so str(exc) is "E_CONFIG_INVALID: R-3: ...", NOT
    # "R-3: ...". find_violations at the bottom of this file gets that wrong;
    # the defect is documented there.
    return ReferenceError_(f"{n}: {msg}")


def validate_shape(value: str) -> None:
    """Validate one scalar against R-1 ~ R-4. Raises ReferenceError_ naming the rule.

    Precondition: the caller has already established that `value` contains a
    "${". A plain string reaches the R-1 branch and is rejected as "not a whole
    node", which is a confusing thing to say about text that was never a
    reference. classify() is the guarded entry point; prefer it.
    """
    # *** ONE validator, not two. An earlier draft checked R-3 in a second place
    # as well. A mutation test disabled that copy and the suite stayed green:
    # the branch was unreachable. An unreachable guard is worse than a missing
    # one -- it reads as protection, so the next maintainer budgets no effort
    # for the case it appears to cover, and nothing ever reports that the cover
    # is not there. CLAUDE.md 3.3: an assertion that has never been red has not
    # been written yet.
    #
    # *** ORDER IS LOAD-BEARING. Three separate constraints, all verified
    # 2026-08-06 by reading the rule id out of the raised message:
    #
    #   R-3 before R-4   "${common.x:-2.5}" contains "-", which is a member of
    #                    the R-4 operator set "+-*/%()". If R-4 ran first the
    #                    default-value syntax would be announced as arithmetic.
    #   loop before R-1  "${common.x:-2.5}" is not a whole-node match either, so
    #                    R-1 would fire and say "not a whole scalar node". That
    #                    is true, and it sends the reader to the R-1 paragraph
    #                    to debug something the R-3 paragraph explains. Naming
    #                    the wrong rule costs more than saying nothing.
    #   R-2 last of the three
    #                    "${2 * common.a}" fails R-2 as well, but "arithmetic is
    #                    not supported" is the diagnosis that matches intent.
    #
    # The loop runs over EVERY occurrence, so a string carrying several
    # references still gets the most specific rule. Only after none of them has
    # a diagnosable defect does R-1 decide whether the string is one reference
    # or a concatenation of them.
    for inner in _ANY_REF.findall(value):
        # R-3. Three spellings of the same idea, all borrowed from templating
        # languages where they are ordinary: ${a:-b}, ${a?} and ${a|b}. They are
        # rejected as SYNTAX rather than at lookup time, because a default that
        # is never taken in the lab is a default nobody notices is wrong.
        if ":-" in inner or inner.endswith("?") or "|" in inner:
            raise _rule("R-3", f"default-value syntax is not provided: {value!r}. "
                               "Unresolvable references are E_CONFIG_INVALID; falling "
                               "back to a default is the silent drift CFG-40 exists to kill")
        # R-4, expression half. Because _WHOLE_NODE's path charset admits none
        # of these characters, this branch can only fire on text that was never
        # going to resolve. Its whole value is giving that text the right name:
        # without it the reader is told "not a whole node" and starts looking
        # for a stray space.
        if any(op in inner for op in "+-*/%()"):
            raise _rule("R-4", f"expressions and arithmetic are not supported: {value!r}. "
                               "A calculator in the loader hides safety logic in a config file")
        head = inner.strip()
        # R-2. The `head and` guard lets an empty "${}" fall through to R-1
        # instead of being reported as a bad namespace -- there is no path in it
        # to have a namespace. Note "${common}" with no dot DOES land here and
        # is reported as R-2 (measured), because it fails the startswith test.
        if head and not head.startswith("common."):
            raise _rule("R-2", f"reference path must start with 'common.', got {value!r}; "
                               "cross-process references make YAML an implicit IPC channel "
                               "that breaks when startup order changes")
    # R-1 last, as the residual: everything diagnosable has already raised, so
    # reaching here and failing means the string is genuinely an interpolation
    # or a concatenation of two otherwise-legal references.
    #
    # This function stops at shape. Whether the path EXISTS is R-3's other half
    # and lives in resolve.lookup -- a shape validator that also did lookups
    # would need the whole flattened tree and could no longer be called on a
    # single value.
    if not _WHOLE_NODE.match(value):
        raise _rule("R-1", f"reference must occupy the whole scalar node, got {value!r}; "
                           "string interpolation silently degrades object/int to string")


def classify(value: Any) -> Optional[str]:
    """Return the referenced dotted path, or None when `value` is not a reference.

    None means "this is not a reference" and nothing else. Every illegal shape
    raises. Non-strings return None so callers can pass leaves of any type.
    """
    # *** Why illegal shapes raise instead of returning None. None would make an
    # illegal reference indistinguishable from an ordinary string, and resolve()
    # would then copy it verbatim into the resolved artifact. A process would
    # start, load the artifact, and receive the literal text
    # "${common.safety.brake.a_mps2}" where it expected a number. Depending on
    # the consumer that is either a crash far from the cause or a silent string
    # comparison that never matches. Both are worse than refusing to start.
    #
    # This two-line guard is also what makes validate_shape's precondition hold
    # everywhere it matters, which is why callers should come through here.
    if not isinstance(value, str) or not _ANY_REF.search(value):
        return None
    validate_shape(value)
    # .group(1) without a None check is safe ONLY because validate_shape has
    # already raised on everything _WHOLE_NODE would fail to match. If a rule is
    # ever relaxed there, this line becomes an AttributeError on None -- so
    # relaxing validate_shape means revisiting this line, not just that one.
    return _WHOLE_NODE.match(value).group(1)


def resolve(tree: Dict[str, Any]) -> Dict[str, Any]:
    """Expand every ${common.*} in `tree` and return a new tree. Input is not mutated.

    Raises ReferenceError_ (E_CONFIG_INVALID) on any violation of R-1 ~ R-4,
    on an unresolvable path, on a cycle, and on a chain over MAX_CHAIN.
    Caller must run this AFTER the L0~L5 overlay, never during it.
    """
    # Flattening is what keeps R-5 intact: a list is a leaf here (see
    # merge.flatten), so it arrives as one value and no element of it is ever
    # inspected. Walking the nested tree instead would make it easy to "helpfully"
    # descend into list elements and silently re-open whole-table replacement.
    flat = flatten(tree)

    # Validate shapes first, so an error names the rule rather than surfacing as
    # a missing key three steps into resolution.
    #
    # This is a full pass over every scalar before any expansion happens, and
    # that is the point: with a single interleaved pass, "${common.a:-2.5}"
    # would be reported only if resolution happened to reach it, so the same
    # defective file would pass or fail depending on dict ordering.
    for key, value in flat.items():
        if isinstance(value, str) and "${" in value:
            classify(value)

    def lookup(path: str, seen: List[str]) -> Any:
        if path in seen:
            # Cycles are CFG-CM-9's job to report in full; here we only refuse to
            # loop forever. !! Do not return a partial value instead.
            #
            # `seen` is seeded by the caller with the ORIGIN key, which is what
            # turns a self-reference (common.a: ${common.a}) into a reported
            # cycle rather than a first hop that looks perfectly ordinary.
            raise _rule("R-4", f"reference cycle through {path!r} (chain {' -> '.join(seen)})")
        if len(seen) >= MAX_CHAIN:
            # Depth limit as well as the cycle test, because the two catch
            # different things: a long chain through distinct keys never repeats
            # a path and would otherwise recurse until Python's own stack limit,
            # which surfaces as RecursionError with no rule id and no key path.
            raise _rule("R-4", f"alias chain longer than {MAX_CHAIN}: {' -> '.join(seen + [path])}")
        if path not in flat:
            # R-3's other half. There is deliberately no branch here that
            # substitutes a default, a None, or the key's own text: this is the
            # exact spot where "just fall back to something sensible" would be
            # written, and CFG-40 exists because that edit is invisible in
            # review and invisible at runtime.
            raise _rule("R-3", f"unresolvable reference {path!r} -- E_CONFIG_INVALID. "
                               "There is no fallback: refusing to start is the point")
        val = flat[path]
        # An alias may point at another alias (R-4 permits nesting inside
        # common.*), so follow it. classify() is reused rather than a bare regex
        # match so a target that is itself malformed raises with a rule id.
        nxt = classify(val) if isinstance(val, str) else None
        return lookup(nxt, seen + [path]) if nxt else val

    # Build a new mapping instead of expanding into `flat` in place. lookup()
    # reads flat, so in-place expansion would let a chain be walked in one hop or
    # in three depending only on which key this loop reached first -- and since
    # MAX_CHAIN is counted on that hop count, the very same file would resolve or
    # raise R-4 according to key order. Note this is not about the input: flatten
    # already returns a fresh dict, so `tree` is safe either way.
    out: Dict[str, Any] = {}
    for key, value in flat.items():
        target = classify(value) if isinstance(value, str) else None
        out[key] = lookup(target, [key]) if target else value

    # Imported locally, unlike flatten at module top. Worth a look if you are
    # tidying imports: there is no cycle forcing this, so the asymmetry is
    # historical rather than load-bearing.
    from .merge import unflatten
    return unflatten(out)


def check_l6(tree: Dict[str, Any], alias_blacklist: Iterable[str]) -> None:
    """R-6: reject a process (L6) config that redefines shared state. Raises on violation.

    `alias_blacklist` is the 10 S5.4.5 alias table, passed in by the caller.
    """
    # *** R-6 is the only rule of the seven with teeth, per the contract: R-1~R-5
    # describe how a reference must be written, and none of them can stop a
    # process config from simply not using one.
    #
    # The table is a PARAMETER rather than an import on purpose. This module owns
    # the reference axis; the alias table is a document artefact that changes on
    # its own schedule, and binding it here would mean every alias-table edit
    # touches the resolver. It also lets the test supply its own list.
    #
    # Two prohibitions, defending two different failures:
    #   * a `common` top-level key in a process config. L6 is NOT one of
    #     layers.LAYERS -- that list stops at L5 and check_namespace therefore
    #     never sees a process file -- so this branch is the only thing standing
    #     between an L6 file and a redefinition of shared state. What makes the
    #     redefinition unfindable afterwards is that each process gets its own
    #     resolved artifact: every artifact stays internally consistent while two
    #     processes hold different values for one quantity, and nothing
    #     downstream compares artifacts to each other.
    #   * a private key whose name is on the alias table, which is how one
    #     physical quantity acquires two names that then drift apart
    if "common" in tree:
        raise _rule("R-6", "L6 process config must not carry a 'common' top-level key; "
                           "shared values are referenced with ${common.*}, never redefined")
    banned = {b for b in alias_blacklist}
    for key in flatten(tree):
        # Both the leaf name and the full dotted path are tested. Leaf, because
        # the drift being prevented is a NAME collision and it does not matter
        # which section it is nested under; full path, so the table can also ban
        # a specific location when the bare name is too common to ban outright.
        #
        # NOTE this rejects by name, not by value, so it stays a purely
        # syntactic check that needs no access to common.*.
        leaf = key.split(".")[-1]
        if leaf in banned or key in banned:
            raise _rule("R-6", f"L6 defines {key!r}, whose name is on the S5.4.5 alias "
                               f"table; use ${{common.*}} instead of a second name for "
                               "the same quantity")
    # Falling off the end means accepted. That path is covered by
    # test_r6_ordinary_private_keys_are_allowed, and it is not decoration: a
    # check_l6 that rejected EVERYTHING would satisfy both negative tests above
    # while making every process config in the system invalid. CLAUDE.md 3.3 --
    # negative cases alone are passed by a broken implementation.


def check_no_anchors(raw_text: str, where: str = "<config>") -> None:
    """R-7: reject YAML anchors and aliases. Takes RAW FILE TEXT, never a parsed tree.

    `where` is only used to build the message, so callers can name a file or a
    document section. Raises ReferenceError_ on the first hit.
    """
    # *** The raw-text signature is the entire point of this function. Anchors
    # are resolved by the YAML parser; by the time you hold a dict they are gone
    # and every file that uses them looks clean. A tree-based version of this
    # check would pass on 100 percent of its intended targets while reporting
    # success -- CLAUDE.md 3.2 calls this the permanently-green assertion, and
    # it is strictly worse than no check, because it also stops anyone from
    # writing the real one.
    #
    # This is not hypothetical here: 12 S12 carried an anchor/alias pair
    # (&d_safe / *d_safe) until 2026-08-05, and
    # test_r7_matches_the_real_defect_this_project_had replays that exact text.
    #
    # Read the SCAN SURFACES block in the module docstring before relying on a
    # pass: list-item anchors are outside what these patterns reach.
    for lineno, line in enumerate(raw_text.split("\n"), 1):
        # Naive comment strip: split on the first '#' regardless of quoting.
        # It exists so prose such as "# & in a comment" does not raise, which
        # test_r7_does_not_fire_on_ordinary_text pins. The known cost is a false
        # NEGATIVE when a '#' appears inside a quoted scalar earlier on the line
        # -- chosen over a false positive deliberately, since a checker that
        # cries wolf on ordinary comments is a checker that gets disabled.
        stripped = line.split("#", 1)[0]
        if _ANCHOR.search(stripped):
            raise _rule("R-7", f"{where}:{lineno}: YAML anchor '&' is not allowed. "
                               "Anchors do not cross files while looking like they do, "
                               "and two sharing mechanisms make expansion order undefinable")
        if _ALIAS.search(stripped):
            raise _rule("R-7", f"{where}:{lineno}: YAML alias '*' is not allowed; "
                               "express sharing with ${common.*}")


def find_violations(tree: Dict[str, Any]) -> List[Tuple[str, str]]:
    """Collect (key, rule) for every shape violation instead of stopping at the first.

    For tooling that reports a whole file at once. resolve() still raises on the
    first violation, because a loader that continued past one would hand a
    process a configuration it should never have accepted.

    WARNING: the second tuple element is currently always the empty string.
    See the comment on the extraction below before relying on it.
    """
    out: List[Tuple[str, str]] = []
    for key, value in flatten(tree).items():
        # Same "${" prefilter as resolve() uses, so the two agree on what counts
        # as worth inspecting. If one is ever changed the other has to move with
        # it, or this reporter will show a clean file that resolve() rejects.
        if not isinstance(value, str) or "${" not in value:
            continue
        try:
            classify(value)
        except ReferenceError_ as exc:
            # *** KNOWN DEFECT, measured 2026-08-06, left unfixed here because
            # this pass changes no behaviour. Do not read this line as working.
            #
            # Observed: find_violations returns [('common.x', ''), ...] -- the
            # rule id is always lost, so callers get a key and an empty string.
            #
            # Cause: ConfigLayerError goes through XbrainError.__init__, which
            # builds the message as "{code}: {message}". str(exc) is therefore
            # "E_CONFIG_INVALID: R-3: ..." and split(":")[0] yields
            # "E_CONFIG_INVALID", which the replace() then blanks out. The line
            # was evidently written against a str() that did not carry the code
            # prefix. Element [1] carries the rule id, with a leading space.
            #
            # How it survived: nothing in the repo calls find_violations and no
            # test names it, so the returned tuple has never been looked at. Do
            # not read the green suite as coverage of this function.
            #
            # Whoever fixes it: assert the rule id itself. A test that checks
            # only the arity of the tuple passes on the broken version too, which
            # is the permanently-green shape of CLAUDE.md 3.2.
            out.append((key, str(exc).split(":")[0].replace("E_CONFIG_INVALID", "").strip()))
    return out
