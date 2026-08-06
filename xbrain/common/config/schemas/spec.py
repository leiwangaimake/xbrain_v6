"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: spec.py
Brief: CFG-10 schema vocabulary and the type/required/range validator engine

Description:
The problem this file solves. Without a schema layer, a wrong-TYPE value in a
config file (t_lat_s written as the string "0.4" instead of the number 0.4)
survives parsing and only detonates deep inside a safety assertion: 11 S9.6 SP-5
does `common.safety.t_lat_s >= 0.4`, and `"0.4" >= 0.4` raises TypeError in
Python 3. The freeze oneshot then exits on a traceback pointing at the assertion,
not at the key. 10 S5.4.6 CFG-10 (grep anchor: "启动时 schema 校验") mandates a
type + range + required check that runs FIRST and reports the key path, the
expected type and the actual type, so the operator sees "common.safety.t_lat_s:
expected number, got string" rather than a stack trace three layers away. This
module is the engine; registry.py holds the per-file schema assets.

Which design section this follows. Types and required-ness come from the key
table in 10 S5.4.5 (grep anchor: "共享参数唯一定义处对照表") plus the per-volume
config sections (11 S9.6 for spec, 11 S1.5.5 for clock, 16 S14 for p4_agent).
The validator itself owns no key knowledge -- it is fed a Schema and a parsed
tree and does nothing else.

What this module deliberately does NOT do, so nothing gets bolted on:

  * It does NOT enforce safety RANGES like SP-1..SP-11 or S-1..S-6. Those are
    CFG-11 / assertion G, whose authority table is 12 S12.1 (10 S5.4.4 assertion
    G row, and 10 S5.4.6 lists CFG-10 and CFG-11 as two layers -- "不得合并成一
    层"). Cross-key relations (a_mps2 <= spec.max_decel_mps2, throttle_speed_mps
    < spec.max_vx_mps) live there, never here. The FieldSpec range mechanism
    below exists for pure per-key domain bounds that are a property of the field
    itself, not for the safety assertions; registry.py explains per file why it
    carries none of those today.
  * It does NOT check namespace placement (which layer may write which prefix).
    That is check_namespace in layers.py (CFG-FZ-16). A key that passes here is
    not thereby known to sit in a legal layer.
  * It does NOT report null-unassigned or missing-shared-key. Those are
    assertion A (residual ${, unresolved path, null unassigned) and assertion M
    (a required shared key supplied by no layer). This check runs BEFORE both,
    so null is a PASS here on purpose: a value left null is the calibration gap
    A must name by path, and swallowing it here would rob A of its input.

The three writings that look correct and are not (each measured in this project):

  1. Coercing before comparing. `float("0.4")` makes the string mutation pass and
     defeats the whole point (the TODO spells this out: "不得靠 YAML 自动转型蒙
     混"). We inspect the ALREADY-PARSED Python type and never convert.
  2. Treating bool as a number. bool is a subclass of int in Python, so
     `isinstance(True, int)` is True; a NUMBER field would silently accept `true`
     for a speed. NUMBER and INTEGER exclude bool explicitly.
  3. Rejecting an int where a number is wanted. YAML parses `2` as int and `2.0`
     as float; a speed of `2` is a legal number. NUMBER accepts int or float, so
     the check does not turn a valid integer literal into a false failure.
"""

from dataclasses import dataclass
from typing import Any, Dict, Optional

# Imported, not spelled as a literal: CLAUDE.md 3.5 forbids E_* string literals
# outside common/errors/, and no_literal_ecode.py enforces it tree-wide. The
# rename-safety argument is the same one ConfigLayerError makes -- a literal here
# would be a second source of truth for the spelling that diverges silently.
from ..merge import flatten
from ...errors import E_CONFIG_INVALID, XbrainError

# ---------------------------------------------------------------------------
# The closed vocabulary of schema types.
#
# Plain str constants rather than an Enum, so a schema entry reads `num()` /
# `text()` (via the helpers below) and the stored token is a value that prints
# straight into an error message with no .value dance. The frozenset is the
# authority a self-test asserts every FieldSpec.type is a member of; a typo in a
# schema token then fails loudly at construction, not silently at validation.
# ---------------------------------------------------------------------------
NUMBER = "number"      # int OR float, never bool -- a measured quantity
INTEGER = "integer"    # int, never bool -- a count, budget, port, row total
STRING = "string"      # text: a path, a url, an enum label, a template
BOOLEAN = "boolean"    # a flag; kept distinct from NUMBER so true != 1 here
MAPPING = "mapping"    # a nested object; enforced only when present as a leaf
LIST = "list"          # a sequence; R-5 makes it a whole-table leaf value
ANY = "any"            # present-and-required, type deliberately not pinned

#: ANY is the HONEST token for a key whose presence is mandated but whose type is
#: genuinely ambiguous in the authoritative docs (e.g. calib_rev, which appears
#: only alongside config_rev and could be a counter or a label). Using ANY says
#: "this must exist, its type is not yet fixed" instead of guessing a token and
#: turning a correct future fill into a false failure. It is used sparingly and
#: each site in registry.py says why; it is NOT a catch-all to avoid thinking.

#: Every legal token. Used by the self-test and by _matches' final guard. Never
#: write its size into code or a comment (CLAUDE.md 3.7) -- membership is the
#: contract, the count is not.
TYPE_TOKENS = frozenset({NUMBER, INTEGER, STRING, BOOLEAN, MAPPING, LIST, ANY})


# The one exception this module raises. XbrainError base so a single
# `except XbrainError` at the freeze call site catches it alongside
# ConfigLayerError and ResolvedConfigError, while a genuine defect (an
# AttributeError from a malformed schema) still travels up untouched -- the same
# reasoning exceptions.py gives for not collapsing the family into one type.
class SchemaError(XbrainError):
    """A config file failed its CFG-10 schema: wrong type, out of range, or a
    required key absent.

    path / expected / actual are fields, not only text inside the message, so a
    caller reporting the failure upward can branch on which key and which type
    without re-parsing the sentence. The message still carries all three because
    the overwhelmingly common handler prints str(e) and nothing else.
    """

    def __init__(self, message: str, *, path: Optional[str] = None,
                 expected: Optional[str] = None, actual: Optional[str] = None):
        # E_CONFIG_INVALID is the code 10 S5.4.6 CFG-10 assigns to this family:
        # a config file that fails its own self-check, startup refused, group L,
        # not retryable. Same code as the other config failures on purpose -- the
        # caller's response (refuse to start) is identical, and a distinct code
        # would invite someone to treat a schema failure as milder.
        super().__init__(E_CONFIG_INVALID, message)
        self.path = path          # dotted key path, or "" for a root-shape fault
        self.expected = expected  # the schema token (or bound) that was wanted
        self.actual = actual      # the type name (or value) actually found


@dataclass(frozen=True)
class FieldSpec:
    """One key's constraint: its type, whether it must be present, and an
    optional inclusive numeric range.

    Frozen because a schema is a shipped asset, not runtime state; a mutable spec
    would let one importer redefine a key's type for every other importer.

    NOTE on defaults: there are none on this dataclass. The ergonomic defaults
    live in the helper constructors below (num/integer/text/...), documented in
    one place. This is metadata about a key, NOT a safety config value, so it is
    not the CLAUDE.md 3.1 case that forbids defaults -- that rule governs the
    VALUES under common.safety.* / common.spec.*, which never get authored here.
    """

    type: str            # a TYPE_TOKENS member
    required: bool       # must this key be PRESENT in the file (value may be null)
    lo: Optional[float]  # inclusive lower bound, or None for "no lower bound"
    hi: Optional[float]  # inclusive upper bound, or None for "no upper bound"


# Helper constructors. They exist so a schema entry is `num()` or `text(required
# =False)` rather than `FieldSpec("number", True, None, None)`, which is both
# noisy and easy to get wrong by positional slip. The keyword-only defaults are
# the single documented place a default lives; see the FieldSpec note on why that
# is not the 3.1 prohibition.
def num(*, required: bool = True, lo: Optional[float] = None,
        hi: Optional[float] = None) -> FieldSpec:
    """A NUMBER field (int or float, not bool). lo/hi are a per-key DOMAIN bound
    only -- never a safety assertion; those belong to assertion G."""
    return FieldSpec(NUMBER, required, lo, hi)      # int or float, not bool


def integer(*, required: bool = True, lo: Optional[float] = None,
            hi: Optional[float] = None) -> FieldSpec:
    """An INTEGER field (int, not bool)."""
    return FieldSpec(INTEGER, required, lo, hi)     # int, not bool


def text(*, required: bool = True) -> FieldSpec:
    """A STRING field. No range: a range on text has no meaning here."""
    return FieldSpec(STRING, required, None, None)  # text; no numeric range


def boolean(*, required: bool = True) -> FieldSpec:
    """A BOOLEAN field."""
    return FieldSpec(BOOLEAN, required, None, None)  # flag; true != 1 here


def mapping(*, required: bool = True) -> FieldSpec:
    """A MAPPING placeholder. Enforced only when it appears as a leaf (null or an
    empty object); a populated object recurses in flatten and never reaches the
    type check, so this mostly documents an intended shape."""
    return FieldSpec(MAPPING, required, None, None)  # nested object leaf


def listof(*, required: bool = True) -> FieldSpec:
    """A LIST field. flatten treats a list as one leaf (R-5 whole-table
    replacement), so this checks the value is a list, not its elements."""
    return FieldSpec(LIST, required, None, None)     # whole-table leaf (R-5)


def anything(*, required: bool = True) -> FieldSpec:
    """An ANY field: presence is checked, type is not. See the ANY note above for
    when this is the honest choice rather than a guessed token."""
    return FieldSpec(ANY, required, None, None)      # present-only, type open


@dataclass(frozen=True)
class Schema:
    """The schema for one config file.

    name       the config file's path relative to the config root, e.g.
               "safety/brake.yaml". Used only in messages, so a failure names the
               file. It is NOT an absolute path and nothing parses one out of it;
               the config root lives in layers.py, never here (that is also why
               no_config_source_read.py finds no source literal in this package).
    authority  the design section this schema was derived from (grep-able anchor,
               NUM-4: volume + section, never a line number).
    fields     dotted-leaf-path -> FieldSpec, matching what merge.flatten yields
               for THIS file's own tree. For an L1/L2/L3 file the paths start
               "common."; for an L6 process file they start with the process's
               private top-level keys (asr_post., bypass., ...).
    """

    name: str
    authority: str
    fields: Dict[str, FieldSpec]


def _type_name(value: Any) -> str:
    """A readable name for an ACTUAL value's type, for the error message.

    Mapped onto the schema vocabulary where it lines up (str->string,
    bool->boolean, int/float->integer/number) so "expected number, got string"
    reads in one vocabulary. bool is checked before int because bool is a
    subclass of int and the unmapped `int` branch would otherwise mislabel True
    as an integer -- the exact confusion NUMBER/INTEGER exclude bool to avoid.
    """
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, int):
        return "integer"
    if isinstance(value, float):
        return "number"
    if isinstance(value, str):
        return "string"
    if isinstance(value, dict):
        return "mapping"
    if isinstance(value, list):
        return "list"
    # Any other Python type reaching a parsed config tree is a parser or caller
    # defect, not a config author's mistake; naming the raw type is the most
    # useful thing to print rather than pretending it is one of ours.
    return type(value).__name__


def _matches(value: Any, token: str) -> bool:
    """True if value is an acceptable instance of the schema token.

    The two subtle rows are NUMBER and INTEGER: both exclude bool (subclass of
    int) so a flag cannot stand in for a quantity, and NUMBER admits int so a
    YAML `2` is not rejected for a field that also allows `2.0`. Everything else
    is a plain isinstance.
    """
    if token == NUMBER:
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if token == INTEGER:
        return isinstance(value, int) and not isinstance(value, bool)
    if token == STRING:
        return isinstance(value, str)
    if token == BOOLEAN:
        return isinstance(value, bool)
    if token == MAPPING:
        return isinstance(value, dict)
    if token == LIST:
        return isinstance(value, list)
    if token == ANY:
        # Reached only for a non-null value (null is handled before _matches is
        # called). ANY accepts it: the presence obligation is met by the required
        # check, and the type is deliberately unconstrained here.
        return True
    # A token outside TYPE_TOKENS means a schema was authored with a bad type
    # string. That is a programming error in registry.py, not a config failure,
    # so it raises ValueError (no config vocabulary) rather than SchemaError --
    # and it raises rather than returning False, because returning False would
    # let a mistyped token silently accept every value.
    raise ValueError(f"schema uses an unknown type token {token!r}; "
                     f"legal tokens are defined in TYPE_TOKENS")


def _has_children(path: str, flat: Dict[str, Any]) -> bool:
    """Does the flattened tree hold any leaf UNDER this path?

    True means the config author put a POPULATED mapping where `path` sits:
    flatten recursed into it, so `path` itself is not a leaf in flat but
    `path + "."` prefixes are. This is how a container key stays detectable
    (common.calib.frames filled to frames.ptz_base....) AND how a mapping written
    where a scalar belongs is caught -- see validate_tree, which turns the second
    case into a type error instead of letting the key silently vanish.
    """
    prefix = path + "."
    return any(key.startswith(prefix) for key in flat)


def _check_range(schema_name: str, path: str, value: Any, spec: FieldSpec) -> None:
    """Inclusive per-key bound check. Only NUMBER/INTEGER carry lo/hi, and value
    is already known non-null and type-correct when this runs.

    This is a DOMAIN bound (a property of the field), never a safety assertion.
    See the module docstring: SP-*/S-* cross-key safety ranges are assertion G.
    """
    # Guard on `is not None`, not truthiness: a bound of 0.0 is a legitimate
    # inclusive edge, and `if spec.lo:` would drop it. Same zero-vs-unset trap
    # CLAUDE.md 3.1 attacks from the config side.
    if spec.lo is not None and value < spec.lo:
        raise SchemaError(
            f"{schema_name}: {path}: value {value!r} is below the inclusive "
            f"lower bound {spec.lo!r}",
            path=path, expected=f">= {spec.lo!r}", actual=repr(value))
    if spec.hi is not None and value > spec.hi:
        raise SchemaError(
            f"{schema_name}: {path}: value {value!r} is above the inclusive "
            f"upper bound {spec.hi!r}",
            path=path, expected=f"<= {spec.hi!r}", actual=repr(value))


def validate_tree(schema: Schema, tree: Any) -> None:
    """Validate one already-parsed config tree against its schema. Raise
    SchemaError on the first violation; return None on success.

    The tree is the RAW parsed content of a single file, not the merged overlay:
    CFG-FZ-17 is a per-file check (19 files, 19 schemas), and it runs before the
    overlay is even built, which is what puts it ahead of assertion A. Callers
    hand in a parsed tree exactly as the overlay loader does, so this stays
    testable with no configs/ tree on disk.
    """
    # An empty file (only comments) parses to None. That is the normal skeleton
    # state, not an error -- there is simply nothing to type-check yet, and any
    # required-key check below will fire on its own if the file was supposed to
    # carry something. A non-dict, non-None top level (a bare scalar or a list at
    # the root) is malformed and cannot carry keyed config, so it raises.
    if tree is None:
        flat: Dict[str, Any] = {}
    elif isinstance(tree, dict):
        flat = flatten(tree)
    else:
        raise SchemaError(
            f"{schema.name}: top level is {_type_name(tree)}, not a mapping; "
            f"a config file must have keyed content at its root",
            path="", expected="mapping", actual=_type_name(tree))

    # Iterate the SCHEMA, not the flattened tree. Iterating the tree instead
    # misses the case the two failing mutations exposed: a scalar key given a
    # POPULATED mapping (`t_lat_s: {foo: bar}`) flattens to `t_lat_s.foo`, so the
    # bare `t_lat_s` leaf is gone and a tree-driven loop never looks at it. Each
    # schema key is in exactly one of three states -- a leaf, a populated mapping,
    # or absent (flatten makes these mutually exclusive) -- and each is handled.
    #
    # Unknown keys in the tree are simply never iterated here, which is the
    # "ignore unknown" rule: rejecting them would duplicate check_namespace or,
    # for a file whose key set is still deferred, reject a correct future fill.
    for path, spec in schema.fields.items():
        if path in flat:
            value = flat[path]
            # null is a PASS, always: declared-but-unassigned is assertion A's to
            # name, and this check runs first precisely so A still sees it. A list
            # or an empty {} is a real leaf and is type-checked like any other.
            if value is None:
                continue
            if not _matches(value, spec.type):
                # All three things CFG-10 requires -- key path, expected type,
                # actual type -- plus the offending value, because for the
                # canonical failure (the string "0.4") the value IS the tell.
                raise SchemaError(
                    f"{schema.name}: {path}: expected {spec.type}, got "
                    f"{_type_name(value)} (value {value!r})",
                    path=path, expected=spec.type, actual=_type_name(value))
            # Range only applies to the numeric tokens; the other helpers store
            # lo == hi == None, so this is a no-op for them.
            if spec.type in (NUMBER, INTEGER):
                _check_range(schema.name, path, value, spec)
        elif _has_children(path, flat):
            # A populated mapping sits here. That is correct only if the schema
            # expected a mapping (or ANY, which constrains nothing); a scalar or
            # list field given an object is the wrong-type case a tree-driven loop
            # would have missed. We do NOT descend -- the children are not in this
            # file's schema (they belong to a deeper layer, e.g. calib frames
            # filled at L4b).
            if spec.type not in (MAPPING, ANY):
                raise SchemaError(
                    f"{schema.name}: {path}: expected {spec.type}, got mapping",
                    path=path, expected=spec.type, actual="mapping")
        else:
            # Absent. Only a required key makes that a failure; a required=False
            # key legitimately missing (clock.yaml's empty skeleton) is fine.
            if spec.required:
                raise SchemaError(
                    f"{schema.name}: required key {path!r} is missing",
                    path=path, expected=f"{spec.type} (required)", actual="absent")
