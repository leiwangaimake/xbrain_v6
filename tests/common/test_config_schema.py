"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_schema.py
Brief: CFG-FZ-17 startup schema check -- type, required, range, and ordering

Description:
CFG-10 schema layer (xbrain/common/config/schemas). Every assertion here carries
the mutation that turns it red, per CLAUDE.md 3.3. Two cases are the ones the
TODO names by hand and matter most:

  test_mutation_1_string_number_is_red
      t_lat_s written as the string "0.4" must go red on the number type. It is
      built by parsing real YAML so the check is proven not to be fooled by YAML
      auto-coercion (the TODO: "不得靠 YAML 自动转型蒙混").

  test_mutation_2_schema_before_assertion_g
      Moving the schema check AFTER a minimal assertion-G stand-in shows what that
      order buys: the stand-in dies in a TypeError that names no key path, where
      the schema first would have reported common.safety.t_lat_s cleanly.

The coverage and baseline cases guard the assets: SCHEMAS must cover exactly the
19 config files, and every file as it ships today must PASS (the schema runs
before assertion A, so null placeholders are not its to reject).
"""

import os
import sys

import pytest
import yaml

# Same path bootstrap the sibling config tests use, so this runs from a bare
# checkout with no install step.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common.config.schemas import (  # noqa: E402
    ANY, BOOLEAN, INTEGER, NUMBER, STRING, TYPE_TOKENS, CONFIG_FILES, SCHEMAS,
    Schema, SchemaError, anything, boolean, integer, listof, mapping, num,
    text, validate_config, validate_tree)
from xbrain.common.config.schemas.spec import FieldSpec, _matches  # noqa: E402
from xbrain.common.errors import E_CONFIG_INVALID, XbrainError  # noqa: E402

CONFIG_ROOT = os.path.join(ROOT, "configs")


def _load(rel):
    """Parse one on-disk config file exactly as the freeze service would."""
    with open(os.path.join(CONFIG_ROOT, rel), encoding="utf-8") as fh:
        return yaml.safe_load(fh)


# A helper to build a brake.yaml tree with one value swapped, so the mutation
# cases differ from the baseline by exactly the field under test.
def _brake_tree(t_lat_s=0.4, a_mps2=2.5, k=1.5):
    return {"common": {"safety": {"t_lat_s": t_lat_s,
                                  "brake": {"a_mps2": a_mps2, "k": k}}}}


# ── coverage: the assets cover exactly the 19 files ──────────────────────────

def test_registry_covers_exactly_the_on_disk_config_set():
    """SCHEMAS must key exactly the 19 config files on disk.

    Mutation: drop one entry from SCHEMAS => the on-disk file has no schema and
    this set difference goes non-empty. The point is to catch a NEW config file
    added later with no schema, which validate_config would otherwise raise on
    only at runtime in the field.
    """
    on_disk = set()
    for dirpath, dirs, files in os.walk(CONFIG_ROOT):
        # sites/ and calib/ are per-instance templates, not among the 19
        # (CFG-FZ-17 triage); prompts/ and secrets/ are not yaml config files.
        # generated/ is a build artifact (whitelist.yaml materialised from
        # 11 S1.1.6, validated by its consumer xbrain/common/zenoh/whitelists.py),
        # and probe/ is a Stage-0 config (thresholds.yaml, CFG-BT-1) the probe
        # validates itself by crashing on any absent key -- neither passes
        # through the SCHEMAS registry, so both sit outside this 19-file set.
        dirs[:] = [d for d in dirs
                   if d not in ("sites", "calib", "prompts", "secrets",
                                "generated", "probe")]
        for name in files:
            if name.endswith(".yaml"):
                on_disk.add(os.path.relpath(os.path.join(dirpath, name), CONFIG_ROOT))
    assert set(CONFIG_FILES) == on_disk
    assert len(CONFIG_FILES) == 19


@pytest.mark.parametrize("rel", CONFIG_FILES)
def test_every_shipped_file_passes_its_schema(rel):
    """The schema runs BEFORE assertion A, so today's null-filled skeletons must
    pass. A schema that rejected a null would rob assertion A of the key path it
    exists to print.

    Mutation guard: if any schema wrongly range-checked or required a value that
    is null on disk, this parametrised case would go red for that file.
    """
    validate_config(rel, _load(rel))


# ── mutation (1): a string where a number belongs ────────────────────────────

def test_mutation_1_string_number_is_red():
    """t_lat_s: "0.4" (string) must be rejected on the number type, with the key
    path, expected type and actual type in the failure.

    Built from real YAML so the quoting distinction is proven to survive to the
    check: `t_lat_s: 0.4` parses to float, `t_lat_s: "0.4"` parses to str, and
    only the second is rejected. This is the "不得靠 YAML 自动转型蒙混" clause.
    """
    doc = 'common:\n  safety:\n    t_lat_s: "0.4"\n    brake:\n      a_mps2: 2.5\n      k: 1.5\n'
    tree = yaml.safe_load(doc)
    assert tree["common"]["safety"]["t_lat_s"] == "0.4"  # parsed as str, quoted
    assert isinstance(tree["common"]["safety"]["t_lat_s"], str)   # YAML kept it str
    with pytest.raises(SchemaError) as e:
        validate_config("safety/brake.yaml", tree)      # goes red on the type
    # key path + expected type + actual type, all three (CFG-10 wording)
    assert e.value.path == "common.safety.t_lat_s"      # the key path
    assert e.value.expected == "number"                 # expected type
    assert e.value.actual == "string"                   # actual type
    assert e.value.code == E_CONFIG_INVALID             # the closed-set code
    assert "common.safety.t_lat_s" in str(e.value)      # path is in the message
    assert "expected number" in str(e.value)            # and so is the expectation


def test_mutation_1_unquoted_number_still_passes():
    """The other half of the mutation: `t_lat_s: 0.4` unquoted parses to float and
    must pass, so the check rejects the string specifically, not every 0.4.
    """
    doc = "common:\n  safety:\n    t_lat_s: 0.4\n    brake:\n      a_mps2: 2.5\n      k: 1.5\n"
    tree = yaml.safe_load(doc)
    assert isinstance(tree["common"]["safety"]["t_lat_s"], float)
    validate_config("safety/brake.yaml", tree)  # no raise


def test_check_does_not_coerce():
    """Documents the trap the mutation guards. A validator that coerced with
    float() would ACCEPT "0.4" and defeat the whole point; float("0.4") succeeds,
    yet our check still rejects it. The rejection is the proof of no coercion.
    """
    assert float("0.4") == 0.4  # coercion would succeed and hide the bug
    with pytest.raises(SchemaError):
        validate_config("safety/brake.yaml", _brake_tree(t_lat_s="0.4"))


# ── mutation (2): schema must run before assertion G ─────────────────────────

# A MINIMAL stand-in for one clause of assertion G (CFG-11). The real assertion G
# lives with the freeze service and is a different task; the CFG-FZ-17 triage
# permits a "最小 range-check 替身" to demonstrate the ordering. This mirrors
# 11 S9.6 SP-5's `common.safety.t_lat_s >= 0.4`, and its whole relevance is that
# `">=" ` between str and float raises TypeError in Python 3.
def _assertion_g_standin(tree):
    """SP-5 clock/brake clause, stripped to the one comparison that matters."""
    t_lat_s = tree["common"]["safety"]["t_lat_s"]
    # No "is a number" pre-check on purpose: this stand-in is what runs when the
    # schema is NOT in front of it, and the missing pre-check is exactly why a
    # string blows up here instead of being reported by path.
    return t_lat_s >= 0.4


def test_mutation_2_schema_before_assertion_g():
    """Correct order (schema, then G): the string is caught by the schema with a
    key path, and G never runs. Mutated order (G, then schema): G dies in a
    TypeError that carries no key path.
    """
    bad = _brake_tree(t_lat_s="0.4")

    # Correct order -- schema first. SchemaError names the key; G is never reached.
    with pytest.raises(SchemaError) as e:
        validate_config("safety/brake.yaml", bad)   # schema runs first
        _assertion_g_standin(bad)  # unreachable; here to show it comes AFTER
    assert e.value.path == "common.safety.t_lat_s"  # clean key path, not a trace

    # Mutated order -- G first (schema moved after it). This is the mutation the
    # TODO names. G raises TypeError, not SchemaError, and the message names no
    # key path: the operator gets a traceback about '>=' and str/float, not
    # "common.safety.t_lat_s".
    with pytest.raises(TypeError) as g:
        _assertion_g_standin(bad)
        validate_config("safety/brake.yaml", bad)  # unreachable in mutated order
    assert not isinstance(g.value, SchemaError)   # a raw TypeError, not our error
    assert "common.safety.t_lat_s" not in str(g.value)   # names no key path


def test_standin_passes_a_correct_number():
    """The stand-in is real logic, not a rigged always-raiser: on a correct float
    it simply evaluates. Without this, the TypeError above could be an artefact of
    a stand-in that raises on everything.
    """
    assert _assertion_g_standin(_brake_tree(t_lat_s=0.4)) is True
    assert _assertion_g_standin(_brake_tree(t_lat_s=0.2)) is False


# ── type tokens ──────────────────────────────────────────────────────────────

def test_number_accepts_int_and_float_rejects_bool_and_str():
    """NUMBER admits int and float (YAML 2 and 2.0 are both numbers) but not bool
    (a subclass of int) and not str. Each rejection has a red mutation: swapping
    the value type flips the assertion.
    """
    s = Schema("t", "test", {"x": num()})
    validate_tree(s, {"x": 2})       # int ok: YAML `2` is a valid number
    validate_tree(s, {"x": 2.0})     # float ok
    for bad in (True, "2", [2], {"a": 1}):    # bool, str, list, mapping all wrong
        with pytest.raises(SchemaError):
            validate_tree(s, {"x": bad})      # each swap flips green -> red


def test_integer_rejects_float_and_bool():
    """INTEGER is stricter than NUMBER: a float is not an integer, and bool is
    excluded for the same subclass reason.
    """
    s = Schema("t", "test", {"n": integer()})
    validate_tree(s, {"n": 3})                # a plain int passes
    for bad in (3.0, True, "3"):              # float, bool, str all rejected
        with pytest.raises(SchemaError):
            validate_tree(s, {"n": bad})      # 3.0 is the telling one (not 3)


def test_boolean_is_not_a_number_and_number_is_not_boolean():
    """The two directions that the int/bool subclassing makes easy to get wrong."""
    with pytest.raises(SchemaError):          # 1 is not a bool
        validate_tree(Schema("t", "test", {"b": boolean()}), {"b": 1})
    with pytest.raises(SchemaError):          # True is not a number
        validate_tree(Schema("t", "test", {"x": num()}), {"x": True})


def test_string_list_mapping_tokens():
    s = Schema("t", "test", {"s": text(), "l": listof(), "m": mapping()})
    validate_tree(s, {"s": "/a/b", "l": [1, 2], "m": {}})    # all three correct
    with pytest.raises(SchemaError):
        validate_tree(s, {"s": 5, "l": [1], "m": {}})     # s wrong (int, not str)
    with pytest.raises(SchemaError):
        validate_tree(s, {"s": "x", "l": {"not": "list"}, "m": {}})   # l wrong


def test_any_accepts_any_nonnull_but_null_still_passes():
    """ANY is presence-only: any non-null value is accepted, and null passes as
    everywhere else (deferred to assertion A).
    """
    s = Schema("t", "test", {"z": anything()})
    for v in (1, "x", [1], {"a": 1}, True, 2.5):   # every non-null type accepted
        validate_tree(s, {"z": v})                 # incl. a populated mapping
    validate_tree(s, {"z": None})                  # null still passes as elsewhere


def test_null_is_always_a_pass():
    """A declared-but-null value passes every token: the schema runs before
    assertion A and must leave the null for A to name.

    Mutation: if the type check fired on null, this brake tree (all null) would
    go red and assertion A would never get to print the key path.
    """
    s = Schema("t", "test", {"x": num(), "s": text(), "b": boolean()})
    validate_tree(s, {"x": None, "s": None, "b": None})   # all null, all pass


# ── required + presence ──────────────────────────────────────────────────────

def test_required_key_absent_is_red():
    """A required key that is not present at all fails with the key path.

    Mutation: mark the key required=False (or add it to the tree) => green. The
    absence, not the value, is what fires.
    """
    s = Schema("t", "test", {"common.safety.t_lat_s": num()})
    with pytest.raises(SchemaError) as e:
        validate_tree(s, {"common": {"safety": {}}})   # key absent entirely
    assert e.value.path == "common.safety.t_lat_s"     # named by its path
    assert "missing" in str(e.value)                   # reported as missing


def test_required_false_absent_is_green():
    """clock.yaml's shape: required=False keys let the empty file pass, while a
    wrong TYPE on a future fill is still caught.
    """
    s = Schema("t", "test", {"common.safety.clock.sync_timeout_s": num(required=False)})
    validate_tree(s, None)                      # empty file: absent + optional = ok
    validate_tree(s, {"common": {"safety": {"clock": {"sync_timeout_s": None}}}})  # null ok
    with pytest.raises(SchemaError):            # but a string is still red
        validate_tree(s, {"common": {"safety": {"clock": {"sync_timeout_s": "5"}}}})


def test_required_container_present_when_populated():
    """A required container counts as present when it is filled, not only when it
    is a bare null leaf. Without this, filling common.calib.frames would be
    reported as frames going MISSING -- a false failure on a correct edit.
    """
    s = Schema("t", "test", {"common.calib.frames": mapping()})
    validate_tree(s, {"common": {"calib": {"frames": None}}})        # null leaf, present
    validate_tree(s, {"common": {"calib": {"frames": {"ptz_base": {"h": 1.0}}}}})  # filled, present


def test_mapping_where_scalar_is_red():
    """A scalar key given a POPULATED mapping must be caught. flatten recurses
    into the mapping, so `t_lat_s: {foo: bar}` becomes `t_lat_s.foo` and a
    tree-driven check would never look at `t_lat_s`. The schema-driven loop turns
    it into a clean type error instead of a silently vanished key.

    Mutation: type the field as mapping() => green; as num() => red. That flip is
    exactly the hole this closes.
    """
    s = Schema("t", "test", {"common.safety.t_lat_s": num()})
    with pytest.raises(SchemaError) as e:
        validate_tree(s, {"common": {"safety": {"t_lat_s": {"foo": "bar"}}}})  # red
    assert e.value.path == "common.safety.t_lat_s"    # the key still named
    assert e.value.actual == "mapping"                # actual reported as mapping
    # and a genuine mapping field accepts the same populated object
    ok = Schema("t", "test", {"common.safety.t_lat_s": mapping()})
    validate_tree(ok, {"common": {"safety": {"t_lat_s": {"foo": "bar"}}}})  # green


def test_unknown_key_is_ignored():
    """An unknown key is not this layer's business (namespace is check_namespace,
    and a deferred file's future keys are not yet known). It passes here.
    """
    s = Schema("t", "test", {"known": num()})
    # "surprise" and "deep.x" are not in the schema, so they are skipped, not
    # rejected; only "known" is type-checked. This keeps a deferred file open.
    validate_tree(s, {"known": 1, "surprise": "anything", "deep": {"x": 9}})


# ── range mechanism (per-key DOMAIN bound only, never a safety assertion) ─────

def test_range_rejects_out_of_bounds_and_accepts_inside():
    """The range mechanism, exercised directly. The production schemas carry no
    ranges (safety ranges are assertion G), so this proves the capability on a
    synthetic spec.

    Mutation: widen the bound past the value => the red turns green, which is how
    a dropped range check would look. The inclusive edges must pass.
    """
    s = Schema("t", "test", {"p": num(lo=0.0, hi=1.0)})
    validate_tree(s, {"p": 0.0})       # inclusive low edge passes
    validate_tree(s, {"p": 1.0})       # inclusive high edge passes
    validate_tree(s, {"p": 0.5})       # interior passes
    with pytest.raises(SchemaError) as lo:
        validate_tree(s, {"p": -0.1})  # below low bound -> red
    assert "lower bound" in str(lo.value)     # message names which bound
    with pytest.raises(SchemaError) as hi:
        validate_tree(s, {"p": 1.1})   # above high bound -> red
    assert "upper bound" in str(hi.value)     # message names which bound


def test_range_zero_bound_is_not_dropped():
    """A bound of 0.0 is a real inclusive edge, not "unset". Guarding on
    truthiness (`if spec.lo:`) would drop it; this pins that it is honoured.
    """
    s = Schema("t", "test", {"p": num(lo=0.0)})
    with pytest.raises(SchemaError):
        validate_tree(s, {"p": -1.0})     # below 0.0 -> red
    validate_tree(s, {"p": 0.0})          # exactly 0.0 -> pass (edge honoured)


# ── malformed structure and self-consistency ─────────────────────────────────

def test_non_mapping_top_level_is_red():
    """A file whose root is a scalar or a list cannot carry keyed config."""
    for bad in ("just a string", [1, 2, 3], 42):   # scalar or list at the root
        with pytest.raises(SchemaError):
            validate_tree(Schema("t", "test", {}), bad)   # each is malformed -> red


def test_empty_or_none_tree_is_green_for_empty_schema():
    """The 14 deferred files: empty on disk, empty schema, must pass. None (a
    comment-only file) and {} both mean "nothing to check yet".
    """
    s = SCHEMAS["p1_motion.yaml"]
    assert s.fields == {}                 # deferred file: intentionally empty
    validate_tree(s, None)                # comment-only file parses to None
    validate_tree(s, {})                  # and an empty mapping also passes


def test_every_fieldspec_type_is_a_known_token():
    """Self-test: no schema was authored with a token outside TYPE_TOKENS. A typo
    like num spelled as a bare string would be caught here, not at validation.
    """
    for schema in SCHEMAS.values():           # every one of the 19 assets
        for path, spec in schema.fields.items():   # every declared field
            assert spec.type in TYPE_TOKENS, (schema.name, path, spec.type)  # legal token


def test_bad_token_raises_valueerror_not_schemaerror():
    """A schema with a nonsense type token is a programming error (ValueError),
    not a config failure (SchemaError), and it must RAISE rather than silently
    accept -- a mistyped token that returned False would accept every value.
    """
    with pytest.raises(ValueError):       # ValueError, not SchemaError
        _matches(5, "not_a_token")        # and it raises, never returns False


def test_validate_config_unknown_path_raises():
    """A config path with no registered schema raises, never passes. A silent
    pass would let a new config file ship unchecked.
    """
    with pytest.raises(SchemaError) as e:
        validate_config("configs_that_do_not_exist.yaml", {})   # no schema -> red
    assert e.value.code == E_CONFIG_INVALID    # reported as a config failure


def test_schema_error_is_an_xbrain_error():
    """SchemaError joins the XbrainError family so one except clause at the freeze
    call site catches it beside ConfigLayerError, while a real defect still
    escapes.
    """
    assert issubclass(SchemaError, XbrainError)   # one except clause covers both
    assert FieldSpec  # imported symbol is part of the asset surface, not dead
