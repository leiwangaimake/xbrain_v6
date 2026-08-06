"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_digest.py
Brief: common_digest and FenceSet crc32 -- golden vectors plus one case per rule

Description:
CFG-CM-10. Two kinds of case live here and they do different work.

The golden vectors are the cross-language contract. Their canonical strings are
hand-written from 10 S5.4.4 and 11 S9A.2 by scripts/gen_digest_golden.py, which
refuses to emit a file if the implementation disagrees with the hand-written
string -- so the file is not a recording of what the code happens to do. The C++
side is held to the same file by test_digest_cross_language.py.

The per-rule cases are what tell you WHICH rule broke. A golden vector that goes
red says only that some byte moved; six rules share one string and the diff can
be read, but a named case is faster and survives the vector set being rewritten.

*** On the mutations the TODO names for this item. Mutation 1 -- "change the key
sort to Unicode code point order, the golden set must go red" -- is NOT
achievable, and the substitution is documented in
test_key_sort_is_utf16_sensitive below. UTF-8 is order-preserving, so byte order
and code point order are the same ordering for every pair of strings; the
mutation is a no-op and could never have been run. The ordering that genuinely
differs is UTF-16 code unit order, and that is what is injected instead.

Mutation 2 -- drop priority from the crc32 recipe, then reorder two fences -- IS
achievable and is run in test_stacking_order_change_must_move_the_crc.
"""

import binascii
import hashlib
import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common.digest import (  # noqa: E402
    FenceRecipeError,
    UnresolvedTree,
    canonical_bytes,
    canonical_fence_string,
    canonical_json,
    common_digest,
    fence_crc32,
    verify_fence_crc32,
)

GOLDEN = os.path.join(ROOT, "tests", "common", "golden")


def _load(name):
    with open(os.path.join(GOLDEN, name), encoding="utf-8") as fh:
        return json.load(fh)["vectors"]


COMMON_VECTORS = _load("common_digest_vectors.json")
FENCE_VECTORS = _load("fence_crc32_vectors.json")


def _by_name(vectors, name):
    for vec in vectors:
        if vec["name"] == name:
            return vec
    raise AssertionError("no vector named %r -- the golden set was renamed "
                         "without updating this test" % name)


# -- golden vectors: the cross-language contract ------------------------------

@pytest.mark.parametrize("vec", COMMON_VECTORS, ids=lambda v: v["name"])
def test_common_digest_golden(vec):
    """*** The core assertion for the config side of CFG-CM-10."""
    assert canonical_json(vec["tree"].get("common", {})) == vec["canonical"], (
        "canonical string differs from the one hand-written in "
        "scripts/gen_digest_golden.py -- fix the implementation or the "
        "hand-written string, never regenerate the file to make this pass"
    )
    assert common_digest(vec["tree"]) == vec["expected"]


@pytest.mark.parametrize("vec", FENCE_VECTORS, ids=lambda v: v["name"])
def test_fence_crc32_golden(vec):
    """*** The core assertion for the fence side of CFG-CM-10."""
    assert canonical_fence_string(vec["fence"]) == vec["canonical"]
    assert fence_crc32(vec["fence"]) == vec["expected"]


def test_the_golden_files_are_not_self_generated():
    """Guards the guard: the vectors must carry their hand-written strings.

    A future edit that dropped the canonical field and kept only the expected
    hash would leave both parametrised tests above green while turning the file
    into a recording of whatever the implementation does. That is CLAUDE.md 3.2
    form 7 -- the conclusion defined into the premise.
    """
    for name in ("common_digest_vectors.json", "fence_crc32_vectors.json"):
        with open(os.path.join(GOLDEN, name), encoding="utf-8") as fh:
            doc = json.load(fh)
        assert doc["vectors"], name + " is empty"
        for vec in doc["vectors"]:
            assert vec.get("canonical"), (
                "%s vector %r has no hand-written canonical string"
                % (name, vec.get("name"))
            )
            assert vec.get("why"), (
                "%s vector %r does not say why it exists; a vector nobody can "
                "explain is one nobody dares change" % (name, vec.get("name"))
            )


# -- one case per rule of 10 S5.4.4 -------------------------------------------

def test_keys_are_sorted_not_insertion_ordered():
    """Mutation: drop the sort => red."""
    assert canonical_json({"z": 1, "a": 2}) == '{"a":2,"z":1}'


def test_key_sort_is_utf16_sensitive():
    """*** The substituted mutation 1, and why the TODO's version cannot run.

    UTF-8 is order-preserving. Sorting on k.encode("utf-8") and sorting on k
    itself produce the same sequence for every input, so the mutation the TODO
    names -- switch to code point order -- changes nothing and could never have
    been observed going red. Claiming otherwise would be exactly the failure
    mode CLAUDE.md 3.2 collects.

    What DOES differ is UTF-16 code unit order, which is the natural ordering in
    Java, JavaScript, C# and any C++ port keyed on std::wstring under Windows.
    U+10400 is the surrogate pair D801 DC00 there, and 0xD801 sorts below
    0xFF21, so the Deseret key moves ahead of the fullwidth A.

    This case injects that mutation and asserts the result is NOT what the
    implementation produces -- so the ordering rule is pinned against the only
    reimplementation that could plausibly get it wrong.
    """
    keys = {"é": 1, "Ａ": 2, "\U00010400": 3}

    # First: the premise. If these two orderings were the same, this case would
    # be asserting nothing at all.
    utf8_order = sorted(keys, key=lambda k: k.encode("utf-8"))
    utf16_order = sorted(keys, key=lambda k: k.encode("utf-16-be"))
    assert utf8_order != utf16_order, (
        "these keys no longer separate the two orderings; pick keys that do, "
        "otherwise the mutation below is a no-op"
    )

    # And the confirmation that the TODO's own mutation really is a no-op, so
    # nobody re-adds it later believing it covers something.
    assert utf8_order == sorted(keys), "UTF-8 byte order IS code point order"

    mutant = "{" + ",".join('"%s":%d' % (k, keys[k]) for k in utf16_order) + "}"
    assert canonical_json(keys) != mutant, "the UTF-16 mutation must not pass"
    assert canonical_json(keys) == (
        "{" + ",".join('"%s":%d' % (k, keys[k]) for k in utf8_order) + "}")


def test_no_whitespace_anywhere():
    """Mutation: use json.dumps defaults => red on the ', ' and ': ' separators."""
    text = canonical_json({"a": {"b": [1, 2]}, "c": "d"})
    assert " " not in text and "\n" not in text and "\t" not in text


def test_non_ascii_is_emitted_directly():
    """Mutation: ensure_ascii=True => red.

    Asserted both ways: the escape must be absent AND the character present. The
    absence alone would also hold for an implementation that dropped the
    character entirely.
    """
    text = canonical_json({"site": "场区"})
    assert "\\u" not in text
    assert "场区" in text


def test_integers_have_no_decimal_point():
    """Mutation: route every number through the float path => red."""
    assert canonical_json({"n": 3}) == '{"n":3}'


def test_integral_float_renders_as_an_integer():
    """2.0 and 2 must agree.

    This is the half of the number rule that a Python-only reading misses: the
    two are the same value here, but on the C++ side they are different types
    reaching different formatters, and only a stated rule keeps them equal.
    """
    assert canonical_json({"n": 2.0}) == canonical_json({"n": 2})


def test_float_trailing_zeros_are_stripped():
    assert canonical_json({"a": 2.50}) == '{"a":2.5}'


def test_the_trailing_zero_strip_is_unreachable_under_the_current_format():
    """*** Read this before trusting the strip step to be covered by anything.

    10 S5.4.4 asks for two things: print via %.17g, then remove trailing zeros.
    The second is a no-op given the first. C99 7.21.6.1 has %g remove trailing
    zeros from the fractional part unless the # flag is given, so no double
    reaches the strip with anything to strip.

    That was found by mutation, not by reading: deleting the strip from the C++
    header left the whole cross-language suite green, which is the signature of
    dead code. A sweep of several hundred thousand random doubles plus a decimal
    sweep found no value that reaches it either.

    The strip is kept in both implementations because the section says to do it
    and because it is what makes the rule survive a future change of format. But
    "kept and covered by a golden vector" would be false, and this case exists so
    the gap is stated rather than assumed away. It is written as a property so it
    fails the moment the premise stops holding -- change the format to %.17f and
    this goes red and tells you a vector is now needed.
    """
    from xbrain.common.digest import format_number

    reachable = []
    for scaled in range(0, 20000):
        value = scaled / 100.0
        text = "%.17g" % value
        if "." in text and "e" not in text and text.rstrip("0") != text:
            reachable.append(value)
    assert not reachable, (
        "the strip is now reachable for %r -- add a golden vector covering it, "
        "because until now nothing did" % (reachable[:3],)
    )
    # And the strip itself still behaves, so removing it later is a deliberate
    # act rather than something that quietly stops mattering.
    assert format_number(2.5) == "2.5"
    assert format_number(2.0) == "2"


def test_float_keeps_all_seventeen_digits_when_it_needs_them():
    """Mutation: use repr() or %g => red.

    repr(0.1) is "0.1", which round-trips in Python and is a DIFFERENT string
    from what a C++ %.17g produces. Both denote the same double; only one can be
    hashed.
    """
    assert canonical_json({"eps": 0.1}) == '{"eps":0.10000000000000001}'


def test_lists_keep_their_order():
    """Mutation: sort list elements alongside map keys => red.

    R-5 makes a list a unit replaced whole, and qos.bindings is first-match-wins,
    so a reordering is a behaviour change the digest has to see.
    """
    assert canonical_json({"b": ["c", "a", "b"]}) == '{"b":["c","a","b"]}'


def test_bool_is_not_rendered_as_a_number():
    """Mutation: check isinstance(v, int) before isinstance(v, bool) => red.

    bool subclasses int in Python, so the unguarded order writes true as 1. C++
    writes true. Neither language's own tests can see the disagreement.
    """
    assert canonical_json({"t": True, "f": False}) == '{"f":false,"t":true}'
    assert canonical_json({"t": True}) != canonical_json({"t": 1})


def test_null_survives_into_the_digest():
    """An unassigned safety parameter is null (CLAUDE.md 3.1).

    If null were dropped, assigning a value to a previously-null key would not
    move the digest, and the freeze line would report the stack as unchanged
    across exactly the edit that matters most.
    """
    before = common_digest({"common": {"safety": {"a_mps2": None}}})
    after = common_digest({"common": {"safety": {"a_mps2": 2.5}}})
    assert before != after


# -- the "only common.*" boundary ---------------------------------------------

def test_private_sections_do_not_enter_the_digest():
    """*** 10 S5.4.4 states the reason: a private edit must not block release."""
    bare = {"common": {"z": 1, "a": 2, "m": 3}}
    with_private = dict(bare, p1_motion={"loop_hz": 20}, p5_gateway={"port": 8080})
    assert common_digest(bare) == common_digest(with_private)


def test_a_change_inside_common_does_move_the_digest():
    """Pairs with the test above.

    An implementation that hashed the empty object regardless of input would
    satisfy every exclusion test here -- CLAUDE.md 3.2 form 1, an assertion a
    do-nothing implementation passes. This is the positive half.
    """
    assert (common_digest({"common": {"a": 1}})
            != common_digest({"common": {"a": 2}}))


def test_absent_common_section_has_a_defined_digest():
    """{} rather than an exception, so no caller has to invent a special case."""
    assert common_digest({}) == common_digest({"common": {}})
    assert len(common_digest({})) == 16


def test_digest_is_sixteen_lowercase_hex_characters():
    digest = common_digest({"common": {"a": 1}})
    assert len(digest) == 16
    assert all(c in "0123456789abcdef" for c in digest)


def test_digest_is_the_prefix_of_the_full_sha256():
    """Pins WHICH sixteen characters.

    The last sixteen, or a hash of the hash, would satisfy every other case
    here -- each is a plausible reading of "take sixteen characters", and a C++
    author reading only the code could pick either.
    """
    tree = {"common": {"a": 1, "b": [2, 3]}}
    full = hashlib.sha256(canonical_bytes(tree)).hexdigest()
    assert common_digest(tree) == full[:16]


def test_unresolved_references_are_refused():
    """Mutation: hash the tree as-is => red.

    Digesting unresolved text fingerprints the reference rather than its value,
    so two configurations differing only in what an alias points at would share a
    digest. The message must name the key, for the reason 10 S5.4.3 gives about
    cycle reports.
    """
    with pytest.raises(UnresolvedTree) as exc:
        common_digest({"common": {"a": {"b": "${common.c}"}}})
    assert "common.a.b" in str(exc.value)


def test_unresolved_reference_inside_a_list_is_also_refused():
    """Lists are walked too.

    merge.flatten treats a list as a leaf for R-5, and inheriting that here would
    let a reference hide inside one -- the check would then pass on precisely the
    tree it exists to reject.
    """
    with pytest.raises(UnresolvedTree) as exc:
        common_digest({"common": {"xs": [1, "${common.y}"]}})
    assert "common.xs[1]" in str(exc.value)


def test_a_resolved_tree_is_not_refused():
    """Pairs with the two above: a checker that raised on every string would
    satisfy both and make every configuration undigestable."""
    common_digest({"common": {"a": "a plain string with no dollar brace"}})


# -- 11 S9A.2: the fence recipe ------------------------------------------------

def test_stacking_order_change_must_move_the_crc():
    """*** Mutation 2 from the TODO, run against a recipe with priority removed.

    The two vectors differ only in which of two overlapping forbid polygons is
    painted first. With priority in the recipe their checksums differ. The
    injected mutant below strips priority exactly as the v0.2 recipe did, and the
    case asserts the mutant COLLIDES -- which is what makes this a mutation that
    was observed going red rather than one that was described.

    Why that collision is the dangerous kind: S-5 answers a repeat commit of the
    same fence_set_id + rev + crc32 with duplicate, and op=ping reconciles on the
    same triple. So the reordering does not take effect, and both mechanisms
    report everything as fine.
    """
    a = _by_name(FENCE_VECTORS, "two_polygons_stacking_order_a")["fence"]
    b = _by_name(FENCE_VECTORS, "two_polygons_stacking_order_b")["fence"]
    assert fence_crc32(a) != fence_crc32(b), "priority must be in the recipe"

    def v02_recipe(fence):
        """The v0.2 recipe: geometry and role only, no priority, no limit."""
        out = "%s|%s|" % (fence["fence_set_id"], fence["rev"])
        for poly in fence["polygons"]:
            out += "%s|%s|%s|%s|" % (poly["poly_id"], poly["role"],
                                     poly["winding"],
                                     "1" if poly["hard_enforce"] else "0")
            for vertex in poly["vertices"]:
                out += "%.8f,%.8f;" % (vertex["lat"], vertex["lon"])
        return "%08x" % (binascii.crc32(out.encode("utf-8")) & 0xFFFFFFFF)

    assert v02_recipe(a) == v02_recipe(b), (
        "the mutant must collide; if it does not, this case is no longer "
        "testing what it claims and the collision it guards against has moved"
    )


def test_lowering_a_speed_limit_must_move_the_crc():
    """*** The second half of the same defect, and the one S9A.2 spells out.

    Identical geometry, limit 1.0 -> 0.5. Under the v0.2 recipe the two are
    byte-identical, so a site lowering its slow zone would get duplicate back
    from commit and keep driving at the old limit.
    """
    a = _by_name(FENCE_VECTORS, "speed_limit_polygon_carries_its_limit")["fence"]
    b = _by_name(FENCE_VECTORS, "lowered_speed_limit_changes_the_crc")["fence"]
    assert a["polygons"][0]["vertices"] == b["polygons"][0]["vertices"]
    assert fence_crc32(a) != fence_crc32(b)


def test_renaming_a_polygon_does_not_move_the_crc():
    """S9A.2 keeps name out on purpose.

    The opposite of the two cases above, and needed for the same reason they
    are: a recipe that hashed the whole polygon object would pass both of them
    and make every HMI rename trigger a stack-wide stage and commit.
    """
    a = _by_name(FENCE_VECTORS, "single_allow_polygon")["fence"]
    b = _by_name(FENCE_VECTORS, "name_is_not_in_the_recipe")["fence"]
    assert a["polygons"][0]["name"] != b["polygons"][0]["name"]
    assert fence_crc32(a) == fence_crc32(b)


def test_a_stray_speed_limit_on_another_role_is_ignored():
    """The field is read only when role is speed_limit.

    S9A.2 makes it required only for that role, so a value left on a zone
    polygon is legal input. Reading it unconditionally would give two fences
    that behave identically two different checksums, and every consumer would
    reject the frame with E_SCHEMA against a fence that is perfectly valid.
    """
    base = {"fence_set_id": "s", "rev": 1, "polygons": [{
        "poly_id": "f-z", "role": "zone", "priority": 0, "winding": "ccw",
        "hard_enforce": False, "vertices": [{"lat": 1.0, "lon": 2.0}]}]}
    stray = json.loads(json.dumps(base))
    stray["polygons"][0]["speed_limit_mps"] = 9.9
    assert fence_crc32(base) == fence_crc32(stray)


def test_empty_field_is_not_a_zero():
    """A non-speed_limit role writes '', not '0.000'.

    Writing a zero would collide with a genuine speed_limit polygon whose limit
    really is zero -- a stop zone -- and the two mean opposite things.
    """
    text = canonical_fence_string(
        _by_name(FENCE_VECTORS, "single_allow_polygon")["fence"])
    assert "|0||" in text, "priority 0 then an EMPTY speed field"
    assert "0.000" not in text


def test_hard_enforce_is_one_or_zero_not_true_or_false():
    text = canonical_fence_string(
        _by_name(FENCE_VECTORS, "hard_enforce_false_writes_zero")["fence"])
    assert "true" not in text and "false" not in text


def test_vertices_are_never_sorted():
    """A rotation of the same triangle must give a different checksum.

    Winding is what distinguishes inside from outside, so a sorted recipe would
    call a fence and its inverse identical -- and would do it silently, since
    both are well-formed polygons.
    """
    a = _by_name(FENCE_VECTORS, "single_allow_polygon")["fence"]
    b = _by_name(FENCE_VECTORS, "vertex_order_is_not_sorted")["fence"]
    assert sorted(map(str, a["polygons"][0]["vertices"])) == \
        sorted(map(str, b["polygons"][0]["vertices"])), "same vertex set"
    assert fence_crc32(a) != fence_crc32(b)


def test_rev_is_in_the_recipe():
    """S-5 keys idempotency on fence_set_id + rev + crc32.

    The two fences are compared by the POLYGON PORTION of their canonical
    strings rather than by their dicts: one of them carries a name and the other
    does not, and name is deliberately outside the recipe. Comparing the dicts
    would fail on a difference the recipe is supposed to ignore, which is the
    opposite of what this case is about.
    """
    a = _by_name(FENCE_VECTORS, "single_allow_polygon")["fence"]
    b = _by_name(FENCE_VECTORS, "rev_bump_changes_the_crc")["fence"]
    tail_a = canonical_fence_string(a).split("|", 2)[2]
    tail_b = canonical_fence_string(b).split("|", 2)[2]
    assert tail_a == tail_b, "everything after the head must be identical"
    assert fence_crc32(a) != fence_crc32(b)


def test_negative_priority_keeps_its_sign():
    text = canonical_fence_string(
        _by_name(FENCE_VECTORS, "negative_priority_keeps_its_sign")["fence"])
    assert "|-5|" in text


def test_crc32_is_eight_lowercase_hex_characters():
    """FV-8 compares against a transmitted string; case is not cosmetic.

    An upper-case implementation would fail every frame while its value, read in
    a log, looks entirely correct.
    """
    value = fence_crc32(_by_name(FENCE_VECTORS, "single_allow_polygon")["fence"])
    assert len(value) == 8
    assert all(c in "0123456789abcdef" for c in value)


def test_crc32_is_ieee_802_3_and_not_a_neighbouring_variant():
    """Pins WHICH crc32.

    The name alone identifies at least four incompatible checksums. This is the
    published check value for IEEE 802.3 over the nine bytes "123456789"; a
    bzip2, POSIX cksum or Castagnoli implementation gives a different one and
    every other case in this file would still pass.
    """
    assert "%08x" % (binascii.crc32(b"123456789") & 0xFFFFFFFF) == "cbf43926"


def test_verify_accepts_a_matching_crc_and_rejects_a_wrong_one():
    fence = dict(_by_name(FENCE_VECTORS, "single_allow_polygon")["fence"])
    fence["crc32"] = fence_crc32(fence)
    assert verify_fence_crc32(fence)
    fence["crc32"] = "deadbeef"
    assert not verify_fence_crc32(fence)


def test_verify_accepts_an_uppercase_transmitted_value():
    """Tolerant on input, strict on output.

    S9A.2 specifies lower case for what we emit; rejecting an upper-case value
    from a peer that is otherwise correct would turn a cosmetic difference into a
    rejected fence.
    """
    fence = dict(_by_name(FENCE_VECTORS, "single_allow_polygon")["fence"])
    fence["crc32"] = fence_crc32(fence).upper()
    assert verify_fence_crc32(fence)


def test_missing_required_field_is_refused_by_name():
    """Malformed input raises, and the message says which field and which polygon.

    "invalid FenceSet" would leave an operator comparing a thirty-polygon
    document against the section by hand.
    """
    with pytest.raises(FenceRecipeError) as exc:
        canonical_fence_string({"fence_set_id": "s", "rev": 1, "polygons": [
            {"poly_id": "f-a", "role": "allow", "winding": "ccw",
             "hard_enforce": True, "vertices": []}]})
    assert "priority" in str(exc.value) and "f-a" in str(exc.value)


def test_speed_limit_role_without_a_limit_is_refused():
    with pytest.raises(FenceRecipeError):
        canonical_fence_string({"fence_set_id": "s", "rev": 1, "polygons": [
            {"poly_id": "f-a", "role": "speed_limit", "winding": "ccw",
             "hard_enforce": False, "priority": 0, "vertices": []}]})


def test_a_string_hard_enforce_is_refused_not_coerced():
    """'false' is a truthy string.

    int(bool('false')) is 1, so a coercing implementation would harden a fence
    that was declared soft -- and would produce a checksum both sides agree on,
    so nothing downstream would notice.
    """
    with pytest.raises(FenceRecipeError):
        canonical_fence_string({"fence_set_id": "s", "rev": 1, "polygons": [
            {"poly_id": "f-a", "role": "allow", "winding": "ccw",
             "hard_enforce": "false", "priority": 0, "vertices": []}]})


def test_vertices_accept_both_wire_and_row_shapes():
    """{lat, lon} and a two-element pair must give the same string.

    A DAO returning rows and a Zenoh payload returning objects describe the same
    geometry; forcing one caller to convert would put a second, untested
    rendering of the same coordinates into the code base.
    """
    obj = {"fence_set_id": "s", "rev": 1, "polygons": [
        {"poly_id": "f-a", "role": "allow", "winding": "ccw",
         "hard_enforce": True, "priority": 0,
         "vertices": [{"lat": 1.5, "lon": 2.5}]}]}
    pair = json.loads(json.dumps(obj))
    pair["polygons"][0]["vertices"] = [[1.5, 2.5]]
    assert fence_crc32(obj) == fence_crc32(pair)


def test_no_tally_is_written_into_the_golden_files():
    """CLAUDE.md 3.7: a count maintained by hand is a count that goes stale."""
    for name in ("common_digest_vectors.json", "fence_crc32_vectors.json"):
        with open(os.path.join(GOLDEN, name), encoding="utf-8") as fh:
            doc = json.load(fh)
        assert "count" not in doc and "total" not in doc
