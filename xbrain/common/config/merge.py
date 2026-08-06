"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: merge.py
Brief: The three merge rules of the L0~L5 overlay axis

Description:
10 S5.4.3 defines exactly three behaviours and they are not interchangeable:

  map / object   recursive deep merge, later leaf wins; keys an upper layer does
                 not mention keep the lower layer's value
  list / array   * WHOLE-TABLE REPLACEMENT (R-5), never element-wise merge
  null           * "key position declared, value not assigned" -- does NOT
                 override the lower layer
  missing        does not participate at all -- semantically different from null,
                 because startup assertion A reports null as "unassigned" and has
                 nothing to report for a key nobody mentioned

*** The null/missing distinction is the whole reason this file is separate from
a three-line dict.update(). 10 S5.4.3 uses it deliberately: common.yaml declares
enu_origin as null so the site layer can fill it, and assertion A can tell "left
for the site to fill" apart from "nobody ever thought about this key".

Collapsing the two does not lose a nicety, it loses the only machine-readable
signal that a site layer forgot something. Assertion A's scope is verbatim
"residual ${, unresolved path, null unassigned" -- it inspects the VALUE and
never asks whether the key exists at all. Assertion M (10 S5.4.4, added
2026-08-05) covers the other half, and it can only do so because this module
keeps the two states apart in the first place.

!! There is no default-value syntax anywhere in the overlay axis (R-3 forbids
${a:-b}). A merge that invents a value when none of the layers supplied one is
the fail-silent path CLAUDE.md 3.1 exists to prevent.

*** WARNING -- null here is the OPPOSITE of JSON Merge Patch (RFC 7386), which
is the model most readers already have in their fingers: there, null DELETES the
key. Here null is the weakest possible write. It cannot remove anything and it
cannot override anything, and the overlay axis has no deletion operator at all.
That is deliberate: a layer able to delete a key would make assertion A's key
list depend on which layers happened to be present, and a deleted safety key is
indistinguishable from a key nobody ever declared.

Scope -- what this module is NOT. It is the three rules and nothing else. It
knows nothing about layers, namespace allowances, provenance, ${common.*}
expansion or the startup assertions; those live in layers.py, __init__.py and
the reference axis (CFG-CM-8). Do not add a "just this once" policy check here:
the rule would then exist in two places, and the copy nobody remembers is the
one that stops matching. That is also why the only exception raised below is a
plain ValueError -- this module has no configuration vocabulary to report in.

Aliasing. deep_merge copies only the nodes it actually walks, so branches that
no upper layer mentioned are SHARED with the input trees rather than cloned.
Read the result; do not mutate it in place. Writing into the returned tree
reaches back into the caller's layer dict and silently changes what a later
reader of that same layer sees.
"""

from typing import Any, Dict

# Why a bare object() and not None, "" or a string such as "<missing>": each of
# those is a value some layer could legitimately write, so a caller could not
# tell "absent" from "present and happens to equal the sentinel". object() has
# no YAML spelling, which is exactly the point -- it cannot arrive from a
# config file.
#
# Compare with `is`, never `==`; identity is the entire contract.
# NEVER let MISSING into a tree that gets merged or written to
# /run/xbrain/resolved. It has no serialisation, and deep_merge would carry it
# through as an ordinary value. It is a return value only.
MISSING = object()


# "Neither input is mutated" below is a statement about writes, not about
# copying. It is NOT a deep copy; see the module docstring on aliasing before
# mutating the result.
def deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    """Merge overlay onto base per 10 S5.4.3. Neither input is mutated.

    map over map recurses; an overlay None keeps the base value; anything else
    replaces outright, lists included (R-5 whole-table replacement).
    """
    out = dict(base)
    for key, value in overlay.items():
        if value is None:
            # null declares the key position without assigning it. If the lower
            # layer has a value, that value survives; if it does not, the key
            # stays present-and-null so assertion A can report it by path.
            #
            # Guard on membership, not truthiness. 0, "", [] and False are all
            # legitimate configured values, and `if not out.get(key)` would let
            # a null from an upper layer wipe them. That is the same confusion
            # between "zero" and "unset" that CLAUDE.md 3.1 attacks from the
            # other side when it forbids writing 0.0 for an uncalibrated
            # parameter.
            if key not in out:
                out[key] = None
            continue
        # Both halves of this conjunction are required, and the overlay half is
        # the one that looks droppable: `isinstance(out.get(key), dict)` alone
        # reads as equivalent, on the reasoning that a non-map overlay value
        # would obviously just replace. Measured, it does not replace -- it
        # enters the recursion and dies there, `AttributeError: 'int' object has
        # no attribute 'items'`, on a config file that is merely laying a scalar
        # over a block. The overlay-side test is what routes those to the
        # replace path below.
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
            continue
        # Anything that is not map-over-map replaces outright, including a map
        # over a scalar and a scalar over a map. There is no shape
        # reconciliation, on purpose: a node half-merged from two different
        # shapes is a shape no layer author wrote, so no reviewer of any single
        # layer file could have caught it.
        #
        # Lists land here too, and that is R-5 whole-table replacement.
        # NEVER add an isinstance(value, list) branch that extends, zips or
        # merges by index. 11 S2.4.7 makes qos.bindings a first-match-wins
        # ordered table: the Q0_safety rules for cmd/estop are written first and
        # the catch-all "xbrain/*/**" -> Q3_cmd is written last. Any element-wise
        # rule that can place an upper layer's entries after that catch-all sends
        # the estop keys to Q3_cmd, whose congestion_control is "block". 10 R-5
        # names the outcome outright: the emergency-stop link is silently
        # downgraded. It has already happened once for a different reason
        # (11 S2.2.12, fixed in v0.3), and nothing reports it until the day
        # something actually has to stop.
        out[key] = value
    return out


# `and value` in the recursion test makes an EMPTY dict a leaf. The obvious
# isinstance(value, dict) on its own would recurse into it, yield nothing, and
# the branch would vanish from the key set entirely -- so neither
# check_namespace nor build_overlay's provenance pass would ever learn that the
# layer wrote there. A key that disappears between the file and the checks is
# the one class of defect this module exists to prevent.
#
# Key names must not contain a literal dot. Nothing escapes it here, and
# unflatten would split such a name back apart into two levels.
def flatten(tree: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    """Dotted key path -> leaf value, for namespace checks and assertion A.

    A leaf is anything that is not a dict: a list is a leaf because R-5 replaces
    it as a unit, and None is a leaf because "declared but unassigned" has to
    stay reportable by its full path.
    """
    out: Dict[str, Any] = {}
    for key, value in tree.items():
        path = f"{prefix}{key}"
        if isinstance(value, dict) and value:
            out.update(flatten(value, path + "."))
        else:
            out[path] = value
    return out


# Not test-only, whatever the "fixture" reading of the name suggests:
# build_overlay runs L5 through here, because env_overlay yields dotted keys
# (common.robot_id) while deep_merge only speaks trees. Changing the signature
# or the raising behaviour breaks the environment layer, not just the tests.
#
# NOTE the collision check is one-sided: the final segment is assigned without
# it, so {"a": 1, "a.b": 2} raises while {"a.b": 2, "a": 1} quietly yields
# {"a": 1}. It does not bite today because neither producer can emit a key that
# is a prefix of another -- env_overlay draws from the fixed ENV_KEY_MAP, and
# flatten emits leaf paths only. Hand-written dicts are the case to watch.
def unflatten(flat: Dict[str, Any]) -> Dict[str, Any]:
    """Inverse of flatten: dotted keys -> nested tree.

    Raises ValueError if a path runs through an existing scalar. Replacing that
    scalar silently would drop it with no record, and afterwards nobody can tell
    whether the layer ever set it.
    """
    out: Dict[str, Any] = {}
    for path, value in flat.items():
        node = out
        parts = path.split(".")
        for part in parts[:-1]:
            # setdefault returns whatever was already stored, so the isinstance
            # test on the next line is what detects the collision. It is not a
            # redundant type guard on a dict we just created.
            node = node.setdefault(part, {})
            if not isinstance(node, dict):
                raise ValueError(f"path {path!r} collides with a scalar at {part!r}")
        node[parts[-1]] = value
    return out
