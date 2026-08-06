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
  list / array   ★ WHOLE-TABLE REPLACEMENT (R-5), never element-wise merge
  null           ★ "key position declared, value not assigned" -- does NOT
                 override the lower layer
  missing        does not participate at all -- semantically different from null,
                 because startup assertion A reports null as "unassigned" and has
                 nothing to report for a key nobody mentioned

★★★ The null/missing distinction is the whole reason this file is separate from
a three-line dict.update(). 10 S5.4.3 uses it deliberately: common.yaml declares
enu_origin as null so the site layer can fill it, and assertion A can tell "left
for the site to fill" apart from "nobody ever thought about this key".

🚫 There is no default-value syntax anywhere in the overlay axis (R-3 forbids
${a:-b}). A merge that invents a value when none of the layers supplied one is
the fail-silent path CLAUDE.md 3.1 exists to prevent.
"""

from typing import Any, Dict

#: Sentinel for "this key was not mentioned by any layer".
#: Distinct from None, which means "declared but unassigned".
MISSING = object()


def deep_merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    """Merge overlay onto base per 10 S5.4.3. Neither input is mutated.

    The three rules, in the order the section states them:

    * both sides map      -> recurse
    * overlay value None  -> keep the base value (null does not override)
    * anything else       -> overlay wins outright, and for lists that means the
                             whole list is replaced, not zipped or extended
    """
    out = dict(base)
    for key, value in overlay.items():
        if value is None:
            # null declares the key position without assigning it. If the lower
            # layer has a value, that value survives; if it does not, the key
            # stays present-and-null so assertion A can report it by path.
            if key not in out:
                out[key] = None
            continue
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
            continue
        # Lists land here on purpose: R-5 whole-table replacement. Element-wise
        # merging would silently reorder qos.bindings, whose order is meaningful.
        out[key] = value
    return out


def flatten(tree: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    """Dotted key path -> leaf value, for namespace checks and assertion A.

    A leaf is anything that is not a dict -- including a list, which is a leaf
    because R-5 replaces it as a unit, and including None, which must stay
    visible so "declared but unassigned" can be reported by its full path.
    """
    out: Dict[str, Any] = {}
    for key, value in tree.items():
        path = f"{prefix}{key}"
        if isinstance(value, dict) and value:
            out.update(flatten(value, path + "."))
        else:
            out[path] = value
    return out


def unflatten(flat: Dict[str, Any]) -> Dict[str, Any]:
    """Inverse of flatten. Used to build layer trees from dotted test fixtures."""
    out: Dict[str, Any] = {}
    for path, value in flat.items():
        node = out
        parts = path.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
            if not isinstance(node, dict):
                raise ValueError(f"path {path!r} collides with a scalar at {part!r}")
        node[parts[-1]] = value
    return out
