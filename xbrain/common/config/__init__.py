"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The L0~L5 overlay axis of the configuration loader

Description:
CFG-CM-7. This module runs the overlay axis only. Two things it deliberately
does NOT do:

  * expand ${common.*} references -- that is the reference axis (CFG-CM-8)
  * detect reference cycles -- CFG-CM-9

*** The order is not negotiable. 10 S5.4.3 requires the whole L0~L5 overlay to
finish BEFORE any reference is expanded. Expanding while merging reads a value
the site layer has not overridden yet, and the failure is invisible whenever the
lab site and the field site happen to agree -- so it passes every test written
in the lab and goes wrong on exactly one robot.

* Layer L6 (process-private) sits after the freeze line and is not part of this
module either: the freeze line computes common_digest over the finished common.*
tree, and L6 may only reference it, never contribute to it.
"""

from typing import Any, Dict, List, Optional, Tuple

from .layers import (ENV_KEY_MAP, ENV_WHITELIST, LAYERS, ConfigLayerError, Layer,
                     check_namespace, env_overlay, resolve_config_root, safety_root)
from .merge import MISSING, deep_merge, flatten, unflatten

__all__ = ["ConfigLayerError", "Layer", "LAYERS", "ENV_WHITELIST", "ENV_KEY_MAP",
           "MISSING", "deep_merge", "flatten", "unflatten", "resolve_config_root",
           "safety_root", "env_overlay", "check_namespace", "build_overlay",
           "OverlayResult"]


class OverlayResult:
    """The merged common.* tree plus where every leaf came from.

    Provenance is not a nicety. When assertion A reports a key as unassigned, the
    first question is which layer was supposed to fill it; without provenance the
    answer is a manual grep across six files.
    """

    __slots__ = ("tree", "provenance")

    def __init__(self, tree: Dict[str, Any], provenance: Dict[str, str]):
        self.tree = tree
        self.provenance = provenance

    def get(self, dotted: str, default: Any = MISSING) -> Any:
        """Leaf by dotted path.

        * Returns MISSING, not None, for a key no layer mentioned. None is a real
        value here -- it means "declared but unassigned" -- so collapsing the two
        would hide exactly the distinction 10 S5.4.3 draws.
        """
        node: Any = self.tree
        for part in dotted.split("."):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node

    def unassigned(self) -> List[str]:
        """Dotted paths whose value is null -- assertion A's input.

        These are keys some layer declared and no layer filled. A key nobody
        mentioned is not in this list, which is the point of the null/missing
        split: only the first kind is a configuration gap someone intended.
        """
        return sorted(k for k, v in flatten(self.tree).items() if v is None)


def build_overlay(layer_trees: Dict[str, Dict[str, Any]],
                  env: Optional[Dict[str, str]] = None,
                  check_namespaces: bool = True) -> OverlayResult:
    """Run L0~L5 in order and return the merged tree with provenance.

    `layer_trees` maps a layer name (L0, L1, L2, L3, L4, L4b) to its already-read
    tree. L5 is not passed in: it comes from the environment, through the
    whitelist in layers.py, so a caller cannot smuggle a fourth variable in by
    building the dict themselves.

    * Namespace checking is on by default and the flag exists only so a test can
    exercise the merge rules in isolation. !! Do not turn it off in production
    code: the allowances are what stop L2 from redefining a safety parameter.
    """
    merged: Dict[str, Any] = {}
    provenance: Dict[str, str] = {}

    for layer in LAYERS:
        if layer.name == "L5":
            flat_env = env_overlay(env)
            tree = unflatten(flat_env) if flat_env else {}
        else:
            tree = layer_trees.get(layer.name) or {}
        if not tree:
            continue
        flat = flatten(tree)
        if check_namespaces:
            check_namespace(layer, flat.keys())
        merged = deep_merge(merged, tree)
        # Record provenance only for leaves this layer actually decided. A null
        # does not override, so it must not claim authorship of the value below.
        for key, value in flat.items():
            if value is not None or key not in provenance:
                provenance[key] = layer.name

    return OverlayResult(merged, provenance)
