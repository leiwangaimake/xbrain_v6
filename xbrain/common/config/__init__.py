"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The L0~L5 overlay axis of the configuration loader

Description:
CFG-CM-7. This module runs the overlay axis only -- the half of 10 S5.4.1 that
answers "the same parameter takes a different value on a different model or a
different site". It ends at the freeze line and not one step past it.

10 S5.4.1 (anchor: "两条正交的轴") is emphatic that the two axes are orthogonal,
because an earlier revision drew them on one vertical line and common.yaml then
looked like yet another layer stacked ON TOP of the site file. The three bands
below are what replaced that picture:

  overlay axis (L0~L5)   same parameter, different value per model / per site.
                         Same key, later layer wins. THIS MODULE.
  freeze line            common.* is final here, common_digest is computed over
                         it, everything downstream is read-only.
  reference axis (L6)    same value read by several processes. Process-private
                         files only take values, never define them. NOT here.

Four things this module deliberately does NOT do. Each of them has its own task
id, and putting any of them inside the loop below would be a design change, not
a refactor:

  * expand ${common.*} references -- reference axis, CFG-CM-8, R-1~R-7
  * detect reference cycles -- CFG-CM-9, three-colour DFS, and it has to run
    BEFORE the freeze line rather than after it
  * validate types and ranges -- CFG-FZ-17 schema check. It runs before
    assertion A, and its own mutation test moves it after assertion G to show
    what that buys: G then dies in a TypeError instead of naming a key path
  * read files. The caller hands in already-parsed layer trees. That is what
    makes the whole axis testable with no configs/ tree on disk, and it is also
    why resolve_config_root lives in layers.py and is not called from here.

*** Why the reference axis MUST run after the overlay axis finishes.

This is the one thing to understand before touching anything in this file.
10 S5.4.3 requires the entire L0~L5 overlay to complete before a single
reference is expanded. Expanding while merging resolves a reference against
whatever the tree happens to hold at that instant, which is a value some later
layer has not overridden yet. Worked example on the key 10 S5.4.3 picks for
exactly this, both in its null row and in the unit test it recommends there:

    L1  common.yaml        common: {geo: {enu_origin: null}}
    L4  sites/lab.yaml     common: {geo: {enu_origin: {lat: 31.0, ...}}}
    L6  p1_motion.yaml     enu_origin: ${common.geo.enu_origin}

  expand while merging   L6 resolves the moment L1 is folded in, and gets null
                         -- or worse, whatever the previously merged site left
                         behind, which is a real number and looks correct
  expand after freeze    L6 resolves against the finished tree, and gets L4

*** And the wrong order is invisible in the lab. Wherever the lab site file and
the field site file happen to agree on a key -- and they agree on most keys,
that is the whole point of having a site layer -- both orders produce the same
number. Every test written in the lab passes. Exactly one robot, on exactly one
site, is wrong. No test added afterwards catches this from the outside, because
from the outside the loader returned a plausible value. The ordering IS the
defence; there is no second line of it.

* Same reasoning, one level up: 10 S5.4.1 calls "引用不由各进程各自展开" the
most critical structural decision in the configuration design. Expansion happens
once, in the xbrain-config-freeze oneshot at Stage 0c, and every process then
reads the same /run/xbrain/resolved/{proc}.yaml snapshot. The failure mode
"各进程解析出不同结果" is removed by construction instead of being detected at
run time. NEVER move expansion into the loop below however convenient it looks
locally, and NEVER add a per-process expand path -- CLAUDE.md 3.6 is the same
rule stated as a coding constraint.

* Layer L6 sits after the freeze line and is not part of this module either. R-6
rejects a common top-level key in a process-private file, so L6 can only consume
what this module produced. One consequence for callers: the tree returned here
is the input to common_digest. Mutating it after build_overlay returns yields a
digest that describes a tree no process ever read, and the digest is what CFG-41
audits, so the corruption would be recorded as if it were the truth.
"""

from typing import Any, Dict, List, Optional, Tuple

from .duplicates import check_l2_not_copy_of_l1
from .layers import (ENV_KEY_MAP, ENV_WHITELIST, LAYERS, ConfigLayerError, Layer,
                     check_namespace, env_overlay, resolve_config_root, safety_root)
from .merge import MISSING, deep_merge, flatten, unflatten

# The public surface of the overlay axis. check_namespace and LAYERS are
# exported because the freeze service and the CFG-FZ-16 per-layer tests drive
# them directly rather than through build_overlay. MISSING is exported because a
# caller that reaches for None instead has already thrown away the
# declared-but-unassigned distinction this whole module is built around.
# check_l2_not_copy_of_l1 is assertion B's U76(9) sub-clause (CFG-FZ-16): it runs
# on the per-layer trees AFTER build_overlay, not inside it, because it has to see
# L1 and L2 apart and build_overlay has already merged them.
__all__ = ["ConfigLayerError", "Layer", "LAYERS", "ENV_WHITELIST", "ENV_KEY_MAP",
           "MISSING", "deep_merge", "flatten", "unflatten", "resolve_config_root",
           "safety_root", "env_overlay", "check_namespace", "build_overlay",
           "OverlayResult", "check_l2_not_copy_of_l1"]


class OverlayResult:
    """The merged common.* tree plus where every leaf came from.

    Provenance is not a nicety. When assertion A reports a key as unassigned, the
    first question is which layer was supposed to fill it; without provenance the
    answer is a manual grep across six files.
    """

    # Two fields, and __slots__ so it stays two. An expanded tree belongs to the
    # reference axis (CFG-CM-8); hanging it off this object would produce an
    # OverlayResult that sometimes carries pre-freeze values and sometimes
    # post-freeze ones, with nothing in the type to say which.
    __slots__ = ("tree", "provenance")

    def __init__(self, tree: Dict[str, Any], provenance: Dict[str, str]):
        # No defensive copy, and `tree` is NOT disjoint from the caller's layer
        # dicts. deep_merge clones only the nodes it actually walked, so every
        # branch no upper layer mentioned is the SAME object as the one in
        # layer_trees -- with one layer supplied, nearly the whole tree is
        # shared. Two consequences, neither optional: writing into this tree
        # reaches back into that layer dict and changes what the next reader of
        # the layer sees, and it also desynchronises common_digest, which is
        # computed over this tree and audited by CFG-41, so the corruption
        # would be recorded as the truth. Read it; never write into it.
        self.tree = tree
        # Dotted leaf path -> layer name. The keys are the same strings
        # unassigned() returns, which is what lets assertion A name a hole and
        # the layer that declared it without a second lookup scheme.
        self.provenance = provenance

    def get(self, dotted: str, default: Any = MISSING) -> Any:
        """Leaf by dotted path.

        * Returns MISSING, not None, for a key no layer mentioned. None is a real
        value here -- it means "declared but unassigned" -- so collapsing the two
        would hide exactly the distinction 10 S5.4.3 draws.
        """
        # Walks the tree instead of indexing flatten(self.tree), for two
        # reasons. flatten rebuilds the entire dotted map on every call, and,
        # more importantly, flatten only ever yields LEAVES. 10 S5.4.2 allows a
        # whole object to be referenced (`profiles: ${common.motion.profiles}`),
        # so a lookup must be able to stop at a subtree; a flatten-based one
        # would report that legal whole-section case as absent.
        node: Any = self.tree
        for part in dotted.split("."):
            if not isinstance(node, dict) or part not in node:
                # Returns `default` rather than raising: the caller is the one
                # who knows whether absence is an error here. The default being
                # MISSING and not None is what keeps "nobody mentioned this key"
                # distinguishable from "a layer declared it and left it null".
                return default
            node = node[part]
        return node

    def unassigned(self) -> List[str]:
        """Dotted paths whose value is null -- assertion A's input.

        These are keys some layer declared and no layer filled. A key nobody
        mentioned is not in this list, which is the point of the null/missing
        split: only the first kind is a configuration gap someone intended.
        """
        # sorted(), so the list assertion A prints is stable from run to run.
        # Insertion order here follows merge order, which follows which layer
        # files exist, so an unsorted report would reshuffle whenever an
        # unrelated site file is added and stop being diffable.
        #
        # The test is `v is None`, never `not v`: 0, "" and [] are assigned
        # values and must not be reported as holes.
        #
        # NOTE what this cannot catch. A safety parameter written as 0.0 instead
        # of null is indistinguishable from a genuine zero at this point, and it
        # is the more dangerous of the two failures -- it passes assertion A,
        # then limits v_max to zero at run time with nothing logged. The defence
        # is CLAUDE.md 3.1 (never write 0.0 to mean unset, write null) plus the
        # CFG-CF-2 scan over configs/, not this function.
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
    # Empty, not seeded with layer_trees["L0"]. Seeding would enter the loop at
    # L1 and so skip the only check L0 gets -- and L0 is the one layer policed
    # by an exclusion list, the list that stops a dataclass default standing in
    # for a safety parameter. See L0_EXCLUDED_PREFIXES for why that is the
    # fail-silent direction rather than merely untidy.
    merged: Dict[str, Any] = {}
    provenance: Dict[str, str] = {}

    # Iterates LAYERS, not layer_trees. Precedence is a property of the design
    # (the L0~L5 figure in 10 S5.4.1), not of the caller's dict ordering. A
    # caller that happened to build {"L4": ..., "L2": ...} would otherwise let
    # the model layer win over the site layer, and nothing anywhere would say
    # so -- the tree would simply hold the wrong number.
    for layer in LAYERS:
        if layer.name == "L5":
            # L5 is never taken from layer_trees, so a caller cannot introduce
            # a fourth environment variable by putting one in the dict -- the
            # whitelist in layers.py is the only way in. XBRAIN_CONFIG_DIR is
            # not one of them (ENV-4) and is resolved before this loop runs.
            flat_env = env_overlay(env)
            # env_overlay hands back dotted keys, because the whitelist is a
            # variable-name -> key-path map. Reshaping to a tree here keeps
            # deep_merge the single merge implementation; a second, flat merge
            # path for L5 would be a second place for the three rules of
            # 10 S5.4.3 to drift.
            tree = unflatten(flat_env) if flat_env else {}
        else:
            # `or {}` absorbs both "layer absent from the dict" and "present but
            # None". An absent layer is normal, not an error: an uncalibrated
            # robot has no calib/ file (L4b), and a site that overrides nothing
            # is a legitimate site. Refusing to start on a missing optional
            # layer would push people towards writing placeholder files, which
            # is how a layer acquires values nobody chose.
            tree = layer_trees.get(layer.name) or {}
        if not tree:
            # An empty layer contributes to neither the tree nor the provenance
            # map, and the second half matters: recording it would let it claim
            # authorship of values it never wrote and send whoever reads
            # assertion A's output to the wrong file.
            continue
        # Flattened first because check_namespace matches full dotted leaf
        # paths, never top-level keys. Why the distinction is the rule and not
        # a detail is argued once, at check_namespace; do not restate it here.
        flat = flatten(tree)
        if check_namespaces:
            # Checked BEFORE the merge, so a rejected layer never lands in
            # `merged`. Checking afterwards would leave a caller that catches
            # ConfigLayerError holding a half-merged tree that looks usable.
            #
            # On the flag itself, see the docstring. NEVER thread it out to
            # freeze: CLAUDE.md 3.6 forbids a "skip the safety assertion" knob,
            # and with the allowances off, L2 may write common.safety.*.
            check_namespace(layer, flat.keys())
        merged = deep_merge(merged, tree)
        # Record provenance only for leaves this layer actually decided. A null
        # does not override, so it must not claim authorship of the value below.
        for key, value in flat.items():
            # Both halves of this condition carry weight, in opposite cases:
            #
            #   value is not None      this layer assigned the value, so it owns
            #                          it, overwriting any earlier claim
            #   key not in provenance  no layer has spoken for this key yet, so
            #                          the layer that declared the hole is the
            #                          correct answer to "who was supposed to
            #                          fill this" -- which is the question
            #                          assertion A leaves its reader holding
            #
            # Drop the first clause and a later null silently repoints an
            # already-filled key at the wrong layer. Drop the second and a key
            # that is null everywhere gets no provenance at all, which is
            # precisely the case where provenance is the thing being asked for.
            if value is not None or key not in provenance:
                provenance[key] = layer.name

    return OverlayResult(merged, provenance)
