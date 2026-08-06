"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: duplicates.py
Brief: Assertion B, the U76(9) sub-clause -- L2 must not restate L1 verbatim

Description:
10 S5.4.4 assertion B is "no duplicate definition". Three of its clauses live
elsewhere and are evaluated on the L6 SOURCE files: a `common` top-level key in a
process config and a private key on the S5.4.5 alias table are both refs.check_l6
(R-6), and the value-based "private leaf same-name-same-value as a common leaf"
lands in the freeze service (CFG-FZ-4). This module is the FOURTH clause, added
by 99 U76(9): the one that compares two OVERLAY layers to each other.

The problem it solves. 10 S5.4.3 lets L2 (models/m20s.yaml) write common.spec.*
and common.motion.*, and common.motion.profiles is one of the things it may
override. But the true source of the profile table is L1 (common.yaml) -- 10
S5.4.5 lists its source as "common.yaml . motion.profiles", and 11 S15 S22 (anchor
"全量档位表同时写进 L1 与 L2") settled that L2 is for MODEL DIFFERENCES only, never
a second copy. Namespaces cannot tell the two apart: a model-difference override
and a wholesale second copy touch the EXACT same key paths, so check_namespace
(layers.py) waves both through -- the LAYERS comment there says as much and points
here. Without this check, someone pastes the whole table into m20s.yaml, both
files pass every namespace rule, and now one shared quantity has two authors that
drift apart the first time only one of them is edited. 11 S15 S22 mutation (3) is
exactly this shape and requires assertion B to go red for it.

The rule, verbatim from 99 U76(9): "L2 里逐键与 L1 完全同路径同值 => 判复制拒绝启动;
L2 改了值的 => 合法机型差异覆盖放行". So, per L2 leaf:
  * same dotted path in L1 AND same value      -> copy -> raise (reject boot)
  * same path, DIFFERENT value                 -> legal model override -> allow
  * path not present in L1 at all              -> model-specific key -> allow

*** The looks-right-but-wrong trap: comparing PATHS and not VALUES. An
implementation that raised whenever an L2 key also exists in L1 would be
permanently red -- it would reject every legitimate model override, which is the
one thing L2 exists to do, and the "改了值的放行" half of U76(9) would be lost.
test_l2_changed_value_is_a_legal_model_override is the mutation that catches that
direction; a no-op implementation is caught by the copy tests. Both are needed
(CLAUDE.md 3.3): a check with only the reject case is passed by a function that
raises on everything, and a check with only the allow case is passed by a
function that raises on nothing.

*** The other trap, measured against the real config: null on either side must
NOT count as a copy. common.yaml (L1) declares the spec keys as bare null so the
model layer can fill them (10 S5.4.5, the "只声明键位 = null" note), and three of
them -- max_vy_mps, max_wz_radps, max_accel_mps2 -- are uncalibrated today, so
m20s.yaml (L2) legitimately also holds null for them (CLAUDE.md 3.1: uncalibrated
is null, never 0.0). That is two nulls at one path, which is "同路径同值" on a
literal reading -- but it is NOT a duplicated value, it is one unfilled key
declared twice, and it is assertion A that owns it (it reports null as
"unassigned" and names the path). Flagging it here would fire a confusing
"duplicate" error on a correct-but-uncalibrated config and, worse, could pre-empt
assertion A's far more useful "fill this key" message. So a null on either side is
skipped, and the copy check only ever fires on two ASSIGNED, equal values.

What this module does NOT do, each belonging to a different owner:
  * namespace placement (is common.safety.* allowed in L2) -- layers.py
    check_namespace, 10 S5.4.3. That runs first, inside build_overlay; this runs
    after, on the ORIGINAL per-layer trees, because build_overlay merges L1 and
    L2 into one tree and the comparison needs them apart.
  * the L6 clauses of assertion B (common top-level key, alias-table names) --
    refs.check_l6, R-6.
  * the value-based L6 "private leaf same-name-same-value as a common leaf" clause
    -- the freeze service, CFG-FZ-4, which has the frozen common.* tree to compare
    against. It is deliberately not here: this module needs no common.* tree, only
    the two adjacent overlay layers.
  * generalising to L4-vs-lower or any other pair. U76(9) is specifically about L2
    restating L1 (model vs shared). Widening it past the ruling would be inventing
    a rule the contract does not state (CLAUDE.md 9.1).
"""

from typing import Any, Dict, Optional

# ConfigLayerError carries the closed-set code E_CONFIG_INVALID (group L,
# retryable "no"): a duplicate definition is a startup self-check failure that
# refuses to let the stack run, the same disposition as every other namespace
# violation in this package. Reusing the type means the freeze service catches one
# exception family for the whole of the overlay/assertion-B surface, and it means
# the code name is imported from the shared library exactly once (in layers.py),
# never spelled as a literal here -- scripts/lint/no_literal_ecode.py enforces
# that, and CLAUDE.md 3.5 is the reason.
from .layers import ConfigLayerError
# flatten yields dotted LEAF paths, which is the only granularity at which "same
# path same value" is well defined. Comparing nested dicts directly would compare
# whole subtrees and miss the case where L2 copies most of a section and changes
# one leaf -- that copied majority is still a second source of truth for those
# leaves. Leaf comparison catches it leaf by leaf, which is what "逐键" means.
from .merge import flatten


def check_l2_not_copy_of_l1(l1_tree: Optional[Dict[str, Any]],
                            l2_tree: Optional[Dict[str, Any]]) -> None:
    """Assertion B / 99 U76(9): reject an L2 leaf that restates L1 byte-for-byte.

    Raises ConfigLayerError (E_CONFIG_INVALID) naming both layers and the offending
    path. A model-difference override (same path, different value) and a
    model-specific key (path absent from L1) both pass.
    """
    # `or {}` mirrors build_overlay: an absent layer is normal, not an error. A
    # robot with no model overrides has an empty (or missing) m20s.yaml, and that
    # is not a copy of anything. flatten(None) would raise; flatten({}) is {}.
    l1 = flatten(l1_tree or {})
    l2 = flatten(l2_tree or {})

    # Iterate L2, look each leaf up in L1. The direction matters: the question is
    # "did the MODEL layer duplicate a shared value", so L2 is the layer under
    # suspicion and L1 is the reference. Iterating L1 instead would ask the
    # mirror-image question and report the shared layer as the offender, sending
    # the operator to edit the wrong file.
    for path, v2 in l2.items():
        if path not in l1:
            # L2 introduces a key L1 never had. That is a model-specific value,
            # exactly what the model layer is for, not a copy. Allowed.
            continue
        v1 = l1[path]
        # Null on either side is a declared-but-unassigned placeholder, not a
        # duplicated value -- see the module docstring on the uncalibrated spec
        # keys. Skip so this check never fires on the correct uncalibrated config;
        # assertion A is what reports those, by path.
        if v1 is None or v2 is None:
            continue
        # Two assigned values at the same path. Equal means L2 restated L1's value
        # verbatim -- a second source of truth. Unequal means the model genuinely
        # differs from the shared baseline, which is a legitimate override.
        #
        # NEVER drop the `== v2` and raise on the path match alone: that turns a
        # copy detector into a "reject any override" detector, which is
        # permanently red and defeats the only purpose L2 has.
        if v1 == v2:
            raise ConfigLayerError(
                # Names L2 and L1 in words (not just "a layer") and the exact leaf
                # path, so the operator can open models/*.yaml and delete the line.
                # The message states the consequence, not only the rule, because a
                # namespace-legal duplicate looks correct until you know why one
                # shared quantity may not have two authors.
                f"L2 (model models/) restates L1 (shared common.yaml) verbatim at "
                f"{path!r} (both = {v1!r}). A byte-identical entry makes L2 a second "
                f"source of truth for a shared value; L2 is for model-difference "
                f"overrides only. Change the value to override, or drop the key and "
                f"let L1 be the single source. See 10 S5.4.4 assertion B, 99 U76(9)."
            )
    # Falling off the end is acceptance. Covered by the override and
    # model-specific-key tests, which are not decoration: a check that raised on
    # everything would pass every copy test above while making every model file
    # illegal (CLAUDE.md 3.3, the same reasoning refs.check_l6 spells out).
