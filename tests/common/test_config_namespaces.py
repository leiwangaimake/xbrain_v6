"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config_namespaces.py
Brief: CFG-FZ-16 -- per-layer namespace allowances and assertion B (U76(9))

Description:
CFG-FZ-16. Two things the earlier CFG-CM-7 suite did not cover:

  * one independent case per overlay layer (L1/L2/L3/L4/L4b/L6) proving the layer
    rejects a key outside its 10 S5.4.3 allowance with E_CONFIG_INVALID and a
    message naming the layer and the offending key. CFG-CM-7 tested only L0, L2
    and L4b; L1/L3/L4 were missing and L6 lived in the reference-axis suite.
  * the three S22 mutations verbatim from 11 S15 (anchor "全量档位表同时写进 L1 与
    L2"), all three of which must go red, and the assertion B sub-clause added by
    99 U76(9) that mutation (3) needs -- L2 restating L1 byte-for-byte.

Every case names the mutation that turns it red (CLAUDE.md 3.3). The layers are
validated in TWO different places by design, and the split is the point of
mutation (2): L1..L4b go through layers.check_namespace (a namespace allowance),
while L6 is not in layers.LAYERS at all and goes through refs.check_l6 (R-6),
because a process-private file is checked for "no common top-level key", not for a
common.* allowance. A test that reached for one mechanism for all six layers would
have to invent an allowance for L6 that the design does not give it.
"""

import copy
import os
import sys

import pytest

# Same three-deep climb every test module in this tree uses to import the package
# without installing it. Kept identical so a reader moving between suites does not
# have to re-derive the path.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from xbrain.common.config import ConfigLayerError  # noqa: E402
from xbrain.common.config import check_l2_not_copy_of_l1  # noqa: E402
from xbrain.common.config import build_overlay, unflatten  # noqa: E402
from xbrain.common.config import refs  # noqa: E402


# ── 逐层独立用例(10 S5.4.3 合并规则表"本层允许写入的命名空间"列)──────────────
#
# Each case sends the layer exactly one out-of-allowance key and asserts three
# things: it raises ConfigLayerError, the code is the closed-set E_CONFIG_INVALID,
# and the message names the layer plus the offending key (the done-criterion:
# "打印层名 + 违规顶层键"). The positive half -- an in-allowance key is accepted --
# is asserted too, so a check that rejected everything cannot pass by the negative
# case alone (CLAUDE.md 3.3). The out-of-allowance key chosen for each layer is a
# key that legitimately BELONGS to a DIFFERENT layer, so the case also documents
# the boundary between the two, not just an arbitrary rejection.


def _reject(layer_name, out_of_allowance_key, expect_in_msg):
    """Build a single-layer overlay that must be rejected, and return the message.

    Factored out because all five namespace layers assert the identical shape and
    a copy per layer would drift -- the same reason the production code keeps one
    check_namespace rather than one branch per layer. The key is passed dotted and
    unflattened here so each call site reads as a single line.
    """
    with pytest.raises(ConfigLayerError) as e:
        # unflatten turns the dotted key into the nested tree build_overlay expects;
        # the value 1.0 is arbitrary -- placement, not value, is what is on trial.
        build_overlay({layer_name: unflatten({out_of_allowance_key: 1.0})})
    # The code is the closed-set value, not a bespoke string: a namespace
    # violation refuses boot the same way every other config self-check does, so a
    # caller can catch one family for the whole surface.
    assert e.value.code == "E_CONFIG_INVALID"
    # The layer name must be in the message because the operator's next move is to
    # open THAT layer's file; a message that only said "namespace violation" would
    # leave them grepping six files.
    assert layer_name in str(e.value), "message must name the layer"
    # The offending key must be in the message so the operator knows which line to
    # delete, not merely which file.
    assert expect_in_msg in str(e.value), "message must name the offending key"
    return str(e.value)


def test_l1_allows_only_common_and_rejects_other_top_level_keys():
    """L1: only common.* ; any other top-level key is rejected.

    speed_profiles is chosen as the offender on purpose: it is the S5.4.5 alias
    for common.motion.profiles, so a bare top-level speed_profiles: block is the
    exact shape 11 S15 S22 mutation (1) uses.

    Mutation: widen L1's allowance to () (unrestricted) => this goes red, and a
    stray top-level block in common.yaml would be silently accepted.
    """
    _reject("L1", "speed_profiles.patrol.max_mps", "speed_profiles")
    # Positive: an ordinary common.* key is accepted and attributed to L1, so the
    # layer is not simply rejecting everything.
    r = build_overlay({"L1": {"common": {"robot_id": "xbrain-01"}}})
    assert r.get("common.robot_id") == "xbrain-01"


def test_l2_allows_only_spec_and_motion():
    """L2: common.spec.* and common.motion.* only.

    The offender is common.safety.*, which is L3's -- and that is not an arbitrary
    pick. 10 S5.4.3 gives L3 the automatic lock (CFG-31); a safety parameter that
    slipped into the model layer would never be locked, so assertion E's
    "safety namespaces intersect the hot-reload whitelist == empty" would silently
    have nothing to protect.

    Mutation: add common.safety. to L2's allowance => red.
    """
    _reject("L2", "common.safety.brake.a_mps2", "common.safety.brake")
    # Positive: a real model-layer key (free-space geometry lives under
    # common.motion.* per U72) is accepted.
    r = build_overlay({"L2": {"common": {"motion": {"free_space": {"r_body_m": 0.48}}}}})
    assert r.get("common.motion.free_space.r_body_m") == 0.48


def test_l3_allows_only_safety():
    """L3: common.safety.* only.

    The offender is a spec key, which belongs to L2. The direction matters: spec
    values are model limits (max_vx_mps and friends), not safety constants, so the
    safety file is the wrong home for them even though both sound "safety-ish".
    """
    _reject("L3", "common.spec.max_vx_mps", "common.spec.max_vx_mps")
    # Positive: the brake latency constant is a genuine L3 leaf.
    r = build_overlay({"L3": {"common": {"safety": {"t_lat_s": 0.4}}}})
    assert r.get("common.safety.t_lat_s") == 0.4


def test_l4_allows_only_geo_site_retention():
    """L4: common.geo.* / common.site.* / common.retention.* only.

    The offender is a calib key -- L4b's. That the two are disjoint is exactly why
    L4 and L4b can share a tier with no order defined between them (10 S5.4.3): if
    a calib key were accepted at L4, folding L4 and L4b would become order-
    dependent and the "no order needed" guarantee would quietly break.
    """
    _reject("L4", "common.calib.robot_id", "common.calib.robot_id")
    # Positive: a site retention key is a genuine L4 leaf.
    r = build_overlay({"L4": {"common": {"retention": {"task_days": 30}}}})
    assert r.get("common.retention.task_days") == 30


def test_l4b_allows_only_calib():
    """L4b: common.calib.* only. Geo is L4's.

    The mirror of the case above: an enu_origin (geo) written into the per-vehicle
    calibration file is rejected, because origin is a site fact, not a per-vehicle
    one, and mixing them is how a robot's calibration would change when it is
    driven to a new site.
    """
    _reject("L4b", "common.geo.enu_origin", "common.geo.enu_origin")
    # Positive: the calibration revision is a genuine per-vehicle (L4b) leaf.
    r = build_overlay({"L4b": {"common": {"calib": {"calib_rev": 3}}}})
    assert r.get("common.calib.calib_rev") == 3


def test_l6_may_not_carry_a_common_top_level_key():
    """L6: process-private keys only, no common top-level key (R-6, assertion B).

    L6 is NOT in layers.LAYERS -- build_overlay stops at L5 -- so it is refs.check_l6
    that stands between a process file and a redefinition of shared state. The
    per-layer analogue of a namespace violation, for L6, is exactly this: a
    process file has no allowance to enumerate, it simply may not carry `common`.

    Mutation: make check_l6 skip the `common in tree` test => red, and a process
    file could redefine a shared value that then drifts from every other reader.
    """
    with pytest.raises(ConfigLayerError) as e:
        # Empty blacklist: this case is about the common top-level key, which is
        # rejected regardless of the alias table. The alias-name half of R-6 has
        # its own cases in the reference-axis suite, so it is not duplicated here.
        refs.check_l6({"common": {"safety": {"brake": {"k": 1.2}}}}, [])
    assert e.value.code == "E_CONFIG_INVALID"
    # R-6's message speaks of L6 and names the forbidden top-level key.
    assert "L6" in str(e.value)
    assert "common" in str(e.value)
    # Positive: an ordinary private tree (values referenced with ${common.*} are
    # what a real L6 file uses) is accepted, so the check is not rejecting all.
    refs.check_l6(unflatten({"p1_motion.loop_hz": 20}), [])


# ── S22 三变异体(11 S15, 逐字)──────────────────────────────────────────────
#
# The three mutations 11 S15 S22 spells out. (1) and (2) are namespace failures
# caught by check_namespace; (3) is namespace-legal on both layers and is caught
# only by assertion B. All three must go red -- an implementation that reddens
# (1) and (2) but not (3) has enforced placement without enforcing single source
# of truth, which is precisely the gap S22 was written to close.


@pytest.mark.parametrize("layer", ["L1", "L2"])
def test_s22_mutation1_top_level_speed_profiles_rejected_in_both_layers(layer):
    """(1) top-level speed_profiles: -- rejected in L1 AND L2, both.

    The two-layer parametrisation is the whole point (the S22 correction). L1 is
    the true source layer of common.motion.profiles, so an implementation that only
    checks L2's namespace and lets L1 through would pass a single-layer test while
    leaving the true-source layer unguarded. Testing both layers is what makes the
    L1 case fail such an implementation -- confirmed by mutation M-A.
    """
    with pytest.raises(ConfigLayerError) as e:
        build_overlay({layer: {"speed_profiles": {"patrol": {"max_mps": 2.0}}}})
    assert e.value.code == "E_CONFIG_INVALID"
    assert layer in str(e.value)
    assert "speed_profiles" in str(e.value)


def test_s22_mutation2_common_safety_brake_moved_into_l2_is_rejected():
    """(2) common.safety.brake into L2 -- rejected.

    This is the case that tells a real namespace check apart from a cheap one. Its
    top-level key IS common, like every legal L2 key, so an implementation that
    only asserts "top-level key == common" passes mutation (1) and fails here --
    confirmed by mutation M-B, which collapses L2's allowance to "common." and
    watches only this case go red.
    """
    with pytest.raises(ConfigLayerError) as e:
        build_overlay({"L2": {"common": {"safety": {"brake": {"a_mps2": 2.5}}}}})
    assert e.value.code == "E_CONFIG_INVALID"
    assert "L2" in str(e.value)
    assert "common.safety.brake" in str(e.value)


def test_s22_mutation3_full_profile_table_in_both_l1_and_l2_is_a_copy():
    """(3) the whole profile table in BOTH L1 and L2 -- assertion B goes red.

    Both placements are namespace-legal: L1 allows common.* and L2 allows
    common.motion.*, so build_overlay accepts both without complaint. That is
    deliberate -- it proves the duplication is invisible to the namespace layer and
    can only be caught by assertion B's value comparison (99 U76(9)). An
    implementation where (1) and (2) pass but (3) does not fail has not implemented
    single-source-of-truth.

    Mutation: make check_l2_not_copy_of_l1 a no-op (return) => this goes red
    (confirmed by M-C).
    """
    # The exact shape S22 names: identical content, both key paths legal.
    profiles = {"common": {"motion": {"profiles": {
        "obstacle_avoid": {"max_mps": 0.5},
        "patrol": {"max_mps": 2.0},
    }}}}
    l1 = profiles
    # deepcopy, not the same object: a byte-identical second copy is exactly the
    # "全表字节相同" hazard U76(9) describes, and sharing the object would not model
    # two files on disk.
    l2 = copy.deepcopy(profiles)

    # Namespaces pass for both layers -- the point of the mutation. If this raised,
    # the test would be proving the wrong thing (that the namespace layer caught it).
    build_overlay({"L1": l1, "L2": l2})

    # Assertion B catches the duplication that the namespace layer let through.
    with pytest.raises(ConfigLayerError) as e:
        check_l2_not_copy_of_l1(l1, l2)
    assert e.value.code == "E_CONFIG_INVALID"
    assert "common.motion.profiles" in str(e.value)


# ── 断言 B / U76(9) 直测: 复制拒绝 / 改值放行 / 缺键放行 / null 不误判 ──────────
#
# The sub-clause has two failure directions and both need a case, because a check
# with only one is passed by a degenerate implementation: reject-everything passes
# the copy case, accept-everything passes the override case. The null case guards a
# third thing -- that a correct-but-uncalibrated config is not mistaken for a copy.


def test_l2_verbatim_copy_of_l1_is_rejected():
    """Same path, same value in both layers => copy => reject.

    Mutation: no-op the check => red (M-C). This is the "implemented nowhere"
    baseline the LAYERS comment in layers.py describes, before U76(9) settled it.
    """
    # spec.max_vx_mps is a real shared leaf; L2 restating L1's value verbatim is
    # the second-source-of-truth the clause exists to stop.
    l1 = {"common": {"spec": {"max_vx_mps": 2.0}}}
    l2 = {"common": {"spec": {"max_vx_mps": 2.0}}}
    with pytest.raises(ConfigLayerError) as e:
        check_l2_not_copy_of_l1(l1, l2)
    assert e.value.code == "E_CONFIG_INVALID"
    assert "common.spec.max_vx_mps" in str(e.value)


def test_l2_changed_value_is_a_legal_model_override():
    """Same path, DIFFERENT value => legal model-difference override => allowed.

    This is the direction 99 U76(9) is careful to keep open ("改了值的放行"). The
    guarding mutation makes check_l2_not_copy_of_l1 raise on a path match
    regardless of value => this goes red (M-D). Without this case the check could
    be a permanently-red "reject any override", which defeats L2's only purpose --
    a model genuinely faster or slower than the shared baseline.
    """
    l1 = {"common": {"spec": {"max_vx_mps": 2.0}}}
    l2 = {"common": {"spec": {"max_vx_mps": 1.5}}}
    # Must NOT raise: the model differs, which is what the model layer is for.
    check_l2_not_copy_of_l1(l1, l2)


def test_l2_key_absent_from_l1_is_model_specific_and_allowed():
    """A key L2 has and L1 never had is a model-specific value, not a copy.

    Nothing in L1 to duplicate, so there is nothing to reject; if this raised, the
    check would be forbidding L2 from introducing any key of its own.
    """
    l1 = {"common": {"spec": {"max_vx_mps": 2.0}}}
    l2 = {"common": {"motion": {"free_space": {"r_body_m": 0.48}}}}
    # Must NOT raise.
    check_l2_not_copy_of_l1(l1, l2)


def test_null_placeholder_in_both_layers_is_not_a_copy():
    """L1 declares a spec key null, L2 leaves it null (uncalibrated) => NOT a copy.

    common.yaml declares the spec keys as null for the model layer to fill, and
    max_wz_radps is uncalibrated today (10 S5.4.5, the "仍无依据的三项"), so
    m20s.yaml legitimately also holds null. Two nulls at one path is "same path,
    same value" on a literal reading -- but it is one unfilled key declared twice,
    not a duplicated value. Assertion A owns it (it reports the null and names the
    key); flagging it here would fail a correct uncalibrated config with a
    misleading "duplicate" error and could pre-empt A's useful "fill this key".

    Mutation: drop the `v1 is None or v2 is None` skip => this goes red (M-E), and
    the real uncalibrated config stops booting for the wrong reason.
    """
    l1 = {"common": {"spec": {"max_wz_radps": None}}}
    l2 = {"common": {"spec": {"max_wz_radps": None}}}
    # Must NOT raise.
    check_l2_not_copy_of_l1(l1, l2)
