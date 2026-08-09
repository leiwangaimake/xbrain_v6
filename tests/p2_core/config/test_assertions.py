"""BIZ-P2-24 -- p2_core.yaml assertion tests + variants."""

from pathlib import Path

import pytest
import yaml

from xbrain.p2_core.config import assertions as A


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent.parent
P2_YAML = REPO / "configs" / "p2_core.yaml"
COMMON_YAML = REPO / "configs" / "common.yaml"


# --- Fixture: load the committed p2_core.yaml + a minimal common tree
# The committed common.yaml has profiles: null in the top block and a
# filled example at the layered block; we build a minimal dict here
# that matches p2_core's shipped keys so check_all passes.

@pytest.fixture()
def real_p2_core() -> dict:
    return yaml.safe_load(P2_YAML.read_text())


@pytest.fixture()
def minimal_common() -> dict:
    """A common tree whose motion.profiles keys match the shipped
    p2_core.health.profile_admission keys (obstacle_avoid, patrol).
    Uses only what the assertions read; does NOT pretend to be the
    full common.yaml."""
    return {
        "motion": {
            "profiles": {
                "obstacle_avoid": {"max_mps": None},
                "patrol": {"max_mps": None},
            }
        }
    }


# --- POSITIVE: shipped p2_core.yaml passes every assertion --------

def test_shipped_p2_core_passes_all_assertions(real_p2_core, minimal_common):
    """POSITIVE: the committed configs/p2_core.yaml passes every
    BIZ-P2-24 assertion when paired with a minimal common tree that
    matches its profile keys. This is the healthy baseline; any of
    the mutation tests below would fail this if run in place of it."""
    A.check_all(real_p2_core, minimal_common,
                claimed_hot_files=[
                    "/opt/xbrain_v6/configs/suspicion_rules.yaml",
                    "/opt/xbrain_v6/configs/speech_presets.yaml",
                ])


# --- assertion C: profile_admission == common.motion.profiles ------

def test_assert_c_matches_when_keys_equal(minimal_common):
    p2 = {"health": {"profile_admission": {
        "obstacle_avoid": {}, "patrol": {},
    }}}
    A.check_profile_admission_matches_common(p2, minimal_common)


def test_assert_c_raises_on_extra_p2_key(minimal_common):
    """VARIANT: p2 has 3 profiles, common has 2 -> assertion C red."""
    p2 = {"health": {"profile_admission": {
        "obstacle_avoid": {}, "patrol": {}, "sprint": {},
    }}}
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_profile_admission_matches_common(p2, minimal_common)
    assert "assertion_C_profile_admission" in str(ei.value)
    assert "sprint" in str(ei.value)


def test_assert_c_raises_on_missing_p2_key(minimal_common):
    """VARIANT (from spec): drop a profile_admission key -> red."""
    p2 = {"health": {"profile_admission": {"obstacle_avoid": {}}}}
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_profile_admission_matches_common(p2, minimal_common)
    assert "patrol" in str(ei.value)


def test_assert_c_raises_when_p2_block_missing(minimal_common):
    p2 = {"health": {}}
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_profile_admission_matches_common(p2, minimal_common)
    assert "assertion_C_profile_admission" in str(ei.value)


# --- switch_order --------------------------------------------------

def test_switch_order_accepts_exact_5_entries():
    p2 = {"mode": {"switch_order":
                    ["device_mode", "payload_light", "ptz", "motion", "audio"]}}
    A.check_switch_order(p2)


def test_switch_order_rejects_reorder():
    """VARIANT: swap payload_light and ptz -> red. This is the exact
    reorder 14 S5.7 ML-5 warns against."""
    p2 = {"mode": {"switch_order":
                    ["device_mode", "ptz", "payload_light", "motion", "audio"]}}
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_switch_order(p2)
    assert "switch_order" in str(ei.value)


def test_switch_order_rejects_missing_entry():
    """VARIANT: drop device_mode -> red."""
    p2 = {"mode": {"switch_order":
                    ["payload_light", "ptz", "motion", "audio"]}}
    with pytest.raises(A.ConfigAssertError):
        A.check_switch_order(p2)


def test_switch_order_rejects_extra_entry():
    p2 = {"mode": {"switch_order":
                    ["device_mode", "payload_light", "ptz", "motion",
                     "audio", "extra_new_step"]}}
    with pytest.raises(A.ConfigAssertError):
        A.check_switch_order(p2)


# --- mode_motion.behavior closed set ------------------------------

def test_mode_motion_accepts_each_of_three_behaviors():
    for b in ("face_target_stop", "face_target_follow", "hold"):
        p2 = {"mode_motion": {
            "d_alarm": {"behavior": b},
            "b_cast":  {"behavior": b},
        }}
        A.check_mode_motion_behaviors(p2)


def test_mode_motion_rejects_face_target_alone():
    """VARIANT (from spec): mode_motion.d_alarm.behavior = 'face_target'
    (missing suffix) -> red. 14 explicitly says NOT downgraded to any
    default, must be E_CONFIG_INVALID."""
    p2 = {"mode_motion": {
        "d_alarm": {"behavior": "face_target"},   # bad
        "b_cast":  {"behavior": "hold"},
    }}
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_mode_motion_behaviors(p2)
    assert "d_alarm" in str(ei.value)
    assert "face_target" in str(ei.value)


def test_mode_motion_rejects_when_b_cast_is_out_of_set():
    p2 = {"mode_motion": {
        "d_alarm": {"behavior": "hold"},
        "b_cast":  {"behavior": "avoid_all"},
    }}
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_mode_motion_behaviors(p2)
    assert "b_cast" in str(ei.value)


# --- redblue_mode consistency --------------------------------------

def test_redblue_mode_matches_when_equal():
    p2 = {"d_mode": {"redblue_mode": 1},
          "arbiter": {"domains": {"payload_light":
                                   {"deter_redblue_mode": 1}}}}
    A.check_redblue_mode_matches(p2)


def test_redblue_mode_raises_when_mismatched():
    """VARIANT: two truths for the same strobe pattern -> red."""
    p2 = {"d_mode": {"redblue_mode": 1},
          "arbiter": {"domains": {"payload_light":
                                   {"deter_redblue_mode": 2}}}}
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_redblue_mode_matches(p2)
    assert "redblue_mode_consistency" in str(ei.value)


# --- no dead profiles (cruise / transit) ---------------------------

def test_no_dead_profiles_accepts_current_two():
    p2 = {"health": {"profile_admission": {
        "obstacle_avoid": {}, "patrol": {},
    }}}
    A.check_no_dead_profiles(p2)


def test_no_dead_profiles_rejects_cruise():
    """VARIANT (from spec): re-adding U33-deleted 'cruise' -> red.
    14 S8.3 verbatim: 'not ignored, not warned, refused'."""
    p2 = {"health": {"profile_admission": {
        "obstacle_avoid": {}, "patrol": {}, "cruise": {},
    }}}
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_no_dead_profiles(p2)
    assert "cruise" in str(ei.value)


def test_no_dead_profiles_rejects_transit():
    p2 = {"health": {"profile_admission": {
        "obstacle_avoid": {}, "transit": {},
    }}}
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_no_dead_profiles(p2)
    assert "transit" in str(ei.value)


# --- hot-update whitelist ------------------------------------------

def test_hot_update_whitelist_accepts_two_documented_files():
    A.check_hot_update_whitelist([
        "/opt/xbrain_v6/configs/suspicion_rules.yaml",
        "/opt/xbrain_v6/configs/speech_presets.yaml",
    ])


def test_hot_update_whitelist_rejects_p2_core():
    """VARIANT (from spec): put p2_core.yaml on the hot-update list ->
    red. 14 S11 CFG-31: NONE of p2_core.yaml keys are hot-updatable;
    the ONLY hot files are suspicion_rules + speech_presets."""
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_hot_update_whitelist([
            "/opt/xbrain_v6/configs/suspicion_rules.yaml",
            "/opt/xbrain_v6/configs/p2_core.yaml",   # this must fail
        ])
    assert "p2_core.yaml" in str(ei.value)


def test_hot_update_whitelist_rejects_common_yaml():
    with pytest.raises(A.ConfigAssertError):
        A.check_hot_update_whitelist([
            "/opt/xbrain_v6/configs/common.yaml",
        ])


# --- check_all short-circuits on first failure --------------------

def test_check_all_returns_first_failure_reason(minimal_common):
    """check_all should not aggregate errors -- raise on first, name it,
    let operator fix and rerun. Verify order: assertion_C first."""
    # bad p2: dead profile 'cruise' AND missing switch_order key
    p2 = {
        "health": {"profile_admission": {
            "obstacle_avoid": {}, "patrol": {}, "cruise": {}}},
        # missing mode.switch_order block entirely
    }
    with pytest.raises(A.ConfigAssertError) as ei:
        A.check_all(p2, minimal_common)
    # Because check_all runs assertion_C first and this p2 has
    # profile_admission mismatched, that fires first.
    assert "assertion_C_profile_admission" in str(ei.value)


# --- Meta: ConfigAssertError carries the rule name ----------------

def test_config_assert_error_exposes_rule_name():
    err = A.ConfigAssertError("some_rule", "boom", seen=1, expected=2)
    assert err.rule == "some_rule"
    assert err.seen == 1
    assert err.expected == 2
    assert "some_rule" in str(err)
