"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_config.py
Brief: RNS startup-assertion tests (A-CVG-5 D2, A-CLS-4 person, null fail-loud)

Description:
Guards config.py's three fail-loud behaviors: null key -> raise naming the path
(CLAUDE.md 3.1), D2 leave_progress<2*r_eff (A-CVG-5), person lock (A-CLS-4).
Each test names the mutant that reddens it (CLAUDE.md 3.3).
"""

from __future__ import annotations

import pytest

from xbrain.p1_motion.rns.config import (
    RnsConfigError,
    assert_leave_progress_below_2r_eff,
    assert_person_locked,
    require,
    run_startup_assertions,
)


def _cfg(delta_s=0.4, person=None):
    m = {"rns": {"wall_follow": {"leave_progress_m": delta_s},
                 "class_map": {},
                 # 11 v2.3 / 20 S5.1A: run_startup_assertions requires the cap key
                 "perception": {"unlocalized_speed_cap_mps": 0.5},
                 # 20 S3.0 / 99 U113-U114: run_startup_assertions requires the world
                 # keys and the avoidance floor
                 "world": {"object_min_confidence": 0.4, "person_min_confidence": 0.1,
                           "v_max_mps": 1.0},
                 "speed": {"avoid_floor_mps": 0.2}}}
    if person is not None:
        m["rns"]["class_map"]["person"] = person
    return m


def test_require_raises_naming_the_key_on_null():
    # CLAUDE.md 3.1: null key -> refuse, and the message must carry the path so
    # the operator can fill it. mutant: require() returning a default reddens.
    cfg = {"rns": {"wall_follow": {"leave_progress_m": None}}}
    with pytest.raises(RnsConfigError) as ei:
        require(cfg, "rns.wall_follow.leave_progress_m")
    assert "rns.wall_follow.leave_progress_m" in str(ei.value)


def test_require_raises_naming_the_key_on_missing():
    with pytest.raises(RnsConfigError) as ei:
        require({"rns": {}}, "rns.wall_follow.leave_progress_m")
    assert "wall_follow" in str(ei.value)


def test_d2_passes_when_below_2r_eff():
    # 0.4 < 2*0.48 = 0.96 -> ok
    assert_leave_progress_below_2r_eff(_cfg(delta_s=0.4), r_eff_m=0.48)


def test_d2_reddens_when_delta_s_reaches_2r_eff():
    # A-CVG-5 exact case (20): delta_s=1.2, r_eff=0.6 -> 1.2 >= 1.2 -> must raise.
    # mutant: drop the D2 check -> this passes -> reddens.
    with pytest.raises(RnsConfigError) as ei:
        assert_leave_progress_below_2r_eff(_cfg(delta_s=1.2), r_eff_m=0.6)
    assert "D2" in str(ei.value)


def test_person_lock_passes_when_absent():
    # person not in class_map is fine -- it is locked in code, not config.
    assert_person_locked(_cfg())


def test_person_lock_passes_when_correct():
    assert_person_locked(_cfg(person="person_stop"))


def test_person_lock_reddens_on_wrong_mapping():
    # A-CLS-4: class_map person->block must refuse. mutant: drop the check ->
    # this passes -> reddens.
    with pytest.raises(RnsConfigError) as ei:
        assert_person_locked(_cfg(person="block"))
    assert "person" in str(ei.value)


def test_run_startup_assertions_wires_both():
    # the boot selfcheck entry: both assertions run; a person violation with a
    # valid D2 still raises.
    with pytest.raises(RnsConfigError):
        run_startup_assertions(_cfg(delta_s=0.4, person="traverse"), r_eff_m=0.6)
    # clean config passes
    run_startup_assertions(_cfg(delta_s=0.4, person="person_stop"), r_eff_m=0.6)


# ── 20 S5.1.1 v1.35 (#20-25): class_map value hygiene (A-CLS-8) ───────────────
from xbrain.p1_motion.rns.config import assert_class_map_values  # noqa: E402


# ── 20 S3.0 RNS-N-18 (99 U113): world keys (A-WLD-6, start half) ─────────────
@pytest.mark.parametrize("key,bad", [
    ("object_min_confidence", None), ("object_min_confidence", 0.0),
    ("object_min_confidence", 1.5), ("person_min_confidence", None),
    ("person_min_confidence", 0.0), ("person_min_confidence", 1.5),
    ("v_max_mps", None), ("v_max_mps", 0.0), ("v_max_mps", -0.1)])
def test_world_keys_null_or_out_of_range_refuse_start(key, bad):
    # A-WLD-6: a null / zero / out-of-range world key refuses to start, naming
    # the key. mutant: drop assert_world_keys from run_startup_assertions ->
    # every case starts -> reddens.
    cfg = _cfg()
    cfg["rns"]["world"][key] = bad
    with pytest.raises(RnsConfigError, match="rns.world." + key):
        run_startup_assertions(cfg, r_eff_m=0.6)


@pytest.mark.parametrize("bad", [None, 0.0, -0.1, 1.5])
def test_avoid_floor_null_nonpositive_or_above_ceiling_refuses_start(bad):
    # 99 U114 (A-WLD-6): speed.avoid_floor_mps must be in (0, world.v_max_mps];
    # 1.5 > the 1.0 ceiling is a floor nobody can reach. mutant: drop the
    # floor check -> every case starts -> reddens.
    cfg = _cfg()
    cfg["rns"]["speed"]["avoid_floor_mps"] = bad
    with pytest.raises(RnsConfigError, match="avoid_floor_mps"):
        run_startup_assertions(cfg, r_eff_m=0.6)


def test_world_keys_missing_refuses_start():
    # A-WLD-6: the whole world section absent -> refuse, not default.
    cfg = _cfg()
    del cfg["rns"]["world"]
    with pytest.raises(RnsConfigError, match="rns.world"):
        run_startup_assertions(cfg, r_eff_m=0.6)


def _cfg_cm(**rows):
    return {"rns": {"wall_follow": {"leave_progress_m": 0.4},
                    "class_map": dict(rows)}}


def test_class_map_values_pass_when_in_closed_set():
    assert_class_map_values(_cfg_cm(car="vehicle_dynamic", bird="ignore",
                                    pit="hazard"))


def test_class_map_out_of_set_value_refuses_start():
    # A-CLS-8: a value outside the behavior closed set would be a string nobody
    # dispatches on. mutant: drop the membership check -> passes -> reddens.
    with pytest.raises(RnsConfigError) as ei:
        assert_class_map_values(_cfg_cm(car="fly"))
    assert "fly" in str(ei.value)


def test_class_map_t_class_row_refuses_start():
    # A-CLS-8: the T class is not an object (11 S3.1B.2 v2.1); a row for it
    # means the contract was misread. mutant: drop the T-class check -> reddens.
    with pytest.raises(RnsConfigError) as ei:
        assert_class_map_values(_cfg_cm(traversable_area="traverse"))
    assert "traversable_area" in str(ei.value)


def test_run_startup_assertions_wires_class_map_values():
    # mutant: remove assert_class_map_values from run_startup_assertions ->
    # a bad value boots -> reddens.
    with pytest.raises(RnsConfigError):
        run_startup_assertions(_cfg_cm(car="fly"), r_eff_m=0.6)
