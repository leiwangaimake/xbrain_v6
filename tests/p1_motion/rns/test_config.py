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
    RnsConfigError, assert_leave_progress_below_2r_eff, assert_person_locked,
    require, run_startup_assertions,
)


def _cfg(delta_s=0.4, person=None):
    m = {"rns": {"wall_follow": {"leave_progress_m": delta_s},
                 "class_map": {}}}
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
