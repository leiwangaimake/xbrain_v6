"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_classify_dispatch.py
Brief: class_map dispatch + static criterion + raw refusal (P3 -- 20 S5.1/S5.5)

Description:
Guards obstacle dispatch: person always dynamic (A-CLS-1), static needs speed AND
dwell (A-CLS-2/3), unmapped -> block (A-CLS-4 reverse), velocity_frame==raw
refused (A-FUS-7). Each test names its mutant.
"""

from __future__ import annotations

import pytest

from xbrain.p1_motion.rns.classify import (
    behavior_class, dispatch_dynamic, is_static, usable_velocity,
)


CM = {"car": "vehicle_dynamic", "chair": "block", "grass": "traverse"}


def test_person_locked_to_person_stop():
    # person is never read from class_map (locked in code).
    assert behavior_class("person", CM) == "person_stop"
    assert behavior_class("person", {"person": "block"}) == "person_stop"  # ignored


def test_unmapped_class_is_block():
    # A-CLS-4 reverse: unmapped -> block+slow, never dropped, never traverse.
    # mutant: return None / traverse for unmapped -> reddens.
    assert behavior_class("forklift", CM) == "block"
    assert behavior_class("car", CM) == "vehicle_dynamic"


def test_static_needs_both_speed_and_dwell():
    # A-CLS-2/3: BOTH. speed alone (no dwell) or dwell alone (over threshold) is
    # NOT static.
    assert is_static(0.05, dwell_s=3.0, v_static_thresh=0.1, t_static_dwell=2.0) is True
    # just-started car: slow but no dwell -> NOT static (A-CLS-2)
    assert is_static(0.05, dwell_s=0.5, v_static_thresh=0.1, t_static_dwell=2.0) is False
    # A-CLS-3 (reverse): parked car with noise reading over threshold -> NOT
    # static -> robot keeps waiting rather than detouring a car that may move.
    assert is_static(0.2, dwell_s=5.0, v_static_thresh=0.1, t_static_dwell=2.0) is False


def test_person_always_dynamic_even_when_still():
    # A-CLS-1: a motionless person is STILL dynamic -> never a detour candidate.
    # mutant: let a static person join the static pile -> A-CLS-1 red.
    assert dispatch_dynamic("person_stop", velocity_mps=0.0, dwell_s=10.0,
                            v_static_thresh=0.1, t_static_dwell=2.0) is True


def test_static_car_leaves_dynamic_pile():
    # a car that passed the static criterion is NOT dynamic (can be detoured).
    assert dispatch_dynamic("vehicle_dynamic", velocity_mps=0.02, dwell_s=5.0,
                            v_static_thresh=0.1, t_static_dwell=2.0) is False
    # a moving car IS dynamic
    assert dispatch_dynamic("vehicle_dynamic", velocity_mps=1.5, dwell_s=5.0,
                            v_static_thresh=0.1, t_static_dwell=2.0) is True


def test_raw_velocity_refused_as_motion_cue():
    # A-FUS-7: raw velocity (ego not removed) is unusable -> None. mutant: return
    # the raw value -> a static tree reads as moving -> reddens.
    assert usable_velocity("raw", -1.5, raw_policy="reject") is None
    assert usable_velocity("ego_removed", 0.8, raw_policy="reject") == 0.8


def test_raw_policy_other_than_reject_raises():
    with pytest.raises(ValueError):
        usable_velocity("raw", -1.5, raw_policy="compensate")


# ── 20 S5.1.1 v1.35 (#20-25): effective_behavior ──────────────────────────────
from xbrain.p1_motion.rns.classify import effective_behavior  # noqa: E402

CM2 = {"car": "vehicle_dynamic", "bird": "ignore", "grass": "traverse",
       "pit": "hazard"}


def test_t_class_is_dropped_not_block():
    # A-CLS-5: traversable_area is the T channel, not an object -> None (drop).
    # mutant: fall through to behavior_class -> "block" -> reddens.
    assert effective_behavior("traversable_area", "confirmed", 0.99, CM2, 0.3) == (None, False)


def test_ignore_class_is_ignore_when_confirmed():
    # A-CLS-6: a confirmed bird maps to ignore (explicit row).
    assert effective_behavior("bird", "confirmed", 0.9, CM2, 0.3) == ("ignore", True)


def test_low_confidence_collapses_to_block_person_immune():
    # A-CLS-7: below min_confidence the class is unknown -> block; ignore must
    # NOT apply. mutant: skip the gate -> ("ignore", True) -> reddens.
    assert effective_behavior("bird", "confirmed", 0.1, CM2, 0.3) == ("block", True)
    # A-CLS-7 reverse: person is immune -- any confidence stops the robot.
    assert effective_behavior("person", "confirmed", 0.05, CM2, 0.3)[0] == "person_stop"
    assert effective_behavior("person", "proxy", 0.05, CM2, 0.3)[0] == "person_stop"


def test_proxy_takes_the_more_conservative_side():
    # proxy vehicle: dynamic rule kept, static pile barred (allow_static False)
    assert effective_behavior("car", "proxy", 0.9, CM2, 0.3) == ("vehicle_dynamic", False)
    # proxy ignore/traverse collapse to block; proxy hazard stays hazard
    assert effective_behavior("bird", "proxy", 0.9, CM2, 0.3) == ("block", False)
    assert effective_behavior("grass", "proxy", 0.9, CM2, 0.3) == ("block", False)
    assert effective_behavior("pit", "proxy", 0.9, CM2, 0.3) == ("hazard", False)
    # confirmed unmapped -> block, static allowed
    assert effective_behavior("forklift", "confirmed", 0.9, CM2, 0.3) == ("block", True)
