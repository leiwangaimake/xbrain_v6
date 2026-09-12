"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ctrl_loop_vy.py
Brief: CtrlLoop lateral channel -- holonomic publish form and estop zeroing of vy

Description:
P7.2 adds vy to the 10-step executor for the holonomic M20S. The property that
must hold is the same one the estop tests pin for vx: a soft-estop tick is
zero on EVERY axis. Also: the non-holonomic form keeps the two-argument publish
(existing callers untouched) and drops + counts any vy it is handed
(12 S4.7.3 TS-4).
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.ctrl_loop import CtrlLoop, CtrlState

pytestmark = pytest.mark.no_device


def test_holonomic_publishes_three_axes():
    got = []
    loop = CtrlLoop(lambda vx, wz, vy: got.append((vx, wz, vy)), holonomic=True)
    loop.transition(CtrlState.ACTIVE)
    t = loop.run_one_tick(computed_vx=0.5, computed_wz=0.1, computed_vy=0.2)
    assert got == [(0.5, 0.1, 0.2)] and t.vy == 0.2


def test_estop_zeroes_vy_too():
    """mutant: keep computed_vy on the estop branch -> lateral leak -> red."""
    got = []
    loop = CtrlLoop(lambda vx, wz, vy: got.append((vx, wz, vy)), holonomic=True)
    loop.transition(CtrlState.ACTIVE)
    t = loop.run_one_tick(computed_vx=0.5, computed_wz=0.1, computed_vy=0.2, estop=True)
    assert got == [(0.0, 0.0, 0.0)] and t.vy == 0.0 and t.stop_reason == "soft_estop"


def test_not_active_zeroes_vy():
    got = []
    loop = CtrlLoop(lambda vx, wz, vy: got.append((vx, wz, vy)), holonomic=True)
    loop.run_one_tick(computed_vx=0.5, computed_wz=0.1, computed_vy=0.2)
    assert got == [(0.0, 0.0, 0.0)]


def test_non_holonomic_keeps_two_arg_publish_and_drops_vy():
    got = []
    loop = CtrlLoop(lambda vx, wz: got.append((vx, wz)))
    loop.transition(CtrlState.ACTIVE)
    t = loop.run_one_tick(computed_vx=0.5, computed_wz=0.1, computed_vy=0.2)
    assert got == [(0.5, 0.1)] and t.vy == 0.0 and loop.vy_dropped == 1


def test_history_is_bounded():
    """The loop runs for the process lifetime at 20 Hz; an unbounded history
    list grew without limit. mutant: plain list -> len == 1500 -> red."""
    from xbrain.p1_motion.ctrl_loop import HISTORY_MAX
    loop = CtrlLoop(lambda vx, wz: None)
    loop.transition(CtrlState.ACTIVE)
    for _ in range(HISTORY_MAX + 300):
        loop.run_one_tick(computed_vx=0.1)
    assert len(loop.history) == HISTORY_MAX
    assert loop.history[-1].tick_no == HISTORY_MAX + 300
