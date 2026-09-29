"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fence_episodes.py
Brief: FenceEpisodeTracker -- one excursion's event sequence, E-2 episode sharing, E-3 no clip flood, degraded/restored

Description:
Drives evaluate() along x on the allow square and feeds the tracker: the
excursion soft_enter -> hard_clip -> breach -> recovered -> soft_exit must
carry one episode id, hard_clip must not repeat while clipping continues,
the next excursion gets the next id, a rev change forgets the memory,
enforcement transitions produce fence_degraded / fence_restored with the
S9A.9 dedup keys, and a degenerate tick reports once.
"""
from __future__ import annotations

import pytest

from xbrain.p1_motion.fence.clip import (
    CompiledFence,
    CompiledPolygon,
    FenceConstants,
    FenceEval,
    evaluate,
)
from xbrain.p1_motion.fence.episodes import FenceEpisodeTracker

pytestmark = pytest.mark.no_device

CFG = FenceConstants(brake_k=1.5, brake_a_mps2=2.5, t_lat_s=0.4, soft_margin_min_m=2.0,
                     predict_dt_s=0.45,
                     margin_by_fix={"rtk_fixed": 0.3, "rtk_float": 1.0},
                     projection_iters=3, v_profile_max_mps=2.0, teleop_cap_degraded_mps=0.5)
SQ = CompiledPolygon("p-outer", "allow", "outer", True, True,
                     ((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)), None)
FENCE = CompiledFence("fs", 1, (SQ,))


def _ev(x, vx=1.0, fix="rtk_fixed"):
    return evaluate(FENCE, CFG, xy=(x, 5.0), yaw_rad=0.0, vx=vx, vy=0.0, fix_type=fix,
                    heading_valid=True)


def _kinds(evs):
    return [(e.kind, e.episode_id) for e in evs]


def test_one_excursion_shares_the_episode_then_increments():
    """mutant: no increment on soft_exit -> the second excursion is episode 0
    again -> red."""
    t = FenceEpisodeTracker()
    assert t.observe(_ev(5.0), rev=1) == []
    assert _kinds(t.observe(_ev(8.5), rev=1)) == [("soft_enter", 0)]
    assert _kinds(t.observe(_ev(9.8), rev=1)) == [("hard_clip", 0)]
    assert _kinds(t.observe(_ev(9.9), rev=1)) == []                 # E-3: still clipping, no repeat
    assert _kinds(t.observe(_ev(10.5), rev=1)) == [("breach", 0)]
    assert _kinds(t.observe(_ev(9.5, vx=-1.0), rev=1)) == [("recovered", 0)]
    assert _kinds(t.observe(_ev(5.0, vx=-1.0), rev=1)) == [("soft_exit", 0)]
    again = t.observe(_ev(8.5), rev=1)
    assert _kinds(again) == [("soft_enter", 1)]
    assert again[0].dedup_key == "fence:soft:p-outer:1" and again[0].role == "allow"
    assert _kinds(t.observe(_ev(9.8), rev=1)) == [("hard_clip", 1)]  # a new episode may clip again


def test_rev_change_forgets_the_memory():
    t = FenceEpisodeTracker()
    t.observe(_ev(8.5), rev=1)
    assert _kinds(t.observe(_ev(8.5), rev=2)) == [("soft_enter", 0)]


def test_enforcement_transitions_are_set_level_events():
    t = FenceEpisodeTracker()
    t.observe(_ev(8.5), rev=1)                                       # soft_enter, in band
    deg = t.observe(_ev(8.5, fix="dgps"), rev=1)
    assert _kinds(deg) == [("fence_degraded", 0)]
    assert deg[0].severity == "alarm" and deg[0].dedup_key == "fence:degraded:rtk_degraded"
    assert t.observe(_ev(8.5, fix="dgps"), rev=1) == []              # no repeat while degraded
    back = t.observe(_ev(8.5), rev=1)
    assert _kinds(back) == [("fence_restored", 0)]                   # and no second soft_enter: memory kept


def test_degenerate_reports_once():
    base = _ev(9.8)
    deg = FenceEval(**dict(vars(base), degenerate=True, vx=0.0, vy=0.0))
    t = FenceEpisodeTracker()
    t.observe(_ev(5.0), rev=1)
    first = t.observe(deg, rev=1)
    assert [e.kind for e in first] == ["soft_enter", "hard_clip", "fence_clip_degenerate"]
    assert first[-1].severity == "alarm" and first[-1].dedup_key == "fence:degenerate:p-outer"
    assert [e.kind for e in t.observe(deg, rev=1)] == []


def test_每一类事件都带S9A9给的去重窗口():
    """11 S9A.9 "窗口"列: 逐多边形那些 60 s, 集合级降级/恢复 300 s.

    *** 窗口必须由发布方随事件发出, p5 侧没有逐类窗口表.
    record_dao._try_merge 在[事件与未结行都没有窗口]时无条件并入 -- 于是
    进程生命期内第一条 breach 开一行, 之后每一条 breach 都静默并进去,
    dedup_count 在涨而云端只看得到一条. 事件"发出去了"却到不了, 比不发更
    坏: 它看起来是做完的.

    MUTATION: 把 fence_degraded 的窗口从 300 改成 60 -> 红;
              从 DEDUP_WINDOW_S_BY_KIND 删掉任意一行 -> 该类事件 KeyError.
    """
    t = FenceEpisodeTracker()
    t.observe(_ev(5.0), rev=1)
    got = {}
    for ev in (_ev(8.5), _ev(9.8), _ev(10.5), _ev(9.5, vx=-1.0),
               _ev(5.0, vx=-1.0)):
        for e in t.observe(ev, rev=1):
            got[e.kind] = e.dedup_window_s
    assert got == {"soft_enter": 60, "hard_clip": 60, "breach": 60,
                   "recovered": 60, "soft_exit": 60}
    # 集合级的那一对: key 里没有 episode, 抖一次就是一对 degraded/restored,
    # 所以窗口是 300 而不是 60.
    t2 = FenceEpisodeTracker()
    t2.observe(_ev(8.5), rev=1)
    deg = t2.observe(_ev(8.5, fix="dgps"), rev=1)
    assert deg[0].kind == "fence_degraded" and deg[0].dedup_window_s == 300
    back = t2.observe(_ev(8.5), rev=1)
    assert back[0].kind == "fence_restored" and back[0].dedup_window_s == 300


def test_表里没有的kind当场抛而不是取一个默认窗口():
    """新增一类事件时应当被要求补一行窗口.

    默认值会让新类静默拿到一个没人裁过的窗口 -- 那正是 CLAUDE.md S3.1 说的
    "被判为已赋值而放行".
    """
    import pytest as _pytest

    from xbrain.p1_motion.fence.episodes import FenceEvent
    bogus = FenceEvent("fence_changed", "info", "p", "n", "allow", 0, "k")
    with _pytest.raises(KeyError):
        _ = bogus.dedup_window_s
