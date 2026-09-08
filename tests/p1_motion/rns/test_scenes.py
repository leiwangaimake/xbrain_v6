"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_scenes.py
Brief: Self-test of the scene builders (P0.6 -- RNS_TODO M-3)

Description:
The scene builders are test infrastructure; a bug in them silently corrupts
every downstream golden test. These tests pin the builder's honesty: None stays
None (RNS-I-1), a blocked bin keeps d_free < d_block (PROF-1 unless a test asks
otherwise), and the geometry-only src (no t_seg) drops bit0.
"""

from __future__ import annotations

from xbrain.p1_motion.rns.types import SrcBit
from tests.p1_motion.rns.scenes import (
    healthy_status, one_object, snapshot, uniform_free, with_block, with_unknown,
)


def test_uniform_free_all_bins_free():
    p = uniform_free(d_free_m=5.0)
    assert all(v == 5.0 for v in p.d_free)
    assert all(v is None for v in p.d_block)


def test_geometry_only_when_no_seg_drops_bit0():
    # no t_seg -> src has GEOM but not SEG (20 S3.1.11 / bit0 completeness).
    p = uniform_free(t_seg_mono_ms=None)
    assert p.has_seg() is False
    assert all((v & SrcBit.SEG) == 0 for v in p.src)
    assert all((v & SrcBit.GEOM) != 0 for v in p.src)


def test_with_block_keeps_prof1():
    # d_free < d_block for the punched bin (PROF-1 holds by construction).
    p = with_block(uniform_free(), bin_index=90, d_block_m=0.8)
    assert p.d_block[90] == 0.8
    assert p.d_free[90] < p.d_block[90]


def test_with_block_can_break_prof1_for_mutant_scenes():
    # a scene MAY set d_free >= d_block on purpose (mutant input for A-FUS-4).
    # the builder must not silently clamp it.
    p = with_block(uniform_free(), bin_index=90, d_block_m=0.8, d_free_m=0.85)
    assert p.d_free[90] == 0.85 and p.d_block[90] == 0.8


def test_with_unknown_nulls_both():
    p = with_unknown(uniform_free(), bin_index=90)
    assert p.d_free[90] is None
    assert p.d_block[90] is None


def test_one_object_default_static_person_at_3m():
    o = one_object()
    assert o.objects[0].class_name == "person"
    assert o.objects[0].r_near == 3.0
    assert o.objects[0].velocity_status == "static"


def test_snapshot_assembles_three_keys():
    s = snapshot(uniform_free(), one_object(), healthy_status())
    assert s.profile is not None
    assert s.objects is not None
    assert s.status is not None
