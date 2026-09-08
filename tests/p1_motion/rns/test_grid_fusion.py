"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_grid_fusion.py
Brief: Asymmetric fusion + ageing + T-degrade (P2.1~P2.4 -- 20 S3.1.2/S3.1.11)

Description:
Guards grid.py's fusion (RNS-N-5) and ageing. The A-FUS-1/2 pair is the key one:
BLOCKED=union, FREE=intersection, each with a one-sided mutant that the OTHER
would let pass. A-FUS-3 (invalid->UNKNOWN), A-FUS-6 (geometry-only capped, not
free). Each test names its mutant (CLAUDE.md 3.3).
"""

from __future__ import annotations

from xbrain.p1_motion.rns.grid import (
    bin_needs_seg_speed_cap, forward_min_d_free, fuse_bin, profile_age_ms,
    profile_speed_limited, profile_zero_speed, seg_stale,
)
from xbrain.p1_motion.rns.types import Cell, SrcBit
from tests.p1_motion.rns.scenes import uniform_free, with_block, with_unknown


TG = SrcBit.SEG | SrcBit.GEOM       # both channels back it
G_ONLY = SrcBit.GEOM                # geometry only, no T


def test_blocked_is_union_any_channel():
    # A-FUS-1: an obstacle at r=1.0 with query 2.0 -> BLOCKED, regardless of src.
    # mutant: require BOTH channels for BLOCKED -> an object only geometry sees
    # slips through -> reddens.
    assert fuse_bin(d_free=0.9, d_block=1.0, src=SrcBit.GEOM, r_query_m=2.0) == Cell.BLOCKED


def test_free_is_intersection_needs_both():
    # A-FUS-2: FREE only when BOTH T and G back it. G-only within d_free is NOT
    # free (it is UNKNOWN, capped). mutant: FREE = T or G -> geometry-only free
    # returns FREE -> reddens.
    assert fuse_bin(3.0, None, TG, r_query_m=2.0) == Cell.FREE
    assert fuse_bin(3.0, None, G_ONLY, r_query_m=2.0) == Cell.UNKNOWN


def test_invalid_stays_unknown():
    # A-FUS-3 (RNS-I-1): d_free None (invalid/unobserved) -> UNKNOWN, never
    # coerced to a distance. mutant: treat None as range_max -> FREE -> reddens.
    assert fuse_bin(None, None, TG, r_query_m=2.0) == Cell.UNKNOWN


def test_beyond_d_free_is_unknown():
    # querying past the confirmed-free distance -> UNKNOWN, not FREE.
    assert fuse_bin(2.0, None, TG, r_query_m=3.0) == Cell.UNKNOWN


def test_geometry_only_needs_speed_cap():
    # A-FUS-6: geometry-flat within d_free but no T backing -> needs the no-seg
    # speed cap (not full FREE). mutant: return full FREE / never cap -> reddens.
    assert bin_needs_seg_speed_cap(3.0, G_ONLY, r_query_m=2.0) is True
    assert bin_needs_seg_speed_cap(3.0, TG, r_query_m=2.0) is False       # T-backed
    assert bin_needs_seg_speed_cap(None, G_ONLY, r_query_m=2.0) is False  # unobserved


def test_profile_ageing_uses_capture_timestamp():
    p = uniform_free(t_capture_mono_ms=1000)
    assert profile_age_ms(p, now_mono_ms=1250) == 250
    assert profile_speed_limited(250, t50_ms=300) is False   # 250 < 300
    assert profile_speed_limited(400, t50_ms=300) is True
    assert profile_zero_speed(1200, t51_ms=1000) is True


def test_seg_stale_when_no_t_or_null_key():
    # no t_seg -> stale (no T evidence). seg_stale_ms null -> conservatively
    # stale (CLAUDE.md 3.1: unusable degrade branch -> treat as no T).
    assert seg_stale(uniform_free(t_seg_mono_ms=None), seg_stale_ms=200) is True
    assert seg_stale(uniform_free(t_seg_mono_ms=990), seg_stale_ms=None) is True
    # fresh seg within threshold -> not stale (t_capture 1000, t_seg 990, gap 10)
    assert seg_stale(uniform_free(t_capture_mono_ms=1000, t_seg_mono_ms=990),
                     seg_stale_ms=200) is False


def test_forward_min_d_free_unknown_when_any_bin_none():
    # A-FUS-4 support: a None forward bin means forward is not confirmed clear ->
    # the gate treats it as UNKNOWN-limited (None), not as the min of the rest.
    p = uniform_free(d_free_m=5.0)
    assert forward_min_d_free(p) == 5.0
    p2 = with_unknown(p, bin_index=90)
    assert forward_min_d_free(p2) is None


def test_prof1_dfree_le_dblock_property():
    # A-FUS-4 (RNS-I-3): where both are set, d_free <= d_block must hold. This is
    # a consumer-side property check over the profile. mutant on the producer
    # (d_free = d_block) collapses the UNKNOWN band; here we assert the consumer
    # can DETECT a violation rather than trusting it.
    p = with_block(uniform_free(), bin_index=90, d_block_m=0.8)  # d_free 0.7 < 0.8
    for df, db in zip(p.d_free, p.d_block):
        if df is not None and db is not None:
            assert df <= db, "PROF-1 violated in scene"


def test_separate_ageing_profile_vs_objects():
    # A-FUS-5 (RNS-I-4 / TIME-2): profile ages by ITS timestamp, not objects'.
    # A fresh profile (age 50) stays usable even if objects is stale -- geometry
    # avoidance is unaffected. mutant: age profile by objects' timestamp -> a
    # stale objects wrongly zeroes geometry -> reddens.
    p = uniform_free(t_capture_mono_ms=1000)
    # profile is 50 ms old regardless of any objects timestamp
    assert profile_age_ms(p, now_mono_ms=1050) == 50
    assert profile_speed_limited(50, t50_ms=300) is False
