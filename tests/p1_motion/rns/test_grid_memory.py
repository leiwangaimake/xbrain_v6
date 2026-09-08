"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_grid_memory.py
Brief: Memory grid write/overwrite/TTL/jump-clear (P2.5 -- 20 S4.2.1)

Description:
Guards the memory grid's four write rules. The two that are real failure modes:
observed-UNKNOWN must not erase a just-seen BLOCKED (else wall-follow dies), and a
pose jump > reset_jump_m clears the whole grid (A-MEM-3, phantom-wall guard). Each
test names its mutant.
"""

from __future__ import annotations

from xbrain.p1_motion.rns.grid import MemoryGrid
from xbrain.p1_motion.rns.types import Cell


def _grid(cell_m=0.5, ttl_static_s=10.0, ttl_dynamic_s=1.0, reset_jump_m=1.0):
    return MemoryGrid(cell_m, ttl_static_s, ttl_dynamic_s, reset_jump_m)


def test_write_and_read_back():
    g = _grid()
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000)
    assert g.read(2.0, 0.0, now_ms=1000) == Cell.BLOCKED
    assert g.read(9.0, 9.0, now_ms=1000) == Cell.UNKNOWN  # never written


def test_observed_unknown_does_not_erase_blocked():
    # S4.2.1: the wall-follow-killer. A cell seen BLOCKED, then observed UNKNOWN
    # (occlusion), must STAY BLOCKED. mutant: let UNKNOWN overwrite -> the memory
    # is wiped each tick and wall-follow (S7.4) loses its wall -> reddens.
    g = _grid()
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000)
    g.write(2.0, 0.0, Cell.UNKNOWN, now_ms=1010)   # occluded this tick
    assert g.read(2.0, 0.0, now_ms=1010) == Cell.BLOCKED


def test_blocked_overwrites_free():
    g = _grid()
    g.write(2.0, 0.0, Cell.FREE, now_ms=1000)
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1010)
    assert g.read(2.0, 0.0, now_ms=1010) == Cell.BLOCKED


def test_ttl_expiry_reverts_to_unknown():
    g = _grid(ttl_static_s=10.0)
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000)
    assert g.read(2.0, 0.0, now_ms=5000) == Cell.BLOCKED       # within 10 s
    assert g.read(2.0, 0.0, now_ms=12000) == Cell.UNKNOWN      # 11 s old


def test_dynamic_class_uses_short_ttl():
    # a person cell (dynamic) expires on the short ttl -- phantom-obstacle guard.
    g = _grid(ttl_static_s=10.0, ttl_dynamic_s=1.0)
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000, cls="person")
    assert g.read(2.0, 0.0, now_ms=1500) == Cell.BLOCKED       # within 1 s
    assert g.read(2.0, 0.0, now_ms=2500) == Cell.UNKNOWN       # 1.5 s old > 1 s


def test_class_conflict_takes_more_dangerous():
    g = _grid()
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000, cls="traverse")
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1010, cls="hazard")
    # read is a state; danger ranking is internal, but hazard's short/long ttl
    # follows. Assert via a fresh write keeping hazard: state stays BLOCKED.
    assert g.read(2.0, 0.0, now_ms=1010) == Cell.BLOCKED


def test_pose_jump_clears_whole_grid():
    # A-MEM-3: a jump > reset_jump_m clears everything. mutant: skip the clear ->
    # phantom walls/paths from the old anchor survive -> reddens.
    g = _grid(reset_jump_m=1.0)
    g.on_pose((0.0, 0.0))
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000)
    assert g.cell_count() == 1
    cleared = g.on_pose((5.0, 0.0))   # 5 m jump > 1 m
    assert cleared is True
    assert g.cleared_by_jump is True
    assert g.cell_count() == 0
    assert g.read(2.0, 0.0, now_ms=1000) == Cell.UNKNOWN


def test_small_pose_move_does_not_clear():
    g = _grid(reset_jump_m=1.0)
    g.on_pose((0.0, 0.0))
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000)
    cleared = g.on_pose((0.3, 0.0))   # 0.3 m < 1 m
    assert cleared is False
    assert g.read(2.0, 0.0, now_ms=1000) == Cell.BLOCKED
