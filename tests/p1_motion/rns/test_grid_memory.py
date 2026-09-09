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


def test_class_conflict_latest_observation_wins():
    # 20 S4.2.1 v1.16 (user ruling 2026-09-09): the LATEST classed observation
    # sets the cell's class -- and thus its TTL tier. A fresh person over an
    # UNEXPIRED hazard must take the DYNAMIC short ttl; the old danger-rank
    # merge kept hazard (higher rank) and the person cell lingered on the
    # static long ttl (phantom wall). mutant: restore the danger-rank merge ->
    # the 3.5 s read below stays BLOCKED -> reddens.
    g = _grid(ttl_static_s=10.0, ttl_dynamic_s=1.0)
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000, cls="hazard")
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=2000, cls="person")  # hazard NOT expired
    assert g.read(2.0, 0.0, now_ms=2500) == Cell.BLOCKED   # within dynamic 1 s
    assert g.read(2.0, 0.0, now_ms=3500) == Cell.UNKNOWN   # person ttl expired
    # and the mirror: latest hazard over person takes the static tier.
    g2 = _grid(ttl_static_s=10.0, ttl_dynamic_s=1.0)
    g2.write(4.0, 0.0, Cell.BLOCKED, now_ms=1000, cls="person")
    g2.write(4.0, 0.0, Cell.BLOCKED, now_ms=1500, cls="hazard")
    assert g2.read(4.0, 0.0, now_ms=4000) == Cell.BLOCKED  # static ttl holds


def test_classless_write_keeps_known_class():
    # a class-less write (pure geometry) keeps the known class: no semantic info
    # this tick is not evidence the class changed.
    g = _grid(ttl_static_s=10.0, ttl_dynamic_s=1.0)
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000, cls="person")
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1200)            # no cls
    assert g.read(2.0, 0.0, now_ms=1900) == Cell.BLOCKED    # dynamic ttl from 1200
    assert g.read(2.0, 0.0, now_ms=2900) == Cell.UNKNOWN    # still person tier


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


def test_expired_class_does_not_pollute_fresh_dynamic_ttl():
    # B6 (review fix): an entry past TTL is UNKNOWN on the read side; the write
    # side must agree, or the expired entry's class pollutes the merge. The
    # observable damage needs classes with DIFFERENT TTLs: an EXPIRED static
    # hazard (long ttl) under a FRESH person (dynamic, short ttl). Before the
    # fix, the merge kept hazard (higher rank), so the person cell read with
    # the STATIC ttl and lingered as a phantom long after the person left.
    # mutant: skip the expiry check on write -> the 13.5 s read below stays
    # BLOCKED (hazard ttl holds it) -> reddens.
    g = _grid(ttl_static_s=10.0, ttl_dynamic_s=1.0)
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000, cls="hazard")
    # 12 s later the hazard entry is EXPIRED (10 s ttl). A person arrives:
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=12000, cls="person")
    # fresh person -> DYNAMIC ttl (1 s): visible at 12.5 s, gone at 13.5 s.
    assert g.read(2.0, 0.0, now_ms=12500) == Cell.BLOCKED
    assert g.read(2.0, 0.0, now_ms=13500) == Cell.UNKNOWN


def test_expired_entry_does_not_block_unknown_write():
    # B6 companion: an observed-UNKNOWN write over an EXPIRED blocked entry must
    # land (the entry is dropped); over a FRESH blocked entry it must still be
    # refused (the S4.2.1 occlusion rule is about fresh memory, not corpses).
    g = _grid(ttl_static_s=1.0)
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=1000, cls="hazard")
    g.write(2.0, 0.0, Cell.UNKNOWN, now_ms=3000)   # expired -> entry dropped
    # a fresh BLOCKED then starts CLEAN (no hazard class inherited):
    g.write(2.0, 0.0, Cell.BLOCKED, now_ms=3010, cls="traverse")
    assert g.read(2.0, 0.0, now_ms=3500) == Cell.BLOCKED
