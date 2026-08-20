"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fence_set.py
Brief: FenceSet builder + crc32 normalisation (11 S9A.2 / S9A.3 broadcast half)

Description:
Pins the fence.db -> cmd/fence broadcast builder: crc32 follows the EXACT 11 S9A.2
normalisation (so the C++/Python receiver self-computes the same value, FV-8), the
crc changes on any geometry/rev change and is stable otherwise, the FenceSet shape
carries role/name/winding/hard_enforce/{lat,lon} vertices, and an invalid active
set (11 S9A.1A FS-5A) is refused before it can go on the wire. Each check names the
mutation it reddens (CLAUDE.md 3.3).
"""
from __future__ import annotations

import json
import zlib

import pytest

from xbrain.p3_task.fence.fence_set import (
    build_fence_set, fence_set_crc32,
)
from xbrain.p3_task.fence.geom import InvalidFenceSet

pytestmark = pytest.mark.no_device


def _row(fid, name, role, pts, hard=None):
    geom = json.dumps({"points": pts})
    he = (0 if role == "warning" else 1) if hard is None else hard
    return (fid, name, role, "polygon", geom, he, 1)


_ALLOW = _row("f-lake", "环湖活动区", "allow",
              [[31.20, 121.50], [31.21, 121.50], [31.21, 121.51]])
_FORBID = _row("f-oil", "油库禁区", "forbid",
               [[31.203, 121.503], [31.205, 121.503], [31.204, 121.505]])


def test_crc32_matches_the_9A2_normalisation():
    # Reproduce the 11 S9A.2 rule by hand and assert the builder's crc equals it.
    # MUTATION: any deviation (wrong field order, missing '|', %.6f instead of
    # %.8f, sorting vertices) changes the crc and reddens this.
    fs = build_fence_set([_ALLOW], fence_set_id="fs-x", rev=7)
    p = fs["polygons"][0]
    s = "fs-x|7|" + "%s|%s|%s|1|" % (p["poly_id"], p["role"], p["winding"])
    for v in p["vertices"]:
        s += "%.8f,%.8f;" % (v["lat"], v["lon"])
    expect = "%08x" % (zlib.crc32(s.encode("utf-8")) & 0xFFFFFFFF)
    assert fs["crc32"] == expect
    assert len(fs["crc32"]) == 8 and fs["crc32"] == fs["crc32"].lower()


def test_crc32_rev_and_geometry_sensitive_else_stable():
    base = build_fence_set([_ALLOW, _FORBID], fence_set_id="fs-a", rev=1)["crc32"]
    # same input -> same crc (byte-identical set, 11 S9A.2 rev-unchanged rule).
    assert build_fence_set([_ALLOW, _FORBID], fence_set_id="fs-a", rev=1)["crc32"] == base
    # rev change -> crc changes (rev is in the crc input).
    assert build_fence_set([_ALLOW, _FORBID], fence_set_id="fs-a", rev=2)["crc32"] != base
    # a moved vertex -> crc changes. MUTATION: hashing only ids would miss it.
    moved = _row("f-oil", "油库禁区", "forbid",
                 [[31.203, 121.503], [31.205, 121.503], [31.2041, 121.505]])
    assert build_fence_set([_ALLOW, moved], fence_set_id="fs-a", rev=1)["crc32"] != base


def test_build_fence_set_shape():
    fs = build_fence_set([_ALLOW, _FORBID], fence_set_id="fs-a", rev=3)
    assert fs["fence_set_id"] == "fs-a" and fs["rev"] == 3
    pa = fs["polygons"][0]
    assert pa["poly_id"] == "f-lake" and pa["name"] == "环湖活动区"
    assert pa["role"] == "allow" and pa["winding"] == "ccw" and pa["hard_enforce"] is True
    # vertices are {lat,lon} objects (11 S9A.2), the frontend/receiver form.
    assert pa["vertices"][0] == {"lat": 31.20, "lon": 121.50}
    # forbid keep-out declares cw winding (receiver still re-normalises).
    assert fs["polygons"][1]["winding"] == "cw"


def test_warning_hard_enforce_false_flows_through():
    warn = _row("f-gate", "岗亭报警区", "warning",
                [[31.201, 121.501], [31.202, 121.501], [31.2015, 121.502]])
    fs = build_fence_set([_ALLOW, warn], fence_set_id="fs-a", rev=1)
    wp = next(p for p in fs["polygons"] if p["poly_id"] == "f-gate")
    # MUTATION: a warning polygon that hard-enforces would let an alarm-only zone clip.
    assert wp["hard_enforce"] is False


def test_build_refuses_invalid_active_set():
    # 11 S9A.1A FS-5A / FV-3: 0 allow and >= 2 allow both refused BEFORE broadcast.
    # MUTATION: dropping the validate call lets a set with no activity area go live.
    with pytest.raises(InvalidFenceSet):
        build_fence_set([_FORBID], fence_set_id="fs-a", rev=1)          # 0 allow
    allow2 = _row("f-lake2", "活动区2", "allow",
                  [[31.30, 121.60], [31.31, 121.60], [31.31, 121.61]])
    with pytest.raises(InvalidFenceSet):
        build_fence_set([_ALLOW, allow2], fence_set_id="fs-a", rev=1)   # 2 allow


def test_crc32_helper_is_pure():
    # fence_set_crc32 is the shared normalisation both sides call. Same polygons
    # + same (id, rev) -> same value, no hidden state. MUTATION: any per-call
    # nondeterminism (dict order, timestamp) would break receiver agreement.
    fs = build_fence_set([_ALLOW, _FORBID], fence_set_id="fs-a", rev=4)
    again = fence_set_crc32("fs-a", 4, fs["polygons"])
    assert again == fs["crc32"]
