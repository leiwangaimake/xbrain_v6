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

from xbrain.common.fence.roles import hard_enforce_for_role
from xbrain.p3_task.fence.fence_set import (
    build_fence_set,
    fence_set_crc32,
)
from xbrain.p3_task.fence.geom import InvalidFenceSet

pytestmark = pytest.mark.no_device


def _row(fid, name, role, pts, hard=None, priority=0, limit=None):
    """One FencesDAO.list_active() row: (fence_id, name, role, kind, geom_json,
    hard_enforce, rev, priority, speed_limit_mps).

    hard_enforce is derived the way the DAO derives it -- false for BOTH
    advisory roles (11 S9A.2: warning AND speed_limit), which is why this
    helper calls the shared function instead of re-spelling the rule. `hard`
    overrides it so a test can hand the builder a row the writers would have
    refused, which is the only way to check what the builder does with one.
    """
    geom = json.dumps({"points": pts})
    he = (1 if hard_enforce_for_role(role) else 0) if hard is None else hard
    # speed_limit zones get a limit by default so a caller does not have to
    # remember FV-13; every other role must carry None (the reverse half).
    if limit is None and role == "speed_limit":
        limit = 1.0
    return (fid, name, role, "polygon", geom, he, 1, priority, limit)


_ALLOW = _row("f-lake", "环湖活动区", "allow",
              [[31.20, 121.50], [31.21, 121.50], [31.21, 121.51]])
_FORBID = _row("f-oil", "油库禁区", "forbid",
               [[31.203, 121.503], [31.205, 121.503], [31.204, 121.505]])


def test_crc32_matches_the_9A2_block_b3_normalisation():
    """手工复现 11 S9A.2 [块 B-3]配方, 断言 builder 的 crc 等于它.

    *** 七段, 不是五段. 块 B-3 在 hard_enforce 之后插了 priority, 再插一段
    speed_limit_mps(role != speed_limit 时写[空串], 但分隔符仍在) --
    本用例原先手抄的是 v0.2 的五段形态, 而它当时能通过, 恰恰因为
    common/fence/geom.py 里那份实现也是 v0.2 的: 两个过时互相印证, 于是
    FV-8 与本判据[同时]通过而线上配方是错的.
    MUTATION: 任何偏差(字段顺序 / 缺一个 '|' / 少写 priority 那一段 /
    %.6f 代替 %.8f / 给顶点排序)都改 crc 并打红本条.
    """
    fs = build_fence_set([_ALLOW], fence_set_id="fs-x", rev=7)
    p = fs["polygons"][0]
    # allow 不带 speed_limit_mps, 所以第六段是空串 -- 下面 "|%s|" 之间什么都
    # 没有的那一处就是它. 空段必须[仍然]产生一个分隔符, 否则两个本该不同的
    # FenceSet 会把字段挤在一起而哈希成同一个值.
    s = "fs-x|7|" + "%s|%s|%s|1|%d||" % (
        p["poly_id"], p["role"], p["winding"], p["priority"])
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


def test_priority_is_on_every_polygon_and_speed_limit_only_on_its_own_role():
    """11 S9A.2 block B-2 的两个 v0.3 字段怎么上线.

    priority 每个多边形都有(必填); speed_limit_mps 只在 role=speed_limit 上
    出现, 其余 role 必须 OMIT 键而不是发 null -- 发 null 会被接收方的 FV-13
    反向判据当成"带了该字段"而整条拒收, 于是每个非限速多边形都被拒.
    MUTATION: 把 if carries_speed_limit(role) 换成无条件赋值 -> allow 多边形
    上出现 speed_limit_mps 键, 本条红.
    """
    slow = _row("f-yard", "车场慢行区", "speed_limit",
                [[31.206, 121.506], [31.207, 121.506], [31.2065, 121.507]],
                priority=10, limit=1.0)
    fs = build_fence_set([_ALLOW, slow], fence_set_id="fs-a", rev=1)
    pa = next(p for p in fs["polygons"] if p["poly_id"] == "f-lake")
    ps = next(p for p in fs["polygons"] if p["poly_id"] == "f-yard")
    # priority: 两个都有, 取各自的值.
    assert pa["priority"] == 0 and ps["priority"] == 10
    # speed_limit_mps: 只有限速区有.
    assert ps["speed_limit_mps"] == 1.0
    assert "speed_limit_mps" not in pa
    # 限速区从不硬执行(S9A.2 恒 false). 这条与 roles 的单测重叠是故意的 --
    # 那条测函数, 这条测它真的被 wire 组装用上了.
    assert ps["hard_enforce"] is False


def test_a_speed_limit_row_with_no_limit_refuses_the_whole_set():
    """限速区没有限速值 = 一个无法施加的区. 抛 InvalidFenceSet 而不是让
    float(None) 抛 TypeError.

    为什么这条路径还存在: 本机 fence.db 的行写在该列存在之前, 而 SQLite 不能
    给已有表 ALTER 出表级 CHECK. 三个写入方都已挡住, 这是读出侧的兜底.
    MUTATION: 删掉这个 raise -> float(None) 抛 TypeError -> 落到调用方的
    generic handler, 于是 1 Hz 广播循环每秒记一条 error 而不是只记一次.
    """
    legacy = _row("f-old", "老慢行区", "speed_limit",
                  [[31.208, 121.508], [31.209, 121.508], [31.2085, 121.509]],
                  limit=0)                 # 0 不是 None, 下面显式换成 None
    legacy = legacy[:8] + (None,)
    with pytest.raises(InvalidFenceSet, match="speed_limit_mps"):
        build_fence_set([_ALLOW, legacy], fence_set_id="fs-a", rev=1)


def test_a_legacy_hard_enforcing_speed_limit_row_is_corrected_on_the_wire():
    """旧行的 hard_enforce 不透传, 按 role 现场派生.

    *** 这不是假设的情形: 本机 data/run/fence.db 的 f-speed(中心限速区,
    state=active)就是 role=speed_limit 且 hard_enforce=1 -- 由只认 warning
    的旧规则写进去的. 透传它 = 把一个操作员标为"提示性"的区域当禁入区发出去,
    一旦 speed_limit 进裁剪几何, 机器人就进不了中心区.
    MUTATION: 改回 bool(_hard_enforce) -> 本条红(它会原样带出 True).
    """
    legacy = _row("f-speed", "中心限速区", "speed_limit",
                  [[31.21, 121.52], [31.22, 121.52], [31.215, 121.53]],
                  hard=1, limit=1.0)          # hard=1: 旧规则留下的值
    fs = build_fence_set([_ALLOW, legacy], fence_set_id="fs-a", rev=1)
    ps = next(p for p in fs["polygons"] if p["poly_id"] == "f-speed")
    assert ps["hard_enforce"] is False
    # 反向也要成立: allow 不会被"顺手归一化"成不硬执行.
    pa = next(p for p in fs["polygons"] if p["poly_id"] == "f-lake")
    assert pa["hard_enforce"] is True
