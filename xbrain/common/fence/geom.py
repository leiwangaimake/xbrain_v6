"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: geom.py
Brief: 跨进程共享的围栏几何原语 + FenceSet crc32 (11 S9A.2)

Description:
本文件解决的问题: p3_task(围栏权威源, 发 cmd/fence)与 p1_motion(围栏唯一执行者,
自算 crc32 比对 + 点在多边形内判定报警区)必须用[逐字节相同]的 crc32 归一化和
[同一套]点在多边形内算法 -- 否则 p1 每次自算比对都失败(11 S9A.2"接收方必须
自算比对"), 或两侧对"机器人是否在报警区内"给出不同答案.

不把这些放进任一进程私有目录, 是因为一旦 p1 和 p3 各存一份 crc32 归一化,
两份迟早漂移(改了一处忘了另一处), 而 crc32 不一致的现象是"围栏永远校验不过",
与网络坏了不可区分. 共享单一真源即杜绝这类漂移(CLAUDE.md 3.5 的精神: 闭集/
算法单一导出, 不字面重复).

它[不]做什么, 边界在哪:
  * 不做 FenceSet 的[构造](那是 p3 从 fence.db 行整形, 见 p3_task/fence/
    fence_set.py), 也不做 painter 复合可行区 union / 录制期多边形校验(那些是
    p3 私有, 留在 p3_task/fence/geom.py).
  * 不做坐标系转换 -- point_in_polygon 在[同一平面](x,y)内判定, 调用方负责先把
    pose 与顶点转到同一系(WGS84 或 ENU).
  * fence_set_crc32 [不再自己实现]归一化(2026-10-08): 配方的唯一实现在
    common/digest/fence.py(块 B-3: 每多边形七段, 含 priority 与 role=
    speed_limit 的 speed_limit_mps), 本函数只做签名适配. 金标向量
    tests/common/golden/fence_crc32_vectors.json 是跨语言契约, 改归一化
    就是改契约.
    *** 本文件原先自带一份[v0.2]配方(只有 poly_id/role/winding/hard_enforce
    + 顶点), 而块 B-3 加的两个字段只落到了 digest 模块与 C++ 头 ==> 两份对的
    没人调, 线上调的那份是过时的. 后果见 fence_set_crc32 的 docstring.

看起来对但会出错的写法: 用 %g / repr(float) 序列化顶点 -- 不同实现给不同位数,
crc32 立刻跨语言不一致. 必须 %.8f 定点(11 S9A.2 逐字).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Sequence

# The block B-3 recipe. digest/fence.py imports nothing of ours on purpose (it
# is a leaf, so the C++ twin can be read against it), which is why the arrow
# points this way and not the other.
from xbrain.common.digest.fence import fence_crc32


@dataclass(frozen=True)
class Circle:
    cx: float
    cy: float
    r: float


@dataclass(frozen=True)
class Polygon:
    points: tuple    # ((x, y), ...) -- 同一平面内, 首尾不重复


def point_in_circle(x: float, y: float, c: Circle) -> bool:
    return math.hypot(x - c.cx, y - c.cy) <= c.r


def point_in_polygon(x: float, y: float, poly: Polygon) -> bool:
    """Ray-cast: cast horizontal ray from (x, y), count crossings.
    Boundary points may be reported either way -- acceptable for
    fence use (a robot at boundary is inside a safe region)."""
    n = len(poly.points)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly.points[i]
        xj, yj = poly.points[j]
        # +1e-12 guards the horizontal-edge divide-by-zero; the ray is cast
        # rightward so only edges straddling y in the +x direction flip state.
        crosses = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        if crosses:
            inside = not inside
        j = i
    return inside


def fence_set_crc32(fence_set_id: str, rev: int,
                    polygons: Sequence[Dict[str, Any]]) -> str:
    """11 S9A.2 block B-3 crc32, by delegation to the one implementation of it.

    This function keeps its (id, rev, polygons) signature -- it is what p3's
    build_fence_set and p1's FV-8 self-check both call -- and hands the recipe
    itself to common/digest/fence.py, which the C++ twin in
    common/include/xbrain/digest/canonical_digest.h is written against line by
    line and which the cross-language golden vectors pin.

    *** Why this is a delegation and not an implementation. It WAS an
    implementation: the v0.2 recipe, poly_id|role|winding|hard_enforce| plus
    vertices. Block B-3 added two fields to the string -- priority and, on a
    speed_limit polygon, its limit -- and both the C++ header and the digest
    module were updated; this function was not. So the two correct
    implementations were the ones nobody called at run time, and the live path
    (p3's broadcast, p1's self-check) used the superseded recipe. S9A.2 states
    the consequence in its own words: "改限速/改叠放次序得到相同 crc32" ==>
    S-5 answers duplicate ==> "改了不生效, 而且不报任何错". On the real
    broadcast path that is worse than it sounds, because p3's main_wiring uses
    this very checksum as the CHANGE DETECTOR that drives fence_rev += 1 -- a
    limit edit produced the same signature, so not even the revision moved.

    Three implementations of one recipe is how two of them end up being right
    and the third one shipping. Now there is one, and the duplicate that
    remains (the C++ header) is pinned to it by test_digest_cross_language.
    """
    return fence_crc32({"fence_set_id": fence_set_id, "rev": rev,
                        "polygons": list(polygons)})


__all__ = ["Circle", "Polygon", "point_in_circle", "point_in_polygon",
           "fence_set_crc32"]
