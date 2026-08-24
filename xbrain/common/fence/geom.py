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
  * fence_set_crc32 的归一化[逐字节][对 C++ 接收方也成立]: 顶点用 %.8f 定点,
    role/winding/hard_enforce 顺序固定 -- 金标向量 tests/common/golden/
    fence_crc32_vectors.json 是跨语言契约, 改归一化就是改契约.

看起来对但会出错的写法: 用 %g / repr(float) 序列化顶点 -- 不同实现给不同位数,
crc32 立刻跨语言不一致. 必须 %.8f 定点(11 S9A.2 逐字).
"""

from __future__ import annotations

import math
import zlib
from dataclasses import dataclass
from typing import Any, Dict, Sequence


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
    """11 S9A.2 crc32: fence_set_id|rev| then per polygon (array order)
    poly_id|role|winding|hard_enforce(1/0)| then each vertex '%.8f,%.8f;'.
    CRC-32 IEEE 802.3 (== zlib.crc32), 8 lowercase hex. The receiver self-computes
    and compares (FV-8); a byte-identical set MUST yield the same crc32."""
    parts = ["%s|%d|" % (fence_set_id, rev)]
    for p in polygons:
        parts.append("%s|%s|%s|%s|" % (
            p["poly_id"], p["role"], p["winding"],
            "1" if p["hard_enforce"] else "0"))
        for v in p["vertices"]:
            parts.append("%.8f,%.8f;" % (v["lat"], v["lon"]))
    data = "".join(parts).encode("utf-8")
    return "%08x" % (zlib.crc32(data) & 0xFFFFFFFF)


__all__ = ["Circle", "Polygon", "point_in_circle", "point_in_polygon",
           "fence_set_crc32"]
