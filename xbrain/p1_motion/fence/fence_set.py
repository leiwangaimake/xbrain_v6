"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: fence_set.py
Brief: P1 侧接收/自算比对/持有 active FenceSet (11 S9A.2/S9A.3, 报警 F1)

Description:
本文件解决的问题: P1 是围栏"唯一执行者"(11 S9A.0), 收到 p3 在 cmd/fence 上广播的
FenceSet 后, 必须[自算 crc32 比对](S9A.2 逐字"接收方必须自算比对")再持有它 --
p3 给的 crc32 不能盲信, 报文可能在传输里损坏, 而一个损坏的围栏集用来判"机器人在
不在报警区"会给错答案.

本批(F1)只做[接收+校验+持有], 不做几何裁剪(那是安全关键的运动约束, 依赖 null 标定
参数, 本子集不碰). 持有的 active 集供 F2(zone_enter 点在多边形内)与 F3(state/fence
广播 active.rev)取用.

它[不]做什么, 边界在哪:
  * 不做 d_eff/inset/v_fence 距离判定与硬裁剪(11 S9A.6/S9A.8) -- 那是运动约束子系统,
    本子集显式不建.
  * 不做 painter 复合可行区 union -- allow/forbid 的几何叠加是裁剪用的, 报警区
    (warning)不参与.
  * 不改几何(S9A.0"不改几何") -- 只解析 p3 给的.

*** FS-7(12 S7.5): 校验失败必须[保持旧几何], 绝不进入"无围栏"状态.
一个坏 FenceSet 到达时, 若清空 active 再失败, 就等于"通信一抖围栏就消失" -- 这是
fail-open. 所以 accept() 校验不过时[抛]并保留旧 active(调用方吞掉异常, active 不动),
NO 不半更新.

*** 看起来对但会出错: 盲信 wire["crc32"].
p3 算的 crc32 只证明"p3 这么想", 不证明报文没坏. 必须用共享库 fence_set_crc32 从
polygons 重算, 与 wire["crc32"] 逐字比. 不一致即报文损坏, 拒绝(FV-8).

*** 闭集 role 越界必抛(CLAUDE.md 3.5): 未知 role 不静默透传/不降级解释. warning
旧名 zone, 但线上只认 FENCE_ROLE(allow/forbid/speed_limit/warning).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from xbrain.common.enums import FENCE_ROLE           # 11 S9A.2 闭集 (CLAUDE.md 3.5)
from xbrain.common.fence.geom import fence_set_crc32  # 跨进程单一真源 (报警 F0)


class FenceSetError(Exception):
    """收到的 FenceSet 不可用(crc32 不符 / role 越界 / 结构坏). 调用方据此保留旧
    active(FS-7), 不清空. maps 到 E_SCHEMA 边界."""


@dataclass(frozen=True)
class HeldPolygon:
    """一个已校验的围栏多边形. vertices 是 (lat, lon) 对的元组(WGS84, 首尾不重复,
    与 p3 广播一致). role 保证在 FENCE_ROLE 闭集内."""
    poly_id: str
    role: str
    name: str
    hard_enforce: bool
    vertices: Tuple[Tuple[float, float], ...]


@dataclass(frozen=True)
class HeldFenceSet:
    """P1 当前持有的 active 集. rev/crc32 是 state/fence.active 的权威三元组之二
    (F3 广播), 供 P2 版本核对(SR-2)与 D 生效确认(S-6)."""
    fence_set_id: str
    rev: int
    crc32: str
    polygons: Tuple[HeldPolygon, ...]

    def warning_polygons(self) -> Tuple[HeldPolygon, ...]:
        """报警区 = role==warning(11 S9A.2, 旧名 zone). F2 的 zone_enter 只在这些
        多边形上判点在内(warning 从不硬执行, 不参与裁剪几何)."""
        return tuple(p for p in self.polygons if p.role == "warning")


def _parse_polygon(raw: Any, idx: int) -> HeldPolygon:
    """一条 wire polygon -> HeldPolygon, 逐字段校验. role 越界即抛(不降级)."""
    if not isinstance(raw, dict):
        raise FenceSetError("polygons[%d] is not an object" % idx)
    role = raw.get("role")
    if role not in FENCE_ROLE:
        # 闭集外必抛(CLAUDE.md 3.5): 一个未知 role 若静默当普通围栏, 报警区会漏判.
        raise FenceSetError(
            "polygons[%d] role %r not in FENCE_ROLE %s"
            % (idx, role, sorted(FENCE_ROLE)))
    verts_raw = raw.get("vertices")
    if not isinstance(verts_raw, list) or len(verts_raw) < 3:
        # 少于 3 顶点围不出面积(FV-1). 一个 1~2 点的"多边形"点在内判定恒 False,
        # 报警区就此静默失效, 所以这里挡住.
        raise FenceSetError(
            "polygons[%d] needs >= 3 vertices, got %s"
            % (idx, len(verts_raw) if isinstance(verts_raw, list) else "none"))
    verts = []
    for v in verts_raw:
        if not isinstance(v, dict) or "lat" not in v or "lon" not in v:
            raise FenceSetError("polygons[%d] vertex missing lat/lon" % idx)
        verts.append((float(v["lat"]), float(v["lon"])))
    return HeldPolygon(
        poly_id=str(raw.get("poly_id")),
        role=role,
        name=str(raw.get("name") or ""),
        hard_enforce=bool(raw.get("hard_enforce")),
        vertices=tuple(verts))


def compile_fence_set(wire: Dict[str, Any]) -> HeldFenceSet:
    """wire FenceSet(11 S9A.2, p3 在 cmd/fence 上广播的形状) -> 已校验的
    HeldFenceSet. 校验不过一律抛 FenceSetError(调用方保留旧 active, FS-7).

    校验三关:
      1. 结构: fence_set_id/rev/crc32/polygons 齐全且类型对;
      2. role 闭集 + 每多边形 >= 3 顶点(_parse_polygon);
      3. crc32 自算比对(S9A.2): 用[发送方声明的] winding/hard_enforce 重算 --
         crc32 归一化含 winding, 而 P1 持有态里不留 winding(只报警用, 无所谓朝向),
         所以重算时从 wire 原样取 winding, 与 wire.crc32 比. 不一致 = 报文损坏.
    """
    if not isinstance(wire, dict):
        raise FenceSetError("FenceSet is not an object")
    fence_set_id = wire.get("fence_set_id")
    rev = wire.get("rev")
    claimed = wire.get("crc32")
    polys_raw = wire.get("polygons")
    if not isinstance(fence_set_id, str) or not fence_set_id:
        raise FenceSetError("FenceSet missing fence_set_id")
    if not isinstance(rev, int):
        raise FenceSetError("FenceSet rev must be an integer")
    if not isinstance(claimed, str) or len(claimed) != 8:
        raise FenceSetError("FenceSet crc32 must be 8 hex chars")
    if not isinstance(polys_raw, list):
        raise FenceSetError("FenceSet polygons must be an array")

    held = tuple(_parse_polygon(p, i) for i, p in enumerate(polys_raw))

    # *** crc32 自算比对(S9A.2 FV-8). 用共享库 + 发送方原样的 winding/hard_enforce.
    # winding 只在 crc32 归一化里出现, P1 持有态不留它(报警不看朝向), 故从 wire 取.
    crc_input = [
        {"poly_id": p.get("poly_id"), "role": p.get("role"),
         "winding": p.get("winding"), "hard_enforce": p.get("hard_enforce"),
         "vertices": p.get("vertices")}
        for p in polys_raw]
    recomputed = fence_set_crc32(fence_set_id, rev, crc_input)
    if recomputed != claimed:
        raise FenceSetError(
            "crc32 mismatch: wire=%s recomputed=%s (frame corrupt, FV-8)"
            % (claimed, recomputed))

    return HeldFenceSet(fence_set_id=fence_set_id, rev=rev, crc32=claimed,
                        polygons=held)


class FenceSetHolder:
    """P1 进程内持有 active FenceSet 的单点. cmd/fence 回调每次调 accept().

    *** Zenoh 回调纪律(CLAUDE.md 4.2): accept() 是纯 CPU(解析+校验)+ 一次原子
    赋值(self._active = ...), NO 不 await/不发布. 赋值在 GIL 下原子, 读者(F2/F3)
    随时读到的要么是旧的完整集要么是新的完整集, 不会读到半更新.

    *** FS-7: accept() 抛异常时 self._active 不动 -- 坏帧保留旧几何, 不清空.
    """

    def __init__(self) -> None:
        self._active: Optional[HeldFenceSet] = None

    @property
    def active(self) -> Optional[HeldFenceSet]:
        return self._active

    def accept(self, wire: Dict[str, Any]) -> HeldFenceSet:
        """校验并换入 active. 成功返回新 active; 失败抛 FenceSetError 且 active
        不变(FS-7). 幂等: 同 rev+crc32 再来一次是无害的重复换入(值相同)."""
        compiled = compile_fence_set(wire)          # 抛则 active 不动
        self._active = compiled                     # 原子换指针
        return compiled


__all__ = ["FenceSetError", "HeldPolygon", "HeldFenceSet", "compile_fence_set",
           "FenceSetHolder"]
