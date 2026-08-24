"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: zones.py
Brief: 报警区入侵判定 zone_enter/zone_exit (11 S9A.9 FE-1, 报警 F2)

Description:
本文件解决的问题: 云端下发的报警区(role=warning 围栏)要在机器人[自身 pose]落入
时报 zone_enter, 退出时报 zone_exit(11 S9A.9 line 逐字"机器人进入 role=warning
多边形 -- 纯点在多边形内, FE-1; 发送方 = p1_motion"). 这是报警链 E 的执行面 --
没有它, 云端画的报警区永远不会因机器人进入而告警.

*** 与感知目标入侵是两回事. person_in_region/vehicle_in_region(v2.0 rules)判的是
[感知目标]在区内, 走 P2 可疑判定引擎(需 perception). 本文件判的是[机器人自身]
pose 在区内(FE-1), 不依赖 perception. 别把两者混做一处.

*** 本子集只做报警区(warning), 不做 forbid/speed_limit 的距离判定与硬裁剪 --
warning 从不硬执行(hard_enforce 恒 false), 纯点在多边形内, 无 d_nom/v_fence
(S9A.9 逐字"无 d_nom/v_fence").

*** 坐标(看起来会错但没错): 顶点与 pose 都是 WGS84 lat/lon, 直接拿 (lat,lon) 当
(x,y) 做点在多边形内判定. 点在多边形内是[仿射不变]的, 而小区域(巡检报警区, 百米
量级)内 lat/lon -> ENU 近似仿射, 所以内/外答案与转 ENU 后一致. NO 不需要为
inside/outside 转 ENU(转 ENU 只有距离/裁剪才需要, 本子集不做).

E-1/E-2/E-3(S9A.9 三条硬规则)在这里的落地:
  * E-1 成对同 channel: zone_enter 与 zone_exit 都归 alarm 通道(网关按 category
    分流, sev 不同 alarm/info 但 category 同 fence). 见发布侧.
  * E-2 episode_id: 一次入区过程 enter/exit [共用]同一 episode, 退出后才 +1 --
    云端据此把 enter...exit 拼成一次完整过程.
  * E-3 不刷屏: 持续在区内不重复发 enter(只在 外->内 跃迁那一拍发一次).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence

from xbrain.common.fence.geom import Polygon, point_in_polygon


@dataclass(frozen=True)
class ZoneEvent:
    """一条 zone_enter / zone_exit. 字段够发布侧组装 S9A.9 的 event/{sev}/fence:
    detail 载 poly_id/poly_name/role/episode_id, dedup_key 按 S9A.9 模板组成."""
    kind: str           # 'zone_enter' | 'zone_exit'
    severity: str       # 'alarm'(enter) | 'info'(exit) -- E-1 两者同 fence 类
    poly_id: str
    poly_name: str
    episode_id: int
    dedup_key: str      # 'fence:zone:{poly_id}:{ep}' | 'fence:zone_exit:{poly_id}:{ep}'


def _enter_key(poly_id: str, ep: int) -> str:
    return "fence:zone:%s:%d" % (poly_id, ep)


def _exit_key(poly_id: str, ep: int) -> str:
    return "fence:zone_exit:%s:%d" % (poly_id, ep)


class ZoneTracker:
    """机器人 pose 在各 warning 多边形内/外的跃迁跟踪器. observe() 每拍调一次,
    返回本拍产生的 zone 事件(通常为空). 纯 CPU, 无 I/O -- 发布由调用方做.

    *** 结构模板同 arb/visibility.ArbPublisher: 持状态, observe 返回要发的事件,
    dedup_key 只在一处组装.
    """

    def __init__(self) -> None:
        # poly_id -> 机器人当前是否在该报警区内.
        self._inside: Dict[str, bool] = {}
        # poly_id -> 当前(进行中或下一次)入区过程的 episode_id. 退出后 +1(E-2).
        self._episode: Dict[str, int] = {}
        # poly_id -> 最近一次已知的 poly_name. 围栏被删而机器人还在区内时, 用它
        # 补一条 zone_exit(此时新集里已无该多边形, 取不到 name).
        self._name: Dict[str, str] = {}

    def observe(self, lat: float, lon: float,
                warning_polys: Sequence) -> List[ZoneEvent]:
        """给定机器人 pose(WGS84)与当前 active 集的报警区(HeldPolygon 序列),
        返回本拍的 zone 事件. 调用方在 pose 不可用(无 GNSS 定位)时[不要]调本函数 --
        pose 未知不能判内外, 保持状态不动(NO 不因定位丢失捏造 enter/exit)."""
        events: List[ZoneEvent] = []
        seen = set()
        for poly in warning_polys:
            pid = poly.poly_id
            seen.add(pid)
            self._name[pid] = poly.name
            # (lat,lon) 当 (x,y): 仿射不变, 小区域内/外判定正确(见头注坐标说明).
            inside = point_in_polygon(lat, lon, Polygon(points=tuple(poly.vertices)))
            was = self._inside.get(pid, False)
            ep = self._episode.get(pid, 0)
            if inside and not was:
                events.append(ZoneEvent(
                    "zone_enter", "alarm", pid, poly.name, ep,
                    _enter_key(pid, ep)))
                self._inside[pid] = True
            elif was and not inside:
                events.append(ZoneEvent(
                    "zone_exit", "info", pid, poly.name, ep, _exit_key(pid, ep)))
                self._inside[pid] = False
                self._episode[pid] = ep + 1        # E-2: 退出后 +1
        # 报警区被删(commit 换集)而机器人还在其内 -> 补一条 zone_exit, 否则云端
        # 永远停在"在区内"(该 enter 没有配对的 exit, 违反 E-1 成对). 区没了即视为
        # 退出.
        for pid in list(self._inside.keys()):
            if self._inside.get(pid) and pid not in seen:
                ep = self._episode.get(pid, 0)
                events.append(ZoneEvent(
                    "zone_exit", "info", pid, self._name.get(pid, ""), ep,
                    _exit_key(pid, ep)))
                self._inside[pid] = False
                self._episode[pid] = ep + 1
        return events


__all__ = ["ZoneEvent", "ZoneTracker"]
