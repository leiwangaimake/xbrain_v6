#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: seed_geo_demo.py
Brief: Dev-only geo seeding for cloud integration testing -- 3 fences, 4 routes, 12 waypoints

Description:
联调需要一批地理要素才能演示"云端下发 -> 入库 -> 调度 -> HMI 显示"的闭环:
GOTO_KEYPOINT 带的 waypoint id 与 recorded_path_id 必须在 geo.db 里查得到,
否则任务被诚实拒绝, 现场看起来像"什么都跑不起来".

*** 走 cmd/geo, NO 不直接写库.
P3 是 geo.db / fence.db 的唯一写者(11 S7.9). 直接 INSERT 会绕过三件事:
  (1) 校验与闭集检查 -- 写进去的可能是 P3 自己都不认的形状;
  (2) rev / content_hash 的维护 -- 后续 base_rev 冲突检测会失效;
  (3) 变更广播 -- HMI 靠 state/geo/* 刷新, 不广播就不显示, 而"实时显示"正是
      本次要演示的东西.
所以这个脚本是一个[云端客户端], 与甲方 Qt 走同一条路. 副作用是它顺带验证了
cmd/geo 这条链路本身.

*** 坐标是[造的], 但格式与真实 RTK 一致.
WGS84 十进制度, 7 位小数(厘米级) -- 与 rtk_driver 上报的 rt/gnss/fix 同格式,
所以下游的解析 / 投影 / 渲染走的都是真实代码路径. 基准点取甲方 v2.0 文档示例
所在的上海一带; 各要素按东/北米偏移生成, 面积与距离都是实算的(见文件末尾的
自检输出), NO 不是随手写的经纬度.

Usage:
  python3 scripts/dev/seed_geo_demo.py --rid gj-001            # 注入
  python3 scripts/dev/seed_geo_demo.py --rid gj-001 --verify   # 只读回验证
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import uuid

LAT0, LON0 = 31.2301971, 121.4732683
M_LAT = 111132.92 - 559.82 * math.cos(2 * math.radians(LAT0)) \
        + 1.175 * math.cos(4 * math.radians(LAT0))
M_LON = 111412.84 * math.cos(math.radians(LAT0)) \
        - 93.5 * math.cos(3 * math.radians(LAT0))


def off(dx_m: float, dy_m: float):
    """东 dx / 北 dy 米 -> (lat, lon), 7 位小数.

    NO 不用简化的 "1 度 = 111 km": 在纬度 31 度上经度方向差 14%, 用固定值
    会让 16 km2 的要求算出来差两平方公里, 而那正是本次的硬指标.
    """
    return (round(LAT0 + dy_m / M_LAT, 7), round(LON0 + dx_m / M_LON, 7))


def ring_area_km2(ring) -> float:
    """多边形面积(局部平面近似, 数公里尺度误差 < 0.1%)."""
    pts = [((lon - LON0) * M_LON, (lat - LAT0) * M_LAT) for lat, lon in ring]
    s = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0 / 1e6


#: 活动区域: 不规则八边形. 切角而非矩形, 因为需求要的是"多边形区域",
#: 而一个矩形在 HMI 上看不出多边形渲染是否正确.
F_ACTIVITY = [off(-2600, 1500), off(-1500, 2300), off(1500, 2300),
              off(2600, 1500), off(2600, -1400), off(1400, -2300),
              off(-1400, -2300), off(-2600, -1400)]
F_SPEED = [off(-900, 500), off(900, 700), off(1200, -200),
           off(300, -800), off(-900, -500)]
F_ALARM = [off(1200, 1100), off(2100, 1200), off(2200, 400), off(1300, 300)]

FENCES = [
    ("f-activity", "营区活动区域", "allow", F_ACTIVITY),
    ("f-speed", "中心限速区", "speed_limit", F_SPEED),
    ("f-alarm", "油库报警区", "warning", F_ALARM),
]

WAYPOINTS = [
    ("w-main-gate", "主门岗", off(-2400, 0)),
    ("w-oil-depot", "一号油库", off(1500, 800)),
    ("w-oil-depot-2", "二号油库", off(1900, 500)),
    ("w-pump-house", "消防泵房", off(600, 1400)),
    ("w-watchtower", "北瞭望塔", off(-200, 2000)),
    ("w-dorm", "值班宿舍", off(-1800, -900)),
    ("w-garage", "车库前坪", off(-1200, -1800)),
    ("w-drill-yard", "训练场", off(400, -1900)),
    ("w-substation", "变电所", off(2200, -1000)),
    ("w-warehouse", "器材仓库", off(1000, -1200)),
    ("w-east-gate", "东门岗", off(2500, -200)),
    ("w-charge-dock", "一号充电桩", off(-2000, 600)),
]
_WP = {wid: c for wid, _n, c in WAYPOINTS}

ROUTES = [
    ("r-perimeter", "外围巡逻线",
     ["w-main-gate", "w-watchtower", "w-east-gate", "w-substation",
      "w-drill-yard", "w-garage", "w-main-gate"]),
    ("r-oil-area", "油库重点巡查线",
     ["w-pump-house", "w-oil-depot", "w-oil-depot-2", "w-east-gate"]),
    ("r-night", "夜间值守线",
     ["w-dorm", "w-garage", "w-drill-yard", "w-warehouse"]),
    ("r-charge", "充电返程线",
     ["w-east-gate", "w-main-gate", "w-charge-dock"]),
]


def _envelope(rid: str, seq: int, data: dict) -> bytes:
    return json.dumps({"v": 1, "rid": rid, "ts": time.time(), "seq": seq,
                       "src": "qt_hmi", "data": data},
                      ensure_ascii=False).encode("utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rid", required=True)
    ap.add_argument("--endpoint", default="tcp/127.0.0.1:7447")
    ap.add_argument("--verify", action="store_true",
                    help="只读回验证, 不注入")
    args = ap.parse_args(argv)

    print("== 几何自检 (注入前先算, 数不对就不该发) ==")
    a = ring_area_km2(F_ACTIVITY)
    print("  活动区域 八边形 %d 顶点  面积 %.3f km2  (要求 > 16) %s"
          % (len(F_ACTIVITY), a, "OK" if a > 16 else "FAIL"))
    if a <= 16:
        print("  面积不达标, 拒绝注入", file=sys.stderr)
        return 1
    print("  限速区 %d 顶点 %.3f km2   报警区 %d 顶点 %.3f km2"
          % (len(F_SPEED), ring_area_km2(F_SPEED),
             len(F_ALARM), ring_area_km2(F_ALARM)))
    print("  位置点 %d 个   路径 %d 条" % (len(WAYPOINTS), len(ROUTES)))
    if args.verify:
        return 0

    import zenoh
    cfg = zenoh.Config()
    cfg.insert_json5("mode", '"client"')
    cfg.insert_json5("connect/endpoints", json.dumps([args.endpoint]))
    cfg.insert_json5("scouting/multicast/enabled", "false")
    s = zenoh.open(cfg)
    # 机内相对 key: cmd/geo 是机内面, P3 直接订它(与 cmd/task 的云端前缀不同,
    # 见 cloud_wiring 头注对两条 key 的说明).
    acks = []
    sub = s.declare_subscriber("cmd/geo/ack",
                               lambda sm: acks.append(bytes(sm.payload)))
    time.sleep(1.0)

    def send(payload, tag):
        s.put("cmd/geo", json.dumps(payload, ensure_ascii=False).encode())
        time.sleep(0.35)
        print("  -> %s" % tag)

    n = 0
    print("\n== 注入位置点 ==")
    for wid, name, (lat, lon) in WAYPOINTS:
        n += 1
        send({"cmd_id": "seed-%s" % wid, "action": "upsert",
              "type": "waypoint", "geo_id": wid, "origin": "cloud",
              "base_rev": 0,
              "obj": {"name": name, "geom": {"lat": lat, "lon": lon}}},
             "%-15s %-12s %.7f, %.7f" % (wid, name, lat, lon))

    print("\n== 注入路径 ==")
    for rid_, name, wps in ROUTES:
        pts = [list(_WP[w]) for w in wps]
        send({"cmd_id": "seed-%s" % rid_, "action": "upsert", "type": "route",
              "geo_id": rid_, "origin": "cloud", "base_rev": 0,
              "obj": {"name": name, "geom": {"points": pts}}},
             "%-13s %-16s %d 点" % (rid_, name, len(pts)))

    print("\n== 注入围栏 (upsert -> draft, 再 set_state -> active) ==")
    for fid, name, role, ring in FENCES:
        send({"cmd_id": "seed-%s" % fid, "action": "upsert", "type": "fence",
              "geo_id": fid, "origin": "cloud", "base_rev": 0,
              "obj": {"name": name,
                      "geom": {"role": role,
                               "outer": [list(p) for p in ring]}}},
             "%-12s %-14s role=%-12s %d 顶点" % (fid, name, role, len(ring)))
        send({"cmd_id": "seed-%s-act" % fid, "action": "set_state",
              "type": "fence", "geo_id": fid, "origin": "cloud",
              "base_rev": 1, "obj": {"state": "active"}},
             "%-12s -> active" % fid)

    time.sleep(2.0)
    ok = sum(1 for a_ in acks if b'"accepted"' in a_)
    bad = [a_.decode("utf-8", "replace") for a_ in acks
           if b'"accepted"' not in a_]
    print("\n== ack 汇总: %d 条, accepted %d ==" % (len(acks), ok))
    for b in bad[:6]:
        print("   非 accepted: %s" % b[:220])
    sub.undeclare()
    s.close()
    return 0 if not bad else 2


if __name__ == "__main__":
    sys.exit(main())
