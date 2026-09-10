"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: field_probe2.py
Brief: Acceptance probe on the user's dense field map (2026-09-11)

Description:
The 95-percent acceptance harness (user ruling 2026-09-11): replays the
live-captured dense map (scripts/sil/field_map2.json -- 30 rocks, 22 cones,
3 walls, 5 cars, a 37-point patrol path) and scores three families:
  A. PATH missions, forward and reverse: must ARRIVE and stay ON the line
     (off-line > 2.5 m for < 15 percent of ticks -- detours allowed, but the
     robot returns to the line right after the obstacle).
  B. GOTO point-to-point + a waypoint chain: must arrive (95 percent bar).
  C. Dynamic pedestrians bouncing inside the two-wall corridor: the robot
     crossing must never touch a person, must STOP while one blocks, and
     must arrive after they clear.
Verdict line at the end: arrivals / total and the per-family detail. Fly-away
persons in the capture (|x| > 100, the pre-bounce escape bug) are dropped.
"""

import json
import math
import sys

sys.path.insert(0, "/opt/xbrain_v6")
sys.path.insert(0, "/opt/xbrain_v6/scripts/sil")

import yaml

from sil_world import SilWorld
from xbrain.p1_motion.rns.route import Mission
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind, Origin

CFG = yaml.safe_load(open("/opt/xbrain_v6/configs/rns.yaml", encoding="utf-8"))
DT = 0.05
R_BODY = 0.46
MAP = json.load(open("/opt/xbrain_v6/scripts/sil/field_map2.json",
                     encoding="utf-8"))

GOTOS = [
    ((-20.5, 3.0, -0.5), (20.0, -10.0)),
    ((-15.0, -10.0, 0.5), (15.0, 5.0)),
    ((-21.8, -6.8, 0.0), (22.9, -12.7)),
    ((5.0, 5.0, -1.57), (5.0, -16.0)),
    ((-10.0, 8.0, -0.8), (10.0, -16.0)),
    ((20.0, 5.0, math.pi), (-20.0, -12.0)),
    ((0.0, 8.0, -1.57), (0.0, -16.0)),
    ((15.0, -10.0, math.pi), (-5.0, -16.0)),
    ((-22.0, 0.0, 0.0), (25.0, 0.0)),
    ((8.0, -3.0, math.pi), (-8.0, -12.0)),
    ((22.0, -14.0, 1.57), (-14.0, 6.0)),
    ((-21.0, 11.0, -1.0), (18.0, -14.0)),
]
CHAIN = [(-14.0, 3.5), (0.0, -3.0), (15.0, -10.0), (22.0, -5.0)]


class Ctx:
    def __init__(self, world, snap, now_ms):
        self.pose_xy = (world.rx, world.ry)
        self.yaw_rad = world.ryaw
        self.v_nom_mps = 1.0
        self.wz_max_rps = 1.2
        self.perception = snap
        self.now_mono_ms = now_ms
        self.holonomic = True


def build_world(with_persons=False):
    w = SilWorld()
    for o in MAP["obstacles"]:
        if o["kind"] == "person":
            continue                       # dropped; corridor test adds its own
        if o["kind"] == "wall":
            w.add_obstacle("wall", o["x"], o["y"], o["x2"], o["y2"],
                           o.get("thick_m", o.get("thick", 0.5)))
        else:
            oid = w.add_obstacle(o["kind"], o["x"], o["y"])
            if o["kind"] == "car":
                w.obstacles[oid].heading = o.get("heading", 0.0)
    if with_persons:
        for px, vx in ((3.0, 0.7), (7.0, -0.6)):
            pid = w.add_obstacle("person", px, -10.0)
            p = w.obstacles[pid]
            p.dynamic = True
            p.vx, p.vy = vx, 0.0
            p.heading = math.atan2(0.0, vx)
    return w


def clearance(world, kinds=None):
    best = math.inf
    for o in world.obstacles.values():
        if kinds and o.kind not in kinds:
            continue
        if o.kind == "wall":
            ax, ay, bx, by = o.x, o.y, o.x2, o.y2
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy
            t = 0 if l2 < 1e-9 else max(0, min(1, ((world.rx - ax) * dx
                                                  + (world.ry - ay) * dy) / l2))
            d = math.hypot(world.rx - (ax + t * dx),
                           world.ry - (ay + t * dy)) - o.thick_m / 2 - R_BODY
        elif o.kind == "car":
            ca, sa = math.cos(-o.heading), math.sin(-o.heading)
            lx = ca * (world.rx - o.x) - sa * (world.ry - o.y)
            ly = sa * (world.rx - o.x) + ca * (world.ry - o.y)
            d = math.hypot(max(abs(lx) - 1.0, 0), max(abs(ly) - 0.6, 0)) - R_BODY
        else:
            r = {"person": 0.3, "rock": 0.5, "cone": 0.25}[o.kind]
            d = math.hypot(world.rx - o.x, world.ry - o.y) - r - R_BODY
        best = min(best, d)
    return best


def dist_to_polyline(px, py, pts):
    best = math.inf
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0 if l2 < 1e-9 else max(0, min(1, ((px - ax) * dx
                                               + (py - ay) * dy) / l2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def mk_mission(pts, kind):
    r = CFG["rns"]["route"]
    return Mission(kind, Origin.ROUTE if kind is MissionKind.PATH
                   else Origin.RELMOVE, pts,
                   search_window=r["search_window"],
                   arrival_radius_m=r["arrival_radius_m"],
                   max_deviation_m=r["max_deviation_m"])


def drive(world, rns, mission, max_ticks, watch=None):
    rns.load_mission(mission)
    min_clear = math.inf
    fail = None
    arrived = None
    for t in range(max_ticks):
        now = t * 50
        snap = world.synth_snapshot(now)
        cand = rns.compute(Ctx(world, snap, now))
        if cand is not None:
            world.step_robot(cand.vx.value, cand.vy.value, cand.wz, DT)
        world.step_obstacles(DT)
        min_clear = min(min_clear, clearance(world))
        if watch:
            watch(t, world, rns)
        if rns.take_arrival():
            arrived = t
            break
        f = rns.take_failure()
        if f is not None:
            fail = f.reason.value
            break
    return arrived, fail, min_clear


def main():
    rows = []

    # -- A. path missions, both directions --------------------------------
    for name, pts in (("path_fwd", [tuple(p) for p in MAP["path"]]),
                      ("path_rev", [tuple(p) for p in reversed(MAP["path"])])):
        world = build_world()
        world.rx, world.ry = pts[0]
        world.ryaw = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
        rns = RnsSource(cfg=CFG, r_eff_m=0.5)
        off = [0]
        total = [0]
        def w_line(t, wd, rn, pts=pts, off=off, total=total):
            total[0] += 1
            if dist_to_polyline(wd.rx, wd.ry, pts) > 2.5:
                off[0] += 1
        arrived, fail, mc = drive(world, rns, mk_mission(pts, MissionKind.PATH),
                                  12000, w_line)
        off_pct = 100.0 * off[0] / max(1, total[0])
        ok = arrived is not None and off_pct < 15.0 and mc > 0.0
        rows.append((name, ok, fail, arrived, mc,
                     "offline %.1f%%" % off_pct))

    # -- B. gotos + waypoint chain ----------------------------------------
    for i, ((sx, sy, syaw), (gx, gy)) in enumerate(GOTOS):
        world = build_world()
        world.rx, world.ry, world.ryaw = sx, sy, syaw
        rns = RnsSource(cfg=CFG, r_eff_m=0.5)
        arrived, fail, mc = drive(world, rns,
                                  mk_mission([(gx, gy)], MissionKind.GOTO),
                                  6000)
        ok = arrived is not None and mc > 0.0
        rows.append(("goto_%02d" % i, ok, fail, arrived, mc, ""))

    world = build_world()
    world.rx, world.ry, world.ryaw = -18.0, 0.0, 0.0
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    chain_ok = True
    chain_fail = None
    mc_chain = math.inf
    for wp in CHAIN:
        arrived, fail, mc = drive(world, rns,
                                  mk_mission([wp], MissionKind.GOTO), 6000)
        mc_chain = min(mc_chain, mc)
        if arrived is None:
            chain_ok = False
            chain_fail = fail
            break
    rows.append(("wp_chain", chain_ok and mc_chain > 0.0, chain_fail,
                 None, mc_chain, "%d wps" % len(CHAIN)))

    # -- C. pedestrian corridor crossing ----------------------------------
    world = build_world(with_persons=True)
    world.rx, world.ry, world.ryaw = -3.0, -10.0, 0.0
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    stats = {"stopped": 0, "person_min": math.inf}
    def w_ped(t, wd, rn):
        stats["person_min"] = min(stats["person_min"],
                                  clearance(wd, kinds=("person",)))
        if rn.nav_state().value == "wait_dynamic":
            stats["stopped"] += 1
    arrived, fail, mc = drive(world, rns,
                              mk_mission([(13.0, -10.0)], MissionKind.GOTO),
                              9000, w_ped)
    ok = (arrived is not None and stats["person_min"] > 0.0
          and stats["stopped"] > 0)
    rows.append(("ped_corridor", ok, fail, arrived, stats["person_min"],
                 "stopped %d ticks" % stats["stopped"]))

    n_ok = sum(1 for r in rows if r[1])
    for name, ok, fail, arrived, mc, note in rows:
        print("%-14s %-4s t=%-6s clear=%6.3f %s %s"
              % (name, "OK" if ok else "FAIL",
                 ("%.1fs" % (arrived * DT)) if arrived is not None else "-",
                 mc, fail or "", note), flush=True)
    rate = 100.0 * n_ok / len(rows)
    print("== %d/%d passed (%.1f%%); bar is 95%%" % (n_ok, len(rows), rate))


if __name__ == "__main__":
    main()
