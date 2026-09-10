"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: batch_probe.py
Brief: Headless batch navigation probe over the 2026-09-10 field map

Description:
Regression probe for the RNS guidance/avoidance stack (20 S4A tuning tool,
promoted into scripts/ after twice being lost from /tmp on device reboots).
Replays the user's field map (8-car column + 14 m wall, corridor between) and
sweeps start x goal pairs covering the wall's far side, the corridor, the car
column's far side, hugging goals, gap centers and deep diagonals. Per case:
arrival, failure reason, time, path length, efficiency (path / straight),
min body clearance (OBB-exact), wall-follow laps. Emits /tmp/probe_<tag>.json
for before/after comparison. Chain mode ("chain" argv[2]) runs consecutive
orders on ONE RnsSource to exercise hot-memory behavior (S4A.4).

Not a pytest asset on purpose: sweeps take ~10 min and their numbers are
tuning telemetry, not assertions -- the pinned scenes live in tests/sil/.
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

# the user's field map, 2026-09-10 (grabbed live from the SIL scene)
CARS = [(-12.60, 2.37), (-12.73, 0.67), (-12.70, -0.87), (-12.70, -2.70),
        (-12.83, -4.47), (-12.90, -6.10), (-13.00, -8.67), (-13.20, -10.63)]
WALL = (-7.77, 2.93, -8.30, -11.00, 0.50)

STARTS = {
    "sw": (-16.5, -9.8, 0.0),
    "east": (-4.0, -6.0, math.pi),
    "south": (-10.5, -14.0, 1.57),
    "north": (-10.0, 6.5, -1.57),
    "corr": (-10.5, -2.0, -1.57),
}
GOALS = {
    "corridor": (-10.07, -7.17),
    "corr_n": (-10.5, 0.0),
    "east_mid": (-5.0, -5.0),
    "wall_back": (-9.4, -5.0),
    "car_back": (-15.0, -5.0),
    "gap_s": (-10.5, -12.3),
    "north_top": (-10.0, 5.0),
    "east_top": (-5.0, 1.5),
    "hug_wall_e": (-7.0, -5.5),      # 0.8 m east of the wall face
    "hug_car_w": (-14.1, -3.5),      # 1.2 m west of the car column
    "gap_mid": (-10.5, -7.4),        # corridor beside the car gap
    "deep_sw": (-17.0, -13.0),
    "deep_ne": (-4.0, 5.5),
}
CASES = [
    ("sw", "corridor"), ("sw", "wall_back"), ("sw", "east_mid"),
    ("east", "corridor"), ("east", "car_back"), ("east", "gap_s"),
    ("south", "corr_n"), ("south", "car_back"),
    ("north", "corridor"), ("north", "gap_s"),
    ("corr", "east_mid"), ("corr", "north_top"), ("corr", "car_back"),
    ("east", "north_top"), ("sw", "east_top"),
    ("east", "hug_wall_e"), ("sw", "hug_wall_e"), ("north", "hug_wall_e"),
    ("east", "hug_car_w"), ("corr", "hug_car_w"),
    ("south", "gap_mid"), ("north", "gap_mid"),
    ("east", "deep_sw"), ("north", "deep_sw"),
    ("sw", "deep_ne"), ("south", "deep_ne"),
]
CHAINS = [
    ("east", ["corridor", "east_mid", "car_back", "gap_s"]),
    ("sw", ["east_mid", "corridor", "north_top"]),
    ("south", ["corr_n", "hug_wall_e", "deep_sw"]),
]

R_BODY = 0.46


class Ctx:
    def __init__(self, world, snap, now_ms):
        self.pose_xy = (world.rx, world.ry)
        self.yaw_rad = world.ryaw
        self.v_nom_mps = 1.0
        self.wz_max_rps = 1.2
        self.perception = snap
        self.now_mono_ms = now_ms
        self.holonomic = True


def clearance(world):
    best = math.inf
    for o in world.obstacles.values():
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


def _mk_world():
    world = SilWorld()
    for x, y in CARS:
        world.add_obstacle("car", x, y)
    world.add_obstacle("wall", WALL[0], WALL[1], WALL[2], WALL[3], WALL[4])
    return world


def _mk_mission(gx, gy):
    r = CFG["rns"]["route"]
    return Mission(MissionKind.GOTO, Origin.RELMOVE, [(gx, gy)],
                   search_window=r["search_window"],
                   arrival_radius_m=r["arrival_radius_m"],
                   max_deviation_m=r["max_deviation_m"])


def _drive(world, rns, gx, gy, t0, max_ticks=4800):
    """One order on an existing world+rns; returns the result row."""
    rns.load_mission(_mk_mission(gx, gy))
    path_len = 0.0
    min_clear = math.inf
    px, py = world.rx, world.ry
    fail = None
    arrived_tick = None
    for t in range(t0, t0 + max_ticks):
        now = t * 50
        snap = world.synth_snapshot(now)
        cand = rns.compute(Ctx(world, snap, now))
        if cand is not None:
            world.step_robot(cand.vx.value, cand.vy.value, cand.wz, DT)
        path_len += math.hypot(world.rx - px, world.ry - py)
        px, py = world.rx, world.ry
        min_clear = min(min_clear, clearance(world))
        if rns.take_arrival():
            arrived_tick = t - t0
            break
        f = rns.take_failure()
        if f is not None:
            fail = f.reason.value
            break
    return arrived_tick, fail, path_len, min_clear, t


def run_case(sname, gname):
    sx, sy, syaw = STARTS[sname]
    gx, gy = GOALS[gname]
    world = _mk_world()
    world.rx, world.ry, world.ryaw = sx, sy, syaw
    rns = RnsSource(cfg=CFG, r_eff_m=0.5)
    arrived, fail, path_len, min_clear, _ = _drive(world, rns, gx, gy, 0)
    straight = math.hypot(gx - sx, gy - sy)
    laps = [round(r.detail.get("followed_m", 0.0), 1)
            for r in rns.audit.drain() if r.kind == "wall_exit"]
    return {
        "case": "%s->%s" % (sname, gname),
        "ok": arrived is not None,
        "fail": fail,
        "time_s": round((arrived if arrived is not None else 4800) * DT, 1),
        "path_m": round(path_len, 1),
        "straight_m": round(straight, 1),
        "eff": round(path_len / straight, 2) if straight > 0.1 else 0.0,
        "min_clear": round(min_clear, 3),
        "wall_laps": laps,
        "end": (round(world.rx, 1), round(world.ry, 1)),
    }


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else "run"
    chain_mode = len(sys.argv) > 2 and sys.argv[2] == "chain"
    out = []
    if chain_mode:
        n_ok = n_all = 0
        for sname, goals in CHAINS:
            sx, sy, syaw = STARTS[sname]
            world = _mk_world()
            world.rx, world.ry, world.ryaw = sx, sy, syaw
            rns = RnsSource(cfg=CFG, r_eff_m=0.5)
            t0 = 0
            for gname in goals:
                gx, gy = GOALS[gname]
                arrived, fail, plen, mclear, t0 = _drive(
                    world, rns, gx, gy, t0 + 1)
                ok = arrived is not None
                n_all += 1
                n_ok += 1 if ok else 0
                row = {"case": "chain[%s]->%s" % (sname, gname), "ok": ok,
                       "fail": fail, "time_s": round((arrived or 0) * DT, 1),
                       "path_m": round(plen, 1), "min_clear": round(mclear, 3)}
                out.append(row)
                print("%-24s %-4s t=%6.1fs path=%6.1fm clear=%6.3f %s"
                      % (row["case"], "OK" if ok else "FAIL", row["time_s"],
                         row["path_m"], row["min_clear"], fail or ""),
                      flush=True)
        print("== %d/%d arrived (chain mode)" % (n_ok, n_all))
    else:
        for sname, gname in CASES:
            r = run_case(sname, gname)
            out.append(r)
            flag = "OK " if r["ok"] else ("TIMEOUT" if r["fail"] is None
                                          else "FAIL")
            print("%-18s %-7s t=%6.1fs path=%6.1fm eff=%5s clear=%6.3f %s %s"
                  % (r["case"], flag, r["time_s"], r["path_m"], r["eff"],
                     r["min_clear"], r["fail"] or "", r["wall_laps"]),
                  flush=True)
        n_ok = sum(1 for r in out if r["ok"])
        print("== %d/%d arrived; report /tmp/probe_%s.json" % (n_ok, len(out),
                                                               tag))
    with open("/tmp/probe_%s.json" % tag, "w") as f:
        json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
