"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: field_probe3.py
Brief: Seeded random goto sweep on the dense field map (95-percent statistic)

Description:
Companion to field_probe2 (hand-picked 16 cases): the user's acceptance bar
is a 95-percent ARRIVAL RATE over "arbitrary points", which a dozen hand-
picked pairs cannot establish. This sweep draws N start/goal pairs from a
FIXED seed over the map's bounding box, keeping only pairs whose endpoints
have >= 1.0 m body clearance and >= 8 m straight-line separation, then runs
each as a cold goto. Determinism: same seed -> same pairs -> same result
(the planner budget is a node quota, never wall-clock). Reports arrivals /
N, failure reasons, and min clearance; a pair is a PASS only if it arrives
with positive clearance. Usage: field_probe3.py <tag> [N] [seed].
"""

import json
import math
import random
import sys

sys.path.insert(0, "/opt/xbrain_v6")
sys.path.insert(0, "/opt/xbrain_v6/scripts/sil")

from field_probe2 import CFG, Ctx, DT, MAP, build_world, clearance, mk_mission
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind


def sample_pairs(n, seed):
    rng = random.Random(seed)
    world = build_world()
    # per-POINT filter: the capture's fly-away persons have one sane axis
    # each (x=-84.7,y=-1123 / x=1098,y=80.8); axis-wise filtering let those
    # lone components stretch the box to y=80 / x=-86 -- open desert far
    # off the map (first sweep: 8 pairs 60-80 m apart on empty ground).
    on_map = [o for o in MAP["obstacles"]
              if abs(o["x"]) < 100 and abs(o["y"]) < 100]
    xs = [o["x"] for o in on_map]
    ys = [o["y"] for o in on_map]
    lo_x, hi_x = min(xs) - 2.0, max(xs) + 2.0
    lo_y, hi_y = min(ys) - 2.0, max(ys) + 2.0

    def clean(x, y):
        world.rx, world.ry = x, y
        return clearance(world) >= 1.0

    pairs = []
    tries = 0
    while len(pairs) < n and tries < n * 200:
        tries += 1
        sx, sy = rng.uniform(lo_x, hi_x), rng.uniform(lo_y, hi_y)
        gx, gy = rng.uniform(lo_x, hi_x), rng.uniform(lo_y, hi_y)
        if not (clean(sx, sy) and clean(gx, gy)):
            continue
        if math.hypot(gx - sx, gy - sy) < 8.0:
            continue
        pairs.append(((round(sx, 1), round(sy, 1)), (round(gx, 1), round(gy, 1))))
    return pairs


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else "rand"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    seed = int(sys.argv[3]) if len(sys.argv) > 3 else 20260911
    pairs = sample_pairs(n, seed)
    rows = []
    for i, ((sx, sy), (gx, gy)) in enumerate(pairs):
        world = build_world()
        world.rx, world.ry = sx, sy
        world.ryaw = math.atan2(gy - sy, gx - sx)
        rns = RnsSource(cfg=CFG, r_eff_m=0.5)
        rns.load_mission(mk_mission([(gx, gy)], MissionKind.GOTO))
        arrived = None
        fail = None
        mc = math.inf
        for t in range(6000):
            now = t * 50
            snap = world.synth_snapshot(now)
            cand = rns.compute(Ctx(world, snap, now))
            if cand is not None:
                world.step_robot(cand.vx.value, cand.vy.value, cand.wz, DT)
            world.step_obstacles(DT)
            mc = min(mc, clearance(world))
            if rns.take_arrival():
                arrived = t
                break
            f = rns.take_failure()
            if f is not None:
                fail = f.reason.value
                break
        ok = arrived is not None and mc > 0.0
        rows.append({"i": i, "start": (sx, sy), "goal": (gx, gy), "ok": ok,
                     "fail": fail, "t": arrived, "clear": round(mc, 3)})
        print("r%02d (%6.1f,%6.1f)->(%6.1f,%6.1f) %-4s t=%-7s clear=%6.3f %s"
              % (i, sx, sy, gx, gy, "OK" if ok else "FAIL",
                 ("%.1fs" % (arrived * DT)) if arrived is not None else "-",
                 mc, fail or ("TIMEOUT" if arrived is None else "")),
              flush=True)
    n_ok = sum(1 for r in rows if r["ok"])
    print("== %d/%d arrived (%.1f%%), seed %d" % (n_ok, len(rows),
                                                 100.0 * n_ok / max(1, len(rows)),
                                                 seed))
    with open("/tmp/probe3_%s.json" % tag, "w") as f:
        json.dump(rows, f, indent=1)


if __name__ == "__main__":
    main()
