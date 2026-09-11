#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: hot_battery.py
Brief: Hot bidirectional path battery on the dense field map (20 S4A.10/S4A.11)

Description:
The third leg of the RNS acceptance harness (user ruling 2026-09-11): ONE
RnsSource, ONE world, the recorded 37-point patrol path driven forward and
reverse alternately N times without rebuilding either. "Hot" because the
memory grid and the guidance layer carry over between missions -- exactly the
in-shift condition (20 S4A.4 task-level lifecycle: the grid outlives one
mission, the planning domain does not). field_probe2.py scores COLD runs
(fresh source per case); this catches the failures only carried-over memory
can cause (stale BLOCKED, eroded walls, a domain that did not clear).

The tick clock is CONTINUOUS across legs (drive(t0_ms=...)): RNS keeps
absolute-time state (verdict mute window, acceptance epoch, 11 S3.1B.5), so a
per-leg restart at 0 would fake an epoch reset and mute verdicts for a whole
leg -- the first hot run of 2026-09-11 scored 3/8 on exactly that artefact.

Scoring per leg: arrived (within the tick budget) AND off-line (> 2.5 m from
the polyline) for < 15 percent of ticks AND min clearance > 0 (no contact).
Verdict line at the end: legs passed / legs run. Reuses field_probe2's world
builder, mission factory, drive loop and geometry helpers so the three legs of
the harness cannot drift apart.

Usage (on the Jetson, absolute paths, ~4 min per 8 legs):
  python3 /opt/xbrain_v6/scripts/sil/hot_battery.py <tag> [legs=8] [diag_leg]
diag_leg: dump that leg's nav-state histogram, last audit records, final
pose / goal distance and stop the battery there (a stuck leg diagnosis).
"""
import math
import sys

sys.path.insert(0, "/opt/xbrain_v6")
sys.path.insert(0, "/opt/xbrain_v6/scripts/sil")

import field_probe2 as fp  # noqa: E402  (its main() only runs under __main__)
from xbrain.p1_motion.rns.source import RnsSource  # noqa: E402
from xbrain.p1_motion.rns.types import MissionKind  # noqa: E402


def main(argv) -> int:
    tag = argv[1] if len(argv) > 1 else "hot"
    legs = int(argv[2]) if len(argv) > 2 else 8
    diag_leg = int(argv[3]) if len(argv) > 3 else None
    fwd = [tuple(p) for p in fp.MAP["path"]]
    rev = list(reversed(fwd))
    world = fp.build_world()
    world.rx, world.ry = fwd[0]
    world.ryaw = math.atan2(fwd[1][1] - fwd[0][1], fwd[1][0] - fwd[0][0])
    rns = RnsSource(cfg=fp.CFG, r_eff_m=0.5)      # ONE source for every leg
    n_ok = 0
    clock_ms = 0                                   # monotonic across legs (see drive)
    for k in range(legs):
        pts = fwd if k % 2 == 0 else rev
        name = "leg%02d_%s" % (k, "fwd" if k % 2 == 0 else "rev")
        off = [0]
        total = [0]

        hist = {}
        trail = []

        def w_line(t, wd, rn, pts=pts, off=off, total=total, hist=hist, trail=trail):
            total[0] += 1
            if fp.dist_to_polyline(wd.rx, wd.ry, pts) > 2.5:
                off[0] += 1
            if diag_leg is not None and k == diag_leg:
                st = rn.nav_state().value
                hist[st] = hist.get(st, 0) + 1
                if t % 200 == 0:          # every 10 s of sim time
                    trail.append((t, round(wd.rx, 1), round(wd.ry, 1), st))

        # the robot starts each leg wherever the previous one ended (hot):
        # no teleport, no re-heading -- the line's first point is behind it.
        arrived, fail, mc = fp.drive(world, rns, fp.mk_mission(pts, MissionKind.PATH),
                                     12000, w_line, t0_ms=clock_ms)
        # advance the shared clock by the ticks actually run (arrival tick or
        # the full budget): the next leg continues the same monotonic timeline
        clock_ms += 50 * ((arrived + 1) if arrived is not None else 12000)
        off_pct = 100.0 * off[0] / max(1, total[0])
        ok = arrived is not None and off_pct < 15.0 and mc > 0.0
        n_ok += 1 if ok else 0
        print("%-12s %-4s t=%-6s clear=%6.3f %s offline %.1f%%"
              % (name, "OK" if ok else "FAIL",
                 ("%.1fs" % (arrived * fp.DT)) if arrived is not None else "-",
                 mc, fail or "", off_pct), flush=True)
        if diag_leg is not None and k == diag_leg:
            gx, gy = pts[-1]
            print("  diag: state histogram %s" % hist, flush=True)
            tr = getattr(getattr(rns, "_mission", None), "tracker", None)
            tr_ints = ({k: v for k, v in vars(tr).items() if isinstance(v, int)}
                       if tr is not None else None)
            print("  diag: final pose (%.2f, %.2f) yaw %.2f, dist to goal %.2f, "
                  "state %s, tracker %s" % (world.rx, world.ry, world.ryaw,
                                            math.hypot(world.rx - gx, world.ry - gy),
                                            rns.nav_state().value, tr_ints), flush=True)
            print("  diag: trail (every 10 s) %s" % trail[-40:], flush=True)
            for rec in rns.audit.drain()[-40:]:
                print("  audit %7d %-24s %s" % (rec.t_mono_ms, rec.kind, rec.detail),
                      flush=True)
            return 1
    print("== %s: %d/%d legs passed" % (tag, n_ok, legs), flush=True)
    return 0 if n_ok == legs else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
