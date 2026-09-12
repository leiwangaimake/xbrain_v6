#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: pose_stub.py
Brief: headless Zenoh SIL world -- stands in for quadruped + rtk_driver + perception for a real p1, exit code = verdict

Description:
P7.2 end-to-end without a browser: the REAL p1_motion (RNS in its arbiter,
20 Hz loop, cmd_vel on the RT plane) drives a simulated robot in the frozen
field map (scripts/sil/field_map2.json). All Zenoh traffic goes through
scripts/sil/zenoh_world.ZenohWorldLink -- the same link the browser SIL uses
in --mode e2e -- so the wire shapes exist once. This script adds only the
headless host: the tick, the route / goto kick-off, the trace line and the
verdict (0 arrived / 2 failed / 3 timeout) that scripts/dev/run_p72_e2e.sh
turns into its exit code.

Options:
  --grant   publish cmd/motion/factor at 1 Hz (p2 Stage-D stand-in; without
            it p1's HealthFactorSlot reports 'never' and correctly refuses)
  --route   fwd | rev: the map's patrol polyline as cmd/motion/route after
            --route-delay-s; --goto X,Y: a single-point route instead
  --relmove DX,DY: a cmd/motion/relative_move (body frame) instead of a route,
            to exercise P1-5 / P1-6 end to end

What it does NOT do: no RNS / p1 logic, no chassis APDU (chassis_stub.py is
the CHS-A side), no UI (sil_server.py --mode e2e).

Run (routers up, p1 running with XBRAIN_ROBOT_ID=dev):
  python3 -u scripts/dev/pose_stub.py --grant --route rev
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts" / "sil"))

from sil_world import SilWorld  # noqa: E402
from zenoh_world import ZenohWorldLink, load_map, read_resolved_p1  # noqa: E402
from xbrain.p1_motion.path.local_frame import LocalFrame  # noqa: E402

TICK_HZ = 20.0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="headless Zenoh SIL world for the p1 E2E")
    ap.add_argument("--rid", default="dev")
    ap.add_argument("--map", default=str(ROOT / "scripts" / "sil" / "field_map2.json"))
    ap.add_argument("--resolved", default=str(ROOT / "data" / "run" / "resolved"))
    ap.add_argument("--grant", action="store_true", help="publish cmd/motion/factor at 1 Hz")
    ap.add_argument("--route", choices=("none", "fwd", "rev"), default="none")
    ap.add_argument("--goto", default=None, help="X,Y single-point route (site metres)")
    ap.add_argument("--relmove", default=None, help="DX,DY body-frame relative_move")
    ap.add_argument("--route-delay-s", type=float, default=3.0)
    ap.add_argument("--arrive-radius-m", type=float, default=0.8)
    ap.add_argument("--max-s", type=float, default=300.0)
    ap.add_argument("--print-every-s", type=float, default=1.0)
    args = ap.parse_args(argv)

    p1 = read_resolved_p1(Path(args.resolved))
    frame = LocalFrame(p1["lat"], p1["lon"])
    world = SilWorld()
    load_map(world, Path(args.map), with_robot=True)
    link = ZenohWorldLink(args.rid, frame, grant=args.grant, relmove_max_m=p1["relmove_max_m"])

    dt = 1.0 / TICK_HZ
    t_start = time.monotonic()
    next_t = t_start
    last_print = -1e9
    sent = False
    route_pts: Optional[List[Tuple[float, float]]] = None
    relmove: Optional[Tuple[float, float]] = None
    if args.relmove:
        relmove = tuple(float(v) for v in args.relmove.split(","))  # type: ignore[assignment]
    elif args.goto:
        gx, gy = (float(v) for v in args.goto.split(","))
        route_pts = [(gx, gy)]
    elif args.route != "none":
        route_pts = list(world.path) if args.route == "fwd" else list(reversed(world.path))
    verdict = 3
    try:
        while True:
            now = time.monotonic()
            now_ms = int(now * 1000.0)
            vx, vy, wz, _fresh = link.latest_cmd(now)
            world.step_obstacles(dt)
            world.step_robot(vx, vy, wz, dt)
            snap = world.synth_snapshot(now_ms)
            link.publish_world(world.rx, world.ry, world.ryaw, vx, vy, snap, now)
            if not sent and now - t_start >= args.route_delay_s:
                sent = True
                if relmove is not None:
                    cid = link.send_relmove(relmove[0], relmove[1])
                    print("[%6.1fs] relative_move sent: %s dx=%.2f dy=%.2f"
                          % (now - t_start, cid, relmove[0], relmove[1]))
                elif route_pts is not None:
                    link.send_route(route_pts, arrive_radius_m=args.arrive_radius_m)
                    print("[%6.1fs] route sent: %d points, end (%.1f, %.1f)"
                          % (now - t_start, len(route_pts), route_pts[-1][0], route_pts[-1][1]))
                else:
                    sent = False
            for ch, st, why in link.drain_changes():
                print("[%6.1fs] %s -> %s%s" % (now - t_start, ch, st, (" (%s)" % why) if why else ""))
                if not sent:
                    continue
                if ch == "progress" and route_pts is not None:
                    verdict = 0 if st == "arrived" else 2 if st == "failed" else verdict
                if ch == "relmove" and relmove is not None:
                    verdict = (0 if st == "succeeded"
                               else 2 if st in ("aborted", "rejected") else verdict)
            if now - last_print >= args.print_every_s:
                last_print = now
                info = link.cmd_info()
                prog, _rm = link.snapshot()
                p = prog or {}
                dist = (math.hypot(route_pts[-1][0] - world.rx, route_pts[-1][1] - world.ry)
                        if route_pts else float("nan"))
                print("[%6.1fs] pose (%.2f, %.2f, %.2f) cmd (%.2f, %.2f, %.2f) n=%d %s/%s "
                      "progress=%s wp=%s/%s dist_end=%.2f"
                      % (now - t_start, world.rx, world.ry, world.ryaw, vx, vy, wz, info["n"],
                         info["source"], info["limiter"], p.get("state"),
                         p.get("waypoint_index"), p.get("waypoint_total"), dist))
            if verdict in (0, 2) or now - t_start >= args.max_s:
                break
            next_t += dt
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    print("verdict: %s (cmd_vel frames=%d)" % (
        {0: "ARRIVED", 2: "FAILED", 3: "TIMEOUT"}[verdict], link.cmd_info()["n"]))
    return verdict


if __name__ == "__main__":
    sys.exit(main())
