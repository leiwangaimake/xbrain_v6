#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: pose_stub.py
Brief: Zenoh edition of the SIL world -- stands in for quadruped + rtk_driver + perception on the dev stack

Description:
P7.2 end-to-end: the REAL p1_motion (RNS in its arbiter, 20 Hz loop, cmd_vel
on the RT plane) drives a simulated robot in the frozen field map
(scripts/sil/field_map2.json, the same world the browser SIL uses). This
script is the three simulated peripherals in one process, wired through the
production keys only, so switching to real hardware is "stop this, start the
real units" with zero p1 change:
  * quadruped stand-in : subscribes xbrain/{rid}/rt/motion/cmd_vel, integrates
                         the SIL kinematics (body vx/vy/wz -> world pose); a
                         cmd_vel older than 200 ms is treated as zero (Tier 1
                         cmd_age timeout, 11 S9.12) -- p1 must keep streaming
  * rtk_driver stand-in: publishes rt/gnss/fix (11 S3.2) + rt/gnss/heading
                         (11 S3.3, ENU yaw, dual_antenna L1) + rt/clock/status
                         (11 S3.11) from the integrated pose, projected to
                         WGS84 about the SAME common.geo.enu_origin p1 reads
                         (taken from the resolved p1_motion.yaml snapshot)
  * perception stand-in: raycasts the world (sil_world.synth_snapshot, the
                         DTOs RNS consumes) and publishes the three 11 S3.1B
                         keys with the schema strings three_keys.py requires
Optional dev stand-ins for what the dev stack does not yet emit:
  --grant  publishes cmd/motion/factor {allow_motion true, speed_factor 1.0,
           max_profile patrol} at 1 Hz -- p2_core's Stage-D grant (14) is not
           built in the voice-loop wiring; without it p1's HealthFactorSlot
           reports 'never' and the loop (correctly) refuses to move
  --route  publishes the map's patrol polyline as cmd/motion/route (11 S3.5A,
           WGS84 points, one chunk) after --route-delay-s; 'rev' reverses it;
           --goto X,Y publishes a single-point route instead (RNS-N-1 goto)
It listens to state/motion/path_progress and relative_move/status and prints
every state change; with a route sent it exits 0 on arrived, 2 on failed, 3
on --max-s timeout -- the E2E verdict scripts/dev/run_p72_e2e.sh relies on.

What it does NOT do: no RNS logic, no p1 logic (nothing is imported from
p1 except the DTO types and the LocalFrame projection), no chassis APDU
(chassis_stub.py is the CHS-A side; this is the Zenoh side).

Run (dev stack routers up, p1 running with XBRAIN_ROBOT_ID=dev):
  python3 scripts/dev/pose_stub.py --grant --route fwd
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts" / "sil"))

from sil_world import SilWorld  # noqa: E402  (scripts/sil, the browser SIL world)
from xbrain.p1_motion.path.local_frame import LocalFrame  # noqa: E402

RT_ENDPOINT = "tcp/127.0.0.1:7449"
GEN_ENDPOINT = "tcp/127.0.0.1:7447"
CMD_AGE_MAX_S = 0.2            # 11 S9.12 Tier 1: cmd_age > 200 ms -> stop
TICK_HZ = 20.0
PROFILE_SCHEMA = "perception_profile_v1"
OBJECTS_SCHEMA = "perception_objects_v1"
STATUS_SCHEMA = "perception_status_v1"
FRAME = "base_link"


def _session(endpoint: str):
    import zenoh
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["%s"]' % endpoint)
    return zenoh.open(conf)


def _unwrap(doc: Any) -> Any:
    if isinstance(doc, dict) and "data" in doc and "v" in doc and "src" in doc:
        return doc["data"]
    return doc


class Wire:
    """11 S3.0 envelope stamper: one seq per key, src fixed per role."""

    def __init__(self, rid: str) -> None:
        self.rid = rid
        self.seq: Dict[str, int] = {}
        try:
            from xbrain.common.envelope import read_local_boot_id
            self.boot = read_local_boot_id()
        except Exception:      # noqa: BLE001 -- non-Linux dev box
            self.boot = None

    def env(self, key: str, src: str, data: Any) -> bytes:
        n = self.seq.get(key, 0)
        self.seq[key] = n + 1
        e: Dict[str, Any] = {"v": 1, "rid": self.rid, "ts": time.time(),   # WALL-CLOCK-OK(align/log)
                             "mono": time.monotonic(), "seq": n, "src": src,
                             "ts_sync": True, "data": data}
        if self.boot:
            e["boot"] = self.boot
        return json.dumps(e, ensure_ascii=True).encode("utf-8")


def load_map(world: SilWorld, path: Path) -> Dict[str, Any]:
    """Same seeding as sil_server.load_field_map: fly-aways dropped, walls
    with thickness, car headings, the patrol polyline, the robot pose."""
    m = json.loads(path.read_text(encoding="utf-8"))
    for o in m.get("obstacles", []):
        if abs(o["x"]) > 100 or abs(o["y"]) > 100:
            continue
        if o["kind"] == "wall":
            world.add_obstacle("wall", o["x"], o["y"], o["x2"], o["y2"],
                               o.get("thick_m", o.get("thick", 0.5)))
        else:
            oid = world.add_obstacle(o["kind"], o["x"], o["y"])
            if o["kind"] == "car":
                world.obstacles[oid].heading = o.get("heading", 0.0)
    if m.get("path"):
        world.path = [(p[0], p[1]) for p in m["path"]]
    r = m.get("robot") or {}
    world.rx = float(r.get("x", 0.0))
    world.ry = float(r.get("y", 0.0))
    world.ryaw = float(r.get("yaw", 0.0))
    return m


def enu_origin_from_resolved(resolved: Path) -> Tuple[float, float]:
    """The one origin p1 uses (geo.enu_origin of its resolved snapshot)."""
    d = yaml.safe_load((resolved / "p1_motion.yaml").read_text(encoding="utf-8"))
    o = d["geo"]["enu_origin"]
    return float(o["lat"]), float(o["lon"])


def route_body(frame: LocalFrame, pts: List[Tuple[float, float]], *, cmd_id: str,
               route_rev: int, arrive_radius_m: float) -> Dict[str, Any]:
    """11 S3.5A RouteGeometry, one chunk, WGS84 points."""
    points = []
    for i, (x, y) in enumerate(pts):
        lat, lon = frame.to_latlon(x, y)
        points.append({"lat": lat, "lon": lon, "seq": i, "arrive_radius_m": arrive_radius_m})
    total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                for i in range(len(pts) - 1))
    return {"v": 1, "op": "set", "cmd_id": cmd_id, "route_id": "r-field", "route_rev": route_rev,
            "loop_mode": "oneway", "total_len_m": round(total, 2), "frame": "wgs84",
            "points": points, "chunk": {"index": 0, "total": 1}}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Zenoh SIL world for the p1 E2E")
    ap.add_argument("--rid", default="dev")
    ap.add_argument("--map", default=str(ROOT / "scripts" / "sil" / "field_map2.json"))
    ap.add_argument("--resolved", default=str(ROOT / "data" / "run" / "resolved"))
    ap.add_argument("--grant", action="store_true", help="publish cmd/motion/factor at 1 Hz")
    ap.add_argument("--route", choices=("none", "fwd", "rev"), default="none")
    ap.add_argument("--goto", default=None, help="X,Y single-point route (site metres)")
    ap.add_argument("--route-delay-s", type=float, default=3.0)
    ap.add_argument("--arrive-radius-m", type=float, default=0.8)
    ap.add_argument("--max-s", type=float, default=300.0)
    ap.add_argument("--print-every-s", type=float, default=1.0)
    args = ap.parse_args(argv)

    lat0, lon0 = enu_origin_from_resolved(Path(args.resolved))
    frame = LocalFrame(lat0, lon0)
    world = SilWorld()
    load_map(world, Path(args.map))
    rid = args.rid
    wire = Wire(rid)
    rt = _session(RT_ENDPOINT)
    gen = _session(GEN_ENDPOINT)
    pubs = {
        "fix": rt.declare_publisher("xbrain/%s/rt/gnss/fix" % rid),
        "heading": rt.declare_publisher("xbrain/%s/rt/gnss/heading" % rid),
        "clock": rt.declare_publisher("xbrain/%s/rt/clock/status" % rid),
        "profile": rt.declare_publisher("xbrain/%s/rt/perception/profile" % rid),
        "objects": rt.declare_publisher("xbrain/%s/rt/perception/objects" % rid),
        "status": rt.declare_publisher("xbrain/%s/rt/perception/status" % rid),
        "factor": gen.declare_publisher("cmd/motion/factor"),
        "route": gen.declare_publisher("cmd/motion/route"),
    }
    cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "rx": -1e9, "n": 0, "limiter": None, "source": None}
    fb = {"progress": None, "relmove": None, "changes": []}

    def on_cmd(sample) -> None:
        try:
            d = _unwrap(json.loads(bytes(sample.payload).decode("utf-8")))
            cmd["vx"], cmd["vy"], cmd["wz"] = float(d["vx"]), float(d.get("vy", 0.0)), float(d["wz"])
            g = d.get("gate") or {}
            cmd["limiter"], cmd["source"] = g.get("limiter"), g.get("source")
            cmd["rx"] = time.monotonic()
            cmd["n"] += 1
        except Exception as exc:      # noqa: BLE001
            print("bad cmd_vel: %s" % exc, file=sys.stderr)

    def on_progress(sample) -> None:
        d = _unwrap(json.loads(bytes(sample.payload).decode("utf-8")))
        prev = fb["progress"]
        fb["progress"] = d
        if prev is None or prev.get("state") != d.get("state"):
            fb["changes"].append(("progress", d.get("state"), d.get("fail_reason")))

    def on_relmove(sample) -> None:
        d = _unwrap(json.loads(bytes(sample.payload).decode("utf-8")))
        fb["relmove"] = d
        fb["changes"].append(("relmove", d.get("state"), d.get("abort_reason")))

    subs = [rt.declare_subscriber("xbrain/%s/rt/motion/cmd_vel" % rid, on_cmd),
            gen.declare_subscriber("state/motion/path_progress", on_progress),
            gen.declare_subscriber("cmd/motion/relative_move/status", on_relmove)]

    dt = 1.0 / TICK_HZ
    t_start = time.monotonic()
    next_t = t_start
    last_1hz = -1e9
    last_print = -1e9
    route_sent = False
    route_pts: Optional[List[Tuple[float, float]]] = None
    if args.goto:
        gx, gy = (float(v) for v in args.goto.split(","))
        route_pts = [(gx, gy)]
    elif args.route != "none":
        route_pts = list(world.path) if args.route == "fwd" else list(reversed(world.path))
    verdict = 3
    try:
        while True:
            now = time.monotonic()
            now_ms = int(now * 1000.0)
            # --- chassis: apply the latest cmd_vel, zero when stale ---------
            if now - cmd["rx"] <= CMD_AGE_MAX_S:
                vx, vy, wz = cmd["vx"], cmd["vy"], cmd["wz"]
            else:
                vx = vy = wz = 0.0
            world.step_obstacles(dt)
            world.step_robot(vx, vy, wz, dt)
            # --- rtk_driver ---------------------------------------------------
            lat, lon = frame.to_latlon(world.rx, world.ry)
            speed = math.hypot(vx, vy)
            pubs["fix"].put(wire.env("fix", "rtk_driver", {
                "lat": lat, "lon": lon, "alt": 4.0, "fix_type": "rtk_fixed", "hdop": 0.6,
                "sats": 24, "cov_h_m": 0.02, "cov_v_m": 0.03, "age_s": 0.05, "t_mono": now}))
            pubs["heading"].put(wire.env("heading", "rtk_driver", {
                "heading_rad": world.ryaw, "heading_true_north_rad": (math.pi / 2 - world.ryaw) % (2 * math.pi),
                "heading_valid": True, "source": "dual_antenna", "level": 1, "cov_rad": 0.005,
                "speed_mps": speed, "cog_rad": world.ryaw, "baseline_m": 0.62,
                "baseline_valid": True, "yaw_capable": True, "i_heading": 1.0,
                "age_s": 0.05, "t_mono": now, "since_mono": t_start}))
            # --- perception ---------------------------------------------------
            snap = world.synth_snapshot(now_ms)
            prof = dataclasses.asdict(snap.profile)
            prof.update({"schema": PROFILE_SCHEMA, "frame": FRAME})
            objs = dataclasses.asdict(snap.objects)
            objs.update({"schema": OBJECTS_SCHEMA, "frame": FRAME})
            stat = dataclasses.asdict(snap.status)
            stat.update({"schema": STATUS_SCHEMA})
            pubs["profile"].put(wire.env("profile", "perception", prof))
            pubs["objects"].put(wire.env("objects", "perception", objs))
            pubs["status"].put(wire.env("status", "perception", stat))
            # --- 1 Hz: clock, grant -------------------------------------------
            if now - last_1hz >= 1.0:
                last_1hz = now
                pubs["clock"].put(wire.env("clock", "rtk_driver", {
                    "sync": True, "source": "gnss", "mono_ref": now, "utc_ref": time.time()}))  # WALL-CLOCK-OK(align/log)
                if args.grant:
                    pubs["factor"].put(json.dumps({
                        "speed_factor": 1.0, "max_profile": "patrol", "allow_motion": True,
                        "reason": "none", "detail_ref": "health/summary"}).encode("utf-8"))
            # --- route once -----------------------------------------------------
            if route_pts is not None and not route_sent and now - t_start >= args.route_delay_s:
                pubs["route"].put(json.dumps(route_body(
                    frame, route_pts, cmd_id="rg-e2e", route_rev=1,
                    arrive_radius_m=args.arrive_radius_m)).encode("utf-8"))
                route_sent = True
                print("[%6.1fs] route sent: %d points, end (%.1f, %.1f)"
                      % (now - t_start, len(route_pts), route_pts[-1][0], route_pts[-1][1]))
            # --- trace + verdict -------------------------------------------------
            while fb["changes"]:
                ch, st, why = fb["changes"].pop(0)
                print("[%6.1fs] %s -> %s%s" % (now - t_start, ch, st, (" (%s)" % why) if why else ""))
                if ch == "progress" and route_sent:
                    if st == "arrived":
                        verdict = 0
                    elif st == "failed":
                        verdict = 2
            if now - last_print >= args.print_every_s:
                last_print = now
                p = fb["progress"] or {}
                dist = (math.hypot(route_pts[-1][0] - world.rx, route_pts[-1][1] - world.ry)
                        if route_pts else float("nan"))
                print("[%6.1fs] pose (%.2f, %.2f, %.2f) cmd (%.2f, %.2f, %.2f) n=%d %s/%s "
                      "progress=%s wp=%s/%s dist_end=%.2f"
                      % (now - t_start, world.rx, world.ry, world.ryaw, vx, vy, wz, cmd["n"],
                         cmd["source"], cmd["limiter"], p.get("state"), p.get("waypoint_index"),
                         p.get("waypoint_total"), dist))
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
        for s in subs:
            try:
                s.undeclare()
            except Exception:      # noqa: BLE001
                pass
        rt.close()
        gen.close()
    print("verdict: %s (cmd_vel frames=%d)" % (
        {0: "ARRIVED", 2: "FAILED", 3: "TIMEOUT"}[verdict], cmd["n"]))
    return verdict


if __name__ == "__main__":
    sys.exit(main())
