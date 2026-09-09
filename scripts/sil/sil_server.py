"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: sil_server.py
Brief: RNS SIL webserver -- fastapi + 20 Hz tick + REAL RnsSource wiring

Description:
The SIL loop: browser (static/index.html) places obstacles/paths/waypoints via
REST, this server runs the 20 Hz tick -- synthesize perception (sil_world) ->
REAL RnsSource.compute -> integrate kinematics -> broadcast state on the
websocket. RNS is wired through its production interfaces only:
  - config: reads configs/rns.yaml (the ONE file, user ruling 2026-09-10);
    startup assertions run in RnsSource.__init__ (D2 + person lock).
  - missions: route.Mission via load_mission (goto = single point, B4 anchors
    the start at the current pose; path = the placed polyline).
  - tick: compute(ctx) with pose/yaw/v_nom/wz_max -- same ctx fields the P7.1
    production wiring will carry, plus the perception snapshot for the
    avoidance stages as they fold in.

"Execute nav" behavior (user spec): enter the path at the NEAREST point from
the robot, walk to the far end, done; pressing again runs it REVERSED. The
nearest-entry trim is done HERE (task-layer job, mirroring p3's U07a remap in
the real system) -- RNS itself never re-orders a route.

Run:  python3 scripts/sil/sil_server.py   (listens on 0.0.0.0:8890)
"""

from __future__ import annotations

import asyncio
import json
import math
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts" / "sil"))

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from sil_world import (BLIND_NEAR_M, FOV_HALF_RAD, RANGE_MAX_M, SilWorld)
from xbrain.p1_motion.rns.route import Mission
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.rns.types import MissionKind, Origin

TICK_HZ = 20.0
DT = 1.0 / TICK_HZ
R_EFF_M = 0.5            # M20S half-diagonal, the SIL r_eff (12 S6A consumer)


def load_v_nom() -> float:
    """The mission nominal speed, from the V6 config hierarchy -- NOT a SIL
    constant (user 2026-09-10: M20S tops out at 2 m/s; a future UGV chassis
    does 25 -- swapping chassis must be a config change, zero code):
      L2 physical ceiling: models/m20s.yaml common.spec.max_vx_mps
      L1 profile:          common.yaml common.motion.profiles.patrol.max_mps
    v_nom = patrol profile (20 S8.1A: goto/path nominal = patrol tier), safety-
    clamped by the physical ceiling (SP-2 direction). null anywhere -> refuse
    with the key path (CLAUDE.md 3.1), never a silent default."""
    model = yaml.safe_load((ROOT / "configs" / "models" / "m20s.yaml")
                           .read_text(encoding="utf-8"))
    common = yaml.safe_load((ROOT / "configs" / "common.yaml")
                            .read_text(encoding="utf-8"))
    ceiling = model["common"]["spec"]["max_vx_mps"]
    patrol = common["common"]["motion"]["profiles"]["patrol"]["max_mps"]
    if ceiling is None or patrol is None:
        raise RuntimeError(
            "v_nom unconfigured: models/m20s.yaml common.spec.max_vx_mps and "
            "common.yaml common.motion.profiles.patrol.max_mps must be set")
    global HOLONOMIC
    HOLONOMIC = bool(model["common"]["spec"]["holonomic"])
    return min(float(patrol), float(ceiling))


V_NOM_MPS = None         # resolved at startup from the config hierarchy
WZ_MAX_RPS = 1.2         # [sim] spec.max_wz_radps is null (V-01: no basis yet)
HOLONOMIC = None         # resolved at startup from models/<chassis>.yaml

app = FastAPI()
world = SilWorld()
CFG = yaml.safe_load((ROOT / "configs" / "rns.yaml").read_text(encoding="utf-8"))
V_NOM_MPS = load_v_nom()                    # 2.0 for M20S; chassis-swappable
rns = RnsSource(cfg=CFG, r_eff_m=R_EFF_M)   # startup assertions run HERE

clients: list = []
nav = {"state": "idle", "direction": 1, "target": None, "last_done": None}

# flight recorder (user 2026-09-10: a long stall could not be diagnosed after
# the fact -- the audit ring lived in-process with no REST). Ring of per-tick
# records, 10 min at 20 Hz; /api/trace pulls it, /api/audit drains RNS events.
from collections import deque
TRACE = deque(maxlen=12000)


class Ctx:
    """The tick context handed to RnsSource.compute -- the SIL stand-in for the
    P7.1 production snapshot. Field names are the production contract."""
    def __init__(self, pose, yaw, snapshot, now_ms):
        self.pose_xy = pose
        self.yaw_rad = yaw
        self.v_nom_mps = V_NOM_MPS
        self.wz_max_rps = WZ_MAX_RPS
        self.perception = snapshot
        self.now_mono_ms = now_ms
        self.holonomic = HOLONOMIC


def speed_gate_f(d_free_fwd: float) -> float:
    """The HOST's four-band speed gate f(d_free) -- 12 S6.2 verbatim bands
    ([3,inf) 2.0 / [1.8,3) 0.5 / [1.25,1.8) 0.2 / [0,1.25) 0). This is p1
    output-layer duty, NOT RNS's; the SIL server IS the sim host, so it clamps
    here exactly where the real p1 would."""
    if d_free_fwd >= 3.0:
        return 2.0
    if d_free_fwd >= 1.8:
        return 0.5
    if d_free_fwd >= 1.25:
        return 0.2
    return 0.0


def _nearest_entry_index(pts, x, y):
    """Task-layer nearest-entry (U07a mirror): index of the closest vertex."""
    return min(range(len(pts)), key=lambda i: (pts[i][0]-x)**2 + (pts[i][1]-y)**2)


def _load_path_mission(direction: int) -> bool:
    pts = list(world.path) if direction > 0 else list(reversed(world.path))
    if len(pts) < 1:
        return False
    i = _nearest_entry_index(pts, world.rx, world.ry)
    remaining = pts[i:] if i < len(pts) - 1 else [pts[-1]]
    rc = CFG["rns"]["route"]
    rns.load_mission(Mission(
        MissionKind.PATH, Origin.ROUTE, remaining,
        search_window=rc["search_window"],
        arrival_radius_m=rc["arrival_radius_m"],
        max_deviation_m=rc["max_deviation_m"]))
    return True


def _load_goto(x: float, y: float) -> None:
    rc = CFG["rns"]["route"]
    rns.load_mission(Mission(
        MissionKind.GOTO, Origin.RELMOVE, [(x, y)],
        search_window=rc["search_window"],
        arrival_radius_m=rc["arrival_radius_m"],
        max_deviation_m=rc["max_deviation_m"]))


# ── REST ─────────────────────────────────────────────────────────────────────
@app.get("/")
async def index():
    return FileResponse(
        str(ROOT / "scripts" / "sil" / "static" / "index.html"),
        headers={"Cache-Control": "no-store, must-revalidate"})


@app.post("/api/obstacle")
async def api_obstacle(body: dict):
    oid = world.add_obstacle(body["kind"], body["x"], body["y"],
                             body.get("x2", 0.0), body.get("y2", 0.0),
                             body.get("thick_m", 0.5))
    return {"oid": oid}


@app.post("/api/obstacle/{oid}/toggle_dynamic")
async def api_toggle(oid: int):
    return {"ok": world.toggle_dynamic(oid)}


@app.post("/api/obstacle/{oid}/update_wall")
async def api_update_wall(oid: int, body: dict):
    return {"ok": world.update_wall(oid, body["x"], body["y"], body["x2"],
                                    body["y2"], body["thick_m"])}


@app.delete("/api/obstacle/{oid}")
async def api_del(oid: int):
    return {"ok": world.remove_obstacle(oid)}


@app.post("/api/path")
async def api_path(body: dict):
    world.path = [(p[0], p[1]) for p in body["points"]]
    return {"n": len(world.path)}


@app.post("/api/waypoint")
async def api_wp(body: dict):
    world.waypoints.append((body["x"], body["y"]))
    return {"n": len(world.waypoints)}


@app.post("/api/nav/execute")
async def api_execute():
    # user spec: press -> nearest-entry, walk to far end; press AGAIN (after the
    # PATH finished) -> reverse. A goto arrival must not count as "again" -- that
    # was the smoke-found bug: goto arrived -> first execute went reversed.
    if nav["state"] == "arrived" and nav["last_done"] == "path":
        nav["direction"] = -nav["direction"]
    else:
        nav["direction"] = 1
    if _load_path_mission(nav["direction"]):
        nav["state"] = "running_path"
        return {"ok": True, "direction": nav["direction"]}
    return {"ok": False, "reason": "no path placed"}


@app.post("/api/nav/goto")
async def api_goto(body: dict):
    _load_goto(body["x"], body["y"])
    nav["state"] = "running_goto"
    nav["target"] = (body["x"], body["y"])
    return {"ok": True}


@app.get("/api/state")
async def api_state():
    # debug/test view of the tick state (the websocket carries the same data)
    return {"robot": {"x": world.rx, "y": world.ry, "yaw": world.ryaw},
            "nav": nav, "n_obstacles": len(world.obstacles),
            "mission_loaded": rns._mission is not None}


@app.get("/api/trace")
async def api_trace(n: int = 600, step: int = 1):
    """Flight recorder pull: last n records, every step-th. Also a per-state
    dwell summary over the WHOLE ring so a stall shows up without eyeballing."""
    from collections import Counter
    recs = list(TRACE)
    dwell = Counter(r["st"] for r in recs)
    moving = sum(1 for r in recs if abs(r["vx"]) > 0.05)
    return {"n_total": len(recs),
            "state_ticks": dict(dwell),
            "moving_ratio": round(moving / len(recs), 3) if recs else None,
            "tail": recs[-n::step]}


@app.get("/api/audit")
async def api_audit():
    """Drain the RNS audit ring (detour/wall enters, exits, failures)."""
    return {"events": [{"t_ms": r.t_mono_ms, "kind": r.kind, "detail": r.detail}
                       for r in rns.audit.drain()]}


@app.post("/api/reset")
async def api_reset():
    world.reset()
    rns.clear_mission()
    nav.update(state="idle", direction=1, target=None)
    return {"ok": True}


# ── 20 Hz tick ───────────────────────────────────────────────────────────────
async def tick_loop():
    last_cmd = (0.0, 0.0, 0.0)
    while True:
        t0 = time.monotonic()
        now_ms = int(t0 * 1000)
        world.step_obstacles(DT)
        snap = world.synth_snapshot(now_ms)
        ctx = Ctx((world.rx, world.ry), world.ryaw, snap, now_ms)
        cand = rns.compute(ctx)
        if cand is not None:
            vx, vy, wz = cand.vx.value, cand.vy.value, cand.wz
        else:
            vx = vy = wz = 0.0
        # host speed gate (12 S6.2): forward-sector min d_free clamps vx.
        fwd = [d for i, d in enumerate(snap.profile.d_free)
               if abs(i - 90) <= 20 and d is not None]
        if fwd and vx > 0.0:
            vx = min(vx, speed_gate_f(min(fwd)))
        world.step_robot(vx, vy, wz, DT)
        last_cmd = (vx, vy, wz)
        if rns.take_arrival():
            nav["last_done"] = "path" if nav["state"] == "running_path" else "goto"
            nav["state"] = "arrived"
        fail = rns.take_failure()
        if fail is not None:
            nav["state"] = "failed: %s" % fail.reason.value
        TRACE.append({
            "t": round(now_ms / 1000.0, 2),
            "x": round(world.rx, 2), "y": round(world.ry, 2),
            "yaw": round(world.ryaw, 2),
            "st": rns.nav_state().value,
            "vx": round(vx, 2), "wz": round(wz, 2),
            "dyn": rns._dyn_prev.value,
            "wall_d": (round(rns._last_d_side, 2)
                       if rns._last_d_side is not None else None),
        })
        await broadcast(snap, last_cmd)
        el = time.monotonic() - t0
        await asyncio.sleep(max(0.0, DT - el))


async def broadcast(snap, cmd):
    if not clients:
        return
    tgt = None
    dist_tgt = None
    if rns._mission is not None:
        tgt = rns._mission.endpoint
        dist_tgt = math.hypot(world.rx - tgt[0], world.ry - tgt[1])
    wall = None
    if rns._wall is not None:
        wall = {"side": rns._wall.side.value,
                "followed_m": round(rns._wall.followed_m, 1),
                "d_side": (round(rns._last_d_side, 2)
                           if rns._last_d_side is not None else None)}
    state = {
        "robot": {"x": world.rx, "y": world.ry, "yaw": world.ryaw,
                  "vx": cmd[0], "vy": cmd[1], "wz": cmd[2],
                  "speed": math.hypot(cmd[0], cmd[1]),
                  "gnss": [round(world.rx, 2), round(world.ry, 2)],
                  "fix": "FIXED"},
        "fov": {"half_rad": FOV_HALF_RAD, "range_m": RANGE_MAX_M,
                "blind_m": BLIND_NEAR_M},
        "obstacles": [{"oid": o.oid, "kind": o.kind, "x": o.x, "y": o.y,
                       "x2": o.x2, "y2": o.y2, "thick": o.thick_m,
                       "dynamic": o.dynamic}
                      for o in world.obstacles.values()],
        "path": world.path,
        "waypoints": world.waypoints,
        "nav": {"state": nav["state"], "direction": nav["direction"],
                "target": tgt, "dist_to_target": dist_tgt,
                "rns_state": rns.nav_state().value,
                "subgoal": rns._subgoal_world, "wall": wall},
    }
    msg = json.dumps(state)
    dead = []
    for ws in clients:
        try:
            await ws.send_text(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.remove(ws)


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.append(ws)
    try:
        while True:
            await ws.receive_text()   # keepalive; commands go via REST
    except WebSocketDisconnect:
        if ws in clients:
            clients.remove(ws)


@app.on_event("startup")
async def on_start():
    asyncio.create_task(tick_loop())


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8890, log_level="warning")
