"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: zenoh_world.py
Brief: the Zenoh side of the SIL world -- one link that plays quadruped + rtk_driver + perception (+ p2 grant) for a real p1

Description:
Shared by the two SIL hosts that drive the REAL p1_motion over the production
keys: scripts/dev/pose_stub.py (headless, exit code = verdict) and
scripts/sil/sil_server.py --mode e2e (the browser UI). Both own a SilWorld;
this module owns everything that touches Zenoh, so the wire shapes exist
exactly once:
  in   xbrain/{rid}/rt/motion/cmd_vel     (11 S3.4 body under the S3.0 envelope)
       state/motion/path_progress          (11 S3.5B)   -> latest body + edges
       cmd/motion/relative_move/status     (11 S9.3.2)  -> latest body + edges
  out  xbrain/{rid}/rt/gnss/fix, rt/gnss/heading, rt/clock/status (11 S3.2 /
       S3.3 / S3.11), rt/perception/{profile,objects,status} (11 S3.1B, the
       schema strings three_keys.py requires), cmd/motion/factor (11 S3.6,
       p2 Stage-D stand-in, optional), cmd/motion/route (11 S3.5A, WGS84 about
       the SAME common.geo.enu_origin p1 reads), cmd/motion/relative_move
       (11 S9.3.2, the goto as a body-frame displacement)
A cmd_vel older than CMD_AGE_MAX_S is applied as zero: 11 S9.12 Tier 1
cmd_age > 200 ms stops the chassis, so p1 must keep streaming.

Everything the p1 side needs to agree on is read from the resolved snapshot
(data/run/resolved/p1_motion.yaml): the ENU origin and the relative_move
distance cap. Never from configs/ (10 S5.4.1).

Threading: Zenoh callbacks (Rust threads) only decode and store under a lock;
the host's tick thread / asyncio loop reads. No asyncio calls in callbacks
(CLAUDE.md 4.2). Subscriber handles are held for the link's lifetime (4.3).

What it does NOT do: no world model (sil_world.py), no verdict / UI (the
hosts), no RNS or p1 logic -- nothing is imported from p1 except the
LocalFrame projection, the same class p1 projects with.
"""
from __future__ import annotations

import dataclasses
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from xbrain.p1_motion.path.local_frame import LocalFrame  # noqa: E402

RT_ENDPOINT = "tcp/127.0.0.1:7449"
GEN_ENDPOINT = "tcp/127.0.0.1:7447"
CMD_AGE_MAX_S = 0.2                 # 11 S9.12 Tier 1: cmd_age > 200 ms -> stop
PROFILE_SCHEMA = "perception_profile_v1"
OBJECTS_SCHEMA = "perception_objects_v1"
STATUS_SCHEMA = "perception_status_v1"
FRAME = "base_link"


def open_session(endpoint: str):
    """A client session to one plane's router (dev stack layout)."""
    import zenoh
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["%s"]' % endpoint)
    return zenoh.open(conf)


def unwrap(doc: Any) -> Any:
    """Body of an 11 S3.0 envelope, or the bare body."""
    if isinstance(doc, dict) and "data" in doc and "v" in doc and "src" in doc:
        return doc["data"]
    return doc


class Wire:
    """11 S3.0 envelope stamper: one seq per key; ts / mono in SECONDS."""

    def __init__(self, rid: str) -> None:
        self.rid = rid
        self.seq: Dict[str, int] = {}
        try:
            from xbrain.common.envelope import read_local_boot_id
            self.boot: Optional[str] = read_local_boot_id()
        except Exception:      # noqa: BLE001 -- non-Linux dev box
            self.boot = None

    def env(self, key: str, src: str, data: Any) -> bytes:
        n = self.seq.get(key, 0)
        self.seq[key] = n + 1
        e: Dict[str, Any] = {"v": 1, "rid": self.rid,
                             "ts": time.time(),          # WALL-CLOCK-OK(align/log)
                             "mono": time.monotonic(), "seq": n, "src": src,
                             "ts_sync": True, "data": data}
        if self.boot:
            e["boot"] = self.boot
        return json.dumps(e, ensure_ascii=True).encode("utf-8")


# ---- resolved snapshot facts p1 and the world must share ----------------------
def read_resolved_p1(resolved: Path) -> Dict[str, Any]:
    """geo.enu_origin (lat, lon) and relative_move.max_distance_m from the
    resolved p1 snapshot -- the values p1 itself runs on."""
    d = yaml.safe_load((resolved / "p1_motion.yaml").read_text(encoding="utf-8"))
    o = d["geo"]["enu_origin"]
    return {"lat": float(o["lat"]), "lon": float(o["lon"]),
            "relmove_max_m": float(d["relative_move"]["max_distance_m"])}


def load_map(world: Any, path: Path, *, with_robot: bool) -> Dict[str, Any]:
    """Seed a SilWorld from the frozen field map: fly-aways dropped, walls with
    thickness, car headings, the patrol polyline; the recorded robot pose only
    when asked (the browser keeps SilWorld's default start)."""
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
    if with_robot:
        r = m.get("robot") or {}
        world.rx = float(r.get("x", 0.0))
        world.ry = float(r.get("y", 0.0))
        world.ryaw = float(r.get("yaw", 0.0))
    return m


# ---- bodies (pure) -------------------------------------------------------------
def route_body(frame: LocalFrame, pts: List[Tuple[float, float]], *, cmd_id: str,
               route_rev: int, arrive_radius_m: float,
               route_id: str = "r-sil") -> Dict[str, Any]:
    """11 S3.5A RouteGeometry, one chunk, WGS84 points about the site origin."""
    points = []
    for i, (x, y) in enumerate(pts):
        lat, lon = frame.to_latlon(x, y)
        points.append({"lat": lat, "lon": lon, "seq": i, "arrive_radius_m": arrive_radius_m})
    total = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                for i in range(len(pts) - 1))
    return {"v": 1, "op": "set", "cmd_id": cmd_id, "route_id": route_id,
            "route_rev": route_rev, "loop_mode": "oneway", "total_len_m": round(total, 2),
            "frame": "wgs84", "points": points, "chunk": {"index": 0, "total": 1}}


def clear_body(cmd_id: str, route_id: str, route_rev: int) -> Dict[str, Any]:
    """11 S3.5A op=clear (20 S9.0.3 cancel)."""
    return {"v": 1, "op": "clear", "cmd_id": cmd_id, "route_id": route_id,
            "route_rev": route_rev, "points": []}


def relmove_body(cmd_id: str, dx_m: float, dy_m: float) -> Dict[str, Any]:
    """11 S9.3.2 relative_move: a body-frame displacement, no rotation."""
    return {"cmd_id": cmd_id, "dx_m": dx_m, "dy_m": dy_m, "dyaw_rad": 0.0,
            "max_speed_mps": 1.5, "max_yaw_rate_radps": 1.2, "timeout_s": 120.0,
            "abort_on_obstacle": False, "source": "sil"}


def world_to_body(x: float, y: float, yaw: float, gx: float, gy: float) -> Tuple[float, float]:
    """A world goal -> (dx, dy) in the robot's body frame (ENU yaw)."""
    ex, ey = gx - x, gy - y
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * ex + s * ey, -s * ex + c * ey)


def perception_bodies(snap: Any) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
    """The three 11 S3.1B bodies from a sil_world PerceptionSnapshot (the DTO
    field names ARE the wire names; only schema / frame are added)."""
    def jsonic(v: Any) -> Any:
        # the DTOs hold tuples; the wire (and three_keys' _array) wants lists.
        if isinstance(v, tuple):
            return [jsonic(e) for e in v]
        if isinstance(v, dict):
            return {k: jsonic(e) for k, e in v.items()}
        return v
    prof = jsonic(dataclasses.asdict(snap.profile))
    prof.update({"schema": PROFILE_SCHEMA, "frame": FRAME})
    objs = jsonic(dataclasses.asdict(snap.objects))
    objs.update({"schema": OBJECTS_SCHEMA, "frame": FRAME})
    stat = jsonic(dataclasses.asdict(snap.status))
    stat.update({"schema": STATUS_SCHEMA})
    return prof, objs, stat


def gnss_bodies(x: float, y: float, yaw: float, speed: float, frame: LocalFrame,
                now_s: float, t_start: float) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """11 S3.2 GnssFix + S3.3 GnssHeading for a pose in site metres: an RTK
    fixed, dual-antenna L1 solution (the healthy case; degraded fixes are a
    scenario knob for later)."""
    lat, lon = frame.to_latlon(x, y)
    fix = {"lat": lat, "lon": lon, "alt": 4.0, "fix_type": "rtk_fixed", "hdop": 0.6,
           "sats": 24, "cov_h_m": 0.02, "cov_v_m": 0.03, "age_s": 0.05, "t_mono": now_s}
    heading = {"heading_rad": yaw,
               "heading_true_north_rad": (math.pi / 2 - yaw) % (2 * math.pi),
               "heading_valid": True, "source": "dual_antenna", "level": 1,
               "cov_rad": 0.005, "speed_mps": speed, "cog_rad": yaw, "baseline_m": 0.62,
               "baseline_valid": True, "yaw_capable": True, "i_heading": 1.0,
               "age_s": 0.05, "t_mono": now_s, "since_mono": t_start}
    return fix, heading


def clock_body(now_s: float) -> Dict[str, Any]:
    """11 S3.11 ClockStatus: synced, (mono_ref, utc_ref) pair."""
    return {"sync": True, "source": "gnss", "mono_ref": now_s,
            "utc_ref": time.time()}                     # WALL-CLOCK-OK(align/log)


def factor_body() -> Dict[str, Any]:
    """11 S3.6 HealthFactor: the p2 Stage-D grant stand-in."""
    return {"speed_factor": 1.0, "max_profile": "patrol", "allow_motion": True,
            "reason": "none", "detail_ref": "health/summary"}


# ---- the link ---------------------------------------------------------------------
class ZenohWorldLink:
    """Sessions, publishers, subscriptions and the latest-value slots."""

    def __init__(self, rid: str, frame: LocalFrame, *, grant: bool = True,
                 relmove_max_m: float = 20.0) -> None:
        self.rid = rid
        self.frame = frame
        self.grant = grant
        self.relmove_max_m = relmove_max_m
        self._wire = Wire(rid)
        self._rt = open_session(RT_ENDPOINT)
        self._gen = open_session(GEN_ENDPOINT)
        self._pubs = {
            "fix": self._rt.declare_publisher("xbrain/%s/rt/gnss/fix" % rid),
            "heading": self._rt.declare_publisher("xbrain/%s/rt/gnss/heading" % rid),
            "clock": self._rt.declare_publisher("xbrain/%s/rt/clock/status" % rid),
            "profile": self._rt.declare_publisher("xbrain/%s/rt/perception/profile" % rid),
            "objects": self._rt.declare_publisher("xbrain/%s/rt/perception/objects" % rid),
            "status": self._rt.declare_publisher("xbrain/%s/rt/perception/status" % rid),
            "factor": self._gen.declare_publisher("cmd/motion/factor"),
            "route": self._gen.declare_publisher("cmd/motion/route"),
            "relmove": self._gen.declare_publisher("cmd/motion/relative_move"),
        }
        self._lock = threading.Lock()
        self.cmd: Dict[str, Any] = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "rx": -1e9, "n": 0,
                                    "limiter": None, "source": None, "v_max": None,
                                    "profile": None, "bad": 0}
        self.progress: Optional[Dict[str, Any]] = None
        self.relmove: Optional[Dict[str, Any]] = None
        self._changes: List[Tuple[str, Any, Any]] = []
        self._route_rev = 0
        self._cmd_seq = 0
        self.t_start = time.monotonic()
        self._last_1hz = -1e9
        self._subs = [
            self._rt.declare_subscriber("xbrain/%s/rt/motion/cmd_vel" % rid, self._on_cmd),
            self._gen.declare_subscriber("state/motion/path_progress", self._on_progress),
            self._gen.declare_subscriber("cmd/motion/relative_move/status", self._on_relmove),
        ]

    # ---- Rust-thread callbacks: decode + store only ----------------------------
    def _on_cmd(self, sample: Any) -> None:
        try:
            d = unwrap(json.loads(bytes(sample.payload).decode("utf-8")))
            g = d.get("gate") or {}
            with self._lock:
                self.cmd.update(vx=float(d["vx"]), vy=float(d.get("vy", 0.0)),
                                wz=float(d["wz"]), rx=time.monotonic(),
                                limiter=g.get("limiter"), source=g.get("source"),
                                v_max=g.get("v_max"), profile=g.get("profile"))
                self.cmd["n"] += 1
        except Exception:      # noqa: BLE001 -- a bad frame is counted, never fatal
            with self._lock:
                self.cmd["bad"] += 1

    def _on_progress(self, sample: Any) -> None:
        d = unwrap(json.loads(bytes(sample.payload).decode("utf-8")))
        with self._lock:
            prev = self.progress
            self.progress = d
            if prev is None or prev.get("state") != d.get("state"):
                self._changes.append(("progress", d.get("state"), d.get("fail_reason")))

    def _on_relmove(self, sample: Any) -> None:
        d = unwrap(json.loads(bytes(sample.payload).decode("utf-8")))
        with self._lock:
            self.relmove = d
            self._changes.append(("relmove", d.get("state"),
                                  d.get("abort_reason") or d.get("code")))

    # ---- host-side reads ----------------------------------------------------------
    def latest_cmd(self, now_s: float) -> Tuple[float, float, float, bool]:
        """(vx, vy, wz, fresh): zero when the last cmd_vel is older than the
        Tier-1 window."""
        with self._lock:
            fresh = now_s - self.cmd["rx"] <= CMD_AGE_MAX_S
            if fresh:
                return self.cmd["vx"], self.cmd["vy"], self.cmd["wz"], True
            return 0.0, 0.0, 0.0, False

    def cmd_info(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.cmd)

    def snapshot(self) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
        with self._lock:
            return self.progress, self.relmove

    def drain_changes(self) -> List[Tuple[str, Any, Any]]:
        with self._lock:
            out, self._changes = self._changes, []
            return out

    # ---- publishing ------------------------------------------------------------------
    def publish_world(self, x: float, y: float, yaw: float, vx: float, vy: float,
                      snap: Any, now_s: float) -> None:
        """One tick's worth of the three simulated peripherals."""
        fix, heading = gnss_bodies(x, y, yaw, math.hypot(vx, vy), self.frame, now_s, self.t_start)
        self._pubs["fix"].put(self._wire.env("fix", "rtk_driver", fix))
        self._pubs["heading"].put(self._wire.env("heading", "rtk_driver", heading))
        prof, objs, stat = perception_bodies(snap)
        self._pubs["profile"].put(self._wire.env("profile", "perception", prof))
        self._pubs["objects"].put(self._wire.env("objects", "perception", objs))
        self._pubs["status"].put(self._wire.env("status", "perception", stat))
        if now_s - self._last_1hz >= 1.0:
            self._last_1hz = now_s
            self._pubs["clock"].put(self._wire.env("clock", "rtk_driver", clock_body(now_s)))
            if self.grant:
                self._pubs["factor"].put(json.dumps(factor_body()).encode("utf-8"))

    def send_route(self, pts: List[Tuple[float, float]], *, arrive_radius_m: float) -> int:
        """cmd/motion/route op=set (one chunk). Returns the route_rev used."""
        self._route_rev += 1
        self._cmd_seq += 1
        self._pubs["route"].put(json.dumps(route_body(
            self.frame, pts, cmd_id="rg-sil-%d" % self._cmd_seq, route_rev=self._route_rev,
            arrive_radius_m=arrive_radius_m)).encode("utf-8"))
        return self._route_rev

    def send_clear(self) -> None:
        self._cmd_seq += 1
        self._pubs["route"].put(json.dumps(clear_body(
            "rg-sil-%d" % self._cmd_seq, "r-sil", self._route_rev)).encode("utf-8"))

    def send_relmove(self, dx_m: float, dy_m: float) -> str:
        """cmd/motion/relative_move for a body-frame displacement."""
        self._cmd_seq += 1
        cmd_id = "rm-sil-%d" % self._cmd_seq
        self._pubs["relmove"].put(json.dumps(relmove_body(cmd_id, dx_m, dy_m)).encode("utf-8"))
        return cmd_id

    def close(self) -> None:
        for s in self._subs:
            try:
                s.undeclare()
            except Exception:      # noqa: BLE001
                pass
        self._subs = []
        self._rt.close()
        self._gen.close()
