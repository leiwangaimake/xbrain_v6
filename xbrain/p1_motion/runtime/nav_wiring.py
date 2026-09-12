"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: nav_wiring.py
Brief: the p1 20 Hz navigation thread -- latest-value slots in, NavTick + MissionHost, cmd_vel / progress / status out

Description:
P7.2: RNS enters the running process. This module owns everything that touches
a session or a clock so the logic underneath (nav/*.py, sources/rns_avoid.py)
stays pure. One thread, fixed 50 ms period (12 S2.2), each tick:
  1 snapshot   pose from the rt/gnss caches (aged by rx time against
               timeouts_ms.gnss), perception_in.latest, the HealthFactorSlot
               view, the soft-estop latch, the teleop arbitration -- read
               ONCE into a NavInputs (RTC-6 latest-value slots, no queues)
  2 entries    drained route / relative_move results -> MissionHost
  3 tick       NavTick.run -> NavOutput (RNS candidate through arbiter + gate)
  4 output     CtrlLoop.run_one_tick publishes rt/motion/cmd_vel EVERY tick
               (chassis Tier 1), estop-first (P1-21), 11 S3.4 body with the
               gate block; then MissionHost.after_tick's emits -> path_progress
               / relative_move/status / event/{sev}/motion
  5 pacing     absolute next-deadline sleep; an overrun is counted, never
               compensated by a burst (a burst of stale cmd_vel is worse than
               one late one)
An exception inside a tick is logged, that tick publishes ZERO through the
same CtrlLoop path, and the loop continues (CLAUDE.md 4.4 "本拍零速 + 落
fault + 下一拍仍在跑").

Threads. Zenoh callbacks (Rust threads, CLAUDE.md 4.2) only decode and store:
route frames are parsed by the RouteAssembler under a lock and the RESULT is
queued; relative_move bodies are queued raw (the translation needs the tick's
pose); factor bodies go straight into the slot. The heartbeat loop in
main_wiring publishes state/teleop from the same TeleopTracker, so both sides
take teleop_lock around it -- the tracker's hysteresis state is not thread
safe by itself.

Keys (11 S1.1.6 whitelist, p1 rows): sub cmd/motion/route (P1-11),
cmd/motion/relative_move (P1-5), cmd/motion/factor (P1-4) on the general
plane; pub xbrain/{rid}/rt/motion/cmd_vel (RT), state/motion/path_progress
(P1-12), cmd/motion/relative_move/status (P1-6), state/arb/motion (P1-22)
and event/{sev}/arbitration (P1-23) on the general plane.
Bodies on the general plane are accepted bare or inside the 11 S3.0 envelope
(unwrap_body) because today's producers differ: p2 publishes relative_move
bare while RT-plane producers envelope everything.

What it does NOT do: no rotation permit / fence clip / jerk limiter (not
chained this phase, NEXT.md), no Nav2 spin delegate, no RobotState estop
back-check (11 S7A.6.5 E-2; quadruped not built). It never reads the config source (NavConfig arrives
from the resolved snapshots via runtime/nav_cfg.py).

Trap: publishing cmd_vel from the heartbeat loop "as well" to be safe. Two
publishers on one Tier-1 key interleave and the chassis sees a 10 Hz jitter
on top of the 20 Hz stream; the CtrlLoop callback is the single sink
(12 S2.2 step 10).
"""
from __future__ import annotations

import collections
import json
import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional, Tuple

from xbrain.common.envelope.envelope import Envelope, encode
from xbrain.p1_motion.ctrl_loop import CtrlLoop, CtrlState
from xbrain.p1_motion.nav.arb_state import SUSPENDED_SOFT_ESTOP, ArbVisibility
from xbrain.p1_motion.nav.health_factor import (STATE_OK, HealthFactorError,
                                                HealthFactorSlot, HealthView)
from xbrain.p1_motion.nav.mission_host import (CH_EVENT, CH_PROGRESS,
                                               CH_RELMOVE, Emit, MissionHost)
from xbrain.p1_motion.nav.nav_tick import NavInputs, NavOutput, NavTick, ctrl_state_for
from xbrain.p1_motion.nav.route_intake import RouteAssembler, RouteIntakeError
from xbrain.p1_motion.path import gnss_pose
from xbrain.p1_motion.path.local_frame import LocalFrameError
from xbrain.p1_motion.rns.source import RnsSource
from xbrain.p1_motion.runtime.nav_cfg import NavConfig
from xbrain.p1_motion.sources.arbiter_p1 import P1Arbiter
from xbrain.p1_motion.sources.rns_avoid import RnsAvoidSource

_logger = logging.getLogger("xbrain.p1.nav")

CMD_ROUTE_TOPIC = "cmd/motion/route"                      # P1-11
CMD_RELMOVE_TOPIC = "cmd/motion/relative_move"            # P1-5
CMD_FACTOR_TOPIC = "cmd/motion/factor"                    # P1-4
STATE_PROGRESS_TOPIC = "state/motion/path_progress"       # P1-12
STATE_ARB_TOPIC = "state/arb/motion"                      # P1-22 (11 S7A.8)
RELMOVE_STATUS_TOPIC = "cmd/motion/relative_move/status"  # P1-6
#: 12 S2.2: 20 Hz.
TICK_PERIOD_S = 0.05
#: periods kept for the heartbeat's p99 / max (20 s at 20 Hz).
_STATS_WINDOW = 400
#: mission nominal tier name reported in gate.profile_req (20 S8.1A).
PROFILE_REQ = "patrol"


def unwrap_body(doc: Any) -> Any:
    """A general-plane frame is the body itself or an 11 S3.0 envelope around
    it; the envelope is recognised by its three always-present keys."""
    if isinstance(doc, dict) and "data" in doc and "v" in doc and "src" in doc:
        return doc["data"]
    return doc


def _load_json(sample: Any) -> Optional[Any]:
    try:
        return json.loads(bytes(sample.payload).decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None


@dataclass(frozen=True)
class PoseView:
    """The tick's pose facts derived from the two gnss caches."""
    xy: Optional[Tuple[float, float]]
    yaw: Optional[float]
    heading_valid: bool
    i_fix: Optional[float]
    i_heading: Optional[float]


class NavRuntime:
    """Owns the thread, the subscriptions and the publishers of the loop."""

    def __init__(self, *, cfg: NavConfig, rid: str, boot: str, rt: Any, gen: Any,
                 perception_in: Any, estop_latch: Any, teleop_tracker: Any,
                 teleop_lock: threading.Lock, gnss_cache: Dict[str, Any],
                 fix_cache: Dict[str, Any], clock_cache: Dict[str, Any],
                 stop_flag: Dict[str, Any]) -> None:
        self._cfg = cfg
        self._rid = rid
        self._boot = boot
        self._rt = rt
        self._gen = gen
        self._perception = perception_in
        self._estop = estop_latch
        self._teleop = teleop_tracker
        self._teleop_lock = teleop_lock
        self._gnss = gnss_cache
        self._fix = fix_cache
        self._clock = clock_cache
        self._stop = stop_flag
        now_ms = int(time.monotonic() * 1000.0)
        # ONE RnsSource for the process lifetime (memory grid / acceptance state
        # / watchdog live in it); its startup assertions (20 S7A.1) run here.
        rns = RnsSource(cfg=cfg.rns, r_eff_m=cfg.r_eff_m)
        self._src = RnsAvoidSource(rns, cfg.rns["rns"])
        self._arb = P1Arbiter()
        self._arbvis = ArbVisibility()          # P1-22/23 (11 S7A.8)
        self._tick = NavTick(self._src, self._arb, v_nom_mps=cfg.v_nom_mps,
                             wz_max_rps=cfg.max_wz_radps,
                             spec_max_vx_mps=cfg.max_vx_mps,
                             holonomic=cfg.holonomic,
                             v_obstacle_avoid_mps=cfg.v_obstacle_avoid_mps)
        self._host = MissionHost(self._src, relmove_limits=cfg.relmove,
                                 holonomic=cfg.holonomic, now_mono_ms=now_ms)
        self._health = HealthFactorSlot(cfg.health_degrade_ms, cfg.health_dead_ms)
        self._assembler = RouteAssembler(cfg.frame)
        self._lock = threading.Lock()
        self._route_q: Deque[Any] = collections.deque()
        self._relmove_q: Deque[Any] = collections.deque()
        self._subs: List[Any] = []                 # strong refs (CLAUDE.md 4.3)
        self._cmd_pub: Any = None
        self._progress_pub: Any = None
        self._status_pub: Any = None
        self._arb_pub: Any = None
        self._ctrl = CtrlLoop(self._publish_cmd_vel, holonomic=cfg.holonomic)
        self._cur: Optional[NavOutput] = None
        self._seq = {"cmd_vel": 0, "progress": 0, "status": 0, "event": 0, "arb": 0}
        self._periods: Deque[float] = collections.deque(maxlen=_STATS_WINDOW)
        # the heartbeat thread reads _periods while this thread appends: a deque
        # iterated during a concurrent append raises, so both sides lock.
        self._stats_lock = threading.Lock()
        self._health_ever_ok = False
        self._inputs_ever_ready = False
        self._fault_events = 0
        self._ticks = 0
        self._overruns = 0
        self._tick_errors = 0
        self._counts = {"route_rx": 0, "route_bad": 0, "relmove_rx": 0,
                        "factor_bad": 0, "publish_fail": 0}
        self._health_state: Optional[str] = None
        self._event_boot = os.urandom(3).hex()
        self._thread: Optional[threading.Thread] = None

    # ---- lifecycle -----------------------------------------------------------
    def declare(self) -> None:
        """Subscriptions + publishers. Handles land in self._subs / self._*_pub
        (the CI static rule wants list.append / self.x =)."""
        self._subs.append(self._gen.declare_subscriber(CMD_ROUTE_TOPIC, self._on_route))
        self._subs.append(self._gen.declare_subscriber(CMD_RELMOVE_TOPIC, self._on_relmove))
        self._subs.append(self._gen.declare_subscriber(CMD_FACTOR_TOPIC, self._on_factor))
        self._cmd_pub = self._rt.declare_publisher("xbrain/%s/rt/motion/cmd_vel" % self._rid)
        self._progress_pub = self._gen.declare_publisher(STATE_PROGRESS_TOPIC)
        self._status_pub = self._gen.declare_publisher(RELMOVE_STATUS_TOPIC)
        self._arb_pub = self._gen.declare_publisher(STATE_ARB_TOPIC)
        _logger.info("p1 nav loop wired: sub %s %s %s; pub rt/motion/cmd_vel %s %s "
                     "(rid=%s holonomic=%s v_nom=%.2f wz_max=%.2f)",
                     CMD_ROUTE_TOPIC, CMD_RELMOVE_TOPIC, CMD_FACTOR_TOPIC,
                     STATE_PROGRESS_TOPIC, RELMOVE_STATUS_TOPIC, self._rid,
                     self._cfg.holonomic, self._cfg.v_nom_mps, self._cfg.max_wz_radps)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="p1-nav-20hz", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Join the thread (stop_flag is the shared exit signal), then undeclare."""
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        for s in self._subs:
            try:
                s.undeclare()
            except Exception:      # noqa: BLE001 -- teardown must not raise
                pass
        self._subs = []

    def stats(self) -> Dict[str, Any]:
        """Heartbeat line: period p99 / max (CLAUDE.md 4.4: P99 <= 60 ms, max
        <= 100 ms), overruns, holder, nav state, intake counts."""
        with self._stats_lock:
            p = sorted(self._periods)
        p99 = p[int(0.99 * (len(p) - 1))] * 1000.0 if p else None
        pmax = p[-1] * 1000.0 if p else None
        out = self._cur
        return {"ticks": self._ticks, "overruns": self._overruns,
                "tick_errors": self._tick_errors,
                "period_p99_ms": None if p99 is None else round(p99, 1),
                "period_max_ms": None if pmax is None else round(pmax, 1),
                "source": out.source if out else None,
                "nav_state": out.nav_state if out else None,
                "limiter": out.limiter if out else None,
                "route_state": self._host.route_state,
                "ctrl_state": self._ctrl.state,
                "health": self._health_state, **self._counts}

    # ---- Rust-thread callbacks (decode + store only, CLAUDE.md 4.2) ----------
    def _on_route(self, sample: Any) -> None:
        doc = _load_json(sample)
        body = unwrap_body(doc)
        with self._lock:
            self._counts["route_rx"] += 1
            try:
                res = self._assembler.accept(body)
            except (RouteIntakeError, LocalFrameError) as exc:
                self._counts["route_bad"] += 1
                n = self._counts["route_bad"]
                if n == 1 or n % 20 == 0:
                    _logger.warning("p1 cmd/motion/route rejected (n=%d): %s", n, exc)
                return
            if res is not None:
                self._route_q.append(res)

    def _on_relmove(self, sample: Any) -> None:
        doc = _load_json(sample)
        with self._lock:
            self._counts["relmove_rx"] += 1
            self._relmove_q.append(unwrap_body(doc))

    def _on_factor(self, sample: Any) -> None:
        doc = _load_json(sample)
        try:
            self._health.on_message(unwrap_body(doc), int(time.monotonic() * 1000.0))
        except HealthFactorError as exc:
            self._counts["factor_bad"] += 1
            n = self._counts["factor_bad"]
            if n == 1 or n % 100 == 0:
                _logger.warning("p1 cmd/motion/factor rejected (n=%d): %s", n, exc)

    # ---- the loop ------------------------------------------------------------
    def _fault_event(self, exc: BaseException) -> None:
        """CLAUDE.md 4.4 '落 fault': event/fault/motion for a failed tick, the
        first and then every 100th so a tight failure does not flood p5."""
        self._fault_events += 1
        if self._fault_events != 1 and self._fault_events % 100 != 0:
            return
        try:
            self._publish_event(Emit(CH_EVENT, {
                "title": "nav_tick_failed",
                "dedup_key": "nav:tick_failed",
                "detail": {"error": type(exc).__name__, "count": self._tick_errors}},
                "fault"))
        except Exception:      # noqa: BLE001 -- the fault path must not raise
            pass

    def _loop(self) -> None:
        self._ctrl.transition(CtrlState.WAIT_INPUT)
        next_t = time.monotonic()
        last = next_t
        while not self._stop.get("stop"):
            t0 = time.monotonic()
            with self._stats_lock:
                self._periods.append(t0 - last)
            last = t0
            try:
                self._one_tick(t0)
            except Exception as exc:      # noqa: BLE001 -- 4.4: zero this tick, keep looping
                self._tick_errors += 1
                _logger.exception("p1 nav tick failed (n=%d): %s", self._tick_errors, exc)
                self._cur = None
                # 12 S11: an abnormal tick is SAFE_STOP (zero); the next good
                # tick moves the state word back through ctrl_state_for.
                self._ctrl.transition(CtrlState.SAFE_STOP)
                self._ctrl.run_one_tick()               # zero on every axis
                self._fault_event(exc)
            self._ticks += 1
            next_t += TICK_PERIOD_S
            delay = next_t - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                self._overruns += 1
                next_t = time.monotonic()

    def _pose_view(self, t0: float) -> PoseView:
        """The gnss caches aged by RX time (timeouts_ms.gnss): a stale fix is
        no fix (12 S3.3 gnss row: stop), a stale heading is no heading."""
        dead_s = self._cfg.gnss_dead_ms / 1000.0
        gh = self._gnss.get("data") if t0 - self._gnss.get("rx", -1e9) <= dead_s else None
        gf = self._fix.get("data") if t0 - self._fix.get("rx", -1e9) <= dead_s else None
        pose = gnss_pose.assemble_pose(gh, gf)
        xy: Optional[Tuple[float, float]] = None
        i_fix = pose.get("i_fix")
        if pose.get("lat") is not None and pose.get("lon") is not None and i_fix is not None:
            try:
                xy = self._cfg.frame.to_xy(pose["lat"], pose["lon"])
            except LocalFrameError:
                xy = None
        heading_valid = bool(pose.get("heading_valid")) and gh is not None
        yaw = pose.get("heading_rad") if heading_valid else None
        if yaw is not None and not isinstance(yaw, (int, float)):
            yaw, heading_valid = None, False
        i_heading = pose.get("i_heading") if heading_valid else None
        return PoseView(xy=xy, yaw=None if yaw is None else float(yaw),
                        heading_valid=heading_valid,
                        i_fix=None if xy is None else float(i_fix),
                        i_heading=None if i_heading is None else float(i_heading))

    def _teleop_active(self, now_ms: int) -> bool:
        with self._teleop_lock:
            return self._teleop.arbitrate(now_ms) is not None

    def _ts_sync(self) -> bool:
        return bool((self._clock.get("data") or {}).get("sync", False))

    def _one_tick(self, t0: float) -> None:
        now_ms = int(t0 * 1000.0)
        ts_wall = time.time()                     # WALL-CLOCK-OK(align/log)
        pose = self._pose_view(t0)
        with self._lock:
            routes = list(self._route_q)
            self._route_q.clear()
            relmoves = list(self._relmove_q)
            self._relmove_q.clear()
        emits: List[Emit] = []
        for r in routes:
            emits += self._host.on_route(r, now_ms, pose.xy)
        health = self._health.view(now_ms)
        for b in relmoves:
            emits += self._host.on_relmove(b, now=now_ms, pose=pose.xy,
                                           yaw_rad=pose.yaw, heading_valid=pose.heading_valid,
                                           allow_motion=health.allow_motion)
        inp = NavInputs(now_mono_ms=now_ms, pose_xy=pose.xy, yaw_rad=pose.yaw,
                        heading_valid=pose.heading_valid, i_fix=pose.i_fix,
                        i_heading=pose.i_heading,
                        perception=self._perception.latest(now_ms), health=health,
                        estop=bool(self._estop.is_active()),
                        teleop_active=self._teleop_active(now_ms), ts_wall_s=ts_wall)
        out = self._tick.run(inp)
        self._cur = out
        # 12 S11 state word (WAIT_GRANT / WAIT_INPUT / SAFE_STOP / READY / ACTIVE):
        # the CtrlLoop zeroes every non-ACTIVE state itself, under the vetoes.
        if health.allow_motion:
            self._health_ever_ok = True
        state = ctrl_state_for(inp, out, health_ever_ok=self._health_ever_ok,
                               inputs_ever_ready=self._inputs_ever_ready)
        if state in (CtrlState.READY, CtrlState.ACTIVE):
            self._inputs_ever_ready = True
        self._ctrl.transition(state)
        # single sink (12 S2.2 step 10): the CtrlLoop callback publishes.
        self._ctrl.run_one_tick(computed_vx=out.vx, computed_wz=out.wz,
                                computed_vy=out.vy, estop=inp.estop)
        emits += self._host.after_tick(inp, out)
        self._publish_emits(emits)
        self._health_edge(health)
        self._publish_arb(inp, now_ms)

    # ---- publishing ----------------------------------------------------------
    def _envelope(self, data: Dict[str, Any], seq_key: str) -> bytes:
        """11 S3.0 envelope through the common encoder: ts / mono are SECONDS
        (float), boot rides with mono (CLK-C4), seq per key. NOT
        gnss_pose.stamp_envelope, which stamps milliseconds -- a C++ consumer
        decoding rt/motion/cmd_vel per S3.0 would read those as seconds."""
        env = Envelope(v=1, rid=self._rid,
                       ts=time.time(),               # WALL-CLOCK-OK(align/log)
                       mono=time.monotonic(), boot=self._boot or None,
                       seq=self._seq[seq_key], src="p1_motion",
                       ts_sync=self._ts_sync(), data=data)
        self._seq[seq_key] += 1
        return json.dumps(encode(env), ensure_ascii=False).encode("utf-8")

    def _publish_cmd_vel(self, vx: float, wz: float, vy: float = 0.0) -> None:
        """CtrlLoop's sink: the 11 S3.4 CmdVel body + gate block. Called with
        (vx, wz, vy) on a holonomic loop, (vx, wz) otherwise."""
        out = self._cur
        gate: Dict[str, Any]
        if out is None:
            # the exception path (SAFE_STOP): a P1-internal fault is a health
            # veto in 11 S9.6.5 terms (row 3, FATAL item), never an estop claim.
            gate = {"v_max": 0.0, "profile": PROFILE_REQ, "profile_req": PROFILE_REQ,
                    "limiter": "health", "limiter_all": ["health"], "h_factor": 0.0,
                    "i_factor": 0.0, "raw_vx": 0.0, "source": "hold"}
        else:
            gate = {"v_max": round(out.v_max, 4), "profile": out.profile,
                    "profile_req": PROFILE_REQ, "limiter": out.limiter,
                    "limiter_all": list(out.limiter_all),
                    "h_factor": round(out.h_factor, 4), "i_factor": round(out.i_factor, 4),
                    "raw_vx": round(out.raw_vx, 4), "source": out.source}
        body = {"vx": round(vx, 4), "vy": round(vy, 4), "wz": round(wz, 4),
                "vz": 0.0, "v_roll": 0.0, "v_pitch": 0.0, "gate": gate}
        try:
            self._cmd_pub.put(self._envelope(body, "cmd_vel"))
        except Exception as exc:      # noqa: BLE001 -- the tick still counts
            self._counts["publish_fail"] += 1
            if self._counts["publish_fail"] == 1:
                _logger.error("p1 cmd_vel publish failed: %s", exc)
            raise

    def _publish_emits(self, emits: List[Emit]) -> None:
        for e in emits:
            try:
                if e.channel == CH_PROGRESS:
                    self._progress_pub.put(self._envelope(e.body, "progress"))
                elif e.channel == CH_RELMOVE:
                    self._status_pub.put(self._envelope(e.body, "status"))
                elif e.channel == CH_EVENT:
                    self._publish_event(e)
            except Exception as exc:      # noqa: BLE001
                self._counts["publish_fail"] += 1
                _logger.error("p1 nav publish %s failed: %s", e.channel, exc)

    def _publish_event(self, e: Emit) -> None:
        """event/{sev}/motion in the shape the p5 pipeline already takes from
        p1 (the zone events): eid / title / dedup_key / detail / src / ts."""
        self._seq["event"] += 1
        key = "event/%s/motion" % (e.severity or "warn")
        self._gen.put(key, json.dumps({
            "eid": "nav-%s-%d" % (self._event_boot, self._seq["event"]),
            "title": e.body.get("title", "nav"),
            "dedup_key": e.body.get("dedup_key", "nav"),
            "detail": e.body.get("detail", {}),
            "src": "p1_motion", "ts": 0.0,
        }, ensure_ascii=False).encode("utf-8"))

    def _publish_arb(self, inp: NavInputs, now_ms: int) -> None:
        """P1-22 state/arb/motion (change + 1 Hz) and P1-23
        event/{sev}/arbitration (change only), 11 S7A.8 / S7A.5.1 / S7A.7."""
        body, events = self._arbvis.observe(
            holder=self._arb.holder(), snapshot=self._arb.snapshot(),
            suspended=SUSPENDED_SOFT_ESTOP if inp.estop else None, now_mono_ms=now_ms)
        try:
            if body is not None:
                self._arb_pub.put(self._envelope(body, "arb"))
            for ev in events:
                self._seq["event"] += 1
                self._gen.put("event/%s/arbitration" % ev.severity, json.dumps({
                    "eid": "arb-%s-%d" % (self._event_boot, self._seq["event"]),
                    "title": ev.action, "dedup_key": ev.dedup_key,
                    "detail": ev.detail, "src": "p1_motion", "ts": 0.0,
                }, ensure_ascii=False).encode("utf-8"))
        except Exception as exc:      # noqa: BLE001
            self._counts["publish_fail"] += 1
            _logger.error("p1 state/arb/motion publish failed: %s", exc)

    def _health_edge(self, view: HealthView) -> None:
        """11 S3.6: the degrade / dead transitions raise a warn event; logged
        on the edge only so a silent p2 does not flood at 20 Hz."""
        if view.state == self._health_state:
            return
        prev, self._health_state = self._health_state, view.state
        if view.state == STATE_OK:
            _logger.info("p1 health factor %s -> ok (speed_factor=%.2f allow=%s)",
                         prev, view.speed_factor, view.allow_motion)
            return
        _logger.warning("p1 health factor %s -> %s (age_ms=%s): speed_factor=%.2f allow=%s",
                        prev, view.state, view.age_ms, view.speed_factor, view.allow_motion)
        if prev is not None:
            self._publish_event(Emit(CH_EVENT, {
                "title": "health_factor_%s" % view.state,
                "dedup_key": "nav:health:%s" % view.state,
                "detail": {"state": view.state, "age_ms": view.age_ms}}, "warn"))
