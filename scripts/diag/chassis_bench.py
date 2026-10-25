#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: chassis_bench.py
Brief: Bench console for the real M20S: enable, stand / prone, estop and six closed-loop basic motions

Description:
Plan B of 2026-10-26 (user ruling): verify the quadruped <-> chassis segment
on its own -- forward, backward, turn left / right in place, strafe left /
right, plus stand / prone and the soft estop -- before perception and RTK are
ready for the full chain (p4 -> p2 -> p1 relmove -> RNS). The full chain
cannot move the robot today for designed reasons: p2 holds allow_motion
false without perception (14 S8.3), p1 refuses a relmove with no RTK fix or
heading (12 S4.5.2), nothing in the stack sends `enable` (BOOT-L1) and P1-7
(cmd/chassis/mode -> rt/chassis/mode) is not implemented.

What this console does, and which contract path each piece uses:
  * cmd_vel   publishes xbrain/{rid}/rt/motion/cmd_vel at 20 Hz for its whole
              life, standing in for p1_motion (11 S9.1, 13 Tier1). The stream
              never pauses: quadruped's Tier1 locks (timeout_lock) 200 ms
              after the last command, and only another `enable` unlocks it.
  * enable / stand / prone   bare GEN cmd/chassis/ctrl (11 S9.3.4) -> the
              real chassis_relay CR-11 -> rt/chassis/ctrl; acks come back on
              cmd/chassis/ctrl/ack (CR-12). Same path p4's voice bypass uses,
              so "站起来" / "趴下" spoken into the robot also work while this
              console runs.
  * nav / normal   rt/chassis/mode {cmd_id, usage_mode} on the RT plane,
              standing in for the missing P1-7 forwarder. Tier1 outputs zero
              unless usage_mode is navigation (NAV-111).
  * estop     bare GEN cmd/estop (11 S7.1) -> chassis_relay CR-1. quadruped
              bumps estop_epoch and holds zero until the cmd_vel stream echoes
              the new epoch; this console echoes it only on the NEXT motion
              command (14 S3.7 / U35: the new motion command is the re-arm).
              A voice estop is seen the same way (epoch change) and aborts the
              running motion.

What it deliberately does NOT do:
  * no obstacle avoidance, no perception, no RNS. Every motion needs a
    person on site who has checked the envelope (2026-10-23: an unattended
    in-place turn pushed over a TV wall). The console asks for that, as the
    literal word yes, before every single motion.
  * no default speeds. The operator passes --lin / --lat / --ang; the
    constants below are ceilings the tool refuses to exceed, not values it
    picks.
  * it never runs next to p1_motion: two cmd_vel publishers would interleave
    p1's zeros with ours. A foreign cmd_vel at startup refuses to start, one
    seen later zeroes the stream and aborts.

Every motion is closed-loop on quadruped's integrated odometry
(rt/chassis/state odom: x, y from MOTION_INFO, yaw from the 200 Hz IMU) and
stops on the FIRST of: target reached, operator key (Enter / Ctrl+C), estop
epoch change, foreign cmd_vel, stale state, odom invalid, timeout, stall
(commanded but not moving -- the 10-23 failure shape), wrong direction,
over-speed (10-23 measured 0.4-0.5 rad/s for a 0.2 command), off-axis drift,
or Tier1 zeroing (stop_reason not none). Each motion writes its own
timestamped CSV under data/run/bench/ and the session keeps a log next to
it, never overwritten (10-23: one shared CSV was overwritten run after run).

Wrong-but-plausible pitfalls this file avoids, each measured on this robot:
  * standing settles at motion_state 17 (rl_control), not 1; prone at 0.
  * a stopped stream does not "pause" the robot, it LOCKS it.
  * the yaw delta is unwrapped; a raw difference jumps 2*pi at the seam.
  * quadruped does not check the cmd_vel `src`, so "only one publisher" is
    this tool's job, not the receiver's.

Usage (on the robot, p1_motion stopped first):
  python3 scripts/diag/chassis_bench.py --rid gj-001 --lin 0.2 --lat 0.15 --ang 0.2
then `help` at the prompt.
"""
from __future__ import annotations

import argparse  # CLI
import json  # payloads
import math  # geometry
import os  # paths
import select  # Enter-to-abort while a motion runs
import sys  # stdin / exit
import threading  # 20 Hz stream + motion worker
import time  # monotonic for every interval (CLK-C1)
import uuid  # cmd_id
from dataclasses import dataclass  # guard result
from typing import Any, Dict, List, Optional, Tuple  # annotations

# ---- bench envelope ceilings (the operator chooses, the tool caps) ---------
# Far below quadruped's own limits (vx 2.0 / vy 0.5 / wz 1.2): this is the
# first motion test on a real machine with no obstacle avoidance at all.
MAX_LIN_MPS = 0.5  # forward / backward
MAX_LAT_MPS = 0.3  # strafe
MAX_ANG_RADPS = 0.5  # in-place turn
MAX_DIST_M = 2.0  # largest single translation
MAX_ANGLE_DEG = 180.0  # largest single turn

# ---- guard tuning (bench constants, not safety parameters of the stack) ----
TICK_S = 0.05  # 20 Hz, the P1 rate quadruped expects
RAMP_S = 0.5  # linear ramp-up from zero: no step into the gait
TAPER_S = 0.6  # start slowing when the rest would take less than this
TAPER_MIN = 0.3  # slowest fraction of the speed during the taper
DONE_TOL_M = 0.02  # translation target tolerance
DONE_TOL_RAD = math.radians(1.5)  # rotation target tolerance
STALL_AFTER_S = 2.0  # no stall verdict before this (gait start-up)
STALL_WIN_S = 1.5  # window the stall check looks back over
STALL_FRAC = 0.25  # moved less than this share of what was commanded
WRONG_DIR_M = 0.10  # moved the wrong way by more than this -> stop
WRONG_DIR_RAD = math.radians(10.0)  # same, for turns
OFFAXIS_M = 0.30  # drift across the commanded axis (or walking in a turn)
SPEED_WIN_S = 0.5  # window for the measured speed
OVERSPEED_K = 1.5  # measured above K * commanded + margin ...
OVERSPEED_LIN = 0.10  # ... margin for m/s
OVERSPEED_ANG = 0.10  # ... margin for rad/s
OVERSPEED_TICKS = 5  # ... for this many consecutive ticks
STATE_STALE_S = 0.5  # rt/chassis/state is 10 Hz: 5 missed = stale
TIER1_ZERO_S = 1.0  # stop_reason not none for this long while commanding
SETTLE_S = 1.0  # zero-speed logging after a motion, to see the coast

#: Standing steady state (13 S6.2 v1.3): the firmware switches to RL control.
MOTION_STATE_READY = 17

#: src of our cmd_vel; anything else on the key is a second publisher.
BENCH_SRC = "chassis_bench"

#: kind -> (axis, sign). axis picks the speed and the progress measure.
KINDS: Dict[str, Tuple[str, int]] = {
    "fwd": ("x", +1), "back": ("x", -1),  # vx
    "left": ("y", +1), "right": ("y", -1),  # vy, REP-103 left positive
    "turnl": ("yaw", +1), "turnr": ("yaw", -1),  # wz, counter-clockwise positive
}


def wrap_pi(a: float) -> float:
    """Angle into (-pi, pi]: a raw yaw difference jumps 2*pi at the seam."""
    return math.atan2(math.sin(a), math.cos(a))  # exact, no branch


def progress_of(kind: str, start: Tuple[float, float, float],
                now: Tuple[float, float, float]) -> Tuple[float, float]:
    """(progress along the commanded direction, off-axis amount).

    Translations project the odom displacement onto the START heading, so
    the measure does not follow a robot that yaws while walking; off-axis is
    the perpendicular part. Rotations measure the yaw change and report the
    translation as off-axis (an in-place turn should not walk). now[2] must
    be the CONTINUOUS yaw (YawUnwrapper): a wrapped yaw flips sign at +-pi,
    so a 180 degree turn would read as -179 and stop as "wrong direction".
    Positive progress = the commanded way, for every kind; a wrong sign
    convention therefore shows up as negative progress and is stopped by the
    wrong-direction check, not by luck.
    """
    axis, sign = KINDS[kind]  # KeyError on an unknown kind: a caller bug
    dx, dy = now[0] - start[0], now[1] - start[1]  # odom frame displacement
    c, s = math.cos(start[2]), math.sin(start[2])  # start heading
    along_x = dx * c + dy * s  # body-x of the start pose
    along_y = -dx * s + dy * c  # body-y (left) of the start pose
    if axis == "x":  # forward / backward
        return sign * along_x, abs(along_y)  # sideways part is the drift
    if axis == "y":  # strafe
        return sign * along_y, abs(along_x)  # fore-aft part is the drift
    return sign * (now[2] - start[2]), math.hypot(dx, dy)  # turn, continuous yaw


class YawUnwrapper:
    """Continuous yaw from odom's wrapped yaw, one sample per tick.

    Each step adds the wrapped DIFFERENCE to the previous sample, which is
    exact as long as the robot turns less than pi between two samples --
    at the 0.5 rad/s ceiling and 20 Hz that is 0.025 rad per tick.
    """

    def __init__(self, yaw0: float) -> None:
        """Start at the motion's first yaw."""
        self.raw = yaw0  # last wrapped sample
        self.total = yaw0  # continuous yaw

    def feed(self, yaw: float) -> float:
        """Next wrapped sample -> continuous yaw."""
        self.total += wrap_pi(yaw - self.raw)  # never jumps 2*pi
        self.raw = yaw  # for the next difference
        return self.total  # continuous


def command_vector(kind: str, scalar: float) -> Tuple[float, float, float]:
    """(vx, vy, wz) for a kind and a non-negative speed scalar."""
    axis, sign = KINDS[kind]  # which axis, which way
    v = sign * scalar  # signed speed on the one axis that moves
    if axis == "x":  # only vx
        return (v, 0.0, 0.0)  # forward / backward
    if axis == "y":  # only vy
        return (0.0, v, 0.0)  # strafe
    return (0.0, 0.0, v)  # only wz


@dataclass
class GuardStep:
    """One tick's verdict: the speed scalar to command, or why it ended."""
    scalar: float  # 0.0 whenever ended is set
    ended: Optional[str]  # None = keep going; "done" or an abort reason


class MotionGuard:
    """The per-tick decision for ONE motion, with no I/O (testable alone).

    step() is called at 20 Hz with what the console observed this tick and
    returns the speed scalar for the stream. Abort checks run BEFORE the
    target check, so a robot that over-shot by running away is reported as
    over-speed, not as a success. Once ended, every later step returns the
    same verdict with zero speed (a latch: nothing re-starts a motion).
    """

    def __init__(self, kind: str, amount: float, speed: float) -> None:
        """amount in metres or radians, speed in m/s or rad/s."""
        if kind not in KINDS:  # closed set (CLAUDE.md 3.5)
            raise ValueError("unknown motion kind %r" % (kind,))
        if not (amount > 0.0 and speed > 0.0):  # also rejects NaN
            raise ValueError("amount and speed must be positive")
        self.kind, self.amount, self.speed = kind, amount, speed  # the request
        self.rot = KINDS[kind][0] == "yaw"  # rotation or translation
        self.tol = DONE_TOL_RAD if self.rot else DONE_TOL_M  # target tolerance
        self.wrong = WRONG_DIR_RAD if self.rot else WRONG_DIR_M  # reverse limit
        self.margin = OVERSPEED_ANG if self.rot else OVERSPEED_LIN  # speed margin
        # Nominal time doubled plus three seconds: generous enough for the
        # ramp and taper, short enough that a robot pushing on something does
        # not keep pushing for long.
        self.timeout_s = amount / speed * 2.0 + 3.0  # seconds
        self.t0: Optional[float] = None  # first step time
        self.hist: List[Tuple[float, float, float]] = []  # (t, progress, cmd)
        self.over_n = 0  # consecutive over-speed ticks
        self.zero_since: Optional[float] = None  # Tier1 zeroing start
        self.ended: Optional[str] = None  # latched verdict
        self.last_progress = 0.0  # for the report

    def _end(self, why: str) -> GuardStep:
        """Latch the verdict and command zero from now on."""
        self.ended = why  # every later step returns this
        return GuardStep(0.0, why)  # zero this very tick

    def _window(self, t: float, span: float) -> Optional[Tuple[float, float, float]]:
        """(dt, dprogress, commanded travel) over the last `span` seconds.

        None while the history is shorter than the span: neither the stall
        nor the over-speed check may judge from a partial window.
        """
        old = [h for h in self.hist if h[0] <= t - span]  # samples before it
        if not old:  # history too short
            return None  # no verdict yet
        t_a, p_a, _c = old[-1]  # the sample just outside the window
        inside = [h for h in self.hist if h[0] > t_a]  # samples inside it
        cmd_int = sum(c for _t, _p, c in inside) * TICK_S  # commanded travel
        return t - t_a, self.last_progress - p_a, cmd_int  # window figures

    def step(self, t: float, progress: float, off_axis: float, *,
             operator_stop: bool, estop_changed: bool, foreign: bool,
             state_stale: bool, odom_valid: bool, stop_reason: str) -> GuardStep:
        """Decide this tick. t is monotonic seconds."""
        if self.ended is not None:  # latched
            return GuardStep(0.0, self.ended)  # same verdict, zero speed
        if self.t0 is None:  # first tick of this motion
            self.t0 = t  # elapsed counts from here
        el = t - self.t0  # elapsed
        self.last_progress = progress  # kept for the windows and the report
        # -- external stops first: they are never the robot's fault
        if operator_stop:  # Enter / Ctrl+C
            return self._end("operator_stop")
        if estop_changed:  # cmd/estop from anyone, voice included
            return self._end("estop")
        if foreign:  # second cmd_vel publisher
            return self._end("foreign_cmd_vel")
        if state_stale:  # flying blind
            return self._end("state_stale")
        if not odom_valid:  # flying blind, the other way
            return self._end("odom_invalid")
        # -- the robot misbehaving
        if progress < -self.wrong:  # going the opposite way
            return self._end("wrong_direction")
        if off_axis > OFFAXIS_M:  # sliding sideways / walking in a turn
            return self._end("off_axis_drift")
        if el > self.timeout_s:  # taking far too long
            return self._end("timeout")
        if stop_reason != "none":  # Tier1 is zeroing our command
            if self.zero_since is None:  # first tick of that
                self.zero_since = t  # start the clock
            if t - self.zero_since >= TIER1_ZERO_S:  # and it persists
                return self._end("tier1_zeroing:%s" % stop_reason)
        else:  # Tier1 passes us through
            self.zero_since = None  # reset the clock
        w = self._window(t, SPEED_WIN_S)  # measured speed window
        if w is not None and w[0] > 0.0:  # a full window
            measured = w[1] / w[0]  # along-axis speed
            recent = self.hist[-int(round(SPEED_WIN_S / TICK_S)):]  # same span
            cmd_max = max(c for _t, _p, c in recent)  # fastest we asked for
            if measured > OVERSPEED_K * cmd_max + self.margin:  # too fast
                self.over_n += 1  # count it
            else:  # within bounds
                self.over_n = 0  # consecutive, not cumulative
            if self.over_n >= OVERSPEED_TICKS:  # sustained, not one noisy tick
                return self._end("over_speed")
        if el >= STALL_AFTER_S:  # past the gait start-up
            s = self._window(t, STALL_WIN_S)  # stall window
            if s is not None and s[2] > 0.0 and s[1] < STALL_FRAC * s[2]:
                return self._end("stall")  # commanded, not moving: blocked?
        # -- the target
        rest = self.amount - progress  # what is left
        if rest <= self.tol:  # close enough
            return self._end("done")
        scalar = self.speed * min(1.0, el / RAMP_S)  # ramp up
        taper = rest / (self.speed * TAPER_S)  # remaining time vs taper span
        scalar = min(scalar, self.speed * max(TAPER_MIN, min(1.0, taper)))  # slow in
        # The first tick commands 0 (el = 0); a window that only ever saw zero
        # commands gives commanded travel 0, which the stall check skips.
        self.hist.append((t, progress, scalar))  # for the windows
        return GuardStep(scalar, None)  # keep going


def envelope(rid: str, seq: int, data: Dict[str, Any]) -> bytes:
    """11 S3.0 envelope as quadruped's read_envelope requires it.

    ts_sync is mandatory there (absent = refused); mono without boot is
    accepted and simply not used for age (mono_usable false).
    """
    body = {"v": 1, "rid": rid,  # version and the robot quadruped checks
            # WALL-CLOCK-OK(align): the S3.0 ts field, never an interval here.
            "ts": time.time(),
            "mono": time.monotonic(), "seq": seq, "src": BENCH_SRC,  # ours
            "ts_sync": False, "data": data}  # bench is not a synced source
    return json.dumps(body).encode("utf-8")  # the wire bytes


def new_cmd_id(prefix: str) -> str:
    """A fresh cmd_id: quadruped treats a repeated one as a duplicate (Q-3)."""
    return "%s-bench-%s" % (prefix, uuid.uuid4().hex[:10])  # unique per call


class ChassisView:
    """Latest rt/chassis/state and rt/chassis/basic, written by zenoh threads.

    Plain dict swaps under a lock; readers take a copy. rx times are
    monotonic so staleness is judged on our own clock.
    """

    def __init__(self) -> None:
        """Empty view: no state seen yet."""
        self._lock = threading.Lock()  # guards the fields below
        self.state: Dict[str, Any] = {}  # rt/chassis/state data
        self.state_rx = -1.0  # monotonic receive time, -1 = never
        self.basic: Dict[str, Any] = {}  # rt/chassis/basic data
        self.foreign_src: Optional[str] = None  # a second cmd_vel publisher

    def on_state(self, data: Dict[str, Any]) -> None:
        """Store one RobotState (zenoh thread)."""
        with self._lock:  # one swap
            self.state, self.state_rx = data, time.monotonic()  # value + age

    def on_basic(self, data: Dict[str, Any]) -> None:
        """Store one BasicStatus (zenoh thread); carries usage_mode."""
        with self._lock:  # one swap
            self.basic = data  # usage_mode readback lives here too

    def on_foreign(self, src: str) -> None:
        """Latch a second cmd_vel publisher until the console exits."""
        with self._lock:  # one swap
            self.foreign_src = src  # never cleared: restart the console

    def snap(self) -> Tuple[Dict[str, Any], float, Dict[str, Any], Optional[str]]:
        """(state, state_rx, basic, foreign_src) copies, consistent."""
        with self._lock:  # one consistent read
            return dict(self.state), self.state_rx, dict(self.basic), self.foreign_src


def pose_of(state: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    """(x, y, yaw) from state.odom, or None when odom is absent / invalid."""
    o = state.get("odom")  # 11 S4.1 RobotState.odom
    if not isinstance(o, dict) or o.get("valid") is not True:  # only explicit true
        return None  # no pose
    try:  # every field must be a number
        return float(o["x"]), float(o["y"]), float(o["yaw_rad"])  # the pose
    except (KeyError, TypeError, ValueError):  # missing or malformed
        return None  # no pose


class Bench:
    """The zenoh side: sessions, the 20 Hz stream, commands and logs."""

    def __init__(self, args: argparse.Namespace) -> None:
        """Open both planes, declare publishers and subscribers, open the log."""
        import zenoh  # imported here so the guard tests need no zenoh
        self.rid = args.rid  # every RT key and envelope carries it
        self.speeds = {"x": args.lin, "y": args.lat, "yaw": args.ang}  # operator's
        self.view = ChassisView()  # latest chassis readback
        self.lock = threading.Lock()  # guards target / echo_epoch / seq
        self.target = (0.0, 0.0, 0.0)  # what the stream sends
        self.echo_epoch = 0  # estop generation we echo
        self.seq = 0  # envelope seq for our RT keys
        self.stop = threading.Event()  # console exit
        stamp = time.strftime("%Y%m%d_%H%M%S")  # file names only
        self.log_dir = args.log_dir  # data/run/bench by default
        os.makedirs(self.log_dir, exist_ok=True)  # first run creates it
        log_path = os.path.join(self.log_dir, "bench_%s_session.log" % stamp)  # per session
        self.session_log = open(log_path, "a", encoding="utf-8")  # noqa: SIM115 -- whole session
        self.subs: List[Any] = []  # long-lived subscriber handles (CLAUDE.md 4.3)
        self.rt = zenoh.open(self._cfg(zenoh, args.rt))  # RT plane, 7449
        self.gen = zenoh.open(self._cfg(zenoh, args.gen))  # GEN plane, 7447
        pre = "xbrain/%s/" % self.rid  # RT keys are rid-prefixed
        self.k_cmd_vel = pre + "rt/motion/cmd_vel"  # quadruped subscribes this
        self.pub_vel = self.rt.declare_publisher(self.k_cmd_vel)  # the stream
        self.pub_mode = self.rt.declare_publisher(pre + "rt/chassis/mode")  # P1-7 stand-in
        # GEN keys are bare: chassis_relay subscribes exactly these (CR-1/CR-11).
        self.pub_ctrl = self.gen.declare_publisher("cmd/chassis/ctrl")  # CR-11
        self.pub_estop = self.gen.declare_publisher("cmd/estop")  # CR-1
        for sess, key, cb in (  # every subscription this console needs
                (self.rt, pre + "rt/chassis/state", self._on_state),  # 10 Hz state
                (self.rt, pre + "rt/chassis/basic", self._on_basic),  # usage_mode
                (self.rt, self.k_cmd_vel, self._on_vel),  # second-publisher watch
                (self.gen, "cmd/chassis/ctrl/ack", self._on_ack),  # CR-12
                (self.gen, "cmd/estop/ack", self._on_ack)):  # CR-10
            self.subs.append(sess.declare_subscriber(key, cb))  # kept alive

    @staticmethod
    def _cfg(zenoh: Any, endpoint: str) -> Any:
        """A client session to one router, no scouting."""
        c = zenoh.Config()  # empty config
        c.insert_json5("mode", '"client"')  # never a peer on the bus
        c.insert_json5("connect/endpoints", json.dumps([endpoint]))  # one router
        c.insert_json5("scouting/multicast/enabled", "false")  # no LAN discovery
        return c  # ready to open

    # -- logging --------------------------------------------------------------
    def say(self, msg: str) -> None:
        """Print and append to the session log with a wall time stamp."""
        line = "%s %s" % (time.strftime("%H:%M:%S"), msg)  # wall time for humans
        print(line, flush=True)  # the operator sees it now
        self.session_log.write(line + "\n")  # and it is kept
        self.session_log.flush()  # survives a kill

    # -- zenoh callbacks (Rust threads: parse and store only, CLAUDE.md 4.2) ---
    @staticmethod
    def _data(sample: Any) -> Optional[Dict[str, Any]]:
        """The envelope's data object, or the body itself when bare."""
        try:  # any producer may send garbage
            d = json.loads(bytes(sample.payload).decode("utf-8"))  # parse
        except (ValueError, UnicodeDecodeError):  # not JSON
            return None  # ignored
        if isinstance(d, dict) and isinstance(d.get("data"), dict):  # enveloped
            return d["data"]  # the body
        return d if isinstance(d, dict) else None  # bare body or nothing

    def _on_state(self, sample: Any) -> None:
        """rt/chassis/state -> the view."""
        d = self._data(sample)  # body
        if d is not None:  # parsed
            self.view.on_state(d)  # stored

    def _on_basic(self, sample: Any) -> None:
        """rt/chassis/basic -> the view."""
        d = self._data(sample)  # body
        if d is not None:  # parsed
            self.view.on_basic(d)  # stored

    def _on_vel(self, sample: Any) -> None:
        """Our own frames come back here too; anything else is a 2nd publisher."""
        try:  # read only the envelope's src
            src = json.loads(bytes(sample.payload).decode("utf-8")).get("src")  # who
        except (ValueError, UnicodeDecodeError, AttributeError):  # unreadable
            src = "unparsable"  # still not ours
        if src != BENCH_SRC:  # someone else
            self.view.on_foreign(str(src))  # latched

    def _on_ack(self, sample: Any) -> None:
        """Print any ctrl / estop ack: the operator reads accepted / rejected."""
        d = self._data(sample) or {}  # body or empty
        self.say("ACK %s: %s" % (sample.key_expr, json.dumps(d, ensure_ascii=False)[:300]))

    # -- the 20 Hz stream -----------------------------------------------------
    def stream(self) -> None:
        """Publish cmd_vel every TICK_S until exit, on a fixed schedule."""
        nxt = time.monotonic()  # schedule origin
        while not self.stop.is_set():  # whole console life
            with self.lock:  # one consistent read
                vx, vy, wz = self.target  # current command
                epoch = self.echo_epoch  # current generation
                self.seq += 1  # S3.0 seq per key
                seq = self.seq  # this frame's seq
            if self.view.snap()[3] is not None:  # second publisher seen
                vx = vy = wz = 0.0  # never add motion to someone else's stream
            data = {"vx": vx, "vy": vy, "wz": wz, "vz": 0.0, "v_roll": 0.0,  # all six
                    "v_pitch": 0.0, "estop_epoch": epoch}  # parse_cmd_vel requires
            self.pub_vel.put(envelope(self.rid, seq, data))  # out
            nxt += TICK_S  # fixed rate, no drift
            time.sleep(max(0.0, nxt - time.monotonic()))  # until the next slot

    def set_target(self, v: Tuple[float, float, float]) -> None:
        """What the stream sends from its next frame on."""
        with self.lock:  # one swap
            self.target = v  # (vx, vy, wz)

    # -- commands ---------------------------------------------------------------
    def ctrl(self, action: str) -> None:
        """enable / stand / prone via GEN cmd/chassis/ctrl (11 S9.3.4).

        origin "hmi": the closed set is hmi | cloud | voice | wecom and an
        operator console on site is what hmi stands for.
        """
        cid = new_cmd_id("cc")  # fresh per command
        body = {"cmd_id": cid, "action": action, "origin": "hmi"}  # S9.3.4 fields
        with self.lock:  # shares seq with the stream
            self.seq += 1  # next seq
            seq = self.seq  # this frame's seq
        # A FULL S3.0 envelope, not the bare body: chassis_relay cannot add the
        # rid to a bare body, and quadruped refuses a frame without one as
        # E_SCHEMA (measured 2026-10-26, S9.3.4 E-3).
        self.pub_ctrl.put(envelope(self.rid, seq, body))  # relay CR-11 picks it up
        self.say("sent cmd/chassis/ctrl action=%s cmd_id=%s" % (action, cid))

    def mode(self, usage: str) -> None:
        """rt/chassis/mode {cmd_id, usage_mode} (stands in for P1-7)."""
        with self.lock:  # shares seq with the stream
            self.seq += 1  # next seq
            seq = self.seq  # this frame's seq
        cid = new_cmd_id("md")  # fresh per command
        data = {"cmd_id": cid, "usage_mode": usage}  # parse_chassis_mode fields
        self.pub_mode.put(envelope(self.rid, seq, data))  # no ack key: read state
        self.say("sent rt/chassis/mode usage_mode=%s cmd_id=%s" % (usage, cid))

    def estop(self) -> None:
        """cmd/estop (11 S7.1): action stop, src_role test."""
        self.set_target((0.0, 0.0, 0.0))  # our own zero first
        cid = new_cmd_id("es")  # fresh per command
        body = {"action": "stop", "cmd_id": cid,  # S7.1 single action value
                "reason": "bench_test", "src_role": "test"}  # audit fields
        self.pub_estop.put(json.dumps(body).encode("utf-8"))  # relay CR-1
        self.say("sent cmd/estop cmd_id=%s" % cid)

    def status(self) -> str:
        """One line of what the motion preconditions depend on."""
        st, rx, basic, foreign = self.view.snap()  # current view
        age = (time.monotonic() - rx) if rx >= 0 else float("inf")  # state age
        pose = pose_of(st)  # odom pose
        pose_s = None if pose is None else "(%.3f, %.3f, %.1fdeg)" % (
            pose[0], pose[1], math.degrees(pose[2]))  # readable pose
        return ("conn=%s usage_mode=%s motion_state=%s(%s) gait=%s timeout_lock=%s "
                "locked=%s stop_reason=%s estop_epoch=%s echo=%d pose=%s "
                "state_age=%.2fs foreign_cmd_vel=%s" % (  # one line, all gates
                    st.get("conn"), basic.get("usage_mode"), st.get("motion_state"),
                    st.get("motion_state_raw"), st.get("gait"), st.get("timeout_lock"),
                    st.get("locked"), st.get("stop_reason"), st.get("estop_epoch"),
                    self.echo_epoch, pose_s, age, foreign))

    def precheck(self) -> Optional[str]:
        """Why a motion may not start now, or None.

        Each refusal names the command that fixes it, so the operator does
        not have to read Tier1 to know what to do next.
        """
        st, rx, basic, foreign = self.view.snap()  # current view
        if foreign is not None:  # p1 or another tool is publishing
            return "a second cmd_vel publisher (src=%s) is on the RT plane" % foreign
        if rx < 0 or time.monotonic() - rx > STATE_STALE_S:  # no fresh state
            return "no fresh rt/chassis/state"
        if st.get("timeout_lock") or st.get("locked"):  # Tier1 lock
            return "chassis locked (timeout_lock=%s) -- run `enable`" % st.get("timeout_lock")
        if basic.get("usage_mode") != "navigation":  # NAV-111 mode gate
            return "usage_mode=%s, not navigation -- run `nav`" % basic.get("usage_mode")
        if st.get("motion_state_raw") != MOTION_STATE_READY:  # not standing
            return "motion_state_raw=%s, not 17 (standing) -- run `stand`" % (
                st.get("motion_state_raw"),)
        if pose_of(st) is None:  # nothing to close the loop on
            return "odom invalid"
        return None  # all gates open

    def run_motion(self, kind: str, amount: float, abort: threading.Event) -> str:
        """Execute one guarded motion; returns the end reason."""
        axis = KINDS[kind][0]  # which speed applies
        guard = MotionGuard(kind, amount, self.speeds[axis])  # the decision
        st = self.view.snap()[0]  # state at the start
        start = pose_of(st)  # start pose
        if start is None:  # precheck passed, but odom may have just dropped
            self.say("motion %s refused: odom invalid at start" % kind)
            return "odom_invalid"
        yaw_c = YawUnwrapper(start[2])  # continuous yaw for the progress
        with self.lock:  # epoch shared with the stream
            new_epoch = int(st.get("estop_epoch") or 0)  # quadruped's generation
            if self.echo_epoch != new_epoch:  # an estop happened since
                # 14 S3.7 / U35: a new motion command re-arms a soft estop.
                self.say("re-arm: echo estop_epoch %d -> %d" % (self.echo_epoch, new_epoch))
            self.echo_epoch = new_epoch  # from now on we agree with quadruped
        stamp = time.strftime("%Y%m%d_%H%M%S")  # file name only
        path = os.path.join(self.log_dir, "bench_%s_%s_%g.csv" % (stamp, kind, amount))
        rows = ["t_s,cmd_vx,cmd_vy,cmd_wz,x,y,yaw,progress,off_axis,odom_vx,odom_vy,"
                "odom_wz,motion_state_raw,stop_reason,timeout_lock,estop_epoch,ended"]
        t0 = nxt = time.monotonic()  # motion clock and schedule
        why = "?"  # end reason once known
        settle_until: Optional[float] = None  # set when the guard ends
        prog, off = 0.0, 0.0  # last good progress
        while True:  # 20 Hz until the settle window is over
            now = time.monotonic()  # this tick
            st, rx, _b, foreign = self.view.snap()  # this tick's view
            pose = pose_of(st)  # this tick's pose
            if pose is not None:  # keep the last good progress otherwise
                cont = (pose[0], pose[1], yaw_c.feed(pose[2]))  # unwrapped yaw
                prog, off = progress_of(kind, start, cont)  # along / across
            if settle_until is None:  # still moving
                g = guard.step(
                    now, prog, off, operator_stop=abort.is_set(),  # Enter / Ctrl+C
                    estop_changed=st.get("estop_epoch") != self.echo_epoch,  # any estop
                    foreign=foreign is not None,  # second publisher
                    state_stale=(rx < 0 or now - rx > STATE_STALE_S),  # blind
                    odom_valid=pose is not None,  # blind, other way
                    stop_reason=str(st.get("stop_reason")))  # Tier1 verdict
                v = command_vector(kind, g.scalar)  # signed axis speed
                self.set_target(v)  # the stream sends it
                if g.ended is not None:  # done or aborted
                    why = g.ended  # remember why
                    settle_until = now + SETTLE_S  # keep logging the coast
            else:  # settling: zero already set
                v = (0.0, 0.0, 0.0)  # logged as zero
            o = st.get("odom") or {}  # raw odom for the log
            p = pose or (float("nan"),) * 3  # NaN when invalid
            rows.append(",".join(str(x) for x in (  # one CSV row per tick
                "%.3f" % (now - t0), v[0], v[1], v[2], "%.4f" % p[0], "%.4f" % p[1],
                "%.4f" % p[2], "%.4f" % prog, "%.4f" % off, o.get("vx"), o.get("vy"),
                o.get("wz"), st.get("motion_state_raw"), st.get("stop_reason"),
                st.get("timeout_lock"), st.get("estop_epoch"),
                why if settle_until is not None else "")))
            if settle_until is not None and now >= settle_until:  # coast logged
                break  # motion over
            nxt += TICK_S  # fixed rate
            time.sleep(max(0.0, nxt - time.monotonic()))  # until the next slot
        self.set_target((0.0, 0.0, 0.0))  # belt and braces
        with open(path, "w", encoding="utf-8") as fh:  # one file per motion
            fh.write("\n".join(rows) + "\n")  # never overwritten (stamped name)
        unit, scale = ("deg", math.degrees) if axis == "yaw" else ("m", float)  # display
        self.say("motion %s %g%s: ended=%s progress_at_stop=%.3f%s after_settle=%.3f%s "
                 "off_axis=%.3fm log=%s" % (  # the result line
                     kind, scale(amount), unit, why, scale(guard.last_progress), unit,
                     scale(prog), unit, off, path))
        return why  # for the caller

    def close(self) -> None:
        """Zero for a second, then stop the stream and close the sessions.

        After this the stream is gone and quadruped locks within 200 ms --
        the safe state, by design, not a fault.
        """
        self.set_target((0.0, 0.0, 0.0))  # zero
        time.sleep(1.0)  # a second of zeros on the wire
        self.stop.set()  # stream thread exits
        time.sleep(2 * TICK_S)  # let it finish its last put
        for s in self.subs:  # explicit undeclare (CLAUDE.md 4.3)
            s.undeclare()  # one at a time
        self.rt.close()  # RT session
        self.gen.close()  # GEN session
        self.session_log.close()  # flush the log


HELP = """commands:
  st                       status line (lock, usage mode, motion state, odom)
  enable                   cmd/chassis/ctrl enable (clears timeout_lock)
  stand | prone            cmd/chassis/ctrl stand / prone
  nav | normal             rt/chassis/mode usage_mode navigation / normal
  estop                    cmd/estop (next motion command re-arms)
  fwd|back|left|right <m>  translate <m> metres (max %.1f)
  turnl|turnr <deg>        turn in place <deg> degrees (max %.0f)
  speed [lin=] [lat=] [ang=]   change speeds (ceilings %.2f / %.2f / %.2f)
  quit                     zero for 1 s, stop the stream (chassis then locks)
during a motion: Enter or Ctrl+C stops it at once.""" % (
    MAX_DIST_M, MAX_ANGLE_DEG, MAX_LIN_MPS, MAX_LAT_MPS, MAX_ANG_RADPS)

#: speed keyword -> (axis, ceiling).
SPEED_KEYS = {"lin": ("x", MAX_LIN_MPS), "lat": ("y", MAX_LAT_MPS),
              "ang": ("yaw", MAX_ANG_RADPS)}  # one table for CLI and REPL


def check_speed(name: str, value: float, ceiling: float) -> float:
    """A speed the operator gave: positive and not above the ceiling."""
    if not (0.0 < value <= ceiling):  # also rejects NaN
        raise ValueError("%s=%g must be in (0, %g]" % (name, value, ceiling))
    return value  # accepted as given


def confirm(prompt: str) -> bool:
    """Ask for the literal word `yes` (anything else is no)."""
    try:  # stdin may be closed
        return input(prompt).strip().lower() == "yes"  # only yes is yes
    except EOFError:  # no terminal
        return False  # no


def do_motion(bench: Bench, kind: str, arg: str) -> None:
    """Parse, precheck, confirm the envelope, run with Enter-to-abort."""
    try:  # the amount
        amount = float(arg)  # metres or degrees
    except ValueError:  # not a number
        bench.say("not a number: %r" % arg)
        return
    rot = KINDS[kind][0] == "yaw"  # degrees or metres
    cap = MAX_ANGLE_DEG if rot else MAX_DIST_M  # bench ceiling
    if not (0.0 < amount <= cap):  # also rejects NaN
        bench.say("amount must be in (0, %g]" % cap)
        return
    why = bench.precheck()  # every gate
    if why is not None:  # one is closed
        bench.say("refused: " + why)
        return
    speed = bench.speeds[KINDS[kind][0]]  # the operator's speed for this axis
    unit, sunit = ("deg", "rad/s") if rot else ("m", "m/s")  # for the question
    if not confirm("%s %g%s at %g%s -- person on site, envelope clear? type yes: " % (
            kind, amount, unit, speed, sunit)):  # asked every time
        bench.say("not confirmed, nothing sent")
        return
    abort = threading.Event()  # Enter / Ctrl+C
    amt = math.radians(amount) if rot else amount  # the guard works in radians
    worker = threading.Thread(target=bench.run_motion, args=(kind, amt, abort),
                              daemon=True)  # the 20 Hz loop
    bench.say("motion %s %g%s start" % (kind, amount, unit))
    worker.start()  # go
    try:  # watch the keyboard while it runs
        while worker.is_alive():  # until the guard ends it
            r, _w, _x = select.select([sys.stdin], [], [], TICK_S)  # poll stdin
            if r:  # any line
                sys.stdin.readline()  # consume it
                abort.set()  # stop now
    except KeyboardInterrupt:  # Ctrl+C
        abort.set()  # stop now
    worker.join()  # wait for the zero and the report


def set_speeds(bench: Bench, rest: List[str]) -> None:
    """`speed lin=0.2 ang=0.15`: each value checked against its ceiling."""
    try:  # all or nothing per keyword
        for kv in rest:  # each assignment
            k, v = kv.split("=")  # keyword, value
            axis, ceil = SPEED_KEYS[k]  # KeyError on an unknown keyword
            bench.speeds[axis] = check_speed(k, float(v), ceil)  # checked
    except (ValueError, KeyError) as exc:  # malformed or above the ceiling
        bench.say("speed refused: %s" % exc)
    bench.say("speeds lin=%g lat=%g ang=%g" % (
        bench.speeds["x"], bench.speeds["y"], bench.speeds["yaw"]))  # current values


def repl(bench: Bench) -> None:
    """The command loop; returns on quit / EOF / Ctrl+C at the prompt."""
    while True:  # one command per line
        try:  # the prompt
            line = input("bench> ").strip()  # one command
        except (EOFError, KeyboardInterrupt):  # leave
            return
        if not line:  # empty line
            continue  # nothing to do
        cmd, *rest = line.split()  # command and arguments
        if cmd == "quit":  # leave
            return
        if cmd == "help":  # the command list
            print(HELP)
        elif cmd == "st":  # status
            bench.say(bench.status())
        elif cmd in ("enable", "stand", "prone"):  # cmd/chassis/ctrl actions
            bench.ctrl(cmd)
        elif cmd in ("nav", "normal"):  # usage mode
            bench.mode("navigation" if cmd == "nav" else "normal")
        elif cmd == "estop":  # soft estop
            bench.estop()
        elif cmd in KINDS and len(rest) == 1:  # one of the six motions
            do_motion(bench, cmd, rest[0])
        elif cmd == "speed":  # change speeds
            set_speeds(bench, rest)
        else:  # anything else
            print("unknown command; `help`")


def main() -> int:
    """Parse, check speeds, refuse next to p1, stream, run the prompt."""
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ap = argparse.ArgumentParser(description="M20S bench console (plan B)")
    ap.add_argument("--rid", required=True, help="robot id, e.g. gj-001")
    ap.add_argument("--lin", type=float, required=True, help="fwd/back speed m/s")
    ap.add_argument("--lat", type=float, required=True, help="left/right speed m/s")
    ap.add_argument("--ang", type=float, required=True, help="turn speed rad/s")
    ap.add_argument("--rt", default="tcp/127.0.0.1:7449", help="RT router")
    ap.add_argument("--gen", default="tcp/127.0.0.1:7447", help="GEN router")
    ap.add_argument("--log-dir", default=os.path.join(root, "data", "run", "bench"),
                    help="CSV and session log directory")  # gitignored data/
    args = ap.parse_args()  # exits on a usage error
    try:  # the operator's speeds against the ceilings
        for name in ("lin", "lat", "ang"):  # all three
            check_speed(name, getattr(args, name), SPEED_KEYS[name][1])  # or raise
    except ValueError as exc:  # above a ceiling or not positive
        print("refused: %s" % exc, file=sys.stderr)
        return 2
    bench = Bench(args)  # sessions up, nothing published yet
    time.sleep(1.5)  # listen first: 30 cmd_vel periods of p1, if it runs
    st, rx, _b, foreign = bench.view.snap()  # what we heard
    if foreign is not None or rx < 0:  # p1 still up, or quadruped down
        bench.say("refused: %s" % (
            "cmd_vel from src=%s is already on the RT plane -- stop p1_motion first"
            % foreign if foreign is not None else
            "no rt/chassis/state -- is quadruped running with rid %s?" % bench.rid))
        bench.close()  # stop is set inside; the stream never started
        return 3
    bench.echo_epoch = int(st.get("estop_epoch") or 0)  # adopt the current generation
    threading.Thread(target=bench.stream, daemon=True).start()  # 20 Hz from now on
    bench.say("bench up: rid=%s speeds lin=%g lat=%g ang=%g; streaming zero "
              "cmd_vel at 20 Hz" % (bench.rid, args.lin, args.lat, args.ang))
    bench.say(bench.status())  # where the gates stand
    print(HELP)  # the command list
    repl(bench)  # until quit
    bench.say("quit: zeroing, then stopping the stream (chassis will timeout_lock)")
    bench.close()  # zero, stop, close
    return 0  # clean


if __name__ == "__main__":
    sys.exit(main())  # exit code to the shell
