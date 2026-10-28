"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: basic_motion_test.py
Brief: TEMPORARY basic-motion test executor: relmove closed-loop on chassis odom, no RNS / gates

Description:
What problem this solves
------------------------
User ruling 2026-10-26: for the basic-motion test, commx commands go through
the STANDARD processes (p4 -> p2 -> p1 -> quadruped) and nothing may block the
six basic motions (forward, back, turn left, turn right, strafe left, strafe
right); the standard flow is restored after the test. On the day of the ruling
the formal chain could not move the robot at all indoors: the mission host
refuses a relmove without an RTK fix and heading (12 S4.5.2), RNS needs
perception, and p2's G-4 / G-5 / G-6 gates hold motion on clock, health and
heading. Without this module the operator's "前进1米" is accepted by p4 and p2
and then silently dropped by p1 -- the robot does not move and nothing says
why.

What it does
------------
Enabled only by XBRAIN_BASIC_MOTION_TEST=1, which scripts/start_basic_motion.sh
sets (and nothing else does; a production unit never sets it). With it,
nav_wiring hands each cmd/motion/relative_move to this executor instead of the
mission host / RNS:
  * one axis at a time: dx_m, dy_m or dyaw_rad (the first non-zero of yaw, y,
    x in that order -- a relmove carrying two axes runs only one of them);
  * closed-loop on quadruped's integrated odometry (rt/chassis/state odom
    x / y / yaw_rad). Indoor ruling: no RTK needed, the chassis odometry is
    the only pose source;
  * speeds come from XBRAIN_BASIC_MOTION_LIN / _LAT / _ANG (the start script;
    raising them is the operator's decision, never this module's);
  * it stops on: target reached, soft estop, stale state / invalid odom,
    timeout, stall, wrong direction, over-speed, off-axis drift -- the same
    guard as scripts/diag/chassis_bench.py MotionGuard, which was exercised on
    the real chassis first.

What it does NOT do (boundaries)
--------------------------------
  * No obstacle avoidance, no fence, no speed gate. The operator is the safety
    function: a person present and the area clear before every motion (the
    10-23 incident, a robot turning unattended, is why).
  * No face-heading (target_yaw_rad) relmoves: they are refused, because the
    absolute heading needs the RTK heading this mode exists to do without.
  * It never truncates a command to fit its limits. A command larger than
    MAX_DIST_M / MAX_ANGLE_RAD is refused whole (see start()).
  * It does not switch the chassis into navigation. nav_wiring's auto-switch
    does that after stand-up; this module only WAITS for it (PendingStart).

Pitfalls measured on the real chassis 2026-10-26 (each one cost a test round)
---------------------------------------------------------------------------
  * A relmove arriving before the chassis is in navigation: Tier 1 outputs
    zero unless usage_mode is navigation (NAV-111), so the executor commanded
    speed, saw no progress and declared a stall. The first turn after stand-up
    always hit it. Hence PendingStart: hold the relmove until navigation.
  * Turning in place below ~0.3 rad/s: the chassis twists its body but never
    steps, so the odometry yaw barely moves and the guard declares a stall.
    The end-of-motion taper used to slow every turn into that band. Hence
    TAPER_MIN = 1.0 (no taper); the user approved 0.3 rad/s as the turn speed.
  * "右转360度" turned 180 degrees: the amount used to be clamped to the cap. A
    motion that differs from the command is worse than none, so it is refused.

How it sits in nav_wiring (the only caller)
-------------------------------------------
NavRuntime builds one BasicMotionTest at construction when enabled() is true,
and otherwise None -- every call site checks `self._bmt is not None`, so with
the switch off this module is never entered and p1 behaves exactly as the
standard build. Per 20 Hz tick, in this order:
  1. every relmove drained from cmd/motion/relative_move this tick goes to
     queue(); none of them reaches the mission host (that is the bypass);
  2. admit() starts the pending one if the chassis is in navigation;
  3. if a motion is active: soft estop -> abort("estop") and zero; otherwise
     step() gives (vx, vy, wz) and the CtrlLoop state is forced ACTIVE so its
     own readiness vetoes do not zero the output. The CtrlLoop still applies
     the estop veto itself, so an estop is honoured twice, never zero times;
  4. nav_wiring's auto-navigation request is suppressed while a motion is
     active (a mode switch mid-motion would stop the chassis), but NOT while
     a relmove is only pending -- the pending one is waiting for exactly that
     switch.
The (vx, vy, wz) go out through the single CtrlLoop sink (12 S2.2 step 10)
like any other p1 output, so quadruped's Tier 1 still applies its own gates
(timeout_lock until enable, usage_mode navigation, motion_state).

Coordinate conventions
----------------------
quadruped's odometry is an integrated planar pose in its own odom frame:
x / y in metres, yaw_rad counter-clockwise positive, as rt/chassis/state
carries them. The relmove body is in the BODY frame at the moment the motion
starts: dx_m forward, dy_m left, dyaw_rad counter-clockwise (turn left > 0).
step() therefore rotates the odometry displacement by -yaw_start:
    ax =  dx * cos(yaw0) + dy * sin(yaw0)     (along the start heading)
    ay = -dx * sin(yaw0) + dy * cos(yaw0)     (to the left of it)
and the commanded velocities are body-frame too (vx forward, vy left, wz
counter-clockwise), which is what rt/motion/cmd_vel carries.

How to read the log (one WARNING line per start and per end)
------------------------------------------------------------
"BASIC-MOTION-TEST start cmd_id=.. axis=.. amount=.. speed=.." then
"BASIC-MOTION-TEST end cmd_id=.. ended=<verdict> progress=..". Verdicts:
  done                         arrived within DONE_TOL_*; normal end.
  stall                        commanded but < STALL_FRAC of it happened over
                               STALL_WIN_S: something blocks the robot, or
                               the chassis is not stepping (check usage_mode
                               and the turn speed, see the pitfalls above).
  wrong_direction              moved the opposite way: a sign error. Stop
                               testing and report it; do not retry.
  off_axis_drift               wandered > OFFAXIS_M from the commanded line.
  over_speed                   faster than commanded for OVERSPEED_TICKS ticks:
                               odometry glitch or the robot is being pushed.
  timeout                      did not arrive in twice the nominal time + 3 s.
  state_stale_or_odom_invalid  rt/chassis/state stopped or odom.valid false:
                               the loop has nothing to close on.
  estop / preempted            soft estop, or a newer relmove arrived.
"relmove refused: <reason>" (start() refused; nothing moved) and "relmove
dropped: chassis not in navigation after 30 s" (admit() gave up) are the two
lines that mean the command never became a motion.

Restoring the standard flow (after the test; user ruling 2026-10-26)
-------------------------------------------------------------------
  1. scripts/start_basic_motion.sh: remove the XBRAIN_BASIC_MOTION_* exports;
  2. delete this module, its test file and the `_bmt` branches in nav_wiring;
  3. p2 motion_intent_wiring: remove the _basic_motion_test() skips of G-4 /
     G-5 / G-6;
  4. keep nav_wiring's auto-navigation (it is not part of the test mode).
Commit 8204818 introduced the mode; its message lists the same pieces.

Tests
-----
tests/p1_motion/runtime/test_basic_motion_test.py drives the executor with a
simulated odometry and pins: arrival, the refusal of an over-limit command
(not truncation), the wait for navigation and its 30 s drop, the absence of
a taper, stall, wrong direction and the stale-state stop.
"""
from __future__ import annotations

import logging
import math
import os
from typing import Any, Dict, List, Optional, Tuple

_logger = logging.getLogger("xbrain.p1.basic_motion_test")

# The constants below are the SAME values as scripts/diag/chassis_bench.py's
# MotionGuard, which ran the six motions on the real chassis first; keeping
# them equal means a bench result and a test-mode result are comparable. They
# are test-mode constants, not safety parameters: the safety parameters of the
# standard flow (common.spec / common.safety, CLAUDE.md 3.1) are not used here
# at all, which is exactly what "every gate off" in the user ruling means.

# Control period. nav_wiring ticks at 20 Hz; the command integral in the stall
# window multiplies by this, so it must match the caller's real period.
TICK_S = 0.05
# Ramp-up time: the commanded speed rises linearly from 0 to the set speed over
# this long, so a standing robot is not asked for a step change of velocity.
RAMP_S = 0.5
# Taper horizon: with a taper, the speed would shrink once the remaining
# distance is less than speed * TAPER_S.
TAPER_S = 0.6
# Floor of the taper, as a fraction of the set speed. 1.0 means NO taper at all
# (the min/max below then always yield the full speed): below ~0.3 rad/s the
# chassis only twists, never steps (measured 2026-10-26), so slowing the end of
# a turn produced a stall instead of a softer stop. The tolerance below absorbs
# the overshoot of stopping from full speed.
TAPER_MIN = 1.0
# Done tolerances: progress within this of the target counts as arrived.
DONE_TOL_M = 0.02
DONE_TOL_RAD = math.radians(1.5)
# Stall: after STALL_AFTER_S of motion, if over the last STALL_WIN_S the robot
# moved less than STALL_FRAC of what was commanded, it is pushing against
# something (or not stepping) -- stop rather than keep pushing.
STALL_AFTER_S = 2.0
STALL_WIN_S = 1.5
STALL_FRAC = 0.25
# Wrong direction: progress more negative than this means the robot moves the
# OPPOSITE way to the command (a sign error somewhere) -- stop at once.
WRONG_DIR_M = 0.10
WRONG_DIR_RAD = math.radians(10.0)
# Off-axis drift: a straight motion that wandered this far sideways (or a turn
# whose centre moved this far) is not the motion that was commanded.
OFFAXIS_M = 0.30
# Over-speed: measured speed over SPEED_WIN_S above OVERSPEED_K times the
# highest recent command plus a margin, for OVERSPEED_TICKS ticks in a row.
# The consecutive count stops one noisy odometry sample from aborting.
SPEED_WIN_S = 0.5
OVERSPEED_K = 1.5
OVERSPEED_LIN = 0.10
OVERSPEED_ANG = 0.10
OVERSPEED_TICKS = 5
# The chassis state older than this is not a pose to close a loop on.
STATE_STALE_S = 0.5
# Largest single motion this test mode executes. p2 itself allows up to 20 m /
# 720 deg; these are tighter because there is no obstacle avoidance here.
MAX_DIST_M = 3.0
MAX_ANGLE_RAD = math.pi
# How long a relmove may wait for the chassis to reach navigation before it is
# dropped. Stand-up plus the auto-switch takes a few seconds; half a minute
# means something else is wrong, and a motion starting that late would surprise
# the operator who sent it.
PENDING_MAX_S = 30.0


def enabled() -> bool:
    """The test switch (set only by scripts/start_basic_motion.sh)."""
    return os.environ.get("XBRAIN_BASIC_MOTION_TEST") == "1"


def wrap_pi(a: float) -> float:
    """Angle wrapped into (-pi, pi], used to unwrap the odometry yaw per tick."""
    return math.atan2(math.sin(a), math.cos(a))


class _Guard:
    """Per-tick decision for one motion (see chassis_bench.MotionGuard).

    step() is called once per control tick with the progress along the
    commanded axis (signed so that positive means "towards the target") and
    the off-axis deviation, and returns (speed scalar, None) to keep going or
    (0.0, verdict) to stop. The abort checks run BEFORE the target check, so a
    robot that reached the target by running away is reported as the fault,
    not as a success.
    """

    def __init__(self, axis: str, sign: int, amount: float, speed: float) -> None:
        """axis is "x", "y" or "yaw"; amount > 0 in metres or radians; speed in
        m/s or rad/s; sign is the direction of the command (+1 / -1)."""
        self.axis, self.sign, self.amount, self.speed = axis, sign, amount, speed
        rot = axis == "yaw"
        # Rotation and translation use their own tolerances and margins.
        self.tol = DONE_TOL_RAD if rot else DONE_TOL_M  # arrival tolerance
        self.wrong = WRONG_DIR_RAD if rot else WRONG_DIR_M  # reverse-motion limit
        self.margin = OVERSPEED_ANG if rot else OVERSPEED_LIN  # over-speed absolute margin
        # Nominal time doubled plus three seconds: generous enough for the ramp,
        # short enough that a robot pushing on something does not keep pushing.
        self.timeout_s = amount / speed * 2.0 + 3.0  # seconds
        self.t0: Optional[float] = None  # time of the first step
        # (t, progress, commanded scalar) per tick that kept going.
        self.hist: List[Tuple[float, float, float]] = []  # newest last
        self.over_n = 0  # consecutive over-speed ticks
        self.last_progress = 0.0  # for the end-of-motion log line

    def _window(self, t: float, span: float):
        """(elapsed, progress made, command integral) over the last `span`
        seconds, measured from the newest history entry at least `span` old;
        None while the history is shorter than that.

        The anchor is a REAL past sample (t_a, p_a), not t - span, so the
        elapsed time is exact even when ticks jitter. The command integral is
        the distance the commands asked for after the anchor:
            cmd_int = sum(c_i for t_i > t_a) * TICK_S
        so "moved" (progress - p_a) and "asked" (cmd_int) are in the same unit
        (metres or radians) and can be compared directly by the stall check."""
        old = [h for h in self.hist if h[0] <= t - span]  # samples at least span old
        if not old:  # history shorter than the window
            return None
        t_a, p_a, _c = old[-1]  # the newest of them is the anchor
        # The distance the commands asked for since t_a (speed * period).
        cmd_int = sum(c for tt, _p, c in self.hist if tt > t_a) * TICK_S
        return t - t_a, self.last_progress - p_a, cmd_int  # elapsed, moved, asked

    def step(self, t: float, progress: float, off_axis: float) -> Tuple[float, Optional[str]]:
        """One tick: the speed scalar (>= 0) and None, or 0.0 and the verdict.

        Order of evaluation (the first that applies wins):
            progress < -wrong            -> wrong_direction
            off_axis > OFFAXIS_M         -> off_axis_drift
            elapsed > timeout_s          -> timeout
            measured speed over SPEED_WIN_S > K * max recent cmd + margin
              for OVERSPEED_TICKS ticks  -> over_speed
            elapsed >= STALL_AFTER_S and moved < STALL_FRAC * asked
              over STALL_WIN_S           -> stall
            remaining <= tol             -> done
            otherwise                    -> keep going at
              speed * min(1, elapsed / RAMP_S)      (ramp-up; no taper)
        The stall check waits STALL_AFTER_S because the ramp and the chassis'
        own start-up latency make the first second look like a stall. Only
        ticks that keep going are appended to the history, so a verdict tick
        never contributes a command the robot was not given."""
        if self.t0 is None:  # first tick of this motion
            self.t0 = t  # elapsed time is measured from here
        el = t - self.t0  # seconds since the first tick
        self.last_progress = progress  # read by _window and the end log
        # Abort checks first (see the class docstring for why the order matters).
        if progress < -self.wrong:  # moving the opposite way
            return 0.0, "wrong_direction"
        if off_axis > OFFAXIS_M:  # left the commanded line
            return 0.0, "off_axis_drift"
        if el > self.timeout_s:  # far longer than nominal
            return 0.0, "timeout"
        w = self._window(t, SPEED_WIN_S)  # measured speed window
        if w is not None and w[0] > 0.0:  # a window exists and has width
            # Compared with the HIGHEST recent command, not the current one,
            # so the ramp's rising command cannot itself look like over-speed.
            recent = self.hist[-int(round(SPEED_WIN_S / TICK_S)):]  # commands inside the window
            cmd_max = max(c for _t, _p, c in recent)  # highest command in it
            self.over_n = self.over_n + 1 if w[1] / w[0] > OVERSPEED_K * cmd_max + self.margin else 0
            if self.over_n >= OVERSPEED_TICKS:  # sustained, not one noisy sample
                return 0.0, "over_speed"
        if el >= STALL_AFTER_S:  # past the start-up grace period
            s = self._window(t, STALL_WIN_S)  # moved vs asked over the stall window
            # Commanded something (s[2] > 0) but moved less than STALL_FRAC of it.
            if s is not None and s[2] > 0.0 and s[1] < STALL_FRAC * s[2]:
                return 0.0, "stall"
        rest = self.amount - progress  # remaining distance or angle
        if rest <= self.tol:  # arrived
            return 0.0, "done"
        # Ramp-up, then the (disabled, TAPER_MIN = 1.0) taper towards the end.
        scalar = self.speed * min(1.0, el / RAMP_S)  # ramp-up
        scalar = min(scalar, self.speed * max(TAPER_MIN, min(1.0, rest / (self.speed * TAPER_S))))
        self.hist.append((t, progress, scalar))  # only ticks that keep going
        return scalar, None  # keep going


def _pose(state: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    """(x, y, yaw) from the rt/chassis/state body, or None when the odometry is
    absent, not valid, or malformed -- never a guessed pose."""
    o = state.get("odom")  # the odom block of rt/chassis/state
    if not isinstance(o, dict) or o.get("valid") is not True:  # absent or untrusted
        return None
    try:  # malformed numbers mean no pose
        return float(o["x"]), float(o["y"]), float(o["yaw_rad"])  # m, m, rad
    except (KeyError, TypeError, ValueError):  # missing or non-numeric field
        return None


class BasicMotionTest:
    """Holds at most one motion and at most one pending relmove.

    nav_wiring calls, once per control tick and in this order:
      queue(body, now_s)            for every relmove received this tick;
      admit(now_s, state, age_s)    starts the pending relmove once the chassis
                                    is in navigation (or drops it after
                                    PENDING_MAX_S);
      step(now_s, state, age_s)     the (vx, vy, wz) for this tick.
    A newer relmove replaces both the running motion and an older pending one:
    the operator's latest command is the one that counts.
    """

    def __init__(self) -> None:
        """Speeds are read from the environment the start script sets. A missing
        variable raises KeyError at construction, which is deliberate: a test
        mode with an invented speed is worse than p1 refusing to start."""
        self.speeds = {"x": float(os.environ["XBRAIN_BASIC_MOTION_LIN"]),  # m/s forward
                       "y": float(os.environ["XBRAIN_BASIC_MOTION_LAT"]),  # m/s lateral
                       "yaw": float(os.environ["XBRAIN_BASIC_MOTION_ANG"])}  # rad/s
        self._g: Optional[_Guard] = None  # the running motion, if any
        self._start: Optional[Tuple[float, float, float]] = None  # pose at start
        # Unwrapped yaw: raw is the last odometry yaw, total accumulates the
        # wrapped deltas so a turn through +-pi does not jump by 2 pi.
        self._yaw_raw = 0.0  # last raw odometry yaw
        self._yaw_total = 0.0  # unwrapped yaw
        self.cmd_id: Optional[str] = None  # cmd_id of the running motion
        self.last_result: Optional[str] = None  # verdict of the last motion
        # (body, time queued) of the relmove waiting for navigation.
        self._pending: Optional[Tuple[Any, float]] = None  # None when nothing waits

    @property
    def active(self) -> bool:
        """True while a motion is running."""
        return self._g is not None

    @property
    def pending(self) -> bool:
        """True while a relmove waits for the chassis to reach navigation."""
        return self._pending is not None

    def queue(self, body: Any, now_s: float) -> None:
        """Accept a relmove; it starts in admit() once the chassis is in
        navigation. A running motion is aborted ("preempted") here, at once:
        the operator sent something newer, and the old motion must not keep
        going while the new one waits."""
        if self.active:  # something is running
            self.abort("preempted")  # stop it now, not after the wait
        self._pending = (body, now_s)  # newer replaces older

    def drop_pending(self, why: str) -> None:
        """Forget a relmove still waiting for navigation (99 U99 mode hold).

        Without this a "go forward" sent just before a broadcast started
        would sit in the slot and start by itself once the chassis reached
        navigation -- during the broadcast, which the hold exists to prevent.
        """
        if self._pending is not None:  # something was waiting
            _logger.warning("BASIC-MOTION-TEST relmove dropped: %s", why)
        self._pending = None  # the slot is empty either way

    def admit(self, now_s: float, state: Dict[str, Any], state_age_s: float) -> Optional[str]:
        """Start the pending relmove if the chassis is in navigation.

        Returns None when nothing was decided this tick (nothing pending, or
        still waiting), "started" when the motion began, or the refusal /
        drop reason. Reported for the caller's log and for the tests.

        *** Why it waits (2026-10-26, real chassis): Tier 1 outputs zero unless
        usage_mode is navigation (NAV-111). A relmove that arrived during
        stand-up was started at once, commanded speed into a zero output, saw
        no progress and ended as "stall" -- the first turn after every stand-up
        failed that way.
        """
        if self._pending is None:  # nothing waits
            return None
        body, t_queued = self._pending  # what waits, since when
        if state.get("usage_mode") == "navigation":  # Tier 1 passes motion now
            self._pending = None
            why = self.start(body, state, state_age_s)  # may still refuse it
            if why is not None:
                _logger.warning("BASIC-MOTION-TEST relmove refused: %s", why)
                return why
            return "started"
        if now_s - t_queued > PENDING_MAX_S:  # waited too long
            self._pending = None
            _logger.warning("BASIC-MOTION-TEST relmove dropped: chassis not in "
                            "navigation after %.0f s (usage_mode=%s)",
                            PENDING_MAX_S, state.get("usage_mode"))
            return "dropped_not_navigation"
        return None

    def start(self, body: Any, state: Dict[str, Any], state_age_s: float) -> Optional[str]:
        """Begin a relmove now; returns None or the reason it was refused.

        Refusal reasons (nothing moves, the caller logs the reason):
            bad_body                          not a dict, or a non-numeric axis
            face_heading_unsupported_in_test  target_yaw_rad present
            no_odom                           odometry absent / invalid / stale
            zero_move                         all three axes zero
            exceeds_test_limit(a > cap)       larger than MAX_DIST_M /
                                              MAX_ANGLE_RAD
        On success the start pose is latched; every later step() measures
        progress from it, in its frame (see "Coordinate conventions")."""
        if not isinstance(body, dict):  # not a relmove body at all
            return "bad_body"
        if body.get("target_yaw_rad") is not None:
            # Face-heading needs an absolute heading (RTK); not in this mode.
            return "face_heading_unsupported_in_test"
        try:
            dx, dy, dyaw = (float(body.get(k) or 0.0) for k in ("dx_m", "dy_m", "dyaw_rad"))
        except (TypeError, ValueError):
            return "bad_body"
        pose = _pose(state)
        if pose is None or state_age_s > STATE_STALE_S:
            return "no_odom"
        # One axis only: yaw first, then lateral, then forward.
        if abs(dyaw) > 1e-6:  # turn
            axis, val, cap = "yaw", dyaw, MAX_ANGLE_RAD
        elif abs(dy) > 1e-6:  # strafe
            axis, val, cap = "y", dy, MAX_DIST_M
        elif abs(dx) > 1e-6:  # forward / back
            axis, val, cap = "x", dx, MAX_DIST_M
        else:
            return "zero_move"
        amount = abs(val)  # size; the sign goes to _Guard
        if amount > cap + 1e-6:
            # Refused, never truncated (2026-10-26): "右转360度" used to turn
            # 180 degrees -- a motion that differs from the command is worse
            # than no motion. p2 allows up to 20 m / 720 deg; this test mode
            # only executes <= MAX_DIST_M / MAX_ANGLE_RAD.
            return "exceeds_test_limit(%.3f > %.3f)" % (amount, cap)
        self._g = _Guard(axis, 1 if val > 0 else -1, amount, self.speeds[axis])
        self._start = pose  # progress is measured from here
        self._yaw_raw = self._yaw_total = pose[2]  # unwrap from the start yaw
        self.cmd_id = body.get("cmd_id")  # for the log lines
        _logger.warning("BASIC-MOTION-TEST start cmd_id=%s axis=%s amount=%.3f speed=%.2f",
                        self.cmd_id, axis, amount * (1 if val > 0 else -1), self.speeds[axis])
        return None

    def abort(self, why: str) -> None:
        """End the running motion (if any) with verdict `why`; logged once."""
        if self._g is not None:  # log only a real end
            _logger.warning("BASIC-MOTION-TEST end cmd_id=%s ended=%s progress=%.3f",
                            self.cmd_id, why, self._g.last_progress)
            self.last_result = why  # for the tests and the operator
        self._g = None  # idle

    def step(self, now_s: float, state: Dict[str, Any], state_age_s: float) -> Tuple[float, float, float]:
        """(vx, vy, wz) for this tick; zero and ended when the guard stops.

        Per tick: read the pose (stop if it is stale or invalid), unwrap the
        yaw, express the displacement in the start frame, turn it into
        (progress, off-axis) for the commanded axis, ask the guard, and map
        the signed scalar back onto the one axis being driven. The two other
        axes are always exactly 0.0 -- a single-axis test must not leak a
        component into another axis."""
        g = self._g  # the running motion, or None
        if g is None:  # idle: hold zero
            return 0.0, 0.0, 0.0
        pose = _pose(state)
        if pose is None or state_age_s > STATE_STALE_S:
            # No pose to close the loop on: stop, do not dead-reckon.
            self.abort("state_stale_or_odom_invalid")
            return 0.0, 0.0, 0.0
        self._yaw_total += wrap_pi(pose[2] - self._yaw_raw)  # wrapped delta: no 2 pi jump
        self._yaw_raw = pose[2]  # next tick's reference
        # Displacement since the start, rotated into the START frame, so
        # "forward 1 m" means along the heading the robot had when it began.
        sx, sy, syaw = self._start  # start pose
        dx, dy = pose[0] - sx, pose[1] - sy  # odom-frame displacement
        c, s = math.cos(syaw), math.sin(syaw)  # rotation by -yaw0
        ax, ay = dx * c + dy * s, -dx * s + dy * c  # start-frame displacement
        if g.axis == "x":
            prog, off = g.sign * ax, abs(ay)  # along; lateral is drift
        elif g.axis == "y":
            prog, off = g.sign * ay, abs(ax)  # left; forward is drift
        else:
            # A turn in place: progress is the unwrapped yaw change, and any
            # translation of the body centre counts as off-axis drift.
            prog, off = g.sign * (self._yaw_total - syaw), math.hypot(dx, dy)
        scalar, ended = g.step(now_s, prog, off)  # the guard decides
        if ended is not None:
            self.abort(ended)
            return 0.0, 0.0, 0.0
        v = g.sign * scalar  # signed speed on the driven axis
        if g.axis == "x":
            return v, 0.0, 0.0  # forward / back
        if g.axis == "y":
            return 0.0, v, 0.0  # strafe
        return 0.0, 0.0, v  # turn
