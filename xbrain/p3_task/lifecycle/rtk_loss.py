"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rtk_loss.py
Brief: 11 S3.3.4 positioning loss -- judge it from state/pose, suspend the running task, build the rtk events

Description:
The problem. When the fix degrades to single / no_fix the speed gate zeroes
the robot (P1 vetoes on rtk, 11 S3.2.1), but until 2026-11-06 nothing else
happened: rtk_driver computed its rtk_lost transition and never published it,
no process suspended the task, nobody spoke, no light changed. The robot
stood still with the task "running", silent, and when the fix came back it
drove on by itself -- the RL-7 "不自动重启" the contract calls the most
important negative rule of the section. Measured on the robot with a GNSS
injector (99 U142).

Who does what (user ruling 2026-11-06, 99 U142): P3 judges the loss from
state/pose, suspends the running task (11 S3.3.4 RL-2, passive /
rtk_degraded -- the suspension withdraws P1's mission through the route
clear of 99 U141, so recovery cannot restart it), holds dispatch and refuses
new submits while it lasts, and publishes event/alarm/rtk (RL-4) and, on
recovery, event/info/rtk. P4 speaks both (RL-3 voice), P2 drives the
red/blue lights (RL-3 light).

What counts as lost (same ruling, and 11 S3.3.3's fix_lost trigger): the fix
is single or no_fix -- or absent (fix_type None: P1 has no fix to report,
rtk_driver silent). rtk_fixed / rtk_float / dgps are NOT lost: they run (the
last two capped at 0.5 m/s by P1). The condition must hold lost_sustain_s
before it is declared and the good condition recover_sustain_s before
recovery (11 S3.3.3: 1.0 s / 2.0 s, degrade fast, recover slow).

What it does NOT do:
  * judge the heading. L2-blind is not a fault (11 S3.3.1) and a heading L3
    only arises from a lost fix in this build; the fix is the one fact.
  * resume anything. Recovery only reports; the suspended task waits for a
    human "继续" (RL-7, 11 S3.3.3 note "恢复不等于自动复跑").
  * read a clock. now_s is passed in (monotonic, CLK-C1).

Traps:
  * Judging a missing state/pose as lost. No pose at all means P1 is not
    publishing -- this module cannot tell anything then, and suspending on a
    P1 restart would be a false alarm. Only a pose whose fix_type says
    single / no_fix / None is a loss.
  * Re-arming the timers on every frame. The sustain is measured from the
    FIRST bad frame of a run; resetting it per frame never fires.
"""

from __future__ import annotations  # postponed annotations

from typing import Any, Dict, Mapping, Optional, Tuple  # plain value types

from xbrain.p3_task.state.machine import apply_transition, validate_suspend_fields  # the graph

#: The fix types that are a positioning failure (99 U142 / 11 S3.3.3 fix_lost).
LOST_FIXES = frozenset({"single", "no_fix"})
#: 11 S4.4 suspend_reason for RL-2 (closed set, common/enums/sets.yaml).
RTK_SUSPEND_KIND = "passive"  # waits for a human (RL-7)
RTK_SUSPEND_REASON = "rtk_degraded"
#: 11 S3.3.4 detail.reason for a fix-driven loss.
REASON_FIX_LOST = "fix_lost"


class RtkLossConfigError(ValueError):
    """A sustain time missing or not positive (CLAUDE.md 3.1: refuse)."""


def fix_is_lost(pose: Mapping[str, Any]) -> bool:
    """True when this state/pose body reports no usable fix."""
    fix = pose.get("fix_type")  # 11 S3.2.1 closed set, or None
    return fix is None or fix in LOST_FIXES  # None: P1 has no fix to report


def _num(v: Any) -> Optional[float]:
    """A real number or None (bools and strings are not positions)."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None  # absent / malformed
    return float(v)  # usable


class RtkLossMonitor:
    """Debounced lost / recovered judgement over successive state/pose bodies.

    update() returns "lost" once when a loss is declared, "recovered" once
    when it ends, else None. The last trustworthy position and heading are
    kept for the alarm (11 S3.3.4: "人工找回机器人的唯一线索")."""

    def __init__(self, lost_sustain_s: Any, recover_sustain_s: Any) -> None:
        for name, v in (("rtk_loss.lost_sustain_s", lost_sustain_s),
                        ("rtk_loss.recover_sustain_s", recover_sustain_s)):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0:
                raise RtkLossConfigError("%s must be a positive number, got %r" % (name, v))
        self._lost_s = float(lost_sustain_s)  # 11 S3.3.3 fix_lost hold
        self._recover_s = float(recover_sustain_s)  # 11 S3.3.3 recovery hold
        self.lost = False  # the declared state (dispatch holds while True)
        self._bad_since: Optional[float] = None  # first bad frame of this run
        self._good_since: Optional[float] = None  # first good frame while lost
        self.lost_since: Optional[float] = None  # when the loss was declared
        self.last_valid_pos: Optional[Dict[str, float]] = None  # last usable lat / lon
        self.last_valid_mono: Optional[float] = None  # when that was
        self.last_valid_heading_rad: Optional[float] = None  # last valid heading
        self.prev_level: Optional[int] = None  # heading level before the loss

    def update(self, pose: Optional[Mapping[str, Any]], now_s: float) -> Optional[str]:
        """Feed one state/pose body (None: none received -- no judgement)."""
        if not isinstance(pose, Mapping):
            return None  # P1 silent: cannot judge, see Traps
        bad = fix_is_lost(pose)  # this frame
        if not bad:
            self._remember(pose, now_s)  # the clue for the next alarm
        if not self.lost:
            if not bad:
                self._bad_since = None  # a good frame breaks the run
                self.prev_level = pose.get("heading_level")  # level before any loss
                return None  # still fine
            if self._bad_since is None:
                self._bad_since = now_s  # first bad frame of the run
            if now_s - self._bad_since < self._lost_s:
                return None  # not sustained yet
            self.lost, self.lost_since, self._good_since = True, now_s, None  # declared
            return "lost"
        if bad:
            self._good_since = None  # recovery run broken
            return None  # still lost
        if self._good_since is None:
            self._good_since = now_s  # first good frame of the recovery run
        if now_s - self._good_since < self._recover_s:
            return None  # recovery not sustained yet
        self.lost, self._bad_since = False, None  # recovered
        return "recovered"

    def _remember(self, pose: Mapping[str, Any], now_s: float) -> None:
        """Keep the last trustworthy position / heading for the alarm."""
        lat, lon = _num(pose.get("lat")), _num(pose.get("lon"))  # position
        if lat is not None and lon is not None:
            self.last_valid_pos = {"lat": lat, "lon": lon}  # 11 S3.3.4 last_valid_pos
            self.last_valid_mono = now_s  # monotonic, B5
        hdg = _num(pose.get("heading_rad"))  # heading
        if pose.get("heading_valid") and hdg is not None:
            self.last_valid_heading_rad = hdg  # last_valid_heading_rad

    def lost_detail(self, pose: Mapping[str, Any], now_s: float) -> Dict[str, Any]:
        """The device half of the 11 S3.3.4 detail (the task half is P3's
        caller: task_id / task_progress / action_taken)."""
        return {"type": "rtk_lost", "reason": REASON_FIX_LOST,  # the only P3 trigger
                "prev_level": self.prev_level, "level": pose.get("heading_level"),  # heading levels
                "fix_type": pose.get("fix_type"), "heading_source": pose.get("heading_source"),
                "cov_h_m": pose.get("cov_h_m"), "speed_mps": pose.get("speed_mps"),
                # time without a usable fix, monotonic (B5); since the run began
                "blind_s": (round(now_s - self._bad_since, 1)
                            if self._bad_since is not None else None),
                "last_valid_heading_rad": self.last_valid_heading_rad,
                "last_valid_mono": self.last_valid_mono,
                "last_valid_pos": self.last_valid_pos}


async def suspend_running_for_rtk(dao, conn, now_mono_ms: int
                                  ) -> Optional[Tuple[str, str, str]]:
    """RL-2: suspend the running task (passive / rtk_degraded); return
    (task_id, "suspended", reason) or None when nothing runs. Same shape and
    transaction discipline as ES-2 (lifecycle/estop_suspend.py)."""
    rows = await dao.list_by_priority()  # (task_id, prio, seq, state)
    running = [tid for tid, _p, _s, state in rows if state == "running"]
    if not running:
        return None  # nothing to suspend: the alarm still goes out
    task_id = running[0]  # at most one (scheduler invariant)
    to_state = apply_transition("running", "suspend").to_state  # the graph agrees
    validate_suspend_fields(to_state, RTK_SUSPEND_KIND, RTK_SUSPEND_REASON)  # CR-8 pair
    await conn.execute("BEGIN IMMEDIATE")  # same write-lock timing as every P3 write
    try:
        await dao.suspend_task(task_id, RTK_SUSPEND_KIND, RTK_SUSPEND_REASON, now_mono_ms)
        await conn.commit()  # never leave the implicit transaction open
    except Exception:
        await conn.rollback()  # nothing half-applied
        raise
    return task_id, to_state, RTK_SUSPEND_REASON


__all__ = ["RtkLossMonitor", "RtkLossConfigError", "suspend_running_for_rtk",
           "fix_is_lost", "LOST_FIXES", "RTK_SUSPEND_KIND", "RTK_SUSPEND_REASON"]
