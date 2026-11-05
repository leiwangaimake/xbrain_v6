"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: laps.py
Brief: How many route passes a patrol task asks for -- loops and round trips (99 U131)

Description:
What problem this solves. Until 2026-11-05 every patrol ran exactly one pass:
route_push_runtime opened the progress run with loop_total = 1 and P1 is
single-pass (nav/progress.py "single pass this phase: 0 / 1"), so a spoken
"沿1号路径巡逻3圈" was refused out loud and "来回巡三次" had nowhere to go.
The user ruled (2026-11-05, 99 U131): laps are counted by P3, which re-pushes
the route once per pass from where the robot is; P1 stays single-pass.

The rule this file holds -- one pass is one push of the U127 patrol plan
(route_plan.plan_patrol) from the robot's position at that moment:
  * closed route: one pass = once round (the plan from the entry point);
    "N圈" = N passes. "来回" means nothing on a ring and is N passes too.
  * open route: one pass = one end-to-end coverage (the first pass goes to
    the farther end first, then back if the near side is a real stretch;
    every later pass starts at an end and runs to the other end).
    "N圈 / N遍 / N趟" = N passes; "来回N次" = 2N passes, so "来回巡一次"
    from one end is A -> B -> A, the user's own example of ONE round trip.
  * loops = 0 is the endless standing patrol of 11 S7.8.3 LM-4: passes = 0,
    which the progress sink reads as "never finished by arrival".

Inputs come from mission_json params.slots (the voice path, 16 S6.7.3 [3]:
loops defaults to 1 when not said); round_trip is the P4 rule layer's flag
for a "来回" sentence. Both are validated at admission (V-8a, E_SCHEMA) with
the same function, so dispatch never meets a value it has to guess about.

Boundary: this decides the COUNT only. Which polyline a pass follows is
route_plan.plan_patrol; when the next pass is pushed is the progress sink
(runtime/progress_sink.py) on P1's arrived frame; the push itself is
route_push_runtime (trigger RP-5).

Looks right but is wrong:
  * counting a round trip as one pass on an open route -- "来回巡三次"
    would then cover the route three times instead of six (A-B-A three
    times), which is not what the operator said;
  * doubling the count on a closed route -- a ring has no "back", and two
    laps for one "来回" would be a lap nobody asked for.
"""

from __future__ import annotations  # postponed annotations

from dataclasses import dataclass  # an immutable value
from typing import Any, Mapping, Optional  # plain types

#: The largest lap count a task may carry -- the P4 rule layer already caps a
#: spoken count at 99 (ptp_slots._laps); the same bound here refuses a frame
#: from any other sender that would otherwise patrol for days.
MAX_LOOPS = 99  # laps, not a time


@dataclass(frozen=True)  # value object
class LapSpec:  # what the task asked for
    """loops as said (0 = endless) and whether it was a "来回" request."""
    loops: int  # 0..MAX_LOOPS; 0 = endless (LM-4)
    round_trip: bool  # a "来回" sentence

    def passes(self, loop_mode: str) -> int:  # the push count
        """Pushes this task needs on a route of loop_mode (header rule):
        a round trip doubles the count on an open route only; 0 stays 0."""
        if self.round_trip and loop_mode != "closed":  # out and back on a line
            return 2 * self.loops  # A -> B -> A is two passes
        return self.loops  # rings, and plain laps on a line


def lap_spec_from_params(params: Optional[Mapping[str, Any]]) -> LapSpec:
    """The LapSpec in mission params (params.slots.loops / round_trip).

    loops absent -> 1 (16 S6.7.3 [3], the one default the voice path has).
    Raises ValueError naming the field for anything else that is not a
    plain int in 0..MAX_LOOPS or a plain bool -- admission turns that into
    E_SCHEMA (15 V-8a), so a bad value never reaches dispatch.
    """
    slots = params.get("slots") if isinstance(params, Mapping) else None  # where the voice path puts them
    slots = slots if isinstance(slots, Mapping) else {}  # none said
    loops = slots.get("loops", 1)  # not said -> one lap (S6.7.3 [3])
    if isinstance(loops, bool) or not isinstance(loops, int):  # True is an int in Python
        raise ValueError("slots.loops must be an int, got %r" % (loops,))  # named
    if not 0 <= loops <= MAX_LOOPS:  # V-8a: loops >= 0, and a sane cap
        raise ValueError("slots.loops must be in 0..%d, got %d"
                         % (MAX_LOOPS, loops))  # named
    rt = slots.get("round_trip", False)  # not said -> plain laps
    if not isinstance(rt, bool):  # "yes" / 1 are a sender bug
        raise ValueError("slots.round_trip must be a bool, got %r" % (rt,))  # named
    return LapSpec(loops=loops, round_trip=rt)  # validated
