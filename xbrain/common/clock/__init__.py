"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The monotonic clock every XBRAIN Python process reads its time from

Description:
What problem this solves. 11 S0.2.1 (grep the heading 时间基准) states D-18: every
timeout, period and age judgement uses CLOCK_MONOTONIC, and the message ts (a
wall clock) is only for cross-host alignment, recording timelines and latency
statistics. The section is explicit that this is a safety item and not a style
preference, and it spells out the failure chain: chronyd steps the wall clock
when RTK first locks, and on a cold start with no RTC battery the step can be
seconds to years. With a wall clock behind it, a forward step makes
cmd_age = now - last_cmd.ts exceed 200 ms instantly, so quadruped Tier 1 locks
mid-drive and needs a manual enable; a backward step makes the age negative, so
either it wraps to a huge value and locks anyway, or the timeout protection is
simply off for the length of the step while the payload keeps executing the last
velocity it was given. That second one is a failure that makes the system FASTER,
which 10 G-3 / CON-07 rule out by name.

So the rule is not negotiable, and this module is the positive half of enforcing
it. scripts/lint/clock_scan.py is the negative half: it says what may not appear
in the source. Neither is enough on its own -- a lint that only forbids leaves
every caller to reach for time.monotonic() directly, and then the reasoning above
lives nowhere and the next person has to rediscover it from an incident.

Which section says what
-----------------------
  * 11 S0.2.1 CLK-C1 -- Python uses time.monotonic(), C++ std::chrono::
    steady_clock, ROS 2 rclcpp::Clock(RCL_STEADY_TIME) and never the default
    constructed rclcpp::Clock(). This module is the Python half of CLK-C1.
  * 11 S0.2.1 CLK-C2 -- timers too, which is why asyncio code uses loop.time().
    See the boundary note below: this module does not wrap the event loop.
  * 11 S0.2.1 CLK-C4 -- a monotonic reading is comparable only on the same host
    within the same boot.
  * CLAUDE.md 3.4 -- the same rule as a three-language table, plus the statement
    that it is checkable by CI, which clock_scan.py implements.

What this module deliberately does NOT do
-----------------------------------------
Every item below is owned somewhere else, and a second implementation here would
be a second thing to keep in step:

  * It does not compute message age. The envelope layer owns that, because age
    has four branches (mono present and boot matching, boot not matching, mono
    absent for a cloud publisher, and negative age) which need the envelope
    fields to decide between -- see 11 S3.0.1, whose heading greps as
    年龄与超时的规范计算方法, and CLK-C5 for the negative case,
    which must clamp to zero AND emit a warn event with detail.kind =
    "negative_age". A clamp buried in a clock helper would satisfy the first half
    of CLK-C5 while silently dropping the event, so this module offers no
    subtraction helper at all: age is not arithmetic, it is a decision.
  * It does not know about boot_id. Two readings are comparable only within one
    boot (CLK-C4), and what makes that checkable is the boot field travelling
    with the message, so the check belongs where the envelope is parsed.
  * It does not decide ClockStatus.sync. 11 S1.5 CLK-A1 gives that to
    rtk_driver alone and CLK-A2 forbids every other process from judging for
    itself -- no reading chronyc, no comparing local clocks. A helper here named
    anything like is_time_good() would be exactly the forbidden thing.
  * It does not hold the S1.6 timeout table. Those thresholds are a separate
    deliverable and must be generated from the contract table rather than typed
    in a second time.
  * It does not read the wall clock, and no wall_now_s() belongs here. The three
    permitted wall-clock uses stay at their call sites carrying the visible
    line-level marker clock_scan.py requires, because the marker is what makes
    each use reviewable one at a time. Centralising them behind a friendly name
    here would turn "three audited exceptions" into "a function anyone may
    call", and the audit trail would be gone.

Traps -- things that look right and are not
-------------------------------------------
  1. datetime.now(timezone.utc) looks like the careful version of
     datetime.now(). It is not: the argument selects a timezone, not a clock.
     The reading still comes from CLOCK_REALTIME and still steps.
  2. time.perf_counter() and time.process_time() are monotonic too, so they pass
     a casual review. Their epoch is undefined, which means a reading from one
     of them cannot be subtracted from a reading of time.monotonic() and cannot
     go into an envelope mono field, where the receiver subtracts it against its
     own time.monotonic(). Mixing them produces an age that is wrong by whatever
     the two epochs happen to differ by -- a number that looks entirely
     plausible. Use them only inside one function, for one measurement, never
     across a boundary.
  3. CLOCK_BOOTTIME (time.clock_gettime(time.CLOCK_BOOTTIME)) is tempting as an
     upgrade because it keeps counting across suspend. Do not substitute it: 11
     S0.2.1 names CLOCK_MONOTONIC, and a process reading BOOTTIME while its
     peers read MONOTONIC produces readings that differ by the total suspended
     time, on the same host, in the same boot -- precisely the comparison
     CLK-C4 says is safe.
  4. 0.0 is a legal reading. On Linux CLOCK_MONOTONIC counts from boot, so an
     early reading really can be a small number, and nothing forbids zero. Do
     NOT use 0.0 as a "no reading yet" sentinel; use None, and let the type say
     Optional[float]. This is the same failure CLAUDE.md 3.1 describes for
     safety parameters: a falsy value that passes an is-it-set test and then
     behaves as an enormous age or a zero deadline, with nothing logged.
  5. A stored monotonic reading must never be persisted. 15 SS-1 states it for
     the task database: after a restart the reading refers to a boot that ended,
     so the elapsed time computed from it is meaningless and arbitrarily large.
"""

import time
from typing import List

# The public surface is two names, and it is written out so that "what does this
# package export" is answerable without reading the body. CFG-CM-12 names both
# of them; do not add a third export without the item that asks for it, and in
# particular see the wall-clock boundary note above before adding one.
__all__: List[str] = ["mono_now_s", "MonoClock"]


# The name carries its unit. 11 spells the contract fields the same way --
# expires_mono_s, requested_mono_s, started_mono -- so a reading from here and
# the field it ends up in are recognisably the same quantity. A bare now() would
# leave the caller to remember whether it was handed seconds or milliseconds,
# and the two are three orders of magnitude apart in a system whose thresholds
# are written in milliseconds (Tier 1 at 200 ms) and whose readings are in
# seconds.
def mono_now_s() -> float:
    """Seconds from CLOCK_MONOTONIC, as a float.

    This is the one call every timeout, period and age in Python starts from.
    """
    # time.monotonic() and nothing else. On Linux CPython this is
    # clock_gettime(CLOCK_MONOTONIC), which is what 11 S0.2.1 names.
    #
    # Why there is no fallback branch here. The obvious "robust" version tries
    # time.monotonic() and drops back to something else if it is unavailable --
    # and the something else is always a wall clock, so the fallback path is the
    # exact defect this module exists to prevent, reached only on the rare
    # machine where nobody is watching. time.monotonic() has been guaranteed
    # present since Python 3.5. There is nothing to fall back to and nothing to
    # guard against.
    return time.monotonic()


class MonoClock:
    """An injectable monotonic clock source.

    Take one of these as a constructor argument wherever a component measures
    time, and tests can drive that component without sleeping.
    """

    # Why this class exists at all, given mono_now_s() is right there.
    #
    # Components that time things -- a watchdog, a retry backoff, a hysteresis
    # window -- are close to untestable when they call the clock directly: the
    # only way to exercise "3.0 s of sustained clearance" is to make the test
    # take 3.0 s. Suites written that way get slow, then get marked skip, and
    # then the hysteresis is not tested at all. Taking the clock as a
    # constructor argument lets a test drive the component through the whole
    # window in microseconds, which is what CLAUDE.md 7.1 means by the injection
    # layer being the most valuable class of test in this project.
    #
    # It is a class rather than a callable parameter with a default, because
    # `def __init__(self, now=time.monotonic)` is a default in the constructor
    # signature -- the shape CLAUDE.md 3.1 rejects for safety parameters -- and
    # it also reads as an invitation to pass any zero-argument float function,
    # including one backed by the wall clock.
    #
    # *** There is deliberately NO source or clock_fn parameter on __init__.
    # A source parameter is a documented way to hand a wall clock to code that
    # believes it holds a monotonic one: MonoClock(source=time.time) is one
    # short line, it type-checks, it runs, and every timeout in the component
    # downstream is then wall-clock based with nothing in the component itself
    # to show it. Test doubles subclass and override now_s() instead, which
    # cannot be done by accident and is visible at the class definition rather
    # than at one call site somewhere.
    def now_s(self) -> float:
        """Current monotonic reading in seconds.

        Overridden by test doubles; production code never overrides it.
        """
        # Delegating rather than calling time.monotonic() again keeps one
        # implementation point. If the module-level function ever has to change
        # -- say the contract moves to a different POSIX clock -- a second copy
        # here would leave half the processes on the old clock, and readings
        # from the two halves would still be floats and still subtract cleanly.
        return mono_now_s()
