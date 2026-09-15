"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_rt_limits.py
Brief: A unit whose process needs SCHED_FIFO and mlockall must raise both limits

Description:
Measured on the ORIN on 2026-09-15: systemd ships DefaultLimitMEMLOCK=65536
(64 KB) and DefaultLimitRTPRIO=0, and no unit in deploy/systemd/ raised either.
Under those defaults both realtime requirements of 13 S9 fail:

  * mlockall(MCL_CURRENT|MCL_FUTURE) returns ENOMEM (RTC-7). The process runs
    on, with its pages pageable, and misses a deadline the first time the
    kernel reclaims one.
  * sched_setscheduler(SCHED_FIFO, 80) returns EPERM (S9.1, the ctrl thread).
    The thread runs on at ordinary priority, invisible until core 5 gets busy.

Neither failure prints anything on its own, and both present as jitter rather
than as an error -- which is the hardest kind of defect to trace back to a unit
file. Hence a test rather than a comment.

Scope, stated because a criterion with an undeclared surface is worth little
(CLAUDE.md 3.2 form 6): this checks the units listed in RT_UNITS below, which
are the ones whose design explicitly assigns SCHED_FIFO. It does NOT scan every
unit and require the limits, because most processes here are Python and must
NOT have them -- granting realtime priority to a process with a garbage
collector is worse than not granting it at all.
"""

import os
import re

import pytest

pytestmark = pytest.mark.no_device

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
UNIT_DIR = os.path.join(ROOT, "deploy", "systemd")

#: unit file -> (highest SCHED_FIFO priority its design assigns, where that is
#: stated). Only units whose design actually asks for realtime scheduling.
RT_UNITS = {
    "xbrain-quadruped.service": (80, "13 S9.1: ctrl SCHED_FIFO 80, chs_b 70"),
}


def _read(name):
    with open(os.path.join(UNIT_DIR, name), encoding="utf-8") as f:
        return f.read()


@pytest.mark.parametrize("unit", sorted(RT_UNITS))
def test_memlock_is_raised(unit):
    """mlockall needs an unbounded RLIMIT_MEMLOCK, not a larger finite one.

    MCL_FUTURE locks pages the process has not allocated yet, so any finite
    ceiling is a limit on how much the process may later allocate -- and the
    failure arrives at the allocation, far from this file.
    """
    text = _read(unit)
    m = re.search(r"^LimitMEMLOCK=(\S+)", text, re.M)
    assert m, (
        "%s does not raise LimitMEMLOCK. systemd's default is 65536 bytes "
        "(measured on the ORIN), and mlockall(MCL_CURRENT|MCL_FUTURE) returns "
        "ENOMEM under it -- the process runs on with pageable memory and misses "
        "its first deadline whenever the kernel reclaims a page (13 RTC-7)" % unit)
    assert m.group(1) == "infinity", (
        "%s sets LimitMEMLOCK=%s; MCL_FUTURE locks pages not yet allocated, so "
        "any finite ceiling becomes an allocation limit whose failure surfaces "
        "far from here" % (unit, m.group(1)))


@pytest.mark.parametrize("unit", sorted(RT_UNITS))
def test_rtprio_covers_what_the_design_asks_for(unit):
    """And is not larger than that: a ceiling above the design grants nothing
    the process uses and removes a bound somebody may later rely on."""
    want, where = RT_UNITS[unit]
    text = _read(unit)
    m = re.search(r"^LimitRTPRIO=(\d+)", text, re.M)
    assert m, (
        "%s does not raise LimitRTPRIO. systemd's default is 0 (measured), and "
        "sched_setscheduler(SCHED_FIFO, %d) returns EPERM under it -- the "
        "thread runs at ordinary priority with nothing in any log (%s)"
        % (unit, want, where))
    got = int(m.group(1))
    assert got >= want, (
        "%s allows RTPRIO %d but %s" % (unit, got, where))
    assert got == want, (
        "%s allows RTPRIO %d while %s -- a ceiling above what the design uses "
        "grants a privilege nothing needs" % (unit, got, where))


def test_python_units_do_not_get_realtime_limits():
    """The negative control, and it is not decoration.

    Without it this file would pass on a change that gave every unit the
    limits, which would put realtime priority within reach of processes that
    have a garbage collector -- 12 RTC-2/RTC-3 exist because a GC pause on a
    FIFO thread starves everything below it. It also proves the criterion above
    can distinguish units at all.
    """
    offenders = []
    for name in sorted(os.listdir(UNIT_DIR)):
        if not name.endswith(".service") or name in RT_UNITS:
            continue
        text = _read(name)
        if re.search(r"^LimitRTPRIO=[1-9]", text, re.M):
            offenders.append(name)
    assert not offenders, (
        "these units raise LimitRTPRIO without appearing in RT_UNITS: %s -- "
        "either the design now assigns them SCHED_FIFO (add them here, with "
        "the section that says so) or the limit should not be there"
        % ", ".join(offenders))
