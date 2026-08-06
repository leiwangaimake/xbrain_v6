"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: The unified arbitration framework core, one semantics for seven domains

Description:
What problem this solves. 11 S7A defines a single arbitration semantics that all
seven resource domains share (motion, speaker, asr, payload_light, ptz, gpu,
dock), and 14 S3 makes it a shared LIBRARY every owning process instantiates
rather than one central service -- because domain 1 (motion) sits inside the
20 Hz loop and cannot afford a cross-process round trip, and because P2 crashing
must not stop P1 arbitrating motion (14 S3.3). This package is that library's
core: register / request / renew / release / ack_preempt / cancel / tick /
holder, plus SourceSpec, the four gen rules, and the lease. BIZ-CM-2 assembles
state/arb and the audit stream on top of it; BIZ-CM-3 adds the disarm semantics;
BIZ-CM-5 adds the T-2/T-3/T-4 preempt refinements.

Layout, and why it is split:
  * model.py -- the frozen value types (SourceSpec, Request, Grant, Holder,
    LastChange, ArbEvent) and the enums / contract constants. Data only, no
    logic, no clock.
  * core.py  -- the Arbiter state machine. All the timing arithmetic is done on
    caller-supplied CLOCK_MONOTONIC milliseconds; the class reads no clock, which
    is what makes a wall-clock step unable to judge holders dead (AB-6).

What this package deliberately does NOT export, so a caller does not mistake the
core for the whole framework:
  * no message codecs -- Grant/Holder are objects; serialising them onto
    cmd/arb/{domain}/grant and state/arb/{domain} is BIZ-CM-2.
  * no disarm (缴械 / soft-estop) surface -- that is BIZ-CM-3.
  * no clock -- import mono_now_s from xbrain.common.clock at the CALL site and
    pass the millisecond value in. Centralising a clock read inside the arbiter
    is the exact AB-6 defect.

Naming note: the documents write the shared library as common/arbiter/. On disk
the Python source is xbrain/common/arbiter/ -- CLAUDE.md 0.2 reserves top-level
common/ for DEPLOYED artifacts (generated C++ headers), and the shared layer's
Python lives under xbrain/. The TODO row for BIZ-CM-1 gives xbrain/common/arbiter/
as its target directory, which is the same reading.
"""

# Re-export the public surface so a caller writes
# `from xbrain.common.arbiter import Arbiter, SourceSpec, Request` without
# knowing which submodule each name lives in. The submodule split is an
# implementation detail; this list is the package's contract.
from .core import Arbiter                     # the per-domain state machine
from .model import (
    ArbAction,                    # audit action enum; BIZ-CM-2 maps it to severity
    ArbEvent,                     # one audit record emitted on a holder change
    DEFAULT_LEASE_MS,             # 11 S7A.4 contract default (1000 ms)
    DEFAULT_WAIT_ATOMIC_TIMEOUT_MS,   # 11 S7A.3 contract default (3000 ms)
    Grant,                        # the result of one request (11 S7A.2)
    GrantResult,                  # granted | denied | queued | ... enum
    Holder,                       # frozen holder snapshot (11 S7A.5.1)
    IMMEDIATE_GRACE_MS,           # 11 S7A.3 immediate preempt grace (100 ms)
    LastChange,                   # most recent holder change (11 S7A.5.1)
    Preempt,                      # the preempt sub-object of a queued grant
    PreemptPolicy,                # immediate | wait_atomic | reject enum
    Request,                      # one acquire attempt (11 S7A.1)
    SourceSpec,                   # a competing source registered at startup
)

# Explicit, so `import *` carries exactly these names and a linter does not prune
# the re-exports above as unused: they are the public half of this package. The
# order is public-surface first (the class and the DTOs a caller touches), then
# the enums, then the contract constants, so the list reads as a table of what
# this package offers rather than an alphabetised dump.
__all__ = [
    "Arbiter",                    # instantiate one per owned domain
    "SourceSpec",                 # register() takes this
    "Request",                    # request() takes this
    "Grant",                      # request() returns this
    "Holder",                     # holder() returns this
    "Preempt",                    # Grant.preempt is this
    "LastChange",                 # last_change() returns this
    "ArbEvent",                   # tick()/drain_events() return these
    "PreemptPolicy",              # a SourceSpec field
    "GrantResult",                # a Grant field
    "ArbAction",                  # an ArbEvent field
    "DEFAULT_LEASE_MS",           # contract constants, for bring-up callers
    "IMMEDIATE_GRACE_MS",         # with no domain config to hand
    "DEFAULT_WAIT_ATOMIC_TIMEOUT_MS",
]
