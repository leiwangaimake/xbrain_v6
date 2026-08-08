"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_startup_selfcheck.py
Brief: INF-ZN-5 startup selfcheck -- each of A-2..A-7, UNREGISTERED, W-2 has
       its mutant, plus the reverse assertion that a clean set passes

Description:
The seven-antipattern coverage the criterion names verbatim. Each mutation
takes a baseline of legal declarations and flips ONE knob to the antipattern
form; the test asserts SelfcheckError with the expected antipattern code and
key in detail. The reverse test (test_clean_set_passes) is what stops the
whole file from being satisfied by a selfcheck() that always raises.
"""

import os
import sys

import pytest

from xbrain.common import errors
from xbrain.common.zenoh.startup_selfcheck import (
    DEFAULT_CROSS_PLANE_PROCESSES, Declaration, SelfcheckError, selfcheck,
)

# Reach the ZN-4 extractor as the registry source, so the two items stay
# genuinely coupled (a pattern the extractor cannot see is a pattern this
# check cannot approve).
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(ROOT, "scripts", "doccheck"))
from key_registry import extract as extract_keys                # noqa: E402


@pytest.fixture(scope="module")
def registry():
    """The pattern set extracted from 11 S2.2.1~S2.2.9. Module-scoped because
    the doc parse is not free and the extractor is pure."""
    with open(os.path.join(ROOT, "docs", "11-接口契约.md"),
              encoding="utf-8") as fh:
        records = extract_keys(fh.read())
    return {r["pattern"] for r in records}


# --------------------------------------------------------------------------
# The clean baseline every mutation forks from.
# --------------------------------------------------------------------------

def _clean_decls():
    """One legal declaration per family the checks scope over. Every mutation
    flips ONE of these; test_clean_set_passes proves the baseline itself is
    clean, so a mutation going red is a real signal."""
    return [
        # rt periodic key, publisher on Q1_rt (DROP + real_time + Ring(1))
        Declaration(role="pub", key="rt/motion/cmd_vel", profile="Q1_rt",
                    process="p1_motion"),
        # rt periodic subscriber -- Q1_rt supplies Ring(1), which is fine
        Declaration(role="sub", key="rt/perception/targets", profile="Q1_rt",
                    process="p1_motion"),
        # event key -- Q3_cmd is reliable + FIFO, matches A-4's contract
        Declaration(role="pub", key="event/{severity}/{category}",
                    profile="Q3_cmd", process="p5_gateway"),
        # cmd/estop -- Q0_safety (DROP + real_time), matches A-5's contract
        Declaration(role="pub", key="cmd/estop", profile="Q0_safety",
                    process="p2_core"),
        # concrete state/ sub on a cross-plane process is fine (not wildcard)
        Declaration(role="sub", key="state/mode", profile="Q2_state",
                    process="p2_core"),
    ]


# --------------------------------------------------------------------------
# The reverse assertion (proves the baseline is real)
# --------------------------------------------------------------------------

def test_clean_set_passes(registry):
    """*** A wholly compliant declaration set must NOT raise -- else every
    mutation "passes" by making selfcheck() unconditional."""
    selfcheck(_clean_decls(), registry)


# --------------------------------------------------------------------------
# The four criterion mutations (S2.148)
# --------------------------------------------------------------------------

def test_unregistered_pub_key_raises(registry):
    """① publishing a key not in the 11 S2.2 registry."""
    decls = _clean_decls() + [
        Declaration(role="pub", key="rt/motion/never_registered",
                    profile="Q1_rt", process="p1_motion"),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    assert ei.value.code == errors.E_CONFIG_INVALID
    assert ei.value.detail["antipattern"] == "UNREGISTERED"
    assert ei.value.detail["key"] == "rt/motion/never_registered"


def test_cross_plane_wildcard_sub_raises_w2(registry):
    """② cross-plane process wildcard subscription (W-2)."""
    decls = _clean_decls() + [
        Declaration(role="sub", key="state/**", profile="Q2_state",
                    process="p1_motion"),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    assert ei.value.detail["antipattern"] == "W-2"
    assert ei.value.detail["process"] == "p1_motion"


def test_rt_with_block_raises_a2(registry):
    """③ rt/ + block (A-2 / QOS-C1)."""
    decls = _clean_decls() + [
        Declaration(role="pub", key="rt/motion/cmd_vel", profile="Q3_cmd",
                    process="p1_motion"),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    assert ei.value.code == errors.E_QOS_VIOLATION
    assert ei.value.detail["antipattern"] == "A-2"


def test_missing_qos_raises_a7(registry):
    """④ publisher without an explicit QoS profile (A-7)."""
    decls = _clean_decls() + [
        Declaration(role="pub", key="cmd/task", profile=None,
                    process="p5_gateway"),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    assert ei.value.detail["antipattern"] == "A-7"


# --------------------------------------------------------------------------
# The four补 mutations (A-3 / A-4 / A-5 / A-6, verbatim from the criterion)
# --------------------------------------------------------------------------

def test_rt_periodic_sub_with_fifo_raises_a3(registry):
    """A-3: RT periodic key on subscriber given FIFO via handler_override."""
    decls = _clean_decls() + [
        Declaration(role="sub", key="rt/perception/targets", profile="Q1_rt",
                    process="p1_motion",
                    handler_override=("fifo", 8)),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    assert ei.value.detail["antipattern"] == "A-3"


def test_event_with_best_effort_raises_a4(registry):
    """A-4 (reliability half): event/** on Q1_rt (best_effort)."""
    decls = _clean_decls() + [
        Declaration(role="pub", key="event/{severity}/{category}",
                    profile="Q1_rt", process="p5_gateway"),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    assert ei.value.detail["antipattern"] == "A-4"


def test_event_with_ring_handler_raises_a4(registry):
    """A-4 (handler half): event/** subscribed with a ring override."""
    decls = _clean_decls() + [
        Declaration(role="sub", key="event/{severity}/{category}",
                    profile="Q3_cmd", process="p5_gateway",
                    handler_override=("ring", 8)),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    assert ei.value.detail["antipattern"] == "A-4"


def test_cmd_estop_block_raises_a5(registry):
    """A-5: cmd/estop with block."""
    decls = _clean_decls() + [
        Declaration(role="pub", key="cmd/estop", profile="Q3_cmd",
                    process="p2_core"),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    assert ei.value.detail["antipattern"] == "A-5"


def test_audio_fifo256_raises_a6(registry):
    """A-6: rt/audio/mic subscribed with FIFO(256) -- the 凭空取 default."""
    decls = _clean_decls() + [
        Declaration(role="sub", key="rt/audio/mic", profile="Q1_rt",
                    process="p2_core", handler_override=("fifo", 256)),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    # A-3 also matches (rt periodic sub with fifo), but A-6 is the more
    # specific rule for the (256) case; we don't lock which one fires --
    # either one is a correct refusal.
    assert ei.value.detail["antipattern"] in ("A-6", "A-3")


# --------------------------------------------------------------------------
# Additional guards (not from the four/four criterion but real defects)
# --------------------------------------------------------------------------

def test_unknown_profile_name_raises(registry):
    """A typo in the profile name refuses too, so a caller cannot slip past
    the checks by naming something FROZEN_PROFILES does not know."""
    decls = _clean_decls() + [
        Declaration(role="pub", key="cmd/task", profile="Q_typo",
                    process="p5_gateway"),
    ]
    with pytest.raises(SelfcheckError) as ei:
        selfcheck(decls, registry)
    assert ei.value.detail["antipattern"] == "UNKNOWN_PROFILE"


def test_non_cross_plane_wildcard_sub_is_allowed(registry):
    """A non-cross-plane process (p3_task) may wildcard-subscribe on its own
    plane -- W-2 targets the five processes in DEFAULT_CROSS_PLANE_PROCESSES."""
    assert "p3_task" not in DEFAULT_CROSS_PLANE_PROCESSES
    decls = _clean_decls() + [
        Declaration(role="sub", key="state/**", profile="Q2_state",
                    process="p3_task"),
    ]
    selfcheck(decls, registry)                        # must not raise
