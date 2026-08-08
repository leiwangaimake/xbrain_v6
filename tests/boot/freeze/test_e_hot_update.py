"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_e_hot_update.py
Brief: CFG-FZ-5 -- assertion E variant (safety key added to whitelist)
       + baseline + coverage of each safety namespace

Description:
E is a pure set-intersection check between two closed sets, so tests
are cheaper than for A/M/B/C/D (no config-root scaffolding needed).
The variant + baseline are directly testable by passing a ctx dict.

CFG-FZ-5 named variant verbatim:
  'common.safety.brake.a_mps2 added to hot-update whitelist' -> E must
  go red.

Coverage extra:
  - each of the five safety namespaces (spec / safety / motion.profiles /
    qos / fence) tested independently so a future edit that dropped
    one from _SAFETY_NAMESPACES gets caught
  - meta-test: the default (real) whitelist is disjoint from safety
    namespaces (regression guard against a future 11 S7.6 addition
    that accidentally overlaps)
"""

import pytest

from xbrain.boot.freeze.assertions.e_hot_update_disjoint import (
    _DEFAULT_HOT_UPDATE_WHITELIST, _SAFETY_NAMESPACES, _is_safety_entry, run,
)
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Reverse baseline: the real (default) whitelist passes
# ---------------------------------------------------------------------------

def test_default_whitelist_passes():
    """The real 11 S7.6 whitelist must be disjoint from safety
    namespaces. If this ever fails, either a safety key was added
    to the real whitelist (a policy defect) or a whitelist scope
    name happened to collide with a safety prefix (a naming defect);
    either way, E's fail-safe premise no longer holds."""
    result = run({})
    assert result["status"] == "pass"
    assert result["assertion"] == "E"
    assert result["whitelist_size"] == len(_DEFAULT_HOT_UPDATE_WHITELIST)
    assert result["safety_namespace_count"] == len(_SAFETY_NAMESPACES)


# ---------------------------------------------------------------------------
# CFG-FZ-5 variant verbatim: safety key added to whitelist
# ---------------------------------------------------------------------------

def test_variant_safety_brake_in_whitelist_is_red():
    """CFG-FZ-5 variant verbatim: 'common.safety.brake.a_mps2' added
    to the whitelist. E must go red with detail listing the offender."""
    bad = _DEFAULT_HOT_UPDATE_WHITELIST | {"common.safety.brake.a_mps2"}
    with pytest.raises(XbrainError) as ei:
        run({"hot_update_whitelist": bad})
    assert ei.value.code == "E_CONFIG_LOCKED"
    assert ei.value.detail["kind"] == "safety_in_hot_update"
    assert "common.safety.brake.a_mps2" in ei.value.detail["entries"]


# ---------------------------------------------------------------------------
# Per-namespace coverage: each of the five safety namespaces triggers red
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("offender", [
    # One offender per safety namespace so a future edit that removes
    # a namespace from _SAFETY_NAMESPACES loses coverage of that group
    # and this parametrize row fails.
    "common.spec.max_vx_mps",
    "common.safety.t_lat_s",
    "common.motion.profiles.patrol.max_mps",
    "common.qos.profiles.q0.priority",
    "common.fence.soft_margin_min_m",
])
def test_each_safety_namespace_is_caught(offender):
    """Add ONE key under each safety namespace to the whitelist; E
    must fire naming that key. Parametrised so failure output tells
    which namespace has the coverage gap."""
    bad = _DEFAULT_HOT_UPDATE_WHITELIST | {offender}
    with pytest.raises(XbrainError) as ei:
        run({"hot_update_whitelist": bad})
    assert offender in ei.value.detail["entries"]


# ---------------------------------------------------------------------------
# Exact-namespace-match: scope named exactly `common.safety` (no leaf)
# ---------------------------------------------------------------------------

def test_exact_namespace_match_is_caught():
    """A scope entry equal to the namespace ROOT (not a leaf under it)
    must also fire. Catches the case where someone thinks 'the whole
    safety namespace should be hot-updatable' and adds the bare
    prefix to the whitelist."""
    bad = _DEFAULT_HOT_UPDATE_WHITELIST | {"common.spec"}
    with pytest.raises(XbrainError) as ei:
        run({"hot_update_whitelist": bad})
    assert "common.spec" in ei.value.detail["entries"]


# ---------------------------------------------------------------------------
# Multiple offenders: E reports ALL of them in one raise
# ---------------------------------------------------------------------------

def test_multiple_offenders_all_reported():
    """E fails-loud with EVERY safety entry, not just the first.
    Different from A/B/C/D (first-fail). E's failure surface is
    'here's the delta to fix'; giving them one at a time forces a
    fix-then-restart cycle per entry."""
    bad = _DEFAULT_HOT_UPDATE_WHITELIST | {
        "common.safety.t_lat_s",
        "common.fence.soft_margin_min_m",
    }
    with pytest.raises(XbrainError) as ei:
        run({"hot_update_whitelist": bad})
    entries = ei.value.detail["entries"]
    assert "common.safety.t_lat_s" in entries
    assert "common.fence.soft_margin_min_m" in entries
    # sorted() -> alphabetical order
    assert entries == sorted(entries)


# ---------------------------------------------------------------------------
# Non-safety entries are ignored (regression: no false positive)
# ---------------------------------------------------------------------------

def test_non_safety_prefix_not_caught():
    """A common.audio.* entry (not a safety namespace) must NOT trip
    E. Guards against an over-broad implementation that flagged
    every common.* key."""
    bad = _DEFAULT_HOT_UPDATE_WHITELIST | {"common.audio.bypass_keywords"}
    # Should NOT raise -- common.audio.* is not on the safety list.
    result = run({"hot_update_whitelist": bad})
    assert result["status"] == "pass"


# ---------------------------------------------------------------------------
# _is_safety_entry unit tests (helper coverage)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("entry,expected", [
    # Exact namespace matches
    ("common.spec", True),
    ("common.safety", True),
    ("common.motion.profiles", True),
    ("common.qos", True),
    ("common.fence", True),
    # Prefix matches (leaf under namespace)
    ("common.safety.brake.a_mps2", True),
    ("common.spec.max_vx_mps", True),
    ("common.motion.profiles.patrol.max_mps", True),
    # Non-safety
    ("common.audio.bypass_keywords", False),
    ("log_level", False),
    ("debug_flags", False),
    # Near-miss: prefix WITHOUT dot separator must NOT match
    # (common.specifically would be a false positive if we didn't
    # require the dot).
    ("common.specifically.something", False),
    ("common.motion.profiles_extra", False),
])
def test_is_safety_entry_helper(entry, expected):
    """Unit-test _is_safety_entry directly: 5 namespace roots + 3
    leaves + 3 non-safety + 2 near-miss avoidance."""
    assert _is_safety_entry(entry, _SAFETY_NAMESPACES) == expected


# ---------------------------------------------------------------------------
# Meta-test: default whitelist stays disjoint
# ---------------------------------------------------------------------------

def test_meta_default_whitelist_disjoint_from_safety():
    """Regression guard: whitelist ∩ any safety namespace prefix = ∅.
    If a future editor adds a new whitelist entry that happens to
    start with 'common.safety.' or any other safety prefix, this
    test fails immediately -- before the change reaches production."""
    for entry in _DEFAULT_HOT_UPDATE_WHITELIST:
        assert not _is_safety_entry(entry, _SAFETY_NAMESPACES), (
            "whitelist entry %r overlaps a safety namespace -- CFG-FZ-5 "
            "would refuse to start" % entry
        )
