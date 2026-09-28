"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_unwired_modules.py
Brief: meta door -- a module nothing in production even names is registered, not silent

Description:
This file solves one problem: a module can sit in xbrain/ with a full
green test suite and NO production caller, and nothing says so.

The repository has already paid for this twice, both recorded in
docs/NEXT.md:
  * route/push.py -- 97 lines, 6 green tests, zero callers, and not one
    of its fields matched 11 S3.5A RouteGeometry. It stayed that way for
    months while the question it was supposed to answer ("can P3 push a
    route?") had the answer "no".
  * the r_eff max() test in the old rns/ tree -- a green test pinning a
    shape that 12 v0.7 had already declared void, i.e. a test that forced
    the WRONG implementation (docs/rns-legacy-audit.md S3).
Both are the same failure: a unit test imports the function directly and
asserts on it, so the test is green whether or not anything calls it.
CLAUDE.md S3.2 form 1 -- "is there an implementation that does nothing
and still passes?" For a module with no callers the answer is always yes,
because "does nothing" is exactly what it already does.

WHAT THIS DOOR DOES
  It computes the set of xbrain/ modules that are not NAMED anywhere else
  in the production trees, and compares it against the frozen registry
  below in BOTH directions:
    * a module becomes unwired and is not in the registry  -> red
    * a registry entry becomes wired (or is deleted)       -> red
  The second direction is the half that makes the list shrink instead of
  rot: you cannot wire something and leave a stale entry behind.

WHY "NAMED" AND NOT "IMPORTED"
  An import-based checker has to model relative imports, imports made
  inside a function body (this repo does that on purpose in the runtime
  wirings to keep startup cheap), and importlib. Getting any of those
  wrong produces a FALSE POSITIVE on a module that really is wired --
  and a door that is red for a wired module is a door somebody relaxes
  to "contains is enough" within the week (CLAUDE.md S3.2 form 2).
  A name-mention check errs the other way: a module mentioned only in a
  neighbouring comment counts as wired and is NOT reported. So this door
  under-reports by construction. What it does report is the hard core:
  modules that no other production file names at all. That asymmetry is
  deliberate; do not "tighten" it into an import graph without also
  proving the false-positive rate is zero.

WHAT THIS DOOR DOES NOT DO
  It does not say a listed module is wrong, dead, or should be deleted.
  Startup self-checks, static-scan helpers and lint bodies legitimately
  have no in-tree caller. It says only: nothing production-side names it,
  therefore no test of it can tell you whether the system uses it. The
  disposition per entry is the note in the registry, and each note is
  supposed to become either a wiring commit or a deletion commit.

SCAN SURFACE (CLAUDE.md S3.2 form 6 -- an undeclared count is unreadable)
  candidates: xbrain/**/*.py, excluding __init__.py and _legacy/ trees
  mentions:   xbrain/**/*.py + scripts/**/*.py
  NOT in the surface: tests/ -- which is where this file lives, so the
  door cannot match its own text (CLAUDE.md S3.2 form 3, self-harm).
  The surface is printed in every failure message; a number produced by
  an unstated surface is not a finding.

RED PROOF (CLAUDE.md S3.3 -- an assertion that never went red is unwritten)
  Drop a new .py under xbrain/ that nothing names: this goes red naming
  it. Delete any registry entry: this goes red the other way. Both were
  run before this file was committed.
"""

import os
import re

import pytest

pytestmark = pytest.mark.no_device

# Repository root, derived from this file so the door works from any cwd.
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# --- scan surface, declared as data so the failure message can print it ---

# Where candidate modules are looked for. Only xbrain/ -- C++ lives in
# common/ and ros2_ws/ and has its own discipline gate (tests/ci).
_CANDIDATE_ROOTS = ("xbrain",)

# Where a mention counts. scripts/ is included because several xbrain
# modules are driven by a scripts/ entry point rather than by another
# xbrain module, and that is wiring, not orphanhood.
_MENTION_ROOTS = ("xbrain", "scripts")

# Directory names excluded from candidacy. _legacy is the tombstone tree
# created by PM1.5 (docs/rns-legacy-audit.md S2): those files are kept
# ON PURPOSE with headstone headers pointing at what replaced them, so
# reporting them would be noise that trains people to ignore this door.
_EXCLUDED_DIR_PARTS = ("__pycache__", "_legacy")


# --- the frozen registry -------------------------------------------------
#
# Every entry is DEBT, not an exemption. The note says why it is here and
# who closes it. Four tags, closed set:
#
#   ruled-dead  a ruling to delete it already exists; only execution is
#               missing. Closing move = delete the file and its tests.
#   no-surface  it guards or serves an interface that does not exist in
#               production, so its own tests are green by construction.
#               Closing move = build the surface, or delete the module.
#   registered  already carried in docs/NEXT.md with a stated blocker;
#               listed here only so the set is complete.
#   untriaged   the 2026-09-28 reconciliation did not get to it. This tag
#               is the one that must go to zero; it is not a resting place.
#
# Do NOT add an entry to make this test green. Adding one is a claim
# that the module has no production caller -- which is the defect, not
# the fix.
_KNOWN_UNWIRED = {
    # ---- ruled-dead ----
    # Routes cmd/motion/behavior{goto} to path_follow(300). 12 v0.8
    # (#20-1/#20-9) REMOVED path_follow from the behavior source closed
    # set; sources/arbiter_p1.py has no PATH_FOLLOW member any more. So
    # this returns a source that cannot exist, and its tests pin it.
    # Same shape as the r_eff max() test: green, and wrong to follow.
    "xbrain/p1_motion/route/behavior_goto.py": "ruled-dead: 12 v0.8 deleted path_follow",

    # ---- no-surface ----
    # Guards the PTZ LAPI WRITE key set. There is no LAPI client in
    # production -- p2_core/ptz/ speaks ONVIF only. A guard with no write
    # site to guard cannot fail, so BIZ-P2-10 is not actually met.
    "xbrain/p2_core/domains/lapi_guard.py": "no-surface: no LAPI client in production, ONVIF only",

    # ---- registered elsewhere ----
    # NEXT S5 SW-23: its regex extracts 0 rows from 18 today, so the
    # CS-A1/CS-A2 two-way diff is empty-vs-empty and always passes.
    "xbrain/p4_agent/registry/cmdset_extractor.py": "registered: NEXT S5 SW-23",
    # NEXT S5 SW-21 UNIT class: implementation exists, zero consumers.
    "xbrain/p2_core/health/restrict_matrix.py": "registered: NEXT S5 SW-21 UNIT class",

    # ---- untriaged, HIGHEST PRIORITY TRANCHE ----
    # These five all name themselves a STARTUP GATE -- "refuse startup",
    # "refuse to boot", "runs at P4 startup, every one refuses process
    # start if it fails". None of them is called. A gate nobody invokes
    # and a gate that always passes are indistinguishable from the
    # outside, which is precisely CLAUDE.md S3.2 form 1 at process scale.
    # Triage each by asking what ACTUALLY refuses startup today, not
    # whether the module exists: for some the behaviour lives elsewhere
    # (a duplicate to delete), for others nothing does it at all.
    "xbrain/boot/failure_class.py": "untriaged: CFG-BT-14 startup failure classifier, never invoked",
    # refuse_to_boot.py left this registry on 2026-09-28: it is now called
    # from xbrain/boot/freeze/__main__.py, which catches the XbrainError an
    # assertion runner raises and renders the 10 S5.4.5 three-section
    # listing instead of letting a traceback out.
    "xbrain/common/zenoh/startup_selfcheck.py": "untriaged: INF-ZN-5 'refuse startup on unregistered keys' -- 11 S2.2 makes this mandatory and nothing runs it",
    "xbrain/common/zenoh/cross_plane_compliance.py": "untriaged: INF-ZN-9 cross-plane forwarding compliance, never invoked",
    # ---- untriaged (2026-09-28 batch D did not reach these) ----
    "xbrain/common/checks/scan_surface.py": "untriaged",
    "xbrain/common/config/locked_keys.py": "untriaged",
    "xbrain/common/enums/cls_permissive.py": "untriaged",
    "xbrain/p1_motion/config/hot_update.py": "untriaged",
    "xbrain/p1_motion/config/zenoh_planes.py": "untriaged",
    "xbrain/p1_motion/gate/negative_vx.py": "untriaged",
    "xbrain/p1_motion/path/pose_assembly.py": "untriaged",
    "xbrain/p1_motion/path/recording_lock.py": "untriaged",
    "xbrain/p1_motion/profile/switch_sm.py": "untriaged",
    "xbrain/p1_motion/rotation/visual_override.py": "untriaged",
    "xbrain/p1_motion/rt_base/rtc.py": "untriaged",
    "xbrain/p1_motion/teleop/four_source.py": "untriaged",
    "xbrain/p2_core/domains/lighting_auto.py": "untriaged",
    "xbrain/p2_core/health/aux/ir_camera.py": "untriaged",
    "xbrain/p2_core/mode/device_map.py": "untriaged",
    "xbrain/p2_core/ptz/drift_home.py": "untriaged",
    "xbrain/p2_core/ptz/hold_ms_renewal.py": "untriaged",
    "xbrain/p3_task/route/suspend_resume.py": "untriaged",
    "xbrain/p4_agent/asr_post/correction_log.py": "untriaged",
    "xbrain/p4_agent/asr_post/three_layer.py": "untriaged",
    "xbrain/p4_agent/audio_rx/gate_observer.py": "untriaged",
    "xbrain/p4_agent/envelope/pose_snap.py": "untriaged",
    "xbrain/p4_agent/failsafe/rotation_reject.py": "untriaged",
    "xbrain/p4_agent/intents_expand/d01_d10.py": "untriaged",
    "xbrain/p4_agent/registry/d_class.py": "untriaged",
    "xbrain/p4_agent/registry/intents_check.py": "untriaged",
    "xbrain/p4_agent/registry/rulings_18b.py": "untriaged",
    # GWY-P4-08 CS-A1..A4: header says "every one refuses process start
    # if it fails", yet P4 never calls it. registry/intents.py has its OWN
    # CS-A1 implementation (check_intents_in_closed_set) -- so there are
    # TWO sources for the same assertion and only one runs. Triage = find
    # which is authoritative, delete the other.
    "xbrain/p4_agent/registry/startup_assertions.py": "untriaged: GWY-P4-08 CS-A* duplicate of registry/intents.py, neither wired from P4 startup",
    "xbrain/p4_agent/registry/time_expr.py": "untriaged",
    "xbrain/p4_agent/registry/tools_projection.py": "untriaged",
    "xbrain/p4_agent/session/level_routing.py": "untriaged",
    "xbrain/p4_agent/templates/restate_engine.py": "untriaged",
    "xbrain/p5_gateway/hmi/data_sets.py": "untriaged",
    "xbrain/p5_gateway/text_input.py": "untriaged",
}

# The tag vocabulary is closed on purpose: a free-text note field drifts
# into "TODO" within a month, and then the registry says nothing.
_LEGAL_TAGS = ("ruled-dead", "no-surface", "registered", "untriaged")


def _walk_py(root):
    """Yield every .py under root, skipping excluded directory parts."""
    base = os.path.join(_REPO, root)
    for dirpath, dirnames, filenames in os.walk(base):
        # Prune in place so os.walk does not descend into excluded trees.
        dirnames[:] = [d for d in dirnames if d not in _EXCLUDED_DIR_PARTS]
        if any(part in _EXCLUDED_DIR_PARTS for part in dirpath.split(os.sep)):
            continue
        for name in filenames:
            if name.endswith(".py"):
                yield os.path.join(dirpath, name)


def _rel(path):
    """Repo-relative path with forward slashes, so registry keys are stable."""
    return os.path.relpath(path, _REPO).replace(os.sep, "/")


def _read(path):
    """Read a source file; unreadable bytes must not take the door down."""
    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        return handle.read()


def _collect_unwired():
    """Return the set of repo-relative candidate modules that no other
    production file names.

    A candidate is any xbrain/ module file other than __init__.py -- the
    package files are naming plumbing, not units of behaviour, and they
    are reached by importing the package, so they are never 'unwired'.
    """
    candidates = {}
    for root in _CANDIDATE_ROOTS:
        for path in _walk_py(root):
            if os.path.basename(path) == "__init__.py":
                continue
            candidates[path] = os.path.basename(path)[:-3]

    corpus = []
    for root in _MENTION_ROOTS:
        for path in _walk_py(root):
            corpus.append((path, _read(path)))

    unwired = set()
    for path, stem in candidates.items():
        # Word boundaries so 'progress' does not satisfy 'progress_sink'
        # and vice versa; the stem is what both import forms spell.
        pattern = re.compile(r"\b" + re.escape(stem) + r"\b")
        if not any(pattern.search(text) for other, text in corpus if other != path):
            unwired.add(_rel(path))
    return unwired


def _surface_note():
    """The sentence every failure message carries. A count without its
    scan surface is not something a reader can act on."""
    return (
        "scan surface: candidates=%s (excluding __init__.py and %s), "
        "mentions=%s, tests/ deliberately NOT scanned"
        % (list(_CANDIDATE_ROOTS), list(_EXCLUDED_DIR_PARTS), list(_MENTION_ROOTS))
    )


def test_no_new_unwired_module_appears():
    """A module that nothing production-side names must be registered.

    This is the direction that catches a route/push.py being born: the
    file lands, its own tests are green, and this door names it the same
    day.
    """
    unwired = _collect_unwired()
    fresh = sorted(unwired - set(_KNOWN_UNWIRED))
    assert not fresh, (
        "these xbrain/ modules are named by NO other production file, so no "
        "test of them can tell you whether the system uses them:\n  %s\n"
        "Wire it, delete it, or add it to _KNOWN_UNWIRED WITH a reason "
        "(adding an entry is admitting the defect, not fixing it).\n%s"
        % ("\n  ".join(fresh), _surface_note())
    )


def test_registry_has_no_stale_entry():
    """The shrink direction: an entry that got wired, or a file that was
    deleted, must leave the registry in the same commit.

    Without this half the registry only grows, and a growing list of
    'known unwired' reads exactly like a clean report.
    """
    unwired = _collect_unwired()
    stale = sorted(set(_KNOWN_UNWIRED) - unwired)
    resolved = [p for p in stale if os.path.exists(os.path.join(_REPO, p))]
    gone = [p for p in stale if not os.path.exists(os.path.join(_REPO, p))]
    assert not stale, (
        "_KNOWN_UNWIRED is stale. Now wired (remove the entry): %s. "
        "No longer on disk (remove the entry): %s.\n%s"
        % (resolved, gone, _surface_note())
    )


def test_every_registry_entry_carries_a_legal_tag():
    """A note field with no vocabulary becomes 'TODO' and stops meaning
    anything. Each entry must open with one of the four tags, and the
    tag must be followed by a reason -- 'untriaged' is the only tag
    allowed to stand alone, and it is the one that must reach zero."""
    bad = []
    for path, note in sorted(_KNOWN_UNWIRED.items()):
        tag = note.split(":", 1)[0].strip()
        if tag not in _LEGAL_TAGS:
            bad.append((path, note, "tag %r not in %s" % (tag, list(_LEGAL_TAGS))))
            continue
        if tag != "untriaged" and ":" not in note:
            bad.append((path, note, "tag %r requires a reason after ':'" % tag))
    assert not bad, "registry entries with an unusable note: %s" % bad
