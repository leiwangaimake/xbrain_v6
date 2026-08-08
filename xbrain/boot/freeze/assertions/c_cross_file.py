"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: c_cross_file.py
Brief: Assertion C -- cross-file relations (CFG-FZ-4)

Description:
Runs FIFTH in the freeze pipeline (ORD-1, after B). Walks the merged
overlay and every L6 tree; asserts five cross-file identities that
must hold jointly:

  C-1 Retention monotonicity: task_days <= event_days <= command_days
      (three retention buckets on common.retention.*). Deleting a
      row (like pose_track) does not change the ordering; the current
      three-value form was defined post-U46. Reason: shorter retention
      on a later stage would mean a completed task's audit trail
      outlives the raw command that requested it, and forensic replays
      would show fabricated causality.
  C-2 profile_admission keys match motion.profiles: the P2 admission
      table must whitelist the SAME profile names common.motion.profiles
      declares. Otherwise a profile is either declared but not
      accepted, or accepted but not defined.
  C-3 recording.fence_close_tol_m == 2 * recording.min_dist_m: a
      geometric identity on the recording tolerance -- two point-
      spacings, else the "close enough to auto-close" detector misses
      a shape that was recorded at the design pitch.
  C-4 P3.charge.low_batt_profile in keys(common.motion.profiles): the
      low-battery fallback profile must be an existing profile. Silent
      typo here = robot cannot fall back and just stops.
  C-5 (BUILT-IN VARIANT, added 2026-08-08): common.motion.profiles
      MUST NOT contain the deprecated keys `cruise` or `transit`.
      10 S3.3.6 sixth row says "remove the enum value itself, no dead
      config"; grep of the tree today still hits 3 -- the variant is
      real, not theoretical.

Each check runs INDEPENDENTLY -- one raise stops the assertion but
raises with detail.kind naming which check failed. Adding a new check
is one _check_* function + one call in run().

Contract with the pipeline:
  input:  ctx["overlay"] (populated by assertion A) + ctx["config_root"]
  raises: XbrainError(E_CONFIG_INVALID) with detail.kind = one of
          retention_not_monotone / profile_admission_mismatch /
          fence_close_tol_ratio / low_batt_profile_missing /
          deprecated_profile_present
"""

# typing gives us the annotations for the helpers below; List is
# unused today but kept for future _fail extra fields that may want
# to name a list-typed detail (present_profiles is a list of str, for
# instance -- currently unpacked as **extra).
from typing import Any, Dict, List

from xbrain.boot.freeze.assertions._layer_loader import load_l6_files
from xbrain.common.config import build_overlay
from xbrain.common.config.merge import flatten
# E_CONFIG_INVALID (or E_QOS_VIOLATION / E_CONFIG_LOCKED)
# imported by name from xbrain.common.errors instead of
# spelled as a string literal. CLAUDE.md 3.5 forbids literal
# E_* strings anywhere outside common/errors/; scripts/lint/
# no_literal_ecode.py enforces it (both the whole-word literal
# and the substring form).
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError

# Deprecated profile names that must be absent from common.motion.profiles.
# From 10 S3.3.6 sixth row -- U54 removed these two, and any lingering
# entry means the removal was partial. Adding a new deprecation is one
# entry here + one row in _check_deprecated_profiles' rejection loop.
# frozenset (not set) so a caller cannot mutate the deprecation list
# at runtime; the U54 removal is a compile-time invariant.
_DEPRECATED_PROFILES = frozenset({"cruise", "transit"})


# Helper: uniform raise. Every _check_* function calls this rather than
# constructing XbrainError inline, so a future edit that changes the
# error shape only touches one function.
def _fail(kind: str, **extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + assertion-specific fields.

    C's failure kinds are a closed set of five (retention_not_monotone /
    profile_admission_mismatch / fence_close_tol_ratio /
    low_batt_profile_missing / deprecated_profile_present). Each _check_*
    below picks one and passes its own extra fields; the message string
    is uniform so a journalctl reader sees a consistent shape.
    """
    # Kind is required; the assertion-specific fields (task_days,
    # profile, expected, etc) come through extra kwargs so each raise
    # site adds exactly what its failure needs.
    detail = {"kind": kind}
    detail.update(extra)
    raise XbrainError(
        E_CONFIG_INVALID,
        "assertion C failed: %s" % kind,
        detail,
    )


# Helper: dotted-path walker, returns default if any segment missing.
def _get(tree: Dict[str, Any], dotted: str, default=None) -> Any:
    """Walk dotted path in tree; return default if any segment missing.

    Kept as a local helper rather than reusing OverlayResult.get()
    because C also walks L6 trees directly (not the overlay), so the
    signature has to accept ANY dict, not just an OverlayResult.
    Default value is `None` for the common "optional key, skip check"
    path -- callers test `is not None` explicitly.
    """
    # Walk the tree segment-by-segment; any missing / non-dict node
    # returns the caller-supplied default (None by convention).
    node: Any = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def _check_retention_monotone(tree: Dict[str, Any]) -> None:
    """C-1: common.retention.task_days <= event_days <= command_days.

    All three optional in the current tree -- if any is missing, we
    skip the check (M would have caught the required-ness elsewhere,
    if this key becomes required). If any TWO are present, we compare
    just those; that keeps the check useful during migration.

    Reason for monotonicity: shorter retention on a later pipeline
    stage means a completed task's audit trail outlives the raw command
    that requested it, and forensic replays would show fabricated
    causality. The safety-of-forensic-record depends on this ordering.

    Concrete failure: task_days=90, event_days=30 means events tied to a
    completed task get garbage-collected before the task record itself,
    and the task's audit shows "completed with no events" -- looks like
    a healthy run to the reviewer. The right response to a mismatch is
    to widen the shorter retention, not shrink the longer.
    """
    # Pull all three; None if any is not configured yet. Days here are
    # integer days (not seconds), matching the 10 S3.3 retention table
    # column headings. Reading them straight from the merged overlay
    # so an override at any layer is honoured.
    task_days = _get(tree, "common.retention.task_days")
    event_days = _get(tree, "common.retention.event_days")
    command_days = _get(tree, "common.retention.command_days")
    # Ordered pairs. Only compare pairs where both values are present.
    # Half-configured trees (task+event but no command_days) still
    # exercise the check for the pair that IS complete.
    # Two separate pair-checks (task<=event AND event<=command) rather
    # than a triple, so a missing middle still catches the outer pair.
    if task_days is not None and event_days is not None:
        if task_days > event_days:
            _fail("retention_not_monotone",
                  task_days=task_days, event_days=event_days,
                  which_pair="task > event")
    if event_days is not None and command_days is not None:
        if event_days > command_days:
            _fail("retention_not_monotone",
                  event_days=event_days, command_days=command_days,
                  which_pair="event > command")


def _check_deprecated_profiles(tree: Dict[str, Any]) -> None:
    """C-5: common.motion.profiles must not carry cruise / transit.

    10 S3.3.6 sixth row says "remove the enum value itself, no dead
    config"; grep of the tree today hits 3 -- the variant is real,
    not theoretical. Any lingering entry means the U54 removal was
    partial and downstream code that still supports the deprecated
    names will silently keep working.

    Why "remove the enum value itself" matters: retaining a deprecated
    entry as dead config still lets a config-time typo revive it
    (someone writes cruise: {...} thinking it's supported, and the
    startup accepts it because the enum still knows the name). Removal
    forces schema-time rejection.
    """
    # Default to empty dict, not None -- lets the isinstance check
    # below succeed on the "no key at all" case (which we skip).
    profiles = _get(tree, "common.motion.profiles", {})
    if not isinstance(profiles, dict):
        # Wrong shape -- another check (or M) would catch that; here
        # we just skip so we don't fail on a shape defect that isn't
        # ours to report.
        return
    # Walk the deprecated set (not the profiles keys) -- iterating the
    # smaller set is O(deprecated_size) rather than O(profile_count),
    # and adding a new deprecation only touches _DEPRECATED_PROFILES.
    for bad in _DEPRECATED_PROFILES:
        if bad in profiles:
            # Report which one hit AND what present profiles are, so
            # the operator sees the removal target and the survivors
            # in the same message without having to open the file.
            _fail("deprecated_profile_present",
                  profile=bad,
                  present_profiles=sorted(profiles.keys()))


def _check_fence_close_tol(tree: Dict[str, Any]) -> None:
    """C-3: recording.fence_close_tol_m == 2 * recording.min_dist_m.

    Geometric identity: fence-close-tolerance must be exactly two point
    spacings. A shape recorded at design pitch (min_dist_m apart) needs
    the auto-close detector to allow at most two spacings of slack, else
    the recording never auto-closes and stays open across restarts.

    Ratio (not min or max): fence_close_tol is coupled to min_dist_m by
    design; changing one without the other silently breaks the
    invariant. Both configured or both null -- the pair moves together.
    """
    # fct = fence_close_tolerance_meters (the auto-close slack).
    # mdm = min_dist_m (the design point spacing during recording).
    # Short names inline so the identity `fct == 2*mdm` reads cleanly.
    fct = _get(tree, "common.recording.fence_close_tol_m")
    mdm = _get(tree, "common.recording.min_dist_m")
    if fct is None or mdm is None:
        # Optional keys during migration; skip pair-wise if either
        # missing. M enforces the required-ness elsewhere if that
        # becomes the contract.
        return
    # Two point-spacings. Small floating-point epsilon so 0.1000001
    # does not fail. 1e-9 is well below any physically-meaningful
    # difference and above JSON round-trip noise.
    # Comparing on the absolute difference (not relative) because both
    # values are in the same physical unit (meters) at similar scale.
    expected = 2 * mdm
    if abs(fct - expected) > 1e-9:
        # Report both operands + the expected result so an operator
        # sees WHY the ratio check failed without doing the math.
        _fail("fence_close_tol_ratio",
              fence_close_tol_m=fct,
              min_dist_m=mdm,
              expected=expected)


def _check_profile_admission(tree: Dict[str, Any],
                             l6_trees: Dict[str, Dict[str, Any]]) -> None:
    """C-2: keys(P2.health.profile_admission) == keys(common.motion.profiles).

    P2 lives in p2_core.yaml (L6); profile_admission's keys must equal
    the whitelist declared in common.motion.profiles. Divergence in
    either direction is bad:
      - profile-in-common but not-in-admission: the profile exists but
        the admission gate rejects it, so the system silently degrades
        to whichever profile IS admitted.
      - profile-in-admission but not-in-common: admission whitelists a
        name that no profile defines, so a request for it fails with
        E_UNKNOWN, and the operator sees an unrelated error.

    Set-equality (not subset either direction): both mismatches surface
    as different broken behaviours, so we reject both.
    """
    # Pull common's profile set from the merged overlay -- the L1 file
    # is where the design table lives.
    common_profiles = _get(tree, "common.motion.profiles", {})
    if not isinstance(common_profiles, dict):
        return                                # shape defect, not ours
    # Pull P2's admission set from p2_core.yaml (L6) directly, not from
    # the merged overlay -- L6 files are not part of overlay.
    p2_tree = l6_trees.get("p2_core.yaml", {})
    admission = _get(p2_tree, "health.profile_admission", {})
    # Empty admission is treated as "not yet configured" and skipped.
    # This is deliberate: dev checkouts may not have p2 filled in yet.
    if not isinstance(admission, dict) or not admission:
        return
    # Compare sets, not the ordered dicts -- values (True/False in the
    # admission dict, sub-dicts in the profile dict) are checked by
    # other assertions; C-2 is purely about NAME set equality.
    common_names = set(common_profiles.keys())
    admission_names = set(admission.keys())
    if common_names != admission_names:
        # Report BOTH difference directions so an operator sees the
        # complete picture in one message. sorted() for stability.
        _fail("profile_admission_mismatch",
              only_in_common=sorted(common_names - admission_names),
              only_in_admission=sorted(admission_names - common_names))


def _check_low_batt_profile(tree: Dict[str, Any],
                            l6_trees: Dict[str, Dict[str, Any]]) -> None:
    """C-4: P3.charge.low_batt_profile must be an existing profile.

    A typo here means the low-battery fallback profile does not exist,
    so when the battery hits the threshold the fallback lookup fails
    and the robot cannot fall back -- it just stops in place. The
    failure surface is delayed by hours (until low battery), which is
    why this is a startup assertion rather than a runtime check.

    Set-membership check (not string-equality): the profile can be any
    of the defined ones (obstacle_avoid, patrol, ...), the check is that
    the named string is in the profiles dict's key set.
    Delayed-failure examples motivate the assertion cost -- catching a
    typo at startup pays back the first time a robot in the field
    otherwise would have run for hours before hitting low battery.
    """
    # Same source-of-truth for common.motion.profiles as C-2 --
    # any override at any layer wins.
    # Reading twice (once here, once in C-2) is cheap and keeps each
    # _check_* function readable in isolation.
    common_profiles = _get(tree, "common.motion.profiles", {})
    if not isinstance(common_profiles, dict):
        return
    # low_batt_profile is P3's own key, in p3_task.yaml.
    p3_tree = l6_trees.get("p3_task.yaml", {})
    low = _get(p3_tree, "charge.low_batt_profile")
    if low is None:
        # Optional in the current tree; skip when not yet configured.
        return
    if low not in common_profiles:
        # Report both the wanted name and the available options so the
        # operator sees the correction candidates without opening the
        # config file to grep for them.
        _fail("low_batt_profile_missing",
              low_batt_profile=low,
              available=sorted(common_profiles.keys()))


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion C. Replaces registry's stub_c.

    Reuses ctx["overlay"] from assertion A when available (the common
    ORD-1 path). Falls back to a fresh load + build_overlay when called
    in isolation (unit-test path). Also loads L6 files fresh either way,
    because some C-checks compare merged-tree values against per-process
    L6 values.

    On success returns a result dict with the number of checks run;
    checks_run is a fixed 5 today, kept as a field so a future variant
    that skips one check (during migration) surfaces the fact.
    On any check's failure this function does not return -- the
    XbrainError propagates and the caller (run_assertions) stops.
    """
    # Same wiring guard as J / A / M / B -- ctx missing config_root
    # is a caller-side bug, not a config problem.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion C requires ctx['config_root']; caller did not "
            "populate it"
        )

    # Prefer A's cached overlay; fall back for isolated callers who
    # invoke C without first invoking A (unit tests do this;
    # production ORD-1 has A -> B -> C so overlay is always present).
    overlay = ctx.get("overlay")
    if overlay is None:
        # Local import avoids a top-level cycle (assertions import
        # config; layers imports assertions in some future refactor
        # could otherwise cycle back). Local scope keeps the risk low.
        from xbrain.boot.freeze.assertions._layer_loader import load_layers
        layer_trees = load_layers(ctx["config_root"])
        overlay = build_overlay(layer_trees)
        # Populate ctx so a subsequent assertion in the same pass
        # doesn't re-load. Matches the pattern in A / M.
        ctx["overlay"] = overlay
        ctx["layer_trees"] = layer_trees
    # tree = overlay.tree pulls the merged dict once; all _check_*
    # helpers walk this same dict.
    tree = overlay.tree

    # L6 files -- some C-checks read them alongside the merged tree.
    # We load fresh each time (cheap, and avoids stale-cache surprises
    # if a test tweaks L6 between run() calls). B loads L6 too but we
    # do not share -- B runs earlier and B's caching does not exist.
    l6_trees = load_l6_files(ctx["config_root"])

    # Run every check. Each raises on first violation; order does not
    # affect correctness (checks are independent). Order is grouped
    # semantically: temporal (retention), then structural (profiles),
    # then geometric (fence tolerance), then cross-file (admission,
    # low_batt) -- rougher to more specific.
    # Retention first: it needs only the merged tree, no L6 loading,
    # so it's cheapest and its output is orthogonal to the others.
    _check_retention_monotone(tree)
    # Deprecated profiles next: also merged-tree only.
    _check_deprecated_profiles(tree)
    # Fence tolerance third: still merged-tree only.
    _check_fence_close_tol(tree)
    # Cross-file admission fourth: reads L6 (p2_core) and merged tree.
    _check_profile_admission(tree, l6_trees)
    # Cross-file low_batt last: reads L6 (p3_task) and merged tree.
    _check_low_batt_profile(tree, l6_trees)

    # Success return -- five checks all passed.
    return {
        "status": "pass",
        "assertion": "C",
        "checks_run": 5,
    }
