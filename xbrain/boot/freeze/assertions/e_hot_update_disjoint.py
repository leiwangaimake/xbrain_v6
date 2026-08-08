"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: e_hot_update_disjoint.py
Brief: Assertion E -- safety namespaces vs hot-update whitelist must be
       disjoint (CFG-FZ-5)

Description:
Runs SEVENTH in the freeze pipeline (ORD-1, after D). The assertion is
a pure set-intersection check between two closed sets:

  LEFT   = five safety namespaces from 10 S5.4.5 (common.safety.* /
           common.spec.* / common.motion.profiles / common.qos.* /
           common.fence.*)
  RIGHT  = hot-update whitelist from 11 S7.6 (5 scope names today:
           log_level / debug_flags / asr_dictionary / speech_presets /
           suspicion_rules)

If any entry on the whitelist matches a safety prefix, the whitelist
opened a hole in the fail-safe: an operator would be able to hot-swap
a safety parameter at runtime, bypassing the freeze-line assertions
that gate the value at startup. Freeze-line assertions exist to guard
what code cannot check after startup; a whitelist that included a
safety key would silently invalidate them.

Failure: E_CONFIG_LOCKED (not E_CONFIG_INVALID -- CLAUDE.md and 11
S7.6 both name this error code for hot-update violations).

CFG-FZ-5 variant verbatim: 'common.safety.brake.a_mps2 added to the
whitelist' -> E must go red. Test constructs the mutant whitelist by
extending the default one and asserts the raise.

Where the whitelist comes from:
  Today, the whitelist is a code-defined constant in 11 (there is no
  config file that carries it as data; that would create a chicken-
  and-egg problem where the whitelist itself needs freezing). In this
  module the whitelist is exposed as _DEFAULT_HOT_UPDATE_WHITELIST
  and can be overridden via ctx['hot_update_whitelist'] for testing.
  When the real 11 whitelist grows, the constant here updates in
  lockstep -- one place.
"""

# typing only; E is code-only, no external deps besides the base error.
# Freeze runs before third-party imports are guaranteed installed, so
# keeping E stdlib-only makes it safe to run at the earliest stage.
from typing import Any, Dict, FrozenSet, Iterable

from xbrain.common.errors.exceptions import XbrainError

# Safety namespaces from 10 S5.4.5 (five groups). Each entry is a
# dotted prefix; membership is by exact match OR prefix-with-dot.
# common.motion.profiles is a subtree name (no trailing dot) since
# the whole subtree is safety-relevant, not just leaves under it.
_SAFETY_NAMESPACES: FrozenSet[str] = frozenset({
    # Vehicle static spec: max_vx_mps / max_vy_mps / max_wz_radps /
    # max_accel_mps2 / max_decel_mps2. Any change requires re-cert.
    "common.spec",
    # Safety timing + distances: t_lat_s / d_safe_m / brake.* .
    # Hot-swappable brake config = runtime speed-gate change =
    # unbounded slip distance. This is the CFG-FZ-5 variant target.
    "common.safety",
    # Speed-profile table (obstacle_avoid / patrol). Hot-swapping
    # profile speeds bypasses the 10 S3.3.6 removal of cruise/transit.
    "common.motion.profiles",
    # Zenoh QoS table. Hot-swap = runtime prio/queue change = missed
    # deadlines on the RT plane.
    "common.qos",
    # Fence geometry margins. Hot-swap = shrinking a fence while a
    # robot is inside it = silent unsafe-state entry.
    "common.fence",
})

# Hot-update whitelist from 11 S7.6. The five scope names currently
# authorised for cmd/config to touch at runtime. Adding a new scope
# here without checking it against _SAFETY_NAMESPACES silently widens
# what config can be changed at runtime.
_DEFAULT_HOT_UPDATE_WHITELIST: FrozenSet[str] = frozenset({
    # Change per-process log verbosity without a restart.
    "log_level",
    # Toggle debug flags (perception ROI overlays, etc.).
    "debug_flags",
    # Update ASR keyword dictionary from headquarters.
    "asr_dictionary",
    # Update speech presets (TTS voice, cadence).
    "speech_presets",
    # Update the suspicion-rules table (P2 alarm generator).
    "suspicion_rules",
})


# Helper: exact-match OR prefix-with-dot check. Kept as a top-level
# function (not inline) so tests can exercise it directly for the
# near-miss cases (common.specifically must NOT match common.spec).
def _is_safety_entry(entry: str, safety_namespaces: Iterable[str]) -> bool:
    """True iff `entry` falls under any safety namespace.

    Match rules:
      - exact equality with a namespace (entry == "common.spec")
      - prefix-with-dot (entry starts with "common.spec.")
    Second form catches deeper keys like common.safety.brake.a_mps2;
    first form catches an entry equal to the namespace root itself.
    """
    # Walk each namespace; return True on first match, False if none.
    # Order does not matter (any hit is a hit).
    for ns in safety_namespaces:
        # exact match: the whole namespace as a scope name
        if entry == ns:
            return True
        # dotted-prefix match: a leaf under the namespace
        # The `+ "."` matters -- prefix without dot would false-flag
        # common.specifically against common.spec.
        if entry.startswith(ns + "."):
            return True
    return False


def _fail(kind: str, **extra: Any) -> None:
    """Raise E_CONFIG_LOCKED (not E_CONFIG_INVALID)."""
    detail = {"kind": kind}
    detail.update(extra)
    # E_CONFIG_LOCKED is the closed-set code for hot-update violations
    # (11 S13; the freeze-line assertion mirrors the runtime one).
    raise XbrainError(
        "E_CONFIG_LOCKED",
        "assertion E failed: %s" % kind,
        detail,
    )


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion E. Replaces registry's stub_e.

    Reads an optional ctx['hot_update_whitelist'] override so tests
    can inject the mutant whitelist without touching module state.
    The default whitelist is _DEFAULT_HOT_UPDATE_WHITELIST above.
    """
    # No config_root guard here -- E is a pure set-intersection check
    # over two closed sets that live in code. It does NOT read files.
    # (Callers may still populate ctx['config_root'] for consistency
    # with other assertions; we simply don't use it.)

    # Whitelist source: ctx override (for tests / future config drop)
    # falls back to the code-defined default.
    whitelist = ctx.get("hot_update_whitelist", _DEFAULT_HOT_UPDATE_WHITELIST)

    # Compute the intersection. sorted() so the failing entry list is
    # stable across runs -- an operator diffing two failure logs sees
    # only real changes, not frozenset iteration shuffles.
    violating = sorted(e for e in whitelist
                       if _is_safety_entry(e, _SAFETY_NAMESPACES))

    if violating:
        # Report ALL violating entries in one raise -- unlike A/B/C/D
        # where we stop on the first defect, E's failure surface is
        # "here's every scope you must remove", and giving them all at
        # once saves an operator from a whack-a-mole fix-cycle.
        _fail("safety_in_hot_update",
              entries=violating,
              whitelist_size=len(whitelist),
              safety_namespace_count=len(_SAFETY_NAMESPACES))

    # All clear: whitelist is disjoint from safety namespaces.
    return {
        "status": "pass",
        "assertion": "E",
        "whitelist_size": len(whitelist),
        "safety_namespace_count": len(_SAFETY_NAMESPACES),
    }
