"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: assertions.py
Brief: BIZ-P2-24 -- p2_core.yaml startup assertions (14 S11 + 10 S5.4.4)

Description:
Five specific consistency checks that run at Stage 0c (config-freeze)
after the generic assertion A (no null / no residual ${}) has passed.
Each check has its own function and its own raise site so a failure
message can name the exact rule.

  check_profile_admission_matches_common  (assertion C, 10 S5.4.4)
      keys(p2_core.health.profile_admission) MUST equal
      keys(common.motion.profiles). Adding a third profile in either
      file without the other means the arbiter's admission table and
      the speed system disagree at Stage 4 release.

  check_switch_order                       (14 S5.7 ML-5)
      p2_core.mode.switch_order MUST be exactly the ordered list
      [device_mode, payload_light, ptz, motion, audio]. Any reorder
      lets a later step overwrite the earlier one's effect (e.g., the
      /mode teardown resetting lights that were just set).

  check_mode_motion_behaviors             (14 S5.6.3 CFG-40)
      p2_core.mode_motion.d_alarm.behavior AND .b_cast.behavior MUST
      each be in the closed set {face_target_stop, face_target_follow,
      hold}. Out-of-set is E_CONFIG_INVALID, NEVER downgraded to a
      default (that would be CLAUDE.md 3.6 fail-silent).

  check_redblue_mode_matches              (14 S7.3.2)
      p2_core.d_mode.redblue_mode MUST equal
      p2_core.payload_light.deter_redblue_mode. Two truths would let a
      strobe fire at pattern X while the D-mode timer thinks Y.

  check_no_dead_profiles                  (14 S8.3, U33 U54)
      profile_admission MUST NOT contain 'cruise' or 'transit' (both
      deleted by U33). Presence -> E_CONFIG_INVALID.

  check_hot_update_whitelist               (14 S11 CFG-31)
      No key in p2_core.yaml is hot-updatable. This module owns the
      list of allowed hot files (currently ['suspicion_rules.yaml',
      'speech_presets.yaml']); the assertion checks that a claimed
      hot-updatable file is one of these and NOT p2_core.yaml.

Every check accepts the parsed dict tree and raises ConfigAssertError
on violation with a specific reason string. The caller (config-freeze)
maps the exception to E_CONFIG_INVALID and prints the reason.
"""

from __future__ import annotations

from typing import Dict, FrozenSet, Iterable, List, Optional


class ConfigAssertError(RuntimeError):
    """Raised by any assertion in this module.

    Carries the rule name, the observed value, and the expected form
    so config-freeze can render an actionable error line (rule,
    key path, seen vs expected).
    """

    def __init__(self, rule: str, message: str,
                 seen: object = None, expected: object = None) -> None:
        self.rule = rule
        self.seen = seen
        self.expected = expected
        super().__init__("[%s] %s" % (rule, message))


# --- Closed sets referenced by assertions --------------------------

# 14 S5.6.3: the ONLY three behaviors allowed for mode_motion.*.behavior.
_MOTION_BEHAVIORS: FrozenSet[str] = frozenset({
    "face_target_stop",
    "face_target_follow",
    "hold",
})

# 14 S5.7 ML-5 mandated ordering. This list is CONTRACT, not tunable.
_SWITCH_ORDER: List[str] = [
    "device_mode", "payload_light", "ptz", "motion", "audio",
]

# 14 S8.3 U33 / U54: deleted profile names that must NEVER re-appear
# in profile_admission (14 refuses their re-entry per assertion 8.3).
_DEAD_PROFILES: FrozenSet[str] = frozenset({"cruise", "transit"})

# 14 S11 CFG-31: only these config files are hot-reloadable.
# p2_core.yaml is deliberately NOT in this set.
_HOT_UPDATE_WHITELIST: FrozenSet[str] = frozenset({
    "suspicion_rules.yaml",
    "speech_presets.yaml",
})


# --- Per-rule assertion functions ---------------------------------

def check_profile_admission_matches_common(
    p2_core: dict, common: dict,
) -> None:
    """10 S5.4.4 assertion C: cross-file key equality.

    keys(p2_core.health.profile_admission) == keys(common.motion.profiles)
    """
    try:
        p2_keys = set(p2_core["health"]["profile_admission"].keys())
    except (KeyError, TypeError, AttributeError) as exc:
        raise ConfigAssertError(
            "assertion_C_profile_admission",
            "p2_core.health.profile_admission is missing or not a mapping: %s"
            % exc,
        ) from exc
    try:
        common_keys = set(common["motion"]["profiles"].keys())
    except (KeyError, TypeError, AttributeError) as exc:
        raise ConfigAssertError(
            "assertion_C_profile_admission",
            "common.motion.profiles is missing or not a mapping: %s" % exc,
        ) from exc
    if p2_keys != common_keys:
        raise ConfigAssertError(
            "assertion_C_profile_admission",
            "profile_admission keys do not match common.motion.profiles: "
            "p2=%s common=%s (symmetric diff: %s)"
            % (sorted(p2_keys), sorted(common_keys),
               sorted(p2_keys ^ common_keys)),
            seen=sorted(p2_keys), expected=sorted(common_keys),
        )


def check_switch_order(p2_core: dict) -> None:
    """14 S5.7 ML-5: mode.switch_order MUST be exactly the 5-value ordered list.

    Any reorder allows a later stage to overwrite an earlier one's
    effect (e.g., POST /mode after payload_light resets the lights).
    """
    try:
        got = p2_core["mode"]["switch_order"]
    except (KeyError, TypeError) as exc:
        raise ConfigAssertError(
            "switch_order",
            "mode.switch_order missing: %s" % exc,
        ) from exc
    if not isinstance(got, list):
        raise ConfigAssertError(
            "switch_order",
            "mode.switch_order must be a list, got %s" % type(got).__name__,
            seen=got, expected=_SWITCH_ORDER,
        )
    if got != _SWITCH_ORDER:
        raise ConfigAssertError(
            "switch_order",
            "mode.switch_order must be exactly %s (order matters); got %s"
            % (_SWITCH_ORDER, got),
            seen=got, expected=_SWITCH_ORDER,
        )


def check_mode_motion_behaviors(p2_core: dict) -> None:
    """14 S5.6.3 CFG-40: mode_motion.{d_alarm,b_cast}.behavior in closed set."""
    try:
        block = p2_core["mode_motion"]
    except (KeyError, TypeError) as exc:
        raise ConfigAssertError(
            "mode_motion_behavior",
            "mode_motion block missing: %s" % exc,
        ) from exc
    for mode_key in ("d_alarm", "b_cast"):
        try:
            behavior = block[mode_key]["behavior"]
        except (KeyError, TypeError) as exc:
            raise ConfigAssertError(
                "mode_motion_behavior",
                "mode_motion.%s.behavior missing: %s" % (mode_key, exc),
            ) from exc
        if behavior not in _MOTION_BEHAVIORS:
            raise ConfigAssertError(
                "mode_motion_behavior",
                "mode_motion.%s.behavior=%r not in closed set %s"
                % (mode_key, behavior, sorted(_MOTION_BEHAVIORS)),
                seen=behavior, expected=sorted(_MOTION_BEHAVIORS),
            )


def check_redblue_mode_matches(p2_core: dict) -> None:
    """14 S7.3.2: d_mode.redblue_mode == payload_light.deter_redblue_mode."""
    try:
        d = p2_core["d_mode"]["redblue_mode"]
        p = (p2_core["arbiter"]["domains"]["payload_light"]
             ["deter_redblue_mode"])
    except (KeyError, TypeError) as exc:
        raise ConfigAssertError(
            "redblue_mode_consistency",
            "redblue_mode fields missing: %s" % exc,
        ) from exc
    if d != p:
        raise ConfigAssertError(
            "redblue_mode_consistency",
            "d_mode.redblue_mode=%r != payload_light.deter_redblue_mode=%r "
            "(two truths for the same strobe pattern)" % (d, p),
            seen={"d_mode": d, "payload_light": p}, expected="equal",
        )


def check_no_dead_profiles(p2_core: dict) -> None:
    """14 S8.3 / U33 / U54: profile_admission MUST NOT contain deleted names."""
    try:
        pa = p2_core["health"]["profile_admission"]
    except (KeyError, TypeError):
        # covered by check_profile_admission_matches_common; not this rule.
        return
    if not isinstance(pa, dict):
        return
    dead = set(pa.keys()) & _DEAD_PROFILES
    if dead:
        raise ConfigAssertError(
            "no_dead_profiles",
            "profile_admission contains U33-deleted profile(s) %s "
            "(cruise / transit were removed and MUST NOT re-appear)"
            % sorted(dead),
            seen=sorted(dead),
            expected="none of %s" % sorted(_DEAD_PROFILES),
        )


def check_hot_update_whitelist(claimed_hot_files: Iterable[str]) -> None:
    """14 S11 CFG-31: only these files are hot-updatable.

    claimed_hot_files is the union of paths the runtime's hot-update
    machinery is willing to reload. The whitelist is
      {suspicion_rules.yaml, speech_presets.yaml}.
    Any other path here is a defect (typically: someone added
    'p2_core.yaml' hoping to hot-tune a knob that isn't safe to reload).
    """
    bad = []
    for path in claimed_hot_files:
        # allow full paths -- just check the basename.
        import os
        name = os.path.basename(path)
        if name not in _HOT_UPDATE_WHITELIST:
            bad.append(name)
    if bad:
        raise ConfigAssertError(
            "hot_update_whitelist",
            "files claimed hot-updatable but not on whitelist: %s "
            "(whitelist is %s)"
            % (bad, sorted(_HOT_UPDATE_WHITELIST)),
            seen=bad, expected=sorted(_HOT_UPDATE_WHITELIST),
        )


# --- Convenience: run every assertion --------------------------------

def check_all(p2_core: dict, common: dict,
              claimed_hot_files: Optional[Iterable[str]] = None) -> None:
    """Run every BIZ-P2-24 assertion in order. Raises on first failure.

    Order is deterministic: assertion C first (cross-file), then
    per-file rules. A single ConfigAssertError names which rule failed
    so config-freeze can print a targeted diagnosis."""
    check_profile_admission_matches_common(p2_core, common)
    check_switch_order(p2_core)
    check_mode_motion_behaviors(p2_core)
    check_redblue_mode_matches(p2_core)
    check_no_dead_profiles(p2_core)
    if claimed_hot_files is not None:
        check_hot_update_whitelist(claimed_hot_files)
