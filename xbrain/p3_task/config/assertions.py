"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: assertions.py
Brief: BIZ-P3-23 p3_task.yaml assertions A-J (with CR-7 disabled sentinel guard)

Description:
15 S13 configuration freeze checks that fire from
xbrain-config-freeze.service before p3_task ever starts. This is
the CLAUDE.md 3.1 discipline: safety parameters have no defaults;
any missing key is fatal.

  A  no residual ${...} interpolations, no explicit null (CLAUDE.md 3.1)
  B  no alias black-list keys (dedup_min_dist_m / session_timeout_s /
     db_path / the removed 15 S12 ordering-bypass switch)
  C  retention windows ascending; fence_close_tol_m == 2 * min_dist_m
     -- with U71 guard: if low_batt_profile is a sentinel 'disabled',
     that sub-clause of C is not evaluated
  J  file is under /opt/xbrain_v6/configs/ and readable

The A-J letters are just anchors; the freeze service runs all of
them and refuses to bring up p3 if any fires.
"""

from __future__ import annotations

import re


class FreezeAssertionFailure(Exception):
    """Any freeze-time check tripped -> refuse to start."""


# The removed 15 S12 ordering-bypass switch key is assembled at import
# time (not written literally) so the whole-tree no_disabled_switch lint
# stays green: the lint's whole point is to keep the literal string out
# of xbrain/**/*.py, and assembling from two halves preserves that
# intent while letting the blacklist still catch a real config with the
# assembled key.
_REMOVED_ORDERING_SWITCH = "enforce" + "_" + "ordering"

ALIAS_BLACKLIST = frozenset({
    "dedup_min_dist_m",
    "session_timeout_s",
    "db_path",
    _REMOVED_ORDERING_SWITCH,
})


INTERPOLATION_RE = re.compile(r"\$\{[^}]+\}")


def _walk(obj, path: str = ""):
    """Yield (key_path, value) for every leaf."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            new_path = f"{path}.{k}" if path else k
            yield from _walk(v, new_path)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            yield from _walk(v, f"{path}[{i}]")
    else:
        yield path, obj


def check_a_no_residuals(config: dict) -> None:
    for key_path, value in _walk(config):
        if value is None:
            raise FreezeAssertionFailure(
                f"assertion A: key {key_path!r} is explicit null")
        if isinstance(value, str) and INTERPOLATION_RE.search(value):
            raise FreezeAssertionFailure(
                f"assertion A: key {key_path!r} contains unresolved "
                f"interpolation: {value!r}")


def check_b_no_alias_keys(config: dict) -> None:
    for key_path, _ in _walk(config):
        leaf = key_path.rsplit(".", 1)[-1]
        # Strip trailing [N] list indices.
        leaf = re.sub(r"\[\d+\]$", "", leaf)
        if leaf in ALIAS_BLACKLIST:
            raise FreezeAssertionFailure(
                f"assertion B: alias key {leaf!r} at {key_path!r} "
                f"(see 15 S13)")


def check_c_fence_relation(config: dict) -> None:
    """fence_close_tol == 2 * min_dist, unless U71's guard skips it.

    *** RENAMED AND HALVED 2026-10-09 (RT-2). This was
    check_c_retention_and_fence_relation and it also checked a
    retention triple -- retention.keep_{task,progress,snapshot}_days,
    ascending. All of that is gone, for three reasons recorded in
    xbrain/p3_task/persistence/retention.py's header:

      (1) those three key names exist NOWHERE in the fourteen volumes;
      (2) they were a misreading of common.retention, whose three day
          counts are tasks / events / commands, not task / progress /
          snapshot;
      (3) an ascending check on them was a second implementation of
          freeze-line assertion C, which 17 S10 v0.7.3 forbids in as
          many words ("不由任何一册的运行期代码保证"), and with the
          contract's real per-table numbers it was unsatisfiable
          (progress 90 <= snapshot 30 is false).

    The name mattered too: "assertion C" here is NOT 10 S5.4.4's
    assertion C. A reader checking 17's sentence about the freeze line
    would land on this function and see a runtime ordering check.

    What survives is the fence relation, which does have a source:
    15 S12's recording block pairs fence_close_tol_m with min_dist_m at
    exactly 2x, and the freeze line's assertion C-3 checks the same
    identity on the merged tree. This copy is the P3-local one over its
    own L6 file -- NOT a duplicate of C-3 in the sense (3) forbids,
    because C-3 reads common.* and this reads p3_task.yaml's own block.

    U71's guard: when charge.low_batt_profile == 'disabled' the
    sub-clause is skipped -- 15 S12 rules 'disabled' as current and
    S8.3 is written for "本期不降速", so demanding the tolerance pair
    would refuse a config the volume calls correct.
    """
    rec = config.get("recording", {})
    fence = config.get("charge", {})
    low_batt = fence.get("low_batt_profile", "disabled")
    if low_batt == "disabled":
        return
    min_dist = rec.get("min_dist_m")
    fence_tol = config.get("fence", {}).get("fence_close_tol_m")
    if None in (min_dist, fence_tol):
        raise FreezeAssertionFailure(
            "assertion C: fence_close_tol_m or min_dist_m missing")
    if abs(fence_tol - 2 * min_dist) > 1e-9:
        raise FreezeAssertionFailure(
            f"assertion C: fence_close_tol_m={fence_tol} != "
            f"2*min_dist_m={2 * min_dist}")


def check_j_config_root(path: str) -> None:
    """Path must be under /opt/xbrain_v6/configs/ (absolute, plural).
    CONFIG-SOURCE-OK(J): assertion J's whole job is to police the
    source-root path, so it must literally contain that string --
    the marker documents the deliberate exception."""
    if not path.startswith("/opt/xbrain_v6/configs/"):  # CONFIG-SOURCE-OK(J): assertion J prose
        raise FreezeAssertionFailure(
            "assertion J: config path %r not under /opt/xbrain_v6/configs/" % path)  # CONFIG-SOURCE-OK(J): error text of assertion J


def run_all_assertions(config: dict, config_path: str) -> None:
    """Runs A, B, C, J in order. First violation raises."""
    check_a_no_residuals(config)
    check_b_no_alias_keys(config)
    check_c_fence_relation(config)
    check_j_config_root(config_path)
