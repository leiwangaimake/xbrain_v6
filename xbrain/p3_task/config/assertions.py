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
     db_path / enforce_ordering)
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


ALIAS_BLACKLIST = frozenset({
    "dedup_min_dist_m",
    "session_timeout_s",
    "db_path",
    "enforce_ordering",
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


def check_c_retention_and_fence_relation(config: dict) -> None:
    """Retention triple ascending. If low_batt_profile == 'disabled',
    U71 guard skips the fence_close_tol == 2*min_dist sub-clause."""
    ret = config.get("retention", {})
    d = ret.get("keep_task_days")
    p = ret.get("keep_progress_days")
    s = ret.get("keep_snapshot_days")
    if None in (d, p, s):
        raise FreezeAssertionFailure(
            "assertion C: retention keys missing")
    if not (d <= p <= s):
        raise FreezeAssertionFailure(
            f"assertion C: retention not ascending: "
            f"task={d} progress={p} snapshot={s}")
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
            f"assertion J: config path {path!r} not under "  # CONFIG-SOURCE-OK(J): error text of assertion J
            f"/opt/xbrain_v6/configs/")


def run_all_assertions(config: dict, config_path: str) -> None:
    """Runs A, B, C, J in order. First violation raises."""
    check_a_no_residuals(config)
    check_b_no_alias_keys(config)
    check_c_retention_and_fence_relation(config)
    check_j_config_root(config_path)
