"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: alias_blacklist.py
Brief: CHK-2-25 alias blacklist (rot_occ_max_cells / fail_ticks) + CHK-2-47 closed-set string lint

Description:
Two static guards.

CHK-2-25: alias black-list for keys that were RENAMED during doc
close-out. The old names must not reappear in configs; the guard
lists them explicitly.

Renames:
  rot_occ_max_cells  -> rot_occ_max      (12 §6A U-2026-08-04)
  fail_ticks         -> recheck_ticks    (14 §7.1)

CHK-2-47: closed-set string lint. E_* codes are already lint-guarded;
this extends the pattern to other closed-set constants (domain
names, plane names, TaskState values). Consumer-side literal strings
of those tokens are refused; consumers must import from
common.closed_sets.
"""

from __future__ import annotations

import re


ALIAS_BLACKLIST = frozenset({
    "rot_occ_max_cells",   # renamed to rot_occ_max
    "fail_ticks",           # renamed to recheck_ticks
})


CLOSED_SET_TOKENS = frozenset({
    # domain names (7)
    "motion", "sensor", "ptz", "audio", "task", "network", "dock",
    # plane names (8)
    "gen", "rt_lo", "chassis", "ros2", "http", "ftp", "ws", "external",
    # task states are checked via schema constants elsewhere
})


class AliasKeyFound(Exception):
    """CHK-2-25: alias key still in use."""


class ClosedSetLiteralFound(Exception):
    """CHK-2-47: literal closed-set string outside common.closed_sets."""


def scan_config_for_alias(config: dict) -> None:
    """Recursive walk; any leaf key in ALIAS_BLACKLIST raises."""

    def _walk(obj, path=""):
        if isinstance(obj, dict):
            for k, v in obj.items():
                if k in ALIAS_BLACKLIST:
                    new_path = f"{path}.{k}" if path else k
                    raise AliasKeyFound(
                        f"alias key {k!r} at {new_path!r}")
                _walk(v, f"{path}.{k}" if path else k)
        elif isinstance(obj, list):
            for i, v in enumerate(obj):
                _walk(v, f"{path}[{i}]")

    _walk(config)


LITERAL_RE = re.compile(r'["\']([A-Za-z_]\w*)["\']')


def check_source_for_closed_set_literals(source: str,
                                            allowlist_functions=("import",)):
    """CHK-2-47: return a list of (line_no, token) hits where a
    literal closed-set token appears in a way that looks like a
    hardcoded string. Import lines are exempt (they always mention
    the token as a name)."""
    hits = []
    for lineno, line in enumerate(source.splitlines(), start=1):
        if any(fn in line for fn in allowlist_functions):
            continue
        for m in LITERAL_RE.finditer(line):
            token = m.group(1)
            if token in CLOSED_SET_TOKENS:
                hits.append((lineno, token))
    return hits
