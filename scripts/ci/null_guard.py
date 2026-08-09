#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: null_guard.py
Brief: INF-DB-2 -- guard that keys pending debt-item closure stay null

Description:
21 (debt register) rows list per-item DEFAULT BEHAVIOUR while the debt is
unclosed. For many rows the default is "the corresponding config key
must stay null until the item is closed" -- filling the key in before
the underlying measurement / vendor answer lands is a silent violation
of the debt contract.

This script scans configs/**/*.yaml, checks each guarded key, and
fails (exit 1) with a report naming the debt item that owns the key
if the key is filled with a non-null value.

Key list assembly (INF-DB-2 verbatim: "auto-extract, NOT hand copy"):

  * PRIMARY: regex-scan 21 rows for the pattern
      `common.<dotted.path>` <null-verb> null
    where <null-verb> in {置 / 恒 / 一律 / 留 / 保持}.
    Captures the {debt_id, key_path, verb} triple.
  * SECONDARY (_EXTRA_KEYS): explicit fallback for keys the doc
    names but not in the auto-extract pattern (e.g. keys with
    prose like "字段缺省" that regex cannot cleanly capture).
    Each _EXTRA_KEYS entry MUST cite the 21 row it references
    (debt_id + doc anchor) so a future auto-extract improvement
    can shrink the list.

PTZ scope note (per user 2026-08-09):
  V6 uses PTZ dome camera as REMOTE-MANUAL ONLY. All autonomous
  PTZ commands are already rejected at INF-DB-3 (E02/E03/E04/E10
  return E_CAPABILITY unconditionally). PTZ config keys therefore
  do not need null-guard here -- the whole functional layer is
  permanently deferred at a higher level. PTZ-owned rows in 21
  (T-PTZ-1 / T-PTZ-3 / M-PTZ-1 / etc.) are EXPLICITLY skipped in
  _PTZ_EXEMPT_DEBT_IDS below.

Usage:
  python3 scripts/ci/null_guard.py               # scan, exit 1 on any non-null
  python3 scripts/ci/null_guard.py --self-test   # inject non-null, verify red
  python3 scripts/ci/null_guard.py -v            # verbose: print every guarded key
"""

import argparse
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import yaml


# The 21 doc path relative to the repo root.
_DEBT_DOC_REL = "docs/21-实测与第三方欠账.md"

# Config root scanned for filled keys.
_CONFIGS_ROOT_REL = "configs"

# User 2026-08-09 correction: PTZ is NOT manual-only. Cloud voice /
# text / local voice can control PTZ via 18 intent set:
#   E01 ptz_move_speed          three-tier closed-set speed  (supported)
#   E09 set_ptz_speed           closed-set speed name         (supported)
#   E02 ptz_home                needs preset_effective        (INF-DB-3 rejects)
#   E03 ptz_preset (goto)       needs preset_effective        (INF-DB-3 rejects)
#   E04 ptz_track               needs preset_effective        (INF-DB-3 rejects)
#   E10 ptz_move_deg (degrees)  needs omega calibration       (INF-DB-3 rejects)
#
# For the four rejected intents, the ONLY way to unblock them is to
# CLOSE T-PTZ-1 (real preset_effective measurement) or T-PTZ-3 (real
# omega measurement). Until then, the corresponding config keys must
# stay null. INF-DB-2 (this file) is the guard that catches an
# operator filling a value before the measurement is recorded --
# which INF-DB-3 alone cannot catch (INF-DB-3 only rejects intents;
# it does not check whether the config claims to be calibrated).
#
# Therefore _PTZ_EXEMPT_DEBT_IDS is EMPTY: every PTZ debt gets the
# same null-guard treatment as V-01 spec keys. Left as a documented
# frozenset for symmetry with _CLOSED_DEBT_IDS + so a future exempt
# addition has an obvious place.
_PTZ_EXEMPT_DEBT_IDS: frozenset = frozenset()

# 21 rows whose 'must be null' claim is superseded by a later user
# decision that pinned a concrete value. Each entry MUST cite the
# closing decision (U-* number / commit) so a reviewer can verify
# the closure is real, not wishful.
#
# _CLOSED_DEBT_IDS is technical debt AGAINST 21 (the 21 row should
# mark itself closed); this list is the bridge until 21 catches up.
_CLOSED_DEBT_IDS: Dict[str, str] = {
    # M-01 default was 'common.safety.t_lat_s = null -> refuse start'.
    # U54 (see 99 §U) pinned t_lat_s = 0.4 as the safety default,
    # split into two half-second budgets (0.2 我方 + 0.2 底盘链路).
    # configs/safety/brake.yaml carries the value with PROVENANCE:
    # decided:U54. Guarding it here would misfire.
    "M-01": "closed by U54 (t_lat_s = 0.4 pinned as safety default)",
}

# Regex to pull the debt id from a row header. 21 rows start with
# `| ` then optional decoration then ``11` V-01`` style token.
_DEBT_ID_RE = re.compile(
    r"^\|[^`]*`([0-9]+|本|18-?[AB]?)`\s*\*\*(?P<id>[A-Z]-?[A-Z0-9-]+)\*\*"
)

# Regex to pull a null-guard claim from a row's default-behaviour
# text. Matches `` `common.foo.bar` `` followed loosely by a null
# verb. Deliberately loose on separator to accept
# `k_ms_per_deg` 恒 null / `omega` 恒 `null` / `a` 置 `null` etc.
_NULL_KEY_RE = re.compile(
    r"`(?P<key>[a-z_][a-z0-9_]*(?:\.[a-z_][a-z0-9_]*)+)`"
    r"[^`|]{0,40}?"
    r"(?:置|恒|一律|留|保持)"
    r"[^`|]{0,15}?"
    r"`?null`?"
)


def _load_debt_rows(doc_path: Path) -> List[Tuple[str, str]]:
    """Yield (debt_id, row_body) for each recognised debt row in 21."""
    text = doc_path.read_text(encoding="utf-8", errors="replace")
    rows: List[Tuple[str, str]] = []
    for line in text.splitlines():
        # Rows start with `| `; also filter out the header line.
        if not line.startswith("|"):
            continue
        m = _DEBT_ID_RE.match(line)
        if not m:
            continue
        debt_id = m.group("id")
        rows.append((debt_id, line))
    return rows


def _extract_null_keys(rows: List[Tuple[str, str]]) -> Dict[str, str]:
    """Return {key_path: debt_id} for keys the doc marks as
    'must stay null while <debt_id> is open'.

    PTZ-owned debts (_PTZ_EXEMPT_DEBT_IDS) are skipped; their keys
    are guarded at INF-DB-3 layer via E_CAPABILITY rejection.
    """
    out: Dict[str, str] = {}
    for debt_id, line in rows:
        if debt_id in _PTZ_EXEMPT_DEBT_IDS:
            continue
        if debt_id in _CLOSED_DEBT_IDS:
            # Debt closed by a later user decision; the key now has
            # a legit non-null value. Skip so the guard does not
            # spuriously fire on the pinned value.
            continue
        for m in _NULL_KEY_RE.finditer(line):
            key = m.group("key")
            # Only capture keys that start with 'common.' or 'ptz.' --
            # 'common.' is the shared config namespace, 'ptz.' is the
            # p2_core.yaml subsection for PTZ config (per CHK-1-07).
            # Other bare identifiers ('covered', 'sources') are prose,
            # not config keys.
            if not (key.startswith("common.") or key.startswith("ptz.")):
                continue
            out.setdefault(key, debt_id)
    return out


# Hand-maintained extras: keys the regex cannot cleanly capture from
# the doc but that ARE null-guarded per 21 prose. Each entry MUST
# cite the 21 row it references so a future auto-extract improvement
# can shrink this list to empty (which would be the win).
#
# Format: {key_path: (debt_id, reason)}.
_EXTRA_KEYS: Dict[str, Tuple[str, str]] = {
    # V-01 five spec limits. Regex catches these when the doc writes
    # them backtick-wrapped; kept here as belt-and-braces because the
    # doc's default-behaviour column varies per row (some list all
    # five, some just "五项 spec 全为 null").
    "common.spec.max_vx_mps":
        ("V-01", "vendor written max_v/wz/accel/decel spec pending"),
    "common.spec.max_vy_mps":
        ("V-01", "vendor written max_v/wz/accel/decel spec pending"),
    "common.spec.max_wz_radps":
        ("V-01", "vendor written max_v/wz/accel/decel spec pending"),
    "common.spec.max_accel_mps2":
        ("V-01", "vendor written max_v/wz/accel/decel spec pending"),
    "common.spec.max_decel_mps2":
        ("V-01", "vendor written max_v/wz/accel/decel spec pending"),
    # PTZ keys: user 2026-08-09 confirmed PTZ IS controlled by AI
    # (cloud voice / text / local voice via 18 intents E01/E09/E02/
    # E03/E04/E10). But calibration debts remain open:
    #   T-PTZ-1: preset_effective must stay null until measured
    #   T-PTZ-3: omega/speed calibration must stay null (bare 'omega'
    #            in 21 -- no common. prefix -- regex cannot capture)
    #   M-PTZ-1: k_ms_per_deg must stay null (belt-and-braces)
    # These keys live under p2_core.yaml's ptz.* section (per CHK-1-07
    # spec), not under common.*, so the auto-extract cannot see them.
    # Explicit here until the doc format converges or an operator
    # closes the debt via field measurement.
    "ptz.preset_effective":
        ("T-PTZ-1", "human-eye preset return verification pending"),
    "ptz.omega_pan":
        ("T-PTZ-3", "external-measured degrees-per-second calibration pending"),
    "ptz.omega_tilt":
        ("T-PTZ-3", "external-measured degrees-per-second calibration pending"),
    "ptz.k_ms_per_deg":
        ("M-PTZ-1", "pulse-duration <-> angle table per speed tier pending"),
}


def all_guarded_keys(doc_path: Path) -> Dict[str, str]:
    """Union of auto-extracted + _EXTRA_KEYS. Returns {key: debt_id}.

    Extras win on key collision (auto value is overridden with the
    hand-curated one so an operator reading the failure sees the
    exact debt row named).
    """
    rows = _load_debt_rows(doc_path)
    auto = _extract_null_keys(rows)
    for key, (debt_id, _reason) in _EXTRA_KEYS.items():
        auto[key] = debt_id
    return auto


def _dotted_walk(tree, prefix: str = "") -> Iterable[Tuple[str, object]]:
    """Yield (dotted_path, leaf_value) for every non-container leaf."""
    if isinstance(tree, dict):
        for k, v in tree.items():
            child = prefix + ("." if prefix else "") + str(k)
            if isinstance(v, (dict, list)):
                yield from _dotted_walk(v, child)
            else:
                yield child, v
    elif isinstance(tree, list):
        # A list-valued key is a leaf for the null-guard purpose;
        # if the key list is guarded, an empty list is still 'not
        # null' and would fire. Treat the list itself as the value.
        yield prefix, tree


def _scan_configs(configs_root: Path, guarded: Dict[str, str]
                  ) -> List[Tuple[str, str, str, object]]:
    """Return list of (yaml_path, key_path, debt_id, value) hits."""
    hits: List[Tuple[str, str, str, object]] = []
    for yaml_path in sorted(configs_root.rglob("*.yaml")):
        try:
            tree = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError):
            continue
        if tree is None:
            continue
        for key, value in _dotted_walk(tree):
            if key in guarded and value is not None:
                # PTZ keys under a broader schema: still exempt if
                # the debt_id is PTZ-owned (belt-and-braces vs auto
                # extract that has already dropped them).
                if guarded[key] in _PTZ_EXEMPT_DEBT_IDS:
                    continue
                hits.append((str(yaml_path), key, guarded[key], value))
    return hits


def _self_test() -> int:
    """Inject a non-null value on a known guarded key, verify caught.
    Uses two separate temp dirs so the 'clean' scan cannot see the
    'filled' fixture (the first-round bug this version fixes)."""
    import tempfile
    guarded = {
        "common.spec.max_vx_mps": "V-01",
        "common.safety.t_lat_s": "M-01",
    }
    tree_pass = {"common": {"spec": {"max_vx_mps": None},
                            "safety": {"t_lat_s": None}}}
    tree_fail = {"common": {"spec": {"max_vx_mps": 2.0},
                            "safety": {"t_lat_s": 0.4}}}
    # Two separate dirs: 'clean' has only the null fixture, 'filled'
    # has only the non-null fixture. Sharing a dir made the clean
    # scan see the filled file and spuriously fail.
    with tempfile.TemporaryDirectory() as clean_dir, \
            tempfile.TemporaryDirectory() as fill_dir:
        (Path(clean_dir) / "a.yaml").write_text(yaml.safe_dump(tree_pass))
        (Path(fill_dir) / "b.yaml").write_text(yaml.safe_dump(tree_fail))
        hits_clean = _scan_configs(Path(clean_dir), guarded)
        hits_fill = _scan_configs(Path(fill_dir), guarded)
    if hits_clean:
        print("self-test FAIL: clean yaml fired: %s" % hits_clean)
        return 1
    fired_keys = {h[1] for h in hits_fill}
    if "common.spec.max_vx_mps" not in fired_keys:
        print("self-test FAIL: max_vx_mps 2.0 not caught")
        return 1
    if "common.safety.t_lat_s" not in fired_keys:
        print("self-test FAIL: t_lat_s 0.4 not caught")
        return 1
    print("self-test PASS: null passes, non-null fires per key")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--doc", default=_DEBT_DOC_REL,
                    help="path to 21-实测与第三方欠账.md")
    ap.add_argument("--configs", default=_CONFIGS_ROOT_REL,
                    help="path to configs/ root")
    args = ap.parse_args()
    if args.self_test:
        return _self_test()
    doc = Path(args.doc)
    configs = Path(args.configs)
    if not doc.is_file():
        print("21 doc missing: %s" % doc)
        return 2
    if not configs.is_dir():
        print("configs root missing: %s" % configs)
        return 2
    guarded = all_guarded_keys(doc)
    print("scan surface: %s (guards %d keys pending debt closure)"
          % (configs, len(guarded)))
    if args.verbose:
        for key, debt in sorted(guarded.items()):
            print("  guarded: %s <- %s" % (key, debt))
    hits = _scan_configs(configs, guarded)
    for yaml_path, key, debt_id, value in hits:
        print("  BAD  %s: %s = %r (debt %s still open)"
              % (yaml_path, key, value, debt_id))
    print("  violations: %d" % len(hits))
    print("criterion: 0 (every guarded key must be null until "
          "its debt closes)")
    return 1 if hits else 0


if __name__ == "__main__":
    sys.exit(main())
