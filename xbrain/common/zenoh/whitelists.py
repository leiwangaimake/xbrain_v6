"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: whitelists.py
Brief: INF-ZN-7 / CFG-BT-17 -- cross-plane whitelists loaded from YAML

Description:
Five committed whitelists (PUB / SUB sets for perception, p1_motion,
chassis_relay, p2_core, p4_agent) are the frozen surface that 11 S1.1.6
defines. Prior versions of this module hard-coded them as literal
frozenset() bodies; CFG-BT-17 (99 U69 / user 方案β) requires the values
to live in a YAML data file and the code to only READ.

Data path:
  configs/generated/whitelist.yaml   authoritative values
  this module                        thin loader + module-level names

Rationale for the split:

  * Data drift is easier to review in a YAML diff than a Python diff.
  * scripts/doccheck/whitelist_gen.py can regenerate the yaml from
    doc source (WL-G1 for p2_core/p4_agent); the reload is a data
    change, not a code change.
  * The runtime import path (`from xbrain.common.zenoh.whitelists
    import P1_MOTION_PUB` etc.) stays byte-for-byte identical, so
    the rest of the tree does not need to change.

Freeze invariants:
  * PUB / SUB sets are frozenset -- consumers cannot mutate.
  * The loader raises at import time if the yaml is malformed
    or missing a required process. A silent empty set would be
    worse than an ImportError.

Two write paths, one truth (source_note in the yaml):
  perception / p1_motion / chassis_relay: HAND tables per 11 S1.1.6
      (small closed surface; RT-C3.b "逐条列出").
  p2_core / p4_agent:                     WL-G1 GENERATED from S2.2
      (25-30 keys per process; hand copy would silently lag).

Both flows land in the same yaml; the source_note field distinguishes.
The runtime does not care which flow produced any given entry.

Why yaml (CFG-BT-17 β / user decision):

  * A yaml diff shows exactly which keys moved without wading through
    a Python frozenset literal wrap; a reviewer can scan {pub added
    3, sub removed 1} in seconds.
  * The doccheck extractor script (whitelist_gen.py) writes the same
    yaml, so 'regenerate' becomes 'overwrite one file' rather than
    'edit N frozenset() blocks in a py file'.
  * A drift-gate test can diff the yaml against the doc's S2.2
    pub/sub columns without touching this loader at all.

Why NOT split by process (one yaml per process):

  * Keeping all five in one file lets a single reviewer see the cross-
    process picture (does p2_core PUB something no consumer SUBs?)
    without opening five files.
  * Frozen invariants live at the top-level of one document, easy
    to enforce with a single schema validator.

Not in scope for this file:
  * WL-G1 generation from doc source -- that's whitelist_gen.py.
  * Runtime enforcement of the whitelist -- that's each process's
    zenoh session factory (rejects a publish/subscribe on an
    unregistered key).
  * The DRIFT GATE (yaml vs doc) -- test_whitelist_gen.py owns it.
"""

# os retained for future path composition; currently pathlib
# handles everything. Kept in the import list to avoid an unused-
# import lint if a future test adds env-var overrides.
import os
from pathlib import Path
from typing import Dict, FrozenSet

# yaml is imported eagerly (not deferred inside _load_yaml) so a
# missing PyYAML surfaces at import time -- the earliest possible
# moment -- rather than the first runtime whitelist consult.
import yaml


# The authoritative data path. Kept as a Path relative to the repo
# root so a caller in a different cwd still reaches it.
# CONFIG-SOURCE-OK(freeze): This file's own path IS the source of
#   the whitelist data; it is not a general config-source read.
# Path lives under configs/generated/ because it is a machine-emitted
# artifact of scripts/doccheck/whitelist_gen.py, not a hand-authored
# config file. The 'generated' subdirectory is the convention for
# these emitted-then-committed artifacts.
_YAML_REL = "configs/generated/whitelist.yaml"

# Every process the yaml MUST cover. A missing process here would
# make its constant name (e.g. P2_CORE_PUB) undefined at import,
# which fails loudly rather than silently.
# The five processes are the closed set of cross-plane whitelist
# owners per 11 S1.1.6; adding a sixth would require doc changes
# to S1.1.6 first, not just this constant.
_REQUIRED_PROCESSES: FrozenSet[str] = frozenset({
    "perception", "p1_motion", "chassis_relay", "p2_core", "p4_agent",
})


def _repo_root() -> Path:
    """Repo root = grandparent of this file's package dir.

    Kept as a function (not a module constant) so tests can
    monkeypatch it to a tmp_path when exercising loader edge
    cases (missing / malformed yaml). A constant would be frozen
    at import time and untestable.
    """
    # __file__ is xbrain/common/zenoh/whitelists.py -> 3 up = repo.
    return Path(__file__).resolve().parents[3]


def _load_yaml() -> Dict[str, Dict[str, FrozenSet[str]]]:
    """Read and validate the whitelist YAML.

    Returns {process_name: {"pub": frozenset, "sub": frozenset}}.

    Raises ImportError-style RuntimeError if the yaml is missing or
    malformed. Called exactly once at module import time; runtime
    consumers only see the resulting module-level frozensets.

    Failure modes (each has its own error message so triage is
    unambiguous):
      * yaml file absent           -> "whitelist yaml missing: <path>"
      * yaml top-level not a dict  -> "missing top-level 'processes'"
      * processes not a mapping    -> "'processes' must be a mapping"
      * required process missing   -> "missing processes: [<list>]"
      * a process not a mapping    -> "<proc> not a mapping"
      * pub / sub not a list       -> "<proc> pub/sub must be lists"

    Deliberately no fallback to defaults: a broken yaml means the
    whitelist enforcement layer above cannot function, so refusing
    to start beats booting with silent zero-set whitelists (which
    would permit-all instead of deny-all).
    """
    # Compose the absolute path; _repo_root() is a function call so
    # a monkeypatch in tests takes effect without touching this line.
    path = _repo_root() / _YAML_REL
    # Missing yaml is a hard startup failure (see docstring). The
    # error message names the regenerator so an operator can copy-
    # paste it to fix the problem without hunting for the tool.
    if not path.is_file():
        raise RuntimeError(
            "whitelist yaml missing: %s -- run "
            "scripts/doccheck/whitelist_gen.py --emit to regenerate"
            % path
        )
    # UTF-8 explicit because yaml source_note may contain CJK
    # (contract prose is bilingual today).
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict) or "processes" not in data:
        raise RuntimeError(
            "whitelist yaml malformed: missing top-level 'processes'"
        )
    procs = data["processes"]
    if not isinstance(procs, dict):
        raise RuntimeError(
            "whitelist yaml malformed: 'processes' must be a mapping"
        )
    # Every required process must appear with pub + sub keys.
    missing = _REQUIRED_PROCESSES - set(procs.keys())
    if missing:
        raise RuntimeError(
            "whitelist yaml missing processes: %s" % sorted(missing)
        )
    out: Dict[str, Dict[str, FrozenSet[str]]] = {}
    for proc, sets in procs.items():
        if not isinstance(sets, dict):
            raise RuntimeError(
                "whitelist yaml %r not a mapping" % proc
            )
        pub = sets.get("pub", []) or []
        sub = sets.get("sub", []) or []
        if not isinstance(pub, list) or not isinstance(sub, list):
            raise RuntimeError(
                "whitelist yaml %r pub/sub must be lists" % proc
            )
        # frozenset so consumers can't mutate.
        out[proc] = {
            "pub": frozenset(pub),
            "sub": frozenset(sub),
        }
    return out


# Load once at module import; the resulting frozensets are the API.
# Import-time load is deliberate:
#   * a bad yaml surfaces at process start (not first whitelist use)
#   * consumers who `from ... import P1_MOTION_PUB` see it as a
#     frozen literal, no lazy-load surprise
#   * tests can monkeypatch _repo_root and re-invoke _load_yaml() to
#     exercise error paths without corrupting the module state
_WHITELISTS = _load_yaml()

# Public alias for the flat data. Existing tests read
# whitelists.WHITELISTS[proc]["pub" | "sub"]; keeping the alias
# preserves that API while _WHITELISTS names the underscore-prefixed
# implementation storage. The two point at the SAME dict; not a copy.
WHITELISTS = _WHITELISTS


# Module-level names preserved byte-for-byte from the previous
# hand-written version so consumers do not need to change imports.
# Ten total names (5 processes x 2 directions). Adding a new
# process = one row in _REQUIRED_PROCESSES + two lines here + one
# entry in the yaml. All three files must move together.
#
# Naming convention: <PROC_NAME>_PUB / _SUB, all upper-case snake.
# Matches the previous convention exactly; any consumer importing
# by name continues to work.
PERCEPTION_PUB: FrozenSet[str] = _WHITELISTS["perception"]["pub"]
PERCEPTION_SUB: FrozenSet[str] = _WHITELISTS["perception"]["sub"]

P1_MOTION_PUB: FrozenSet[str] = _WHITELISTS["p1_motion"]["pub"]
P1_MOTION_SUB: FrozenSet[str] = _WHITELISTS["p1_motion"]["sub"]

CHASSIS_RELAY_PUB: FrozenSet[str] = _WHITELISTS["chassis_relay"]["pub"]
CHASSIS_RELAY_SUB: FrozenSet[str] = _WHITELISTS["chassis_relay"]["sub"]

P2_CORE_PUB: FrozenSet[str] = _WHITELISTS["p2_core"]["pub"]
P2_CORE_SUB: FrozenSet[str] = _WHITELISTS["p2_core"]["sub"]

P4_AGENT_PUB: FrozenSet[str] = _WHITELISTS["p4_agent"]["pub"]
P4_AGENT_SUB: FrozenSet[str] = _WHITELISTS["p4_agent"]["sub"]
