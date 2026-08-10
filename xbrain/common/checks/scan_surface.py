"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: scan_surface.py
Brief: CHK-2-51 scan-surface meta-gate (SCAN_SURFACE + self-exclusion)

Description:
Every scan-class script (scripts/ci/, scripts/lint/, scripts/doccheck/)
must:

  1. Export SCAN_SURFACE = a dict describing its include / exclude
     paths + file extensions. This is what makes the scope
     auditable: 'what files does this rule actually look at?'.
  2. Exclude its own path from its SCAN_SURFACE (form 3 in
     CLAUDE.md §3.2 'judgement self-injury': the check must not
     see itself, or its own rule-text would satisfy the rule and
     the check would never go red).

THIS module is the meta-gate; it itself follows the discipline:
  * exports SCAN_SURFACE with itself excluded
  * a self-test asserts that removing the self-exclusion makes
    the gate report itself as a violation

Docs (markdown) are ALWAYS excluded from source-code scan surfaces
(CLAUDE.md §2.2 exception).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Iterable, Tuple


SCAN_SURFACE = {
    "include": ("scripts/ci", "scripts/lint", "scripts/doccheck"),
    "exclude": ("xbrain/common/checks/scan_surface.py",
                  "docs"),   # docs/ never scanned
    "extensions": (".py",),
}


class ScanSurfaceViolation(Exception):
    pass


@dataclass(frozen=True)
class ScriptSurface:
    """One scan script's declared SCAN_SURFACE."""
    path: str                    # relative path of the script
    include: Tuple[str, ...]
    exclude: Tuple[str, ...]
    extensions: Tuple[str, ...]


def load_script_surface(path: str, module_globals: dict) -> ScriptSurface:
    """Read SCAN_SURFACE from a scan script's globals. Missing or
    ill-shaped -> raise."""
    if "SCAN_SURFACE" not in module_globals:
        raise ScanSurfaceViolation(
            f"script {path!r} does not export SCAN_SURFACE")
    surf = module_globals["SCAN_SURFACE"]
    if not isinstance(surf, dict):
        raise ScanSurfaceViolation(
            f"script {path!r} SCAN_SURFACE must be a dict, got "
            f"{type(surf).__name__}")
    for k in ("include", "exclude", "extensions"):
        if k not in surf:
            raise ScanSurfaceViolation(
                f"script {path!r} SCAN_SURFACE missing key {k!r}")
    return ScriptSurface(
        path=path,
        include=tuple(surf["include"]),
        exclude=tuple(surf["exclude"]),
        extensions=tuple(surf["extensions"]),
    )


def check_self_excluded(script: ScriptSurface) -> None:
    """The script MUST list itself under exclude (or any parent
    path of itself under exclude). If not, the script would scan
    its own text and satisfy any rule the text prescribes -- the
    'judgement self-injury' failure mode."""
    excluded = False
    for x in script.exclude:
        if script.path == x or script.path.startswith(x.rstrip("/") + "/"):
            excluded = True
            break
    if not excluded:
        raise ScanSurfaceViolation(
            f"script {script.path!r} does not exclude ITSELF from "
            f"SCAN_SURFACE.exclude; it would scan its own text "
            f"(CLAUDE.md §3.2 form 3)")


def check_docs_never_included(script: ScriptSurface) -> None:
    """CLAUDE.md §2.2: docs/ is exempt from source-code scans;
    a source-class scan script listing docs/ under include is a
    misclassification."""
    for inc in script.include:
        if inc.startswith("docs/") or inc == "docs":
            raise ScanSurfaceViolation(
                f"script {script.path!r} includes docs/ ({inc!r}) in "
                f"a source-code scan surface; docs/ markdown is "
                f"exempt from CLAUDE.md §2.2 charset rules")


def audit_scan_scripts(scripts: Iterable[ScriptSurface]) -> None:
    """Run every check on every scan script. First failure raises."""
    for s in scripts:
        check_self_excluded(s)
        check_docs_never_included(s)
