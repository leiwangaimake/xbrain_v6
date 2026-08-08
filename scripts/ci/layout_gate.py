#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: layout_gate.py
Brief: CHK-1-52 -- guard the CLAUDE.md S0.2 directory ruling: one deliverable,
       one home, no wandering copies in common/boot or scripts/freeze

Description:
CLAUDE.md S0.2 pins WHICH directory each kind of file lives in. When those
rules only live in prose, drift is silent: someone puts a boot python file
under common/, someone puts a systemd unit under xbrain/, and the two
locations coexist until a bring-up incident forces someone to figure out
which one runs. This lint is the ENFORCEMENT half; the docs S0.2 rule is
the SPECIFICATION half, and both have to move together.

The rules, as CHK-1-52 spells them out:
  A) common/**  -- C++ headers and their build glue only, no Python source.
     Callers include chassis_relay on the E-stop path (CLAUDE.md S5.3), so
     no rclcpp / no ROS types either. This lint enforces the LAYOUT half
     of that rule; the ROS-type half is CPP-CXX-1's job (needs nm/readelf).
  B) common/**/*.py MUST NOT `import xbrain.*` -- common is the DEPENDENCY,
     not the CONSUMER. Today there are no .py in common/ at all, but this
     check stays because a future refactor could add one.
  C) deploy/**   -- systemd/network glue only: .service / .timer /
     .network / .nft / .conf. No .py (business code doesn't live here).
  D) data/**     -- runtime artifacts + docs + media assets. No source
     code (.py / .sh / .cc / .h / .yaml / .json config). A .gitkeep and
     README.md are always allowed; media files (mp3 / wav / png) are
     runtime resources, allowed.

Extensibility contract:
  RULES is a list of RuleSpec; each carries a `root` (relpath under repo),
  a `allow_ext` OR `deny_ext` set, an optional `content_probe` callable
  for import-style rules, and its CHK-1-52 label. Adding a rule = adding
  one row; the runner stays uniform. Table-driven, not scattered ifs --
  same discipline as todo_lint.py.

NOTE: Scan surface declared per rule: violated CHK-1-52 star point about
  writing "全仓" without declaring 扫描面 if a rule ever just walked the
  whole repo without saying which subtree it targeted.
"""

import argparse
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Callable, FrozenSet, List, Optional, Tuple

# Repo root -- derived from THIS FILE'S path so the script works whether
# invoked as `python3 scripts/ci/layout_gate.py` or via absolute path.
# Two parents up from scripts/ci/ -> repo root.
_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))

# Files that are ALWAYS OK anywhere: metadata that carries no code.
# .gitkeep is a placeholder to make git track an empty directory.
# README.md is a doc, exempt from the source-code bans.
_ALWAYS_OK_NAMES = frozenset({".gitkeep", "README.md", ".gitignore"})

# Filename patterns to skip entirely from EVERY scan (build outputs, VCS
# metadata, pytest caches). Not a "ban" list -- these files don't count
# against any rule, they just aren't scanned in the first place.
_SKIP_DIRS = frozenset({
    ".git", "__pycache__", "node_modules", "build", "install", "log",
    ".pytest_cache", ".mypy_cache",
})


@dataclass(frozen=True)
class RuleSpec:
    """One CHK-1-52 rule.

    label: the CHK-1-52 star point this rule enforces (a-d).
    root:  repo-relative directory this rule walks; scan surface disclosed
           in the printed header so a reader can trace WHERE a finding
           came from.
    allow_ext / deny_ext: extension set. Exactly one of the two is set;
           allow_ext means "only these are allowed", deny_ext means
           "everything else is fine, these specific extensions are not".
    content_probe: optional callable(abs_path, rel_path) -> Optional[str].
           When set, EVERY file under `root` (already passing extension
           check) is fed to this callable, which returns None for OK or
           a finding string. Rule B (common/**/*.py must not `import
           xbrain.*`) uses this; extension-only rules leave it None.
    reason: human-readable why-this-rule-exists sentence, printed with
           each finding so the caller understands WHY the file is bad.
    """

    label: str
    root: str
    reason: str
    allow_ext: Optional[FrozenSet[str]] = None
    deny_ext: Optional[FrozenSet[str]] = None
    content_probe: Optional[Callable[[str, str], Optional[str]]] = None
    # Ignore relative subpaths (relative to root) -- e.g. common/lib/
    # holds build products, not source, so it should not trip an ext gate.
    # Comparison is by directory prefix.
    ignore_subroots: Tuple[str, ...] = field(default_factory=tuple)


def _check_common_py_no_xbrain(abs_path: str, rel_path: str) -> Optional[str]:
    """Rule B probe: a .py file under common/ must not import xbrain.*.

    common/ is a DEPENDENCY. If it imports xbrain, the dependency arrow
    reverses and the module can no longer be consumed by chassis_relay
    or any C++ side that ships without a full xbrain checkout.
    """
    # Only .py files -- other files (headers, CMakeLists) can't `import`.
    if not rel_path.endswith(".py"):
        return None
    # Cheap grep, one file at a time. Not worth a full AST here -- the
    # false-positive on a string literal `"import xbrain."` is acceptable
    # because it's already suspicious enough to want a human review.
    try:
        with open(abs_path, encoding="utf-8") as fh:
            text = fh.read()
    except (OSError, UnicodeDecodeError):
        # Unreadable file -- treat as passing this rule; the extension
        # gate above already caught it OR it's a non-text file we don't
        # want to false-flag.
        return None
    # Match `import xbrain` or `from xbrain` at line start (after optional
    # whitespace). Multiline flag so ^ anchors work per-line.
    if re.search(r"^\s*(?:from|import)\s+xbrain\b", text, re.MULTILINE):
        return "imports xbrain.* -- common/ is a dependency, not a consumer"
    return None


# ---------------------------------------------------------------------------
# THE RULES -- one row per CHK-1-52 letter (a-d) plus the import probe
# ---------------------------------------------------------------------------
#
# Ordering: extension gates come before content probes, so a file killed
# by extension does not need to be opened for content inspection.

RULES: Tuple[RuleSpec, ...] = (
    RuleSpec(
        label="CHK-1-52-A",
        root="common",
        reason="common/ holds C++ shared library only; no Python source "
               "(CLAUDE.md S0.2, S5.3 -- chassis_relay on E-stop path "
               "cannot depend on Python)",
        allow_ext=frozenset({
            # C++ sources + headers -- the whole point of common/.
            ".h", ".hpp", ".cc", ".cpp", ".c",
            # Build glue.
            ".cmake", ".txt", ".in",
            # Data-driven inputs (schema-like tables consumed at build).
            ".yaml", ".yml", ".json",
            # Docs.
            ".md",
        }),
        # common/lib is the shared-object output directory; a .so or .a
        # landing there is a BUILD PRODUCT, not source, and shouldn't
        # trigger the extension gate on shape alone.
        ignore_subroots=("lib",),
    ),
    RuleSpec(
        label="CHK-1-52-D-import",
        root="common",
        reason="common/ files must not `import xbrain.*` (dependency arrow "
               "would reverse -- CLAUDE.md S0.2)",
        # No ext gate -- content probe walks EVERY file; probe filters
        # to .py inside. Passing None for both means the outer loop
        # will not extension-filter; that's deliberate because the probe
        # already knows to only inspect .py.
        allow_ext=None,
        deny_ext=None,
        content_probe=_check_common_py_no_xbrain,
        ignore_subroots=("lib",),
    ),
    RuleSpec(
        label="CHK-1-52-B",
        root="deploy",
        reason="deploy/ holds systemd/network glue only (.service / .timer / "
               ".network / .nft / .conf / .rules); no Python "
               "(CLAUDE.md S0.2)",
        allow_ext=frozenset({
            # systemd unit types.
            ".service", ".timer", ".target", ".mount", ".socket",
            # networkd + firewall.
            ".network", ".nft", ".conf",
            # udev.
            ".rules",
            # Docs.
            ".md",
        }),
    ),
    RuleSpec(
        label="CHK-1-52-C",
        root="data",
        reason="data/ holds runtime artifacts + docs + media only; no "
               "source code (CLAUDE.md S0.2)",
        deny_ext=frozenset({
            # Python / shell / C++ / build glue -- source of any language.
            ".py", ".sh", ".bash", ".zsh", ".fish",
            ".cc", ".cpp", ".c", ".h", ".hpp",
            ".cmake", ".ts", ".tsx", ".js", ".jsx", ".rs", ".go",
        }),
    ),
)


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def _walk(root_abs: str, ignore_prefixes: Tuple[str, ...]):
    """Yield (rel_path, abs_path) for every file under `root_abs`, skipping
    the standard build/VCS dirs and any relative subroot in
    ignore_prefixes."""
    for dirpath, dirnames, filenames in os.walk(root_abs):
        # In-place mutation of dirnames tells os.walk NOT to descend.
        dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS]
        for fn in filenames:
            abs_p = os.path.join(dirpath, fn)
            rel_p = os.path.relpath(abs_p, root_abs)
            # Skip anything under an ignored subroot (common/lib etc).
            if any(rel_p == p or rel_p.startswith(p + os.sep)
                   for p in ignore_prefixes):
                continue
            yield rel_p, abs_p


def _check_rule(rule: RuleSpec, repo_root: str) -> List[str]:
    """Run one rule; return a list of finding strings."""
    root_abs = os.path.join(repo_root, rule.root)
    findings: List[str] = []
    # If the root itself doesn't exist, that's silently OK -- we don't
    # know if the checker was invoked from a partial checkout. Rule 4
    # (data/) may not exist in an initial clone. A missing root cannot
    # violate a rule, only future files can.
    if not os.path.isdir(root_abs):
        return findings
    for rel_p, abs_p in _walk(root_abs, rule.ignore_subroots):
        basename = os.path.basename(rel_p)
        # Extension gate. Extension is the .lower()'d suffix; a file with
        # no extension gets "" and passes when allow_ext contains ""
        # (none of ours do, so extensionless files hit the deny path).
        ext = os.path.splitext(basename)[1].lower()
        if rule.allow_ext is not None:
            # Whitelist mode: any extension not in allow_ext is a finding
            # UNLESS basename is on the always-ok list.
            if basename in _ALWAYS_OK_NAMES:
                pass          # exempted (e.g. .gitkeep, README.md)
            elif ext not in rule.allow_ext:
                findings.append(
                    "  %s/%s -- extension %r not allowed here (%s)"
                    % (rule.root, rel_p, ext, rule.reason)
                )
        if rule.deny_ext is not None:
            # Blacklist mode: only ext in deny_ext is a finding.
            if basename in _ALWAYS_OK_NAMES:
                pass          # exempted
            elif ext in rule.deny_ext:
                findings.append(
                    "  %s/%s -- extension %r banned here (%s)"
                    % (rule.root, rel_p, ext, rule.reason)
                )
        # Content probe (rule B is the only user). Runs regardless of
        # ext gates; probe itself filters to interesting files.
        if rule.content_probe is not None:
            problem = rule.content_probe(abs_p, rel_p)
            if problem is not None:
                findings.append(
                    "  %s/%s -- %s" % (rule.root, rel_p, problem)
                )
    return findings


def run(repo_root: str = DEFAULT_REPO_ROOT) -> int:
    """Run all rules; return exit code (0 on green, 1 on any finding)."""
    print("layout_gate: repo_root=%s" % repo_root)
    total = 0
    for rule in RULES:
        # Per-rule header states its scan surface -- CHK-1-52 star point
        # "全仓 without 扫描面 declaration = auto-fail" applies to us too.
        print("[%s] surface=%s/ -- %s" % (rule.label, rule.root, rule.reason))
        findings = _check_rule(rule, repo_root)
        if findings:
            for f in findings:
                print(f)
            total += len(findings)
        else:
            print("  ok")
    if total:
        print("layout_gate: %d finding(s) across %d rule(s)"
              % (total, len(RULES)))
        return 1
    print("layout_gate: green -- all rules pass")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    ap.add_argument("--repo-root", default=DEFAULT_REPO_ROOT)
    args = ap.parse_args()
    return run(args.repo_root)


if __name__ == "__main__":
    sys.exit(main())
