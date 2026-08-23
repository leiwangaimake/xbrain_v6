#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: check_deps.py
Brief: CHK-0-55 -- verify installed Python dependencies match the runtime
       lock file AND (if zenohd is available) major-version-align with it

Description:
Runs the four CHK-0-55 checks in order and exits 1 on any failure. This is
the STARTUP-time gate, so its exit code is what deploy scripts key off:
  ① lock file well-formed: every line is `name==x.y.z` (no `>=`, no bare
     package name). A drift into `>=` here is exactly how zenoh-python 0.x
     silently landed in a 1.x deploy and broke every pub/sub.
  ② installed version == lock version for every entry (importlib.metadata).
  ③ if zenohd binary is present, its major version equals Python
     eclipse-zenoh's major (zenoh-python 1.x wire is incompatible with 0.x
     zenohd -- pubs deliver zero frames to subs, no error anywhere). If
     zenohd is ABSENT, print a WARN and continue (dev machines have Python
     zenoh only) unless --strict is passed; --strict requires zenohd.
  ④ every third-party top-level import under xbrain/ and common/ appears
     in the lock file. A NEW import that forgot to add itself here fails
     here rather than at first-load on the robot.

Design notes:
  * The lock is a plain text file (not requirements.in / not a pip-compile
    output), because CHK-0-55 ① wants a REGEX-checkable format and pip's
    outputs carry hashes / markers that this line-shape regex would reject.
  * Assertion ③ reads zenohd --version by subprocess, not by parsing the
    binary. A binary we cannot execute is treated as absent (permission /
    exec-format errors count as "not there for our purposes").
  * Assertion ④'s scan is AST-based (ast.parse per .py), not regex, so a
    string containing the word "import" cannot cause a false positive.
"""

import argparse
import ast
import importlib.metadata as im
import os
import re
import shutil
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_LOCK = os.path.join(_HERE, "requirements-runtime.txt")
DEFAULT_ROOT = os.path.dirname(os.path.dirname(_HERE))

# Line shape: exact `name==x.y.z`. Name follows PEP 503 loosely (letters /
# digits / hyphen / underscore / period). Version MUST have >= 3 dotted
# segments -- the criterion says `x.y.z` verbatim, and accepting `x.y` or
# bare `x` lets a partial-pin (`PyYAML==6`) slip through and match ANY 6.*
# on install. A tail suffix (like 1.0.0rc1 or 1.0.0.post1) is allowed.
_LOCK_LINE_RE = re.compile(
    r"^([A-Za-z0-9_.-]+)==(\d+\.\d+\.\d+(?:[a-zA-Z0-9_.]*)?)\s*$"
)

# Import-name -> distribution name for the handful of packages whose PyPI
# name differs from the module the code imports. Keep this SMALL: adding an
# entry here is easier than teaching the checker to guess.
_IMPORT_TO_DIST = {
    "yaml": "PyYAML",
    "zenoh": "eclipse-zenoh",
}

# What the AST scan considers first-party (not a third-party import to lock).
_FIRST_PARTY = frozenset({
    "xbrain", "tests", "common", "scripts", "services", "ros2_ws",
})


def load_lock(path):
    """Return {distribution_name: version} from `path`. Assertion ①: raises
    ValueError on any malformed line, naming the line number and content."""
    entries = {}
    with open(path, encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            m = _LOCK_LINE_RE.match(line)
            if not m:
                # Message names the offending exact text so the fix is a
                # single sed. No wildcards / markers permitted, per contract.
                raise ValueError(
                    "%s:%d: not `name==x.y.z`: %r" % (path, lineno, raw.rstrip())
                )
            entries[m.group(1)] = m.group(2)
    return entries


def _major(version):
    """First dotted segment as int; anything unparsable returns None so the
    caller can decide whether a missing major counts as a mismatch."""
    seg = version.split(".", 1)[0]
    try:
        return int(seg)
    except ValueError:
        return None


def check_installed(lock):
    """Assertion ②: every locked distribution is installed at the exact
    version. Returns [(name, wanted, got_or_None), ...]; empty list = green."""
    deltas = []
    for name, want in lock.items():
        try:
            got = im.version(name)
        except im.PackageNotFoundError:
            deltas.append((name, want, None))
            continue
        if got != want:
            deltas.append((name, want, got))
    return deltas


def check_zenoh_wire(lock, strict=False):
    """Assertion ③: zenohd binary major == eclipse-zenoh Python major.

    Returns (ok, detail_line). ok=False means refuse startup. When zenohd is
    absent AND strict is False, ok=True with a WARN detail so operators see
    the check WAS run and DID skip -- silence-is-not-success (CLAUDE.md 3.2).
    """
    py_version = lock.get("eclipse-zenoh")
    if py_version is None:
        return True, "eclipse-zenoh not in lock; skipping wire check"
    py_major = _major(py_version)
    zenohd = shutil.which("zenohd")
    if zenohd is None:
        if strict:
            return False, ("zenohd not found on PATH but --strict was passed; "
                           "install zenohd or run without --strict")
        return True, ("WARN: zenohd binary not on PATH; wire check skipped "
                      "(dev environment)")
    try:
        out = subprocess.check_output([zenohd, "--version"], text=True,
                                      stderr=subprocess.STDOUT, timeout=5)
    except (subprocess.SubprocessError, OSError) as exc:
        # An unrunnable zenohd is not "absent" -- it is a defect that must
        # not silently pass, so this returns not-ok regardless of --strict.
        return False, "zenohd --version failed: %s" % exc
    # zenohd prints e.g. "zenohd 1.9.0 built with rustc ..." on the first line.
    m = re.search(r"(\d+)\.(\d+)\.(\d+)", out)
    if not m:
        return False, "could not parse zenohd version: %r" % out
    zd_major = int(m.group(1))
    if zd_major != py_major:
        return False, ("major mismatch: eclipse-zenoh Python is %d.x, zenohd "
                       "binary is %d.x -- wire protocols do not agree"
                       % (py_major, zd_major))
    return True, "OK: eclipse-zenoh %d.x matches zenohd %d.x" % (py_major, zd_major)


def scan_third_party_imports(root):
    """Return the set of third-party top-level module names imported anywhere
    under `root`. AST-based so a string with the word "import" cannot cause
    a false positive."""
    stdlib = set(sys.stdlib_module_names)
    found = set()
    for dirpath, dirnames, files in os.walk(root):
        # skip caches and virtualenvs; the caller only ever passes a source dir.
        dirnames[:] = [d for d in dirnames if d not in
                       ("__pycache__", ".pytest_cache", "build", ".venv")]
        for f in files:
            if not f.endswith(".py"):
                continue
            path = os.path.join(dirpath, f)
            try:
                tree = ast.parse(open(path, encoding="utf-8").read())
            except (SyntaxError, UnicodeDecodeError):
                # A file we cannot parse cannot be scanned; report so a
                # reviewer knows the check was NOT applied to that path.
                sys.stderr.write("check_deps: could not parse %s\n" % path)
                continue
            optional = _optional_import_names(tree)
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for n in node.names:
                        found.add(n.name.split(".")[0])
                elif isinstance(node, ast.ImportFrom):
                    # Relative imports have level > 0 and no module of their
                    # own to lock; skip.
                    if node.level == 0 and node.module:
                        found.add(node.module.split(".")[0])
            found -= optional
    return {n for n in found if n not in stdlib and n not in _FIRST_PARTY}


def _optional_import_names(tree):
    """Top-level module names imported INSIDE a try whose handler catches
    ImportError -- i.e. the code already works without them.

    *** Why assertion 4 needs this. The lock exists so a bring-up install has
    every module the runtime needs. A module the source itself treats as
    optional -- imported under try/except ImportError, with a fallback on the
    handler path -- is by construction not needed for the process to run.
    Locking it would make an optional dependency mandatory, and would
    contradict the fallback the author deliberately wrote.

    Concretely: hmi/geo_timezone.py imports tzfpy this way and falls back to
    common.timezone, so a host without it shows the configured zone instead of
    the GPS-derived one. Degraded, not broken.

    *** Conservative: only the imports LEXICALLY inside the try body count, and
    only when a handler names ImportError (or ModuleNotFoundError, its
    subclass) or is a bare `except:`. An import in the else/finally clause, or
    under a try that catches something else, is NOT optional and stays
    required -- an unknown guard keeps the stricter answer.
    """
    names = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Try):
            continue
        catches_import = False
        for h in node.handlers:
            if h.type is None:
                catches_import = True
            else:
                for t in (h.type.elts if isinstance(h.type, ast.Tuple)
                          else [h.type]):
                    if isinstance(t, ast.Name) and t.id in (
                            "ImportError", "ModuleNotFoundError"):
                        catches_import = True
        if not catches_import:
            continue
        for stmt in node.body:
            for sub in ast.walk(stmt):
                if isinstance(sub, ast.Import):
                    for n in sub.names:
                        names.add(n.name.split(".")[0])
                elif isinstance(sub, ast.ImportFrom):
                    if sub.level == 0 and sub.module:
                        names.add(sub.module.split(".")[0])
    return names


def check_imports_covered(lock, imports):
    """Assertion ④: every third-party import maps (via _IMPORT_TO_DIST or
    identity) to a distribution present in the lock. Returns the list of
    IMPORT names not covered; empty list = green."""
    locked = set(lock)
    uncovered = []
    for name in sorted(imports):
        dist = _IMPORT_TO_DIST.get(name, name)
        if dist not in locked:
            uncovered.append(name)
    return uncovered


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().split("\n")[0])
    ap.add_argument("--lock", default=DEFAULT_LOCK,
                    help="path to requirements-runtime.txt")
    ap.add_argument("--root", default=DEFAULT_ROOT,
                    help="project root; scans root/xbrain and root/common")
    ap.add_argument("--strict", action="store_true",
                    help="require zenohd to be present (assertion ③)")
    args = ap.parse_args()

    try:
        lock = load_lock(args.lock)                 # ① well-formed
    except ValueError as exc:
        print("FAIL ①: %s" % exc)
        return 1
    print("① lock well-formed: %d entries" % len(lock))

    deltas = check_installed(lock)                  # ② installed == locked
    if deltas:
        for name, want, got in deltas:
            print("FAIL ②: %s wants %s, installed %s"
                  % (name, want, got or "MISSING"))
        return 1
    print("② installed matches: %d entries" % len(lock))

    ok3, detail3 = check_zenoh_wire(lock, strict=args.strict)   # ③ wire major
    print("③ zenoh wire: %s" % detail3)
    if not ok3:
        return 1

    # ④ scan xbrain/ and common/ for third-party imports; both are the areas
    # the runtime pulls into memory. tests / scripts are dev-only.
    imports = set()
    for sub in ("xbrain", "common"):
        p = os.path.join(args.root, sub)
        if os.path.isdir(p):
            imports |= scan_third_party_imports(p)
    uncovered = check_imports_covered(lock, imports)
    if uncovered:
        for name in uncovered:
            dist = _IMPORT_TO_DIST.get(name, name)
            print("FAIL ④: import %r (distribution %r) not in lock"
                  % (name, dist))
        return 1
    print("④ imports covered: %s" % sorted(imports))

    print("ALL CHECKS OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
