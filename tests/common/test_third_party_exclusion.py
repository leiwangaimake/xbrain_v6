"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_third_party_exclusion.py
Brief: The vendored tree common/third_party is outside every source lint, provably

Description:
What problem this solves. 99 U85 item 6 (2026-09-15) admits a vendored,
unmodified nlohmann/json single header under common/third_party/. That file
carries a non-ASCII byte, eleven three-way comparisons under version macros and
hundreds of E_-shaped identifiers -- every one a legitimate hit for
charset_lint, header_lint, cxx_discipline_audit or no_literal_ecode if they
ever walked it. The fix is one shared exclusion list (charset_lint
.THIRD_PARTY_SNAPSHOTS), imported by the other walkers, and this file is the
test that keeps that promise honest.

The positive control is the important half (CLAUDE.md 3.3). "No walker yields a
vendored file" is also true of an empty tree and of a walker that yields
nothing at all. So the second test mutates the shared list to DROP the entry
and asserts the walkers then DO find json.hpp: the file exists, the walkers
reach it, and only the exclusion hides it.

Why clock_scan.py is deliberately NOT on this list. Its scan surface is pinned
by tests/common/test_clock.py, which reports any shrink of the counted files as
a failure -- pruning a subtree there trips exactly that guard. The vendored
header reads no wall clock (json.hpp has no chrono or time call at all), so
clock_scan keeps walking it and stays green on its own merits; if a future
upgrade of the header ever introduces one, clock_scan is the lint that should
say so rather than the one that was taught to look away.

What this file does not claim: that the vendored header is correct or that its
consumers keep it off realtime threads (13 QD-7). The first is upstream's job,
the second is a rule on the quadruped sources, checked where they are lint-ed.
"""
import importlib.util
import os
import sys

import pytest

# INF-TS-1: every test file carries a module-level hardware marker. This one
# reads files on disk and runs no device, so no_device is the honest value --
# and a NEW file without it silently enlarges the legacy debt allowlist that
# tests/meta/test_marker_coverage.py is there to shrink.
pytestmark = pytest.mark.no_device

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
VENDORED_REL = "common/third_party"
VENDORED_FILE = os.path.join(ROOT, VENDORED_REL, "nlohmann", "json.hpp")
LINT_DIR = os.path.join(ROOT, "scripts", "lint")


def _load(name, rel):
    """Import a script that is not a package, by file path.

    The lint scripts live under scripts/ with no __init__.py, and each inserts
    its own directory into sys.path for the shared import; putting scripts/lint
    first here makes `import charset_lint` resolve to the same module object
    for every loaded script, which is what the mutation test relies on.
    """
    if LINT_DIR not in sys.path:
        sys.path.insert(0, LINT_DIR)
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, rel))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def mods():
    """The five walkers, loaded once. charset_lint is imported under its own
    name so that the scripts' `import charset_lint` shares it."""
    charset = _load("charset_lint", "scripts/lint/charset_lint.py")
    header = _load("header_lint", "scripts/lint/header_lint.py")
    ecode = _load("no_literal_ecode", "scripts/lint/no_literal_ecode.py")
    cxx = _load("cxx_discipline_audit", "scripts/ci/cxx_discipline_audit.py")
    singular = _load("no_config_singular", "scripts/lint/no_config_singular.py")
    return charset, header, ecode, cxx, singular


def _under_vendored(path):
    return os.path.relpath(path, ROOT).startswith(VENDORED_REL + os.sep)


def _all_walked(mods):
    """Every path any of the five walkers would scan, as one list."""
    charset, header, ecode, cxx, singular = mods
    walked = list(charset.iter_sources())
    walked += list(header.iter_sources())
    walked += list(ecode.iter_sources())
    for rule in cxx.RULES:
        walked += cxx._files_for(rule)
    walked += singular._walk(ROOT)
    return walked


def test_the_vendored_file_exists():
    """The subject must be on disk; otherwise the exclusion tests test nothing."""
    assert os.path.isfile(VENDORED_FILE), VENDORED_FILE
    assert os.path.isfile(os.path.join(os.path.dirname(VENDORED_FILE), "LICENSE.MIT"))


def test_vendored_tree_is_declared_in_the_shared_list(mods):
    """One list, not four: the entry lives in charset_lint and nowhere else."""
    charset, header, ecode, cxx, singular = mods
    assert VENDORED_REL in charset.THIRD_PARTY_SNAPSHOTS
    # The other four read charset_lint's list rather than keeping a copy.
    assert header.THIRD_PARTY_SNAPSHOTS is charset.THIRD_PARTY_SNAPSHOTS
    assert ecode.charset_lint is charset
    assert cxx.charset_lint is charset
    assert singular.charset_lint is charset


def test_no_walker_yields_a_vendored_file(mods):
    """The negative half: none of the five scan surfaces reaches the tree."""
    walked = _all_walked(mods)
    assert walked, "the scan surfaces are empty; nothing was actually checked"
    leaked = sorted({p for p in walked if _under_vendored(p)})
    assert not leaked, "vendored files reached a lint walker: %s" % leaked


def test_dropping_the_entry_makes_the_walkers_see_it(mods, monkeypatch):
    """*** The positive control (CLAUDE.md 3.3): remove the exclusion and the
    walkers must FIND json.hpp. A walker that never reaches common/ at all
    would pass the negative test above; this one catches it.
    """
    charset, header, ecode, cxx, singular = mods
    without = tuple(t for t in charset.THIRD_PARTY_SNAPSHOTS if t != VENDORED_REL)
    monkeypatch.setattr(charset, "THIRD_PARTY_SNAPSHOTS", without)
    monkeypatch.setattr(header, "THIRD_PARTY_SNAPSHOTS", without)
    seen = {p for p in _all_walked(mods) if p == VENDORED_FILE}
    assert seen == {VENDORED_FILE}, (
        "with the exclusion removed the walkers should reach json.hpp; "
        "they did not, so the negative test proves nothing")
