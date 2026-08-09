"""INF-CI-4 -- every doccheck script has --self-test or is exempted."""

import subprocess
import sys
from pathlib import Path


DOCCHECK_DIR = Path(__file__).parent.parent.parent / "scripts" / "doccheck"


# Explicit exemption table. Each entry MUST carry a reason. A script
# added later without either a --self-test or an entry here will fail
# test_every_doccheck_script_covered.
_EXEMPT = {
    "closed_set_snapshot.py":
        "produces a snapshot artifact; self-test would compare against "
        "itself. The scan discipline is enforced by diffing two runs, "
        "not by asserting on one.",
    "mark_exemptions.py":
        "applies SEC-12 exemption markers as an operator action; the "
        "check is on the resulting file state, not on the applier.",
    "prune_dead_docs.py":
        "one-shot content pruner; a self-test would run the prune on a "
        "fixture, which is exactly what CI would not want (mutates docs).",
    "verify_merge_claims.py":
        "reads patch blocks and diffs against surrounding text; the "
        "self-test would be a full round-trip of the doc which is "
        "outside the tool's own scope.",
}


def _all_scripts():
    return sorted(p.name for p in DOCCHECK_DIR.glob("*.py")
                  if p.name != "__init__.py")


def _has_self_test(name: str) -> bool:
    src = (DOCCHECK_DIR / name).read_text()
    return "--self-test" in src or "self_test" in src


def test_every_doccheck_script_covered():
    """Each script has --self-test OR is in _EXEMPT with a reason."""
    scripts = _all_scripts()
    problems = []
    for name in scripts:
        if _has_self_test(name):
            continue
        if name in _EXEMPT:
            continue
        problems.append(name)
    assert not problems, (
        "scripts without --self-test and no exemption: %s. "
        "Either add --self-test or add an entry to _EXEMPT here."
        % problems)


def test_exempt_entries_are_real_scripts():
    """Every entry in _EXEMPT names a script that exists on disk."""
    scripts = set(_all_scripts())
    for name in _EXEMPT:
        assert name in scripts, "_EXEMPT names %r but no such script" % name


def test_exempt_entries_have_reasons():
    """Every _EXEMPT entry's reason must be substantial (>= 20 chars)."""
    for name, reason in _EXEMPT.items():
        assert len(reason) >= 20, \
            "%s reason too short: %r" % (name, reason)


def test_no_exemption_shadows_a_self_tested_script():
    """A script that HAS --self-test must NOT also be in _EXEMPT.
    Otherwise the exemption becomes silent debt: the reason says
    'no self-test' but a self-test exists and could be run."""
    for name in _EXEMPT:
        assert not _has_self_test(name), \
            "%s has --self-test AND is in _EXEMPT (remove the exemption)" % name


def test_every_self_tested_script_actually_passes():
    """Run --self-test on every script that has one; each must exit 0.

    This is the "meta" half of INF-CI-4: it's not enough for the flag
    to exist; the self-test must actually pass. A script whose
    self-test rots into always-passing (or always-failing) is worse
    than one without.
    """
    failed = []
    for name in _all_scripts():
        if not _has_self_test(name):
            continue
        r = subprocess.run(
            [sys.executable, str(DOCCHECK_DIR / name), "--self-test"],
            capture_output=True, text=True, timeout=60)
        if r.returncode != 0:
            failed.append((name, r.returncode,
                           (r.stdout + r.stderr)[:200]))
    assert not failed, \
        "self-test failures:\n" + "\n".join(
            "  %s exit=%d %s" % t for t in failed)


def test_at_least_five_scripts_have_self_test():
    """Sanity: don't let the exempt list grow unbounded.

    If more than half of scripts end up exempt, the discipline is
    broken. This test warns when a proliferation of exemptions
    starts.
    """
    total = len(_all_scripts())
    exempt = len(_EXEMPT)
    with_self_test = total - exempt
    assert with_self_test >= 5, \
        "only %d/%d scripts have --self-test (rest exempted); the " \
        "coverage discipline is trending toward zero" % (with_self_test, total)
