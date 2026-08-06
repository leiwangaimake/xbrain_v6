"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_no_config_source_read.py
Brief: Metatest for the no-source-read lint -- its scan surface and its exemptions

Description:
CFG-CM-11, lint side. What is tested here is mostly not "does it find a hit" --
the lint's own --self-test covers that, with one probe per case. What needs a
metatest is everything ABOUT the lint that could quietly stop being true:

  * the scan surface. A lint that scans one directory instead of three still
    reports zero, and zero from a shrunken surface is indistinguishable from zero
    from a clean tree. So SCAN_DIRS is pinned here, and so is the fact that it
    excludes scripts/ -- the lint lives there, and a surface that included it
    would make the criterion unreachable and invite it being loosened.

  * the exemption mechanism. It only works while every exemption is visible, so
    this asserts the tag set is closed, and that the script contains no silent
    path-based skip alongside the declared one.

  * the mutation CFG-CM-11 names verbatim: open("/opt/xbrain_v6/configs/
    p1_motion.yaml") inside p1_motion must turn the lint red. It is run against a
    temp tree shaped like the real one, and a separate case pins that the real
    surface would have covered it.

The lint script's own docstring claims this file asserts its scan surface. It did
not exist when that sentence was written; a claim in a docstring about a test is
worth exactly as much as the test.
"""

import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts", "lint"))

import no_config_source_read as L  # noqa: E402

SCRIPT = os.path.join(ROOT, "scripts", "lint", "no_config_source_read.py")


# -- the scan surface ----------------------------------------------------------

def test_scan_surface_is_exactly_the_three_trees_the_item_names():
    """*** Pinned, because shrinking it is invisible in the output.

    Mutation: drop ros2_ws or services from SCAN_DIRS => red. The lint would go
    on printing "violations: 0" and a reader would have no way to tell that a
    tree had stopped being checked.
    """
    assert L.SCAN_DIRS == ("xbrain", "ros2_ws", "services")


def test_the_lint_is_outside_its_own_scan_surface():
    """*** CLAUDE.md 3.2 form 3, 判据自伤.

    The script necessarily contains the pattern it searches for -- it is spelled
    out twice in its own Description. If scripts/ were scanned, the hit count
    could never reach zero, and the repair anyone reaches for is to loosen the
    criterion until it passes.

    Mutation: add "scripts" to SCAN_DIRS => red.
    """
    assert "scripts" not in L.SCAN_DIRS
    assert "tests" not in L.SCAN_DIRS, (
        "tests/ must stay out too: a fixture has to be able to build a source "
        "path, and tests/common/test_config_overlay.py already asserts the "
        "value of DEFAULT_CONFIG_ROOT"
    )
    # And the premise: the script really does contain the pattern, so the
    # exclusion above is load-bearing rather than a precaution against nothing.
    with open(SCRIPT, encoding="utf-8") as fh:
        assert L._CONFIG_ROOT_LITERAL in fh.read(), (
            "if the script no longer contains the literal, this test is "
            "asserting a property that costs nothing -- check whether the "
            "exclusion is still needed rather than deleting the test"
        )


def test_an_empty_or_absent_tree_is_reported_and_does_not_pass_for_clean():
    """ros2_ws/ holds no scannable file today.

    Without the per-directory counts, the report would read as three trees
    checked and all clean. Mutation: remove the NOTE and the counts => red.
    """
    out = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True)
    assert "ros2_ws" in out.stdout
    for tree in L.SCAN_DIRS:
        assert tree + "/" in out.stdout or "NOTE " + tree in out.stdout, (
            "every tree in the surface must appear in the report with either a "
            "file count or a NOTE saying it contributed nothing: " + tree
        )


# -- the exemption mechanism ---------------------------------------------------

def test_the_exempt_tag_set_is_closed_and_is_what_the_item_permits():
    """freeze plus the three assertions that genuinely evaluate on the source.

    Mutation: add a tag => red. An open tag set degrades the mechanism into
    "write any word and the check goes quiet", which is an exemption that is
    visible in the file and invisible in review.
    """
    assert set(L.EXEMPT_TAGS) == {"freeze", "J", "B", "K"}
    for tag, why in L.EXEMPT_TAGS.items():
        assert len(why) > 30, "tag %r must say what it licenses" % tag


def test_every_declared_tree_exemption_carries_a_reason():
    """The rule-2 tree exemption is declared data, not a code path.

    On the FROZEN_STRING_TREES precedent in charset_lint.py: an exemption with a
    written reason can be argued with; one expressed as an `if` in the middle of
    a scan cannot.
    """
    assert L.SYMBOL_RULE_EXEMPT_TREES, "at least the defining package is exempt"
    for tree, why in L.SYMBOL_RULE_EXEMPT_TREES.items():
        assert len(why) > 40, "exemption for %r must justify itself" % tree


def test_the_declared_exemptions_are_printed_not_merely_honoured():
    """An exemption nobody sees is a silent one.

    Mutation: stop printing the exemption list => red. The count would still be
    right and the report would no longer show which lines are exempt or why.
    """
    out = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True)
    assert "declared exemptions" in out.stdout
    assert "layers.py" in out.stdout, (
        "the one real exemption in the tree must be visible in the report"
    )


def test_the_script_has_no_undeclared_path_based_skip():
    """*** Guards against the exemption mechanism growing a second, silent form.

    The declared mechanisms are the marker and SYMBOL_RULE_EXEMPT_TREES. A
    filename or path comparison anywhere else in the scan path would be an
    exemption nobody can find by reading the declarations -- which is precisely
    what the criterion's "visible marker" requirement rules out.

    This is a text check and therefore coarse; it is here because the failure it
    guards against is someone adding `if "p1_motion" in path: return` to make a
    build green, and a coarse check does catch that.
    """
    with open(SCRIPT, encoding="utf-8") as fh:
        body = fh.read()
    # Everything after the constants, so the declarations themselves do not match.
    start = body.index("def _files_under")
    code = body[start:]
    for suspicious in (".endswith(\"p1_motion", "if \"p1_motion", "if \"xbrain/p",
                       "SKIP_FILES", "IGNORE_FILES", "EXCLUDE_FILES"):
        assert suspicious not in code, (
            "found what looks like an undeclared per-file skip: %r" % suspicious
        )


# -- the mutation the item names verbatim -------------------------------------

def _run_against(tree_root):
    """Run the lint with its ROOT pointed at a temp tree, and return (rc, out)."""
    env = dict(os.environ)
    # A subprocess with an overridden ROOT, rather than monkeypatching the
    # module in-process: main() is what CI invokes, so main() is what should be
    # exercised. Patching the module would test a path CI never takes.
    code = (
        "import sys; sys.path.insert(0, %r)\n"
        "import no_config_source_read as L\n"
        "L.ROOT = %r\n"
        "sys.exit(L.main())\n" % (os.path.join(ROOT, "scripts", "lint"), tree_root)
    )
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True,
                          text=True, env=env)
    return proc.returncode, proc.stdout


def test_the_named_mutation_turns_the_lint_red(tmp_path):
    """*** CFG-CM-11 verbatim: open the source file inside p1_motion => red.

    Built in a temp tree rather than in the repository, so a crash mid-test
    cannot leave a poisoned file behind. The case below pins that the real scan
    surface covers the same location, which is what makes the temp tree stand in
    for it.
    """
    proc_dir = tmp_path / "xbrain" / "p1_motion"
    proc_dir.mkdir(parents=True)
    (proc_dir / "loader.py").write_text(
        'def load():\n'
        '    return open("%s/p1_motion.yaml")\n' % L._CONFIG_ROOT_LITERAL,
        encoding="utf-8")

    rc, out = _run_against(str(tmp_path))
    assert rc == 1, "the lint must exit non-zero:\n" + out
    assert "p1_motion/loader.py" in out
    assert "unmarked source-root literal" in out


def test_the_symbol_form_of_the_same_mistake_turns_it_red(tmp_path):
    """The variant with no literal at all.

    os.path.join(DEFAULT_CONFIG_ROOT, ...) is the realistic way a process ends up
    reading the source, and rule 1 alone never sees it. Mutation: delete rule 2
    => red.
    """
    proc_dir = tmp_path / "xbrain" / "p1_motion"
    proc_dir.mkdir(parents=True)
    (proc_dir / "loader.py").write_text(
        'import os\n'
        'from xbrain.common.config import DEFAULT_CONFIG_ROOT\n'
        'def load():\n'
        '    return open(os.path.join(DEFAULT_CONFIG_ROOT, "p1_motion.yaml"))\n',
        encoding="utf-8")

    rc, out = _run_against(str(tmp_path))
    assert rc == 1, "the lint must exit non-zero:\n" + out
    assert "source-root symbol" in out


def test_a_process_reading_the_resolved_snapshot_is_clean(tmp_path):
    """*** The positive case.

    Without it, a lint that failed every file would satisfy both mutations above.
    This is the shape a process is supposed to have, and it must pass.
    """
    proc_dir = tmp_path / "xbrain" / "p1_motion"
    proc_dir.mkdir(parents=True)
    (proc_dir / "loader.py").write_text(
        'from xbrain.common.config.resolved import load_resolved\n'
        'def load():\n'
        '    return load_resolved("p1_motion")\n',
        encoding="utf-8")

    rc, out = _run_against(str(tmp_path))
    assert rc == 0, out


def test_the_real_scan_surface_would_have_covered_that_location():
    """What lets the temp-tree cases above stand in for the real thing.

    They exercise xbrain/p1_motion/, and this asserts the production surface
    includes xbrain/. Mutation: remove xbrain from SCAN_DIRS => red, and without
    this case the temp-tree tests would keep passing while the real tree went
    unscanned.
    """
    assert "xbrain" in L.SCAN_DIRS


# -- the lint's own probes -----------------------------------------------------

def test_the_scripts_self_test_passes():
    """The per-case probes, run as part of the suite.

    A lint with a --self-test nobody invokes is a lint whose probes rot. This is
    the cheapest way to keep them honest.
    """
    proc = subprocess.run([sys.executable, SCRIPT, "--self-test"],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "self-test: PASS" in proc.stdout


def test_the_repository_currently_passes():
    """The live run, which is what CI will gate on."""
    proc = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr


def test_the_criterion_states_what_it_cannot_establish():
    """CLAUDE.md 3.2 form 7.

    A scan cannot decide whether a process reads the source -- a path assembled
    at run time matches neither rule. The report must say so, otherwise a green
    run reads as a guarantee it is not.

    Mutation: delete the caveat from the printed criterion => red.
    """
    out = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True)
    assert "does NOT establish" in out.stdout
    assert "assembled at run time" in out.stdout


@pytest.mark.parametrize("marker_case,expect_violation", [
    ("CONFIG-SOURCE-OK(freeze): the freeze service reads the source by design", False),
    ("CONFIG-SOURCE-OK(bogus): I would rather this did not fail", True),
    ("CONFIG-SOURCE-OK(J): ok", True),
    ("just a comment mentioning the path", True),
])
def test_marker_forms(tmp_path, marker_case, expect_violation):
    """One case per way a marker can be right or wrong.

    Parametrised rather than written out, because what matters is that all four
    are checked -- a suite covering only the accepting case would bless the
    other three.
    """
    path = tmp_path / "probe.py"
    path.write_text('# %s\np = "%s/x.yaml"\n' % (marker_case, L._CONFIG_ROOT_LITERAL),
                    encoding="utf-8")
    violations, _exempt, _prose = L.scan_file(str(path), "xbrain/probe.py")
    assert bool(violations) is expect_violation, violations
