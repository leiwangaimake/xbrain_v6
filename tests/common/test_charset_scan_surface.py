"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_charset_scan_surface.py
Brief: charset_lint reaches .json / .json5 / .service and deploy/, provably in both directions

Description:
What problem this solves. CLAUDE.md 2.2 has said since 2026-08-11 that the
iron rule covers every file that is not .md / .pdf / .doc(x), and it names
".json / .json5 add to scripts/lint/charset_lint.py's scan surface" outright.
The script's extension tuple never grew: for seven weeks the document asserted
a check that did not exist. That is CLAUDE.md 3.2's fourth shape -- a verbatim
quoted as evidence when nothing had made it true -- and this file is what stops
it recurring, because a surface nobody measures is a surface that shrinks back.

Why the green half is not enough. tests/common/test_lints.py already runs
charset_lint.py and asserts exit 0. That assertion passes just as happily with
the old ten-extension tuple, with deploy/ missing from SOURCE_DIRS, and with a
walker that yields nothing at all -- CLAUDE.md 3.2's first shape. So the tests
here are the other half:

  1. reach, on the real tree. Named files that exist on disk today and were
     invisible to the linter before 2026-09-30 must now be yielded. If someone
     trims SOURCE_EXT or drops deploy/ from SOURCE_DIRS, this is what says so.
  2. red, end to end. One file per newly admitted extension, each carrying a
     banned character, under a synthetic root -- main() must return 1 and must
     name every one of them. This exercises iter_sources, comment_lines_of,
     scan and the criterion together, which is the path a real violation takes.
  3. the exclusions exclude, and only what the ruling names. Each exclusion is
     paired with a positive control: the same bytes under a name the ruling
     does not exempt ARE reported. Without that pairing an exclusion test is
     satisfied by a walker that finds nothing.

Boundaries. This file does not assert the repository is clean -- that is
test_lints.py's charset_lint entry, and keeping the two apart matters: one says
"the rule holds today", this one says "the rule is capable of failing". It does
not test the vendored-tree exclusion either; test_third_party_exclusion.py owns
that list and its own positive control.

The looks-right-but-wrong writing this file must avoid. Spelling a forbidden
character literally anywhere in here would put the criterion inside its own
scan surface -- charset_lint walks tests/ -- and the file would report itself
on every run. That is CLAUDE.md 3.2's third shape, which this project has
caught three times. Every banned character below is built with chr() from its
codepoint, the way charset_lint builds its own tables.
"""

import importlib.util
import io
import os
import sys

import pytest

# INF-TS-1: every test file carries a module-level hardware marker. This one
# reads files on disk and writes into tmp_path, so no_device is the honest
# value.
pytestmark = pytest.mark.no_device

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LINT_DIR = os.path.join(ROOT, "scripts", "lint")

# FULLWIDTH COMMA. By codepoint, never as a literal -- see the header note on
# the self-harming criterion.
BANNED = chr(0xFF0C)


def _load_charset_lint():
    """Import scripts/lint/charset_lint.py by path.

    The lint scripts are not a package and each inserts scripts/lint into
    sys.path for the shared import, so LINT_DIR goes first here too; that makes
    `import charset_lint` inside charset_fix resolve to the same module object.
    """
    if LINT_DIR not in sys.path:
        sys.path.insert(0, LINT_DIR)
    spec = importlib.util.spec_from_file_location(
        "charset_lint", os.path.join(LINT_DIR, "charset_lint.py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules["charset_lint"] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def cl():
    return _load_charset_lint()


# Real files that the pre-2026-09-30 surface could not see, one per reason.
# Named individually rather than counted: a count tells you the surface changed
# but not which half of it, and CLAUDE.md 3.7 keeps measured numbers out of the
# source anyway.
INVISIBLE_BEFORE = (
    # .json5 AND deploy/ -- both were missing, either alone would hide it
    "deploy/zenoh/zenohd-gen.json5",
    "deploy/zenoh/zenohd-rt.json5",
    # .service, the extension CLAUDE.md 2.2 does not name but its scope covers
    "deploy/systemd/xbrain-llm.service",
    "deploy/systemd/xbrain-p1-motion.service",
    # .mount, same file class as the units beside it
    "deploy/systemd/run-xbrain.mount",
    # .json in a directory that was already on the surface: the extension alone
    # was what hid this one, and it held the largest single pile of violations
    "scripts/doccheck/scan_manifest.json",
    # .json5 under configs/, the tree the 2026-08-12 widening already covered
    # for .yaml only
    "configs/zenoh/router_rt.json5",
)


@pytest.mark.parametrize("rel", INVISIBLE_BEFORE)
def test_a_file_the_old_surface_could_not_see_is_now_walked(cl, rel):
    """Reach, on the real tree, file by file.

    Both halves are asserted. The file must exist -- an exclusion test whose
    subject has been deleted quietly becomes vacuous -- and iter_sources must
    yield it.

    mutant: drop ".service" from SOURCE_EXT, or "deploy" from SOURCE_DIRS ->
    the matching parameters here go red while charset_lint.py still exits 0,
    which is exactly the state that lasted seven weeks.
    """
    assert os.path.isfile(os.path.join(ROOT, rel)), rel
    walked = {os.path.relpath(p, ROOT) for p in cl.iter_sources()}
    assert rel in walked, (
        "%s is inside CLAUDE.md 2.2's declared surface but charset_lint does "
        "not walk it" % rel)


def _write(root, rel, text):
    """Create root/rel with text, making parents as needed."""
    path = os.path.join(root, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    io.open(path, "w", encoding="utf-8").write(text)
    return path


def _run_main(cl, monkeypatch, capsys, root, dirs):
    """main() over a synthetic root, returning (exit code, printed output).

    iter_sources reads ROOT and SOURCE_DIRS at call time, so patching the two
    module attributes is enough to redirect the whole walk. Everything else --
    the extension tuple, the exclusions, the comment/string split, the
    criterion -- runs exactly as it does on the repository.
    """
    monkeypatch.setattr(cl, "ROOT", root)
    monkeypatch.setattr(cl, "SOURCE_DIRS", dirs)
    rc = cl.main()
    return rc, capsys.readouterr().out


# One file per extension admitted on 2026-09-30, each with the violation in the
# place that extension actually keeps prose: a // comment for json5, a #
# comment for a unit file, a bare string value for json (which has no comments
# at all, so its hits can only ever land in the string bucket).
NEW_EXT_CASES = (
    ("deploy/zenoh/router.json5", "// note%s two\n{ a: 1 }\n" % BANNED),
    ("deploy/systemd/x.service", "# note%s two\n[Unit]\n" % BANNED),
    ("deploy/systemd/x.mount", "# note%s two\n[Mount]\n" % BANNED),
    ("scripts/doccheck/m.json", '{"note": "one%s two"}\n' % BANNED),
)


def test_a_violation_in_each_newly_admitted_extension_turns_the_run_red(
        cl, monkeypatch, capsys, tmp_path):
    """The gate can fail, once per new extension, through main() itself.

    Run as one case rather than parametrised, because the thing worth proving
    is that a single run reports ALL of them: a walker that stopped at the
    first match would pass four separate parametrised cases and still hide
    three violations from anyone reading the output.

    The synthetic root keeps this off the repository. Writing the probe file
    into the real tree would leave a dirty file behind on any failure, and a
    parallel session's `git add -A` has swallowed a stray file here before.
    """
    root = str(tmp_path)
    for rel, text in NEW_EXT_CASES:
        _write(root, rel, text)
    rc, out = _run_main(cl, monkeypatch, capsys, root, ("deploy", "scripts"))

    assert rc == 1, out
    for rel, _text in NEW_EXT_CASES:
        assert rel in out, (
            "%s carries a banned character and the run did not name it:\n%s"
            % (rel, out))


def test_the_same_bytes_without_the_banned_character_stay_green(
        cl, monkeypatch, capsys, tmp_path):
    """The negative control for the test above.

    Without this, "rc == 1" would also be satisfied by a criterion that is red
    on every input, which CLAUDE.md 3.2 lists as its own failure shape: an
    always-red check gets loosened until it is always green.
    """
    root = str(tmp_path)
    for rel, text in NEW_EXT_CASES:
        _write(root, rel, text.replace(BANNED, ", "))
    rc, out = _run_main(cl, monkeypatch, capsys, root, ("deploy", "scripts"))
    assert rc == 0, out


# Each exclusion, paired with the control that proves the walker would
# otherwise have found it. rel_hidden and rel_found hold identical bytes and
# differ only in the name CLAUDE.md 2.2 exempts.
EXCLUSION_CASES = (
    # "golden 测试向量" -- frozen vectors are input data under measurement, so
    # their punctuation is evidence rather than our prose
    ("tests/common/golden/vectors.json", "tests/common/live/vectors.json"),
    # freeze output, and iron rule 2 forbids hand-editing it
    ("configs/MANIFEST.json", "configs/manifest_source.json"),
    # CMake writes this one on every configure
    ("ros2_ws/q/build/compile_commands.json", "ros2_ws/q/build/db.json"),
    # the ASR golden corpus, named by path because it is not under a directory
    # called golden
    ("services/asr/selftest/gold.json", "services/asr/selftest/notes.json"),
)


@pytest.mark.parametrize("rel_hidden,rel_found", EXCLUSION_CASES)
def test_an_exclusion_hides_its_own_file_and_nothing_else(
        cl, monkeypatch, capsys, tmp_path, rel_hidden, rel_found):
    """Both directions of one exclusion, from identical bytes.

    The positive control is the half that carries the weight (CLAUDE.md 3.3).
    "The excluded file is not reported" is also true of a walker that reports
    nothing, of a root with no files in it, and of an extension tuple that
    never admitted .json -- so the same content under a non-exempt name has to
    come back red in the same run.

    mutant: empty EXCLUDED_DIR_NAMES, EXCLUDED_BASENAMES or EXCLUDED_PATHS ->
    the corresponding parameter goes red on its first assertion.
    """
    root = str(tmp_path)
    text = '{"note": "one%s two"}\n' % BANNED
    _write(root, rel_hidden, text)
    _write(root, rel_found, text)
    dirs = ("tests", "configs", "ros2_ws", "services")
    rc, out = _run_main(cl, monkeypatch, capsys, root, dirs)

    assert rel_hidden not in out, (
        "CLAUDE.md 2.2 puts %s outside the scan surface:\n%s" % (rel_hidden, out))
    assert rel_found in out, (
        "the control file holds the same bytes under a name the ruling does "
        "not exempt; if it is not reported, the exclusion above proves "
        "nothing:\n%s" % out)
    assert rc == 1, out


def test_a_data_file_beside_a_frozen_script_is_not_declared_debt(
        cl, monkeypatch, capsys, tmp_path):
    """is_frozen is Python-only, and that is what makes .json enforceable.

    FROZEN_STRING_TREES lists scripts/doccheck because those scripts print
    Chinese reports a human reads; rewriting the strings would change what they
    print. A data file next door prints nothing and has no such claim, but the
    tree prefix matched it all the same -- so scripts/doccheck/scan_manifest
    .json's 141 violations would have been filed as declared debt and the run
    would have stayed green. An extension that can report nothing is an
    extension that was not added.

    mutant: drop the .py guard in is_frozen -> rc becomes 0 here.

    The synthetic path has to be the real prefix, scripts/doccheck, not just
    a directory called doccheck. An earlier draft of this test used the short
    name, so is_frozen never matched it and the mutant above left the test
    green -- a mutation the assertion claimed to catch and did not. That is
    the whole failure mode CLAUDE.md 3.3 exists for, met while writing the
    check for it.
    """
    root = str(tmp_path)
    _write(root, "scripts/doccheck/m.json", '{"note": "one%s two"}\n' % BANNED)
    rc, out = _run_main(cl, monkeypatch, capsys, root, ("scripts",))
    assert cl.FROZEN_STRING_TREES.get("scripts/doccheck"), (
        "this test aims at the scripts/doccheck entry; if it is gone the test "
        "is vacuous rather than passing")
    assert "[frozen]" not in out, out
    assert rc == 1, out


def test_a_json5_comment_is_counted_as_a_comment_not_as_a_string(
        cl, monkeypatch, capsys, tmp_path):
    """The classify_lines guard, seen from the bucket it decides.

    The Python tokenizer reads // as floor division and returns a valid token
    stream for a .json5 file in which no comment line is a COMMENT. Under that
    reading the four CJK characters in a router config land in the string
    bucket -- the one that can be frozen debt -- instead of the comment bucket
    the criterion enforces. The misclassification is silent, so it is asserted
    on the printed split rather than only on the exit code.

    mutant: remove the .py guard in charset_fix.classify_lines -> the counts
    below swap and this goes red.
    """
    root = str(tmp_path)
    _write(root, "zenoh/r.json5", "// note%s two\n{ a: 1 }\n" % BANNED)
    rc, out = _run_main(cl, monkeypatch, capsys, root, ("zenoh",))
    assert rc == 1, out
    line = [ln for ln in out.split("\n") if "zenoh/r.json5" in ln and "comments" in ln]
    assert line, out
    assert "comments   1" in line[0] and "strings   0" in line[0], line[0]
