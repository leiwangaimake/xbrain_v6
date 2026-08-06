"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_no_literal_ecode.py
Brief: CFG-CM-2 -- the Python export surface, and the metatest for its CI rule

Description:
Two halves, because the item has two halves.

THE EXPORT SURFACE. CFG-CM-2 words its criterion as "from common.errors import
E_TIMEOUT, XbrainError" being usable, retryable(E_LOCKED) answering with one of
the 11 S13.2 states, and an out-of-set code raising UnknownErrorCode rather than
returning None or degrading to something plausible. Until 2026-08-06 the first
clause was simply false: the package re-exported UnknownErrorCode and nothing
else, so the import line in the criterion raised ImportError. The cases below
pin all three so that cannot regress quietly.

Package naming, since the criterion and the tree disagree on the surface. The
documents write this library as common/errors/; on disk the Python source is
xbrain/common/errors/, because CLAUDE.md 0.2 reserves the top-level common/ for
DEPLOYED artifacts and says the shared layer's Python source does not go there.
The TODO row's target-directory column says xbrain/common/errors/, so that is the
reading used here. Nothing is asserted about a top-level common package: creating
one would break the directory rule, and asserting it does not exist would pin an
absence nobody asked for.

THE CI RULE. What needs a metatest is mostly not "does the lint find a hit" --
its own --self-test does that, one probe per case, and this file runs it. What
needs asserting is everything ABOUT the lint that could quietly stop being true:

  * the scan surface. A lint that walks one tree instead of four still prints
    zero, and zero from a shrunken surface is indistinguishable from zero from a
    clean one. So SCAN_DIRS is pinned, including that scripts/ and tests/ are
    outside it -- the lint's own text and this file both contain the pattern it
    hunts, and a surface covering them could never reach zero, which is how a
    criterion ends up loosened until it passes (CLAUDE.md 3.2 forms 2 and 3).

  * the exemption mechanism. It works only while every exemption is visible, so
    the tag set is asserted closed, the reasons are asserted substantive, and the
    declared exemptions are asserted to be PRINTED rather than merely honoured.

  * the mutation CFG-CM-2 names: one line reading code = "E_SCHEMA" inside
    services/asr/. The literal is 19 PMT-2 word for word; the landing point is
    not -- PMT-2 says perception source, and the CFG-CM-2 row moves it to
    services/asr/, where a real source file exists to hold it today. It is run
    against a temp tree here so a crash mid-test cannot leave a poisoned file in
    the service, and a separate case pins that the real surface covers that
    location -- which is what lets the temp tree stand in for it. It was also run
    once against the real tree by hand, and went red on services/asr/core/,
    before this file existed.

  * the defect this lint had while being written, kept as a regression case:
    it skipped any line holding a comment, so `code = "E_SCHEMA"  # note` was
    written off as prose. That made the whole run go green for a reason unrelated
    to the tree being clean -- CLAUDE.md 3.2 form 1. Fixed by working on tokens
    instead of lines; test_a_trailing_comment_does_not_hide_a_literal is what
    stops it coming back.
"""

import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts", "lint"))

import no_literal_ecode as L  # noqa: E402

from xbrain.common import errors  # noqa: E402

SCRIPT = os.path.join(ROOT, "scripts", "lint", "no_literal_ecode.py")


# -- the export surface the criterion words verbatim ---------------------------

def test_the_criterion_import_line_works():
    """*** The clause that was false until 2026-08-06.

    "from common.errors import E_TIMEOUT, XbrainError" -- one import line
    carrying a code and the exception base. The package used to export only
    UnknownErrorCode, so this raised ImportError, and the workaround people reach
    for is `except Exception`, which CLAUDE.md 4.5 forbids for a reason spelled
    out in exceptions.py: a handler that wide swallows an AttributeError and
    leaves the P1 loop running on assumptions that have already failed.

    Mutation: drop XbrainError from the package import or from __all__ => red.
    Run, and it fails with ImportError before this module is even collected,
    which is the loudest possible form of red.
    """
    from xbrain.common.errors import E_TIMEOUT, XbrainError  # noqa: F401

    # The value IS the wire string. Asserted rather than assumed, because the
    # binding is generated at import time from codes.yaml: a constant that
    # existed but held something else would satisfy the import above.
    assert E_TIMEOUT == "E_TIMEOUT"
    assert issubclass(XbrainError, Exception)


def test_the_whole_exception_family_comes_from_one_place():
    """One import point for the family, so one except clause stays possible.

    ClosedSetViolation is raised by xbrain/common/enums/ and UnknownErrorCode by
    this package; both inherit XbrainError. If a caller has to learn two module
    paths to catch them, the shape actually written is `except Exception`.
    """
    from xbrain.common.errors import ClosedSetViolation, UnknownErrorCode, XbrainError

    assert issubclass(UnknownErrorCode, XbrainError)
    assert issubclass(ClosedSetViolation, XbrainError)
    # And the same objects the defining module holds, not lookalikes. Two classes
    # with one name is how `except` silently stops matching.
    from xbrain.common.errors import exceptions as defining
    assert UnknownErrorCode is defining.UnknownErrorCode
    assert ClosedSetViolation is defining.ClosedSetViolation


def test_star_import_carries_the_names_the_criterion_uses():
    """__all__ is the only thing that can carry the generated code names.

    globals() injection is invisible to the default star-export rule, so a name
    missing from __all__ exists on the module and vanishes under `import *`.
    Mutation: remove XbrainError from __all__ => red.
    """
    for name in ("XbrainError", "UnknownErrorCode", "ClosedSetViolation",
                 "E_TIMEOUT", "E_LOCKED"):
        assert name in errors.__all__, name


def test_retryable_of_locked_is_one_of_the_three_states():
    """*** 11 S13.2, which calls this column a safety item and not a UX one.

    The section defines three states -- retryable, conditional, not retryable --
    and says why both misreadings cause real faults: E_LOCKED (HES not cleared)
    read as retryable makes a cloud client resend motion commands at 10 Hz, and
    E_BUSY read as not retryable makes one ordinary TTS pre-emption end hailing
    for good.

    n_a is a fourth value in codes.yaml, for OK and E_DUPLICATE, which the
    contract calls a success rather than a failure. It is deliberately excluded
    here: an E_LOCKED classified n_a would be a failure the client is told is not
    one.
    """
    three_states = {errors.RETRY_YES, errors.RETRY_CONDITIONAL, errors.RETRY_NO}
    assert errors.retryable(errors.E_LOCKED) in three_states
    assert errors.retryable(errors.E_LOCKED) != errors.RETRY_YES, (
        "E_LOCKED marked retryable would make the cloud resend motion at 10 Hz"
    )
    assert errors.retryable(errors.E_BUSY) != errors.RETRY_NO, (
        "E_BUSY marked not-retryable would make one pre-emption end hailing"
    )


def test_an_out_of_set_code_raises_and_does_not_degrade():
    """*** 11 S13.6: no pass-through and no nearest-neighbour interpretation.

    Three forbidden shapes, one case each, because they fail differently:
      * returning None -- the caller branches on a falsy value and reports
        nothing
      * returning a plausible row, E_INTERNAL being the obvious candidate --
        a contract violation becomes an ordinary-looking log line and the cloud
        client can no longer tell them apart
      * raising from only one of the accessors -- every route must reach the
        same single lookup point, or the next one added quietly becomes a hole

    Note the asymmetry that reads like a contradiction: the exception itself
    CARRIES code E_INTERNAL. That is the opposite move from degrading to it --
    execution stops -- and E_INTERNAL is the right row because reaching here
    means our own process used a code absent from our own generated table.
    """
    bogus = "E_TOTALLY_MADE_UP"
    assert bogus not in errors.ALL_CODES, "pick a value the table really lacks"

    for accessor in (errors.info, errors.retryable, errors.detail_requirement,
                     errors.is_failure):
        with pytest.raises(errors.UnknownErrorCode) as caught:
            accessor(bogus)
        # The offending value must be recoverable as data, not only as text: a
        # handler that has to parse the message back apart breaks the next time
        # somebody improves the wording.
        assert caught.value.bad == bogus
        assert bogus in str(caught.value), (
            "the message must name the value that was not in the set"
        )


def test_the_out_of_set_message_does_not_suggest_a_replacement():
    """No "did you mean" -- a printed candidate is the first step to applying it.

    11 S13.6 forbids nearest-neighbour interpretation, and the way that rule
    erodes is a helpful suggestion in the message, followed later by a patch that
    quietly acts on it. Mutation: format the known set into the message => red,
    and it would also write the set's size into logs, which CLAUDE.md 3.7 keeps
    out of them.
    """
    with pytest.raises(errors.UnknownErrorCode) as caught:
        errors.info("E_TIMEOU")  # one character off E_TIMEOUT, the tempting case
    assert "E_TIMEOUT" not in str(caught.value)


def test_a_valid_code_still_returns_its_row():
    """*** The positive case, without which an implementation that raises on
    everything passes every negative case above.

    CLAUDE.md 3.2 form 1: ask whether a do-nothing implementation would pass. One
    that raised UnknownErrorCode unconditionally would satisfy the whole section
    above and be catastrophically wrong.
    """
    row = errors.info(errors.E_TIMEOUT)
    assert row.code == errors.E_TIMEOUT
    assert row.retryable in {errors.RETRY_YES, errors.RETRY_CONDITIONAL,
                             errors.RETRY_NO, errors.RETRY_NA}
    assert errors.is_failure(errors.E_TIMEOUT)


# -- the CI rule: scan surface -------------------------------------------------

def test_scan_surface_is_the_four_trees_that_hold_shipped_source():
    """*** Pinned, because shrinking it is invisible in the output.

    Mutation: drop services/ from SCAN_DIRS => red, and the lint would go on
    printing "violations: 0" with nothing to tell a reader that the tree holding
    the item's own mutation had stopped being checked.
    """
    assert L.SCAN_DIRS == ("xbrain", "ros2_ws", "services", "common")


def test_the_lint_and_this_test_are_outside_the_scan_surface():
    """*** CLAUDE.md 3.2 form 3, 判据自伤.

    Both files necessarily contain the pattern: the lint carries it in its
    self-test cases, and this file cannot pin E_TIMEOUT == "E_TIMEOUT" without
    writing it. If scripts/ or tests/ were scanned the hit count could never
    reach zero, and the repair anyone reaches for is to loosen the rule.
    """
    assert "scripts" not in L.SCAN_DIRS
    assert "tests" not in L.SCAN_DIRS
    # And the premise, so the exclusion is load-bearing rather than a precaution
    # against nothing: both files really do contain a code literal.
    for path in (SCRIPT, os.path.abspath(__file__)):
        with open(path, encoding="utf-8") as fh:
            body = fh.read()
        assert L.CODE_RE.search(body), (
            "if %s no longer contains the pattern, check whether the exclusion "
            "is still needed rather than deleting this case" % path
        )


def test_extensions_cover_the_three_the_item_names():
    """.py, .cc and .h verbatim, plus the two spellings of the same languages.

    A rule that stopped at the extension is a rule someone evades by naming a
    file .cpp, and common/ already ships .h while xbrain/common/rtcomm/ ships
    more -- so the C++ half is not hypothetical.
    """
    for ext in (".py", ".cc", ".h"):
        assert ext in L.SOURCE_EXT
    for ext in (".cpp", ".hpp"):
        assert ext in L.SOURCE_EXT


def test_an_empty_or_absent_tree_is_reported_and_does_not_pass_for_clean():
    """ros2_ws/ holds no scannable file today.

    Without the per-directory counts the report would read as four trees checked
    and all clean, when one of them held nothing at all -- CLAUDE.md 3.2 form 6.
    Mutation: delete the counts and the NOTE => red.
    """
    out = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True)
    for tree in L.SCAN_DIRS:
        assert tree + "/" in out.stdout or "NOTE " + tree in out.stdout, (
            "every tree must appear with a file count or a NOTE saying it "
            "contributed nothing: " + tree
        )


# -- the CI rule: exemption mechanism ------------------------------------------

def test_the_exempt_tag_set_is_closed_and_each_tag_says_what_it_licenses():
    """Two tags, and neither may be handed out for convenience.

    An open tag set degrades the mechanism into "write any word and the check
    goes quiet". Mutation: add a third tag without a reason => red.
    """
    assert set(L.EXEMPT_TAGS) == {"cycle", "as12"}
    for tag, why in L.EXEMPT_TAGS.items():
        assert len(why) > 40, "tag %r must say what it licenses" % tag


def test_the_declared_exemptions_are_printed_not_merely_honoured():
    """An exemption nobody sees is a silent one.

    This case is also what caught the tokenizer defect: the E_INTERNAL exemption
    stopped appearing in this list while the run stayed green, which is what
    revealed that the line was being skipped as prose rather than exempted.
    """
    out = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True)
    assert "declared exemptions" in out.stdout
    for expected in ("exceptions.py", "rest.py"):
        assert expected in out.stdout, (
            "the real exemptions in the tree must be visible in the report: "
            + expected
        )


def test_the_script_has_no_undeclared_path_based_skip():
    """*** Guards against the mechanism growing a second, silent form.

    The declared mechanism is the marker and nothing else. A filename or path
    comparison inside the scan would be an exemption nobody can find by reading
    the declarations. Coarse by nature, and here because the failure it guards
    against is someone adding `if "rest.py" in path: return` to make a build
    green.
    """
    with open(SCRIPT, encoding="utf-8") as fh:
        body = fh.read()
    code = body[body.index("def _files_under"):]
    for suspicious in ('if "rest.py', 'if "services', ".endswith(\"rest",
                       "SKIP_FILES", "IGNORE_FILES", "EXCLUDE_FILES",
                       "EXEMPT_TREES"):
        assert suspicious not in code, (
            "found what looks like an undeclared per-file skip: %r" % suspicious
        )


# -- the CI rule: the mutation the item names verbatim -------------------------

def _run_against(tree_root):
    """Run the lint with its ROOT pointed at a temp tree; returns (rc, stdout)."""
    # A subprocess with an overridden ROOT rather than a monkeypatched module:
    # main() is what CI invokes, so main() is what gets exercised.
    code = (
        "import sys; sys.path.insert(0, %r)\n"
        "import no_literal_ecode as L\n"
        "L.ROOT = %r\n"
        "sys.exit(L.main())\n" % (os.path.join(ROOT, "scripts", "lint"), tree_root)
    )
    proc = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    return proc.returncode, proc.stdout


def test_the_named_mutation_turns_the_lint_red(tmp_path):
    """*** The mutation CFG-CM-2 names: one line in services/asr/.

    The literal is 19 PMT-2 word for word (grep 把 `"E_SCHEMA"` 字面量); the
    directory is the CFG-CM-2 row's, because PMT-2 aims at perception source and
    there is none to aim at yet.

    Built in a temp tree so a crash mid-test cannot leave the probe behind in the
    service. It was also run once against the real services/asr/core/ by hand and
    went red there, which is the run that actually retires CLAUDE.md 3.3 for this
    assertion; the case below pins that the production surface covers the same
    location.
    """
    svc = tmp_path / "services" / "asr" / "core"
    svc.mkdir(parents=True)
    (svc / "probe.py").write_text('def handle():\n    code = "E_SCHEMA"\n'
                                  '    return code\n', encoding="utf-8")

    rc, out = _run_against(str(tmp_path))
    assert rc == 1, "the lint must exit non-zero:\n" + out
    assert "services/asr/core/probe.py" in out
    assert "E_SCHEMA" in out and "string literal" in out


def test_the_real_scan_surface_would_have_covered_that_location():
    """What lets the temp tree stand in for the real one.

    Mutation: remove services from SCAN_DIRS => red. Without this case the temp
    tree cases would stay green while the real services/ went unscanned.
    """
    assert "services" in L.SCAN_DIRS
    assert os.path.isdir(os.path.join(ROOT, "services", "asr"))


def test_the_cpp_half_of_the_mutation_turns_it_red(tmp_path):
    """The same mistake in the language the criterion also names.

    Mutation: restrict SOURCE_EXT to .py => red. A Python-only rule would leave
    the deployed headers and the ROS 2 nodes free to spell codes by hand, which
    is where a drift is hardest to notice because nobody greps C++ for it.
    """
    node = tmp_path / "ros2_ws" / "src"
    node.mkdir(parents=True)
    (node / "relay.cc").write_text('const char* k = "E_BUSY";\n', encoding="utf-8")

    rc, out = _run_against(str(tmp_path))
    assert rc == 1, out
    assert "relay.cc" in out


def test_a_process_using_the_imported_constant_is_clean(tmp_path):
    """*** The positive case.

    Without it, a lint that failed every file would satisfy both mutations above
    -- the permanently-red shape that gets loosened into a permanently-green one.
    This is what correct code looks like and it must pass.
    """
    proc_dir = tmp_path / "xbrain" / "p1_motion"
    proc_dir.mkdir(parents=True)
    (proc_dir / "gate.py").write_text(
        'from xbrain.common.errors import E_TIMEOUT\n'
        'def fail():\n'
        '    return E_TIMEOUT\n', encoding="utf-8")

    rc, out = _run_against(str(tmp_path))
    assert rc == 0, out


def test_the_biz_cm_0_mutation_in_p2_core_turns_the_lint_red(tmp_path):
    """*** BIZ-CM-0 criterion 1, whose mutation names a different tree.

    The item that adds the arbitration closed sets words its own mutation as a
    hardcoded "E_BUSY" inside p2_core, where CFG-CM-2 words its one as
    code = "E_SCHEMA" inside services/asr. The two are not interchangeable: they
    land in different trees, and a SCAN_DIRS that had lost xbrain/ would still
    fail the services/ case and pass this one. E_BUSY is also the code most
    likely to be typed by hand in that process, because 11 S7A returns it from
    the arbitration denial path that p2_core implements.

    Run in a temp tree so no probe can be left behind in a real process;
    test_the_real_scan_surface_would_have_covered_p2_core pins that the
    production surface reaches the same location.
    """
    proc_dir = tmp_path / "xbrain" / "p2_core" / "arbiter"
    proc_dir.mkdir(parents=True)
    (proc_dir / "domain.py").write_text(
        'def deny():\n    return {"code": "E_BUSY"}\n', encoding="utf-8")

    rc, out = _run_against(str(tmp_path))
    assert rc == 1, "the lint must exit non-zero:\n" + out
    assert "xbrain/p2_core/arbiter/domain.py" in out
    assert "E_BUSY" in out and "string literal" in out


def test_the_real_scan_surface_would_have_covered_p2_core():
    """What lets the temp tree stand in for the real one, for the BIZ-CM-0 case.

    xbrain/p2_core/ does not exist on disk yet, so there is nothing to point at
    and no way to run the item's mutation against a real file. What CAN be
    asserted is that the surface covers the tree the process will live in, which
    is the property the temp-tree case borrows. Mutation: remove xbrain from
    SCAN_DIRS => red.

    Deliberately NOT asserted: that p2_core exists. Pinning an absence would turn
    green the day the process is written, which is the wrong direction entirely.
    """
    assert "xbrain" in L.SCAN_DIRS
    assert os.path.isdir(os.path.join(ROOT, "xbrain"))


def test_a_trailing_comment_does_not_hide_a_literal(tmp_path):
    """*** Regression case for the defect this lint shipped with for one hour.

    Original defect: scan_file skipped any line the tokenizer reported as holding
    a comment, so `code = "E_SCHEMA"  # note` was classified as prose in full and
    produced no finding. The run went green for a reason unrelated to the tree
    being clean, and the only visible symptom was a declared exemption quietly
    disappearing from the report -- CLAUDE.md 3.2 form 1, and it would have
    disarmed the item's own mutation for anyone who wrote it with a comment.

    Fixed by judging string TOKENS instead of lines (python_pieces).
    """
    svc = tmp_path / "services" / "asr"
    svc.mkdir(parents=True)
    (svc / "probe.py").write_text('code = "E_SCHEMA"  # a note, not a marker\n',
                                  encoding="utf-8")
    rc, out = _run_against(str(tmp_path))
    assert rc == 1, "a trailing comment must not turn the line into prose:\n" + out


def test_a_docstring_naming_a_code_is_not_a_violation(tmp_path):
    """The other direction, and the reason the tokenizer is used rather than a
    stricter regex.

    Documentation naming a code -- services/asr/api/rest.py does it in its module
    docstring, quoting the refusal body 11 AS-12 fixes -- is prose. A rule that
    failed on it would push people to delete the explanation, and this project's
    files are mostly explanation.

    Mutation: classify docstrings as code => red. That mutation was run, and the
    first version of this case survived it -- the probe text carried the code
    inside braces, and brace spans were being stripped from every literal rather
    than only from f-strings, so the case passed for a reason that had nothing to
    do with docstrings. The probe below carries no braces, so the only thing
    keeping it green is the classification it claims to test. The stripping bug
    it exposed is fixed in the lint and covered by its json_body probe.
    """
    svc = tmp_path / "services" / "asr"
    svc.mkdir(parents=True)
    (svc / "probe.py").write_text('"""Refuses with code E_BUSY, see 11 AS-12."""\n'
                                  'X = 1\n', encoding="utf-8")
    rc, out = _run_against(str(tmp_path))
    assert rc == 0, out


def test_a_hand_written_json_body_is_not_excused_by_its_braces(tmp_path):
    """*** The hole the docstring mutation exposed.

    A literal like '{"code": "E_BUSY"}' is the most direct way there is to put a
    code on the wire, and an implementation that strips every brace span -- as
    this lint did while it was being written -- blanks it out completely. Only an
    actual f prefix earns the strip now.

    Mutation: strip brace spans from every literal, not only f-strings => red.
    """
    svc = tmp_path / "services" / "asr"
    svc.mkdir(parents=True)
    (svc / "probe.py").write_text('BODY = \'{"code": "E_BUSY"}\'\n', encoding="utf-8")
    rc, out = _run_against(str(tmp_path))
    assert rc == 1, "a hand-written wire body must be caught:\n" + out
    assert "E_BUSY" in out


@pytest.mark.parametrize("marker,expect_violation", [
    ("ECODE-OK(cycle): the defining module cannot import its own exports", False),
    ("ECODE-OK(as12): AS-12 fixes this response body shape, 11 S11A.8.2", False),
    ("ECODE-OK(bogus): I would rather this did not fail", True),
    ("ECODE-OK(cycle): ok", True),
    ("just a comment mentioning the code", True),
])
def test_marker_forms(tmp_path, marker, expect_violation):
    """One case per way a marker can be right or wrong.

    Parametrised because what matters is that all five are checked: a suite
    covering only the accepting case would bless the other four, and two of them
    -- an invented tag and a two-letter reason -- are exactly how an exemption
    mechanism turns into an off switch.
    """
    path = tmp_path / "probe.py"
    path.write_text('# %s\ncode = "E_SCHEMA"\n' % marker, encoding="utf-8")
    violations, _exempt, _prose = L.scan_file(str(path), "services/asr/probe.py")
    assert bool(violations) is expect_violation, violations


# -- the lint's own probes and its live run ------------------------------------

def test_the_scripts_self_test_passes():
    """The per-case probes, run as part of the suite.

    A --self-test nobody invokes rots exactly as fast as a lint nobody invokes;
    comment_ratio.py in this repo was silently a no-op through one whole fix.
    """
    proc = subprocess.run([sys.executable, SCRIPT, "--self-test"],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "self-test: PASS" in proc.stdout


def test_the_repository_currently_passes():
    """The live run, which is what CI gates on."""
    proc = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True)
    assert proc.returncode == 0, proc.stdout + proc.stderr


def test_the_criterion_states_what_it_cannot_establish():
    """CLAUDE.md 3.2 form 7 -- a rule that defines its conclusion into its premise.

    A scan cannot decide whether a process invents a code: a value assembled at
    run time matches nothing. The report must say so, or a green run reads as a
    guarantee it is not. Mutation: delete the caveat from the printed criterion
    => red.
    """
    out = subprocess.run([sys.executable, SCRIPT], capture_output=True, text=True)
    assert "does NOT establish" in out.stdout
    assert "assembled at run time" in out.stdout
    assert "OK" in out.stdout, "the success code being out of scope must be stated"
