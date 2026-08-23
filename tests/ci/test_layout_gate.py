"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_layout_gate.py
Brief: CHK-1-52 -- four mutation variants (a/b/c/d) each turn one rule red,
       plus a reverse-clean fixture that must stay green

Description:
Each CHK-1-52 mutation is a real file check: layout_gate.py scans a tmp
mini-repo we build in the test, and we assert which rule reports the
injected file. A rule that quietly accepted a mutant would let real
drift into the tree; a rule that reported the CLEAN case (false-positive)
would give operators alert fatigue. Both directions matter.

Mutations from CHK-1-52 verbatim:
  (a) common/boot/failure_class.py            -> CHK-1-52-A (ext gate)
  (b) deploy/whatever.py                       -> CHK-1-52-B (ext gate)
  (c) data/whatever.sh                         -> CHK-1-52-C (ext gate)
  (d) common/x.py that `import xbrain.p2_core` -> CHK-1-52-D-import
                                                  (content probe)

The scaffold builds a minimal-but-realistic layout in tmp_path and
invokes run() directly (rather than shelling out to python3) so the
test stays fast and captures the exit code and stdout in one call.
"""

import contextlib
import io
import os

import pytest

from scripts.ci.layout_gate import RULES, run

# INF-TS-1 三档 marker. 本文件是纯静态/元检查(读文件与仓库状态),
# 不碰任何硬件, 故 no_device -- 2026-08-23 从 legacy 未标记名单迁出.
pytestmark = pytest.mark.no_device


def _make_clean_scaffold(root):
    """Build a minimal green layout under `root`: common/ common/lib/
    deploy/ data/ each populated with an ALLOWED file. This is the
    baseline the clean-run test asserts is exit 0."""
    os.makedirs(root / "common" / "include" / "xbrain" / "clock", exist_ok=True)
    (root / "common" / "CMakeLists.txt").write_text("# clean build glue\n")
    (root / "common" / "include" / "xbrain" / "clock" / "clk.h").write_text(
        "// clean header\n"
    )
    os.makedirs(root / "common" / "lib", exist_ok=True)
    # A .so under lib/ MUST be ignored by CHK-1-52-A's ext gate because
    # common/lib is ignore_subroots. This is the reverse check that the
    # gate doesn't over-report build products as source.
    (root / "common" / "lib" / ".gitkeep").write_text("")
    os.makedirs(root / "deploy" / "systemd", exist_ok=True)
    (root / "deploy" / "systemd" / "xbrain.service").write_text("[Unit]\n")
    (root / "deploy" / "systemd" / "README.md").write_text("# unit index\n")
    os.makedirs(root / "data", exist_ok=True)
    (root / "data" / ".gitkeep").write_text("")
    (root / "data" / "README.md").write_text("# data layout\n")
    (root / "data" / "sounds").mkdir()
    (root / "data" / "sounds" / "beep.wav").write_bytes(b"\x00" * 16)


def _run_and_capture(tmp_path):
    """Invoke run() with stdout captured. Returns (exit_code, stdout_text)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = run(str(tmp_path))
    return rc, buf.getvalue()


# ---------------------------------------------------------------------------
# Reverse (baseline): a clean scaffold MUST exit 0 with every rule green
# ---------------------------------------------------------------------------

def test_clean_scaffold_is_green(tmp_path):
    """Baseline. If this ever fails, the tree layout drifted OR a rule
    over-reports on something innocuous -- either way, none of the
    mutant tests below are meaningful until this one holds.

    The clean scaffold is deliberately MINIMAL: common with a header +
    CMakeLists, deploy with a unit, data with gitkeep + README + a wav
    file. Adding files here is legitimate; adding files that trip a
    rule means the rule set moved and the tests need updating too.
    """
    _make_clean_scaffold(tmp_path)
    rc, out = _run_and_capture(tmp_path)
    # Fail with the captured stdout so debugging a regression is one
    # step, not two.
    assert rc == 0, "clean layout should be green; output:\n" + out
    # Sanity: every rule must have printed its "ok" line -- catches a
    # rule that silently no-op'd (its root missing) without failing.
    for rule in RULES:
        assert ("[%s]" % rule.label) in out, \
            "rule %s should have run; got:\n%s" % (rule.label, out)


# ---------------------------------------------------------------------------
# Mutation (a): a Python file under common/ -> CHK-1-52-A extension gate
# ---------------------------------------------------------------------------

def test_variant_a_python_under_common_is_red(tmp_path):
    """(a) 'common/boot/failure_class.py' -- verbatim from CHK-1-52. A .py
    file anywhere under common/ (except lib/, which is ignored) trips
    rule A: common/ is C++ only. mutation would be moving a Python
    module into common/ 'so both C++ and Python can share it', which is
    exactly what §0.2 forbids -- common/ is a DEPENDENCY, and putting
    Python in it lets chassis_relay ingest a Python type accidentally."""
    _make_clean_scaffold(tmp_path)
    # Inject: common/boot/failure_class.py -- the exact path CHK-1-52
    # names in variant (a).
    os.makedirs(tmp_path / "common" / "boot", exist_ok=True)
    injected = tmp_path / "common" / "boot" / "failure_class.py"
    injected.write_text("# should not be here\n")
    rc, out = _run_and_capture(tmp_path)
    assert rc == 1, "gate should be red; output:\n" + out
    # Absolute path in output, per CHK-1-52 star point (a).
    assert "common/boot/failure_class.py" in out
    # Rule label attribution: the finding should be attached to A.
    # Split on the rule headers so a mis-attribution is caught.
    sections = out.split("[CHK-1-52-A]")
    assert len(sections) >= 2 and \
        "common/boot/failure_class.py" in sections[1].split("[CHK-1-52")[0]


# ---------------------------------------------------------------------------
# Mutation (b): a Python file under deploy/ -> CHK-1-52-B
# ---------------------------------------------------------------------------

def test_variant_b_python_under_deploy_is_red(tmp_path):
    """(b) any .py under deploy/. deploy/ is systemd + network glue only;
    business logic (Python) goes into xbrain/. This mutation happens
    when someone writes a 'deployment helper script' and drops it into
    deploy/ instead of scripts/, which quietly makes the whole tree
    look like it has runtime code in an infra dir."""
    _make_clean_scaffold(tmp_path)
    injected = tmp_path / "deploy" / "systemd" / "helper.py"
    injected.write_text("# should not be here\n")
    rc, out = _run_and_capture(tmp_path)
    assert rc == 1, "gate should be red; output:\n" + out
    assert "deploy/systemd/helper.py" in out
    # Rule attribution.
    sections = out.split("[CHK-1-52-B]")
    assert len(sections) >= 2 and \
        "deploy/systemd/helper.py" in sections[1].split("[CHK-1-52")[0]


# ---------------------------------------------------------------------------
# Mutation (c): a shell script under data/ -> CHK-1-52-C
# ---------------------------------------------------------------------------

def test_variant_c_shell_under_data_is_red(tmp_path):
    """(c) a .sh under data/. data/ is an artifact store, not a source
    tree. Dropping a shell script there is the classic 'one-off tool I
    put next to the file it processes' anti-pattern that then gets
    cargo-culted into a real dependency."""
    _make_clean_scaffold(tmp_path)
    injected = tmp_path / "data" / "cleanup.sh"
    injected.write_text("#!/bin/sh\n# should not be here\n")
    rc, out = _run_and_capture(tmp_path)
    assert rc == 1, "gate should be red; output:\n" + out
    assert "data/cleanup.sh" in out
    sections = out.split("[CHK-1-52-C]")
    assert len(sections) >= 2 and \
        "data/cleanup.sh" in sections[1].split("[CHK-1-52")[0]


# ---------------------------------------------------------------------------
# Mutation (d): common/**.py imports xbrain.* -> CHK-1-52-D-import
# ---------------------------------------------------------------------------

def test_variant_d_common_imports_xbrain_is_red(tmp_path):
    """(d) a common/**.py that `import xbrain.p2_core`. common/ is a
    DEPENDENCY -- if it imports xbrain, the dependency arrow reverses
    and chassis_relay (which links against common/ but does NOT have
    xbrain/ in its runtime path) breaks at load. The extension gate
    (rule A) would ALSO fire on the .py extension, but the content
    probe (rule D-import) is the more specific finding and both are
    reported so a reader sees both problems, not just the outermost.
    """
    _make_clean_scaffold(tmp_path)
    # Extension gate would flag it too, so put it under `lib/` which
    # is ignored by rule A's ext gate but scanned by rule D-import.
    # The rule D-import probe does NOT ignore lib -- verify.
    #
    # Actually rule D-import ALSO ignore_subroots=('lib',), so lib is
    # exempt from both. Test the "not-lib" path instead: common/util/x.py
    os.makedirs(tmp_path / "common" / "util", exist_ok=True)
    injected = tmp_path / "common" / "util" / "wants_xbrain.py"
    injected.write_text(
        "# Should be caught by both A (extension) and D-import (content).\n"
        "import xbrain.p2_core\n"
    )
    rc, out = _run_and_capture(tmp_path)
    assert rc == 1, "gate should be red; output:\n" + out
    # Both rules should report: A on ext, D-import on content.
    sections_a = out.split("[CHK-1-52-A]")
    sections_d = out.split("[CHK-1-52-D-import]")
    assert len(sections_a) >= 2 and \
        "common/util/wants_xbrain.py" in \
        sections_a[1].split("[CHK-1-52")[0], \
        "rule A should report the .py extension violation"
    assert len(sections_d) >= 2 and \
        "common/util/wants_xbrain.py" in \
        sections_d[1].split("[CHK-1-52")[0], \
        "rule D-import should report the xbrain import"


# ---------------------------------------------------------------------------
# Ignore semantics: common/lib/ is exempt from ext gate (build products)
# ---------------------------------------------------------------------------

def test_common_lib_gitkeep_is_exempt(tmp_path):
    """common/lib/ holds build products (.so / .a). The ext gate MUST
    not fire there. If it did, adding a compiled library would break
    the layout check.
    """
    _make_clean_scaffold(tmp_path)
    # Simulate a build product: common/lib/libxbrain_common.so
    lib = tmp_path / "common" / "lib" / "libxbrain_common.so"
    lib.write_bytes(b"\x7fELF")  # ELF magic bytes; content doesn't matter
    rc, out = _run_and_capture(tmp_path)
    assert rc == 0, "common/lib/ .so should be exempt; output:\n" + out


# ---------------------------------------------------------------------------
# Missing-root tolerance: a partial checkout without data/ must not
# false-fail (rule silently skips a missing root).
# ---------------------------------------------------------------------------

def test_missing_root_is_silent_ok(tmp_path):
    """A partial checkout without a data/ tree must not fail the layout
    gate. Rule bodies check os.path.isdir(root) and return early on
    absence."""
    # Only common/ + deploy/, no data/.
    os.makedirs(tmp_path / "common", exist_ok=True)
    (tmp_path / "common" / "CMakeLists.txt").write_text("")
    os.makedirs(tmp_path / "deploy" / "systemd", exist_ok=True)
    (tmp_path / "deploy" / "systemd" / "x.service").write_text("[Unit]\n")
    rc, out = _run_and_capture(tmp_path)
    assert rc == 0, "missing root should be tolerated; output:\n" + out
