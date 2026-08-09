"""INF-CI-2 variant 2 -- File field mismatch check in header_lint."""

import subprocess
import sys
from pathlib import Path
import tempfile
import os

# Load header_lint by path.
LINT_PATH = Path(__file__).parent.parent.parent / "scripts" / "lint" / "header_lint.py"


def _make_file(dir_, name, declared_file):
    """Write a stub with correct header except File field."""
    path = os.path.join(dir_, name)
    with open(path, "w") as f:
        f.write('"""\n')
        f.write("Copyright (c) 2026 Hachist Robotics\n")
        f.write("Author: wanglei@hachist.com\n")
        f.write("上海哈船智能船舶技术有限公司\n")
        f.write("File: %s\n" % declared_file)
        f.write("Brief: a short brief that is more than 8 chars\n")
        f.write("\n")
        f.write("Description:\n")
        f.write("Substantial description here explaining what this is.\n")
        f.write('"""\n')
        f.write("def foo(): pass\n")
    return path


def test_matching_file_field_passes():
    """A file whose header File: field matches the basename is clean."""
    # Import the checker directly for a unit-style test.
    sys.path.insert(0, str(LINT_PATH.parent))
    try:
        import header_lint as hl
    finally:
        sys.path.pop(0)
    with tempfile.TemporaryDirectory() as td:
        p = _make_file(td, "foo.py", "foo.py")
        problems, has_header = hl.check(p)
        assert has_header
        assert not any("File field" in x for x in problems), problems


def test_mismatched_file_field_caught():
    """CFG-CI-2 variant 2 verbatim: File field naming a DIFFERENT file
    is a copy-paste header and must be flagged."""
    sys.path.insert(0, str(LINT_PATH.parent))
    try:
        import header_lint as hl
    finally:
        sys.path.pop(0)
    with tempfile.TemporaryDirectory() as td:
        p = _make_file(td, "foo.py", "bar.py")   # File says bar, actual is foo
        problems, has_header = hl.check(p)
        assert has_header
        assert any("File field" in x for x in problems), \
            "expected File field mismatch, got: %s" % problems


def test_absent_file_field_not_falsely_matched():
    """A file whose header lacks a File field falls to the 'missing field'
    branch, not the mismatch branch."""
    sys.path.insert(0, str(LINT_PATH.parent))
    try:
        import header_lint as hl
    finally:
        sys.path.pop(0)
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, "foo.py")
        with open(path, "w") as f:
            f.write('"""\n')
            f.write("Copyright (c) 2026 Hachist Robotics\n")
            f.write("Author: wanglei@hachist.com\n")
            f.write("上海哈船智能船舶技术有限公司\n")
            # File field deliberately omitted.
            f.write("Brief: a short brief that is more than 8 chars\n")
            f.write("Description:\nx\n")
            f.write('"""\n')
        problems, has_header = hl.check(path)
        # Should report missing File, not report File field mismatch.
        assert any("missing field: File" in x for x in problems)
        assert not any("File field" in x for x in problems)
