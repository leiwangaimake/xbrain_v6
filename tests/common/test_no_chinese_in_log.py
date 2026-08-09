"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_no_chinese_in_log.py
Brief: common tests -- no chinese in log

Description:
CFG-CM-15 (partial) no_chinese_in_log tests.
"""


import subprocess
import sys
from pathlib import Path


LINT = Path(__file__).parent.parent.parent / "scripts" / "lint" / "no_chinese_in_log.py"


def _run(*args):
    return subprocess.run([sys.executable, str(LINT), *args],
                          capture_output=True, text=True)


def test_self_test_passes():
    r = _run("--self-test")
    assert r.returncode == 0


def test_the_repository_currently_passes():
    r = _run()
    assert r.returncode == 0, r.stdout


def test_log_info_chinese_caught(tmp_path):
    (tmp_path / "x.py").write_text(
        "import logging\n"
        "log = logging.getLogger()\n"
        "def f(): log.info('中文 msg')\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 1
    assert "log/print" in r.stdout


def test_print_chinese_caught(tmp_path):
    (tmp_path / "x.py").write_text("def g(): print('输出')\n")
    r = _run(str(tmp_path))
    assert r.returncode == 1


def test_raise_chinese_caught(tmp_path):
    (tmp_path / "x.py").write_text(
        "def h(): raise ValueError('参数错误')\n")
    r = _run(str(tmp_path))
    assert r.returncode == 1
    assert "raise-msg" in r.stdout


def test_docstring_chinese_allowed(tmp_path):
    """Docstrings can be Chinese per CLAUDE.md 2.1 (single-file consistency)."""
    (tmp_path / "x.py").write_text(
        "def f():\n"
        "    '''这是文档字符串, 应该允许.'''\n"
        "    return 1\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 0


def test_comment_chinese_allowed(tmp_path):
    (tmp_path / "x.py").write_text("# 这是注释\ndef f(): return 1\n")
    r = _run(str(tmp_path))
    assert r.returncode == 0


def test_exempt_marker_same_line(tmp_path):
    (tmp_path / "x.py").write_text(
        "def f(): raise ValueError('参数错误')  # NO-CHINESE-LOG-LINT\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 0


def test_exempt_marker_line_above(tmp_path):
    """Multi-line raise: marker on raise line covers literal on next."""
    (tmp_path / "x.py").write_text(
        "def f():\n"
        "    raise ValueError(  # NO-CHINESE-LOG-LINT\n"
        "        '参数错误')\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 0


def test_english_log_ok(tmp_path):
    (tmp_path / "x.py").write_text(
        "import logging\n"
        "log = logging.getLogger()\n"
        "def f(): log.info('english is fine')\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 0


def test_regex_info_call_not_flagged(tmp_path):
    """re.match(...).info() would be caught by the naive attr rule
    but the value chain heuristic requires logger-adjacent."""
    (tmp_path / "x.py").write_text(
        "import re\n"
        "def f(): return re.match('x', 'y').info('中文')\n"
    )
    r = _run(str(tmp_path))
    # re.match(...).info() -- the value chain is Call.attr, root is
    # Call not Name; our heuristic wants Name(log/logger/...) or an
    # attr containing 'log'/'logger'. So this should NOT fire.
    assert r.returncode == 0
