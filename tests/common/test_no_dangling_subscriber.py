"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_no_dangling_subscriber.py
Brief: common tests -- no dangling subscriber

Description:
CFG-DC-3 part 2 no_dangling_subscriber tests.
"""


import subprocess
import sys
from pathlib import Path

import pytest


pytestmark = pytest.mark.no_device


LINT = Path(__file__).parent.parent.parent / "scripts" / "lint" / "no_dangling_subscriber.py"


def _run(*args):
    return subprocess.run([sys.executable, str(LINT), *args],
                          capture_output=True, text=True, timeout=60)


def test_self_test_passes():
    r = _run("--self-test")
    assert r.returncode == 0, r.stdout


def test_the_repository_currently_passes():
    r = _run()
    assert r.returncode == 0, r.stdout


def test_bare_call_caught(tmp_path):
    (tmp_path / "x.py").write_text(
        "class C:\n"
        "    def go(self, s):\n"
        "        s.declare_subscriber('k', lambda m: None)\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 1


def test_underscore_discard_caught(tmp_path):
    (tmp_path / "x.py").write_text(
        "def go(s):\n"
        "    _ = s.declare_subscriber('k', lambda m: None)\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 1


def test_self_attribute_ok(tmp_path):
    (tmp_path / "x.py").write_text(
        "class C:\n"
        "    def go(self, s):\n"
        "        self.sub = s.declare_subscriber('k', lambda m: None)\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 0


def test_list_append_ok(tmp_path):
    (tmp_path / "x.py").write_text(
        "class C:\n"
        "    def go(self, s):\n"
        "        self.subs.append(s.declare_subscriber('k', lambda m: None))\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 0


def test_registry_declare_ok(tmp_path):
    """SubscriberRegistry.declare(...) wrapping is legal capture."""
    (tmp_path / "x.py").write_text(
        "class C:\n"
        "    def go(self, s):\n"
        "        self._subs.declare(s, 'k', lambda m: None)\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 0


def test_exempt_marker(tmp_path):
    (tmp_path / "x.py").write_text(
        "def go(s):\n"
        "    s.declare_subscriber('k', lambda m: None)  # NO-DANGLING-SUB-LINT\n"
    )
    r = _run(str(tmp_path))
    assert r.returncode == 0
