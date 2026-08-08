"""CFG-DC-3 (partial) no_config_singular tests."""

import subprocess
import sys
from pathlib import Path


LINT = Path(__file__).parent.parent.parent / "scripts" / "lint" / "no_config_singular.py"


def _run(*args):
    return subprocess.run([sys.executable, str(LINT), *args],
                          capture_output=True, text=True)


def test_self_test_passes():
    assert _run("--self-test").returncode == 0


def test_the_repository_currently_passes():
    r = _run()
    assert r.returncode == 0, r.stdout


def test_singular_yaml_path_caught(tmp_path):  # NO-CONFIG-SINGULAR-LINT
    (tmp_path / "x.py").write_text('p = "config/foo.yaml"\n')  # NO-CONFIG-SINGULAR-LINT
    r = _run(str(tmp_path))
    assert r.returncode == 1


def test_plural_yaml_path_ignored(tmp_path):
    (tmp_path / "x.py").write_text('p = "configs/foo.yaml"\n')
    assert _run(str(tmp_path)).returncode == 0


def test_zenoh_key_not_flagged(tmp_path):
    """Zenoh key names like cmd/config/ack must not fire (they are not paths)."""
    (tmp_path / "x.py").write_text('k = "cmd/config/ack"\n')
    assert _run(str(tmp_path)).returncode == 0


def test_python_package_path_not_flagged(tmp_path):
    """Module path xbrain.common.config in a string is not a filesystem path."""
    (tmp_path / "x.py").write_text('m = "xbrain.common.config"\n')
    assert _run(str(tmp_path)).returncode == 0


def test_absolute_deploy_path_caught(tmp_path):  # NO-CONFIG-SINGULAR-LINT
    (tmp_path / "x.py").write_text('p = "/opt/xbrain_v6/config/foo.yaml"\n')  # NO-CONFIG-SINGULAR-LINT
    assert _run(str(tmp_path)).returncode == 1


def test_exempt_marker(tmp_path):  # NO-CONFIG-SINGULAR-LINT
    (tmp_path / "x.py").write_text(
        'p = "config/foo.yaml"  # NO-CONFIG-SINGULAR-LINT\n')  # NO-CONFIG-SINGULAR-LINT
    assert _run(str(tmp_path)).returncode == 0
