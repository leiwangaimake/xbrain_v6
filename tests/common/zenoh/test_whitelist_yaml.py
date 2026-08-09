"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_whitelist_yaml.py
Brief: zenoh tests -- whitelist yaml

Description:
CFG-BT-17 -- whitelist yaml loader tests.
"""


import subprocess
import sys
from pathlib import Path

import pytest
import yaml


pytestmark = pytest.mark.no_device


REPO_ROOT = Path(__file__).parent.parent.parent.parent
YAML_PATH = REPO_ROOT / "configs" / "generated" / "whitelist.yaml"


def test_yaml_file_exists():
    assert YAML_PATH.is_file(), "whitelist yaml missing at %s" % YAML_PATH


def test_yaml_parses():
    with open(YAML_PATH, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert isinstance(data, dict)
    assert "processes" in data
    assert isinstance(data["processes"], dict)


def test_all_five_processes_present():
    with open(YAML_PATH, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    procs = data["processes"]
    for expected in ("perception", "p1_motion", "chassis_relay",
                     "p2_core", "p4_agent"):
        assert expected in procs, "missing process: %s" % expected


def test_every_entry_has_pub_and_sub():
    with open(YAML_PATH, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    for proc, sets in data["processes"].items():
        assert "pub" in sets, "%s missing pub" % proc
        assert "sub" in sets, "%s missing sub" % proc
        assert isinstance(sets["pub"], list)
        assert isinstance(sets["sub"], list)


def test_module_import_matches_yaml():
    """The module-level constants come from the yaml. Verify each
    matches what a fresh yaml read would produce."""
    with open(YAML_PATH, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    procs = data["processes"]
    from xbrain.common.zenoh import whitelists as W
    for name in ("perception", "p1_motion", "chassis_relay",
                 "p2_core", "p4_agent"):
        yaml_pub = frozenset(procs[name]["pub"])
        yaml_sub = frozenset(procs[name]["sub"])
        code_pub = getattr(W, name.upper() + "_PUB")
        code_sub = getattr(W, name.upper() + "_SUB")
        assert code_pub == yaml_pub, \
            "%s PUB mismatch: yaml=%s code=%s" % (
                name, yaml_pub - code_pub, code_pub - yaml_pub)
        assert code_sub == yaml_sub, "%s SUB mismatch" % name


def test_missing_yaml_raises_at_import(tmp_path, monkeypatch):
    """Renaming the yaml would break import loudly, not silently."""
    fake_repo = tmp_path
    (fake_repo / "configs" / "generated").mkdir(parents=True)
    # Absent yaml: _load_yaml() must raise.
    from xbrain.common.zenoh import whitelists as W
    monkeypatch.setattr(W, "_repo_root", lambda: fake_repo)
    with pytest.raises(RuntimeError, match="whitelist yaml missing"):
        W._load_yaml()


def test_malformed_yaml_raises(tmp_path, monkeypatch):
    """A yaml missing the 'processes' key raises."""
    fake_repo = tmp_path
    (fake_repo / "configs" / "generated").mkdir(parents=True)
    (fake_repo / "configs" / "generated" / "whitelist.yaml").write_text(
        "schema_version: 1\n"
    )
    from xbrain.common.zenoh import whitelists as W
    monkeypatch.setattr(W, "_repo_root", lambda: fake_repo)
    with pytest.raises(RuntimeError, match="missing top-level 'processes'"):
        W._load_yaml()


def test_missing_process_raises(tmp_path, monkeypatch):
    """A yaml with only 3 processes raises, not silently defaults."""
    fake_repo = tmp_path
    (fake_repo / "configs" / "generated").mkdir(parents=True)
    (fake_repo / "configs" / "generated" / "whitelist.yaml").write_text(
        yaml.safe_dump({
            "schema_version": 1,
            "processes": {
                "perception": {"pub": [], "sub": []},
                "p1_motion": {"pub": [], "sub": []},
                "chassis_relay": {"pub": [], "sub": []},
            }
        })
    )
    from xbrain.common.zenoh import whitelists as W
    monkeypatch.setattr(W, "_repo_root", lambda: fake_repo)
    with pytest.raises(RuntimeError, match="missing processes"):
        W._load_yaml()


def test_frozensets_are_immutable():
    from xbrain.common.zenoh.whitelists import P1_MOTION_PUB
    with pytest.raises((AttributeError, TypeError)):
        P1_MOTION_PUB.add("new/key")


def test_source_note_present():
    """The yaml should self-document its source (CFG-BT-17 traceability)."""
    with open(YAML_PATH, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert data.get("source_note"), "source_note field missing"
    assert "CFG-BT-17" in data["source_note"], \
        "source_note should reference CFG-BT-17 traceability"
