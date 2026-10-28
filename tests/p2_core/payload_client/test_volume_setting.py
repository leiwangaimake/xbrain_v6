"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_volume_setting.py
Brief: The start volume is max by default and the operator's explicit D10 value once set, across restarts

Description:
Pins the 2026-10-28 user ruling (99 U100 continued): broadcast / alarm / D
mode default to max volume, but a volume set explicitly by voice or text
stays, even across a restart. A "restart" here is a second VolumeSetting on
the same file -- exactly what a new P2 process does. Also pins the failure
direction: an unreadable state file means the DEFAULT (loud), never silence
and never a refused start. Each test names the mutation that turns it red
(CLAUDE.md 3.3).
"""

from __future__ import annotations  # postponed annotations

import json  # inspect the state file

import pytest  # fixtures and raises

from xbrain.p2_core.payload_client.volume_setting import VolumeSetting  # under test

pytestmark = pytest.mark.no_device  # pure file I/O in tmp_path


def _path(tmp_path):  # one state file per test
    """State file path inside the test's temp dir."""
    return str(tmp_path / "p2_volume.json")  # does not exist yet


def test_default_when_nothing_was_set(tmp_path):
    """No file = never set on this robot: the default (max) applies.
    MUTATION: effective() ignores the default -> fails."""
    v = VolumeSetting(_path(tmp_path), 100)  # fresh robot
    assert v.effective() == 100 and v.origin() == "default"  # max by default


def test_an_explicit_value_survives_a_restart(tmp_path):
    """The heart of the ruling. MUTATION: record() only updates memory
    (no file write) -> the second instance sees the default -> fails."""
    VolumeSetting(_path(tmp_path), 100).record(60)  # operator: "volume 60"
    again = VolumeSetting(_path(tmp_path), 100)  # P2 restarted
    assert again.effective() == 60 and again.origin() == "explicit"  # still 60


def test_mute_is_an_explicit_value_too(tmp_path):
    """0 ("mute", 18-A D10) is legal and kept. MUTATION: a falsy check
    (`if pct:`) in load treats 0 as unset -> 100 -> fails."""
    VolumeSetting(_path(tmp_path), 100).record(0)  # muted
    assert VolumeSetting(_path(tmp_path), 100).effective() == 0  # stays muted


def test_the_latest_explicit_value_wins(tmp_path):
    """MUTATION: record() keeps the first value -> fails."""
    v = VolumeSetting(_path(tmp_path), 100)  # one process
    v.record(30)  # first setting
    v.record(85)  # changed its mind
    assert v.effective() == 85  # in memory
    assert VolumeSetting(_path(tmp_path), 100).effective() == 85  # and after restart


def test_a_truncated_file_falls_back_to_the_default(tmp_path):
    """A power cut can leave a 0-byte file (seen on this robot's git
    objects). Loud side, not a crash. MUTATION: let the JSON error
    propagate -> raises -> fails."""
    (tmp_path / "p2_volume.json").write_text("")  # what a cut leaves
    v = VolumeSetting(_path(tmp_path), 100)  # must not raise
    assert v.effective() == 100 and v.origin() == "default"  # max


@pytest.mark.parametrize("content", ['{"volume_pct": 150}', '{"volume_pct": true}',
                                     '{"volume_pct": "60"}', "[60]"])
def test_foreign_content_falls_back_to_the_default(tmp_path, content):
    """Out of range / wrong type / not an object. MUTATION: drop the range
    check in load -> 150 restored -> fails."""
    (tmp_path / "p2_volume.json").write_text(content)  # hand-edited
    assert VolumeSetting(_path(tmp_path), 100).effective() == 100  # default


def test_bad_values_are_refused_and_the_file_is_untouched(tmp_path):
    """MUTATION: drop the range check in record -> 101 persisted -> fails."""
    v = VolumeSetting(_path(tmp_path), 100)  # fresh
    v.record(40)  # a good value first
    for bad in (101, -1, True, 50.0):  # each a caller bug
        with pytest.raises(ValueError):  # loud
            v.record(bad)  # refused
    assert json.loads((tmp_path / "p2_volume.json").read_text()) == {"volume_pct": 40}  # unchanged
    with pytest.raises(ValueError):  # a bad DEFAULT is a config error at start
        VolumeSetting(_path(tmp_path), 120)  # refused


def test_the_write_leaves_no_temp_file(tmp_path):
    """write-new + rename: only the final file remains. MUTATION: write the
    temp file and skip the rename -> no state file, a .new left -> fails."""
    VolumeSetting(_path(tmp_path), 100).record(70)  # one write
    assert sorted(p.name for p in tmp_path.iterdir()) == ["p2_volume.json"]  # nothing else
