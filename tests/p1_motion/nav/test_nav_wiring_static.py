"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_nav_wiring_static.py
Brief: P7.2 wiring presence -- keys subscribed / published, single sink, teardown, config gate

Description:
Same discipline as test_p1_estop's wiring check: the runtime cannot be run
under pytest (it needs two zenohd routers), so the assertions that CAN be
made offline are made on the source text and on the importable pieces:
the three general-plane subscriptions and three publishers exist by their
contract key names, cmd_vel is published from exactly one place (12 S2.2 step
10), every declared subscriber is held in a list and undeclared, __main__
hands the NavConfig (or None, loudly) to the wiring, and unwrap_body accepts
both a bare body and an 11 S3.0 envelope.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

from xbrain.p1_motion.runtime.nav_wiring import (CMD_FACTOR_TOPIC, CMD_RELMOVE_TOPIC,
                                                 CMD_ROUTE_TOPIC, RELMOVE_STATUS_TOPIC,
                                                 STATE_PROGRESS_TOPIC, TICK_PERIOD_S,
                                                 unwrap_body)

pytestmark = pytest.mark.no_device

_P1 = Path(__file__).resolve().parents[3] / "xbrain" / "p1_motion"
_NAV = (_P1 / "runtime" / "nav_wiring.py").read_text(encoding="utf-8")
_MAIN = (_P1 / "runtime" / "main_wiring.py").read_text(encoding="utf-8")
_ENTRY = (_P1 / "__main__.py").read_text(encoding="utf-8")


def test_contract_keys():
    assert (CMD_ROUTE_TOPIC, CMD_RELMOVE_TOPIC, CMD_FACTOR_TOPIC) == (
        "cmd/motion/route", "cmd/motion/relative_move", "cmd/motion/factor")
    assert (STATE_PROGRESS_TOPIC, RELMOVE_STATUS_TOPIC) == (
        "state/motion/path_progress", "cmd/motion/relative_move/status")
    assert TICK_PERIOD_S == 0.05


def test_subscriptions_are_held_and_undeclared():
    """CLAUDE.md 4.3: every declare_subscriber lands in self._subs; stop()
    undeclares them. mutant: drop one append -> the count red."""
    subs = re.findall(r"self\._subs\.append\(self\._gen\.declare_subscriber\((\w+)", _NAV)
    assert sorted(subs) == ["CMD_FACTOR_TOPIC", "CMD_RELMOVE_TOPIC", "CMD_ROUTE_TOPIC"]
    assert "s.undeclare()" in _NAV


def test_cmd_vel_has_a_single_sink():
    """12 S2.2 step 10: one publisher of rt/motion/cmd_vel, called from the
    CtrlLoop callback only."""
    assert _NAV.count('declare_publisher("xbrain/%s/rt/motion/cmd_vel" % self._rid)') == 1
    assert _NAV.count("self._cmd_pub.put(") == 1
    assert "rt/motion/cmd_vel" not in _MAIN.replace("rt/motion/cmd_vel pub", "")


def test_main_wiring_starts_the_runtime_only_with_rid_and_config():
    assert "nav_rt = NavRuntime(" in _MAIN
    assert "nav_rt.declare()" in _MAIN and "nav_rt.start()" in _MAIN
    assert "nav_rt.stop()" in _MAIN
    assert "teleop_lock = threading.Lock()" in _MAIN
    assert _MAIN.count("with teleop_lock:") == 2
    assert 'gnss_cache["rx"] = time.monotonic()' in _MAIN
    assert 'fix_cache["rx"] = time.monotonic()' in _MAIN


def test_entry_builds_nav_config_from_both_snapshots_and_never_defaults():
    assert 'load_resolved("rns"' in _ENTRY
    assert "build_nav_config(_cfg.tree, _rns.tree)" in _ENTRY
    assert "nav_cfg=nav_cfg" in _ENTRY
    assert "p1 nav loop DISABLED" in _ENTRY


def test_unwrap_body_accepts_bare_and_enveloped():
    body = {"cmd_id": "rm-1", "dx_m": 1.0}
    assert unwrap_body(body) is body
    env = {"v": 1, "rid": "dev", "ts": 1.0, "seq": 3, "src": "p2_core", "data": body}
    assert unwrap_body(env) is body
    assert unwrap_body(None) is None
