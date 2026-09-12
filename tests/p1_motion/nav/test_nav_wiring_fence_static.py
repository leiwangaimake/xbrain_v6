"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_nav_wiring_fence_static.py
Brief: the fence stage is wired -- holder into NavRuntime, compile per rev, events on event/{sev}/fence, state/fence from the tick, config refs

Description:
Text checks in the style of test_nav_wiring_static: main_wiring hands the
FenceSetHolder to NavRuntime and builds state/fence from the tick's latest
evaluation; nav_wiring compiles the active set, passes fix_type + geometry
into NavInputs, and publishes fence events under the fence category (not
motion); configs/p1_motion.yaml references the shared brake / fence truth
keys instead of copying values, and carries no blacklisted alias.
"""
from __future__ import annotations

import pathlib

import pytest

pytestmark = pytest.mark.no_device

_ROOT = pathlib.Path(__file__).resolve().parents[3]
_NAV = (_ROOT / "xbrain" / "p1_motion" / "runtime" / "nav_wiring.py").read_text(encoding="utf-8")
_MAIN = (_ROOT / "xbrain" / "p1_motion" / "runtime" / "main_wiring.py").read_text(encoding="utf-8")
_CFG = (_ROOT / "configs" / "p1_motion.yaml").read_text(encoding="utf-8")


def test_main_wiring_hands_the_holder_to_the_runtime_and_fills_state_fence():
    assert "fence_holder=fence_holder)" in _MAIN
    assert "nav_rt.latest_fence()" in _MAIN and "clip=runtime_state_fields(" in _MAIN
    assert _MAIN.index("nav_rt = None") < _MAIN.index("nav_rt = NavRuntime(")


def test_nav_wiring_compiles_per_rev_and_feeds_the_tick():
    assert "compile_fence(held, self._cfg.frame)" in _NAV
    assert "fence=self._fence_geometry()" in _NAV and "fix_type=pose.fix_type" in _NAV
    assert "fence_consts=cfg.fence" in _NAV
    assert 'self._gen.put("event/%s/fence" % fe.severity' in _NAV
    assert "self._episodes.observe(out.fence" in _NAV


def test_config_references_the_shared_truth_keys():
    for ref in ("${common.safety.brake.k}", "${common.safety.brake.a_mps2}",
                "${common.safety.t_lat_s}", "${common.fence.soft_margin_min_m}",
                "${common.fence.predict_dt_s}", "${common.fence.margin_by_fix.rtk_fixed}",
                "${common.fence.margin_by_fix.rtk_float}"):
        assert ref in _CFG, ref
    assert "projection_iters: 3" in _CFG
    for alias in ("margin_soft_m", "soft_margin_m:", "fence_margin_m"):
        assert alias not in _CFG, alias
