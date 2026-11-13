"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fence_enu_origin.py
Brief: 99 U193 -- the cmd/fence FenceSet carries NO enu_origin; P3 never sends it, P5 never reads it

Description:
User ruling 2026-11-13: "系统删除用广播的围栏的正式原点, 不再使用. HMI 显示原点
永远用有效机器人 GPS 定位作为原点" (99 U193, after U192).

~~守 2026-09-02 补的接线: FenceSet 的 enu_origin 是 11 S9A.2 必填字段, p3
必须从 common.geo.enu_origin 读出并随发布带上, p5 从 cmd/fence 采纳它为权威原点~~
(2026-09-02 ~ 2026-11-13). Why that no longer holds: no consumer projected
fences with it (P1 reads common.geo.enu_origin itself, P2 judges in metres
around the robot), and P5 adopting it drew the whole HMI map ~10 km away on
this robot, whose common.geo.enu_origin is the sim placeholder of
configs/sites/sim.yaml -- once U183 made P3 send it with every FenceSet.
The display origin is the robot's GPS fix (99 U192, test_display_origin.py).

Each case names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import inspect  # signature check
import pathlib  # source checks

import pytest  # marker

from xbrain.p3_task.fence.fence_set import build_fence_set  # under test

pytestmark = pytest.mark.no_device  # pure logic + source checks

_ROOT = pathlib.Path(__file__).resolve().parents[2]  # repo root

#: FencesDAO.list_active() row shape: (fence_id, name, role, kind, geom_json,
#: hard_enforce, rev, priority, speed_limit_mps); one activity area
_ROWS = [("f-a", "活动区", "allow", "polygon",
          '{"points": [[31.24, 121.45], [31.25, 121.46], [31.23, 121.47]]}',
          1, 1, 0, None)]


def _src(*parts):  # one source file's text
    return (_ROOT.joinpath(*parts)).read_text(encoding="utf-8")  # utf-8 (Chinese comments)


def test_the_fence_set_has_no_origin_field():
    """MUTATION: the "enu_origin" key put back (any value) -> red."""
    fs = build_fence_set(_ROWS, fence_set_id="fs-active", rev=1)  # one set
    assert "enu_origin" not in fs  # not null: absent
    assert set(fs) == {"fence_set_id", "rev", "crc32", "soft_margin_min_m", "polygons"}


def test_build_fence_set_takes_no_origin():
    """The parameter is gone, so no caller can feed one back in.
    MUTATION: the enu_origin parameter restored -> red."""
    assert "enu_origin" not in inspect.signature(build_fence_set).parameters


def test_p3_neither_reads_nor_passes_the_origin():
    """__main__ no longer reads common.geo.enu_origin for P3, and the wiring
    has no origin parameter. MUTATION: the old read / pass-through restored
    -> red."""
    main = _src("xbrain", "p3_task", "__main__.py")  # entry point
    wiring = _src("xbrain", "p3_task", "runtime", "main_wiring.py")  # wiring
    assert '_geo.get("enu_origin")' not in main and "enu_origin=_enu" not in main
    assert "enu_origin=enu_origin" not in wiring  # no call site passes it
    assert "enu_origin: Optional[Dict[str, float]] = None" not in wiring  # no parameter


def test_p5_reads_no_origin_from_cmd_fence():
    """P5's cmd/fence handler takes no origin, and the U192 log-only copy is
    gone. MUTATION: d.get("enu_origin") or config_enu_origin restored -> red."""
    src = _src("xbrain", "p5_gateway", "runtime", "main_wiring.py")  # P5 wiring
    assert 'd.get("enu_origin")' not in src  # the cmd/fence handler reads none
    assert 'hmi_state["config_enu_origin"]' not in src  # no state written
    assert '"config_enu_origin": None,' not in src  # no state key declared
    assert 'hmi_state["enu_origin_authoritative"] = True' not in src  # U192 still holds


def test_the_gps_display_origin_stays_the_only_origin():
    """99 U192 unchanged: the HMI origin comes from state/pose only.
    MUTATION: the GPS adoption removed -> red."""
    src = _src("xbrain", "p5_gateway", "runtime", "main_wiring.py")  # P5 wiring
    assert 'o = origin_from_pose(hmi_state["pose"])' in src  # the one source
    assert 'and not hmi_state.get("enu_origin_authoritative")' not in src  # not gated
