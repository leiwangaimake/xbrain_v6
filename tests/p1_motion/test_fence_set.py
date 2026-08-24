"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fence_set.py
Brief: P1 FenceSet 接收/自算比对/持有 (11 S9A.2/S9A.3, 报警 F1)

Description:
守 P1 侧围栏接收的四条不变式: crc32 自算比对(报文损坏必拒), role 闭集越界必抛, 
少于 3 顶点必拒, FS-7 坏帧保留旧 active. 每条配一个会让它变红的变异体.
"""

import pytest

from xbrain.common.fence.geom import fence_set_crc32
from xbrain.p1_motion.fence.fence_set import (FenceSetError, FenceSetHolder,
                                              build_fence_runtime_state,
                                              compile_fence_set)

_RING = [{"lat": 34.697, "lon": 135.505}, {"lat": 34.698, "lon": 135.505},
         {"lat": 34.698, "lon": 135.506}]


def _wire(polys=None, fence_set_id="fs-active", rev=1):
    """一份[合法]的 wire FenceSet: crc32 用共享库算真值(与 p3 广播同源)."""
    if polys is None:
        polys = [{"poly_id": "f-allow", "role": "allow", "name": "camp",
                  "winding": "ccw", "hard_enforce": False, "vertices": _RING},
                 {"poly_id": "f-zone", "role": "warning", "name": "gate",
                  "winding": "cw", "hard_enforce": False, "vertices": _RING}]
    crc = fence_set_crc32(fence_set_id, rev, polys)
    return {"fence_set_id": fence_set_id, "rev": rev, "crc32": crc,
            "enu_origin": None, "soft_margin_min_m": None, "polygons": polys}


def test_a_valid_fence_set_compiles_and_holds():
    """合法 FenceSet 换入 active, rev/crc32/多边形数对上."""
    holder = FenceSetHolder()
    assert holder.active is None
    held = holder.accept(_wire())
    assert holder.active is held
    assert held.fence_set_id == "fs-active" and held.rev == 1
    assert len(held.polygons) == 2
    # 报警区抽取: 只 role==warning 的那一个进 zone 判定.
    assert [p.poly_id for p in held.warning_polygons()] == ["f-zone"]


def test_crc32_mismatch_is_rejected():
    """*** wire.crc32 与自算不符 = 报文损坏, 必拒(S9A.2 FV-8).

    MUTATION: compile_fence_set 跳过 crc32 比对(信 wire.crc32) -> 这里不抛 -> 红.
    """
    bad = _wire()
    bad["crc32"] = "deadbeef"                        # 改坏声明的 crc32
    with pytest.raises(FenceSetError):
        compile_fence_set(bad)


def test_unknown_role_is_rejected_not_degraded():
    """*** role 闭集越界必抛(CLAUDE.md 3.5), 不静默当普通围栏.

    MUTATION: _parse_polygon 把未知 role 放行 -> 这里不抛 -> 红.
    """
    polys = [{"poly_id": "f-allow", "role": "allow", "name": "c",
              "winding": "ccw", "hard_enforce": False, "vertices": _RING},
             {"poly_id": "f-x", "role": "restricted", "name": "x",   # 越界 role
              "winding": "cw", "hard_enforce": False, "vertices": _RING}]
    with pytest.raises(FenceSetError):
        compile_fence_set(_wire(polys=polys))


def test_polygon_with_fewer_than_three_vertices_is_rejected():
    """< 3 顶点围不出面积(FV-1), 点在内恒 False -> 报警区静默失效, 必拒."""
    polys = [{"poly_id": "f-allow", "role": "allow", "name": "c",
              "winding": "ccw", "hard_enforce": False,
              "vertices": [{"lat": 34.0, "lon": 135.0},
                           {"lat": 34.1, "lon": 135.0}]}]           # 只 2 点
    with pytest.raises(FenceSetError):
        compile_fence_set(_wire(polys=polys))


def test_fs7_bad_frame_keeps_old_active():
    """*** FS-7: 坏帧保留旧 active, 绝不进入"无围栏"态(fail-open).

    MUTATION: accept() 在 compile 前先清空 self._active -> 坏帧后 active 变 None
    -> 这里 active is good 的断言红.
    """
    holder = FenceSetHolder()
    good = holder.accept(_wire(rev=1))
    # 来一帧坏的(crc32 不符) -> accept 抛, 但 active 必须还是那份好的.
    bad = _wire(rev=2)
    bad["crc32"] = "00000000"
    with pytest.raises(FenceSetError):
        holder.accept(bad)
    assert holder.active is good, "坏帧把 active 冲掉了 -- 违反 FS-7(不进无围栏态)"


def test_rev_bump_swaps_active_atomically():
    """新 rev 的合法帧换入, active 指到新集(F3 广播 active.rev 的依据)."""
    holder = FenceSetHolder()
    holder.accept(_wire(rev=1))
    new = holder.accept(_wire(rev=2))
    assert holder.active is new and holder.active.rev == 2


def test_fence_runtime_state_carries_active_rev_and_is_honest():
    """*** F3: state/fence 载 active.rev/crc32(D 的锚)且 enforcement 诚实.

    子集不裁剪 -> enforcement=warn_only + degrade_reason=clip_deferred(不谎报
    full). allow 走 fail-safe 拒动.

    MUTATION: build 里 enforcement 写 "full" -> 这里断言 warn_only 红(谎报在裁剪).
    """
    holder = FenceSetHolder()
    held = holder.accept(_wire(rev=7))
    st = build_fence_runtime_state(held, now_mono_s=100.5, applied_mono_s=100.0)
    assert st["active"]["rev"] == 7 and st["active"]["crc32"] == held.crc32
    assert st["active"]["name"] == "camp"          # allow 围栏的 name
    assert st["active"]["applied_mono_s"] == 100.0
    assert st["src_age_s"] == 0.5                  # 单调钟 now-applied
    assert st["enforcement"] == "warn_only"
    assert st["degrade_reason"] == "clip_deferred"
    # fail-safe 拒动: 无裁剪就不许自主运动/不接任务.
    assert st["allow"] == {"autonomous": False, "accept_task": False,
                           "teleop_max_mps": 0.0}


def test_fence_runtime_state_no_fence_is_honest_none():
    """*** 还没收到 cmd/fence: 老实报 src_state=none / no_fence, active 为 null.

    MUTATION: 无 held 时也编一个 active -> 这里 active is None 断言红.
    """
    st = build_fence_runtime_state(None, now_mono_s=10.0, applied_mono_s=None)
    assert st["active"] is None
    assert st["src_state"] == "none" and st["degrade_reason"] == "no_fence"
    assert st["enforcement"] == "disabled"


def test_p1_wiring_actually_subscribes_cmd_fence():
    """*** 守接线: main_wiring 真的订了 cmd/fence 并喂给 holder.accept.

    holder 单测再全, 没接到 cmd/fence 上也是空转. 用 AST 查 main_wiring:
    有一个 declare_subscriber 落在 cmd/fence, 且回调里调 fence_holder.accept.

    MUTATION: 删掉 main_wiring 里 gen.declare_subscriber(CMD_FENCE_TOPIC,...) ->
    这里红(与"表对而线没接"同一道防线).
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain" / "p1_motion"
           / "runtime" / "main_wiring.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    # 本模块的字符串常量表(CMD_FENCE_TOPIC = "cmd/fence").
    consts = {t.id: n.value.value
              for n in tree.body if isinstance(n, ast.Assign)
              for t in n.targets
              if isinstance(t, ast.Name) and isinstance(n.value, ast.Constant)
              and isinstance(n.value.value, str)}
    subbed_keys = set()
    called = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = getattr(node.func, "attr", "") or getattr(node.func, "id", "")
            if fn == "declare_subscriber" and node.args:
                a = node.args[0]
                if isinstance(a, ast.Constant):
                    subbed_keys.add(a.value)
                elif isinstance(a, ast.Name):
                    subbed_keys.add(consts.get(a.id))
            called.add(fn)

    assert "cmd/fence" in subbed_keys, (
        "main_wiring 没订 cmd/fence -- FenceSet 收不到, F2/F3 全空转")
    assert "accept" in called, (
        "cmd/fence 回调没调 fence_holder.accept -- 收了不校验不持有")


def test_p1_wiring_publishes_state_fence():
    """*** 守接线(F3): main_wiring 声明 state/fence 发布者并在循环调 build.

    MUTATION: 删掉 state_fence_pub = gen.declare_publisher(STATE_FENCE_TOPIC) 或
    循环里的 build_fence_runtime_state 调用 -> 这里红(D 拿不到 active.rev).
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2] / "xbrain" / "p1_motion"
           / "runtime" / "main_wiring.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    consts = {t.id: n.value.value
              for n in tree.body if isinstance(n, ast.Assign)
              for t in n.targets
              if isinstance(t, ast.Name) and isinstance(n.value, ast.Constant)
              and isinstance(n.value.value, str)}
    pub_keys = set()
    called = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            fn = getattr(node.func, "attr", "") or getattr(node.func, "id", "")
            called.add(fn)
            if fn == "declare_publisher" and node.args:
                a = node.args[0]
                if isinstance(a, ast.Constant):
                    pub_keys.add(a.value)
                elif isinstance(a, ast.Name):
                    pub_keys.add(consts.get(a.id))
    assert "state/fence" in pub_keys, (
        "main_wiring 没发 state/fence -- D(SET_ALARM_CONFIG 终态)拿不到 active.rev")
    assert "build_fence_runtime_state" in called, (
        "循环没调 build_fence_runtime_state -- 发的不是 FenceRuntimeState")
