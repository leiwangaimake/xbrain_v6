"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fence_enu_origin.py
Brief: FenceSet 的 enu_origin 必须随发布带上 (11 S9A.2 必填字段)

Description:
守 2026-09-02 补的接线. 缺陷原样: build_fence_set 早就有 enu_origin 形参,
configs 也早就声明了 common.geo.enu_origin(带 FV-ORG-1/2/3 三条启动断言),
L4 sites/{site_id}.yaml 也是唯一定义处 -- 但 p3 的两个调用点都没传, 形参取默认
None, 于是 cmd/fence 上这个[必填]字段恒为 null.

*** 后果不是"少一个字段".
11 S9A.2 逐字: enu_origin 是"本地 ENU 平面的锚点, 全系统唯一". 没有它:
  - HMI 拿不到投影原点, 围栏/路径几何数据齐全却一个图元都画不出来;
  - 11 FV-7(所有顶点距 enu_origin <= 20km, 超出 E_SCHEMA)连判据都无从谈起 --
    该校验至今未实现, 因为它依赖的值一直是 null.

*** 为什么这个洞能活这么久.
默认值是 None 而不是抛错, 所以"没接线"与"配置里没值"表现完全一样, 都是
安静地发一个 null. 而在没有场地标定的开发环境里, 后者本来就是预期状态
(sites/ 下只有 _skeleton.yaml) -- 于是缺陷被合理现象完美掩盖.

Boundaries: 只测[传没传]与[传对没传对]. NO 不测 enu_origin 的取值是否正确
(那是现场标定的事), 也不测 FV-7(独立一批).
"""
from __future__ import annotations

import pytest

pytestmark = pytest.mark.no_device

from xbrain.p3_task.fence.fence_set import build_fence_set

#: FencesDAO.list_active() 的行形状: (fence_id, name, role, kind, geom_json,
#: hard_enforce, rev). 只放一个 allow -- FV-3 要求恰好一个.
_ROWS = [("f-a", "活动区", "allow", "polygon",
          '{"points": [[31.24, 121.45], [31.25, 121.46], [31.23, 121.47]]}',
          1, 1)]
_ORIGIN = {"lat": 31.2304, "lon": 121.4737, "alt": 4.0}


def test_enu_origin_is_carried_into_the_fence_set():
    """*** 传进去的原点必须原样出现在 FenceSet 里.

    变异体: build_fence_set 里 "enu_origin": enu_origin 改成 None => 本条红.
    """
    fs = build_fence_set(_ROWS, fence_set_id="fs-active", rev=1,
                         enu_origin=_ORIGIN)
    assert fs["enu_origin"] == _ORIGIN, (
        "enu_origin 没随 FenceSet 带出去, 实得 %r" % fs["enu_origin"])


def test_enu_origin_absent_stays_none_not_a_fabricated_zero():
    """*** 没传就是 None, NO 不能兜底成 0/0/0.

    一个 {0,0,0} 的假原点会把所有围栏投影到几内亚湾外海(赤道与本初子午线的
    交点), 而且[不报任何错] -- 操作员看到的是一张空地图或错得离谱的图元,
    没有任何线索指向配置. 这正是 CLAUDE.md 3.1 那条"0.0 冒充已赋值".

    变异体: 给 enu_origin 形参一个 {"lat":0,"lon":0,"alt":0} 默认值 => 本条红.
    """
    fs = build_fence_set(_ROWS, fence_set_id="fs-active", rev=1)
    assert fs["enu_origin"] is None, (
        "未标定时编了一个原点: %r" % fs["enu_origin"])


def test_p3_wiring_passes_enu_origin_to_the_published_fence_set():
    """*** 守接线那一半: 发布路径必须把 enu_origin 传下去.

    上面两条只证明 build_fence_set 会带; 但缺陷本体在[调用点没传]. 两个文件
    各自都"对", 合起来是错的 -- 与 2026-09-01 那次 mic streaming 信号同一形状.
    静态查 wiring 源码(与 test_cloud_key_surface_wired 同一手法).

    变异体: 把发布调用点的 enu_origin=enu_origin 删掉 => 本条红.
    """
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[2]
           / "xbrain" / "p3_task" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    assert "enu_origin=enu_origin" in src, (
        "p3 的 build_fence_set 发布调用点没有传 enu_origin -- "
        "cmd/fence 上该必填字段会恒为 null")
    assert "enu_origin: Optional[Dict[str, float]] = None" in src, (
        "_amain / run_voice_loop_wiring 没有接收 enu_origin 的形参")


def test_main_reads_enu_origin_from_the_resolved_config():
    """*** 守另一半: __main__ 必须从解析产物里取值, 而不是就地写死.

    11 第 7815 行逐字"各进程不得各自选原点", 单一来源是 L4 场地层. 在 p3 里
    写一个坐标常量会让换场地时这台车用旧原点 -- 而且不会有任何报错.

    变异体: 把 _cfg.get("geo") 换成一个字面量 dict => 本条红.
    """
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[2]
           / "xbrain" / "p3_task" / "__main__.py").read_text(encoding="utf-8")
    assert '_geo.get("enu_origin")' in src, (
        "__main__ 没有从解析产物读 enu_origin")
    assert "enu_origin=_enu" in src, "读了但没往下传"


# --- p5 侧: 权威原点必须被采纳且优先于兜底 ---------------------------

def test_p5_adopts_the_authoritative_origin_from_cmd_fence():
    """*** p5 必须从 cmd/fence 取权威 enu_origin, 而不是只认首次定位.

    p5 原来只有一条"首次 GPS 定位当原点"的兜底路(W4 待办), 它的注释自己写着
    "The authoritative origin is common.geo.enu_origin" -- 但代码从不读它.
    p3 那半修好之后, 权威值已经躺在 cmd/fence 里, p5 却看不见, HMI 的
    enu_origin 仍是 null, 地图照样画不出来. 两个文件各自都"对", 合起来是错的.

    变异体: 删掉 _on_cmd_fence 里的 enu_origin 采纳块 => 本条红.
    """
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[2]
           / "xbrain" / "p5_gateway" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    assert 'eo = d.get("enu_origin")' in src, (
        "p5 的 _on_cmd_fence 没有从 FenceSet 取 enu_origin")
    assert 'hmi_state["enu_origin_authoritative"] = True' in src, (
        "取了但没标记为权威 -- 兜底路径会把它顶掉")


def test_the_first_fix_fallback_yields_to_the_authoritative_origin():
    """*** 优先级不能反: 配置里的测绘锚点 > 首次定位兜底.

    写反的后果很隐蔽: 先开机(拿到 GPS 首次定位)再配围栏的顺序下, 系统会一直
    用那个跟着开机位置漂移的兜底原点, 而权威值明明已经到了. 地图上所有图元
    整体偏移, 但每个图元的相对关系都对 -- 看起来"只是没对齐", 没人会想到是
    原点选错了.

    变异体: 兜底那个 if 去掉 not ..._authoritative 条件 => 本条红.
    """
    import pathlib
    src = (pathlib.Path(__file__).resolve().parents[2]
           / "xbrain" / "p5_gateway" / "runtime" / "main_wiring.py").read_text(
               encoding="utf-8")
    assert 'and not hmi_state.get("enu_origin_authoritative")' in src, (
        "首次定位兜底没有让位于权威原点 -- 优先级反了")
