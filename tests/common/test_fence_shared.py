"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_fence_shared.py
Brief: 守 crc32/点在多边形内是[跨进程单一真源](11 S9A.2, 报警 F0)

Description:
p1_motion 自算 crc32 比对(11 S9A.2"接收方必须自算比对")与报警区点在多边形内
判定, 必须与 p3_task 用[同一个]实现. 本文件把这条不变式钉成断言: p3 侧导出的
符号[就是]共享库的对象(is 同一性), 而不是各存一份. 谁日后在 p3 里重新写一份
本地 crc32/point_in_polygon, 本用例立刻红 -- 那正是漂移的起点(改一处忘另一处,
crc32 跨进程不一致, 现象与网络坏不可区分).
"""

from xbrain.common.fence.geom import fence_set_crc32 as shared_crc32
from xbrain.common.fence.geom import point_in_polygon as shared_pip


def test_p3_symbols_are_the_shared_ones_not_a_copy():
    """*** 单一真源: p3 导出的 crc32/point_in_polygon 就是共享库对象(is).

    MUTATION: 在 p3_task/fence/geom.py 里重新 def 一份 point_in_polygon(而不是
    从 common re-import) -> `is` 断裂 -> 这里红.
    """
    from xbrain.p3_task.fence.fence_set import fence_set_crc32 as p3_crc32
    from xbrain.p3_task.fence.geom import point_in_polygon as p3_pip

    assert p3_crc32 is shared_crc32, (
        "p3 的 fence_set_crc32 不是共享库那一个 -- 有人复制了一份, crc32 迟早漂移")
    assert p3_pip is shared_pip, (
        "p3 的 point_in_polygon 不是共享库那一个 -- 两侧判定会分叉")


def test_crc32_is_deterministic_and_order_sensitive():
    """crc32 对同一 FenceSet 恒定, 且对多边形顺序敏感(数组序是归一化的一部分)."""
    polys = [
        {"poly_id": "f-a", "role": "allow", "winding": "ccw",
         "hard_enforce": False, "priority": 0,
         "vertices": [{"lat": 34.0, "lon": 135.0}, {"lat": 34.1, "lon": 135.0},
                      {"lat": 34.1, "lon": 135.1}]},
        {"poly_id": "f-b", "role": "warning", "winding": "cw",
         "hard_enforce": False, "priority": 0,
         "vertices": [{"lat": 34.02, "lon": 135.02},
                      {"lat": 34.03, "lon": 135.02},
                      {"lat": 34.03, "lon": 135.03}]},
    ]
    c1 = shared_crc32("fs-x", 3, polys)
    c2 = shared_crc32("fs-x", 3, polys)
    assert c1 == c2 and len(c1) == 8            # 恒定 + 8 位 hex
    # 换序 -> 不同 crc32(否则 p1 会把两个不同的 active 集当成同一版)
    assert shared_crc32("fs-x", 3, list(reversed(polys))) != c1


def test_point_in_polygon_basic():
    """一个方形报警区: 内点 True, 外点 False(zone_enter 判定的地基)."""
    from xbrain.common.fence.geom import Polygon

    square = Polygon(points=((0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)))
    assert shared_pip(5.0, 5.0, square) is True
    assert shared_pip(15.0, 5.0, square) is False


def test_the_shared_crc32_is_the_block_b3_recipe_not_the_v02_one():
    """*** 这条是上面那条 `is` 判据[看不见]的那一半.

    `is` 只证明两侧用的是同一个函数对象, 不证明那个函数的配方是对的 --
    2026-10-08 实测: p1/p3 确实共用 common/fence/geom.fence_set_crc32,
    而它当时自带的是 11 S9A.2 的[v0.2]配方(只有 poly_id/role/winding/
    hard_enforce + 顶点). 块 B-3 加的 priority 与 speed_limit_mps 只落到了
    common/digest/fence.py 与 C++ 头 ==> 两份对的实现没人在运行期调.
    于是那条 `is` 判据在真缺陷上[恒绿](CLAUDE.md 3.2 形态 1).

    S9A.2 自己写明了后果: "改限速/改叠放次序得到相同 crc32" -> S-5 幂等答
    duplicate -> "改了不生效, 而且不报任何错". 本条就钉这两件事.

    MUTATION: 把 geom.fence_set_crc32 换回自带的 v0.2 配方(或从块 B-3 的
    七段里去掉 priority / speed_limit_mps 任一段) -> 对应那个断言红.
    """
    base = {"poly_id": "f-a", "role": "allow", "winding": "ccw",
            "hard_enforce": True, "priority": 10,
            "vertices": [{"lat": 34.0, "lon": 135.0},
                         {"lat": 34.1, "lon": 135.0},
                         {"lat": 34.1, "lon": 135.1}]}
    # 1) 只改 priority(叠放次序) -> crc32 必须变.
    restacked = dict(base, priority=20)
    assert shared_crc32("fs-x", 1, [base]) != shared_crc32("fs-x", 1, [restacked])
    # 2) 只改 speed_limit_mps(区内限速) -> crc32 必须变. 这是契约逐字点名的
    #    那个场景: 把慢行区调低而 crc32 不动.
    slow = {"poly_id": "f-s", "role": "speed_limit", "winding": "cw",
            "hard_enforce": False, "priority": 0, "speed_limit_mps": 2.0,
            "vertices": base["vertices"]}
    slower = dict(slow, speed_limit_mps=1.0)
    assert shared_crc32("fs-x", 1, [slow]) != shared_crc32("fs-x", 1, [slower])
    # 3) 反向: 共享实现必须与 digest 模块(C++ 头的孪生, 跨语言金标钉的那一个)
    #    逐字节一致. 不是 `is` 同一函数 -- geom 做签名适配 -- 而是同一个值.
    from xbrain.common.digest.fence import fence_crc32
    assert shared_crc32("fs-x", 1, [slow]) == fence_crc32(
        {"fence_set_id": "fs-x", "rev": 1, "polygons": [slow]})
