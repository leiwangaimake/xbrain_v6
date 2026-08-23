"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_m5_exit.py
Brief: CHK-1-05 -- M5 出口标准 G-1 / G-1a / G-5 / G-6 / G-7 的可执行验收

Description:
M5 是交付出口, 而它的验收此前只有一条"报告里必须列全"的元检查 -- 也就是说
[一份手写的, 把每条都写成 pass 的报告就能过]. 这是 CLAUDE.md 3.2 形态1 最贵
的一种: 通过的不是机器人, 是一个 markdown 文件.

本文件是那五条的执行体; 报告由 scripts/ci/m5_acceptance.py 从[执行结果]生成,
手写的条目会被拒(判据变异体1).

*** 五条里今天只有两条能真跑, 逐条说清卡在哪 -- NO 不许把做不到的写成 stub
再宣布通过.

  G-5  新增指令源只需注册, 不改既有代码       -> 真跑
       检验方式与判据原文不同, 理由写在用例里: git diff 在测试里是恒空的
       (测试不改文件), 那样的断言测不出任何东西.
  G-6  换机型只改配置, 不改代码               -> 真跑(同上)
  G-1  AI 层全杀, 路径仍走完并零速停车        -> xfail: 要 P1 的 20 Hz 环真跑,
       而 ctrl_loop 今天没有接线(见 CHK-1-04 / NEXT SW-21)
  G-1a P2 缺失必须停车(与 G-1 方向相反)       -> xfail: speed_factor 3s->0.3 /
       10s->allow_motion=false 的断流判定[根本没有实现](CHK-1-04 逐行核实)
  G-7  rosbag + 事件流复现同一决策序列        -> xfail: 没有复现工具, 也没有
       归档 rosbag

*** 为什么 xfail(strict=True) 而不是 skip, 也不是 stub.
判据变异体3 逐字禁 skip. 而 stub 在这里更糟: G-1a 要观察的正是那个"不存在的
断流判定", 我写一个桩去扮演它, 再断言桩的行为符合预期 -- 两边都是我写的,
永远通过, 什么也没验证. strict 保证实现一旦补上, 用例意外通过时 XPASS 失败,
逼人回来写真断言.

Boundaries: 不判断 G-* 的标准本身对不对(那是 10 S1.1 的事), 只执行它们.
"""
from __future__ import annotations

import pathlib
import re

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]


# --- G-5: 新增指令源只需注册 -----------------------------------------

def test_g5_a_new_command_source_needs_registration_only():
    """*** G-5 (CON-01): 加一个虚拟指令源, 既有仲裁代码改动为零.

    判据原文的检验是 "git diff --stat 只触及注册表/配置". 本用例[不]那样做,
    理由必须写清楚: 在 pytest 里跑 git diff 是恒空的 -- 测试不修改工作树,
    所以那条断言无论实现对错都会通过. 那是形态1.

    真正可证伪的检验有两半, 两半都要:
      (1) 语义半: 一个从没出现过的 source_id 能走完整个仲裁流程(注册 ->
          请求 -> 持有 -> 被更高优先级抢占), 全程不需要 Arbiter 知道它是谁;
      (2) 静态半: Arbiter 代码里不得有按具体 source_id 的分支 -- 有一个,
          "新增源不改代码"就是假的, 而语义半仍然会通过(因为你测的那个源
          恰好不在硬编码分支里).
    """
    from xbrain.common.arbiter.core import Arbiter
    from xbrain.common.arbiter.model import PreemptPolicy, Request, SourceSpec

    arb = Arbiter("motion", 3000)
    # 一个刻意取的, 全仓从未出现过的 id. 用 "voice"/"cloud" 这类既有 id
    # 会让本用例即使在有硬编码分支的实现上也通过.
    arb.register(SourceSpec("virtual_probe_src", 400, True,
                            PreemptPolicy.WAIT_ATOMIC, None, None, None))
    grant = arb.request("virtual_probe_src", Request("vp-1", 1000))
    assert grant.result == "granted", (
        "一个新注册的源拿不到空闲域: %r -- 那就不是 [只需注册]" % (grant,))

    # 更高优先级的另一个新源必须能抢占它 -- 抢占逻辑同样不该认识具体 id.
    arb.register(SourceSpec("virtual_higher_src", 900, True,
                            PreemptPolicy.IMMEDIATE, None, None, None))
    grant2 = arb.request("virtual_higher_src", Request("vh-1", 1000))
    assert grant2.result in ("granted", "queued"), (
        "更高优先级的新源既没拿到也没排队: %r" % (grant2,))


def test_g5_arbiter_has_no_hardcoded_source_branches():
    """G-5 的静态半. 见上一条用例的说明.

    MUTATION: 往 core.py 里加一句 `if source_id == "voice": ...` -> 这里红.
    """
    hits = []
    for path in sorted((ROOT / "xbrain" / "common" / "arbiter").glob("*.py")):
        for lineno, line in enumerate(
                path.read_text(encoding="utf-8").split("\n"), 1):
            code = line.split("#", 1)[0]          # 注释里举例不算
            if re.search(r'(==|!=)\s*"(voice|cloud|hmi|teleop|task|path_follow'
                         r'|rns_avoid|charge)"', code):
                hits.append("%s:%d %s" % (path.name, lineno, code.strip()[:60]))
    assert not hits, (
        "仲裁核心里有按具体 source_id 的分支, G-5 不成立:\n  " + "\n  ".join(hits))


# --- G-6: 换机型只改配置 ---------------------------------------------

def test_g6_a_new_platform_is_config_only():
    """*** G-6 (CON-05 / CFG-20): 造一份履带车配置, 代码侧零改动.

    履带车与 M20S 的关键差异是 holonomic: 11 S9.6 逐字"M20S 支持横移;
    履带车须为 false". 所以本用例把 holonomic=False 喂进真实的能力门,
    断言横移类意图被拒 -- 全程没有一行代码知道"履带车"这个词.
    """
    from xbrain.p2_core.runtime import motion_intent_wiring as miw

    # 找一个需要 holonomic 的意图. 不硬编码具体意图名: 那张闭集会增删,
    # 硬编码会让本用例在闭集调整后测一个不存在的意图.
    needs = getattr(miw, "_NEEDS_HOLONOMIC", None)
    assert needs, "找不到 _NEEDS_HOLONOMIC -- 能力门的形状变了"
    intent = sorted(needs)[0]

    # M20S: holonomic=True -> 不因这一项被拒.
    # 履带车: holonomic=False -> 必须被拒, 且理由是能力(E_CAPABILITY)不是别的.
    #
    # *** 第一版用"按签名搜函数"的办法找能力门, 结果一个都没找到并返回 None,
    # 断言于是在比较 None -- 一个找不到被测对象却照样给出结论的用例. 改成
    # 直接调真入口 evaluate(): 找不到它会 ImportError, 那是该有的失败方式.
    verdicts = {}
    for holo in (True, False):
        # 帧必须先过 G-2 的必填检查才能走到能力门. 第一版少了 auth_level
        # 与 slots, 两种机型都被 G-2 拦下 -> 结论一样 -> 用例报"门没看配置",
        # 而门其实根本没被走到. 一个在被测逻辑之前就返回的用例, 测的是
        # 别的东西.
        # *** 帧要走到 G-7(holonomic 那道门), 前面 G-2..G-6 必须全通.
        # 逐次修的过程本身说明问题: 第一版少 auth_level 卡在 G-2, 补了又
        # 卡在 G-4(缺 clock). 每一次两种机型都拿到[同样]的拒绝, 于是用例
        # 报"门没看配置"-- 而门根本没被走到. 一个在被测逻辑之前就返回的
        # 用例, 测的是别的东西, 且它的失败信息会把人引向错误的方向.
        verdicts[holo] = miw.evaluate(
            {"cmd_id": "m5-g6", "turn_id": "m5-t1", "channel": "mic_local",
             "auth_level": "L1", "intent": intent,
             "slots": {"distance_m": 1.0, "angle_deg": 30.0}},
            limits=miw.MotionLimits(max_distance_m=20.0, max_angle_deg=720.0),
            clock={"ts_sync": True},
            health={"allow_motion": True},
            pose={"yaw_capable": True},
            robot={},
            holonomic=holo)
    tracked, wheeled = verdicts[False], verdicts[True]
    assert not tracked.passed, (
        "履带车(holonomic=false)没有拒绝横移类意图 %s: %r" % (intent, tracked))
    # 反向: 两种机型必须走到[不同]结论, 否则这道门根本没在看配置 --
    # 一个"永远拒绝"的实现能通过上一条断言.
    assert (wheeled.passed, wheeled.gate) != (tracked.passed, tracked.gate), (
        "holonomic=true 与 false 得到完全一样的结论 %r -- 门没有看配置"
        % (tracked,))


def test_g6_no_code_branches_on_a_platform_name():
    """G-6 的静态半: 代码里不得按机型名分支.

    一句 `if model == "m20s"` 会让"换机型只改配置"当场不成立, 而且它极难
    被发现 -- 换上履带车配置后大部分功能仍然正常, 只有那一个分支悄悄走错.

    MUTATION: 往任意 xbrain/ 文件里加 `if model == "m20s":` -> 红.
    """
    hits = []
    for path in (ROOT / "xbrain").rglob("*.py"):
        for lineno, line in enumerate(
                path.read_text(encoding="utf-8", errors="replace").split("\n"), 1):
            code = line.split("#", 1)[0]
            if re.search(r'(==|!=)\s*"(m20s|M20S|tracked|履带车)"', code):
                hits.append("%s:%d" % (path.relative_to(ROOT), lineno))
    assert not hits, "代码里有按机型名的分支, G-6 不成立: %s" % hits[:5]


def test_g6_the_platform_config_carries_the_capability_bit():
    """机型能力位必须真的落在 configs/models/ 里, 不是写死在代码里."""
    import yaml

    models = ROOT / "configs" / "models"
    assert models.is_dir(), "没有 configs/models/ -- 机型配置无处可放"
    found = []
    for path in sorted(models.glob("*.yaml")):
        body = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        spec = (body.get("common") or {}).get("spec") or {}
        if "holonomic" in spec:
            found.append((path.name, spec["holonomic"]))
    assert found, "configs/models/ 里没有一份声明 holonomic 的机型配置"


# --- G-1 / G-1a / G-7: 今天做不到, 逐条说明 --------------------------

@pytest.mark.xfail(strict=True, reason=(
    "G-1 要杀掉 AI 层后观察当前路径走完并 path_progress.state=='arrived'. "
    "那要 P1 的 20 Hz 控制环真在跑, 而 ctrl_loop 今天没有接线 -- "
    "见 CHK-1-04 逐行核实与 NEXT SW-21"))
def test_g1_ai_layer_death_still_finishes_the_path():
    """G-1 (NAV-34). 实现出现后把标记摘掉并在这里写真断言.

    DIRECTION: finish-path
    """
    from xbrain.p1_motion import ctrl_loop

    # *** 这条断言的第一版是判据自伤, 当场被抓到.
    # 它写的是 `"freshness" in src` -- 而 ctrl_loop 的注释里就有那个词
    # (头注写着 "freshness -> arbiter tick -> gate" 的步骤顺序). 断言恒真,
    # xfail(strict) 于是报 XPASS 失败. 要检测的字串出现在被检测文件的注释里,
    # 与 CHK-1-50 那次 "no if(ROS_DISTRO ...)" 是同一种失效.
    #
    # 改用 AST 查 import: 注释里写什么都不影响它, 而真接线一定要 import.
    import ast

    tree = ast.parse(pathlib.Path(ctrl_loop.__file__).read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module)
        elif isinstance(node, ast.Import):
            imported.update(a.name for a in node.names)
    assert any("freshness" in m for m in imported), (
        "ctrl_loop 仍未 import freshness -- 输入超时到降级这条链没有接起来")


@pytest.mark.xfail(strict=True, reason=(
    "G-1a 要观察 kill p2_core 后 3 s speed_factor->0.3 / 10 s "
    "allow_motion=false. CHK-1-04 逐行核实: 该断流判定[根本没有实现]. "
    "NO 不写桩去扮演它 -- 桩喂桩永远通过, 什么也没验证"))
def test_g1a_p2_death_must_stop_the_robot():
    """G-1a (CON-07 / NFR-12). 与 G-1 方向相反的那一半.

    DIRECTION: must-stop

    *** 判据变异体2 守的正是这个方向: 给 P2 缺失加一条"保持最后一次健康度
    继续跑"的例外路径, 这条必须红 -- 10 S1.1 已逐字判该支"不采用".
    今天它红着的原因不同(没有实现), 但方向是对的.
    """
    import importlib

    mod = importlib.import_module("xbrain.p2_core.health.factor")
    assert hasattr(mod, "factor_stale_policy"), "断流判定仍未实现"


@pytest.mark.xfail(strict=True, reason=(
    "G-7 要拿归档 rosbag + 事件流跑复现工具重建同一决策序列. "
    "复现工具不存在, 归档 rosbag 也不存在"))
def test_g7_an_incident_can_be_replayed():
    """G-7 (EVT-11 / P-7)."""
    tool = ROOT / "scripts" / "replay" / "replay_incident.py"
    assert tool.is_file(), "复现工具不存在"


# --- 元测试: 五条都必须在这里出现 ------------------------------------

#: M5 出口标准里由 CHK-1-05 负责的五条. 与 10 S1.1 的表体对齐 --
#: test_all_five_g_items_have_a_case 会拿它与本文件的用例名求差集.
G_ITEMS = ("G-1", "G-1a", "G-5", "G-6", "G-7")


def test_all_five_g_items_have_a_case():
    """*** 少一条就少验一项, 而且是静默的.

    判据点名的是这五条(G-1 / G-1a / G-5 / G-6 / G-7). 本文件用函数名里的
    g1 / g1a / g5 / g6 / g7 前缀标注归属, 这里反查.

    MUTATION: 删掉 test_g7_* -> 红.
    """
    src = pathlib.Path(__file__).read_text(encoding="utf-8")
    for item in G_ITEMS:
        token = "def test_" + item.lower().replace("-", "") + "_"
        assert token in src, "%s 没有对应用例(期望函数名以 %s 开头)" % (item, token)


def test_meta_g1_and_g1a_are_opposite_directions():
    # *** 名字必须以 test_meta_ 开头, NO 不能叫 test_g1_*.
    # 第一版叫 test_g1_and_g1a_...., 于是报告生成器按前缀把它算成 G-1 的
    # 一条 passed -- 而 G-1 本体是 xfailed. 报告里 G-1 那一格于是同时有
    # passed 与 xfailed 两行, 读的人很容易只看到前一行.
    # 一份让人误读的报告与一份写错的报告, 对验收的伤害是一样的.
    """*** 判据变异体2 的结构保证: G-1 与 G-1a 必须是一对反向断言.

    G-1 说"AI 层全死也要走完路径", G-1a 说"P2 死了必须停下且不得走完".
    两条如果不小心写成同向(比如都断言"继续跑"), 那么 10 S1.1 逐字判为
    "不采用"的那条例外路径就会同时通过两边 -- 而这正是它存在的理由.

    这里守的是措辞层面: 两条用例的说明里必须分别出现"走完"与"停"的方向词.
    """
    src = pathlib.Path(__file__).read_text(encoding="utf-8")
    # *** 切片必须[包含装饰器]: 方向词写在 xfail 的 reason 里, 而 reason 在
    # def 之前. 第一版从 "def test_g1_..." 切起, 把 reason 切掉了, 于是这条
    # 用例在报"G-1 没写它要求路径走完"-- 而文件里明明写了.
    def _block(start_marker, end_marker):
        head = src.index(start_marker)
        # 往回找到这条用例的装饰器起点(最近的一个空行之后).
        back = src.rfind("\n\n", 0, head)
        return src[back if back > 0 else head:src.index(end_marker)]

    g1 = _block("def test_g1_ai_layer_death", "@pytest.mark.xfail(strict=True, reason=(\n    \"G-1a")
    g1a = _block("def test_g1a_p2_death", "@pytest.mark.xfail(strict=True, reason=(\n    \"G-7")
    # *** 查[显式方向标记], 不查措辞, 也不查函数名.
    # 两版都踩过坑:
    #   v1 查函数名里的 "stop" -- 而函数就叫 must_stop_the_robot, 断言恒真,
    #      把说明里的方向词全删掉也不红(变异体实测没红, 抓出来的);
    #   v2 改查说明里的中文"停" -- 而 G-1a 的 reason 用的是英文
    #      (speed_factor->0.3 / allow_motion=false), 同一个意思换个措辞
    #      就红了. 一条依赖措辞的断言, 迟早被人改措辞时误伤然后放宽.
    # 所以让两条用例各自[声明]方向, 这里只比较声明.
    directions = {}
    for tag, block in (("G-1", g1), ("G-1a", g1a)):
        found = re.findall(r"DIRECTION:\s*(\S+)", block)
        assert len(found) == 1, (
            "%s 必须恰好声明一次 DIRECTION, 实际 %d 次" % (tag, len(found)))
        directions[tag] = found[0]
    assert directions["G-1"] == "finish-path", (
        "G-1 的方向声明变了: %r" % directions["G-1"])
    assert directions["G-1a"] == "must-stop", (
        "G-1a 的方向声明变了: %r" % directions["G-1a"])
    # 核心那一句: 两条必须相反. 10 S1.1 把 G-1a 单列正是因为它是失效方向
    # 核对项 -- 两条同向就等于把那个核对丢了.
    assert directions["G-1"] != directions["G-1a"], (
        "G-1 与 G-1a 声明了同一个方向 -- 10 S1.1 判该支不采用")
