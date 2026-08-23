"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_m5_acceptance.py
Brief: CHK-1-05 -- 守 scripts/ci/m5_acceptance.py: 手写报告必须被拒

Description:
判据变异体1 逐字: "手工写一份把 8 条都写成 pass 的报告, 而不跑任何用例 =>
报告生成器必须拒绝(无执行结果即无条目)". 本文件正面攻击这一条 -- 直接伪造
各种形状的报告喂给 verify_report(), 看它拒不拒.

*** 为什么这条比套件本身还重要.
M5 是交付出口. 出口验收的对手不是"实现有 bug", 是"有人想让它看起来通过" --
而那件事只需要编辑一个 markdown. 一个宽松的生成器与一个严格的生成器, 在
一切正常时输出完全一样, 差别只在被蒙混的那一刻显现. 所以这里逐种伪造手法
各写一条:
  * 整份手写(证据用例根本不存在)          -> 拒
  * 把 xfailed 改写成 passed(证据存在但状态不符) -> 拒
  * 删掉不好看的那一行(少写也是伪造)      -> 拒
  * 空执行结果 + 满页 pass                -> 拒

Boundaries: 不判断 G-* 标准本身, 也不判断某条用例写得对不对 -- 那是
tests/acceptance/test_m5_exit.py 的事. 这里只保证[报告等于执行结果].
"""
from __future__ import annotations

import importlib.util
import pathlib
import subprocess
import sys

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
GEN = ROOT / "scripts" / "ci" / "m5_acceptance.py"
SUITE = ROOT / "tests" / "acceptance" / "test_m5_exit.py"


def _gen():
    """按路径 import -- 它是 script, 不是包."""
    spec = importlib.util.spec_from_file_location("m5_acceptance", GEN)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


#: 一次"真实"的执行结果, 手写在这里当夹具.
#: NO 不在每条用例里现跑一遍真套件: 那会让本文件耗时翻几倍, 而且本文件
#: 要验的是 verify_report 的判定逻辑, 不是套件的结论.
_RESULTS = {
    "test_g1_ai_layer_death_still_finishes_the_path": "xfailed",
    "test_g1a_p2_death_must_stop_the_robot": "xfailed",
    "test_g5_a_new_command_source_needs_registration_only": "passed",
    "test_g7_an_incident_can_be_replayed": "xfailed",
}


def _report(rows):
    head = ["| item | verdict | evidence (testcase) |", "|---|---|---|"]
    return "\n".join(head + ["| %s | %s | %s |" % r for r in rows]) + "\n"


def test_a_faithful_report_passes():
    """先钉基线: 一份如实的报告必须通过.

    没有这条, 一个"永远拒绝"的 verify_report 会让下面每条伪造用例都通过 --
    而它同时会让真报告也生不出来, 也就是把整个门变成永远红.
    """
    gen = _gen()
    rows = [(gen.item_of(n) or "?", s, n) for n, s in sorted(_RESULTS.items())]
    problems = gen.verify_report(_report(rows), _RESULTS)
    assert not problems, "如实的报告被拒了: %s" % problems


def test_a_hand_written_all_pass_report_is_rejected():
    """*** 判据变异体1 的正面形态: 整份手写, 一个用例也没跑.

    这是本文件存在的理由. 伪造者写的是"看起来对"的内容 -- 用例名像模像样,
    verdict 全是 passed -- 而那些用例根本没有执行结果.
    """
    gen = _gen()
    fake = _report([
        ("G-1", "passed", "test_g1_everything_is_fine"),
        ("G-1a", "passed", "test_g1a_everything_is_fine"),
        ("G-5", "passed", "test_g5_everything_is_fine"),
        ("G-6", "passed", "test_g6_everything_is_fine"),
        ("G-7", "passed", "test_g7_everything_is_fine"),
    ])
    problems = gen.verify_report(fake, _RESULTS)
    assert problems, "整份手写的全 pass 报告被接受了"
    # 报出的问题必须点名是哪一行 -- 只说"报告有问题"没法处置.
    assert any("test_g1_everything_is_fine" in p for p in problems), (
        "拒了但没说是哪一行: %s" % problems)


def test_upgrading_an_xfail_to_pass_is_rejected():
    """*** 更隐蔽的一种: 用例是真的, 只把结论改好看.

    这种伪造比整份手写危险 -- 证据用例真的存在且真的跑过, 只有 verdict
    那一格被改了. 只查"用例存不存在"的实现会放过它.
    """
    gen = _gen()
    rows = [(gen.item_of(n) or "?",
             "passed" if s == "xfailed" else s, n)
            for n, s in sorted(_RESULTS.items())]
    problems = gen.verify_report(_report(rows), _RESULTS)
    assert problems, "把 xfailed 改写成 passed 被接受了"
    assert any("xfailed" in p for p in problems), (
        "拒了但没说清实际状态: %s" % problems)


def test_dropping_an_inconvenient_row_is_rejected():
    """*** 第三种: 不改任何一行, 只把不好看的那行删掉.

    剩下的每一行都是真的, 每一行的 verdict 也都对 -- 但整份报告在撒谎.
    只做"报告 -> 执行结果"单向检查的实现会放过它, 所以 verify_report
    必须双向查.
    """
    gen = _gen()
    rows = [(gen.item_of(n) or "?", s, n)
            for n, s in sorted(_RESULTS.items()) if s != "xfailed"]
    problems = gen.verify_report(_report(rows), _RESULTS)
    assert problems, "删掉 xfailed 那几行后报告被接受了"
    assert any("报告里没有它" in p for p in problems), (
        "拒了但没指出是漏写: %s" % problems)


def test_an_empty_execution_cannot_back_any_row():
    """*** 边界: 一次都没跑, 却有一份满页 pass 的报告.

    这是"无执行结果即无条目"最直白的形态. 一个把空结果当成"没有冲突"的
    实现会全盘接受.
    """
    gen = _gen()
    fake = _report([("G-5", "passed", "test_g5_a_new_command_source_needs_"
                                      "registration_only")])
    problems = gen.verify_report(fake, {})
    assert problems, "空执行结果下, 报告里的 pass 行被接受了"


def test_generated_report_verifies_against_its_own_run():
    """闭环: 生成器自己产出的报告必须能通过自己的校验.

    这条同时守住一个容易发生的偏差 -- 生成器与校验器各自演进, 到某天
    生成的报告自己都过不了校验, 于是有人把校验关掉.
    """
    gen = _gen()
    report = gen.build_report(_RESULTS)
    problems = gen.verify_report(report, _RESULTS)
    assert not problems, "自己生成的报告过不了自己的校验: %s" % problems


def test_meta_test_cases_are_not_attributed_to_any_g_item():
    """*** 报告不得让人误读.

    第一版把元测试 test_g1_and_g1a_are_opposite_directions 按前缀算成了
    G-1 的一条 passed -- 而 G-1 本体是 xfailed. 报告里 G-1 那格于是同时
    有 passed 与 xfailed 两行, 读的人很容易只看前一行.
    一份让人误读的报告与一份写错的报告, 对验收的伤害一样.

    MUTATION: 去掉 item_of 里的 test_meta_ 排除 -> 红.
    """
    gen = _gen()
    # *** 这里守的是[命名约束], 不是一段排除代码.
    # 第一版在 item_of 里加了 `startswith("test_meta_") -> None` 的排除,
    # 而元测试改名之后那段排除永远走不到 -- 变异体删掉它, 一条用例都不红.
    # 一段不可证伪的防御是死代码(CLAUDE.md 9.3), 已删. 真正起作用的是
    # 命名: 元测试不以 test_g<数字>_ 开头, 于是前缀匹配自然认不出它.
    src = (ROOT / "tests" / "acceptance" / "test_m5_exit.py").read_text(
        encoding="utf-8")
    import re as _re
    for name in _re.findall(r"^def (test_\w+)", src, _re.M):
        item = gen.item_of(name)
        if item is None:
            continue
        # 认得出归属的, 必须真是那一条的执行体 -- 元测试(守这套东西本身的)
        # 不许占用 test_g<数字>_ 前缀.
        assert "_and_" not in name and "have_a_case" not in name, (
            "元测试 %s 用了 test_g* 前缀, 会被算成 %s 的一条结论 -- "
            "报告里那一格于是多出一行与它无关的 passed" % (name, item))
    assert gen.item_of("test_meta_g1_and_g1a_are_opposite_directions") is None
    # 反向: 真正的 G-1 用例仍要认得出.
    assert gen.item_of("test_g1_ai_layer_death_still_finishes_the_path") == "G-1"
    assert gen.item_of("test_g1a_p2_death_must_stop_the_robot") == "G-1a"


def test_g1a_is_not_swallowed_by_g1():
    """前缀匹配顺序: test_g1a_* 必须归 G-1a 而不是 G-1.

    两条是[方向相反]的一对(一条要求走完路径, 一条要求停车). 认错归属会让
    G-1a 那一格空着而 G-1 多一行 -- 恰好把失效方向核对项弄丢, 而那正是
    10 S1.1 特意把 G-1a 单列的理由.

    *** 这条用例的第一版不可证伪, 是变异体抓出来的.
    它只断言 item_of("test_g1a_anything") == "G-1a"; 而把 item_of 里的匹配
    顺序颠倒过来, 它照样通过 -- 因为前缀带下划线("test_g1_"), test_g1a_*
    本来就不匹配 G-1. 也就是说它守的是一个不存在的风险, 而生成器注释里
    那句"g1a 必须先于 g1"也跟着错了(已订正).
    真正会把 g1a 吞掉的是[前缀省掉分隔符], 所以改钉那个.

    MUTATION: 把 item_of 的前缀拼法改成 "test_" + "g1"(去掉尾部下划线)
    -> 这里红.
    """
    gen = _gen()
    assert gen.item_of("test_g1a_anything") == "G-1a"
    # 分隔符不能省: 少了它, test_g1a_* 会被 G-1 的前缀吞掉.
    assert gen.item_of("test_g1anything") is None, (
        "item_of 把 test_g1anything 认成了某一条 G-* -- 前缀少了分隔符, "
        "test_g1a_* 会被 G-1 吞掉")


def test_the_suite_actually_runs_under_the_generator():
    """守接线: 生成器真的能跑起套件并拿到 junit 结果.

    只测 verify_report 的话, 一个 run_suite 永远失败的生成器也能让上面
    每条通过 -- 而它在 CI 里的表现是"拒绝出报告".
    """
    proc = subprocess.run([sys.executable, str(GEN)],
                          capture_output=True, text=True, cwd=str(ROOT))
    assert proc.returncode == 0, "生成器跑不通:\n%s%s" % (proc.stdout, proc.stderr)
    assert "| item | verdict |" in proc.stdout, "没有生成报告表体"
    # 五条都要出现在报告里(NOT-RUN 也算出现) -- 少一条就是漏验一项.
    gen = _gen()
    for item in gen.G_ITEMS:
        assert ("| %s |" % item) in proc.stdout, "报告里没有 %s" % item


def test_report_is_not_committed_as_a_source_file():
    """*** 报告是产物, 不是真源.

    一份入库的报告会被人当成"上次验收的结论"引用, 而它与今天的代码没有
    任何关系. CHK-1-49 的 UNREGISTERED_BY_DESIGN 里那条构建戳是同一道理:
    产物不入库, 要看就现跑.
    """
    stray = [p.name for p in (ROOT / "docs").glob("*M5*acceptance*")]
    assert not stray, "验收报告被当成文档入库了: %s" % stray
