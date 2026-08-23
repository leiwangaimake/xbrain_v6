"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cxx_discipline.py
Brief: CHK-1-50 -- 守 scripts/ci/cxx_discipline_audit.py 的规则表与每条规则的活性

Description:
这个门今天处在一个特别容易骗人的状态: ros2_ws/ 下只有 sensor 一个包,
quadruped / chassis_relay 的源码都还没建, 所以 9 条规则里有 7 条[没有扫描
对象]. 一个遍历空目录后 return 0 的实现, 输出与"全部通过"完全一样.

所以本文件不看门在真实仓库上的结论(那个结论今天没有信息量), 而是拿
tests/fixtures/cxx_discipline/ 下的样例逐条问: 这条规则活着吗?

*** 判据逐字: "只跑违规半不算写完".
一个"什么都报"的实现能通过全部违规样例. 所以每条规则都要两半:
  * 违规样例必须被抓到;
  * 合规样例一条都不许误报.
少任何一半, 规则的正确性都没被约束住.

*** 判据点名的三条元测试, 各在下面有对应用例:
  (a) 规则表的行集合 与 TODO 判据列里出现的规则号集合, 双向差集为空;
  (b) 把任一规则的实现体删空后重跑, 它的违规样例必须不再变红 -> 报"已失活";
  (c) 脚本自身必须在全部扫描面之外(它正文里就写着要 grep 的那些字串).

Boundaries: 不判断规则内容对不对(那是 11/13 的事), 只保证[表是完整的,
每条是活的, 边界是声明过的].
"""
from __future__ import annotations

import importlib.util
import pathlib
import re

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
GATE = ROOT / "scripts" / "ci" / "cxx_discipline_audit.py"
FIXTURES = ROOT / "tests" / "fixtures" / "cxx_discipline"
TODO = ROOT / "docs" / "XBRAIN_V6_TODO.md"


def _gate():
    """按路径 import -- 它是 script, 不是包."""
    spec = importlib.util.spec_from_file_location("cxx_discipline_audit", GATE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


#: 从 TODO [判据列]提取纪律规则号.
#:
#: *** (?<!T-) 这个否定环视是实测补上的.
#: 没有它时, "T-CHS-3" / "T-TIER1-1" 这类[测试项编号]会被一并抓进来 --
#: \b 在 "T-CHS" 的连字符处成立. 表现是规则全集多出四个在 11/13 里[找不到
#: 定义行]的号, 而那四个会让下面的双向差集永远红.
_RULE_IN_TODO = re.compile(
    r"(?<!T-)\b(CRL-[0-9]+|DDS-[0-9]+|RT-C[0-9]+|CPP-[0-9]+|PB-[0-9]+)\b")


def _rule_ids_in_todo():
    """TODO 表格第 5 列(判据列)里出现过的纪律规则号."""
    found = set()
    for line in TODO.read_text(encoding="utf-8").splitlines():
        if not line.startswith("| `"):
            continue
        cells = line.split("|")
        if len(cells) < 6:
            continue
        found.update(m.group(1) for m in _RULE_IN_TODO.finditer(cells[5]))
    return found


def _covered_ids(gate):
    """规则表覆盖的规则号.

    PB-5 在表里拆成 PB-5a / PB-5b 两行(同一条文档规则的两个可查侧面:
    发行版判断宏, 与 C++ 标准号). 归一回 PB-5 再比, 否则双向差集会把
    这个拆分报成"多了两个少了一个".
    """
    ids = {r.rid for r in gate.RULES} | set(gate.NOT_STATICALLY_CHECKABLE)
    # also: 一条规则显式声明它同时覆盖了哪些规则号(DDS-4 覆盖 RT-C5).
    # 不算进来的话, 双向差集会把"同一约束的两侧表述"报成漏挂.
    for rule in gate.RULES:
        ids.update(rule.also)
    return {re.sub(r"([0-9])[a-z]$", r"\1", i) for i in ids}


def test_the_rule_table_is_not_empty():
    """*** 守本文件的前提.

    表为空时下面每条用例都空过, 而门本身照样 exit 0. 一个不检查任何东西的
    门报绿, 是 CLAUDE.md 3.2 形态1.
    """
    gate = _gate()
    assert len(gate.RULES) >= 8, "规则表只有 %d 条" % len(gate.RULES)


def test_rule_table_and_todo_agree_in_both_directions():
    """*** 判据元测试 (a): 防漏挂新规则.

    往 TODO 判据列里写一个新规则号而不挂进规则表 -> 这里红. 反过来, 表里
    有一个 TODO 从没提过的号, 也红 -- 那多半是自造规则号(3.5 禁止的那类).

    MUTATION: 从 RULES 里删掉 DDS-3 且不放进 NOT_STATICALLY_CHECKABLE
    -> 立刻红.
    """
    gate = _gate()
    in_todo = _rule_ids_in_todo()
    covered = _covered_ids(gate)
    assert in_todo, "TODO 判据列里一个规则号都没抓到 -- 提取器坏了"
    missing = sorted(in_todo - covered)
    extra = sorted(covered - in_todo)
    assert not missing, (
        "TODO 判据列提到但规则表没有(要么实现, 要么写进 "
        "NOT_STATICALLY_CHECKABLE 并说明为什么): %s" % missing)
    assert not extra, (
        "规则表里有 TODO 从没提过的规则号 -- 是自造的吗: %s" % extra)


def test_every_rule_id_exists_verbatim_in_the_design_docs():
    """规则号必须在 11 或 13 里真实存在, NO 不自造.

    一个自造的规则号读起来与真规则毫无区别, 而它背后没有任何设计依据 --
    等于把一条个人偏好伪装成契约.
    """
    gate = _gate()
    docs = "\n".join(
        (ROOT / "docs" / name).read_text(encoding="utf-8")
        for name in ("11-接口契约.md", "13-quadruped与Tier1详细设计.md"))
    all_ids = {r.rid for r in gate.RULES} | set(gate.NOT_STATICALLY_CHECKABLE)
    for rule in gate.RULES:
        # also 声明的号同样不许自造 -- 否则可以靠往 also 里塞一个假号让
        # 上面那条双向差集变绿.
        all_ids.update(rule.also)
    for rid in sorted(all_ids):
        base = re.sub(r"([0-9])[a-z]$", r"\1", rid)
        assert base in docs, "规则号 %s 在 11/13 里找不到" % rid


@pytest.mark.parametrize("rid", sorted(
    d.name for d in (pathlib.Path(__file__).resolve().parents[2]
                     / "tests" / "fixtures" / "cxx_discipline").iterdir()
    if d.is_dir()))
def test_each_rule_catches_bad_and_spares_good(rid):
    """*** 每条规则两半都验 -- 判据逐字"只跑违规半不算写完".

    违规半单独看不出问题: 一条 pattern 写成 "." 的规则能抓住每一个违规样例,
    也能把每一个合规样例报成违规. 合规半就是用来杀掉那种实现的.
    """
    gate = _gate()
    rule = next((r for r in gate.RULES if r.rid == rid), None)
    assert rule is not None, "样例目录 %s 没有对应的规则" % rid
    folder = FIXTURES / rid
    bad = sorted(folder.glob("bad_*"))
    good = sorted(folder.glob("good_*"))
    assert bad and good, "%s 缺违规或合规样例" % rid

    status_bad, hits = gate.audit_rule(rule, files=[str(p) for p in bad])
    assert status_bad == "VIOLATION", (
        "%s 的违规样例没被抓到 -- 规则可能已失活" % rid)
    assert hits, "%s 报了违规却没给出命中位置" % rid

    status_good, ghits = gate.audit_rule(rule, files=[str(p) for p in good])
    assert status_good == "OK", (
        "%s 的合规样例被误报: %s" % (rid, ghits))


def test_every_checkable_rule_has_a_fixture_pair():
    """反向: 规则表里的每条都必须有样例目录.

    没有这条, 新加一条规则可以完全不配样例 -- 它的正确性从此不受任何约束,
    而上面那条参数化用例[不会红], 因为它是按样例目录枚举的.
    """
    gate = _gate()
    have = {d.name for d in FIXTURES.iterdir() if d.is_dir()}
    missing = sorted({r.rid for r in gate.RULES} - have)
    assert not missing, "这些规则没有 fixtures: %s" % missing


def test_a_gutted_rule_is_reported_as_dead(tmp_path):
    """*** 判据元测试 (b): 把规则实现体删空, 它的违规样例必须不再变红.

    这是在证明[上面那条参数化用例真的在起作用]. 如果一条规则被改成永远
    不命中, 而违规样例照样"通过", 那说明抓到违规的其实不是这条规则.

    做法: 把 pattern 换成一个不可能命中的模式, 再喂违规样例.
    """
    gate = _gate()
    rule = next(r for r in gate.RULES if r.rid == "DDS-3")
    bad = sorted((FIXTURES / "DDS-3").glob("bad_*"))
    # 先确认原规则确实抓得到 -- 否则下面的"不再变红"毫无意义.
    assert gate.audit_rule(rule, files=[str(p) for p in bad])[0] == "VIOLATION"
    gutted = gate.Rule(rule.rid, rule.doc, rule.scan,
                       r"ZZ_THIS_CANNOT_MATCH_ANYTHING_ZZ", rule.sense,
                       rule.why, suffixes=rule.suffixes)
    status, _hits = gate.audit_rule(gutted, files=[str(p) for p in bad])
    assert status != "VIOLATION", (
        "规则体已被掏空, 违规样例却仍被判违规 -- 抓到它的不是这条规则")


def test_the_auditor_is_outside_every_scan_surface():
    """*** 判据元测试 (c): 判据自伤.

    本脚本正文里就写着 CYCLONEDDS_URI / dds_create_writer / 7447 这些要被
    grep 的字串. 一旦它落进扫描面, 每条规则都会命中它自己 -> 恒红 ->
    被人放宽成"包含即可" -> 恒绿 (CLAUDE.md 3.2 形态3 的标准剧本).

    今天它在 scripts/ci/ 而扫描面只有 ros2_ws/ 与 common/, 是"天然在外" --
    而天然的东西会变, 所以钉住它.

    MUTATION: 往某条规则的 scan 里加 "scripts" -> 立刻红.
    """
    gate = _gate()
    rel = GATE.relative_to(ROOT).as_posix()
    for rule in gate.RULES:
        for scan in rule.scan:
            assert not rel.startswith(scan.rstrip("/") + "/"), (
                "规则 %s 的扫描面 %s 覆盖了审计脚本自己" % (rule.rid, scan))
    # 更强的一条: 拿真扫描面文件列表去查, 免得路径前缀比较有漏网.
    for rule in gate.RULES:
        files = gate._files_for(rule)
        assert str(GATE) not in files, (
            "规则 %s 的扫描结果里含审计脚本自己" % rule.rid)


def test_fixtures_are_outside_every_scan_surface():
    """样例目录同理: 它里面装的就是[必须被抓到的违规代码].

    落进扫描面的话, 门在干净仓库上会报 9 条违规, 然后被人加一条 "跳过
    tests/" 的例外 -- 而那条例外迟早会被写宽.
    """
    gate = _gate()
    rel = FIXTURES.relative_to(ROOT).as_posix()
    for rule in gate.RULES:
        for scan in rule.scan:
            assert not rel.startswith(scan.rstrip("/") + "/"), (
                "规则 %s 的扫描面覆盖了 fixtures" % rule.rid)


def test_the_gate_declares_its_scan_surface():
    """判据逐字: 必须打印扫了哪些包, 哪些后缀, NO 不得只写"全仓".

    一个不声明扫描面的结论, 读的人无法判断它覆盖了什么 -- 本仓已经因为
    这个吃过亏(3.2 形态6: "在用未登记码 = 0"而扫描范围只含 7 册).
    """
    import subprocess
    import sys as _sys
    proc = subprocess.run([_sys.executable, str(GATE)],
                          capture_output=True, text=True, cwd=str(ROOT))
    assert "scan surface:" in proc.stdout, "没有打印扫描面声明"
    assert "packages=" in proc.stdout and "suffixes=" in proc.stdout
    assert ".cc" in proc.stdout, "后缀没列出来"


def test_rules_without_a_scan_target_are_announced_not_swallowed():
    """*** 今天最要紧的一条.

    9 条规则里 7 条的扫描目录根本不存在(quadruped / chassis_relay 未建).
    这些规则今天一条也没在守 -- 而那不是"通过". 门必须把它们打出来.

    MUTATION: 把 NO-TARGET 那段 print 删掉 -> 红. 删掉之后门的输出与
    "全部通过"一模一样, 这正是要防的.
    """
    import subprocess
    import sys as _sys
    gate = _gate()
    homeless = [r.rid for r in gate.RULES if not gate._files_for(r)]
    if not homeless:
        pytest.skip("所有规则都有扫描对象了 -- C++ 侧已建成, 本用例失去前提")
    proc = subprocess.run([_sys.executable, str(GATE)],
                          capture_output=True, text=True, cwd=str(ROOT))
    assert "NO-TARGET" in proc.stdout, (
        "有 %d 条规则没有扫描对象, 门却没说:\n%s" % (len(homeless), proc.stdout))
    for rid in homeless:
        assert rid in proc.stdout, "%s 没有扫描对象却没被点名" % rid


def test_not_checkable_entries_each_say_why():
    """静态查不了的那几条, 每条都要说清为什么.

    一条只写着"查不了"的记录, 与一条写着"需要跨函数类型流分析"的记录,
    对下一个想补上它的人价值差一个量级. 而且理由写下来才能被复核 --
    也许下一个人有更好的办法.
    """
    gate = _gate()
    for rid, why in gate.NOT_STATICALLY_CHECKABLE.items():
        assert len(why) > 40, "%s 的理由太短, 没法复核: %r" % (rid, why)


def test_comment_lines_are_stripped_before_matching():
    """*** 这条守的是本门第一次运行就踩到的误报.

    PB-5a 的模式命中了 common/CMakeLists.txt 里的一行注释
    "no if(ROS_DISTRO ...)" -- 那句话正是在说这里遵守了规则.

    危害不止噪声: 本项目要求详细头注(2.5), 而头注里最该写的就是"本文件
    不用 CYCLONEDDS_URI"这类边界说明. 不剥注释, 越是注释写得好的文件越
    会被报违规 -- 门先惩罚守规矩的人, 然后被关掉.

    MUTATION: 让 code_lines 原样返回 -> 红.
    """
    gate = _gate()
    cxx = "\n".join([
        "/* header says: this file must not use CYCLONEDDS_URI at all. */",
        "// and neither may it call dds_create_writer",
        "void f() { g(); }",
    ])
    stripped = gate.code_lines(cxx, is_cmake=False)
    assert "CYCLONEDDS_URI" not in "".join(stripped)
    assert "dds_create_writer" not in "".join(stripped)
    # 代码那一行必须留下来 -- 剥过头会让规则什么都查不到.
    assert "g()" in "".join(stripped)
    cmake = "# no if(ROS_DISTRO STREQUAL humble) here\nset(CMAKE_CXX_STANDARD 17)"
    out = "".join(gate.code_lines(cmake, is_cmake=True))
    assert "ROS_DISTRO" not in out
    assert "CMAKE_CXX_STANDARD 17" in out


def test_block_comment_state_carries_across_lines():
    """块注释跨行: 本项目的头注全是 /* ... */ 多行块.

    只处理单行的实现会在头注第二行开始漏 -- 而头注恰恰是边界说明最集中的
    地方, 漏在那里等于没剥.
    """
    gate = _gate()
    text = "\n".join([
        "/*",
        " * This file must never touch CYCLONEDDS_URI.",
        " */",
        "int x = 1;",
    ])
    out = gate.code_lines(text, is_cmake=False)
    assert "CYCLONEDDS_URI" not in "".join(out), "块注释没有跨行剥干净"
    assert "int x = 1;" in "".join(out)
