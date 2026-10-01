"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_affinity_gate.py
Brief: CHK-1-03 -- 守 scripts/ci/check_affinity.py 与 10 S3.2 的核分配

Description:
10 S3.2 的核分配不是建议: 核 4 由 P1 独占是 20 Hz 周期 P99 的前提, 核 6 集中
四个 RT/急停类 C++ 进程是因为它们无 GIL 无 GC, 核 7 被明确标注"过载是有意
接受的代价". 这些数字同时活在两个地方 -- 文档表体与 systemd 单元 -- 谁改了
都不会通知对方.

分叉的表现是[没有表现]: 进程照常起, 环照常跑, 只是抖动变大, 要到 T7 时延
实测才暴露, 那时很难倒推回某个单元文件里的一个数字.

*** 本文件守四件事, 每件配变异体:
  1. 门读的是[表体]而不是硬编码副本 -- 判据变异体2 打在这里. 做法是喂一份
     改过核号的文档副本, 门的结论必须跟着变;
  2. 单元改核号而不改表 -> 必须红且点名那个进程(判据变异体1);
  3. 进程名 -> 单元名是[推导]的, 不是手维护的对照表(第三个会漂移的副本);
  4. PENDING_DOC_DECISION 的边界钉死 -- 新增一条就红.

*** 判据 (2)(3)(4) 要真机, 本文件只做 (1), 三条各说明卡在哪:
  (2) taskset -pc 读实际掩码 -- 要进程真在跑;
  (3) chrt -p 比 SCHED_FIFO 优先级 -- 要 quadruped 在跑, 而它是 C++ 侧
      尚未建成的进程(NEXT S2), 没有 Tier1 线程可比;
  (4) /proc/cmdline 里的 isolcpus -- 要目标机的内核 cmdline, 开发机上读到的
      是开发机自己的, 断言它等于部署基线毫无意义.
三条都归 needs_orin/needs_chassis 档, NO 不在这里写一个恒真的替身 --
那正是 CLAUDE.md 3.2 形态1.

Boundaries: 不设置任何亲和, 不改单元. 只比对声明.
"""
from __future__ import annotations

import importlib.util
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
GATE = ROOT / "scripts" / "ci" / "check_affinity.py"
DOC = ROOT / "docs" / "10-顶层设计.md"
UNIT_DIR = ROOT / "deploy" / "systemd"


def _gate():
    """按路径 import -- 它是 script, 不是包."""
    spec = importlib.util.spec_from_file_location("check_affinity", GATE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run(*args):
    return subprocess.run([sys.executable, str(GATE)] + list(args),
                          capture_output=True, text=True, cwd=str(ROOT))


def test_the_table_parses_to_something():
    """*** 守本文件的前提.

    解析到 0 个进程时, 双向差集两边都空 = "完全一致", 门报绿. 下面每条用例
    也都会空过. 一个读不到表的门报通过, 是 CLAUDE.md 3.2 形态1 的标准长相.
    门自己在这种情况下会抛 SystemExit, 这里把那个行为也钉住.
    """
    table = _gate().parse_core_table(DOC.read_text(encoding="utf-8"))
    assert len(table) >= 14, "只解析到 %d 个进程, 表结构可能变了" % len(table)
    # 核 4 必须恰好是 P1 一个进程 -- 独占是 20 Hz 的前提, 不是偏好.
    on_core_4 = [p for p, (cores, _i) in table.items() if 4 in cores]
    assert on_core_4 == ["xbrain_motion"], (
        "核 4 上有 %s -- 10 S3.2 要求 P1 独占" % on_core_4)


def test_anchor_must_be_unique():
    """*** 这条是本轮踩出来的.

    第一版锚点写的是"CPU 亲和"四个字, 而 10 在别处也有这四个字(S3.1 进程
    清单表里 P1 那一行). find() 命中的是那一处, 解析器从错的位置往下扫,
    吐出 VACUUM / task.db / h_factor 这些完全不相干的映射 -- [而且没有抛].
    一个解析到错表却照常出结果的门, 比一个报错的门危险得多.

    MUTATION: 把锚点改短(去掉括号那半截), 命中数变 >1, 门必须 SystemExit
    而不是"取第一处先跑着".
    """
    text = DOC.read_text(encoding="utf-8")
    gate = _gate()
    assert text.count(gate._TABLE_ANCHOR) == 1, (
        "锚点 %r 在 10 里命中 %d 次, 不唯一"
        % (gate._TABLE_ANCHOR, text.count(gate._TABLE_ANCHOR)))
    with pytest.raises(SystemExit):
        gate.parse_core_table(text.replace(gate._TABLE_ANCHOR, "X", 1))


def test_gate_is_green_on_the_committed_tree():
    """基线. 一个在干净树上就红的门会被当噪声关掉, 那时它守的东西全丢."""
    proc = _run()
    assert proc.returncode == 0, "干净树上就红了:\n%s%s" % (proc.stdout, proc.stderr)


def test_a_unit_rebinding_without_a_doc_change_is_caught():
    """*** 判据变异体1: 把某个单元的 CPUAffinity 改成 7 而不改表.

    必须红[并点名那个进程]-- 只说"有 1 处不一致"的话, 运维要挨个 grep 20 个
    单元才知道是哪个.
    """
    target = UNIT_DIR / "xbrain-p1-motion.service"
    original = target.read_text(encoding="utf-8")
    injected = original.replace("CPUAffinity=4", "CPUAffinity=7", 1)
    # 注入必须真的改了文件. 本仓踩过一次"变异体没生效, 看起来像断言失效".
    assert injected != original, "注入没改动文件, 这次变异体不成立"
    try:
        target.write_text(injected, encoding="utf-8")
        proc = _run()
        assert proc.returncode != 0, "单元改了核号却报绿"
        assert "xbrain_motion" in proc.stdout, (
            "报了不一致但没点名进程:\n%s" % proc.stdout)
        assert "MISMATCH" in proc.stdout
    finally:
        target.write_text(original, encoding="utf-8")


def test_removing_the_affinity_line_is_caught_separately():
    """"绑错了"与"根本没绑"要分开报: 前者改一个数字, 后者要判断该不该绑,
    两种处置不同, 一条消息盖住两件事会把人引到错的方向."""
    target = UNIT_DIR / "xbrain-quadruped.service"
    original = target.read_text(encoding="utf-8")
    stripped = "\n".join(ln for ln in original.split("\n")
                         if not ln.startswith("CPUAffinity="))
    assert stripped != original
    try:
        target.write_text(stripped, encoding="utf-8")
        proc = _run()
        assert proc.returncode != 0
        assert "UNBOUND" in proc.stdout and "quadruped" in proc.stdout
    finally:
        target.write_text(original, encoding="utf-8")


def test_the_gate_reads_the_table_not_a_hardcoded_copy():
    """*** 判据变异体2, 也是这个门最容易退化成的样子.

    一个把核分配抄进代码的门, 在表体被改动后[继续报绿]-- 它守的是自己那份
    副本, 不是设计. 从外面看不出区别: 两种实现在干净树上都是绿的.

    证伪办法: 喂一份[改过核号]的文档副本. 真读表体的门会立刻报 MISMATCH
    并点名; 读硬编码副本的门结论不变, 依旧绿.
    """
    with tempfile.TemporaryDirectory() as d:
        fake = pathlib.Path(d) / "10.md"
        text = DOC.read_text(encoding="utf-8")
        gate = _gate()
        idx = text.index(gate._TABLE_ANCHOR)
        head, tail = text[:idx], text[idx:]
        # 只改表体里 quadruped 那一行的核号 5 -> 0.
        patched = tail.replace("| **5** | ★ **`quadruped` 独占**",
                               "| **0** | ★ **`quadruped` 独占**", 1)
        assert patched != tail, "表体行的写法变了, 本用例的注入不再成立"
        fake.write_text(head + patched, encoding="utf-8")
        proc = _run("--doc", str(fake))
        assert proc.returncode != 0, (
            "改了表体核号, 门却没反应 -- 它读的很可能不是表体:\n%s"
            % proc.stdout)
        assert "quadruped" in proc.stdout


def test_unit_names_are_derived_not_hand_mapped():
    """*** 第三个会漂移的副本, 是这类门最常见的隐藏成本.

    表在文档里, 单元在磁盘上; 中间再放一张手写对照表, 就有了三处需要人保持
    同源的地方. 所以映射必须是[推导]的, 例外必须少且各写理由.

    MUTATION: 往 _UNIT_EXCEPTIONS 里加一条不必要的映射 -> 这里红.
    """
    gate = _gate()
    # 上限从 2 提到 4: 补登 payload-service 时暴露出 Nav2 与 payload-service
    # 两条名字对不上的映射 -- 它们一直存在, 只是此前表体里看不见它们
    # (payload 整表未列, Nav2 没加反引号), 所以门也就无从发现.
    # ! 提上限是有代价的, 每一条例外都是一处要人维护的同源关系, 所以要
    # 逐条写理由(下面那个 len(why) 断言在守).
    assert len(gate._UNIT_EXCEPTIONS) <= 4, (
        "例外映射变多了(%d 条) -- 每一条都是一处需要人维护的同源关系, 需复核"
        % len(gate._UNIT_EXCEPTIONS))
    # 每条例外都要在源码里带理由(注释形式). 没有这条, 上限一提就会有人
    # 往里塞映射 -- 而一张没人说得清为什么的对照表, 就是那个"第三个会漂移
    # 的副本".
    import inspect
    src = inspect.getsource(gate)
    block = src[src.index("_UNIT_EXCEPTIONS = {"):]
    block = block[:block.index("\n}\n")]
    for key in gate._UNIT_EXCEPTIONS:
        idx = block.index('"%s"' % key)
        before = block[:idx]
        # 该行之前必须有注释(最近一段以 # 开头的连续行).
        prev = [ln for ln in before.split("\n") if ln.strip()][-1]
        assert prev.strip().startswith("#"), (
            "例外映射的理由缺失: %s 上面没有注释" % key)

    # 推导规则本身要能自证: 表体里带 (Pn) 的必须落到 xbrain-pn-*.
    assert gate.unit_name_for("xbrain_motion", "P1") == "xbrain-p1-motion.service"
    assert gate.unit_name_for("chassis_relay", "") == "xbrain-chassis-relay.service"


def test_every_derived_unit_actually_exists():
    """推导出来的单元名必须在磁盘上真存在.

    没有这条, 一条推导错的规则会表现为"该单元没绑核", 而真相是名字根本就
    没对上 -- 两者读起来一模一样, 处置完全相反.
    """
    gate = _gate()
    table = gate.parse_core_table(DOC.read_text(encoding="utf-8"))
    missing = [gate.unit_name_for(p, i) for p, (_c, i) in table.items()
               if not (UNIT_DIR / gate.unit_name_for(p, i)).is_file()]
    assert not missing, "推导出的单元名在磁盘上不存在: %s" % missing


#: 允许出现在 PENDING_DOC_DECISION 里的键, 与门自己那份分开写 --
#: 一份被测者自己维护的白名单证明不了任何事.
#
# 2026-08-23 起为空: payload-service 已由用户裁决并补登进 10 S3.2 核 7 行,
# 那一条按规矩从待裁决清单移走了. 空集合是[好状态], 但它会让下面按条遍历
# 的用例空过 -- 所以那条用例自己带了一句"集合为空时也要说话"的断言.
_EXPECTED_PENDING = {}


def test_pending_doc_decisions_are_exactly_what_was_reviewed():
    """*** 这个口子只能用来"等裁决", NO 不能用来让门变绿.

    集合必须与这里逐条复核过的那份完全相等: 新增 -> 红, 直到有人写清理由;
    删除 -> 也红, 免得口子比它的理由活得久.

    MUTATION: 往 PENDING_DOC_DECISION 里加任意一行 -> 立刻红.
    """
    actual = set(_gate().PENDING_DOC_DECISION)
    assert actual == set(_EXPECTED_PENDING), (
        "待裁决集合与已复核的不一致: 多了 %s, 少了 %s"
        % (sorted(actual - set(_EXPECTED_PENDING)),
           sorted(set(_EXPECTED_PENDING) - actual)))


def test_each_pending_entry_points_at_where_the_decision_lives():
    """一条写着"先放过"的待裁决, 与一条指向 NEXT SW-20 的待裁决, 对下一个
    读者的价值差一个量级."""
    gate = _gate()
    if not gate.PENDING_DOC_DECISION:
        # *** 空集合时本用例会空过, 所以在这里显式说明它是空的.
        # 一条"遍历空集合因而通过"的断言与一条"逐条查过都对"的断言, 在
        # 报告里看不出区别 -- 而前者什么也没验证(CLAUDE.md 3.2 形态1).
        assert _EXPECTED_PENDING == {}, (
            "门里没有待裁决项, 但复核清单里还有 %s" % sorted(_EXPECTED_PENDING))
        return
    for unit, why in gate.PENDING_DOC_DECISION.items():
        assert _EXPECTED_PENDING[unit] in why, (
            "%s 的理由没有指向后续条目(期望提到 %s): %r"
            % (unit, _EXPECTED_PENDING[unit], why))
    # 指向的条目必须真的写在 NEXT 里, 否则这是个死指针.
    nxt = (ROOT / "docs" / "NEXT.md").read_text(encoding="utf-8")
    for tag in set(_EXPECTED_PENDING.values()):
        assert tag in nxt, "%s 在 docs/NEXT.md 里不存在" % tag


def test_pending_entries_are_still_absent_from_the_table():
    """反向: 一旦文档补登了它, 这条就必须从待裁决清单里移走.

    否则一个已经解决的问题会永远挂在"等裁决"里, 而真正等裁决的那些就被
    淹没了.
    """
    gate = _gate()
    table = gate.parse_core_table(DOC.read_text(encoding="utf-8"))
    claimed = {gate.unit_name_for(p, i) for p, (_c, i) in table.items()}
    resolved = sorted(set(gate.PENDING_DOC_DECISION) & claimed)
    assert not resolved, (
        "10 S3.2 现在登记了这些单元, 应从 PENDING_DOC_DECISION 移除: %s"
        % resolved)


def test_units_cite_the_table_rather_than_restating_the_number():
    """每个绑核的单元都要写清它凭什么是这个核.

    一个光秃秃的 CPUAffinity=6 会被下一个人当成随手填的, 然后为了"让它跑
    起来"改掉. 注释里必须带上 10 S3.2 的出处.
    """
    bad = []
    for path in sorted(UNIT_DIR.glob("*.service")):
        lines = path.read_text(encoding="utf-8").split("\n")
        for i, line in enumerate(lines):
            if not line.startswith("CPUAffinity="):
                continue
            # 往上找最近的连续注释块.
            block = []
            j = i - 1
            while j >= 0 and lines[j].startswith("#"):
                block.append(lines[j])
                j -= 1
            if not any(re.search(r"S3\.2|3\.2", b) for b in block):
                bad.append(path.name)
    assert not bad, "这些单元的 CPUAffinity 没有指向 10 S3.2: %s" % bad


@pytest.mark.needs_orin
def test_runtime_masks_match_the_table():
    """判据(2): taskset -pc 的实际掩码必须与表体一致.

    NO 不在开发机上写一个恒真的替身 -- 进程没跑时"读不到掩码"与"掩码正确"
    是两件事, 把前者当通过就是 3.2 形态1.
    """
    if shutil.which("taskset") is None:
        pytest.skip("taskset absent")
    pytest.skip("needs the full stack running on the target; see NEXT SW-20")


# 判据(3) 的扫描面与登记表.
#
# 受 10 S3.2 那条约束的三个进程, 连同它们的源码树. teleop_input 的格子是
# None 而不是从表里删掉 -- 它 NEXT S2 标着"未建 [GATED-HW], 依赖遥控器",
# 而一个从扫描面里消失的名字与一个扫过且干净的名字在结果上不可区分
# (CLAUDE.md 3.2 形态六). 判据会把它作为 ABSENT 报出来.
#
# rtk_driver 没有自己的包: 它的实现是 ros2_ws/sensor/src/rtk_driver.cc.
# 这里扫整个 sensor 包而不是单个文件, 取保守侧 -- 多扫几个同包文件不会漏判,
# 少扫一个会.
_RT_CONSTRAINED = {
    "chassis_relay": "ros2_ws/chassis_relay",
    "rtk_driver": "ros2_ws/sensor",
    "teleop_input": None,
}

# 把线程切到 SCHED_FIFO 的调用形态. 前两个是 common/include/xbrain/rtcomm/
# rt_thread.h 的封装(本仓唯一正当入口), 后两个是绕过封装的裸系统调用.
_FIFO_CALLS = ("apply_fifo_priority(", "start_fifo_thread(",
               "pthread_setschedparam(", "sched_setscheduler(")

# 已登记的实时优先级: 调用点 -> 优先级. 空表不是占位, 它是今天的事实 --
# 这三个进程一个都没有设 SCHED_FIFO, 跑在普通优先级上.
#
# *** 这个空表正是本判据的要点. 约束今天成立, 但成立的原因是"他们恰好没设",
# 不是有东西在保证. chassis_relay 在急停链路上(CRL-5 单跳 < 200 us), 给它加
# 实时优先级是很自然的下一步 -- 那一天如果没有本判据, 不会有任何东西变红.
_DECLARED_RT_PRIORITIES: dict = {}


def _quadruped_ctrl_priority():
    """quadruped Tier 1 线程的 SCHED_FIFO 优先级, 读 configs/ 源.

    值的执行体在 quadruped_config.cc: require_int("realtime.
    sched_fifo_priority.ctrl"), 缺键即拒绝启动. 这里读同一个键的源文件 --
    tests/ 不在 no_config_source_read 的 SCAN_DIRS 内, 夹具读源是允许的
    (那条 lint 的 SCAN_DIRS 注释自己写明了理由).
    """
    import yaml
    with open(ROOT / "configs" / "quadruped.yaml", encoding="utf-8") as fh:
        tree = yaml.safe_load(fh)
    node = tree
    for key in ("quadruped", "realtime", "sched_fifo_priority", "ctrl"):
        assert isinstance(node, dict) and key in node, (
            "configs/quadruped.yaml 里找不到 realtime.sched_fifo_priority.ctrl "
            "-- 13 S9.1 的线程表没有落点, 本判据失去比较基准")
        node = node[key]
    assert isinstance(node, int), "ctrl 优先级不是整数: %r" % (node,)
    return node


def _scan_fifo_calls(tree):
    """(文件, 行号, 行文) for 每个 SCHED_FIFO 设置调用, 跳过注释行."""
    hits = []
    for dirpath, dirnames, filenames in os.walk(tree):
        dirnames[:] = [d for d in dirnames if d not in ("build", "install", "log")]
        for name in filenames:
            if not name.endswith((".cc", ".cpp", ".h", ".hpp")):
                continue
            path = os.path.join(dirpath, name)
            with open(path, encoding="utf-8", errors="replace") as fh:
                for lineno, line in enumerate(fh, 1):
                    stripped = line.strip()
                    if stripped.startswith(("//", "*", "/*")):
                        continue
                    if any(call in line for call in _FIFO_CALLS):
                        hits.append((os.path.relpath(path, ROOT), lineno,
                                     stripped[:90]))
    return hits


def test_realtime_priorities_are_below_quadruped_tier1():
    """判据(3): chassis_relay / rtk_driver / teleop_input 的 SCHED_FIFO
    优先级必须严格小于 quadruped Tier 1 线程 (10 S3.2 逐字, 理由是"防止将来
    核 6 被合并回核 5 时静默产生优先级反转").

    *** 本判据 2026-10-01 从一句无条件 skip 改写而来, 原理由逐字是
    "quadruped (C++) not built yet; see NEXT S2" -- 而 quadruped 2026-09-21
    已上机, 那句话早就不成立了. 顺着查下去发现更要紧的: 那条约束当时没有
    任何执行体 -- deploy/systemd/ 下零 CPUSchedulingPriority/Policy, 三个进程
    源码里零 SCHED_FIFO 设置调用. 约束"成立", 但成立的原因是巧合 (NEXT SW-30).

    改成静态判据之后它不再需要任何进程在跑, needs_chassis 标记因此摘掉 --
    原来那个标记本身就是上一版实现方式的产物, 不是约束的性质.

    *** 它不能证明什么, 一并写明 (3.2 形态七): 这是对源码与单元文件的
    静态检查. 一个在运行期由外部工具 (chrt / systemd drop-in / 父进程继承)
    改上去的优先级, 不出现在任何被扫文件里, 本判据看不见. 运行期那一半是
    判据(2) 的事, 它要全栈在跑 (NEXT SW-20).
    """
    ctrl_priority = _quadruped_ctrl_priority()

    # 扫描面逐个报出来, 包括不存在的那个 -- 无条件打印, 不只在失败时.
    surface, absent = [], []
    for proc, tree in sorted(_RT_CONSTRAINED.items()):
        if tree is None or not (ROOT / tree).is_dir():
            absent.append(proc)
        else:
            surface.append((proc, tree))
    print("判据(3) 扫描面: quadruped ctrl=SCHED_FIFO %d (configs/quadruped.yaml)"
          % ctrl_priority)
    for proc, tree in surface:
        print("  扫 %-16s %s" % (proc, tree))
    for proc in absent:
        print("  ABSENT %-13s 尚未建成, 本轮无源码可扫 (NEXT S2)" % proc)
    assert surface, "三个受约束进程一个都不存在, 本判据无对象可扫"

    # systemd 单元里的声明式优先级.
    unit_hits = []
    units = ROOT / "deploy" / "systemd"
    if units.is_dir():
        for unit in sorted(units.glob("*.service")):
            for lineno, line in enumerate(unit.read_text().split("\n"), 1):
                if line.startswith(("CPUSchedulingPolicy", "CPUSchedulingPriority")):
                    unit_hits.append((unit.name, lineno, line.strip()))
    print("  systemd 单元里的调度声明: %d 处" % len(unit_hits))

    # 源码里的 SCHED_FIFO 设置点.
    code_hits = []
    for proc, tree in surface:
        for rel, lineno, text in _scan_fifo_calls(str(ROOT / tree)):
            code_hits.append((proc, rel, lineno, text))

    undeclared = [h for h in code_hits
                  if "%s:%d" % (h[1], h[2]) not in _DECLARED_RT_PRIORITIES]
    assert not undeclared, (
        "这些调用点把线程切到了 SCHED_FIFO, 而 _DECLARED_RT_PRIORITIES 里没有登记:\n"
        + "\n".join("  %s  %s:%d  %s" % h for h in undeclared)
        + "\n10 S3.2 要求这三个进程的优先级严格低于 quadruped 的 Tier 1 (ctrl=%d).\n"
          "登记它: 在 _DECLARED_RT_PRIORITIES 里加一行 '<文件>:<行>': <优先级>, "
          "并在 NEXT 里写清依据.\n"
          "删掉本断言不是选项 -- 它是这条约束唯一的执行体 (NEXT SW-30)." % ctrl_priority)

    for where, priority in sorted(_DECLARED_RT_PRIORITIES.items()):
        assert priority < ctrl_priority, (
            "%s 登记的优先级 %d 不低于 quadruped ctrl 的 %d -- 10 S3.2 要求严格小于, "
            "理由是核合并后的静默优先级反转" % (where, priority, ctrl_priority))
