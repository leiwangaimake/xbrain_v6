#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: m5_acceptance.py
Brief: CHK-1-05 -- generate the M5 exit report FROM test results, never by hand

Description:
M5 是交付出口. 在本脚本之前, 它的验收只有一条"报告里必须列全"的元检查 --
也就是说[一份手写的, 把每条都写成 pass 的 markdown 就能通过]. 通过的不是
机器人, 是一个文件. 这是 CLAUDE.md 3.2 形态1 里代价最高的一种.

本脚本做的事只有一件: 跑验收用例, 把[执行结果]转成报告. 报告里的每一条都
必须能追溯到一个真实跑过的 testcase; 追溯不到的一律不出现在报告里, 也不会
被算作通过.

*** 这个方向是不对称的, 必须写清楚.
一个宽松的生成器(允许报告里有没有执行结果支撑的条目)与一个严格的生成器,
在[一切正常]的时候输出完全一样. 差别只在有人想蒙混的时候显现 -- 而那正是
出口验收要防的场景. 所以 verify_report() 是本文件的主体, 不是附属.

*** 为什么用 junit-xml 而不是自己数.
pytest 的 --junit-xml 是内置的(不需要插件), 且它记录的是[实际执行]的 testcase
节点: 一个没跑的用例不会有节点, 一个 xfail 的用例带 skipped/failure 子节点.
自己解析 stdout 会在输出格式变化时静默失配, 而"少认出几条"表现为报告变短,
不是报错.

Boundaries: 不跑真机, 不判断 G-* 标准本身. 只负责"报告等于执行结果".

  python3 scripts/ci/m5_acceptance.py            # 跑用例并生成报告
  python3 scripts/ci/m5_acceptance.py --verify <report.md>
                                                 # 校验一份已有报告
"""

import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

#: 验收用例所在. 报告只从这里的执行结果生成.
SUITE = "tests/acceptance/test_m5_exit.py"

#: CHK-1-05 负责的五条. NO 不从报告里读 -- 报告是产物, 不是真源;
#: 拿产物当清单会让"漏了一条"变成"报告里没有所以不需要".
G_ITEMS = ("G-1", "G-1a", "G-5", "G-6", "G-7")


def run_suite(junit_path):
    """跑验收用例, 把结果写成 junit xml. 返回 pytest 的退出码.

    退出码不作为通过判据: xfail 会让 pytest 返回 0, 而一条 xfail 的 G-*
    显然不是"通过". 判据在 verify_report 里.
    """
    proc = subprocess.run(
        [sys.executable, "-m", "pytest", SUITE, "-q",
         "--junit-xml=" + junit_path],
        capture_output=True, text=True, cwd=ROOT)
    return proc.returncode, proc.stdout + proc.stderr


def parse_junit(junit_path):
    """{用例名: 状态}. 状态取 passed / failed / xfailed / error.

    一个用例[没有出现在 xml 里]与它 passed 是完全不同的两件事, 所以这里
    只登记出现过的节点, 缺席由调用方判定.
    """
    tree = ET.parse(junit_path)
    out = {}
    for case in tree.iter("testcase"):
        name = case.get("name") or ""
        status = "passed"
        for child in case:
            tag = child.tag
            if tag == "failure" or tag == "error":
                status = "failed"
            elif tag == "skipped":
                # pytest 把 xfail 记成 skipped, type 里带 xfail 字样.
                kind = (child.get("type") or "") + (child.get("message") or "")
                status = "xfailed" if "xfail" in kind.lower() else "skipped"
        out[name] = status
    return out


def item_of(case_name):
    """用例名 -> 它验的是哪一条 G-*, 认不出返回 None.

    约定: 函数名以 test_g1_ / test_g1a_ / test_g5_ ... 开头. 归属写在名字里
    而不是另建一张对照表 -- 对照表是第三个会漂移的副本(与 CHK-1-03 同理).

    *** 一句订正: 早先这里写着"g1a 必须先于 g1 匹配, 否则 test_g1a_* 会被
    算到 G-1 头上". 那句话是错的, 变异体实测出来的 -- 把顺序颠倒过来一条
    用例都不红. 原因是前缀[带下划线分隔符]: G-1 的前缀是 "test_g1_", 而
    "test_g1a_foo" 不以它开头. 顺序无关.
    真正要守的是[分隔符不能省]: 前缀写成 "test_g1" 才会把 g1a 吞掉 --
    那一条由 test_g1a_is_not_swallowed_by_g1 钉住.
    """
    for item in ("G-1a", "G-1", "G-5", "G-6", "G-7"):
        prefix = "test_" + item.lower().replace("-", "") + "_"
        if case_name.startswith(prefix):
            return item
    return None


def build_report(results):
    """执行结果 -> 报告文本. 每行都带它的来源用例名."""
    lines = ["# M5 exit acceptance report",
             "",
             "Generated from test execution by scripts/ci/m5_acceptance.py.",
             "Rows without a matching testcase are reported as NOT-RUN; this",
             "file is a product, not a source -- editing it by hand does not",
             "make anything pass (see verify_report).",
             "",
             "| item | verdict | evidence (testcase) |",
             "|---|---|---|"]
    by_item = {}
    for name, status in sorted(results.items()):
        item = item_of(name)
        if item is None:
            continue
        by_item.setdefault(item, []).append((name, status))
    for item in G_ITEMS:
        cases = by_item.get(item, [])
        if not cases:
            lines.append("| %s | NOT-RUN | (no testcase) |" % item)
            continue
        for name, status in cases:
            lines.append("| %s | %s | %s |" % (item, status, name))
    return "\n".join(lines) + "\n"


def verify_report(report_text, results):
    """*** 判据变异体1: 手写一份全 pass 的报告必须被拒.

    校验两个方向, 两个都要:
      (1) 报告里每一条判 passed 的行, 必须有一个[同名且真的 passed]的
          testcase. 手写的行找不到证据 -> 拒;
      (2) 执行结果里每一条 G-* 用例, 必须出现在报告里. 少写一条同样是
          伪造 -- 把一条 failed 从报告里删掉, 剩下的看起来就全绿了.

    返回问题列表, 空列表表示这份报告确实等于那次执行.
    """
    problems = []
    seen = set()
    for line in report_text.split("\n"):
        if not line.startswith("|") or line.startswith("| item"):
            continue
        cells = [c.strip() for c in line.split("|")]
        if len(cells) < 5 or set(cells[1]) <= set("- :"):
            continue
        item, verdict, evidence = cells[1], cells[2], cells[3]
        if verdict == "NOT-RUN":
            continue
        seen.add(evidence)
        actual = results.get(evidence)
        if actual is None:
            problems.append(
                "%s 这一行的证据用例 %r 在本次执行结果里不存在 -- "
                "手写的行不算数" % (item, evidence))
        elif actual != verdict:
            problems.append(
                "%s 报告写 %s 而实际是 %s (用例 %s)"
                % (item, verdict, actual, evidence))
    for name, status in results.items():
        if item_of(name) is None:
            continue
        if name not in seen:
            problems.append(
                "执行了 %s (%s) 但报告里没有它 -- 少写一条与写错一条同样是伪造"
                % (name, status))
    return problems


def main():
    if "--verify" in sys.argv:
        # 校验一份已有报告: 重跑一次用例拿到真结果再比.
        path = sys.argv[sys.argv.index("--verify") + 1]
        with tempfile.TemporaryDirectory() as tmp:
            junit = os.path.join(tmp, "j.xml")
            run_suite(junit)
            results = parse_junit(junit)
        with open(path, encoding="utf-8") as handle:
            problems = verify_report(handle.read(), results)
        for line in problems:
            print("  " + line)
        print("criterion: every reported row is backed by a real testcase")
        return 1 if problems else 0

    with tempfile.TemporaryDirectory() as tmp:
        junit = os.path.join(tmp, "j.xml")
        rc, output = run_suite(junit)
        if not os.path.isfile(junit):
            # 用例一条都没跑起来. NO 不生成一份空报告 -- 空报告在
            # "全部通过"与"什么都没跑"之间没有区别.
            print("pytest did not produce a junit xml; refusing to report")
            print(output[-2000:])
            return 1
        results = parse_junit(junit)
    report = build_report(results)
    sys.stdout.write(report)
    # 摘要行放最后, 便于 CI 抓. NO 不把条数写进任何 markdown(3.7).
    not_run = [i for i in G_ITEMS
               if not any(item_of(n) == i for n in results)]
    xfailed = [n for n, s in results.items()
               if s == "xfailed" and item_of(n)]
    print("criterion: report rows == execution results")
    print("items: %d, not-run: %s, xfailed cases: %d"
          % (len(G_ITEMS), ",".join(not_run) or "none", len(xfailed)))
    # 有 failed 才算门失败. xfail 是[已声明的未实现], 它让报告不全绿,
    # 但不该阻断 CI -- 阻断的话这条门会在实现补齐前被关掉.
    failed = [n for n, s in results.items() if s == "failed"]
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
