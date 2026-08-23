#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: gen_drift_gate.py
Brief: CHK-1-49 -- one gate over every checked-in generated artifact

Description:
本项目把若干生成物[入库]: errors.h 从 codes.yaml 生成, closed_sets.h 从
sets.yaml 生成, whitelists.py 与 11 S1.1.6/S2.2 的表体对齐. 入库的好处是
消费者(C++ 编译, 运行期 import)不必依赖生成器; 代价是[副本会漂移].

漂移的表现是本项目最怕的那种: 有人手改了 errors.h 加一个码, 编译过, 测试绿,
而 codes.yaml -- 单一真源 -- 里没有这个码. 于是 Python 侧与 C++ 侧的错误码
闭集分叉, 而两边各自都"自洽". 直到联调才会发现, 且现象是一条码在一侧不认识.

本门做的事: 逐条重跑生成器, 与入库副本比, 不等就红并打出差异行.

*** 为什么需要一个[登记表], 而不是"扫描所有带 GENERATED 头注的文件".
两种做法各能抓到对方漏掉的一半, 所以两种都要:
  * 登记表 -> 逐条跑: 抓"文件被手改了";
  * 双向差集(元测试, tests/ci/test_gen_drift_gate.py) -> 抓"新加了生成物
    但忘了登记", 那种文件谁也不会去比对它, 漂移可以存在到天荒地老.
判据的变异体 (c) 打的正是后者.

*** 三条登记项里有一条不是逐字节比对, 这个差异必须写明白.
whitelists.py 的生成器(scripts/doccheck/whitelist_gen.py)不产出该文件的完整
文本 -- 它从 11 提取出五张白名单, 与入库文件里的数据结构做[语义]比对. 所以
它这一条走 --check 子进程, 不走逐字节. 硬把它写成逐字节, 只能靠"再写一个渲染
器", 那个渲染器会成为第二个真源, 比现在更糟.

Boundaries: 只比对, 不修. 修的办法是跑生成器自己的 --write. 本门不写任何
文件, 也不碰工作树 -- 一个"顺手帮你重新生成"的门是永远绿的(CLAUDE.md 3.2
形态①), 因为它在检查之前就把差异抹掉了.

  python3 scripts/ci/gen_drift_gate.py          # 比对, 非零即漂移
  python3 scripts/ci/gen_drift_gate.py --list   # 打印登记表
"""

import difflib
import importlib.util
import os
import subprocess
import sys

#: 仓库根. 本脚本在 scripts/ci/ 下, 所以上两级.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))


class Entry:
    """登记表的一行: 一个生成器, 它的生成物, 以及用哪种方式比对.

    mode 只有两个值, 且都必须显式写出来 -- 默认成其中一种会让新增登记项在
    作者没想清楚的情况下选到错的那个:
      "render"  生成器暴露 render() -> str, 与生成物逐字节比;
      "check"   生成器自带 --check(语义比对), 跑子进程看退出码.
    """

    def __init__(self, generator, outputs, mode, why=""):
        assert mode in ("render", "check"), mode
        self.generator = generator      # 相对仓库根
        self.outputs = tuple(outputs)   # 相对仓库根
        self.mode = mode
        self.why = why                  # mode == "check" 时必须说明理由


#: 生成器登记表. 新增生成物必须加在这里, 否则元测试红(判据变异体 c).
REGISTRY = (
    Entry("scripts/gen/gen_errors.py",
          ("common/include/xbrain/errors/errors.h",),
          "render"),
    Entry("scripts/gen/closed_sets_h.py",
          ("common/include/xbrain/enums/closed_sets.h",),
          "render"),
    Entry("scripts/doccheck/whitelist_gen.py",
          ("xbrain/common/zenoh/whitelists.py",),
          "check",
          why="生成器只从 11 提取白名单集合, 不渲染整份文件; 它的 --check "
              "做的是集合语义比对. 见本文件头注."),
)

#: 磁盘上带生成头注, 但[有意不登记]的文件, 每条附理由.
#: 这份清单是元测试双向差集的唯一豁免口, 所以它必须短且每条说得出为什么.
UNREGISTERED_BY_DESIGN = {
    # 构建戳: 内容含 git describe 与构建时刻, [按定义]不可重现 -- 重跑一次
    # 就与入库副本不同, 把它放进登记表等于给自己造一条永远红的断言, 而永远
    # 红的断言最终会被改成永远绿(CLAUDE.md 3.2 形态2). 它的正确守法是发版
    # 门(CHK-0-54, RELEASE_ONLY), 不是这里.
    "xbrain/common/version/_build.py":
        "build stamp, not reproducible; guarded by the release gate instead",
}


def _load_generator(rel_path):
    """按路径 import 一个生成器脚本 -- 它们是 scripts, 不是包."""
    spec = importlib.util.spec_from_file_location(
        "gen_" + os.path.basename(rel_path)[:-3], os.path.join(ROOT, rel_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _diff_lines(want, have, path):
    """入库副本与新渲染之间的差异行, 供人直接读.

    只打差异不打全文: 生成的头文件有几百行, 全文 diff 会把真正变了的那一两行
    埋掉, 而读的人正是为了那一两行才来看的.
    """
    return list(difflib.unified_diff(
        have.splitlines(keepends=True), want.splitlines(keepends=True),
        fromfile=path + " (committed)", tofile=path + " (fresh render)",
        n=2))


def check_entry(entry):
    """比对一条登记项, 返回失败说明列表(空 = 通过)."""
    if entry.mode == "render":
        return _check_render(entry)
    return _check_subprocess(entry)


def _check_render(entry):
    """逐字节比对: 重跑 render(), 与入库副本比."""
    failures = []
    module = _load_generator(entry.generator)
    if not hasattr(module, "render"):
        # 登记成 render 模式却没有 render() -- 登记表写错了, 报出来而不是
        # 静默跳过. 跳过会让这一条从此不受任何检查.
        return ["%s: 登记为 render 模式但没有 render()" % entry.generator]
    want = module.render()
    for rel in entry.outputs:
        path = os.path.join(ROOT, rel)
        if not os.path.isfile(path):
            failures.append("MISSING %s -- run %s --write"
                            % (rel, entry.generator))
            continue
        with open(path, encoding="utf-8") as handle:
            have = handle.read()
        if have != want:
            failures.append("DRIFT   %s differs from a fresh render:\n%s"
                            % (rel, "".join(_diff_lines(want, have, rel))))
    return failures


def _check_subprocess(entry):
    """跑生成器自己的 --check, 转印它的输出."""
    proc = subprocess.run(
        [sys.executable, os.path.join(ROOT, entry.generator), "--check"],
        capture_output=True, text=True, cwd=ROOT)
    if proc.returncode == 0:
        return []
    # 把子进程说的话原样带出来: 它比本门更清楚差在哪里.
    return ["DRIFT   %s --check exited %d:\n%s%s"
            % (entry.generator, proc.returncode, proc.stdout, proc.stderr)]


def registered_outputs():
    """登记表里出现过的全部生成物路径."""
    out = set()
    for entry in REGISTRY:
        out.update(entry.outputs)
    return out


def main():
    if "--list" in sys.argv:
        for entry in REGISTRY:
            print("%-40s %-8s %s" % (entry.generator, entry.mode,
                                     ", ".join(entry.outputs)))
        return 0
    failures = []
    for entry in REGISTRY:
        failures.extend(check_entry(entry))
    for line in failures:
        print("  " + line)
    # 判据句写在扫描面之外的位置(本行在 print 里, 不含它自己要比对的内容),
    # 避免 CLAUDE.md 3.2 形态3 的判据自伤.
    print("criterion: every registered artifact equals a fresh run of its generator")
    print("registry entries: %d" % len(REGISTRY))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
