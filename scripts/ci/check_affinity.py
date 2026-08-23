#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: check_affinity.py
Brief: CHK-1-03 -- systemd CPUAffinity must match the 10 S3.2 core table

Description:
10 S3.2 给了 Orin NX 八个核的逐进程分配, 并且这个分配不是建议: 核 4 由
P1 独占(isolcpus 隔离)是 20 Hz 周期 P99 的前提, 核 6 集中四个 RT/急停类
C++ 进程是因为它们无 GIL 无 GC, 核 7 明确标着"已过载"是有意接受的代价.

表在文档里, 生效在 systemd 单元里. 两处各写一遍, 谁也不通知谁 -- 而分叉的
表现是[没有表现]: 进程照常启动, 周期照常跑, 只是抖动变大, 要到 T7 时延实测
才会被发现, 那时已经很难倒推回一个单元文件里的数字.

*** 判据逐字要求"从表体现场解析, NO 不硬编码", 这条不是洁癖.
硬编码一份副本的门会在表体被改动后[继续报绿]-- 它守的是自己那份副本, 不是
设计. 这正是 CLAUDE.md 3.2 形态1: 一个什么都没验证的实现照样通过.
所以本门每次运行都重新解析 10 S3.2 的表格, 表变了门就跟着变.

*** 判据 (2)(3)(4) 需要真机, 本门只做 (1).
  (2) taskset -pc 读实际掩码  -- 要进程在跑;
  (3) chrt -p 读 FIFO 优先级  -- 要 quadruped 在跑(且它是 C++ 侧未建的);
  (4) 内核 cmdline 的 isolcpus -- 要目标机的 /proc/cmdline.
三条都归 needs_orin 档, 见 tests/deploy/test_affinity_gate.py 里的说明.

Boundaries: 只比对静态声明. 不改单元文件, 不设置任何亲和.

  python3 scripts/ci/check_affinity.py           # 双向差集, 非零即不一致
  python3 scripts/ci/check_affinity.py --table   # 打印解析出的表
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
DOC = os.path.join(ROOT, "docs", "10-顶层设计.md")
UNIT_DIR = os.path.join(ROOT, "deploy", "systemd")

#: 表格所在小节的标题逐字. 用锚点定位而不是行号 -- CLAUDE.md NUM-4: 文档
#: 持续增删, 行号必然漂移, 而漂移后的引用仍然"看起来核对过了".
#:
#: *** 锚点必须唯一, 这是实测出来的.
#: 第一版写的是 "CPU 亲和"(四个字), 而 10 里在别处也提到这四个字(S3.1 的
#: 进程清单表里 P1 那一行就有 "CPU 亲和"). find() 命中的是那一处, 于是
#: 解析器从错的位置往下扫, 吃进了 VACUUM / task.db / h_factor 这些完全
#: 不相干的条目 -- 而且[没有抛], 它给出了六个看起来像模像样的映射.
#: 一个解析到错表却照常出结果的门, 比一个报错的门危险得多.
#: *** 全角括号用码位写, NO 不写字面量.
#: 这个串是去[匹配 markdown 正文]的, 里面那对全角括号就是文档里的那两个字符 --
#: 换成 ASCII 括号锚点就对不上, 门会当场 SystemExit. 但 CLAUDE.md 2.2 又要求
#: 源文件里零全角标点. 两个约束的交集就是码位转义: 运行期是同一个串, 源文件里
#: 没有那个字符. 本仓 tests/meta/test_progress_parser.py 的记号常量同一写法.
_TABLE_ANCHOR = "**CPU 亲和\uff08Orin NX 8 核\uff09**"

#: 核号列的写法: "0" / "2-3"(文档里用的是 en dash) / "**4**"(带强调).
#: 三种都要认 -- 只认其中一种会静默少解析几行, 而少解析的表现是差集变小,
#: 也就是门变松.
#: 破折号同样用码位: 文档里 "2-3" 用的是 en dash, 而 ASCII 连字符与 em dash
#: 都出现过. 三种都认, 只认一种会静默少解析几行 -- 而少解析的表现是差集变小,
#: 也就是门变松, 不是报错.
_CORE_CELL = re.compile(
    "^\\s*\\**\\s*(\\d+)\\s*(?:[-\\u2013\\u2014]\\s*(\\d+))?\\s*\\**\\s*$")

#: 分配列里的进程名: 反引号包起来的那些. 文档在同一格里还有 "(P5)" 这样的
#: 标注与中文说明, 反引号是唯一稳定的边界.
_PROC = re.compile(r"`([A-Za-z][A-Za-z0-9_.-]*)`")

#: 同上, 但把紧跟在反引号后的 "(P3)" 一并捕获. 表体逐字写 `xbrain_task`(P3),
#: 那个 P3 是把文档里的进程名对到 systemd 单元名的唯一线索 -- 没有它就只能
#: 手维护一张对照表, 而对照表是第三个会漂移的副本.
_PROC_WITH_INDEX = re.compile(
    r"`([A-Za-z][A-Za-z0-9_.-]*)`\s*(?:\((P[1-5])\))?")


def parse_core_table(text):
    """从 10 S3.2 解析 {进程名: 核集合}. 解析不到表就抛, NO 不返回空.

    返回空字典会让下游的双向差集变成"两边都空, 完全一致", 于是一个找不到
    表格的门报绿 -- 又一次形态1.
    """
    hits = text.count(_TABLE_ANCHOR)
    if hits != 1:
        # 零命中 = 标题被改了; 多命中 = 锚点不再唯一, 下面的 find() 会
        # 随机取第一处. 两种都必须停下, NO 不许"取第一个先跑着".
        raise SystemExit("10 S3.2 锚点 %r 命中 %d 次, 应恰为 1"
                         % (_TABLE_ANCHOR, hits))
    idx = text.find(_TABLE_ANCHOR)
    mapping = {}
    started = False
    for line in text[idx:].splitlines():
        if line.startswith("|"):
            started = True
        elif started:
            # 表结束就停 -- 这一节后面还有别的表(内存预算等), 继续扫会把
            # 它们的行也当成核分配读进来.
            break
        else:
            continue
        cells = line.split("|")
        if len(cells) < 3:
            continue
        m = _CORE_CELL.match(cells[1])
        if not m:
            continue                       # 表头 / 分隔行
        lo = int(m.group(1))
        hi = int(m.group(2)) if m.group(2) else lo
        cores = frozenset(range(lo, hi + 1))
        # 连 (Pn) 标注一起抓: 它是推导单元名的唯一依据(见 unit_name_for).
        for proc, p_index in _PROC_WITH_INDEX.findall(cells[2]):
            if proc in _NOT_A_PROCESS:
                continue
            mapping[proc] = (cores, p_index)
    if not mapping:
        raise SystemExit("解析到 0 个进程 -- 表格结构变了, 门已失效")
    return mapping


#: 表体里被反引号包起来, 但[不是进程]的词. 每条附理由 -- 与本仓其它豁免
#: 清单同一个规矩: 豁免必须逐条列出且说得出为什么, 否则半年后没人分得清
#: 哪些是有意排除, 哪些是漏了.
_NOT_A_PROCESS = {
    "isolcpus": "kernel cmdline parameter, not a process (10 S3.2 core-4 row)",
}

#: 进程名 -> 单元名的两条例外, 各附理由. 其余全部按下面的规则推导, NO 不
#: 维护一张全量映射表 -- 那张表会与两侧同时漂移, 而漂移不会有任何报错.
_UNIT_EXCEPTIONS = {
    # 表体按二进制名写(llama-server), 单元按角色名(llm). 这是有意的:
    # 换一个推理后端时单元名不该跟着变.
    "llama-server": "xbrain-llm.service",
    # 表体是 zenoh 官方桥的全名, 单元名截短. 全名里的 ros2dds 是桥的类型,
    # 不是我方进程的一部分.
    "zenoh-bridge-ros2dds": "xbrain-zenoh-bridge.service",
}


#: 绑了核, 但 10 S3.2 表体里没有登记的单元. 每条必须附[理由]与[去哪儿
#: 看后续], 且由 tests/deploy/test_affinity_gate.py 钉住集合边界 --
#: 新增一条会让元测试红, 免得这个口子被用来让门变绿.
#:
#: *** 这不是"豁免", 是"等一个裁决".
#: 与本仓 progress.py 的 NON_FAILURE / gen_drift_gate.py 的
#: UNREGISTERED_BY_DESIGN 同一个规矩: 每多一个不算失败的格子, 就多一个
#: 藏真失败的地方, 所以要小, 要逐条, 要说得出为什么.
PENDING_DOC_DECISION = {
    "xbrain-payload.service":
        "payload-service is one of the 15 resident processes (CLAUDE.md S0.1) "
        "but 10 S3.2 does not list it on any core. The unit already pins core 7; "
        "whether that is right is a design call, not something a CI gate may "
        "decide. See NEXT SW-20.",
}


def unit_name_for(proc, p_index):
    """由进程名(与它在表体里的 (Pn) 标注)推导单元文件名.

    NO 不写死一张全量对照表: 表在文档里, 单元在磁盘上, 中间再放一张手写
    表就有了第三个会漂移的副本. 规则只有两条:
      * 带 (Pn) 标注的 -> xbrain-pn-<下划线后半段>.service
        (表体逐字 `xbrain_motion`(P1) -> xbrain-p1-motion.service)
      * 其余 -> xbrain-<下划线换横杠>.service
    对不上的两个走 _UNIT_EXCEPTIONS, 各写了理由.
    """
    if proc in _UNIT_EXCEPTIONS:
        return _UNIT_EXCEPTIONS[proc]
    if p_index:
        # xbrain_motion + P1 -> xbrain-p1-motion
        tail = proc.split("_", 1)[1] if "_" in proc else proc
        return "xbrain-%s-%s.service" % (p_index.lower(), tail.replace("_", "-"))
    return "xbrain-%s.service" % proc.replace("_", "-")


def parse_units():
    """{单元名: 核集合} -- 只收真的写了 CPUAffinity 的单元."""
    out = {}
    for name in sorted(os.listdir(UNIT_DIR)):
        if not name.endswith(".service"):
            continue
        with open(os.path.join(UNIT_DIR, name), encoding="utf-8") as handle:
            for line in handle:
                if line.startswith("CPUAffinity="):
                    spec = line.split("=", 1)[1].strip()
                    cores = set()
                    for part in spec.replace(",", " ").split():
                        if "-" in part:
                            lo, hi = part.split("-", 1)
                            cores.update(range(int(lo), int(hi) + 1))
                        else:
                            cores.add(int(part))
                    out[name] = frozenset(cores)
    return out


def main():
    # --doc 覆盖 10 的路径. 存在的唯一理由是让"这个门读的是表体"可被证伪:
    # 测试拿一份改过核号的文档副本喂进来, 门的结论必须跟着变. 一个硬编码
    # 副本的门在这种输入下[结论不变], 那正是判据变异体2 要打的地方.
    doc_path = DOC
    if "--doc" in sys.argv:
        doc_path = sys.argv[sys.argv.index("--doc") + 1]
    with open(doc_path, encoding="utf-8") as handle:
        table = parse_core_table(handle.read())
    units = parse_units()
    if "--table" in sys.argv:
        for proc in sorted(table):
            cores, p_index = table[proc]
            print("%-24s %-28s %s" % (proc, unit_name_for(proc, p_index),
                                      sorted(cores)))
        return 0
    failures = []
    # 方向一: 表体说该绑, 单元里没绑或绑错.
    for proc in sorted(table):
        cores, p_index = table[proc]
        unit = unit_name_for(proc, p_index)
        if not os.path.isfile(os.path.join(UNIT_DIR, unit)):
            # 单元文件根本不存在 -- 与"存在但没写 CPUAffinity"分开报: 前者
            # 是进程还没建, 后者是建了但漏了绑核, 两种处置完全不同.
            failures.append("NO-UNIT   %-24s 表体要求核 %s, 但没有 %s"
                            % (proc, sorted(cores), unit))
        elif unit not in units:
            failures.append("UNBOUND   %-24s 表体要求核 %s, %s 里没有 CPUAffinity"
                            % (proc, sorted(cores), unit))
        elif units[unit] != cores:
            failures.append("MISMATCH  %-24s 表体核 %s, %s 里是 %s"
                            % (proc, sorted(cores), unit, sorted(units[unit])))
    # 方向二: 单元里绑了, 表体没登记. 这一向同样重要 -- 一个不在预算表里的
    # 绑核会悄悄挤占别人的核, 而它在文档上是不存在的.
    claimed = {unit_name_for(p, table[p][1]) for p in table}
    for unit in sorted(set(units) - claimed - set(PENDING_DOC_DECISION)):
        failures.append("UNLISTED  %-24s 绑了核 %s, 但 10 S3.2 表体里没有它"
                        % (unit, sorted(units[unit])))
    # 待裁决的照样打出来, 只是不计入失败 -- 一个不打印就放过的口子, 三个月后
    # 没人记得它存在.
    for unit in sorted(set(units) & set(PENDING_DOC_DECISION)):
        print("  PENDING   %-24s 绑了核 %s, 等文档裁决(见 NEXT SW-20)"
              % (unit, sorted(units[unit])))
    for line in failures:
        print("  " + line)
    print("criterion: doc table and unit CPUAffinity agree in both directions")
    print("parsed processes: %d, units with CPUAffinity: %d, failures: %d"
          % (len(table), len(units), len(failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
