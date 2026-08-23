#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: cxx_discipline_audit.py
Brief: CHK-1-50 -- table-driven static audit of the numbered C++ discipline rules

Description:
13 与 11 给 C++ 侧定了一批纪律规则(CRL-* / DDS-* / RT-C* / CPP-* / PB-5), 每条
都写着"必须 / 不得", 而**没有一条有执行体**. 它们靠代码审查, 也就是靠人记得.

这类规则的失效方式很一致: 不是有人故意违反, 是有人不知道有这条. 比如 DDS-3
禁用 CYCLONEDDS_URI 的理由是"环境变量是进程级的, 等于把两域配置绑死" -- 一个
不知道双域架构的人加一行 setenv 会觉得自己在做好事, 而代码跑起来一切正常,
直到域 42 的配置被域 0 的那份覆盖.

*** 三个设计约束, 每个都对应一种本项目实测过的失效:

(1) 表驱动, NO 不写成散落的 if.
    散落的 if 没法回答"现在一共守着几条""某条还活着吗". 表让规则集合成为
    一个可以被元测试检查的对象(见 tests/ci/test_cxx_discipline.py).

(2) *** 扫描面为空必须[明说], NO 不许静默通过.
    这是本门最要紧的一条. 今天 ros2_ws/ 下只有 sensor(rtk_driver)一个包 --
    quadruped / chassis_relay / perception 的源码都还没建. 也就是说本表里多数
    规则的扫描目录[根本不存在]. 一个遍历空目录后 return 0 的门, 输出与"全部
    通过"一模一样, 那正是 CLAUDE.md 3.2 形态1: 一个什么都没检查的实现照样报绿.
    => 目录不存在或零文件的规则一律打印 NO-TARGET, 并计入摘要行.

(3) *** 本脚本必须在全部扫描面之外(判据元测试 c).
    本文件正文里就写着 CYCLONEDDS_URI / system_clock 这些要被 grep 的字串.
    如果它落在扫描面内, 每条规则都会命中它自己 -> 恒红 -> 被人放宽成"包含
    即可" -> 恒绿. CLAUDE.md 3.2 形态3 判据自伤的标准剧本. 脚本在 scripts/ci/,
    扫描面只有 ros2_ws/ 与 common/, 天然在外; 但"天然"会变, 所以由
    test_the_auditor_is_outside_every_scan_surface 钉住.

*** 13 条编号规则里只有 9 条静态查得了.
另外 4 条逐条写在 NOT_STATICALLY_CHECKABLE 里并说明为什么 -- NO 不许为了让
数字好看而硬凑一个弱代理. 一条"看起来在查 CRL-6"但实际什么都拦不住的规则,
比明说查不了更糟: 它让人以为有人在守.

Boundaries: 只做静态文本判定. 不编译, 不跑, 不判断运行期指标(CRL-5 的
"单跳 < 200 us"这类一律不在本门). 单调钟规则(CLAUDE.md 3.4)已由
scripts/ci/static_rules.py 覆盖, 本门不重复.

  python3 scripts/ci/cxx_discipline_audit.py           # 审计, 非零即违规
  python3 scripts/ci/cxx_discipline_audit.py --rules   # 打印规则表与扫描面
"""

import os
import re
import sys

#: 仓库根. 本脚本在 scripts/ci/ 下.
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

#: C++ 源码后缀. 大多数规则四种都扫; CMake 类规则单独写自己的后缀.
CXX = (".cc", ".cpp", ".h", ".hpp")


class Rule:
    """一条规则: 规则号 -> 扫描面 -> 命中模式 -> 处置.

    sense 有两个值, 且必须显式写:
      "forbid"  命中即违规(绝大多数规则是这种);
      "require" 扫描面里[至少一个文件]必须命中, 否则违规(DDS-9 这类
                "必须打印域号"的正向要求).
    默认成其中一种会让新规则在作者没想清楚方向时选错, 而选错的表现是
    这条规则从此永远绿.
    """

    def __init__(self, rid, doc, scan, pattern, sense, why,
                 suffixes=CXX, allow=(), also=()):
        assert sense in ("forbid", "require"), sense
        self.rid = rid              # 规则号, 与 11/13 逐字一致
        self.doc = doc              # 出处: 册号 + 节号(NUM-4: 不写行号)
        self.scan = tuple(scan)     # 扫描面, 仓库相对目录
        self.pattern = re.compile(pattern)
        self.sense = sense
        self.why = why              # 违反了会怎样 -- 给读到报告的人看
        self.suffixes = tuple(suffixes)
        # allow: 命中后仍放行的行内标记. 唯一用途是文档已登记的例外
        # (DDS-4 的 /CHARGE), 每条都要能指回文档.
        self.allow = tuple(allow)
        # also: 本条同时覆盖的其它规则号.
        #
        # *** 为什么要有这个字段, 而不是复制一条规则.
        # 有些规则是同一条约束的两侧表述: DDS-4"域 0 侧只创建 reader"与
        # RT-C5"CHS-B/C 只读, 不得写入"说的是同一件事, 一个从实现侧写, 一个
        # 从契约侧写. 复制成两行会让同一个违规被报两次(读的人以为有两处问题),
        # 而只留一行又会让元测试的双向差集报"RT-C5 漏挂了".
        # 显式声明覆盖关系是唯一不撒谎的写法.
        self.also = tuple(also)


#: 规则表. 规则号取自 11 / 13, NO 不自造 -- 元测试会拿它与 TODO 判据列里
#: 出现的规则号集合求双向差集.
RULES = (
    Rule("CRL-3", "13 CRL-3",
         ("ros2_ws/chassis_relay",),
         r"\b(yaml-cpp|YAML::|std::getenv|getenv\s*\(|nlohmann::json::parse)",
         "forbid",
         "白名单必须硬编码于代码. 一个可配置的白名单等于可被改成通用桥 -- "
         "而 chassis_relay 在急停链路上, 通用桥意味着任意消息可以走急停路径"),
    Rule("DDS-3", "13 DDS-3",
         ("ros2_ws/quadruped",),
         r"CYCLONEDDS_URI",
         "forbid",
         "环境变量是进程级的, 用它传配置等于把域 0 与域 42 的配置绑死. "
         "改用 dds_create_domain(0, config_xml) 在进程内注入"),
    Rule("DDS-4", "13 DDS-4 / 11 RT-C5",
         ("ros2_ws/quadruped",),
         r"dds_create_writer",
         "forbid",
         "域 0 侧只创建 reader. 向厂商域写入 = 违反冻结的厂商契约; "
         "唯一登记例外是 /CHARGE(软急停中止对接), 见 11 S7.1.3",
         allow=("DDS-4-ALLOW(/CHARGE)",),
         # RT-C5 是同一条约束的契约侧表述, 由本条一并覆盖.
         also=("RT-C5",)),
    Rule("DDS-9", "13 DDS-9",
         ("ros2_ws/quadruped",),
         r"hello_ack.*transport|runtime\.transport",
         "require",
         "启动时打印三个实体的实际域号与端点, 是双域配错的唯一低成本自证手段. "
         "配错的现象是 participant 起来了一个包收不到, 与网络不通不可区分"),
    Rule("RT-C4", "11 S1.1.5 RT-C4",
         ("ros2_ws/quadruped",),
         r"7447|zenohd-gen|plane\s*=\s*\"?gen",
         "forbid",
         "quadruped 不持通用面 session. 通用面绑 0.0.0.0 且无鉴权, 营区网内"
         "任一主机都能发布 -- 唯一能让腿动起来的进程必须在网络上不可达"),
    Rule("CPP-2", "13 CPP-2",
         ("ros2_ws/quadruped",),
         r"noexcept",
         "require",
         "ctrl / chs_b 线程入口函数必须 noexcept. 一次未捕获异常 = 进程死 = "
         "轴指令停发; 虽然安全但不必要地停机"),
    Rule("CPP-4", "13 CPP-4",
         ("ros2_ws/quadruped",),
         r"std::atomic_flag",
         "require",
         "tx_guard 必须是 atomic_flag(ATOMIC_FLAG_INIT). 用 mutex 会让实时侧"
         "阻塞在锁上, 那正是这条约定要避免的"),
    Rule("PB-5a", "13 PB-5",
         ("ros2_ws", "common"),
         r"__GNUC__\s*>|ROS_DISTRO|defined\s*\(\s*(HUMBLE|JAZZY)",
         "forbid",
         "NO 不写任何发行版判断宏. 平台基线 D-45(humble/22.04 vs Jazzy/24.04)"
         "尚未拍板, 写了判断宏, 裁决一下来就要全推倒",
         suffixes=CXX + (".txt", ".cmake")),
    Rule("PB-5b", "13 PB-5 / CPP-1",
         ("ros2_ws", "common"),
         r"CMAKE_CXX_STANDARD\s+(?!17\b)\d+",
         "forbid",
         "C++ 版本恰为 17. 20/23 的 concepts / coroutines / <format> 一旦用上, "
         "平台基线裁决时无法回退",
         suffixes=(".txt", ".cmake")),
)

#: 编号规则里[静态查不了]的, 逐条写明为什么. 这份清单是本门唯一的口子,
#: 由元测试钉住边界(新增一条即红).
#:
#: *** 为什么不硬凑一个弱代理.
#: 一条"看起来在查 CRL-6"但实际拦不住任何东西的规则, 比明说查不了更糟 --
#: 它让读报告的人以为有人在守. CLAUDE.md 3.2 形态1 的另一种长相.
NOT_STATICALLY_CHECKABLE = {
    "CRL-1":
        "只做搬运不做业务判断: 判定的是[意图]而不是某个符号的有无. 任何 grep "
        "代理(clamp/min/max)都会把合法的类型转换一并报进来, 变成噪声后被关掉. "
        "归代码审查, 见 13 CRL-1.",
    "CRL-6":
        "CR-11/12 与 CR-1~3 必须同线程且该线程不承载 Q3 流量: 判定的是线程亲和"
        "与流量分配, 需要跟踪 handler 注册到哪个 executor. 静态文本判不了.",
    "DDS-5":
        "域 0 与域 42 不共享消息对象(值拷贝): 需要跨函数的类型流分析才能判断"
        "一个 IMU 对象有没有跨域直传. grep 只能看到类型名, 看不到它流去哪儿.",
    "RT-C3":
        "禁跨面通用转发, 可持双 session 但须同时满足 a-e 五项: 五项里含"
        "'转发的 key 必须在白名单内'这类语义条件, 且白名单在 11 S1.1.6. "
        "已由 scripts/doccheck/whitelist_gen.py 从另一侧守着.",
}


def code_lines(text, is_cmake):
    """逐行返回"去掉注释后的代码部分". 行号从 1 开始.

    *** 这不是洁癖, 是本门第一次运行就踩到的.
    PB-5a 的模式立刻命中了 common/CMakeLists.txt 里的一行注释:
    "no if(ROS_DISTRO ...) and no CMAKE_SYSTEM_VERSION test below." --
    那句话[正是在说这里遵守了规则]. 一条把"我没违反"读成违反的规则, 是
    CLAUDE.md 3.2 形态3 判据自伤的一个变体: 描述规则的文字自己撞进扫描面.

    危害不止于噪声: 本项目要求每个文件写详细头注(2.5), 而头注里最该写的
    就是"本文件不引 rclcpp""不用 CYCLONEDDS_URI"这类边界说明. 不剥注释的话,
    越是注释写得好的文件越会被报违规 -- 门会先惩罚守规矩的人, 然后被关掉.

    块注释用状态机剥, 因为本项目的头注全是 C 风格块注释, 而头注正是边界
    说明最集中的地方.

    ! 已知剩余面: 字符串字面量里的 "//" 会被当成注释起点. 真要命中一条规则,
    需要"某行有个含 // 的字符串, 且同一行后半段有违规写法" -- 这种行本身
    就该被 review 打回. 记在这里而不是假装没有.
    """
    out = []
    in_block = False
    for raw in text.split("\n"):
        line = raw
        if is_cmake:
            # CMake 只有行注释, 且 # 之后到行尾全是注释.
            out.append(line.split("#", 1)[0])
            continue
        buf = []
        i = 0
        while i < len(line):
            if in_block:
                end = line.find("*/", i)
                if end < 0:
                    i = len(line)
                else:
                    in_block = False
                    i = end + 2
                continue
            if line.startswith("//", i):
                break                       # 行注释, 后面全丢
            if line.startswith("/*", i):
                in_block = True
                i += 2
                continue
            buf.append(line[i])
            i += 1
        out.append("".join(buf))
    return out


def _files_for(rule):
    """规则的实际扫描对象. 目录不存在就返回空 -- 由调用方报 NO-TARGET."""
    out = []
    for rel in rule.scan:
        base = os.path.join(ROOT, rel)
        if not os.path.isdir(base):
            continue
        for dirpath, _dirnames, filenames in os.walk(base):
            for name in sorted(filenames):
                if name.endswith(rule.suffixes):
                    out.append(os.path.join(dirpath, name))
    return sorted(out)


def audit_rule(rule, files=None):
    """审一条规则. 返回 (状态, 明细行列表).

    状态三值, 三者必须分开报:
      "OK"        查过了, 没问题;
      "VIOLATION" 查过了, 有问题;
      "NO-TARGET" 没得查(目录不存在或零文件).
    把第三种混进第一种, 就是"什么都没检查却报通过".
    """
    if files is None:
        files = _files_for(rule)
    if not files:
        return "NO-TARGET", []
    hits = []
    for path in files:
        with open(path, encoding="utf-8", errors="replace") as handle:
            raw = handle.read()
        is_cmake = path.endswith((".txt", ".cmake"))
        stripped = code_lines(raw, is_cmake)
        raw_lines = raw.split("\n")
        for lineno, line in enumerate(stripped, 1):
            if not rule.pattern.search(line):
                continue
            # 文档已登记的例外: 标记写在[原始行]上(它在注释里), 所以查
            # raw 而不是剥过的 -- 剥完就找不到标记了.
            if any(tag in raw_lines[lineno - 1] for tag in rule.allow):
                continue
            hits.append((os.path.relpath(path, ROOT), lineno,
                         raw_lines[lineno - 1].strip()[:70]))
    if rule.sense == "forbid":
        if hits:
            return "VIOLATION", hits
        return "OK", []
    # require: 扫描面里至少要有一处命中.
    if hits:
        return "OK", []
    return "VIOLATION", [("<scan surface>", 0,
                          "no file in %s matches the required pattern"
                          % ", ".join(rule.scan))]


def main():
    if "--rules" in sys.argv:
        for rule in RULES:
            print("%-8s %-22s %-10s %s"
                  % (rule.rid, ",".join(rule.scan), rule.sense, rule.doc))
        for rid, why in sorted(NOT_STATICALLY_CHECKABLE.items()):
            print("%-8s %-22s %-10s %s" % (rid, "-", "not-checkable", why[:60]))
        return 0
    # *** 扫描面声明. 判据逐字要求打印"扫了哪些包, 哪些后缀", NO 不得只写
    # "全仓" -- 一个不声明扫描面的结论, 读的人无法判断它覆盖了什么
    # (CLAUDE.md 3.2 形态6).
    packages = sorted({rel for rule in RULES for rel in rule.scan})
    suffixes = sorted({sfx for rule in RULES for sfx in rule.suffixes})
    print("scan surface: packages=%s suffixes=%s"
          % (",".join(packages), ",".join(suffixes)))
    failures, no_target = 0, []
    for rule in RULES:
        status, hits = audit_rule(rule)
        if status == "NO-TARGET":
            no_target.append(rule.rid)
            continue
        if status == "VIOLATION":
            failures += 1
            for path, lineno, text in hits:
                print("  %-8s %s:%d  %s" % (rule.rid, path, lineno, text))
            print("  %-8s WHY: %s" % ("", rule.why))
    if no_target:
        # *** 打印而不是跳过. 这些规则今天一条也没在守, 而那不是"通过".
        print("  NO-TARGET (source not built yet, nothing scanned): %s"
              % ", ".join(no_target))
    print("criterion: every rule with a scan target reports zero violations")
    print("rules: %d checkable, %d not-statically-checkable, "
          "%d without a scan target, %d violated"
          % (len(RULES), len(NOT_STATICALLY_CHECKABLE), len(no_target),
             failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
