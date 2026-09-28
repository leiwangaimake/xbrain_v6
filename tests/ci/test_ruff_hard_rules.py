"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_ruff_hard_rules.py
Brief: 把 ruff 里[不是风格问题而是缺陷]的那几条锁成 0, 并守住这个规则集本身

Description:
这个门解决的问题: pyproject 的 [tool.ruff] 选了 E/F/I/B/W 五大类, 而全仓现在
有四位数的命中, 于是 scripts/run_ci_gates.sh 的 ruff 档[一直是红的]. 一个
长期红的门, 与没有门是同一件事 -- 它不会在某条[新的]undefined-name 混进来
的那一刻变红, 因为它昨天也是红的 (CLAUDE.md 3.2 形态二: 永远红的断言最终会被
改成永远绿的).

所以本文件不看那四位数, 只挑出[运行到就出事 / 已经在骗人]的六条, 把它们锁
成 0. 其余的是声明过的存量债, 由 run_ci_gates.sh 报总数, 不在这里判定.

六条各自是什么缺陷 (逐条见 HARD_RULES 的 why 列):
  F821 / F811 / B023  -- 运行期真错误或必然的行为错误;
  B017 / B011 / B905  -- 三种[一条永远绿的断言](CLAUDE.md 3.2 形态一):
      pytest.raises(Exception) 任何异常都满足;
      assert False 在 assertion rewriting 关掉且 -O 时整条消失;
      zip() 默认在短的那侧截断, 多出来的那些从未被检查而测试照样绿.

Boundaries:
  * 不判断规则内容对不对 -- 那是 ruff 的事. 本文件只保证[这六条在全仓为 0]
    且[这个集合不能被悄悄缩小].
  * 不碰其余 ruff 规则. 新增一条就要先把全仓那条清零, 否则本门当场红 --
    这是有意的成本, 也是它唯一的价值所在.
  * 扫描面不是本文件声明的: 它是 pyproject.toml [tool.ruff].exclude
    (ros2_ws/perception / common/third_party / data / docs/temp / ...).
    一个扫描面没声明的 lint 报出的数字没人能解释 (CLAUDE.md 3.2 形态六),
    所以这里显式复述一遍它的来源而不是另立一份.

*** ruff 缺失时[必须红, NO 不许 skip]. tests/requirements.txt 的 Category C
已经把 ruff 列为依赖. 一个"工具不在就跳过"的门, 在没装工具的机器上与全绿不可
区分, 正是 CLAUDE.md 3.2 形态一; run_cxx_tests.sh 对 ROS 缺失采取的也是同一
立场(SKIP 不许当 pass, 默认严格).
"""
from __future__ import annotations

import os
import pathlib
import shutil
import subprocess

import pytest

# no_device: 这是一个纯静态门, 不碰底盘 / 不碰 ORIN 的 GPU, 所以它必须留在
# 开发机的默认选择 (pytest -m no_device) 里. 一个只在真机上跑的 lint 门等于
# 没有门 -- 违规通常是在开发机上写出来的.
pytestmark = pytest.mark.no_device

# 仓库根. 从本文件往上两级 (tests/ci/ -> tests/ -> ROOT). 不硬编码
# /opt/xbrain_v6: 同一台机器上开两个 checkout 时, 硬编码会让这个门去扫
# 另一个仓库, 报出的数字与本次改动无关 (CLAUDE.md 6 对 shell 的同一条要求).
ROOT = pathlib.Path(__file__).resolve().parents[2]

# 与 scripts/run_ci_gates.sh 的分工, 免得下一个人以为二者重复:
#   * run_ci_gates.sh ruff  -- 跑[全部]选中的规则, 报总数. 它今天是红的,
#     而且在存量债清完之前会一直红, 所以它是一份[报告], 不是一道[门].
#   * 本文件               -- 只跑六条, 锁成 0. 它今天是绿的, 因此明天任何
#     一条新的 F821/B017/... 都能让它当场红. 这才是门的定义.
# 两者都需要: 前者防止债被忘掉, 后者防止债继续长.

#: 规则号 -> 留它在这里的理由.
#:
#: 理由不是装饰: 它是下一个想删掉某一行的人必须先反驳的东西. 一条规则被从
#: 这张表里拿掉, 意味着那一类缺陷从此可以静默进仓; 所以每一行都写清[它具体
#: 让什么东西坏掉], 而不是复述 ruff 官方的一句话描述.
#:
#: 六条的挑选口径只有一条: [运行到就出事] 或 [已经在骗人]. 凡是"读起来不好看"
#: 的都不在这里 -- 那些是风格债, 由 run_ci_gates.sh 报总数.
#:
#: 表为空 / 少掉四条骨干之一 / 理由写成复述, 下面的元测试当场红.
HARD_RULES = {
    # 运行到即崩. 危险的不是崩, 是[崩在一个 except Exception 里].
    # 2026-09-28 实测: p2_core 关机路径上写着 speak_sub.undeclare(), 而
    # speak_sub 这个名字在那个函数里从未存在过; NameError 是 Exception 的
    # 子类, 被同一段的 except 咽掉, 于是文档写明的 shutdown 第 2 步一次都
    # 没执行过, 九个 cmd/* 订阅一直活到会话关闭.
    "F821": "undefined name -- 运行到即 NameError. 本仓实测过一次: "
            "p2_core 关机路径上的 speak_sub.undeclare(), 被同段的 "
            "except Exception 咽掉, 于是九个订阅一次都没被撤销过.",
    # 后定义覆盖先定义. 在[测试文件]里这不是风格问题: 两个同名 def, 后一个
    # 把前一个从模块命名空间里挤掉, pytest 只会收集到后一个 -- 前一条判据
    # 从此一次都不运行, 而测试总数照样每天在涨.
    "F811": "redefined-while-unused -- 后定义覆盖先定义. 落在测试文件里时, "
            "被覆盖的那个同名测试从此一次都不会运行.",
    # 闭包按名字查外层作用域, 查的是[调用时]的值, 不是[定义时]的值. 循环里
    # 建 N 个回调再一起注册, 最终全部看到最后一圈的那个值; 同步立即调用时
    # 它恰好正确, 所以这类缺陷会潜伏到有人把调用改成异步的那一天.
    "B023": "闭包捕获循环变量 -- 循环里注册的 N 个回调最终全看到最后一次的值.",
    # CLAUDE.md 3.2 形态一的教科书样子. 一条 raises(Exception) 连"被测函数
    # 根本没跑到"都能满足 -- 签名改了抛 TypeError, 对象没了抛 AttributeError,
    # 名字拼错抛 NameError, 三者全绿. 收窄到具体类型(最好带 match=)之后,
    # 断言才第一次开始说一件可证伪的事.
    "B017": "pytest.raises(Exception) -- 任何异常都满足, 包括 AttributeError / "
            "TypeError / NameError. 被测代码换一条完全不同的错误路径, 断言照样绿.",
    # try: f(); assert False; except E: 这种写法里, assert False 是唯一区分
    # "拒了"和"没拒"的东西. 它一旦不执行, 控制流直接往下走, 非法输入全部被
    # 放行而测试照样绿. 触发条件见头注的实测: 单开 -O 不够(pytest 会重写
    # 测试模块里的 assert), 要 --assert=plain 或这段代码不被 pytest 收集.
    "B011": "assert False -- assertion rewriting 关掉(--assert=plain)且 -O 时"
            "整条消失, 于是'没抛异常'这件事不再有人报告.",
    # 本仓有大量"两个列表逐项对账"的判据. zip() 默认在短的那侧停, 于是长的
    # 那侧多出来的项从未被检查 -- 而循环体里的 assert 条条通过, 测试全绿.
    # 加 strict=True 之后长度本身成为判据的一部分; 确实要截断的(滑动配对)
    # 写 strict=False 并注明为什么, 让"截断"从默认行为变成一次显式选择.
    "B905": "zip() 无 strict -- 默认在短的那侧静默截断. 两个列表逐项对账的判据, "
            "长度不等时多出来的那些从未被检查而测试照样绿.",
}


def _ruff() -> str:
    """ruff 的绝对路径. 找不到就 fail, NO 不 skip (见头注最后一段).

    返回绝对路径而不是 "ruff" 字符串: subprocess 里用裸名字会再走一遍 PATH
    查找, 而下面的回退分支恰恰是为了处理 PATH 里没有的情况 -- 两者混用会让
    回退看起来生效了而实际仍然 FileNotFoundError.
    """
    # 正常路径: 装在 PATH 上(venv / 系统包 / pipx).
    exe = shutil.which("ruff")
    if exe:
        return exe
    # 回退一次: ORIN 上是 pip install --user, 落在 ~/.local/bin. 非交互式
    # ssh (bash -c) 不读 .profile, 所以那一段不在 PATH 里 -- 这不是"工具
    # 没装", 只是"这个 shell 看不到", 两者的处置完全不同, 所以显式找一次.
    fallback = pathlib.Path(os.path.expanduser("~/.local/bin/ruff"))
    if fallback.is_file() and os.access(fallback, os.X_OK):
        return str(fallback)
    # 到这里才是真的没装. fail 而不是 skip: 见头注最后一段.
    pytest.fail(
        "ruff not found on PATH or in ~/.local/bin. It is a declared test "
        "dependency (tests/requirements.txt Category C): "
        "pip install -r tests/requirements.txt. This gate fails rather than "
        "skips on purpose -- a skipped lint gate is indistinguishable from a "
        "green one (CLAUDE.md 3.2 form 1).")
    raise AssertionError("unreachable")        # pytest.fail 不返回; 为静态检查


def test_the_hard_rule_set_is_not_empty_and_each_row_says_why():
    """空表 / 无理由的行 = 这个门什么都不查, 而它照样 exit 0.

    与 tests/ci/test_toolchain_config.py 里 'ruff 的 select 为空' 那条同源:
    一个可以被清空的判据集合, 等于一条永远绿的断言.
    """
    # 第一层: 表不能是空的. 一个 select 为空的 lint 什么都不查而照样 exit 0.
    assert HARD_RULES, "HARD_RULES 为空 -- 本门什么都不查"
    for code, why in HARD_RULES.items():
        # 第二层: 键必须真的长得像一个 ruff 规则号 (字母 + 数字). 写错的
        # 规则号 ruff 会直接拒绝启动, 但那会表现成本门的另一种失败 --
        # 在这里先断言一次, 失败信息里才有那个写错的字符串本身.
        assert code[0].isalpha() and code[1:].isdigit(), code
        # 第三层: 理由必须写成人话, 而不是复述规则号. 30 字符是下限而不是
        # 目标: 一行"B017 is bad"照样能骗过长度检查, 所以还额外禁止理由里
        # 出现规则号本身 -- 复述的最省事写法正是把号码抄一遍.
        assert len(why) >= 30 and code not in why, \
            "%s 的理由太短或只是复述规则号: %r" % (code, why)
    # 第四层: 四条骨干必须在内. 这一层防的是"为了让门变绿而缩小规则集" --
    # 那是 CLAUDE.md 3.2 形态二的标准收场 (判据一直红 -> 被放宽 -> 永远绿).
    # 这四条是本批 (2026-09-28) 逐条查出真缺陷的那几类, 删掉任何一条都会让
    # 那一类判据洞重新可以静默进来.
    for must in ("B017", "B011", "B905", "F821"):
        assert must in HARD_RULES, "%s 被从 HARD_RULES 里删掉了" % must


def test_the_repository_has_zero_hard_rule_findings():
    """全仓这六条必须是 0.

    MUTATION (2026-09-28 实测): 在扫描面内任意 .py 里写一个未定义名字, 本条红,
    并在失败信息里打印文件:行:规则号.

    为什么调外部进程而不是 import ruff: ruff 是 Rust 二进制, 没有稳定的
    Python API; 而且这样跑出来的结果与开发者在命令行敲的那一条[逐字相同],
    失败信息可以直接复制去复现.
    """
    # --no-fix 是显式写的而不是靠默认: ruff 在某些配置下会自动修, 而一个
    # 边跑边改源码的"门"报出的 0 说明不了任何事.
    # cwd=ROOT 让 pyproject.toml 的 exclude 生效 -- 扫描面必须是声明过的
    # 那一个, 不是本进程恰好在哪个目录 (CLAUDE.md 3.2 形态六).
    out = subprocess.run(
        [_ruff(), "check", ".", "--no-fix",
         "--select", ",".join(sorted(HARD_RULES)),
         "--output-format=concise"],
        cwd=str(ROOT), capture_output=True, text=True)
    # concise 格式每条命中一行 "path:line:col: CODE message". 汇总行
    # ("Found N errors." / "All checks passed!") 不是命中, 滤掉.
    findings = [ln for ln in out.stdout.splitlines()
                if ":" in ln and not ln.startswith("Found ")
                and not ln.startswith("All checks")]
    # 两半都要: returncode 防"ruff 根本没跑起来"(坏配置 / 坏规则号, 它会
    # 非零退出且一条 finding 都不打印), findings 防"退出码 0 但有输出".
    # 只看其中一半, 另一半的失效模式就会读成全绿.
    #
    # 失败信息里打[条数 + 前 40 条原文]: 条数用于一眼看出量级, 原文用于
    # 直接定位. 截到 40 条是为了不让一次大规模回归把终端刷爆 -- 条数那一半
    # 仍然是完整的, 所以截断不会让人低估问题.
    assert out.returncode == 0 and not findings, (
        "ruff hard rules (%s) 命中 %d 条 -- 每一条都不是风格问题, 见 "
        "HARD_RULES 里对应的理由:\n%s"
        % (",".join(sorted(HARD_RULES)), len(findings),
           "\n".join(findings[:40])))
