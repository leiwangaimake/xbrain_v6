"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_wiring_teardown_covers_subs.py
Brief: 每个用局部名字接住的 declare_subscriber, 都必须出现在该模块的拆除路径里

Description:
这个判据守的缺陷, 2026-09-28 在三个接线模块里各抓到一次, 形状完全相同:

  * p2_core/runtime/main_wiring.py  finally 里写的是 speak_sub.undeclare(),
    而 speak_sub 这个名字从未存在过 (ruff F821). NameError 被同段的
    except Exception 咽掉, 于是九个 GEN 面订阅一次都没被撤销过.
  * p1_motion/runtime/main_wiring.py  teleop_sub 声明了, 四个同伴都在 finally
    里拆, 唯独它不在 (ruff F841).
  * p5_gateway/runtime/main_wiring.py  拆除元组列了 11 个实体, 而该函数里
    声明了 20 个订阅; 漏掉的九个同样被 F841 报成 "赋值了从不使用".

三处的共同点, 也是本文件存在的理由: [声明]与[拆除]是两张分别手写的名单,
它们之间没有任何东西负责对账. 漏掉一个的代价不是崩溃, 而是一个活过了
自己那批同伴的订阅 -- 它的回调在 server 已经 should_exit / 子系统已经
stop() / 域对象已经 shutdown() 之后继续被 Rust 线程调用. 这类缺陷不报错,
只是让关机窗口里到达的那一帧走进一个半拆掉的世界.

*** 为什么不用 "ruff F841 为 0" 代替本文件:
F841 只说 "这个名字没被读过". 一个把 teleop_sub 塞进随便哪个 print 或者
一句 `_ = teleop_sub` 的改动就能让它闭嘴, 而订阅照样没被拆. 本文件问的是
一个更具体的问题: 这个名字有没有出现在一条真的会调用 undeclare() 的路径上.

Boundaries:
  * 只管[用局部名字接住]的那一种写法 (X = sess.declare_subscriber(...)).
    self._subs.append(...) / SubscriberRegistry.declare(...) 这两种由容器
    自己负责, 不在本判据内 -- p3 / p4 / nav_wiring / cloud_wiring /
    three_keys / subscriber_registry 走的都是那两种.
  * 不判断拆除的[顺序]对不对(那是各模块 shutdown order 的事), 只判断
    [在不在].
  * 扫描面是算出来的, 不是抄下来的: 遍历 xbrain/ 下所有 .py, 谁出现
    declare_subscriber 谁就进. 一个写死的文件名单会在下一个接线模块诞生
    那天静默失效 (CLAUDE.md 3.2 形态六).
  * 与 CLAUDE.md 8.2 那条静态规则 (declare_subscriber 的返回值必须落到
    self.x / list.append / SubscriberRegistry.declare) 不重叠: 那条管的是
    [句柄有没有强引用, 也就是订阅活不活得下来]; 本条管的是 [活下来的那些
    有没有被拆]. 一个模块可以完全通过那条而在本条上红 -- p1 / p5 今天就是.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

# no_device: 纯 AST 判据, 不开会话不连底盘. 必须留在开发机的默认选择里 --
# 漏拆一个订阅是在写接线代码的那台机器上发生的, 不是在真机上.
pytestmark = pytest.mark.no_device

# 从本文件往上两级 (tests/p_processes/ -> tests/ -> ROOT). 不硬编码
# /opt/xbrain_v6: 一台机器上开两个 checkout 时, 硬编码会让判据去扫另一个
# 仓库, 得出的结论与本次改动无关.
ROOT = pathlib.Path(__file__).resolve().parents[2]

# 扫描面的根. 只扫 xbrain/ -- scripts/ 下的 SIL 与 stub 也开订阅, 但它们
# 是手工工具, 进程活到 Ctrl-C 为止, 没有 "关机窗口" 这个概念, 把它们拉进
# 来只会制造一批必须逐条解释的例外 (而例外表一长, 判据就开始被绕过).
XBRAIN = ROOT / "xbrain"


def _sources():
    """xbrain/ 下每个提到 declare_subscriber 的 .py, 连同它的源码文本.

    算出来而不是写死: 见头注 Boundaries 第三条. 具体的失效方式是这样的 --
    一份手写的文件名单在今天是对的, 而下一个接线模块诞生那天它仍然是
    "全绿", 因为新模块根本不在名单里. 扫描面必须跟着仓库长, 否则判据的
    覆盖率会随时间单调下降而没有任何信号.

    粗筛用字符串 in 而不是先 AST: 绝大多数文件一个字符都不含, 先做一次
    廉价的子串判断比对上千个文件建语法树快得多; 真正的判定在
    _declared_local_names 里用 AST 做, 所以注释里的假阳性不会漏进结论.

    返回 (Path, text) 而不是只返回 Path: 调用方两样都要 (一个给断言信息
    用, 一个给 ast.parse 用), 读两遍文件是白读一次磁盘.
    """
    out = []
    for p in sorted(XBRAIN.rglob("*.py")):
        text = p.read_text(encoding="utf-8")
        if "declare_subscriber" in text:
            out.append((p, text))
    return out


def _declared_local_names(tree):
    """返回 {X}: 形如 `X = <任意表达式>.declare_subscriber(...)` 的局部名字.

    收哪些, 不收哪些, 以及为什么:

      收    teleop_sub = gen.declare_subscriber(K, cb)
            -- 句柄只被一个局部名字拿着. 它的存活靠[函数栈帧还在],
               它的拆除靠[有人记得在 finally 里点它的名]. 这两件事都没有
               任何机制保证, 所以它们正是本文件要对账的那一类.

      不收  self._subs.append(sess.declare_subscriber(K, cb))
            -- 句柄进了对象持有的容器, 拆除是 "for s in self._subs" 一次
               写完的循环, 不存在[漏点某一个的名]这种失效方式.

      不收  self.sub = sess.declare_subscriber(K, cb)
            -- 同理, 生命周期跟着对象走, 由对象的 close()/stop() 负责.

    用 AST 而不是正则: 正则分不清 "x = s.declare_subscriber(...)" 与注释里
    的同一串字符, 而本仓的接线文件恰好在注释里反复引用 CLAUDE.md 4.3 的
    原句 "declare_subscriber(...) 的返回值必须接住" -- 用正则会把那些注释
    数成声明, 判据从第一天起就是错的.
    """
    # ast.walk 而不是只看顶层: 这些声明埋在 `with open_planes(...) as (rt,
    # gen):` 里面, 再往里还有 try. 逐层下钻的写法每加一层嵌套就要改一次,
    # 而 walk 对结构不敏感 -- 这里不需要知道它在第几层, 只需要知道它存在.
    names = set()
    for node in ast.walk(tree):
        # 只看赋值语句. 表达式语句 sess.declare_subscriber(...) 单独出现
        # (返回值直接丢弃) 是另一种缺陷, 由 CLAUDE.md 8.2 的静态规则管,
        # 不在本文件的范围内 -- 那种写法根本没有名字可以对账.
        if not isinstance(node, ast.Assign):
            continue
        val = node.value
        # 只认 <something>.declare_subscriber(...) 这一种调用形状.
        # 用 attr 名而不是完整的 "gen.declare_subscriber": 会话变量在三个
        # 模块里分别叫 gen / rt / session / self._gen, 绑死接收者名字会让
        # 判据在下一个模块里静默漏掉 (CLAUDE.md 3.2 形态六).
        if not (isinstance(val, ast.Call)
                and isinstance(val.func, ast.Attribute)
                and val.func.attr == "declare_subscriber"):
            continue
        for t in node.targets:
            # targets 是列表是因为 a = b = expr 合法. 只收 Name:
            # Attribute (self.x) 与 Subscript (d["x"]) 按上面的理由不收.
            if isinstance(t, ast.Name):
                names.add(t.id)
    return names


def _torn_down_names(tree):
    """返回所有[出现在一条会调用 undeclare() 的路径上]的名字.

    认两种形状, 因为本仓这两种都在用:
      (a) 直接调用   X.undeclare()
      (b) 列进容器再遍历   for e in (A, B, C): ... e.undeclare()
          -- 元组/列表里的每个 Name 都算被拆.
    (b) 必须真的在循环体里调 undeclare 才算: 否则 "把名字塞进任意一个元组"
    就能骗过本判据, 那又变回一条永远绿的断言.

    这一条是本文件与 "ruff F841 为 0" 的真正差别所在, 值得说透:
    F841 问的是 "这个名字被读过吗". 于是 `_ = teleop_sub`, 或者把它写进
    一句 log, 或者塞进一个跟拆除毫无关系的元组, 都能让 F841 闭嘴, 而订阅
    照样活到会话关闭. 本函数问的是一个窄得多的问题 -- 这个名字有没有出现
    在一条[确实会调用 undeclare()] 的路径上. 塞进元组是不够的: 那个元组
    还得被一个循环遍历, 且循环体里必须真的对循环变量调 undeclare().

    NO 不做跨函数分析. 三个接线模块的 finally 与声明都在同一个函数里,
    引入跨函数的数据流分析会让这条判据复杂到没人能审 -- 而复杂到没人能审
    的判据, 下一次红的时候会被当成误报关掉.
    """
    # 与 _declared_local_names 一样用 walk: 拆除块通常在 finally 里, 外面
    # 还套着 with 与 try, 深度与声明处并不相同.
    torn = set()
    for node in ast.walk(tree):
        # (a) 直接点名: X.undeclare(). 要求接收者是 Name 而不是任意表达式,
        # 因为本判据对账的就是名字; self._subs[0].undeclare() 之类不属于
        # 这里要管的写法.
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "undeclare"
                and isinstance(node.func.value, ast.Name)):
            torn.add(node.func.value.id)
        # (b) for <var> in (A, B, ...):  ... <var>.undeclare()
        #
        # p5_gateway 用的就是这个形状: 一个长元组 + 一个 for + 一个
        # try/except. 这里只认字面量 Tuple / List 作为 iter -- 如果哪天
        # 它变成一个变量 (for e in entities:), 本判据会把那些名字算成
        # [没拆], 判据变红. 那是有意的保守方向: 宁可误报一次让人来看,
        # 也不要因为看不懂而默认放过 (漏报的代价是又一个活过关机的订阅).
        if isinstance(node, ast.For) and isinstance(node.target, ast.Name):
            loop_var = node.target.id
            # 循环体里必须真的对循环变量调 undeclare(). 用 ast.walk 覆盖
            # 嵌套 (本仓这里包着一层 try/except 和一个 if entity is None).
            calls_undeclare = any(
                isinstance(n, ast.Call)
                and isinstance(n.func, ast.Attribute)
                and n.func.attr == "undeclare"
                and isinstance(n.func.value, ast.Name)
                and n.func.value.id == loop_var
                for n in ast.walk(node))
            if calls_undeclare and isinstance(node.iter, (ast.Tuple, ast.List)):
                for elt in node.iter.elts:
                    if isinstance(elt, ast.Name):
                        torn.add(elt.id)
    return torn


def test_the_scan_surface_is_not_empty_and_covers_the_three_wirings():
    """扫描面塌成空集时本文件会全绿 -- 先把这件事本身钉住.

    三个 main_wiring 是 2026-09-28 实际出过缺陷的那三个; 它们必须在面内.
    这一条同时守着 _sources() 的 rglob 没有因为路径改动而扫了个寂寞.

    为什么需要单独一条: 下面那条对账判据在扫描面为空时会[全绿] -- 没有
    文件可扫, missing 自然是空列表. 那是 CLAUDE.md 3.2 形态一最常见的成因:
    一条断言在它的输入消失时给出与 "一切正常" 完全相同的输出. 本条把
    [扫描面非空] 与 [三个已知模块在面内] 变成两个独立可红的事实.

    点名三个模块是有代价的: 哪天它们改名或合并, 本条会红. 那正是想要的 --
    改名的人会被迫来这里确认新路径仍在面内, 而不是让判据悄悄少守一块.
    """
    # 相对路径而不是绝对: 断言失败时打出来的字符串要能直接拿去 grep,
    # 且不随 checkout 位置变化.
    paths = [str(p.relative_to(ROOT)) for p, _ in _sources()]
    assert paths, "扫描面为空 -- xbrain/ 下一个 declare_subscriber 都没找到"
    for must in ("xbrain/p1_motion/runtime/main_wiring.py",
                 "xbrain/p2_core/runtime/main_wiring.py",
                 "xbrain/p5_gateway/runtime/main_wiring.py"):
        assert must in paths, "%s 不在扫描面内" % must


def test_every_locally_held_subscriber_is_undeclared_somewhere():
    """声明名单与拆除名单对账.

    MUTATION (2026-09-28 实测): 从 p5_gateway 的拆除元组里删掉任意一个名字
    (例 teach_sub), 本条红并逐条打印 模块 + 漏掉的名字; 放回即绿.

    失败信息里给的是[模块 + 名字]而不是一句 "有 N 处不匹配": 这条判据红的
    时候, 读到它的人需要的是 "去哪个 finally 里补哪个名字", 一个计数只会
    让他再跑一遍工具. 判定量不写进注释与文档 (CLAUDE.md 3.7), 但失败信息
    是现场求值的输出, 越具体越好.
    """
    missing = []
    for path, text in _sources():
        tree = ast.parse(text)
        declared = _declared_local_names(tree)
        # 没有任何局部名字形式的声明 -> 这个模块走的是容器持有那一路,
        # 跳过. 不是 "通过", 是 "本判据对它没有意见".
        if not declared:
            continue
        torn = _torn_down_names(tree)
        # 差集方向只有一个: 声明了却没拆 = 缺陷. 反方向 (拆了却没声明)
        # 不在这里判 -- 那正是 p2_core 那条 F821 的形状, 由 ruff 硬规则门
        # (tests/ci/test_ruff_hard_rules.py) 负责, 两条判据各守一半.
        for name in sorted(declared - torn):
            missing.append("%s: %s" % (path.relative_to(ROOT), name))
    # 修法只有一条, 写在这里免得下一个人选错: 把名字补进那个 finally /
    # 拆除元组. NO 绝对不要靠删掉赋值来让判据闭嘴 -- 那会让句柄失去唯一的
    # 强引用, Python GC 一收, Rust 端订阅[悄悄注销] (CLAUDE.md 4.3), 从
    # "关机时多活一会儿" 变成 "开机起就一帧都收不到".
    assert not missing, (
        "以下 declare_subscriber 的句柄用局部名字接住了, 但没有任何一条\n"
        "undeclare() 路径提到它们 -- 声明与拆除两张名单漂开了:\n  %s"
        % "\n  ".join(missing))
