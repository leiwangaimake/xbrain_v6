"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_no_new_unwired_modules.py
Brief: 不许再多出"建了没接线"的模块

Description:
2026-09-04 终测一轮抓到三次同一个形状: 模块按契约实现了, 有测试, 测试全绿,
而生产代码[一次都没调用它].
  * Dispatcher.assert_complete  -- "缺执行器就拒绝启动"的闸门, 零实例化
  * mode/b_mode_forward         -- B 模式 gen 守卫, 零调用. 本轮建
    broadcast_rx 时把它的两条规则重新实现了一遍(not_active / wrong_stream),
    等于同一个判断有两份实现; 且它按 chunk_gen 判, 而 v2.0 的 AudioChunk
    根本没有 gen 字段, 只有 stream_id. 按 9.3 删掉, 不留被取代的脚手架.
    -- 这一条是本判据[第一次运行时自己抓到的]: 我以为接上了, 其实只在
    broadcast_rx 的注释里提了它一句.
  * ext/estop                   -- 急停回执七字段 + 100ms 判据, 零调用
三者的测试都在测[模块本身], 没有一条在测[系统用了它]. ext/estop 更进一步:
它连报文形状都是错的(action 读 data 顶层), 而夹具照着同一个错误假设写,
于是两边"对上了" -- 接线那天才发现每一条合规急停都会被拒.

*** 本判据只锁基线, NO 不逐条审.
全库 373 个模块里有 160 个没有任何生产 import(43%). 那是结构性事实, 不是
能顺手补的缺陷 -- 逐条审要单独一轮. 批量登记成"已知债"等于橡皮图章, 比
不做更坏(CLAUDE.md 3.2 形态二: 一条够不着的判据会被放宽到永远不报).
所以这里只钉住[数量不再增长]: 新写一个模块却忘了接线, 本条红.

要让本条通过的正当做法只有两个: 接线, 或者删掉它(9.3: 不留没有调用方的
未来脚手架). 改基线数字只有在[真的减少]时才做.
"""

import ast
import os

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

#: 2026-09-04 实测值. 只允许下降.
#: *** 降了要跟着改小, 否则本条会退化成"永远通不过的下限"从而被人放宽.
#: 160 -> 159: 本轮删掉了被 broadcast_rx 取代的 b_mode_forward.
#: 159 -> 171 (2026-09-08, RNS_TODO P0.4): RNS 新骨架 13 文件建齐但未接线
#:   (source.is_active 恒 false, 12 stub 待 P1~P6 填), + 旧 rns/ 10 文件迁
#:   _legacy/ 改了模块名. 二者都是 P0.4 的预期中间态 -- 接线在 P7.2 仲裁集成,
#:   _legacy 在 PM1.2 消化完即删. 届时本基线回落.
UNWIRED_BASELINE = 171


def _imports_of(root: Path):
    """一棵树里出现过的所有被导入名(相对 import 还原成绝对名)."""
    got = set()
    for dirpath, _dirs, files in os.walk(root):
        if "__pycache__" in dirpath:
            continue
        for fn in files:
            if not fn.endswith(".py"):
                continue
            path = Path(dirpath) / fn
            name = str(path.relative_to(ROOT))[:-3].replace(os.sep, ".")
            pkg = name.rsplit(".", 1)[0]
            try:
                tree = ast.parse(path.read_text(encoding="utf-8"))
            except Exception:      # noqa: BLE001
                continue
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    if node.level:
                        # 相对 import: level=1 是本包, 2 是上一级, 以此类推.
                        base = pkg.split(".")
                        if node.level > 1:
                            base = base[:len(base) - (node.level - 1)]
                        mod = ".".join(
                            base + ([node.module] if node.module else []))
                    else:
                        mod = node.module or ""
                    got.add(mod)
                    for alias in node.names:
                        got.add(mod + "." + alias.name)
                elif isinstance(node, ast.Import):
                    for alias in node.names:
                        got.add(alias.name)
    return got


def _production_modules():
    out = []
    for dirpath, _dirs, files in os.walk(ROOT / "xbrain"):
        if "__pycache__" in dirpath:
            continue
        for fn in files:
            # __init__ / __main__ 是包入口, 天然没有"被导入"的名字.
            if not fn.endswith(".py") or fn in ("__init__.py", "__main__.py"):
                continue
            path = Path(dirpath) / fn
            out.append(str(path.relative_to(ROOT))[:-3].replace(os.sep, "."))
    return out


def unwired_modules():
    prod = _imports_of(ROOT / "xbrain")
    return sorted(m for m in _production_modules() if m not in prod)


def test_no_new_module_is_built_without_being_wired():
    """*** 数量只许降, 不许升.

    MUTATION: 新建一个 xbrain/ 下的模块而不在生产代码里 import 它 -> 红.
    """
    unwired = unwired_modules()
    assert len(unwired) <= UNWIRED_BASELINE, (
        "新增了 %d 个没有生产调用方的模块(基线 %d). 新增的很可能在这里面:\n%s"
        % (len(unwired) - UNWIRED_BASELINE, UNWIRED_BASELINE,
           "\n".join("  " + m for m in unwired[-12:])))


def test_the_three_modules_this_criterion_was_born_from_are_wired_now():
    """*** 正向锚点: 光有"数量不涨"会被一个恒返回空表的实现通过.

    这三个是 2026-09-04 抓到的原始案例. 它们必须[不在]名单里 --
    哪天有人把接线撤了, 本条红, 而数量判据未必会红(撤一条接线只让数量
    从 160 涨到 161, 恰好在基线之上一格).
    """
    unwired = set(unwired_modules())
    for m in ("xbrain.p5_gateway.ext.estop",
              "xbrain.p2_core.audio.broadcast_rx",
              "xbrain.p2_core.audio.broadcast_sink"):
        assert m not in unwired, "%s 又变成没有生产调用方的模块了" % m
