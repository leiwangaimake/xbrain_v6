"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_key_surface_wired.py
Brief: 云端 key 面必须[真的接线], 不只是登记在表里

Description:
2026-08-23 核实云端契约时查出一件事: `outbound/key_surface.py` 里有一张
完整且正确的云端 key 登记表(CHK-1-41, 覆盖 v2.0 全部 17 条), 而 P5 的真实
接线里[一条云端 key 都没有] -- 入站 0/5, 出站 1/12(只有 state/link).

*** 守这张表的用例此前是这样写的:
    assert_surface_matches(P5_EXPECTED_PUBLISHERS, P5_EXPECTED_SUBSCRIBERS)
它把登记表喂给自己, 两个集合当然相等 -- 恒真. 它的两个变异体(删一条 -> 红)
也只是在证明 diff() 会算差集, 不是在证明[代码真的订阅了那些 key].

这次的形状比一般的形态1 更隐蔽: [登记表本身是正确且完整的], 所以任何人读它
都会以为云端面已经建好了. 一个错的表还能被读出来, 一个对的表配一条自证的
断言, 读不出来.

*** 本文件的判据: 从[真实接线]静态提取 key, 与登记表求双向差集.
提取方式是 AST -- 找 declare_subscriber(X) / declare_publisher(X) 的第一个
实参, X 是常量就取值, 是名字就去解析同文件里的常量赋值. NO 不 grep 字符串:
key 走常量(CMD_TASK_ACK_TOPIC 这类), grep 字面量会一条都找不到, 而"找不到"
与"没接线"在 grep 眼里一样.

Boundaries: 只判断[有没有 declare], 不判断 declare 之后的回调对不对 --
那由各条 key 自己的用例负责. 但"连 declare 都没有"是一切的前提.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
P5 = ROOT / "xbrain" / "p5_gateway"


def _string_constants(tree):
    """模块顶层的 NAME = "字面量" 赋值."""
    out = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if not (isinstance(node.value, ast.Constant)
                and isinstance(node.value.value, str)):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name):
                out[target.id] = node.value.value
    return out


def _declared_keys(method_names):
    """P5 真实接线里, 用指定方法声明过的 key 集合.

    method_names 例: {"declare_subscriber"} 或 {"declare_publisher"}.
    """
    found = set()
    for path in sorted(P5.rglob("*.py")):
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError:
            continue
        consts = _string_constants(tree)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            fn = node.func
            name = fn.attr if isinstance(fn, ast.Attribute) else getattr(
                fn, "id", "")
            if name not in method_names or not node.args:
                continue
            key = _resolve_key(node.args[0], consts)
            if key and key.startswith(CLOUD_HEAD):
                found.add(key[len(CLOUD_HEAD):])
    return found


def _declared_internal_keys(method_names):
    """机内(相对)key. 与云端面分开数, 见 _declared_keys 的说明."""
    found = set()
    for path in sorted(P5.rglob("*.py")):
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError:
            continue
        consts = _string_constants(tree)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            fn = node.func
            name = fn.attr if isinstance(fn, ast.Attribute) else getattr(
                fn, "id", "")
            if name not in method_names or not node.args:
                continue
            key = _resolve_key(node.args[0], consts)
            if key and not key.startswith(CLOUD_HEAD):
                found.add(key)
    return found


def _resolve_key(arg, consts):
    """把 declare_* 的第一个实参还原成 key 字符串, 还原不出返回 None.

    三种形态, 都在真实接线里出现过:
      "cmd/task"                    字面量
      CMD_TASK_TOPIC                模块顶层常量
      CLOUD_CMD_TASK % rid          常量模板 % 变量  <- 云端 key 都是这种
    最后一种是必须支持的: 云端 key 带 rid 前缀, 不可能写成字面量.
    """
    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
        return arg.value
    if isinstance(arg, ast.Name):
        return consts.get(arg.id)
    if isinstance(arg, ast.BinOp) and isinstance(arg.op, ast.Mod):
        # "模板" % rid -- 右边是运行期的值, 取左边的模板即可.
        return _resolve_key(arg.left, consts)
    return None


#: 云端 key 的前缀模板. 只有带它的声明才算云端面.
CLOUD_HEAD = "xbrain/%s/"


def test_the_extractor_finds_something():
    """*** 守本文件的前提.

    提取器返回空集时, 下面每条"未接线"的断言都会报出全部 key -- 看起来像
    一次灾难, 而真相可能只是 AST 走法写错了. 反过来, 如果断言方向是"集合
    为空即通过", 那提取器坏掉时会全绿.

    所以先钉住: P5 确实声明了一批 key(机内的那些), 提取器能看见它们.
    """
    subs = _declared_internal_keys({"declare_subscriber"})
    assert len(subs) >= 10, (
        "只从 P5 真实接线里提取到 %d 条机内订阅 -- 提取器可能坏了, "
        "而不是代码真的没接线: %s" % (len(subs), sorted(subs)))
    cloud = _declared_keys({"declare_subscriber"})
    assert cloud, "云端(带 rid 前缀的)订阅一条都没提取到"


def test_cloud_inbound_keys_are_actually_subscribed():
    """*** 云端入站面必须真的 declare_subscriber.

    登记表说 P5 订阅这 5 条云端 key; 本条查它们是否真的出现在接线里.

    * 这条今天是红的, 且[必须让它红] -- 云端联调的第一个动作就是 Qt 往
    cmd/task 发一条报文, 而现在没有人在听. 用 xfail 掩盖它, 等于把联调
    当天才会发现的事推迟到联调当天.
    """
    from xbrain.p5_gateway.outbound.key_surface import P5_EXPECTED_SUBSCRIBERS

    subs = _declared_keys({"declare_subscriber"})
    missing = sorted(set(P5_EXPECTED_SUBSCRIBERS) - subs)
    assert not missing, (
        "登记表声明 P5 订阅这些云端 key, 而真实接线里没有 declare_subscriber: "
        "%s\n"
        "  -> 云端发过来的报文不会被任何人收到. 登记表正确不代表接线存在."
        % missing)


def test_cloud_outbound_keys_are_actually_published():
    """*** 云端出站面必须真的 declare_publisher.

    同上. 出站缺失的表现更隐蔽: Qt 连上来了, 订阅也成功了, 只是永远收不到
    任何状态 -- 而 Zenoh 不会因为没有发布者而报错.
    """
    from xbrain.p5_gateway.outbound.key_surface import P5_EXPECTED_PUBLISHERS

    pubs = _declared_keys({"declare_publisher"})
    # 通配 key 在接线里会写成具体值(event/warn/system 这类), 单独处理.
    expected = {k for k in P5_EXPECTED_PUBLISHERS if "{" not in k}
    missing = sorted(expected - pubs)
    assert not missing, (
        "登记表声明 P5 发布这些云端 key, 而真实接线里没有 declare_publisher: "
        "%s\n"
        "  -> Qt 订阅成功但永远收不到内容, 且 Zenoh 不会报错."
        % missing)


def test_the_registry_covers_every_v2_key():
    """登记表必须覆盖 v2.0 的全部 17 条 key.

    这一条今天是绿的(登记表确实完整), 保留它是为了: 将来客户契约增加一条
    key 时, 登记表必须跟着长 -- 否则新 key 会连"未接线"都报不出来,
    因为它根本不在被检查的集合里.
    """
    import re

    from xbrain.p5_gateway.outbound.key_surface import (
        P5_EXPECTED_PUBLISHERS, P5_EXPECTED_SUBSCRIBERS)

    qt = (ROOT / "docs" / "MISSON" / "任务枚举_qt端v2.0.md").read_text(
        encoding="utf-8")
    v2_keys = set()
    for m in re.finditer(r"^\|\s*`xbrain/\{rid\}/([^`]+)`", qt, re.M):
        v2_keys.add(m.group(1))
    assert len(v2_keys) >= 15, "只从 v2.0 解析到 %d 条 key" % len(v2_keys)
    registered = set(P5_EXPECTED_PUBLISHERS) | set(P5_EXPECTED_SUBSCRIBERS)
    missing = sorted(v2_keys - registered)
    assert not missing, (
        "v2.0 有这些 key 而登记表里没有 -- 它们连[未接线]都报不出来: %s"
        % missing)


def test_no_cloud_key_is_silently_dropped_from_the_registry():
    """反向: 登记表里不许有 v2.0 没有的云端 key.

    多出来的一条意味着我方在往一个客户不订阅的 key 上发东西, 或者在等一条
    客户不会发的报文 -- 两种都是白做的工.
    """
    import re

    from xbrain.p5_gateway.outbound.key_surface import (
        P5_EXPECTED_PUBLISHERS, P5_EXPECTED_SUBSCRIBERS)

    qt = (ROOT / "docs" / "MISSON" / "任务枚举_qt端v2.0.md").read_text(
        encoding="utf-8")
    v2_keys = {m.group(1) for m in
               re.finditer(r"^\|\s*`xbrain/\{rid\}/([^`]+)`", qt, re.M)}
    registered = set(P5_EXPECTED_PUBLISHERS) | set(P5_EXPECTED_SUBSCRIBERS)
    extra = sorted(registered - v2_keys)
    assert not extra, (
        "登记表里有 v2.0 没有的云端 key: %s" % extra)


def test_an_internal_publisher_does_not_count_as_a_cloud_one():
    """*** 本条是对上面那两条判据的自查, 补于 2026-08-24.

    2026-08-23 那版提取器把 xbrain/%s/X 归一成 X 之后就与机内的裸 X 混在
    一起了. 于是 state/link 被算成"云端已发布" -- 而真相是它只发在机内
    相对 key 上, Qt 订的 xbrain/{rid}/state/link 上一个字节都没有.
    [一条本该抓这件事的判据, 自己制造了一个假绿.]

    这比原来的病更隐蔽: 原来是断言恒真(形态1), 现在是断言看错了对象 --
    它确实在读真实接线, 只是把两个不同的东西数成了一个.

    判据: 两个提取器的结果不得有交集意义上的混淆 -- 机内提取器必须能看到
    state/link(它确实在那儿), 而云端提取器必须看不到它, 除非真有一条带
    前缀的声明.
    """
    internal = _declared_internal_keys({"declare_publisher"})
    cloud = _declared_keys({"declare_publisher"})

    assert "state/link" in internal, (
        "机内提取器看不到 state/link -- 提取器坏了")

    # *** 判据必须用一条[只在机内存在]的 key.
    # 用 state/link 判不行 -- 它现在两边都有(机内 11 S4.6 形状 + 云端
    # v2.0 S4.1 形状), 于是"云端集合里有它"是正确的, 判不出混淆.
    # probe/estop/ping 是 P5 内部的探针 key(11 CR-2), 客户契约里根本没有
    # 这条 -- 它出现在云端集合里, 只可能是提取器把机内的算进去了.
    #
    # MUTATION: 让 _declared_keys 对不带前缀的 key 也照收 -> 这里红.
    assert "probe/estop/ping" in internal, "提取器看不到机内探针 key"
    assert "probe/estop/ping" not in cloud, (
        "机内 key probe/estop/ping 被算进了云端面 -- 两个提取器混淆了, "
        "于是[任何]只发在机内的 key 都会被算成云端已接线. "
        "2026-08-24 就是这样让 state/link 假绿了一整轮.")
