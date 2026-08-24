"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_cloud_acceptance_path.py
Brief: 云端五类指令的执行链路走到哪 -- 可执行的验收地图

Description:
用户 2026-08-24 给的验收路径逐字: "云端下发所有支持的指令, XBRAIN_V6 给出
相应的执行结果, 实际控制底盘的任务指令 XBRAIN 在最终输出部分看控制下发给
quadruped 的消息即可(因为没有物理底盘)".

*** 本文件把那条路径做成[可执行的], 而不是写进一份会过期的文档.
一份"链路完整性"的 markdown 表在第一次改动后就开始腐烂, 而且没人会在改动
时想起去更新它. 这里的判据是从[真实的 pub/sub 图]算出来的: 谁发这条 key,
谁订这条 key, 一路走到进程外的那个出口.

*** 最要紧的一条: 断裂处必须被[点名], NO 不许静默通过.
一个只判"链路完整"的用例, 在链路断了的时候会红 -- 然后有人把它改成
"包含即可"或直接 skip, 于是它变成永远绿(CLAUDE.md 3.2 形态②). 所以本文件
用的是[冻结当前真相]的写法: 每一类的终点写死在 TERMINUS 表里, 与实测的
pub/sub 图对不上就红. 链路补通了同样会红 -- 那是好事, 它强制改表, 而改表
的人会看到这段说明.

*** 扫描面声明(CLAUDE.md 3.2 形态⑥):
只扫 xbrain/ 下的 .py, 排除 tests/. C++ 侧(quadruped / chassis_relay)不在
扫描面内 -- 它们的订阅在 .cc 里, 本文件看不到. 所以"终点是 p1 的 APDU 出口"
这个结论只到[Python 侧把 APDU 交给 ChassisClient]为止, 再往下是真机的事.
"""
from __future__ import annotations

import ast
import pathlib

import pytest

pytestmark = pytest.mark.no_device

ROOT = pathlib.Path(__file__).resolve().parents[2]
XBRAIN = ROOT / "xbrain"


# --- 从真实代码算 pub/sub 图 -----------------------------------------

_GRAPH_CACHE = {}


def _module_consts(tree):
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(
                node.value, ast.Constant) and isinstance(
                    node.value.value, str):
            for t in node.targets:
                if isinstance(t, ast.Name):
                    out[t.id] = node.value.value
    return out


def _key_of(arg, local, shared):
    """把 declare_* 的 key 实参还原成字符串.

    *** 必须解析[跨模块导入的]常量, 否则判据是瞎的.
    2026-08-24 第一版只查本模块的常量表, 于是 p2_core 那句
    `subs.declare(session, SPEAK_TOPIC, ...)` 解不出来 -- SPEAK_TOPIC 定义在
    speaker_wiring.py 里. 判据据此报出"cmd/audio/speak 全仓无订阅者", 看起来
    像一次重大发现, 实际是提取器看不见.
    这正是 CLAUDE.md 3.2 那条: 一个瞎掉的扫描器会把一切报成缺失, 而缺失的
    形状与真发现一模一样.
    """
    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
        return arg.value
    if isinstance(arg, ast.Name):
        if arg.id in local:
            return local[arg.id]
        # 跨模块: 全仓唯一同名常量才认. 同名不同值的一律不认 -- 猜一个会
        # 把 A 模块的 key 记到 B 模块头上.
        values = shared.get(arg.id)
        return next(iter(values)) if values and len(values) == 1 else None
    if isinstance(arg, ast.BinOp) and isinstance(arg.op, ast.Mod):
        return _key_of(arg.left, local, shared)
    return None


def _graph():
    """(subscribers, publishers): key -> {进程名}. 结果缓存 -- 解析整个
    xbrain/ 要十几秒, 而本文件有五条用例都要用.

    进程名取 xbrain/ 下的第一层目录 -- p1_motion / p2_core / ... 这一层
    正好是进程边界(10 S3.1), 所以"跨进程链路"这个问题在这张图上可答.
    """
    if _GRAPH_CACHE:
        return _GRAPH_CACHE["subs"], _GRAPH_CACHE["pubs"]
    trees = {}
    shared: dict = {}
    for path in sorted(XBRAIN.rglob("*.py")):
        try:
            trees[path] = ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError:
            continue
        for name, value in _module_consts(trees[path]).items():
            shared.setdefault(name, set()).add(value)

    subs: dict = {}
    pubs: dict = {}
    for path, tree in trees.items():
        rel = path.relative_to(XBRAIN)
        proc = rel.parts[0] if len(rel.parts) > 1 else "common"
        local = _module_consts(tree)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            fn = node.func
            name = fn.attr if isinstance(fn, ast.Attribute) else getattr(
                fn, "id", "")
            if name in ("declare_subscriber", "declare"):
                # SubscriberRegistry.declare(session, key, handler) -- key 是
                # 第二个实参; declare_subscriber(key, cb) -- 第一个.
                idx = 1 if name == "declare" and len(node.args) >= 3 else 0
                key = _key_of(node.args[idx], local, shared)
                if key:
                    subs.setdefault(key, set()).add(proc)
                continue
            if name in ("declare_publisher", "put"):
                key = _key_of(node.args[0], local, shared)
                if key:
                    pubs.setdefault(key, set()).add(proc)
    _GRAPH_CACHE["subs"], _GRAPH_CACHE["pubs"] = subs, pubs
    return subs, pubs


# --- 冻结当前真相 -----------------------------------------------------
#
# 每一类云端指令 -> (机内 key, 这条 key 今天的消费进程集合).
# *** 这张表是[实测结果的冻结], 不是设计愿望. 改动链路时它会红, 那正是
# 它存在的理由.

TERMINUS = {
    # 五类里唯一真正驱动底盘的. p3 收下并记账, 但[p3 到运动执行那一跳
    # 今天还不存在] -- 见 test_goto_stops_at_p3.
    "GOTO_KEYPOINT": ("cmd/task", {"p3_task"}),
    "STOP_TASK": ("cmd/task", {"p3_task"}),
    # cmd/estop 三个软件订阅者全接: p2_core(域1缴械+爆闪, 批62) +
    # p1_motion(本拍零速+latch, 批63) + p3_task(ES-1 freeze 冻结调度, 批64).
    # 契约(11 S1.4)第四个 chassis_relay 是 C++, 不在本扫描面.
    "ESTOP": ("cmd/estop", {"p2_core", "p1_motion", "p3_task"}),
    # 报警区配置落 geo 库. p5 只是发布者, 不订它.
    "SET_ALARM_CONFIG": ("cmd/geo", {"p3_task"}),
    # 喊话经 p2_core 的 speaker_wiring 出 TTS. 这条是通的.
    "AUDIO_CONTROL": ("cmd/audio/speak", {"p2_core"}),
}


def test_the_graph_is_not_blind():
    """*** 守本文件的前提, 补于变异体实测之后.

    上面每一条"链路断了"的断言都是[集合为空即成立]的形状. 提取器一旦坏掉,
    每条 key 都解不出来, 于是每一条都通过 -- 而"断链"与"扫描器瞎了"在结论上
    完全一样(CLAUDE.md 3.2: 一个瞎掉的扫描器会把一切报成缺失).

    第一版没有这条, 于是"提取器恒返回 None"的变异体[没有变红].

    判据: 这张图必须认出足够多的 key 与足够多的进程. 数字取得宽松, 它不是
    覆盖率指标, 只是"提取器还活着"的下界.
    """
    subs, pubs = _graph()

    assert len(subs) >= 15, (
        "只从 xbrain/ 解出 %d 条订阅 -- 提取器可能坏了, 而不是代码没接线: %s"
        % (len(subs), sorted(subs)))
    assert len(pubs) >= 15, "只解出 %d 条发布" % len(pubs)
    procs = {p for ps in subs.values() for p in ps}
    assert {"p1_motion", "p2_core", "p3_task", "p5_gateway"} <= procs, (
        "少了进程: %s" % sorted({"p1_motion", "p2_core", "p3_task",
                                 "p5_gateway"} - procs))


def test_the_router_targets_match_the_terminus_table():
    """路由器的目标 key 与本表一致.

    路由器改了目标而本表没跟着改, 下面每一条断言就都在看错的 key.
    """
    from xbrain.p5_gateway.inbound.task_router import OPEN_TASK_TYPES

    assert set(TERMINUS) == set(OPEN_TASK_TYPES), (
        "TERMINUS 表与路由器开放的任务类型对不上: %s"
        % sorted(set(TERMINUS) ^ set(OPEN_TASK_TYPES)))


def test_every_cloud_command_reaches_its_recorded_consumers():
    """*** 五类逐条: 路由目标 key 的真实订阅者 == 表里冻结的集合.

    这一条既防"链路断了"也防"链路悄悄变了": 多一个订阅者同样会红, 因为
    一条指令多一个消费者意味着它可能被执行两次.
    """
    subs, _pubs = _graph()

    problems = []
    for task_type, (key, expected) in sorted(TERMINUS.items()):
        # p5_gateway 既是转发者也可能是订阅者; 去掉它自己发出去那一侧,
        # 剩下的才是"下游".
        actual = set(subs.get(key, set()))
        if actual != expected:
            problems.append("%s -> %s: 实测消费者 %s, 表里写的 %s"
                            % (task_type, key, sorted(actual) or "无",
                               sorted(expected) or "无"))
    assert not problems, (
        "云端指令的下游与冻结的验收地图对不上:\n  " + "\n  ".join(problems)
        + "\n  -> 链路补通了就改 TERMINUS 表(那是好事); 链路断了就是回归.")


def test_goto_stops_at_p3_and_never_reaches_the_chassis_today():
    """*** 联调当天最要紧的一条事实, 写成断言而不是写进文档.

    云端 GOTO_KEYPOINT 走到 p3_task 就停了: p3 把任务记进 task.db 并跑
    状态机, 但[它不发 cmd/motion/intent] -- 而那条 key 才是 p1_motion 转成
    CHS-A APDU 发给底盘的入口.
    => 联调时云端会收到 ack=accepted 和 state/task 的状态流转, 而底盘那侧
    一个 APDU 都不会出现.

    * 这不是本轮引入的缺口, 是既有的 PB8"执行接线"未做. 但云端联调会第一次
    把它暴露在客户面前, 所以必须在这里点名.

    链路补通后本条会红 -> 改 TERMINUS 表并删掉这条用例.
    """
    subs, pubs = _graph()

    assert "p3_task" not in pubs.get("cmd/motion/intent", set()), (
        "p3_task 开始发 cmd/motion/intent 了 -- 任务执行链路已补通, "
        "请更新 TERMINUS 表并删除本用例")
    # 反向: 这条 key 确实有一个通往底盘的消费者, 只是没人给它喂任务.
    assert "p1_motion" in subs.get("cmd/motion/intent", set()), (
        "cmd/motion/intent 连 p1_motion 都不订了 -- 那语音的运动指令也断了")


def test_the_chassis_exit_is_p1_turning_intents_into_apdu():
    """验收路径的终点(Python 侧).

    用户要看的"下发给 quadruped 的消息"就是这一步的产物: p1_motion 把
    MotionIntent 换成 CHS-A APDU 交给 ChassisClient. 再往下是 socket 与
    真机, 不在本文件扫描面内.
    """
    src = (XBRAIN / "p1_motion" / "runtime" / "main_wiring.py").read_text(
        encoding="utf-8")
    tree = ast.parse(src)
    names = {getattr(n.func, "id", "") or getattr(n.func, "attr", "")
             for n in ast.walk(tree) if isinstance(n, ast.Call)}

    assert "intent_to_apdu" in names, "p1 不再把 intent 转 APDU"
    assert "send_apdu" in names, "p1 不再把 APDU 交给底盘客户端"


def test_estop_reaches_all_three_software_subscribers():
    """*** cmd/estop 三个软件订阅者全接(CLD-1a 批62-64 后).

    批59 查出 cmd/estop 零订阅者(三条软件急停路径发进空气). 现全接:
      p2_core    域1 缴械 -> state/arb/motion 广播 + 域4 爆闪 (14 S3.7, 批62)
      p1_motion  本拍零速 + stop_reason latch + re-arm (P1-21, 批63)
      p3_task    ES-1 freeze 冻结调度 (15 S11.1, 批64)
    契约(11 S1.4)第四个订阅者 chassis_relay 是 C++(CR-1 纯转发到
    rt/safety/estop), 不在本扫描面 -- 那是真正的执行路径(SE-1a).

    * 三个软件侧各司其职且互不依赖: p2 广播缴械态供 p1 冗余读, p1 自己也
    直接订(不依赖广播到达), p3 冻结任务调度. 一条软件急停同时触发三者.

    这条从"补通即红"变成"全接即冻结": 若哪个订阅丢了 -> 红.
    """
    subs, pubs = _graph()

    got = subs.get("cmd/estop", set())
    assert got == {"p2_core", "p1_motion", "p3_task"}, (
        "cmd/estop 的软件订阅者集合变了: %s -- 少一个就是一条软件急停路径断了"
        % sorted(got))
    assert "p5_gateway" in pubs.get("cmd/estop", set()), (
        "连发布者都没了 -- 那 HMI 的 ESTOP 按钮也断了")


def test_audio_control_is_the_one_fully_wired_chain():
    """反向对照: 五类里 AUDIO_CONTROL 的下游是通的.

    * 没有这条, 上面那些"断了"的断言会显得像扫描器坏了 -- 一个瞎掉的
    提取器会把[每一条]都报成断链, 而那与真发现形状一样(CLAUDE.md 3.2).
    有一条确实通的, 才证明提取器在工作.

    云端 AUDIO_CONTROL -> cmd/audio/speak -> p2_core 的 speaker_wiring
    -> GZH-2 的 TTS. 半双工门控也在 p2 那一侧(18 S13.1), 所以云端喊话
    与本地语音的互斥是[免费得到的] -- 两者走同一个门.
    """
    subs, _pubs = _graph()

    assert subs.get("cmd/audio/speak") == {"p2_core"}, (
        "喊话链路变了: %s" % sorted(subs.get("cmd/audio/speak", [])))


def test_the_cloud_face_itself_is_complete():
    """与上面两条对照: 网关这一侧是全通的.

    17 条云端 key 全部真接线(批54-58), 断的是它下游的两跳. 分清这一点很
    重要 -- 联调当天若把"云端没反应"当成网关问题, 会往错的方向查一整天.
    """
    from xbrain.p5_gateway.outbound.key_surface import (
        P5_EXPECTED_PUBLISHERS, P5_EXPECTED_SUBSCRIBERS)

    assert len(set(P5_EXPECTED_PUBLISHERS) | set(P5_EXPECTED_SUBSCRIBERS)) >= 17
