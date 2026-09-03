"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_broadcast_key_owned_by_p2.py
Brief: audio/broadcast 的订阅者必须是 p2_core, 且只有它

Description:
守 11 S2.2 / RT-A3 的物理隔离: audio/broadcast 只被 p2_core 订阅,
audio/voice_in 只被 p4_agent 订阅.

为什么这条判据放在 p2 而不是 p5: 2026-09-03 之前订它的是 p5(收到只累加
字节数然后丢弃), 而 p5 那侧的 key surface 判据只检查"p5 该订的都订了" --
把 audio/broadcast 从 p5 挪走之后, 如果不在这里补一条, 这条 key 就[彻底
没人守]了: p2 那侧的订阅哪天被误删, 没有任何判据会红, 而现象是"喊话没
声音", 与硬件不在线不可区分.

判据用源码静态扫描而不是起进程: 起 p2 需要 Zenoh router + 配置 + 声卡,
在 CI 上跑不动; 而"这条 key 有没有被订"是一个静态可答的问题.
"""

import pytest

import re
from pathlib import Path

# INF-TS-1 三档 marker. 纯函数 / 静态检查, 不碰任何硬件 -> no_device.
pytestmark = pytest.mark.no_device


ROOT = Path(__file__).resolve().parents[3]
P2_WIRING = ROOT / "xbrain" / "p2_core" / "runtime" / "main_wiring.py"
P5_DIR = ROOT / "xbrain" / "p5_gateway"
P4_DIR = ROOT / "xbrain" / "p4_agent"


def test_p2_subscribes_the_cloud_broadcast_key():
    """p2 必须真的订上它.

    MUTATION: 把 p2 的 declare_subscriber("xbrain/%s/audio/broadcast") 删掉
    -> 这里红.
    """
    src = P2_WIRING.read_text(encoding="utf-8")
    # 找 declare_subscriber 的实参里带 audio/broadcast 的那一处.
    m = re.search(r'declare_subscriber\(\s*\n?\s*"xbrain/%s/audio/broadcast"',
                  src)
    assert m, "p2_core 没有订阅 xbrain/{rid}/audio/broadcast"


def test_the_subscription_is_guarded_by_a_real_rid():
    """*** rid 缺失时不能兜 "unknown".

    那会让本进程去订 xbrain/unknown/audio/broadcast -- 一条永远收不到帧的
    key, 现象与"云端没发"不可区分. 2026-09-01 已经在 p5 上踩过一次同型的
    坑(XBRAIN_ROBOT_ID 未设, 云端面整个哑掉两分钟).
    """
    src = P2_WIRING.read_text(encoding="utf-8")
    assert 'os.environ.get("XBRAIN_ROBOT_ID")' in src, (
        "rid 的取法变了, 确认没有引入 unknown 兜底")
    assert 'os.environ.get("XBRAIN_ROBOT_ID", "unknown")' not in src.split(
        "_bcast_rid")[1][:200], "订阅用的 rid 兜了 unknown"


def test_neither_p5_nor_p4_subscribes_it():
    """*** RT-A3 的隔离是[订阅关系]上的, 不依赖任何运行时模式判定.

    p4 订上它会破坏 RT-A3(audio/broadcast 与 audio/voice_in 两条链路必须
    物理隔离); p5 订上它就回到 2026-09-03 之前那个"看起来有人在处理, 
    实际进黑洞"的状态.
    """
    for d, who in ((P5_DIR, "p5_gateway"), (P4_DIR, "p4_agent")):
        for f in d.rglob("*.py"):
            src = f.read_text(encoding="utf-8")
            for m in re.finditer(r"declare_subscriber\([^)]{0,120}", src):
                assert "audio/broadcast" not in m.group(0), (
                    "%s 订阅了 audio/broadcast (%s)" % (who, f.name))
