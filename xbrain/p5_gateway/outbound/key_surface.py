"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: key_surface.py
Brief: CHK-1-41 P5 pub/sub key surface + 11 §2.2 bidirectional diff

Description:
P5's actual Zenoh publisher + subscriber sets MUST equal the ones
11 §2.2 assigns to P5 (bidirectional diff empty). Missing keys
mean a spec commitment is unfulfilled; extra keys mean the code
publishes something the contract didn't sanction.

Because the spec table lives in a large markdown file, the
projection here takes the SPEC set as a tuple + expects the
implementation set as a tuple, and compares. In deploy the spec
set comes from a §2.2 parser (a separate item); here we define the
CURRENT expected sets and provide the diff helper.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Set, Tuple


# The current P5 pub/sub commitments. Updated when 11 §2.2 changes.
# The whole point of this module is that when someone drops a
# publisher without updating this frozen set, the diff test reddens.
P5_EXPECTED_PUBLISHERS = frozenset({
    "state/link",
    "state/media",
    "state/geo/manifest",
    "event/{severity}/{category}",
    "cmd/task/ack",
    "cmd/estop/ack",
    "state/task",         # projection out of internal state/task
    "state/mode",
    "state/robot",
    "state/audio",
    "cmd/media/session/ack",
    "data/file/index",
})


P5_EXPECTED_SUBSCRIBERS = frozenset({
    # 11 S2.2.2A: Qt 的 1 Hz 在线心跳. 它是 S4.6 云端链路判定的输入 --
    # 在它之前只能靠"有没有收到任何云端报文"间接推断, 而 Qt 可以长时间
    # 只订阅不发布.
    "heartbeat/qt",
    "cmd/task",           # cmd/task/ext -> normalised inbound
    "cmd/estop",
    "cmd/media/session",
    "cmd/file/ack",
    # NO audio/broadcast [不在]本集合里.
    # 11 S2.2 逐字规定它的订阅者是"仅 p2_core"(RT-A3: audio/broadcast 只被
    # p2_core 订阅, audio/voice_in 只被 p4_agent 订阅, 两条链路在订阅关系上
    # 物理隔离, 不依赖任何运行时模式判定). 网关订它是越界 --
    # 2026-09-03 之前确实订着, 收到只累加字节数然后丢弃, 于是云端看到订阅
    # 存在而 PCM 进了黑洞. 现由 p2 直接订(p2_core/runtime/main_wiring).
    # *** 本集合是[p5 应订的]云端 key, 不是[云端全部]的 key -- 两者的差
    # 登记在下面的 OWNED_BY_OTHER_PROCESS 里.
})


#: v2.0 的云端 key 里[不归 p5]的那些, 连同归谁.
#:
#: *** 为什么要单独登记而不是干脆不提.
#: "登记表覆盖 v2.0 全部 key"是一条元判据: 客户契约新增 key 时登记表必须
#: 跟着长, 否则新 key 连"未接线"都报不出来 -- 它根本不在被检查的集合里.
#: 把 audio/broadcast 从视野里抹掉就正好制造了这个盲区: 哪天 p2 那侧的订阅
#: 被误删, 没有任何判据会红.
OWNED_BY_OTHER_PROCESS = {
    # 11 S2.2 / RT-A3: audio/broadcast 只被 p2_core 订阅, audio/voice_in
    # 只被 p4_agent 订阅. 两条链路在订阅关系上物理隔离.
    "audio/broadcast": "p2_core",
}


@dataclass(frozen=True)
class KeySurfaceDiff:
    spec_only_publishers: Tuple[str, ...]
    impl_only_publishers: Tuple[str, ...]
    spec_only_subscribers: Tuple[str, ...]
    impl_only_subscribers: Tuple[str, ...]

    def is_empty(self) -> bool:
        return not (self.spec_only_publishers
                    or self.impl_only_publishers
                    or self.spec_only_subscribers
                    or self.impl_only_subscribers)


def diff(actual_pubs: Iterable[str],
          actual_subs: Iterable[str],
          expected_pubs: Iterable[str] = P5_EXPECTED_PUBLISHERS,
          expected_subs: Iterable[str] = P5_EXPECTED_SUBSCRIBERS) -> KeySurfaceDiff:
    ap, as_ = set(actual_pubs), set(actual_subs)
    ep, es = set(expected_pubs), set(expected_subs)
    return KeySurfaceDiff(
        spec_only_publishers=tuple(sorted(ep - ap)),
        impl_only_publishers=tuple(sorted(ap - ep)),
        spec_only_subscribers=tuple(sorted(es - as_)),
        impl_only_subscribers=tuple(sorted(as_ - es)),
    )


class KeySurfaceDivergence(Exception):
    pass


def assert_surface_matches(actual_pubs: Iterable[str],
                             actual_subs: Iterable[str]) -> None:
    d = diff(actual_pubs, actual_subs)
    if not d.is_empty():
        raise KeySurfaceDivergence(
            "P5 key surface diverges from 11 §2.2:\n"
            "  spec_only_publishers=%s\n"
            "  impl_only_publishers=%s\n"
            "  spec_only_subscribers=%s\n"
            "  impl_only_subscribers=%s"
            % (d.spec_only_publishers, d.impl_only_publishers,
               d.spec_only_subscribers, d.impl_only_subscribers))
