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
    "audio/broadcast",
})


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
