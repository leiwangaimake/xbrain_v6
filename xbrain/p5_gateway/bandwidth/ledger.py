"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: ledger.py
Brief: GWY-P5-16 uplink bandwidth ledger + UG-1 / UG-2 (never boost)

Description:
17 S16 the uplink bandwidth ledger tracks (bytes_out per second)
across three planes: control, data, media. Two guards, both
non-boosting:

  UG-1  the AGGREGATE budget is fixed; a plane cannot borrow from
        another. When plane X saturates, X drops; Y is not helped.
        Rationale: if we let planes borrow, an emergency estop
        (control plane) could be delayed by an FTP transfer in
        progress. The whole design assumes each plane's budget is
        satisfied INDEPENDENTLY.

  UG-2  no plane may raise its own budget dynamically. Config
        changes require a freeze reload; runtime code CANNOT bump
        its ceiling. This closes a well-known bug pattern where a
        'temporary boost' during incident response becomes
        permanent.
"""

from __future__ import annotations

from dataclasses import dataclass, field


PLANES = frozenset({"control", "data", "media"})


class UnknownPlane(Exception):
    pass


class BandwidthBoostForbidden(Exception):
    """UG-2: attempted to raise a plane's budget at runtime."""


@dataclass
class Ledger:
    """Per-plane bytes/s budget + current usage. Budgets are set at
    construction; raise_budget is intentionally unimplemented."""
    budgets: dict            # plane -> Bps
    usage: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        unknown = set(self.budgets) - PLANES
        if unknown:
            raise UnknownPlane(f"unknown planes: {sorted(unknown)}")
        for p in PLANES:
            self.usage.setdefault(p, 0)

    def record(self, plane: str, bytes_out: int) -> None:
        if plane not in PLANES:
            raise UnknownPlane(plane)
        self.usage[plane] += bytes_out

    def over_budget(self, plane: str) -> bool:
        if plane not in self.budgets:
            return False
        return self.usage.get(plane, 0) > self.budgets[plane]

    def reset(self) -> None:
        """End-of-interval reset (called by the ledger tick)."""
        for p in self.usage:
            self.usage[p] = 0

    def raise_budget(self, plane: str, new_budget: int) -> None:
        """UG-2: refused. Config change requires freeze reload."""
        raise BandwidthBoostForbidden(
            f"cannot raise {plane!r} budget at runtime "
            f"(requested {new_budget}); config change requires "
            f"freeze reload")

    def borrow_from(self, giver: str, taker: str) -> None:
        """UG-1: refused."""
        raise BandwidthBoostForbidden(
            f"cannot borrow between planes ({giver!r} -> {taker!r})")
