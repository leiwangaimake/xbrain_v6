"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: source.py
Brief: MOT-PM-3 perception source abstraction

Description:
P1's 20 Hz loop consumes one perception frame per tick.

RETIRING (2026-09-08, PM1.4/P0.2): PerceptionFrame here models the v0.2 key set
(rt/perception/targets + lidar/grid ages). The RNS consume face moved to
rns/inputs.py (three keys rt/perception/{profile,objects,status}, 11 S3.1B).
This abstraction stays for any non-RNS consumer during transition; its lidar_*
/ grid_* fields are dead (no LiDAR). Removed when P2 wires rns/inputs fully.
"""



from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator, List, Optional


@dataclass(frozen=True)
class PerceptionFrame:
    """One snapshot the P1 loop consumes at 20 Hz."""
    mono_ms: int
    targets: tuple = ()            # list of targets, tuple for hashable
    d_free_m: Optional[float] = None
    grid_age_ms: int = 0
    lidar_age_ms: int = 0
    cam_rgbd_age_ms: int = 0


class PerceptionSource:
    """Abstract; caller subclasses for real (Zenoh sub) vs replay."""
    def next(self, now_mono_ms: int) -> Optional[PerceptionFrame]:
        raise NotImplementedError


@dataclass
class ReplayPerceptionSource(PerceptionSource):
    """Test source: deterministic frame sequence."""
    frames: List[PerceptionFrame] = field(default_factory=list)
    _idx: int = 0

    def next(self, now_mono_ms: int) -> Optional[PerceptionFrame]:
        if self._idx >= len(self.frames):
            return None
        f = self.frames[self._idx]
        self._idx += 1
        return f
