"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: degradation.py
Brief: MOT-PM-4 input freshness / degradation table (12 S3.3)

Description:
Each perception input has two age thresholds: degraded (start shedding capability, e.g. drop obstacle_avoid profile) and fail (stop). Table values come from 12 S3.3; the classify() helper converts one age into the closed-set state the caller acts on.
"""



from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Freshness(str, Enum):
    OK = "ok"
    DEGRADED = "degraded"
    FAILED = "failed"


@dataclass(frozen=True)
class InputThreshold:
    degraded_age_ms: int
    fail_age_ms: int


def classify(age_ms: int, thresh: InputThreshold) -> Freshness:
    if age_ms >= thresh.fail_age_ms:
        return Freshness.FAILED
    if age_ms >= thresh.degraded_age_ms:
        return Freshness.DEGRADED
    return Freshness.OK


# 12 S3.3 defaults for the four key inputs.
GRID_THRESH = InputThreshold(degraded_age_ms=500, fail_age_ms=2000)
LIDAR_THRESH = InputThreshold(degraded_age_ms=500, fail_age_ms=1000)
CAM_THRESH = InputThreshold(degraded_age_ms=300, fail_age_ms=1000)
FENCE_THRESH = InputThreshold(degraded_age_ms=1000, fail_age_ms=5000)
