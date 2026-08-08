"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: xbrain-config-freeze service (Type=oneshot) -- Stage 0c

Description:
CFG-FZ-1 lands the skeleton: ASSERT_REGISTRY driving execution order (ORD-1),
MANIFEST.json production, tmp-fs resolved snapshots. Every INDIVIDUAL
assertion (A / J / M / B / C / D / E / G / I / K / L / ...) lives here too
but is filled in by its own item (CFG-FZ-2 = J, CFG-FZ-3 = A+M, and so on).
This file establishes the FRAMEWORK; the mutations CFG-FZ-1 fires cover
ORD-1 ordering + registry-vs-manifest bidirectional-diff empty, not any
individual assertion's contents.

Never import from here into a runtime process. The freeze service runs and
exits; the runtime opens /run/xbrain/resolved/{proc}.yaml through
xbrain.common.config.resolved.load_resolved. If a runtime module reached
into xbrain.boot the pipeline would be running two loaders and the "one
snapshot" invariant CFG-CM-11 defends would silently fail.
"""

from xbrain.boot.freeze.pipeline import build_manifest, run_freeze
from xbrain.boot.freeze.registry import (
    ASSERT_REGISTRY, AssertSpec, ordered_assertion_names,
)

__all__ = [
    "AssertSpec", "ASSERT_REGISTRY", "ordered_assertion_names",
    "build_manifest", "run_freeze",
]
