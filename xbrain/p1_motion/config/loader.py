"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: loader.py
Brief: MOT-PM-25 P1 config loader (reads /run/xbrain/resolved only)

Description:
P1's config loader is a thin wrapper around
xbrain.common.config.resolved.load_resolved for the 'p1_motion' proc
name. This module's job is the STARTUP SELFCHECK that runs after the
config is loaded: RCG-1..RCG-4 constants present, capability bits
consistent with hardware, and the alias-blacklist keys (keep_dist_m
etc. duplicated from p2_core.yaml) NOT present here.
"""

from __future__ import annotations

from typing import Dict, FrozenSet


class P1SelfcheckError(RuntimeError):
    """RCG constants missing or forbidden alias present."""


# 14 S5.6.3 declares keep_dist_m / max_speed_mps live ONLY in
# p2_core.yaml.mode_motion. Their presence in p1_motion.yaml is a
# duplicate truth source and refused.
FORBIDDEN_ALIAS_KEYS: FrozenSet[str] = frozenset({
    "keep_dist_m",
    "max_speed_mps",
})


def check_no_alias_keys(cfg: Dict) -> None:
    """Walk the config and refuse if any FORBIDDEN_ALIAS_KEYS appear
    at any depth (they belong to p2_core.yaml)."""
    def walk(node, path=""):
        if isinstance(node, dict):
            for k, v in node.items():
                if k in FORBIDDEN_ALIAS_KEYS:
                    raise P1SelfcheckError(
                        "p1_motion.yaml contains forbidden alias key %r "
                        "at %s.%s (belongs to p2_core.yaml.mode_motion; "
                        "10 S5.4.5 alias blacklist)"
                        % (k, path, k))
                walk(v, path + "." + k)
        elif isinstance(node, list):
            for i, item in enumerate(node):
                walk(item, path + "[%d]" % i)
    walk(cfg)


def check_rcg_constants(cfg: Dict) -> None:
    """RCG-1..RCG-4 must have their constants declared (r_eff_fallback
    etc.). Missing means the rotation permit judge would run without
    a bounded threshold."""
    rns = cfg.get("rns", {})
    if not isinstance(rns, dict):
        return
    # rcg block presence check.
    rcg = rns.get("rcg", {})
    if not isinstance(rcg, dict):
        raise P1SelfcheckError(
            "rns.rcg block missing (RCG-1..RCG-4 constants required)")
    for key in ("r_eff_fallback_m",):
        if key not in rcg:
            raise P1SelfcheckError(
                "rns.rcg.%s missing (RCG-1 requires it)" % key)
