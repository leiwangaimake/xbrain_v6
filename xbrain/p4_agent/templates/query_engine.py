"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: query_engine.py
Brief: GWY-P4-15 -- query_templates.yaml + QT-1..QT-11 hard branches

Description:
16 S8.4 query answer templates. For queries with dynamic state
(e.g., 'battery level?' 'current mode?'), each template has 3
branches:

  ok       normal answer
  unknown  state is genuinely unknown (fresh boot, sensor missing)
  shadow   answer available but suspicious (e.g., stale reading)
  est      answer is an estimate (e.g., time-of-day approximate)

QT-1..QT-11 (16 S8.4): the 11 categories that MUST have all four
branches. Loading a template that has 'ok' but omits 'unknown' or
'shadow' branches -> refuse to load.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, FrozenSet, List


# 16 S8.2.1 QT-1..QT-11 -> 16 S8.4 template branch keys. KEY IS THE
# TEMPLATE NAME (query_light_state), NOT "QT-1_battery".
#
# 2026-08-11 (GWY-P4-34 / 32.B): the earlier table drifted badly from
# the doc -- it had QT-1=battery with ok/unknown/shadow branches, but
# 16 S8.2.1 QT-1 is G26 query_light_state (payload/chassis/all), QT-3 is
# G28 strobe (shadow-only, "per our record cannot read back"), QT-5 is
# G32 ptz_pose (unknown / unknown_moving, position readback is a fake
# value), etc. The eight-hard-branch table (16 S8.2.1) plus the S8.4
# template library are the authority; the branch keys below are the
# EXACT branch names from the S8.4 template blocks.
_QT_REQUIRED_BRANCHES: Dict[str, FrozenSet[str]] = {
    "query_light_state":    frozenset({"payload", "chassis", "all"}),      # QT-1/QT-2 G26
    "query_strobe_state":   frozenset({"shadow"}),                          # QT-3 G28
    "query_speaking":       frozenset({"idle", "speaking", "queued"}),      # QT-4 G30
    "query_ptz_pose":       frozenset({"unknown", "unknown_moving"}),       # QT-5 G32
    "query_ptz_zoom":       frozenset({"step_only"}),                       # QT-6 G33
    "query_ptz_owner":      frozenset({"held", "stale"}),                   # QT-7 G37
    "query_active_fence":   frozenset({"agree", "disagree"}),               # QT-8 G39
    "query_fence_relation": frozenset({"ok_in", "ok_out", "degraded"}),     # QT-9 G41
    "query_fence_detail":   frozenset({"ok", "timeout"}),                   # QT-11 G42
    # QT-10 G34 query_ptz_track has no multi-branch template in 16 S8.4
    # (it is "answer 'tracking' + target description"), so no required
    # branch set -- check_qt_branches no-ops for it, same as any
    # non-multi-branch query template.
}


class TemplateSchemaError(RuntimeError):
    """query_templates.yaml row missing a required branch."""


def check_qt_branches(template_id: str,
                      declared_branches: FrozenSet[str]) -> None:
    """Refuse if template_id is a QT-* category and its declared
    branches don't cover the required set."""
    req = _QT_REQUIRED_BRANCHES.get(template_id)
    if req is None:
        return    # not a QT template; no branch requirement here
    missing = req - declared_branches
    if missing:
        raise TemplateSchemaError(
            "QT template %s missing required branch(es) %s"
            % (template_id, sorted(missing)))
