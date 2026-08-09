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


# Categories that need the multi-branch shape per 16 S8.4.
_QT_REQUIRED_BRANCHES: Dict[str, FrozenSet[str]] = {
    "QT-1_battery":       frozenset({"ok", "unknown", "shadow"}),
    "QT-2_mode":          frozenset({"ok", "unknown"}),
    "QT-3_position":      frozenset({"ok", "unknown", "est"}),
    "QT-4_time":          frozenset({"ok", "est"}),
    "QT-5_next_task":     frozenset({"ok", "unknown"}),
    "QT-6_light":         frozenset({"ok", "unknown"}),
    "QT-7_target_count":  frozenset({"ok", "unknown"}),
    "QT-8_current_task":  frozenset({"ok", "unknown", "shadow"}),
    "QT-9_speed":         frozenset({"ok", "unknown"}),
    "QT-10_fence_name":   frozenset({"ok", "unknown"}),
    "QT-11_charge_state": frozenset({"ok", "unknown", "shadow"}),
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
