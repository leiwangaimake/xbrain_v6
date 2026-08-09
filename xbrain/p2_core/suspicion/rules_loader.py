"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: rules_loader.py
Brief: BIZ-P2-14 -- suspicion_rules.yaml loader + hot reload discipline

Description:
14 S6 rule engine. suspicion_rules.yaml is the ONE integrally-hot-
updatable file in the system (RE-1..RE-6 govern schema).

Reload discipline (14 S6 RE-1/2):
  * Reload is ATOMIC: the whole file re-parses; if it fails schema,
    the OLD ruleset stays alive and a warn event fires. NEVER a
    half-loaded state.
  * A rule that references a non-existent field (e.g., a targets
    attribute) fails schema at load; do not silently drop.

RE-3a: rules with a time_window MUST have ts_sync=true or they do
NOT match. Failure direction: no false positive on unauthenticated time.

RE-7 night patrol: enabled=false makes the night_patrol.window field
inaccessible; any rule referencing it is dropped from the loaded set
(explicit degrade, not silent).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml


class RulesSchemaError(RuntimeError):
    """Rules YAML failed schema. Raised by parse; caller (hot-reload
    driver) catches and preserves the previous ruleset."""


@dataclass
class Rule:
    """One parsed suspicion rule."""
    id: str
    when: Dict[str, Any]         # trigger conditions
    then: Dict[str, Any]         # actions
    time_window: Optional[Dict[str, str]] = None
    requires_night_patrol: bool = False


@dataclass
class Ruleset:
    """The parsed, ATOMIC ruleset. Callers hold a reference; a
    successful reload swaps this out. During a failed reload the
    old Ruleset stays in place."""
    rules: List[Rule] = field(default_factory=list)
    version: int = 0            # incremented on each successful load


def parse_ruleset(yaml_text: str) -> Ruleset:
    """Parse a yaml text into a Ruleset. Raises RulesSchemaError on
    ANY schema violation -- half-parsed rules are NEVER returned."""
    try:
        doc = yaml.safe_load(yaml_text)
    except yaml.YAMLError as exc:
        raise RulesSchemaError("YAML parse failed: %s" % exc) from exc
    if doc is None:
        # Empty document is a valid empty ruleset (skeleton state).
        return Ruleset(rules=[], version=1)
    if not isinstance(doc, dict):
        raise RulesSchemaError(
            "rules root must be a mapping; got %s" % type(doc).__name__)
    raw_rules = doc.get("rules", [])
    if not isinstance(raw_rules, list):
        raise RulesSchemaError(
            "rules.rules must be a list; got %s" % type(raw_rules).__name__)
    parsed: List[Rule] = []
    for i, row in enumerate(raw_rules):
        if not isinstance(row, dict):
            raise RulesSchemaError(
                "rules[%d] not a mapping" % i)
        try:
            rid = row["id"]
            when = row["when"]
            then = row["then"]
        except KeyError as exc:
            raise RulesSchemaError(
                "rules[%d] missing required field %s" % (i, exc)
            ) from exc
        if not isinstance(rid, str) or not rid:
            raise RulesSchemaError(
                "rules[%d].id must be non-empty string" % i)
        parsed.append(Rule(
            id=rid, when=when, then=then,
            time_window=row.get("time_window"),
            requires_night_patrol=bool(row.get("requires_night_patrol", False)),
        ))
    return Ruleset(rules=parsed, version=1)


def filter_by_night_patrol(rs: Ruleset, night_patrol_enabled: bool) -> Ruleset:
    """RE-7: when night_patrol is disabled, drop rules that need it.
    Returns a NEW Ruleset (original is unchanged)."""
    if night_patrol_enabled:
        return rs
    kept = [r for r in rs.rules if not r.requires_night_patrol]
    return Ruleset(rules=kept, version=rs.version)


def filter_by_ts_sync(rs: Ruleset, ts_sync: bool) -> Ruleset:
    """RE-3a: rules with time_window are DROPPED (not matched) if
    ts_sync is False. Failure direction = no false positive."""
    if ts_sync:
        return rs
    kept = [r for r in rs.rules if r.time_window is None]
    return Ruleset(rules=kept, version=rs.version)
