"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: evolution.py
Brief: GWY-P4-21 -- instruction set evolution + hot-update boundaries + CF-1/2/3

Description:
16 S12 evolution rules govern what can be hot-updated vs what
requires a restart.

Hot-updatable (whitelist):
  * suspicion_rules.yaml (already done by P2 side)
  * speech_presets.yaml
  * asr_dict.yaml (L1 exact-replace)
  * query_templates.yaml
  * restate_templates.yaml
  * chitchat.yaml

Non-hot: intents.yaml, cmdset_18.json, prompts/*, p4_agent.yaml
(any of these needs a P4 restart; hot-swap would leave grammar out
of sync with routing).

CF-1: no two hot files may claim ownership of the same key
CF-2: intents.yaml must NOT reference an intent name whose id
      differs from cmdset_18.json's mapping (drift detection)
CF-3: version bump requires ALL hot files to carry a compatible
      version field; a hot file with an older major version blocks
      the reload
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, List


HOT_UPDATABLE_FILES: FrozenSet[str] = frozenset({
    "suspicion_rules.yaml",
    "speech_presets.yaml",
    "asr_dict.yaml",
    "query_templates.yaml",
    "restate_templates.yaml",
    "chitchat.yaml",
})


NON_HOT_FILES: FrozenSet[str] = frozenset({
    "intents.yaml",
    "cmdset_18.json",
    "p4_agent.yaml",
})


class EvolutionError(RuntimeError):
    """CF-1/2/3 violation."""


def check_cf1_no_shared_key(files_and_keys: dict) -> None:
    """CF-1: no key claimed by two hot files.

    files_and_keys: {filename: [keys]}"""
    seen: dict = {}
    for fname, keys in files_and_keys.items():
        for k in keys:
            if k in seen:
                raise EvolutionError(
                    "CF-1: key %r claimed by both %s and %s"
                    % (k, seen[k], fname))
            seen[k] = fname


def check_cf3_version_compat(current_major: int,
                              file_version: str) -> None:
    """CF-3: hot file's major version MUST match current_major."""
    try:
        file_major = int(file_version.split(".")[0])
    except (ValueError, AttributeError) as exc:
        raise EvolutionError(
            "CF-3: bad file version %r; expected N.M.P"
            % file_version) from exc
    if file_major != current_major:
        raise EvolutionError(
            "CF-3: hot file major version %d != current major %d"
            % (file_major, current_major))


def is_hot_updatable(filename: str) -> bool:
    import os
    name = os.path.basename(filename)
    return name in HOT_UPDATABLE_FILES
