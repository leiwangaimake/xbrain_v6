"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: presets.py
Brief: BIZ-P2-30 -- speech_presets.yaml loader + BIT announce sequence

Description:
14 S11 announce block references speech presets by ID. The presets
themselves live in a separate YAML (configs/speech_presets.yaml)
which is one of the two hot-updatable files. This module loads it,
validates schema, and assembles the announce sequence for BIT-03
per the announce.policy in p2_core.yaml.

Reload discipline (same as suspicion_rules):
  * atomic; failed schema -> old ruleset stays
  * unknown preset ID at load = SchemaError (do not silently fail
    at speak time)

speech_preset health item:
  * OK        loader accepted the yaml
  * DEGRADED  at least one preset had a warning (e.g., >12 chars TTS)
  * FAIL      YAML failed schema -> fallback to code defaults
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import yaml


class SpeechPresetError(RuntimeError):
    """speech_presets.yaml failed schema."""


@dataclass(frozen=True)
class SpeechPreset:
    id: str
    text: str
    voice: str = "male"          # closed set: male / female


@dataclass(frozen=True)
class PresetSet:
    presets: Dict[str, SpeechPreset]
    version: int


def parse_presets(yaml_text: str) -> PresetSet:
    """Parse speech_presets.yaml. Raises SpeechPresetError on any
    schema violation."""
    try:
        doc = yaml.safe_load(yaml_text)
    except yaml.YAMLError as exc:
        raise SpeechPresetError("YAML parse failed: %s" % exc) from exc
    if doc is None:
        return PresetSet(presets={}, version=1)
    if not isinstance(doc, dict):
        raise SpeechPresetError(
            "root must be a mapping, got %s" % type(doc).__name__)
    presets_raw = doc.get("presets", [])
    if not isinstance(presets_raw, list):
        raise SpeechPresetError("presets must be a list")
    out: Dict[str, SpeechPreset] = {}
    for i, row in enumerate(presets_raw):
        if not isinstance(row, dict):
            raise SpeechPresetError("presets[%d] not a mapping" % i)
        try:
            pid = row["id"]
            text = row["text"]
        except KeyError as exc:
            raise SpeechPresetError(
                "presets[%d] missing %s" % (i, exc)) from exc
        voice = row.get("voice", "male")
        if voice not in ("male", "female"):
            raise SpeechPresetError(
                "presets[%d].voice=%r not in {male, female}"
                % (i, voice))
        if not isinstance(pid, str) or not pid:
            raise SpeechPresetError(
                "presets[%d].id must be non-empty string" % i)
        if pid in out:
            raise SpeechPresetError(
                "presets[%d].id %r duplicates an earlier entry" % (i, pid))
        out[pid] = SpeechPreset(id=pid, text=text, voice=voice)
    return PresetSet(presets=out, version=1)


def build_announce_sequence(
    preset_set: PresetSet,
    fail_item_names: List[str],
    max_items: int,
) -> List[SpeechPreset]:
    """BIT-03 fail_only policy: play `bit_head` then one preset per
    failing item (up to max_items), then `bit_tail_continue`.

    Missing presets are silently dropped (a preset yaml that lacks
    'bit_head' produces a shorter but still-valid announce sequence)."""
    seq: List[SpeechPreset] = []
    head = preset_set.presets.get("bit_head")
    if head:
        seq.append(head)
    for name in fail_item_names[:max_items]:
        p = preset_set.presets.get("bit_fail_" + name)
        if p:
            seq.append(p)
    tail = preset_set.presets.get("bit_tail_continue")
    if tail:
        seq.append(tail)
    return seq
