"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: speech_presets.py
Brief: Loader/validator for configs/speech_presets.yaml (GWY-P4-29)

Description:
What this solves. The preset speech library is the closed set behind D08
speak_preset and the G31 query; a preset that exists in the yaml but fails the
id-form rule would be taught to the LLM in one shape and validated in another,
and a voice value outside the device's two-gender set ([31] byte: male/female)
would fail at the 8519 link with no hint of which entry was wrong. This loader
makes every such defect a startup error naming the entry, in the same fail-loud
style as the intent registry (16 S5.3 ID-1..ID-3).

What each entry must satisfy (GWY-P4-29 + 16 S5.3 ID-2 + 14 S7.3.3):
  * preset_id  -- present, `p-<slug>` form (ID-2: 预设语句 is one of the six
                  geo-object prefixes; the existing validate_geo_object_id is
                  reused rather than a second regex that would drift), unique.
  * text       -- present, non-empty (an empty preset would "play" silence and
                  read as a dead speaker).
  * voice      -- present, in {male, female}: the GZH-2 TTS has exactly two
                  gender values ([31] 性别字节). Not defaulted -- GWY-P4-29 puts
                  the voice ON the preset precisely so nothing else needs one.

What this does NOT do:
  * It does not synthesise or check WAVs. The offline pre-synthesis
    (data/speech_presets/{preset_id}.wav + manifest, 11 S8.8.2) is P2's Phase 2
    job; this table is its input.
  * It does not enforce "D08 has no voice slot" -- that is an intents.yaml
    property and its test lives with the registry tests (the GWY-P4-29 (2)
    criterion), not in this loader.

Worked examples (each BAD line is a real mutation the tests inject):
  OK   {preset_id: p-warn_leave, text: "...", voice: male}
  BAD  {preset_id: warn_01, ...}        -- the 14 S7.3.3 sketch key form; ID-2
       requires p-<slug>, and accepting both would teach the LLM one shape and
       validate another. The shared geo-id validator rejects it.
  BAD  {preset_id: w-gate_east, ...}    -- a VALID geo id, wrong prefix: a
       waypoint is not a preset, and the p- check on top of the validator is
       what stops the whole six-prefix family from leaking in here.
  BAD  {voice: robot}                    -- off the device's two-gender set;
       raised, never coerced (a coerced default gender would make every preset
       "work" while sounding wrong, which nobody reports as a bug).
  BAD  two rows sharing p-warn_leave     -- the second silently shadows the
       first in any dict view; refused by name instead.
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple

from xbrain.common.errors import E_CONFIG_INVALID, XbrainError

from xbrain.p4_agent.registry.geo_id import validate_geo_object_id

__all__ = ["SpeechPreset", "SpeechPresetError", "load_speech_presets", "VOICES"]

#: The device's two TTS gender values ([31] byte; male -> 0). A closed pair, not
#: a growing enum -- the hardware has exactly these.
VOICES: Tuple[str, ...] = ("male", "female")


class SpeechPresetError(XbrainError):
    """A speech_presets.yaml defect. Raised at load, naming the entry, so a bad
    table refuses startup instead of failing at the first D08 utterance."""

    def __init__(self, message: str):
        # Mirror the intent-registry error shape. The code comes from the
        # common.errors export, never a string literal (CLAUDE.md 3.5; the
        # no_literal_ecode lint caught exactly that in this file's first draft).
        super().__init__(E_CONFIG_INVALID, message)


@dataclass(frozen=True)
class SpeechPreset:
    """One preset sentence: id (p- form), verbatim text, device voice gender.

    Frozen, like every registry row: a consumer that could mutate a preset
    in place would fork it from the WAV that was pre-synthesised from it.
    """

    preset_id: str      # p-<slug> (16 S5.3 ID-2), the D08 slot value
    text: str           # the sentence, verbatim from 14 S7.3.3
    voice: str          # male | female -- the GZH-2 [31] gender byte


def load_speech_presets(mapping: dict) -> List[SpeechPreset]:
    """Validate a parsed speech_presets.yaml mapping into SpeechPreset rows.

    Takes the parsed dict (not a path) so tests can feed mutations without
    touching disk -- same pattern as load_intent_registry. Raises
    SpeechPresetError naming the offending entry on ANY defect; never skips or
    coerces a bad row (a silently-dropped preset is a command that stops
    working with no error anywhere).

    The validation contract, in check order (each check names what breaks if it
    were missing -- these are the mutations test_speech_presets.py injects):

      1. 'presets' key present   -- the comment-only skeleton parses to None;
         without this check that None would TypeError three lines later with no
         mention of which file or key was at fault.
      2. non-empty list          -- an empty library makes D08 and the G31 query
         vacuous while the config LOOKS filled in; refusing is what keeps
         "configured" and "usable" the same state.
      3. field presence per row  -- checked before any value logic so a missing
         key names entry+field instead of surfacing as a bare KeyError.
      4. geo-id form             -- via the SHARED validator; a second local
         regex here is exactly how two id grammars drift apart.
      5. p- prefix on top        -- the shared validator accepts all six geo
         prefixes; this narrows to presets so a waypoint id cannot pose as one.
      6. id uniqueness           -- a duplicate's second row silently shadows
         the first in any {id: row} view built downstream.
      7. non-blank text          -- an empty sentence "plays" silence; the field
         symptom would be a speaker presumed dead, not a config presumed wrong.
      8. voice in VOICES         -- the device byte has two values; anything
         else must refuse here, not fail at the 8519 link with no entry name.

    Order matters only in that presence (3) precedes value checks (4..8); the
    value checks are independent of each other.
    """
    if not isinstance(mapping, dict) or "presets" not in mapping:
        # A comment-only skeleton parses to None; a file without the key is the
        # same defect. Name the key so the fix is obvious.
        raise SpeechPresetError("speech_presets.yaml has no 'presets' key")
    rows = mapping["presets"]
    if not isinstance(rows, list) or not rows:
        # An empty library would make D08/G31 vacuous while looking configured.
        raise SpeechPresetError("'presets' must be a non-empty list")

    out: List[SpeechPreset] = []
    seen: Dict[str, int] = {}                       # preset_id -> first index
    for i, row in enumerate(rows):
        if not isinstance(row, dict):
            raise SpeechPresetError("presets[%d] is not a mapping" % i)
        # Field presence first, so a missing key names itself rather than
        # surfacing as a KeyError with no entry context.
        for field in ("preset_id", "text", "voice"):
            if field not in row:
                raise SpeechPresetError("presets[%d] missing '%s'" % (i, field))
        pid, text, voice = row["preset_id"], row["text"], row["voice"]
        # ID-2 p- form, via the shared validator (one regex, no drift). It
        # accepts all six geo prefixes, so the preset-specific half -- must be
        # the p- prefix -- is asserted here on top.
        validate_geo_object_id(pid)
        if not pid.startswith("p-"):
            raise SpeechPresetError(
                "presets[%d] id %r is a geo-object id but not a preset (p-) id"
                % (i, pid))
        if pid in seen:
            # Two rows with one id: the second would silently shadow the first
            # in any dict view of the library.
            raise SpeechPresetError(
                "duplicate preset_id %r (presets[%d] and presets[%d])"
                % (pid, seen[pid], i))
        seen[pid] = i
        if not isinstance(text, str) or not text.strip():
            raise SpeechPresetError("presets[%d] (%s) has empty text" % (i, pid))
        if voice not in VOICES:
            # Off the device's two-gender set: raise, never coerce to a default
            # gender (the same no-silent-degrade rule the closed sets use).
            raise SpeechPresetError(
                "presets[%d] (%s) voice %r not in %s" % (i, pid, voice, list(VOICES)))
        out.append(SpeechPreset(pid, text, voice))
    return out
