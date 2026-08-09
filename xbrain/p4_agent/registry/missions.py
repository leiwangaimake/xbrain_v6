"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: missions.py
Brief: Loader for the 11 mission prompts (GWY-P4-11) -- binds each file's
       emitted-intent set to the 16 S6.7 group table at load time

Description:
What this solves. A mission prompt teaches the LLM which intents it may emit;
the GBNF slice (GWY-P4-12, Phase 2) enforces the same set mechanically. If the
prompt on disk drifts from the 16 S6.7 group table -- an intent dropped from an
example, or a stray one taught by a new example -- the drift surfaces as
misrouted commands at runtime, with nothing pointing at the file. This loader
makes the drift a startup error naming the mission and the differing names.

How emission is measured (and why exactly this way). A mission's emitted set is
the union of two teaching shapes, intersected with the registry names:
  * example emissions -- "intent":"NAME" inside the few-shot JSON outputs;
  * rule teachings   -- an arrow form, `-> NAME` (the prompts write e.g.
    "...(arrow) patrol_repeat, 不填 route"), which teaches emission exactly
    like an example does (M3's patrol_repeat and M4's ptz_stop_track are
    taught ONLY this way).
A plain whole-word scan was tried first and is WRONG here, in two measured
ways: M4's prompt uses `hold` as a `behavior` SLOT VALUE (and hold is also
intent A04), and M8's category closed set contains `estop` (also intent A01) --
both would count as emissions under a bare word scan. A slot value is not an
emission; the two shapes above are how the prompt actually licenses an intent
token in the OUTPUT position. unknown/out_of_scope are excluded -- they are the
+1 of the criterion's "closed set + 1", not business intents.

The criterion (GWY-P4-11 (2)): per mission, emitted-set size + 1 (unknown) <= 5;
the single exception is M4_follow = 6 (16 S6.7 group table, 唯一破例). The
loader asserts BOTH that cap and exact equality with the per-mission expected
set -- equality is what catches a stray extra teaching that still fits the cap.

M9 / M10 are validated for existence and mechanism only:
  * M9_clarify is the template family -- it fills slots for an already-chosen
    intent and emits none of its own;
  * M10_fallback's emission is the per-turn top-K candidate line (U47f: <= 4,
    R4: empty productions not merged) -- DYNAMIC, so its static text carries
    example candidate ids that are NOT its emission set; scanning it would
    count teaching examples as emissions and always fail. The mechanism line
    (the ke-xuan candidate-line contract, U+53EF U+9009) is asserted instead.

KNOWN GAP, registered not papered over (CLAUDE.md 3.2): the 16 S6.7 group table
lists G11 query_events_period under M8_events, but the M8 prompt (rule 3) folds
period queries into query_events_recent's unit/n and never emits G11. Until 16
resolves this (fix the prompt or fix the table), EXPECTED_EMISSIONS reflects
what the prompt actually teaches, and KNOWN_GAPS carries the delta with its
reason; a regression test keeps the gap visible.
"""

import os
import re
from typing import Dict, FrozenSet, Iterable, List, Tuple

from xbrain.common.errors import E_CONFIG_INVALID, XbrainError

__all__ = ["MissionError", "load_missions", "emitted_intents",
           "EXPECTED_EMISSIONS", "KNOWN_GAPS", "MISSIONS", "MISSION_CAP"]

#: The 11 mission group keys, verbatim from 16 S6.7 (v0.8: 10 -> 11 with
#: M6b_mark). File name = group key + .txt (16 S3630 文件名 = mission 组 key).
#: A tuple, not read from the directory: an eleventh-file-missing must FAIL,
#: and globbing the directory would happily "load" whatever ten files remain.
#: The files themselves carry no header comment -- their bytes go straight
#: into the LLM context, so provenance lives in the README and here.
MISSIONS: Tuple[str, ...] = (
    "M1_translate", "M2_turn", "M3_nav", "M4_follow", "M5_speak",
    "M6_naming", "M6b_mark", "M7_objref", "M8_events", "M9_clarify",
    "M10_fallback",
)

#: Per-mission emitted-intent sets, derived from the 16 S6.7 group table's
#: 覆盖指令 column (ids folded to names via configs/intents.yaml), MINUS the
#: registered gaps below. None marks the two non-emitting groups (see module
#: docstring). Kept as names, not ids: the prompt text carries names.
EXPECTED_EMISSIONS: Dict[str, FrozenSet[str]] = {
    "M1_translate": frozenset({"move_forward", "move_backward",
                               "move_left", "move_right"}),          # A05-A08
    "M2_turn": frozenset({"turn_left", "turn_right",
                          "turn_around", "face_heading"}),           # A09-A12
    "M3_nav": frozenset({"goto_waypoint", "patrol_route",
                         "patrol_schedule", "patrol_repeat"}),       # B01-B04
    # The one 6-total (5 + unknown) exception the group table marks 唯一破例.
    "M4_follow": frozenset({"follow_target", "stop_follow", "set_motion_behavior",
                            "ptz_track", "ptz_stop_track"}),         # B11 B12 C07 E04 E05
    "M5_speak": frozenset({"speak_preset", "speak_custom"}),         # D08 D09
    "M6_naming": frozenset({"record_route_save", "record_fence_save",
                            "rename_object"}),                       # F03 F09 F14
    "M6b_mark": frozenset({"record_waypoint", "record_dock"}),       # F06 F10
    "M7_objref": frozenset({"delete_route", "delete_waypoint",
                            "delete_fence", "set_active_fence"}),    # F11 F12 F13 F15
    # G11 query_events_period is in the doc table but NOT here: KNOWN_GAPS.
    "M8_events": frozenset({"query_events_recent", "query_events_by_type",
                            "query_event_detail", "generate_report"}),  # G10 G12 G14 H02
    "M9_clarify": None,                  # template family: emits nothing of its own
    "M10_fallback": None,                # dynamic top-K (U47f <= 4); see docstring
}

#: mission -> (missing-intent-name, why). The register of doc-vs-prompt deltas;
#: emptied only by fixing 16 (either side), never by loosening the loader.
#: A gap here is a DECISION pending in the doc, not a tolerance: the regression
#: test asserts both that the entry exists and that the prompt still lacks the
#: name, so whichever side of 16 gets fixed, the fixer is forced to clear this
#: register in the same change -- a gap cannot silently rot in place.
KNOWN_GAPS: Dict[str, Tuple[str, str]] = {
    "M8_events": ("query_events_period",
                  "16 S6.7 table lists G11 under M8, but the M8 prompt rule 3 "
                  "folds period queries into query_events_recent unit/n and "
                  "never emits G11; registered 2026-08-07 (GWY-P4-11)"),
}

#: The criterion cap on emitted names (+1 unknown makes the contract's 5/6).
#: Stored as the single exception plus a default, not a full table: writing 11
#: rows of "4" invites editing one row instead of asking why the budget moved.
MISSION_CAP: Dict[str, int] = {"M4_follow": 5}      # every other mission: 4
_DEFAULT_CAP = 4


class MissionError(XbrainError):
    """A mission-prompt defect the pipeline must not start on.

    One type for every refusal, same shape as IntentRegistryError: the caller
    response is always "do not start", and a type per cause would invite
    catching one kind and carrying on with a half-loaded prompt set.
    """

    def __init__(self, message: str):
        # Closed-set code from the shared export, never a literal (CLAUDE.md 3.5).
        super().__init__(E_CONFIG_INVALID, message)


def emitted_intents(text: str, registry_names: Iterable[str]) -> FrozenSet[str]:
    """The registry intent names the prompt text TEACHES AS OUTPUT.

    Union of the two teaching shapes (module docstring: example emissions and
    arrow-rule teachings), intersected with the registry so a stray token that
    is not an intent can never count. Deliberately NOT a whole-word scan --
    that was measured to count M4's `hold` slot value and M8's `estop`
    category value as emissions, which they are not.
    """
    # "intent":"NAME" -- the few-shot output position.
    found = set(re.findall(r'"intent"\s*:\s*"([a-z_]+)"', text))
    # -> NAME (either ASCII or U+2192 arrow) -- the rule-teaching position.
    found |= set(re.findall(r"(?:\u2192|->)\s*([a-z_]+)", text))
    return frozenset(found & set(registry_names))


def load_missions(directory: str, registry_names: Iterable[str]) -> Dict[str, str]:
    """Load and validate all 11 mission prompts; return {mission: text}.

    registry_names is the intent closed set (the loaded registry's names), so
    this loader has no second copy of it. Raises MissionError naming the
    mission on: a missing/empty file; an emitted set differing from
    EXPECTED_EMISSIONS (either direction -- a dropped teaching and a stray one
    are both routing drift); a set over the criterion cap; or M10 missing its
    candidate-line mechanism.

    The check order per mission, and what each refusal means downstream:

      1. file exists    -- absent file = a mission the triage can route to but
         the pipeline cannot load; at runtime that is an utterance answered
         with an internal error, so it refuses at startup instead (criterion 4).
      2. non-empty      -- an empty prompt "loads" and then classifies nothing;
         the runtime symptom would be every utterance of that group falling to
         unknown, which reads as a model problem, not a config one.
      3. emission == expected -- BOTH directions, because the fixes differ: a
         missing name means a teaching was dropped (re-add the rule/example);
         a stray name means the prompt quietly teaches a neighbouring group's
         intent (remove it, or the same utterance classifies differently
         depending on which mission happened to load).
      4. cap            -- the criterion's own budget line (emitted + 1 <= 5,
         M4 6). Unreachable while check 3 holds, but kept独立: if someone
         grows EXPECTED_EMISSIONS past the budget, the cap must still fail
         rather than follow the table upward.
      5. M10 mechanism  -- its emission is per-turn dynamic, so the only
         static thing that CAN be held is the candidate-line contract.

    Worked example of a drift refusal:
      M2_turn missing turn_around  ->  "M2_turn emitted-intent drift:
      missing=['turn_around'] stray=[]"  -- the fixer re-adds the teaching or,
      if 16 S6.7 genuinely moved the intent, updates EXPECTED_EMISSIONS in the
      same commit that changes the doc (the pair must move together).
    """
    names = list(registry_names)
    out: Dict[str, str] = {}
    for mission in MISSIONS:
        path = os.path.join(directory, mission + ".txt")
        if not os.path.isfile(path):
            # 4 of the criterion: absent file refuses with E_CONFIG_INVALID.
            raise MissionError("mission prompt missing: %s" % path)
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        if not text.strip():
            # An empty prompt "loads" and routes nothing -- refuse by name.
            raise MissionError("mission prompt empty: %s" % mission)
        expected = EXPECTED_EMISSIONS[mission]
        if expected is not None:
            got = emitted_intents(text, names)
            if got != expected:
                # Both directions named: the fix differs (re-add a dropped
                # teaching vs remove a stray one), so the message must too.
                raise MissionError(
                    "%s emitted-intent drift: missing=%s stray=%s"
                    % (mission, sorted(expected - got), sorted(got - expected)))
            cap = MISSION_CAP.get(mission, _DEFAULT_CAP)
            if len(got) > cap:
                # +1 (unknown) puts the closed set over the contract's 5 (6 for
                # M4). Unreachable while expected == got holds, but the cap is
                # the CRITERION's own line and must fail independently if the
                # expected table above is ever grown past it.
                raise MissionError(
                    "%s emits %d intents; cap is %d (+1 unknown = the 16 S6.7 "
                    "budget)" % (mission, len(got), cap))
        elif mission == "M10_fallback":
            # The dynamic group: its contract is the candidate line. Without
            # the ke-xuan marker the top-K mechanism (U47f/R4) cannot anchor.
            if "可选" not in text:
                raise MissionError(  # NO-CHINESE-LOG-LINT: names a CJK doc marker
                    "M10_fallback lost its 可选 candidate-line contract")
        out[mission] = text
    return out
