"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: intents.py
Brief: The intent registry loader and its load-time refusals (ID-1, ID-3, CS-A1)

Description:
What this solves. P4 routes 128 intents, and every one must carry the four
facts a downstream stage needs before it will act: which 18 id it is, how it is
routed (16 S6.6), what confirmation level it executes at (16 S8.3A.2 / 18
S13.1), and which slots it fills (16 S6.6). A registry missing any of those for
any intent is not a smaller registry -- it is one that will route a phrase and
then find, at execution time, that it does not know whether to confirm first.
So this refuses to start on a hole and names the key, rather than starting and
discovering the hole on the phrase (16 S5.3 ID-1, GWY-P4-07 criterion 1).

Where the data comes from, and where it does NOT. The registry content lives in
configs/intents.yaml, migrated from 16 S6.6 (route/slots) and 16 S8.3A.2 (auth).
This module reads a MAPPING or a product path handed to it; it never names the
configuration source and never reads docs/temp/_*. The 128-name closed set that
CS-A1 checks against is likewise INJECTED (from the future configs/cmdset_18.json,
GWY-P4-09) -- see check_intents_in_closed_set for why it cannot be self-derived.

The three ID rules, and the one subtlety in ID-3. ID-1 (fields present) and the
closed-set checks are whole-registry properties. ID-3 forbids a `direction` (or
`unit`) slot, but ONLY on the eight chassis relative-move intents of 11
S9.3.2A.4 (MI-1): for those, direction is carried by the intent NAME
(move_forward vs move_backward vs turn_left ...), so a direction slot would
resurrect the v0.1 relative_move{direction,...} pseudo-intent that MI-1 deleted.
The PTZ pair E01 ptz_move / E06 ptz_zoom legitimately carry `direction` -- pan/
tilt/zoom is a real degree of freedom, not a name-encoded one -- so ID-3 is
scoped to the MI-1 set and must NOT be a blanket "no direction slot anywhere",
which would reject the real registry (ID-2 itself lives in geo_id.py).

What looks right but is wrong (recorded so the next author does not redo it):
  * a blanket direction-slot ban rejects E01/E06 and the whole registry fails
    to load -- scope ID-3 to MI1_MOTION_INTENTS;
  * folding H01/H03 to a single auth loses the deep/force_step branch: the
    registry keeps auth_by_slot for them, and this loader validates those levels
    too, so a typo there refuses startup rather than surfacing at S8.3A.2 later;
  * `0`/`""`/`[]` are assigned values, only None (a yaml `null`) and an absent
    key are holes -- reporting an empty slot list as a hole would send someone
    to fill in a key that is already correct.
"""

import re
from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, Iterable, List, Mapping, Optional, Tuple

# The refusal code and the base exception both come from the shared library, so
# a literal E_* never appears here (CLAUDE.md 3.5, no_literal_ecode.py).
from ...common.errors import E_CONFIG_INVALID
from ...common.errors.exceptions import XbrainError

# --- Closed sets, all defined by 16 / 18 / 11 and kept local -----------------
#
# None of these is a contract closed set from 11 S13, so none belongs in
# xbrain/common/enums/ (whose whole value is that a metatest diffs it back
# against 11). They are written here with their anchors, the same choice
# config/loader.py makes for HISTORY_SCENARIOS. Each is a tuple where order is
# printed in a message, and a frozenset where only membership is asked.

#: The confirmation levels, 16 S0.3.1 / S8.3A.2 (U53 split the old L1 into
#: L1a/L1b). A tuple so a failure message prints them in one fixed order --
#: an unordered set would print two identical failures differently.
CONFIRM_LEVELS: Tuple[str, ...] = ("L0", "L1a", "L1b", "L2", "L3")

#: The triage routes, 16 S6.6. Order is fastest to slowest, which is also the
#: latency pairing 16 S6.6.1 keys off (bypass->T0, fastpath->T2, ... ).
ROUTES: Tuple[str, ...] = ("bypass", "fastpath", "fastpath_then_llm", "llm")

#: The eight chassis relative-move intents, 11 S9.3.2A.4. This IS the MI-1
#: domain: direction is encoded in the name, so none of these may take a
#: direction slot. A frozenset because ID-3 asks membership, not order.
MI1_MOTION_INTENTS: FrozenSet[str] = frozenset({
    "move_forward", "move_backward", "move_left", "move_right",
    "turn_left", "turn_right", "turn_around", "face_heading"})

#: Slot names ID-3 forbids on an MI-1 intent, 16 S5.3: direction and unit are
#: carried by the intent name / fixed by it, so they never enter slots. (16 S5.3
#: also names gear/"档位", but no gear slot exists in the 18 set, so the two
#: concrete forbidden names are these.) A frozenset for the same reason.
NAME_ENCODED_SLOTS: FrozenSet[str] = frozenset({"direction", "unit"})

#: The 18 id shape: one class letter A-J, then two digits (16 S6.6 / 18 S2).
INTENT_ID_RE = re.compile(r"^[A-J][0-9]{2}$")

#: The four fields every entry must carry (16 S5.3 ID-1 / GWY-P4-07 criterion 1).
#: A tuple, iterated in this order so the missing-key report is deterministic.
REQUIRED_FIELDS: Tuple[str, ...] = ("id", "route", "auth", "slots")

#: The top-level key intents.yaml groups the registry under.
TOP_KEY = "intents"


# --- The validation contract (why five checks, and what each one is for) ------
#
# The registry is the typed surface every later P4 stage reads: the intent
# router, the GBNF grammar builder, the confirmation-level gate. A wrong entry
# here is not a local bug -- it becomes a command the robot mis-routes, a level
# that silently downgrades a confirmation, or a direction slot on an intent whose
# direction was supposed to be fixed by its name. So the file's job is to refuse
# a bad registry at load time, with the whole reason, rather than let any of that
# reach a stage that can act on it.
#
# The five checks and the specific failure each stops:
#
#   check_fields_present      (ID-1)  -- an entry missing id/route/auth/slots.
#       A router keyed on a missing route defaults it somewhere; a missing auth
#       is a confirmation level nobody chose. The message prints the key path,
#       because "a field is missing" over 128 rows is a hand-diff.
#
#   check_closed_sets                 -- route/auth/id/auth_by_slot out of set.
#       11 S13.6: an out-of-set value is refused, never read as the nearest known
#       one. A route typo "fastpath_llm" must not be silently coerced to
#       "fastpath_then_llm" -- that is a different latency budget.
#
#   check_no_name_encoded_slot (ID-3) -- a direction/unit slot on an MI-1 intent.
#       11 S9.3.2A.3 MI-1: the eight chassis relative-move intents carry direction
#       IN THE NAME. A direction slot on move_forward resurrects the deleted
#       relative_move{direction,...} shape, the exact ambiguity MI-1 removed.
#       Scoped to MI1_MOTION_INTENTS so PTZ E01/E06, which DO take a direction
#       slot legitimately, are not swept up.
#
#   check_intents_in_closed_set (CS-A1) -- the name set must EQUAL the 18 set.
#       16 S0.5 is a two-way check. A subset test (every registered name is in
#       18) stays green when a name was BOTH dropped here AND never in 18 -- the
#       forgotten-half failure GWY-P4-08's metatest exists to catch. So both
#       directions are reported.
#
# WHY the checks return lists instead of raising. load_intent_registry runs them
# all and reports every problem at once. A checker that raised on the first hole
# would surface one per restart, which trains an operator to ask for a
# start-anyway switch -- CLAUDE.md 3.6 forbids exactly that switch. Each check is
# also wired to its own mutation test (GWY-P4-08), so each must stand alone.
#
# WHY these closed sets live here and not in xbrain/common/enums/. None is a
# contract closed set from 11 S13, so the enums metatest (which diffs enums back
# against 11) has nothing to diff them against. Their authority is 16/18/11, cited
# on each constant above; the same call config/loader.py makes for its own local
# sets. This is consistent with U66: a closed set's home is wherever its single
# authoritative source is, not a blanket "all enums in one file".
#
# --- Reference: what route and auth MEAN (so a reviewer can spot a wrong one) --
#
# route decides HOW an utterance for this intent is turned into a command, and
# the four values are ordered fastest-to-slowest because that is also the latency
# class 16 S6.6.1 pairs each with:
#   bypass            -- straight to the safety path, no NLU at all. The three
#                        estop-class intents (A01/A02/A03) only. Fastest, and the
#                        one route where the LLM being down must not matter.
#   fastpath          -- deterministic keyword/pinyin/fuzzy match to one intent,
#                        no LLM. The 92 commands whose surface forms are known and
#                        finite ("stop", "go to A", "turn left").
#   fastpath_then_llm -- try the fastpath; fall through to the LLM only if it does
#                        not match. The 20 commands that have common fixed forms
#                        AND a long tail of paraphrases.
#   llm               -- always the LLM. The 13 commands that are open-ended
#                        enough (a described destination, a free-text query) that
#                        no finite keyword set covers them.
# A route typo therefore is not cosmetic: it moves a command between "answered in
# milliseconds without the model" and "answered only if the model is up".
#
# auth is the confirmation level the command must clear before it acts, 16
# S8.3A.2 (U53 split the old single L1 into L1a/L1b when it found the half-duplex
# audio gate makes both spoken-veto windows zero, so the two must be named apart):
#   L0   -- act immediately, no confirmation (a query, a light toggle).
#   L1a  -- announce-then-act, action and the TTS run in parallel (U53).
#   L1b  -- announce-then-act, but the announcement is a heads-up, NOT a stop
#           window -- the half-duplex gate means there is no spoken veto (U53).
#   L2   -- an explicit second confirmation is required before acting.
#   L3   -- the highest bar; a deliberate, unambiguous re-confirmation.
# A wrong auth is the quiet-dangerous failure: an L3 command written L0 acts with
# no confirmation at all, and nothing downstream re-derives the level -- the
# registry is the single place it is decided, which is why a typo here refuses
# startup rather than being caught later.
#
# Worked examples (what passes, what this file refuses):
#   OK      go_to:        {id: B05, route: fastpath, auth: L0, slots: [waypoint]}
#   OK      estop:        {id: A01, route: bypass,   auth: L0, slots: []}
#   ID-1    <anything>:    a missing route/auth/slots/id -> the key path is named.
#   set     move_forward: {..., slots: [direction]} -> ID-3: move_forward is MI-1,
#           its direction is in the name; a direction slot is the deleted
#           relative_move shape.
#   closed  go_to:        {..., route: fastpath_llm} -> not one of the four
#           routes; reported, never coerced to fastpath_then_llm.
#   CS-A1   a name here that 18 does not have, OR an 18 name absent here -> both
#           directions are the failure, because 16 S0.5 is a two-way equality.


class IntentRegistryError(XbrainError):
    """An intent registry that must not be started on.

    One type for every refusal in this file, because the caller's response is
    always the same -- do not start -- and a type per cause would offer a choice
    that does not exist and invite someone to catch one kind and carry on.
    Distinct from config.P4ConfigError only by owner: that one means the snapshot
    or the freeze line is wrong, this one means the registry CONTENT is wrong.
    Both derive from XbrainError so a caller may still treat them alike.
    """

    def __init__(self, message: str):
        super().__init__(message)
        # E_CONFIG_INVALID is what 10 S5.4.4 assigns to configuration that fails
        # its own self-check: startup refused, not retryable, not degraded
        # through. Imported, never a literal (CLAUDE.md 3.5).
        self.code = E_CONFIG_INVALID


@dataclass(frozen=True)
class IntentEntry:
    """One validated registry row. Frozen so a consumer cannot mutate the
    shared registry after load.

    Built only by _build_entries, which runs AFTER every check has passed, so
    the type can assume its fields are in-set: this is the typed surface the
    rest of P4 reads, and it never has to re-validate.
    """

    name: str                       # the intent name, i.e. the yaml key
    id: str                         # 18 id, e.g. "A05"
    route: str                      # member of ROUTES
    auth: str                       # base confirmation level, member of CONFIRM_LEVELS
    slots: Tuple[str, ...]          # slot names, in file order
    # Present only for the two slot-level-split intents (H01, H03); None for the
    # other 126, where auth alone is the whole story. Not {} -- an empty mapping
    # would read as "split, but into nothing", which is a different claim.
    auth_by_slot: Optional[Mapping[str, Mapping[Any, str]]] = None


class IntentRegistry:
    """The loaded registry: name -> IntentEntry, plus the id index.

    Deliberately thin. Counts (per-route, per-level) are computed on demand and
    never stored or written down (CLAUDE.md 3.7); accessors beyond what a caller
    needs are not added, because every stored field is a place a default could
    later hide.
    """

    __slots__ = ("_by_name", "_by_id")

    def __init__(self, entries: Iterable[IntentEntry]):
        # Two indexes, both built once here from the same entry list so they
        # cannot drift. by_id is what 16 S8.3A registers against (level is per
        # id row); by_name is what the router matches on a parsed phrase.
        self._by_name: Dict[str, IntentEntry] = {}
        self._by_id: Dict[str, IntentEntry] = {}
        for e in entries:
            # Both maps take the same object, so a later reader sees one row.
            self._by_name[e.name] = e
            self._by_id[e.id] = e

    @property
    def names(self) -> FrozenSet[str]:
        """The registered intent names. This IS the CS-A1 key set."""
        # Frozen so a caller cannot grow the set it was handed for a diff.
        return frozenset(self._by_name)

    @property
    def entries(self) -> Tuple[IntentEntry, ...]:
        """All entries, in no promised order."""
        # A tuple, immutable: the registry is shared, and a list would let one
        # reader reorder or drop rows for every other.
        return tuple(self._by_name.values())

    def by_name(self, name: str) -> IntentEntry:
        """The entry for an intent name, or KeyError -- never a default."""
        # KeyError on purpose: a missing name is a caller bug, and a default
        # would hand back a row that does not exist as if it did.
        return self._by_name[name]

    def by_id(self, intent_id: str) -> IntentEntry:
        """The entry for an 18 id, or KeyError -- never a default."""
        return self._by_id[intent_id]

    def route_histogram(self) -> Dict[str, int]:
        """route -> count, computed now. For the asserters and reports; the
        number is never copied into a comment or a doc (CLAUDE.md 3.7)."""
        # Seeded with every route at zero so a route with no members still
        # appears -- a missing key would read as "route unknown", not "empty".
        out = {r: 0 for r in ROUTES}
        for e in self._by_name.values():
            out[e.route] += 1
        return out

    def auth_histogram(self) -> Dict[str, int]:
        """base-auth level -> count, computed now (per id, so H01/H03 count once
        at their default level). The 130-row histogram is a DIFFERENT quantity;
        expand auth_by_slot to get it. Not stored, not written down."""
        out = {lv: 0 for lv in CONFIRM_LEVELS}
        for e in self._by_name.values():
            out[e.auth] += 1
        return out


# --- the individual checks, each returning problems rather than raising -------
#
# Returning a list lets load_intent_registry aggregate every problem and report
# them at once (a checker that surfaces one hole per restart trains people to
# ask for a switch that starts anyway, which CLAUDE.md 3.6 forbids). Each is
# also driven directly by a mutation test, so each must stand alone.
#
# Mutation coverage (CLAUDE.md 3.3 -- an assertion never seen red is unwritten).
# Each check has a mutant in test_intents.py / GWY-P4-08 that MUST turn it red,
# and a positive case beside it that a do-nothing checker would fail:
#   check_fields_present        drop an entry's `auth` -> must report that key.
#   check_closed_sets           set a route to "fastpath_llm" -> must report it,
#                               and NOT coerce it to the nearest known route.
#   check_no_name_encoded_slot  give move_forward a `direction` slot -> must
#                               report it; give it to PTZ E01 -> must NOT (scope).
#   check_intents_in_closed_set drop one name from the registry with the oracle
#                               unchanged -> the reverse-difference must fire;
#                               this is the half a subset test silently passes.
# A check that stayed green under its mutant would be the empty-shell failure
# (CLAUDE.md 3.2 form 1) this whole file exists to keep out of the registry.


def _missing(entry: Any, field: str) -> bool:
    """Whether `field` is a hole in `entry`: absent, or present but null.

    `is None`, never `not v`: 0 / "" / [] / false are assigned values, and an
    empty slots list is legitimate (a bypass intent has no slots). Treating
    those as holes is the exact mistake config.check_no_unassigned_keys documents.
    """
    # A non-mapping entry has no fields at all; treat every field as a hole so
    # the caller reports the entry once rather than crashing on entry[field].
    if not isinstance(entry, Mapping):
        return True
    return field not in entry or entry[field] is None


def check_fields_present(raw: Mapping[str, Any]) -> List[str]:
    """ID-1 / criterion 1: every entry carries id, route, auth, slots.

    The key path is in every message (intents.<name>.<field>) because 'a field
    is missing' without saying which key leaves the reader to diff 128 rows by
    hand -- and the criterion says verbatim to print the key path.
    """
    problems: List[str] = []
    for name, entry in raw.items():
        # A non-mapping value (e.g. `foo: bar`, or the empty `foo:` that yaml
        # reads as None) cannot carry the four fields. Reported once here rather
        # than four times in the loop below.
        if not isinstance(entry, Mapping):
            problems.append(
                "%s.%s is %r, not a mapping of {id, route, auth, slots}"
                % (TOP_KEY, name, entry))
            continue
        # Iterated in REQUIRED_FIELDS order so the report is deterministic.
        for field in REQUIRED_FIELDS:
            if _missing(entry, field):
                problems.append("%s.%s.%s is missing or null (ID-1, 16 S5.3)"
                                % (TOP_KEY, name, field))
    return problems


def check_closed_sets(raw: Mapping[str, Any]) -> List[str]:
    """route, auth, id shape, and any auth_by_slot levels are all in-set.

    Out-of-set values are reported here rather than passed through or read as
    the nearest known one (11 S13.6). Defensive on absence: a missing field is
    ID-1's finding, so this skips it rather than reporting the same hole twice.
    """
    problems: List[str] = []
    for name, entry in raw.items():
        # Non-mapping entries are check_fields_present's finding; skipping here
        # keeps one malformed entry from being reported by two checks at once.
        if not isinstance(entry, Mapping):
            continue
        # route: one of the four triage values. Guarded on `is not None` so an
        # ABSENT route is left to ID-1 (its finding), and only a PRESENT but
        # wrong value is reported here -- otherwise the same hole prints twice.
        route = entry.get("route")
        if route is not None and route not in ROUTES:
            # Reported, never coerced to the nearest known route: "fastpath_llm"
            # must not become "fastpath_then_llm", a different latency budget.
            problems.append("%s.%s.route %r is outside the route closed set %s "
                            "(16 S6.6)" % (TOP_KEY, name, route, list(ROUTES)))
        # auth: one of the five confirmation levels. A wrong level here is a
        # silently downgraded confirmation -- an L3 command running at L0.
        auth = entry.get("auth")
        if auth is not None and auth not in CONFIRM_LEVELS:
            problems.append("%s.%s.auth %r is outside the confirmation-level "
                            "closed set %s (16 S8.3A.2)"
                            % (TOP_KEY, name, auth, list(CONFIRM_LEVELS)))
        # id: the A-J plus two digits shape. str() first so a yaml int (e.g. an
        # unquoted id that parses as a number) cannot slip past the regex.
        intent_id = entry.get("id")
        if intent_id is not None and not INTENT_ID_RE.match(str(intent_id)):
            problems.append("%s.%s.id %r is not an 18 id (one letter A-J plus "
                            "two digits, 16 S6.6)" % (TOP_KEY, name, intent_id))
        # auth_by_slot is optional, but if present its leaf values are also
        # confirmation levels: a typo there is a level that S8.3A.2's per-row
        # expansion would otherwise carry silently into the two split intents.
        abs_ = entry.get("auth_by_slot")
        if isinstance(abs_, Mapping):
            for slot, mapping in abs_.items():
                if not isinstance(mapping, Mapping):
                    continue  # malformed shape is not this check's job
                for slot_val, level in mapping.items():
                    if level not in CONFIRM_LEVELS:
                        problems.append(
                            "%s.%s.auth_by_slot.%s.%s level %r is outside %s "
                            "(16 S8.3A.2)" % (TOP_KEY, name, slot, slot_val,
                                              level, list(CONFIRM_LEVELS)))
    return problems


def check_no_name_encoded_slot(raw: Mapping[str, Any]) -> List[str]:
    """ID-3 / criteria 3+4: no direction/unit slot on an MI-1 motion intent.

    Scoped to MI1_MOTION_INTENTS on purpose. The rule is 'direction is carried
    by the intent name' (11 S9.3.2A.3 MI-1), true only for the eight chassis
    relative-move intents; PTZ E01/E06 carry a direction slot correctly, and a
    blanket ban would reject them and the whole registry. The mutation that must
    turn this red is move_forward -> [direction, amount, unit]: that resurrects
    relative_move, and it is caught because move_forward IS in the MI-1 set.
    """
    problems: List[str] = []
    for name, entry in raw.items():
        # Only the eight MI-1 intents are in scope; everything else (incl. the
        # PTZ intents that DO carry a direction slot) skips. Scoping here rather
        # than banning the slot names everywhere is what keeps E01/E06 legal.
        if not isinstance(entry, Mapping) or name not in MI1_MOTION_INTENTS:
            continue
        slots = entry.get("slots")
        # A missing or malformed slots list is ID-1's finding, not this check's;
        # skipping keeps the same hole from being reported by two checks.
        if not isinstance(slots, (list, tuple)):
            continue
        # Any of the name-encoded slot names present on a motion intent is the
        # defect -- direction/unit belong to the name, so their appearance as a
        # slot is the relative_move shape MI-1 removed, resurfacing.
        for bad in NAME_ENCODED_SLOTS:
            if bad in slots:
                problems.append(
                    "%s.%s.slots contains %r, but %s is an MI-1 chassis-motion "
                    "intent whose direction is carried by the intent name, not "
                    "a slot (ID-3, 16 S5.3 / 11 S9.3.2A.3 MI-1). This is the "
                    "relative_move{direction,...} shape MI-1 deleted"
                    % (TOP_KEY, name, bad, name))
    return problems


def check_intents_in_closed_set(names: Iterable[str],
                                cmdset_names: Iterable[str]) -> List[str]:
    """CS-A1 / criterion 5: the registry name set EQUALS the 18 closed set.

    *** Bidirectional, and the direction people forget is the one that matters.
    A subset test (every registered name is in the closed set) passes when a
    name was BOTH dropped from the registry AND never in the closed set -- and a
    superset test misses the mirror. 16 S0.5 CS-A1 is verbatim a two-way check;
    the metatest for it (GWY-P4-08) exists precisely to catch a one-way rewrite.
    So both differences are reported.

    *** Why cmdset_names is injected and not derived here. The 128-name closed
    set is a property of 18, not of this file, and its machine-readable form
    (configs/cmdset_18.json, produced by the GWY-P4-09 extractor from the 18
    tables) does not exist yet. Deriving the closed set from the registry itself
    would be circular -- adding a bad name would just grow the set it is checked
    against, and nothing would ever be caught (CLAUDE.md 3.2 form 7, a rule that
    cannot be falsified). So the oracle is passed in.
    """
    reg = set(names)
    cmd = set(cmdset_names)
    problems: List[str] = []
    # Sorted so two runs over the same broken pair read identically.
    only_in_registry = sorted(reg - cmd)
    only_in_cmdset = sorted(cmd - reg)
    # Direction 1: a name the registry has that 18 does not -- a coined name.
    if only_in_registry:
        problems.append(
            "CS-A1 (16 S0.5): %d intent name(s) in the registry are not in the "
            "18 closed set: %s. An intent name's only source is 18's intent "
            "columns; the registry may reference, never coin (16 S5.3 ID-1)"
            % (len(only_in_registry), only_in_registry))
    # Direction 2: a name 18 has that the registry dropped -- the forgotten half.
    if only_in_cmdset:
        problems.append(
            "CS-A1 (16 S0.5): %d intent name(s) in the 18 closed set are absent "
            "from the registry: %s. The registry must carry every command in "
            "the set, not a subset" % (len(only_in_cmdset), only_in_cmdset))
    return problems


def _build_entries(raw: Mapping[str, Any]) -> List[IntentEntry]:
    """Turn the validated raw mapping into typed, frozen entries.

    Called only after every check has passed, so it does not re-validate: it
    trusts that id/route/auth are in-set and slots is a list. Slots become a
    tuple (immutable, order preserved); auth_by_slot is carried through for the
    two split intents and left None otherwise.

    Runs LAST, never interleaved with the checks. Building typed objects and then
    validating them would mean the type had to tolerate half-built rows and
    re-check what the raw-dict checks already proved; validating the raw dict and
    building once here keeps a single source of truth for "is this row legal".
    """
    out: List[IntentEntry] = []
    for name, entry in raw.items():
        # Direct field access (entry["id"]), not .get: every field was proven
        # present by check_fields_present, so a KeyError here would be a real bug
        # in the check order, not a config problem to swallow.
        out.append(IntentEntry(
            name=name,
            id=entry["id"],
            route=entry["route"],
            auth=entry["auth"],
            slots=tuple(entry["slots"]),
            # .get, not [ ]: absent means "not a split intent", which is None.
            auth_by_slot=entry.get("auth_by_slot")))
    return out


# --- The loading contract (the order is the specification, not a convenience) -
#
# load_intent_registry runs four stages, and the order is load-bearing:
#
#   1. Shape gate. Without a top-level `intents:` mapping there is nothing to
#      iterate, so this RAISES immediately rather than returning an empty
#      problem list -- an empty file is a different failure from a file full of
#      bad entries, and reporting it as "no problems" would be wrong.
#   2. The three raw-dict checks (ID-1, closed sets, ID-3) run and their problem
#      lists are CONCATENATED, not short-circuited. All three see every entry, so
#      one bad row that is missing a field AND carries a wrong route is reported
#      for both -- the operator fixes the file once, not once per boot.
#   3. CS-A1 runs only if a cmdset oracle was supplied. None is an explicit
#      deferral (the machine-readable 18 set, configs/cmdset_18.json, is the later
#      GWY-P4-09 item), NOT a silent skip: a runtime caller MUST pass it once it
#      exists, and the CS-A1 metatest always passes the set derived from the
#      migration source, so the check is exercised even while the oracle file is
#      absent.
#   4. Only when nothing is wrong are the typed, frozen entries built. Building
#      first and validating the typed objects would mean re-deriving what the
#      checks already know; validating the raw dict and building once at the end
#      keeps the two from drifting.
#
# The messages are sorted before raising so two runs over the same broken file
# read identically -- an unordered dump makes one defect look like several, and a
# reviewer comparing two boots would chase a difference that is only ordering.
#
# Nothing in this file names a configuration source or assembles a path: the
# resolved product's path is INJECTED by the caller (the freeze line's output
# under /run/xbrain/resolved/ at runtime, a fixture in a test). That is the
# no_config_source_read rule -- the one guarantee the loader makes about the
# source is that it never reaches for it (10 S5.4.1: processes read the product,
# never the source).
def load_intent_registry(mapping: Mapping[str, Any],
                         cmdset_names: Optional[Iterable[str]] = None
                         ) -> IntentRegistry:
    """Validate a parsed intents mapping and return the typed registry.

    `mapping` is the whole parsed file: it must carry a top-level `intents`
    dict. Order of checks: shape first (is there an intents mapping at all),
    then the raw-dict checks (ID-1, closed sets, ID-3) aggregated together so
    every problem is reported at once, then CS-A1 IF a cmdset was supplied.
    Only if nothing is wrong are the typed entries built.

    *** cmdset_names is the CS-A1 oracle. When None, CS-A1 is NOT enforced --
    a real gap, not a default: the machine-readable 18 closed set
    (configs/cmdset_18.json) is a later item (GWY-P4-09). A runtime caller MUST
    pass it once it exists; the CS-A1 metatest always passes the set derived
    from the migration source.
    """
    # Shape gate: without a top-level intents mapping there is nothing to check.
    if not isinstance(mapping, Mapping) or TOP_KEY not in mapping:
        raise IntentRegistryError(
            "intent registry has no top-level %r mapping; got %s. "
            "configs/intents.yaml groups the entries under `intents:` (16 S5.3)"
            % (TOP_KEY, sorted(mapping) if isinstance(mapping, Mapping)
               else type(mapping).__name__))
    raw = mapping[TOP_KEY]
    # The value under `intents:` must itself be a mapping of name -> fields.
    if not isinstance(raw, Mapping):
        raise IntentRegistryError(
            "intent registry %r is %r, not a mapping of name -> fields"
            % (TOP_KEY, type(raw).__name__))

    # Aggregate every problem the raw-dict checks find, so one restart surfaces
    # all of them rather than one per boot cycle.
    problems: List[str] = []
    problems += check_fields_present(raw)     # ID-1
    problems += check_closed_sets(raw)        # route / auth / id / auth_by_slot
    problems += check_no_name_encoded_slot(raw)  # ID-3
    # CS-A1 only when an oracle was supplied; None is an explicit deferral.
    if cmdset_names is not None:
        problems += check_intents_in_closed_set(raw.keys(), cmdset_names)

    # Sorted so two runs over the same broken file read identically -- an
    # unordered dump makes one defect look like several.
    if problems:
        raise IntentRegistryError(
            "intent registry refused (%d problem(s)):\n  %s"
            % (len(problems), "\n  ".join(sorted(problems))))

    # Nothing wrong: build the typed, frozen surface the rest of P4 reads.
    return IntentRegistry(_build_entries(raw))


def load_intent_registry_from_yaml(path: str,
                                   cmdset_names: Optional[Iterable[str]] = None
                                   ) -> IntentRegistry:
    """Read a resolved intents product from `path` and validate it.

    `path` is injected by the caller -- the runtime passes the freeze line's
    product under /run/xbrain/resolved/, a test passes a fixture. This module
    names no configuration source and assembles no path (no_config_source_read.py
    rule 1/2): the one guarantee it makes about the source is that it does not
    reach for it.
    """
    # yaml imported at call time, not module top, for the reason
    # common/config/resolved.py does it: keeping import side effects off the
    # package-import path (10 S3.3.7's W-1 window needs a process to come up far
    # enough to report WHY the stack did not start).
    import yaml  # noqa: PLC0415
    with open(path, encoding="utf-8") as fh:
        mapping = yaml.safe_load(fh)
    return load_intent_registry(mapping, cmdset_names=cmdset_names)
