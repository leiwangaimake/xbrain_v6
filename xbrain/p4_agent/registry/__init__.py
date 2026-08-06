"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: Public surface of P4's intent registry, and nothing else

Description:
What this package is. The intent registry (GWY-P4-07): the loader that turns
configs/intents.yaml into a validated, typed table of the 128 intents, plus the
load-time refusals that own 16 S5.3's three ID rules -- ID-1 (every entry has
id/route/auth/slots), ID-2 (the geographic-object id shape), ID-3 (no
name-encoded direction slot on an MI-1 chassis-motion intent) -- and CS-A1 (the
registry name set equals the 18 closed set).

Why import runs nothing and reads nothing. Importing this package must be free
of side effects, for the reason config/__init__.py gives: 10 S3.3.7's W-1
observation window depends on a process coming up far enough to report WHY the
stack did not start, and a package that read a snapshot at import time would
take that ability down with everything else. The registry is loaded by an
explicit call, handed a mapping or a product path.

What lives where, so the next addition lands in the right file:
  * intents.py -- the loader, the closed sets (routes, confirmation levels, the
    MI-1 motion set), the checks and the IntentRegistry/IntentEntry types;
  * geo_id.py -- ID-2 only: the <prefix>-<slug> shape and its validator, kept
    separate because it validates id STRINGS that flow through few-shot and
    mission tables, not fields of the registry itself.

Why the check_* functions are exported alongside the loader. They are what the
mutation tests drive directly (CLAUDE.md 3.3: an assertion with no red mutation
is not written). A test able to call only load_intent_registry would have to
build a whole registry to probe one refusal, and that cost is tests that stop
being written -- config/__init__.py exports its check_* functions for the same
reason.

What this package deliberately does NOT do: it does not name or read the
configuration source (the loader takes an injected path), and it does not
derive the CS-A1 closed set from the registry itself -- that oracle is injected,
because a self-derived set cannot be falsified (see check_intents_in_closed_set).
"""

from .geo_id import (GEO_ID_PREFIXES, GeoIdError, is_valid_geo_object_id,
                     validate_geo_object_id)
from .intents import (CONFIRM_LEVELS, INTENT_ID_RE, MI1_MOTION_INTENTS,
                      NAME_ENCODED_SLOTS, REQUIRED_FIELDS, ROUTES,
                      IntentEntry, IntentRegistry, IntentRegistryError,
                      check_closed_sets, check_fields_present,
                      check_intents_in_closed_set, check_no_name_encoded_slot,
                      load_intent_registry, load_intent_registry_from_yaml)

__all__ = [
    # loader and types
    "IntentRegistry", "IntentEntry", "IntentRegistryError",
    "load_intent_registry", "load_intent_registry_from_yaml",
    # the checks, exported for the mutation tests
    "check_fields_present", "check_closed_sets",
    "check_no_name_encoded_slot", "check_intents_in_closed_set",
    # closed sets and shapes
    "CONFIRM_LEVELS", "ROUTES", "MI1_MOTION_INTENTS", "NAME_ENCODED_SLOTS",
    "INTENT_ID_RE", "REQUIRED_FIELDS",
    # ID-2 geographic-object id
    "GEO_ID_PREFIXES", "GeoIdError",
    "is_valid_geo_object_id", "validate_geo_object_id",
]
