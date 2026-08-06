"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: __init__.py
Brief: CFG-10 startup schema check (type + range + required) and 19 schema assets

Description:
CFG-FZ-17. This package is the schema layer the freeze oneshot runs BEFORE it
builds the L0~L5 overlay and therefore before startup assertion A. Its whole
reason to exist is ordering: a wrong-TYPE value (t_lat_s written as the string
"0.4") that reaches assertion G blows up as `"0.4" >= 0.4`, a TypeError, and the
oneshot exits on a traceback that names the assertion, not the key. Running a
type/required/range pass first turns that into "safety/brake.yaml:
common.safety.t_lat_s: expected number, got string", which is a key path an
operator can act on. 10 S5.4.6 CFG-10 (grep anchor: "启动时 schema 校验") mandates
exactly this, and 10 S5.4.6 keeps CFG-10 and CFG-11 as two separate layers.

The three collaborators this package does NOT overlap with:

  * assertion A / M -- null-unassigned and missing-shared-key. This check passes
    null on purpose (it runs first, so A still gets to name the calibration gap).
  * assertion G / CFG-11 -- the safety RANGES (SP-1..SP-11, S-1..S-6), authority
    12 S12.1. Cross-key and safety bounds live there; this layer is per-key type
    and structure. The range mechanism here is for pure domain bounds only.
  * check_namespace (layers.py, CFG-FZ-16) -- which layer may write which prefix.

Delivered as a library, tested without the freeze service on disk -- the same
independence check_namespace has. validate_config(rel_path, tree) takes an
already-parsed tree, so the mutation tests build trees in memory.

Public surface:

  validate_config(rel_path, tree)  look a file's schema up by config-relative
                                   path and validate its parsed tree. Raises
                                   SchemaError (E_CONFIG_INVALID) on failure.
  validate_tree(schema, tree)      validate against an explicit Schema; used by
                                   tests and by validate_config.
  SchemaError                      the failure type; carries path/expected/actual.
  SCHEMAS / CONFIG_FILES           the 19 assets, and their fixed path set.
  Schema / FieldSpec               the asset types.
  num/integer/text/boolean/mapping/listof/anything   FieldSpec constructors.
  NUMBER/INTEGER/STRING/BOOLEAN/MAPPING/LIST/ANY/TYPE_TOKENS   the type tokens.

Naming note: on disk this is xbrain/common/config/schemas/ (Python runtime under
xbrain/, CLAUDE.md 0.2 and U65). The deployed top-level common/ is for generated
artifacts only and holds no schema source.
"""

# Re-export the engine and the assets so a caller writes one import line. The
# star-safety __all__ below is explicit because a linter deletes an imported name
# that is not used locally, and these are the package's public half, not dead
# code -- the same argument the errors package makes for its __all__.
from .registry import CONFIG_FILES, SCHEMAS, validate_config
from .spec import (ANY, BOOLEAN, INTEGER, LIST, MAPPING, NUMBER, STRING,
                   TYPE_TOKENS, FieldSpec, Schema, SchemaError, anything,
                   boolean, integer, listof, mapping, num, text, validate_tree)

__all__ = [
    # engine
    "validate_config", "validate_tree", "SchemaError", "Schema", "FieldSpec",
    # constructors
    "num", "integer", "text", "boolean", "mapping", "listof", "anything",
    # tokens
    "NUMBER", "INTEGER", "STRING", "BOOLEAN", "MAPPING", "LIST", "ANY",
    "TYPE_TOKENS",
    # assets
    "SCHEMAS", "CONFIG_FILES",
]
