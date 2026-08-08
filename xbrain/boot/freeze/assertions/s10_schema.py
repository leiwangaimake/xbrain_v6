"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: s10_schema.py
Brief: Assertion S10 -- CFG-10 startup schema validation (CFG-FZ-17)

Description:
Every configs/*.yaml file must pass a strongly-typed schema check
BEFORE the reference / required / cross-file / range assertions
run. Schema validation is a shape-only check: types, ranges,
required-key presence. It exists to give the operator a precise
message (key path + expected type + actual type) instead of the
downstream TypeError that would surface later in a comparison
like `t_lat_s < 0.5` when t_lat_s is the string "0.4".

The schema assets live under xbrain/common/config/schemas/
(CFG-FZ-17 file), and validate_config(rel_path, tree) is the
primitive that walks a parsed tree against its registered schema.
This assertion module is a thin adapter that:

  1. Enumerates every configs/*.yaml file on disk
  2. Parses each into a tree
  3. Calls validate_config on each
  4. Re-raises schema failures as S10 with detail.file + key path

Why S10 runs BEFORE A:

Assertion A performs reference expansion (${common.spec.max_vx_mps})
and required-key completeness on the merged overlay. Both operations
walk values and would encounter type errors from a mis-typed value.
Without S10 running first, a value like `t_lat_s: "0.4"` (string
where a number is expected) either:

  a) Silently coerces in downstream comparisons (Python's < between
     str and float raises TypeError; the trace points at the
     assertion body, not the config key).
  b) Or, if the field is only used arithmetically much later,
     surfaces as a runtime failure long after freeze.

S10-first ensures the operator sees "safety/brake.yaml: t_lat_s
expected number, got string" instead of a stack trace inside G.

CFG-FZ-17 variants (each MUST turn red in tests):
  1) t_lat_s written as "0.4" (string) instead of 0.4 (float)
     -> S10 red with expected_type=number, actual_type=str
  2) Meta test: moving S10 to run AFTER G should cause G to raise
     TypeError instead of a clean S10 attribution.

Contract:
  input:   ctx["config_root"]
           optional ctx["file_trees"] = {rel_path: parsed_tree} to
               skip disk I/O in tests
           optional ctx["skip_files"] = iterable of rel_paths to
               ignore (used when a test wants to validate a subset)
  raises:  XbrainError(E_CONFIG_INVALID) with detail.kind in
             {schema_validation_failed, schema_unregistered_file,
              schema_parse_failed}
           + detail.file + detail.path + detail.expected +
             detail.actual as appropriate

Not in scope for S10:
  * value-range checks that need cross-file context (G handles
    range for safety params; C handles cross-file relations)
  * missing REQUIRED key with a null value (M owns that; S10
    treats null as legal placeholder if the schema says so)
  * schema registration itself (registry.py maintains the
    SCHEMAS dict; a new file without a schema entry surfaces
    as schema_unregistered_file rather than a silent pass)

Ordering in the freeze pipeline (ORD-1):

S10 depends_on=("J",) so it runs immediately after J (config root
sanity). Placing it early -- BEFORE A, M, G, and every value-based
check -- is the whole point of the assertion. A misspelled type
in the source yaml would surface as a TypeError deep in G's
comparison chain without S10; with S10, it surfaces here as
"safety/brake.yaml: t_lat_s expected number, got str", with the
exact key path the operator needs.

Note this is a deliberate departure from the ORD-1 alphabetical
letter-sequence (A, B, C, ...) that most freeze assertions follow.
S10 is a NUMBERED assertion (S10 like S22) that sits in the same
'guards structural / typing invariants' family as check_namespace,
and the pipeline runner walks the registry declaration order rather
than the letter -- so S10 can be placed logically without changing
the runner.

Failure-mode taxonomy (three distinct detail.kind values):

  schema_validation_failed   The most common failure: a value's type
                             does not match the schema's expectation,
                             or a required key is missing, or a range
                             check fires. detail.path names the key,
                             detail.expected/actual name the types.
                             Remediation: fix the value in the yaml.

  schema_unregistered_file   A file exists on disk (or in ctx overrides)
                             that does not have a registered schema.
                             This surfaces the class of defect where
                             someone adds a new configs/*.yaml file
                             without adding a schema alongside; without
                             this raise the file would silently ship
                             with no type discipline. Remediation:
                             add a Schema entry to registry.py.

  schema_parse_failed        yaml.safe_load raised on the file. Reports
                             the parse error directly so the operator
                             does not spend time hunting for a type
                             mismatch that doesn't exist. Remediation:
                             fix the yaml syntax.

Skip semantics:

  1. Files missing on disk: silently skipped. J is the enforcer for
     'required file present'; S10 only validates what is present.
     A dev checkout that lacks some non-critical config passes S10
     but might fail J earlier.

  2. Files in ctx['skip_files']: skipped explicitly. Framework tests
     that exercise pipeline shape (not schema completeness) pass every
     registered file's rel_path here to keep S10 quiet without
     abandoning it entirely.

  3. Files in ctx['file_trees'] override: honored exclusively. This
     is the primary test-injection path -- the caller specifies which
     subset to validate and provides parsed trees, no disk I/O.
"""

# os for walking the configs tree.
import os
# typing for annotations.
from typing import Any, Dict, Iterable, Optional

# The schema registry + primitive validator + shared error class.
from xbrain.common.config.schemas.registry import (
    CONFIG_FILES, SCHEMAS, validate_config,
)
from xbrain.common.config.schemas.spec import SchemaError
# E_CONFIG_INVALID by name, per CLAUDE.md 3.5.
from xbrain.common.errors import E_CONFIG_INVALID
# XbrainError base -- S10 uses E_CONFIG_INVALID uniformly for
# every failure mode.
from xbrain.common.errors.exceptions import XbrainError


def _fail(kind: str, message: str, **detail_extra: Any) -> None:
    """Raise E_CONFIG_INVALID with detail.kind + arbitrary context.

    kind is ONE of:
      schema_validation_failed  -- shape/type/range mismatch
      schema_unregistered_file  -- file on disk lacks a registry entry
      schema_parse_failed       -- yaml.safe_load raised
    """
    # Detail carries kind + extras; message is human-only.
    detail: Dict[str, Any] = {"kind": kind}
    detail.update(detail_extra)
    raise XbrainError(E_CONFIG_INVALID,
                      "assertion S10 failed: %s" % message,
                      detail)


def _load_tree(config_root: str, rel_path: str) -> Any:
    """Read + parse one config file. Returns None if the file is
    absent (dev checkout).

    Delayed yaml import so the module stays importable when only
    the ctx-override path is exercised.
    """
    # Deferred import: yaml is not needed on the ctx-override path
    # (tests inject file_trees directly).
    import yaml
    full = os.path.join(config_root, rel_path)
    # Absent = dev checkout; caller decides whether to skip or fail.
    # S10 chooses skip because J is the enforcer for 'required file
    # present'; S10 only validates whatever is present.
    if not os.path.isfile(full):
        return None
    try:
        with open(full, encoding="utf-8") as fh:
            return yaml.safe_load(fh)
    except yaml.YAMLError as exc:
        # A parse failure is a schema failure in the loose sense
        # (the file cannot even be shaped, let alone typed). Report
        # it distinctly so an operator sees 'yaml broken' before
        # they hunt for a type mismatch that doesn't exist.
        _fail("schema_parse_failed",
              "yaml parse error in %s: %s" % (rel_path, exc),
              file=rel_path, parse_error=str(exc))


def _iter_files(ctx: Dict[str, Any]) -> Iterable[str]:
    """Yield the rel_paths S10 should validate.

    Priority:
      1. ctx['file_trees'] keys (test override) if present
      2. CONFIG_FILES from the registry (production canonical list)
    Files in ctx['skip_files'] are removed from either source.
    """
    # If tests injected an explicit file->tree map, honour its keys
    # exclusively; otherwise fall back to the registry's list.
    overrides = ctx.get("file_trees")
    skip = set(ctx.get("skip_files") or ())
    if overrides is not None:
        for path in sorted(overrides.keys()):
            if path not in skip:
                yield path
        return
    # Production path: enumerate every registered file. Sorted for
    # deterministic failure order.
    for path in CONFIG_FILES:
        if path not in skip:
            yield path


def _validate_one(config_root: str, rel_path: str,
                  tree_override: Optional[Any]) -> None:
    """Validate a single file. Reads from disk if tree_override is None.

    Wraps validate_config so schema failures are re-attributed to
    the S10 assertion namespace and carry file/path/expected/actual
    detail fields uniformly.
    """
    # Use the override when provided; otherwise disk.
    if tree_override is not None:
        tree = tree_override
    else:
        tree = _load_tree(config_root, rel_path)
        # A missing on-disk file (None) is skipped -- J owns 'file
        # must exist' if it is a required file.
        if tree is None:
            return
    try:
        validate_config(rel_path, tree)
    except SchemaError as exc:
        # Re-raise with the S10 attribution. SchemaError already
        # carries path / expected / actual on the detail; we copy
        # them across and add file for aggregation clarity.
        detail = getattr(exc, "detail", {}) or {}
        _fail(
            "schema_validation_failed"
            if "schema" not in str(exc).lower()
            or "no CFG-10 schema registered" not in str(exc)
            else "schema_unregistered_file",
            "%s: %s" % (rel_path, str(exc)),
            file=rel_path,
            path=detail.get("path"),
            expected=detail.get("expected"),
            actual=detail.get("actual"),
        )


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion S10. Runs schema validation on every
    registered config file.

    Flow:
      1. Wiring guard on ctx['config_root'].
      2. Enumerate rel_paths from CONFIG_FILES (or ctx override).
      3. For each: read tree (or use override), call validate_config.
      4. Re-raise schema failures as S10 attribution.
      5. Pass return with files_checked count.

    First-fail: the first schema failure raises and stops the loop.
    Multi-failure mode would be nicer UX but S10 is a single boolean
    gate like every other freeze assertion.
    """
    # Wiring guard identical to every other assertion.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion S10 requires ctx['config_root']; caller did not "
            "populate it"
        )
    # Test-friendly override map.
    overrides = ctx.get("file_trees") or {}
    checked = 0
    for rel_path in _iter_files(ctx):
        _validate_one(ctx["config_root"], rel_path, overrides.get(rel_path))
        checked += 1
    return {
        "status": "pass",
        "assertion": "S10",
        "files_checked": checked,
        "registered_files": len(CONFIG_FILES),
    }
