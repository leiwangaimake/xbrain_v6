"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: inventory.py
Brief: MANIFEST.layers[] rows with per-layer sha256, and the config_rev over them

Description:
Answers the question an operator asks after an incident: WHICH configuration
was this machine running, and which file changed since the last boot. Without
this module MANIFEST.layers[] stays an empty list and MANIFEST.config_rev
carries a placeholder string, so 10 S5.4.6 "配置版本可审计" (CFG-41, verbatim
"MANIFEST.config_rev + 逐层 sha256 + 启动事件 detail.kind = config_digest")
has no landing point at all -- the row is in the design table and nothing
computes it.

Two identities live in this system and they are deliberately different in
scope. Confusing them is the main hazard here:
  * common_digest (xbrain/common/digest/) hashes the RESOLVED common.* subtree.
    It GATES MOTION: 10 S5.4.4 makes P2 hold "禁止运动" on a mismatch. A change
    to a per-process private section must NOT move it -- 10 S5.4.4 verbatim
    "[*] 只覆盖 common.*, 不含进程私有段 -- 私有段变更不应阻塞放行".
  * config_rev (this module) hashes the LAYER SOURCE FILES as they sit on disk.
    It gates NOTHING; it is the audit trail. A comment-only edit moves it, and
    that is intended -- "did anyone touch configs/" is the question it answers.

Why config_rev is content-derived rather than the "<timestamp>+g<git sha>"
shape of the sample value in 10 S5.4.6:
  * a timestamp changes on every boot, so two boots of the SAME configuration
    produce two different revs and the value can no longer answer "is this the
    same config as yesterday";
  * a git sha is blind to an operator's uncommitted edit of configs/common.yaml
    -- and that edit is exactly the case this exists to catch. An identity that
    cannot see the change it is deployed to detect is fail-silent, CLAUDE.md
    S3.2 form 1 ("一条永远绿的断言").
The "layers-" prefix keeps it visibly distinct from common_digest in an
incident report. Both are sixteen hex characters, and an investigator who
mixed the two up would chase the wrong half of the system.

What this module does NOT do:
  * it does not parse YAML. It hashes RAW BYTES, so two files that differ by a
    comment hash differently. Hashing the parsed tree instead would make the
    identity depend on the loader's normalisation, and two genuinely different
    files could then share a rev.
  * it does not hash the MERGED tree. A merged-tree hash cannot say WHICH file
    changed, and that half of the answer is the actionable one.
  * it does not decide which files the freeze reads. It MIRRORS the loader in
    _layer_loader.py. The mirror is pinned by tests/boot/freeze/test_inventory.py,
    which OBSERVES the paths load_layers actually opens and compares them to
    this enumerator. Making _read_dir call this enumerator would make that test
    true by construction and unfalsifiable (CLAUDE.md S3.2 form 7, "定义式冒充
    实测结论"), so the duplicated walk is deliberate and the test is what
    guarantees the two stay equal.

Scope boundary, stated so nobody has to infer it: layers[] covers L1..L5 --
the OVERLAY axis -- and stops there. The L6 per-process sources (p2_core.yaml
and its siblings) get no layer row; their identity is MANIFEST.processes[*]
.sha256, taken by the materialiser over the bytes it WROTE. That is the
stronger statement for L6: a comment-only edit of p2_core.yaml changes the
source but changes nothing the process will read, and the snapshot hash
correctly does not move. Putting L6 into layers[] as well would give the same
file two identities that disagree, and an operator would have to know which
one meant what.

A file is listed only when it EXISTS. An absent file gets no row, and its
disappearance is still detected because the row set -- and therefore
config_rev -- changes. Emitting a row with a placeholder hash for a missing
file would be the fail-silent spelling: the row would look like evidence the
file was read.
"""

import hashlib
import os
from typing import Any, Dict, List, Optional

# _LAYER_SOURCES / _NO_VARIANT_LAYERS are imported rather than re-spelled so
# the FILENAMES have exactly one definition: adding an L-layer to the loader
# automatically brings it into the audit trail. The WALK (which files inside a
# directory count, and in what order) is re-implemented below on purpose --
# see the module docstring on why that duplication is the point.
from xbrain.boot.freeze.assertions._layer_loader import (
    _LAYER_SOURCES, _NO_VARIANT_LAYERS, variant_of, variant_sibling,
)
# ENV_WHITELIST is the L5 closed set (10 S5.4.3). Reading it here rather than
# listing three variable names keeps the L5 row honest if the whitelist ever
# changes -- though 10 S5.4.3 forbids extending it, so in practice this is a
# guard against the code drifting, not against the contract drifting.
from xbrain.common.config.layers import ENV_WHITELIST
from xbrain.common.digest.canonical import canonical_json
from xbrain.common.digest.digest import DIGEST_HEX_LEN

#: Levels whose files are marked `locked: true` in MANIFEST.layers[], per the
#: 10 S5.4.4 MANIFEST sample block. L3 is the safety layer (ENV-2: same source
#: on every machine); L4b is per-robot calibration (CAL-*: replaced only by the
#: calibration procedure, never hand-edited). The flag is a statement about who
#: is allowed to change the file, not about filesystem permissions.
_LOCKED_LEVELS = frozenset({"L3", "L4b"})

#: The two picked layers. Their filename comes from a value INSIDE the tree
#: being loaded (common.site_id / common.robot_id), so they cannot be part of
#: _LAYER_SOURCES; fv_org_enu._load_l4_tree / _load_l4b_tree read them with
#: exactly this path shape and with NO variant sibling (10 S5.4.7 applies the
#: variant to L1/L2 and to L6, not to the picked layers). Mirroring that
#: "no sibling" behaviour matters: adding one here would list a file the
#: freeze never actually read.
_PICKED_LAYERS = (
    ("L4", "sites"),
    ("L4b", "calib"),
)

#: The L5 row has no file and no hash -- 10 S5.4.4 writes it as
#: `{ level: "L5", path: "env", keys: [...] }`. This constant is the literal
#: that appears in the `path` field so the row is greppable in a MANIFEST.
_ENV_PATH = "env"


def file_sha256(path: str) -> str:
    """Full 64-character sha256 of the file's raw bytes.

    Full length, not truncated to DIGEST_HEX_LEN: a layer row is read by a
    human comparing two MANIFESTs, and truncation buys nothing here while
    costing the ability to paste the value into `sha256sum -c`. The 16-char
    truncation belongs to common_digest, where the value is carried inside
    every startup event and length is a wire cost.
    """
    # Read whole, in binary. Config files are kilobytes; a chunked loop would
    # add a branch that no test could distinguish from this one.
    #
    # Binary mode is load-bearing, not a habit. Text mode would decode with the
    # locale codec and normalise newlines, so the SAME file would hash
    # differently on a machine with a different locale, and a CRLF line ending
    # introduced by an editor on a laptop would become invisible. Both would
    # show up as "config_rev did not move after an edit" -- the one symptom
    # this module exists to prevent.
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def _row(level: str, path: str) -> Dict[str, Any]:
    """One MANIFEST.layers[] entry for an existing file."""
    row: Dict[str, Any] = {
        # Level first so a human scanning the JSON reads the layer name before
        # the long path -- the field order survives into the file because
        # run_freeze dumps with sort_keys=False for nested structures.
        "level": level,
        # Absolute. A MANIFEST is read off-machine during a post-mortem, and a
        # relative path would resolve against whatever cwd the reader happens
        # to have -- systemd gives the freeze unit its own, and nobody reading
        # the file afterwards knows what it was.
        "path": os.path.abspath(path),
        # Hash the file as it is NOW, at freeze time. Re-hashing later (in
        # --check, or in a P-process) and finding a different value is exactly
        # the signal: the source was edited after the snapshot was taken.
        "sha256": file_sha256(path),
    }
    if level in _LOCKED_LEVELS:
        row["locked"] = True
    return row


def _base_yaml_files(dir_path: str) -> List[str]:
    """The BASE *.yaml files a directory layer contributes, in read order.

    Mirrors _layer_loader._read_dir's filters one for one:
      * only *.yaml (README / schema files are common noise in these dirs);
      * skip names starting with '_' (repo convention: placeholder skeletons,
        whose null values would otherwise enter the tree);
      * skip variant-suffixed names (X_sim.yaml is an OVERLAY on X.yaml, never
        a layer member in its own right) -- they are attached to their base
        below, so an X_sim.yaml with no X.yaml is read by nobody and listed by
        nobody, which is the loader's behaviour too;
      * sorted() because deep_merge is last-wins and the order therefore
        carries precedence.
    A missing directory contributes nothing rather than raising: L2/L3 may be
    empty in a dev checkout, and the loader treats that the same way.
    """
    if not os.path.isdir(dir_path):
        return []
    out: List[str] = []
    for name in sorted(os.listdir(dir_path)):
        if not name.endswith(".yaml"):
            continue
        if name.startswith("_"):
            continue
        if variant_of(name) is not None:
            continue
        out.append(os.path.join(dir_path, name))
    return out


def _with_sibling(base_path: str, variant: Optional[str]) -> List[str]:
    """[base] plus its X_<variant>.yaml sibling when one exists on disk.

    The base itself is included even when absent from disk; _extend below
    drops non-existent paths. That split matters for L1 under a variant: the
    loader reads common_sim.yaml as an overlay whether or not common.yaml is
    there, so the sibling has to be reachable independently of the base.
    """
    paths = [base_path]
    if variant is not None:
        sib = variant_sibling(base_path, variant)
        if os.path.isfile(sib):
            paths.append(sib)
    return paths


def _extend(rows: List[Dict[str, Any]], level: str, paths: List[str]) -> None:
    """Append a row for each path that is a real file. See the module
    docstring on why a missing file gets no row at all."""
    for path in paths:
        # isfile, not exists: a DIRECTORY named common.yaml would satisfy
        # exists() and then blow up inside file_sha256 with an IsADirectoryError
        # that names no layer. Skipping it here means the row set changes and
        # the operator sees a missing layer instead of a stack trace.
        if os.path.isfile(path):
            rows.append(_row(level, path))


def _env_row(env: Optional[Dict[str, str]]) -> Dict[str, Any]:
    """The L5 row: which whitelisted variables were set, never their values.

    Names only, matching the 10 S5.4.4 sample. The VALUE of, say,
    XBRAIN_SITE_ID is already inside the resolved tree and therefore already
    inside common_digest; recording it again here would create a second source
    for the same fact, and MED-S5 (11 S7.4.9) is emphatic that the layer
    trail must not become a place where environment content accumulates.

    An env change is still detected: XBRAIN_SITE_ID selects the L4 file, so a
    different site produces a different L4 row path, and config_rev moves.
    """
    source = os.environ if env is None else env
    return {
        "level": "L5",
        "path": _ENV_PATH,
        # Iterate the ENVIRONMENT and filter by the whitelist, not the other
        # way round. Walking ENV_WHITELIST (a frozenset of str) would make the
        # pre-sort order depend on PYTHONHASHSEED, which is randomised per
        # process -- so dropping the sorted() would produce a DIFFERENT
        # config_rev on each boot from an unchanged configuration, and no test
        # could reliably catch it because the bug only shows on the seeds
        # where the set happens to come out unsorted. This spelling puts the
        # order in the caller's hands, which makes the sort observable.
        # sorted() itself is what guarantees byte-stability: os.environ
        # iterates in insertion order, and a systemd unit and an interactive
        # shell do not build the environment in the same order.
        "keys": sorted(name for name in source if name in ENV_WHITELIST),
    }


def layer_rows(config_root: str, *,
               variant: Optional[str] = None,
               site_id: Optional[str] = None,
               robot_id: Optional[str] = None,
               env: Optional[Dict[str, str]] = None) -> List[Dict[str, Any]]:
    """Build MANIFEST.layers[] for one freeze pass.

    site_id / robot_id come from the RESOLVED tree, not from the raw L1 file:
    L5 can override either, and the picked layer must name the file that was
    actually read. Passing the raw value would list sites/<L1 value>.yaml while
    the freeze had merged sites/<env value>.yaml -- a MANIFEST that names the
    wrong file is worse than one that names none.
    """
    rows: List[Dict[str, Any]] = []
    # Declaration order of _LAYER_SOURCES is the overlay precedence order
    # (L1 lowest). Emitting rows in that order makes the MANIFEST readable
    # top-to-bottom as "who wins over whom", which is the question an operator
    # brings to this list.
    for level, kind, frag in _LAYER_SOURCES:
        full = os.path.join(config_root, frag)
        # ENV-2 / 10 S5.4.7: the safety layer takes no variant, so its base
        # files are listed without a sibling lookup.
        layer_variant = None if level in _NO_VARIANT_LAYERS else variant
        if kind == "file":
            bases = [full]
        elif kind == "dir":
            bases = _base_yaml_files(full)
        else:
            # Same shape as load_layers' own guard: an unknown kind is a
            # construction bug in _LAYER_SOURCES, not a config problem, so it
            # gets a plain AssertionError rather than an XbrainError that a
            # dashboard would file under "operator error".
            raise AssertionError("unknown layer kind %r for %s" % (kind, level))
        for base in bases:
            _extend(rows, level, _with_sibling(base, layer_variant))
    for level, subdir in _PICKED_LAYERS:
        # Falsy (None or "") means the picker value was never assigned, and
        # _load_l4_tree / _load_l4b_tree return {} for that case -- no file was
        # read, so no row.
        picked = site_id if level == "L4" else robot_id
        if not picked:
            continue
        # No path traversal guard here on purpose: site_id / robot_id come out
        # of the resolved tree, which assertion D has already checked for
        # identity consistency, and a value that escaped the directory would
        # simply fail isfile() in _extend and produce no row. Adding a guard
        # would suggest this function is a trust boundary; it is not.
        _extend(rows, level,
                [os.path.join(config_root, subdir, "%s.yaml" % picked)])
    rows.append(_env_row(env))
    return rows


def config_rev_of(rows: List[Dict[str, Any]]) -> str:
    """CFG-41 config_rev: one identity over the whole layer inventory.

    Serialised through canonical_json rather than a local json.dumps so there
    is ONE canonicalisation rule in this repository (10 S5.4.4). A second
    serialiser here would be a second thing to keep byte-compatible with the
    C++ side for no gain, and the first symptom of a drift would be two
    machines reporting different revs for the same configs.

    Wrapped in {"layers": rows} rather than hashing the bare list: canonical
    JSON of a list and of a single-key object differ, and the wrapper leaves
    room to grow the input without silently changing every previously recorded
    rev under the same spelling.
    """
    # list(rows) copies the sequence so a caller handing in a live list cannot
    # have it mutated by the serialiser, and so a tuple and a list of the same
    # rows give the same rev (canonical_json serialises both as a JSON array,
    # but the copy makes the intent explicit rather than incidental).
    text = canonical_json({"layers": list(rows)})
    # UTF-8 encoded once, on the whole string, for the same reason
    # digest.common_digest does it that way: the digest is defined over UTF-8
    # bytes, and spreading the encode across the serialiser would put the
    # choice of codec in more than one place.
    full = hashlib.sha256(text.encode("utf-8")).hexdigest()
    # Truncated to DIGEST_HEX_LEN, the same sixteen characters common_digest
    # uses -- one length constant for both identities, so a future change to
    # the digest width cannot leave the two disagreeing.
    return "layers-" + full[:DIGEST_HEX_LEN]
