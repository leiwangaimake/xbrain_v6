"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: i_model_and_engine.py
Brief: Assertion I -- TRT engine / model sha256 + build_env agreement
       (CFG-FZ-10)

Description:
The DEFECT this assertion catches has one distinctive property that
justifies making it a bring-up gate rather than a runtime check:
TensorRT engine failures are DELAYED-EXPLOSION defects (11 S11A.4.3
AIR-V1c). A TRT engine built for JetPack 7.1 can be deserialised
under JetPack 7.2 and appear healthy; the mismatch only surfaces at
the first enqueue call -- which happens after boot, after mode
transition, when the robot is already on patrol. Static comparison
at freeze time is the ONLY point where the wrong engine can be
refused without endangering an active mission.

Three sub-checks, all against the on-disk model tree at
<models_root>/{llm,asr,tts,vision}/<name>/current/MODEL.json:

  1. MODEL.json.files[].sha256 vs on-disk sha256 (byte-for-byte).
     Fails: detail.kind = model_digest_mismatch.
     Catches: a corrupted or replaced weight file. Common cause is
     a partial rsync or an interrupted OTA.

  2. For kind=="vision" entries, MODEL.json.build_env fields must
     match the runtime {jetpack, tensorrt, sm} triple literally.
     Fails: detail.kind = engine_env_mismatch.
     Catches: JetPack was OTA-upgraded without rebuilding the TRT
     engine. This is the AIR-V1c case above -- the one that would
     otherwise crash mid-mission.

  3. current symlink exists AND points to an existing dir AND that
     dir contains a parseable MODEL.json.
     Fails: detail.kind = model_missing.
     Catches: switch_current.sh crashed halfway; an admin deleted a
     version dir but forgot the symlink; MODEL.json edit produced
     malformed json.

Contract:
  input:  ctx["config_root"]
          optional ctx["models_root"] to override the default
              path /opt/xbrain/models (used by tests).
          optional ctx["runtime_env"] dict{jetpack, tensorrt, sm}
              to inject runtime version triple (used by tests to
              avoid depending on dpkg / nvidia-smi being present).
              If absent AND at least one kind=='vision' MODEL.json
              is found, runtime_env is probed via _probe_runtime.
              If the probe fails on a host that has no vision
              models to check, the probe is not attempted -- I
              never touches dpkg unless it needs to.
          optional ctx["kind_filter"] iterable to limit checked
              kinds. Default = all four (llm/asr/tts/vision).
  raises: XbrainError(E_CONFIG_INVALID) with detail.kind in
          {model_digest_mismatch, engine_env_mismatch, model_missing}
          + detail.model / detail.path / detail.expected / detail.actual
          as appropriate.

CFG-FZ-10 named variants (each MUST turn red in tests):
  (1) One byte of a weight file changed
      -> model_digest_mismatch
  (2) MODEL.json.build_env.tensorrt changed by one digit
      -> engine_env_mismatch
  (3) current symlink broken (target dir removed)
      -> model_missing

Rationale for reading the ON-DISK tree instead of overlay:

MODEL.json is NOT a config file; it is a build-artifact metadata
descriptor shipped with the engine binary. It lives outside
configs/ (under /opt/xbrain/models/ per 11 S11A.4.1). Overlay
layers L0-L6 do not touch it. Reading it via the config layer
would be an abuse of the layer machinery.

Rationale for a per-model iteration (not one merged tree):

Each model dir is INDEPENDENT. A broken llm model does not affect
vision, and we want the error message to name the specific model
that failed. Iterating and collecting all failures would be nicer
UX than first-fail, but assertion I follows the freeze convention
of first-fail (each assertion is one boolean gate). All-failures
mode is a follow-up if the operator asks for it.

Not in scope for I (handled elsewhere):

  * whether MODEL.json.schema field is "xbrain.model/1" -- schema
    validation belongs in the config layer, not the freeze gate
  * that the engine file name embeds trt<ver>-jp<ver>-sm<arch>
    (11 S11A.4.1) -- name-lint job, not a runtime blocker
  * running the golden frame through the engine -- deep BIT, not
    freeze
  * regenerating a stale engine -- provisioning, not freeze

Ordering inside the freeze pipeline (ORD-1):

I depends_on=("J",) in the registry, meaning it runs after J
(config root sanity) so the models_root path derivation has a
known-good config_root to start from. I is orthogonal to the
config-tree assertions (A/M/B/C/D/E/F/G/N/O/FV-ORG/C-6+MR-1/L)
because it does not touch the config tree; the depends_on chain
is about ordering, not data flow. Placing I late in the pipeline
also means the more expensive sha256 streams (a 2 GB gguf takes
~10s on the ORIN SSD) only run when the cheap config checks are
already green.

Failure-mode taxonomy (why three distinct kinds, not one generic):

The three sub-checks catch three different upstream causes:

  model_digest_mismatch    -> corrupted / partially-transferred
                              artifact. Remediation is 're-run OTA'
                              or 'restore from backup'.
  engine_env_mismatch      -> platform version drift. Remediation
                              is 'rebuild engine with trtexec' or
                              'roll back JetPack'.
  model_missing            -> operator error during a switch or a
                              half-finished install. Remediation is
                              'redeploy the model dir'.

Merging them into a single 'model_bad' kind would force operators
to read the message text to decide the fix. Keeping them split
means /run/xbrain/resolved/MANIFEST.json can carry the kind field
directly and downstream dashboards can categorise without parsing
strings.

Why static comparison instead of loading the engine and probing:

Loading a TRT engine takes several hundred MB of RAM per engine
(the workspace and the deserialised network); doing so for every
model at freeze time would 3x the freeze RAM footprint. The
build_env comparison is O(3 string equality) and pure file I/O.
The one thing it CANNOT catch that a real load could is 'the
engine file itself is corrupted in a way that still passes sha256
but fails to deserialise' -- an extraordinarily rare case. The
sha256 sub-check covers 'corrupted' already; anything left is a
build-time bug that would also affect every other host with the
same engine, so it would be caught in T1/T2/T8 baseline runs
(AIR-V1e) before promotion.

Fail-silent risk removed:

Every path that could 'do nothing and pass' has a distinct signal:

  models_root absent          -> skipped_reason=models_root_absent
  no models of a kind         -> counts_by_kind[kind]=0
  no vision models            -> runtime_env_probed=false
  runtime probe returns None  -> engine_env_mismatch(reason=
                                 runtime_probe_unavailable) if any
                                 vision model exists

An observer who sees vision_seen=true and runtime_env_probed=false
knows something is wrong (contradictory shape) even without a
raise. That is deliberate: the pass return is not just 'green', it
is 'green + these counts'.
"""

# hashlib for the sha256 comparison; hashlib.sha256 is the same
# algorithm used to build MODEL.json.files[].sha256 on the build
# side (11 S11A.4.2), so a byte-for-byte comparison is defined.
import hashlib
# json for MODEL.json parse. json5 would tolerate the doc's example
# comment-style syntax, but the shipped MODEL.json is strict json.
import json
# os for path joins, exists checks, readlink, and symlink target
# resolution (islink + realpath).
import os
# re for probing dpkg -s output (kept only for _probe_runtime).
import re
# subprocess for the runtime probe. Test callers inject runtime_env
# directly to skip subprocess entirely.
import subprocess
# typing for annotations; Optional used on runtime_env because the
# probe may return None on hosts without JetPack.
from typing import Any, Dict, Iterable, List, Optional, Tuple

# XbrainError base -- I uses E_CONFIG_INVALID uniformly (all three
# defects are "config or shipped-artifact is wrong", not runtime
# operational failures).
from xbrain.common.errors import E_CONFIG_INVALID
from xbrain.common.errors.exceptions import XbrainError

# Default runtime model tree root per 11 S11A.4.1 verbatim. Test
# callers override via ctx['models_root'] to point at a tmp_path
# scaffold; production reads from the real deploy path.
_DEFAULT_MODELS_ROOT = "/opt/xbrain/models"

# The four model kinds recognised by 11 S11A.4.1. Any other subdir
# under models_root is silently ignored (dev leftovers, README, etc.)
# so a stray directory does not turn freeze red.
_KIND_DIRS = ("llm", "asr", "tts", "vision")

# The build_env keys that must match the runtime triple for vision
# entries. l4t/cuda are ALSO listed in the doc's MODEL.json example
# but the assertion table (11 S11A.4.2) only names jetpack/tensorrt/sm
# as the runtime-checked triple -- so only those three are enforced
# here. Adding l4t/cuda would be an OVER-enforcement (a check the
# doc does not authorise), which the "assertions are the letter of
# the doc, not the spirit" rule forbids.
_VISION_ENV_KEYS = ("jetpack", "tensorrt", "sm")

# Buffer size for streaming sha256 over large weight files (llm models
# are gigabytes). 64 KiB is enough to saturate SSD read on the target
# platform and avoids the "read whole file into RAM" antipattern that
# would otherwise trip on the 2 GB gguf.
_HASH_CHUNK_BYTES = 64 * 1024


# ---------------------------------------------------------------------------
# Failure helpers
# ---------------------------------------------------------------------------

def _fail(kind: str, message: str, **detail_extra: Any) -> None:
    """Raise E_CONFIG_INVALID with kind in the closed set.

    kind is ONE of:
      model_digest_mismatch  -- file sha256 does not match MODEL.json
      engine_env_mismatch    -- build_env vs runtime disagree
      model_missing          -- current symlink or MODEL.json absent

    detail_extra fields are attached as-is (model, path, expected,
    actual, key, ...) so the failure record is fully triageable
    without reading source.
    """
    # Compose the detail dict; kind is the primary discriminator
    # for the closed set, everything else is context for triage.
    detail: Dict[str, Any] = {"kind": kind}
    detail.update(detail_extra)
    # Prefix the message with 'assertion I failed:' so the log line
    # names the assertion without the caller having to add it.
    raise XbrainError(E_CONFIG_INVALID,
                      "assertion I failed: %s" % message,
                      detail)


# ---------------------------------------------------------------------------
# Runtime probe (JetPack / TensorRT / SM)
# ---------------------------------------------------------------------------

def _probe_dpkg_version(package: str) -> Optional[str]:
    """Run dpkg -s <package> and return the Version: field.

    Returns None on any failure (dpkg absent, package not installed,
    parse mismatch). None is the caller's signal to fall through to
    the assertion's own missing-runtime-info handling.

    Not raising here because a host without dpkg is a legitimate
    dev-machine state; the assertion only cares about vision models,
    and if none exist the probe is never called.
    """
    # Cheap subprocess call; capture stdout, discard stderr, do NOT
    # use shell=True (nothing to interpolate, avoids injection).
    try:
        proc = subprocess.run(
            ["dpkg", "-s", package],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=5, check=False)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0:
        return None
    # dpkg -s emits 'Version: X.Y.Z-...' on its own line. Any first
    # match wins; format is stable across dpkg releases.
    m = re.search(r"^Version:\s*(\S+)",
                  proc.stdout.decode("utf-8", "replace"),
                  re.MULTILINE)
    return m.group(1) if m else None


def _probe_nvidia_sm() -> Optional[str]:
    """Return GPU compute capability (e.g. '87') via nvidia-smi.

    nvidia-smi reports '8.7' with a dot; the doc's build_env writes
    it as '87' (no dot) per 11 S11A.4.2 example. We strip the dot
    so the comparison is against the doc's format.

    Returns None if nvidia-smi is absent or unparseable.
    """
    try:
        proc = subprocess.run(
            ["nvidia-smi", "--query-gpu=compute_cap",
             "--format=csv,noheader"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=5, check=False)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    if proc.returncode != 0:
        return None
    # Take the first non-empty line; strip the dot ('8.7' -> '87').
    lines = [ln.strip() for ln in
             proc.stdout.decode("utf-8", "replace").splitlines()
             if ln.strip()]
    if not lines:
        return None
    return lines[0].replace(".", "")


def _probe_runtime() -> Dict[str, Optional[str]]:
    """Assemble the {jetpack, tensorrt, sm} triple from the host.

    Returns a dict with three keys; any that could not be probed
    are None. The caller decides how to treat None: it is a
    'runtime-info-unavailable' outcome, not a 'runtime says X'.
    """
    # dpkg package name for JetPack. 'nvidia-jetpack' is the meta
    # package that pins the whole stack version (11 S11A.4.1 pins
    # 'JetPack 7.2').
    jetpack = _probe_dpkg_version("nvidia-jetpack")
    # TensorRT bin package. 'tensorrt' is the umbrella; the version
    # field is the TRT release (10.16.2 for JetPack 7.2 per 11 S1.2).
    tensorrt = _probe_dpkg_version("tensorrt")
    # GPU compute capability via nvidia-smi.
    sm = _probe_nvidia_sm()
    return {"jetpack": jetpack, "tensorrt": tensorrt, "sm": sm}


# ---------------------------------------------------------------------------
# Sub-check 3: current symlink + MODEL.json parseability
# ---------------------------------------------------------------------------

def _resolve_current_dir(kind_dir: str, model_name: str) -> str:
    """Return the resolved 'current' target dir under a model root.

    Raises E_CONFIG_INVALID(kind=model_missing) if the symlink is
    absent, the target does not exist, or the target is not a dir.
    """
    # Layout: <kind_dir>/<model_name>/current -> <version_dir>
    # Reference 11 S11A.4.1 verbatim -- 'current -> 1.0.0'.
    current = os.path.join(kind_dir, model_name, "current")
    # islink() returns False for missing paths OR non-symlink files.
    # Both are 'no current symlink' from the operator's view.
    if not os.path.islink(current):
        _fail("model_missing",
              "current symlink absent: %s" % current,
              model=model_name, path=current, reason="no_symlink")
    # Resolve via realpath so relative symlinks (e.g. current -> 1.0.0)
    # work; realpath returns the absolute target regardless.
    target = os.path.realpath(current)
    # Existence check on the resolved target -- symlink points into
    # a hole is the CFG-FZ-10 variant (3) failure mode.
    if not os.path.isdir(target):
        _fail("model_missing",
              "current -> %s does not exist" % target,
              model=model_name, path=current, target=target,
              reason="dangling_symlink")
    # Success: return the resolved dir for the caller to read
    # MODEL.json inside it.
    return target


def _load_model_json(current_dir: str, model_name: str) -> Dict[str, Any]:
    """Read + parse MODEL.json inside a resolved 'current' dir.

    Raises E_CONFIG_INVALID(kind=model_missing) on read failure or
    parse error. The doc calls this 'MODEL.json can be parsed' --
    a syntactic error is treated as 'no valid MODEL.json = missing'.
    """
    path = os.path.join(current_dir, "MODEL.json")
    if not os.path.isfile(path):
        _fail("model_missing",
              "MODEL.json absent inside %s" % current_dir,
              model=model_name, path=path)
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        _fail("model_missing",
              "MODEL.json unreadable/unparseable: %s (%s)"
              % (path, exc),
              model=model_name, path=path,
              parse_error=str(exc))
        # unreachable, _fail always raises; return keeps mypy happy.
        return {}


# ---------------------------------------------------------------------------
# Sub-check 1: files[].sha256 vs on-disk
# ---------------------------------------------------------------------------

def _sha256_of(path: str) -> str:
    """Stream sha256 hex digest of a file.

    Reads in 64 KiB chunks so the whole file never sits in RAM.
    Weight files can be 2+ GB; a naive .read() would spike memory
    and OOM on the ORIN's 8 GB shared pool. Streaming keeps the
    resident set at the chunk size regardless of file size.
    """
    # Fresh hasher per file; state is not shared across calls.
    h = hashlib.sha256()
    # Binary mode is mandatory -- text mode would decode bytes and
    # break the digest on any non-ASCII byte.
    with open(path, "rb") as fh:
        # Read-until-EOF loop; b"" from read() means EOF.
        while True:
            chunk = fh.read(_HASH_CHUNK_BYTES)
            # Zero-length read = EOF; break before update() so we do
            # not feed an empty chunk (harmless but noise).
            if not chunk:
                break
            # hashlib.update accumulates state; O(1) per call apart
            # from the actual hashing work.
            h.update(chunk)
    # hexdigest returns lowercase hex per hashlib docs; MODEL.json
    # is written with lowercase hex too, so string compare works.
    return h.hexdigest()


def _check_files_sha256(model_dir: str, model_name: str,
                        model_json: Dict[str, Any]) -> None:
    """Compare every MODEL.json.files[] sha256 against on-disk.

    Raises E_CONFIG_INVALID(kind=model_digest_mismatch) on the first
    mismatch. First-fail is deliberate -- if one file is bad, the
    model is unusable; enumerating the rest is diagnostic noise.

    A malformed files[] entry (wrong shape, missing keys) is raised
    as model_missing, not model_digest_mismatch, because the digest
    check cannot even run -- the metadata says nothing to compare.
    """
    # files can be absent or empty for a placeholder model; skip
    # sha256 check when there is nothing declared. The 'current' +
    # parse check above already fires if MODEL.json itself is
    # broken, so an empty files[] is just 'nothing to verify here'.
    files = model_json.get("files") or []
    # Shape guard: MODEL.json.files must be a list per 11 S11A.4.2.
    # A dict or scalar here is metadata malformation.
    if not isinstance(files, list):
        _fail("model_missing",
              "MODEL.json.files is not a list for %s" % model_name,
              model=model_name)
    # Iterate every declared file; first mismatch wins.
    for entry in files:
        # entry shape per 11 S11A.4.2:
        #   {path: "model-q4_k_m.gguf", sha256: "...", bytes: N}
        # A malformed entry is a model_missing (metadata unusable).
        # The 'bytes' field is doc'd but not verified -- sha256
        # subsumes it (any byte-count change changes the digest).
        if not isinstance(entry, dict):
            _fail("model_missing",
                  "MODEL.json.files entry not a dict: %r" % entry,
                  model=model_name)
        # Pull the two fields we consume; any type other than str
        # is a schema violation.
        rel = entry.get("path")
        expected = entry.get("sha256")
        if not isinstance(rel, str) or not isinstance(expected, str):
            _fail("model_missing",
                  "MODEL.json.files entry missing path/sha256: %r"
                  % entry, model=model_name)
        # Resolve relative to the model dir; MODEL.json paths are
        # always relative to the version dir per the doc example.
        abs_path = os.path.join(model_dir, rel)
        # The file MUST exist -- MODEL.json claims it does. Missing
        # here is model_missing, not model_digest_mismatch, because
        # we cannot compute a digest to compare in the first place.
        if not os.path.isfile(abs_path):
            _fail("model_missing",
                  "declared file absent: %s" % abs_path,
                  model=model_name, path=abs_path)
        # Byte-for-byte comparison. This is the expensive step
        # (whole file streamed) so it runs last after cheap shape
        # / existence checks have narrowed the failure surface.
        actual = _sha256_of(abs_path)
        # String equality on lowercase hex; both sides are hashlib
        # output so no normalisation needed.
        if actual != expected:
            # Mismatch is the CFG-FZ-10 variant (1) failure mode:
            # 'change one byte of a weight file' -> this branch.
            _fail("model_digest_mismatch",
                  "sha256 mismatch on %s (expected %s, got %s)"
                  % (abs_path, expected, actual),
                  model=model_name, path=abs_path,
                  expected=expected, actual=actual)


# ---------------------------------------------------------------------------
# Sub-check 2: build_env vs runtime (vision only)
# ---------------------------------------------------------------------------

def _check_build_env(model_name: str, model_json: Dict[str, Any],
                     runtime_env: Dict[str, Optional[str]]) -> None:
    """Compare MODEL.json.build_env against the runtime triple.

    Only invoked when MODEL.json.kind == 'vision'. Compares three
    keys literally (11 S11A.4.2 verbatim). Any None on the runtime
    side yields a distinct failure so the operator knows the probe
    itself broke rather than the versions disagreeing.
    """
    build_env = model_json.get("build_env")
    # A vision model without a build_env block is a spec violation
    # (11 S11A.4.2 says '仅 kind == "vision" (TensorRT engine) 必填').
    # Treat as engine_env_mismatch: the check cannot proceed and the
    # model is not deployable.
    if not isinstance(build_env, dict):
        _fail("engine_env_mismatch",
              "vision model %s missing build_env block" % model_name,
              model=model_name, expected="build_env dict",
              actual=type(build_env).__name__)
    for key in _VISION_ENV_KEYS:
        expected = build_env.get(key)
        actual = runtime_env.get(key)
        # Missing on MODEL.json side -- same treatment as no block.
        if expected is None:
            _fail("engine_env_mismatch",
                  "vision model %s build_env missing key '%s'"
                  % (model_name, key),
                  model=model_name, key=key, expected=None)
        # Missing on runtime side -- the probe returned None. Distinct
        # message so 'the version disagrees' is not confused with
        # 'we could not read the version'.
        if actual is None:
            _fail("engine_env_mismatch",
                  "vision model %s cannot verify build_env.%s: "
                  "runtime probe returned None (dpkg / nvidia-smi "
                  "absent?)" % (model_name, key),
                  model=model_name, key=key, expected=expected,
                  actual=None, reason="runtime_probe_unavailable")
        # Literal string comparison (both sides normalised to str by
        # the probes and the doc's example). No numeric tolerance --
        # AIR-V1c says any change invalidates the engine.
        if str(actual) != str(expected):
            # Mismatch is the CFG-FZ-10 variant (2) failure mode.
            _fail("engine_env_mismatch",
                  "build_env.%s mismatch for %s: expected %r, "
                  "runtime %r" % (key, model_name, expected, actual),
                  model=model_name, key=key,
                  expected=str(expected), actual=str(actual))


# ---------------------------------------------------------------------------
# Iteration + runner
# ---------------------------------------------------------------------------

def _iter_models(models_root: str,
                 kinds: Iterable[str]) -> Iterable[Tuple[str, str, str]]:
    """Yield (kind, model_name, kind_dir) for each model dir found.

    A model_name is any subdir under <models_root>/<kind>/. If a
    kind dir does not exist (e.g. no vision models yet), it is
    silently skipped -- that is a 'no model of that kind on this
    host' state, not a defect. The scan is sorted so failure order
    is deterministic across runs (same first-fail on same defect).
    """
    # Loop the kind whitelist, not os.listdir(models_root). Extra
    # subdirs under models_root (README, .git, backup-2026-08-01)
    # would otherwise get treated as unknown kinds and confuse the
    # user with a name-lookup error.
    for kind in kinds:
        # Compose the per-kind dir; may or may not exist.
        kind_dir = os.path.join(models_root, kind)
        # A kind not deployed on this host is benign; production
        # will not have all four kinds on every unit (e.g. some
        # deploys have no vision at all). We do not warn.
        if not os.path.isdir(kind_dir):
            continue
        # Sorted so failure order is deterministic; makes 'this
        # test failed because model X' reproducible across CI runs.
        for name in sorted(os.listdir(kind_dir)):
            path = os.path.join(kind_dir, name)
            # Skip stray files inside a kind dir (README, .gitkeep).
            # A model always lives in a subdir.
            if os.path.isdir(path):
                # Emit the triple; downstream builds the current
                # symlink path from (kind_dir, name).
                yield kind, name, kind_dir


def run(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Real body for assertion I. Replaces registry's stub for I.

    Flow:
      1. Wiring guard on ctx['config_root'] (matches other assertions).
      2. Resolve models_root from ctx override or default.
      3. Iterate every model dir under {llm,asr,tts,vision}.
      4. For each: resolve current symlink, parse MODEL.json.
      5. Verify files[].sha256 for all kinds.
      6. Verify build_env vs runtime for kind=='vision' only.
      7. Return pass with per-kind counts for observability.
    """
    # Wiring guard identical to every other assertion. AssertionError
    # (not XbrainError) because a missing config_root is a caller
    # bug, not a config-artifact defect.
    if "config_root" not in ctx:
        raise AssertionError(
            "assertion I requires ctx['config_root']; caller did not "
            "populate it"
        )
    # Resolve models_root: ctx override wins (used by tests to point
    # at a tmp_path); default is the doc-defined /opt/xbrain/models.
    models_root = ctx.get("models_root") or _DEFAULT_MODELS_ROOT
    # kind_filter: default all four kinds. Tests may narrow this to
    # focus on a single kind's variant.
    kinds = tuple(ctx.get("kind_filter") or _KIND_DIRS)

    # models_root not existing at all is a legitimate 'no models
    # shipped' state on a dev machine; skip cleanly rather than red.
    # A deploy that expects models but has none will surface via a
    # different check (bring-up will fail when perception tries to
    # load its engine); assertion I's job is to catch WRONG models,
    # not ABSENT-but-expected ones.
    if not os.path.isdir(models_root):
        return {"status": "pass", "assertion": "I",
                "models_root": models_root, "models_checked": 0,
                "skipped_reason": "models_root_absent"}

    # Runtime env: probed lazily. Only vision models need it, and
    # dev machines without JetPack should not have to pay the
    # subprocess cost when they have no vision models to check.
    # Tests inject via ctx['runtime_env'] to avoid dpkg entirely.
    runtime_env = ctx.get("runtime_env")

    # Per-kind counters for the pass return shape; helpful for logs
    # to spot silent-zero cases (e.g., vision expected but 0 checked
    # means the deploy is missing vision models entirely).
    counts: Dict[str, int] = {k: 0 for k in _KIND_DIRS}
    vision_seen = False

    # Iterate every declared model. First failure raises; the loop
    # otherwise runs to completion so a healthy tree pings every
    # counter. Order within a kind is sorted (see _iter_models) so
    # a CFG-FZ-10 variant is reproducible: same defect -> same
    # first-fail model name.
    for kind, name, kind_dir in _iter_models(models_root, kinds):
        # Sub-check 3: current symlink + MODEL.json parseability.
        # This fires FIRST because the later checks depend on the
        # MODEL.json being loadable at all -- there is no point
        # asking 'is sha256 right?' before we can read the sha256.
        current_dir = _resolve_current_dir(kind_dir, name)
        # Parse MODEL.json inside the resolved current dir.
        # _load_model_json raises model_missing on malformed json.
        model_json = _load_model_json(current_dir, name)
        # Sub-check 1: byte-for-byte sha256 on every declared file.
        # Runs on all four kinds; large weight files are streamed
        # via _sha256_of to avoid holding a 2 GB gguf in RAM.
        # First mismatch raises; loop terminates.
        _check_files_sha256(current_dir, name, model_json)
        # Sub-check 2: build_env vs runtime, vision only.
        # The doc scopes this to TensorRT engines (11 S11A.4.2
        # verbatim: 'kind=="vision" 的 build_env ...'). llm/asr/tts
        # do not have platform-tied build_env because they run on
        # generic backends (llama.cpp, sherpa-onnx) that are ABI-
        # portable across JetPack minor versions.
        if str(model_json.get("kind", "")) == "vision":
            # Mark that we saw a vision model; used in return value
            # to let observers distinguish 'no vision to probe'
            # from 'we probed but did not need to'.
            vision_seen = True
            # Lazily probe runtime the first time a vision model
            # appears, if the caller did not inject. Deferring the
            # probe to the first vision model keeps dev machines
            # without dpkg / nvidia-smi runnable when they have
            # only llm/asr models to check.
            if runtime_env is None:
                runtime_env = _probe_runtime()
            # Compare MODEL.json.build_env vs runtime_env; raises
            # engine_env_mismatch on any of the three keys.
            _check_build_env(name, model_json, runtime_env)
        # Bump the per-kind counter only after all three checks
        # passed for this model. counts_by_kind reads as 'how many
        # models of this kind were fully verified'.
        counts[kind] = counts.get(kind, 0) + 1

    # Success: report per-kind counts + whether the runtime probe
    # was exercised. The vision_seen flag lets observers verify
    # that a deploy expecting vision models actually had one
    # checked -- a deploy that expects 1 vision but shows
    # counts_by_kind['vision']==0 is a config defect that this
    # assertion alone cannot detect (nothing to iterate over) but
    # the return shape surfaces the fact for a downstream check.
    return {
        # Fixed status; every assertion returns 'pass' on success
        # and raises on failure. No mixed 'warn' state.
        "status": "pass",
        # Fixed assertion label; matches registry row 'I'.
        "assertion": "I",
        # Echo the resolved models_root so a caller can confirm
        # WHICH tree was checked (production vs test).
        "models_root": models_root,
        # Total models verified across all kinds.
        "models_checked": sum(counts.values()),
        # Per-kind breakdown for silent-zero detection.
        "counts_by_kind": counts,
        # True iff at least one kind=='vision' model was seen.
        "vision_seen": vision_seen,
        # True iff runtime_env was actually populated (probed or
        # injected) AND a vision model was there to consume it.
        "runtime_env_probed": runtime_env is not None and vision_seen,
    }
