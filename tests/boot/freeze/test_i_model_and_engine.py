"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_i_model_and_engine.py
Brief: CFG-FZ-10 -- assertion I three variants + baseline + skip cases

Description:
Three CFG-FZ-10 variants named verbatim in docs/XBRAIN_V6_TODO.md:
  (1) change one byte of a weight file        -> model_digest_mismatch
  (2) build_env.tensorrt off by one digit     -> engine_env_mismatch
  (3) break current symlink                   -> model_missing

Plus:
  * baseline: three-kind scaffold passes
  * vision without build_env block            -> engine_env_mismatch
  * runtime probe returns None (dev machine)  -> engine_env_mismatch
  * MODEL.json parse error                    -> model_missing
  * MODEL.json declares an absent file        -> model_missing
  * no vision model present + no runtime_env  -> skip probe entirely
  * models_root missing entirely              -> pass (dev checkout)
  * wiring guard on ctx['config_root']

Every test uses a tmp_path model tree; no dpkg / nvidia-smi is called.
"""

# hashlib for computing MODEL.json.files[].sha256 on the fly so the
# test scaffold is self-consistent (no fixed hex strings that would
# drift as fixture files change).
import hashlib
# json for writing MODEL.json into the scaffold.
import json
# os for path joins + symlink calls.
import os

import pytest

from xbrain.boot.freeze.assertions.i_model_and_engine import run
from xbrain.common.errors.exceptions import XbrainError


# ---------------------------------------------------------------------------
# Scaffolding helpers
# ---------------------------------------------------------------------------

# Runtime triple used for every test; matches 11 S11A.4.2 example
# (jetpack=7.2, tensorrt=10.16.2, sm=87). Written as constants so the
# tests read naturally against the doc.
_RUNTIME_ENV_MATCH = {"jetpack": "7.2", "tensorrt": "10.16.2", "sm": "87"}


def _sha256_bytes(data: bytes) -> str:
    """Compute sha256 hex over the given bytes (for scaffold files)."""
    return hashlib.sha256(data).hexdigest()


def _write_file(path, data: bytes) -> None:
    """Write bytes to path, creating parent dirs as needed."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)


def _build_model_dir(models_root, kind, name, version,
                     files_map, extra_json=None):
    """Build one model at <root>/<kind>/<name>/<version>/ + current.

    files_map is {relpath: bytes}. MODEL.json is written with real
    sha256 values. extra_json is merged into MODEL.json for kind-
    specific fields (e.g. build_env for vision).
    """
    ver_dir = os.path.join(models_root, kind, name, version)
    os.makedirs(ver_dir, exist_ok=True)
    files_meta = []
    # Write each file and record its real sha256 in MODEL.json.
    for rel, data in files_map.items():
        _write_file(os.path.join(ver_dir, rel), data)
        files_meta.append({
            "path": rel,
            "sha256": _sha256_bytes(data),
            "bytes": len(data),
        })
    model_json = {
        "schema": "xbrain.model/1",
        "kind": kind,
        "name": name,
        "version": version,
        "files": files_meta,
    }
    if extra_json:
        model_json.update(extra_json)
    with open(os.path.join(ver_dir, "MODEL.json"), "w") as f:
        json.dump(model_json, f)
    # Create the current symlink: current -> <version>.
    current_path = os.path.join(models_root, kind, name, "current")
    # Symlink target is RELATIVE (matches doc example 'current -> 1.0.0').
    if os.path.islink(current_path):
        os.remove(current_path)
    os.symlink(version, current_path)
    return ver_dir


def _green_scaffold(tmp_path):
    """Build a three-kind (llm + asr + vision) scaffold at tmp_path/models."""
    models_root = str(tmp_path / "models")
    # llm: single gguf, sha256 self-consistent.
    _build_model_dir(models_root, "llm", "qwen-3b", "1.0.0",
                     {"model.gguf": b"weights-llm"})
    # asr: two files (model + tokenizer).
    _build_model_dir(models_root, "asr", "paraformer", "2.1.0",
                     {"model.onnx": b"weights-asr", "tokens.txt": b"a b c"})
    # vision: engine + golden frame, with build_env matching the
    # runtime triple used by tests.
    _build_model_dir(models_root, "vision", "yolov11n", "1.0.0",
                     {"yolov11n.engine": b"weights-vision"},
                     extra_json={"build_env": dict(_RUNTIME_ENV_MATCH)})
    return models_root


def _configs_root(tmp_path):
    """Build a minimal configs root for the wiring-guard ctx field."""
    root = tmp_path / "configs"
    root.mkdir()
    return str(root)


def _ctx(tmp_path, **extra):
    """Standard ctx: config_root + runtime_env + models_root."""
    c = {
        "config_root": _configs_root(tmp_path),
        "models_root": _green_scaffold(tmp_path),
        "runtime_env": dict(_RUNTIME_ENV_MATCH),
    }
    c.update(extra)
    return c


# ---------------------------------------------------------------------------
# Baseline: healthy scaffold passes
# ---------------------------------------------------------------------------

def test_green_scaffold_passes(tmp_path):
    """Three models (llm/asr/vision), sha256 consistent, build_env matches."""
    ctx = _ctx(tmp_path)
    result = run(ctx)
    assert result["status"] == "pass"
    assert result["assertion"] == "I"
    assert result["models_checked"] == 3
    assert result["counts_by_kind"] == {"llm": 1, "asr": 1, "tts": 0,
                                         "vision": 1}
    assert result["vision_seen"] is True


def test_no_models_root_is_pass(tmp_path):
    """Dev checkout without any models tree -> pass with skipped_reason."""
    ctx = {"config_root": _configs_root(tmp_path),
           "models_root": str(tmp_path / "does-not-exist")}
    result = run(ctx)
    assert result["status"] == "pass"
    assert result["models_checked"] == 0
    assert result["skipped_reason"] == "models_root_absent"


def test_no_vision_model_skips_runtime_probe(tmp_path):
    """When there is no vision model, runtime_env is never touched."""
    models_root = str(tmp_path / "models")
    _build_model_dir(models_root, "llm", "qwen-3b", "1.0.0",
                     {"model.gguf": b"weights-llm"})
    ctx = {"config_root": _configs_root(tmp_path),
           "models_root": models_root}  # NOTE: no runtime_env
    result = run(ctx)
    assert result["status"] == "pass"
    assert result["vision_seen"] is False
    assert result["runtime_env_probed"] is False


# ---------------------------------------------------------------------------
# CFG-FZ-10 variant (1): change one byte of a weight file
# ---------------------------------------------------------------------------

def test_variant_1_weight_byte_flip_is_digest_mismatch(tmp_path):
    """CFG-FZ-10 variant 1 verbatim: one byte of a weight file
    changed -> model_digest_mismatch."""
    ctx = _ctx(tmp_path)
    # Flip one byte of the llm weight file AFTER MODEL.json was
    # written. MODEL.json still records the ORIGINAL sha256.
    weight_path = os.path.join(ctx["models_root"], "llm", "qwen-3b",
                                "1.0.0", "model.gguf")
    with open(weight_path, "rb") as f:
        data = bytearray(f.read())
    data[0] ^= 0xFF   # flip one byte, unambiguously changes sha256
    with open(weight_path, "wb") as f:
        f.write(bytes(data))
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.code == "E_CONFIG_INVALID"
    assert ei.value.detail["kind"] == "model_digest_mismatch"
    assert ei.value.detail["model"] == "qwen-3b"
    assert ei.value.detail["expected"] != ei.value.detail["actual"]


def test_variant_1b_second_file_byte_flip_also_caught(tmp_path):
    """A byte flip in the SECOND declared file of a model must also
    fire (proves we iterate the files list, not just [0])."""
    ctx = _ctx(tmp_path)
    tokens_path = os.path.join(ctx["models_root"], "asr", "paraformer",
                                "2.1.0", "tokens.txt")
    with open(tokens_path, "rb") as f:
        data = f.read()
    with open(tokens_path, "wb") as f:
        f.write(data + b"X")   # append, changes sha256
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "model_digest_mismatch"
    assert ei.value.detail["path"].endswith("tokens.txt")


# ---------------------------------------------------------------------------
# CFG-FZ-10 variant (2): build_env.tensorrt off by one digit
# ---------------------------------------------------------------------------

def test_variant_2_tensorrt_off_by_one_digit_is_env_mismatch(tmp_path):
    """CFG-FZ-10 variant 2 verbatim: build_env.tensorrt changed by
    one digit -> engine_env_mismatch."""
    ctx = _ctx(tmp_path)
    # Read the vision MODEL.json, change tensorrt version, write back.
    mj_path = os.path.join(ctx["models_root"], "vision", "yolov11n",
                            "1.0.0", "MODEL.json")
    with open(mj_path) as f:
        mj = json.load(f)
    mj["build_env"]["tensorrt"] = "10.16.3"   # off by one from 10.16.2
    with open(mj_path, "w") as f:
        json.dump(mj, f)
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "engine_env_mismatch"
    assert ei.value.detail["key"] == "tensorrt"
    assert ei.value.detail["expected"] == "10.16.3"
    assert ei.value.detail["actual"] == "10.16.2"


def test_variant_2b_jetpack_mismatch_also_caught(tmp_path):
    """AIR-V1c also covers jetpack drift; a JetPack version mismatch
    must fire the same failure mode."""
    ctx = _ctx(tmp_path)
    mj_path = os.path.join(ctx["models_root"], "vision", "yolov11n",
                            "1.0.0", "MODEL.json")
    with open(mj_path) as f:
        mj = json.load(f)
    mj["build_env"]["jetpack"] = "7.1"   # runtime has 7.2
    with open(mj_path, "w") as f:
        json.dump(mj, f)
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["key"] == "jetpack"


def test_vision_without_build_env_is_env_mismatch(tmp_path):
    """A vision model shipped without build_env block cannot be
    verified -> engine_env_mismatch."""
    models_root = str(tmp_path / "models")
    # Build without extra_json: no build_env block.
    _build_model_dir(models_root, "vision", "yolov11n", "1.0.0",
                     {"yolov11n.engine": b"weights-vision"})
    ctx = {"config_root": _configs_root(tmp_path),
           "models_root": models_root,
           "runtime_env": dict(_RUNTIME_ENV_MATCH)}
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "engine_env_mismatch"


def test_runtime_probe_none_is_reported_distinctly(tmp_path):
    """When the runtime probe returns None (e.g., dev machine has no
    dpkg), a vision model must fire with reason=runtime_probe_unavailable
    so the operator can distinguish 'probe broken' from 'versions differ'."""
    ctx = _ctx(tmp_path)
    ctx["runtime_env"] = {"jetpack": None, "tensorrt": None, "sm": None}
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "engine_env_mismatch"
    assert ei.value.detail["reason"] == "runtime_probe_unavailable"


# ---------------------------------------------------------------------------
# CFG-FZ-10 variant (3): break current symlink
# ---------------------------------------------------------------------------

def test_variant_3_broken_current_symlink_is_model_missing(tmp_path):
    """CFG-FZ-10 variant 3 verbatim: current symlink target removed
    -> model_missing."""
    ctx = _ctx(tmp_path)
    # Remove the version dir that llm/qwen-3b/current points to.
    ver_dir = os.path.join(ctx["models_root"], "llm", "qwen-3b", "1.0.0")
    # Two-step: os.remove each file then rmdir; simpler: shutil.rmtree.
    import shutil
    shutil.rmtree(ver_dir)
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "model_missing"
    assert ei.value.detail["reason"] == "dangling_symlink"


def test_variant_3b_missing_current_symlink_is_model_missing(tmp_path):
    """current symlink not created at all -> model_missing."""
    ctx = _ctx(tmp_path)
    # Delete the current symlink entirely.
    os.remove(os.path.join(ctx["models_root"], "llm", "qwen-3b",
                            "current"))
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "model_missing"
    assert ei.value.detail["reason"] == "no_symlink"


def test_model_json_absent_is_model_missing(tmp_path):
    """current -> valid dir but MODEL.json absent -> model_missing."""
    ctx = _ctx(tmp_path)
    os.remove(os.path.join(ctx["models_root"], "llm", "qwen-3b",
                            "1.0.0", "MODEL.json"))
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "model_missing"


def test_model_json_malformed_is_model_missing(tmp_path):
    """MODEL.json exists but is not valid json -> model_missing."""
    ctx = _ctx(tmp_path)
    mj = os.path.join(ctx["models_root"], "llm", "qwen-3b",
                       "1.0.0", "MODEL.json")
    with open(mj, "w") as f:
        f.write("{ not valid json")
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "model_missing"
    # parse_error field must carry the JSONDecodeError text so
    # operator can grep it.
    assert "parse_error" in ei.value.detail


def test_declared_file_absent_is_model_missing(tmp_path):
    """MODEL.json lists a file that does not exist -> model_missing."""
    ctx = _ctx(tmp_path)
    # Delete the weight file but leave MODEL.json intact (which
    # still lists it).
    os.remove(os.path.join(ctx["models_root"], "llm", "qwen-3b",
                            "1.0.0", "model.gguf"))
    with pytest.raises(XbrainError) as ei:
        run(ctx)
    assert ei.value.detail["kind"] == "model_missing"


# ---------------------------------------------------------------------------
# Wiring guard
# ---------------------------------------------------------------------------

def test_requires_config_root():
    """Missing ctx['config_root'] is a caller bug -> AssertionError."""
    with pytest.raises(AssertionError, match=r"ctx\['config_root'\]"):
        run({})
