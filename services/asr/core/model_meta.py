"""Model identity: what is loaded, which version it is, and what its bytes hash to.

11 AS-11 requires every transcription to carry model.name and model.version, and
11 §11A.8.1 requires GET /v1/models to publish name / version / sha256 / loaded. Both
answers come from the same place -- the model directory on disk -- so they are derived
here once rather than in the route handlers.

* version and sha256 come from MODEL.json, the sidecar 11 §11A.4.1 puts in every version
directory. When it is absent this module returns None for version rather than inventing
one. That is the whole point of the module: 11 §11A.7 calls model.version "换模型后金标集
回归失败时唯一能对上的线索", and a fabricated version would be worse than a null one --
null says "this deploy cannot be identified", a made-up string says "it can" and sends the
next person chasing a version that was never built.

sha256 is computed from the actual network files rather than read from MODEL.json, because
its job is to detect that the bytes on disk are NOT what the sidecar claims. Reading the
value the file asserts about itself would make the check vacuous. It is cached after the
first call: the files are megabytes, the answer cannot change while the process holds them
open, and GET /v1/models is polled.
"""
from __future__ import annotations

import hashlib
import json
import logging
import os
from dataclasses import dataclass
from typing import Optional, Sequence

logger = logging.getLogger(__name__)

# 11 §11A.4.1 puts this sidecar in each version directory beside the model files.
MODEL_JSON = "MODEL.json"

# Read in blocks rather than whole-file: the ASR export is tens of megabytes and the LLM
# gguf is two gigabytes, and this runs in a request handler.
_HASH_BLOCK = 1 << 20


@dataclass(frozen=True)
class ModelMeta:
    """Identity of one loaded model, as 11 §11A.8.1's /v1/models row describes it."""

    # Directory-derived identifier, e.g. "paraformer-zh-2023-09-int8". Always available:
    # it is built from the path and precision, so it needs no sidecar.
    name: str
    # * Semantic version from MODEL.json, or None when no sidecar exists. None is a truthful
    # "unidentified deploy", never a placeholder -- see this module's docstring.
    version: Optional[str]
    # Hex sha256 over the network files, in the order given. None only if they are
    # unreadable, which is itself worth publishing rather than hiding behind a zero hash.
    sha256: Optional[str]
    # Whether the engine actually holds this model. This service loads exactly one model
    # during lifespan startup and aborts if it fails, so a served response always has True;
    # the field exists because the contract's row does, and because a future multi-model
    # deploy would need it to mean something.
    loaded: bool


def read_version(model_dir: str) -> Optional[str]:
    """Read the version string from a model directory's MODEL.json.

    Args:
        model_dir: directory holding the model files and, per 11 §11A.4.1, MODEL.json.

    Returns:
        The sidecar's "version" value, or None when the sidecar is missing, unreadable,
        malformed, or does not carry a non-empty string version.

    Every failure mode returns None rather than raising. This is called while answering a
    health probe: a service that refuses to report its health because its version file has
    a stray comma is worse than one that reports health with an unknown version, and 11
    §11A.5.1 treats a non-200 as `fail` regardless of why.
    """
    path = os.path.join(model_dir, MODEL_JSON)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except FileNotFoundError:
        # The expected case until 11 §11A.4.1's layout is built; not worth a log line.
        return None
    except (OSError, ValueError) as error:
        # Present but broken IS worth saying, because someone wrote it and it is wrong.
        logger.warning("%s is unreadable, reporting version=null: %s", path, error)
        return None
    version = payload.get("version") if isinstance(payload, dict) else None
    if not isinstance(version, str) or not version.strip():
        logger.warning("%s has no usable string version, reporting version=null", path)
        return None
    return version.strip()


def hash_files(paths: Sequence[str]) -> Optional[str]:
    """Compute one sha256 over several files, in the order given.

    Args:
        paths: model file paths to digest. Order is significant and must be stable across
            calls, or the same deploy would hash differently on different runs.

    Returns:
        Lowercase hex digest, or None if any file could not be read.

    The files are folded into a SINGLE digest, and each contributes its basename and length
    before its bytes. Without those separators, moving bytes from the end of one file to the
    start of the next would leave the digest unchanged -- which for a model export split
    into encoder/decoder/joiner parts is not a hypothetical, since a mismatched set is
    exactly the failure this is meant to catch.
    """
    digest = hashlib.sha256()
    for path in paths:
        try:
            size = os.path.getsize(path)
            # Frame each file so concatenation cannot be ambiguous (see docstring).
            digest.update(f"{os.path.basename(path)}:{size}\n".encode("utf-8"))
            with open(path, "rb") as handle:
                while True:
                    block = handle.read(_HASH_BLOCK)
                    if not block:
                        break
                    digest.update(block)
        except OSError as error:
            logger.warning("cannot hash %s, reporting sha256=null: %s", path, error)
            return None
    return digest.hexdigest()
