"""Model identity must never be fabricated, and must never fail a health probe.

core/model_meta.py sits on two hot paths with opposite failure preferences, and both are
easy to get wrong in the same edit:

  * 11 §11A.7 calls model.version "换模型后金标集回归失败时唯一能对上的线索". A
    fabricated version -- "unknown", "0.0.0", the directory name -- would satisfy every
    schema check and send the next person chasing a build that was never made. So every
    test below that supplies a broken sidecar asserts None, not a placeholder.

  * 11 §11A.5.1 turns any non-200 from /healthz into `fail`, and three of those open the
    breaker for 60 s (16 §9.3). So no input may raise: a service that refuses to report
    its health because MODEL.json has a stray comma is worse than one that reports health
    with an unknown version.
"""
from __future__ import annotations

import json
import os

from services.asr.core import model_meta


def _write(directory, payload: str) -> None:
    """Write a raw MODEL.json body, so malformed cases can be expressed as text."""
    with open(os.path.join(str(directory), model_meta.MODEL_JSON), "w",
              encoding="utf-8") as handle:
        handle.write(payload)


def test_version_is_read_from_the_sidecar(tmp_path) -> None:
    # The ordinary case: a well-formed MODEL.json is where version comes from. Written as
    # the real builder writes it, so the test would catch a key rename on either side.
    _write(tmp_path, json.dumps({"schema": "xbrain.model/1", "kind": "asr",
                                 "name": "paraformer-zh-2023-09", "version": "1.0.0"}))
    assert model_meta.read_version(str(tmp_path)) == "1.0.0"


def test_surrounding_whitespace_is_stripped(tmp_path) -> None:
    # A trailing newline in a hand-edited sidecar must not become part of the identity --
    # it would make an otherwise-equal version compare unequal in the audit trail.
    _write(tmp_path, json.dumps({"version": " 1.2.3\n"}))
    assert model_meta.read_version(str(tmp_path)) == "1.2.3"


def test_missing_sidecar_yields_none(tmp_path) -> None:
    # The state every deploy is in before 11 §11A.4.1's layout is built. Must be None, and
    # must not raise: this is the common case, not an error.
    assert model_meta.read_version(str(tmp_path)) is None


def test_malformed_sidecar_yields_none_rather_than_raising(tmp_path) -> None:
    # * The health-probe half of the contract. Truncated JSON is what a half-finished
    # deploy leaves behind, and it must degrade to "unknown version", never to a 500 that
    # 11 §11A.5.1 reads as `fail`.
    _write(tmp_path, '{"version": "1.0.0"')
    assert model_meta.read_version(str(tmp_path)) is None


def test_a_sidecar_without_a_usable_version_yields_none(tmp_path) -> None:
    # * The anti-fabrication half. Each of these is a plausible sidecar that carries no
    # version, and each must produce None rather than a stand-in derived from what IS
    # present -- name, kind and quant are all tempting and all wrong.
    for payload in ('{"name": "paraformer-zh-2023-09", "kind": "asr"}',  # absent
                    '{"version": ""}',                                    # empty
                    '{"version": "   "}',                                 # whitespace
                    '{"version": 100}',                                   # not a string
                    '{"version": null}',                                  # explicit null
                    '["version", "1.0.0"]'):                              # not an object
        _write(tmp_path, payload)
        assert model_meta.read_version(str(tmp_path)) is None, payload


def test_hash_is_stable_and_tracks_content(tmp_path) -> None:
    # sha256 answers "are the bytes on disk still the bytes that were tested", so it must
    # be reproducible across calls and must move when any byte does.
    first = tmp_path / "model.onnx"
    first.write_bytes(b"weights")
    paths = [str(first)]
    digest = model_meta.hash_files(paths)
    assert digest is not None and len(digest) == 64
    assert model_meta.hash_files(paths) == digest
    first.write_bytes(b"weightt")
    assert model_meta.hash_files(paths) != digest


def test_a_byte_moved_between_files_changes_the_hash(tmp_path) -> None:
    # * This is why hash_files frames each file with its name and length instead of just
    # concatenating bytes. A multi-file export (encoder/decoder/joiner) whose parts came
    # from different builds is exactly the mismatch this check exists to catch, and a
    # plain concatenation would hash both arrangements identically.
    a, b = tmp_path / "a.bin", tmp_path / "b.bin"
    a.write_bytes(b"xy")
    b.write_bytes(b"z")
    split_one = model_meta.hash_files([str(a), str(b)])
    a.write_bytes(b"x")
    b.write_bytes(b"yz")
    split_two = model_meta.hash_files([str(a), str(b)])
    assert split_one != split_two


def test_order_is_significant(tmp_path) -> None:
    # The caller passes a sorted list precisely so the digest is reproducible; if order did
    # not matter that sort would be dead code, and a later edit could drop it unnoticed.
    a, b = tmp_path / "a.bin", tmp_path / "b.bin"
    a.write_bytes(b"one")
    b.write_bytes(b"two")
    assert model_meta.hash_files([str(a), str(b)]) != model_meta.hash_files([str(b), str(a)])


def test_an_unreadable_file_yields_none_rather_than_a_partial_hash(tmp_path) -> None:
    # * Returning a digest computed over the files that COULD be read would be a digest of
    # something that is not the model, and it would compare unequal to the manifest for a
    # reason ("a file is missing") the value itself cannot express. None says "cannot
    # answer", which is the truth.
    present = tmp_path / "present.bin"
    present.write_bytes(b"data")
    assert model_meta.hash_files([str(present), str(tmp_path / "gone.bin")]) is None
