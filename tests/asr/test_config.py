"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_config.py
Brief: Unit tests for asr-service configuration defaults and environment overrides.

Description:
  Covers the two things AsrConfig promises: the documented defaults are what an
  un-configured deploy gets, and every field is overridable through the environment with
  a bad value rejected loudly rather than silently ignored. The boolean parser gets the
  most attention because it is the one field where a typo could flip the model precision
  (int8 vs fp32) and quietly change RTF -- the service's hard acceptance metric.

  monkeypatch.setenv/delenv is used so the process environment is restored after each
  test and the suite cannot leak an override into an unrelated case.

  Import reach: this module imports only services.asr.config, which pulls in nothing
  beyond the standard library, so it runs anywhere. Run from the /opt/xbrain_v6 root:
      python3 -m pytest tests/asr/test_config.py
"""
from __future__ import annotations

import os
from dataclasses import FrozenInstanceError

import pytest

from services.asr.config import _FAMILY_MODEL_DIRS, AsrConfig, AsrConfigError
from services.asr.core import families


def test_defaults_match_documented_values() -> None:
    # The listen port must stay clear of payload-service (18080) and llama-server (8080),
    # and int8 must be the default because RTF < 0.5 depends on the fast model path.
    config = AsrConfig()
    assert config.port_asr == 18081
    assert config.use_int8 is True
    # ★ 1, and asserted rather than left loose because raising it is actively harmful under
    # AIR-P3's CPUAffinity=7: four onnxruntime threads on one shared core measured rtf 0.452
    # against 0.119 at one thread. Someone tuning for throughput would reasonably raise this
    # and make the deployed system 3.7x slower, so the value is pinned by a test.
    assert config.num_threads == 1
    assert config.hotwords_score == 2.0
    # The default family is a measured choice (see config.py), so it is asserted here: a
    # silent change of architecture is exactly the kind of drift that shows up later as an
    # unexplained accuracy difference.
    assert config.model_family == "paraformer"
    # An engine capability floor, not a speech-length policy -- and low enough that a real
    # one-syllable command ("停" is about 190 ms) can never be rejected by it.
    assert config.min_audio_ms == 100


def test_default_paths_are_absolute_and_inside_the_service_tree() -> None:
    # Model and hotword paths are derived from the module location, not hardcoded, so the
    # services/asr/ tree stays relocatable. Both must therefore be absolute and sit beside
    # the package rather than under some caller's working directory.
    #
    # The model directory is asserted by its SHAPE, not by any exact name: which model is
    # deployed follows the default family and is expected to change when a better one is
    # measured, and the version directory it resolves to changes on every model bump.
    # What must hold is that it stays inside the service tree (relocatable) and ends at
    # 11 §11A.4.1's `current` symlink rather than at a version -- because §11A.4.1 defines
    # rollback as 「改软链 + 重启单元」, and a config naming a version directly would turn
    # that flip into a code change. Asserting the layout is the point; asserting the name
    # is what made this test fail the first time the layout was built correctly.
    config = AsrConfig()
    assert os.path.isabs(config.model_dir)
    assert os.path.isabs(config.hotwords_file)
    assert os.path.basename(config.model_dir) == "current"
    inside = os.path.join("services", "asr", "models")
    assert inside in config.model_dir
    assert config.hotwords_file.endswith(os.path.join("services", "asr", "hotwords.txt"))


def test_default_model_dir_follows_the_default_family() -> None:
    # The pairing matters more than either value: a family and a directory that disagree
    # produce a "missing model file" at startup, and the whole point of deriving one from
    # the other is that an un-configured deploy cannot land in that state.
    # Checked against the model-name component -- the directory that names the model, which
    # is `current`'s parent under 11 §11A.4.1's <name>/<ver>/ + current layout. basename is
    # "current" for every family, so it cannot distinguish them.
    config = AsrConfig()
    model_name_dir = os.path.basename(os.path.dirname(config.model_dir))
    assert config.model_family in model_name_dir


def test_from_env_without_overrides_returns_defaults(monkeypatch: pytest.MonkeyPatch) -> None:
    # With every key unset, from_env must reproduce the class defaults exactly -- that is
    # what makes "no configuration" a supported, documented deployment.
    for key in ("PORT_ASR", "ASR_MODEL_FAMILY", "ASR_MODEL_DIR", "ASR_HOTWORDS_FILE",
                "ASR_HOTWORDS_SCORE", "ASR_NUM_THREADS", "ASR_INT8", "ASR_MIN_AUDIO_MS",
                "ASR_REQUIRE_SAMPLE_RATE"):
        monkeypatch.delenv(key, raising=False)
    assert AsrConfig.from_env() == AsrConfig()


def test_from_env_applies_every_override(monkeypatch: pytest.MonkeyPatch) -> None:
    # Each field is overridable; a field that silently ignored its key would leave an
    # operator unable to retune the service on the box without editing source.
    monkeypatch.setenv("PORT_ASR", "19000")
    monkeypatch.setenv("ASR_MODEL_FAMILY", "transducer")
    monkeypatch.setenv("ASR_MODEL_DIR", "/tmp/model")
    monkeypatch.setenv("ASR_HOTWORDS_FILE", "/tmp/hw.txt")
    monkeypatch.setenv("ASR_HOTWORDS_SCORE", "3.5")
    monkeypatch.setenv("ASR_NUM_THREADS", "8")
    monkeypatch.setenv("ASR_INT8", "0")
    monkeypatch.setenv("ASR_MIN_AUDIO_MS", "250")
    config = AsrConfig.from_env()
    assert config.port_asr == 19000
    assert config.model_family == "transducer"
    assert config.model_dir == "/tmp/model"
    assert config.hotwords_file == "/tmp/hw.txt"
    assert config.hotwords_score == 3.5
    assert config.num_threads == 8
    assert config.use_int8 is False
    assert config.min_audio_ms == 250


def test_family_override_moves_the_model_dir_with_it(monkeypatch: pytest.MonkeyPatch) -> None:
    # Setting the family ALONE must not leave model_dir pointing at the previous family's
    # tree: the two would then describe different architectures, and the failure would be
    # a confusing "missing file" rather than the successful switch the operator asked for.
    monkeypatch.delenv("ASR_MODEL_DIR", raising=False)
    monkeypatch.setenv("ASR_MODEL_FAMILY", "transducer")
    config = AsrConfig.from_env()
    assert config.model_family == "transducer"
    # Compared against the paraformer default rather than a literal path: the invariant is
    # that the two move TOGETHER, and a literal would have to be edited every time either
    # the layout or the shipped export changes -- which is how this assertion broke before.
    assert config.model_dir != AsrConfig().model_dir
    assert "zipformer" in config.model_dir
    assert os.path.basename(config.model_dir) == "current"


def test_explicit_model_dir_wins_over_the_family_default(monkeypatch: pytest.MonkeyPatch) -> None:
    # An operator pointing at a bench copy of the same family must not have it overridden
    # by the derived default -- explicit configuration always beats derivation.
    monkeypatch.setenv("ASR_MODEL_FAMILY", "transducer")
    monkeypatch.setenv("ASR_MODEL_DIR", "/tmp/bench-copy")
    assert AsrConfig.from_env().model_dir == "/tmp/bench-copy"


def test_unknown_family_is_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    # Same rule as ASR_INT8=flase: an unrecognised value must fail loudly rather than fall
    # back, because quietly loading a different architecture than the operator asked for
    # surfaces only as an accuracy change nobody can explain.
    monkeypatch.setenv("ASR_MODEL_FAMILY", "zipformer")  # plausible, but not a family name
    with pytest.raises(AsrConfigError) as excinfo:
        AsrConfig.from_env()
    assert "ASR_MODEL_FAMILY" in str(excinfo.value)


def test_bool_override_accepts_documented_words(monkeypatch: pytest.MonkeyPatch) -> None:
    # Case and surrounding whitespace must not matter: the same value written by a shell
    # export, a systemd unit or a docker env file should behave identically.
    for word in ("1", "true", "TRUE", " yes ", "on"):
        monkeypatch.setenv("ASR_INT8", word)
        assert AsrConfig.from_env().use_int8 is True
    for word in ("0", "false", "No", "off"):
        monkeypatch.setenv("ASR_INT8", word)
        assert AsrConfig.from_env().use_int8 is False


def test_bad_overrides_raise_config_error(monkeypatch: pytest.MonkeyPatch) -> None:
    # A misspelled boolean must NOT be read as false: silently swapping the model
    # precision is exactly the failure the strict word list exists to prevent.
    monkeypatch.setenv("ASR_INT8", "flase")
    with pytest.raises(AsrConfigError):
        AsrConfig.from_env()
    monkeypatch.delenv("ASR_INT8")
    # The numeric parsers fail the same way, and the message must name the offending key.
    monkeypatch.setenv("PORT_ASR", "eighteen-thousand")
    with pytest.raises(AsrConfigError) as excinfo:
        AsrConfig.from_env()
    assert "PORT_ASR" in str(excinfo.value)
    monkeypatch.delenv("PORT_ASR")
    monkeypatch.setenv("ASR_HOTWORDS_SCORE", "loud")
    with pytest.raises(AsrConfigError):
        AsrConfig.from_env()


def test_config_is_frozen() -> None:
    # The config is shared with the decode worker thread, so it must be immutable: an
    # accidental write would be a data race rather than a visible error.
    config = AsrConfig()
    with pytest.raises(FrozenInstanceError):
        config.num_threads = 16  # type: ignore[misc]


def test_family_without_a_shipped_directory_refuses_to_guess(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # core/families.py can LOAD more architectures than this tree ships weights for. The
    # dangerous case is not "file missing" -- it is that paraformer, sense_voice and the
    # CTC families all look for "model*.onnx" plus tokens.txt, so falling back to another
    # family's directory finds every file it expects and builds a recognizer of one
    # architecture on another's weights. Refusing is the only outcome that cannot be
    # mistaken for success.
    unpaired = [name for name in families.FAMILY_NAMES if name not in _FAMILY_MODEL_DIRS]
    if not unpaired:
        pytest.skip("every declared family ships a directory; nothing to refuse")
    monkeypatch.delenv("ASR_MODEL_DIR", raising=False)
    monkeypatch.setenv("ASR_MODEL_FAMILY", unpaired[0])
    with pytest.raises(AsrConfigError) as excinfo:
        AsrConfig.from_env()
    # The message has to name the escape hatch, or the operator is simply stuck.
    assert "ASR_MODEL_DIR" in str(excinfo.value)


def test_unpaired_family_still_works_with_an_explicit_directory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Refusing to GUESS must not mean refusing to run: a family with no shipped weights is
    # still selectable once the operator says where its export lives, which is what makes
    # evaluating a new candidate model an environment variable rather than a patch.
    unpaired = [name for name in families.FAMILY_NAMES if name not in _FAMILY_MODEL_DIRS]
    if not unpaired:
        pytest.skip("every declared family ships a directory")
    monkeypatch.setenv("ASR_MODEL_FAMILY", unpaired[0])
    monkeypatch.setenv("ASR_MODEL_DIR", "/tmp/candidate-export")
    config = AsrConfig.from_env()
    assert config.model_family == unpaired[0]
    assert config.model_dir == "/tmp/candidate-export"


def test_sample_rate_gate_defaults_to_the_contract_rate() -> None:
    # 11 AS-2 fixes this interface at 16 kHz. The default enforces it, because the whole
    # audio plane produces exactly that rate and anything else means the capture chain is
    # wrong -- a fault the engine would otherwise hide by resampling silently.
    assert AsrConfig().require_sample_rate == 16000


def test_sample_rate_gate_can_be_disabled(monkeypatch: pytest.MonkeyPatch) -> None:
    # Zero accepts any rate, for a bench replaying archived audio where the strictness
    # protects nothing. It must be reachable without editing source.
    monkeypatch.setenv("ASR_REQUIRE_SAMPLE_RATE", "0")
    assert AsrConfig.from_env().require_sample_rate == 0
