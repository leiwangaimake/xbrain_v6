"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_hotwords.py
Brief: Unit tests for hotword vocabulary parsing, normalization and materialization.

Description:
  Hotword biasing is a hard requirement for this service, and it fails SILENTLY when the
  vocabulary is wrong: a phrase whose modeling units are not separated simply stops
  biasing, and nothing in the decode output says so. These tests are therefore the only
  place that failure mode is caught, and they assert the exact spacing the engine expects
  rather than just "some string came back".

  Coverage: the authoring conveniences (comments, blank lines, already-spaced legacy
  lines, duplicates), the tokenizer's CJK-versus-latin rule, reading from a file, the
  materialized file's engine format, and the guarantee that the temporary file is removed
  even when the body of the with-block raises.

  The shipped services/asr/hotwords.txt is also parsed, so a bad edit to the real
  vocabulary is caught by the unit suite rather than on the device.

  Import reach: standard library only. Run from the /opt/xbrain_v6 root:
      python3 -m pytest tests/asr/test_hotwords.py
"""
from __future__ import annotations

import os

import pytest

from services.asr.config import AsrConfig
from services.asr.core.hotwords import (
    HotwordsError,
    load_hotwords,
    load_token_vocab,
    materialize,
    parse_hotwords,
    select_encodable,
)


def test_phrase_is_split_into_per_character_units() -> None:
    # The engine matches per modeling unit (cjkchar), so every Chinese character must be
    # its own space-separated token. This is the failure that would silently disable
    # biasing if it regressed, which is why it is asserted literally.
    assert parse_hotwords("开始巡逻") == ["开 始 巡 逻"]


def test_already_spaced_lines_normalize_identically() -> None:
    # A line hand-written in the engine's own format must not become double-spaced: the
    # file's author should not have to know which convention is in use.
    assert parse_hotwords("前 进") == parse_hotwords("前进") == ["前 进"]


def test_comments_and_blank_lines_are_ignored() -> None:
    # Comments are what let the vocabulary document why each term is listed; they must
    # never reach the engine, where they would become nonsense hotwords.
    text = "# section header\n\n前进\n   \n#后退\n停止\n"
    assert parse_hotwords(text) == ["前 进", "停 止"]


def test_duplicates_are_dropped_preserving_order() -> None:
    # Listing a phrase twice would apply its bias twice -- an invisible way for one term
    # to dominate, since the author sees only two identical lines.
    assert parse_hotwords("前进\n停止\n前 进\n") == ["前 进", "停 止"]


def test_latin_runs_stay_one_token() -> None:
    # Non-CJK characters are not per-character modeling units, so exploding "GZH" into
    # "G Z H" would produce tokens the model has never seen. Runs stay grouped, and an
    # adjacent Chinese character still separates.
    assert parse_hotwords("GZH2机器狗") == ["GZH2 机 器 狗"]


def test_load_reads_a_file(tmp_path) -> None:
    # The file is UTF-8 by contract: a service inheriting an ascii locale from systemd
    # must still read a Chinese vocabulary, so the encoding is pinned in the loader.
    path = tmp_path / "hw.txt"
    path.write_text("# comment\n返航\n", encoding="utf-8")
    assert load_hotwords(str(path)) == ["返 航"]


def test_load_of_a_missing_file_raises() -> None:
    # A configured-but-absent vocabulary means the deploy intended biasing and did not get
    # it. Degrading silently would resurface much later as misrecognized commands.
    with pytest.raises(HotwordsError):
        load_hotwords("/nonexistent/hotwords.txt")


def test_empty_vocabulary_is_allowed(tmp_path) -> None:
    # A file with only comments is a legitimate "bias nothing" deployment; the recognizer
    # reads the empty list as "use plain greedy search".
    path = tmp_path / "hw.txt"
    path.write_text("# nothing here\n", encoding="utf-8")
    assert load_hotwords(str(path)) == []


def test_materialize_writes_engine_format_and_cleans_up() -> None:
    # The engine reads a PATH during construction, so the normalized text must exist as a
    # real file -- one entry per line, newline terminated -- and must not outlive that use.
    entries = ["前 进", "停 止"]
    with materialize(entries) as path:
        assert os.path.isfile(path)
        with open(path, "r", encoding="utf-8") as handle:
            assert handle.read() == "前 进\n停 止\n"
    assert not os.path.exists(path)


def test_materialize_cleans_up_after_a_failure() -> None:
    # Recognizer construction can raise (a missing model, an engine error) while the
    # temporary file is live; the cleanup runs in a finally so a failed startup still
    # leaves no stray copy of the vocabulary in /tmp.
    captured = {}
    with pytest.raises(HotwordsError):
        with materialize(["前 进"]) as path:
            captured["path"] = path
            raise HotwordsError("simulated construction failure")
    assert not os.path.exists(captured["path"])


def test_shipped_vocabulary_parses() -> None:
    # The real hotwords.txt is hand-maintained, so a bad edit should fail here rather than
    # on the device. Every entry must be non-empty and space-separated as the engine wants.
    entries = load_hotwords(AsrConfig().hotwords_file)
    assert entries
    for entry in entries:
        assert entry.strip() == entry
        assert "  " not in entry


def test_token_vocab_reads_symbols_not_ids(tmp_path) -> None:
    # tokens.txt is "SYMBOL ID" per line. Only the symbol is vocabulary; picking up the
    # numeric id as well would make every id string look like a valid modeling unit.
    path = tmp_path / "tokens.txt"
    path.write_text("<blk> 0\n前 12\n进 13\n\n", encoding="utf-8")
    assert load_token_vocab(str(path)) == {"<blk>", "前", "进"}


def test_token_vocab_reports_a_missing_file(tmp_path) -> None:
    # A missing symbol table means the model deploy is broken; it must surface as this
    # module's own error rather than a bare OSError from an unrelated-looking open().
    with pytest.raises(HotwordsError):
        load_token_vocab(str(tmp_path / "absent.txt"))


def test_select_encodable_splits_on_the_symbol_table() -> None:
    # The engine drops a phrase whose characters are not all in the symbol table, and it
    # does so with only a stderr warning. Doing the same test here is what turns that
    # invisible loss into something the service can report, so it is asserted exactly.
    vocab = {"前", "进", "开", "始"}
    usable, rejected = select_encodable(["前 进", "开 始 巡 逻"], vocab)
    assert usable == ["前 进"]
    assert rejected == [("开 始 巡 逻", ["巡", "逻"])]


def test_select_encodable_ignores_the_unit_separator() -> None:
    # Spaces separate modeling units and are never content, so they must not be looked up
    # as symbols -- otherwise every multi-character phrase would be rejected.
    usable, rejected = select_encodable(["前 进"], {"前", "进"})
    assert usable == ["前 进"]
    assert rejected == []


def test_shipped_vocabulary_is_checked_against_the_real_model() -> None:
    # The deployed export's symbol table cannot represent every character, so some shipped
    # phrases legitimately cannot be biased. What must hold is that the majority survive:
    # a vocabulary that mostly fails to load is a mismatch worth failing the suite over,
    # and this catches it on the dev box instead of as unexplained misrecognition later.
    config = AsrConfig()
    vocab = load_token_vocab(os.path.join(config.model_dir, "tokens.txt"))
    usable, rejected = select_encodable(load_hotwords(config.hotwords_file), vocab)
    assert len(usable) > len(rejected)
    # Every rejection must name at least one offending character, since that string is
    # what the startup warning shows the operator to act on.
    for _, missing in rejected:
        assert missing
