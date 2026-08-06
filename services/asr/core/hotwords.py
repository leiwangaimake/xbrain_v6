"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: hotwords.py
Brief: Load the hand-written hotword vocabulary and materialize it for sherpa-onnx.

Description:
  Hotword biasing is a hard requirement for this service: the near-field 功能1 dialogue is
  mostly short robot commands, and an unbiased decoder happily returns a homophone that
  the intent layer cannot match. sherpa-onnx supplies the biasing itself (it is prebuilt
  inference infrastructure, outside R0); the vocabulary LOADING is ours to write, and it
  is what this module does -- plan section 11 names "热词加载" as our glue explicitly.

  The engine wants a file whose every line is one phrase with its modeling units already
  separated by spaces ("前 进"). That is a bad format to maintain by hand: the spaces are
  invisible in review, a missing one silently disables that single phrase, and the file
  can carry no comments explaining why a term is listed. So services/asr/hotwords.txt is
  authored in the natural form -- one phrase per line, "#" comments and blank lines
  allowed -- and this module normalizes it to the engine's form.

  Because sherpa-onnx takes a PATH (it reads the vocabulary itself at recognizer
  construction) and not a list of strings, the normalized text has to exist as a file.
  materialize() writes it to a temporary file, yields the path for exactly the duration
  of that construction, and removes it afterwards. The alternative -- keeping the repo
  file in the engine's format -- was rejected because it trades a 20-line, tested
  transformation for a permanently error-prone authoring experience. Writing the
  normalized copy back into the source tree was likewise rejected: a service must not
  mutate its own deployment at startup.

  Tokenization rule: the recognizer runs with modeling_unit="cjkchar", so each Chinese
  character is its own unit and must stand as its own token. Runs of non-CJK characters
  (a latin model name, a digit) are kept together as one token instead of being exploded
  letter by letter, which would not correspond to any unit the model knows.

  Vocabulary check: not every phrase a human would write can actually be biased. A symbol
  table holds a bounded set of units, and a character outside it is only ever emitted as a
  raw-byte sequence, which cannot appear in a biasing entry. The engine handles that by
  warning on stderr and dropping the phrase -- so an authored vocabulary can quietly shrink
  with nothing in the service log to say so. select_encodable() performs the same test up
  front, turning the loss into a named, counted startup warning instead.

  * How much this matters was measured on 2026-08-03: the icefall multi-zh-hans export
  carries only 2000 entries (about 1400 single Chinese characters plus 256 byte symbols),
  and 15 of the 45 phrases in hotwords.txt were unencodable against it -- including 巡逻,
  路径, 趴下, 返航 and 驱离, i.e. most of the words the vocabulary exists to protect. The
  same file against a 5539-character export encodes all 45.

  ! WHEN THIS MODULE RUNS AT ALL: only under the transducer family. sherpa-onnx exposes
  contextual biasing on no other architecture, so recognizer.py does not read the
  vocabulary when the loaded family cannot bias -- and the current default family
  (paraformer) cannot. This module is therefore dormant in the default deployment and
  live again the moment ASR_MODEL_FAMILY selects a transducer. It is kept rather than
  deleted because that switch is expected: the model choice is still being measured, and
  the biasing measurement (worth about one point of exact-match once the vocabulary gap is
  closed) is part of what that decision rests on.
"""
from __future__ import annotations

import os
import tempfile
from contextlib import contextmanager
from typing import Iterator, List, Sequence, Set, Tuple

# Comment marker for the authored file. A line whose first non-space character is this is
# dropped entirely, which is what lets the vocabulary document itself.
_COMMENT_PREFIX = "#"

# The CJK Unified Ideographs block. This is the range that modeling_unit="cjkchar" treats
# as one-character-per-unit, so it is exactly the test for "this character is its own
# token". Rare characters outside the basic block fall into the non-CJK branch and are
# grouped -- acceptable, because the model's 2000-token vocabulary does not cover them
# either, so such a phrase could not be biased regardless of how it were split.
_CJK_FIRST = "一"
_CJK_LAST = "鿿"


class HotwordsError(ValueError):
    """Raised when the hotword vocabulary cannot be read or parsed.

    House rule bans bare Exception. A dedicated type lets app startup report a bad
    vocabulary as its own operator-fixable fault, clearly distinct from a missing model
    file or an engine failure -- three problems with three different remedies. ValueError
    is the base because an unreadable vocabulary path is a bad configured value.
    """


def _is_cjk(char: str) -> bool:
    """Report whether a character is a CJK ideograph that forms its own modeling unit.

    Args:
        char: a single character.

    Returns:
        True when the character lies in the CJK Unified Ideographs block.
    """
    return _CJK_FIRST <= char <= _CJK_LAST


def _tokenize(phrase: str) -> str:
    """Rewrite one phrase into the engine's space-separated modeling units.

    Args:
        phrase: a phrase as authored, e.g. "开始巡逻" or already-spaced "开 始 巡 逻".

    Returns:
        The phrase with one space between modeling units, e.g. "开 始 巡 逻".

    Whitespace already present in the input is simply a separator and is discarded before
    regrouping, so a phrase authored either way normalizes to the same result -- which is
    the point: the file's author should not have to know the engine's convention, and a
    hand-spaced legacy line must not become double-spaced garbage.
    """
    tokens: List[str] = []
    # Accumulates a run of adjacent non-CJK characters (latin, digits) so they emit as one
    # token rather than one token per character.
    pending = ""
    for char in phrase:
        if char.isspace():
            # A space ends any run in progress; it carries no meaning of its own here.
            if pending:
                tokens.append(pending)
                pending = ""
            continue
        if _is_cjk(char):
            # A CJK character both closes the current run and stands alone as a unit.
            if pending:
                tokens.append(pending)
                pending = ""
            tokens.append(char)
            continue
        pending += char
    # Flush a run that reached the end of the phrase without a terminator.
    if pending:
        tokens.append(pending)
    return " ".join(tokens)


def parse_hotwords(text: str) -> List[str]:
    """Turn the authored vocabulary text into normalized, deduplicated entries.

    Args:
        text: the full contents of a hotwords file.

    Returns:
        One normalized entry per usable line, in file order, with duplicates removed.

    Duplicates are dropped because listing a phrase twice would apply its bias twice --
    an invisible way to make one term dominate, since the file's author sees only two
    identical lines. Order is preserved so the materialized file stays diff-friendly
    against the source, which matters when a decode is being explained after the fact.

    Splitting parsing (text in, entries out) from reading (path in) is what lets the unit
    tests cover every format rule without touching the filesystem.
    """
    entries: List[str] = []
    # Membership is tested against the NORMALIZED form, so "前进" and "前 进" on two lines
    # count as the same phrase -- which is the whole point, since they are identical to
    # the engine and the author cannot see the difference.
    seen = set()
    for raw in text.splitlines():
        line = raw.strip()
        # Blank lines and comments are formatting, not vocabulary. Every line that gets
        # past this check holds at least one non-space character, and _tokenize turns
        # each of those into a token, so the normalized result is never empty.
        if not line or line.startswith(_COMMENT_PREFIX):
            continue
        normalized = _tokenize(line)
        if normalized in seen:
            continue
        seen.add(normalized)
        entries.append(normalized)
    return entries


def load_hotwords(path: str) -> List[str]:
    """Read and normalize the hotword vocabulary from a file.

    Args:
        path: filesystem path to the authored hotwords file (UTF-8).

    Returns:
        The normalized entries; an empty list when the file has no vocabulary lines.

    Raises:
        HotwordsError: if the file cannot be read.

    An empty result is NOT an error: a deployment may legitimately want unbiased
    decoding, and the recognizer treats "no entries" as "run plain greedy search". A
    missing or unreadable file IS an error, because it means the deployment intended a
    vocabulary and did not get one -- silently degrading to no biasing there would show
    up much later as unexplained misrecognition of the very commands that matter most.
    """
    try:
        # Explicit UTF-8 rather than the locale default: the vocabulary is Chinese, and a
        # service inheriting a POSIX/ascii locale from systemd would otherwise fail to
        # decode it -- a fault that would appear only on the deployed box.
        with open(path, "r", encoding="utf-8") as handle:
            text = handle.read()
    except OSError as exc:
        # Chain the original so errno and the path survive into the log.
        raise HotwordsError(f"cannot read hotwords file {path}: {exc}") from exc
    return parse_hotwords(text)


def load_token_vocab(path: str) -> Set[str]:
    """Read the model's symbol table and return the units it can emit.

    Args:
        path: filesystem path to the model's tokens.txt (UTF-8).

    Returns:
        Every symbol in the table, as a set. Callers test single characters against it.

    Raises:
        HotwordsError: if the file cannot be read.

    The file's format is one "SYMBOL ID" pair per line. The symbol is taken with rsplit on
    the LAST space so a symbol that itself contains a space survives; splitting on the
    first space would truncate it and silently shrink the vocabulary.
    """
    try:
        with open(path, "r", encoding="utf-8") as handle:
            lines = handle.read().splitlines()
    except OSError as exc:
        raise HotwordsError(f"cannot read tokens file {path}: {exc}") from exc
    # A trailing blank line is normal and carries no symbol, so empty lines are skipped
    # rather than contributing an empty-string member that would match nothing anyway.
    return {line.rsplit(" ", 1)[0] for line in lines if line}


def select_encodable(
    entries: Sequence[str], vocab: Set[str]
) -> Tuple[List[str], List[Tuple[str, List[str]]]]:
    """Split entries into those this model can bias and those it cannot.

    Args:
        entries: normalized entries from parse_hotwords/load_hotwords.
        vocab: the model's symbol set, from load_token_vocab.

    Returns:
        (usable, rejected), where rejected pairs each dropped entry with the sorted
        characters that are absent from the model's symbol table.

    This check exists because sherpa-onnx does it too, but silently: with
    modeling_unit="cjkchar" it splits every entry into single characters, looks each one
    up, prints a C++ warning to stderr for a miss, and then DROPS the whole phrase. On the
    deployed model that quietly disabled a third of the shipped vocabulary -- and a
    disabled hotword is invisible, because decoding still succeeds, just without the bias
    that the phrase was listed to provide. Doing the same test here turns that into a
    startup log line naming the phrase and the offending characters.

    A character can be absent because this export's symbol table is a 2000-entry
    sentencepiece vocabulary with byte fallback: it holds the ~1400 commonest Chinese
    characters, and anything rarer is emitted as raw UTF-8 byte symbols instead. Such a
    phrase is still TRANSCRIBED correctly (the bytes decode back to the character); it
    simply cannot be boosted, because the biasing FST is built over whole symbols and
    there is no way to write a byte sequence in the entry format.

    Entries are checked per character rather than per space-separated token because that
    is what the engine does regardless of how the entry is spaced.
    """
    usable: List[str] = []
    rejected: List[Tuple[str, List[str]]] = []
    for entry in entries:
        # Spaces are the engine's unit separator, never content, so they are removed
        # before the per-character test rather than being looked up as a symbol.
        missing = sorted({char for char in entry if not char.isspace() and char not in vocab})
        if missing:
            rejected.append((entry, missing))
            continue
        usable.append(entry)
    return usable, rejected


@contextmanager
def materialize(entries: Sequence[str]) -> Iterator[str]:
    """Write entries to a temporary engine-format file for the duration of the block.

    Args:
        entries: normalized entries as returned by load_hotwords/parse_hotwords.

    Yields:
        The path of a temporary file holding one entry per line, UTF-8, newline
        terminated -- exactly the format sherpa-onnx's hotwords_file expects.

    The lifetime is a context manager because the engine reads the file once, while the
    recognizer is being constructed, and never again: keeping the temporary file alive
    past that point would leave a stray copy of the vocabulary in /tmp for every service
    start. delete=False plus an explicit unlink is used instead of NamedTemporaryFile's
    own auto-delete because the file must be CLOSED (fully flushed) while still existing,
    so another process -- the engine's C++ side -- can open it by path.

    The removal runs in a finally, so a recognizer that fails to construct still leaves no
    file behind; and it tolerates an already-absent file so cleanup can never mask the
    original construction error with a second, meaningless one.
    """
    handle = tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", suffix=".txt", prefix="asr_hotwords_", delete=False
    )
    path = handle.name
    try:
        # One entry per line, each newline terminated -- including the last, since a file
        # whose final line lacks a newline is a classic way to lose that last hotword in
        # a line-oriented reader.
        for entry in entries:
            handle.write(entry + "\n")
        # Close before yielding so the engine sees a complete file, not a partially
        # flushed one still held open by this process.
        handle.close()
        yield path
    finally:
        # close() is idempotent, so this covers the case where the write itself raised.
        handle.close()
        try:
            os.unlink(path)
        except OSError:
            # Cleanup is best effort by design: if /tmp is read-only or the file is
            # already gone, that must not replace whatever real error is propagating.
            pass
