"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_asr_post_pipeline.py
Brief: 16 S3 ASR post-processing -- L1 dict + L2 pinyin, wired before the classifier

Description:
What was wrong. 16 S3 designs a three-layer correction between ASR and the
classifier (L1 exact dict, L2 near-homophone by pinyin distance, L3 closed-set
snap). The module existed, but nothing called it, L2 was a pass-through and the
dictionary was empty. On 2026-10-24 the robot heard "照明灯调到最暗" as
"照明灯调到最岸" -- "岸" and "暗" are both "an", a textbook L2 case -- and the
sentence missed D17 entirely.

What these cases pin.
  * L2 fixes the measured homophone errors: 最岸/对岸 -> 最暗 inside
    "照明灯调到...", and 盗闪 -> 爆闪 inside "在打开盗闪灯".
  * L2 never manufactures a command the operator did not say. A first prototype
    with every intent keyword as a target turned "灯开到最大" into "灯开倒退大"
    (A06 backward, a motion command), "关闭" into "关机", "后态" into "后退".
    Two rules close that, and each has a case: targets are D/E keywords only
    (payload and PTZ, the classes whose words were being misheard), and a
    two-character window must be an exact homophone (distance 0).
  * A word that is already a legal keyword of ANY class is never touched.
  * A tie between two targets is not resolved (16 S3.3: "多个同分候选 不替换").
  * Non-CJK characters in the text do not shift the pinyin alignment.
  * L1 replaces from the yaml dict, reloads it when the file changes, and keeps
    the previous dict when a reload fails (16 S3.2 hot update, CFG-30).
  * The safety bypass sees BOTH texts: raw first, corrected second (16 S4
    constraint 1). Correction can add a stop word; it can never hide one.
  * End to end through the turn handler: "照明灯调到最岸" reaches D17.

Why the sentences are real ones. Every "heard" string below was either read
off the 2026-10-24 p4 log on .1.26 or produced by replaying that day's mic
recordings through the live ASR service. Invented misreads would test what we
imagine paraformer gets wrong; these test what it did get wrong.

Why the negative cases outnumber the positive ones. A correction layer that
fires too often is worse than none: every sentence the operator says passes
through it, and one wrong rewrite can turn a light command into a motion
command (the prototype did exactly that). So the cases that say "this must NOT
change" are the ones that carry the safety argument; the positive cases only
show the layer is worth having.

Mutants this file kills (run by hand, see the batch commit message):
  * L2 targets widened to every class        -> never-manufactures case
  * protected-keyword skip removed            -> legal-keyword case
  * 2-character exact rule removed            -> two-character case
  * tie rule removed (first candidate wins)   -> tie case
  * per-character alignment replaced by a     -> non-CJK case
    whole-string pinyin index
  * bypass checks only the corrected text     -> raw-text bypass case
  * bypass checks only the raw text           -> corrected-text bypass case
  * handler skips the post-processor          -> end-to-end case

Boundaries: L3 (geo-name snap) is not exercised -- p4 has no geo catalogue
subscription yet, so its legal set is empty (documented in 16 S3). The ASR
model itself is not involved: every case starts from a transcript string.
"""
from __future__ import annotations

import json
import os

import pytest
import yaml

from xbrain.p4_agent.asr_post.three_layer import (
    AsrPostProcessor,
    L2Pinyin,
    L2_TARGET_CLASSES,
    load_l1_entries,
)
from xbrain.p4_agent.registry.intents import load_intent_registry
from xbrain.p4_agent.runtime.orchestrator_turn import (
    build_orchestrator,
    make_turn_handler,
)
from xbrain.p4_agent.runtime.turn_orchestrator import OrchestratorSession
from xbrain.p4_agent.session.chitchat import ChitchatResponder

# Pure unit cases: no zenoh, no ASR service, no LLM, no payload hardware.
# pypinyin is the only non-stdlib dependency beyond the repo itself
# (tests/requirements.txt).
pytestmark = pytest.mark.no_device

# The real tables, not hand-made ones. L2's safety argument is about the
# REAL keyword list: the dangerous rewrites (到最 -> 倒退) only exist because
# A06's real keywords are what they are. A synthetic registry would not
# contain them and the never-manufactures case would pass vacuously.
# The absolute paths match the rest of tests/p4_agent (the suite runs on the
# robot from /opt/xbrain_v6).
_INTENTS = "/opt/xbrain_v6/configs/intents.yaml"
# Only needed so the orchestrator can speak on declined turns.
_CHITCHAT = "/opt/xbrain_v6/configs/chitchat.yaml"


def _reg():
    """The production registry -- the vocabulary L2 corrects towards.

    Loaded fresh per call: no case can mutate a shared registry and leak the
    change into another case.
    """
    return load_intent_registry(yaml.safe_load(open(_INTENTS, encoding="utf-8")))


def _l2():
    """L2 built exactly as p4 __main__ builds it: D/E targets, all protected.

    If this helper and __main__._build_asr_post ever disagree, the cases below
    stop describing what runs on the robot. They are kept as two separate
    constructions on purpose (the test must not import the wiring it checks),
    so a change to one needs the matching change to the other.
    """
    reg = _reg()
    # Targets: only payload (D) and PTZ (E) keywords. 18 class letter is the
    # first character of the intent id ("D17", "E03").
    targets = [k for e in reg.entries if e.id[:1] in L2_TARGET_CLASSES
               for k in e.keywords]
    # Protected: every keyword of every class, so a window that already IS a
    # legal word (a motion verb included) is never rewritten into another.
    protected = [k for e in reg.entries for k in e.keywords]
    # max_distance and ignore_tone are the p4_agent.yaml asr_post values.
    return L2Pinyin(targets=targets, protected=protected, max_distance=1,
                    ignore_tone=True)


# --- L2 -------------------------------------------------------------------
# The positive half: the layer earns its place by fixing what was measured.

@pytest.mark.parametrize("heard, meant", [
    # Each pair is (what ASR returned, what the operator said). The meant
    # side is the D-class keyword sentence the operator was reading from 18.
    ("照明灯调到最岸", "照明灯调到最暗"),    # 2026-10-24 live log
    ("照明灯调到对岸", "照明灯调到最暗"),    # 2026-10-24 replay of the p4 cut
    ("在打开盗闪灯", "在打开爆闪灯"),        # 2026-10-24 live log
])
def test_l2_fixes_the_measured_homophone_errors(heard, meant):
    """The errors actually seen on the robot come back as the command.

    "最岸" is an exact homophone of "最暗" (zui an). "对岸" is not -- it is one
    letter off (dui/zui) -- and is fixed only because the window it sits in is
    longer than two characters ("调到对岸" vs the D17 keyword "调到最暗"), where
    distance 1 is allowed. "盗闪灯" (dao) vs "爆闪灯" (bao) is the same shape.

    The second assertion matters for 16 S3.5: a correction that happened but
    left no trail cannot be counted, so the dictionary can never be improved
    from field data.
    """
    text, corrections = _l2().apply(heard)
    # The sentence the operator meant, character for character.
    assert text == meant, (heard, text, corrections)
    # And the change is reported, not silent.
    assert corrections, "a correction happened but was not reported"


# The negative half: these carry the safety argument (see the module header).

@pytest.mark.parametrize("text", [
    "灯开到最大",      # prototype turned this into 灯开倒退大 (A06 backward)
    "声音调到最大",    # same failure: 到最 -> 倒退
    "好的关闭共闪灯",  # prototype: 关闭 -> 关机
    "在打开后态中",    # prototype: 后态 -> 后退
    "今天天气不错",    # ordinary speech must pass through untouched
    "到这里啊",        # the 2026-10-24 misread of 调到最暗; nothing fits it
    # Near-misses of NON-target keywords at 3+ characters, where distance 1
    # is allowed. Only the D/E restriction keeps these: with every class as
    # a target, L2 rewrites them into the command (measured 2026-10-25).
    "关鸡吧",          # -> 关机吧, H08 shutdown
    "往后腿",          # -> 往后退, A06 move_backward
    "站齐来",          # -> 站起来, A03 stand
])
def test_l2_never_manufactures_a_command_the_operator_did_not_say(text):
    """None of these may change: a wrong correction is worse than none.

    The first four are the prototype's actual rewrites. Each one turned a
    harmless sentence into a motion or power command -- "倒退" and "后退" are
    A-class motion keywords, "关机" is a shutdown. The robot would have
    executed a command nobody gave. They stay unchanged now for two
    independent reasons: A/B/C words are not L2 targets at all, and a
    two-character window like "到最" needs an exact homophone, which "倒退"
    (dao tui) is not.

    The next two show the layer leaves ordinary speech and an un-fixable
    misread alone. "到这里啊" is deliberately not corrected into anything:
    the right answer for it is the B01 clarify path, not a guess.

    The last three are the ones the 2-character rule cannot protect: three
    characters, one letter (or zero) away from a motion or shutdown keyword.
    A misheard motion word therefore reaches the classifier as heard and is
    handled there (keyword miss -> tier-2 LLM / clarify), never turned into
    the motion command by a spelling fix.
    """
    out, corrections = _l2().apply(text)
    # Unchanged, and no correction reported.
    assert out == text and corrections == [], (text, out, corrections)


@pytest.mark.parametrize("text", [
    # Target classes (D / E): a correct target word must not drift to a
    # neighbouring target word.
    "关闭照明灯",    # D02 light_off
    "打开爆闪灯",    # D06 strobe_on
    "云台向左",      # E01 ptz_move
    "左右扫一遍",    # E07 ptz_scan
    # A word shared by two classes (C04 and D12, the CF-1 pair): protected
    # whichever class finally takes it.
    "停止喊话",      # C04 exit_broadcast / D12 speak_stop
    "别喊了",        # D12 speak_stop
    # Non-target classes: protected even though L2 never corrects towards
    # them -- the window must reach the classifier as the word it is.
    "前进",          # A05 move_forward
    "后退",          # A06 move_backward
    "站起来",        # A03 stand
])
def test_a_legal_keyword_of_any_class_is_left_alone(text):
    """Already-correct commands, including non-target classes, are untouched.

    Why this needs its own rule. Two legal keywords can sit at pinyin distance
    1 from each other. Without the protected set, a correctly heard D keyword
    could be "corrected" into a neighbouring D keyword, and the operator's
    exact words would be replaced by a different command. The motion words
    (前进 / 后退 / 站起来) are in the list because protection covers every
    class, not just the targets: a window that already spells a legal A-class
    word must reach the classifier as that word.
    """
    out, corrections = _l2().apply(text)
    # Byte-identical output and an empty correction trail.
    assert out == text and corrections == []


def test_a_two_character_window_must_be_an_exact_homophone():
    """2 chars: distance 0 only. 'duian' vs 'zuian' is distance 1 -> kept.

    Two syllables give a one-letter edit no room: almost every two-character
    word has a legal neighbour one letter away, which is how "关闭" became
    "关机" (guanbi / guanji: one letter, b -> j; "关机" is H08 shutdown). So
    two-character windows are fixed only by a pure homophone.

    The single-target L2 here isolates the rule from the registry: whatever
    the keyword list looks like, a two-character window at distance 1 stays.
    """
    l2 = L2Pinyin(targets=["最暗"], protected=["最暗"], max_distance=1,
                  ignore_tone=True)
    # zui an == zui an: an exact homophone, fixed.
    assert l2.apply("最岸")[0] == "最暗"
    # one letter off: allowed for 3-5 chars, not for 2.
    assert l2.apply("对岸")[0] == "对岸"


def test_a_tie_between_targets_is_not_resolved():
    """16 S3.3: equal-score candidates -> no replacement.

    "报闪灯" (bao) is distance 0 from both "暴闪灯" and "爆闪灯" (both bao).
    Picking either would be a coin toss presented as a correction. The design
    hands a tie to L3 / the LLM; here (no L3) the text is left as heard. An
    implementation that took the first candidate in list order would rewrite
    it and fail.
    """
    l2 = L2Pinyin(targets=["暴闪灯", "爆闪灯"], protected=["暴闪灯", "爆闪灯"],
                  max_distance=1, ignore_tone=True)
    # Two equally good candidates: neither wins.
    assert l2.apply("报闪灯")[0] == "报闪灯"


def test_non_cjk_characters_do_not_shift_the_alignment():
    """Digits/latin between Chinese text must not misalign the pinyin.

    pypinyin returns a run of non-CJK characters as ONE item. An index built
    from that list would be off by the run's length from the character index,
    so the replacement would land on the wrong characters. Per-character pinyin over
    each CJK run keeps the two indexes the same.
    """
    # Two non-CJK characters ("12"), so pypinyin's one-item-per-run output is
    # one item SHORTER than the text. A single digit would not show it: "3"
    # is one character and one item, and a whole-string index lines up by
    # luck. The whole-string mutant returns "12号打开爆闪灯灯" here.
    out, _ = _l2().apply("12号点打开盗闪灯")
    # The digits and the place word are untouched; only 盗 -> 爆.
    assert out == "12号点打开爆闪灯", out


# --- L1 -------------------------------------------------------------------
# L1 is the field-maintained part: a line added on site must take effect
# without a restart, and a bad line must not take the robot's dictionary away.

def _write(path, mapping):
    """Write an asr_dict.yaml with the given exact-replace table.

    Same top-level shape as configs/asr_dict.yaml (`exact:` mapping), so the
    loader under test is the production loader reading a production-shaped
    file.
    """
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump({"exact": mapping}, fh, allow_unicode=True)


def _bump_mtime(path):
    """Move the file's mtime two seconds forward.

    The reload trigger is an mtime change. The repo lives on sshfs with
    one-second mtime granularity (CLAUDE.md 7.2.1), so two writes in the same
    second look unchanged; bumping by two seconds makes every reload case
    deterministic instead of depending on how fast the test runs.
    """
    st = os.stat(path)
    os.utime(path, ns=(st.st_atime_ns, st.st_mtime_ns + 2_000_000_000))


def test_l1_replaces_and_reloads_when_the_file_changes(tmp_path):
    """16 S3.2: a new line in the dict takes effect without a restart.

    The third assertion is the half that is easy to miss: a reload REPLACES
    the table, it does not merge. An entry removed on site must stop applying,
    or a wrong entry could never be withdrawn without restarting p4.
    """
    p = tmp_path / "asr_dict.yaml"
    _write(p, {"共闪灯": "爆闪灯"})
    post = AsrPostProcessor(str(p), l2=None, log_corrections=False)
    # The first table applies.
    assert post.apply("关闭共闪灯")[0] == "关闭爆闪灯"
    # Edit the file on site: one entry swapped for another.
    _write(p, {"照平灯": "照明灯"})
    _bump_mtime(p)
    # The new entry applies on the very next turn ...
    assert post.apply("打开照平灯")[0] == "打开照明灯"
    # ... and the removed one no longer does.
    assert post.apply("关闭共闪灯")[0] == "关闭共闪灯"


def test_a_broken_dict_keeps_the_previous_one(tmp_path):
    """A bad edit on site must not wipe the working dictionary.

    The alternative behaviours are both worse: raising would take the voice
    loop down on a typo, and falling back to an empty table would silently
    switch off every correction the site depends on. Keeping the last good
    table (and logging an ERROR, which the implementation does) leaves the
    robot as it was until someone fixes the file.
    """
    p = tmp_path / "asr_dict.yaml"
    _write(p, {"共闪灯": "爆闪灯"})
    post = AsrPostProcessor(str(p), l2=None, log_corrections=False)
    # Not valid yaml: an unterminated flow sequence.
    p.write_text("exact: [this is: not a mapping", encoding="utf-8")
    _bump_mtime(p)
    # The old entry still applies.
    assert post.apply("关闭共闪灯")[0] == "关闭爆闪灯"


def test_l1_keys_outside_two_to_five_characters_are_refused(tmp_path):
    """16 S3.2: L1 keys are 2-5 character phrases.

    A one-character key is the dangerous case: "灯" -> "等" would rewrite every
    light command in every sentence. The loader refuses it outright rather
    than skipping it, so the person editing the file sees the error instead of
    wondering why their entry does nothing.
    """
    p = tmp_path / "asr_dict.yaml"
    _write(p, {"灯": "等"})
    with pytest.raises(ValueError):
        load_l1_entries(str(p))


def test_corrections_name_their_layer(tmp_path):
    """The correction trail says which layer did what (16 S3.5 fields).

    16 S3.5 lists `layer` as a field of asr_correction. The table does not
    exist yet (the INFO log line stands in for it), but the trail already
    carries the layer so the log line can be counted per layer -- which is
    how L1 entries are meant to be found (frequent L2 fixes become L1 lines).
    """
    p = tmp_path / "asr_dict.yaml"
    _write(p, {"共闪灯": "爆闪灯"})
    post = AsrPostProcessor(str(p), l2=_l2(), log_corrections=False)
    # One L1 fix and one L2 fix in the same sentence.
    text, corr = post.apply("调到最岸 关闭共闪灯")
    assert text == "调到最暗 关闭爆闪灯"
    # Both layers appear in the trail, each under its own name.
    assert {c[0] for c in corr} == {"L1", "L2"}


# --- wiring ---------------------------------------------------------------
# The orchestrator half: what the corrected text is used for, and what the
# raw text is still used for.

def _orch():
    """A real orchestrator with the production registry and no LLM.

    No tier-2 function: every case below is decided by the safety bypass or
    the keyword layer, and a stray LLM call would be a bug, not a fallback.
    """
    cc = ChitchatResponder(yaml.safe_load(open(_CHITCHAT, encoding="utf-8")))
    return build_orchestrator(_reg(), cc, l2_timeout_ms=8000)


def test_the_bypass_also_matches_the_raw_text():
    """Raw '停止' corrected into something else still fires the estop.

    16 S4: the safety bypass must not be weakened by anything placed in front
    of it. If a correction ever rewrote a heard stop word into a light
    command, a bypass that looked only at the corrected text would miss the
    stop. The handler therefore passes the raw transcript alongside; this
    case feeds an extreme pair (raw "停止", corrected "调到最暗") to prove
    the raw side alone is enough.
    """
    d = _orch().handle_turn("调到最暗", OrchestratorSession(), 1_000,
                            raw_text="停止")
    assert d.kind == "bypass"


def test_the_bypass_also_matches_the_corrected_text():
    """A stop word that only appears after correction also fires.

    The other direction: a misheard stop ("听止") that the correction layer
    repairs must reach the bypass as a stop. Checking only the raw text would
    throw away the one correction that matters most.
    """
    d = _orch().handle_turn("停止", OrchestratorSession(), 1_000,
                            raw_text="听止")
    assert d.kind == "bypass"


class _FixedPost:
    """A post-processor double: maps one sentence, reports one correction.

    The handler case below is about the WIRING (does the handler call the
    post-processor and classify its output), not about L2's rules, which the
    cases above already cover. A double keeps that case independent of the
    keyword list.
    """

    def __init__(self, src, dst):
        # The one sentence this double rewrites, and what it rewrites it to.
        self.src, self.dst = src, dst

    def apply(self, raw):
        # Same return shape as AsrPostProcessor.apply: (text, corrections).
        if raw == self.src:
            return self.dst, [("L2", self.src, self.dst, 0)]
        return raw, []


def test_the_turn_handler_classifies_the_corrected_text():
    """End to end: the misheard sentence reaches D17 after correction.

    This is the 2026-10-24 failure replayed through the real handler and
    orchestrator: the heard sentence goes in, a cmd/payload frame for D17 at
    level "min" (darkest) comes out. Before the wiring, the same input missed
    D17 and the light did not change.
    """
    handler = make_turn_handler(_orch(), OrchestratorSession(),
                                asr_post=_FixedPost("照明灯调到最岸",
                                                    "照明灯调到最暗"))
    pubs = handler("照明灯调到最岸")
    keys = [k for k, _ in pubs]
    # A payload command was published ...
    assert "cmd/payload" in keys, keys
    body = json.loads(dict(pubs)["cmd/payload"].decode("utf-8"))
    # ... for the light-level intent, at its darkest setting.
    assert body["intent_id"] == "D17" and body["level"] == "min"


def test_without_a_post_processor_the_handler_is_unchanged():
    """asr_post=None keeps the old behaviour (the pre-2026-10-25 handler).

    asr_post is optional: a caller that passes none gets the handler exactly
    as it was. The only production caller is runtime/main_wiring.py, which
    passes what __main__._build_asr_post returned: a processor (L1 alone when
    pypinyin is missing), or None when the asr_post.* config keys are
    unusable -- logged at ERROR, and the turn then runs exactly as here. This case also shows the
    end-to-end case above is not passing for some other reason -- without the
    correction, the same sentence does NOT produce the payload command.
    """
    handler = make_turn_handler(_orch(), OrchestratorSession())
    keys = [k for k, _ in handler("照明灯调到最岸")]
    assert "cmd/payload" not in keys
