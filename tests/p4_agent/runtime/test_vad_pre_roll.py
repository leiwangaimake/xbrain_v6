"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_vad_pre_roll.py
Brief: VAD pre-roll -- the audio from before the energy onset rides into the utterance

Description:
11 S8.7.2 registers vad_pre_roll_ms = 300: "cache the audio from BEFORE the VAD
trigger, so the first syllable is not eaten". The segmenter opened the utterance
on the first frame whose mean-abs energy crossed the threshold and threw away
everything before it. A soft onset (a consonant, the rising edge of a vowel) sits
below that threshold, so the head of every utterance was cut.

Measured 2026-10-24 on .1.26 (recordings replayed through the same ASR service):
the utterance cut where p4 cut it came back "照明灯调到对岸"; the same audio started
0.4 s earlier came back "照明灯调到最暗". And "调到最暗" came back "到这里啊" -- the
leading "调" gone, which then also tripped the B01 single-character keyword.

What the cases pin, and why each needs its own case:
  * the closed utterance BEGINS with exactly the last N quiet frames before the
    onset (N = pre_roll_ms / frame_ms), in order. A do-nothing implementation
    starts with the onset frame and fails here; one that keeps the frames but
    reverses them (a deque popped from the wrong end) fails the ORDER check.
  * frames older than N are NOT kept. An unbounded buffer passes the first case
    and then ships every second of room noise since the last turn into ASR --
    the decode gets slower and the transcript picks up noise words.
  * a lead-in shorter than N is kept whole, and nothing is invented to pad it.
  * pre-roll frames are not counted as speech, so min_utterance_ms still drops a
    click that has a quiet lead-in. Counting them would turn every door slam
    into a full ASR round trip.
  * the buffer restarts after each utterance, so one turn's lead-in can never
    leak into the next.
  * pre_roll_ms = 0 is the old onset behaviour; a negative value is refused.

How the frames are built. Each quiet frame is filled with a small distinct value
(its "tag", always below the energy threshold) so the test can read back WHICH
frames ended up in the utterance by sampling one value per frame. Voiced frames
use 500 or 600, well above the threshold of 100.

Why 300 ms and not "enough to be safe". 11 S8.7.2 registers the value. Longer
pre-roll is not free: every extra frame is decoded by ASR on every turn, and the
paraformer decode time grows with audio length (RTF ~0.12 measured on .1.26).
300 ms covers a consonant onset; it does not try to cover a speaker who pauses
mid-word, which is tail_silence_ms's job, not this one.

What these cases do NOT cover. They do not prove the ASR then reads the word
correctly -- that depends on the model and was checked by replaying the
2026-10-24 recordings through the live ASR service (see the header above). They
also do not cover p2: the 11 S8.7.2 table places VAD gating in p2_core
audio_io, which today publishes every frame; the segmenter here is p4's, and it
needs the pre-roll either way, because a frame p2 forwards early is still
discarded by p4 if p4 opens the utterance on the energy onset.

How the bug was found, so the next reader can reproduce it. Two recordings
were taken off rt/audio/mic with a read-only tap while an operator spoke a light
command about one metre from the HK-MIC. Each utterance was then cut twice from
the same recording -- once at the window p4 had sent to ASR, once 0.4 s earlier
-- and both were POSTed to asr-service /v1/audio/transcriptions. The earlier cut
recovered the command word for word. Speech level was not the issue: the frames
peaked between -11 and -16 dBFS, far above the threshold of 300 mean-abs.

Why the fix lives in the segmenter and not in a threshold change. Lowering
energy_threshold would also open utterances earlier, but it opens them on room
noise as well -- every HVAC hum and footstep becomes an ASR call. Pre-roll keeps
the threshold where it is and only changes WHAT is sent once an utterance opens.

Mutants this file kills (each was run by hand while writing it):
  * pre-roll ignored (utterance starts at the onset)     -> first three cases
  * buffer never trimmed                                -> second case
  * buffer kept newest-first                            -> first case (order)
  * pre-roll counted into speech_frames                 -> min-utterance case
  * buffer not cleared at the onset                     -> restart case

Boundaries: pure segmenter; frame transport (rt/audio/mic) and ASR are not
involved. The live wiring (p4 __main__ passes --vad-pre-roll-ms) is covered by
restarting p4 on the robot, not here.
"""
from __future__ import annotations

import pytest

from xbrain.p4_agent.runtime.vad import (
    VadConfig,
    VadConfigError,
    VadState_,
    feed_frame,
)

pytestmark = pytest.mark.no_device

# 20 ms at 16 kHz: the frame size p2 publishes on rt/audio/mic (11 S8.7.1).
FRAME = 320
# Energy threshold for these cases; quiet tags stay strictly below it.
# (Production runs at 300; the value only has to separate quiet from voiced.)
THRESH = 100


def _cfg(pre_roll_ms: int) -> VadConfig:
    """A segmenter whose only variable is the pre-roll.

    The other three values are small on purpose: the cases stay a few dozen
    frames long and every frame count in them can be checked by hand.

    tail 60 ms = three silent frames close an utterance; min 40 ms = two
    voiced frames are the shortest utterance that is kept.
    """
    return VadConfig(energy_threshold=THRESH, tail_silence_ms=60,
                     min_utterance_ms=40, frame_ms=20, pre_roll_ms=pre_roll_ms)


def _quiet(tag: int):
    """A below-threshold frame whose constant value identifies it.

    The tag doubles as the frame's identity when it is read back out of the
    closed utterance, so it must stay strictly below the threshold or the
    frame would open the utterance itself.
    """
    assert 0 < tag < THRESH
    return [tag] * FRAME


def _voiced(level: int = 500):
    """An above-threshold frame (speech).

    The level is a marker as much as a loudness: the restart case uses 600 for
    the second utterance so its onset can be located in the read-back values.
    """
    return [level] * FRAME


def _run(cfg, frames):
    """Feed frames through one segmenter, return the last closed utterance.

    Cases that expect exactly one utterance use this; the restart case drives
    the state by hand because it needs both utterances, not just the last.
    Returning the LAST closed one (rather than the first) means a case that
    accidentally produces two utterances still fails on its content checks
    instead of passing on a stale first one.
    """
    # One fresh state per run: the segmenter is stateful, and reusing a state
    # across cases would let one case's lead-in leak into the next.
    st = VadState_()
    # The segmenter returns an utterance only on the frame that closes it.
    out = None
    for f in frames:
        r = feed_frame(st, f, cfg)
        # Keep the last closed utterance; None means "still open or idle".
        if r is not None:
            out = r
    return out


def _close_tail():
    """Three silent frames: enough trailing silence (60 ms) to close.

    Silence here is literal zeros, which is below every threshold and carries
    no tag; it marks the end of an utterance without being mistaken for a
    lead-in frame when the per-frame values are read back.
    """
    return [[0] * FRAME] * 3


def test_the_utterance_begins_with_the_last_pre_roll_frames_in_order():
    """The head of the utterance is the last 15 quiet frames, oldest first.

    300 ms / 20 ms = 15 frames. Twenty quiet frames tagged 1..20 go in, so the
    kept ones must be 6..20 and the dropped ones 1..5. Reading one sample per
    frame recovers the tags in the order they sit in the utterance.
    """
    # 300 ms of pre-roll at 20 ms frames.
    cfg = _cfg(300)
    # Twenty identifiable quiet frames, then five voiced, then the close.
    quiet = [_quiet(t) for t in range(1, 21)]
    utter = _run(cfg, quiet + [_voiced()] * 5 + _close_tail())
    # Five voiced frames (100 ms) clear the 40 ms floor, so it must close.
    assert utter is not None
    # One sample per frame is enough: every frame is a constant.
    head = [utter[i * FRAME] for i in range(15)]
    # Oldest kept frame first; the five oldest (1..5) were dropped.
    assert head == list(range(6, 21)), head
    # The onset frame follows the pre-roll immediately: nothing is lost or
    # duplicated at the seam.
    assert utter[15 * FRAME] == 500


def test_frames_older_than_the_pre_roll_window_are_not_kept():
    """Twice the window of quiet frames still yields exactly 15 of them.

    The total length is the strongest form of the check: 15 pre-roll + 5
    voiced + 3 tail frames and nothing more. An unbounded buffer would make it
    40 + 5 + 3.
    """
    # Same window as above.
    cfg = _cfg(300)
    # Forty quiet frames: twice the window, so the bound must bite.
    quiet = [_quiet(t) for t in range(1, 41)]
    utter = _run(cfg, quiet + [_voiced()] * 5 + _close_tail())
    # Exact length: the window, the speech, the closing tail.
    assert len(utter) == (15 + 5 + 3) * FRAME
    # Only the newest 15 (tags 26..40) survive.
    assert all(utter[i * FRAME] >= 26 for i in range(15))


def test_a_short_lead_in_is_kept_whole():
    """Speech right after start-up: two quiet frames, both kept, none invented.

    Padding a short lead-in to the full window with silence would look
    harmless but would change the timing every downstream consumer sees.
    """
    # Window of 15, but only two quiet frames exist before the onset.
    cfg = _cfg(300)
    utter = _run(cfg, [_quiet(7), _quiet(8)] + [_voiced()] * 5 + _close_tail())
    # Both lead-in frames, in order, at the very start.
    assert [utter[0], utter[FRAME]] == [7, 8]
    # And the onset comes third: no silence was synthesised in front of them.
    assert utter[2 * FRAME] == 500


def test_pre_roll_does_not_count_as_speech_for_the_min_utterance_floor():
    """One voiced frame (20 ms) is below the 40 ms floor: dropped.

    The quiet lead-in in front of it must not lift it over the floor -- the
    floor exists so a single click does not cost a whole ASR round trip, and a
    click always has a quiet lead-in.
    """
    # Full window of quiet lead-in in front of a single voiced frame.
    cfg = _cfg(300)
    quiet = [_quiet(t) for t in range(1, 21)]
    # 15 pre-roll frames + 1 voiced would be 320 ms if pre-roll counted as
    # speech; only the 20 ms voiced frame counts, which is below the floor.
    assert _run(cfg, quiet + [_voiced()] + _close_tail()) is None


def test_the_pre_roll_restarts_after_each_utterance():
    """Utterance 1's lead-in (tag 9) never appears in utterance 2.

    Between the two, the only quiet frames are two tagged 3; those are the
    second utterance's pre-roll and must sit right before its onset (600).
    """
    cfg = _cfg(300)
    # ONE state across both utterances -- that is the point of this case.
    st = VadState_()
    outs = []
    # Utterance 1: lead-in tagged 9, speech at 500, closing tail.
    # Utterance 2: lead-in tagged 3 (two frames), speech at 600, closing tail.
    seq = ([_quiet(9)] * 20 + [_voiced()] * 5 + _close_tail()
           + [_quiet(3)] * 2 + [_voiced(600)] * 5 + _close_tail())
    for f in seq:
        r = feed_frame(st, f, cfg)
        if r is not None:
            outs.append(r)
    # Both utterances closed.
    assert len(outs) == 2
    second = outs[1]
    # Recover the per-frame values of the second utterance.
    lead = [second[i * FRAME] for i in range(len(second) // FRAME)]
    # Nothing of the first utterance's lead-in survived the close.
    assert 9 not in lead, lead
    # Its own two quiet frames sit right before its onset.
    assert lead[lead.index(600) - 2:lead.index(600)] == [3, 3]


def test_zero_pre_roll_keeps_the_old_onset_behaviour():
    """pre_roll_ms = 0: the utterance starts at the onset frame, as before."""
    # Pre-roll off.
    cfg = _cfg(0)
    utter = _run(cfg, [_quiet(t) for t in range(1, 21)] + [_voiced()] * 5
                 + _close_tail())
    # The first sample is the onset frame's: no quiet frame was kept.
    assert utter[0] == 500


def test_a_negative_pre_roll_is_refused():
    """A negative pre-roll is a typo, not a value: refused, not read as 0."""
    # The error names the field so the operator knows which flag is wrong.
    with pytest.raises(VadConfigError, match="pre_roll_ms"):
        _cfg(-20)
