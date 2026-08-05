"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: measure_tts_timing.py
Brief: Measure when the device actually stops speaking, to set tts_gate_margin_ms.

Description:
  A calibration tool, not a test. THE DEVICE WILL HAIL OUT LOUD, several times and for up
  to half a minute at a stretch, so somebody has to be present and willing before this is
  run.

  What it is for: the device emits no "TTS finished" event (plan section 9), so turn_loop
  cannot wait for the end of speech -- it waits payload-service's est_ms plus
  tts_gate_margin_ms and then reopens the microphone. If that total is short, the robot
  hears its own tail, transcribes it, and answers itself. tts_gate_margin_ms defaults to
  700 ms, which is a guess. This tool replaces the guess with a measurement by listening to
  the loudspeaker with the same microphone the gate is protecting.

  The measurement is anchored where the gate is anchored. turn_loop starts its timer at the
  instant POST /tts RETURNS, not when it was sent, so that is the zero this tool measures
  from -- otherwise the recommended margin would be right about a clock nothing uses.

  Timing is counted in FRAMES, not in the wall-clock arrival time of each frame. Capture
  arrives in bursts the size of the ALSA period, so arrival times are jittery by tens of
  milliseconds, whereas frames are exactly 20 ms of audio each and cannot drift. The fixed
  capture latency this ignores (up to one arecord buffer) makes the measured end of speech
  slightly LATE, which biases the recommendation long -- the safe direction, because
  reopening late costs a moment of silence and reopening early costs a robot arguing with
  itself.

  Several lengths are spoken, not one, because a single length cannot tell the two possible
  faults apart. If the device simply starts late, the overrun is a constant and a fixed
  margin is exactly the right fix. If the device speaks SLOWER than the 180 ms/char the
  estimate assumes, the overrun grows with length, and then no fixed margin is correct --
  the fix is payload-service's PER_CHAR_MS, and this tool says so rather than recommending
  a margin that only holds for short replies.

  It refuses to answer when the microphone cannot hear the loudspeaker at all. That is not
  a hypothetical: issue E8 has the USB microphone's input floating, and in that state this
  tool would find no speech, compute an end of speech at zero, and recommend a margin that
  is confidently wrong. Reporting "the microphone did not hear it" is the only honest
  output for a run where nothing was heard.

  It does not change the mode. POST /tts is allowed in idle and func1, so a calibration run
  needs no transition and therefore cannot leave the payload parked in one -- it only
  refuses to start if the mode is one where TTS would be rejected anyway.

  Run it on the Orin with payload-service up and AI_runtime NOT running (the loop would be
  holding the microphone):
      cd /opt/xbrain_v6 && python3 -m scripts.ai_runtime.measure_tts_timing
"""
from __future__ import annotations

import argparse
import asyncio
import statistics
import sys
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from tests.ai_runtime import payload_client
from tests.ai_runtime.config import AiRuntimeConfig
from tests.ai_runtime.local_mic import LocalMicError, LocalMicStream
from tests.ai_runtime.payload_client import PayloadClientError

# Frames are 20 ms of audio each, which is what makes a frame count a clock.
_FRAME_MS = 20

# Lines the device will speak, chosen to span the range of reply lengths turn_loop can
# produce (reply_max_chars bounds it at 120). Real sentences rather than a repeated
# syllable, because a hailing device is the intended listener's experience and an operator
# judging whether the audio sounded right needs something judgeable.
_PHRASES = (
    "你好",
    "前方有障碍请注意",
    "本机器人正在执行巡逻任务请配合现场工作人员的指引",
    "各位注意这里是自动巡逻机器人当前正在执行区域安全检查任务请保持安全距离不要靠近设备"
    "也不要触碰机身如需帮助请联系现场值班人员或者拨打管理处电话我们会尽快安排工作人员"
    "前来处理谢谢配合请大家配合现场管理规定有序通行不要在此处长时间停留",
)

# Speech is declared above this multiple of the measured floor. The loudspeaker is on the
# same chassis as the microphone, so its sound arrives far louder than anything else in the
# room; a low multiple is enough and keeps a quiet trailing syllable inside the utterance.
_TRIGGER_RATIO = 3.0
# Consecutive loud frames required to call it the start, so a click as the amplifier
# switches on is not mistaken for the first syllable.
_ONSET_FRAMES = 3
# Quiet frames required to call it the end. This is the number that decides what "stopped"
# means, and it has to exceed the gap between words -- 400 ms is longer than any pause
# inside a sentence and far shorter than the silence after one.
_SILENCE_FRAMES = 20
# How long to keep listening past the estimate. The whole point is to catch a device that
# runs over, so the window has to extend well beyond est_ms or the overrun would be clipped
# to whatever the window happened to be.
_TAIL_LISTEN_MS = 4000
# Quiet time between phrases, so one utterance's tail cannot be counted as the next one's
# floor.
_SETTLE_MS = 1500
# Added to the worst measured overrun. The measurement is a handful of samples on one
# device; this covers the run-to-run spread that a handful of samples cannot show.
_SAFETY_MS = 300
# Never recommend less than this, however early the device finishes. The margin also
# absorbs start-up jitter, and a margin of zero would leave the gate depending entirely on
# an estimate being exact.
_MIN_MARGIN_MS = 200
# Recommendations are rounded up to this, because a margin is a coarse safety allowance and
# a number like 743 implies a precision this measurement does not have.
_ROUND_MS = 50
# Above this growth in overrun between the shortest and longest phrase, the estimate's
# per-character rate is wrong and a fixed margin cannot fix it.
_SLOPE_WARN_MS = 500

# Modes in which POST /tts is refused, checked up front so the run fails with an
# instruction instead of a 409 halfway through.
_TTS_BLOCKED_MODES = ("func2", "func3")


@dataclass(frozen=True)
class PhraseTiming:
    """What one spoken phrase measured.

    Frozen because these are results: once a phrase has been spoken and heard, nothing
    downstream has any business editing what was observed.
    """

    text: str
    est_ms: int
    lead_ms: int
    end_ms: int

    @property
    def chars(self) -> int:
        """Characters spoken, the input to payload-service's estimate."""
        return len(self.text)

    @property
    def spoken_ms(self) -> int:
        """How long the loudspeaker was actually producing speech."""
        return self.end_ms - self.lead_ms

    @property
    def overrun_ms(self) -> int:
        """How long the device was still speaking after the estimate had expired.

        This IS the quantity tts_gate_margin_ms has to cover: the gate opens at
        est_ms + margin measured from the same zero as end_ms, so a margin below this
        number reopens the microphone into the device's own voice.
        """
        return self.end_ms - self.est_ms


def _frame_level(frame: bytes) -> float:
    """Return one frame's mean absolute amplitude in int16 counts.

    The same measure measure_mic_level prints and the energy VAD applies, so the numbers
    from the two calibration tools can be read side by side without conversion.
    """
    # Widened to int32 before abs() because -32768 has no positive int16 counterpart and
    # would wrap to itself, biasing exactly the loud frames this tool is looking for.
    return float(np.abs(np.frombuffer(frame, dtype="<i2").astype("<i4")).mean())


def _find_speech(levels: List[float], trigger: float) -> Optional[tuple]:
    """Locate the start and end of one utterance in a run of frame levels.

    Args:
        levels: frame levels from the instant POST /tts returned onwards.
        trigger: the level above which a frame counts as speech.

    Returns:
        (onset_index, end_index) as frame indices, where end_index is the first frame of
        the silence that closed the utterance, or None if no utterance was found or it
        never ended inside the window.

    The end is defined as "loud, then quiet for _SILENCE_FRAMES", not "the last loud
    frame", because the last loud frame is only knowable in hindsight and the gate has to
    act on a rule the same recording would produce twice. Requiring the trailing silence
    also means a device that never stops is reported as unmeasured rather than as having
    stopped at the end of the window.
    """
    onset = None
    run = 0
    for index, level in enumerate(levels):
        if level > trigger:
            run += 1
            if run >= _ONSET_FRAMES:
                # The onset is the first frame of the run, not the frame that confirmed
                # it; the earlier frames were speech too, they just were not yet proof.
                onset = index - _ONSET_FRAMES + 1
                break
        else:
            run = 0
    if onset is None:
        return None
    quiet = 0
    for index in range(onset, len(levels)):
        if levels[index] > trigger:
            quiet = 0
            continue
        quiet += 1
        if quiet >= _SILENCE_FRAMES:
            return onset, index - _SILENCE_FRAMES + 1
    return None


async def _drain(mic: LocalMicStream, levels: List[float]) -> None:
    """Append the level of every frame the microphone produces, for as long as it runs.

    A task of its own rather than a loop in the caller, because POST /tts blocks on a
    worker thread and the phrase has to be recorded while it is in flight. Left unread for
    that long, the ALSA buffer would overrun and lose the very frames that hold the start
    of speech.
    """
    async for frame in mic:
        levels.append(_frame_level(frame))


async def _measure_phrase(
    config: AiRuntimeConfig, mic_levels: List[float], text: str
) -> Optional[PhraseTiming]:
    """Speak one phrase and measure when the device really stopped.

    Args:
        config: supplies the payload URL and the TTS voice.
        mic_levels: the growing list of frame levels the drain task appends to.
        text: the line to speak.

    Returns:
        The timing, or None if the microphone never heard the loudspeaker.

    Raises:
        PayloadClientError: if the device refused to speak.
    """
    # The floor is remeasured before every phrase rather than once at the start, because
    # the amplifier's own hiss is not the same before it has been driven as after.
    floor_start = len(mic_levels)
    await asyncio.sleep(_SETTLE_MS / 1000.0)
    floor_window = mic_levels[floor_start:len(mic_levels)]
    floor = statistics.median(floor_window) if floor_window else 0.0
    trigger = max(floor * _TRIGGER_RATIO, 1.0)

    est_ms = await asyncio.to_thread(payload_client.speak, config, text)
    # Taken immediately after the call returns: this index is the zero that turn_loop's
    # gate timer starts from, so every duration below is measured from here.
    ack_index = len(mic_levels)
    await asyncio.sleep((est_ms + _TAIL_LISTEN_MS) / 1000.0)

    found = _find_speech(mic_levels[ack_index:len(mic_levels)], trigger)
    print(
        f"  {len(text):3d} chars  est={est_ms:5d}ms  floor={floor:6.1f}  "
        f"trigger={trigger:6.1f}",
        flush=True,
    )
    if found is None:
        return None
    onset, end = found
    return PhraseTiming(text=text, est_ms=est_ms, lead_ms=onset * _FRAME_MS, end_ms=end * _FRAME_MS)


async def _run(config: AiRuntimeConfig, repeats: int) -> List[Optional[PhraseTiming]]:
    """Speak every phrase the configured number of times, listening throughout.

    The microphone is opened ONCE around the whole run rather than per phrase, because
    opening it costs an arecord start-up whose latency would land inside the very interval
    being measured.
    """
    results: List[Optional[PhraseTiming]] = []
    levels: List[float] = []
    async with LocalMicStream(config) as mic:
        drain = asyncio.create_task(_drain(mic, levels))
        try:
            for _ in range(repeats):
                for text in _PHRASES:
                    if drain.done():
                        # The microphone died mid-run; its exception is the real story and
                        # awaiting the task is what re-raises it.
                        await drain
                    results.append(await _measure_phrase(config, levels, text))
        finally:
            drain.cancel()
            try:
                await drain
            except asyncio.CancelledError:
                pass
    return results


def _round_up(value: float) -> int:
    """Round a millisecond recommendation up to the next _ROUND_MS."""
    return int(_ROUND_MS * -(-value // _ROUND_MS))


def _report(timings: List[PhraseTiming]) -> int:
    """Print the per-phrase table and the recommendation.

    Returns:
        0 if a margin could be recommended, 1 if the measurement says a margin is the
        wrong instrument.
    """
    print("")
    print("  chars    est_ms   lead_ms   spoken_ms   end_ms   overrun_ms")
    for timing in sorted(timings, key=lambda t: t.chars):
        print(
            f"  {timing.chars:5d}  {timing.est_ms:8d}  {timing.lead_ms:8d}  "
            f"{timing.spoken_ms:10d}  {timing.end_ms:7d}  {timing.overrun_ms:10d}"
        )
    worst = max(t.overrun_ms for t in timings)
    print(f"\nworst overrun: {worst} ms")
    # Compared at the extremes rather than fitted, because four points do not support a
    # regression and the question is only whether the error grows with length at all.
    shortest = min(timings, key=lambda t: t.chars)
    longest = max(timings, key=lambda t: t.chars)
    slope = longest.overrun_ms - shortest.overrun_ms
    print(
        f"overrun grows by {slope} ms between {shortest.chars} and {longest.chars} chars"
    )
    if slope > _SLOPE_WARN_MS:
        # A growing overrun means the estimate's slope is wrong, and a constant added to a
        # wrong slope is still wrong at the far end. Recommending a margin here would hide
        # the real fault behind a number that passes the short cases.
        measured_per_char = (longest.spoken_ms - shortest.spoken_ms) / max(
            longest.chars - shortest.chars, 1
        )
        print(
            f"the device speaks about {measured_per_char:.0f} ms/char, so the overrun is "
            f"proportional, not constant.\n"
            f"Fix payload-service's PER_CHAR_MS (currently 180) to about "
            f"{measured_per_char:.0f} and measure again; a fixed "
            f"tts_gate_margin_ms cannot cover every reply length.",
            file=sys.stderr,
        )
        return 1
    recommended = max(_round_up(worst + _SAFETY_MS), _MIN_MARGIN_MS)
    print(f"\nrecommended AI_TTS_GATE_MARGIN_MS = {recommended}")
    return 0


def main() -> int:
    """Speak the phrases, measure the overruns, and recommend a margin.

    Returns:
        0 on a usable measurement, 1 if the device refused, the microphone failed, nothing
        was heard, or the overrun is proportional rather than constant.
    """
    parser = argparse.ArgumentParser(
        description="calibrate tts_gate_margin_ms -- THE DEVICE WILL SPEAK OUT LOUD",
    )
    parser.add_argument("--repeats", type=int, default=1, help="times to speak each phrase")
    parser.add_argument("--device", default=None, help="ALSA device, default from config")
    args = parser.parse_args()

    overrides = {"mic_alsa_device": args.device} if args.device else {}
    config = AiRuntimeConfig(**overrides)
    try:
        mode = payload_client.get_status(config).get("mode")
    except PayloadClientError as exc:
        print(f"cannot reach payload-service: {exc}", file=sys.stderr)
        return 1
    if mode in _TTS_BLOCKED_MODES:
        print(
            f"payload is in mode {mode}, which refuses POST /tts; stop AI_runtime first",
            file=sys.stderr,
        )
        return 1

    print("the device is about to speak out loud, several times", flush=True)
    try:
        results = asyncio.run(_run(config, args.repeats))
    except LocalMicError as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        return 1
    except PayloadClientError as exc:
        print(f"the device refused to speak: {exc}", file=sys.stderr)
        return 1

    timings = [t for t in results if t is not None]
    if not timings:
        # Every phrase was inaudible. Saying so is the whole guard: the alternative is a
        # recommendation derived from a recording of a room the loudspeaker is not in.
        print(
            "the microphone did not hear the loudspeaker on any phrase -- nothing measured.\n"
            "Either the device did not actually speak, or the microphone input is dead "
            "(see issue E8). Check by ear before trusting any margin.",
            file=sys.stderr,
        )
        return 1
    if len(timings) < len(results):
        # A partial result is still usable, but which phrases went missing changes what the
        # remaining numbers mean, so it is stated rather than silently dropped.
        print(
            f"heard {len(timings)} of {len(results)} phrases; the rest were inaudible or "
            f"never stopped",
            file=sys.stderr,
        )
    return _report(timings)


if __name__ == "__main__":
    sys.exit(main())
