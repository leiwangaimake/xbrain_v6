"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: measure_mic_level.py
Brief: Measure the local microphone's noise floor and speech level to set vad_threshold.

Description:
  A calibration tool, not a test. It measures the microphone's idle level, then WAITS for
  a person to speak, then measures the speech level -- giving the pair of numbers the
  energy VAD's threshold has to sit between.

  Why it is needed: vad_threshold's default of 40 was calibrated against the PAYLOAD
  microphone, whose idle floor measured about 2 counts. The USB microphone now standing in
  for it has an idle floor around 65, so 40 sits BELOW its noise and the energy backend
  would report speech continuously. The floor is easy to measure unattended; the speech
  level is not, because it needs a person talking.

  Why it waits for speech instead of recording a fixed window: the first version used a
  15 s window and produced a flat, speechless recording, because the operator was still
  reading the instructions when the window closed. A tool that has to be raced against a
  stopwatch reports a confident wrong answer when the race is lost -- and a flat recording
  yields a plausible-looking threshold that is really just noise measured twice. Waiting
  removes the coordination problem entirely.

  Why the floor and the speech level are separated by TIME rather than by percentile:
  taking p10 and p90 of one mixed recording assumes a known speech-to-silence ratio. Here
  the quiet part is measured before the trigger and the loud part after it, so the two
  populations are separated by when they happened, which needs no such assumption.

  Why the recommendation is the geometric mean of the two levels: the threshold should sit
  the same number of dB above the noise as below speech, because the two errors it trades
  off -- triggering on noise, missing a quiet talker -- are symmetric in loudness, not in
  linear amplitude.

  It touches no service, no mode and no device: capture only, nothing is played. Run it
  from the repository root, on the Orin, and speak when it says to:

      cd /opt/xbrain_v6 && python3 -m scripts.ai_runtime.measure_mic_level

  The reported separation matters more than the recommended number. A ratio below about 4x
  means speech and noise overlap and no threshold separates them -- a signal to move the
  microphone or stay on the silero backend, not to pick a cleverer value.
"""
from __future__ import annotations

import argparse
import asyncio
import statistics
import sys
from typing import List, Optional

import numpy as np

from tests.ai_runtime.config import AiRuntimeConfig
from tests.ai_runtime.local_mic import LocalMicError, LocalMicStream

# Frames are 20 ms, so this converts between durations and frame counts.
_FRAME_MS = 20
# Speech is declared when frames exceed this multiple of the measured floor. 2x is low
# enough to trigger on a quiet talker and high enough that fan noise drift does not.
_TRIGGER_RATIO = 2.0
# Consecutive frames required, so a chair creak or a keystroke cannot start the capture.
_TRIGGER_FRAMES = 5
# Percentile taken as the speech level. Not the max: one syllable peak does not describe
# the sustained level a threshold has to sit below.
_SPEECH_PCT = 90
# Below this speech-to-noise ratio the two populations overlap and no threshold works.
_MIN_USABLE_RATIO = 4.0


def _frame_level(frame: bytes) -> float:
    """Return one frame's mean absolute amplitude in int16 counts.

    The same measure the energy backend uses, so the numbers printed here can be compared
    against vad_threshold directly rather than converted.
    """
    # Widened to int32 before abs() because -32768 has no positive int16 counterpart and
    # would wrap to itself, quietly biasing every measurement that clips.
    return float(np.abs(np.frombuffer(frame, dtype="<i2").astype("<i4")).mean())


def _bars(levels: List[float], peak: float, label: str) -> List[str]:
    """Render a per-second bar chart so the operator can SEE what was recorded.

    Without it, a run where nobody spoke looks identical in the summary statistics to one
    where somebody did -- which is exactly the failure this tool was rewritten to avoid.
    """
    per_second = 1000 // _FRAME_MS
    rows = []
    for index in range(0, len(levels), per_second):
        window = levels[index:index + per_second]
        mean = statistics.mean(window)
        rows.append(
            f"  {label} {index // per_second:2d}s  mean={mean:7.1f}  "
            + "#" * int(40 * mean / peak)
        )
    return rows


async def _measure(config: AiRuntimeConfig, args) -> Optional[tuple]:
    """Measure the floor, wait for speech, then measure speech.

    Returns:
        (floor_levels, speech_levels), or None if nobody spoke before the timeout.
    """
    floor_frames = args.floor_seconds * 1000 // _FRAME_MS
    speech_frames = args.speech_seconds * 1000 // _FRAME_MS
    wait_frames = args.wait * 1000 // _FRAME_MS
    floor_levels: List[float] = []
    speech_levels: List[float] = []
    async with LocalMicStream(config) as mic:
        print(f"measuring noise floor for {args.floor_seconds}s -- stay quiet", flush=True)
        async for frame in mic:
            floor_levels.append(_frame_level(frame))
            if len(floor_levels) >= floor_frames:
                break
        floor = statistics.median(floor_levels)
        trigger = floor * _TRIGGER_RATIO
        print(f"floor = {floor:.1f}, will trigger above {trigger:.1f}", flush=True)
        print(f"SPEAK NOW -- say a few sentences (waiting up to {args.wait}s)", flush=True)
        # Count consecutive loud frames rather than a single one, so a transient does not
        # start the capture and leave the real speech outside the window.
        run, waited = 0, 0
        async for frame in mic:
            waited += 1
            if _frame_level(frame) > trigger:
                run += 1
                if run >= _TRIGGER_FRAMES:
                    break
            else:
                run = 0
            if waited >= wait_frames:
                return None
        print(f"speech detected, recording {args.speech_seconds}s -- keep talking", flush=True)
        async for frame in mic:
            speech_levels.append(_frame_level(frame))
            if len(speech_levels) >= speech_frames:
                break
    return floor_levels, speech_levels


def main() -> int:
    """Measure both levels and recommend a threshold.

    Returns:
        0 on a usable measurement, 1 if capture failed, nobody spoke, or the levels
        overlap so far that no threshold separates them.
    """
    parser = argparse.ArgumentParser(description="calibrate vad_threshold for the local mic")
    parser.add_argument("--floor-seconds", type=int, default=3, help="quiet measurement")
    parser.add_argument("--speech-seconds", type=int, default=8, help="speech measurement")
    parser.add_argument("--wait", type=int, default=180, help="how long to wait for speech")
    parser.add_argument("--device", default=None, help="ALSA device, default from config")
    args = parser.parse_args()

    overrides = {"mic_alsa_device": args.device} if args.device else {}
    config = AiRuntimeConfig(**overrides)
    try:
        result = asyncio.run(_measure(config, args))
    except LocalMicError as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        return 1
    if result is None:
        print(f"no speech within {args.wait}s -- nothing measured", file=sys.stderr)
        return 1

    floor_levels, speech_levels = result
    floor = statistics.median(floor_levels)
    speech = float(np.percentile(speech_levels, _SPEECH_PCT))
    peak = max(max(floor_levels), max(speech_levels)) or 1.0
    print("\n".join(_bars(floor_levels, peak, "quiet")))
    print("\n".join(_bars(speech_levels, peak, "speech")))
    print(f"noise floor : median = {floor:.1f}  (max {max(floor_levels):.1f})")
    print(f"speech level: p{_SPEECH_PCT} = {speech:.1f}  (max {max(speech_levels):.1f})")
    if floor <= 0.0:
        # A digital-silence floor makes the ratio meaningless and the geometric mean zero.
        print("noise floor is zero -- microphone is muted or disconnected", file=sys.stderr)
        return 1
    ratio = speech / floor
    print(f"separation  : {ratio:.1f}x")
    if ratio < _MIN_USABLE_RATIO:
        print(
            f"speech is only {ratio:.1f}x the noise -- too close to separate by level. "
            "Move the microphone closer or keep vad_backend=silero.",
            file=sys.stderr,
        )
        return 1
    # Geometric mean: equidistant from both levels in dB (module docstring).
    print(f"\nrecommended AI_VAD_THRESHOLD = {(floor * speech) ** 0.5:.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
