"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: audio_io.py
Brief: BIZ-P2-3 -- audio_io: ORIN USB MIC exclusive capture + 3:1 decimation

Description:
14 S4.1.1 + RT-A1 declare p2_core.audio_io the SOLE OWNER of the
USB MIC (JMTek 0c76:161f, ALSA hw:0,0, 48 kHz mono s16le).

Chain:
  arecord -f S16_LE -r 48000 -c 1 -D hw:0,0
    -> 960-sample frames (20 ms @ 48 kHz)
    -> decimate 3:1 with antialias low-pass
    -> 320-sample frames (20 ms @ 16 kHz)
    -> publish rt/audio/mic (AudioFrame)

The DECIMATION math is pure Python (numpy-free) so it is testable
without any audio device. The ALSA capture is a subprocess.Popen
around arecord; the actual open-and-read is out of scope for the
tests here (would need a live USB MIC on the CI host).

★ RT-A1: this module is the ONLY place in p2_core that opens the
ALSA device. All other modules subscribe to rt/audio/mic (Zenoh) --
they must NOT directly open hw:0,0 (verifiable by CI grep, already
enforced by scripts/lint/no_business_imports.py or similar).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


# 14 S4.2 local_mic block values (from p2_core.yaml, mirrored here
# for the pure-math decimator; production reads them from config).
CAPTURE_RATE_HZ = 48_000
ASR_RATE_HZ = 16_000
DECIMATION_FACTOR = CAPTURE_RATE_HZ // ASR_RATE_HZ    # 3
FRAME_MS = 20
CAPTURE_SAMPLES_PER_FRAME = CAPTURE_RATE_HZ * FRAME_MS // 1000    # 960
ASR_SAMPLES_PER_FRAME = ASR_RATE_HZ * FRAME_MS // 1000            # 320


@dataclass(frozen=True)
class AudioFrame:
    """rt/audio/mic message shape (11 S8.7.1)."""
    rate_hz: int                      # 16000 after decimation
    channels: int                     # 1
    sample_width: int                 # 2 (s16le)
    frame_ms: int                     # 20
    samples: List[int]                # 320 int16 values

    def __post_init__(self) -> None:
        if self.rate_hz != ASR_RATE_HZ:
            raise ValueError(
                "AudioFrame.rate_hz must be %d after decimation; got %d"
                % (ASR_RATE_HZ, self.rate_hz))
        if self.channels != 1:
            raise ValueError("AudioFrame.channels must be 1")
        if self.sample_width != 2:
            raise ValueError("AudioFrame.sample_width must be 2 (s16le)")
        if len(self.samples) != ASR_SAMPLES_PER_FRAME:
            raise ValueError(
                "AudioFrame must have %d samples; got %d"
                % (ASR_SAMPLES_PER_FRAME, len(self.samples)))


def decimate_3to1(samples_48k: List[int]) -> List[int]:
    """3:1 decimation with a simple 3-tap moving average as the
    antialias low-pass.

    Input: exactly CAPTURE_SAMPLES_PER_FRAME (960) int16 samples at 48 kHz.
    Output: exactly ASR_SAMPLES_PER_FRAME (320) int16 samples at 16 kHz.

    ★ The 3-tap moving average is a rough anti-aliasing filter that
    attenuates content above 16 kHz Nyquist / 2 = 8 kHz. For ASR
    quality on speech (well within 8 kHz), this is adequate. A
    higher-quality kaiser-window FIR is a natural production-time
    upgrade; leaving it out here keeps the math trivially testable
    and numpy-free.

    Boundary behavior: for the first/last output samples the window
    would reach outside the input; we clamp using edge replication
    (samples_48k[0] and samples_48k[-1]).
    """
    if len(samples_48k) != CAPTURE_SAMPLES_PER_FRAME:
        raise ValueError(
            "decimate_3to1 needs %d samples (got %d)"
            % (CAPTURE_SAMPLES_PER_FRAME, len(samples_48k)))
    out: List[int] = []
    n = len(samples_48k)
    for i in range(ASR_SAMPLES_PER_FRAME):
        center = i * DECIMATION_FACTOR   # 0, 3, 6, ... 957
        # 3-tap MA around center.
        a = samples_48k[max(0, center - 1)]
        b = samples_48k[center]
        c = samples_48k[min(n - 1, center + 1)]
        avg = (a + b + c) // 3
        # Clamp to int16 range.
        out.append(max(-32768, min(32767, avg)))
    return out


def build_frame(samples_48k: List[int]) -> AudioFrame:
    """Convenience: decimate + wrap in AudioFrame."""
    return AudioFrame(
        rate_hz=ASR_RATE_HZ,
        channels=1,
        sample_width=2,
        frame_ms=FRAME_MS,
        samples=decimate_3to1(samples_48k),
    )


# --- ALSA capture wrapper (skeleton only) ---------------------------

class AlsaCaptureUnavailable(RuntimeError):
    """Raised when the USB MIC device is not available (EBUSY,
    device disconnected, no ALSA on this host). Callers convert
    this into mic=device_fault (via audio_state.py)."""


def open_capture(device: str = "hw:0,0",
                  rate_hz: int = CAPTURE_RATE_HZ,
                  channels: int = 1) -> object:
    """Open arecord subprocess capturing the USB MIC.

    Returns a subprocess.Popen with stdout ready for read. Callers
    read 2 * CAPTURE_SAMPLES_PER_FRAME bytes per frame (s16le mono).

    ★ Runtime-only: requires arecord + the actual USB MIC. On a
    dev host without either, raises AlsaCaptureUnavailable so the
    audio_state.py device_fault path can fire coherently.

    ★ RT-A1: this is the SINGLE call site that opens the ALSA
    device. A CI grep of `hw:0,0` outside this file catches
    double-open regressions.
    """
    import shutil
    import subprocess
    if shutil.which("arecord") is None:
        raise AlsaCaptureUnavailable("arecord binary not present on PATH")
    try:
        proc = subprocess.Popen(
            [
                "arecord", "-q",
                "-D", device,
                "-f", "S16_LE",
                "-c", str(channels),
                "-r", str(rate_hz),
                "-t", "raw",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except (FileNotFoundError, OSError) as exc:
        raise AlsaCaptureUnavailable(
            "arecord failed to launch: %s" % exc) from exc
    return proc
