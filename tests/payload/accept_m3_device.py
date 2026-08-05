"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: accept_m3_device.py
Brief: Manual on-device (hardware-in-the-loop) acceptance for the M3 audio paths.

Description:
  Drives the REAL GZH-2 device through payload-service's own from-scratch core layer
  (DeviceLink + protocol/audio_8519 + codec), exercising the exact code paths that the
  HTTP POST /tts and WS /mic / /play handlers call, but WITHOUT the fastapi/uvicorn/
  websockets web stack -- so it runs on the Orin system python3.10 that already has
  opuslib+numpy. This is the substance of development-plan section 13 acceptance:
    - tts  : send one short [31] utterance    -> device should SPEAK (verify by ear).
    - mic  : arm [40], capture ~3s uplink, decode Opus -> save a clean 16k PCM WAV.
    - play : synth a short soft tone, resample->8k, Opus-encode, stream as [42], stop [11].

  This is NOT a pytest (it needs real hardware and makes sound), so it is named without a
  test_ prefix and pytest will not collect it. Run it explicitly on the Orin:
      cd /opt/xbrain_v6 && python3 tests/payload/accept_m3_device.py [tts|mic|play|all]
"""
from __future__ import annotations

import os
import sys
import time
import wave

import numpy as np

# Make "from services.payload..." resolve whether this is launched as a plain script or
# via -m: ensure the repo root (three levels up from tests/payload/) is on sys.path.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from services.payload.codec.opus_stream import OpusDecoderStream, OpusEncoderStream
from services.payload.codec.resample import resample_linear
from services.payload.config import PayloadConfig
from services.payload.core.device_link import DeviceLink, DeviceLinkError
from services.payload.protocol.audio_8519 import (
    VOICE_MALE,
    build_hail,
    build_hail_stop,
    build_tts,
)

# Where the decoded mic capture is written for a human to listen to (existing dir).
_MIC_WAV = "/opt/speaker/samples/m3_mic_accept.wav"
# Short, punctuation-free Chinese utterance (spoken dialogue is allowed to be Chinese).
_TTS_TEXT = "你好音频测试"
# A brief, soft test tone for /play; amplitude kept low because office testing is loud.
_TONE_SECONDS = 1.0
_TONE_HZ = 440.0
_TONE_AMPL = 0.12


def _wait_audio_up(link: DeviceLink, timeout_s: float = 8.0) -> bool:
    # start() only spawns the supervisor threads; the socket connects in the background,
    # so poll the published flag until it is up (or the deadline passes).
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if link.audio_connected:
            return True
        time.sleep(0.1)
    return link.audio_connected


def _accept_tts(link: DeviceLink, config: PayloadConfig) -> None:
    # Mirrors POST /tts: build one [31] frame and hand it to the socket owner. The device
    # gives no "TTS finished" event, so wait the estimate (plus margin) before moving on.
    est = config.estimate_tts_ms(_TTS_TEXT)
    print(f"[tts] sending [31] voice=male text={_TTS_TEXT!r} est_ms={est}")
    link.send_audio([build_tts(VOICE_MALE, _TTS_TEXT)])
    print(f"[tts] sent; device should SPEAK now (waiting est+margin={est + 500} ms)")
    time.sleep(est / 1000.0 + 0.5)
    print("[tts] done -- verify you HEARD the utterance from the device speaker")


def _accept_mic(link: DeviceLink, seconds: float = 3.0) -> None:
    # Mirrors WS /mic: arm [40], let the sink collect raw Opus, then decode to 16k PCM.
    packets: list[bytes] = []
    print(f"[mic] arming [40], capturing ~{seconds:.0f}s of uplink (make some noise) ...")
    link.start_recording(packets.append)
    time.sleep(seconds)
    link.stop_recording()
    print(f"[mic] captured {len(packets)} Opus packets")
    dec = OpusDecoderStream(16000)
    pcm = bytearray()
    for packet in packets:
        pcm.extend(dec.decode(packet))
    dur = len(pcm) / 2.0 / 16000.0
    with wave.open(_MIC_WAV, "wb") as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(16000)
        out.writeframes(bytes(pcm))
    print(f"[mic] decoded {len(pcm)} bytes = {dur:.2f}s of 16k mono PCM")
    print(f"[mic] saved WAV -> {_MIC_WAV} (listen to verify it is CLEAN 16k)")


def _accept_play(link: DeviceLink) -> None:
    # Mirrors WS /play: synth 16k PCM, resample->8k, Opus-encode to 480-sample frames,
    # stream them as [42] hails, wait for playback, then [11] so the hail ends cleanly.
    n = int(16000 * _TONE_SECONDS)
    samples = np.arange(n) / 16000.0
    tone = _TONE_AMPL * 32767.0 * np.sin(2 * np.pi * _TONE_HZ * samples)
    pcm16k = np.round(tone).astype("<i2").tobytes()
    pcm8k = resample_linear(pcm16k, 16000, 8000)
    enc = OpusEncoderStream(8000, 480)
    frames = [build_hail(p) for p in enc.encode(pcm8k)]
    frames += [build_hail(p) for p in enc.flush()]
    print(f"[play] streaming {_TONE_SECONDS:.0f}s {int(_TONE_HZ)}Hz tone as {len(frames)} [42] frames")
    link.send_audio(frames)
    # Let the device play the buffered stream before stopping, else [11] would truncate it.
    time.sleep(_TONE_SECONDS + 0.4)
    link.send_audio([build_hail_stop()])
    print("[play] sent [11] stop; verify you HEARD the tone from the device speaker")


def main() -> int:
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    config = PayloadConfig.from_env()
    link = DeviceLink(config)
    link.start()
    try:
        if not _wait_audio_up(link):
            print("[ERROR] audio link (8519) did not connect -- is the device powered/cabled?",
                  file=sys.stderr)
            return 1
        print(f"[ok] audio link up to {config.device_host}:{config.port_audio}")
        if which in ("tts", "all"):
            _accept_tts(link, config)
            time.sleep(0.5)
        if which in ("mic", "all"):
            _accept_mic(link)
            time.sleep(0.5)
        if which in ("play", "all"):
            _accept_play(link)
        print("[done] M3 device acceptance finished")
        return 0
    except DeviceLinkError as exc:
        print(f"[ERROR] device link failure: {exc}", file=sys.stderr)
        return 1
    finally:
        # Always tear the link down cleanly (flush-close) so the device is left idle.
        link.stop()


if __name__ == "__main__":
    raise SystemExit(main())
