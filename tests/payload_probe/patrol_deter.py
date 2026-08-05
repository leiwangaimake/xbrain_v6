#!/usr/bin/env python3
"""GZH-2 "驱离模式" — intruder deterrent for the robot-dog payload.

Lights (TCP 8529), running continuously:
  * 补光灯/探照灯 : steady ON at max brightness (nighttime illumination).
  * 红蓝警示灯     : police DOUBLE-FLASH (two quick pulses + pause), software-timed.

Audio (TCP 8519), an infinite loop of two phases:
  * Phase A siren : one Wail+Yelp 快慢 cycle streamed to the loudspeaker via [42] Opus 8k.
  * Phase B voice : male TTS [31] "你已进入管制区域，请立即离开" x3, 1s between, then back to siren.

Light and audio run in separate threads; everything stops together (Ctrl-C or --seconds).

  python3 patrol_deter.py --host 192.168.144.38            # auto-makes siren.wav beside this script, runs until Ctrl-C
  python3 patrol_deter.py --host 192.168.144.38 --siren-level 0.3 --regen-siren   # quieter siren
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, "/opt/speaker/tools")   # dev box
sys.path.insert(0, "/home/jack")           # Orin (probes deployed here)
import probe_8519 as A   # noqa: E402  audio: framing + Opus + [42]/[31] + [14]
import probe_8529 as L   # noqa: E402  lights: CRC + searchlight + red/blue frames

DEV = "192.168.144.38"
CMD_VOLUME = 14
TTS_TEXT = "你已进入管制区域，请立即离开"


def lights_loop(host, port, timeout, mode, bright, pulse_on, gap, pulses, pause, stop):
    """Searchlight steady-max + red/blue police double-flash until `stop` is set."""
    s = L.connect(host, port, timeout)
    rb_on = L.build_frame(L.MSG_REDBLUE, [mode])
    rb_off = L.build_frame(L.MSG_REDBLUE, [0])
    try:
        # searchlight: strobe off, steady on, max brightness
        s.sendall(L.build_frame(L.MSG_STROBE, [0]))
        s.sendall(L.build_frame(L.MSG_BRIGHT, [bright]))
        s.sendall(L.build_frame(L.MSG_LIGHT, [1]))
        while not stop.is_set():
            for _ in range(pulses):
                if stop.is_set():
                    break
                s.sendall(rb_on)
                stop.wait(pulse_on)
                s.sendall(rb_off)
                stop.wait(gap)
            stop.wait(pause)
    finally:
        try:
            s.sendall(rb_off)                        # red/blue off
            s.sendall(L.build_frame(L.MSG_LIGHT, [0]))  # searchlight off
            # 8529 streams 0x25 status we never read; close() with unread rx
            # makes Linux send RST, which can drop the just-sent off frames.
            # Brief pause lets them flush + ACK on the LAN before teardown.
            time.sleep(0.2)
        except OSError:
            pass
        s.close()


def audio_loop(host, port, timeout, wav, enc, volume,
               tts_voice, tts_reps, tts_dur, tts_gap, stop):
    """Infinite loop: one siren cycle, then male TTS x`reps`, then repeat."""
    A.need_opus("siren stream")
    pcm = A.read_wav_mono(wav, A.HAIL_FS)
    pkts = A.encode_opus_stream(pcm, A.HAIL_FS, A.HAIL_FRAME)
    if not pkts:
        print("  siren: WAV produced no audio frames — check --wav")
        return
    frame_dt = A.HAIL_FRAME / A.HAIL_FS
    tts_payload = bytes([tts_voice & 0xFF]) + TTS_TEXT.encode("utf-8")
    s = A.connect(host, port, timeout)
    try:
        if volume is not None:
            s.sendall(A.frame(CMD_VOLUME, bytes([max(0, min(100, volume))]), enc=enc))
        while not stop.is_set():
            # Phase A: one Wail+Yelp cycle (the "爆闪快慢周期")
            start = time.monotonic()
            for i, pk in enumerate(pkts):
                if stop.is_set():
                    break
                s.sendall(A.frame(A.CMD_HAIL_PC, pk, enc=enc))
                delay = start + (i + 1) * frame_dt - time.monotonic()
                if delay > 0:
                    stop.wait(delay)
            s.sendall(A.frame(A.CMD_HAIL_STOP_MOBILE, enc=enc))
            if stop.is_set():
                break
            stop.wait(0.3)                             # let the siren tail settle
            # Phase B: male-voice announcement x reps, 1s between
            for k in range(tts_reps):
                if stop.is_set():
                    break
                s.sendall(A.frame(A.CMD_TTS_V2, tts_payload, enc=enc))
                stop.wait(tts_dur)
                if k < tts_reps - 1:
                    stop.wait(tts_gap)
    finally:
        try:
            s.sendall(A.frame(A.CMD_HAIL_STOP_MOBILE, enc=enc))
            s.sendall(A.frame(A.CMD_TTS_STOP, enc=enc))
        except OSError:
            pass
        s.close()


def ensure_siren(path, level, accent_hz, force):
    """Auto-generate the deterrent siren WAV if missing (or --regen-siren).

    Keeps it to one Wail+Yelp quick-slow cycle and locks the amplitude accent to
    the flash group rate (声光同拍). numpy-only; touches no device.
    """
    if os.path.exists(path) and not force:
        return
    import siren_gen as SG   # noqa: E402
    pcm = SG.build_siren(SG.FS_DEFAULT, 6.0, "combo", 600.0, 1500.0,
                         4.0, 0.30, 3.5, 2.5, accent_hz, 0.22, level=level)
    SG.write_wav(path, pcm, SG.FS_DEFAULT)
    print("   siren: auto-generated %s (6.0s Wail+Yelp, level=%.2f, accent=%.2fHz)"
          % (path, level, accent_hz))


def main():
    ap = argparse.ArgumentParser(description="GZH-2 intruder deterrent: lights + siren + TTS loop.")
    ap.add_argument("--host", default=DEV)
    ap.add_argument("--light-port", type=int, default=8529)
    ap.add_argument("--audio-port", type=int, default=8519)
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--seconds", type=float, default=0.0, help="run time (0 = until Ctrl-C)")
    ap.add_argument("--wav", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "siren.wav"))
    ap.add_argument("--enc", default="bracket")
    # lights
    ap.add_argument("--mode", type=int, default=1, help="red/blue on-state mode 1-16")
    ap.add_argument("--bright", type=int, default=L.BRIGHT_MAX, help="searchlight brightness 1-30")
    ap.add_argument("--pulse-on", type=float, default=0.09, help="each flash on-time (s)")
    ap.add_argument("--gap", type=float, default=0.07, help="off-time between the two flashes (s)")
    ap.add_argument("--pulses", type=int, default=2, help="flashes per burst")
    ap.add_argument("--pause", type=float, default=0.40, help="dark pause after the burst (s)")
    # audio
    ap.add_argument("--volume", type=int, default=None, help="set device volume 0-100 at start (default: leave as-is)")
    ap.add_argument("--tts-voice", type=int, default=0, help="0=male, 1=female")
    ap.add_argument("--tts-reps", type=int, default=3)
    ap.add_argument("--tts-dur", type=float, default=4.0, help="estimated seconds per TTS utterance (no done-event)")
    ap.add_argument("--tts-gap", type=float, default=1.0, help="seconds between TTS repeats")
    ap.add_argument("--siren-level", type=float, default=0.45,
                    help="siren amplitude 0-1 when auto-generating the WAV (office-quiet default)")
    ap.add_argument("--regen-siren", action="store_true",
                    help="force-regenerate the siren WAV even if --wav already exists")
    ap.add_argument("--no-audio", action="store_true", help="lights only")
    ap.add_argument("--no-light", action="store_true", help="audio only")
    ap.add_argument("--dry-run", action="store_true", help="build siren + print schedule, send nothing")
    args = ap.parse_args()

    mode = max(1, min(16, args.mode))
    bright = max(1, min(L.BRIGHT_MAX, args.bright))
    group = args.pulses * (args.pulse_on + args.gap) + args.pause
    print("== 驱离模式 ==  补光灯常亮max(%d) + 红蓝双闪(mode %d, group=%.2fs≈%.2f/s)"
          % (bright, mode, group, 1.0 / group))
    print("   audio loop: siren 1 cycle -> 男声TTS x%d(间隔%.0fs) -> 循环 | match siren --accent-hz %.2f"
          % (args.tts_reps, args.tts_gap, 1.0 / group))

    if not args.no_audio:
        ensure_siren(args.wav, args.siren_level, 1.0 / group, args.regen_siren)

    if args.dry_run:
        A.need_opus("dry-run (encode siren)")
        pcm = A.read_wav_mono(args.wav, A.HAIL_FS)
        pkts = A.encode_opus_stream(pcm, A.HAIL_FS, A.HAIL_FRAME)
        cyc = len(pkts) * A.HAIL_FRAME / A.HAIL_FS
        loop = cyc + 0.3 + args.tts_reps * args.tts_dur + (args.tts_reps - 1) * args.tts_gap
        print("   dry-run: siren %s -> %d frames (%.2fs); one full loop ≈ %.1fs; sends nothing"
              % (args.wav, len(pkts), cyc, loop))
        return

    print("   host=%s  run=%s  TTS='%s' (%s)"
          % (args.host, ("%.1fs" % args.seconds) if args.seconds > 0 else "until Ctrl-C",
             TTS_TEXT, "male" if args.tts_voice == 0 else "female"))

    stop = threading.Event()
    threads = []
    if not args.no_light:
        threads.append(threading.Thread(target=lights_loop, args=(
            args.host, args.light_port, args.timeout, mode, bright,
            args.pulse_on, args.gap, args.pulses, args.pause, stop), daemon=True))
    if not args.no_audio:
        threads.append(threading.Thread(target=audio_loop, args=(
            args.host, args.audio_port, args.timeout, args.wav, args.enc, args.volume,
            args.tts_voice, args.tts_reps, args.tts_dur, args.tts_gap, stop), daemon=True))

    for t in threads:
        t.start()
    try:
        if args.seconds > 0:
            stop.wait(args.seconds)
        else:
            while any(t.is_alive() for t in threads):
                time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n  stopping...")
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2.0)
        print("  驱离模式 OFF (灯全灭, 警笛/TTS 停止).")


if __name__ == "__main__":
    main()
