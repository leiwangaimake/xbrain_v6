#!/usr/bin/env python3
"""Generate a police-style Wail+Yelp siren WAV for the GZH-2 deterrent mode.

Output is 8000 Hz mono 16-bit PCM -- the same format the 8519 [42] hail path wants,
so the file streams straight to the robot loudspeaker (it also plays on any speaker).

  python3 siren_gen.py --out /tmp/siren.wav --seconds 8 --pattern combo --accent-hz 1.4
"""
import argparse
import wave

import numpy as np

FS_DEFAULT = 8000
HARMONICS = (1.0, 0.35, 0.12)   # fundamental + brightness -> electronic-siren timbre


def _lfo_raised_cos(n, fs, period):
    """Smooth 0..1..0 sweep control at the given period (seconds)."""
    t = np.arange(n) / fs
    return 0.5 - 0.5 * np.cos(2.0 * np.pi * t / period)


def _sweep_freq(fs, dur, f_lo, f_hi, sweep_period):
    n = int(round(dur * fs))
    return f_lo + (f_hi - f_lo) * _lfo_raised_cos(n, fs, sweep_period)


def build_siren(fs, seconds, pattern, f_lo, f_hi, wail_period, yelp_period,
                wail_len, yelp_len, accent_hz, accent_depth, level=0.9,
                harmonics=HARMONICS):
    # 1) tile wail/yelp segments into one instantaneous-frequency track
    if pattern == "wail":
        order = [("wail", wail_len)]
    elif pattern == "yelp":
        order = [("yelp", yelp_len)]
    else:
        order = [("wail", wail_len), ("yelp", yelp_len)]
    segs, total, i = [], 0.0, 0
    while total < seconds:
        kind, ln = order[i % len(order)]
        ln = min(ln, seconds - total + 1.0 / fs)
        period = wail_period if kind == "wail" else yelp_period
        segs.append(_sweep_freq(fs, ln, f_lo, f_hi, period))
        total += ln
        i += 1
    f_inst = np.concatenate(segs)[: int(round(seconds * fs))]
    n = len(f_inst)

    # 2) integrate phase once (click-free across segment seams), add harmonics
    phase = 2.0 * np.pi * np.cumsum(f_inst) / fs
    sig = np.zeros(n)
    for k, a in enumerate(harmonics, start=1):
        sig += a * np.sin(k * phase)
    sig /= sum(abs(a) for a in harmonics)

    # 3) optional amplitude accent locked to the flash group rate (声光同拍)
    if accent_hz > 0 and accent_depth > 0:
        t = np.arange(n) / fs
        bump = 0.5 - 0.5 * np.cos(2.0 * np.pi * ((t * accent_hz) % 1.0))
        sig *= (1.0 - accent_depth) + accent_depth * bump

    # 4) master level + short fades so the file (and its loop seam) never clicks
    sig *= level
    fade = int(0.008 * fs)
    if n > 2 * fade > 0:
        w = np.linspace(0.0, 1.0, fade)
        sig[:fade] *= w
        sig[-fade:] *= w[::-1]

    return (np.clip(sig, -1.0, 1.0) * 32767.0).astype("<i2")


def write_wav(path, pcm_i16, fs):
    w = wave.open(path, "wb")
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(fs)
    w.writeframes(pcm_i16.tobytes())
    w.close()


def main():
    ap = argparse.ArgumentParser(description="Generate a Wail+Yelp police siren WAV (8k mono).")
    ap.add_argument("--out", default="/tmp/siren.wav")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--pattern", choices=["wail", "yelp", "combo"], default="combo")
    ap.add_argument("--fs", type=int, default=FS_DEFAULT)
    ap.add_argument("--f-lo", type=float, default=600.0)
    ap.add_argument("--f-hi", type=float, default=1500.0)
    ap.add_argument("--wail-period", type=float, default=4.0, help="seconds per wail up-down sweep")
    ap.add_argument("--yelp-period", type=float, default=0.30, help="seconds per yelp up-down sweep")
    ap.add_argument("--wail-len", type=float, default=3.5, help="wail seconds per combo cycle")
    ap.add_argument("--yelp-len", type=float, default=2.5, help="yelp seconds per combo cycle")
    ap.add_argument("--accent-hz", type=float, default=0.0,
                    help="amplitude accent rate to match the flash group (0=off)")
    ap.add_argument("--accent-depth", type=float, default=0.22)
    ap.add_argument("--level", type=float, default=0.9, help="master amplitude 0-1 (lower = quieter)")
    args = ap.parse_args()

    pcm = build_siren(args.fs, args.seconds, args.pattern, args.f_lo, args.f_hi,
                      args.wail_period, args.yelp_period, args.wail_len, args.yelp_len,
                      args.accent_hz, args.accent_depth, level=args.level)
    write_wav(args.out, pcm, args.fs)
    peak = float(np.max(np.abs(pcm)) / 32767.0)
    extra = ("  accent=%.2f@%.2fHz" % (args.accent_depth, args.accent_hz)) if args.accent_hz > 0 else ""
    print("wrote %s  fs=%d  dur=%.2fs  samples=%d  peak=%.2f  pattern=%s%s"
          % (args.out, args.fs, len(pcm) / args.fs, len(pcm), peak, args.pattern, extra))


if __name__ == "__main__":
    main()
