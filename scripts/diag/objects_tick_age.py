#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: objects_tick_age.py
Brief: Replay a perception objects capture on the RNS tick and report age tiers

Description:
Answers one question with measured data instead of a derivation: "on the RNS
control tick, how old is the objects message RNS is acting on, and can the
static-obstacle dwell ever complete?"

Why this needs a replay and not the publisher's own age statistic:
  * perception reports publish age (t_publish - t_capture). RNS judges the
    tier on EVERY tick with L2 age = now_tick - t_capture of the last
    ACCEPTED frame (20 S3.1.8, 11 S3.1B.5). Between two frames that age grows
    by one tick per tick, so the per-tick age is a sawtooth whose peaks are
    roughly latency + inter-frame gap -- well above the publish-age P99.
  * a single tick over objects_age_ok_ms resets the dwell of every track
    (source.py tier-2 branch, A-AGE-4), so what decides whether a parked car
    can ever be detoured is the longest RUN of consecutive tier-1 ticks, not
    the share of tier-1 ticks. The report prints both.

Input: the per-frame objects ledger perception's harness writes
(`<run>/ledger/ledger_objects.csv.gz`, columns seq, rx_mono_ns,
t_capture_mono_ms, ...), or a raw `wire.jsonl` capture. rx_mono_ns is the
receive time on the same host CLOCK_MONOTONIC as t_capture, so the replay
needs no clock alignment. A run directory may be given instead of a file.

Defaults are read from configs/rns.yaml (objects_age_ok_ms, t_static_dwell_s)
so the replay judges against the values RNS actually runs with; --thresholds
adds candidate values to compare. T-52 (500 ms) and the 50 ms tick are the
11 S1.6.1 / P1 loop constants.

Windows: --window LABEL=START-END in seconds from the first frame (e.g.
"llm=840-1080"), or wall-clock HH:MM:SS-HH:MM:SS when the run directory
holds CLOCK_OFFSET.txt (wall_minus_mono_ns). Overlapping co-load windows
(an LLM benchmark, a thermal soak) are how a capture is split into "quiet"
and "loaded" without a second run. --decimate N keeps every N-th frame to
estimate a lower inference rate at the same latency.

Read-only: it never touches the capture. Not a gate -- it prints numbers for
a human decision on objects_age_ok_ms (20 S3.1.8, joint value with the
perception side) and exits 0, or 2 on unusable input.

Usage:
  python3 scripts/diag/objects_tick_age.py <run_dir | ledger_objects.csv.gz | wire.jsonl>
  python3 scripts/diag/objects_tick_age.py RUN --window quiet=16:43:19-16:56:19 \\
      --window llm=16:56:49-17:00:49 --decimate 2 --thresholds 250,300
"""

import argparse   # command line
import base64     # wire.jsonl payloads are base64 JSON
import csv        # ledger rows
import gzip       # ledger is gzip-compressed
import json       # wire.jsonl lines and payloads
import os         # paths
import sys        # exit codes
import time       # wall-clock window parsing

import yaml       # configs/rns.yaml (same loader the RNS tests use)

# --------------------------------------------------------------------------
# Constants owned elsewhere -- named here once, with their source
# --------------------------------------------------------------------------

#: P1 control loop period: RNS is ticked at 20 Hz (20 S3.1.10 F-1).
TICK_MS = 50  # one tick = 50 ms

#: 11 S1.6.1 T-52: objects older than this is a lost channel (tier 3).
#: Mirrors source.T52_OBJECTS_MS; reported, not judged, by this tool.
T52_MS = 500  # tier 3 boundary

#: Repo root: this file is scripts/diag/objects_tick_age.py.
# Scripts/diag -> repo root
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# --------------------------------------------------------------------------
# Input
# --------------------------------------------------------------------------

def resolve_input(path):  # file or run dir in
    """Map a run directory to the best capture inside it.

    The ledger is preferred: it survives when the raw wire capture has been
    pruned (perception deletes multi-hundred-MB wire files after a run) and
    it is two orders of magnitude smaller to read.
    """
    if os.path.isdir(path):  # a run directory was given
        for rel in ("ledger/ledger_objects.csv.gz", "wire.jsonl"):  # preference order
            cand = os.path.join(path, rel)  # candidate path
            if os.path.exists(cand):  # first one present wins
                return cand, path        # capture file, run dir (for clock)
        # Nothing replayable here
        raise FileNotFoundError("no ledger_objects.csv.gz or wire.jsonl under " + path)
    return path, os.path.dirname(os.path.dirname(path)) if "ledger" in path \
        else os.path.dirname(path)       # run dir guess for CLOCK_OFFSET.txt


def load_frames(capture):  # -> [(rx_ms, tc_ms)]
    """Return [(rx_ms, t_capture_ms)] sorted by receive time.

    Frames without t_capture are skipped: RNS's three_keys rejects such a
    message outright (19 S2.4), so it never becomes the accepted frame.
    """
    frames = []  # (receive ms, capture ms) pairs
    if capture.endswith(".csv.gz"):  # perception ledger format
        with gzip.open(capture, "rt", encoding="utf-8") as fh:  # text mode for csv
            for row in csv.DictReader(fh):  # one row per received frame
                if row.get("t_capture_mono_ms"):  # empty cell = no stamp
                    frames.append((int(row["rx_mono_ns"]) / 1e6,  # ns -> ms, same clock as tc
                                   int(row["t_capture_mono_ms"])))  # already ms
    else:  # raw wire capture
        with open(capture, encoding="utf-8") as fh:  # can be hundreds of MB
            for line in fh:  # streamed, never loaded whole
                if '/rt/perception/objects"' not in line:
                    continue  # cheap pre-filter: most lines are profile
                rec = json.loads(line)  # one received packet
                body = json.loads(base64.b64decode(rec["payload_b64"]))  # decode payload
                body = body.get("data", body)  # envelope or bare payload
                if body.get("t_capture_mono_ms") is not None:  # stamped frames only
                    # Rx ns -> ms
                    frames.append((rec["rx_mono_ns"] / 1e6, body["t_capture_mono_ms"]))
    frames.sort()  # receive order is the order RNS sees them
    return frames  # sorted by receive time


def accepted_only(frames):  # 11 S3.1B.5 acceptance
    """Drop frames RNS would not accept: 11 S3.1B.5 strictly increasing
    t_capture. A duplicate or late frame never refreshes the age, so keeping
    it would make the replay look fresher than RNS ever is."""
    out, last = [], None  # accepted frames, last accepted t_capture
    for rx, tc in frames:  # receive order
        if last is None or tc > last:  # strictly newer only
            out.append((rx, tc))  # accepted
            last = tc  # new baseline
    return out  # what RNS consumes


def read_rns_defaults():  # -> (ok_ms, dwell_s)
    """(objects_age_ok_ms, t_static_dwell_s) from configs/rns.yaml.

    Read rather than hard-coded so the replay judges against the deployed
    values; a missing key is an error, not a silent default (CLAUDE.md 3.1).
    """
    # The ONE rns.yaml
    with open(os.path.join(ROOT, "configs", "rns.yaml"), encoding="utf-8") as fh:
        rns = yaml.safe_load(fh)["rns"]  # top-level rns: block
    return (int(rns["perception"]["objects_age_ok_ms"]),  # tier 1/2 boundary
            float(rns["dynamic"]["t_static_dwell_s"]))  # static dwell, s


def wall_offset_ms(run_dir):  # -> ms or None
    """wall - mono offset in ms from CLOCK_OFFSET.txt, or None if absent.

    The harness writes it at start and end; the start line is used (the
    two agree to microseconds over a 30 min run on this box)."""
    path = os.path.join(run_dir, "CLOCK_OFFSET.txt")  # written by the harness
    if not os.path.exists(path):  # not every run has it
        return None  # wall windows unavailable
    with open(path, encoding="utf-8") as fh:  # two key=value lines
        for tok in fh.read().split():  # tokens on both lines
            if tok.startswith("wall_minus_mono_ns="):  # start-of-run value
                return int(tok.split("=", 1)[1]) / 1e6  # ns -> ms
    return None  # no usable offset


def parse_window(spec, t0_ms, offset_ms, day):  # -> (label, a_ms, b_ms)
    """LABEL=A-B -> (label, start_rx_ms, end_rx_ms).

    A and B are seconds from the first frame, or HH:MM:SS wall time (needs
    the run's clock offset; `day` is the capture's local date)."""
    label, rng = spec.split("=", 1)  # "llm=..." -> label, range
    a, b = rng.split("-", 1)  # start, end

    def conv(x):  # endpoint converter
        """One endpoint in the receive (mono) time base."""
        if ":" in x:  # HH:MM:SS form
            if offset_ms is None:  # needs the run clock
                # Refuse rather than guess
                raise ValueError("wall-clock window needs CLOCK_OFFSET.txt in the run dir")
            # Local wall seconds
            wall = time.mktime(time.strptime(day + " " + x, "%Y-%m-%d %H:%M:%S"))
            return wall * 1000.0 - offset_ms  # wall -> mono ms
        return t0_ms + float(x) * 1000.0  # seconds-from-start form
    return label, conv(a), conv(b)  # window in mono ms


# --------------------------------------------------------------------------
# Replay
# --------------------------------------------------------------------------

def tick_ages(frames, tick_ms=TICK_MS):  # -> one age per tick
    """Age RNS sees on each tick: now - t_capture of the newest frame
    received at or before the tick. Ticks before the first frame are skipped
    (no accepted message yet -- RNS's own null path, not an age)."""
    ages = []  # one per tick
    i, last_tc = 0, None  # frame cursor, newest tc
    tick, end = frames[0][0], frames[-1][0]  # replay span
    while tick <= end:  # one iteration per tick
        while i < len(frames) and frames[i][0] <= tick:  # frames that arrived by now
            last_tc = frames[i][1]  # newest accepted frame so far
            i += 1  # advance cursor
        if last_tc is not None:  # skip pre-first-frame
            ages.append(tick - last_tc)  # L2 age (20 S3.1.8)
        tick += tick_ms  # next tick
    return ages  # per-tick ages


def dwell_stats(ages, thr_ms, dwell_s, tick_ms=TICK_MS):  # -> (over, best_s, reach)
    """(share of ticks over thr, longest tier-1 run in s, share of ticks on
    which a dwell of dwell_s could have completed).

    Mirrors the A-AGE-4 reset: any tick over thr restarts the run."""
    need = int(round(dwell_s * 1000.0 / tick_ms))  # consecutive ticks needed
    run = best = reach = over = 0  # counters
    for a in ages:  # tick by tick
        if a > thr_ms:  # tier 2 (or worse)
            over += 1  # count it
            run = 0            # tier 2: dwell reset
        else:  # tier 1
            run += 1  # extend run
            reach += run >= need  # dwell done on this tick
        best = max(best, run)  # longest run so far
    n = max(len(ages), 1)  # avoid /0
    return over / n, best * tick_ms / 1000.0, reach / n  # shares and seconds


def pct(sorted_vals, q):  # helper
    """Nearest-rank percentile of an already sorted list."""
    return sorted_vals[min(len(sorted_vals) - 1, int(q * len(sorted_vals)))]  # clamped index


def summarize(label, frames, thresholds, dwell_s):  # -> table row
    """One result row (list of cells) for a frame set."""
    lat = sorted(rx - tc for rx, tc in frames)          # receive latency L
    gaps = [b[0] - a[0] for a, b in zip(frames, frames[1:])]  # inter-frame gaps, ms
    span_s = (frames[-1][0] - frames[0][0]) / 1000.0  # capture span, s
    ages = tick_ages(frames)  # replayed tick ages
    s = sorted(ages)  # for percentiles
    row = [label, "%.1f" % (len(frames) / span_s if span_s else 0.0),  # set name, rate
           # #20-11 tier-2 gap share
           "%.1f" % (100.0 * sum(g > 50 for g in gaps) / max(len(gaps), 1)),
           "%.0f/%.0f" % (pct(lat, .5), pct(lat, .99)),  # receive latency
           "%.0f/%.0f/%.0f/%.0f" % (pct(s, .5), pct(s, .99), pct(s, .999), s[-1]),  # what RNS sees
           "%.2f" % (100.0 * sum(a > T52_MS for a in ages) / len(ages))]  # tier 3 share
    for thr in thresholds:  # one column per threshold
        over, best, reach = dwell_stats(ages, thr, dwell_s)  # replay at this value
        row.append("%.1f%% / %.1fs / %.1f%%" % (100 * over, best, 100 * reach))  # over/run/reach
    return row  # cells


def render(headers, rows):  # table renderer
    """Plain ASCII table (same style as tests/llm/benchmark.py)."""
    # Column widths
    w = [max(len(str(h)), *(len(str(r[i])) for r in rows)) for i, h in enumerate(headers)]
    sep = "+" + "+".join("-" * (x + 2) for x in w) + "+"  # border

    def fmt(cells):  # row formatter
        """One row, right-aligned."""
        return "|" + "|".join(" %s " % str(c).rjust(x) for c, x in zip(cells, w)) + "|"
    # Header, =, rows, border
    return "\n".join([sep, fmt(headers), sep.replace("-", "=")] + [fmt(r) for r in rows] + [sep])


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():  # entry point
    """Parse, replay every window (and decimations), print the table."""
    ap = argparse.ArgumentParser(description="Replay objects capture on the RNS tick")  # parser
    # Positional input
    ap.add_argument("capture", help="run dir, ledger_objects.csv.gz or wire.jsonl")
    ap.add_argument("--window", action="append", default=[],  # repeatable
                    help="LABEL=START-END (s from first frame, or HH:MM:SS wall)")
    ap.add_argument("--skip-s", type=float, default=30.0,  # warm-up skip
                    help="drop the first N s as warm-up (perception ledger rule)")
    ap.add_argument("--decimate", type=int, action="append", default=[],  # repeatable
                    help="also replay keeping every N-th frame (lower rate)")
    ap.add_argument("--thresholds", default="",  # candidates
                    help="extra objects_age_ok_ms candidates, comma separated")
    args = ap.parse_args()  # Namespace

    try:  # input problems are exit 2
        capture, run_dir = resolve_input(args.capture)  # file + run dir
        frames = accepted_only(load_frames(capture))  # RNS-accepted frames
    except (OSError, ValueError, KeyError) as e:  # unreadable / malformed
        print("unusable input: %s" % e, file=sys.stderr)  # say why
        return 2  # unusable input
    if len(frames) < 100:  # too short to mean anything
        # Say why
        print("only %d objects frames -- too few to replay" % len(frames), file=sys.stderr)
        return 2  # unusable input

    ok_ms, dwell_s = read_rns_defaults()  # deployed values
    # Deployed first, then candidates
    thresholds = [ok_ms] + [int(x) for x in args.thresholds.split(",") if x.strip()]
    t0 = frames[0][0]  # first frame rx
    steady = [f for f in frames if f[0] >= t0 + args.skip_s * 1000.0]  # post warm-up
    offset = wall_offset_ms(run_dir)  # for wall windows
    day = (time.strftime("%Y-%m-%d", time.localtime((t0 + offset) / 1000.0))  # capture local date
           if offset is not None else None)

    sets = [("whole run", steady)]  # (label, frames) to replay
    for spec in args.window:  # user windows
        label, a, b = parse_window(spec, t0, offset, day)  # -> mono range
        sets.append((label, [f for f in steady if a <= f[0] < b]))  # frames inside it
    for n in args.decimate:  # lower-rate estimates
        # Decimate base sets only
        sets += [("%s /%d" % (lab, n), fr[::n]) for lab, fr in list(sets) if "/" not in lab]

    # One row per set
    rows = [summarize(lab, fr, thresholds, dwell_s) for lab, fr in sets if len(fr) > 10]
    # Provenance line
    print("capture: %s  (%d accepted objects frames, tick %d ms)" % (capture, len(frames), TICK_MS))
    # What is judged
    print("judged against rns.yaml objects_age_ok_ms=%d, t_static_dwell_s=%.1f; T-52=%d"
          % (ok_ms, dwell_s, T52_MS))
    # Cell legend
    print("per threshold cell: ticks over / longest tier-1 run / ticks where the dwell completes")
    headers = ["set", "Hz", "gap>50ms %", "L p50/p99", "tick age p50/p99/p99.9/max",  # columns
               ">T52 %"] + ["thr %d" % t for t in thresholds]
    print(render(headers, rows))  # the table
    return 0  # success


if __name__ == "__main__":  # script entry
    sys.exit(main())  # exit code
