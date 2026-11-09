#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: benchmark.py
Brief: Benchmark the on-box asr-service (services/asr) and print a result table

Description:
Measures the ASR service the way AI_runtime uses it -- one multipart POST to
/v1/audio/transcriptions per utterance on the loopback port -- and reports the
numbers that decide whether the speech stage fits its budget:

  * client end-to-end latency (upload + decode + response), the figure the
    dialogue turn actually waits for;
  * server decode time and RTF (decode time / audio time). The service
    returns rtf and audio_duration_ms, so decode_ms = rtf * duration -- the
    server's own number, free of HTTP overhead. Requirement: RTF < 0.5, and
    00 S4.10.6 budgets ~350 ms of decode for a 3 s utterance;
  * accuracy against known text (character error rate and exact-match count),
    as a guard: a fast run that transcribes garbage is not a result;
  * the server's CPU time per run, read from /proc, so "fast because it used
    many cores" can be told apart from "fast on one core";
  * memory: resident and peak (VmHWM) of the server process;
  * the AS-12 admission rule: two simultaneous requests must give one 200 and
    one IMMEDIATE 429 E_BUSY -- the opposite of llama-server, which queues.

Two CPU placements, because they give very different numbers:
  * as-launched: whatever affinity the server was started with (from a shell
    that is all 8 cores);
  * core 7: 11 AIR-P3 pins the service to CPUAffinity=7 under systemd, a core
    it shares with P4/P5/HMI. The benchmark re-pins every thread of the RUNNING
    server with sched_setaffinity (no restart, no root: same user) and restores
    the original mask afterwards, even on error. This is the production figure.

Audio corpus: the six 16 kHz gold WAVs in services/asr/selftest/ (synthesized
speech, see its README -- fine for timing, NOT an accuracy benchmark). Longer
cases are built by concatenating them with 300 ms of silence, and the expected
text is the concatenation of their gold texts, so CER stays meaningful.

Optional live microphone (--mic SECONDS): records from the USB HKMIC through
ALSA plughw (the device is natively 2 ch 44.1/48 kHz; plughw converts to the
16 kHz mono the interface requires), sends it once, and prints the level and
the transcript. It checks the capture chain end to end; it is not timed.

What this does NOT do: start or stop the service (start it with
services/asr/asr_server.sh first), judge pass/fail (it prints a verdict line
for RTF only, against the 0.5 requirement), or write audio to disk outside a
temp file deleted right after the upload (11 AS-14 is about the service).
Not collected by pytest: no test_ prefix, needs a live service.

Usage:
  python3 tests/asr/benchmark.py                   # both placements, 5 runs
  python3 tests/asr/benchmark.py --runs 10 --no-pin
  python3 tests/asr/benchmark.py --mic 4           # also a live-mic check
"""

# Standard library only, so the benchmark does not depend on the very packages
# whose deployment it is meant to check.
import argparse    # command line
import io          # in-memory WAV building
import json        # responses, --json-out
import math        # dBFS
import os          # /proc, sched_setaffinity
import statistics  # median
import subprocess  # arecord for --mic
import sys         # exit codes
import tempfile    # mic capture file
import threading   # concurrency probe
import time        # monotonic clock
import unicodedata  # display width of Chinese cells in the table
import urllib.error    # 429 arrives as HTTPError
import urllib.request  # HTTP client
import uuid        # multipart boundary
import wave        # WAV read/write

# --------------------------------------------------------------------------
# Defaults
# --------------------------------------------------------------------------

#: Loopback only (11 AS-13 / AIR-P1); 18081 is the 99 U52 port of asr-service.
DEFAULT_URL = "http://127.0.0.1:18081"  # override with --url

#: This file lives in tests/asr/, the corpus in services/asr/selftest/.
# Tests/asr/benchmark.py -> repo root
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SELFTEST_DIR = os.path.join(REPO_ROOT, "services", "asr", "selftest")  # gold WAVs + gold.json

#: The production core (11 AIR-P3, deploy/systemd/xbrain-ai-asr.service).
PROD_CORE = 7  # the single shared core

#: The contract rate (11 AS-2); the service rejects anything else with 400.
RATE = 16000  # Hz

#: Silence between concatenated utterances, as a speaker's pause would be.
GAP_MS = 300  # ms of zeros between joined clips

#: Requirement the verdict line checks (11 AS, RTF < 0.5).
RTF_LIMIT = 0.5  # hard requirement

#: Default ALSA capture device for --mic: the USB HKMIC by card NAME, so the
#: index can move between boots without breaking it.
MIC_DEVICE = "plughw:CARD=HKMIC,DEV=0"  # see --mic-device


# --------------------------------------------------------------------------
# Corpus
# --------------------------------------------------------------------------

def load_corpus():  # corpus loader
    """[(name, pcm_bytes, gold_text)] from selftest/gold.json, in file order.

    Every file is checked to be 16 kHz mono 16-bit, because concatenating
    frames of different formats would produce noise without any error.
    """
    # Manifest shipped with the service
    with open(os.path.join(SELFTEST_DIR, "gold.json"), encoding="utf-8") as fh:
        cases = json.load(fh)["cases"]  # [{"wav":..., "text":...}]
    out = []  # one entry per gold utterance
    for c in cases:  # gold.json order = report order
        with wave.open(os.path.join(SELFTEST_DIR, c["wav"]), "rb") as w:  # read-only build artefact
            fmt = (w.getframerate(), w.getnchannels(), w.getsampwidth())  # (rate, ch, bytes)
            if fmt != (RATE, 1, 2):  # rate, mono, 16-bit
                # Refuse rather than mix formats
                raise ValueError("%s is %s, need (16000, 1, 2)" % (c["wav"], fmt))
            # Raw PCM without the header
            out.append((c["wav"], w.readframes(w.getnframes()), c["text"]))
    return out  # [(name, pcm, text)]


def to_wav(pcm):  # helper for build_cases
    """Wrap raw 16 kHz mono s16le bytes in a RIFF header (in memory)."""
    buf = io.BytesIO()  # never touches disk
    with wave.open(buf, "wb") as w:  # header written on close
        w.setnchannels(1)    # mono
        w.setsampwidth(2)    # 16-bit
        w.setframerate(RATE)  # contract rate
        w.writeframes(pcm)  # payload
    return buf.getvalue()  # complete WAV file bytes


def build_cases(corpus):  # case table
    """Return {case: [(wav_bytes, gold_text, audio_s), ...]} for the run.

    * short: each gold file on its own (~1.1-1.7 s) -- a spoken command;
    * medium: three files joined (~4.8 s) -- a sentence-length request,
      closest to the 3 s figure the decode budget is written for;
    * long: all six joined (~9.8 s) -- an unusually long turn, to show how
      decode time scales with length.

    ! A joined clip can score worse than its parts: seen 2026-10-20, "命名为"
    alone is exact but after "...围栏" it decodes as "并用营" (CER 0.12 on
    that medium clip). That is the model reacting to cross-sentence context
    on synthesized audio, not a benchmark fault -- read medium/long CER as a
    sanity guard, never as an accuracy figure.
    """
    gap = b"\x00\x00" * (RATE * GAP_MS // 1000)  # 16-bit zero samples

    def join(items):  # closure over gap
        """One clip from several corpus items, with its concatenated gold."""
        pcm = gap.join(p for _, p, _ in items)  # pauses between, none at ends
        # (wav, gold, seconds)
        return (to_wav(pcm), "".join(t for _, _, t in items), len(pcm) / 2.0 / RATE)
    return {  # insertion order = run order
        "short": [join([c]) for c in corpus],  # 6 clips
        "medium": [join(corpus[0:3]), join(corpus[3:6])],  # 2 clips
        "long": [join(corpus)],  # 1 clip
    }  # end of case table


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------

def get_json(url, timeout=10):  # control endpoints only
    """GET a JSON endpoint and decode it."""
    with urllib.request.urlopen(url, timeout=timeout) as r:  # raises on 4xx/5xx
        return json.loads(r.read().decode("utf-8"))  # all replies are JSON


def transcribe(base, wav_bytes, timeout=60):  # the AI_runtime call
    """POST one WAV; return (http_status, body_dict, client_ms).

    multipart/form-data is built by hand -- urllib has no helper for it and
    the stdlib-only rule keeps `requests` out. A 4xx/5xx is RETURNED, not
    raised: the concurrency probe needs the 429 as data.
    """
    boundary = uuid.uuid4().hex  # cannot occur inside PCM by chance (in practice)
    body = (b"--" + boundary.encode() + b"\r\n"  # one form field "file"
            b'Content-Disposition: form-data; name="file"; filename="a.wav"\r\n'
            b"Content-Type: audio/wav\r\n\r\n" + wav_bytes + b"\r\n"
            b"--" + boundary.encode() + b"--\r\n")
    req = urllib.request.Request(  # same endpoint as AI_runtime
        base + "/v1/audio/transcriptions", data=body,
        headers={"Content-Type": "multipart/form-data; boundary=" + boundary})
    t0 = time.monotonic()  # monotonic: immune to NTP steps
    try:  # HTTPError carries 4xx/5xx
        with urllib.request.urlopen(req, timeout=timeout) as r:  # raises HTTPError on >= 400
            status, raw = r.status, r.read()  # 200 path
    except urllib.error.HTTPError as e:  # turn the error into data
        status, raw = e.code, e.read()  # 400 / 429 / 503 path
    ms = (time.monotonic() - t0) * 1000.0  # client wall time
    try:  # body may be an HTML error page
        data = json.loads(raw.decode("utf-8"))  # service replies JSON
    except ValueError:  # non-JSON body
        data = {"raw": raw[:200].decode("utf-8", "replace")}  # non-JSON error page
    return status, data, ms  # never raises for HTTP status


# --------------------------------------------------------------------------
# Server process: find it, read its CPU time and memory, pin it
# --------------------------------------------------------------------------

def find_server_pid():  # used for pinning and /proc reads
    """PID of `python3 -m services.asr.app`, or None if not found/visible.

    Scanned from /proc rather than via pgrep, so the benchmark's own command
    line can never match itself (it does not contain the module name).
    """
    for pid in filter(str.isdigit, os.listdir("/proc")):  # every process visible to us
        try:  # process may vanish mid-scan
            with open("/proc/%s/cmdline" % pid, "rb") as fh:  # argv, NUL separated
                argv = fh.read().split(b"\0")  # NUL-separated argv
        except OSError:  # gone or not ours
            continue  # process exited mid-scan
        if b"services.asr.app" in argv:  # exact argv element match
            return int(pid)  # first match: one service
    return None  # not running, or another user


def cpu_seconds(pid):  # per-request CPU accounting
    """utime + stime of the whole process in seconds (all threads)."""
    with open("/proc/%d/stat" % pid) as fh:  # kernel accounting line
        # Fields after the ")" of comm: comm may contain spaces, so split there.
        fields = fh.read().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")  # fields 14, 15


def memory_mb(pid):  # read once at the end
    """(VmRSS, VmHWM) in MB: current and peak resident memory."""
    vals = {}  # key -> kB
    with open("/proc/%d/status" % pid) as fh:  # kernel memory summary
        for line in fh:  # "Key:   value kB" lines
            if line.startswith(("VmRSS:", "VmHWM:")):  # current and high-water
                k, v = line.split(":")  # "VmRSS", " 123 kB"
                vals[k] = int(v.split()[0])  # "123456 kB"
    return vals.get("VmRSS", 0) / 1024.0, vals.get("VmHWM", 0) / 1024.0  # kB -> MB


def thread_ids(pid):  # tids
    """Every thread of the process; affinity is per thread on Linux."""
    return [int(t) for t in os.listdir("/proc/%d/task" % pid)]  # one directory per thread


def set_affinity(pid, cpus):  # pin helper
    """Pin every existing thread of `pid` to `cpus`; return the old masks.

    Per thread, because sched_setaffinity(pid) only moves the main thread --
    the onnxruntime and uvicorn worker threads would stay where they were and
    the "pinned" figure would be fiction. Threads created later inherit from
    their creator, which is now pinned too.
    """
    old = {}  # tid -> previous mask, for restore
    for tid in thread_ids(pid):  # snapshot of current threads
        try:  # races with thread exit
            old[tid] = os.sched_getaffinity(tid)  # remember for restore
            os.sched_setaffinity(tid, cpus)  # the actual pin
        except OSError:  # thread exited
            pass  # thread exited between listdir and the call
    return old  # tid -> mask


def restore_affinity(old):  # undo helper
    """Put each thread back on the mask it had before set_affinity()."""
    for tid, mask in old.items():  # only threads we changed
        try:  # may have exited since
            os.sched_setaffinity(tid, mask)  # back to the original mask
        except OSError:  # thread exited
            pass  # thread gone; nothing to restore


# --------------------------------------------------------------------------
# Accuracy
# --------------------------------------------------------------------------

def cer(ref, hyp):  # accuracy guard
    """Character error rate = Levenshtein(ref, hyp) / len(ref).

    Characters, not words: Chinese has no spaces to split on. Punctuation and
    spaces are dropped from both sides first, so a model that adds a comma is
    not scored as wrong -- the same normalisation idea as the service's own
    selftest, kept local so this file stays standalone.
    """
    def norm(s):  # normaliser
        """Keep letters/digits/CJK only."""
        return [ch for ch in s if ch.isalnum()]  # drops punctuation and spaces
    r, h = norm(ref), norm(hyp)  # char lists
    prev = list(range(len(h) + 1))  # DP row for the empty prefix of r
    for i, rc in enumerate(r, 1):  # rows: reference chars
        cur = [i]  # deleting i chars of r
        for j, hc in enumerate(h, 1):  # columns: hypothesis chars
            cur.append(min(prev[j] + 1,                 # deletion
                           cur[j - 1] + 1,              # insertion
                           prev[j - 1] + (rc != hc)))   # substitution / match
        prev = cur  # roll the DP row
    return prev[-1] / max(len(r), 1)  # edits / reference length


# --------------------------------------------------------------------------
# Measurement
# --------------------------------------------------------------------------

def run_case(base, pid, clips, runs, warmup):  # one case, one placement
    """Measure one case: every clip, `runs` times, after `warmup` passes.

    Returns a list of per-request dicts. CPU time is the server's delta over
    each single request, which is exact because requests run one at a time.
    """
    for _ in range(warmup):  # warmup passes
        for wav, _, _ in clips:  # every clip once
            transcribe(base, wav)  # discarded: first decodes pay allocator growth
    out = []  # per-request measurements
    for _ in range(runs):  # measured passes
        for wav, gold, audio_s in clips:  # every clip of the case
            c0 = cpu_seconds(pid) if pid else None  # before: server CPU seconds
            status, data, ms = transcribe(base, wav)  # the timed request
            c1 = cpu_seconds(pid) if pid else None  # after
            if status != 200:  # a 4xx/5xx here is a real fault
                raise RuntimeError("HTTP %d: %s" % (status, data))  # abort: numbers would be wrong
            err = cer(gold, data.get("text", ""))  # computed once, used twice
            rtf = float(data.get("rtf", 0.0))  # server-side decode / audio
            out.append({  # one row of raw data
                "audio_s": audio_s,  # audio length, s
                "client_ms": ms,  # client E2E
                "decode_ms": rtf * audio_s * 1000.0,  # server decode, no HTTP
                "rtf": rtf,  # as reported
                # Server CPU during this request
                "cpu_ms": None if c0 is None else (c1 - c0) * 1000.0,
                "cer": err,
                "exact": err == 0.0,
                "text": data.get("text", ""),  # kept for --json-out
            })
    return out  # per-request list


def concurrency_probe(base, wav):  # AS-12 probe
    """Two simultaneous requests: expect one 200 and one fast 429 E_BUSY.

    The two threads wait on a barrier so they really are concurrent; a 429
    that arrives only after the 200 would mean the service queued instead of
    refusing, which 11 AS-12 forbids.
    """
    gate = threading.Barrier(2)  # release both at the same instant
    res = [None, None]  # [(status, body, ms)] x 2

    def worker(i):  # thread body
        """One request after the barrier; result kept in its own slot."""
        gate.wait()  # start together
        res[i] = transcribe(base, wav)  # one request
    th = [threading.Thread(target=worker, args=(i,)) for i in range(2)]  # two clients
    for t in th:  # launch
        t.start()
    for t in th:  # wait for both
        t.join()  # transcribe() bounds itself with a timeout
    return res  # order = thread index


def record_mic(device, seconds):  # --mic helper
    """Record `seconds` from ALSA `device` as 16 kHz mono s16 WAV bytes.

    arecord writes to a temp file that is deleted as soon as it is read --
    nothing of the live audio stays on disk.
    """
    fd, path = tempfile.mkstemp(suffix=".wav")  # temp path for arecord
    os.close(fd)  # arecord opens it by name
    try:  # delete even on failure
        subprocess.run(["arecord", "-q", "-D", device, "-f", "S16_LE", "-r", str(RATE),
                        "-c", "1", "-d", str(seconds), path], check=True, timeout=seconds + 10)
        with open(path, "rb") as fh:  # whole file incl. header
            return fh.read()
    finally:  # cleanup
        os.unlink(path)  # always remove the capture


def level_dbfs(wav_bytes):  # mic level
    """(peak, rms) of a 16-bit WAV in dBFS; -inf for digital silence."""
    with wave.open(io.BytesIO(wav_bytes), "rb") as w:  # parse the capture
        raw = w.readframes(w.getnframes())  # all samples
    s = memoryview(raw).cast("h")  # signed 16-bit samples, native = little endian here
    if len(s) == 0:  # zero-length capture
        return float("-inf"), float("-inf")  # no level to report
    peak = max(abs(x) for x in s)  # loudest sample
    rms = math.sqrt(sum(x * x for x in s) / len(s))  # average energy

    def db(v):  # amplitude -> dB
        """Amplitude -> dBFS relative to full scale 32768."""
        return 20 * math.log10(v / 32768.0) if v > 0 else float("-inf")
    return db(peak), db(rms)  # (peak, rms)


# --------------------------------------------------------------------------
# Statistics and tables (same ASCII style as tests/llm/benchmark.py)
# --------------------------------------------------------------------------

def med_p90(vals, nd=0):  # cell formatter
    """ "median (p90)" with nearest-rank p90; "-" for an empty list."""
    vs = sorted(v for v in vals if v is not None)  # cpu_ms may be None
    if not vs:  # nothing measured
        return "-"  # table placeholder
    p90 = vs[min(len(vs) - 1, int(round(0.9 * (len(vs) - 1))))]  # nearest-rank p90
    return "%.*f (%.*f)" % (nd, statistics.median(vs), nd, p90)  # "med (p90)"


def disp_width(s):
    """Terminal columns of `s`: CJK (East Asian Wide/Fullwidth) chars take two.

    len() counts a Chinese character as one, but a terminal draws it two
    columns wide, so a transcript cell padded by len() pushes its "|" right
    and the table border no longer lines up.
    """
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in s)


def render_table(headers, rows):  # table renderer
    """Plain ASCII table, cells right-aligned; pastes cleanly anywhere.

    Borders are ASCII, not box-drawing characters, so the table survives an
    ssh terminal, a log file and a chat paste without an encoding surprise.
    Widths are display widths (see disp_width) so Chinese text cells align.
    """
    # Column width = widest of the header and every cell in that column.
    widths = [max([disp_width(h)] + [disp_width(str(r[i])) for r in rows])
              for i, h in enumerate(headers)]
    sep = "+" + "+".join("-" * (w + 2) for w in widths) + "+"  # one space each side

    def fmt(cells):  # row formatter
        """One row; manual left padding because str.rjust counts len(), not columns."""
        return "|" + "|".join(" %s%s " % (" " * (w - disp_width(str(c))), c)
                              for c, w in zip(cells, widths)) + "|"
    # The "=" rule under the header separates it without needing colour.
    return "\n".join([sep, fmt(headers), sep.replace("-", "=")] + [fmt(r) for r in rows] + [sep])


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def parse_args():  # flags, all optional
    """Command line."""
    ap = argparse.ArgumentParser(description="Benchmark the on-box asr-service")  # argparse
    ap.add_argument("--url", default=DEFAULT_URL, help="service base URL")  # loopback default
    # 5 x 9 clips = 45 per placement
    ap.add_argument("--runs", type=int, default=5, help="measured passes per case")
    # Warmup is per case
    ap.add_argument("--warmup", type=int, default=1, help="discarded passes per case")
    ap.add_argument("--no-pin", action="store_true",
                    help="skip the core-%d placement (as-launched only)" % PROD_CORE)
    # AS-12 check
    ap.add_argument("--no-concurrency", action="store_true", help="skip the 429 probe")
    ap.add_argument("--mic", type=int, default=0, metavar="SECONDS",
                    help="also record SECONDS from the USB mic and transcribe it")
    # Device by card name
    ap.add_argument("--mic-device", default=MIC_DEVICE, help="ALSA capture device")
    ap.add_argument("--json-out", help="also write raw results to this file")  # raw data file
    return ap.parse_args()  # Namespace


def main():  # entry point
    """Run placements x cases, the probe, the optional mic check, then tables."""
    args = parse_args()  # exits itself on --help
    base = args.url.rstrip("/")  # tolerate trailing /
    if args.runs < 1:  # usage check first
        print("--runs must be >= 1", file=sys.stderr)  # usage error
        return 2  # argparse convention
    try:  # service must be up
        health = get_json(base + "/healthz", timeout=5)  # also proves the model loaded
        status = get_json(base + "/status", timeout=5)  # model/decoding identity
    except (urllib.error.URLError, OSError, ValueError) as e:  # not listening / not ready
        print("cannot reach asr-service at %s: %s" % (base, e), file=sys.stderr)  # cause
        print("start it first: bash services/asr/asr_server.sh", file=sys.stderr)  # remedy
        return 1  # no server
    pid = find_server_pid()  # None when run against a remote/other-user server
    model = health.get("models", {}).get("asr", {})  # name + version
    print("asr-service benchmark")  # header
    print("  url=%s model=%s v%s decoding=%s pid=%s"
          % (base, model.get("name"), model.get("version"), status.get("decoding"), pid))

    cases = build_cases(load_corpus())  # built once, reused
    placements = [("as-launched", None)]  # None = leave affinity alone
    if not args.no_pin:  # production placement
        if pid is None:  # other user or container
            print("  (server pid not visible: core-%d placement skipped)" % PROD_CORE)
        else:  # pid known
            placements.append(("core %d" % PROD_CORE, {PROD_CORE}))
    # Everything printed comes from here
    report = {"server": {"health": health, "status": status, "pid": pid},
              "placements": {}, "concurrency": None, "mic": None}
    t_start = time.monotonic()  # for the total-time line

    for pname, cpus in placements:  # as-launched first, then core 7
        old = set_affinity(pid, cpus) if cpus else {}  # {} = untouched
        try:  # restore in finally
            if pid:  # mask is unknown without a pid
                mask = sorted(os.sched_getaffinity(pid))  # what the kernel reports now
                print("\n== placement: %s (server cpus %s)" % (pname, ",".join(map(str, mask))))
            res = {}  # case -> list of per-request dicts
            for cname, clips in cases.items():  # short, medium, long
                res[cname] = run_case(base, pid, clips, args.runs, args.warmup)  # measure
                r = res[cname]  # alias
                print("  [%s] %d requests: e2e %s ms, rtf %s, cer mean %.3f"
                      % (cname, len(r), med_p90([x["client_ms"] for x in r]),
                         med_p90([x["rtf"] for x in r], 3),
                         statistics.mean(x["cer"] for x in r)))
            report["placements"][pname] = res  # store
        finally:  # always runs
            restore_affinity(old)  # never leave the server pinned by accident

    if not args.no_concurrency:  # optional probe
        wav = cases["long"][0][0]  # long clip: the 200 takes long enough to overlap
        report["concurrency"] = concurrency_probe(base, wav)  # two overlapping long requests

    if args.mic > 0:  # optional live check
        # Prompt the speaker
        print("\n[mic] recording %d s from %s -- speak now" % (args.mic, args.mic_device))
        try:  # arecord may be missing
            wav = record_mic(args.mic_device, args.mic)  # temp file deleted inside
            peak, rms = level_dbfs(wav)  # input level
            st, data, ms = transcribe(base, wav)  # not timed for the table
            report["mic"] = {"status": st, "text": data.get("text", data), "client_ms": ms,
                             "peak_dbfs": peak, "rms_dbfs": rms}
        except (OSError, subprocess.SubprocessError) as e:  # arecord missing or device busy
            report["mic"] = {"error": str(e)}  # reported, not raised

    mem = memory_mb(pid) if pid else (None, None)  # peak includes the whole run
    print_tables(report, args.runs, mem)  # final output
    print("\ntotal benchmark time: %.1f s" % (time.monotonic() - t_start))
    if args.json_out:  # optional raw dump
        with open(args.json_out, "w", encoding="utf-8") as fh:  # UTF-8: transcripts are Chinese
            json.dump(report, fh, ensure_ascii=False, indent=2, default=str)
        print("raw results written to %s" % args.json_out)  # where it went
    return 0  # success


def print_tables(report, runs, mem):  # final tables
    """The final tables: performance, admission probe, mic, memory, verdict."""
    print("\n" + "=" * 30 + " RESULTS " + "=" * 30)  # banner
    print("cells are MEDIAN (p90) over all requests of the case (%d passes); "
          "decode/RTF are server-side, E2E is client wall time" % runs)
    rows = []  # performance rows
    worst_rtf = 0.0  # for the verdict line: p90-ish worst over everything
    for pname, res in report["placements"].items():  # placement order
        for cname, r in res.items():  # case order
            exact = sum(x["exact"] for x in r)  # exact matches
            worst_rtf = max([worst_rtf] + [x["rtf"] for x in r])
            rows.append([pname, cname, len(r), "%.2f" % statistics.mean(x["audio_s"] for x in r),
                         med_p90([x["client_ms"] for x in r]),
                         med_p90([x["decode_ms"] for x in r]),
                         med_p90([x["rtf"] for x in r], 3),
                         med_p90([x["cpu_ms"] for x in r]),
                         "%.3f" % statistics.mean(x["cer"] for x in r),
                         "%d/%d" % (exact, len(r))])
    print("\nPerformance")  # table 1
    print(render_table(["placement", "case", "n", "audio s", "E2E ms", "decode ms", "RTF",
                        "server CPU ms", "CER", "exact"], rows))

    if report["concurrency"]:  # table 2
        rows = [["req %d" % (i + 1), st, d.get("code", "-") if st != 200 else "-",
                 "%.0f" % ms] for i, (st, d, ms) in enumerate(report["concurrency"])]
        print("\nAdmission (2 simultaneous long requests; expect one 200 + one fast 429)")
        print(render_table(["request", "HTTP", "code", "E2E ms"], rows))
        codes = sorted(st for st, _, _ in report["concurrency"])  # expected [200, 429]
        print("admission rule (11 AS-12): %s"
              % ("OK -- refused, not queued" if codes == [200, 429] else "UNEXPECTED %s" % codes))

    m = report["mic"]  # table 3
    if m is not None:  # only with --mic
        print("\nLive microphone")  # title
        if "error" in m:  # capture failed
            print("  capture failed: %s" % m["error"])
        else:  # captured
            print(render_table(["HTTP", "peak dBFS", "RMS dBFS", "E2E ms", "text"],
                               [[m["status"], "%.1f" % m["peak_dbfs"], "%.1f" % m["rms_dbfs"],
                                 "%.0f" % m["client_ms"], m["text"] or "(empty)"]]))

    if mem[0] is not None:  # pid known
        print("\nserver memory: RSS %.0f MB, peak (VmHWM) %.0f MB  (systemd MemoryMax=1G)" % mem)
    # The one requirement checked here
    print("RTF verdict: worst single request %.3f vs requirement < %.1f -> %s"
          % (worst_rtf, RTF_LIMIT, "PASS" if worst_rtf < RTF_LIMIT else "FAIL"))


if __name__ == "__main__":  # script entry
    sys.exit(main())  # exit code from main
