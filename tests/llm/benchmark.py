#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: benchmark.py
Brief: Benchmark the on-box llama-server (services/llm) and print a result table

Description:
Measures the LLM service the way AI_runtime actually uses it -- POST
/v1/chat/completions on the loopback port -- and reports the numbers that decide
whether a dialogue turn feels responsive on the robot:

  * TTFT (time to first token), measured on the CLIENT from request send to the
    first streamed content delta. This is what a user waits for before speech
    can start, and it includes HTTP, queueing, template and prompt prefill.
  * prompt prefill speed and generation speed, taken from the SERVER's own
    `timings` block (the last streamed chunk), so they exclude network noise.
  * end-to-end latency on the client, and the token counts actually processed
    (prompt_n includes the chat-template overhead, not just our text).
  * board telemetry sampled by tegrastats while each case runs: GPU load, RAM,
    CPU/GPU/junction temperatures and input power -- so a slow run can be told
    apart from a throttled one. A warning is printed when the hottest reading
    comes within THERMAL_MARGIN_C of the passive (throttle) trip point.
  * a concurrency probe: two simultaneous requests against a server that runs
    with --parallel 1 (11 AS-3). The second one QUEUES rather than being
    refused, and 11 S11A.3.4 is exactly about that; the probe shows how long.

What makes the numbers comparable between runs (each choice is deliberate):
  * cache_prompt=false, so every run pays the full prefill. With the cache on,
    run 2..N would reuse run 1's KV and report a prefill speed that never occurs
    for a fresh utterance.
  * ignore_eos=true with a fixed max_tokens, so every run generates exactly the
    same number of tokens. Without it the model stops whenever it likes and the
    generation rate is averaged over a different length each time.
  * temperature=0 and a fixed seed, so the work done is identical per run.
  * prompts are built to an exact TOKEN length through the server's /tokenize
    and /detokenize, not to a character count -- Chinese characters per token
    vary, and a character-sized prompt would drift between model versions.
  * warmup runs are executed and discarded: the first request after load pays
    CUDA graph capture and allocator growth, which is a one-off cost.

Default cases follow 11 S11A.3.2: prompt <= 900 tokens and predict 256 is the
contract upper bound, so `long` sits right on it; `short` and `medium` cover a
typical one-line question and a context-heavy turn.

What this does NOT do: it does not start or stop the service (start it with
services/llm/llm_server.sh first), it does not judge pass/fail against a
threshold (the contract budget is not frozen, 11 S14.2 F-23), and it does not
measure ASR or TTS. It is not collected by pytest (the file name has no test_
prefix) because it needs a running service and takes minutes.

Usage:
  python3 tests/llm/benchmark.py                 # all cases, 5 runs each
  python3 tests/llm/benchmark.py --runs 10 --cases short,long
  python3 tests/llm/benchmark.py --json-out /tmp/llm_bench.json
"""

# Standard library only: the box has no pip dependencies installed yet (R36
# bring-up), and a benchmark that needs `requests` would fail before measuring.
import argparse    # command line
import json        # request bodies, SSE chunks, --json-out
import os          # basename of the model path
import re          # tegrastats line parsing
import statistics  # median / mean
import subprocess  # tegrastats child process
import sys         # exit code, stderr
import threading   # tegrastats reader + concurrency probe
import time        # monotonic clock for TTFT / E2E
import urllib.error    # connection errors are caught by type
import urllib.request  # HTTP client

# --------------------------------------------------------------------------
# Defaults
# --------------------------------------------------------------------------

#: The service binds loopback only (11 AIR-P1), so the benchmark runs on the
#: box itself. Port 18082 is the 99 U52 allocation used by llm_server.sh.
DEFAULT_URL = "http://127.0.0.1:18082"  # override with --url

#: name -> (prompt tokens of OUR text, tokens to generate). The server adds
#: the chat-template tokens on top, and the table reports the real prompt_n.
#: Insertion order is the run order and the table row order.
CASES = {  # case table, see comment above
    "short": (32, 64),     # one-line question, short spoken reply
    "medium": (300, 128),  # turn carrying some dialogue history
    "long": (900, 256),    # 11 S11A.3.2 upper bound: prompt <= 900, predict 256
}

#: Filler for building prompts. Chinese because that is the real workload;
#: punctuation is ASCII because this is a source file (CLAUDE.md 2.2). The
#: content is irrelevant to the measurement -- only its token count matters.
FILLER = (
    "消防场区巡检机器人在夜间执行巡逻任务, 沿预设路线经过仓库, 配电房和停车场, "
    "通过深度相机和激光雷达感知周围障碍物, 遇到可疑人员时进行语音喊话和灯光警示, "
    "并把现场视频和事件实时上报到云端值班平台. "
)

#: How close (C) to the passive trip point counts as "possibly throttled".
#: 5 C because the trip is where the governor STARTS cutting clocks, and a
#: sampled max lags the true peak by up to one tegrastats interval.
THERMAL_MARGIN_C = 5

#: sysfs zone whose lowest-but-real passive trip is reported. gpu-thermal
#: because the LLM load is GPU-bound; on Orin its trips are 70 C (fan curve
#: step, not a throttle) and 99 C (throttle), hence the >= 80 filter below.
TRIP_ZONE = "gpu-thermal"


def passive_trip_c():
    """The GPU zone's throttling passive trip in C, or None if unreadable.

    Read from sysfs rather than hard-coded: trip points differ between
    JetPack releases and carrier boards, and the warning must match the board.
    """
    import glob  # local: only this helper needs it
    for zone in glob.glob("/sys/devices/virtual/thermal/thermal_zone*"):
        try:
            if open(zone + "/type").read().strip() != TRIP_ZONE:
                continue  # some other sensor
            trips = []  # passive trips of this zone, in C
            for tp in glob.glob(zone + "/trip_point_*_type"):
                if open(tp).read().strip() == "passive":
                    temp = int(open(tp.replace("_type", "_temp")).read()) / 1000.0
                    if temp >= 80.0:  # skip the 70 C fan-curve trip, see TRIP_ZONE
                        trips.append(temp)
            return min(trips) if trips else None  # first real throttle step
        except (OSError, ValueError):
            return None  # unreadable sysfs: no warning rather than a wrong one
    return None  # not a Jetson


#: The question appended after the filler, so the model has a real task.
QUESTION = "请根据以上内容, 用简短的话总结机器人的主要工作."


# --------------------------------------------------------------------------
# HTTP helpers (stdlib only, so nothing extra has to be installed on the box)
# --------------------------------------------------------------------------

def http_json(url, payload=None, timeout=30.0):  # 30 s: tokenize of 900 tok is ms
    """GET (payload None) or POST a JSON body and return the decoded JSON.

    Used only for the small control endpoints (/health, /props, /tokenize,
    /detokenize). The streamed completion has its own reader in run_once()
    because it must timestamp chunks as they arrive, not after the body ends.
    """
    data = None  # None makes urllib issue a GET
    headers = {}  # Content-Type only when there is a body
    if payload is not None:  # POST path
        # ensure_ascii=False keeps the Chinese prompt as UTF-8 bytes instead
        # of \\u escapes, which would triple the request size for no reason.
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")  # body bytes
        headers["Content-Type"] = "application/json"  # llama-server requires it
    req = urllib.request.Request(url, data=data, headers=headers)  # POST iff data
    with urllib.request.urlopen(req, timeout=timeout) as resp:  # raises on 4xx/5xx
        return json.loads(resp.read().decode("utf-8"))  # all control replies are JSON


def server_info(base):  # identity block of the report
    """Collect what identifies the run: health, model, build, slots, n_ctx.

    Recorded so a table pasted into a report says WHICH binary and model
    produced it -- two runs are only comparable if these match. Any key this
    server version does not expose is shown as "?" rather than guessed.
    """
    # /health first: if the model is still loading it answers 503, urlopen
    # raises, and main() reports "cannot reach" instead of timing a cold load.
    info = {"health": http_json(base + "/health", timeout=5).get("status")}  # "ok" when ready
    props = http_json(base + "/props", timeout=5)  # one call carries the rest
    info["model"] = os.path.basename(props.get("model_path", "?"))  # file name is enough
    info["build"] = props.get("build_info", "?")  # e.g. b1-4c1004c
    info["slots"] = props.get("total_slots", "?")  # == --parallel; 1 by 11 AS-3
    # n_ctx sits under default_generation_settings in this server version; the
    # `or {}` covers a server that sends the key with a null value.
    gen = props.get("default_generation_settings", {}) or {}  # may be null
    info["n_ctx"] = gen.get("n_ctx", "?")  # 2048 by llm_server.sh
    return info  # printed in the header and the RESULTS block


# --------------------------------------------------------------------------
# Prompt construction to an exact token length
# --------------------------------------------------------------------------

def build_prompt(base, target_tokens):  # target excludes chat template
    """Return text whose tokenization is exactly `target_tokens` tokens.

    Tokenize a long repetition of the filler once, keep the first N token ids
    minus the question's tokens, then detokenize. Cutting on token ids (not
    characters) is what makes the length exact; the question is kept whole at
    the end so the model always has the same task. The server's own tokenizer
    is used so the count matches the loaded model, whatever it is.
    """
    # The question's own length comes out of the budget, so the total is exact.
    q_tokens = http_json(base + "/tokenize", {"content": QUESTION})["tokens"]  # question length
    budget = max(target_tokens - len(q_tokens), 0)  # a tiny target = question only
    # Each filler repetition is well over 20 tokens, so budget//20 + 2 copies
    # always exceed the budget; the surplus is cut off on the next line.
    text = FILLER * (budget // 20 + 2)  # over-long on purpose
    tokens = http_json(base + "/tokenize", {"content": text})["tokens"][:budget]
    if not tokens:  # budget 0: target shorter than the question
        return QUESTION  # nothing to detokenize; /detokenize rejects an empty list
    body = http_json(base + "/detokenize", {"tokens": tokens})["content"]
    return body + QUESTION  # filler first, task last, as a real turn would be


# --------------------------------------------------------------------------
# One streamed request
# --------------------------------------------------------------------------

def run_once(base, prompt, n_gen, timeout=300.0):  # 300 s: long case is ~15 s
    """Send one streamed chat request and return its measurements.

    TTFT and end-to-end time are client-side; prefill/generation speeds come
    from the server's `timings` in the final chunk. A line that cannot be
    parsed is skipped rather than aborting the run -- SSE keep-alive and
    comment lines are not JSON -- but a run with no timings at all is an
    error, because every figure in the table would then be invented.
    """
    payload = {  # OpenAI chat body + llama.cpp extensions
        "messages": [{"role": "user", "content": prompt}],  # template applied by --jinja
        "max_tokens": n_gen,    # exact length together with ignore_eos
        "stream": True,         # needed to see the FIRST token's arrival time
        "ignore_eos": True,     # fixed generation length, see module doc
        "cache_prompt": False,  # full prefill every run, see module doc
        "temperature": 0,       # greedy: identical work per run
        "seed": 42,             # belt and braces for any sampler left non-greedy
    }
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")  # same as http_json
    # Same endpoint AI_runtime calls, so the path measured is the path used.
    req = urllib.request.Request(base + "/v1/chat/completions", data=data,
                                 headers={"Content-Type": "application/json"})
    t0 = time.monotonic()  # monotonic: wall clock may be stepped by NTP mid-run
    ttft = None     # seconds to first content delta; stays None if none arrives
    timings = None  # server-side timings from the final chunk
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        for raw in resp:  # iterating the response yields one SSE line at a time
            line = raw.decode("utf-8", "replace").strip()  # never die on a split byte
            if not line.startswith("data:"):  # SSE data field only
                continue  # blank separators and ":" comment lines carry nothing
            body = line[5:].strip()  # drop the "data:" field name
            if body == "[DONE]":  # stream terminator
                break  # OpenAI-style end marker; timings came in the chunk before
            try:
                chunk = json.loads(body)  # one SSE event
            except ValueError:  # json.JSONDecodeError subclasses it
                continue  # not JSON: skip, see docstring
            # The first chunk that carries actual text marks TTFT. Role-only
            # deltas arrive before prefill finishes and must not count, or
            # TTFT would read ~0 ms and hide the very cost being measured.
            choices = chunk.get("choices") or [{}]  # final timings chunk may have none
            if ttft is None and choices[0].get("delta", {}).get("content"):
                ttft = time.monotonic() - t0  # first visible token
            if "timings" in chunk:  # present on the final chunk
                timings = chunk["timings"]  # last one wins; only the final chunk has it
    e2e = time.monotonic() - t0  # includes the tail after the last token
    if timings is None:  # old server or broken stream
        raise RuntimeError("server returned no timings block")
    g = timings.get  # short alias: the dict below reads every field once
    return {  # one row of per-run data (also --json-out)
        # A run with no content at all (impossible with ignore_eos, but cheap
        # to guard) reports TTFT == E2E rather than None, so stats still work.
        "ttft_ms": (ttft if ttft is not None else e2e) * 1000.0,
        "e2e_ms": e2e * 1000.0,  # client wall time, whole request
        "prompt_n": g("prompt_n", 0),              # template + our text, tokens
        "prompt_ms": g("prompt_ms", 0.0),          # server prefill time
        "prompt_tps": g("prompt_per_second", 0.0),  # prefill rate
        "gen_n": g("predicted_n", 0),              # should equal max_tokens
        "gen_ms": g("predicted_ms", 0.0),  # server decode time
        "gen_tps": g("predicted_per_second", 0.0),  # decode rate
        "gen_ms_per_tok": g("predicted_per_token_ms", 0.0),  # inverse of gen_tps
    }


# --------------------------------------------------------------------------
# Board telemetry
# --------------------------------------------------------------------------

class TegraSampler:  # one instance, reused per case
    """Run tegrastats in the background and keep the parsed samples.

    tegrastats needs no root for these fields. If it is missing (not a Jetson)
    the sampler records nothing and the table shows "-", rather than failing
    the benchmark over an optional column. Sampling covers only the measured
    runs of one case: warmup and prompt building are outside the window.
    """

    #: Field patterns; any of them may be absent on a given board or line, so
    #: each is matched independently. Names follow the R36 tegrastats format.
    RX = {  # field -> regex
        "ram_mb": re.compile(r"RAM (\d+)/\d+MB"),     # used MB of total
        "gpu_pct": re.compile(r"GR3D_FREQ (\d+)%"),   # GPU busy %
        "cpu_c": re.compile(r"cpu@([\d.]+)C"),        # CPU cluster temperature
        "gpu_c": re.compile(r"gpu@([\d.]+)C"),        # GPU temperature
        "tj_c": re.compile(r"tj@([\d.]+)C"),          # junction (hottest) temp
        # No EMC field: on R36.4.3 with jetson_clocks applied, tegrastats
        # prints no EMC_FREQ at all (seen 2026-10-20), so it was always "-".
        "vin_mw": re.compile(r"VDD_IN (\d+)mW"),      # whole-module input power
    }

    def __init__(self, interval_ms=250):  # does not start sampling
        """250 ms: a short case lasts ~3 s, so this still gives ~12 samples."""
        self.interval_ms = interval_ms  # passed straight to tegrastats
        self.samples = []   # one dict per parsed line
        self.proc = None    # tegrastats child, None when not running
        self.thread = None  # reader thread draining its stdout

    def start(self):  # call right before the measured runs
        """Begin sampling; clears any previous case's samples."""
        self.samples = []  # a fresh window per case
        try:
            self.proc = subprocess.Popen(  # line-buffered text output
                ["tegrastats", "--interval", str(self.interval_ms)],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        except (OSError, ValueError):  # FileNotFoundError is an OSError
            self.proc = None  # not a Jetson: telemetry columns become "-"
            return  # sampler stays inert
        # A reader thread, because tegrastats never exits on its own and a
        # blocking read in the main thread would stall the benchmark.
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()  # returns immediately

    def _read(self):  # runs in the reader thread
        """Parse each line as it arrives; fields absent on a line are skipped."""
        for line in self.proc.stdout:  # ends when stop() terminates the child
            s = {k: float(m.group(1))   # every field this line actually has
                 for k, rx in self.RX.items() for m in [rx.search(line)] if m}
            if s:  # skip lines with none of our fields
                self.samples.append(s)  # list.append is atomic under the GIL

    def stop(self):  # safe to call when start() failed
        """Stop tegrastats and return {field: (mean, max)} over the window."""
        if self.proc is not None:  # only if start() succeeded
            self.proc.terminate()  # SIGTERM; tegrastats exits promptly on it
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()  # never leave a stray sampler behind
            self.proc = None  # stop() is idempotent
        if self.thread is not None:  # reader may be absent
            self.thread.join(timeout=2)  # let it drain the last buffered line
            self.thread = None  # ready for the next case
        out = {}  # field -> (mean, max)
        for key in self.RX:  # fixed field order
            vals = [s[key] for s in self.samples if key in s]  # samples carrying this field
            if vals:  # a field never seen is simply absent -> "-" in the table
                out[key] = (statistics.mean(vals), max(vals))  # max shows throttle peaks
        return out  # empty dict when tegrastats was absent


# --------------------------------------------------------------------------
# Statistics and table rendering
# --------------------------------------------------------------------------

def summarize(values):  # values: one per measured run, never empty (--runs >= 1)
    """median / p90 / min / max of a list. p90 uses the nearest-rank method.

    Median rather than mean as the headline: a single slow run (a GC pause, a
    thermal step) should not move the reported figure. p90 is shown beside it
    so that a bad tail is still visible instead of being averaged away.
    """
    vs = sorted(values)  # copy; caller list untouched
    p90 = vs[min(len(vs) - 1, int(round(0.9 * (len(vs) - 1))))]  # nearest rank
    return {"med": statistics.median(vs), "p90": p90, "min": vs[0], "max": vs[-1]}


def render_table(headers, rows):  # rows: lists, same length as headers
    """Plain ASCII box table: every cell right-aligned to its column width.

    ASCII only (no box-drawing characters) so it pastes cleanly into a report,
    a terminal over ssh, or a log file without an encoding surprise. The row
    under the header uses "=" so the header stands out without colour.
    """
    # Column width = widest of the header and every cell in that column.
    widths = [max([len(h)] + [len(str(r[i])) for r in rows])
              for i, h in enumerate(headers)]
    sep = "+" + "+".join("-" * (w + 2) for w in widths) + "+"  # 1 space each side

    def fmt(cells):  # closure over widths
        """One table row, cells right-aligned so numbers line up by digit."""
        return "|" + "|".join(" " + str(c).rjust(w) + " "
                              for c, w in zip(cells, widths)) + "|"
    return "\n".join([sep, fmt(headers), sep.replace("-", "=")]
                     + [fmt(r) for r in rows] + [sep])


def num(x, nd=1):  # nd: decimals
    """Format a number to `nd` decimals, or "-" when the value is absent."""
    return "-" if x is None else "%.*f" % (nd, x)  # %.*f takes precision first


def med_p90(stat, nd=1):  # stat from summarize()
    """ "median (p90)" -- the cell format used throughout the performance table."""
    return "%s (%s)" % (num(stat["med"], nd), num(stat["p90"], nd))


# --------------------------------------------------------------------------
# Concurrency probe
# --------------------------------------------------------------------------

def concurrency_probe(base, prompt, n_gen):  # returns [result1, result2]
    """Fire two identical requests at once and report each one's measurements.

    With --parallel 1 the second request waits for the first to finish. The
    gap between the two is the queueing cost the gateway must plan for
    (11 S11A.3.4: take the GPU token BEFORE the HTTP call, because the server
    queues instead of refusing). An error in one request is recorded in its
    slot instead of raised, so the other request's result is not lost.
    """
    results = [None, None]  # index = request number - 1

    def worker(i):  # closure over results
        """Thread body: one request, result or error stored in its own slot."""
        try:
            results[i] = run_once(base, prompt, n_gen)
        except (urllib.error.URLError, RuntimeError, OSError) as e:
            results[i] = {"error": str(e)}
    threads = [threading.Thread(target=worker, args=(i,)) for i in range(2)]  # two clients
    for t in threads:
        t.start()  # started back to back, so both arrive within ~1 ms
    for t in threads:
        t.join()   # no timeout: run_once already bounds itself at 300 s
    return results  # order = thread index, not finish order


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def parse_args():  # all flags optional
    """Command line; defaults reproduce the run whose table is in the report."""
    ap = argparse.ArgumentParser(description="Benchmark the on-box llama-server")
    ap.add_argument("--url", default=DEFAULT_URL, help="server base URL")
    ap.add_argument("--runs", type=int, default=5, help="measured runs per case")
    ap.add_argument("--warmup", type=int, default=1, help="discarded runs per case")
    ap.add_argument("--cases", default=",".join(CASES),
                    help="comma-separated subset of: " + ",".join(CASES))
    ap.add_argument("--no-concurrency", action="store_true",
                    help="skip the two-request queueing probe")
    ap.add_argument("--no-tegrastats", action="store_true",
                    help="skip board telemetry sampling")
    ap.add_argument("--json-out", help="also write raw results to this file")
    return ap.parse_args()


def print_tables(info, names, report, runs):  # runs: for the header line only
    """Print the final result tables -- the last thing on screen before exit.

    Three tables: performance (always), telemetry (unless disabled) and the
    concurrency probe (unless disabled). Everything printed here is derived
    from `report`, so the --json-out file carries the same evidence.
    """
    print("\n" + "=" * 30 + " RESULTS " + "=" * 30)
    print("server: model=%s build=%s slots=%s n_ctx=%s"
          % (info["model"], info["build"], info["slots"], info["n_ctx"]))
    print("cells are MEDIAN (p90) over %d runs; prefill/gen rates are server "
          "timings, TTFT/E2E are client wall time" % runs)

    # Performance: one row per case. prompt_n / gen_n are taken from the first
    # run because they are identical across runs by construction (see doc).
    rows = []  # performance rows
    for name in names:  # cases run in the order given
        rr = report["cases"][name]["runs"]  # per-run dicts
        s = {k: summarize([r[k] for r in rr])
             for k in ("ttft_ms", "e2e_ms", "prompt_tps", "gen_tps", "gen_ms_per_tok")}
        rows.append([name, rr[0]["prompt_n"], rr[0]["gen_n"],
                     med_p90(s["ttft_ms"], 0), med_p90(s["prompt_tps"]),
                     med_p90(s["gen_tps"]), num(s["gen_ms_per_tok"]["med"]),
                     med_p90(s["e2e_ms"], 0),
                     "%s-%s" % (num(s["gen_tps"]["min"]), num(s["gen_tps"]["max"]))])
    print("\nPerformance")  # table 1
    print(render_table(["case", "prompt_n", "gen_n", "TTFT ms", "prefill tok/s",
                        "gen tok/s", "ms/tok", "E2E ms", "gen min-max"], rows))

    # Telemetry: "mean / max" per field over the measured runs of each case.
    if any(report["cases"][n]["tegrastats"] for n in names):
        rows = []  # telemetry rows
        for name in names:
            t = report["cases"][name]["tegrastats"]  # field -> (mean, max)

            def mm(key, nd=0):
                """ "mean / max" for one field, or "-" if it was never sampled."""
                return "-" if key not in t else "%s / %s" % (num(t[key][0], nd),
                                                             num(t[key][1], nd))
            rows.append([name, mm("gpu_pct"), mm("ram_mb"), mm("cpu_c", 1),
                         mm("gpu_c", 1), mm("tj_c", 1), mm("vin_mw")])
        print("\nBoard telemetry during measured runs (mean / max)")  # table 2
        print(render_table(["case", "GPU %", "RAM MB", "CPU C", "GPU C", "TJ C",
                            "VDD_IN mW"], rows))
        # A rate measured at the throttle point is not the board's rate: say
        # so next to the table instead of leaving the reader to spot it.
        trip = passive_trip_c()  # None off-Jetson
        hot = max([v[1] for n in names for k, v in report["cases"][n]["tegrastats"].items()
                   if k.endswith("_c")] or [0.0])  # hottest max over all cases
        if trip is not None and hot >= trip - THERMAL_MARGIN_C:
            print("WARNING: max temperature %.1f C is within %d C of the passive "
                  "trip %.1f C -- rates above may include thermal throttling"
                  % (hot, THERMAL_MARGIN_C, trip))

    # Concurrency: one row per request, then the queueing gap between them.
    if report["concurrency"] is not None:
        rows, lat = [], []  # concurrency rows, successful E2E values
        for i, r in enumerate(report["concurrency"]):
            if "error" in r:
                rows.append(["req %d" % (i + 1), "error", r["error"][:40], "-"])
            else:
                lat.append(r["e2e_ms"])  # for the queueing gap
                rows.append(["req %d" % (i + 1), num(r["ttft_ms"], 0),
                             num(r["e2e_ms"], 0), num(r["gen_tps"])])
        print("\nConcurrency (2 simultaneous short requests)")  # table 3
        print(render_table(["request", "TTFT ms", "E2E ms", "gen tok/s"], rows))
        if len(lat) == 2:  # only meaningful when both requests succeeded
            print("queueing cost of the later request: ~%.0f ms "
                  "(server queues, does not refuse -- 11 S11A.3.4)" % abs(lat[1] - lat[0]))


def main():  # exit code: 0 ok, 1 no server, 2 usage
    """Run every selected case, then the probe, then print the tables."""
    args = parse_args()  # exits itself on --help
    base = args.url.rstrip("/")  # tolerate a trailing slash on --url
    names = [c.strip() for c in args.cases.split(",") if c.strip()]  # tolerate spaces
    unknown = [n for n in names if n not in CASES]  # typos
    if unknown or args.runs < 1:
        # Usage error (exit 2, argparse's convention), checked before any
        # request so a typo does not cost a minute of warmup first.
        print("bad --cases %s or --runs %d" % (unknown, args.runs), file=sys.stderr)
        return 2

    # Fail early with a clear cause if the service is not up -- the most common
    # mistake is forgetting to start llm_server.sh.
    try:
        info = server_info(base)  # also proves the model is loaded
    except (urllib.error.URLError, OSError, ValueError) as e:
        print("cannot reach llama-server at %s: %s" % (base, e), file=sys.stderr)
        print("start it first: bash services/llm/llm_server.sh", file=sys.stderr)
        return 1

    print("llama-server benchmark")
    print("  url=%s model=%s build=%s slots=%s n_ctx=%s"
          % (base, info["model"], info["build"], info["slots"], info["n_ctx"]))
    print("  runs=%d warmup=%d cases=%s" % (args.runs, args.warmup, ",".join(names)))

    sampler = None if args.no_tegrastats else TegraSampler()
    report = {"server": info, "cases": {}, "concurrency": None}  # == --json-out
    t_start = time.monotonic()  # for the total time line

    for name in names:  # cases run in the order given
        p_tok, n_gen = CASES[name]  # targets for this case
        prompt = build_prompt(base, p_tok)  # outside the timed window
        print("\n[%s] prompt~%d tok, generate %d tok" % (name, p_tok, n_gen))
        for i in range(args.warmup):
            run_once(base, prompt, n_gen)  # result discarded on purpose
            print("  warmup %d done" % (i + 1))
        if sampler:
            sampler.start()  # telemetry window = measured runs only
        runs = []  # measured runs of this case
        for i in range(args.runs):
            r = run_once(base, prompt, n_gen)
            runs.append(r)  # kept raw for --json-out
            # Per-run progress line, so a stall or a drifting rate is visible
            # live instead of only in the final median.
            print("  run %d: ttft=%.0fms prefill=%.0ftok/s gen=%.1ftok/s e2e=%.0fms"
                  % (i + 1, r["ttft_ms"], r["prompt_tps"], r["gen_tps"], r["e2e_ms"]))
        report["cases"][name] = {"runs": runs,
                                 "tegrastats": sampler.stop() if sampler else {}}

    if not args.no_concurrency:
        # Always the short case: the probe measures QUEUEING, and a short
        # request keeps the probe cheap while the wait stays clearly visible.
        print("\n[concurrency] 2 simultaneous 'short' requests (server slots=%s)"
              % info["slots"])
        p_tok, n_gen = CASES["short"]  # same size as the short case
        report["concurrency"] = concurrency_probe(base, build_prompt(base, p_tok), n_gen)

    print_tables(info, names, report, args.runs)  # last output before exit
    print("\ntotal benchmark time: %.1f s" % (time.monotonic() - t_start))

    if args.json_out:
        # Raw per-run data, so a later report can recompute any statistic.
        with open(args.json_out, "w", encoding="utf-8") as fh:  # UTF-8: may hold Chinese
            json.dump(report, fh, ensure_ascii=False, indent=2)
        print("raw results written to %s" % args.json_out)
    return 0  # success


if __name__ == "__main__":
    sys.exit(main())
