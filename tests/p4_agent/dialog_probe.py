#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: dialog_probe.py
Brief: Live conversation transcript for the running P4-Agent voice loop

Description:
Why this exists. To judge the P4-Agent human-machine interaction -- feel,
fluency, correctness -- an operator speaks into the MIC and needs to SEE, in
one place and in real time, both halves of each turn: what ASR heard (plus the
intent P4 classified it as) and what text P4 decided to speak back (the string
that is handed to the GZH-2 device for synthesis). This tool is that single
window. It is a passive observer of the already-running stack started by
scripts/dev/start_voice_loop.sh; it drives nothing and holds no MIC handle.

The two halves come from two different sources because the pipeline puts them
in two different places:
  * REPLY (what goes to TTS) is PUBLISHED on the GEN plane, bare key
    cmd/audio/speak, payload {"schema":"p4_speak_v1","text":...} (16 S9.1 /
    intent_dispatch CMD_AUDIO_SPEAK). It is authoritative -- exactly the bytes
    the speaker path receives -- but it is NOT written to any log.
  * HEARD (what ASR produced) and the classified intent/kind are only in the
    p4_agent log line 'orch_turn: text=... kind=... intent=...'
    (orchestrator_turn.make_turn_handler). They are NOT published on the bus.
So a faithful transcript must merge a Zenoh subscription (reply) with a tail of
the p4_agent log (heard). Turns are half-duplex and strictly serial (one
utterance closes VAD before the next opens), so printing each event as it
arrives already reads as an ordered dialogue; no timestamp correlation needed.

What it does NOT do, and why:
  * It does NOT open the MIC or call ASR/LLM itself. The MIC is p2's exclusive
    ALSA handle (RT-A1, CLAUDE.md S0.1); a second opener would fail or steal it.
    This tool only reflects what the live stack already produced.
  * It does NOT add a bus publication for the heard text. That would be a new
    GEN key and thus a contract change (11 S1.1.6 cross-plane whitelist); out
    of scope for a dev probe. Tailing the log is the no-contract-change path.
  * It is NOT a pytest asset despite living under tests/. It needs the full
    live stack + a human talking, so it is named without a test_ prefix and is
    never collected. It follows tests/ai_runtime and tests/payload_probe, the
    established home for interactive AI/hardware harnesses.

Pitfalls this file was written around:
  * The reply payload does NOT carry the heard text, and the heard log line
    does NOT carry the reply text -- neither source alone is a transcript. Do
    not "simplify" this to one source; you lose half the dialogue.
  * orch_turn logs EVERY turn including kind=overheard (16 S5.2.1: not
    addressed to the robot -> publishes nothing). Such a turn shows a HEARD
    line with no following REPLY -- that is correct, not a dropped reply.
  * The reply-to-heard latency is a real fluency signal, so it is measured with
    the monotonic clock (CLK-C1), never a wall delta.

NOT for production: a scripts/dev-class inspector that happens to live in
tests/. Reads no config source; its only clock use is monotonic.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
import time

# GEN-plane router endpoint. The reply key cmd/audio/speak is a BARE key on the
# general plane (tcp/127.0.0.1:7447), same plane zenoh_echo.py calls 'gen'.
# Hard-coded like zenoh_echo does: a throwaway inspector stays dependency-free,
# and a moved port surfaces as an obvious 'connect refused' fixed in one line.
_DEFAULT_GEN_ENDPOINT = "tcp/127.0.0.1:7447"

# The one key that carries the spoken reply. Every speaking branch of the turn
# orchestrator (reply / confirm / denial / did-not-catch / dispatch ack) lands
# here (intent_dispatch: D/G/I/J prefixes -> CMD_AUDIO_SPEAK), so subscribing
# to this single key captures the whole audible side of the dialogue.
_DEFAULT_SPEAK_KEY = "cmd/audio/speak"

# The p4_agent stdout+stderr log start_voice_loop.sh redirects to. Python
# logging (basicConfig, format '%(asctime)s %(name)s %(levelname)s %(message)s')
# writes here, so this is where the orch_turn lines appear.
_DEFAULT_LOG = "/opt/xbrain_v6/logs/voice_loop/p4_agent.log"

# The heard+decision line, verbatim from orchestrator_turn.make_turn_handler:
#   _logger.info("orch_turn: text=%r kind=%s intent=%s", text, kind, intent)
# text is %r (repr), so it arrives quoted; _unrepr strips the one quote layer.
# Non-greedy up to ' kind=' so a heard utterance containing the word 'kind'
# cannot swallow the field boundary.
_RE_ORCH = re.compile(
    r"orch_turn: text=(?P<t>.+?) kind=(?P<kind>\S+) intent=(?P<intent>\S+)\s*$")

# ASR failed for this utterance (turn_loop._on_frame catches TurnLoopError):
#   _logger.warning("turn_loop asr error: %s", exc)
# Surfaced so a silent turn is explained (mic too quiet, service down) instead
# of the operator wondering why nothing happened.
_RE_ASR_ERR = re.compile(r"turn_loop asr error: (?P<msg>.+?)\s*$")


def _unrepr(s: str) -> str:
    """Strip one layer of matching outer quotes left by %r. Leaves an already
    bare string untouched. Kept deliberately dumb -- it is display text, not a
    value we parse further, so full ast.literal_eval would be over-engineering
    (and would choke on a stray backslash in a partial log line)."""
    s = s.strip()
    if len(s) >= 2 and s[0] == s[-1] and s[0] in ("'", '"'):
        return s[1:-1]
    return s


class _Transcript:
    """Serialises the two producer threads (Zenoh reply callback + log tail)
    onto one stdout, and measures reply latency on the monotonic clock.

    One lock: the Zenoh callback runs on Zenoh's Rust thread pool while the tail
    runs on our own thread, so both printers must not interleave a line. Only
    print + a scalar timestamp happen under the lock -- no await, no queue, no
    bus publish (CLAUDE.md 4.2), which is exactly what is allowed off that
    thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        # Monotonic instant of the most recent HEARD line still awaiting a
        # reply. None means 'no turn open', so a stray reply prints without a
        # bogus latency. CLK-C1: monotonic so a wall-clock step mid-dialogue
        # never turns a latency into a negative or a huge number.
        self._heard_mono: float | None = None

    def heard(self, text: str, kind: str, intent: str) -> None:
        """Print the operator side of a turn and open the latency window."""
        with self._lock:
            self._heard_mono = time.monotonic()
            print("[HEARD] %s   (intent=%s kind=%s)" % (text, intent, kind),
                  flush=True)

    def reply(self, text: str) -> None:
        """Print the robot side of a turn and close the latency window. The
        '+N ms' is HEARD->REPLY wall-felt latency, the headline fluency number
        an operator watches while talking."""
        with self._lock:
            gap = ""
            if self._heard_mono is not None:
                dt_ms = int((time.monotonic() - self._heard_mono) * 1000)
                gap = " (+%d ms)" % dt_ms
                self._heard_mono = None
            print("[REPLY]%s %s" % (gap, text), flush=True)

    def note(self, msg: str) -> None:
        """Meta line (tail started, asr error, rotation). Kept visually distinct
        so it never reads as either half of the dialogue."""
        with self._lock:
            print("[note ] %s" % msg, flush=True)


def _parse_log_line(line: str, tx: _Transcript) -> None:
    """Turn one p4_agent log line into at most one transcript event. Anything
    that matches neither pattern is ordinary log noise and is dropped -- the
    probe shows the dialogue, not the whole log (that is what tail -f is for)."""
    m = _RE_ORCH.search(line)
    if m is not None:
        tx.heard(_unrepr(m.group("t")), m.group("kind"), m.group("intent"))
        return
    m = _RE_ASR_ERR.search(line)
    if m is not None:
        tx.note("asr error: " + m.group("msg"))


def _tail_log(path: str, tx: _Transcript, stop_evt: threading.Event) -> None:
    """Follow the p4_agent log from its END and feed matching lines to tx.

    From the end, not the start: we want the turns happening NOW, not a replay
    of everything since the stack booted. Handles the one rotation case that
    actually occurs here -- start_voice_loop.sh truncating the same path on a
    restart -- by detecting an inode change or a file that shrank below our read
    cursor and reopening, so a mid-session restart does not leave the probe
    staring at a stale fd forever."""
    # The launcher may still be creating the file; wait for it rather than
    # erroring. Bounded only by stop_evt so Ctrl-C during the wait exits clean.
    while not stop_evt.is_set() and not os.path.exists(path):
        time.sleep(0.5)
    if stop_evt.is_set():
        return
    tx.note("tailing %s for HEARD/intent" % path)

    def _open():
        fh = open(path, "r", encoding="utf-8", errors="replace")
        fh.seek(0, os.SEEK_END)
        return fh, os.fstat(fh.fileno()).st_ino

    fh, inode = _open()
    try:
        while not stop_evt.is_set():
            line = fh.readline()
            if line:
                _parse_log_line(line, tx)
                continue
            # Caught up. Sleep briefly, then check for truncation/rotation.
            time.sleep(0.2)
            try:
                st = os.stat(path)
            except FileNotFoundError:
                # File went away (rare). Keep the old fd; a recreate is picked
                # up on the next successful stat via the inode check below.
                continue
            if st.st_ino != inode or st.st_size < fh.tell():
                # Restarted stack: reopen and follow the new file from its end.
                tx.note("p4_agent.log rotated/truncated; reattaching")
                fh.close()
                fh, inode = _open()
    finally:
        fh.close()


def _open_gen(endpoint: str):
    """Open a client-mode Zenoh session on the GEN router. Import zenoh lazily
    so --help works without zenoh-python installed. mode=client: attach as a
    leaf, never a peer -- no scouting, no gossip (same rationale as
    zenoh_echo.py); just receive forwarded publications."""
    import zenoh  # noqa: PLC0415 -- lazy so --help needs no zenoh install

    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["%s"]' % endpoint)
    return zenoh.open(conf)


def _make_speak_cb(tx: _Transcript):
    """Build the cmd/audio/speak subscriber callback. Pulls .text out of the
    p4_speak_v1 payload; on any decode/parse failure falls back to a bounded
    raw repr so a malformed frame is visible, not swallowed."""
    def _cb(sample) -> None:
        raw = bytes(sample.payload)
        try:
            obj = json.loads(raw.decode("utf-8"))
            text = obj.get("text", "")
            # A speak frame with no text field is a wiring bug worth seeing.
            if not text:
                text = "(empty text; raw=%s)" % raw[:120].decode("utf-8",
                                                                 "replace")
        except (ValueError, UnicodeDecodeError):
            text = raw[:200].decode("utf-8", "replace")
        tx.reply(text)
    return _cb


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="dialog_probe.py",
        description="Live P4-Agent conversation transcript: HEARD (from the "
                    "p4_agent log) merged with REPLY (from GEN cmd/audio/speak).")
    ap.add_argument("--gen-endpoint", default=_DEFAULT_GEN_ENDPOINT,
                    help="GEN router endpoint (default: %s)"
                         % _DEFAULT_GEN_ENDPOINT)
    ap.add_argument("--speak-key", default=_DEFAULT_SPEAK_KEY,
                    help="reply key to subscribe (default: %s)"
                         % _DEFAULT_SPEAK_KEY)
    ap.add_argument("--log", default=_DEFAULT_LOG,
                    help="p4_agent log to tail for HEARD/intent (default: %s)"
                         % _DEFAULT_LOG)
    ap.add_argument("--no-log", action="store_true",
                    help="do not tail the log; show REPLY only (bus-only mode)")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="auto-stop after N seconds; <=0 runs until Ctrl-C "
                         "(default: 0)")
    args = ap.parse_args(argv)

    tx = _Transcript()

    # Connect to the GEN plane first: without the reply half there is no point
    # tailing the log, and a down router is the most common failure -- say so
    # plainly rather than starting a tail that looks alive but shows nothing.
    try:
        session = _open_gen(args.gen_endpoint)
    except ImportError:
        print("zenoh-python not installed (pip install eclipse-zenoh)",
              file=sys.stderr)
        return 2
    except Exception as exc:      # noqa: BLE001 -- surface the connect failure
        print("cannot attach to GEN plane (%s): %s\n"
              "is zenohd-gen up? (start_voice_loop.sh brings it up on 7447)"
              % (args.gen_endpoint, exc), file=sys.stderr)
        return 1

    # Strong ref to the subscriber for the whole run: dropping it would let
    # Python GC silently unsubscribe on the Rust side (CLAUDE.md 4.3). It is
    # held by this local until session.close() in the finally below.
    sub = session.declare_subscriber(args.speak_key, _make_speak_cb(tx))

    stop_evt = threading.Event()
    tail_thread = None
    if not args.no_log:
        # daemon=True: the tail is pure I/O with no cleanup owed, so a hard exit
        # must never hang on it; stop_evt is the graceful path, daemon the
        # backstop.
        tail_thread = threading.Thread(
            target=_tail_log, args=(args.log, tx, stop_evt),
            name="dialog-probe-tail", daemon=True)
        tail_thread.start()

    print("dialog_probe attached: gen=%s key=%r log=%s"
          % (args.gen_endpoint, args.speak_key,
             "(off)" if args.no_log else args.log), file=sys.stderr)
    print("speak into the MIC; HEARD = what ASR got, REPLY = what goes to TTS. "
          "Ctrl-C to stop.", file=sys.stderr)

    # Monotonic deadline (CLK-C1): 0 or negative means run until Ctrl-C.
    deadline = None if args.seconds <= 0 else time.monotonic() + args.seconds
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Order: stop the tail, then drop the sub ref, then close the session.
        stop_evt.set()
        if tail_thread is not None:
            tail_thread.join(timeout=1.0)
        del sub
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
