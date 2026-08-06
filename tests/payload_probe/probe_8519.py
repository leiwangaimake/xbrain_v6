#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probe_8519.py -- GZH-2 payload . TCP 8519 audio-protocol probe (Step-1 bring-up).

Resolves the three blocking unknowns before wiring up the xbrain_v6 voice loop
(ref: docs/GZH-2_控制协议开发文档.md):

  (b) On-wire encoding of the 8519 command number:
        bracket  ->  literal ASCII  b"[40]"     (default; strong evidence from
                     the [89]/[99] echo replies and "前4位为[40]" in the doc)
        dec1     ->  one decimal byte  0x28
        ascii    ->  ASCII digits  b"40"
      Detected empirically with the harmless [89] "get SN" query.   -> `encoding`, `info`

  (a) Whether firmware allows realtime-hail [42] and record [40] CONCURRENTLY
      (this decides if full-duplex 功能2 is even possible).           -> `duplex`

  (c) Record-uplink first-frame latency + jitter over the SIYI RF link. -> `record`

It also exercises the 功能1 I/O paths directly:
      record  -> mic in  (Opus 16k mono)      tts / play -> speaker out.

Target: NVIDIA Jetson Orin (aarch64). Pure stdlib except OPTIONAL Opus:
      pip install opuslib   &&   sudo apt-get install -y libopus0
Without opuslib every protocol test still runs on raw bytes; only Opus
encode (audible tone / hail) and decode (record -> WAV) are skipped.

Offline dev: run `python3 probe_8519.py mock` in one shell (a fake device),
then point the other sub-commands at 127.0.0.1.

Examples:
  python3 probe_8519.py info
  python3 probe_8519.py encoding
  python3 probe_8519.py record --seconds 5 --wav /tmp/mic.wav
  python3 probe_8519.py tts --text "系统自检完成" --v2 --voice 1
  python3 probe_8519.py play --tone --secs 3
  python3 probe_8519.py duplex --seconds 8
"""

import argparse
import math
import socket
import sys
import threading
import time
import wave
from array import array

try:
    import opuslib  # type: ignore
    HAVE_OPUS = True
except Exception:
    HAVE_OPUS = False

# ---- protocol constants ------------------------------------------------------
DEFAULT_HOST = "192.168.144.38"   # doc §2 (real IP is site-specific)
DEFAULT_PORT = 8519

REC_FS, REC_FRAME = 16000, 320    # record uplink : 16k mono, 20ms frame
HAIL_FS, HAIL_FRAME = 8000, 480   # hail downlink : 8k  mono, 60ms frame

CMD_VOLUME = 14
CMD_HAIL_MOBILE, CMD_HAIL_STOP_MOBILE = 10, 11
CMD_HAIL_PC = 42
CMD_TTS, CMD_TTS_LOOP, CMD_TTS_STOP = 15, 16, 17
CMD_TTS_V2, CMD_TTS_V2_LOOP = 31, 32
CMD_REC_START, CMD_REC_STOP = 40, 41
CMD_GET_SN, CMD_GET_UID = 89, 90
CMD_REPORT = 99

ENCODINGS = ("bracket", "dec1", "ascii")


# ---- framing -----------------------------------------------------------------
def cmd_prefix(cmd, enc):
    """On-wire bytes for a command number under the chosen encoding."""
    if enc == "bracket":
        return b"[%d]" % cmd
    if enc == "dec1":
        return bytes([cmd & 0xFF])
    if enc == "ascii":
        return b"%d" % cmd
    raise ValueError("unknown encoding: %r" % enc)


def frame(cmd, payload=b"", enc="bracket"):
    return cmd_prefix(cmd, enc) + payload


def split_by_marker(buf, marker):
    """Split a captured uplink stream on repeated command-prefix markers.
    Heuristic: Opus payloads can coincidentally contain the marker bytes."""
    out = []
    i = buf.find(marker)
    while i >= 0:
        j = buf.find(marker, i + len(marker))
        out.append(bytes(buf[i + len(marker): (j if j >= 0 else len(buf))]))
        i = j
    return out


# ---- opus / pcm helpers ------------------------------------------------------
def _opus_app():
    return getattr(opuslib, "APPLICATION_AUDIO", 2049)


def encode_opus_stream(pcm, fs, frame_size):
    """int16-LE mono PCM -> list of Opus packets (last frame zero-padded)."""
    enc = opuslib.Encoder(fs, 1, _opus_app())
    step = frame_size * 2
    pkts = []
    for off in range(0, len(pcm), step):
        chunk = pcm[off:off + step]
        if len(chunk) < step:
            chunk = chunk + b"\x00" * (step - len(chunk))
        pkts.append(enc.encode(chunk, frame_size))
    return pkts


def decode_opus_packets(packets, fs):
    """list of Opus packets -> concatenated int16-LE PCM. Bad packets skipped."""
    dec = opuslib.Decoder(fs, 1)
    pcm = bytearray()
    ok = bad = 0
    for p in packets:
        if not p:
            continue
        try:
            pcm += dec.decode(p, 5760)   # 5760 = generous max samples/frame
            ok += 1
        except Exception:
            bad += 1
    return bytes(pcm), ok, bad


def tone_pcm(fs, seconds, f0=440.0, f1=None, amp=0.30):
    """Mono int16 sine (or f0->f1 sweep) -- an unmistakable audible test signal."""
    n = int(fs * seconds)
    f1 = f0 if f1 is None else f1
    buf = array("h", bytes(2 * n))
    phase = 0.0
    peak = int(amp * 32767)
    for i in range(n):
        f = f0 + (f1 - f0) * (i / n)
        phase += 2.0 * math.pi * f / fs
        buf[i] = int(peak * math.sin(phase))
    return buf.tobytes()


def resample_linear(pcm, sr_in, sr_out):
    if sr_in == sr_out:
        return pcm
    src = array("h")
    src.frombytes(pcm)
    n_in = len(src)
    if n_in == 0:
        return pcm
    n_out = max(1, int(n_in * sr_out / sr_in))
    out = array("h", bytes(2 * n_out))
    for i in range(n_out):
        x = i * sr_in / sr_out
        i0 = int(x)
        i1 = min(i0 + 1, n_in - 1)
        frac = x - i0
        out[i] = int(src[i0] * (1.0 - frac) + src[i1] * frac)
    return out.tobytes()


def read_wav_mono(path, target_fs):
    w = wave.open(path, "rb")
    ch, sw, fs, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
    raw = w.readframes(n)
    w.close()
    if sw != 2:
        raise SystemExit("WAV must be 16-bit PCM (got sampwidth=%d)" % sw)
    a = array("h")
    a.frombytes(raw)
    if ch > 1:
        mono = array("h", bytes(2 * (len(a) // ch)))
        for i in range(len(mono)):
            s = 0
            for c in range(ch):
                s += a[i * ch + c]
            mono[i] = int(s / ch)
        a = mono
    return resample_linear(a.tobytes(), fs, target_fs)


def write_wav(path, pcm, fs):
    w = wave.open(path, "wb")
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(fs)
    w.writeframes(pcm)
    w.close()


# ---- net helpers -------------------------------------------------------------
def connect(host, port, timeout):
    s = socket.create_connection((host, port), timeout=timeout)
    s.settimeout(timeout)
    return s


def drain(s, idle=1.0, maxbytes=1 << 20):
    """Read a request/reply burst: return once the peer is idle for `idle`s.
    Only safe when the reply is followed by a gap > idle. For a stream that
    keeps arriving (e.g. the 2s [99] auto-report), use capture_for()."""
    s.settimeout(idle)
    buf = bytearray()
    while len(buf) < maxbytes:
        try:
            d = s.recv(4096)
        except socket.timeout:
            break
        if not d:
            break
        buf += d
    return bytes(buf)


def capture_for(s, seconds, maxbytes=1 << 20):
    """Read for a fixed wall-clock window regardless of ongoing traffic."""
    end = time.monotonic() + seconds
    s.settimeout(0.3)
    buf = bytearray()
    while time.monotonic() < end and len(buf) < maxbytes:
        try:
            d = s.recv(4096)
        except socket.timeout:
            continue
        if not d:
            break
        buf += d
    return bytes(buf)


def show(label, data):
    txt = ""
    try:
        txt = "  " + data.decode("utf-8")
    except Exception:
        pass
    print("  %-14s %d bytes  %s%s"
          % (label, len(data), data[:48].hex(" "), txt if len(txt) < 90 else ""))


def need_opus(what):
    if not HAVE_OPUS:
        raise SystemExit(
            "%s needs Opus. Install:  pip install opuslib && "
            "sudo apt-get install -y libopus0" % what)


# ---- sub-commands ------------------------------------------------------------
def cmd_selftest(args):
    print("== selftest (no network) ==")
    for enc in ENCODINGS:
        print("  frame(40) %-8s -> %s" % (enc, frame(CMD_REC_START, enc=enc).hex(" ")))
    print("  opuslib available:", HAVE_OPUS)
    if HAVE_OPUS:
        pcm = tone_pcm(HAIL_FS, 0.30, 300, 3000)
        pkts = encode_opus_stream(pcm, HAIL_FS, HAIL_FRAME)
        back, ok, bad = decode_opus_packets(pkts, HAIL_FS)   # decode at same fs
        print("  opus roundtrip: %d pkts, decoded %d ok / %d bad, %d PCM bytes"
              % (len(pkts), ok, bad, len(back)))
    print("OK")


def _query(host, port, timeout, cmd, enc):
    s = connect(host, port, timeout)
    try:
        s.sendall(frame(cmd, enc=enc))
        return drain(s, 1.2)
    finally:
        s.close()


def cmd_encoding(args):
    print("== encoding autodetect ==  %s:%d" % (args.host, args.port))
    winner = None

    print("  probe 1 — [89] get-SN (silent; NOTE: unsupported on some 三合一 units):")
    for enc in ENCODINGS:
        try:
            resp = _query(args.host, args.port, args.timeout, CMD_GET_SN, enc)
        except Exception as e:
            print("    %-8s ERROR %s" % (enc, e))
            continue
        echoed = resp.startswith(cmd_prefix(CMD_GET_SN, enc))
        show(enc, resp)
        print("             -> %s" % ("no reply" if not resp else
              ("REPLY, prefix echoed" if echoed else "REPLY")))
        if echoed:
            winner = enc
        elif resp and winner is None:
            winner = enc

    if winner is None:
        print("  probe 2 — [40] record uplink (opens mic ~0.8s, NO sound emitted):")
        for enc in ENCODINGS:
            try:
                s = connect(args.host, args.port, args.timeout)
                s.sendall(frame(CMD_REC_START, enc=enc))
                resp = capture_for(s, 0.8)
                s.sendall(frame(CMD_REC_STOP, enc=enc))
                s.close()
            except Exception as e:
                print("    %-8s ERROR %s" % (enc, e))
                continue
            echoed = cmd_prefix(CMD_REC_START, enc) in resp
            print("    %-8s %d bytes uplink  %s" % (enc, len(resp),
                  "([40] prefix seen)" if echoed else ("(data, no prefix)" if resp else "(silent)")))
            if echoed:
                winner = enc
            elif resp and winner is None:
                winner = enc

    print("\n  recommended --enc = %s"
          % (winner or "??? (no response to [89] or [40] — check cabling / IP / port)"))
    return winner


def cmd_info(args):
    print("== info ==  %s:%d  enc=%s" % (args.host, args.port, args.enc))
    s = connect(args.host, args.port, args.timeout)
    try:
        s.sendall(frame(CMD_GET_SN, enc=args.enc))
        show("SN [89]", drain(s, 1.0))
        s.sendall(frame(CMD_GET_UID, enc=args.enc))
        show("UID [90]", drain(s, 1.0))
        print("  listening %.0fs for [99] auto-report ..." % args.listen)
        rep = capture_for(s, args.listen)
        if cmd_prefix(CMD_REPORT, args.enc) in rep or b"volume_real" in rep:
            show("report [99]", rep)
            print("  -> device sends the 2s auto-report (standalone / 四合一 behaviour).")
        else:
            print("  -> no [99] auto-report seen (expected for 三合一, or none in window).")
    finally:
        s.close()


def cmd_record(args):
    print("== record ==  [40] start, capture %.1fs" % args.seconds)
    marker = cmd_prefix(CMD_REC_START, args.enc)
    s = connect(args.host, args.port, args.timeout)
    buf = bytearray()
    first = None
    stamps = []
    try:
        s.sendall(frame(CMD_REC_START, enc=args.enc))
        t0 = time.monotonic()
        s.settimeout(0.5)
        while time.monotonic() - t0 < args.seconds:
            try:
                d = s.recv(8192)
            except socket.timeout:
                continue
            if not d:
                print("  ! peer closed the connection")
                break
            if first is None:
                first = time.monotonic() - t0
            stamps.append(time.monotonic())
            buf += d
    finally:
        try:
            s.sendall(frame(CMD_REC_STOP, enc=args.enc))
        except Exception:
            pass
        s.close()

    total = len(buf)
    dur = args.seconds
    print("  captured        %d bytes  (~%.1f kbit/s payload)" % (total, total * 8 / dur / 1000))
    print("  first uplink    %s" % ("%.0f ms" % (first * 1000) if first else "NONE — no data"))
    if len(stamps) > 2:
        gaps = [(stamps[i] - stamps[i - 1]) * 1000 for i in range(1, len(stamps))]
        print("  recv chunks     %d   gap min/avg/max = %.1f / %.1f / %.1f ms"
              % (len(stamps), min(gaps), sum(gaps) / len(gaps), max(gaps)))
    pkts = split_by_marker(buf, marker)
    print("  [40] markers    %d   (=> uplink is prefixed per frame)" % len(pkts))
    if not total:
        print("  -> wrong --enc, or record not supported on this variant. Try `encoding`.")
        return
    if args.wav:
        if not HAVE_OPUS:
            raw = args.wav + ".raw"
            with open(raw, "wb") as f:
                f.write(buf)
            print("  opuslib missing -> saved raw stream to %s (decode offline)" % raw)
        else:
            src = pkts if len(pkts) > 1 else [bytes(buf)]
            pcm, ok, bad = decode_opus_packets(src, REC_FS)
            write_wav(args.wav, pcm, REC_FS)
            print("  decoded %d ok / %d bad -> %s  (%.2fs @ %dHz)"
                  % (ok, bad, args.wav, len(pcm) / 2 / REC_FS, REC_FS))


def _hail_packets(args):
    if args.wav:
        pcm = read_wav_mono(args.wav, HAIL_FS)
    else:
        pcm = tone_pcm(HAIL_FS, args.secs, 300, 3000)
    return encode_opus_stream(pcm, HAIL_FS, HAIL_FRAME)


def cmd_play(args):
    need_opus("play")
    pkts = _hail_packets(args)
    dur = HAIL_FRAME / HAIL_FS
    print("== play ==  cmd [%d], %d Opus frames (%.1fs), paced %.0fms"
          % (args.cmd, len(pkts), len(pkts) * dur, dur * 1000))
    s = connect(args.host, args.port, args.timeout)
    try:
        start = time.monotonic()
        for i, pk in enumerate(pkts):
            s.sendall(frame(args.cmd, pk, enc=args.enc))
            nxt = start + (i + 1) * dur
            time.sleep(max(0, nxt - time.monotonic()))
        stop = CMD_HAIL_STOP_MOBILE
        s.sendall(frame(stop, enc=args.enc))
        print("  sent stop [%d]. Listen at the device speaker." % stop)
    finally:
        s.close()


def cmd_tts(args):
    print("== tts ==  %s" % ("V2 voice=%d" % args.voice if args.v2 else "V1"))
    if args.v2:
        cmd, payload = CMD_TTS_V2, bytes([args.voice & 0xFF]) + args.text.encode("utf-8")
    else:
        cmd, payload = CMD_TTS, args.text.encode("utf-8")
    s = connect(args.host, args.port, args.timeout)
    try:
        s.sendall(frame(cmd, payload, enc=args.enc))
        print("  sent [%d] %r — device should speak it now." % (cmd, args.text))
        show("reply", drain(s, 1.0))
    finally:
        s.close()


def cmd_raw(args):
    data = bytes.fromhex(args.hex.replace(" ", ""))
    print("== raw ==  send %s" % data.hex(" "))
    s = connect(args.host, args.port, args.timeout)
    try:
        s.sendall(data)
        show("reply", drain(s, args.listen))
    finally:
        s.close()


def cmd_duplex(args):
    """THE money test: does [40] uplink survive while [42] downlink is active?"""
    need_opus("duplex (needs Opus to synthesise the hail downlink)")
    win = args.seconds / 4.0
    print("== duplex ==  concurrency test over %.0fs" % args.seconds)
    print("  phases: [A] record only | [B] record+hail | [C] record only | stop")

    s = connect(args.host, args.port, args.timeout)
    bins = {}          # int(second-bucket relative) -> bytes
    lock = threading.Lock()
    stop = threading.Event()
    dropped = threading.Event()
    t0 = time.monotonic()

    def reader():
        s.settimeout(0.3)
        while not stop.is_set():
            try:
                d = s.recv(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            if not d:
                dropped.set()
                break
            b = int((time.monotonic() - t0) / win)
            with lock:
                bins[b] = bins.get(b, 0) + len(d)

    rt = threading.Thread(target=reader, daemon=True)

    hail = _hail_packets(argparse.Namespace(wav=None, secs=win))
    dur = HAIL_FRAME / HAIL_FS

    try:
        s.sendall(frame(CMD_REC_START, enc=args.enc))   # phase A
        rt.start()
        time.sleep(win)

        # phase B -- stream hail while still reading uplink
        hstart = time.monotonic()
        i = 0
        while time.monotonic() - hstart < win and not dropped.is_set():
            s.sendall(frame(CMD_HAIL_PC, hail[i % len(hail)], enc=args.enc))
            i += 1
            nxt = hstart + i * dur
            time.sleep(max(0, nxt - time.monotonic()))
        s.sendall(frame(CMD_HAIL_STOP_MOBILE, enc=args.enc))

        time.sleep(win)                                  # phase C
    finally:
        try:
            s.sendall(frame(CMD_REC_STOP, enc=args.enc))
        except Exception:
            pass
        stop.set()
        rt.join(timeout=1.0)
        s.close()

    def rate(phase_idx):
        with lock:
            return bins.get(phase_idx, 0) / win
    a, b, c = rate(0), rate(1), rate(2)
    print("  uplink bytes/s   A=%.0f   B=%.0f   C=%.0f" % (a, b, c))

    print("\n  VERDICT:")
    if dropped.is_set():
        print("  ✗ device DROPPED the connection under concurrency -> not full-duplex on one socket.")
        print("    Retry with --two-sockets, or treat 功能2 as half-duplex (PTT).")
    elif a == 0:
        print("  ? no baseline uplink in phase A — fix --enc / record support first (`encoding`).")
    elif b >= 0.6 * a:
        print("  ✓ uplink CONTINUED during hail (B≈A) -> CONCURRENT OK, full-duplex 功能2 feasible.")
        print("    Next: build the two AEC paths (office-local easy; robot-side network echo hard).")
    elif b <= 0.2 * a:
        print("  ✗ uplink STOPPED during hail -> firmware is half-duplex (hail XOR record).")
        print("    Full-duplex 功能2 NOT possible; use PTT. AI 功能1 must gate record around TTS.")
    else:
        print("  ~ uplink DROPPED but did not stop (B between 0.2A and 0.6A) -> partial/degraded;")
        print("    inspect jitter, retest with --two-sockets before deciding.")


# ---- built-in mock device (offline dev only) --------------------------------
def cmd_mock(args):
    enc = args.enc
    print("== MOCK device ==  listening %s:%d  enc=%s  half_duplex=%s"
          % (args.bind, args.port, enc, args.half_duplex))
    if not HAVE_OPUS:
        print("  (opuslib missing: record uplink will be random bytes, not decodable audio)")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(4)

    def make_rec_frame():
        if HAVE_OPUS:
            pcm = tone_pcm(REC_FS, REC_FRAME / REC_FS, 600, 600)
            pk = encode_opus_stream(pcm, REC_FS, REC_FRAME)[0]
        else:
            pk = bytes((i * 7) & 0xFF for i in range(80))
        return cmd_prefix(CMD_REC_START, enc) + pk

    def handle(conn, addr):
        print("  + conn from %s" % (addr,))
        conn.settimeout(0.2)
        recording = threading.Event()
        last_downlink = [0.0]
        stop = threading.Event()

        def sender():
            period = REC_FRAME / REC_FS
            nxt = time.monotonic()
            last_report = time.monotonic()
            while not stop.is_set():
                now = time.monotonic()
                if (not args.no_autoreport) and now - last_report >= 2.0:
                    try:
                        conn.sendall(frame(
                            CMD_REPORT,
                            b'"{"volume_real": 58, "volume_limit": 100, "temperature": "23.56"}"',
                            enc=enc))
                    except OSError:
                        break
                    last_report = now
                if recording.is_set() and now >= nxt:
                    half_block = args.half_duplex and (now - last_downlink[0] < 0.3)
                    if not half_block:
                        try:
                            conn.sendall(make_rec_frame())
                        except OSError:
                            break
                    nxt += period
                time.sleep(0.005)

        st = threading.Thread(target=sender, daemon=True)
        st.start()
        buf = bytearray()
        tokens = {cmd_prefix(c, enc): c for c in (
            CMD_GET_SN, CMD_GET_UID, CMD_REC_START, CMD_REC_STOP,
            CMD_HAIL_PC, CMD_HAIL_MOBILE, CMD_HAIL_STOP_MOBILE,
            CMD_TTS, CMD_TTS_V2, CMD_VOLUME)}
        try:
            while not stop.is_set():
                try:
                    d = conn.recv(4096)
                except socket.timeout:
                    continue
                if not d:
                    break
                buf += d
                # consume recognised tokens; treat trailing bytes as payload
                progress = True
                while progress:
                    progress = False
                    for tok, c in tokens.items():
                        idx = buf.find(tok)
                        if idx != 0:
                            continue
                        progress = True
                        del buf[:len(tok)]
                        if c == CMD_GET_SN:
                            conn.sendall(frame(CMD_GET_SN, b'"172a783135658ac7"', enc=enc))
                        elif c == CMD_GET_UID:
                            conn.sendall(frame(CMD_GET_UID, b'"e3deacfd02b1b686"', enc=enc))
                        elif c == CMD_REC_START:
                            recording.set()
                        elif c == CMD_REC_STOP:
                            recording.clear()
                        elif c in (CMD_HAIL_PC, CMD_HAIL_MOBILE):
                            last_downlink[0] = time.monotonic()
                            buf.clear()          # drain the opus payload
                        elif c in (CMD_HAIL_STOP_MOBILE, CMD_TTS, CMD_TTS_V2, CMD_VOLUME):
                            buf.clear()
                        break
        finally:
            stop.set()
            conn.close()
            print("  - conn closed %s" % (addr,))

    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=handle, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("\n  mock stopped.")
    finally:
        srv.close()


# ---- CLI ---------------------------------------------------------------------
def build_parser():
    # connection args shared by every sub-command, so they can be given AFTER
    # the sub-command name (e.g. `mock --port 8520`, `record --host 1.2.3.4`).
    conn = argparse.ArgumentParser(add_help=False)
    conn.add_argument("--host", default=DEFAULT_HOST)
    conn.add_argument("--port", type=int, default=DEFAULT_PORT,
                      help="target port (client) / listen port (mock)")
    conn.add_argument("--enc", choices=ENCODINGS, default="bracket",
                      help="8519 command-number encoding (run `encoding` to confirm)")
    conn.add_argument("--timeout", type=float, default=5.0)

    p = argparse.ArgumentParser(
        description="GZH-2 TCP 8519 audio-protocol probe (Step-1 bring-up).",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("selftest", help="local checks, no network",
                   parents=[conn]).set_defaults(fn=cmd_selftest)
    sub.add_parser("encoding", help="auto-detect the command-number encoding",
                   parents=[conn]).set_defaults(fn=cmd_encoding)

    sp = sub.add_parser("info", help="SN / UID / auto-report probe", parents=[conn])
    sp.add_argument("--listen", type=float, default=5.0, help="seconds to watch for [99]")
    sp.set_defaults(fn=cmd_info)

    sp = sub.add_parser("record", help="[40] capture mic uplink -> stats / WAV", parents=[conn])
    sp.add_argument("--seconds", type=float, default=5.0)
    sp.add_argument("--wav", help="decode + save (needs opuslib)")
    sp.set_defaults(fn=cmd_record)

    sp = sub.add_parser("play", help="[42] stream Opus hail to device speaker", parents=[conn])
    sp.add_argument("--wav", help="source WAV (else a tone sweep)")
    sp.add_argument("--secs", type=float, default=3.0, help="tone length")
    sp.add_argument("--cmd", type=int, default=CMD_HAIL_PC, help="42 PC / 10 mobile")
    sp.set_defaults(fn=cmd_play)

    sp = sub.add_parser("tts", help="[15]/[31] speak text on the device", parents=[conn])
    sp.add_argument("--text", required=True)
    sp.add_argument("--v2", action="store_true", help="use [31] with voice gender")
    sp.add_argument("--voice", type=int, default=0, help="0 male / 1 female (V2)")
    sp.set_defaults(fn=cmd_tts)

    sp = sub.add_parser("duplex", help="concurrency test: [40]+[42] together", parents=[conn])
    sp.add_argument("--seconds", type=float, default=8.0)
    sp.set_defaults(fn=cmd_duplex)

    sp = sub.add_parser("raw", help="send arbitrary hex, print reply", parents=[conn])
    sp.add_argument("--hex", required=True)
    sp.add_argument("--listen", type=float, default=1.5)
    sp.set_defaults(fn=cmd_raw)

    sp = sub.add_parser("mock", help="run a fake 8519 device for offline dev", parents=[conn])
    sp.add_argument("--bind", default="127.0.0.1")
    sp.add_argument("--half-duplex", action="store_true",
                    help="simulate hail XOR record (uplink pauses during hail)")
    sp.add_argument("--no-autoreport", action="store_true")
    sp.set_defaults(fn=cmd_mock)
    return p


def main():
    args = build_parser().parse_args()
    try:
        args.fn(args)
    except KeyboardInterrupt:
        print("\ninterrupted.")
        sys.exit(130)
    except (ConnectionRefusedError, socket.timeout, OSError) as e:
        print("network error: %s" % e)
        print("  check: cable/switch, device IP (--host), port %d, and that the APP "
              "isn't already holding the socket." % args.port)
        sys.exit(1)


if __name__ == "__main__":
    main()
