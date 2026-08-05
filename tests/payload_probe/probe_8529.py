#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probe_8529.py — GZH-2 payload · TCP 8529 lights / 红蓝警示 / 抛投钩 probe.

Frame:   8D | len | MSG_ID | payload[len] | CRC8
         CRC8 = CRC-8/MAXIM (poly 0x31 reflected 0x8C, init 0x00) over len+MSG_ID+payload.
Controls are NO-REPLY. The device pushes a 0x25 status report ~every 500ms
(len 0x18 = 24 payload bytes) carrying 探照灯 (B7 on / B6 strobe / B0-B5 brightness)
and 红蓝警示灯 (0/1). The exact byte offsets inside that 24-byte payload are not
documented — `status` prints the raw payload and auto-locates the 探照灯 byte by
matching the value we just commanded, so we can pin the offset empirically.

MSG_IDs:  01 light on/off(0/1) · 02 brightness(0-30) · 03 strobe(0/1)
          04 throw-hook(0/1/2 then 0/1) · 07 red-blue mode(0x00-0x10, 0=off,1-16 patterns)

Target: NVIDIA Jetson Orin. Pure stdlib, no external deps.
Offline dev: `python3 probe_8529.py mock` then point commands at 127.0.0.1.

Examples:
  python3 probe_8529.py selftest                       # verify CRC vs known frames
  python3 probe_8529.py scene --host 192.168.144.38    # 常亮最亮 -> 爆闪 -> 红蓝 -> off
  python3 probe_8529.py light on  --host H              # searchlight on
  python3 probe_8529.py bright --level 30 --host H
  python3 probe_8529.py strobe on --host H
  python3 probe_8529.py redblue --mode 1 --host H
  python3 probe_8529.py status --host H --seconds 3     # watch 0x25 reports
  python3 probe_8529.py off --host H                    # everything off
"""

import argparse
import socket
import sys
import time

DEFAULT_HOST = "192.168.144.38"
DEFAULT_PORT = 8529
HEADER = 0x8D

MSG_LIGHT = 0x01      # payload 0/1
MSG_BRIGHT = 0x02     # payload 0-30
MSG_STROBE = 0x03     # payload 0/1
MSG_HOOK = 0x04       # payload sel(0/1/2) + on(0/1)
MSG_REDBLUE = 0x07    # payload 0x00-0x10  (0 off, 1-16 modes)
STATUS_ID = 0x25
STATUS_PLEN = 0x18    # 24 payload bytes
# 0x25 status payload offsets — verified on GZH-2 三合一 real HW:
LIGHT_OFF = 3        # payload[3] = 探照灯 byte (see light_byte: b7 on / b6 strobe / b0-5 bright)
REDBLUE_OFF = 4      # payload[4] = 红蓝 mode (0=off, 1-16)

BRIGHT_MAX = 30

# frames verified in the spec (docs/GZH-2_控制协议开发文档.md §7) — CRC ground truth
KNOWN_FRAMES = {
    "light on":     "8D 01 01 01 31",
    "light off":    "8D 01 01 00 6F",
    "bright 7":     "8D 01 02 07 B9",
    "strobe on":    "8D 01 03 01 A0",
    "redblue m2":   "8D 01 07 02 79",
    "bright 30":    "8D 01 02 1E B8",
    "hook all on":  "8D 02 04 00 01 C7",
    "redblue m16":  "8D 01 07 10 58",
}


# ---- CRC + framing -----------------------------------------------------------
def crc8_maxim(data):
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 1) else (crc >> 1)
    return crc & 0xFF


def build_frame(msg_id, payload):
    body = bytes([len(payload), msg_id]) + bytes(payload)
    return bytes([HEADER]) + body + bytes([crc8_maxim(body)])


def build_status_frame(payload):
    # The 0x25 report's CRC covers the PAYLOAD ONLY (verified on real HW),
    # not len+id+payload like control frames.
    p = bytes(payload)
    return bytes([HEADER, len(p), STATUS_ID]) + p + bytes([crc8_maxim(p)])


def light_byte(on, strobe, bright):
    v = (bright & 0x3F)
    if on:
        v |= 0x80
    if strobe:
        v |= 0x40
    return v


# ---- status parsing ----------------------------------------------------------
def parse_status_frames(buf):
    """Return list of (payload24, crc_ok) for every 0x25 report found."""
    out = []
    marker = bytes([HEADER, STATUS_PLEN, STATUS_ID])
    total = 1 + 1 + 1 + STATUS_PLEN + 1     # 28
    i = buf.find(marker)
    while i >= 0 and i + total <= len(buf):
        frame = buf[i:i + total]
        payload = frame[3:3 + STATUS_PLEN]
        # NOTE: the 0x25 report CRC covers the PAYLOAD ONLY (verified on real HW),
        # unlike control frames whose CRC covers len+id+payload.
        crc_ok = frame[-1] == crc8_maxim(payload)
        out.append((bytes(payload), crc_ok))
        i = buf.find(marker, i + total)
    return out


def report_status(buf, expect_light=None):
    frames = parse_status_frames(buf)
    if not frames:
        print("     (no 0x25 status report seen — this variant may not push status)")
        return
    payload, crc_ok = frames[-1]
    print("     status 0x25  crc=%s  payload=%s"
          % ("ok" if crc_ok else "BAD", payload.hex(" ")))
    # Confirmed on GZH-2 三合一 real HW: payload[3]=探照灯 byte, payload[4]=红蓝 mode.
    sl = payload[LIGHT_OFF]
    print("     -> 探照灯 payload[%d]=0x%02X (on=%d strobe=%d bright=%d/30)"
          % (LIGHT_OFF, sl, (sl >> 7) & 1, (sl >> 6) & 1, sl & 0x3F))
    print("     -> 红蓝  payload[%d]=0x%02X (mode, 0=off, 1-16)"
          % (REDBLUE_OFF, payload[REDBLUE_OFF]))
    if expect_light is not None and sl != expect_light:
        print("     -> note: 探照灯 byte 0x%02X != commanded 0x%02X "
              "(brightness memory retained when off is normal)" % (sl, expect_light))


# ---- net ---------------------------------------------------------------------
def connect(host, port, timeout):
    s = socket.create_connection((host, port), timeout=timeout)
    s.settimeout(timeout)
    return s


def capture_for(s, seconds, maxbytes=1 << 20):
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


def send_frames(args, frames, watch=0.0, expect_light=None):
    s = connect(args.host, args.port, args.timeout)
    try:
        for note, f in frames:
            s.sendall(f)
            print("  -> %-18s %s" % (note, f.hex(" ")))
        if watch > 0:
            report_status(capture_for(s, watch), expect_light)
    finally:
        s.close()


# ---- sub-commands ------------------------------------------------------------
def cmd_selftest(args):
    print("== selftest (CRC vs spec-verified frames) ==")
    ok = True
    specs = {
        "light on":    build_frame(MSG_LIGHT, [1]),
        "light off":   build_frame(MSG_LIGHT, [0]),
        "bright 7":    build_frame(MSG_BRIGHT, [7]),
        "strobe on":   build_frame(MSG_STROBE, [1]),
        "redblue m2":  build_frame(MSG_REDBLUE, [2]),
        "bright 30":   build_frame(MSG_BRIGHT, [30]),
        "hook all on": build_frame(MSG_HOOK, [0, 1]),
        "redblue m16": build_frame(MSG_REDBLUE, [16]),
    }
    for name, want_hex in KNOWN_FRAMES.items():
        got = specs[name]
        want = bytes.fromhex(want_hex.replace(" ", ""))
        good = got == want
        ok = ok and good
        print("  %-12s %s  %s" % (name, got.hex(" ").upper(),
                                   "OK" if good else "MISMATCH expected " + want_hex))
    print("ALL GOOD" if ok else "!! CRC/framing MISMATCH — do not send to hardware")
    if not ok:
        sys.exit(1)


def cmd_light(args):
    send_frames(args, [("light %s" % args.state,
                        build_frame(MSG_LIGHT, [1 if args.state == "on" else 0]))],
                watch=args.watch)


def cmd_bright(args):
    lvl = max(0, min(BRIGHT_MAX, args.level))
    if lvl != args.level:
        print("  (clamped to 0-%d)" % BRIGHT_MAX)
    send_frames(args, [("bright %d" % lvl, build_frame(MSG_BRIGHT, [lvl]))], watch=args.watch)


def cmd_strobe(args):
    send_frames(args, [("strobe %s" % args.state,
                        build_frame(MSG_STROBE, [1 if args.state == "on" else 0]))],
                watch=args.watch)


def cmd_redblue(args):
    m = max(0, min(16, args.mode))
    send_frames(args, [("redblue mode %d" % m, build_frame(MSG_REDBLUE, [m]))], watch=args.watch)


def cmd_flash(args):
    # Software strobe: the device's built-in 爆闪 (0x03) is a fixed firmware rate
    # with no speed parameter, so we drive the flashing ourselves for full control.
    hz = max(0.5, min(12.0, args.hz))
    if hz != args.hz:
        print("  (clamped hz to 0.5-12)")
    bright = max(1, min(BRIGHT_MAX, args.bright))
    half = 1.0 / (2.0 * hz)
    cycles = max(1, int(round(args.seconds * hz)))
    on_light = build_frame(MSG_LIGHT, [1])
    off_light = build_frame(MSG_LIGHT, [0])
    on_bright = build_frame(MSG_BRIGHT, [bright])
    off_bright = build_frame(MSG_BRIGHT, [0])
    print("== software flash ==  %.1f Hz  %.1fs (~%d cycles)  method=%s  bright=%d"
          % (hz, args.seconds, cycles, args.method, bright))
    s = connect(args.host, args.port, args.timeout)
    try:
        s.sendall(build_frame(MSG_STROBE, [0]))   # make sure hardware 爆闪 is off
        s.sendall(on_bright)
        s.sendall(on_light)
        time.sleep(0.15)
        for _ in range(cycles):
            s.sendall(off_bright if args.method == "bright" else off_light)
            time.sleep(half)
            s.sendall(on_bright if args.method == "bright" else on_light)
            time.sleep(half)
        if args.keep_on:
            s.sendall(on_bright)
            s.sendall(on_light)
            print("  done — searchlight left steady ON at bright %d." % bright)
        else:
            s.sendall(off_light)
            print("  done — searchlight off.")
    finally:
        s.close()


def cmd_off(args):
    send_frames(args, [
        ("light off", build_frame(MSG_LIGHT, [0])),
        ("strobe off", build_frame(MSG_STROBE, [0])),
        ("redblue off", build_frame(MSG_REDBLUE, [0])),
    ])
    print("  all lights off.")


def cmd_status(args):
    print("== status ==  listening %.1fs for 0x25 (500ms cadence)" % args.seconds)
    s = connect(args.host, args.port, args.timeout)
    try:
        buf = capture_for(s, args.seconds)
    finally:
        s.close()
    frames = parse_status_frames(buf)
    print("  %d report(s), %d bytes total" % (len(frames), len(buf)))
    for i, (p, crc_ok) in enumerate(frames[:6]):
        print("  #%d crc=%s payload=%s" % (i, "ok" if crc_ok else "BAD", p.hex(" ")))
    if not frames and buf:
        print("  raw (no 0x25 marker): %s" % buf[:40].hex(" "))


def cmd_scene(args):
    """The requested light test: 常亮最亮 -> 爆闪(像警灯) -> 红蓝警灯 -> off."""
    hold, watch = args.hold, args.watch
    print("== scene ==  device %s:%d   (each phase holds %.1fs)" % (args.host, args.port, hold))
    s = connect(args.host, args.port, args.timeout)

    def do(note, frame):
        s.sendall(frame)
        print("  -> %-18s %s" % (note, frame.hex(" ")))

    def watch_and_hold(expect_light=None, expect_note=""):
        if watch > 0:
            report_status(capture_for(s, watch), expect_light)
        remain = hold - watch
        if remain > 0:
            time.sleep(remain)

    try:
        print("\nPhase 1 — 补光灯 常亮最亮 (steady, brightness %d)" % BRIGHT_MAX)
        do("strobe off", build_frame(MSG_STROBE, [0]))
        do("light on", build_frame(MSG_LIGHT, [1]))
        do("bright %d" % BRIGHT_MAX, build_frame(MSG_BRIGHT, [BRIGHT_MAX]))
        watch_and_hold(expect_light=light_byte(1, 0, BRIGHT_MAX))   # 0x9E

        print("\nPhase 2 — 爆闪 like a police light (strobe on, brightness %d)" % BRIGHT_MAX)
        do("strobe on", build_frame(MSG_STROBE, [1]))
        watch_and_hold(expect_light=light_byte(1, 1, BRIGHT_MAX))   # 0xDE

        print("\nPhase 3 — 红蓝警灯 on (mode %d); searchlight off" % args.mode)
        do("strobe off", build_frame(MSG_STROBE, [0]))
        do("light off", build_frame(MSG_LIGHT, [0]))
        do("redblue mode %d" % args.mode, build_frame(MSG_REDBLUE, [args.mode]))
        watch_and_hold(expect_light=light_byte(0, 0, 0))            # 0x00

        if args.keep:
            print("\n(--keep: leaving 红蓝 mode %d ON; run `off` to stop)" % args.mode)
        else:
            print("\nPhase 4 — all off")
            do("light off", build_frame(MSG_LIGHT, [0]))
            do("strobe off", build_frame(MSG_STROBE, [0]))
            do("redblue off", build_frame(MSG_REDBLUE, [0]))
    finally:
        s.close()


def cmd_raw(args):
    data = bytes.fromhex(args.hex.replace(" ", ""))
    print("== raw ==  %s  (crc of body would be 0x%02X)"
          % (data.hex(" "), crc8_maxim(data[1:-1]) if len(data) >= 3 else 0))
    s = connect(args.host, args.port, args.timeout)
    try:
        s.sendall(data)
        if args.listen > 0:
            report_status(capture_for(s, args.listen))
    finally:
        s.close()


def cmd_mock(args):
    print("== MOCK 8529 ==  listening %s:%d" % (args.bind, args.port))
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(4)
    state = {"on": 0, "strobe": 0, "bright": 0, "redblue": 0}

    def status_frame():
        p = bytearray(STATUS_PLEN)
        p[1] = 0x05                                                        # const seen on real HW
        p[8] = p[16] = 0x7F                                                # const seen on real HW
        p[LIGHT_OFF] = light_byte(state["on"], state["strobe"], state["bright"])
        p[REDBLUE_OFF] = state["redblue"] & 0xFF
        return build_status_frame(p)

    def handle(conn, addr):
        print("  + conn %s" % (addr,))
        conn.settimeout(0.2)
        buf = bytearray()
        last = 0.0
        try:
            while True:
                now = time.monotonic()
                if now - last >= 0.5:
                    try:
                        conn.sendall(status_frame())
                    except OSError:
                        break
                    last = now
                try:
                    d = conn.recv(4096)
                except socket.timeout:
                    continue
                if not d:
                    break
                buf += d
                while len(buf) >= 4 and buf[0] == HEADER:
                    ln = buf[1]
                    total = 3 + ln + 1
                    if len(buf) < total:
                        break
                    mid, payload, crc = buf[2], buf[3:3 + ln], buf[total - 1]
                    if crc == crc8_maxim(buf[1:total - 1]):
                        if mid == MSG_LIGHT:
                            state["on"] = payload[0]
                        elif mid == MSG_BRIGHT:
                            state["bright"] = payload[0]
                        elif mid == MSG_STROBE:
                            state["strobe"] = payload[0]
                        elif mid == MSG_REDBLUE:
                            state["redblue"] = payload[0]
                        print("    rx id=0x%02X payload=%s -> %s" % (mid, payload.hex(), state))
                    del buf[:total]
                if buf and buf[0] != HEADER:      # resync
                    j = buf.find(bytes([HEADER]))
                    del buf[:(j if j >= 0 else len(buf))]
        finally:
            conn.close()
            print("  - conn closed %s" % (addr,))

    import threading
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
    conn = argparse.ArgumentParser(add_help=False)
    conn.add_argument("--host", default=DEFAULT_HOST)
    conn.add_argument("--port", type=int, default=DEFAULT_PORT,
                      help="target port (client) / listen port (mock)")
    conn.add_argument("--timeout", type=float, default=5.0)
    conn.add_argument("--watch", type=float, default=0.0,
                      help="after sending, read status this many seconds")

    p = argparse.ArgumentParser(
        description="GZH-2 TCP 8529 lights probe.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("selftest", help="verify CRC vs spec frames",
                   parents=[conn]).set_defaults(fn=cmd_selftest)

    sp = sub.add_parser("light", help="searchlight on/off", parents=[conn])
    sp.add_argument("state", choices=["on", "off"])
    sp.set_defaults(fn=cmd_light)

    sp = sub.add_parser("bright", help="searchlight brightness 0-30", parents=[conn])
    sp.add_argument("--level", type=int, required=True)
    sp.set_defaults(fn=cmd_bright)

    sp = sub.add_parser("strobe", help="searchlight strobe on/off", parents=[conn])
    sp.add_argument("state", choices=["on", "off"])
    sp.set_defaults(fn=cmd_strobe)

    sp = sub.add_parser("redblue", help="red/blue warning mode 0-16 (0=off)", parents=[conn])
    sp.add_argument("--mode", type=int, required=True)
    sp.set_defaults(fn=cmd_redblue)

    sp = sub.add_parser("flash", help="software strobe at a chosen rate (faster than built-in 爆闪)",
                        parents=[conn])
    sp.add_argument("--hz", type=float, default=5.0, help="flashes per second (0.5-12, default 5)")
    sp.add_argument("--seconds", type=float, default=6.0, help="how long to flash")
    sp.add_argument("--bright", type=int, default=BRIGHT_MAX, help="on-phase brightness 1-30")
    sp.add_argument("--method", choices=["onoff", "bright"], default="onoff",
                    help="onoff=toggle 0x01 lamp; bright=toggle 0x02 brightness 30<->0")
    sp.add_argument("--keep-on", dest="keep_on", action="store_true",
                    help="leave searchlight steady on at the end")
    sp.set_defaults(fn=cmd_flash)

    sub.add_parser("off", help="all lights off", parents=[conn]).set_defaults(fn=cmd_off)

    sp = sub.add_parser("status", help="watch 0x25 status reports", parents=[conn])
    sp.add_argument("--seconds", type=float, default=3.0)
    sp.set_defaults(fn=cmd_status)

    sp = sub.add_parser("scene", help="常亮最亮 -> 爆闪 -> 红蓝 -> off", parents=[conn])
    sp.add_argument("--hold", type=float, default=4.0, help="seconds to hold each phase")
    sp.add_argument("--mode", type=int, default=1, help="red/blue mode for phase 3")
    sp.add_argument("--keep", action="store_true", help="leave red/blue on at the end")
    sp.set_defaults(fn=cmd_scene)

    sp = sub.add_parser("raw", help="send arbitrary hex frame", parents=[conn])
    sp.add_argument("--hex", required=True)
    sp.add_argument("--listen", type=float, default=1.0)
    sp.set_defaults(fn=cmd_raw)

    sp = sub.add_parser("mock", help="run a fake 8529 device", parents=[conn])
    sp.add_argument("--bind", default="127.0.0.1")
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
        print("  check cabling, device IP (--host), and that port %d is open." % args.port)
        sys.exit(1)


if __name__ == "__main__":
    main()
