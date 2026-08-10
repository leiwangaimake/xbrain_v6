#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: chassis_stub.py
Brief: Dev-side CHS-A endpoint stub (bind :30004, print received APDUs)

Description:
When the real M20S chassis is not connected, run this on the ORIN to
receive CHS-A frames p1_motion / chassis_relay would have sent. It
binds tcp/0.0.0.0:30004 (matches 13 S2.2 channel 1 for control), reads
the 16-byte APDU header + ASDU JSON payload, prints one line per frame
to stdout + appends to /opt/xbrain_v6/logs/chassis_stub.log (V6 rule:
all runtime logs under /opt/xbrain_v6/logs/, never /tmp).

The stub does NOT synthesise chassis state feedback -- p1 will
observe cmd_age stall + state/chassis_basic timeout, which is the
correct behaviour for 'chassis offline'.

Usage:
  python3 scripts/dev/chassis_stub.py [--port 30004] [--host 0.0.0.0]

Stop with Ctrl-C.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import sys
import threading
import time


DEFAULT_PORT = 30004
DEFAULT_HOST = "0.0.0.0"
LOG_PATH = "/opt/xbrain_v6/logs/chassis_stub.log"

# CHS-A APDU header = 16 bytes (13 S2.2). Exact field layout is
# vendor-specific; for the stub we just read 16 bytes then read the
# ASDU length hint stored in the last 4 bytes (BE uint32).
APDU_HEADER_BYTES = 16


def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()) + \
           f".{int((time.time() % 1) * 1000):03d}"


def _ensure_log_dir() -> None:
    os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)


def _write_log(line: str) -> None:
    _ensure_log_dir()
    with open(LOG_PATH, "a", encoding="utf-8") as fh:
        fh.write(line + "\n")


def handle_client(client_sock: socket.socket, addr: tuple) -> None:
    """One connection. Loops reading APDU header + ASDU JSON."""
    peer = f"{addr[0]}:{addr[1]}"
    print(f"[{_now_iso()}] connect {peer}", flush=True)
    _write_log(f"connect {peer}")
    try:
        while True:
            header = _read_exact(client_sock, APDU_HEADER_BYTES)
            if header is None:
                break
            # Last 4 bytes of header hold ASDU length hint (big-endian).
            asdu_len = struct.unpack(">I", header[-4:])[0]
            if asdu_len == 0 or asdu_len > 65_536:
                # Malformed or heartbeat; print header and continue.
                print(f"[{_now_iso()}] {peer} HEADER "
                      f"{header.hex()} (asdu_len={asdu_len})",
                      flush=True)
                _write_log(f"{peer} HEADER {header.hex()} asdu_len={asdu_len}")
                continue
            body = _read_exact(client_sock, asdu_len)
            if body is None:
                break
            try:
                obj = json.loads(body.decode("utf-8"))
                summary = json.dumps(obj, separators=(",", ":"),
                                       ensure_ascii=False)[:200]
            except (ValueError, UnicodeDecodeError):
                summary = body[:80].hex() + f"...({asdu_len}B)"
            line = (f"[{_now_iso()}] {peer} APDU header={header.hex()} "
                    f"ASDU {summary}")
            print(line, flush=True)
            _write_log(line)
    except (ConnectionResetError, BrokenPipeError, OSError) as exc:
        print(f"[{_now_iso()}] {peer} disconnect: {exc}", flush=True)
    finally:
        try:
            client_sock.close()
        except OSError:
            pass


def _read_exact(sock: socket.socket, n: int):
    """Read exactly n bytes; return bytes or None on close."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--host", default=DEFAULT_HOST)
    args = ap.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((args.host, args.port))
    except OSError as exc:
        print(f"chassis_stub: cannot bind {args.host}:{args.port}: {exc}",
              file=sys.stderr)
        return 1
    server.listen(4)
    print(f"chassis_stub listening on {args.host}:{args.port} "
          f"(log -> {LOG_PATH})", flush=True)
    _ensure_log_dir()
    _write_log(f"start on {args.host}:{args.port}")

    try:
        while True:
            client, addr = server.accept()
            t = threading.Thread(target=handle_client,
                                   args=(client, addr),
                                   name=f"chassis_stub[{addr[0]}]",
                                   daemon=True)
            t.start()
    except KeyboardInterrupt:
        print("\nchassis_stub: exiting on Ctrl-C", flush=True)
        _write_log("exit on SIGINT")
        return 0
    finally:
        try:
            server.close()
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
