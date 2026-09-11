#!/usr/bin/env python3
"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: perception_rx_audit.py
Brief: receiver-side ledger for the three perception keys (11 S3.1B.3/.5 v2.1)

Description:
The RNS side's half of the frame-identity reconciliation the acceptance plan
requires (perception-rns-reply-20260911 Q3.2): subscribe the three keys on the
RT plane exactly as p1_motion does, validate each sample with the production
parsers (perception_src/three_keys), and keep, per frame, the identity
timestamp, the publish timestamp and the receive time on this host's
CLOCK_MONOTONIC. At the end print one JSON summary and optionally a CSV of
every frame, so producer-side and receiver-side ledgers can be joined on
t_capture_mono_ms.

What it measures (the contract's own quantities, nothing else):
  L2 per key      t_rx - t_capture (profile / objects), t_rx - t_publish
                  (status): the age the consumer sees (11 S3.1B.5 v2.1 L2)
  objects cadence dpub = t_publish[i] - t_publish[i-1] over UNIQUE new
                  results (dedup by t_capture): max, P99, count and share
                  > 50 ms, count > 100 ms, and the unfinished gap at exit
                  (11 S3.1B.3 v2.1 tiered gate: tier 1 max <= 100 ms, tier 2
                  share > 50 ms <= 1 %)
  acceptance      dup / out_of_order / future / epoch_reset counts per key
                  (rns/inputs.classify_arrival, the same function RNS runs)
  rejections      samples the strict parser refused, with the last reason
  T-5x            how many samples arrived with an age already past T-50 /
                  T-51 (profile), T-52 (objects), T-53 (status)

What it is NOT: the acceptance verdict. It reports the numbers; the verdict
is taken per window against 11 S3.1B.3 v2.1 by people, with the producer's
ledger beside it. It also never publishes anything.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from xbrain.p1_motion.perception_src.three_keys import (  # noqa: E402
    KEYS, key_expr, parse_payload)
from xbrain.p1_motion.rns.inputs import (  # noqa: E402
    ARRIVAL_ACCEPT, ARRIVAL_EPOCH_RESET, classify_arrival)

RT_ENDPOINT = "tcp/127.0.0.1:7449"
# 11 S1.6.1 consumer timeouts (ms); the ledger only counts arrivals past them
T50, T51, T52, T53 = 300, 1000, 500, 3000
TIER1_MAX_MS = 100.0        # 11 S3.1B.3 v2.1 tier 1: max dpub (incl. open gap)
TIER2_LIMIT_MS = 50.0       # tier 2: share of dpub > 50 ms must be <= 1 %


def _now_ms() -> int:
    return int(time.monotonic() * 1000)


def _percentile(values: List[float], q: float) -> Optional[float]:
    """Nearest-rank percentile (same rule 19 S5.1 uses for latency_ms_p99)."""
    if not values:
        return None
    s = sorted(values)
    k = max(0, min(len(s) - 1, int(round(q * len(s) + 0.5)) - 1))
    return s[k]


class Ledger:
    """Per-key receive ledger; on_sample runs on the Rust thread and only
    appends (CLAUDE.md 4.2), summary() is read once at exit."""

    def __init__(self) -> None:
        self.rows: List[Dict[str, Any]] = []
        self.stats: Dict[str, Dict[str, Any]] = {
            k: {"received": 0, "rejected": 0, "last_error": None,
                "accept": 0, "dup": 0, "out_of_order": 0, "future": 0,
                "epoch_reset": 0, "last_t": None, "l2": [],
                "past_t_first": 0, "past_t_second": 0}
            for k in KEYS}
        self.dpub: List[float] = []
        self._last_pub: Optional[int] = None
        self.t_start = _now_ms()

    def on_sample(self, key: str, sample: Any) -> None:
        rx = _now_ms()
        st = self.stats[key]
        try:
            dto = parse_payload(key, bytes(sample.payload))
        except Exception as exc:      # noqa: BLE001 -- ledger, count everything
            st["rejected"] += 1
            st["last_error"] = str(exc)
            return
        st["received"] += 1
        if key == "status":
            ident = dto.t_publish_mono_ms
            t_pub = dto.t_publish_mono_ms
        else:
            ident = dto.t_capture_mono_ms
            t_pub = getattr(dto, "t_publish_mono_ms", None)
        verdict = classify_arrival(st["last_t"], ident, rx)
        st[verdict] += 1
        if verdict in (ARRIVAL_ACCEPT, ARRIVAL_EPOCH_RESET):
            st["last_t"] = ident
            age = rx - ident
            st["l2"].append(age)
            first, second = {"profile": (T50, T51), "objects": (T52, None),
                             "status": (T53, None)}[key]
            if age > first:
                st["past_t_first"] += 1
            if second is not None and age > second:
                st["past_t_second"] += 1
            if key == "objects" and t_pub is not None:
                # cadence over unique new results (11 S3.1B.3 v2.1); a dup or
                # out-of-order frame never contributes an interval
                if self._last_pub is not None and verdict == ARRIVAL_ACCEPT:
                    self.dpub.append(float(t_pub - self._last_pub))
                self._last_pub = t_pub
        self.rows.append({"rx_mono_ms": rx, "key": key, "ident_mono_ms": ident,
                          "t_publish_mono_ms": t_pub, "verdict": verdict,
                          "age_ms": rx - ident})

    def summary(self) -> Dict[str, Any]:
        now = _now_ms()
        out: Dict[str, Any] = {"window_s": round((now - self.t_start) / 1000.0, 1),
                               "keys": {}}
        for k, st in self.stats.items():
            out["keys"][k] = {
                "received": st["received"], "rejected": st["rejected"],
                "last_error": st["last_error"],
                "accept": st["accept"], "dup": st["dup"],
                "out_of_order": st["out_of_order"], "future": st["future"],
                "epoch_reset": st["epoch_reset"],
                "l2_p50_ms": _percentile(st["l2"], 0.50),
                "l2_p99_ms": _percentile(st["l2"], 0.99),
                "l2_max_ms": max(st["l2"]) if st["l2"] else None,
                "past_t_first": st["past_t_first"],
                "past_t_second": st["past_t_second"],
            }
        d = self.dpub
        open_gap = None
        if self._last_pub is not None and self.stats["objects"]["l2"]:
            # unfinished gap: from the last unique result's publish stamp to
            # "now" on the receiver clock (both monotonic, same machine)
            open_gap = float(now - self._last_pub)
        over50 = sum(1 for x in d if x > TIER2_LIMIT_MS)
        over100 = sum(1 for x in d if x > TIER1_MAX_MS)
        max_d = max(d) if d else None
        tier1_max = max([x for x in (max_d, open_gap) if x is not None], default=None)
        out["objects_cadence"] = {
            "intervals": len(d),
            "dpub_max_ms": max_d, "dpub_p99_ms": _percentile(d, 0.99),
            "over_50ms": over50,
            "over_50ms_share": (over50 / len(d)) if d else None,
            "over_100ms": over100,
            "open_gap_ms": open_gap,
            "tier1_pass": (tier1_max is not None and tier1_max <= TIER1_MAX_MS
                           and over100 == 0),
            "tier2_pass": (bool(d) and (over50 / len(d)) <= 0.01),
        }
        return out

    def write_csv(self, path: Path) -> None:
        with path.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=["rx_mono_ms", "key", "ident_mono_ms",
                                               "t_publish_mono_ms", "verdict",
                                               "age_ms"])
            w.writeheader()
            for r in self.rows:
                w.writerow(r)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="perception_rx_audit.py",
        description="Receiver-side ledger for rt/perception/{profile,objects,status}")
    ap.add_argument("--rid", default="dev", help="robot id in the key (default dev)")
    ap.add_argument("--seconds", type=float, default=30.0,
                    help="listen window; <= 0 runs until Ctrl-C (default 30)")
    ap.add_argument("--csv", default=None, help="write every frame to this CSV")
    args = ap.parse_args(argv)
    try:
        import zenoh
    except ImportError:
        print("zenoh-python not installed (pip install eclipse-zenoh)", file=sys.stderr)
        return 2
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["%s"]' % RT_ENDPOINT)
    try:
        session = zenoh.open(conf)
    except Exception as exc:      # noqa: BLE001 -- surface the connect failure
        print("cannot attach to RT plane (%s): %s" % (RT_ENDPOINT, exc), file=sys.stderr)
        return 1
    ledger = Ledger()
    subs = []                       # strong refs (CLAUDE.md 4.3)
    import functools
    for key in KEYS:
        subs.append(session.declare_subscriber(
            key_expr(args.rid, key), functools.partial(ledger.on_sample, key)))
    print("listening rid=%s on %s for %s" % (
        args.rid, RT_ENDPOINT, "ever" if args.seconds <= 0 else "%.0fs" % args.seconds),
        file=sys.stderr)
    deadline = None if args.seconds <= 0 else time.monotonic() + args.seconds
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for s in subs:
            try:
                s.undeclare()
            except Exception:      # noqa: BLE001
                pass
        session.close()
    print(json.dumps(ledger.summary(), indent=2, sort_keys=True))
    if args.csv:
        ledger.write_csv(Path(args.csv))
        print("csv: %s (%d rows)" % (args.csv, len(ledger.rows)), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
