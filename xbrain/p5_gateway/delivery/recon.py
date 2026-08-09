"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: recon.py
Brief: GWY-P5-04 delivery reconciliation §3Y L3 (recon/req * rsp + RC-1..5)

Description:
17 S6 defines a per-batch reconciliation between p5 (producer) and
consumer (cloud, hmi). Every N minutes, the consumer publishes a
'recon/req' listing its last-delivered event_seq per source; p5
compares to its ground truth and publishes 'recon/rsp' with the
missing seq ranges.

Reconciliation rules:
  RC-1  seq ranges are inclusive-inclusive
  RC-2  missing set may be empty (perfect state)
  RC-3  duplicate seqs in the response are illegal (dedupe first)
  RC-4  a range wider than max_recon_window_seqs is refused
        (avoid huge one-shot replays that starve fresh traffic)
  RC-5  reconciliation runs INDEPENDENTLY per consumer
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SeqRange:
    """Inclusive-inclusive range of missing event_seqs."""
    from_seq: int
    to_seq: int

    def __post_init__(self) -> None:
        if self.from_seq > self.to_seq:
            raise ValueError(f"empty range {self.from_seq}..{self.to_seq}")


class ReconWindowExceeded(Exception):
    """RC-4: attempted reconciliation exceeds max_recon_window_seqs."""


def compute_missing(consumer_last: int, producer_last: int) -> SeqRange | None:
    """Return the missing range for one consumer.  None if in sync
    or ahead of producer."""
    if consumer_last >= producer_last:
        return None
    return SeqRange(consumer_last + 1, producer_last)


def enforce_window(ranges, max_window: int) -> None:
    """RC-4: reject any single range wider than max_window."""
    for r in ranges:
        width = r.to_seq - r.from_seq + 1
        if width > max_window:
            raise ReconWindowExceeded(
                f"range {r.from_seq}..{r.to_seq} width={width} "
                f"exceeds max_window={max_window}")


def dedupe_ranges(ranges):
    """RC-3: merge overlapping / adjacent ranges."""
    if not ranges:
        return ()
    ordered = sorted(ranges, key=lambda r: r.from_seq)
    out = [ordered[0]]
    for r in ordered[1:]:
        prev = out[-1]
        if r.from_seq <= prev.to_seq + 1:
            out[-1] = SeqRange(prev.from_seq, max(prev.to_seq, r.to_seq))
        else:
            out.append(r)
    return tuple(out)
