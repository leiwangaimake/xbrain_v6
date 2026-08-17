"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: recon.py
Brief: 17 S3Y.3 reconciliation -- recon/req builder + resend-set computation (pure)

Description:
The periodic hole-check that catches events the reconnect-backfill missed (17 S3Y.3):
P5 sends event/recon/req {channel, my_max_seq, my_min_seq}; the cloud answers
event/recon/rsp {their_max_seq, missing_ranges?, truncated?}; P5 resends the gap on
event/replay/{channel}. This module is the PURE core (no DAO, no Zenoh) so the
diffing logic tests with plain ints -- the ReconRunner (uplink/cloud.py) drives it.

Two load-bearing rules from the contract, both easy to get subtly wrong:
  * my_min_seq is REQUIRED in the req (S3Y.3): it is the smallest ch_seq we STILL
    hold. Without it the cloud reports rows we purged at 90-day retention as holes,
    and we try to resend rows that no longer exist -> an endless recon loop. So the
    resend set is CLAMPED to [my_min, my_max]: we never promise to resend what we
    dropped.
  * a rsp with `truncated` (the cloud had more holes than recon.max_ranges) must NOT
    be read as "everything else is fine" -- we resend only what was listed this
    round and reconcile again next period. compute_resend_seqs never invents the
    unlisted holes, so a truncated rsp just yields a partial set; the caller loops.

This is NOT the old delivery/recon.py model (event_seq per SOURCE, level info/warn/
error) -- that keyed on a per-producer cursor that never existed in the contract.
Here the cursor is per-CHANNEL ch_seq (U18 / U18a), matching record.db.
"""

from __future__ import annotations

from typing import List, Optional


def build_recon_req(channel: str, my_min: int, my_max: int, req_id: str) -> dict:
    """The event/recon/req `data` (17 S3Y.3). my_max = MAX(ch_seq) PRODUCED (not
    confirmed_upto); my_min = MIN(ch_seq) still held. All four fields are required
    by the contract -- the cloud needs my_min to not report purged rows as holes."""
    return {
        "req_id": req_id,
        "channel": channel,
        "my_max_seq": my_max,
        "my_min_seq": my_min,
    }


def compute_resend_seqs(their_max: Optional[int],
                        missing_ranges: Optional[list],
                        truncated: bool,
                        my_max: int, my_min: int,
                        max_resend: int) -> List[int]:
    """The ch_seqs to resend for one channel, given the cloud's rsp.

    Union of (a) everything we hold ABOVE their_max_seq -- (their_max, my_max] -- and
    (b) any explicitly-listed missing_ranges [[a,b],...] (inclusive). Both are CLAMPED
    to [my_min, my_max] so we never try to resend a row past retention (the endless-
    loop trap, S3Y.3). Deduped, ascending, capped at max_resend (RC-5); the overflow
    waits for the next recon period. `truncated` is not consulted here -- a truncated
    rsp simply carries fewer ranges, so the set is naturally partial and the caller
    reconciles again. (Accepted so callers can pass it through without a lint warn.)
    """
    _ = truncated
    if max_resend < 0:
        raise ValueError(f"max_resend must be >= 0, got {max_resend}")
    want = set()
    # (a) everything past the cloud's no-hole upper bound, but not below what we hold.
    if their_max is not None:
        lo = max(their_max + 1, my_min)
        for s in range(lo, my_max + 1):
            want.add(s)
    # (b) explicit holes, each clamped into [my_min, my_max].
    for rng in (missing_ranges or []):
        a = max(int(rng[0]), my_min)
        b = min(int(rng[1]), my_max)
        for s in range(a, b + 1):
            want.add(s)
    return sorted(want)[:max_resend]
