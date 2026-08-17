"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_b.py
Brief: GWY-P5-03/04/05/06 replay + recon + backpressure + approval queue tests

Description:
Batch B: reconnect replay batches respect max_batch (R-1), reject
head < cursor (R-2), drop-oldest-info-first preserves warn/error;
reconciliation missing-range computation (RC-2 empty), width
enforcement (RC-4), range dedup / merge (RC-3); backpressure
classifier and ingress-drop (info only, warn/error never); approval
queue AV-5 idempotency + AV-7 non-pending mutation refused + AV-3
TTL expiry + RB-3 rollback cycle detection.
"""

import pytest

from xbrain.p5_gateway.approval.queue import (
    ApprovalCycleDetected, ApprovalEntry, ApprovalMutationForbidden,
    ApprovalQueue, ApprovalState,
)
from xbrain.p5_gateway.backpressure.watermark import (
    BackpressureState, BackpressureThresholds,
    classify, should_drop_at_ingress, trim_info_from_queue,
)
from xbrain.p5_gateway.delivery.recon import (
    ReconWindowExceeded, SeqRange,
    compute_missing, dedupe_ranges, enforce_window,
)


pytestmark = pytest.mark.no_device


# GWY-P5-03 replay moved to tests/p5_gateway/reconnect/test_backfill.py -- the
# old per-consumer/event_seq/level model was replaced by the contract backfill
# (per-channel ch_seq cursors, 4:1 weight, rate limiter, EventReplay messages).


# --- GWY-P5-04 recon ---

def test_missing_range_none_when_in_sync():
    """RC-2: consumer caught up = empty missing set."""
    assert compute_missing(100, 100) is None


def test_missing_range_computed_when_lagging():
    r = compute_missing(50, 60)
    assert r == SeqRange(51, 60)


def test_recon_window_enforced():
    """RC-4: any range wider than max_window is refused."""
    with pytest.raises(ReconWindowExceeded):
        enforce_window([SeqRange(1, 100)], max_window=50)


def test_recon_window_within_limits_ok():
    enforce_window([SeqRange(1, 50)], max_window=50)   # inclusive


def test_dedupe_merges_overlapping_ranges():
    r = dedupe_ranges([SeqRange(1, 5), SeqRange(3, 7)])
    assert r == (SeqRange(1, 7),)


def test_dedupe_merges_adjacent_ranges():
    r = dedupe_ranges([SeqRange(1, 5), SeqRange(6, 10)])
    assert r == (SeqRange(1, 10),)


def test_dedupe_keeps_disjoint_ranges():
    r = dedupe_ranges([SeqRange(1, 5), SeqRange(10, 15)])
    assert r == (SeqRange(1, 5), SeqRange(10, 15))


def test_seq_range_rejects_inverted():
    with pytest.raises(ValueError):
        SeqRange(from_seq=10, to_seq=5)


# --- GWY-P5-05 backpressure ---

THR = BackpressureThresholds(low=100, high=500, overflow=1000)


def test_classify_normal():
    assert classify(50, THR) == BackpressureState.NORMAL


def test_classify_elevated():
    assert classify(600, THR) == BackpressureState.ELEVATED


def test_classify_overflow():
    assert classify(1100, THR) == BackpressureState.OVERFLOW


def test_ingress_drops_info_in_elevated():
    assert should_drop_at_ingress("info", BackpressureState.ELEVATED) is True


def test_ingress_never_drops_warn_or_error():
    for lvl in ("warn", "error"):
        assert should_drop_at_ingress(lvl, BackpressureState.OVERFLOW) is False


def test_ingress_normal_never_drops():
    assert should_drop_at_ingress("info", BackpressureState.NORMAL) is False


def test_trim_info_preserves_warn_error():
    q = [{"level": "info"}, {"level": "warn"},
         {"level": "info"}, {"level": "error"}]
    trim_info_from_queue(q, target_len=2)
    assert all(e["level"] in ("warn", "error") for e in q)


# --- GWY-P5-06 approval queue ---

def _entry(id: str, submitted: int = 0, ttl: int = 60000):
    return ApprovalEntry(approval_id=id, action={},
                           source="AV-1", submitted_ms=submitted,
                           ttl_ms=ttl)


def test_approval_submit_idempotent():
    """AV-5: duplicate approval_id returns existing entry."""
    q = ApprovalQueue()
    e1 = q.submit(_entry("a", submitted=0))
    e2 = q.submit(_entry("a", submitted=100))
    assert e1 is e2 and e1.submitted_ms == 0


def test_approval_approve_pending_ok():
    q = ApprovalQueue()
    q.submit(_entry("a"))
    q.approve("a", now_ms=10)
    assert q.entries["a"].state == ApprovalState.APPROVED


def test_approval_approve_terminal_rejected():
    """AV-7: mutation of non-pending refused."""
    q = ApprovalQueue()
    q.submit(_entry("a"))
    q.reject("a", now_ms=5)
    with pytest.raises(ApprovalMutationForbidden):
        q.approve("a", now_ms=10)


def test_approval_expire_stale_pending():
    """AV-3: TTL expiry marks pending entries EXPIRED."""
    q = ApprovalQueue()
    q.submit(_entry("a", submitted=0, ttl=100))
    n = q.expire_stale(now_ms=200)
    assert n == 1
    assert q.entries["a"].state == ApprovalState.EXPIRED


def test_approval_expire_ignores_terminal():
    """AV-4: terminal entries are unaffected by expiry sweep."""
    q = ApprovalQueue()
    q.submit(_entry("a", submitted=0, ttl=100))
    q.approve("a", now_ms=50)
    n = q.expire_stale(now_ms=200)
    assert n == 0
    assert q.entries["a"].state == ApprovalState.APPROVED


def test_rollback_depth_over_limit_raises():
    """RB-3: rollback chain too deep -> cycle detected."""
    q = ApprovalQueue(max_rollback_depth=3)
    q.check_rollback_depth(3)     # ok
    with pytest.raises(ApprovalCycleDetected):
        q.check_rollback_depth(4)
