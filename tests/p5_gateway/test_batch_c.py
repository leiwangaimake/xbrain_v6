"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_c.py
Brief: GWY-P5-07/08/10/13 telemetry + delivery + HMI WS + REST tests

Description:
Batch C: 4-class telemetry ring buffer (capacity respected,
unknown class rejected); weak-link downsample factor exact = 4;
delivery 3-stage transitions enforced (DP-1: submitted->in_flight
->delivered; stuck retry counted; DP-6 stuck timeout); HMI WS 9-down
and 5-up closed sets rejected on unknown kind; rate-limit bucket
depletion; REST GET-only enforcement; fences degraded -> 503.
"""

import pytest

from xbrain.p5_gateway.delivery.handshake import (
    DeliveryRecord, DeliveryStage, InvalidDeliveryTransition,
    is_stuck, transition,
)
from xbrain.p5_gateway.hmi.ws_protocol import (
    DOWN_MESSAGE_KINDS, RateLimitBucket, UP_MESSAGE_KINDS,
    UnknownMessageKind, classify_down, classify_up,
)
from xbrain.p5_gateway.rest.endpoints import (
    EndpointNotAllowed, READONLY_ENDPOINTS,
    check_readonly, fences_endpoint,
)
from xbrain.p5_gateway.telemetry.aggregator import (
    Sample, TELEMETRY_CLASSES, TelemetryClass, TelemetryRing,
    WEAK_LINK_DOWNSAMPLE, is_weak_link, uplink_cadence_ms,
)


pytestmark = pytest.mark.no_device


# --- GWY-P5-07 telemetry ---

def test_telemetry_classes_closed_set():
    assert TELEMETRY_CLASSES == frozenset(TelemetryClass)


def test_weak_link_triggers_on_rtt():
    assert is_weak_link(rtt_ms=200.0, loss_pct=0.0,
                          rtt_threshold_ms=100.0,
                          loss_threshold_pct=5.0) is True


def test_weak_link_triggers_on_loss():
    assert is_weak_link(50.0, 10.0, 100.0, 5.0) is True


def test_weak_link_false_when_below_both():
    assert is_weak_link(50.0, 0.0, 100.0, 5.0) is False


def test_downsample_factor_is_four():
    assert uplink_cadence_ms(1000, weak_link=True) == 4000
    assert uplink_cadence_ms(1000, weak_link=False) == 1000
    assert WEAK_LINK_DOWNSAMPLE == 4


def test_telemetry_ring_capacity():
    r = TelemetryRing(capacity_per_class=2)
    for i in range(3):
        r.append(Sample(TelemetryClass.SYSTEM.value, i, {"v": i}))
    snap = r.snapshot(TelemetryClass.SYSTEM.value)
    assert len(snap) == 2 and snap[0].fields["v"] == 1


def test_telemetry_ring_unknown_class_raises():
    r = TelemetryRing(capacity_per_class=2)
    with pytest.raises(ValueError):
        r.append(Sample("halfway", 0, {}))


# --- GWY-P5-08 delivery handshake ---

def test_delivery_submit_to_in_flight_ok():
    rec = DeliveryRecord(event_seq=1, consumer="cloud",
                           stage=DeliveryStage.SUBMITTED, updated_ms=0)
    transition(rec, DeliveryStage.IN_FLIGHT, now_ms=10, max_retries=3)
    assert rec.stage == DeliveryStage.IN_FLIGHT


def test_delivery_delivered_is_terminal():
    """DP-1: no transition out of DELIVERED."""
    rec = DeliveryRecord(1, "cloud", DeliveryStage.DELIVERED, 0)
    with pytest.raises(InvalidDeliveryTransition):
        transition(rec, DeliveryStage.IN_FLIGHT, now_ms=10, max_retries=3)


def test_delivery_submitted_cant_skip_to_delivered():
    rec = DeliveryRecord(1, "cloud", DeliveryStage.SUBMITTED, 0)
    with pytest.raises(InvalidDeliveryTransition):
        transition(rec, DeliveryStage.DELIVERED, now_ms=10, max_retries=3)


def test_delivery_stuck_retry_counts():
    rec = DeliveryRecord(1, "cloud", DeliveryStage.STUCK, 0)
    transition(rec, DeliveryStage.IN_FLIGHT, now_ms=10, max_retries=3)
    assert rec.retry_count == 1


def test_delivery_retry_max_exceeded_raises():
    rec = DeliveryRecord(1, "cloud", DeliveryStage.STUCK, 0,
                           retry_count=3)
    with pytest.raises(InvalidDeliveryTransition, match="retry_count"):
        transition(rec, DeliveryStage.IN_FLIGHT, now_ms=10, max_retries=3)


def test_is_stuck_by_timeout():
    """DP-6: in_flight past timeout is stuck."""
    rec = DeliveryRecord(1, "cloud", DeliveryStage.IN_FLIGHT,
                           updated_ms=0)
    assert is_stuck(rec, now_ms=6000, timeout_ms=5000) is True
    assert is_stuck(rec, now_ms=1000, timeout_ms=5000) is False


def test_is_stuck_only_for_in_flight():
    """A submitted record is not 'stuck' by this definition."""
    rec = DeliveryRecord(1, "cloud", DeliveryStage.SUBMITTED, 0)
    assert is_stuck(rec, now_ms=10000, timeout_ms=5000) is False


# --- GWY-P5-10 HMI WS ---

def test_ws_down_kinds_closed_set():
    assert len(DOWN_MESSAGE_KINDS) == 9


def test_ws_up_kinds_closed_set():
    assert len(UP_MESSAGE_KINDS) == 5


def test_classify_down_valid_kind():
    assert classify_down({"kind": "event"}) == "event"


def test_classify_down_unknown_raises():
    with pytest.raises(UnknownMessageKind):
        classify_down({"kind": "halfway"})


def test_classify_up_ack_ok():
    assert classify_up({"kind": "ack"}) == "ack"


def test_classify_up_unknown_raises():
    with pytest.raises(UnknownMessageKind):
        classify_up({"kind": "burn"})


def test_rate_limit_depletion():
    b = RateLimitBucket(capacity=3, tokens=3,
                          fill_rate_per_ms=0.0, last_refill_ms=0)
    assert b.try_take(now_ms=0) is True
    assert b.try_take(now_ms=0) is True
    assert b.try_take(now_ms=0) is True
    assert b.try_take(now_ms=0) is False   # depleted, no refill rate


def test_rate_limit_refills():
    b = RateLimitBucket(capacity=3, tokens=0,
                          fill_rate_per_ms=0.001, last_refill_ms=0)
    # After 3000ms, refill of 3 (3000 * 0.001 = 3).
    assert b.try_take(now_ms=3000) is True


# --- GWY-P5-13 REST ---

def test_rest_get_ok():
    check_readonly("GET", "/api/health")


def test_rest_post_rejected():
    with pytest.raises(EndpointNotAllowed, match="read-only"):
        check_readonly("POST", "/api/health")


def test_rest_unknown_endpoint_rejected():
    with pytest.raises(EndpointNotAllowed, match="unknown endpoint"):
        check_readonly("GET", "/api/wombats")


def test_readonly_endpoints_exactly_eight():
    assert len(READONLY_ENDPOINTS) == 8


def test_fences_degraded_returns_503():
    r = fences_endpoint(fence_db_degraded=True)
    assert r.status == 503 and r.body == {"error": "E_DEGRADED"}


def test_fences_normal_returns_200():
    r = fences_endpoint(fence_db_degraded=False)
    assert r.status == 200 and "fences" in r.body
