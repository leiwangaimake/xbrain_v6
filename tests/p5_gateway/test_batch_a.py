"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_batch_a.py
Brief: GWY-P5-25 lifecycle skeleton + GWY-P5-01 pipeline stage order

Description:
The lifecycle skeleton (nine-task registry, the two record.db writers) plus the
pipeline STAGE-ORDER gate. The event schema + stage-branch tests that used to
live here were against the placeholder model (categories {safety,...}, level
{info,warn,error}, per-consumer cursor); they were removed when the pipeline +
record.db were rewritten onto the real contract. Their replacements are
tests/p5_gateway/event/test_pipeline.py and tests/p5_gateway/persistence/.
"""

import pytest

from xbrain.p5_gateway.event.pipeline import (
    PIPELINE_STAGES, PipelineOrderViolation, assert_stage_order,
)
from xbrain.p5_gateway.lifecycle.skeleton import (
    DuplicateTaskRegistration, IncompleteTaskRegistration,
    NINE_TASKS, P1_P2_WRITERS, TaskRegistry,
)


pytestmark = pytest.mark.no_device


# --- GWY-P5-25 skeleton ---

def test_registry_starts_empty():
    r = TaskRegistry.empty()
    assert r.tasks == {}


def test_registry_rejects_unknown_task_name():
    r = TaskRegistry.empty()
    with pytest.raises(DuplicateTaskRegistration, match="unknown"):
        r.register("halfway", object())


def test_registry_rejects_duplicate_register():
    r = TaskRegistry.empty()
    r.register("watchdog", object())
    with pytest.raises(DuplicateTaskRegistration, match="already"):
        r.register("watchdog", object())


def test_registry_assert_complete_fails_when_missing():
    r = TaskRegistry.empty()
    r.register("watchdog", object())
    with pytest.raises(IncompleteTaskRegistration, match="missing"):
        r.assert_complete()


def test_registry_all_nine_pass():
    r = TaskRegistry.empty()
    for n in NINE_TASKS:
        r.register(n, object())
    r.assert_complete()


def test_p1_p2_are_the_two_writers():
    """P-1 event_pipeline, P-2 delivery_ledger. No others."""
    assert P1_P2_WRITERS == frozenset(
        {"event_pipeline", "delivery_ledger"})


def test_writer_check():
    r = TaskRegistry.empty()
    assert r.can_write_record_db("event_pipeline") is True
    assert r.can_write_record_db("hmi_ws") is False


# --- GWY-P5-01 pipeline stage order (persist MUST precede cloud) ---

def test_stage_order_matches_expected():
    assert_stage_order(PIPELINE_STAGES)


def test_stage_order_bad_raises():
    # MUTATION: swapping persist after cloud must be rejected -- it would let a
    # crash lose a cloud-acked event (17 S3.1).
    bad = ("schema", "dedupe", "level", "cloud", "persist", "hmi", "delivered")
    with pytest.raises(PipelineOrderViolation):
        assert_stage_order(bad)
