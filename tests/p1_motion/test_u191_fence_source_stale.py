"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_u191_fence_source_stale.py
Brief: 99 U191 -- P3 silent on cmd/fence: P1 keeps enforcing the last set and says so (src_state stale)

Description:
11 S9A.4 "P3 挂掉 / 通用面断链": P1 keeps the current active set, and once the
last cmd/fence is older than stale_s, src_state = "stale", reported once,
enforcement unchanged. The fence plan (section 2) asked for exactly this;
until 2026-11-13 P1 kept the set (correct) but never reported, and src_age_s
counted from the last rev CHANGE, so P3's 5 s keepalive never refreshed it.
User ruling 2026-11-13: stale after 15 s. Each case names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import pytest  # marker

from xbrain.p1_motion.fence.fence_set import (  # under test
    FenceSetError,
    FenceSetHolder,
    SourceStaleTracker,
    build_fence_runtime_state,
)

pytestmark = pytest.mark.no_device  # pure logic


def _wire(rev):  # an empty, correctly checksummed FenceSet (FV-8 self-check)
    from xbrain.common.fence.geom import fence_set_crc32  # the shared recipe
    return {"fence_set_id": "fs", "rev": rev, "crc32": fence_set_crc32("fs", rev, []),
            "polygons": []}


def test_stale_and_restored_are_told_once_each():
    """MUTATION: no edge memory -> an event every tick -> red; the restore
    never reported -> red; stale judged before any frame -> red."""
    t = SourceStaleTracker(15.0)  # the user's 15 s
    assert t.observe(None, 100.0) is None  # nothing ever received: fence_lost's business
    assert t.observe(100.0, 114.9) is None  # within 15 s
    assert t.observe(100.0, 115.1) == "fence_source_stale"  # just past
    assert t.observe(100.0, 130.0) is None  # still stale: not repeated
    assert t.observe(131.0, 131.5) == "fence_source_restored"  # a frame came back
    assert t.observe(131.0, 132.0) is None  # fine again: silent


def test_state_fence_reports_stale_without_degrading():
    """src_state stale, src_age_s from the last receipt, enforcement and the
    active set untouched. MUTATION: stale flag ignored -> "active" -> red; age
    from applied instead of receipt -> 90 not 20 -> red."""
    holder = FenceSetHolder()  # empty set is enough for the state body
    held = holder.accept(_wire(3), rx_mono_s=80.0)  # received at 80 s
    assert holder.last_rx_mono == 80.0  # recorded
    body = build_fence_runtime_state(held, now_mono_s=100.0, applied_mono_s=10.0,
                                     last_rx_mono_s=80.0, stale=True)  # 20 s silent
    assert body["src_state"] == "stale" and body["src_age_s"] == 20.0
    assert body["active"]["rev"] == 3  # still the last set
    fresh = build_fence_runtime_state(held, now_mono_s=100.0, applied_mono_s=10.0,
                                      last_rx_mono_s=99.0)  # the keepalive refreshed it
    assert fresh["src_state"] == "active" and fresh["src_age_s"] == 1.0
    assert fresh["enforcement"] == body["enforcement"]  # stale does not degrade


def test_a_bad_frame_does_not_count_as_alive():
    """FS-7 keeps the old geometry on a bad frame; it must not refresh the
    liveness either. MUTATION: rx stamped before validation -> red."""
    holder = FenceSetHolder()  # empty
    holder.accept(_wire(1), rx_mono_s=10.0)  # good frame
    with pytest.raises(FenceSetError):  # refused, FS-7
        holder.accept({"rev": "x"}, rx_mono_s=50.0)  # garbage
    assert holder.last_rx_mono == 10.0  # unchanged


def test_the_wiring_stamps_the_receipt_and_publishes_transitions():
    """Source check of p1 main_wiring (the loop needs live sessions).
    MUTATION: accept without rx_mono_s / no observe call / no event publish -> red."""
    import inspect  # read the wiring source

    import xbrain.p1_motion.runtime.main_wiring as mw  # the module
    src = inspect.getsource(mw)  # its text
    assert "fence_holder.accept(d, rx_mono_s=time.monotonic())" in src  # receipt stamped
    assert "fence_stale.observe(_rx, now)" in src  # judged every pass
    assert "_publish_source_event(_stale_ev, _held, now - _rx)" in src  # and reported
    assert "nav_cfg.fence_source_stale_s" in src  # the configured 15 s
