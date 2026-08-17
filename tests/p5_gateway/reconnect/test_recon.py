"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_recon.py
Brief: recon pure core -- req builder + resend-set computation (17 S3Y.3)

Description:
Pins the diffing logic with plain ints (no DAO, no Zenoh). The load-bearing case is
the my_min clamp: a resend set must never include a ch_seq below what we still hold,
or we promise the cloud rows we purged at retention and recon never converges (the
endless-loop trap, S3Y.3). Mutations paired per 3.3.
"""

import pytest

from xbrain.p5_gateway.reconnect.recon import build_recon_req, compute_resend_seqs


pytestmark = pytest.mark.no_device


def test_build_req_fields():
    r = build_recon_req("alarm", my_min=3, my_max=9, req_id="rc-1-alarm")
    assert r == {"req_id": "rc-1-alarm", "channel": "alarm",
                 "my_max_seq": 9, "my_min_seq": 3}


def test_tail_above_their_max():
    # We hold 1..5; the cloud has up to 2 -> resend 3,4,5.
    assert compute_resend_seqs(2, None, False, my_max=5, my_min=1,
                               max_resend=100) == [3, 4, 5]


def test_my_min_clamp_never_resends_purged_rows():
    # We hold only 5..8 (1..4 purged at retention); cloud has nothing (their_max=0).
    # MUTATION: without the my_min clamp this returns 1..8 -> we promise 1..4 which
    # no longer exist -> recon can never converge (S3Y.3 endless loop).
    assert compute_resend_seqs(0, None, False, my_max=8, my_min=5,
                               max_resend=100) == [5, 6, 7, 8]


def test_explicit_missing_ranges():
    assert compute_resend_seqs(10, [[3, 4], [7, 7]], False,
                               my_max=10, my_min=1, max_resend=100) == [3, 4, 7]


def test_missing_ranges_clamped_to_held():
    # Range dips below my_min -> clamp up; we cannot resend what we dropped.
    assert compute_resend_seqs(10, [[1, 6]], False,
                               my_max=10, my_min=5, max_resend=100) == [5, 6]


def test_union_of_tail_and_ranges_deduped():
    # tail (their_max=3) = {4,5,6}; ranges {5,6} -> union deduped ascending.
    assert compute_resend_seqs(3, [[5, 6]], False,
                               my_max=6, my_min=1, max_resend=100) == [4, 5, 6]


def test_max_resend_cap():
    # RC-5: at most max_resend this round; the rest waits for the next period.
    out = compute_resend_seqs(0, None, False, my_max=100, my_min=1, max_resend=10)
    assert out == list(range(1, 11))


def test_nothing_missing_is_empty():
    assert compute_resend_seqs(5, None, False, my_max=5, my_min=1,
                               max_resend=100) == []


def test_negative_max_resend_rejected():
    with pytest.raises(ValueError):
        compute_resend_seqs(0, None, False, my_max=5, my_min=1, max_resend=-1)
