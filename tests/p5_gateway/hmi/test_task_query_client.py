"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_task_query_client.py
Brief: P5 query/tasks client -- selector build + reply parse (11 S12.2A)

Description:
Pins the P5 side of the task-panel data path: build_task_selector must join
params with ';' (zenoh, not '&'); parse_get_reply must take the first ok reply
and degrade to an EMPTY page (never raise) when P3 does not answer; query_tasks
must wire the two together over a session.get(). Each check names the mutation it
reddens (CLAUDE.md 3.3). The load-bearing one is the ';' separator -- '&' would
silently make P3 answer the default page.
"""
from __future__ import annotations

from xbrain.p5_gateway.hmi.task_query_client import (
    build_task_selector, parse_get_reply, query_tasks,
)


# -- selector build: ';' separated (zenoh, not '&') ----------------------------

def test_selector_is_semicolon_separated():
    assert build_task_selector("history", 20) == "query/tasks?scope=history;limit=20"
    assert (build_task_selector("current", 50, before=42)
            == "query/tasks?scope=current;limit=50;before=42")


def test_selector_never_uses_ampersand():
    # MUTATION: joining with '&' (HTTP habit) collapses the tail into one zenoh
    # param, so P3 sees only scope and answers limit/before at their defaults.
    assert "&" not in build_task_selector("history", 20, before=5)


def test_before_omitted_when_none():
    assert "before" not in build_task_selector("current", 10, before=None)


# -- reply parse ---------------------------------------------------------------

class _Ok:
    def __init__(self, payload):
        self.payload = payload


class _Reply:
    def __init__(self, payload):
        self.ok = _Ok(payload)


class _ErrReply:
    ok = None            # an error reply carries no sample


def test_parse_takes_first_ok_reply():
    page = parse_get_reply(
        [_Reply(b'{"tasks":[{"task_id":"t-1"}],"has_more":false,"next_before":null}')])
    assert page["tasks"][0]["task_id"] == "t-1"


def test_parse_no_reply_is_empty_page_not_raise():
    # Queryable down / dropped selector -> empty page, NEVER an exception.
    # MUTATION: raising would 500 the panel on a pure display read.
    assert parse_get_reply([]) == {
        "tasks": [], "has_more": False, "next_before": None}


def test_parse_skips_error_reply():
    # An error reply (ok is None) is skipped; the first OK reply wins. MUTATION:
    # treating the err reply as data -> AttributeError on .payload.
    page = parse_get_reply(
        [_ErrReply(), _Reply(b'{"tasks":[],"has_more":false,"next_before":null}')])
    assert page == {"tasks": [], "has_more": False, "next_before": None}


# -- query_tasks: build + get + parse ------------------------------------------

class _FakeSession:
    def __init__(self, payload):
        self._payload = payload
        self.last_selector = None

    def get(self, selector):
        self.last_selector = selector
        return [_Reply(self._payload)]


def test_query_tasks_builds_selector_and_parses():
    sess = _FakeSession(
        b'{"tasks":[{"task_id":"t-9"}],"has_more":true,"next_before":7}')
    page = query_tasks(sess, scope="history", limit=20, before=None)
    # The selector P3 receives is the ';'-joined one.
    assert sess.last_selector == "query/tasks?scope=history;limit=20"
    assert page["tasks"][0]["task_id"] == "t-9" and page["has_more"] is True
    assert page["next_before"] == 7
