"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_teach_logs.py
Brief: P3 teach commands leave one INFO line each, with the save result (user request 2026-11-13)

Description:
A cloud recording of 4号路径 on 2026-11-13 left no trace in the P3 log: the
route appeared in geo.db and nothing said when it started, who sent it, or
what was saved. TeachRuntime now logs one line per teach command, built from
the ack it answers, plus the max-duration auto-finish. These cases pin the
lines and their cost (none per sample). Each names the mutant it kills.
"""
from __future__ import annotations  # postponed annotations

import pytest  # marks, caplog

from tests.p3_task.test_teach_runtime import _cmd, _drive, _start, rt  # noqa: F401 -- the shared fixture and helpers

pytestmark = pytest.mark.no_device  # pure logic over in-memory databases

LOGGER = "xbrain.p3.teach"  # the runtime's logger


def _lines(caplog):
    """The teach log lines, in order."""
    return [r.getMessage() for r in caplog.records if r.name == LOGGER]  # only ours


@pytest.mark.asyncio
async def test_a_recording_logs_start_finish_save_and_nothing_per_sample(rt, caplog):  # noqa: F811
    """MUTATION: drop the _log_outcome call -> no lines -> red; log each pose
    sample -> more than 3 lines -> red; drop the applied block -> no geo_id
    or length on the save line -> red."""
    with caplog.at_level("INFO", logger=LOGGER):  # capture INFO
        sid = await _start(rt)  # start
        await _drive(rt, sid, metres=2.0, t0=101.0, count=5)  # five samples
        await rt.handle(_cmd("finish", session_id=sid), now_mono_s=110.0, now_ms=2000)  # finish
        await rt.handle(_cmd("save", session_id=sid, save={"name": "4号路径"}),
                        now_mono_s=111.0, now_ms=2100)  # save
        await rt.handle(_cmd("query"), now_mono_s=112.0, now_ms=2200)  # a query: silent
    lines = _lines(caplog)  # what P3 wrote
    assert len(lines) == 3, lines  # start, finish, save -- not the samples, not the query
    assert "p3 teach start from p4_agent/local_voice -> accepted OK" in lines[0]  # who and result
    assert "kind=route" in lines[0] and ("session=%s" % sid) in lines[0]  # what was started
    assert "p3 teach finish" in lines[1] and "points=5" in lines[1]  # the finish counts
    assert "p3 teach save" in lines[2] and "name=4号路径" in lines[2]  # the save
    assert "saved=r-" in lines[2] and "total_len_m=" in lines[2] and "points=5" in lines[2]  # its result


@pytest.mark.asyncio
async def test_a_refused_command_logs_its_code_and_reason(rt, caplog):  # noqa: F811
    """MUTATION: log only accepted acks -> red."""
    await _start(rt)  # a live session
    with caplog.at_level("INFO", logger=LOGGER):  # capture INFO
        await rt.handle(_cmd("finish", session_id="ts-wrong"), now_mono_s=110.0, now_ms=2000)  # wrong id
    line = _lines(caplog)[-1]  # the refusal line
    assert "p3 teach finish" in line and "-> rejected" in line and "reason=" in line  # said why


@pytest.mark.asyncio
async def test_an_unparsable_command_is_logged_as_a_warning(rt, caplog):  # noqa: F811
    """MUTATION: drop the parse-failure log -> nothing -> red."""
    with caplog.at_level("INFO", logger=LOGGER):  # capture INFO
        await rt.handle({"cmd_id": "c-x"}, now_mono_s=100.0, now_ms=1000)  # no action
    recs = [r for r in caplog.records if r.name == LOGGER]  # ours
    assert recs and recs[-1].levelname == "WARNING" and "c-x" in recs[-1].getMessage()  # named


@pytest.mark.asyncio
async def test_the_max_duration_auto_finish_is_logged(rt, caplog):  # noqa: F811
    """MUTATION: drop the expire log -> red."""
    sid = await _start(rt)  # a live session
    await _drive(rt, sid, metres=2.0, t0=101.0, count=2)  # two points
    deadline = rt._session.deadline_mono_s  # the configured limit
    with caplog.at_level("INFO", logger=LOGGER):  # capture INFO
        assert rt.expire(deadline + 1.0) == "max_duration"  # past it
    line = _lines(caplog)[-1]  # the auto-finish line
    assert sid in line and "max duration" in line and "points=2" in line  # what it kept
