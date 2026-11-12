"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_speaker_playback_fault.py
Brief: 99 U166 -- speaker failing while online is alarm, speaker offline is fault

Description:
User ruling 2026-11-12: if the speaker malfunctions while the device is online
(its link to payload-service is up) the health event is sev alarm; if the
device is unreachable (link down, physically offline) it is fault. Recommended
detection, accepted: a TTS request payload-service fails while the link is up
counts as a malfunction -- except payload-service's 503 (device not
reachable: offline) and 409 (another mode owns the audio path), which are not
-- the next successful playback clears it, a link drop
clears it too (offline supersedes, a reconnect starts clean); no test playback
is ever made (it would make noise). Pins the three pieces end to end:
SpeakerDomain records the failure, refresh_health turns it into payload_speaker
fail "playback failed: ...", change_events files it as alarm and still files
link down as fault -- including the fail -> fail change of cause.
"""

from __future__ import annotations  # postponed annotations

import time  # monotonic ms for the domain
from pathlib import Path  # the production config

import yaml  # its arbiter block

from xbrain.p2_core.domains.hub import ArbHub  # domain-2 arbitration, as in production

from xbrain.p2_core.health.aggregate import (  # the health side
    PLAYBACK_FAILED,  # the cause marker
    HealthAggregator,  # item table
    refresh_health,  # one tick of observations
)
from xbrain.p2_core.health.change_events import (  # the event side
    HealthChangeTracker,  # baseline + hold
    health_change_event,  # sev decision
)
from xbrain.p2_core.health.items import HealthState  # states
from xbrain.p2_core.runtime.speaker_wiring import (  # the speaker
    SpeakerDomain,  # under test
    SpeakerHwError,  # what a failed TTS raises
    SpeakerWiringConfig,  # its config
)


class _Pub:  # gate publisher double
    """Swallows rt/audio/gate frames."""

    def put(self, _payload):  # ignore
        return None  # nothing to record here


class _Rt:  # RT session double
    """Hands out the one gate publisher."""

    def declare_publisher(self, _key):  # gate only
        return _Pub()  # a sink


def _speaker(fail_with=None, status=500):
    """A hub-less SpeakerDomain whose TTS fails with fail_with (HTTP status), else plays 0 ms."""
    cfg = SpeakerWiringConfig(payload_base_url="http://127.0.0.1:1",  # never called
                              tts_http_timeout_s=1.0, est_ms_per_sentence=0.0)  # no wait
    spk = SpeakerDomain(cfg, _Rt(), lambda: int(time.monotonic() * 1000))  # no hub, no mic

    def tts(_text):  # replaces the HTTP call
        if spk._next_fail:  # scripted failure
            raise SpeakerHwError(spk._next_fail, status=spk._status)  # as tts_client errors surface
        return 0.0  # played instantly

    spk._next_fail = fail_with  # what the next call does
    spk._status = status  # its HTTP status (None = no answer)
    spk._invoke_tts = tts  # no HTTP
    return spk  # ready


def test_a_failed_tts_is_remembered_until_one_plays_or_the_link_drops():
    """MUTATION: never set on failure -> no alarm ever -> red; not cleared on
    success -> a recovered speaker stays faulty -> red; clear_playback_fault a
    no-op -> a reconnect replays the outage as a playback failure -> red."""
    spk = _speaker("http 500")  # the device answers but cannot play
    assert spk.playback_fault() is None  # nothing tried yet
    assert spk.handle_speak("hello")["ok"] is False  # the request failed
    assert spk.playback_fault() == "http 500"  # remembered
    spk._next_fail = None  # the device works again
    assert spk.handle_speak("hello")["ok"] is True  # played
    assert spk.playback_fault() is None  # cleared by the success
    spk._next_fail = "timeout"  # fails once more
    spk.handle_speak("hello")  # failed
    spk.clear_playback_fault()  # p2 saw the link go down
    assert spk.playback_fault() is None  # offline supersedes


def test_the_arbitrated_path_production_uses_records_and_clears_too():
    """p2 builds its SpeakerDomain WITH the arbitration hub (main_wiring), which
    takes _speak_arbitrated, not the hub-less branch above. MUTATION: drop the
    note / the clear in that branch -> red."""
    root = Path(__file__).resolve().parents[3]  # repo root
    arb = yaml.safe_load((root / "configs" / "p2_core.yaml").read_text(  # production arbiter
        encoding="utf-8"))["arbiter"]  # block
    cfg = SpeakerWiringConfig(payload_base_url="http://127.0.0.1:1",  # never called
                              tts_http_timeout_s=1.0, est_ms_per_sentence=0.0)  # no wait
    spk = SpeakerDomain(cfg, _Rt(), lambda: int(time.monotonic() * 1000),  # with a hub
                        hub=ArbHub(arb))  # as in main_wiring
    state = {"fail": "http 500"}  # scripted

    def tts(_text):  # replaces the HTTP call
        if state["fail"]:  # failing
            raise SpeakerHwError(state["fail"], status=500)  # not 503 / 409
        return 0.0  # played instantly

    spk._invoke_tts = tts  # no HTTP
    assert spk.handle_speak("hello", source="tts_local")["ok"] is False  # failed
    assert spk.playback_fault() == "http 500"  # remembered
    state["fail"] = None  # works again
    assert spk.handle_speak("hello", source="tts_local")["ok"] is True  # played
    assert spk.playback_fault() is None  # cleared


def test_offline_and_mode_conflict_replies_are_not_a_malfunction():
    """payload-service POST /tts: 503 = device not reachable (offline, the link
    poll reports it as fault), 409 = another mode owns the audio path. Found
    live 2026-11-12: the boot greeting's 503 was filed as an online failure.
    MUTATION: drop the NOT_A_SPEAKER_FAULT check -> both remembered -> red;
    treat a no-answer (status None) as offline -> a hung service never alarms
    -> red."""
    for status in (503, 409):  # offline, busy
        spk = _speaker("tts returned %d" % status, status=status)  # scripted
        spk.handle_speak("hello")  # fails
        assert spk.playback_fault() is None  # not a malfunction
    hung = _speaker("tts request failed: timeout", status=None)  # no HTTP answer
    hung.handle_speak("hello")  # fails
    assert hung.playback_fault() == "tts request failed: timeout"  # online, cannot play


def _detail(agg):
    """payload_speaker's (state, detail) after refresh_health."""
    e = agg.entries["payload_speaker"]  # the item
    return e.state, e.detail  # what build_summary publishes


def test_refresh_health_tells_online_failure_from_offline():
    """MUTATION: ignore speaker_playback_fail -> online failure reads ok -> red;
    apply it while the link is down -> offline reported as a playback failure
    -> red."""
    agg = HealthAggregator()  # fresh table
    refresh_health(agg, {}, now_mono_s=1.0, device_states={"payload_speaker": True},  # online
                   speaker_playback_fail="http 500")  # but cannot play
    assert _detail(agg) == (HealthState.FAIL, PLAYBACK_FAILED + ": http 500")  # function fail
    refresh_health(agg, {}, now_mono_s=2.0, device_states={"payload_speaker": False},  # offline
                   speaker_playback_fail="http 500")  # stale flag ignored
    assert _detail(agg) == (HealthState.FAIL, "link down")  # offline wins
    refresh_health(agg, {}, now_mono_s=3.0, device_states={"payload_speaker": True},  # back
                   speaker_playback_fail=None)  # and playing
    assert _detail(agg) == (HealthState.OK, "link up")  # healthy
    # only the speaker: the siren shares the audio link but is not the speaker
    refresh_health(agg, {}, now_mono_s=4.0, device_states={"payload_siren": True},  # siren up
                   speaker_playback_fail="http 500")  # speaker flag
    assert agg.entries["payload_siren"].state == HealthState.OK  # untouched


def _item(state, detail):
    """One summary item as change_events sees it."""
    return {"payload_speaker": {"state": state, "level": "degraded", "detail": detail}}  # body


def test_online_failure_is_alarm_offline_is_fault_and_a_cause_change_reports():
    """MUTATION: playback fail filed as fault -> red; link down filed as alarm
    -> red; fail -> fail with a new cause swallowed (state-only tracking) ->
    the pulled cable never reports -> red."""
    t = HealthChangeTracker(0.0)  # no hold: judge the rule alone
    t.observe(_item("ok", "link up"), 0.0)  # baseline
    a = t.observe(_item("fail", PLAYBACK_FAILED + ": http 500"), 1.0)  # online, cannot play
    assert len(a) == 1 and health_change_event(a[0], eid="e", ts=1.0)["sev"] == "alarm"  # U166
    b = t.observe(_item("fail", "link down"), 2.0)  # then the cable is pulled
    assert len(b) == 1 and (b[0]["from"], b[0]["to"]) == ("fail", "fail")  # a change of cause
    ev = health_change_event(b[0], eid="e", ts=2.0)  # the event
    assert ev["sev"] == "fault"  # offline
    assert ev["detail"]["from_reason"].startswith(PLAYBACK_FAILED)  # what it was before
    assert t.observe(_item("fail", "link down"), 3.0) == []  # steady: nothing more
    c = t.observe(_item("ok", "link up"), 4.0)  # recovered
    assert health_change_event(c[0], eid="e", ts=4.0)["sev"] == "warn"  # recovery is warn
