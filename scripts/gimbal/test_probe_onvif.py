"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: test_probe_onvif.py
Brief: Tests for the ONVIF probe's parsing, against the two vendors' real reply shapes.

Description:
  The probe's only job is to read facts off a camera correctly, so what is worth testing
  is the parsing -- and specifically the places where the TWO cameras in the 布控球 differ,
  because those are where a scrape that works on one silently returns nothing on the other.

  The fixtures below are trimmed from responses the real devices sent. That matters more
  than it sounds: the first version of the tag scrape used \\w+ for the namespace prefix,
  which cannot match the visible-light camera's "SOAP-ENV:" because a hyphen is not a word
  character. Every field came back empty and the probe reported a bare XML header instead
  of "Sender not Authorized". Invented fixtures would have used a tidy prefix and passed.
"""
from __future__ import annotations

import json
from datetime import datetime, timezone

import pytest

from scripts.gimbal.probe_onvif import (
    OnvifError,
    PtzSpace,
    _at,
    _attribute_values,
    _explain_rejection,
    _fault_tail,
    _first,
    _wsse_header,
    load_credentials,
    tag_values,
)

# Trimmed from the visible-light camera. Note the SOAP-ENV prefix and the 1.1-style
# faultstring carried inside a 1.2 envelope -- both are what this vendor really sends.
_HYPHEN_PREFIX_FAULT = (
    '<?xml version="1.0" encoding="UTF-8"?>'
    '<SOAP-ENV:Envelope xmlns:SOAP-ENV="http://www.w3.org/2003/05/soap-envelope" '
    'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">'
    "<SOAP-ENV:Body><SOAP-ENV:Fault><SOAP-ENV:Code>"
    "<SOAP-ENV:Value>SOAP-ENV:Sender</SOAP-ENV:Value></SOAP-ENV:Code>"
    "<SOAP-ENV:Reason><SOAP-ENV:Text>Sender not Authorized</SOAP-ENV:Text>"
    "</SOAP-ENV:Reason></SOAP-ENV:Fault></SOAP-ENV:Body></SOAP-ENV:Envelope>"
)
# Trimmed from the thermal camera: a plain "s:"/"tds:" prefix pair, no hyphens.
_PLAIN_PREFIX_FAULT = (
    '<?xml version="1.0" encoding="utf-8"?>'
    '<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
    'xmlns:ter="http://www.onvif.org/ver10/error">'
    "<s:Body><s:Fault><s:Code><s:Value>s:Sender</s:Value>"
    "<s:Subcode><s:Value>ter:NotAuthorized</s:Value></s:Subcode></s:Code>"
    "</s:Fault></s:Body></s:Envelope>"
)


def test_a_hyphenated_namespace_prefix_is_still_matched():
    """SOAP-ENV: is a legal prefix and one of our two cameras uses it."""
    assert tag_values(_HYPHEN_PREFIX_FAULT, "Text") == ["Sender not Authorized"]


def test_a_plain_namespace_prefix_is_still_matched():
    """The other camera uses s:, so the same scrape has to serve both."""
    assert tag_values(_PLAIN_PREFIX_FAULT, "Value") == ["s:Sender", "ter:NotAuthorized"]


def test_a_tag_the_device_omitted_reads_as_a_placeholder_not_a_crash():
    """Some firmwares leave the serial blank; a probe must still report the rest."""
    assert _first("<tds:Model>X</tds:Model>", "SerialNumber") == "-"
    assert _first("<tds:Model> X </tds:Model>", "Model") == "X"


def test_an_unrecognised_fault_is_shown_from_the_fault_element_not_the_top():
    """Cutting from the front of a SOAP body shows only xmlns declarations.

    This is the reason the operator saw an XML header instead of a reason for three runs.
    """
    tail = _fault_tail(_HYPHEN_PREFIX_FAULT)
    assert tail.startswith("<SOAP-ENV:Fault")
    assert "Sender not Authorized" in tail


def test_a_body_with_no_fault_element_falls_back_to_its_start():
    """A non-fault error body still has to produce something readable."""
    assert _fault_tail("<html>gateway timeout</html>").startswith("<html>")


def test_repeated_profile_tokens_collapse_but_keep_document_order():
    """Both vendors repeat the token on nested elements; the first is the main stream."""
    listing = '<Profiles token="MainStream"><X token="MainStream"/></Profiles><Profiles token="SubStream"/>'
    assert _attribute_values(listing, "token") == ["MainStream", "SubStream"]


@pytest.mark.parametrize(
    "values,index,expected",
    [(["a", "b"], 0, "a"), (["a", "b"], 1, "b"), (["a"], 5, "-"), ([], 0, "-")],
)
def test_a_short_scraped_list_yields_a_placeholder_rather_than_indexerror(values, index, expected):
    """Encoding and resolution are scraped independently of the token list.

    A device that omits one for a single profile would otherwise lose the whole report.
    """
    assert _at(values, index) == expected


def test_the_security_header_never_carries_the_password_in_the_clear():
    """The digest is the point: the LAN carrying this is shared with the payload."""
    header = _wsse_header("admin", "hunter2")
    assert "hunter2" not in header
    assert "PasswordDigest" in header
    assert "<Nonce" in header and "<Created" in header


def test_two_headers_for_the_same_password_differ():
    """A fresh nonce per request is what stops a captured header being replayed."""
    assert _wsse_header("admin", "same") != _wsse_header("admin", "same")


def test_a_ptz_space_keeps_the_reported_bounds_verbatim():
    """Bounds are printed for a human; parsing them to float would invent precision."""
    space = PtzSpace(uri="http://x/PanTiltSpace", x_min="-1", x_max="1", y_min="-1", y_max="1")
    assert (space.x_min, space.x_max) == ("-1", "1")


def test_each_host_gets_its_own_credential(tmp_path):
    """The two cameras are different vendors with independently set passwords."""
    path = tmp_path / "creds.json"
    path.write_text(json.dumps({
        "192.168.1.13": {"user": "admin", "password": "one"},
        "192.168.1.108": {"user": "onvif", "password": "two"},
    }))
    assert load_credentials(path, "192.168.1.13") == ("admin", "one")
    assert load_credentials(path, "192.168.1.108") == ("onvif", "two")


def test_an_unknown_host_is_refused_rather_than_probed_with_someone_elses_password(tmp_path):
    """Falling back to another host's credential would spend a failed attempt for nothing."""
    path = tmp_path / "creds.json"
    path.write_text(json.dumps({"192.168.1.13": {"user": "admin", "password": "one"}}))
    with pytest.raises(OnvifError, match="192.168.1.99"):
        load_credentials(path, "192.168.1.99")


def test_a_missing_credentials_file_says_what_the_file_should_contain(tmp_path):
    """The operator meets this error first; it has to be self-explanatory."""
    with pytest.raises(OnvifError, match="password"):
        load_credentials(tmp_path / "absent.json", "192.168.1.13")


def test_a_world_readable_credentials_file_is_reported(tmp_path, capsys):
    """A warning, not a refusal -- see the rationale in load_credentials."""
    path = tmp_path / "creds.json"
    path.write_text(json.dumps({"h": {"user": "u", "password": "p"}}))
    path.chmod(0o644)
    load_credentials(path, "h")
    assert "readable by other users" in capsys.readouterr().err


def test_the_password_is_never_part_of_the_command_line(tmp_path):
    """argv is world readable through /proc on the shared Orin."""
    from scripts.gimbal.probe_onvif import main
    with pytest.raises(SystemExit):
        main(["--host", "1.2.3.4", "--password", "secret"])


def _time_reply(year=2026, month=7, day=28, hour=12, minute=0, second=0):
    """A GetSystemDateAndTimeResponse in the shape the real cameras send."""
    return (
        "<tds:GetSystemDateAndTimeResponse><tds:SystemDateAndTime>"
        "<tt:UTCDateTime>"
        f"<tt:Time><tt:Hour>{hour}</tt:Hour><tt:Minute>{minute}</tt:Minute>"
        f"<tt:Second>{second}</tt:Second></tt:Time>"
        f"<tt:Date><tt:Year>{year}</tt:Year><tt:Month>{month}</tt:Month>"
        f"<tt:Day>{day}</tt:Day></tt:Date>"
        "</tt:UTCDateTime></tds:SystemDateAndTime></tds:GetSystemDateAndTimeResponse>"
    )


def test_clock_skew_is_measured_against_the_cameras_utc_block(monkeypatch):
    """Skew is what separates a wrong password from a drifted clock."""
    import scripts.gimbal.probe_onvif as probe
    now = datetime.now(timezone.utc)
    monkeypatch.setattr(probe, "soap_call", lambda *a, **k: _time_reply(
        now.year, now.month, now.day, now.hour, now.minute, now.second))
    assert abs(probe.camera_clock_skew("1.2.3.4")) < 5


def test_a_camera_that_will_not_report_its_clock_does_not_abort_the_probe(monkeypatch):
    """Losing one diagnostic must not cost the whole report."""
    import scripts.gimbal.probe_onvif as probe
    monkeypatch.setattr(probe, "soap_call", lambda *a, **k: "<tds:Nothing/>")
    assert probe.camera_clock_skew("1.2.3.4") is None


def test_an_impossible_clock_reads_as_unknown_rather_than_crashing(monkeypatch):
    """An unset clock can report month 0; that is a finding, not a traceback."""
    import scripts.gimbal.probe_onvif as probe
    monkeypatch.setattr(probe, "soap_call", lambda *a, **k: _time_reply(month=0))
    assert probe.camera_clock_skew("1.2.3.4") is None


def test_rtsp_accepting_the_credential_tells_the_operator_to_stop_trying_passwords():
    """This is the branch that prevents a pointless lockout."""
    text = _explain_rejection(skew=1.0, rtsp_ok=True)
    assert "password is RIGHT" in text
    assert "Do NOT try other passwords" in text


def test_rtsp_rejecting_the_credential_too_is_reported_as_a_real_wrong_password():
    """Two independent services on one user list agreeing is actual evidence."""
    text = _explain_rejection(skew=1.0, rtsp_ok=False)
    assert "really is wrong" in text


def test_no_rtsp_verdict_is_reported_as_unconfirmed_rather_than_guessed():
    """Absence of evidence must not be dressed up as a conclusion."""
    assert "unconfirmed" in _explain_rejection(skew=1.0, rtsp_ok=None)


def test_a_large_skew_is_named_as_the_likely_cause_ahead_of_the_password():
    """The whole point of measuring it is to reorder what the operator tries first."""
    text = _explain_rejection(skew=4000.0, rtsp_ok=None)
    assert "LIKELY THE CAUSE" in text


def test_the_error_type_is_distinct_from_a_transport_failure():
    """"The camera said no" and "the network is broken" need different fixes."""
    assert issubclass(OnvifError, Exception)
    assert not issubclass(OnvifError, OSError)
