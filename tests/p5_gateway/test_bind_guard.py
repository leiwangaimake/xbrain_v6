"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_bind_guard.py
Brief: GWY-P5-17 static guards -- 0.0.0.0 ban, forbidden segments, unassigned
       refusal, pending-keys hatch, retention shape; each with its mutant

Description:
Covers the static half of GWY-P5-17 (the runtime `ss -lntup` + per-segment
connect probe is CR-NET-1's script and lands with the Phase 2 process; the
guard's docstring states that boundary). The committed p5_gateway.yaml today is
shape-clean but carries null binds (LAN2 pending U-15) -- so the REAL file's
expected outcome is a refusal that names the unassigned keys, which is the
designed unfilled-config behaviour, and the tests pin exactly that.
"""

import copy
import os

import pytest
import yaml

from xbrain.p5_gateway.config import (
    P5ConfigError, PENDING_KEYS_ALLOWED, check_p5_config,
)

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
P5_YAML = os.path.join(ROOT, "configs", "p5_gateway.yaml")


def real_mapping():
    """The committed p5_gateway.yaml, parsed."""
    with open(P5_YAML, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def assigned():
    """The real mapping with the pending binds filled with PLAUSIBLE addresses,
    so the clean-pass path is testable before U-15 lands. 10.99.x is none of the
    registered segments (11 S1.1.9.2), standing in for the未定 LAN2."""
    m = copy.deepcopy(real_mapping())
    m["hmi"]["bind"] = ["10.99.0.2:8080", "192.168.1.50:8080", "127.0.0.1:8080"]
    m["delivery"]["ftp"]["listen_address"] = ["10.99.0.2", "192.168.1.50"]
    return m


# --------------------------------------------------------------------------
# the committed file
# --------------------------------------------------------------------------

def test_real_file_refuses_naming_the_unassigned_binds():
    """*** Today's truth: the file is shape-clean but LAN2/wifi binds are null
    (U-15 / deployment pending), so startup must refuse AND name every
    unassigned key path in one message -- the designed unfilled-config
    behaviour, not an error in the file."""
    with pytest.raises(P5ConfigError) as err:
        check_p5_config(real_mapping())
    msg = str(err.value)
    assert "unassigned" in msg
    assert "hmi.bind[0]" in msg and "hmi.bind[1]" in msg
    assert "delivery.ftp.listen_address[0]" in msg


def test_real_file_has_no_forbidden_shape():
    """The refusal above must be ONLY about unassigned entries: with the nulls
    filled with plausible addresses, the whole file passes -- i.e. nothing else
    in the committed yaml trips a guard."""
    check_p5_config(assigned())                      # must not raise


def test_real_pending_keys_are_exactly_the_three():
    """The committed escape hatch carries the three CR-EVT-1 keys, no more."""
    assert real_mapping()["startup"]["pending_keys"] == list(PENDING_KEYS_ALLOWED)


# --------------------------------------------------------------------------
# the mutations
# --------------------------------------------------------------------------

def test_bind_0000_is_refused_with_the_consequence_named():
    """*** Criterion 2's mutation (static half): hmi.bind back to 0.0.0.0:8080
    must refuse -- a taken-over camera on LAN4 could press estop otherwise."""
    m = assigned()
    m["hmi"]["bind"][0] = "0.0.0.0:8080"
    with pytest.raises(P5ConfigError, match="0.0.0.0"):
        check_p5_config(m)


def test_ftp_0000_is_refused_too():
    """Same rule, same reason, on the vsftpd list (17 S5.5 COM-43)."""
    m = assigned()
    m["delivery"]["ftp"]["listen_address"][0] = "0.0.0.0"
    with pytest.raises(P5ConfigError, match="0.0.0.0"):
        check_p5_config(m)


@pytest.mark.parametrize("addr", [
    "10.21.33.5:8080",        # LAN1 chassis (candidate range, V-15)
    "10.21.31.5:8080",        # LAN1 chassis (other candidate)
    "192.168.144.38:8080",    # LAN3 GZH-2 itself
    "192.168.66.13:8080",     # LAN4 the visible-light camera
])
def test_forbidden_segment_bind_is_refused(addr):
    """*** A bind on LAN1/LAN3/LAN4 must refuse even without 0.0.0.0 -- listing
    a device segment explicitly is the same exposure, spelled politely."""
    m = assigned()
    m["hmi"]["bind"][0] = addr
    with pytest.raises(P5ConfigError, match="forbidden segment"):
        check_p5_config(m)


def test_smuggled_pending_key_is_refused():
    """*** A fourth key in startup.pending_keys is an unregistered key riding
    the escape hatch (17 S3.5.6: 对且仅对这三条放行)."""
    m = assigned()
    m["startup"]["pending_keys"].append("event/side_channel")
    with pytest.raises(P5ConfigError, match="side_channel"):
        check_p5_config(m)


def test_empty_pending_keys_is_legal():
    """Post-CR-EVT-1 state: an empty list must pass -- the regression the doc
    asks for once the three keys are registered."""
    m = assigned()
    m["startup"]["pending_keys"] = []
    check_p5_config(m)                               # must not raise


def test_dict_retention_days_is_refused():
    """*** Criterion 5's mutation: the struck v0.1 {info: 7, warn: 30, ...}
    shape must refuse -- per-severity retention re-opens the split 17 S10.1
    removed."""
    m = assigned()
    m["event"]["retention_days"] = {"info": 7, "warn": 30}
    with pytest.raises(P5ConfigError, match="per-severity"):
        check_p5_config(m)


def test_missing_bind_list_is_refused_not_defaulted():
    """No declared binds must refuse: a gateway falling back to a library
    default would listen on exactly the 0.0.0.0 this file bans."""
    m = assigned()
    del m["hmi"]["bind"]
    with pytest.raises(P5ConfigError, match="hmi.bind"):
        check_p5_config(m)


def test_empty_skeleton_is_refused():
    """A comment-only yaml parses to None; same refusal as a missing file."""
    with pytest.raises(P5ConfigError):
        check_p5_config(None)
