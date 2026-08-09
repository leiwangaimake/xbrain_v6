"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_domains.py
Brief: domains tests -- domains

Description:
BIZ-P2-5..10 -- domain arbiters + LAPI guard + auto lighting tests.
"""


from pathlib import Path

import pytest
import yaml

from xbrain.p2_core.domains import factory, lapi_guard, lighting_auto


pytestmark = pytest.mark.no_device


REPO = Path(__file__).parent.parent.parent.parent
P2_YAML = REPO / "configs" / "p2_core.yaml"


@pytest.fixture()
def arbiter_yaml() -> dict:
    doc = yaml.safe_load(P2_YAML.read_text())
    return doc["arbiter"]


# --- factory.build_all builds all 4 domains -----------------------

def test_build_all_returns_four_domain_arbiters(arbiter_yaml):
    arbs = factory.build_all(arbiter_yaml)
    assert set(arbs.keys()) == {"speaker", "asr", "payload_light", "ptz"}


def test_speaker_arbiter_has_seven_registered_sources(arbiter_yaml):
    """BIZ-P2-5 domain 2. 14 S4.1 lists exactly 7 sources."""
    arbs = factory.build_all(arbiter_yaml)
    src_names = {s.source_id for s in arbs["speaker"].sources()}
    assert src_names == {
        "alarm_d", "broadcast_b",
        "tts_cloud", "tts_wecom", "tts_local",
        "prompt_tone", "bit_announce",
    }


def test_asr_arbiter_has_three_sources(arbiter_yaml):
    """BIZ-P2-6 domain 3. 14 S4.2 lists exactly 3 sources."""
    arbs = factory.build_all(arbiter_yaml)
    src_names = {s.source_id for s in arbs["asr"].sources()}
    assert src_names == {"asr_cloud", "asr_wecom", "asr_local"}


def test_payload_light_has_two_sources_with_correct_lease(arbiter_yaml):
    """BIZ-P2-7 domain 4. 14 S4.3 sources: mode_driver (900, resident)
    and manual (500, reject, 30s lease)."""
    arbs = factory.build_all(arbiter_yaml)
    specs = {s.source_id: s for s in arbs["payload_light"].sources()}
    assert "mode_driver" in specs
    assert "manual" in specs
    # These are SourceSnapshot; we know the shape from arbiter.model.
    # Detailed lease/policy check is via the YAML block directly.
    pl_yaml = arbiter_yaml["domains"]["payload_light"]["sources"]
    assert pl_yaml["mode_driver"]["lease_timeout_s"] is None   # resident
    assert pl_yaml["mode_driver"]["policy"] == "immediate"
    assert pl_yaml["manual"]["policy"] == "reject"
    assert pl_yaml["manual"]["lease_timeout_s"] == 30.0


def test_ptz_has_three_sources(arbiter_yaml):
    """BIZ-P2-9 domain 5. 14 S4.4 sources: manual_cloud/auto_track/preset_patrol."""
    arbs = factory.build_all(arbiter_yaml)
    src_names = {s.source_id for s in arbs["ptz"].sources()}
    assert src_names == {"manual_cloud", "auto_track", "preset_patrol"}


# --- variant: bad policy value in YAML -> DomainConfigError -------

def test_build_all_rejects_off_contract_policy_value(arbiter_yaml):
    """Corrupt the YAML in-memory with an invalid policy string -> raise."""
    corrupt = yaml.safe_load(yaml.safe_dump(arbiter_yaml))
    corrupt["domains"]["asr"]["sources"]["asr_local"]["policy"] = "magic"
    with pytest.raises(factory.DomainConfigError) as ei:
        factory.build_all(corrupt)
    assert "asr_local" in str(ei.value)
    assert "policy" in str(ei.value)


def test_build_all_rejects_missing_domain(arbiter_yaml):
    corrupt = yaml.safe_load(yaml.safe_dump(arbiter_yaml))
    del corrupt["domains"]["ptz"]
    with pytest.raises(factory.DomainConfigError) as ei:
        factory.build_all(corrupt)
    assert "ptz" in str(ei.value)


# --- LAPI guard (BIZ-P2-10) ----------------------------------------

def test_lapi_guard_accepts_two_authorised_keys():
    for k in ("FocusMode", "ShieldTrigger.MovePTZ"):
        lapi_guard.check_write_key(k)   # must not raise


def test_lapi_guard_rejects_bitrate_write():
    """VARIANT (spec verbatim): bitrate=16384 or 6144 write must be
    caught -- that's the removed PTZ boost being silently reintroduced."""
    for bad in ("VideoBitrate", "bitrate"):
        with pytest.raises(lapi_guard.LapiWriteViolation) as ei:
            lapi_guard.check_write_key(bad)
        assert "boost" in str(ei.value).lower() or bad in str(ei.value)


def test_lapi_guard_rejects_arbitrary_unknown_key():
    with pytest.raises(lapi_guard.LapiWriteViolation):
        lapi_guard.check_write_key("SomeNewSetting")


def test_lapi_batch_check_reports_all_bad():
    bad = lapi_guard.check_batch([
        "FocusMode", "bitrate", "ShieldTrigger.MovePTZ", "I_interval",
    ])
    assert set(bad) == {"bitrate", "I_interval"}


# --- Auto lighting (BIZ-P2-8) --------------------------------------

def _lighting_inputs(**over):
    d = dict(
        photocell_lux=None,
        image_lux_equiv=None,
        almanac_sun_elev_deg=None,
        on_lux_equiv=None,
        off_lux_equiv=None,
        night_on_sun_elev_deg=-6.0,
        night_off_sun_elev_deg=-3.0,
        redblue_strobe_active=False,
        currently_on=False,
    )
    d.update(over)
    return lighting_auto.LightingInputs(**d)


def test_lighting_fail_safe_all_sources_unavailable():
    """A-6: all sources unavailable -> ON. Fail-safe direction."""
    assert lighting_auto.decide_light_effective(_lighting_inputs()) is True


def test_lighting_almanac_below_night_on_returns_true():
    """Sun well below horizon -> dark -> on."""
    inp = _lighting_inputs(almanac_sun_elev_deg=-30.0)
    assert lighting_auto.decide_light_effective(inp) is True


def test_lighting_almanac_above_night_off_returns_false():
    """Sun above -3 deg -> not dark -> off (from not-currently-on)."""
    inp = _lighting_inputs(almanac_sun_elev_deg=10.0)
    assert lighting_auto.decide_light_effective(inp) is False


def test_lighting_hysteresis_stays_on_across_off_threshold_only_when_crossed():
    """A-3: with currently_on=True, staying on until value crosses
    off_thresh in the light direction. Photocell case."""
    # currently_on=True; on=100, off=300; value=200 (between).
    # Since currently_on and value <= off (200 <= 300), stay on.
    inp = _lighting_inputs(
        photocell_lux=200.0,
        on_lux_equiv=100.0, off_lux_equiv=300.0,
        currently_on=True,
    )
    assert lighting_auto.decide_light_effective(inp) is True


def test_lighting_a7_strobe_active_forces_on_when_any_source_dark():
    """A-7: red/blue strobe ON + any source judges dark -> on,
    regardless of chain order."""
    inp = _lighting_inputs(
        almanac_sun_elev_deg=-30.0,
        redblue_strobe_active=True,
    )
    assert lighting_auto.decide_light_effective(inp) is True


def test_lighting_photocell_beats_image_when_both_available():
    """A-1 chain order: photocell first. If photocell says light,
    image saying dark is ignored (photocell wins the chain)."""
    inp = _lighting_inputs(
        # photocell reads bright (well above off_thresh, so off).
        photocell_lux=500.0,
        image_lux_equiv=10.0,   # image says dark
        on_lux_equiv=100.0, off_lux_equiv=300.0,
        currently_on=False,
    )
    # photocell current=500, on=100, off=300, not currently_on ->
    # 500 > 100 (only turn on if value <= on), so light stays OFF.
    assert lighting_auto.decide_light_effective(inp) is False
