"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: test_web_server_skeleton.py
Brief: Guards the HMI web server's two safety red lines -- NET-C9 bind + no fake pose

Description:
The HMI web server (17 S6.10) is an ESTOP-capable control surface, so two of its
behaviours are safety-relevant and must not silently regress:

  1. NET-C9 (17 S6.10.3 / 10 S4.5): it must NEVER bind 0.0.0.0. A wildcard bind
     on this surface exposes unauthenticated estop/mode/ptz to the whole network.

  2. 3.1/3.2 fail-silent: with no pose source (perception/rtk_driver/chassis all
     GATED, 17 S6.10.4) the snapshot must report the pose as unavailable with
     null fields, NEVER a zeroed pose the map would plot as a real (0,0) fix.

Each test carries the mutation that would turn it red (3.3): drop the wildcard
guard and 0.0.0.0 is accepted; default the pose to 0 and the no-source snapshot
stops reporting unavailable.
"""

from __future__ import annotations

import pytest

from xbrain.p5_gateway.hmi import data_readers as D
from xbrain.p5_gateway.hmi.ui_config import UiConfigError, build_ui_config
from xbrain.p5_gateway.hmi.web_server import (
    HmiBindError, make_bound_sockets, parse_bind_entry,
)


# -- NET-C9: never bind 0.0.0.0 ---------------------------------------------

@pytest.mark.parametrize("wildcard", ["0.0.0.0:18083", "::" + ":18083", ":18083"])
def test_wildcard_bind_is_refused(wildcard):
    """MUTATION (guards NET-C9): remove the wildcard check in parse_bind_entry
    and one of these is accepted -> an ESTOP surface bound wide. Must raise."""
    with pytest.raises(HmiBindError):
        parse_bind_entry(wildcard)


def test_all_null_bind_refuses_to_start():
    """A bind list with no interface must refuse (an HMI reachable from nowhere
    is a deploy bug the operator must see, not a silent wildcard fallback)."""
    with pytest.raises(HmiBindError):
        make_bound_sockets([None, None])


def test_explicit_interface_binds_that_exact_address():
    """A concrete host binds to THAT host, never widened. Port 0 lets the OS
    pick so the test needs no fixed free port; the assertion is on the host."""
    socks = make_bound_sockets([None, "127.0.0.1:0"])
    try:
        assert socks[0].getsockname()[0] == "127.0.0.1"
    finally:
        for s in socks:
            s.close()


# -- 3.1/3.2: no source -> no fabricated pose --------------------------------

def test_snapshot_without_pose_reports_unavailable_not_zero():
    """MUTATION (guards fail-silent): default pose lat/lon to 0.0 and this fails.
    With no pose source the map must be told 'no fix', not handed (0,0)."""
    snap = D.build_snapshot()          # no sources wired
    assert snap["pose"]["available"] is False
    assert snap["pose"]["lat"] is None
    assert snap["pose"]["fix_type"] is None
    # geo layers absent -> available False (grey the layer), not empty-as-real.
    assert snap["geo"]["fences"]["available"] is False


def test_progress_without_total_is_not_a_fabricated_fraction():
    """A task with no expanded route (total_steps None) must yield total None,
    never a made-up 2/3 (17 S6.10.4). MUTATION: coerce total to done and fail."""
    snap = D.build_snapshot(tasks=[{"task_id": "t-1", "state": "running",
                                    "total_steps": None, "current_step": 0}])
    assert snap["plan"]["plans"][0]["progress"] == {"done": 0, "total": None}


def test_located_nowhere_event_has_no_map_dot():
    """An event whose pose was never stamped keeps pos None -> stream only, no
    map dot (do not plot at 0,0). MUTATION: pass pos through and this fails."""
    snap = D.build_snapshot(events=[{"eid": "e-1", "title": "报警", "sev": "alarm",
                                     "pos": {"lat": None, "lon": None}}])
    assert snap["events"]["items"][0]["pos"] is None


# -- ui_config: malformed config refuses, does not default -------------------

def test_ui_config_refuses_missing_group():
    """A hmi.web subtree missing a presentation group is a config defect, not a
    silent browser-default fallback. MUTATION: return {} on missing and fail."""
    with pytest.raises(UiConfigError):
        build_ui_config({"map": {}})       # missing font/layout/fence/route/waypoint


_FULL_WEB = {"map": {}, "font": {}, "layout": {},
             "fence": {}, "route": {}, "waypoint": {}}


def test_ui_config_forwards_site_timezone():
    """The site tz reaches the frontend via ui_config (once-loaded), so the
    footer clock ticks in the site zone. MUTATION: drop the timezone key and the
    footer clock silently uses the operator's browser zone -> this fails."""
    cfg = build_ui_config(_FULL_WEB, site_timezone="Asia/Tokyo")
    assert cfg["timezone"] == "Asia/Tokyo"


def test_ui_config_timezone_defaults_none():
    """No site tz (minimal mode) -> timezone None, and the frontend falls back to
    the browser zone rather than blocking the page (DISPLAY value, not hmi.bind)."""
    assert build_ui_config(_FULL_WEB)["timezone"] is None


# -- OpenAPI schema builds (JSONResponse routes must carry response_model=None) --

def test_openapi_schema_builds(tmp_path):
    """FastAPI builds the OpenAPI schema by inspecting each route's return
    annotation. A route annotated `-> JSONResponse` (a Response subclass) WITHOUT
    response_model=None makes app.openapi() raise PydanticUserError -- with
    `from __future__ import annotations` the annotation is the unresolved string
    'JSONResponse' -- and /openapi.json 500s. MUTATION: drop response_model=None
    on /api/fences (or /api/fences/active) and app.openapi() raises here.
    build_app never calls the provider during construction/schema-gen (the routes
    only reference it inside their bodies), so a bare stub provider is enough."""
    from xbrain.p5_gateway.hmi.web_server import build_app

    class _StubProvider:      # schema gen inspects signatures, never calls these
        pass

    app = build_app(_FULL_WEB, _StubProvider(), lambda: None, str(tmp_path))
    schema = app.openapi()    # raises if any route return-annotation is unresolved
    assert "/api/fences" in schema["paths"]
    assert "/api/fences/active" in schema["paths"]
