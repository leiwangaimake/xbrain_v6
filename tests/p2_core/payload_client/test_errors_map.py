"""BIZ-P2-2 -- payload-service error map tests + spec variant."""

import pytest

from xbrain.p2_core.payload_client import errors_map as em


pytestmark = pytest.mark.no_device


# --- 2xx returns None (success) ------------------------------------

@pytest.mark.parametrize("code", [200, 201, 202, 204])
def test_2xx_returns_none(code):
    assert em.map_http_status(code) is None


# --- 400/422 -> E_SCHEMA -------------------------------------------

@pytest.mark.parametrize("code", [400, 422])
def test_400_422_map_to_e_schema(code):
    err = em.map_http_status(code)
    assert err.code == em.E_SCHEMA
    assert err.reason == "schema"


# --- 409 -> E_BUSY with payload_mode --------------------------------

def test_409_maps_to_busy_with_payload_mode():
    err = em.map_http_status(409, body={"payload_mode": "deter"})
    assert err.code == em.E_BUSY
    assert err.reason == "payload_mode"
    assert err.detail["payload_mode"] == "deter"


# --- 503 -> E_UNHEALTHY device_link_down + affects payload_* -------

def test_503_maps_to_unhealthy_device_link_down():
    err = em.map_http_status(503)
    assert err.code == em.E_UNHEALTHY
    assert err.reason == "device_link_down"
    # payload_* items should be marked as affected.
    assert "payload_speaker" in err.detail["affects"]


# --- connect refused -> E_UNHEALTHY service_down -------------------

def test_connect_error_maps_to_service_down():
    err = em.map_connect_error()
    assert err.code == em.E_UNHEALTHY
    assert err.reason == "service_down"
    assert "payload_svc" in err.detail["affects"]


# --- VARIANT (spec): 503 and connect_refused MUST NOT collapse -----

def test_variant_503_and_connect_refused_have_distinct_reasons():
    """VARIANT (spec P2-2 verbatim): 'if you collapse 503 and connect
    refused into one branch, detail.reason must distinguish 换线 from
    重启服务'. Test that the two paths produce DIFFERENT reasons."""
    r503 = em.map_http_status(503)
    rconn = em.map_connect_error()
    assert r503.reason != rconn.reason, \
        "503 (device_link_down) MUST NOT collapse with connect refused (service_down)"
    assert r503.reason == "device_link_down"
    assert rconn.reason == "service_down"


# --- WS close 1003 / 1011 ------------------------------------------

def test_ws_1003_maps_to_schema():
    err = em.map_ws_close(1003)
    assert err.code == em.E_SCHEMA
    assert err.reason == "ws_unsupported_data"


def test_ws_1011_maps_to_unhealthy():
    err = em.map_ws_close(1011)
    assert err.code == em.E_UNHEALTHY
    assert err.reason == "ws_internal_error"


# --- Unmapped codes get a generic UNHEALTHY, never None -----------

def test_unmapped_http_status_returns_unhealthy_not_none():
    err = em.map_http_status(418)
    assert err is not None
    assert err.code == em.E_UNHEALTHY
