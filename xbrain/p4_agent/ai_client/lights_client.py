"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: lights_client.py
Brief: HTTP client for payload-service /lights (red-blue warning lamp)

Description:
Thin wrapper over payload-service POST /lights. Called by p2's
payload subscriber (or by any test harness) to turn the red-blue
warning lamp on / off. The lamp is what the requirements call
'爆闪' (00 PAY-02c settles this term); the searchlight's own
strobe (services/payload/api/rest.py LightsCommand.strobe field)
is a device capability the project does NOT use (VOI-43 + PER-42),
so this client deliberately touches only the `redblue` field.

redblue value is a PATTERN SELECTOR (00 PAY-02c / 14 GL-4d):
  * 0            -> off
  * 1 .. 16      -> firmware pattern index (rate is baked into pattern,
                    caller does NOT synthesise flashing by toggling)

We default the on-value to 1 (the first pattern). If a specific
pattern is ever needed, thread `pattern` through the call site.
"""

from __future__ import annotations

import requests


class LightsClientError(Exception):
    pass


_LIGHTS_PATH = "/lights"


def set_redblue(base_url: str, *, on: bool, timeout_s: float,
                pattern: int = 1) -> dict:
    """POST /lights with just the redblue field set. Returns the ack
    dict from payload-service. Raises LightsClientError on any HTTP
    or transport failure.

    `on=True`  -> redblue = pattern (default 1)
    `on=False` -> redblue = 0

    All other lamp aspects are left unchanged (searchlight,
    brightness, searchlight-strobe): the field-is-None-means-untouched
    semantics of LightsCommand does that for us.
    """
    if not 1 <= pattern <= 16:
        raise LightsClientError(
            "pattern %r out of 1..16 (00 PAY-02c red-blue index range)"
            % pattern)
    url = base_url.rstrip("/") + _LIGHTS_PATH
    body = {"redblue": pattern if on else 0}
    return _post_lights(url, body, timeout_s)


def set_searchlight(base_url: str, *, on: bool, timeout_s: float,
                    bright: int = None) -> dict:
    """POST /lights to turn the searchlight (照明灯 / 补光灯) on or off.

    This is the STEADY illumination lamp (14 GL-2: 探照灯 only 常亮 or off,
    never strobe), NOT the red-blue 爆闪 warning lamp (that is set_redblue).
    The 2026-08-11 ORIN test showed D01/D02 were dropped -- p2 only handled
    the red-blue lamp; this is the searchlight half.

    on=True  -> searchlight = true; a bright value (0..30, 14 GL-2
                MSG_BRIGHT range) is sent alongside so the lamp visibly
                illuminates rather than resuming a possibly-zero remembered
                level. on=False -> searchlight = false (brightness left for
                the device to remember, per LightsCommand None-means-untouched).
    """
    url = base_url.rstrip("/") + _LIGHTS_PATH
    if on:
        body = {"searchlight": True}
        if bright is not None:
            if not 0 <= bright <= 30:
                raise LightsClientError(
                    "bright %r out of 0..30 (14 GL-2 MSG_BRIGHT range)"
                    % bright)
            body["bright"] = bright
    else:
        body = {"searchlight": False}
    return _post_lights(url, body, timeout_s)


def _post_lights(url: str, body: dict, timeout_s: float) -> dict:
    """Shared POST /lights transport. Raises LightsClientError on any HTTP
    or transport failure."""
    try:
        r = requests.post(url, json=body, timeout=timeout_s)
    except requests.RequestException as exc:
        raise LightsClientError(
            "lights request to %s failed: %s" % (url, exc)) from exc
    if r.status_code != 200:
        raise LightsClientError(
            "lights returned %d: %s" % (r.status_code, r.text[:200]))
    try:
        return r.json()
    except ValueError as exc:
        raise LightsClientError(
            "lights response not JSON: %s" % r.text[:200]) from exc
