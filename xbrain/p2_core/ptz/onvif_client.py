"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: onvif_client.py
Brief: ONVIF WS-Security client for the 可见光 camera PTZ (jog + zoom + focus)

Description:
Ported from the field-proven /usr/local/lib/ptz/onvif.py (PTZ 布控球实测报告
S4). The 可见光 camera (default 192.168.66.13) drives the PELCO-D pan/tilt
head, and the ONLY usable jog primitive is ONVIF ContinuousMove/Stop --
the camera's LAPI has no continuous-move endpoint, and absolute positioning
on a PELCO-D head is accepted-but-not-executed (a no-op, position readback
is a fixed fake (180,0)). That is exactly the T-PTZ-1 wall 18-B E02/E03
sit behind.

The authentication is the single form found to work (report S4.3, from an
8-way exhaustion): WS-Security UsernameToken with PasswordDigest =
SHA1(nonce + created + password), Created WITHOUT milliseconds, and it MUST
NOT be combined with HTTP digest/basic auth -- stacking HTTP auth makes the
device reject with 'Sender not Authorized' (ter:NotAuthorized), the trap
that reads as a bad credential. No ONVIF user needs to be created; the
device admin account is enough.

Traps this guards (each already cost time in the report):
  * combining HTTP auth with WS-Security -> NotAuthorized (do NOT add
    an Authorization header)
  * a Created field with milliseconds -> some firmwares reject it
  * not reading the response body on a keep-alive connection -> the
    socket cannot be reused

The `Created` timestamp is a WALL-CLOCK protocol field required by ONVIF;
this is a legitimate exception to CLK-C1 (which governs timeouts/periods/
age, not a protocol-mandated UTC timestamp). Host + credentials are INJECTED
(never hardcoded here) -- they come from configs/secrets/onvif_credentials
.json (freeze assertion J validates its 0600 mode).

Only stdlib is used (urllib / http.client / hashlib / base64 / xml) so this
adds no dependency to the p2_core runtime.
"""
from __future__ import annotations

import base64
import datetime
import hashlib
import http.client
import os
import re
from typing import Optional


# ONVIF XML namespaces (report S4.3).
NS = {
    "s": "http://www.w3.org/2003/05/soap-envelope",
    "tds": "http://www.onvif.org/ver10/device/wsdl",
    "trt": "http://www.onvif.org/ver10/media/wsdl",
    "tptz": "http://www.onvif.org/ver20/ptz/wsdl",
    "timg": "http://www.onvif.org/ver20/imaging/wsdl",
    "tt": "http://www.onvif.org/ver10/schema",
}

_WSS = ("http://docs.oasis-open.org/wss/2004/01/"
        "oasis-200401-wss-wssecurity-secext-1.0.xsd")
_WSU = ("http://docs.oasis-open.org/wss/2004/01/"
        "oasis-200401-wss-wssecurity-utility-1.0.xsd")
_PWD_DIGEST = ("http://docs.oasis-open.org/wss/2004/01/"
               "oasis-200401-wss-username-token-profile-1.0#PasswordDigest")


class OnvifError(RuntimeError):
    """An ONVIF call failed at the transport level or returned a SOAP Fault."""


def _ws_header(user: str, pwd: str) -> str:
    """Build the WS-Security UsernameToken PasswordDigest header.

    digest = base64(SHA1(nonce + created + pwd)); Created has NO milliseconds
    (report S4.3). A fresh nonce + timestamp per call, so a captured header
    cannot be replayed."""
    nonce = os.urandom(16)
    # Wall-clock UTC is REQUIRED by the ONVIF protocol here (not a timeout).
    created = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")
    digest = base64.b64encode(
        hashlib.sha1(nonce + created.encode() + pwd.encode()).digest()).decode()
    return (f'<s:Header><Security s:mustUnderstand="1" xmlns="{_WSS}">'
            f"<UsernameToken><Username>{user}</Username>"
            f'<Password Type="{_PWD_DIGEST}">{digest}</Password>'
            f"<Nonce>{base64.b64encode(nonce).decode()}</Nonce>"
            f'<Created xmlns="{_WSU}">{created}</Created>'
            f"</UsernameToken></Security></s:Header>")


def soap_fault(xml: str) -> Optional[str]:
    """Return the SOAP Fault text if `xml` is a fault/error, else None."""
    if "Fault" not in xml and "error>" not in xml:
        return None
    m = re.search(r"<[^>]*(?:Text|faultstring)[^>]*>([^<]{0,200})", xml)
    return m.group(1) if m else "unknown fault"


class OnvifSession:
    """Keep-alive ONVIF SOAP client for one camera (report S4: reusing the
    TCP connection cuts the per-command cost that otherwise reads as sluggish
    jog control). NOT combined with HTTP auth (WS-Security only).

    Not thread-safe: the p2 PTZ domain serialises its calls under one lock,
    like every other single-connection device client here."""

    __slots__ = ("_host", "_user", "_pwd", "_timeout", "_conn")

    def __init__(self, host: str, user: str, pwd: str,
                 *, timeout: float = 4.0) -> None:
        # host is 'ip' or 'ip:port'; default ONVIF port is 80.
        self._host = host if ":" in host else host + ":80"
        self._user = user
        self._pwd = pwd
        self._timeout = timeout
        self._conn = None

    def call(self, path: str, body: str) -> str:
        """POST one SOAP body to `path` (e.g. '/onvif/ptz'); return the
        response text. Reconnects once if the pooled socket was reaped.
        Raises OnvifError only on a hard transport failure after the retry."""
        env = ('<?xml version="1.0" encoding="UTF-8"?>'
               '<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope">'
               f"{_ws_header(self._user, self._pwd)}"
               f"<s:Body>{body}</s:Body></s:Envelope>").encode()
        headers = {"Content-Type": "application/soap+xml; charset=utf-8",
                   "Connection": "keep-alive"}
        last_exc = None
        for attempt in (1, 2):          # the peer may reap the connection
            try:
                if self._conn is None:
                    self._conn = http.client.HTTPConnection(
                        self._host, timeout=self._timeout)
                self._conn.request("POST", path, env, headers)
                # Body MUST be read fully or the connection cannot be reused.
                return self._conn.getresponse().read().decode(
                    "utf-8", "replace")
            except Exception as exc:      # noqa: BLE001 -- retried / re-raised
                last_exc = exc
                self._close_quietly()
        raise OnvifError("onvif call to %s%s failed: %s"
                         % (self._host, path, last_exc))

    def close(self) -> None:
        self._close_quietly()

    def _close_quietly(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:             # noqa: BLE001
                pass
            self._conn = None


# --- PTZ (report S4.4: ContinuousMove/Stop is the usable jog primitive) --

def ptz_continuous(session: OnvifSession, ptz_path: str, token: str,
                   *, pan: float = 0.0, tilt: float = 0.0,
                   zoom: float = 0.0) -> str:
    """Start a continuous move. Velocity is a 2-D PanTilt vector + Zoom, one
    command drives both axes (report S6.1). pan<0 left / >0 right; tilt>0 up
    / <0 down; zoom>0 in / <0 out."""
    body = (f'<ContinuousMove xmlns="{NS["tptz"]}">'
            f"<ProfileToken>{token}</ProfileToken>"
            f'<Velocity xmlns:tt="{NS["tt"]}">'
            f'<tt:PanTilt x="{pan}" y="{tilt}"/>'
            f'<tt:Zoom x="{zoom}"/>'
            f"</Velocity></ContinuousMove>")
    return session.call(ptz_path, body)


def ptz_stop(session: OnvifSession, ptz_path: str, token: str,
             *, pantilt: bool = True, zoom: bool = True) -> str:
    """Stop pan/tilt and/or zoom motion."""
    body = (f'<Stop xmlns="{NS["tptz"]}"><ProfileToken>{token}</ProfileToken>'
            f"<PanTilt>{str(pantilt).lower()}</PanTilt>"
            f"<Zoom>{str(zoom).lower()}</Zoom></Stop>")
    return session.call(ptz_path, body)


def get_profile_token(session: OnvifSession,
                      media_path: str = "/onvif/media") -> Optional[str]:
    """Fetch the first media profile token (the ProfileToken ContinuousMove
    needs). Returns None if none found."""
    xml = session.call(
        media_path,
        '<GetProfiles xmlns="http://www.onvif.org/ver10/media/wsdl"/>')
    toks = re.findall(r'token="([^"]+)"', xml)
    return toks[0] if toks else None
