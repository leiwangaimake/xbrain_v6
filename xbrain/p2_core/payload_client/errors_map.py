"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: errors_map.py
Brief: BIZ-P2-2 -- payload-service HTTP status -> closed-set error code map

Description:
BIZ-P2-2 assertion table (11 S7.5A + 8.13.5):

  HTTP 400/422    -> E_SCHEMA
  HTTP 409        -> E_BUSY   detail.reason=payload_mode detail.payload_mode=<mode>
  HTTP 503        -> E_UNHEALTHY detail.reason=device_link_down + payload_* fail
  connect refused -> E_UNHEALTHY detail.reason=service_down + payload_svc fail
  WS close 1003   -> event/warn/system (implementation bug -- schema mismatch)
  WS close 1011   -> fail

* VARIANT of spec: 503 and connect_refused MUST NOT collapse into
the same detail.reason -- '换线' (device link changed) is a device-
side event; 'service_down' is a service-side event. Operator's next
action is different for each; collapsing them would erase that
distinction.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional


# Closed-set codes we produce; must be imported from errors module in
# production callers (this file returns strings and the caller wraps
# them in XbrainError with the imported constant).
E_SCHEMA = "E_SCHEMA"
E_BUSY = "E_BUSY"
E_UNHEALTHY = "E_UNHEALTHY"


@dataclass(frozen=True)
class MappedError:
    """The rendered error for a payload-service failure. code +
    reason (detail.reason) + optional detail dict."""
    code: str
    reason: str
    detail: Dict[str, Any]


def map_http_status(status_code: int,
                    body: Optional[Dict[str, Any]] = None) -> Optional[MappedError]:
    """Return the mapped error for a payload-service HTTP response,
    or None if the status is 2xx (success)."""
    if 200 <= status_code < 300:
        return None
    body = body or {}
    if status_code in (400, 422):
        return MappedError(
            code=E_SCHEMA,
            reason="schema",
            detail={"http_status": status_code, "body_hint": body},
        )
    if status_code == 409:
        return MappedError(
            code=E_BUSY,
            reason="payload_mode",
            detail={
                "http_status": 409,
                "payload_mode": body.get("payload_mode"),
            },
        )
    if status_code == 503:
        return MappedError(
            code=E_UNHEALTHY,
            reason="device_link_down",
            detail={
                "http_status": 503,
                # Callers should set the relevant payload_* health
                # items to fail; this dict signals which family.
                "affects": ["payload_speaker", "payload_siren",
                             "payload_light"],
            },
        )
    # Any other non-2xx status is unmapped -- caller decides how to
    # surface. Returning None here would be wrong (indicates success);
    # return a generic UNHEALTHY.
    return MappedError(
        code=E_UNHEALTHY,
        reason="unmapped_http",
        detail={"http_status": status_code},
    )


def map_connect_error() -> MappedError:
    """A connect-refused (ECONNREFUSED, connect timeout, socket EOF
    before any HTTP status) means the payload-service process itself
    is down or has not started yet. Distinct from 503 which means
    'service up but the device link failed'."""
    return MappedError(
        code=E_UNHEALTHY,
        reason="service_down",
        detail={
            "affects": ["payload_svc"],
        },
    )


def map_ws_close(code: int) -> MappedError:
    """WebSocket close codes we care about.
      1003 = 'unsupported data' = client and server disagree on
             message shape (implementation bug).
      1011 = 'internal error' = server crashed the connection.
    """
    if code == 1003:
        return MappedError(
            code=E_SCHEMA,      # implementation-side schema issue
            reason="ws_unsupported_data",
            detail={"ws_close_code": 1003},
        )
    if code == 1011:
        return MappedError(
            code=E_UNHEALTHY,
            reason="ws_internal_error",
            detail={"ws_close_code": 1011},
        )
    return MappedError(
        code=E_UNHEALTHY,
        reason="ws_unmapped_close",
        detail={"ws_close_code": code},
    )
