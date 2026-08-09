"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司
File: hello.py
Brief: MOT-PM-24 rt/chassis/hello / hello_ack handshake with quadruped

Description:
At P1 startup and on every reconnect, P1 sends ONE hello message on
rt/chassis/hello and waits for hello_ack. The wire shape is fixed
verbatim: {"type": "hello", "proto_version": "1.0", "client":
"p1_motion"}. quadruped acknowledges with {"type": "hello_ack",
"proto_version": "1.0", "server": "quadruped"}. Version mismatch
is a hard startup refusal.
"""

from __future__ import annotations

from dataclasses import dataclass


PROTO_VERSION = "1.0"


def build_hello() -> dict:
    """Produce the hello message. Fields are verbatim."""
    return {
        "type": "hello",
        "proto_version": PROTO_VERSION,
        "client": "p1_motion",
    }


def build_hello_ack() -> dict:
    """Produce the hello_ack (server-side; kept here for symmetry)."""
    return {
        "type": "hello_ack",
        "proto_version": PROTO_VERSION,
        "server": "quadruped",
    }


class HandshakeError(RuntimeError):
    """hello_ack rejected: wrong type or proto_version mismatch."""


def validate_hello_ack(msg: dict) -> None:
    """Refuse a hello_ack that is missing fields or on a different
    proto_version. Refuse strictly: no silent accept of newer major."""
    if not isinstance(msg, dict):
        raise HandshakeError("hello_ack not a dict")
    if msg.get("type") != "hello_ack":
        raise HandshakeError("hello_ack.type != 'hello_ack'; got %r"
                              % msg.get("type"))
    if msg.get("proto_version") != PROTO_VERSION:
        raise HandshakeError(
            "hello_ack.proto_version %r != %r; refuse startup"
            % (msg.get("proto_version"), PROTO_VERSION))
    if msg.get("server") != "quadruped":
        raise HandshakeError("hello_ack.server != 'quadruped'")
