"""
Copyright (c) 2026 Hachist Robotics
Author: wanglei@hachist.com
上海哈船智能船舶技术有限公司

File: probe_onvif.py
Brief: Ask a PTZ camera over ONVIF what it is and what it can do, without moving it.

Description:
  A discovery tool, not a test. It is the first thing to run against a camera we have
  never talked to, and it answers the questions a client cannot be written without:
  which vendor and firmware, what the RTSP URL actually is, which PTZ coordinate spaces
  the head accepts, and what its real limits are.

  Why a probe at all, when three manuals came with the device: the manuals are GUI
  manuals. They document menus -- "select Setup > Network > RTSP" -- and never state a
  single wire-level fact. The RTSP URL, the ONVIF service paths and the PTZ space URIs
  appear in none of them, so every one of those has to be read off the device itself.

  Why ONVIF rather than either vendor's own web API: the 布控球 carries two cameras from
  DIFFERENT vendors -- the visible-light head and the thermal core answer on different
  web stacks and even spell their ONVIF service paths differently (/onvif/media versus
  /onvif/media_service). ONVIF is the one interface both of them answer, so building on
  it costs one code path where the private APIs would cost two, and it keeps the door
  open for the next camera, which will be a third vendor.

  Why it starts from GetCapabilities instead of a hardcoded path table: GetCapabilities
  is answered WITHOUT authentication by both cameras and returns each device's own
  service addresses. Asking the device where its services live is what makes one code
  path work across vendors -- a path table would have to grow a row per model, and would
  be wrong the first time a firmware update moved something.

  Why it queries GetNodes rather than trusting the ranges printed in the manual: the
  general manual states P 0~360.0 deg, T -25.0~90.0 deg, Z 1~33, but those are the
  numbers a HUMAN sees in the web UI. ONVIF moves the head in a coordinate SPACE, and a
  client that picks the wrong space gets a fault rather than movement. Only the device's
  own node declaration says which spaces it accepts and what each one's bounds are.

  Why it is strictly read-only: it would be easy to make it nudge the head to prove PTZ
  works, and that is exactly what it must not do. A 360-degree head on a deployable ball
  is a physical hazard in an office, and a read-only probe is one that can be run
  unattended, repeatedly, and against a camera that is already pointed at something.

  Why the password comes from a file and never from the command line: argv is world
  readable through /proc on the Orin, which is a shared machine, so a password passed as
  an argument is exposed to every other user for as long as the process runs and lands in
  shell history besides. The credentials file is read once, is expected to be mode 600,
  and is the only place a secret lives. Nothing in this repository contains a password.

  Why it never tries a second credential after the first is refused: these cameras are
  shared lab equipment, and a tool that walks a candidate list is indistinguishable from
  one attacking the device -- both to a lockout policy and to anyone reading the logs.
  One host, one credential, one attempt.

  Run it from the repository root, on the Orin, which is the only host routed to the
  camera subnet:

      cd /opt/xbrain_v6 && python3 -m scripts.gimbal.probe_onvif --host 192.168.1.13

  The credentials file defaults to configs/onvif_credentials.json and maps host to user
  and password:

      {"192.168.1.13": {"user": "admin", "password": "..."}}
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import re
import socket
import stat
import sys
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# SOAP envelopes are assembled as text rather than with an XML builder because ONVIF is
# namespace-sensitive in ways a generic builder tends to get subtly wrong, and because the
# handful of requests here are fixed -- there is no user input to escape into them.
_NS = (
    'xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
    'xmlns:tds="http://www.onvif.org/ver10/device/wsdl" '
    'xmlns:trt="http://www.onvif.org/ver10/media/wsdl" '
    'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" '
    'xmlns:tt="http://www.onvif.org/ver10/schema"'
)
_WSSE = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd"
_UTIL = "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd"
_PASSWORD_DIGEST = (
    "http://docs.oasis-open.org/wss/2004/01/"
    "oasis-200401-wss-username-token-profile-1.0#PasswordDigest"
)
_BASE64_ENCODING = (
    "http://docs.oasis-open.org/wss/2004/01/"
    "oasis-200401-wss-soap-message-security-1.0#Base64Binary"
)
# Namespace prefixes are matched with hyphens and dots included, not with \w, because the
# visible-light camera prefixes its envelope "SOAP-ENV:" -- a \w-only pattern silently
# fails to match it and makes a perfectly explicit fault look like an empty response.
_PREFIX = r"[\w.-]+"
# Long enough for a camera that is busy encoding, short enough that an unreachable host
# fails while the operator is still watching.
_TIMEOUT_S = 8.0
# The capability categories worth asking for. "All" is accepted by both cameras and saves
# a round trip per category.
_CATEGORY_ALL = "All"
# Devices commonly allow a few minutes of drift on the WS-Security timestamp. 60s is well
# inside every implementation's window, so a skew above it is worth naming as a suspect
# without yet being proof.
_MAX_CLOCK_SKEW_S = 60.0
# A path no camera can serve, used to make an RTSP DESCRIBE answer a question purely about
# authentication. See credential_works_on_rtsp.
_IMPOSSIBLE_RTSP_PATH = "hachist-probe-nonexistent"


class OnvifError(Exception):
    """Raised when the camera refuses a request or cannot be reached.

    Separate from the built-in URL errors so a caller can tell "the camera said no" from
    "the network is broken", which are different problems with different fixes.
    """


@dataclass(frozen=True)
class DeviceInfo:
    """What the camera says it is."""

    manufacturer: str
    model: str
    firmware: str
    serial: str


@dataclass(frozen=True)
class StreamProfile:
    """One media profile and the RTSP URL that plays it."""

    token: str
    encoding: str
    width: str
    height: str
    uri: str


@dataclass(frozen=True)
class PtzSpace:
    """One PTZ coordinate space the head declares, with its bounds.

    The bounds are kept as strings exactly as reported. They are printed for a human to
    read, and converting them to floats here would invent precision the report does not
    need while risking a crash on a device that reports something unexpected.
    """

    uri: str
    x_min: str
    x_max: str
    y_min: str
    y_max: str


def _wsse_header(user: str, password: str) -> str:
    """Build a WS-Security UsernameToken header with a digested password.

    Digest rather than plaintext because the ONVIF profile requires it and because the
    camera link is an untrusted LAN shared with the payload; a nonce and timestamp also
    stop a captured header from being replayed later.
    """
    nonce = os.urandom(16)
    created = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    digest = base64.b64encode(
        hashlib.sha1(nonce + created.encode() + password.encode()).digest()
    ).decode()
    return (
        f'<s:Header><Security s:mustUnderstand="1" xmlns="{_WSSE}"><UsernameToken>'
        f"<Username>{user}</Username>"
        f'<Password Type="{_PASSWORD_DIGEST}">{digest}</Password>'
        f'<Nonce EncodingType="{_BASE64_ENCODING}">'
        f"{base64.b64encode(nonce).decode()}</Nonce>"
        f'<Created xmlns="{_UTIL}">{created}</Created>'
        f"</UsernameToken></Security></s:Header>"
    )


def soap_call(url: str, body: str, user: str = "", password: str = "") -> str:
    """POST one SOAP body and return the response text.

    An empty user means the request is sent unauthenticated, which is how the capability
    lookup runs before any credential is known to be correct.

    Raises:
        OnvifError: on a SOAP fault, an HTTP error, or an unreachable host.
    """
    header = _wsse_header(user, password) if user else ""
    envelope = f'<?xml version="1.0"?><s:Envelope {_NS}>{header}<s:Body>{body}</s:Body></s:Envelope>'
    request = urllib.request.Request(
        url,
        data=envelope.encode(),
        headers={"Content-Type": "application/soap+xml; charset=utf-8"},
    )
    try:
        with urllib.request.urlopen(request, timeout=_TIMEOUT_S) as response:
            return response.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as exc:
        # A SOAP fault arrives as an HTTP 400 with the real reason in the body, so the
        # body is worth more to the operator than the status code.
        detail = exc.read().decode("utf-8", "replace")
        # Tried in order of usefulness, and across both SOAP generations: one vendor
        # returns a 1.2 Reason/Text, the other a 1.1 faultstring inside a 1.2 envelope.
        # Falling back to the raw body would print an XML header and hide the reason,
        # so the scrape is worth widening rather than shortening.
        reason: List[str] = []
        for tag in ("Text", "faultstring", "Value", "Reason"):
            reason = tag_values(detail, tag)
            if reason:
                break
        raise OnvifError(f"{url} refused the request: {reason[-1] if reason else _fault_tail(detail)}") from exc
    except OSError as exc:
        raise OnvifError(f"{url} is unreachable: {exc}") from exc


def _fault_tail(detail: str) -> str:
    """Return the part of an unrecognised fault body that carries the reason.

    Truncating a SOAP fault from the front shows nothing but namespace declarations --
    these envelopes open with a dozen xmlns attributes before the first byte of content.
    Cutting from the Fault element instead puts the actual complaint on screen, which is
    the whole point of surfacing the body when no known tag matched.
    """
    marker = re.search(r"<(?:%s:)?Fault\b" % _PREFIX, detail)
    start = marker.start() if marker else 0
    return detail[start:start + 400].replace("\n", " ")


def tag_values(xml: str, tag: str) -> List[str]:
    """Return the text of every occurrence of one tag, ignoring its namespace prefix.

    A regex rather than a parser because the only thing wanted from these responses is a
    handful of leaf values, and the namespace prefixes differ between the two vendors --
    which is precisely the case where a parser needs more configuration than the scrape
    needs code.
    """
    pattern = r"<(?:%s:)?%s\b[^>]*>(.*?)</(?:%s:)?%s>" % (_PREFIX, tag, _PREFIX, tag)
    return [match.group(1) for match in re.finditer(pattern, xml, re.S)]


def _attribute_values(xml: str, attribute: str) -> List[str]:
    """Return every value of one attribute, in document order and without duplicates.

    Profile tokens repeat across nested elements in both vendors' responses, and the
    order matters: the first profile is conventionally the main stream.
    """
    seen: List[str] = []
    for value in re.findall(r'%s="([^"]+)"' % attribute, xml):
        if value not in seen:
            seen.append(value)
    return seen


def discover_services(host: str) -> Dict[str, str]:
    """Ask the camera where its own ONVIF services live.

    Returns:
        A mapping of lowercase service keyword ("media", "ptz", ...) to its address.

    This is what lets one code path serve both vendors: the visible-light head publishes
    /onvif/media and /onvif/ptz, the thermal core publishes /onvif/media_service and
    /onvif/ptz_service, and neither is guessable from the other.
    """
    body = f"<tds:GetCapabilities><tds:Category>{_CATEGORY_ALL}</tds:Category></tds:GetCapabilities>"
    xml = soap_call(f"http://{host}/onvif/device_service", body)
    services: Dict[str, str] = {}
    for address in tag_values(xml, "XAddr"):
        # Key on the last path segment with any _service suffix removed, so both spellings
        # collapse to the same key.
        leaf = address.rstrip("/").rsplit("/", 1)[-1]
        services[leaf.replace("_service", "").lower()] = address
    return services


def camera_clock_skew(host: str) -> Optional[float]:
    """Return how many seconds the camera's clock is ahead of ours, or None if it will not say.

    Worth a whole extra request because of how this failure presents: the WS-Security
    digest is computed over a timestamp, so a camera whose clock has drifted rejects a
    PERFECTLY CORRECT password with exactly the same "NotAuthorized" that a wrong password
    produces. Without this check the obvious next move is to try more passwords, which is
    the one move that risks a lockout and cannot possibly help.

    GetSystemDateAndTime is deliberately unauthenticated in the ONVIF spec precisely so it
    can be used this way, so asking costs nothing and burns no credential attempt.
    """
    try:
        xml = soap_call(f"http://{host}/onvif/device_service", "<tds:GetSystemDateAndTime/>")
    except OnvifError:
        # A device that will not report its time is not a reason to abort the probe; the
        # caller simply loses one diagnostic.
        return None
    # Read the UTC block specifically. Devices also report a local time whose offset is
    # described separately, and mixing the two would invent a skew that is not there.
    utc = re.search(r"<(?:%s:)?UTCDateTime>(.*?)</(?:%s:)?UTCDateTime>" % (_PREFIX, _PREFIX),
                    xml, re.S)
    if not utc:
        return None
    fields = {}
    for name in ("Year", "Month", "Day", "Hour", "Minute", "Second"):
        values = tag_values(utc.group(1), name)
        if not values:
            return None
        fields[name] = int(values[0])
    try:
        reported = datetime(fields["Year"], fields["Month"], fields["Day"],
                            fields["Hour"], fields["Minute"], fields["Second"],
                            tzinfo=timezone.utc)
    except ValueError:
        # An unset clock can report an impossible date; that is itself the finding.
        return None
    return (reported - datetime.now(timezone.utc)).total_seconds()


def device_information(services: Dict[str, str], user: str, password: str) -> DeviceInfo:
    """Read the nameplate: vendor, model, firmware, serial.

    This is also the credential check. It is the first authenticated call, so if the
    password is wrong the tool stops here with one clear message instead of producing a
    page of identical failures.
    """
    url = services.get("device", "")
    xml = soap_call(url, "<tds:GetDeviceInformation/>", user, password)
    return DeviceInfo(
        manufacturer=_first(xml, "Manufacturer"),
        model=_first(xml, "Model"),
        firmware=_first(xml, "FirmwareVersion"),
        serial=_first(xml, "SerialNumber"),
    )


def _first(xml: str, tag: str) -> str:
    """Return the first value of a tag, or a placeholder when the device omits it.

    Omissions are common and harmless -- some firmwares leave the serial blank -- so a
    missing field must not abort a probe whose whole purpose is to report what IS there.
    """
    values = tag_values(xml, tag)
    return values[0].strip() if values else "-"


def stream_profiles(services: Dict[str, str], user: str, password: str) -> List[StreamProfile]:
    """List every media profile with the RTSP URL that plays it.

    The URL is fetched per profile rather than assembled from a vendor template, because
    the template differs per vendor and per firmware and is the single most common reason
    a stream "does not work" when the camera is in fact fine.
    """
    url = services.get("media", "")
    if not url:
        return []
    listing = soap_call(url, "<trt:GetProfiles/>", user, password)
    tokens = _attribute_values(listing, "token")
    encodings = tag_values(listing, "Encoding")
    widths = tag_values(listing, "Width")
    heights = tag_values(listing, "Height")
    profiles: List[StreamProfile] = []
    for index, token in enumerate(tokens):
        body = (
            "<trt:GetStreamUri><trt:StreamSetup>"
            "<tt:Stream>RTP-Unicast</tt:Stream>"
            "<tt:Transport><tt:Protocol>RTSP</tt:Protocol></tt:Transport>"
            f"</trt:StreamSetup><trt:ProfileToken>{token}</trt:ProfileToken>"
            "</trt:GetStreamUri>"
        )
        try:
            uri = _first(soap_call(url, body, user, password), "Uri")
        except OnvifError as exc:
            # One unplayable profile must not hide the others: thermal cores commonly
            # publish a profile for a stream the model does not actually carry.
            uri = f"(refused: {exc})"
        profiles.append(
            StreamProfile(
                token=token,
                encoding=_at(encodings, index),
                width=_at(widths, index),
                height=_at(heights, index),
                uri=uri,
            )
        )
    return profiles


def _at(values: List[str], index: int) -> str:
    """Index a scraped list tolerantly.

    The encoding and resolution lists are scraped independently of the token list, so a
    device that omits one of them for one profile would otherwise raise IndexError and
    lose the whole report.
    """
    return values[index].strip() if index < len(values) else "-"


def ptz_spaces(services: Dict[str, str], user: str, password: str) -> List[PtzSpace]:
    """Report the coordinate spaces the head accepts, with each one's bounds.

    This is the fact a PTZ client is actually built on. The generic space normalises pan
    and tilt to -1..1 while a degree space carries real angles, and sending coordinates
    valid in one space to a device expecting the other is accepted-and-ignored on some
    firmwares -- a silent failure worth one extra request to avoid.
    """
    url = services.get("ptz", "")
    if not url:
        return []
    xml = soap_call(url, "<tptz:GetNodes/>", user, password)
    spaces: List[PtzSpace] = []
    # Each space is one <...Space> element carrying a URI and an XRange/YRange pair, so
    # the block is split on the URI boundaries rather than parsed as a tree.
    for block in re.split(r"(?=<(?:\w+:)?URI>)", xml):
        uris = tag_values(block, "URI")
        if not uris:
            continue
        mins = tag_values(block, "Min")
        maxs = tag_values(block, "Max")
        spaces.append(
            PtzSpace(
                uri=uris[0].strip(),
                x_min=_at(mins, 0),
                x_max=_at(maxs, 0),
                y_min=_at(mins, 1),
                y_max=_at(maxs, 1),
            )
        )
    return spaces


def _report(host: str, info: DeviceInfo, profiles: List[StreamProfile],
            spaces: List[PtzSpace], services: Dict[str, str]) -> None:
    """Print the findings in the order a client author needs them."""
    print(f"host        : {host}")
    print(f"device      : {info.manufacturer} {info.model}  fw={info.firmware} sn={info.serial}")
    print("services    :")
    for name in sorted(services):
        print(f"  {name:<12} {services[name]}")
    print("streams     :")
    for profile in profiles:
        size = f"{profile.width}x{profile.height}"
        print(f"  {profile.token:<28} {profile.encoding:<6} {size:<11} {profile.uri}")
    print("ptz spaces  :")
    for space in spaces:
        # The trailing segment is the part that distinguishes the spaces; the common
        # prefix is the same ONVIF schema URL on every one of them.
        leaf = space.uri.rsplit("/", 1)[-1]
        print(f"  {leaf:<34} x[{space.x_min},{space.x_max}] y[{space.y_min},{space.y_max}]")
    if not spaces:
        print("  (none reported -- this camera exposes no PTZ node)")


def credential_works_on_rtsp(host: str, user: str, password: str) -> Optional[bool]:
    """Test the same credential against RTSP, which shares the camera's main user list.

    Returns:
        True if RTSP accepted it, False if RTSP rejected it, None if RTSP gave no verdict.

    This is the evidence that separates "the password is wrong" from "ONVIF specifically
    is refusing it" -- two failures that look identical from the ONVIF side and have
    completely different fixes. Without it the only way to make progress is to try more
    passwords against a device that may be perfectly happy with this one.

    The request deliberately names a path the camera cannot have, because the verdict
    wanted is purely about authentication: a rejected credential answers 401 whatever the
    path, while an accepted one gets far enough to answer 404. That avoids needing to know
    the stream URL, which is itself one of the things the probe is trying to discover.
    """
    url = f"rtsp://{host}:554/{_IMPOSSIBLE_RTSP_PATH}"
    try:
        challenge = _rtsp_request(host, f"DESCRIBE {url} RTSP/1.0\r\nCSeq: 1\r\n\r\n")
        if not challenge.startswith("RTSP/1.0 401"):
            # No challenge means RTSP is not gating on credentials at all, so it can say
            # nothing about whether this one is right.
            return None
        realm = re.search(r'realm="([^"]*)"', challenge)
        nonce = re.search(r'nonce="([^"]*)"', challenge)
        if not realm or not nonce:
            return None
        # Standard RFC 2617 digest, computed here rather than pulled from a library
        # because the only consumer is this one request.
        ha1 = hashlib.md5(f"{user}:{realm.group(1)}:{password}".encode()).hexdigest()
        ha2 = hashlib.md5(f"DESCRIBE:{url}".encode()).hexdigest()
        response = hashlib.md5(f"{ha1}:{nonce.group(1)}:{ha2}".encode()).hexdigest()
        authorization = (
            f'Digest username="{user}", realm="{realm.group(1)}", '
            f'nonce="{nonce.group(1)}", uri="{url}", response="{response}"'
        )
        answer = _rtsp_request(
            host, f"DESCRIBE {url} RTSP/1.0\r\nCSeq: 2\r\n{'Authorization: ' + authorization}\r\n\r\n")
    except OSError:
        return None
    if answer.startswith("RTSP/1.0 401"):
        return False
    # Anything that is not another challenge means the credential got past the gate.
    return True


def _rtsp_request(host: str, request: str) -> str:
    """Send one RTSP request and return the response head.

    Raw sockets because RTSP is not HTTP and urllib will not speak it, and because only
    the status line and the WWW-Authenticate header are wanted -- reading the first chunk
    is enough and avoids waiting on a stream that will never start.
    """
    with socket.create_connection((host, 554), timeout=_TIMEOUT_S) as sock:
        sock.sendall(request.encode())
        return sock.recv(2048).decode("utf-8", "replace")


def _explain_rejection(skew: Optional[float], rtsp_ok: Optional[bool]) -> str:
    """Spell out the causes of an ONVIF auth rejection, cheapest-and-most-silent first.

    Written out in full because the instinctive response to "not authorized" is to try
    another password, and most of the causes below make that useless while risking the
    lockout that would end the session. The RTSP verdict is what turns this from a list of
    possibilities into an actual diagnosis, so it is stated last and loudest.
    """
    if skew is None:
        clock = "camera would not report its clock"
    elif abs(skew) > _MAX_CLOCK_SKEW_S:
        clock = f"camera clock is {skew:+.0f}s off -- LIKELY THE CAUSE, fix the clock first"
    else:
        clock = f"camera clock is {skew:+.0f}s off, close enough -- not the cause"
    if rtsp_ok is True:
        verdict = ("RTSP ACCEPTED this same credential, so the password is RIGHT and the\n"
                   "     problem is ONVIF-specific: check that ONVIF is enabled and that\n"
                   "     this user is allowed to use it. Do NOT try other passwords.")
    elif rtsp_ok is False:
        verdict = ("RTSP also REJECTED this credential, so the password really is wrong\n"
                   "     for this camera. Read the nameplate again -- do NOT guess.")
    else:
        verdict = "RTSP gave no verdict, so the password remains unconfirmed either way."
    return (
        "  causes, in the order worth checking:\n"
        f"  1. clock skew: {clock}\n"
        "  2. some of these cameras keep a SEPARATE ONVIF user list from the web UI,\n"
        "     so a working web login can still be refused here (the 热成像 core does\n"
        "     this; the 可见光 one does not)\n"
        f"  3. the credential: {verdict}"
    )


def load_credentials(path: Path, host: str) -> Tuple[str, str]:
    """Look up one host's ONVIF user and password in the credentials file.

    Returns:
        (user, password) for the host.

    Raises:
        OnvifError: when the file is missing, malformed, or has no entry for the host.

    Keyed by host rather than holding a single credential because the 布控球 carries two
    cameras from different vendors whose passwords are set independently -- a one-password
    file would guarantee that probing the second camera reuses the first one's secret,
    which is the exact failed attempt this tool is built to avoid.
    """
    try:
        raw = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise OnvifError(
            f"cannot read credentials at {path}: {exc}. "
            'Expected {"<host>": {"user": "...", "password": "..."}}'
        ) from exc
    # Warn rather than refuse: a wrong mode is worth telling the operator about, but
    # refusing to run would strand anyone whose filesystem cannot represent the mode.
    mode = path.stat().st_mode
    if mode & (stat.S_IRGRP | stat.S_IROTH):
        print(f"warning: {path} is readable by other users", file=sys.stderr)
    try:
        entries = json.loads(raw)
        entry = entries[host]
        return entry["user"], entry["password"]
    except (ValueError, KeyError, TypeError) as exc:
        raise OnvifError(f"no usable credential for {host} in {path}: {exc}") from exc


def main(argv: Optional[List[str]] = None) -> int:
    """Probe one camera and print what a client would need to talk to it.

    Returns:
        0 when the camera answered, 1 when it refused or could not be reached.
    """
    parser = argparse.ArgumentParser(description="probe a PTZ camera over ONVIF, read-only")
    parser.add_argument("--host", required=True, help="camera address, e.g. 192.168.1.13")
    # No --password option exists on purpose: see the module docstring on argv exposure.
    parser.add_argument("--credentials", type=Path,
                        default=Path("configs/onvif_credentials.json"),
                        help="JSON file mapping host to user and password")
    args = parser.parse_args(argv)

    # Measured before the authenticated calls so the number is available to explain a
    # rejection, and outside the try so an early failure cannot leave it unbound.
    skew = camera_clock_skew(args.host)
    try:
        user, password = load_credentials(args.credentials, args.host)
        services = discover_services(args.host)
        info = device_information(services, user, password)
        profiles = stream_profiles(services, user, password)
        spaces = ptz_spaces(services, user, password)
    except OnvifError as exc:
        print(f"probe failed: {exc}", file=sys.stderr)
        if "uthoriz" in str(exc):
            # Only reached once ONVIF has already refused, so this spends no extra attempt
            # on a working path and cannot itself be what triggers a lockout.
            rtsp_ok = credential_works_on_rtsp(args.host, *load_credentials(args.credentials, args.host))
            print(_explain_rejection(skew, rtsp_ok), file=sys.stderr)
        return 1
    if skew is not None and abs(skew) > _MAX_CLOCK_SKEW_S:
        # Reported even on success: the margin may be shrinking toward the failure point,
        # and it also makes recorded timestamps untrustworthy for correlating with logs.
        print(f"warning: camera clock is {skew:+.0f}s off this host's", file=sys.stderr)
    _report(args.host, info, profiles, spaces, services)
    return 0


if __name__ == "__main__":
    sys.exit(main())
